"""Replay a recorded RGB file through OAK inference and the live ROS postprocessor."""
import argparse
from collections import Counter, OrderedDict
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from hl_oak_interfaces.msg import PerceptionFrame
from hl_oak_perception.bridge import QOS
from hl_oak_perception.common import image_array

sys.path.insert(0, str(Path(__file__).resolve().parent))
from video_file_sender import VideoFileSender, select_camera


def stamp_key(stamp):
    return stamp.sec, stamp.nanosec


def panel(image, title):
    h, w = image.shape[:2]
    scale = min(640/w, 324/h)
    resized = cv2.resize(image, (round(w*scale), round(h*scale)))
    canvas = np.zeros((360, 640, 3), np.uint8)
    y, x = 34+(324-resized.shape[0])//2, (640-resized.shape[1])//2
    canvas[y:y+resized.shape[0], x:x+resized.shape[1]] = resized
    cv2.putText(canvas, title, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, .6, (255,255,255), 1)
    return canvas


class Validation(Node):
    def __init__(self, output, duration, fps):
        super().__init__('oak_recorded_video_validation')
        self.output, self.duration, self.fps = output, duration, fps
        self.slots = OrderedDict()
        self.counts = Counter()
        self.states = Counter()
        self.stop_states = Counter()
        self.reasons = Counter()
        self.samples = []
        self.times = []
        self.bins = {}
        self.last_snapshot = -30.
        self.next_video_time = 0.
        self.last_mosaic = None
        self.last_result_wall = time.monotonic()
        self.stale = Counter()
        self.log = (output/'frames.jsonl').open('w')
        self.writer = cv2.VideoWriter(str(output/'ros_replay.mp4'), cv2.VideoWriter_fourcc(*'mp4v'), fps, (1280,720))
        if not self.writer.isOpened():
            raise RuntimeError('Cannot open result MP4 writer')
        self.create_subscription(PerceptionFrame, 'oak/frame', self.frame, QOS)
        self.create_subscription(String, 'oak/postprocess/result', self.result, 1)
        self.create_subscription(Image, 'oak/postprocess/debug_image', lambda m: self.picture('debug', m), QOS)
        self.create_subscription(Image, 'oak/postprocess/top_image', lambda m: self.picture('top', m), QOS)

    def slot(self, key):
        value = self.slots.setdefault(key, {})
        while len(self.slots) > 24:
            self.slots.popitem(last=False)
        return value

    def frame(self, message):
        metadata = json.loads(message.metadata_json)
        if metadata['source'].get('input_source') != 'pc':
            return
        self.counts['ros_frames'] += 1
        if metadata['depth_available'] or np.count_nonzero(image_array(message.depth)):
            self.counts['invalid_pc_depth_frames'] += 1
        key = stamp_key(message.header.stamp)
        self.slot(key).update(metadata=metadata, rgb=image_array(message.rgb),
            lane=image_array(message.lane_mask), road=image_array(message.drivable_mask))
        self.join(key)

    def result(self, message):
        result = json.loads(message.data)
        if result.get('stale'):
            self.stale[result.get('reason', 'unknown')] += 1
            return
        key = result['stamp']['sec'], result['stamp']['nanosec']
        self.slot(key)['result'] = result
        self.join(key)

    def picture(self, name, message):
        key = stamp_key(message.header.stamp)
        self.slot(key)[name] = image_array(message)
        self.join(key)

    def join(self, key):
        slot = self.slots[key]
        if not all(name in slot for name in ('metadata', 'rgb', 'lane', 'road', 'result', 'debug', 'top')):
            return
        self.slots.pop(key)
        metadata, result = slot['metadata'], slot['result']
        media = metadata['source']['source_media_time_s']
        self.last_result_wall = time.monotonic()
        self.counts['matched_frames'] += 1
        self.counts['boundary_valid'] += bool(result['boundary_valid'])
        self.counts['boundary_pair_valid'] += bool(result['boundary_pair_valid'])
        self.counts['guidance_valid'] += bool(result['guidance_valid'])
        self.counts['guidance_display'] += bool(result['guidance_path_display_available'])
        self.counts['object_frames'] += bool(metadata['objects'])
        state = result['left_boundary_state']+'/'+result['right_boundary_state']
        self.states[state] += 1
        self.stop_states[result['stop_line_state']] += 1
        self.reasons[result['guidance_reason']] += 1
        self.times.append(float(media))
        bucket = self.bins.setdefault(int(media//30), Counter())
        bucket['frames'] += 1
        bucket['pair_valid'] += bool(result['boundary_pair_valid'])
        bucket['guidance_valid'] += bool(result['guidance_valid'])
        bucket[state] += 1
        # Check the exact visible TOP midpoint invariant when a path exists.
        edges = np.asarray(result.get('guidance_target_edges_top', []), dtype=float)
        target = np.asarray(result.get('guidance_target_top', []), dtype=float)
        if edges.shape == (2,2) and target.shape == (2,):
            error = float(np.linalg.norm(edges.mean(axis=0)-target))
            if error > 1e-5:
                self.counts['guidance_midpoint_violations'] += 1
        self.log.write(json.dumps(dict(media_time_s=media, objects=metadata['objects'], result=result))+'\n')
        inference = slot['rgb'].copy()
        h, w = inference.shape[:2]
        road = cv2.resize(slot['road'], (w,h), interpolation=cv2.INTER_NEAREST) != 0
        lane = cv2.resize(slot['lane'], (w,h), interpolation=cv2.INTER_NEAREST) != 0
        inference[road] = (.7*inference[road]+.3*np.array([0,255,0])).astype(np.uint8)
        inference[lane] = (0,0,255)
        for obj in metadata['objects']:
            p1 = round(obj['xmin']*w), round(obj['ymin']*h)
            p2 = round(obj['xmax']*w), round(obj['ymax']*h)
            cv2.rectangle(inference, p1, p2, (0,255,255), 2)
        title = f'{media:.1f}s  L:{result["left_boundary_state"]} R:{result["right_boundary_state"]}'
        mosaic = np.vstack((np.hstack((panel(slot['rgb'], f'RGB {media:.1f}s'),
                                      panel(inference, 'OAK inference: road/lane/objects'))),
                            np.hstack((panel(slot['debug'], title),
                                      panel(slot['top'], f'ROS TOP  path:{result["guidance_path_display_state"]}')))))
        if self.last_mosaic is None:
            self.last_mosaic = mosaic
        # Preserve source media duration; frames between received results hold.
        while self.next_video_time < media:
            self.writer.write(self.last_mosaic)
            self.next_video_time += 1/self.fps
        self.last_mosaic = mosaic
        if media-self.last_snapshot >= 30.:
            name = f'frame_{media:07.2f}.jpg'
            cv2.imwrite(str(self.output/name), mosaic)
            self.samples.append(dict(media_time_s=media, image=name, states=state,
                                     guidance=result['guidance_path_display_state']))
            self.last_snapshot = media
            self.progress()

    def progress(self):
        value = dict(counts=dict(self.counts), boundary_states=dict(self.states),
            stop_line_states=dict(self.stop_states), guidance_reasons=dict(self.reasons),
            source_start_s=self.times[0] if self.times else None,
            source_end_s=self.times[-1] if self.times else None,
            max_media_gap_s=max(np.diff(self.times), default=0.),
            stale_messages=dict(self.stale), bins=self.bins, samples=self.samples)
        (self.output/'progress.json').write_text(json.dumps(value, indent=2))
        self.log.flush()
        print(json.dumps(dict(media_s=value['source_end_s'], counts=value['counts'])), flush=True)
        return value

    def finish(self):
        if self.last_mosaic is not None:
            while self.next_video_time < self.duration:
                self.writer.write(self.last_mosaic)
                self.next_video_time += 1/self.fps
        self.writer.release()
        self.log.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--video', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--device-ip', default='169.254.100.228')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    capture = cv2.VideoCapture(str(args.video))
    duration = capture.get(cv2.CAP_PROP_FRAME_COUNT)/capture.get(cv2.CAP_PROP_FPS)
    capture.release()
    rclpy.init()
    node = Validation(args.output, duration, 5.)
    sender = VideoFileSender(args.device_ip, args.video, loop=False, fps=10)
    started = time.monotonic()
    report = {}
    try:
        # Allow ROS discovery before selecting the file as the device source.
        while time.monotonic()-started < 2:
            rclpy.spin_once(node, timeout_sec=.1)
        sender.start()
        deadline = time.monotonic()+duration+60
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            status = sender.snapshot()
            if status['state'] == '오류':
                raise RuntimeError(status['error'])
            if status['state'] == '재생 완료':
                drain = time.monotonic()+3
                while time.monotonic() < drain:
                    rclpy.spin_once(node, timeout_sec=.02)
                break
        else:
            raise RuntimeError('Replay timed out')
        report = node.progress()
        report.update(video=str(args.video), duration_s=duration, sender=sender.snapshot(),
            elapsed_s=time.monotonic()-started,
            passed=bool(node.times and node.times[-1] > duration-3 and
                        not node.counts['invalid_pc_depth_frames'] and
                        not node.counts['guidance_midpoint_violations']))
    finally:
        sender.close()
        report['camera_restore'] = select_camera(args.device_ip)
        node.finish()
        (args.output/'report.json').write_text(json.dumps(report, indent=2))
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps({k:v for k,v in report.items() if k not in ('samples','bins')}), flush=True)
    return 0 if report.get('passed') else 1


if __name__ == '__main__':
    raise SystemExit(main())
