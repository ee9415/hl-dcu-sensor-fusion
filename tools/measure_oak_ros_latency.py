"""Same-host monotonic end-to-end latency, correlated by original video frame index."""
import argparse
import json
from pathlib import Path
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parent))
import video_file_sender as video


def statistics(values):
    return dict(samples=len(values), mean_ms=float(np.mean(values)),
        median_ms=float(np.median(values)), p95_ms=float(np.percentile(values,95)),
        min_ms=float(np.min(values)), max_ms=float(np.max(values))) if values else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--video', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=30.)
    parser.add_argument('--device-ip', default='169.254.100.228')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    sent = {}
    rows = []
    lock = threading.Lock()
    fit = video.fit_frame
    send = video.send_message
    prepare_ns = None
    def timed_fit(frame):
        nonlocal prepare_ns
        prepare_ns = time.monotonic_ns()
        return fit(frame)
    def timed_send(connection, header, payload=b''):
        if header.get('op') == 'frame':
            with lock:
                sent[header['frame_index']] = dict(prepare_ns=prepare_ns, send_ns=time.monotonic_ns())
        return send(connection, header, payload)
    video.fit_frame, video.send_message = timed_fit, timed_send
    rclpy.init()
    node = Node('oak_roundtrip_latency_measurement')
    def timing(message):
        observed_ns = time.monotonic_ns()
        value = json.loads(message.data)
        source = value['source']
        if source.get('input_source') != 'pc' or source.get('source_file') != args.video.name:
            return
        index = source.get('source_frame_index')
        with lock:
            start = sent.get(index)
        if start is None:
            return
        published = value['result_publish_returned_monotonic_ns']
        received = value['host_bundle_received_monotonic_ns']
        callback = value['callback_monotonic_ns']
        processed = value['processed_monotonic_ns']
        rows.append(dict(frame_index=index, media_time_s=source.get('source_media_time_s'),
            jpeg_prepare_ms=(start['send_ns']-start['prepare_ns'])/1e6,
            send_to_result_publish_ms=(published-start['send_ns'])/1e6,
            prepare_to_result_publish_ms=(published-start['prepare_ns'])/1e6,
            send_to_host_bundle_ms=(received-start['send_ns'])/1e6,
            host_bundle_to_ros_callback_ms=(callback-received)/1e6,
            host_postprocess_ms=(processed-callback)/1e6,
            result_serialize_publish_ms=(published-processed)/1e6,
            result_publish_to_timing_receive_ms=(observed_ns-published)/1e6))
    subscription = node.create_subscription(String, 'oak/postprocess/timing', timing, 10)
    sender = video.VideoFileSender(args.device_ip, args.video, loop=False, fps=10, end_s=args.seconds)
    report = {}
    try:
        deadline = time.monotonic()+2
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        sender.start()
        deadline = time.monotonic()+args.seconds+30
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            status = sender.snapshot()
            if status['state'] == '오류':
                raise RuntimeError(status['error'])
            if status['state'] == '재생 완료':
                drain = time.monotonic()+2
                while time.monotonic() < drain:
                    rclpy.spin_once(node,timeout_sec=.02)
                break
        # Separate steady state from the initial source-switch warmup.
        steady = [row for row in rows if row['media_time_s'] >= 3.]
        fields = [key for key in rows[0] if key.endswith('_ms')] if rows else []
        report = dict(video=str(args.video), input_fps=10, export_fps_limit=None,
            duration_s=args.seconds, samples=len(rows), warmup_excluded_s=3.,
            all_samples={field:statistics([row[field] for row in rows]) for field in fields},
            steady_state={field:statistics([row[field] for row in steady]) for field in fields},
            sender=sender.snapshot(), passed=len(steady)>=20)
        (args.output/'samples.json').write_text(json.dumps(rows,indent=2))
    finally:
        sender.close()
        report['camera_restore'] = video.select_camera(args.device_ip)
        (args.output/'report.json').write_text(json.dumps(report,indent=2))
        video.fit_frame, video.send_message = fit, send
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(report,indent=2))
    return 0 if report.get('passed') else 1


if __name__=='__main__':
    raise SystemExit(main())
