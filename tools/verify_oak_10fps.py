"""Prove 10FPS input -> actual NN completions -> every ROS topic, by source identity."""
import argparse
from collections import Counter
import json
from pathlib import Path
import socket
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray
from hl_oak_interfaces.msg import PerceptionFrame
from hl_oak_perception.bridge import QOS

sys.path.insert(0,str(Path(__file__).resolve().parent))
import video_file_sender as video


def stats(values):
    return dict(samples=len(values),mean=float(np.mean(values)),p95=float(np.percentile(values,95)),
                maximum=float(max(values))) if values else None


class Probe(Node):
    def __init__(self,path):
        super().__init__('oak_no_loss_10fps_verification')
        self.path=path
        self.frames={}
        self.results={}
        self.result_counts=Counter()
        self.frame_counts=Counter()
        self.receipts={}
        self.timing={}
        self.images={}
        self.invalid_depth=0
        self.errors=[]
        self.create_subscription(PerceptionFrame,'oak/frame',self.frame,QOS)
        self.create_subscription(String,'oak/postprocess/result',self.result,64)
        self.create_subscription(String,'oak/postprocess/timing',self.time_result,64)
        for name,kind in (
            ('rgb/image_raw',Image),('drivable_mask',Image),('lane_mask',Image),
            ('depth/image_raw',Image),('rgb/camera_info',CameraInfo),
            ('depth/camera_info',CameraInfo),('detections',Detection2DArray),
            ('postprocess/debug_image',Image),('postprocess/top_image',Image)):
            self.images[name]=Counter()
            self.create_subscription(kind,'oak/'+name,lambda m,n=name:self.image(n,m),QOS)

    @staticmethod
    def key(header):
        return header.stamp.sec,header.stamp.nanosec

    def is_input(self,source):
        return source.get('input_source')=='pc' and source.get('source_file')==self.path.name

    def frame(self,message):
        metadata=json.loads(message.metadata_json)
        if not self.is_input(metadata['source']):
            return
        index=metadata['source']['source_frame_index']
        self.frame_counts[index]+=1
        self.frames[index]=dict(key=self.key(message.header),nn_seq=metadata['nn_seq'],
            frame_seq=message.sequence,export_seq=metadata.get('export_seq'))
        if metadata['depth_available'] or np.any(np.frombuffer(message.depth.data,dtype='<u2')):
            self.invalid_depth+=1

    def result(self,message):
        received=time.monotonic_ns()
        value=json.loads(message.data)
        source=value.get('source',{})
        if value.get('stale') or not self.is_input(source):
            return
        index=source['source_frame_index']
        self.result_counts[index]+=1
        self.receipts[index]=received
        self.results[index]=dict(frame_seq=value['frame_seq'],export_seq=value.get('export_seq'),
            source_epoch=source['source_epoch'])

    def time_result(self,message):
        value=json.loads(message.data)
        if self.is_input(value['source']):
            self.timing[value['source']['source_frame_index']]=value

    def image(self,name,message):
        self.images[name][self.key(message.header)]+=1


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--video',type=Path,required=True)
    parser.add_argument('--seconds',type=float,default=60)
    parser.add_argument('--device-ip',default='169.254.100.228')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    expected=set(range(round(args.seconds*10)))
    completions={}
    completion_counts=Counter()
    stop=threading.Event()
    ready=threading.Event()
    telemetry_errors=[]
    connection=socket.create_connection((args.device_ip,8557),timeout=3)
    connection.settimeout(3)
    def telemetry():
        try:
            connection.sendall(b'HELLO\n')
            with connection.makefile('rb') as stream:
                json.loads(stream.readline())
                ready.set()
                while not stop.is_set():
                    line=stream.readline()
                    if not line:
                        break
                    value=json.loads(line)
                    if (value.get('input_source')=='pc' and value.get('source_file')==args.video.name
                            and 'nn_seq' in value):
                        index=value['source_frame_index']
                        completion_counts[index]+=1
                        completions[index]=dict(nn_seq=value['nn_seq'],frame_seq=value['frame_seq'])
        except (OSError,ValueError) as exc:
            if not stop.is_set():telemetry_errors.append(str(exc))
        finally:
            ready.set()
    worker=threading.Thread(target=telemetry,daemon=True)
    worker.start()
    sent={}
    original_send=video.send_message
    def tracked_send(connection,header,payload=b''):
        if header.get('op')=='frame':
            sent[header['frame_index']]=time.monotonic_ns()
        return original_send(connection,header,payload)
    video.send_message=tracked_send
    rclpy.init()
    node=Probe(args.video)
    sender=video.VideoFileSender(args.device_ip,args.video,loop=False,fps=10,end_s=args.seconds)
    report={}
    began=None
    try:
        if not ready.wait(5) or telemetry_errors:
            raise RuntimeError('Camera telemetry unavailable')
        discovery=time.monotonic()+2
        while time.monotonic()<discovery:
            rclpy.spin_once(node,timeout_sec=.02)
        began=time.monotonic()
        sender.start()
        deadline=began+args.seconds+20
        last_progress=began
        completed_at=None
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.01)
            status=sender.snapshot()
            if status['state']=='오류':
                raise RuntimeError(status['error'])
            if status['state']=='재생 완료' and completed_at is None:
                completed_at=time.monotonic()
            complete=(expected<=set(node.frames) and expected<=set(node.results)
                      and expected<=set(completions) and expected<=set(node.timing))
            keys={row['key'] for row in node.frames.values()}
            images_complete=all(keys<=set(counter) for counter in node.images.values())
            if completed_at and ((complete and images_complete) or time.monotonic()-completed_at>10):
                break
            if time.monotonic()-last_progress>=10:
                progress=dict(elapsed_s=round(time.monotonic()-began,1),sent=len(sent),
                    nn=len(completions),raw_ros=len(node.frames),results=len(node.results))
                (args.output/'progress.json').write_text(json.dumps(progress))
                print(json.dumps(progress),flush=True)
                last_progress=time.monotonic()
        identities={}
        for name,values in (('input',sent),('nn',completions),('raw_ros',node.frames),('result',node.results)):
            identities[name]=dict(count=len(values),missing=sorted(expected-set(values)),
                                  extra=sorted(set(values)-expected))
        by_index={value['key']:index for index,value in node.frames.items()}
        images={}
        for name,counter in node.images.items():
            indices={by_index[key] for key in counter if key in by_index}
            images[name]=dict(count=len(indices),missing=sorted(expected-indices),
                duplicates=sum(max(0,count-1) for key,count in counter.items() if key in by_index))
        sequence_errors=[]
        for index in expected & set(completions) & set(node.frames) & set(node.results):
            camera,frame,result=completions[index],node.frames[index],node.results[index]
            if not (camera['nn_seq']==camera['frame_seq']==frame['nn_seq']==frame['frame_seq']==result['frame_seq']):
                sequence_errors.append(index)
        steady=sorted(index for index in expected & set(node.receipts) & set(sent) if index>=30)
        latencies=[(node.receipts[i]-sent[i])/1e6 for i in steady]
        output_span=(node.receipts[steady[-1]]-node.receipts[steady[0]])/1e9 if len(steady)>1 else None
        rate=(len(steady)-1)/output_span if output_span else 0
        duplicates=dict(nn=sum(max(0,n-1) for n in completion_counts.values()),
            raw_ros=sum(max(0,n-1) for n in node.frame_counts.values()),
            result=sum(max(0,n-1) for n in node.result_counts.values()))
        stages={}
        for name,start,end in (
            ('host_delivery_ms','host_bundle_received_monotonic_ns','callback_monotonic_ns'),
            ('postprocess_ms','callback_monotonic_ns','processed_monotonic_ns'),
            ('publish_ms','processed_monotonic_ns','result_publish_returned_monotonic_ns')):
            stages[name]=stats([(node.timing[i][end]-node.timing[i][start])/1e6
                               for i in steady if i in node.timing])
        report=dict(duration_s=args.seconds,input_fps=10,expected_frames=len(expected),
            identities=identities,topics=images,duplicates=duplicates,
            sequence_mismatches=sequence_errors,invalid_pc_depth_frames=node.invalid_depth,
            steady_output_fps=rate,send_to_result_receive_ms=stats(latencies),stages=stages,
            elapsed_s=time.monotonic()-began,sender=sender.snapshot(),telemetry_errors=telemetry_errors,
            passed=all(not row['missing'] and not row['extra'] for row in identities.values())
                and all(not row['missing'] and not row['duplicates'] for row in images.values())
                and not any(duplicates.values()) and not sequence_errors and not node.invalid_depth
                and not telemetry_errors and rate>=9.5)
        (args.output/'identities.json').write_text(json.dumps(dict(camera=completions,
            raw_ros=node.frames,results=node.results,timing=node.timing,sent=sent),indent=2))
    finally:
        sender.close()
        report['camera_restore']=video.select_camera(args.device_ip)
        stop.set()
        try:connection.shutdown(socket.SHUT_RDWR)
        except OSError:pass
        worker.join(timeout=4)
        connection.close()
        video.send_message=original_send
        (args.output/'report.json').write_text(json.dumps(report,indent=2))
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(report,indent=2),flush=True)
    return 0 if report.get('passed') else 1


if __name__=='__main__':
    raise SystemExit(main())
