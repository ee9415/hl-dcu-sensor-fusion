"""Measure arrival jitter, continuity and host latency without GUI timing guesses."""
import argparse
import json
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=120)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('oak_stream_interval_check')
    arrivals = {}
    subscriptions = []
    statuses = []
    latencies = []
    discontinuities = []
    previous = None
    stale = 0

    def receive(message, topic):
        nonlocal previous, stale
        arrivals[topic].append(time.monotonic())
        if topic.endswith('/result'):
            data = json.loads(message.data)
            stale += bool(data.get('stale'))
            identity = (data.get('session'), data.get('export_seq'))
            if not data.get('stale') and identity[1] is not None:
                if previous and previous[0] == identity[0] and identity[1] != previous[1]+1:
                    discontinuities.append([previous, identity])
                previous = identity
        elif topic.endswith('/bridge_status'):
            statuses.append(json.loads(message.data))
        elif topic.endswith('/timing'):
            data = json.loads(message.data)
            received = data.get('host_bundle_received_monotonic_ns')
            if received:
                latencies.append((data['result_publish_returned_monotonic_ns']-received)/1e6)

    for topic, kind in (
        ('/oak/postprocess/debug_image', Image), ('/oak/postprocess/top_image', Image),
        ('/oak/postprocess/result', String), ('/oak/bridge_status', String),
        ('/oak/postprocess/timing', String),
    ):
        arrivals[topic] = []
        subscriptions.append(node.create_subscription(kind, topic,
            lambda message, topic=topic: receive(message, topic),
            QoSProfile(depth=32, reliability=ReliabilityPolicy.RELIABLE)))
    start = time.monotonic()
    progress = start
    while time.monotonic()-start < args.seconds:
        rclpy.spin_once(node, timeout_sec=.05)
        if time.monotonic()-progress >= 30:
            print(json.dumps(dict(elapsed=round(time.monotonic()-start),
                counts={topic: len(rows) for topic, rows in arrivals.items()},
                stale=stale, discontinuities=len(discontinuities))), flush=True)
            progress = time.monotonic()
    report = dict(seconds=time.monotonic()-start, topics={}, stale=stale,
        discontinuities=discontinuities, bridge_first=statuses[:1], bridge_last=statuses[-1:])
    for topic, rows in arrivals.items():
        delta = np.diff(rows)
        stats = dict(count=len(rows))
        if len(delta):
            stats.update(fps=len(delta)/(rows[-1]-rows[0]),
                interval_ms_p50_p95_max=(np.percentile(delta, [50, 95, 100])*1000).tolist(),
                gaps_over_250ms=int(sum(delta > .25)), bursts_under_20ms=int(sum(delta < .02)))
        report['topics'][topic] = stats
    if latencies:
        report['host_latency_ms_p50_p95_max'] = np.percentile(latencies, [50, 95, 100]).tolist()
    path = Path(args.output)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report), flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
