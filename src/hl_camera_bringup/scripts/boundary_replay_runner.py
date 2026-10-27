"""Deterministic boundary/stop-line/render replay for bit-exact A/B checks.

Self-contained (stdlib + NumPy); runs on the OAK4 container or in WSL.
  run      Import one source tree, replay every sequence, pickle per-frame records.
  compare  Exactly compare two record files (dtype, shape, bytes; NaN-aware).

Each sequence starts from a fresh tracker, mirroring Yolopv2Postprocess, and a
source_epoch change inside captured frames resets it exactly as on the device.
Comparison tolerance is zero: any byte difference is reported.
"""
import argparse
import hashlib
import importlib
import json
import pickle
import platform
import struct
import sys
import types
from pathlib import Path

import numpy as np


class CapturedFrame:
    """Stands in for dai.ImgFrame; keeps the exact array given to setCvFrame."""
    Type = types.SimpleNamespace(NV12=0, BGR888i=1, RAW16=2)

    def __init__(self):
        self.data = None

    def setCvFrame(self, data, kind):
        self.data = np.array(data, copy=True)

    def setTimestamp(self, value):
        pass

    def setSequenceNum(self, value):
        pass


class Preview:
    def __init__(self, image):
        # 6x nearest upsampling: the trackers' 1080p->180x320 sampling is then
        # the identity on the stored image, as captured on the device.
        self.full = np.repeat(np.repeat(image, 6, axis=0), 6, axis=1)

    def getCvFrame(self):
        return self.full

    def getTimestamp(self):
        return 0.

    def getSequenceNum(self):
        return 0


def import_tree(tree):
    sys.path.insert(0, str(tree))
    try:
        import depthai  # noqa: F401  (real runtime on the device)
    except ImportError:
        class Host:
            def createOutput(self, **kwargs):
                return None
        from unittest.mock import MagicMock
        stub = types.ModuleType('depthai')
        stub.node = types.SimpleNamespace(HostNode=Host, ThreadedHostNode=Host)
        stub.Node, stub.DatatypeEnum, stub.LengthUnit = MagicMock(), MagicMock(), MagicMock()
        stub.NNData = type('NNData', (), {})
        stub.ImgDetection = type('ImgDetection', (), {})
        stub.ImgFrame = CapturedFrame
        sys.modules['depthai'] = stub
    modules = {name: importlib.import_module(name)
               for name in ('image_boundaries', 'stop_line', 'lane_boundaries', 'yolopv2_node')}
    for name, module in modules.items():
        if Path(module.__file__).resolve().parent != Path(tree).resolve():
            raise RuntimeError(f'{name} imported from {module.__file__}, not {tree}')
    # Render through the real _result_frame, but capture the BGR array instead
    # of converting to NV12 inside the runtime.
    modules['yolopv2_node'].dai = types.SimpleNamespace(ImgFrame=CapturedFrame)
    return modules


def scene(rng, n, shift, curve, left=True, right=True, dashed=False, base=60, stop_y=None):
    image = np.full((180, 320, 3), base, np.uint8)
    image += rng.integers(0, 12, image.shape, dtype=np.uint8)
    if stop_y is not None:
        image[stop_y:stop_y+4, :] = 235  # transverse bar spanning the corridor
    for y in range(78, 176):
        if dashed and (y+3*n) % 24 < 9:
            continue
        for visible, x in ((left, 160-(y-65)*.9), (right, 160+(y-65)*.9)):
            if visible:
                x = int(x+shift+curve*((y-125)/50)**2)
                image[y, max(0, x-1):min(320, x+2)] = 230
    road = np.zeros((180, 320), np.uint8)
    road[90:] = 255
    lane = np.zeros((180, 320), np.uint8)
    return image, road, lane


def synthetic_sequences():
    """Covers observe, loss/hold, reacquire, dashes, dt reset and a scene cut."""
    rng = np.random.default_rng(20261027)
    frames, t = [], 0.
    for n in range(160):
        shift = 8*np.sin(n/9.)
        curve = 14*np.sin(n/23.)
        left = not 40 <= n < 58
        right = not 75 <= n < 85
        dashed = 95 <= n < 125
        base = 200 if 135 <= n < 140 else 60  # bright cut, then back
        stop_y = 150-(n-142)//2 if 142 <= n < 156 else None  # approaching stop line
        image, road, lane = scene(rng, n, shift, curve, left, right, dashed, base, stop_y)
        if n % 7 == 0:
            lane[np.abs(np.arange(320)-(160+shift))[None, :].repeat(180, 0) < 2] = 255
        t += 1.5 if n == 110 else .1  # dt>1 reset
        boxes = np.array([[.55, .55, .70, .80]]) if 60 <= n < 70 else np.zeros((0, 4))
        frames.append(dict(image=image, road=road, lane=lane, timestamp=t, boxes=boxes, epoch=0))
    return {'synthetic_motion_loss_reacquire': frames}


def fixture_sequences(tree):
    path = Path(tree)/'tests'/'fixtures'/'parked_dashed_right_20261023.npz'
    if not path.exists():
        return {}
    f = np.load(path)
    road = f['road'].astype(np.uint8)*255
    frames = []
    for n in range(40):
        shift = int(round(3*np.sin(n/5.)))
        frames.append(dict(image=np.roll(f['rgb'], shift, axis=1), road=np.roll(road, shift, axis=1),
                           lane=np.zeros((180, 320), np.uint8), timestamp=.1*(n+1),
                           boxes=f['boxes'], epoch=0))
    return {'fixture_parked_dashed_right': frames}


def captured_sequences(directory):
    files = sorted(Path(directory).glob('*.npz')) if directory else []
    frames = []
    for path in files:
        f = np.load(path)
        frames.append(dict(image=f['image'], road=f['road'], lane=f['lane'],
                           timestamp=float(f['timestamp']), boxes=f['boxes'].reshape(-1, 4),
                           epoch=int(f['epoch']), frame_index=int(f['frame_index'])))
    return {'captured_device': frames} if frames else {}


def snapshot(obj, skip=('config',)):
    return {k: pickle.loads(pickle.dumps(v)) for k, v in vars(obj).items() if k not in skip}


def replay(modules, frames):
    ib, sl, node = modules['image_boundaries'], modules['stop_line'], modules['yolopv2_node']
    config = modules['lane_boundaries'].BoundaryConfig.load()
    post = node.Yolopv2Postprocess.__new__(node.Yolopv2Postprocess)
    post._result_size, post._source_size = (960, 540), (1920, 1080)
    records, tracker, stop, epoch = [], None, None, object()
    for frame in frames:
        if frame['epoch'] != epoch:
            tracker, stop, epoch = ib.ImageBoundaryTracker(config), sl.StopLineTracker(), frame['epoch']
        detections = [types.SimpleNamespace(xmin=a, ymin=b, xmax=c, ymax=d, label=0, confidence=.9)
                      for a, b, c, d in frame['boxes']]
        estimate = tracker.update(frame['road'], frame['lane'], frame['timestamp'],
                                  rgb=frame['image'], detections=detections)
        stop_line = stop.update(frame['image'], estimate, frame['timestamp'], detections)
        measurements = [types.SimpleNamespace(valid=False, z_mm=0.) for _ in detections]
        result = post._result_frame(Preview(frame['image']), frame['road'], frame['lane'],
                                    detections, measurements, estimate, stop_line).data
        bev = modules['lane_boundaries'].render_boundary_bev(estimate)
        sl.draw_stop_line(bev, stop_line, estimate)
        node.Yolopv2Postprocess._draw_text(bev, 'BOUNDARIES '+estimate.state.upper().replace('_', ' ')
                                           + ' REF '+estimate.reference_side.upper(), 10, 10, (0, 220, 255))
        records.append(dict(estimate=snapshot(estimate, ()), telemetry=estimate.telemetry(config),
                            tracker=snapshot(tracker), stop_line=stop_line.telemetry(),
                            stop_tracker=snapshot(stop), result_sha256=hashlib.sha256(result).hexdigest(),
                            bev_sha256=hashlib.sha256(np.ascontiguousarray(bev)).hexdigest()))
    return records


def differences(a, b, path='', out=None):
    out = [] if out is None else out
    if len(out) >= 20:
        return out
    if type(a) is not type(b):
        out.append(f'{path}: type {type(a).__name__} != {type(b).__name__}')
    elif isinstance(a, np.ndarray):
        if a.dtype != b.dtype or a.shape != b.shape or a.tobytes() != b.tobytes():
            out.append(f'{path}: array {a.dtype}{a.shape} differs')
    elif isinstance(a, dict):
        if a.keys() != b.keys():
            out.append(f'{path}: keys {sorted(set(a) ^ set(b))}')
        for k in a.keys() & b.keys():
            differences(a[k], b[k], f'{path}.{k}', out)
    elif isinstance(a, (list, tuple)):
        if len(a) != len(b):
            out.append(f'{path}: length {len(a)} != {len(b)}')
        for i, (x, y) in enumerate(zip(a, b)):
            differences(x, y, f'{path}[{i}]', out)
    elif isinstance(a, (float, np.floating)):
        if struct.pack('<d', float(a)) != struct.pack('<d', float(b)):
            out.append(f'{path}: {a!r} != {b!r}')
    elif a != b:
        out.append(f'{path}: {a!r} != {b!r}')
    return out


def environment():
    return dict(numpy=np.__version__, python=platform.python_version(), machine=platform.machine())


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest='command', required=True)
    r = sub.add_parser('run')
    r.add_argument('--tree', required=True, type=Path, help='apps/yolopv2_standalone source tree')
    r.add_argument('--captured', type=Path, help='directory of captured device input .npz frames')
    r.add_argument('--output', required=True, type=Path)
    c = sub.add_parser('compare')
    c.add_argument('baseline', type=Path)
    c.add_argument('candidate', type=Path)
    args = p.parse_args()
    if args.command == 'run':
        modules = import_tree(args.tree)
        sequences = {**synthetic_sequences(), **fixture_sequences(args.tree),
                     **captured_sequences(args.captured)}
        result = dict(environment=environment(),
                      sequences={name: replay(modules, frames) for name, frames in sequences.items()})
        args.output.write_bytes(pickle.dumps(result))
        print(json.dumps({name: len(v) for name, v in result['sequences'].items()}))
        return
    a, b = (pickle.loads(path.read_bytes()) for path in (args.baseline, args.candidate))
    report = dict(environment=a['environment'], candidate_environment=b['environment'],
                  sequences={}, identical=a['environment'] == b['environment'])
    for name in sorted(a['sequences'].keys() | b['sequences'].keys()):
        x, y = a['sequences'].get(name, []), b['sequences'].get(name, [])
        diff = [] if len(x) == len(y) else [f'frame count {len(x)} != {len(y)}']
        first = None
        for i, (u, v) in enumerate(zip(x, y)):
            found = differences(u, v, f'[{i}]')
            if found and first is None:
                first = i
            diff.extend(found[:20-len(diff)])
        report['sequences'][name] = dict(frames=len(x), identical=not diff,
                                         first_mismatch_frame=first, differences=diff)
        report['identical'] &= not diff
    print('REPLAY_REPORT='+json.dumps(report))
    sys.exit(0 if report['identical'] else 1)


if __name__ == '__main__':
    main()
