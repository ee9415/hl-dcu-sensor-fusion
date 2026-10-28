"""Compare trackers on an explicitly synthetic, repeated false boundary."""
import argparse
import importlib.util
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
APP = ROOT / 'apps/yolopv2_standalone'
sys.path.insert(0, str(APP))


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module.ImageBoundaryTracker


def replay(cls):
    tracker = cls()
    def curve(x):
        return np.column_stack((np.full(48, float(x)), tracker.rows))
    tracker.history = [curve(80), curve(200)]
    tracker.scores = [.8, .05]
    tracker.last_valid_pair = [curve(80), curve(200)]
    results = []
    for index in range(3):
        observations = [(curve(80), .8, np.ones(48, bool)),
                        (curve(245), .8, np.ones(48, bool))]
        result = tracker._fuse(observations, [p.copy() for p in tracker.history],
                               np.eye(2, 3), 0., .1)
        results.append({'index': index, 'right_x_row_20': float(result.image_right[20, 0]),
                        'state': result.right_state, 'confidence': result.right_confidence,
                        'diagnostics': result.detection_diagnostics[1]})
    return results


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--candidate', type=Path, required=True,
                   help='Isolated candidate image_boundaries.py to compare')
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    baseline = load(APP/'image_boundaries.py', 'false_candidate_baseline')
    candidate = load(a.candidate, 'false_candidate_experiment')
    result = {'synthetic': True, 'assumed_true_right_x': 200,
              'injected_false_right_x': 245,
              'note': 'Artificial constant candidates supplied directly to fusion; not a real-world frequency estimate.',
              'baseline': replay(baseline), 'candidate': replay(candidate)}
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(json.dumps(result, indent=2), encoding='utf8')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
