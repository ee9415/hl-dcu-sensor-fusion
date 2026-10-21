"""Summarize the latest telemetry session's observed-to-inferred continuity."""
import argparse
import csv
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('csv', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    with args.csv.open(encoding='utf-8-sig', newline='') as stream:
        # The multiview may be appending its last row during this snapshot.
        rows = [row for row in csv.DictReader(stream)
                if row.get('right_boundary_dev') is not None]
    session = rows[-1]['session']
    rows = [row for row in rows if row['session'] == session]
    ages, counts = {}, {}
    for side in ('left', 'right'):
        last = None
        longest = 0.
        count = 0
        for row in rows:
            timestamp = float(row['nn_timestamp_s'])
            state = row[f'{side}_boundary_state']
            if state == 'observed':
                last = timestamp
            elif state in ('inferred', 'held') and last is not None:
                age = timestamp-last
                longest = max(longest, age)
                count += int(age > 2.)
            else:
                last = None
        ages[side] = longest
        counts[side] = count
    result = dict(session=session, frames=len(rows),
                  longest_continuation_s=ages, continued_after_2s_frames=counts,
                  centerline_true=sum(r['centerline_valid'] == 'True' for r in rows),
                  sequence_gaps=sum(int(r['sequence_gap'] or 0) for r in rows))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
