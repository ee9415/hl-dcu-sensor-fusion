"""Inspect a YOLOPv2 RVC4 archive and safely extract its referenced DLC."""
import argparse
import hashlib
import json
from pathlib import Path
import tarfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    with tarfile.open(args.archive, 'r:xz') as archive:
        config = json.load(archive.extractfile('config.json'))
        model = config['model']
        dlc_name = model['metadata']['path']
        if Path(dlc_name).name != dlc_name or not dlc_name.endswith('.dlc'):
            raise ValueError('Expected a single DLC filename in archive metadata')
        inputs = model['inputs']
        if len(inputs) != 1 or inputs[0]['shape'] not in ([1,320,320,3], [1,3,320,320]):
            raise ValueError(f'Unexpected input: {inputs}')
        expected = {
            'det_stride_8': ([1,40,40,255], [1,255,40,40]),
            'det_stride_16': ([1,20,20,255], [1,255,20,20]),
            'det_stride_32': ([1,10,10,255], [1,255,10,10]),
            'drivable_area': ([1,320,320,2], [1,2,320,320]),
            'lane_line': ([1,320,320,1], [1,1,320,320]),
        }
        outputs = {o['name']: o for o in model['outputs']}
        if set(outputs) != set(expected):
            raise ValueError(f'Unexpected output names: {list(outputs)}')
        for name, shapes in expected.items():
            if outputs[name]['shape'] not in shapes:
                raise ValueError(f'Unexpected output shape: {outputs[name]}')
        data = archive.extractfile(dlc_name).read()
        if not data:
            raise ValueError('Empty DLC')
        target = args.output / dlc_name
        if target.exists() and target.read_bytes() != data:
            raise FileExistsError(f'Refusing to overwrite different DLC: {target}')
        target.write_bytes(data)
        buildinfo = json.load(archive.extractfile('buildinfo.json')) if 'buildinfo.json' in archive.getnames() else None
    result = dict(archive=str(args.archive), archive_sha256=hashlib.sha256(args.archive.read_bytes()).hexdigest(),
                  dlc=str(target), dlc_sha256=hashlib.sha256(data).hexdigest(), dlc_bytes=len(data),
                  config=config, buildinfo=buildinfo, tensor_schema_verified=True,
                  device_inference_verified=False)
    (args.output / 'archive_inspection.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
