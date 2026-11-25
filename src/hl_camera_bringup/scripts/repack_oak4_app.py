"""Rebuild an OAK4 package offline using unchanged verified runtime layers.

The source must be a minimal staged application directory. This replaces the
entire app layer, preserving the baseline package and its two runtime layers.
"""
import argparse
import hashlib
import io
import json
from pathlib import Path
import tarfile
import tomllib
from datetime import datetime, timezone

import lz4.frame


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    configuration = tomllib.loads((args.source/'oakapp.toml').read_text())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    layer = args.output.with_suffix('.app-layer.lz4')
    with tarfile.open(args.baseline) as baseline:
        metadata = json.load(baseline.extractfile('metadata.json'))
        if metadata['identifier'] != configuration['identifier']:
            raise ValueError('Package identifier mismatch')
        layers = [m.name for m in baseline.getmembers() if m.name.startswith('layers/')]
        if layers != [f'layers/lower-{n}.tar.lz4' for n in range(3)]:
            raise ValueError(f'Unsupported baseline layer structure: {layers}')
        old_files = {}
        with lz4.frame.open(baseline.extractfile(layers[2]), 'rb') as stream:
            with tarfile.open(fileobj=stream, mode='r|') as old_app:
                with lz4.frame.open(layer, 'wb', compression_level=0) as compressed:
                    with tarfile.open(fileobj=compressed, mode='w|') as new_app:
                        for member in old_app:
                            name = member.name.removeprefix('./')
                            if name == 'app' or name.startswith('app/'):
                                if name in ('app/requirements.txt', 'app/install-runtime.sh', 'app/runtime-base.toml'):
                                    old_files[Path(name).name] = old_app.extractfile(member).read()
                                continue
                            new_app.addfile(member, old_app.extractfile(member) if member.isfile() else None)

                        def normalized(member):
                            if member.name == 'app/oakapp.toml':
                                return None
                            member.uid = member.gid = 0
                            member.uname = member.gname = 'root'
                            member.mode = 0o755 if member.isdir() else 0o644
                            return member

                        new_app.add(args.source, arcname='app', filter=normalized)
        for name, original in old_files.items():
            if original.replace(b'\r\n', b'\n') != (args.source/name).read_bytes().replace(b'\r\n', b'\n'):
                raise ValueError(f'Runtime dependency file changed: {name}; build a new runtime instead')
        metadata['app_version'] = configuration['app_version']
        metadata['system']['created_timestamp'] = datetime.now(timezone.utc).isoformat()
        metadata['entrypoint'] = configuration['entrypoint']
        hashes = {}
        with tarfile.open(args.output, 'w') as result:
            for member in baseline.getmembers():
                if member.name in (layers[2], 'metadata.json'):
                    continue
                payload = baseline.extractfile(member) if member.isfile() else None
                # Runtime layers are copied as-is, with no recompression.
                result.addfile(member, payload)
            result.add(layer, arcname=layers[2])
            data = json.dumps(metadata, separators=(',', ':')).encode()
            info = tarfile.TarInfo('metadata.json')
            info.size = len(data)
            info.mode = 0o644
            result.addfile(info, io.BytesIO(data))
        with tarfile.open(args.output) as result:
            for name in layers[:2]:
                def stream_hash(stream):
                    digest = hashlib.sha256()
                    for block in iter(lambda: stream.read(1024*1024), b''):
                        digest.update(block)
                    return digest.hexdigest()
                original = stream_hash(baseline.extractfile(name))
                copied = stream_hash(result.extractfile(name))
                if original != copied:
                    raise ValueError(f'Runtime layer hash mismatch: {name}')
                hashes[name] = copied
    report = dict(baseline=str(args.baseline), source=str(args.source), output=str(args.output),
                  bytes=args.output.stat().st_size, runtime_layers=hashes,
                  version=configuration['app_version'], runtime_layers_unchanged=True)
    args.output.with_suffix('.build.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
