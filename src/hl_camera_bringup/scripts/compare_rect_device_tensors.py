"""Capture rectangular RVC4 outputs for one deterministic input, restoring the app."""
import base64
import json
import subprocess
import sys
import time
import uuid
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
OAKCTL = r'C:\Program Files\oakctl\bin\oakctl.exe'
APP = '00000000-0000-0000-0000-000000000000'
DEVICE = '192.168.201.2'
OUTPUT = ROOT / 'artifacts' / 'rectangular_model_20261027' / 'device_tensor_compare'


def main():
    road = '--road' in sys.argv
    road_path = OUTPUT / 'road_input.jpg'
    if road and len(sys.argv) > sys.argv.index('--road') + 1:
        road_path = Path(sys.argv[sys.argv.index('--road') + 1])
    outdir = OUTPUT / (road_path.stem if road else 'synthetic')
    outdir.mkdir(parents=True, exist_ok=True)
    cmd = [OAKCTL, 'app', 'exec', '-d', DEVICE, APP, '--']
    def remote(code, timeout=120):
        payload = base64.b64encode(code.encode()).decode()
        p = subprocess.run(cmd + ['python3.12', '-c',
                           f"import base64;exec(base64.b64decode('{payload}'))"],
                           capture_output=True, text=True, timeout=timeout)
        if p.returncode or 'Traceback (most recent call last)' in p.stdout + p.stderr:
            raise RuntimeError(p.stdout + p.stderr)
        return p.stdout

    backup = '/app/tensor_compare_backup_' + uuid.uuid4().hex[:10]
    output = '/app/tensor_compare_' + uuid.uuid4().hex[:10] + '.npz'
    main_source = (ROOT / 'apps/yolopv2_standalone/main.py').read_text()
    node_source = (ROOT / 'apps/yolopv2_standalone/yolopv2_node.py').read_text()
    main_source = main_source.replace("PIPELINE_STAGE = os.environ.get(\"PIPELINE_STAGE\", \"full\").lower()", "PIPELINE_STAGE = 'static_nn'")
    main_source = main_source.replace("os.environ.get('YOLOPV2_INPUT_HEIGHT', '320')", "os.environ.get('YOLOPV2_INPUT_HEIGHT', '192')")
    main_source = main_source.replace('yolopv2_person_320.rvc4.tar.xz', 'yolopv2_person_320x192.rvc4.tar.xz')
    main_source = main_source.replace("if PIPELINE_STAGE == 'static_nn' and MODEL_SIZE != (320, 320):\n    raise ValueError('static_nn diagnostic currently requires 320x320')\n", '')
    main_source = main_source.replace('        archive = dai.NNArchive(MODEL_ARCHIVE)\n', '        preprocess.model_size = MODEL_SIZE\n        archive = dai.NNArchive(MODEL_ARCHIVE)\n', 1)
    main_source = main_source.replace('        pipeline.create(Yolopv2NnProbe).build(neural_network.out)', '        probe = pipeline.create(Yolopv2NnProbe).build(neural_network.out)\n        probe.model_size = MODEL_SIZE', 1)
    input_expression = 'diagnostic_bgr_frame()[:192].copy()'
    if road:
        encoded = base64.b64encode(road_path.read_bytes()).decode()
        node_source = 'import base64\nimport cv2\n' + node_source
        input_expression = f"cv2.imdecode(np.frombuffer(base64.b64decode({encoded!r}), dtype=np.uint8), cv2.IMREAD_COLOR)"
    node_source = node_source.replace('frame_data = diagnostic_bgr_frame()', 'frame_data = ' + input_expression, 1)
    old = '        self._frames += 1\n        now = time.monotonic()'
    capture = f'''        from pathlib import Path as _ProbePath
        if not _ProbePath({output!r}).exists():
            tensors = {{name: nn_data.getTensor(name, dequantize=True, storageOrder=dai.TensorInfo.StorageOrder.NCHW) for name in self.TENSOR_NAMES}}
            tensors['bgr'] = {input_expression}
            np.savez({output!r}, **tensors)
'''
    assert node_source.count(old) >= 1
    node_source = node_source.replace(old, capture + old, 1)
    assert "PIPELINE_STAGE = 'static_nn'" in main_source and 'preprocess.model_size = MODEL_SIZE' in main_source
    sources = {name: base64.b64encode(data.encode()).decode() for name, data in
               [('main.py', main_source), ('yolopv2_node.py', node_source)]}
    manifest = {'backup': backup, 'device_output': output, 'restored': False}
    (outdir / 'manifest.json').write_text(json.dumps(manifest, indent=2))
    installed = False
    try:
        remote(f'''from pathlib import Path
b=Path({backup!r}); b.mkdir()
for name in ['main.py','yolopv2_node.py']:
 p=Path('/app')/name; (b/name).write_bytes(p.read_bytes())
print('BACKED_UP')''')
        installed = True
        for name, data in sources.items():
            staged = backup + '/' + name + '.b64'
            for pos in range(0, len(data), 18000):
                remote(f"from pathlib import Path;p=Path({staged!r});p.open('a').write({data[pos:pos+18000]!r})")
        remote(f'''from pathlib import Path
import base64
for name in ['main.py','yolopv2_node.py']:
 raw=base64.b64decode((Path({backup!r})/(name+'.b64')).read_text()); compile(raw,name,'exec'); (Path('/app')/name).write_bytes(raw)
print('INSTALLED')''')
        subprocess.run(cmd + ['sv', 'restart', '/etc/service/backend'], check=True, capture_output=True, timeout=30)
        for _ in range(40):
            state = remote(f"from pathlib import Path;print('READY='+str(Path({output!r}).exists()))")
            if 'READY=True' in state:
                break
            time.sleep(2)
        else:
            raise RuntimeError('No device tensor capture after 80 seconds')
        text = remote(f"import base64;from pathlib import Path;print('DATA='+base64.b64encode(Path({output!r}).read_bytes()).decode())", timeout=180)
        blob = text.split('DATA=', 1)[1].splitlines()[0].strip()
        (outdir / 'device_tensors.npz').write_bytes(base64.b64decode(blob, validate=True))
        print('Captured device tensors:', (outdir / 'device_tensors.npz').stat().st_size, 'bytes', flush=True)
    finally:
        if installed:
            remote(f'''from pathlib import Path
import hashlib
b=Path({backup!r})
for name in ['main.py','yolopv2_node.py']:
 data=(b/name).read_bytes(); p=Path('/app')/name; p.write_bytes(data); assert p.read_bytes()==data
print('RESTORED')''')
            subprocess.run(cmd + ['sv', 'restart', '/etc/service/backend'], check=True, capture_output=True, timeout=30)
            manifest['restored'] = True
            (outdir / 'manifest.json').write_text(json.dumps(manifest, indent=2))
            print('Device sources restored and backend restarted', flush=True)


if __name__ == '__main__':
    main()
