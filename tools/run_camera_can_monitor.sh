#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG="$ROOT/artifacts/camera_can/monitor_config.json"
python3 - "$ROOT" "$CONFIG" "${1:-}" "${2:-53200}" "${3:-}" <<'PY'
import json,sys,subprocess
from pathlib import Path
root, output = Path(sys.argv[1]), Path(sys.argv[2])
host = sys.argv[3] or json.loads(subprocess.check_output(
    ['ip','-json','route','show','default']))[0]['gateway']
config = json.loads((root/'src/hl_camera_can/config/camera_can.json').read_text())
if sys.argv[5]:
    if sys.argv[5] != 'bench':
        raise ValueError('Third argument must be bench or empty')
    overrides=json.loads((root/'src/hl_camera_can/config/camera_can.monitor_bench.json').read_text())
    for section,values in overrides.items():config[section].update(values)
config['can'].update(driver_backend='windows_tcp', windows_host=host,
    windows_port=int(sys.argv[4]), windows_key_file=str(root/'artifacts/camera_can/windows.key'),
    windows_allow_virtual=False, windows_monitor_only=True)
output.parent.mkdir(parents=True, exist_ok=True)
output.write_text(json.dumps(config,indent=2),encoding='utf-8')
print('Windows monitor only: '+host+':'+sys.argv[4],flush=True)
if sys.argv[5]:print('UNCALIBRATED BENCH PROFILE: provisional geometry/quality; monitor only',flush=True)
PY
exec bash "$ROOT/tools/run_camera_can.sh" "config_path:=$CONFIG" enable_driver:=true
