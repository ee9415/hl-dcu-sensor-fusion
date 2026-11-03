"""Read-only OAK4/DepthAI2 device discovery; never starts a camera pipeline."""
from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import subprocess

ROOT=Path(__file__).resolve().parents[1]


def run(command):
    completed=subprocess.run(command,capture_output=True,text=True,encoding='utf-8',
        errors='replace',timeout=20,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
    if completed.returncode:
        raise RuntimeError((completed.stderr or completed.stdout)[-700:].strip())
    return completed.stdout


def parse_oak4(text):
    # oakctl can prepend an update notice; parse only its JSON object.
    start=text.find('{')
    if start<0:raise ValueError('oakctl 검색 결과에 JSON이 없습니다.')
    payload=json.JSONDecoder().raw_decode(text[start:])[0]
    return [dict(family='oak4',serial=str(d['serial_number']),
                 ip=(d.get('ip_addresses') or [''])[0],model=d.get('model','OAK4'),
                 state=d.get('agent_mode','')) for d in payload.get('items',[])]


def parse_oakd(text):
    for line in text.splitlines():
        if line.startswith('OAKDISCOVERY '):
            return json.loads(line[len('OAKDISCOVERY '):])
    raise ValueError('OAK-D 검색 결과를 읽을 수 없습니다.')


def windows_fallback():
    """Use Windows networking when WSL broadcast discovery cannot reach PoE."""
    profile=run(['cmd.exe','/c','echo','%USERPROFILE%']).strip()
    python=profile+r'\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe'
    python=run(['wslpath','-u',python]).strip()
    sdk=ROOT/'artifacts/oakd_windows_sdk'
    if not sdk.exists():
        raise RuntimeError('Windows 검색 SDK가 없습니다. IP 직접 연결을 사용해 주세요.')
    script=run(['wslpath','-w',str(Path(__file__).with_name('discover_oakd_worker.py'))]).strip()
    sdk_path=run(['wslpath','-w',str(sdk)]).strip()
    return parse_oakd(run([python,script,'--sdk-path',sdk_path]))


def discover_cameras(oakctl,oakd_python):
    devices=[];warnings=[]
    def oak4():
        return parse_oak4(run([str(oakctl),'device','list','--all','--format','json']))
    def oakd():
        return parse_oakd(run([str(oakd_python),str(Path(__file__).with_name('discover_oakd_worker.py'))]))
    with ThreadPoolExecutor(max_workers=2) as pool:
        tasks=[('OAK4',pool.submit(oak4)),('OAK-D',pool.submit(oakd))]
        local_oakd=[]
        for name,task in tasks:
            try:
                found=task.result()
                devices.extend(found)
                if name=='OAK-D':local_oakd=found
            except Exception as exc:
                warnings.append(name+': '+str(exc))
    if os.environ.get('WSL_DISTRO_NAME') and not local_oakd:
        try:
            devices.extend(windows_fallback())
        except Exception as exc:
            warnings.append('Windows 검색: '+str(exc))
    unique={(d['family'],d['serial']):d for d in devices}
    return list(unique.values()),warnings
