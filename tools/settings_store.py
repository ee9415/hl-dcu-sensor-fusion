"""Single active settings document; atomic updates preserve unrelated sections."""
# 통합 설정 파일(artifacts/config/settings.json) 하나를 읽고 쓰는 공통 모듈.
# 섹션 단위로 갱신하며, 원자적 교체로 다른 섹션과 원본 파일을 보호한다.
import json
import os
from pathlib import Path
import shutil
import tempfile
import threading
import math

DEFAULT_PATH = Path(__file__).resolve().parents[1] / 'artifacts/config/settings.json'
# 여러 스레드(GUI, 녹화 등)가 동시에 설정을 갱신해도 섞이지 않게 하는 잠금
LOCK = threading.RLock()


def preferences(data):
    """화면/녹화 환경설정(preferences) 값을 검사한다. 잘못된 값이면 ValueError."""
    if not isinstance(data,dict):raise ValueError('잘못된 화면 설정')
    display=data['display'];recording=data['recording'];selected=data['selected']
    if any(type(display[key]) is not bool for key in ('full_processing','boundary_visible')):
        raise ValueError('잘못된 표시 설정')
    if type(display.get('contour_visible',False)) is not bool:
        raise ValueError('잘못된 컨투어 표시 설정')
    if display['lane_mode'] not in ('normal','lane_score','lane_mask','raw_rgb','lane_bands','lane_bands_rgb','lane_compare'):
        raise ValueError('잘못된 차선 표시 모드')
    threshold=display['threshold']
    if type(threshold) not in (int,float) or not math.isfinite(threshold) or not 0<=threshold<=1:
        raise ValueError('잘못된 차선 임계값')
    speed=display.get('skeleton_speed',1.0)
    if type(speed) not in (int,float) or not math.isfinite(speed) or not .5<=speed<=3:
        raise ValueError('잘못된 스켈레톤 반응 속도')
    if set(selected)!={'rgb','yolopv2'} or any(type(v) is not bool for v in selected.values()):
        raise ValueError('잘못된 녹화 영상 선택')
    if any(not isinstance(recording[k],str) for k in ('output_dir','session_name')):
        raise ValueError('잘못된 녹화 저장 경로')
    if recording['resolution'] not in ('원본 스트림','1920×1080','1280×720') or recording['fps'] not in ('10','15','20','30'):
        raise ValueError('잘못된 녹화 해상도 또는 FPS')
    if type(recording['verify_output']) is not bool:raise ValueError('잘못된 녹화 검증 설정')
    return data


def read(path):
    """설정 파일 전체를 dict로 읽는다. 파일이 없으면 빈 dict."""
    path=Path(path)
    if not path.exists(): return {}
    data=json.loads(path.read_text(encoding='utf-8'))
    if not isinstance(data,dict): raise ValueError('잘못된 설정 파일')
    return data


def update(path, changes):
    """changes의 섹션만 바꿔 저장한다. 내용이 같으면 쓰지 않고, 이전 파일은 .bak으로 남긴다."""
    path=Path(path)
    with LOCK:
        data=read(path);data.update(changes)
        if path.exists() and read(path)==data:return
        path.parent.mkdir(parents=True,exist_ok=True)
        temporary=None
        try:
            with tempfile.NamedTemporaryFile('w',encoding='utf-8',dir=path.parent,
                                             prefix='settings_',suffix='.tmp',delete=False) as stream:
                temporary=Path(stream.name)
                json.dump(data,stream,ensure_ascii=False,indent=2)
                stream.flush();os.fsync(stream.fileno())
            if path.exists():shutil.copy2(path,path.with_suffix(path.suffix+'.bak'))
            temporary.replace(path)
        finally:
            if temporary is not None:temporary.unlink(missing_ok=True)


def migrate(path=DEFAULT_PATH):
    """One-time import; previous files remain as rollback evidence, never rewritten."""
    # 예전 개별 설정 파일(cameras.json, video_input.json, credentials.json)을 통합 파일로 한 번만 옮긴다.
    # 원본은 before_unified_settings 폴더에 복사해 둔다.
    path=Path(path)
    if path.exists():return
    folder=path.parent;data={};sources=[]
    for filename,section in [('cameras.json',None),('video_input.json','video_input'),
                              ('credentials.json','credentials')]:
        source=folder/filename
        if not source.exists():continue
        value=read(source)
        if section:data[section]=value
        else:data.update(value)
        sources.append(source)
    legacy=Path(os.environ.get('APPDATA',Path.home()/'AppData/Roaming'))/'OAK4DPro/credentials.json'
    if legacy.exists():
        credentials=read(legacy);credentials.update(data.get('credentials',{}))
        data['credentials']=credentials;sources.append(legacy)
    if not data:return
    backup=folder/'before_unified_settings'
    backup.mkdir(parents=True,exist_ok=True)
    for index,source in enumerate(sources):shutil.copy2(source,backup/(str(index)+'_'+source.name))
    update(path,data)
