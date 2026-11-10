"""Local camera registration persistence. No passwords or device operations."""
# CAM01~CAM06 카메라 등록 정보(IP, 장치 ID, 입력 종류, 녹화 선택)를 settings.json에 저장/로드한다.
# 비밀번호는 다루지 않으며 장치에 접속하지도 않는다.
import ipaddress
import json
from pathlib import Path
import os
import shutil
import tempfile
import settings_store

DEFAULT_PATH=settings_store.DEFAULT_PATH


def defaults(ip='',serial='',python=''):
    """기본 카메라 6대 설정. CAM01은 'OAK4 공유', 나머지는 '실제 OAK-D'이고 CAM01만 녹화 선택 상태다."""
    return dict(version=1,oakd_python=python,cameras=[dict(name=f'CAM{i+1:02d}',
        ip=ip if i==0 else '',serial=serial if i==0 else '',selected=i==0,
        kind='OAK4 공유' if i==0 else '실제 OAK-D') for i in range(6)])


def validate(data):
    """카메라 설정 형식을 검사해 정규화된 dict를 반환한다.
    IP 형식, 장치 ID 제어문자, 입력 종류, 실제 장치 간 IP/ID 중복(가상 영상은 제외)을 확인한다.
    """
    if not isinstance(data,dict) or data.get('version')!=1:raise ValueError('지원하지 않는 카메라 설정 형식입니다.')
    if not isinstance(data.get('cameras'),list) or len(data['cameras'])!=6:raise ValueError('카메라 설정은6개여야 합니다.')
    seen_ip=set();seen_id=set();result=[]
    for i,item in enumerate(data['cameras']):
        if not isinstance(item,dict) or item.get('name')!=f'CAM{i+1:02d}':raise ValueError('카메라 번호가 올바르지 않습니다.')
        ip=item.get('ip','');serial=item.get('serial','');kind=item.get('kind')
        if not isinstance(ip,str) or not isinstance(serial,str) or len(serial)>128:raise ValueError('잘못된 IP/장치ID 형식입니다.')
        ip,serial=ip.strip(),serial.strip()
        if ip:ipaddress.ip_address(ip)
        if any(ord(c)<32 for c in serial):raise ValueError('장치ID에 제어문자가 있습니다.')
        if kind not in (('OAK4 공유',) if i==0 else ('실제 OAK-D','가상 영상')):raise ValueError('입력 종류가 올바르지 않습니다.')
        if type(item.get('selected')) is not bool:raise ValueError('녹화 선택 값이 올바르지 않습니다.')
        if kind!='가상 영상':
            if (ip and ip in seen_ip) or (serial and serial in seen_id):raise ValueError('중복된 IP 또는 장치ID입니다.')
            if ip:seen_ip.add(ip)
            if serial:seen_id.add(serial)
        result.append(dict(name=item['name'],ip=ip,serial=serial,kind=kind,selected=item['selected']))
    python=data.get('oakd_python','')
    if not isinstance(python,str):raise ValueError('Python 경로가 올바르지 않습니다.')
    return dict(version=1,cameras=result,oakd_python=python,reset=bool(data.get('reset',False)))


def load(path):
    """settings.json에서 카메라 설정을 읽는다. 파일이 없으면 None."""
    path=Path(path)
    return validate(json.loads(path.read_text(encoding='utf-8'))) if path.exists() else None


def save(path,data):
    """카메라 설정을 원자적으로 저장한다. 다른 섹션은 유지하고, 내용이 같으면 쓰지 않으며, 이전 파일은 .bak으로 남긴다."""
    path=Path(path);data=validate(data)
    # Camera resets/updates must preserve source, display, recording and secrets.
    try:
        previous=settings_store.read(path)
        for key,value in previous.items():
            if key not in data:data[key]=value
    except (OSError,ValueError):pass
    try:
        if settings_store.read(path)==data:return
    except (OSError,ValueError,TypeError):pass
    path.parent.mkdir(parents=True,exist_ok=True)
    temp=None
    try:
        with tempfile.NamedTemporaryFile('w',encoding='utf-8',dir=path.parent,prefix='cameras_',suffix='.tmp',delete=False) as stream:
            temp=Path(stream.name)
            json.dump(data,stream,ensure_ascii=False,indent=2);stream.flush();os.fsync(stream.fileno())
        if path.exists():shutil.copy2(path,path.with_suffix('.json.bak'))
        # 임시 파일에 다 쓴 뒤 교체해서, 쓰기 도중 실패해도 원본이 깨지지 않게 한다.
        temp.replace(path)
    finally:
        if temp is not None:temp.unlink(missing_ok=True)


def migrate_recordings(root,base):
    """Use the latest real, nonempty recording per slot, never virtual test data."""
    # 예전 six_camera_* 녹화 폴더의 session.json에서 카메라별 최신 실제 녹화 정보를 가져와 설정을 채운다.
    # (설정, 가져온 카메라 수)를 반환한다.
    result=validate(base);found=set()
    for folder in sorted(Path(root).glob('six_camera_*'),reverse=True):
        for path in sorted(folder.glob('*/session.json')):
            try:
                record=json.loads(path.read_text(encoding='utf-8'))
                name=record['camera'];i=int(name[-2:])-1
                if name in found or i not in range(6) or name!=f'CAM{i+1:02d}':continue
                if record.get('input_kind')!='camera' or record.get('frames',0)<=0:continue
                ip=str(record.get('ip',''));ipaddress.ip_address(ip)
                status=record.get('source_status')
                serial=str((status.get('mxid') if isinstance(status,dict) else '') or result['cameras'][i]['serial'])
                result['cameras'][i].update(ip=ip,serial=serial,selected=True,
                    kind='OAK4 공유' if i==0 else '실제 OAK-D')
                found.add(name)
            except (OSError,ValueError,KeyError,TypeError):continue
    return validate(result),len(found)
