"""Persist applied video input controls without starting a publisher."""
# 영상 입력 설정(카메라/PC 영상 파일, 구간, 반복, FPS)을 settings.json의 video_input 섹션에 저장/로드한다.
# 저장만 할 뿐 영상 전송을 시작하지는 않는다.
import json
import os
from pathlib import Path
import tempfile
import settings_store


def validate(data):
    """입력 설정 값을 검사하고 허용된 키만 남긴 dict를 반환한다. FPS는 10/15/20만 허용, 종료 시간은 시작보다 커야 한다."""
    if not isinstance(data, dict) or data.get('mode') not in ('카메라', 'PC 영상 파일'):
        raise ValueError('잘못된 영상 입력 설정')
    for field in ('file', 'start', 'end'):
        if not isinstance(data.get(field), str):
            raise ValueError('잘못된 영상 입력 설정: ' + field)
    if type(data.get('loop')) is not bool or type(data.get('fps')) is not int or data['fps'] not in (10,15,20):
        raise ValueError('잘못된 반복 또는 FPS 설정')
    from video_file_sender import parse_video_time
    start=parse_video_time(data['start'])
    end=parse_video_time(data['end']) if data['end'].strip() else None
    if end is not None and end<=start:
        raise ValueError('종료 시간은 시작 시간보다 커야 합니다')
    return {key:data[key] for key in ('mode','file','loop','start','end','fps')}


def load(path):
    """저장된 입력 설정을 읽는다. 파일이나 섹션이 없으면 None."""
    path=Path(path)
    if not path.exists():return None
    data=settings_store.read(path)
    value=data.get('video_input')
    if value is None:return None
    return validate(value)


def save(path,data):
    """검사한 입력 설정을 video_input 섹션에 저장한다(다른 섹션은 유지)."""
    data=validate(data)
    settings_store.update(path,{'video_input':data})
