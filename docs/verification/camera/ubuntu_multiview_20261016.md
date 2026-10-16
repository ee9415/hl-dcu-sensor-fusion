# Ubuntu 멀티뷰 실행 검증

작성일: 2026-10-16

## 변경 및 근거

기존 `tools/oak_recorder.py`를 Windows와 Ubuntu에서 공용으로 사용한다.
Linux에서는 Windows DPAPI를 호출하지 않고 비밀번호를 디스크에 저장하지 않는다.
프로세스 생성 플래그와 oakctl 경로를 플랫폼별로 처리하고 Ubuntu 기본 Pillow의
LANCZOS API와 호환되도록 했다. 자동 연결 옵션과 Bash 실행기를 추가했다.

## 실행 방법

Ubuntu 터미널에서:

```bash
bash /mnt/d/git_hub/hl-dcu-sensor-fusion/tools/run_oak_recorder.sh --device-ip 192.168.0.4 --auto-connect
```

필수 시스템 패키지는 python3-tk, python3-opencv, python3-pil.imagetk,
fonts-noto-cjk다. 이번에는 기존 Python 패키지를 재사용하고 한글 글꼴만 설치했다.
WSLg DISPLAY=:0, WAYLAND_DISPLAY=wayland-0 환경에서 실행했다.
카메라 앱이 이미 실행 중이면 oakctl 없이 RTSP에 바로 연결한다.
앱이 꺼져 있다면 Windows에서 앱을 실행하거나 Ubuntu용 oakctl을 별도 설치해야 한다.
Windows에 저장한 비밀번호는 Ubuntu로 가져오지 않는다.

## 검증 방법 및 결과

`src/hl_camera_bringup/scripts/check_recorder_gui.py --keep-open`으로 실제 Tk 창을
띄우고 20초 후 수신 프레임과 ImageTk 미리보기 존재를 확인했다. 창은 유지했다.

| 항목 | 결과 |
| --- | --- |
| Tk 창 mapped | true |
| RGB | 연결됨, 1920x1080, 미리보기 생성됨 |
| YOLOPv2 결과 | 연결됨, 960x540, 미리보기 생성됨 |
| BEV | 연결됨, 640x480, 미리보기 생성됨 |
| Python 문법 검사 | 통과 |
| git diff --check | 통과 |

최초 실행에서 Ubuntu Pillow의 Image.Resampling 미지원 오류를 발견해 수정한 뒤
재실행 검증을 통과했다. 이는 프레임 수신 및 UI 반영 검증이며 실제 지연 측정이나
중심선 정확도 검증은 아니다. 녹화/저장 대화상자 전체 동작은 이번에 검증하지 않았다.
Depth 패널은 아직 출력 연동 대기 상태다. ROS2 노드가 아닌 독립 Tk 디버깅 UI이므로
이번 변경에는 colcon, ROS topic, TF 검증이 적용되지 않는다.
