# CAM01 순수 RGB 전송

6대 녹화 전용 OAK4 프로그램이다. `Camera(CAM_A) → VideoEncoder(H.264) → RTSP`
노드만 생성하며 모델, 신경망, StereoDepth, 차선 후처리, TOP 뷰를 로드하지 않는다.
카메라 구동·인코딩·전송 프로그램은 필요하며, YOLOPv2 앱과 독립적으로 실행된다.

- 영상: `rtsp://<CAM01 IP>:8560/rgb` (1920×1080, 30 FPS 요청)
- 상태: `http://<CAM01 IP>:8561/status` (장치 ID, 실행 세대, 카메라 프레임 상태)
- 식별자: `com.qsentech.oak4-rgb-only`
- `rtsp_server.py`는 기존 standalone 앱의 검증된 H.264 전송 구현을 복사한 파일이다.
  두 앱이 각각 독립된 디렉터리로 배포되므로 함께 포함한다.

멀티뷰 재시작 후 **6대 카메라 녹화 → CAM01 연결**을 누르면 인식·검증 수신과
PC 영상 전송을 종료하고 AI 앱을 중지한 뒤 **설치된 RGB 앱만 시작**한다.
연결 버튼은 빌드·설치 명령을 실행하지 않는다. 패키지가 없으면 설치 안내 오류를 표시한다.
연결 완료 뒤 선택 카메라 녹화를 시작한다. CAM02~06은 기존 DepthAI 2 RGB 경로를 사용하며,
새로 연결할 때 녹화 설정의 FPS를 촬영 파이프라인에 전달한다.

2026-11-03 수정: AI/RGB를 서로 다른 설치 앱 ID로 보관한다. 개발 앱 ID(전체 0)는
설치된 앱으로 인정하지 않는다. 시작 준비와 실행 후 첫 프레임을 기다리며,
긴 작업은 CLI에 10초마다 경과 시간을 표시한다.
멀티뷰 시작·장치 부팅 때 자동 실행하지 않으며, 연결/시작 버튼으로만 실행한다.

패키지를 갱신할 때만 다음 작업을 별도로 수행한다(Windows PowerShell).
출력 폴더는 먼저 생성한다. 소스 수정 후에는 패키지를 다시 빌드·설치해야 한다.

```powershell
& 'C:/Program Files/oakctl/bin/oakctl.exe' app build apps/rgb_only -d 192.168.201.2 --optimized --output artifacts/rgb_only_20261103/lean
& 'C:/Program Files/oakctl/bin/oakctl.exe' app build apps/yolopv2_standalone -d 192.168.201.2 --optimized --output artifacts/rgb_only_20261103/lean
# 아래에는 각 빌드가 반환한 실제 .oakapp 파일 경로를 사용한다.
& 'C:/Program Files/oakctl/bin/oakctl.exe' app install <RGB패키지.oakapp> -d 192.168.201.2 --start false --enable false --keep-others
& 'C:/Program Files/oakctl/bin/oakctl.exe' app install <AI패키지.oakapp> -d 192.168.201.2 --start false --enable false --keep-others
```

## 배포 이미지 축소 (2026-11-03)

기본 이미지는 `python:3.12.11-slim-bookworm`으로 고정한다. 카메라 SDK와
GStreamer 전송 의존성만 설치하며 AI 백엔드에는 runit을 사용한다.
`install-runtime.sh`는 컴파일 의존성 설치·Python 패키지 설치·컴파일 의존성 제거를
같은 이미지 레이어에서 수행하고 pip/apt 캐시, 정적 Python 아카이브·헤더와
NumPy 테스트 데이터를 제거한다. 필수 SDK·GStreamer 모듈은 빌드 중 import/플러그인 검사를 한다.

배포에는 현재 기본 모델 `yolopv2_person_320.rvc4.tar.xz`만 포함한다.
다른 모델과 테스트·캐시는 `.oakappignore`로 제외하며 로컬 원본은 유지한다.
다른 모델을 배포할 때는 해당 ignore 규칙과 모델 경로를 함께 조정해야 한다.
패키지는 `--optimized`로 생성한다. 이 옵션은 빌드 시간이 더 걸릴 수 있다.

근거: [OAK 앱의 기본 이미지 설정](https://docs.luxonis.com/software-v3/oak-apps/configuration),
[oakctl 최적화 빌드 옵션](https://docs.luxonis.com/software-v3/oak-apps/oakctl).

**인식·검증 → 영상 연결**은 RGB 전송 프로그램을 멈추고 기존 AI 앱으로 돌아간다.
CAM01 녹화 중이거나 RGB 연결을 준비 중이면 전환을 차단한다. 탭 선택만으로 모드를 바꾸지 않는다.
CAM01 IP 및 장치 ID를 올바르게 등록해야 한다. 이전 설정 파일의 `OAK4 공유` 값은
호환을 위해 유지하지만 화면과 실행 경로는 독립 RGB이다.

각 카메라 상태와 `session.json`에 수신 FPS, 저장 FPS, 중복 프레임,
건너뛴 수신 프레임 수를 기록한다. 저장은 기존 고정 FPS 방식이다.
`skipped_source_frames`는 PC 수신 순번 중 저장 시 선택되지 않은 프레임 수이며,
저장 FPS보다 촬영 FPS가 높을 때 정상적으로 증가할 수 있다.
카메라에서 PC에 도착하기 전의 손실까지 측정한 값은 아니다.
수신 FPS는 녹화 구간의 수신 순번 증가량, 저장 FPS는 작성 프레임 수를 실제 경과 시간으로 나눈 값이다.

## 검증

```sh
PYTHONPATH=apps/yolopv2_standalone:tools python3 -m unittest discover -s apps/yolopv2_standalone/tests
```

회귀 테스트는 카메라/인코더만 생성되는지, 잘못된 입력·장치 ID를 거부하는지,
모드 전환과 녹화 보호, 6개 가상 수신기의 동시 녹화·MP4 검증을 확인한다.
물리 카메라 6대의 동시 FPS는 장비 연결 상태에서 별도로 측정해야 한다.

사용한 DepthAI v3 API는 [장치 API](https://docs.luxonis.com/software-v3/depthai/depthai-components/device)와
[인코딩 예제](https://docs.luxonis.com/software-v3/depthai/examples/video_encoder/video_encode)를 기준으로 한다.
