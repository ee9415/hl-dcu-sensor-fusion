# OAK-D PoE 6대 YOLOP 추론 및 녹화

Ubuntu 22.04, Python 3.10, DepthAI 2.32.0.0 환경에서 OAK-D PoE 6대를 MX ID로 고정 매핑하여 실행하는 비 ROS 2 프로그램이다. 각 장치는 독립적인 DepthAI pipeline을 사용하며, 320x320 letterbox 영상이 YOLOP와 장치 내부 H.265 encoder에 각각 전달된다.

## DCU 접속 정보

| 항목 | 값 |
| --- | --- |
| DCU IP | `192.168.201.4` |
| OS | Ubuntu 22.04 |
| Python | 3.10 |
| SSH 사용자/포트 | `halla` / `22` |

개발 PC가 DCU와 같은 네트워크에 연결되어 있는지 확인한 다음 접속한다.

```bash
ping -c 3 192.168.201.4
ssh <DCU_USER>@192.168.201.4
```

SSH 포트가 22가 아니면 `ssh -p <SSH_PORT> <DCU_USER>@192.168.201.4`를 사용한다. 비밀번호, 개인키 경로, 실제 사용자명은 저장소나 실행 스크립트에 넣지 않는다.

개발 PC에서 이 도구만 DCU로 복사하는 예시는 다음과 같다.

```bash
scp -r tools/yolop_6cam_recorder <DCU_USER>@192.168.201.4:~/
scp models/yolop_320x320.blob <DCU_USER>@192.168.201.4:~/yolop_6cam_recorder/models/
```

## 카메라 고정 매핑

| 이름 | MX ID |
| --- | --- |
| `cam_01` | `1944301001E1761300` |
| `cam_02` | `19443010E131771300` |
| `cam_03` | `194430105130731300` |
| `cam_04` | `19443010517C731300` |
| `cam_05` | `194430105111771300` |
| `cam_06` | `1944301071DA761300` |

녹화 전 `dai.Device.getAllAvailableDevices()` 결과를 이 표와 대조한다. 한 대라도 없으면 누락 이름과 MX ID를 출력하고 어떤 녹화 worker도 시작하지 않는다. `--inspect-nn`은 진단 전용이므로 지정한 카메라 한 대만으로 실행할 수 있다.

## DCU 설치

DCU에 SSH로 접속한 뒤:

```bash
cd ~/yolop_6cam_recorder
sudo apt update
sudo apt install -y python3.10-venv ffmpeg
python3.10 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
chmod +x run_yolop_6cam_recorder.sh
```

DepthAI PoE 장치가 검색되는지 먼저 확인한다.

```bash
python - <<'PY'
import depthai as dai
for d in dai.Device.getAllAvailableDevices():
    print(d.getMxId())
PY
```

카메라가 검색되지 않으면 DCU의 카메라용 NIC 주소/서브넷, 스위치 전원, PoE 링크, 방화벽을 확인한다. DCU 관리 주소 `192.168.201.4`와 카메라 주소 대역이 반드시 같다는 뜻은 아니므로, 카메라 NIC 설정은 실제 배선 설계를 따른다.

## YOLOP tensor 확인

YOLOP Blob마다 output 이름과 shape가 다르다. 최초 실행은 녹화 없이 실제 `NNData` 메타데이터를 확인한다.

```bash
./run_yolop_6cam_recorder.sh \
  --blob models/yolop_320x320.blob \
  --inspect-nn \
  --inspect-camera cam_01
```

출력된 세 tensor 이름을 실제 값으로 지정한다. 예시 이름은 설명용이며 Blob에 그대로 적용하면 안 된다.

```bash
./run_yolop_6cam_recorder.sh \
  --blob models/yolop_320x320.blob \
  --inspect-nn \
  --det-layer '<ACTUAL_DETECTION_NAME>' \
  --drivable-layer '<ACTUAL_DRIVABLE_NAME>' \
  --lane-layer '<ACTUAL_LANE_NAME>'
```

DepthAI가 shape를 제공하지 않거나 변환 순서가 다른 Blob은 쉼표 형식으로 shape도 지정한다.

```text
--det-shape 1,25200,6
--drivable-shape 1,2,320,320
--lane-shape 1,2,320,320
```

검출 출력은 다음 두 형식을 지원한다.

- `--det-format decoded-xyxy`: `N x 6`, `[x1,y1,x2,y2,confidence,class_id]`
- `--det-format yolo-xywh`: `N x (5+classes)`, `[cx,cy,w,h,objectness,class_scores...]`
- `--det-format auto`: 마지막 차원이 6이면 전자, 그보다 크면 후자로 해석

Blob이 anchors/grid 적용 전의 여러 raw YOLO head를 출력한다면 모델 고유 anchors/stride 후처리가 필요하므로 이 프로그램에 바로 사용할 수 없다. 이 경우 decoded detection을 포함하도록 Blob을 export하거나 모델 전용 decoder를 추가해야 한다. tensor 불일치는 오류로 종료되며 조용히 빈 검출로 처리하지 않는다.

클래스 이름은 쉼표 문자열 또는 JSON 배열 파일로 설정한다.

```bash
--class-names 'person,bicycle,car,motorcycle,bus,truck'
# 또는
--class-names models/classes.json
```

## 실행

### Blob 없이 영상만 녹화

`--record-only`를 사용하면 NeuralNetwork 노드를 생성하지 않으므로 `--blob`이 필요하지 않다. 6대 카메라 확인, 320x320 letterbox, 장치 내부 H.265 인코딩, 카메라별 H.265/JSONL 저장과 상태 감시는 동일하게 수행한다.

```bash
./run_yolop_6cam_recorder.sh \
  --record-only \
  --output recordings \
  --fps 10 \
  --bitrate-kbps 2000 \
  --duration 600
```

record-only JSONL은 encoded frame을 기준으로 host 시간, 장치 시간, 카메라 이름, MX ID와 frame 번호를 기록한다. `objects`는 빈 배열이고 `drivable_area`와 `lane_area`는 `null`이다.

### YOLOP 추론과 영상 녹화

```bash
source .venv/bin/activate
./run_yolop_6cam_recorder.sh \
  --blob models/yolop_320x320.blob \
  --output recordings \
  --fps 10 \
  --bitrate-kbps 2000 \
  --duration 600 \
  --det-layer '<ACTUAL_DETECTION_NAME>' \
  --drivable-layer '<ACTUAL_DRIVABLE_NAME>' \
  --lane-layer '<ACTUAL_LANE_NAME>'
```

`--duration 0` 또는 옵션 생략 시 Ctrl+C까지 계속 녹화한다. 프로그램은 5초마다 카메라별 추론 FPS, 녹화 FPS, 저장 frame 수와 JSON record 수를 출력한다. 특정 장치 worker에서 tensor, 연결 또는 파일 오류가 발생하면 main thread로 전달되어 전체 세션을 명시적으로 종료한다. encoded/NN packet이 모두 15초간 끊긴 카메라도 stall 오류로 처리한다.

모든 옵션:

```bash
./run_yolop_6cam_recorder.sh --help
```

## 출력

```text
recordings/yolop_YYYYMMDD_HHMMSS/
├── cam_01.h265
├── cam_01.jsonl
├── cam_02.h265
├── cam_02.jsonl
├── cam_03.h265
├── cam_03.jsonl
├── cam_04.h265
├── cam_04.jsonl
├── cam_05.h265
├── cam_05.jsonl
├── cam_06.h265
├── cam_06.jsonl
└── session_metadata.json
```

YOLOP 모드의 JSONL 각 줄에는 UTC host 시간, 장치 부팅 기준 timestamp(초), 카메라 이름, MX ID, NN frame sequence, 객체 목록, 주행 가능 영역 요약, 차선 영역 요약이 기록된다. 분할 결과는 저장량을 제한하기 위해 전체 mask가 아니라 foreground 비율과 class별 pixel 수를 저장한다. record-only 모드는 encoded frame 단위의 기본 메타데이터만 기록한다.

## H.265 재생과 MP4 변환

raw H.265 elementary stream 재생:

```bash
ffplay -fflags nobuffer -flags low_delay -framerate 10 recordings/yolop_YYYYMMDD_HHMMSS/cam_01.h265
```

재인코딩 없이 MP4 container로 변환:

```bash
ffmpeg -framerate 10 -i cam_01.h265 -c:v copy -tag:v hvc1 cam_01.mp4
```

일부 player가 raw stream timestamp를 제대로 처리하지 못하면 재인코딩한다.

```bash
ffmpeg -framerate 10 -i cam_01.h265 -c:v libx265 -pix_fmt yuv420p -movflags +faststart cam_01.mp4
```

6개 파일 일괄 변환:

```bash
for f in cam_*.h265; do
  ffmpeg -framerate 10 -i "$f" -c:v copy -tag:v hvc1 "${f%.h265}.mp4"
done
```

`-framerate` 값은 녹화 시 `--fps`와 같아야 한다.

## 실시간 Web UI

Web UI는 Blob 없이 6대 preview, 녹화 시작/정지 및 디렉터리별 재생을 한 프로세스에서 관리한다. 카메라별 pipeline은 320x320 letterbox 영상을 H.265 녹화 encoder와 MJPEG preview encoder에 동시에 전달한다.

```bash
chmod +x run_yolop_6cam_web_ui.sh
./run_yolop_6cam_web_ui.sh \
  --host 127.0.0.1 \
  --port 8080 \
  --output recordings \
  --fps 10 \
  --bitrate-kbps 2000 \
  --preview-quality 75
```

개발 PC에서 별도 terminal을 열고 SSH tunnel을 유지한다.

```bash
ssh -L 8080:127.0.0.1:8080 halla@192.168.201.4
```

그 상태에서 개발 PC의 브라우저로 다음 주소를 연다.

```text
http://127.0.0.1:8080
```

화면은 3열 2단으로 구성된다.

1. 첫 번째 단: `cam_04`, `cam_01`, `cam_05`
2. 두 번째 단: `cam_02`, `cam_06`, `cam_03`

영상 영역은 16:9로 표시하여 320x320 letterbox preview의 상하 여백을 잘라내고 화면 높이를 줄인다. 각 패널의 목록에서 `cam_01`~`cam_06`을 자유롭게 선택할 수 있다. 동일 카메라를 여러 패널에서 선택하는 것도 가능하다.

- **녹화 시작**: 버튼을 누른 시각을 기준으로 `recordings/yolop_YYYYMMDD_HHMMSS/`를 생성하고 6대 H.265/JSONL 저장을 함께 시작한다.
- **녹화 정지**: 모든 H.265/JSONL과 metadata를 flush하고 닫는다. preview는 계속 표시된다.
- **녹화 디렉터리 재생**: 목록에서 session을 고르고 `6개 동시 재생` 버튼을 누른다. 서버가 raw H.265를 브라우저 호환 H.264 MP4로 변환하고 6개 파일이 모두 로드될 때까지 기다린 뒤, 같은 3/3 화면에서 전부 0초부터 동시에 재생한다. 같은 버튼을 다시 누르면 6개 영상이 처음부터 함께 재시작된다.
- **실시간 보기**: 재생 화면에서 live preview로 복귀한다.

재생 변환은 `ffmpeg`와 `libx264`를 우선 사용한다. `ffmpeg`가 없으면 GStreamer의 `h265parse`, `avdec_h265`, `x264enc`, `h264parse`, `mp4mux` plugin을 자동으로 사용한다. 변환 결과는 각 session의 `.playback/` 아래에 cache되므로 같은 session의 두 번째 재생부터는 다시 변환하지 않는다. 긴 녹화는 최초 변환에 시간이 걸리며 UI에 진행 카메라 수가 표시된다.

Web UI와 `yolop_6cam_recorder.py`를 동시에 실행하면 안 된다. OAK 장치는 한 프로세스만 점유할 수 있으므로 기존 recorder를 먼저 종료한 뒤 Web UI를 실행한다.

### 관리망에서 직접 접속

loopback 이외의 주소에 bind할 때는 HTTP Basic 인증이 필수다. 비밀번호를 명령행에 직접 넣지 말고 소유자만 읽을 수 있는 파일로 만든다.

```bash
umask 077
openssl rand -base64 24 > ~/.yolop_web_password

./run_yolop_6cam_web_ui.sh \
  --host 192.168.201.4 \
  --port 8080 \
  --auth-user halla \
  --auth-password-file ~/.yolop_web_password \
  --output recordings \
  --fps 10 \
  --bitrate-kbps 2000
```

같은 관리망의 PC에서 `http://192.168.201.4:8080`을 열고 인증 정보를 입력한다. HTTP Basic 인증이므로 신뢰할 수 있는 폐쇄망에서만 사용한다. 인터넷에 포트 포워딩하지 말고, 인터넷 원격 접속이 필요하면 VPN 또는 TLS reverse proxy를 적용한다. 기본 bind 주소는 계속 `127.0.0.1`이다.

현재 DCU에 생성된 비밀번호는 SSH로 확인할 수 있다.

```bash
ssh -p 22 halla@192.168.201.4 'cat /home/halla/.yolop_web_password'
```

### 동시 접속과 녹화 제어

- 활성 브라우저는 최대 3개까지 허용한다. 같은 브라우저의 여러 탭은 쿠키를 공유하므로 한 접속자로 계산된다.
- 탭을 닫은 뒤 요청이 30초 동안 없으면 해당 접속 자리가 자동으로 반환된다.
- `http://192.168.201.4:8080` 접속은 외부 사용자 권한이다.
- DCU 자체의 `http://127.0.0.1:8080` 접속은 localhost 관리자 권한이다. 개발 PC에서는 `ssh -L 8080:127.0.0.1:8080 halla@192.168.201.4` tunnel을 연 뒤 `http://127.0.0.1:8080`에 접속하면 같은 관리자 권한이 된다.
- 외부 접속 3개가 모두 사용 중이어도 localhost 관리자는 가장 오래된 외부 접속 하나를 내보내고 우선 입장한다.
- 외부 사용자가 녹화를 시작하면 그 브라우저와 localhost 관리자만 녹화를 정지할 수 있다.
- 상단 상태 표시에서 현재 권한, 접속 수, 녹화 제어자를 확인할 수 있다.

### 재부팅 후 수동 실행

자동 실행 서비스 없이 필요할 때 다음 관리 스크립트를 사용한다.

```bash
cd /home/halla/yolop_6cam_recorder
./manage_yolop_6cam_web_ui.sh start
```

개발 PC에서 DCU에 로그인하지 않고 명령 한 줄로 실행할 수도 있다.

```bash
ssh -p 22 halla@192.168.201.4 \
  '/home/halla/yolop_6cam_recorder/manage_yolop_6cam_web_ui.sh start'
```

지원 명령은 다음과 같다.

```bash
./manage_yolop_6cam_web_ui.sh start    # 시작
./manage_yolop_6cam_web_ui.sh stop     # 파일과 장치를 안전하게 닫고 정지
./manage_yolop_6cam_web_ui.sh restart  # 재시작
./manage_yolop_6cam_web_ui.sh status   # 실행 상태 확인
./manage_yolop_6cam_web_ui.sh logs     # 실시간 로그, Ctrl+C는 로그 보기만 종료
```

`start`는 카메라 6대가 모두 검색될 때까지 최대 120초 기다리고, Web UI 포트가 열렸는지 확인한다. 이미 실행 중이면 두 번째 프로세스를 만들지 않는다.

## 오류 및 운영 주의사항

- 시작 시 출력 경로의 여유 공간이 1 GiB보다 작으면 시작하지 않고, 녹화 중 256 MiB 아래로 내려가면 안전 종료한다. 2,000 kbps x 6대는 영상만 약 1.5 MB/s(시간당 약 5.4 GB)이며 JSONL과 filesystem 여유분을 별도로 고려한다.
- H.265와 JSONL은 동일 sequence의 완전한 1:1 저장을 보장하지 않는다. encoder와 NN은 서로 독립적인 장치 node이며 NN 처리 지연/queue 상태가 다를 수 있으므로 `frame_number`와 장치 시간을 기준으로 분석한다.
- Ctrl+C, duration 종료, worker 오류 모두 `finally` 경로에서 worker 파일을 닫은 뒤 모든 DepthAI device를 닫는다.
- 강제 `kill -9`, DCU 전원 차단 또는 저장 장치 분리는 `finally` 실행을 보장하지 않는다.
- 6대 동시 H.265와 NN 결과를 수신할 수 있도록 카메라 네트워크와 저장 장치 처리량을 사전에 검증한다.
