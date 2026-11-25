# OAK4 추론과 ROS 2 Humble 후처리 분리

> 2026-11-25: 기본 입력10FPS, export_fps=null(전송 샘플링 없음), FIFO 순서 보존.
> 600개 입력→NN 완료→ROS 원본/후처리 및 관련9개 토픽 각각600개 검증.
> [10FPS 변경과 실측](../verification/camera/ros_offload_ordered10_20261125.md).

## 역할

OAK4는 RGB·StereoDepth 촬영, RGB 정렬, YOLOPv2 추론, NN 출력 해석,
NMS, 원본 이진 마스크 생성과 객체 XYZ 계산을 담당한다.
WSL Ubuntu 22.04의 `hl_oak_perception`은 기존 Python 모듈을 재사용해
차선 컨투어·경계 추적, 정지선 검출, 가상 안내 경로, TOP 표시를 계산한다.
모델과 카메라 실행 환경은 변경하지 않는다.

`apps/yolopv2_standalone/ros_offload.json`의 `camera_postprocess: false`가
ROS 후처리 모드다. `true`로 바꿔 코드 배포하면 기존 장치 후처리로 복귀한다.
원본 전송은 두 모드 모두 지원하고 수신기가 있을 때만 묶음을 생성한다.
ROS 종료 시 카메라의 후처리를 자동으로 켜지 않는다.
기존 불변 런처의 정상 판정과 호환되도록 장치 telemetry의 processing_mode는
full을 유지한다. 실제 후처리 위치는 postprocess_location=ros,
postprocess_mode=ros_offload로 명시하며, 장치가 경계를 계산했다고 해석하지 않는다.

## 전송 규격

TCP 8560에 연결해 `OAK_PERCEPTION_1\n`을 보낸다. 각 응답은
6바이트 magic `OAKP\x00\x01`, 4바이트 big-endian 본문 길이,
4바이트 JSON 헤더 길이, UTF-8 헤더, 배열별 raw 또는 zlib 데이터로 구성한다.
본문 최대 24MiB, 헤더 최대 64KiB, 배열 해제 후 최대 16MiB다.
버전·크기·형식·동기화를 수신기에서 검증한다.

| 데이터 | 형식과 좌표 |
|---|---|
| 원본 RGB | BGR8, 960×540, 1920×1080의 stride-2 샘플, 오버레이 없음·무압축 원본 |
| 주행 가능/차선 마스크 | mono8 0/255, NN letterbox 제거 후 원본 FOV, 기본 320×180 |
| 객체 | NMS 후 클래스·신뢰도·정규화 원본 xyxy, 유효한 경우 광학좌표 XYZ(m) |
| Depth | RGB에 정렬된 1920×1080 uint16 little-endian, mm, 0 무효, 무손실 압축 |
| 보정값 | 내보낸 RGB 크기에 맞춘 K와 원본 Depth K, rectified 스트림, D=0 |
| 식별 | 서버 session, 연속 export_seq, RGB/NN/Depth sequence와 원래 촬영 타임스탬프 |
| 출처 | 카메라/PC 영상, 소스·반복·처리 세대와 기존 사용자 설정 |

RGB와 NN sequence 일치 및 타임스탬프 차이 1ms 이하를 요구한다.
Depth는 별도 센서 sequence를 보존하고 RGB 촬영 시각과 75ms 이하 차이만 허용한다.
이는 동일 시각의 하드웨어 동기화를 주장하는 값이 아니라 허용 오차다.
PC 영상에서는 Depth를 0으로 보내며 depth_available=false로 표시한다.

기본 input_fps는10, export_fps는null이다. 추론 완료 결과를 시간 간격으로
걸러내지 않는다. OAK_INPUT_FPS를 명시하면 입력FPS를 재지정할 수 있다.
카메라 인코딩 큐와 클라이언트별 큐, 호스트 수신 큐는 각각 FIFO8개다.
가득 차면 이전 결과를 지우지 않고 생산자를 기다리게 한다. 카메라 큐가2초간
진행하지 못하면 명시적 오류를 내므로 과부하를 누락 없이 정상인 것처럼 숨기지 않는다.
연결 중인 처리 세션의 순서를 보존하며, 연결 단절 중의 영구 저장/재전송 기능은 아니다.

PC 입력도 ROS 모드에서는 ordered다. 다음 JPEG로 이전 것을 덮어쓰지 않고
소비될 때까지 기다리며, stop도 마지막 수락 프레임이 소비된 뒤 응답한다.
송신기는 장치의 input_delivery_mode=ordered 응답을 보고 늦은 경우 시간 따라잡기로
중간 프레임을 건너뛰지 않는다. 입력 과부하라면 재생이 늦어질 수 있다.

장치 wall clock은 정확하지 않을 수 있으므로 ROS Header는 최초 수신의
호스트 ROS 시각에 장치 capture clock을 대응한 추정 시각이다.
원본 장치 시각은 metadata_json에 보존한다. 절대 시각 동기화나 센서 간
정밀 융합을 위해서는 별도 clock synchronization 검증이 필요하다.

## 빌드와 실행

WSL 터미널에서 저장소 루트로 이동한다.

```bash
source /opt/ros/humble/setup.bash
colcon --log-base artifacts/ros_offload/log build \
  --base-paths src/hl_oak_interfaces src/hl_oak_perception \
  --build-base artifacts/ros_offload/build \
  --install-base artifacts/ros_offload/install
bash tools/run_oak_ros.sh 169.254.100.228
```

의존성: ROS Humble, vision_msgs, colcon, NumPy, OpenCV.
코어 알고리즘은 apps의 원본 파일을 빌드 때 share/core에 복사하므로
알고리즘 수정 후 ROS 패키지도 다시 빌드한다. 별도 알고리즘 사본은 관리하지 않는다.
실행 중 동일 launch를 중복 실행하지 않는다.

PowerShell에서 실행할 수도 있다.

```powershell
wsl.exe -d Ubuntu-22.04 -- bash /mnt/d/git_hub/hl-dcu-sensor-fusion/tools/run_oak_ros.sh 169.254.100.228
```

## ROS 토픽

| 토픽 | 형식 |
|---|---|
| `/oak/frame` | hl_oak_interfaces/PerceptionFrame, 프레임 전체의 원자적 묶음 |
| `/oak/frame_ack` | std_msgs/String JSON, 후처리 완료한 session/frame_seq/export_seq 확인 |
| `/oak/rgb/image_raw` | sensor_msgs/Image bgr8 |
| `/oak/drivable_mask`, `/oak/lane_mask` | sensor_msgs/Image mono8 |
| `/oak/depth/image_raw` | sensor_msgs/Image 16UC1, mm |
| `/oak/rgb/camera_info`, `/oak/depth/camera_info` | sensor_msgs/CameraInfo |
| `/oak/detections` | vision_msgs/Detection2DArray, 내보낸 RGB 픽셀 좌표 |
| `/oak/bridge_status` | std_msgs/String JSON, 연결 상태 |
| `/oak/postprocess/result` | std_msgs/String JSON, 경계·정지선·안내 경로·유효성 |
| `/oak/postprocess/timing` | std_msgs/String JSON, 구독 시 프레임별 호스트 단계 시각 |
| `/oak/postprocess/debug_image` | sensor_msgs/Image, ROS 후처리 RGB |
| `/oak/postprocess/top_image` | sensor_msgs/Image, ROS 후처리 TOP |

후처리 입력은 `/oak/frame` 하나로 받아 독립 영상 토픽의 수신 순서에 의존하지 않는다.
영상과 묶음 토픽은 RELIABLE, KEEP_LAST32, 결과와 timing 토픽은 RELIABLE/64다.
최신 프레임 덮어쓰기와 수신 후1초 경과 시 폐기하는 경로는 제거했다.
연속 export_seq가 끊기면 bridge/processor에서 명시적 오류로 처리한다.
Image.data는 기본 uint8 배열에 직접 복사하고 NumPy 읽기는 버퍼 뷰를 사용한다.
개별 센서/디버그 토픽은 구독자가 있을 때 발행하며 묶음 입력은 계속 발행한다.
경계 좌표는 기존 320×180 영상 좌표이고 TOP은 미보정 표시다.
미터 단위 nav_msgs/Path나 차량 제어 명령을 발행하지 않는다.
객체 XYZ는 광학좌표계(+X 오른쪽, +Y 아래, +Z 전방)이며 m 단위다.
RGB·마스크·Depth의 해상도가 다르므로 같은 인덱스로 접근하면 안 된다.

세션·소스 세대 변경, 역행 시각, 1초 초과 촬영 간격에서 추적을 초기화한다.
ROS 입력이 2초 이상 없으면 stale=true와 각 valid=false를 반복 발행한다.
마지막 디버그 영상이 화면에 남아 있어도 유효한 최신 결과로 판단하면 안 된다.

기존 멀티뷰의 RGB·Depth와 추론 결과 RTSP는 유지한다.
장치의 TOP RTSP는 `TOP POSTPROCESS ROS`라는 안내 화면으로 대체되며,
실제 후처리 TOP은 ROS 토픽에서 확인한다. 기존 멀티뷰의 표시 설정 일부는
장치에서 경계 계산을 다시 켜지 않는다. ROS 알고리즘은 계속 동작한다.

### 전달 완료 확인 및 결과 화면 (2026-11-25)

RELIABLE/KEEP_LAST만으로는 수신 큐에 남은 대용량 프레임의 순서를 보장하지
못하므로 bridge는 후처리 완료 확인 전까지 한 묶음만 전송 중으로 유지한다.
0.5초 동안 확인이 없으면 같은 묶음을 재전송한다. processor는 중복 묶음을
다시 처리하지 않고 완료 확인만 반복한다. 완료 확인이 일치해야 다음 묶음을
꺼내며, 후처리 구독자가 없으면 수신 FIFO에서 꺼내지 않는다.
`bridge_status`의 acknowledged/retransmitted/awaiting_ack로 진행 상태를 확인한다.
bridge와 processor는 함께 빌드·재시작해야 한다.

순서 단절을 발견하면 processor는 오류 로그와 stale 결과를 발행하고 추적기를
초기화한 뒤 현재 프레임부터 복구한다. 결과의 sequence_gaps와 continuity_error로
단절을 드러내며, 누락된 프레임이 복원된 것으로 간주하지 않는다.
이전처럼 RuntimeError로 노드 전체가 종료되지는 않는다. 카메라 연결 단절 중의
영구 저장·재전송 기능을 추가한 것은 아니다.

WSL에서 결과 화면을 열고 수신 간격을 측정한다.

```bash
source /opt/ros/humble/setup.bash
python3 tools/view_oak_ros.py
# 별도 터미널
python3 tools/check_oak_ros_stream.py --seconds 120 \
  --output artifacts/ros_offload/stream_check.json
```

뷰어는 RELIABLE 구독을 사용하고 새 영상이 도착했을 때만 그린다. Esc로 종료한다.
측정은 평균 FPS뿐 아니라 간격 P50/P95/최대, 250ms 초과 공백, 20ms 미만 몰림,
결과 순서 단절, stale, 호스트 수신→결과 발행 지연을 기록한다.
화면 표시까지의 실제 지연을 측정하는 도구는 아니다.

### 대용량 영상 DDS 설정

`tools/run_oak_ros.sh`와 `tools/view_oak_ros.py`는 별도 프로필을 지정하지 않았다면
`tools/ros_fastdds_large_images.xml`을 사용한다. SHM 영역을64MiB로 확보하고
기본 전송도 유지한다. 사용자가 설정한 FASTRTPS_DEFAULT_PROFILES_FILE은 덮어쓰지 않는다.
기본 약512KiB 영역은 debug1,555,200bytes/TOP921,600bytes보다 작았다.
기본 설정에서 최대 약3초의 수신 몰림이 관찰됐으며, 확대 설정 적용 후 PC 영상
30초에서는 각299개·약10FPS, 최대 간격166ms,250ms초과 공백0이었다.
설정은 ROS 프로세스 생성 시 적용되므로 기존 노드를 종료한 것을 확인한 뒤 재실행한다.
`ros2 topic info /oak/frame`에서 발행자·구독자는 각각1개여야 한다.

## 검증과 복구

```bash
bash tools/verify_oak_ros.sh --synthetic --seconds 12 --output artifacts/ros_offload/synthetic
bash tools/verify_oak_ros.sh --seconds 20 --output artifacts/ros_offload/live
```

원본 frame·후처리 결과·debug·TOP 각각 3개 이상 수신을 검사하고
리포트 및 예시 PNG를 저장한다. 검출 정확도나 도로 주행 적합성을 검사하는 시험은 아니다.
PC 입력의 Depth 무효화, 프레임 불일치 거부, 압축 해제 크기 제한,
추적 세션 초기화는 `test_ros_offload.py`에서 검사한다.

장치 배포는 기존 `deploy_code_release.py`의 build/deploy/status/rollback을 사용한다.
실행 환경 fingerprint가 일치할 때만 적용하며 이전 활성 코드는 복구본으로 남긴다.
이번 배포 직전 상태는 `artifacts/ros_offload/before_deploy.json`, 적용 결과는
`artifacts/ros_offload/deployed.json`을 참조한다.

## 2026-11-24 실장비 확인 (이전5FPS 설정)

- 카메라 169.254.100.228 / 1142227768, 설치 앱 61fbe0c1-ad26-42dc-93c4-a8d2b5191fce.
- 실행 환경 0.33.10 유지, 94,329bytes 코드만 배포. 활성 release
  e9f2f1d52d6ed026ba238ea9ca6bf52a9c154d79f3ea88de38461adb89fd7396,
  pending/failure 없음. 이전 7da3df0d… 코드가 복구본이다.
- ROS 두 패키지 빌드 성공. 전체 Python 회귀 427개 통과.
- 20초 실장비 수신: frame/result/debug/TOP 각각 76개, stale 0.
  이는 약 3.8Hz의 수신 건수이며 지연·정확도 성능 보장값이 아니다.
- 마지막 frame RGB 960×540, 마스크 320×180, Depth 1920×1080 16UC1,
  depth_available=true, 유효 Depth 1,069,163픽셀, RGB/Depth 시각 차이 약 0.471ms.
- 격리 도메인 합성 입력 중단 시험: 마지막 4초 송신 중지 후 stale 메시지 3개 확인.
- 실내 카메라 장면으로 전송·계산 실행을 확인했다. 도로 영상에 대한 경계·정지선·
  안내 경로 정확도 비교는 이번 시험의 범위가 아니다.
- 첫 후보는 구형 런처의 mode 문자열 검사로 자동 복구됐다. 호환 필드 분리 후 재배포해
  정상 확정했다. 큰 ROS 영상은 BEST_EFFORT에서 누락되어 RELIABLE로 수정했다.
- 리포트: artifacts/ros_offload/live2/report.json, disconnect/report.json,
  unittest_final.log. ROS launch는 WSL에서 실행 중이며 재부팅 자동 시작은 설정하지 않았다.
