# OAK4 Runtime Output and Resource Snapshot

- 측정일: 2026-10-15 23:46 KST
- 장비: OAK4-D R9 `1142227768`, IP `192.168.0.4`
- OS: Luxonis OS RVC4 1.41.0, `linux/arm64`
- 앱: `com.qsentech.oak4-yolopv2-standalone`, 컨테이너 버전 `0.5.1`
- 파이프라인: `full`

이 문서는 현재 개발 앱의 설정값과 실행 중인 장비에서 `oakctl`로 확인한 한 시점의
성능 표본을 기록한다. 장시간 부하 시험이나 실차 성능 보증값은 아니다.

## 영상 입력과 내부 처리

| 항목 | 해상도 | 설정 FPS | 용도 |
| --- | ---: | ---: | --- |
| CAM_A RGB | 1920x1080 | 10 | NV12, 왜곡 보정, 원본 및 NN 입력 원천 |
| CAM_B stereo | 640x400 | 10 | 좌측 Depth 입력 |
| CAM_C stereo | 640x400 | 10 | 우측 Depth 입력 |
| RGB 정렬 Depth | 1920x1080 | 10 | CAM_A 좌표, 16-bit mm, 0은 무효 |
| YOLOPv2 입력 | 1x3x320x320 | 10 | letterbox RGB tensor |
| 주행영역·차선 내부 마스크 | 320x180 | 약 10 | letterbox 제거 후 후처리 |
| 합성 결과 | 960x540 | 약 10 | 객체·마스크·거리·중심선 |
| Centerline BEV | 640x480 | 약 10 | 개발용 IPM과 3차식 중심선 |

주요 논리 노드는 Camera 3개, H.264 encoder 3개, StereoDepth, ImageAlign,
ImageManip, NeuralNetwork, 전처리·후처리 HostNode와 RTSP StreamOutput 3개다.

## 외부 출력

RTSP 영상은 3개다.

| 출력 | 주소 | 해상도 | 코덱 | 설정 FPS |
| --- | --- | ---: | --- | ---: |
| 원본 RGB | `rtsp://192.168.0.4:8555/rgb` | 1920x1080 | H.264 Main | 10 |
| YOLOPv2 결과 | `rtsp://192.168.0.4:8554/yolopv2` | 960x540 | H.264 Main | 10 |
| BEV·가상 중심선 | `rtsp://192.168.0.4:8553/bev` | 640x480 | H.264 Main | 10 |

RemoteConnection이 실제 광고한 토픽은 4개다.

| 토픽 | 형식/내용 |
| --- | --- |
| `DepthAligned` | 1920x1080 RGB 정렬 Depth `ImgFrame` |
| `SpatialDetections` | 유효 Depth 객체의 CAM_A optical X/Y/Z |
| `YOLOPv2Result` | 960x540 H.264 `EncodedFrame` |
| `CenterlineBEV` | 640x480 H.264 `EncodedFrame` |

Windows 멀티뷰는 4패널이지만 현재 PC-facing RTSP 영상은 3개다. 원본 RGB,
결과, BEV는 실시간이고 RGB 정렬 Depth 패널은 별도 Depth 영상 출력이 추가될
때까지 대기 화면이다. 주행영역과 차선 마스크는 내부에서 생성되어 결과 영상에
합성되며 별도 공개 토픽은 아니다.

## 런타임 성능

최신 `HEALTH` 로그의 안정 구간은 다음과 같다.

| 항목 | 측정값 |
| --- | ---: |
| 설정 FPS | 10.00 |
| 실제 누적 평균 FPS | 9.98 |
| 목표 대비 처리율 | 99.8% |
| 후처리 시간 | 약 32.4~34.0 ms/frame |
| 측정 시 누적 처리 프레임 | 7,265 이상 |
| 객체 수 | 11/frame |
| 중심선 | valid |
| 중심선 residual | 약 0.47 m |
| 유효 Depth 객체 | 0 |

현재 카메라가 모니터 영상을 촬영하는 조건에서는 화면 속 차량이 실제 입체 물체가
아니므로 유효 Depth 객체 0은 정상이다. residual도 개발용 고정 IPM 결과이며 차량
좌표 정확도 검증값이 아니다.

## CPU, 메모리, 전력과 온도

| 항목 | 측정값 |
| --- | ---: |
| 장비 CPU | 논리 코어 6개, 약 20.8~25.5% |
| 장비 메모리 | 약 2.72 GiB / 7.03 GiB, 약 38.7% |
| 앱 `python3.12` CPU | `ps` 기준 약 70.3% of one core |
| 앱 `python3.12` RSS | 약 950 MiB, 장비 메모리의 약 13.1% |
| 장비 소비전력 | 관측 약 2.7~6.2 W, 단일 표본 4.43 W |
| 카메라 온도 | 약 42~43 C |
| GPU 온도 | 약 41~45 C |
| CPU 최고 관측 온도 | 약 61 C |
| 장비 경고 | 없음 |

프로세스 CPU 70.3%는 장비 전체의 70.3%가 아니라 Linux에서 코어 하나를 100%로
표현한 값이다. GPU/NPU 개별 engine 사용률은 현재 `oakctl device stats`에서
제공하지 않으므로 별도 계측 없이는 확정하지 않는다.

## 실제 녹화 표본과 네트워크 부하

58초 녹화 표본은 다음과 같다.

| 파일 | 해상도 | FPS | 프레임 | 시간 | 크기 | 평균 비트레이트 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| RGB | 1920x1080 | 10 | 582 | 58.2 s | 141.39 MiB | 약 20.41 Mbps |
| YOLOPv2 | 960x540 | 10 | 581 | 58.1 s | 35.28 MiB | 약 5.09 Mbps |

두 녹화 스트림의 합계는 약 25.5 Mbps다. 1프레임 차이는 독립 RTSP 수신 스레드의
시작·종료 시점 차이다. BEV는 이 표본에서 MP4 녹화 대상이 아니므로 합계에서
제외했다. RTSP client가 없을 때 외부 Ethernet 전송량은 매우 작지만, client가
연결되면 선택한 스트림 수와 영상 복잡도에 따라 증가한다.

## 저장공간과 운영 주의점

| 영역 | 측정값 |
| --- | ---: |
| 시스템 `/` | 4.52 GB / 5.19 GB, 약 87% |
| 데이터 `/data` | 2.79 GB / 82.74 GB, 약 3.4% |

시스템 파티션 여유가 작으므로 새 앱 이미지와 패키지를 계속 쌓지 않는다. 대용량
영상과 장기 로그는 `/data` 또는 외부 PC에 저장한다.

`oakctl app list`의 개발 앱 행은 버전을 `1.1.0`으로 표시했지만 실행 컨테이너의
`OAKAGENT_APP_VERSION`은 `0.5.1`이었다. 현재 앱 버전 판단에는 컨테이너 값을
사용한다.

## 판단

- 현 구성은 목표 10 FPS에서 약 9.98 FPS를 유지한다.
- CPU·메모리·온도에는 현재 부하 기준 여유가 있다.
- 20~30 FPS 증가는 NN, 후처리, encoder와 메모리 대역폭을 함께 재측정해야 한다.
- 다음 계측 후보는 장시간 CPU·온도 추이, 세 RTSP 동시 접속 네트워크 부하,
  프레임 drop/latency, NPU/GPU engine 사용률과 실제 차량 환경의 Depth 정확도다.
