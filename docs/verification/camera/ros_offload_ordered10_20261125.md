# 10FPS 추론 결과 순서 보존 및 호스트 최적화 (2026-11-25)

## 최종 상태

CAM01 169.254.100.228/1142227768, 앱 UUID61fbe0c1-ad26-42dc-93c4-a8d2b5191fce,
실행환경0.33.10 유지.95,196bytes 코드만 배포했다.
활성 release `12c21dba0a11cddaae23331faacbb3ad6711d66ac8434870cc7daa452b9a64cd`,
pending/failure 없음. 직전 e9f2f1d… 코드를 복구본으로 유지한다.
현재 라이브 카메라 입력이며 WSL의 bridge/postprocess 두 노드를 실행 중이다.

## 변경

- ros_offload.json: input_fps=10, export_fps=null. 추론 결과를5FPS로 샘플링하지 않는다.
- 카메라 인코딩/클라이언트 송신/호스트 수신은 FIFO8개. 최신 결과로 이전 결과를
  교체하지 않는다. 큐가 막히면 역압을 적용하며 카메라 큐가2초 이상 진행하지 못하면
  명시적 오류를 낸다. 수신된 오래된 프레임을 조용히 폐기하지 않는다.
- source_frame_index와 별도로 연속 export_seq를 부여해 bridge/processor에서 누락을 감지한다.
- PC 입력도 순서 보존 모드. 프레임 소비를 기다리고 stop 시 마지막 프레임을 먼저
  소비한다. VideoFileSender는 이 모드에서 시간 따라잡기를 위한 건너뛰기를 하지 않는다.
- ROS Image의 bytes setter를 호출하면 바이트마다 Python 검증이 실행되던 경로를
  제거했다. 기본 uint8 배열의 frombytes와 NumPy 버퍼 뷰를 사용한다.
- 기가비트 유선 링크에서 RGB는 무손실 raw, 마스크와1920×1080 Depth는 zlib로 보낸다.
  해상도·픽셀값·보정값은 줄이지 않았다.
- ROS 영상/묶음 QoS RELIABLE/32, 후처리 결과 및 timing RELIABLE/64.
- 결과 JSON에 source, export_seq, objects, depth_available을 포함한다.

메모리에 순서를 보존하는 연결 세션 전송이며, 연결 단절 중의 영구 저장·재생은
구현하지 않았다. 이 검증은 아래10FPS 입력/구독 부하 조건의 결과다.

## 병목 확인

WSL의 동일 RGB/Depth 배열을 ROS Image로 만드는5회 비교:

| 데이터 | 기존 bytes setter 평균 | uint8 배열 직접 복사 평균 |
|---|---:|---:|
| RGB960×540 BGR8 | 65.79ms | 0.81ms |
| Depth1920×1080 uint16 | 173.12ms | 1.81ms |

전송 payload를 작게 만들기 위해 영상을 손실 압축하거나 Depth를 색상 영상으로
대체한 결과가 아니다. tools/benchmark_oak_ros_images.py로 재현한다.

## PC 도로 영상 60초 전체 토픽 검증

사용자 cam_01_rgb_20261104_073117.mp4의0~60초를10FPS로 전송했다.
camera telemetry(NN 이후), ROS frame, postprocess result를 원본 index0~599로 대조했다.
동시에 아래 개별 토픽도 모두 구독해 각 frame header와 대응했다.

| 단계/토픽 | 고유 프레임 | 누락/중복 |
|---|---:|---:|
| PC 입력 | 600 | 0/0 |
| 실제 NN 추론 완료 | 600 | 0/0 |
| /oak/frame | 600 | 0/0 |
| /oak/postprocess/result | 600 | 0/0 |
| /oak/rgb/image_raw | 600 | 0/0 |
| /oak/drivable_mask | 600 | 0/0 |
| /oak/lane_mask | 600 | 0/0 |
| /oak/depth/image_raw | 600 | 0/0 |
| /oak/rgb/camera_info | 600 | 0/0 |
| /oak/depth/camera_info | 600 | 0/0 |
| /oak/detections | 600 | 0/0 |
| /oak/postprocess/debug_image | 600 | 0/0 |
| /oak/postprocess/top_image | 600 | 0/0 |

NN/RGB/ROS/result sequence 불일치0, PC 영상에 실제 Depth 혼입0.
종료 응답에서 received=consumed=600, dropped=0.
전체 실행60.47초, 처음3초 제외 결과 수신 간격 기준9.967FPS.
파일 마지막 프레임의 결과까지 기다려 대조했으며, 매100ms 정확한 균등 간격의 보장은 아니다.

동일 호스트 monotonic clock으로 측정한 정상 상태570프레임:

| 구간 | 평균 | P95 |
|---|---:|---:|
| PC 전송 시작→ROS 결과 구독 콜백 | 272.91ms | 388.09ms |
| 원본 묶음 복원 완료→후처리 ROS 콜백 | 17.10ms | 26.34ms |
| 호스트 후처리 | 52.37ms | 63.74ms |
| 결과 직렬화·publish | 0.48ms | 0.59ms |

기존 원본 묶음→콜백 평균354.87ms에서17.10ms로 줄었다.
기존 시험은 결과 publish 반환, 이번 끝단은 실제 result 구독 콜백이므로
전체 지연의 측정 끝점 차이를 구분해야 한다. 이번 전송→결과 수신 최대는694.56ms.

## 실제 Depth가 있는 라이브 입력 확인

별도30.02초 창에서 원본 frame309개/result309개/debug310개/TOP310개 수신,
stale0. 창 시작/종료의 비동기 토픽 경계 때문에 개별 카운터는1개 차이가 있다.
마지막 프레임은 source=camera, depth_available=true,
Depth1920×1080 16UC1, 유효 Depth856,212픽셀, RGB/Depth 시각 차이 약1.016ms.
카메라 로그의 export_rejected=0, export_errors=0, export_queue_high_water=1.
라이브 입력에서도10FPS 수준을 유지하고 큐 누적이 없음을 확인했다.

## 테스트 및 자료

- 카메라/멀티뷰 전체 회귀432개 통과(68.48초).
- ROS 메시지 직렬화·역직렬화, stride 입력 복사, 잘못된 버퍼 거부3개 통과.
- FIFO가 가득 찼을 때 앞 프레임을 보존하는지, 과부하 시 오류를 내는지,
  빠른12개 입력도 전송 상한 없이 순서대로 보내는지, PC stop이 마지막 프레임을
  지우지 않는지, raw/zlib 모두 무손실인지 검사했다.
- artifacts/ros_offload/ordered10_60s_20261125/report.json 및 identities.json
- artifacts/ros_offload/ordered10_live_20261125/report.json
- artifacts/ros_offload/ordered10_before.json, ordered10_deployed.json
- artifacts/ros_offload/tests_10fps.log, ordered10_ros.stdout.log 및 stderr.log

재현:

```bash
bash tools/verify_oak_10fps.sh \
  --video artifacts/recordings/six_camera/six_camera_20261104_073117_05e93c/cam01/cam_01_rgb_20261104_073117.mp4 \
  --seconds 60 --output artifacts/ros_offload/ordered10_verification
```

ROS 노드가 실행 중이고 PC publisher가 없는 상태에서 실행한다.
검증 종료 시 실제 카메라 입력으로 복구한다.
