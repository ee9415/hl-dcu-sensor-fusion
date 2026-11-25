# ROS postprocess 영상 몰림 원인 비교 (2026-11-25)

후처리 JSON이 약100ms 간격으로 도착하는데 debug/TOP 영상은 약3초씩 묶여
도착했다. PC 재시작 측정에서 result 최대160ms, debug최대2,994ms,
TOP최대2,963ms였다. 이후 기본 설정에서는 bridge ACK 재전송과 수신 FIFO 적체,
카메라 session 변경 및 stale까지 관찰됐다. 평균 FPS만으로 정상 판단할 수 없다.

## 근거와 비교

- debug960×540 BGR8=1,555,200bytes, TOP640×480=921,600bytes.
- 실제 `/dev/shm/fastrtps_*` 영역은 약537KiB로 Fast DDS 기본512KiB 데이터 영역과
  부가정보 크기에 해당했다. 전체 `/dev/shm` 공간 부족은 아니었다(7.7GiB 중2.5MiB 사용).
- [Fast DDS2.6 공식 문서](https://fast-dds.docs.eprosima.com/en/2.6.x/fastdds/transport/shared_memory/shared_memory.html)는
  기본512KiB와 메시지 크기 이하 영역의 데이터 손실 위험을 명시한다.
- 분리 도메인의1.55MB/10FPS/100개 합성 영상: 기본 설정 수신100개·전송 지연
  중앙14.0ms/P95 25.1ms,64MiB 설정100개·중앙3.4ms/P95 4.0ms.
  단순 합성 시험의 기본 설정에서는3초 몰림까지 재현되지는 않았다.
- 실제 프로세스에64MiB 프로필을 적용한 뒤 단일 bridge/processor로 비교했다.
  카메라 입력30초 영상/TOP 각300개, 최대166ms,250ms초과공백0.
- PC 도로 영상 재시작 후30초 영상/TOP/result 각299개, 약9.99FPS,
  영상최대165.8ms/TOP165.4ms,250ms초과공백0,stale0,순서단절0.
  각 토픽20ms미만간격은1회. 호스트수신→결과발행 P95 83.3ms.

비교 재시작 시 기존 nohup 런치가 SIGINT를 무시해 잠시2개씩 실행됐다.
중복 상태 측정(`live_shm64.json`)은 비교 근거에서 제외했다. 이전 PID들을
SIGTERM으로 종료하고 발행자/구독자 각1개를 확인한 뒤 `live_shm64_single.json`과
`pc_shm64.json`을 측정했다. 이는 최초 몰림 이전의 원인이라고 해석하지 않는다.

주요 원인은 영상 크기에 맞지 않은 DDS 전송 설정으로 판단한다. 패킷 캡처로
내부 재전송 경로까지 특정한 것은 아니다. 입력은10FPS이며 화면 표시 지연이나
장시간 무결성을 입증한 시험은 아니다. 카메라 모델·앱은 변경하지 않았다.

## 적용 상태

`tools/ros_fastdds_large_images.xml`을 추가하고 ROS 실행기·뷰어에서 기본 사용한다.
명시된 사용자 프로필은 보존한다. 현재 ROS 프로세스는 동일내용의 실험용 프로필
`artifacts/ros_offload/stream_fix_20261125/shm64.xml`로 실행 중이며,
다음 실행부터 저장된 tools 프로필을 사용한다. PC 도로 영상 전송과 결과 창 유지.

측정 자료: `artifacts/ros_offload/stream_fix_20261125/pc_restart.json`,
`live_default.json`, `live_shm64_single.json`, `pc_shm64.json`.
구문 검사와 실제 미들웨어 XML 로딩을 확인했다.
