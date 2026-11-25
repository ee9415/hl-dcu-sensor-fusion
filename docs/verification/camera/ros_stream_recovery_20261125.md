# ROS 후처리 영상 단절 복구 (2026-11-25)

## 확인한 문제

카메라 bridge는 연결을 유지했지만 processor가 `ROS inference frame sequence gap`의
미처리 RuntimeError로 종료됐다. 종료 후 25초 측정에서 debug/TOP/result는 각각0개,
bridge_status만25개였다. 재시작 직후의10FPS만으로 지속 동작을 판단할 수 없었다.
RELIABLE 구독과 달리 기본 `ros2 topic hz`에서는 영상 수신이 희박하게 관찰되어,
이 검증에서는 표시기와 측정기 모두 RELIABLE 구독을 명시했다.

## 수정

- bridge→processor는 한 번에 한 묶음만 전송하고 완료 확인을 기다린다.
  0.5초 후 같은 묶음을 재전송하며, processor는 중복 입력을 다시 처리하지 않고
  확인만 반복한다. 입력 FIFO는 유지하고 후처리 구독자가 없으면 꺼내지 않는다.
- 순서 오류를 로그·stale 결과·sequence_gaps/continuity_error로 드러내고,
  추적기를 초기화한 뒤 현재 프레임부터 복구한다. 빠진 프레임을 복원했다고
  간주하지 않는다. 카메라 연결 단절 중의 영구 저장은 추가하지 않았다.
- 결과 뷰어는 RELIABLE 구독과 새 프레임만 그리는 방식으로 변경했다.
  기존 반복 그리기 프로세스 약85%→새 뷰어 약16~18% CPU를 관찰했다.
- 카메라 앱·모델은 변경하지 않았다. 현재 PC 도로 영상 입력을 유지했다.

## 검증

WSL Ubuntu22.04, CAM01 169.254.100.228, 기존 A-YOLOM v4 앱·10FPS 입력.
ROS 두 패키지 빌드 성공. 전달 제어/복구/직렬화9개와 기존 오프로딩9개 테스트 통과.

120초 측정(각 토픽의 구독 시작/수신 순서는 독립적):

| 항목 | 결과 |
|---|---|
| debug/TOP/result | 각각1,200개, 약10.004FPS |
| stale/결과 순서 단절 | 각각0 |
| debug 수신 간격 P50/P95/최대 | 97.4/149.2/322.8ms |
| TOP 수신 간격 P50/P95/최대 | 97.1/146.8/236.5ms |
| debug/TOP 250ms 초과 공백 | 3/0회 |
| 호스트 묶음 수신→결과 발행 P50/P95/최대 | 57.7/80.3/113.0ms |
| FIFO high-water/재전송 | 1/0 |

이후 processor를0.65초 SIGSTOP/SIGCONT로 일시 정지·재개했다.
bridge는1회 재전송, FIFO high-water6, 이후 pending0으로 따라잡았다.
재개 후20초 측정은 debug/TOP203개, result204개, 순서 단절/stale0.
이20초 표본은 정지 구간 이후 시작되므로 정지 중의 공백을 포함하지 않는다.
bridge/processor/결과 뷰어는 검증 종료 후 실행 상태로 유지했다.

평균 FPS는 화면 표시 지연을 뜻하지 않는다. 입력10FPS 자체의100ms 간격과
관찰된 수신 지터는 남아 있으며,30FPS 수준의 연속성을 검증한 것은 아니다.
프레임 누락이 처음 발생한 DDS 내부 원인을 패킷 단위로 특정한 것은 아니다.

자료: `artifacts/ros_offload/stream_fix_20261125/report.json`, `recovery.json`,
`ros.log`, `viewer.log`. 재현 명령은 [운영 안내](../../operation/oak_ros_offload.md).
