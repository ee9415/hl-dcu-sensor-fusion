# 기본 인식 / 전체 후처리 옵션 (2026-10-27)

## 사용자 동작

멀티뷰를 재실행하고 영상 연결한 뒤 **전체 후처리 (경계 추적·가상 경계·정지선)** 체크박스로 전환한다. 기본값은 해제다.

| 옵션 | 인식 결과 화면 | 실행 계산 |
|---|---|---|
|해제: 기본|객체 박스·녹색 주행 영역·하늘색 원본 NN 차선 마스크|기존 텐서 decode/NMS/마스크 처리와 표시|
|체크: 전체|기존 좌우 경계·가상 경계·신뢰도·정지선 표시|기본 결과에 경계 추적/시간 보정/정지선/BEV 추가|

모델 추론은 한 번만 수행한다. 별도의 NN이나 두 개의 인식 영상 인코더를 만들지 않고 기존 결과 RTSP8554를 재사용한다. 기본 모드는 경계/정지선/BEV 계산 함수를 호출하지 않는다. BEV 창은 비활성 안내 화면으로 갱신한다. 전체 모드는 이전 알고리즘과 임계값을 유지하며 중심선은 추가하지 않았다.

Depth·객체 거리 데이터의 기존 공급 경로와 PC 입력 거리 무효화는 유지한다. 기본 화면은 NN 결과에 집중해 객체 거리 라벨을 그리지 않는다. 기본 모드도 RGB/결과/Depth 영상 생성·송출이 남아 있으므로 완전한 실전 무화면 모드나 순수 NPU 벤치마크는 아니다.

## 전환과 데이터 정합

TCP8558에 processing 명령(basic/full)을 추가했다. 기존 PC 발행자 연결과 파일 위치·반복 회차를 바꾸지 않는다. 모드가 실제로 바뀔 때 processing_epoch를 증가시키고 주입 프레임에 해당 모드·세대를 저장한다. 출력은 NN 시퀀스에 매칭된 입력의 모드를 사용한다.

모드 전환은 이후 주입 프레임에 적용된다. 이미 대기 중인 프레임은 원래 모드로 처리될 수 있고, 입력이 일시정지된 경우 재개 후 적용된다. UI 문구와 영상의 BASIC/BOUNDARIES 표시로 구분한다. 소스 또는 처리 세대가 바뀌면 경계/정지선 추적기를 초기화해 보관 경계가 재등장하지 않도록 한다.

기본 모드의 boundary/stop 상태는 unavailable과 구분해 disabled로 기록한다. CSV에 processing_mode/processing_epoch를 추가했다. 장치 재시작과 UI 연결 시 기본 인식을 선택한다. 실패한 모드 변경은 UI가 마지막 확인된 값으로 돌아가며 오류를 표시한다.

검증 도구 verify_pc_video_device.py 및 long_run_monitor.py에는 --processing basic/full을 추가했다. 과거 비교 조건을 유지하도록 CLI 기본값은 full이고 종료 시 기본 모드로 복귀한다. 성능 비교 시 두 실행의 모드를 확인해야 한다.

## 검증

- 전체125개 회귀 검사 통과. 기본에서 ImageBoundaryTracker.update, StopLineTracker.update, render_boundary_bev 호출 시 실패하도록 설정한 검사와 원본 차선 마스크 픽셀 검사 통과. 전체 복귀의 이력 초기화, publisher 유지, 프레임별 모드 provenance도 확인.
- 실제 Tk 체크박스 전체→기본 전환과 기존 FPS/반복/진행바/탐색 검사 통과.
- 장치에 yolopv2_node.py, video_transport.py 배포. 백업 /app/deploy_backup_39ec426142. 모델과20FPS 입력 상한은 유지했다. 영구 .oakapp 재빌드는 별도다.
- 사용자 원본 도로 영상60~76초를20Hz로 끊김 없이 반복하며 기본→전체→기본 각 약8초 확인:
  - 기본157프레임/19.59FPS, 경계 disabled157, 지연P95 132ms.
  - 전체54프레임/6.67FPS, tracking54, 지연P95 1365ms.
  - 기본 복귀170프레임, disabled170. 처음에는 대기열 해소로20.94FPS였으며 초기2초를 제외하면19.63FPS. 일시적 큐 해소 속도를 지속 처리량으로 해석하지 않는다.
  - 전 단계 frame/NN 불일치0, 전체 PC 결과 중복0, 송신 연결/파일 위치 유지, 오류0.
- 위 시험은 연속 영상에서 모드를 바꿔 각 단계의 장면·열 조건이 완전히 같지는 않다. 엄밀한 전후 성능 비교나 장시간19.6FPS 보장은 아니다. 목적은 옵션 적용·계산 생략·복귀 검증이다.
- 근거 artifacts/processing_modes_20261027/report.json, frames.jsonl 및 기존 gui_check.json. 도구 src/hl_camera_bringup/scripts/verify_processing_modes.py.

검증 종료 상태는 카메라/기본 모드다. 원복은 deploy_app_files.py rollback --device-ip 192.168.201.2 --backup /app/deploy_backup_39ec426142. ROS2 인터페이스 변경은 없어 colcon/ROS2 launch 검사는 해당하지 않는다.
