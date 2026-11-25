# A-YOLOM v4 OAK-4D Pro PoE 적용 및 추론 검증

날짜: 2026-11-25. 사용자 요청 범위는 연결된 OAK-4D Pro PoE 한 대이다. OAK-D Pro PoE는 미연결로 변경하지 않았다.

## 적용 결과

CAM01 `169.254.100.228`, device ID `1142227768`, RVC4에 A-YOLOM v4 추가 학습 모델을 적용했다. 실제 장치의 NN 입력은 RGB FP16 NHWC `[1,384,672,3]`이며 RGB 0~255를255로 나눈다. 원본 카메라1920×1080을 비율 보존해672×384에 넣고 패딩114를 사용한다.

- 아카이브 SHA256 `a2144ca62766806c5d8100f80ca2dca8eacffb395cd106b9266adac5be5c9a97`가 장치 시작 로그와 일치했다.
- 새 패키지0.34.0, UUID `d0fd9213-9d81-483f-b24c-bee7236ab061`, DepthAI3.8.0, SNPE2.41.0 대상sm8550.
- 새 앱은 기본 embedded 코드로 실행하며 active/pending/previous는 모두null이다. 코드 릴리스 실패가 아니라 새 앱 저장소의 초기 상태이다.
- runtime fingerprint `b945fc7184d97c72db54bf9f0c72af2e7e47e05eecccd774a76b3cc1249fe641`.
- 기존 앱0.33.10 UUID `61fbe0c1-ad26-42dc-93c4-a8d2b5191fce`는 중지 상태로 보존했다. 기존 active `12c21dba0a11cddaae23331faacbb3ad6711d66ac8434870cc7daa452b9a64cd`와 previous `e9f2f1d52d6ed026ba238ea9ca6bf52a9c154d79f3ea88de38461adb89fd7396`도 삭제하지 않았다.

## 코드 및 영향

`model_config.json`에서 모델 계열·크기·아카이브를 선택하고, 시작 전에 아카이브 입력과 필수 출력 이름을 검사한다. 런처는 모델 파일을 설치 컨테이너의 모델 디렉토리에서 읽는다.

`ayolom_postprocess.py`는 차량1클래스 검출 헤드와 추가4클래스 검출 헤드를 모두 DFL(reg_max16), stride grid 및 sigmoid로 디코드한다. YOLOv8에는 objectness가 없으므로 공통 NMS 입력에만 상수1을 넣는다. 클래스ID는0 vehicle,1 person,2 tl_red,3 tl_yellow,4 tl_green이다.

분할은 배경 포함2채널 logits를 사용한다. 672×384에서 원본16:9 영상의 패딩 상하3행을 제거하고 logits를320×180으로 bilinear 보간한 후 클래스 선택한다. 기존 ROS·경계 추적의 좌표를 보존하며 모델 추론 해상도를 줄이지 않는다. ROS topic/namespace/TF 및 기존 전송 포트는 바꾸지 않았다.

## 패키지 빌드와 복구본

일반 `oakctl app build`는 장치 외부DNS 실패와1970년 시계 상태로 기본 이미지를 가져오지 못했다. 장치 네트워크·시계 설정을 변경하지 않고 `repack_oak4_app.py`로 정상0.33.10 패키지의 런타임2개 레이어를 그대로 복사했다. requirements/install-runtime/runtime-base가 기존과 같은지도 검사하고, 앱 레이어 전체를 현재 코드·새 모델로 교체했다. 기존 실행 중 `/app` 파일 덮어쓰기는 수행하지 않았다.

- 전체 패키지: `artifacts/ayolom_deployment_20261125/dist/com-qsentech-oak4-yolopv2-standalone_0-34-0.oakapp`, 894,177,280 bytes.
- 코드·모델 staging:38파일,20,006,772 bytes. 이전 모델·원본PT·ONNX·다른 변환파일은 설치 패키지에 넣지 않았다.
- 보존한 `layers/lower-0.tar.lz4` SHA256 `31a9a92b9738bc626b11ea47c93c81c9ecbd7880de8b12aaf7b3c372216ba5d1`.
- 보존한 `layers/lower-1.tar.lz4` SHA256 `4ef48cd0979960bec6a244be472c93d8e6bb3c21f2f2fb627e171fb82444be6a`.
- `--force-new --start=false --enable=false --keep-others`로 별도 설치 후 기존 앱 중지→새 앱 시작했다. 부팅 자동 시작은 기존처럼 비활성 상태를 유지한다.

## 실제 장치 검증

라이브 카메라30초 결과는 `live/report.json`에 보존했다.300프레임·고유300·arrival10.0034FPS·capture9.9998FPS, RGB/NN 시퀀스 불일치0, export 시퀀스 누락0, 오류0이다. 최대RGB/Depth 시간차1.412ms였다. 장면은 사무실이므로 도로/차선 마스크0픽셀을 검출 실패나 정확도 결과로 해석하지 않는다.

장치의 실제8개 출력은 NCHW 기준 기본 검출 `[1,65,48,84]`, `[1,65,24,42]`, `[1,65,12,21]`, 도로/차선 각각 `[1,2,384,672]`, 추가 검출 `[1,68,48,84]`, `[1,68,24,42]`, `[1,68,12,21]`로 확인했다. 모두 유한값이었으며 로그 `NN_MODEL`에 최소·최댓값도 기록했다.

기존 CAM01 도로 영상0~20초를10FPS로 장치에 전송했다. 입력/응답/NN완료200개, 원본인덱스0~199 누락0, RGB/NN 불일치0, 송신오류0이다. 장치 처리 완료 알림 수신 구간 기준9.865FPS였고 입력20초당 완료 수는10FPS이다. 두 측정 정의를 구분한다. 종료 응답에서received=consumed=200, dropped=0을 확인하고 카메라 입력으로 복귀했다.

별도35초 묶음 수집347개에서 도로 마스크 최대25,717픽셀, 차선 최대1,738픽셀, 차량·사람·녹색 신호등 클래스 출력이 관찰됐다. 이 구간에는 도로 재생 전후의 라이브 프레임도 포함된다. 검출된 클래스가 정답이라는 의미는 아니다.

## ROS 확인 및 최종 상태

최초 ROS 검사는 원본181개만 수신하고 result/debug/TOP이0개였다. `ros2 node list`와 프로세스 목록에서 기존 bridge만 실행 중임을 확인했다. bridge는 유지하고 기존 `hl_oak_perception postprocess` 노드만 재실행했다. 소스 코드는 변경하지 않았다.

재검사20초에서 frame201·result203·debug203·TOP203, stale0, passed=true를 확인했다. 구독 연결 시점 차이로 토픽 개수가 다르며 각 개수를 동일 프레임 일대일 정합성 검증 결과로 취급하지 않는다. 최종 입력은 실제camera, Depth1080×1920 16UC1, RGB540×960, 마스크180×320이다. 새 카메라 앱과 ROS bridge·후처리를 유지했다. 후처리 PID와 로그는 검증 폴더의 `ros_postprocess.pid`/`ros_postprocess.log`에 저장했다.

## 테스트 및 한계

- Linux 전체436개 중435개 통과, 진단용 패치 테스트1개가 분할 함수의 이전 직접 호출 문자열을 기대해 실패했다. 실제 모델별 dispatch 호출을 검사하도록 갱신 후 해당 파일2개 테스트 모두 통과했다.
- 새 DFL의 분석적 박스 기하, 추가 클래스 매핑, NCHW/NHWC 동등성, 패딩 제거/배경 tie 및 비유한값 거부4개 검사 통과. 런처/복구9개와 ROS 계약9개도Linux에서 통과했다.
- Windows 전체 검사는Linux 전용fcntl 오류 및 정지로 중단하고Linux로 대체했다.
- 본 검증은 모델 로드·출력·연속 추론·전송·ROS 연결을 확인한 것이다. 동일 입력의 DLC/ONNX 수치 동등성과 정답 레이블 기반 검출 정확도, 실차 도로 주행 성능은 검증하지 않았다. OAK-D Pro PoE의 실장치 시험도 남아 있다.
- 작업 전부터 있던 ROS 관련 미커밋 변경을 보존했다. 이번 결과에 다른 작업의 변경을 섞어 자동 커밋·푸시하지 않았다.

## 재현 및 복구

실제 카메라 결과 검사:

```powershell
.venv/Scripts/python.exe src/hl_camera_bringup/scripts/verify_ayolom_oak4.py --device-ip 169.254.100.228 --seconds 30 --output artifacts/ayolom_deployment_20261125/recheck
```

이전 모델로 복구할 때만 다음 명령을 실행한다:

```powershell
& 'C:/Program Files/oakctl/bin/oakctl.exe' app stop d0fd9213-9d81-483f-b24c-bee7236ab061 -d 169.254.100.228
& 'C:/Program Files/oakctl/bin/oakctl.exe' app start 61fbe0c1-ad26-42dc-93c4-a8d2b5191fce --enable=false -d 169.254.100.228
```

검증 자료는 `artifacts/ayolom_deployment_20261125/`의 package manifest, build JSON, camera.log, installed_apps, active/previous release, live/road_bundles/road_replay/ros_recheck 하위 결과에 보존했다.
