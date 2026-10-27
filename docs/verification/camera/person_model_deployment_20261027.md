# 사용자 person RVC4 모델 장치 배포 (2026-10-27)

## 배포 대상 및 파일

- 장치: OAK4-D Pro 1142227768, 192.168.201.2.
- 실행 중인 개발 앱: 00000000-0000-0000-0000-000000000000.
- 사용자 지정 원본 폴더: D:/git_hub/OAK_4D_PRO/apps/yolopv2_standalone/models/yolopv2_person.
- 실제 배포 파일: yolopv2_person_320.rvc4.tar.xz, 140,340,160바이트.
- SHA256: e5ee74a1dca29d877cb57559b59ded12ac3a204c4cf848bd170840fcbc640afa.
- 장치 경로: /app/models/yolopv2_person_320.rvc4.tar.xz.
- 저장소 내 로컬 빌드 사본: apps/yolopv2_standalone/models/yolopv2_person_320.rvc4.tar.xz. 대용량 바이너리 제외 규칙에 따라 Git에는 포함하지 않는다.

이미 RVC4로 변환된 아카이브를 사용했으며 PT/ONNX 파일을 직접 실행하지 않는다. 이전 변환·다운로드 검증은 person_model_export_20261027.md 참조. 파일명과 달리 검출 출력은80클래스 구조다. 사람 전용 검출 모델이라고 확인한 것은 아니다.

## 변경과 보호 범위

main.py의 기본 아카이브를 person 파일로 변경했다. YOLOPV2_MODEL_ARCHIVE 환경 변수로 다른 아카이브 경로를 지정할 수도 있다. RGB FP16 NHWC 및 /255 명시적 전처리, 기존 후처리, 경계 추적, PC 영상 입력, RTSP/텔레메트리 경로는 유지했다. 기존 baseline 파일 /app/models/yolopv2_320.rvc4.tar.xz는 덮어쓰지 않았다.

deploy_person_model.py는 아카이브의 입력/출력 이름·크기를 확인하고 로컬 빌드 사본을 준비한다. 장치의 임시 TCP8559 HTTP 업로드 서버는 임의 토큰 경로와 정확한 크기만 허용한다. 업로드한 데이터의 SHA256이 일치할 때만 모델 경로로 교체하고 서버를 종료한다. 유효 요청이 없으면120초 뒤 종료한다. source main.py도 문법 검사 후 교체하고 backend만 재시작했다. 상시 외부 업로드 서비스를 추가하지 않았다.

교체 전 main.py와 이전 person 파일(존재 시)은 /app/person_model_backup_533c123720에 보존했다. baseline 모델로 복구하려면 이 디렉터리의 main.py를 /app/main.py로 복원한 뒤 sv restart /etc/service/backend를 실행한다. 기본 모델 선택 변경이므로 다음 앱 빌드에는 person 아카이브 로컬 사본이 필요하다. Git clone만으로 모델 바이너리가 제공되지는 않는다.

현재 배포는 개발 컨테이너 수정이다. 영구 배포용 .oakapp 이미지를 다시 빌드/설치한 것은 아니다. 외부 원본 폴더의 파일은 수정하지 않았다.

## 실제 검증

1. PC/장치 파일 SHA256 일치 및 main.py 문법 검사 통과. deployment.json에 장치 경로·백업·main SHA256 기록.
2. 장치 로그에서 2026-10-27 07:17:17 UTC `Model: /app/models/yolopv2_person_320.rvc4.tar.xz`, 07:17:29 UTC full pipeline running 확인.
3. 도로 녹화 파일의2~5초 구간을12초 동안 반복 송신하고 일시정지/재개·카메라 복귀 확인.
4. PC 입력 결과116프레임, 출력8.796 FPS. 송신135/수신135, 소비125. 구간 밖 시각0, NN/프레임 시퀀스 불일치0, 중심선0, PC 입력의 유효 Depth 객체0, 오류0. 경계 상태 tracking116프레임, 객체 결과1203건 반환. 이는 검출 정확도 평가를 의미하지 않는다.
5. 짧은 검사에서 지속10FPS 목표는 충족하지 못했다. 이전 모델의 다른 구간·시간대 검사와 직접적인 성능 우열을 단정하지 않는다.
6. 교체 후 RTSP 단회 연결 검사에서 인식 결과960×540, Depth640×360, BEV640×480 수신/저장 성공. 원본 RGB는 연결 실패했으므로 네 멀티뷰 전체 정상으로 판정하지 않는다. streams/streams.json 참조.

근거: artifacts/person_deployment_20261027/deployment.json 및 verification/report.json, verification/frames.jsonl, verification/telemetry/의 CSV. 실제 추론 실행과 텐서 스키마 호환성은 확인했지만 DLC/ONNX 수치 동등성 및 사람 검출 정확도의 정량 평가는 수행하지 않았다. ROS2 인터페이스 변경이 없어 colcon/ROS2 launch 검사는 해당하지 않는다.

장치 backend 재시작 시 기존 PC 송신 연결은 끊어진다. 검증 종료 시 실제 카메라 모드로 복귀했으며, 사용자가 PC 영상으로 시험하려면 멀티뷰에서 소스 적용/전송 시작을 다시 눌러야 한다.
