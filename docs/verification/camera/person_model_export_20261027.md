# YOLOPv2 person 모델 내보내기 (2026-10-27)

후속 상태: 같은 날짜에 사용자 요청으로 실제 장치 배포/추론을 수행했다. 아래 변환 당시 미검증 기록과 구분하며, 최신 결과는 [장치 배포 검증](person_model_deployment_20261027.md)을 참조한다.

## 입력 및 산출물

- 사용자 지정 원본: `D:/git_hub/OAK_4D_PRO/apps/yolopv2_standalone/models/yolopv2_person/yolopv2_person.pt`
- 원본 SHA256: `748dec45f214032fc29cd91d9bf6ff4ec3cad5b0d28fa238c62350d7b6a499e4`
- ONNX: 같은 폴더의 `yolopv2_person_320.onnx`
- ONNX SHA256: `31a1e27b98e1ef34f0c021480479c0da021fee1cd0168eab5ae7c25fa91ead82`
- 검증 결과: 같은 폴더의 `yolopv2_person_320.validation.json`
- 이전 중첩 폴더의 모델과 해시 동일. 새 경로에서 재생성했으며 ONNX 해시도 동일.

## 검증

`src/hl_camera_bringup/scripts/export_person_onnx.py`로 원본 TorchScript를 로드하고 320×320 고정 입력, opset17로 내보냈다. ONNX checker 통과. 실제 이미지 및 고정 seed27 임의 입력 각각에서 PyTorch와 ONNX Runtime의 5개 출력 모두 `atol=2e-4, rtol=1e-4` 검사 통과. 전체 최대 절대오차 `4.57763671875e-05`.

입력은 RGB float32 NCHW `[1,3,320,320]`, 값 범위0~1 (`pixel/255`). 출력은 검출3개 `[1,255,40,40]`, `[1,255,20,20]`, `[1,255,10,10]`, 도로 `[1,2,320,320]`, 차선 `[1,1,320,320]`. 검출 출력은 80클래스 구조이며 파일명만으로 person 단일 클래스 모델이라고 판단하지 않는다.

Windows 기존 PyTorch는 DLL 정책 오류로 실행되지 않아 작업 폴더의 격리된 WSL Python3.10 환경을 사용했다. PyTorch2.5.1+cpu, ONNX1.17.0, ONNX Runtime1.20.1, NumPy1.26.4.

## RVC4 변환 결과 및 남은 검증

OAK4-D Pro 목표 설정은 RVC4, FP16, SNPE2.41.0. 초기 Chrome 제어는 실패했으나 이후 사용자가 연 앱 내 브라우저에서 URL과 로그인 상태를 정상 확인했다. 사용자가 파일을 직접 선택한 뒤 `yolopv2_person_320.onnx` 표시, 320×320, FP16, SNPE2.41.0을 확인하고 제출했다.

- 새 작업: Model 3 - Instance 1, 시작2026-10-27 14:31 KST, 종료14:36 KST. Jobs의 Completed 및 인스턴스 Success 확인.
- 모델ID `aim_TcJqVELh67WsngvbR5Dxru`, 버전ID `aimv_TcJqVHc4wvgsg1MwuUruNz`, 인스턴스ID `aimi_UE7qo7RbM6YjHiUU6HLsof`, Hub 해시 `5d33740`.
- 출력 파일명: `yolopv2_person_320.rvc4.tar.xz`.
- 결과 URL: https://hub.luxonis.com/ai/models/aim_TcJqVELh67WsngvbR5Dxru/version/aimv_TcJqVHc4wvgsg1MwuUruNz?view=successful-export&instanceId=aimi_UE7qo7RbM6YjHiUU6HLsof
- 성공 화면: `artifacts/person_conversion_20261027/rvc4_success.png`.
- 다운로드 버튼은 화면에 있지만 자동 다운로드 이벤트 대기와 링크 클릭이 시간 초과 또는 저장 시작 없음으로 끝났다. 로컬 새 person 아카이브는 확인하지 못했다. 다운로드 후 `inspect_rvc4_archive.py`로 텐서 스키마·DLC 및 SHA256을 검사할 수 있도록 도구를 준비했고 구문 검사는 통과했다. 실제 새 아카이브에 대한 실행 검증은 **대기**.
- 클라우드 변환 성공과 실제 장치 수치 검증을 구분한다. DLC 내부 출력 수치 및 장치 추론은 **미검증**. 카메라 모델 교체/배포 없음.
- Quick Conversions의 기존 Completed가 가리킨 Model1/해시0455dd6은 이번 결과로 취급하지 않았다. Jobs에는 이전 Model2 작업도 Completed로 있으나 출처 확인 없이 이번 산출물로 사용하지 않았다.

## 사용자 다운로드 파일 검증 완료

- 2026-10-27 15:05 저장된 `yolopv2_person_320.rvc4.tar.xz`를 사용자 지정 원본 폴더에서 확인. 크기140,340,160 bytes, SHA256 `e5ee74a1dca29d877cb57559b59ded12ac3a204c4cf848bd170840fcbc640afa`.
- 사용자 스크린샷의 Hub 표시 해시는 `c7e818b`. 앞서 자동 제출한 Model3의 `5d33740`과 다르므로 동일 인스턴스라고 단정하지 않는다. 받은 파일의 buildinfo는 2026-10-27 06:00:01 UTC 변환 경로이며 원본 ONNX SHA256은 포함하지 않는다. 원본과 DLC의 수치 동등성은 미검증.
- 파일 내부 메타데이터의 모델명 `yolopv2_person_320`, FP16, SNPE2.41.0.251128145156_191518, target sm8550, graph optimization level2 확인.
- 입력NHWC `[1,320,320,3]`, 검출출력 `[1,40,40,255]`/`[1,20,20,255]`/`[1,10,10,255]`, 도로 `[1,320,320,2]`, 차선 `[1,320,320,1]` 모두 검사 통과.
- DLC158,624,007 bytes를 원본 폴더의 `yolopv2_person_320.dlc`로 추출·복사하고 SHA256 `51527c3e0f6b4c2759a945803499d9cc46418b4dca6c127e7b3a0fe3bd1b9a3f` 일치 확인.
- 동일 폴더에 `yolopv2_person_320.rvc4.inspection.json` 및 기존 validation.json의 다운로드 검증 상태 저장.
- 다운로드한 아카이브의 preprocessing mean/scale은 null이고 dai_type은 BGR888i. 원본 입력은 RGB pixel/255이므로, 배포 시 색상 순서와 정규화를 코드 또는 NNArchive 런타임 설정에서 명시적으로 맞춰야 한다. 다운로드 원본 아카이브는 수정하지 않았다.
- 실제 장치 추론, DLC/ONNX 출력 수치 비교, 처리 FPS 및 새 모델 배포는 수행하지 않았다.
