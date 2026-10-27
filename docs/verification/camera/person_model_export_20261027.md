# YOLOPv2 person 모델 내보내기 (2026-10-27)

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

## 남은 변환 단계

OAK4-D Pro 목표 설정은 RVC4, FP16, SNPE2.41.0. 사용자가 HubAI 업로드를 승인하고 직접 웹에 업로드했다고 알렸지만, 브라우저 URL 검증 실패 후 자동 승인 검토가 추가 UI 접근을 거절하여 최종 변환 상태는 확인하지 못했다. 해당 제한을 우회하지 않았다.

최종 `.rvc4.tar.xz`/`.dlc` 생성 및 장치 추론 검증은 **미완료**. 다운로드 폴더의 2026-10-02 `yolopv2_320.rvc4.tar.xz`는 이전 모델이므로 이번 변환 결과로 취급하지 않는다. 카메라 모델 교체/배포 없음.
