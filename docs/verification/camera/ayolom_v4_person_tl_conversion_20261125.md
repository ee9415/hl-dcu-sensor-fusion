# A-YOLOM v4 추가 학습 모델 변환 (2026-11-25)

후속 실장치 검증: OAK-4D Pro PoE에0.34.0 패키지를 적용하여672×384 추론, 라이브300프레임 약10.00FPS 및 도로 영상200/200프레임 완료를 확인했다. ROS 후처리·debug·TOP도 확인하고 라이브 카메라로 복귀했다. OAK-D Pro PoE는 미연결로 미적용이다. [배포·실측·복구 기록](ayolom_v4_oak4_deployment_20261125.md). 아래 변환 당시의 미검증 표기와 구분한다.

## 최종 상태: 두 플랫폼 변환 완료 (2026-11-25)

이 절이 아래 생성·정리 당시의 RVC4 대기 기록을 대체한다. 브라우저 업로드 제약을 해결하는 대신 공식 Luxonis Docker 이미지로 로컬 변환했다. 카메라 배포와 실제 장치 추론은 수행하지 않았다.

- 결과 폴더: `apps/yolopv2_standalone/models/ayolom_v4_person_tl_20261110/oak_4d_pro_poe_rvc4/`.
- NN 아카이브: `ayolom_v4_person_tl_672x384.rvc4.tar.xz`, 19,666,588 bytes, SHA256 `a2144ca62766806c5d8100f80ca2dca8eacffb395cd106b9266adac5be5c9a97`.
- DLC: `ayolom_v4_person_tl_672x384.dlc`, 22,927,619 bytes, SHA256 `742dfb46dbbc30fbc2e3af834dfb1e1bb0b55565d62eb8f0e2d3b03539e1cccc`.
- 보조 파일: `conversion.yaml`, `config.json`, `buildinfo.json`, `archive_inspection.json`, `STATUS.txt`.
- 공식 이미지: `ghcr.io/luxonis/modelconverter-rvc4:2.41.0-latest`, digest `sha256:7554639323ecc2c81c4652286f729029f37f6a31eea9ebcfb56b23d27fad09b9`.
- ModelConverter 0.6.2, SNPE `2.41.0.251128145156_191518`, FP16_STANDARD, sm8550, optimization level 2, float IO. 양자화·캘리브레이션을 적용하지 않았다.
- `snpe-onnx-to-dlc`와 `snpe-dlc-graph-prepare` 모두 종료 코드 0. SM8550 캐시 생성 성공. 전체 변환 약 30.64초.
- 변환에 사용한 복사본 ONNX가 검증된 원본 ONNX와 SHA256 일치함을 확인했다.
- 아카이브 입력 NHWC FP16 `[1,384,672,3]`, 출력 8개가 원본의 NCHW→NHWC 변환 shape와 모두 일치했다. 아카이브 내부 DLC와 별도 DLC의 SHA256도 일치했다.
- RVC4에 수동 NNData 텐서를 넣을 때 RGB, `pixel/255`, NHWC FP16을 사용한다. 정규화가 DLC에 포함되지 않았다. RVC2 blob에는 정규화가 포함되어 있으므로 두 입력 처리를 구분한다. 아카이브의 이미지 preprocessing 필드는 `config.json`에 원본 그대로 보존했다.
- 현재 YOLOPv2 앱의 출력 디코더와 호환되는 모델이 아니다. A-YOLOM의 두 YOLOv8 DFL 검출 헤드 및 두 2채널 분할 헤드 처리가 필요하다.
- 컴파일 모델의 수치 동등성, 실제 카메라 추론 및 FPS는 미검증이다. ONNX와 PyTorch의 수치 비교 완료 사실과 구분한다.

재현 시 저장된 `conversion.yaml`을 입력으로 공식 이미지의 `convert rvc4 --path ... --output-dir ...`를 사용한다. 입력 모델 폴더는 읽기 전용 `/models`, 작업 출력은 쓰기 가능한 `/app/output`에 마운트했다. 중간 산출물과 컴파일 로그는 `artifacts/ayolom_conversion_20261125/rvc4_output/ayolom_v4_person_tl_672x384/`에 보존했다.

## 파일 정리 (2026-11-25)

사용자 요청으로 원본 모델 디렉토리 아래에 플랫폼별로 구분했다. 아래 본문의 생성 당시 경로 중 ONNX와 validation JSON은 `onnx_shared/`, RVC2 blob과 inspection JSON은 `oak_d_pro_poe_rvc2/`로 이동했다. 이동 전후 모델 바이너리 SHA256 일치를 확인했다. `v4_person_tl_final.pt`는 원래 위치에 보존했다. `oak_4d_pro_poe_rvc4/`에는 변환 미완료 상태를 나타내는 `STATUS.txt`만 있으며, RVC4 모델 파일은 아직 없다. Hub에서 수동 선택할 ONNX도 새 `onnx_shared/` 경로를 사용해야 한다.

## 요청 및 현재 상태

- 사용자 원본: `apps/yolopv2_standalone/models/ayolom_v4_person_tl_20261110/v4_person_tl_final.pt`
- 요청 입력: 가로 672 × 세로 384.
- 대상: OAK-D Pro PoE (RVC2), OAK-4D Pro PoE (RVC4).
- ONNX 생성 및 원본 대비 수치 검증: 완료.
- 사용자가 Luxonis Hub/BlobConverter 업로드 및 두 플랫폼 변환을 승인했다.
- RVC2 `.blob`: 생성 및 다운로드 완료. 장치 실행은 미검증.
- RVC4 DLC 포함 `.rvc4.tar.xz`: 미생성. Chrome 파일 업로드 자동화가 응답하지 않으며 현재 Chrome 창이 최소화되어 native 접근도 실패했다. Hub에는 RVC4, 384(H)×672(W), FP16, SNPE 2.41.0 설정을 준비했다.
- 카메라 앱 설정 변경 및 장치 배포: 수행하지 않음.

## 출처

- 원본 SHA256: `196b3228339bc1e5004fd87bded5fd039ba67225a08d5d6dab00c6fed291981c`.
- 공식 구현: <https://github.com/JiayuanWang-JW/YOLOv8-multi-task>.
- 구현 커밋: `9cf56e851339bd3ec2db7c9b278385ee1e98fee2`.
- 체크포인트 메모: `A-YOLOM v4 + person + traffic light (red/yellow/green)`.
- 실제 구조: `model.46` Detect(1), `model.47` Segment(1), `model.48` Segment(1), `model.61` Detect(4).
- `model.names` 앞 5개와 메모는 vehicle, person, tl_red, tl_yellow, tl_green를 나타낸다. 나머지 names 항목이 실제 검출 클래스라는 의미는 아니다. 클래스별 정량 정확도는 검증하지 않았다.

## 산출물 및 입력 계약

원본 폴더에 `ayolom_v4_person_tl_672x384.onnx`와 `ayolom_v4_person_tl_672x384.validation.json`을 생성했다.

- ONNX SHA256: `46dca08efcd1671121a7ead6708698009f516cb7688bdd663ad4ed6124a549d6`.
- ONNX opset 12, float32 RGB NCHW `[1,3,384,672]`, 값 범위 0~1 (`pixel/255`).
- 비율 보존 letterbox 패딩 값 114. 고정 입력으로 내보냈으며 다른 크기의 입력은 지원하지 않는다.
- 재현 도구: `src/hl_camera_bringup/scripts/export_ayolom_onnx.py`.
- 환경: WSL Ubuntu 22.04, 작업 공간의 `artifacts/model_conversion_env`, PyTorch 2.5.1+cpu, torchvision 0.20.1+cpu, ONNX 1.17.0, ONNX Runtime 1.20.1.

```bash
artifacts/model_conversion_env/bin/python src/hl_camera_bringup/scripts/export_ayolom_onnx.py \
  --source artifacts/ayolom_conversion_20261125/source \
  --weights apps/yolopv2_standalone/models/ayolom_v4_person_tl_20261110/v4_person_tl_final.pt \
  --output apps/yolopv2_standalone/models/ayolom_v4_person_tl_20261110/ayolom_v4_person_tl_672x384.onnx \
  --image artifacts/frame_597_source.jpg
```

## 출력 계약

두 검출 헤드와 두 분할 헤드를 모두 보존했다. 검출은 DFL/grid decode 및 NMS 전 원시 출력으로 내보냈다.

| 출력 | NCHW shape | 의미 |
| --- | --- | --- |
| det_stride_8 | `[1,65,48,84]` | 기본 검출, stride 8 |
| det_stride_16 | `[1,65,24,42]` | 기본 검출, stride 16 |
| det_stride_32 | `[1,65,12,21]` | 기본 검출, stride 32 |
| drivable_area | `[1,2,384,672]` | 주행 영역과 배경의 점수 |
| lane_line | `[1,2,384,672]` | 차선과 배경의 점수 |
| person_tl_stride_8 | `[1,68,48,84]` | 추가 검출 4클래스, stride 8 |
| person_tl_stride_16 | `[1,68,24,42]` | 추가 검출 4클래스, stride 16 |
| person_tl_stride_32 | `[1,68,12,21]` | 추가 검출 4클래스, stride 32 |

검출 채널은 reg_max=16인 64개 회귀 분포와 클래스 logits이다. YOLOv8 방식의 DFL softmax, anchor grid, stride 및 sigmoid 클래스 점수, NMS가 필요하다. YOLOPv2 anchor/objectness 디코더를 재사용하면 안 된다. 분할은 공식 추론과 동일하게 logits를 bilinear 보간한 뒤 채널 argmax를 적용한다. 두 분할 모두 배경 포함 2채널이므로 기존 YOLOPv2의 차선 1채널 threshold 처리와 다르다.

## 검증 결과

- ONNX checker 통과.
- 실제 이미지 `artifacts/frame_597_source.jpg` 및 seed 27 임의 입력 각각에 대해 8개 출력, 총 16개 비교 통과.
- 허용 오차 `atol=2e-4`, `rtol=1e-4`; 전체 최대 절대오차 `6.4849853515625e-05`.
- Adaptive Concatenate의 학습된 weight에 따른 분기는 고정 그래프에 반영된다. 실제 이미지와 임의 입력으로 원본 출력을 비교했다.
- 컴파일러 FP16 오차, 장치 추론, FPS, 실제 영상 정확도는 미검증.

## 다음 단계

RVC2 변환에는 `blobconverter==1.4.3`, OpenVINO 2022.1, FP16, 6 SHAVEs를 사용했다. `optimizer_params=['--scale_values=[255,255,255]']`, `compile_params=['-ip U8']`로 정규화를 컴파일 그래프에 포함했다. 따라서 blob에는 RGB U8 planar 0~255 이미지를 공급하며, 호스트에서 다시 255로 나누면 안 된다.

- 파일: 원본 폴더의 `ayolom_v4_person_tl_672x384_openvino_2022.1_6shave.blob`.
- 크기: 10,972,660 bytes.
- SHA256: `f6a39a651194090f03cf45a3788bbc50a79e48fa281cb042292565179ff52718`.
- DepthAI 2.30.0.0의 `OpenVINO.Blob` 파서로 NCHW U8 입력 `[1,3,384,672]`, FP16 출력 8개, 6 SHAVEs/6 slices, OpenVINO 2022.1을 확인했다. 모든 출력 이름·shape가 검증된 ONNX와 일치한다. 동일 이름의 `.inspection.json`에 기록했다.
- 6 SHAVEs는 변환 설정이며 장치 stereo와 병행 실행 시 자원 배치 및 처리 속도는 별도 검증이 필요하다.

RVC4는 같은 ONNX에서 기존 앱 환경과 맞는 SNPE 2.41.0 FP16으로 컴파일한다. 변환 시 자동 YOLO 검출 파서를 적용하지 않고 8개 원시 출력을 보존한다. 산출물 내부의 입력 색상·dtype·정규화·출력 layout을 검사하고 실제 장치 실행 여부와 구분하여 기록한다. Chrome 파일 선택 수동 단계가 필요하다.

공식 변환 문서: <https://docs.luxonis.com/software-v3/ai-inference/conversion/>. RVC4 SNPE 버전은 장치 런타임과 일치해야 한다. 두 플랫폼의 컴파일 산출물은 서로 교환할 수 없다.
