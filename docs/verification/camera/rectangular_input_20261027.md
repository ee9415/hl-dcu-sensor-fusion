# Rectangular YOLOPv2 input verification — 2026-10-27

## Scope

The existing 320×320 archive was compared with a 320×192 RVC4 FP16 archive
converted from the same TorchScript model. Device PC input used the 60–76 s
road-video segment at a requested 10 fps in full processing mode. Reported
output rates use the verifier's telemetry publication clock; they are not
exposure-to-display measurements.

## Compatibility and regression

- Hub conversion succeeded for RVC4, SNPE 2.41.0, with NHWC FP16 input
  `[1,192,320,3]` and five outputs. The archive SHA-256 is
  `eb41e41492381f29210b6ae194941ed8f24fb68eabfe3583bfa8cfc97009a4b7`.
- The device's measured 320×320 preprocessing letterbox is black; RGB is
  normalized to [0,1]. This observation is from one PC input frame.
- Host unit tests: 128 passed. Device NumPy replay against HEAD matched exactly
  for 40 fixture frames and 160 synthetic frames. This replay did not include
  a fresh real-device capture.
- Device trial input shape was confirmed as `[1,3,192,320]`. The prior model
  remains on the device; source backup is `/app/deploy_backup_513f9d38e7`.

## Results and limits

- Three 20 s runs: original model 10.07, 9.86, 9.92 fps; rectangular model
  10.09, 9.96, 9.95 fps. These are short-window measurements near the input
  rate and do not establish an advantage or a strict 10 fps guarantee.
- Three-minute original-model PC replay measured 9.98 fps, with latency p50/p95
  212/375 ms. The rectangular replay measured 9.97 fps, 154/235 ms and
  minimum full-minute throughput 9.97 fps. Maximum observed CPU sensor
  temperature was 95.0 °C in the rectangular run; throttling state is unknown.
  The separate runs are indicative, not a controlled causal attribution.
- For 145 matching source-frame indices from one run per model, the median
  mean absolute boundary difference was 0.12 px left and 0.23 px right in
  320×180 coordinates; the corresponding 95th percentiles were 1.23 and 0.97
  px. Temporal histories may differ. This is model-to-model agreement, not
  ground-truth accuracy.
- The visual comparison shows right inferred boundaries away from visible
  white paint in some frames for both models. Rectangular input has not been
  shown to correct this pre-existing tracking issue.
- Live road-camera depth is unverified.
- Long-run monitor's source-index drop percentage must not be interpreted as
  packet loss for a 30 fps video intentionally sampled at 10 fps.
- After the trial, device sources were restored byte-for-byte from
  `/app/deploy_backup_513f9d38e7`; camera input and basic-mode telemetry were
  observed after restart. The 320×192 archive remains stored separately.

## 2026-10-28 device FP16 versus ONNX

Temporarily ran static inference using identical BGR input, with the actual
RGB conversion and FP16 division by 255 performed before comparing RVC4 FP16
and ONNX FP32 outputs. Sources were backed up and byte-verified on restoration;
the original 320×320 app was confirmed running after both road captures.
This diagnostic is not a performance measurement.

Two JPEG-encoded road frames at approximately 65 s and 73 s were tested. They
are samples of the existing video, not camera exposures. For the 65 s frame,
lane-mask intersection over union was 0.9809 (35 disagreeing pixels), road
argmax differed at 128 pixels, and both outputs produced 10 detections after
the existing NMS. For the 73 s frame, lane-mask IoU was 0.9628 (79 pixels),
road argmax differed at 65 pixels, and ONNX produced 17 detections versus 16
on the device. Matching depends on NMS and confidence thresholds; no ground
truth was used. These examples do not establish acceptable accuracy or isolate
the cause of numerical differences. Detection raw-head mean absolute differences
for the first road frame were 0.246, 0.332 and 0.101, respectively.

An additional artificial pattern had no positive lane pixels in either output
and no detections; lane IoU is undefined for this empty union. The model has
not passed a representative multi-scene or threshold-sensitive equivalence
gate. Results are in ignored `artifacts/rectangular_model_20261027/`.

The device telemetry from the prior 320×192 replay also helps narrow down the
right-boundary discrepancy: around 60 s the right side was inferred; at 61 s
it was `aligning`, with a regional observation on 11 rows, observation quality
0.478 and 90th-percentile candidate-to-prior difference 69.57 pixels in
320×180 coordinates. The reported residual after fusion was 61.95 pixels and
the resulting confidence about 0.000016. By 62 s the residual had fallen to
20.33 pixels; at 63 s the state was observed and residual 0.56 pixels. The
fusion implementation caps the maximum update at 80 pixels per second. These
data indicate a large mismatch and gradual alignment, but do not prove that
the regional candidate itself was the correct white line: its pixels were not
captured. Further frame-level evidence is needed before changing that cap or
the candidate selection criteria.

## 2026-10-28 candidate-pixel trace

The original 320×320 device model was instrumented temporarily and 25 PC
input frames from 60.01–62.41 s were recorded with the candidate arrays,
support, tracker history, paint mask and the device's RGB input. Instrumented
replay is intrusive and must not be used for FPS evaluation. The device source
was restored byte-for-byte and the original app restarted. The raw trace and
overlay are ignored artifacts under `device_candidate_trace/`.

At 60.01 s the right boundary was inferred. At 61.01 s a partial regional
candidate was supported on 11 rows with quality 0.478. Its 90th-percentile
separation from the prior was 73.79 pixels; after fusion the residual was
66.16 pixels and confidence approximately 0.000008. At image row y=160 in
the 320×180 tracker coordinates, candidate x=288.0, prior x=215.4, and
output x=222.9. In the captured RGB overlay, supported candidate points at
this time follow visible bright stripe fragments while the inferred boundary
remains toward the road interior. This visual assessment has not been checked
against labelled ground truth, and the fitted candidate also spans unsupported
rows. By 62.41 s the state had returned to observed. Some intermediate frames
had no candidate, prolonging alignment.

The existing fusion limits the largest correction to 80 pixels per second;
this and intermittent candidate support explain gradual convergence in this
trace. A global increase would also allow false candidates to move the boundary
faster. No tracker thresholds or device inference code were changed in this
diagnosis. Any selective reacquisition change needs repeated-candidate,
occlusion and false-positive regression tests and a fresh non-instrumented
performance measurement.

After restoring the original source, two uninstrumented 20-second PC replays
of the same segment reported 9.00 and 9.92 output FPS at a 10 FPS requested
input. Both functional checks passed, but neither establishes sustained
10 FPS and the cause of the first run's slower output was not determined.
One subsequent device-stat sample had no warnings; this does not exclude
earlier throttling or other transient causes.

## 2026-10-28 multi-scene baseline and offline recovery experiment

The 320×320 device model was measured on the same source video at 10 FPS
requested input, 15 seconds per segment: 240–256 s (straight), 300–316 s
(vehicles and crossing), and 360–376 s (curve). Output rates were 9.31,
10.00 and 9.87 FPS respectively. These are single short runs, not a sustained
throughput or general accuracy assessment. Six matching-source overlays are
saved under ignored `artifacts/rectangular_model_20261027/multiscene/`.
On the selected crossing frames, boundaries can depart from visible paint
even when telemetry says `observed`; observation state is not ground truth.

An offline ablation changed the maximum correction from 80 to 160 px/s
unconditionally. In a 50-frame FP32 host replay at 60–65 s, right boundary
frame-to-frame 90th-percentile displacement had a 95th percentile of 7.98 px
before versus 15.64 px with the ablation. The observed state first appeared at
62.1 versus 62.0 s. Thus faster movement can also create larger jumps.

A separate experimental condition allowed 160 px/s only when a low-confidence
side had at least two consecutive consistent candidates, sufficient support,
and a plausible pair. On the exact 25 recorded device-input frames replayed
offline with the original 320×320 masks, the right side first became observed
at 62.406 s with the existing logic and 62.206 s experimentally. The 95th
percentile of frame-to-frame 90th-percentile displacement increased from
7.90 to 13.18 px. A three-candidate version produced no earlier recovery on
the same capture. In a separate 50-frame FP32 host replay at 300–305 s the
two-candidate condition did not activate and the right output was unchanged.
That test does not establish safety against consistent false positives or
occlusions. The experiments are not deployed and FPS was not measured under
them; thresholds remain unchanged pending representative labelled clips,
negative cases, and device A/B performance measurements.

## 2026-10-28 provisional sparse paint references

Three provisional manual reference files in `labels/` identify visible paint
centres in 320×180 image coordinates. They are draft visual estimates made
from three isolated frames of one video, with approximately 4 px estimated
annotation uncertainty; they are not independently reviewed ground truth.
Do not interpolate reference points through occlusion or use this sample for
overall accuracy claims. For frames 7337 and 10940, references use the decoded
original frame; for frame 1830 they use captured device input. Device transport
can introduce small differences for the former frames.

`evaluate_boundary_references.py` matches frame index and media epoch and
measures horizontal distance only at marked rows; it reports missing curves
instead of treating them as zero error, and does not extrapolate. On existing
320×320 device telemetry, draft straight frame 7337 had 6 marked points with
median/max absolute error 1.62/2.52 px; a second curve frame 10940 had 6 points
with 0.83/2.34 px. Both values are within the estimated annotation uncertainty.
On the previously captured difficult frame 1830, the three right-side points
had absolute errors 44.71, 53.41 and 65.06 px. Four left-side points were also
recorded separately; aggregate error is not a substitute for side-by-side
evaluation.

On a host replay of the same device-captured frame sequence, the two-hit
experimental rule changed right-side errors from 44.71/53.41/65.06 px to
39.34/46.39/57.42 px for frame 1830. This isolated improvement remains much
larger than annotation uncertainty and does not validate the rule's safety or
device FPS. The left side was intentionally omitted from that host replay's
prediction export and must not be counted as a tracking failure. No tracker
modification was deployed. 131 unit tests passed after adding three tests for
the reference evaluator.

## 2026-10-28 provisional occlusion and crossing samples

`labels/draft_occlusion_9140_9282.json` adds three sparse frames from the
300–316 s device replay (9140, 9209 and 9282; approximately 304.70, 307.00
and 309.43 s). Ten points were placed by visual estimate on visible paint,
excluding occluded areas, crossing stripes and parking-line junctions. They
are unreviewed, with estimated 4 px annotation uncertainty. They use decoded
source video frames; JPEG/NV12 device input may differ slightly.

All ten points matched a recorded output point at the same row; median/max
horizontal error was 1.29/2.41 px, within the estimated annotation uncertainty.
At those particular instants, the reported left/right states were `held/held`,
`partial/partial`, and `observed/observed`. Three sparse snapshots do not
measure exact reappearance or reacquisition time, do not score the occluded
segments, and do not establish that the full drawn curves are correct.

For the 300–316 s telemetry only, consecutive PC output samples separated by
0.08–0.15 s had 154 comparable pairs at image row y=130. The 95th percentile
of absolute horizontal displacement was 0.99 px left and 1.66 px right; maxima
were 2.99 and 2.97 px respectively. These are image-space changes, include
camera/scene movement and cannot alone diagnose a tracker jump. They exclude
missing curves, pauses, and larger time gaps.

The existing five negative/regression tests for long occlusion, crossed
candidates, lamp-post rejection, narrow-pair rejection, and crosswalk/objects
in stop-line detection passed. They test the current code and fixtures, not
the experimental two-hit recovery rule. No tracker or device deployment was
changed; repeated false-positive clips and dense reviewed labels remain needed.

## 2026-10-28 sparse temporal reference and repeated false candidate

`labels/draft_temporal_right_1818_1872.json` contains ten provisional right
paint points in five device-input frames between 60.61 and 62.41 seconds.
Frames are approximately 0.4–0.5 seconds apart, not a densely annotated
sequence; points are visually estimated with local brightness as a guide,
unreviewed, and subject to estimated 4 px uncertainty. No points were placed
on vehicle-occluded or unclear segments. The tracker was instrumented during
capture; the data cannot be used to measure normal FPS or claim an exact
reappearance instant.

At 60.61 s, the three marked-row errors were 60.57/64.79/76.85 px; at 61.01 s
two were 44.71/53.41 px; at 61.51 s two were 32.46/38.20 px; at 62.01 s one
was 26.50 px; and at 62.41 s two were 2.94/2.50 px. Points represent different
visible rows in each frame, so this is not a continuous single-point trajectory.
On a host replay of the same captured inputs, the two-hit experiment improved
those five sample error lists to 60.57/64.79/76.85, 39.34/46.39, 27.3/32.1,
20.6, and 1.8/1.2 px respectively. This limited sample does not validate
the experimental rule or prove an exact recovery time.

The two-hit experimental tracker passed the five pre-existing occlusion,
crossing, pole, narrow-pair and stop-line tests; these do not cover every
false-positive case. A deliberately synthetic fusion-only scenario injected
the same wrong right candidate x=245 for three frames, while assuming the true
right boundary was x=200 and supplying a consistent left candidate x=80.
At the second frame the baseline was x=216 and the two-hit trial x=224; at
the third frame both moved to x=245 and reported observed after current-pair
reacquisition. `replay_repeated_false_candidate.py` reproduces this scenario.
It demonstrates that plausible repeated candidate consistency alone cannot
distinguish a real boundary from a persistent false detection; it does not
establish how frequently this occurs in real video. Existing and experimental
code require additional visual-evidence gating and real negative clips before
deployment. No device source or tracker setting was changed in this step.

## 2026-10-28 preliminary paint-support gate prototype

`prototype_paint_support_gate.py` evaluates a local-pixel support condition
only before the existing three-frame pair-history rebase. For each side it
checks at least eight supported in-frame rows spanning at least 20 pixels,
and requires 40% of sampled rows to have a paint-mask pixel within ±2 x and
±1 y pixels. The threshold is an uncalibrated experimental choice. Gradual
fusion is not gated. The experiment runs offline only and is not device code.

On replay of the 25 captured input frames, ten calls lacked sufficient support
on at least one side, but right-side results and the first observed frame
(62.406 s) matched the baseline. In the three-frame synthetic repeated-false
case without right paint, baseline right x moved 208→216→245 while the gate
limited the immediate pair rebase to 208→216→224. This still drifts toward the
wrong candidate via normal fusion. When the false candidate had matching
paint, both moved 208→216→245: pixel presence cannot determine whether a
painted line is a lane boundary. These are constructed examples, not measured
real-world rates. In the sampled real frame at 61.01 s the lane segmentation
also covers portions of the curb, so YOLOPv2 mask overlap alone is not a
semantic guarantee either. The 25-frame data and one frame overlay are not
enough to validate safety or general accuracy.

Three prototype tests, including the deliberately demonstrated painted-false
limitation, and the existing test suite passed (134 total). The prototype is
not enabled in the tracker, has not been deployed, and has not been benchmarked
for FPS on the device. Repeated painted parking/crosswalk/curb candidates,
occlusions, and several labelled real scenes remain required before choosing a
semantic/geometry policy or changing runtime behaviour.

## 2026-10-28 방향 기반 픽셀 지표 오프라인 시험

`prototype_directional_evidence.py`는 현재 후보 곡선의 ±4행에서 후보 방향을
따르는 표식 픽셀의 연속 비율과, 후보 양옆 ±4~8픽셀의 가로 방향 표식 비율을
기록하는 실험용 코드다. 두 값은 영상 좌표의 형태 지표이며 차선 의미를
판별하는 신뢰도나 추적기 조건이 아니다.

가느다란 종방향 및 가로방향 선을 사용한 단순 합성 사례에서 종방향 연속
비율은 각각 0.91과 0.00이었다. 그러나 차선과 기하학적으로 동일한 주차선은
같은 0.91이므로 이 지표만으로는 의미를 구분할 수 없다. 실제 장치 입력의
60.61~62.41초 기록 중 오른쪽 후보가 있었던 15프레임에서는 종방향 비율이
약 0.20~0.63으로 변했다. 일부 프레임에서 보이는 선이 짧거나 끊겨 있어
합성 사례의 높은 비율을 그대로 기준으로 삼으면 진짜 후보를 놓칠 수 있다.
이 후보들이 모두 정답이라는 뜻은 아니며 전체 영상에 대한 통계도 아니다.

지표와 합성 반례를 확인하는 4개 테스트를 추가했고 전체 138개가 통과했다.
오프라인 진단이므로 장치 추적기에 연결하지 않았고 FPS도 측정하지 않았다.
실제 주차선·횡단보도·차량 가림의 반복 사례에서 방향 지표와 검토된 위치
기준을 비교하고, 필요한 경우 주행 영역 경계 및 시간 연속성 등 별도 맥락을
조합해야 한다. 단순 방향 또는 색상만으로 차선을 확정하지 않는다.

## 2026-10-28 주행 영역·시간 관계 오프라인 시험

`prototype_road_context.py`는 후보의 관측 행마다 영상에서 도로 안쪽과
바깥쪽으로 4·8·16픽셀 떨어진 지점의 주행 영역 마스크 값을 비교한다.
이 값은 영상 좌표의 맥락 지표일 뿐 실제 차선의 정답이나 거부 조건이 아니다.
주행 영역 마스크는 주차 공간까지 포함할 수 있고 자체 분할 오차도 있다.

실제 장치 기록의 오른쪽 후보가 있던 15프레임에서 8픽셀 거리 기준,
안쪽은 주행 가능·바깥쪽은 불가능으로 나온 행의 비율 중앙값은 0.25였다.
후보 양쪽 모두 주행 가능으로 나온 행의 비율 중앙값은 0.75였다. 예를 들어
프레임 1830에서는 원래 후보와 수평으로 ±20픽셀 옮긴 가상 후보의 전이 비율이
모두 0이었다. 이동 후보는 알고리즘이 실제 검출한 오검출이 아니다. 원본 후보도
레이블로 확정된 정답이 아니며 이 수치는 전체 도로 성능을 뜻하지 않는다.

별도 320×192 ONNX FP32 호스트 재생에서 8픽셀 전이 비율의 프레임별 중앙값은
60~65초 주차 곡선의 후보 42프레임에서 0.31, 300~305초 횡단보도 구간의 후보
50프레임에서 0.40이었다. 같은 후보를 인위적으로 ±20픽셀 이동한 비교군의
중앙값은 모두 0이었다. 동일 영상과 적은 구간의 탐색 결과이고 일부 원래
후보도 전이 비율이 0이므로, 이 수치로 합격 기준을 정하거나 후보를 배제할
수 없다. 장치 FP16에서 별도 측정한 값과 호스트 FP32 값은 직접 동등 비교가
아니다.

장치 기록의 연속 후보 중 공통 관측 행이 5개 이상인 11쌍에서, 움직임 보상
없이 계산한 후보 간 수평 차이의 90백분위는 10쌍이 4픽셀 이하였다. 반복
오검출도 안정적으로 나타날 수 있으므로 시간 일관성만으로 정답을 확정하지
않는다. 주행 영역 지표의 좌우 방향·영역 내부·화면 밖 처리 등을 포함해 전체
142개 테스트가 통과했다. 본 시험은 오프라인 진단이며 추적기 배포 및 장치
FPS 측정은 하지 않았다.

## 2026-10-28 실제 횡단보도 구간 후보와 관측 행 시험

`capture_pc_boundary_candidates.py`에 시작·종료 시각과 출력 폴더 선택을
추가했다. PC 입력 303~319초 구간에서 장치의 303.03~305.43초 25프레임을
일시 계측했다. 계측 후 원래 장치 코드를 바이트 단위로 복구하고 앱이 실행
중인 것을 확인했다. 계측은 프레임 처리에 영향을 줄 수 있어 이 실행의 FPS를
정상 성능으로 사용하지 않는다. 첫 시도는 Windows 경로 변환 오류로 실패했고
자동 복구 후 경로 처리 수정 및 재수집에 성공했다.

장치 프레임 9123(304.13초)의 왼쪽 `_detect_full` 후보는 점수 0.852와 함께
곡선 좌표를 반환했으나 명시적인 관측 행 마스크는 반환하지 않았다. 곡선의
유효 41행을 표식 마스크의 근방 ±2 x, ±1 y 픽셀로 확인한 결과 19행은 표식
픽셀이 있었고 22행은 없었다. 이는 근방 검색의 추정치이며 원래 피팅에 실제
사용된 행 목록이나 확정된 정답은 아니다. 횡단보도 주변 후보가 틀렸다고 이
수치만으로 확정하지 않는다. `_fuse`는 관측 행 정보가 없는 후보에 대해 유효
곡선 행을 지원 행으로 취급하므로 피팅 구간·공백과 실제 관측을 구분하기
어렵다. 화면 비교는 무시되는 로컬 산출물 `device_crosswalk_trace/`에 있다.

분리된 오프라인 시험에서 `_detect_full`이 실제 피팅 점과 가까운 행의 지원
마스크를 반환하게 했다. 단순히 이 마스크를 기존 융합에 적용하면 전체 142개
검사 중 기존 2개가 실패했다: `test_real_left_short_fragment_replaces_duplicate_right_paint`
및 `test_wide_but_wrong_low_confidence_prior_can_reacquire`. 동일한 두 검사는
기존 코드에서 통과했다. 실험이 짧은 실제 선과 오래된 경계의 복구에 부작용을
줄 수 있으므로 추적기나 장치에는 적용하지 않았다.

동일한 장치 기록을 호스트에서 재생한 보조 비교에서 주차 곡선 25프레임 중
상태가 달라진 프레임은 5개, 횡단보도 25프레임에서는 3개였다. 오른쪽 출력의
프레임별 90백분위 위치 차이 최대는 각각 8.83/0.89픽셀이었다. 정답 검증이
아니며 장치 처리 속도도 측정하지 않았다. 다음 수정은 관측과 표시용 연장을
분리하되, 기존 단편 재검출과 폭·교차 보호가 그대로 작동하는지 별도로 시험해야
한다.

## 피팅 관측점 진단 분리 (2026-10-28)

기본 추적과 융합은 변경하지 않고 `_detect_full`에서 선택한 피팅의 입력점과 곡선 유효 행을 선택적으로 기록한다. `trace_fit_evidence`는 기본 False이며 수집 스크립트의 `--fit-evidence`로 일시 활성화할 수 있다. 이 기록은 해당 검출 경로의 피팅 근거이며 최종 경계의 정답이나 실제 융합 support 마스크를 뜻하지 않는다.

장치 횡단보도 구간 25프레임을 수집하고 임시 계측을 원복했다. 프레임 9123의 왼쪽은 피팅 입력점 13개와 곡선 유효 41행, 오른쪽은 18개와 28행이었다. 두 구간의 실제 장치 입력 50프레임 호스트 재생에서 진단 유무에 따른 경계와 추적 기록 차이는 없었다. 장치 NumPy의 기존 대비 진단 꺼짐 비교는 fixture 40프레임과 합성 160프레임이 동일했고 전체 회귀 검사 144개가 통과했다.

진단 기본 꺼짐으로 tracker 파일만 장치에 배포했다. 백업은 `/app/deploy_backup_60ba6581e3`이다. 도로 영상 60~76초 구간의 20초 단일 검사에서 전체 후처리 발행 10.02 FPS, 입력 시각 기준 9.97 FPS였고 카메라 입력이 복구되었다. 지속적인 10 FPS와 도로 장면의 실제 카메라·Depth 성능을 보장하는 측정은 아니다. 추적 알고리즘은 바뀌지 않아 경계 정확도 개선을 확인한 단계도 아니다.

## 반복 짧은 선 조각 보정 (2026-10-28)

전체 관측 후보가 없을 때 이전 경계와 위치·기울기가 맞는 선 조각이 3회 이상 반복되고 5행 이상, 8픽셀 이상에 걸쳐 있으면 해당 관측 행만 낮은 품질 0.45로 융합한다. 기존 시간적 보정 제한과 경계 쌍 검사는 그대로 적용한다. 관측 범위 밖을 현재 측정으로 취급하지 않는다. 같은 방향의 다른 표식이 반복되는 경우의 오검출 가능성은 남는다.

전체 146개 검사가 통과했으며 신규 검사는 반복 조건, 지원 범위, 불일치 방향과 흔들리는 조각의 거부를 확인한다. 저장된 장치 도로 영상 각 25프레임 재생에서 주차 곡선은 변경이 없고 횡단보도는 6프레임에서 새 보정이 발동했다. 왼쪽 표시 좌표 차이는 최대 약 0.20픽셀이다. 실제 정답 표시가 없는 비교이므로 정확도 개선이라고 단정할 수 없다. 장치 별도 수집 25프레임에서 8프레임에 보정 발동을 확인하고 임시 계측을 원복했다.

장치 배포 백업은 `/app/deploy_backup_08a9a5eeae`. 횡단보도 303~319초 반복의 60초 전체 후처리 측정은 발행 9.971 FPS, 입력 시각 기준 9.966 FPS, PC 프레임 611개, 시퀀스 불일치 0, 중심선 0, 카메라 입력 복구 성공이다. 장시간 10 FPS 이상 목표는 확인되지 않았으며 실제 도로 카메라와 Depth, 정답 대비 정확도도 미검증이다. 원자료는 `artifacts/rectangular_model_20261027/multiscene/fragment_device/`와 `fragment_device_trace/`에 저장했다.

## 도로 장면별 여유와 지연 (2026-10-28)

횡단보도 프레임 9135, 9144, 9150의 원본과 보정 전후를 나란히 비교했다. 표시 경계는 거의 겹쳐 시각적으로 뚜렷한 개선을 확인하지 못했다. 정답 좌표가 없으므로 정확도를 수치화하지 않았다. 비교 이미지는 `artifacts/rectangular_model_20261027/multiscene/fragment_comparison.jpg`이다.

동일 320x320 모델과 전체 후처리에서 원본 29.997 FPS 도로 영상의 구간을 반복 전송했다. 지연은 장치 입력 추정 시각부터 텔레메트리 발행까지이며 PC 디코딩과 화면 출력은 포함하지 않는다. 처음/마지막 지연은 각 100프레임의 95백분위다.

| 영상 구간 | 요청 입력 | 시간 | 결과 발행 FPS | 지연 p50/p95 ms | 처음/마지막 p95 ms |
|---|---:|---:|---:|---:|---:|
| 303~319초 | 10 | 60초 | 9.971 | 142/194 | 171/177 |
| 303~319초 | 10.5 | 90초 | 10.474 | 152/264 | 191/192 |
| 303~319초 | 12 | 60초 | 11.471 | 808/1354 | 256/1052 |
| 60~76초 | 10.5 | 60초 | 10.084 | 1147/1478 | 739/1546 |

각 조건은 1회 측정이며 프레임·NN 시퀀스 불일치와 중심선 출력은 0, 종료 후 카메라 입력 복구를 확인했다. 10.5 FPS에서 장면에 따라 지연이 누적되어 일반 기본값 변경의 근거가 부족하다. 송신 ACK/소비 카운터는 이전 세션 누적값이 섞여 단일 실행의 누락률로 사용하지 않았다.

기존 계측 스크립트를 현재 전체 후처리 분기에 맞추고 구간·FPS 인자를 추가했다. 첫 계측은 이전 코드의 삽입 위치가 없어 실패했고 자동 복구됐다. 수정 후 60~76초, 10.5 FPS 조건 30초 임시 계측에서 전체 후처리 평균/p95 106.0/144.8ms, 경계 추적 48.2/72.5ms, 결과 생성·전송 23.0/33.3ms, BEV 12.5/19.9ms, 정지선 8.0/13.3ms였다. 항목은 전체 후처리에 포함되며 더하면 안 된다. 계측 자체의 부하가 있으므로 계측 FPS는 일반 성능이 아니다. 장치 계측 코드는 원복했고 manifest와 배포 경계 파일 해시를 확인했다. 다음 최적화는 정확도 동등성을 유지하는 경계 계산 또는 결과 영상 생성의 비용 절감 후보를 먼저 실험해야 한다.

## 입력 해상도 결정용 다중 장면 비교 (2026-10-28)

모델 입력 크기와 카메라·전송 영상 해상도는 별개다. 16:9 영상에 검정 레터박스를 적용하면 320x320과 320x192 모델의 유효 영상은 모두 320x180이며, 416x256의 유효 영상은 416x234다. 이번 비교는 같은 원본 영상의 60, 62, 244, 248, 300, 304, 308, 312, 360, 364, 368, 372초 프레임 12개에서 수행했다. 원본 영상 한 편이므로 독립적인 날씨·카메라 조건은 아니다.

320x320과 320x192의 ONNX FP32 비교에서 320x180 좌표 차선 마스크 IoU는 중앙값 0.947, 범위 0.902~0.965였다. 원본 TorchScript FP32의 320x320과 416x256 비교에서는 416의 유효 출력 확률을 320x180으로 선형 보간한 후 임계값 0.5를 적용했으며 IoU 중앙값 0.811, 범위 0.748~0.835였다. 두 비교의 320x320 차선 양성 픽셀 수는 12프레임 모두 일치했다. IoU는 모델 간 일치도이지 정답 대비 정확도가 아니다.

416x256은 일부 영역의 표식 폭이 더 좁게 나타났지만 두 해상도 모두 도로 가장자리나 횡단보도 표식에도 차선 마스크를 출력한다. 동일 기존 NMS 후 프레임당 객체 수는 320x320이 2~9개, 416x256이 2~10개였고 일부 프레임에서 차이가 있었다. 객체 수는 정답을 뜻하지 않는다. 모델별 마스크·객체의 정답 라벨과 시간 연속 경계 추적 비교, 416x256의 ONNX/RVC4 변환과 장치 처리 속도는 아직 없다.

현 시점에는 실장·속도 검증 이력이 있는 320x320을 기준으로 유지한다. 320x192는 동일 유효 폭에서 장치 FPS 우위를 증명하지 못했고, 416x256은 장면별 정확도 개선과 10 FPS 이상을 입증하지 못해 최종 채택하지 않는다. 비교 도구는 `src/hl_camera_bringup/scripts/compare_resolution_scenes.py`, `compare_torch_resolution.py`; 출력은 무시되는 `artifacts/rectangular_model_20261027/multiscene/resolution_scene_compare/` 및 `torch_416_scene_compare/`에 있다. 추가 장치 배포는 하지 않았다.
