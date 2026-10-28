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
