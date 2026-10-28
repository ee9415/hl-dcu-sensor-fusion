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
