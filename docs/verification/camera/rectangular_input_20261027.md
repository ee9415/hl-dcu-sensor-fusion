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
- FP16 device tensor values have not yet been compared directly with the
  corresponding ONNX outputs; live road-camera depth is also unverified.
- Long-run monitor's source-index drop percentage must not be interpreted as
  packet loss for a 30 fps video intentionally sampled at 10 fps.
- After the trial, device sources were restored byte-for-byte from
  `/app/deploy_backup_513f9d38e7`; camera input and basic-mode telemetry were
  observed after restart. The 320×192 archive remains stored separately.
