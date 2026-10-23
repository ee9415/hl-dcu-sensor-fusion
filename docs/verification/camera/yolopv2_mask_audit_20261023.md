# YOLOPv2 lane-mask audit — 2026-10-23

## Method

Captured 10 synchronized tracker updates on camera 192.168.201.2 with the deployed 0.33.0 source. Export: `artifacts/lane_mask_audit_0330/boundary_debug.npz`. Removed the diagnostic wrapper and restored the original node after capture. Capture restarts the backend; this is not uninterrupted historical tracking.

`compare_nn_lane.py` compares the exact exported camera implementation's RGB-only evidence, NN-assisted evidence, filtered paint and final telemetry curves. NN input is the binary mask after >=0.5 threshold and resizing to 320x180, not raw logits. Counts use the lower 60% (y>=72). The image shows full-frame evidence for context. Final curves in the comparison are single-width diagnostic lines, not the production support/thickness rendering.

## Results

Representative frame 0:

| Stage | Pixels |
| --- | ---: |
| NN binary lane | 2092 |
| RGB evidence | 2148 |
| Combined evidence | 2487 |
| Additional evidence attributable to NN | 339 |
| Combined evidence passing brightness alone | 742 |
| Final paint after all filters | 287 |
| NN-added evidence surviving all filters | 54 |

Across 10 frames, NN adds 339–368 evidence pixels before filtering; 50–57 survive all filters. Brightness threshold is 200.33–200.87/255. These are candidate counts, not precision/recall or confirmed lane pixels. NN-mask thickness differs from ridge thickness; a raw NN-to-paint retention ratio is not a missed-lane rate.

Visually, NN identifies the near left white marking and the right white marking, but also curb/road-edge and transverse parking markings. Directly drawing all NN output would introduce incorrect boundaries.

In the manually inspected near-left window x=97..119, y=148..164, NN has 108–110 pixels, RGB evidence 75–77, combined evidence 77–79 and final paint 60–62. No NN-exclusive additions survive in that window. This window is a diagnostic region, not a ground-truth annotation. The visible short left marking survives candidate generation: any failure to follow this segment also requires analysis of candidate selection and curve fitting, not only segmentation or brightness filtering.

## Decision and limits

Retain NN evidence; it already contributes to the pipeline. Do not replace current detection with raw NN-mask contours. A useful next controlled experiment is to use NN overlap as a candidate ranking/support signal, retaining RGB geometry, road/object rejection and pair plausibility. Brightness relaxation must be evaluated separately against curb, parking-line and pole false positives.

No tracker thresholds or boundary algorithms changed in this audit. No new FPS claim: capture profiling adds overhead and the 10-frame static sample cannot validate motion, occlusion recovery or general accuracy. Dynamic A/B verification is still required before deploying a different fusion strategy.

## Reproduction

Run `capture_left_debug.py install artifacts/lane_mask_audit_0330`, wait for bounded capture, export, then restore. Run `compare_nn_lane.py artifacts/lane_mask_audit_0330` in Python with NumPy/OpenCV. Outputs: comparison.jpg, comparison.json and masks.npz. Capture helper now removes stale capture files and accepts a destination argument.
