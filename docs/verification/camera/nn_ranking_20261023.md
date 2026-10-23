# Limited NN candidate ranking — 2026-10-23

## Change and side-effect controls

Added `BoundaryConfig.image_nn_rank_weight` (default 0.06, range 0..0.1; zero disables). NN overlap is used only to break ties among geometrically accepted regional/full candidates. Regional ranking receives at most 1.8 point-equivalent units; full ranking at most 0.06. At least six observed points spanning ten rows and >=50% NN overlap are required. Equal weighting per row prevents mask thickness from inflating support. No extrapolated coordinates are evaluated.

NN does not change reported confidence, paint extraction, brightness thresholds, pole/object filtering, minimum length, gap/tangent checks, pair width validation, history or rendering. The binary NN mask is replaced each update, including empty masks. This does not make a detected curb distinguishable from a lane when both satisfy geometry; that remains a known limitation.

## Verification

- Existing 71 tests pass (30.889 s); three new NN ranking tests pass.
- Synthetic equal parallel candidates: NN-supported candidate wins without increased confidence.
- NN alone cannot create a boundary from no paint, a horizontal line, or a too-short fragment.
- Empty NN input clears prior NN support; invalid weights rejected.
- Cold-start A/B replay of 10 saved frames, weight 0 vs 0.06: valid pairs 10/10 both, maximum common-coordinate difference 0 pixels. Thus no accuracy improvement demonstrated on this scene. Host tracker timing ~34 ms is diagnostic only, not camera FPS.

## Deployment and actual camera results

Uploaded and hash-verified the two Python modules into the existing cached camera app, then restarted backend. This is a running-container source deployment, not a newly built persistent app image; image version metadata remains unchanged. Existing deployment helper was used.

- image_boundaries.py SHA256: b9a856bdaac7f6b8bf684036e74c463ad4152ea032fc586cfa912dc7c0350c31
- lane_boundaries.py SHA256: 2792fb447d0f28ff7e460f441708807a6fda7ed1a0413b776e1c37b43394093e
- `artifacts/nn_rank_0330_fps`: 301 frames, 29.994 s, 10.0019 processed FPS, sequence gaps 0, invalid pairs 0, centerline valid 0.
- Left PARTIAL 301/301; right OBSERVED 301/301.
- RGB/result/depth/BEV streams all received and saved in `artifacts/nn_rank_0330_streams`. Result image inspected: near-left white segment and right line are drawn; left upper extent remains incomplete.

No claim of fixing missing upper-left connections. Static-scene sampling cannot verify curve transitions, occlusion/reappearance or all curb false positives. No additional NN inference is introduced. The remaining short-segment selection/fitting issue needs a separate measured change rather than increasing NN weight or globally relaxing thresholds.

## Reproduction

Use `replay_nn_ranking.py artifacts/lane_mask_audit_0330` for saved-frame A/B. Run unittest discovery under apps/yolopv2_standalone. Use `measure_boundary_fps.py --device-ip 192.168.201.2 --seconds 30 --output ...` and `capture_boundary_streams.py` for device telemetry and multi-view snapshots.
