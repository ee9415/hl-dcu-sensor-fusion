# Opposite curvature guidance — 2026-10-23

## Implementation and limits

`opposite_curve_weight` defaults to 0.25, accepts 0..0.5, and zero disables guidance. This only adjusts the bounded virtual tail of a confirmed PARTIAL side; all observed coordinates, support flags, confidence, tracking history and learned width remain unchanged.

Require opposite OBSERVED, confidence >=0.6, >=12 support rows, >=24-pixel support span, five consecutive reliable frames, and confidence advantage >=0.08. The opposite's local tangent is removed before transferring curvature residual: its slope and position are not copied. The remaining quadratic residual is blended into the weak tail, preserving position and tangent at the join. This is an image-space prior under a local perspective approximation, not calibrated physical road curvature. Hills or camera tilt may violate it.

Endpoint correction is capped at two pixels at 24-pixel extension, smoothed with a 0.5-second time constant and limited to two pixels/second. Existing slope, tangent-deviation, image bounds and pair-spacing checks still apply. On loss of the confidence advantage, target influence decays. When the side ceases to be PARTIAL, guidance is cleared. Source/time resets clear both reference counts and guidance.

Observed and virtual portions each remain quadratic with a continuous position/tangent join; the complete boundary need not be one exact global quadratic. Existing telemetry polynomial coefficients are a global least-squares summary of the displayed points. Consumers needing exact geometry must use boundary point arrays. No measured segment is bent merely to force global coefficient equality.

## Validation

- Existing 77 tests pass; three additional tests cover virtual-only influence after confirmation, rejection of linear perspective slope transfer/weak references, and reset clearing. Six virtual-tail tests total pass.
- Camera source deployment into existing running container, hash verified and backend restarted. No new app image built.
- image_boundaries.py SHA256: 9edeb733f3f12317663fd2661e96d8967b94b0da6f61cfa949fb4e1c6f20f128
- lane_boundaries.py SHA256: 64b09514380a82f297d7e5e3990581a81eb6b43f713805f01b896397fd0d8ff4
- artifacts/opposite_curve_fps: 301 frames / 30.0187 seconds = 9.99376 FPS; sequence gaps 0, invalid pairs 0, centerline valid 0.
- All measured frames: left PARTIAL, right OBSERVED. Applied tail correction at the 24-pixel endpoint equivalent: -0.880..-1.001 pixels at processing resolution 320x180. Direction agrees with the observed right curvature residual; this is a small change, not proof of true hidden-lane accuracy.
- artifacts/opposite_curve_streams/result.jpg visually inspected: observed left and right lines retained and virtual left tail connected. Static scene only; dynamic turns/occlusion/reappearance and camera pitch effects remain unverified.

## Rollback

Set opposite_curve_weight=0 to disable this guidance. Existing bounded virtual continuation remains available independently.
