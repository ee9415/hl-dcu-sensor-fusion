# False curvature on straight left marking — 2026-10-23

## Method

Captured ten synchronized updates after bounded diagnostic restart, exported exact camera modules and masks to artifacts/left_straight_audit, then restored normal node. audit_left_straight.py traces full-detector winning points, compares line/quadratic fits and computes representative centers of the exact contiguous paint runs containing those points. Coordinates are 320x180 image pixels. No production algorithm changed in this analysis.

## Root cause: point selection drifts across the paint stripe

_detect_full seeds using a run mean, but every following row chooses the individual candidate pixel nearest the extrapolated position via options[argmin(abs(options-predicted))]. It does not retain a consistent location within the stripe. On a finite-width stripe, this feeds the previous prediction back into point selection and switches edges.

Representative captured frame:

| y | Contiguous surviving stripe x range | Chosen x | Stripe midpoint | Offset |
| ---: | --- | ---: | ---: | ---: |
|164|49..57|49|53|-4|
|154|60..67|60|63.5|-3.5|
|148|67..73|70|70|0|
|140|76..82|82|79|+3|
|128|90..94|94|92|+2|
|104|115..122|115|118.5|-3.5|

The selected path moves from one side of the same stripe to the other, producing curvature unsupported by stripe centers. Upper fragments also change the representative edge after the gap.

## Quantitative ablation on identical stripe runs

For x(y)=C0+C1*y+C2*y^2:

| Samples | Straight fit MAE | Quadratic fit MAE | C2 |
| --- | ---: | ---: | ---: |
| Current selected pixels | 2.25850 | 0.73558 | -0.00647939 |
| Contiguous stripe midpoint | 0.33545 | 0.33346 | -0.0000658931 |

Midpoint line max residual is 1.3023px. Curvature magnitude drops about 99% on these same stripe runs. Thus the visible straight-marking hypothesis is supported by these pixels; the bend primarily comes from representative point selection.

Current selected-point line/quadratic differ by up to 4.6421px over the observed extent. At 3x display this is about 14px. This is model difference, not a fully annotated physical boundary error.

## Model and fusion contribution

The detector always fits degree 2 and accepts the resulting curve through a 3.5px inlier gate. It does not compare a straight hypothesis or penalize unnecessary curvature. Hence it can confidently fit biased pixels: current quality is 0.83484 even though representative positions are inconsistent within the stripe.

Representative measured C2=-0.00647938795; final fused C2=-0.00647964883. All ten measurements have the same selected points/C2; final C2 converges to the measured value. This is not motion noise creating curvature or a retained old curve resisting a straight current observation. Repeated static input reinforces the same biased observation.

Opposite curvature assistance only applies to PARTIAL virtual tails; this left side is OBSERVED. It is not the source here. The previous right-upper regional fix also does not trigger: left regional estimate has no independently supported upper band to challenge the full curve.

The current observation ends at y94 while retained output extends to y86; unobserved retained extent may display more of the curve, but curvature already exists over the observed points. Do not treat the upper continuation as current paint measurement.

## Correction proposal, not applied

1. Track stripe components/runs and use a consistent within-stripe representative (center or validated subpixel ridge) rather than nearest individual pixel. Reject broad transverse markings/merged components; do not blindly average all white pixels in a row.
2. Compare robust straight and quadratic fits to those representatives; prefer zero C2 when residual improvement is negligible and apply hysteresis to prevent framewise model switching. A line is the C2=0 case of the same polynomial representation.
3. Retain curved-lane tests so a valid road bend is not flattened; include variable stripe thickness, glare, occlusion, separate parallel paint and parking stripes.
4. Re-evaluate observed vs extrapolated support to avoid falsely bright extensions.

Artifacts: left_straight_report.json (all ten frames and per-row pixel groups), left_line_vs_curve.jpg (selected samples red, their line fit green and quadratic yellow). Diagnostic script and document only; camera wrapper removed and original algorithm restored.
