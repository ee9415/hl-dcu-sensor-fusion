# Bounded image-motion boundary verification — 2026-10-22

## Scope and changes

Camera 192.168.201.2; app 0.26.0. No centerline or fixed lower anchor/metric width.
Measured affine registration is applied directly. Inertial velocity is smoothed
only for fallback and damped when registration fails. Candidate association uses
innovation and 3-of-5 persistence; short support fits a line, longer support a
bounded quadratic. Unseen portions use endpoint tangents. Whole-curve blending
limits measurement correction to 8 pixels/frame at 320x180 without global refit.
Latent coordinates are bounded to a 32px offscreen margin.

Each side uses its own warped history first. The reliable opposite line is
excluded from candidate association. Missing history initializes from the other
side; history older than 1s with confidence below .1 may slowly recover from
opposite-side evidence and previous width, staying at low confidence.
OBSERVED means p90 displayed-to-candidate residual <=4px; ALIGNING confidence
is reduced by residual. Missing evidence decays confidence. This heuristic is
not a calibrated probability or actual annotated-white-line accuracy.
Current supported segments are highlighted; virtual estimates are muted solids.
Residual, observation age and processing timings are exported to telemetry/CSV.

## Verification

52 unittest cases passed, including legacy BEV/stop-line/integration coverage.
New tracker cases cover known affine translation, 30s missing paint with damped
fallback, valid motion that must not decay, 3-frame remote candidate confirmation,
8px correction limit, curved paint fitting, tangent extrapolation, opposite-side
independence, repeated affine expansion with recovery, and opposite-line rejection.
`git diff --check` passed. ROS build/launch/topic/TF tests do not apply to this
standalone DepthAI app change.

## Deployment iterations and limitations

The first device run exposed compounded affine expansion outside the image and
local-taper kinks. A bounded latent model and complete-curve blending fixed the
numerical runaway. An additional run exposed left/right identity confusion;
association exclusion and low-confidence opposite repair were then added.
Intermediate results in artifacts/deploy_0260_review, deploy_0260_final and
 deploy_0260_curve and deploy_0260_verified are NOT final-build acceptance evidence.

Multiview RGB/result/BEV/depth streams connected. Depth is black in this monitor
capture setup and does not validate metric road depth. Straight visible-paint
segments were visually aligned, but long occlusion can still drift and recovery
can take multiple frames. Camera motion is a local affine image approximation,
not IMU or vehicle dynamics. Pixel-perfect curved/occluded accuracy and false
OBSERVED rates against hand-labelled ground truth remain unverified.

When neither side is observed, extrapolated crossing/offscreen pairs fall back
to the last drawable pair as HELD with additional confidence reduction. This
keeps a virtual guide but does not assert its location matches unseen paint.

## Final-build device evidence

- Artifact directory: `artifacts/deploy_0260_acceptance` (report.json + frames.jsonl).
- 601 frames / 59.995s; processed 10.000800 FPS; source gaps 0.
- Invalid pairs 0; centerline output 0; observed-residual rule violations 0.
- Boundary stage mean 18.35ms, p95 26.44ms, maximum 48.75ms.
- Reacquisition transitions 67. Left states {'predicted': 234, 'aligning': 44, 'inferred': 68, 'held': 238, 'observed': 17}; right states {'observed': 118, 'predicted': 194, 'aligning': 41, 'held': 238, 'inferred': 10}.
- Device/local LF-normalized tracker SHA256: `6d706c9eb8ab579b8580cff1a73dd07dd8743a6d2b42b0dd98035010cb968591`.
- Final live multiview showed muted predicted/held boundaries through unmarked road,
  junction and crossing scenes. These are continuity checks, not accurate lane labels.
- Large ALIGNING residuals remain (left maximum 161px, right 266px at 320x180),
  so convergence after long occlusion is not considered fully satisfactory.
  Zero OBSERVED residual violations only tests the state rule against the selected
  candidate, not whether that candidate is the correct white boundary.
- Final-build curved-white-line accuracy and false-positive rate require annotated
  replay verification. Earlier visible-paint UI checks are not substituted for that.
- Camera app and multiview remain running. No camera time/auth/network changes made.
