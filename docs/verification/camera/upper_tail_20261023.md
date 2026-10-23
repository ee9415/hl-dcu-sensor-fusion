# Left upper continuation — 2026-10-23

## Variable-level cause

Captured ten updates in artifacts/left_upper_audit, restoring the normal node afterwards. candidate_report.json records candidate extents and fusion inputs/outputs.

- Left regional bands: [0,4,4]; no upper-band segment survived. The chosen regional curve was x183.98,y152 to x150.75,y128, quality 0.5733: the right marking was also admitted by the left search window.
- Left full/legacy detectors returned None. Temporal merging produced a mixed candidate, but duplicate-pair validation correctly rejected it.
- The actual near-left fragment had nine samples spanning y148..164 (x~116.09..101.69 after fit). Three-frame duplicate recovery selected this fragment with quality 0.5.
- Fusion prior and measurement both had exactly those nine rows; output remained y148..164. No reliable upper-left observation reached fusion. Extending the search ROI or disabling duplicate rejection would not fix this capture and could reuse the right marking for both sides.

## Change

After validated tracking history and width learning are committed, create a bounded display continuation for a PARTIAL side recovered through >=3 repeated short-fragment confirmations, only while the opposite is OBSERVED. Require >=8 support rows and >=14-pixel span. Preserve the same quadratic and observed positions. Maximum upward extension: 24 pixels at 320x180. Reject slope >1.5, deviation from the local tangent >4 pixels, image overflow, or an implausible pair. Stop at the first failed row.

Virtual rows do not feed history, width learning, support flags or confidence. Existing rendering displays them dimly while observed portions remain bright. This deliberately fixes missing virtual continuation, not unobserved white-line detection. The extrapolated direction can still differ from the occluded true marking; no ground-truth accuracy claim is made.

## Verification and deployment

- Existing 74 tests pass, plus three new tests for bounded continuation, unchanged inputs/support, opposite/confirmation requirements and steep-line rejection.
- Camera running-container source deployment and hash verification succeeded; no new persistent app image built.
- image_boundaries.py SHA256: 347b515a686d4e4ce69304f9b215d75e89240a7d9b031000c0c47112c6d271a6.
- artifacts/upper_tail_fps: 301 frames, 29.998 s, 10.00066 FPS, sequence gaps 0, invalid pairs 0, centerline valid 0.
- All 301 frames: left PARTIAL/right OBSERVED. Left display contains 20–21 rows, with 11–12 virtual rows instead of only nine observed rows. Representative extent expands y148..164 to y124..164; pair gap limits may stop at y126.
- artifacts/upper_tail_streams/result.jpg visually confirms the dark extension joins the bright left segment. This is a stationary scene; dynamic occlusion/reappearance and curvature accuracy remain unverified.

## Diagnostics tooling

boundary_debug_capture now captures regional segments, detector diagnostics and fusion support variables. inspect_upper_candidates.py exports selected candidate and final extent summaries without changing tracker decisions.
