# Right upper marking pixel/code audit — 2026-10-23

## Scope and collection

User provided a scene with a visible right white marking above a transverse road marking/speed-hump area. Before diagnostics, preserved 31 live telemetry frames: right OBSERVED26/ALIGNING5, no invalid pairs. Then captured 10 synchronized tracker updates in artifacts/right_upper_pixels and restored the normal node. Diagnostic restarts reset history; the offline replay uses the exported exact source and captured prior for its frame. No lane algorithm changed in this task.

All following coordinates refer to the 320x180 processing image, not screenshot coordinates. Binary NN mask is thresholded/resized input, not model logits.

## Pixel survival

Representative threshold 184.6. Visually identified upper white samples:

| x | y | Channel mean | NN lane | Final paint |
| ---: | ---: | ---: | --- | --- |
|179|100|235.00|true|true|
|182|102|235.00|true|true|
|185|104|234.00|true|true|
|188|106|239.67|true|true|
|190|108|224.67|true|true|

Thus those white pixels survive NN, brightness, object/road/pole filtering. They lie inside the lower-60% ROI. This is not an ROI cut or AI omission for these pixels.

## Wrong path in _detect_full

Offline Python line tracing on captured source reproduces selected seed start=168, x=267. Its lower path follows paint through (219,128). Empty rows then permit jumps:

| Step | Prediction | Gate | Selected point |
| --- | ---: | ---: | --- |
| y116 after y128 | x207 | 8.0 | (214,116) |
| y114 | x212.875 | 5.5 | (213,114) |
| y96 after y114 | x204 | 9.5 | (203,96) |

These upper points are away from the visible lane continuation. The full detector continues across missing rows with gate=min(16,5+0.25*gap), without the legacy fallback's explicit long-gap direction check or the regional detector's six-row stop. The local slope changes after attaching the off-line pixels.

At y100, prediction becomes x206 with gate8.5. Actual surviving paint x178..180 is outside this search window, so options is empty. At y106 prediction x209/gate7 similarly excludes true paint x187..189. The detector has not lost the pixels globally; it excludes them locally because the path has drifted.

All four final points (219,128), (214,116), (213,114), (203,96) are retained by residual<3.5 in the quadratic fit. A flexible quadratic fits the incorrect point chain, so residual alone does not expose the wrong association. Normalized-u coefficients (u=(y-125)/50): [217.6133,39.2196,28.6179]. Positive curvature makes the upper curve turn toward the road edge.

## Correct regional result loses final selection

Regional detector joins near fragment y128..152 and upper fragment y100..106. Representative regional x(y100)=179.782, quality0.5773. Full selected x(y100)=205.158, quality0.8582. _detect chooses full because regional quality is lower; the extends branch also requires agreement within 3px, so cannot replace this disagreeing full curve.

All ten frames show regional x(y100)=179.782..180.242, but selected full x(y100)=195.264..205.158. The systematic displacement relative to the regional white-line estimate is about 15..25 processing pixels. Regional/full scores use different span/inlier/residual formulas, not a calibrated shared confidence measure.

Full candidates, representative frame:

| Start row | Quality | Final rank | Temporal support | NN overlap |
| ---: | ---: | ---: | ---: | ---: |
|168|0.858174|1.056930|0.985829|0.869565|
|156|0.860578|1.043687|0.868229|0.904762|
|144|0.813767|0.979878|0.801107|0.933333|

Ranking uses quality +0.10*temporal +0.08*min(1,len(xs)/40) +NN weight*overlap. Longer chains gain rank even if part of the chain is on different paint. Span and inlier terms reward the incorrect association; lower-region NN overlap remains high despite wrong upper pixels.

## Ablation and fusion

Run the exact full detector with history on/off and NN rank weight .06/0: all four combinations still select x(y100)=205.158. Thus neither added NN ranking nor history is necessary for the failure. History can reinforce it, but is not the primary cause here.

Captured raw selected curve upper endpoint is x204.182 at y90; motion-predicted prior x201.598; fused output x203.113. The deformation exists before fusion, which blends it rather than originating it. Pair plausibility checks spacing/crossing, not white-pixel alignment, so a wide smooth incorrect right curve can pass.

## Recommended correction (not applied)

1. Direction/fragment checks for full-detector gap bridging, rather than relaxing paint brightness or widening the ROI.
2. Score per-band actual paint alignment and penalize unsupported/gap-driven span; compare regional/full candidates with common evidence metrics.
3. When independent upper regional paint contradicts the full model, fit/reject using those observations instead of automatically preferring the full scalar score. Protect against parking stripes, curb edges and short noisy upper fragments with tangent, repeatability and pair checks.
4. Preserve this captured frame as a regression fixture; test full-path selection without history as well as warm history. Do not fix by a stronger opposite-curve prior.

Artifacts: comparison.jpg (six stages), right_pixel_trace.jpg (cyan selected full, green regional, red inlier points), right_pixel_trace.json (pixel values, winning search steps, ten-frame comparison and ablations). trace_right_pixels.py reproduces the audit offline.
