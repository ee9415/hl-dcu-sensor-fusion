# Straight stripe representative-point fix — 2026-10-23

## Cause and change

The full detector previously selected the individual white pixel closest to its prediction. Alternating sides of a thick straight marking produced a spurious quadratic C2 around -0.00648 in the captured scene. Full detection now uses centers of complete contiguous runs, before applying the search gate. Runs wider than 12 pixels are excluded to avoid treating transverse paint as a lane center. Run endpoints are extracted with vectorized NumPy operations.

A linear model replaces the quadratic only on the same robust inliers when linear MAE <=0.8px, maximum error <=2px and quadratic MAE improvement <=0.25px. Genuine stronger curvature keeps its quadratic. Existing temporal fusion smooths updates; no extra mode hysteresis/state was introduced. Numerical rank ties within 1e-9 keep deterministic seed order. This preserves NN tie-breaking without changing confidence.

Scope is the full detector. Regional and short-fragment tracking, actual-support rendering, pair checks, history recovery and centerline-disabled behavior remain intact. Wide merged paint can now be skipped; the existing gap limit, regional candidates and history remain fallbacks. Regional paths can still produce small curvature and require separate evidence before broader changes.

## Regression and replay

- All 90 tests pass on the final vectorized code (WSL Python, 32.01s).
- New actual-camera mask regression: straight boundary with/without old curved prior.
- Synthetic regressions: real quadratic preserved; variable stripe widths plus a transverse mark do not bend a straight boundary.
- Existing right-upper association, NN ranking, history recovery, opposite-side assistance and pipeline tests pass.
- Ten stored left frames: seven produce C2 approximately zero; remaining C2 0.000601..0.000913, versus earlier approximately -0.00648. These are pixel-space coefficients, not physical road curvature.
- Ten stored right frames retain x(y100)=178.625..179.314 at the upper white marking.
- Local identical-mask benchmark, median full detector: left 3.01ms baseline /3.23ms current; right 2.94ms /2.83ms. This is not device FPS or an end-to-end benchmark.
- Tools: replay_left_straight_fix.py, replay_right_upper_fix.py, benchmark_stripe_detector.py. Fixtures contain masks only.

## Device verification

Initial implementation deployed into existing camera container with source hash verification and backend restart. Four stream snapshots were received (RGB, result, depth, BEV). RGB/result snapshots are sequential, not frame-synchronized. Inspected result shows substantially straighter yellow boundary, but compression artifacts remain and this does not establish pixel accuracy over moving footage.

Initial 30s measurement: 259 frames, 8.615 FPS, 39 source-sequence gaps, no invalid pairs, no centerline, 103 restored-pair frames. Left observed110/partial61/held80/aligning8; right observed182/partial1/held73/aligning3. Left mean |C2| 0.000150, max0.001761 across this changing input.

Repeat without capture: 233 frames, 7.767 FPS, 55 sequence gaps. Right held126/233. Therefore 10 FPS and consistently observed bilateral tracking were NOT established by these runs. Do not conflate valid retained pairs with successful fresh detection. Final vectorized deployment measurement is recorded below after completion.

Final vectorized deployment SHA256: `57256f2412a31ffa965be543bb10489e32e837feeeea7680e8e39b6479b1cfea` (camera verified; backend restarted).

- Final 30s run: 312 frames /29.8693s = **10.412 FPS**, source sequence gaps3; centerline0.
- Both observed111 frames; tracking113, held186, predicted7, unavailable6; invalid pairs6; restored-pair flag188 frames.
- This reaches the throughput target for this sample, but does NOT establish continuous detection or sustained 10 FPS across all scenes. Input differed across runs; the higher final throughput cannot be attributed solely to vectorization. Held/predicted states are not evidence of fresh white-line detection.
- Artifacts: `artifacts/left_straight_fix_final_fps/frames.jsonl`, `report.json`; earlier captures/measurements under `artifacts/left_straight_fix_*`.
- Final four streams received; inspected `artifacts/left_straight_fix_final_streams/result.jpg` and `bev.jpg`. Final sampled road scene shows both OBSERVED, a substantially straight yellow marking along visible white paint, with some upper-end bend still present. No claim that all remaining curvature or stop-line accuracy was resolved.

No new persistent app image built; deployment replaces sources in the existing execution container. ROS2 build/topic/TF checks are inapplicable to this standalone container change.
