# Moving boundary recovery and throughput — 2026-10-23

## Evidence and root cause

User reported tracking failure on entry to the next road section, followed by recovery after pausing the video. Screenshots show BOTH OBSERVED -> BOTH HELD / REF NONE -> BOTH OBSERVED. Screenshots alone do not identify exact failing variables.

Collected a preliminary 100-frame snapshot, then a second 100-frame sequence after the user confirmed starting continuous repeat playback. The second sequence spans 18.4001 source seconds (debug capture adds overhead; this is NOT a 10 FPS measurement). Captured RGB/masks, candidates, prior, fusion variables and telemetry. Used identical sequential inputs with cold-start baseline and changed trackers. This controls input differences but does not reproduce every pre-capture internal state.

Confirmed problems:

1. Reacquisition compared raw frame coordinates with 3px/4px limits. Real motion could reset the three-observation confirmation even when the marking was consistently detected.
2. Valid measured flow was temporally smoothed before geometric prediction. This mixes geometric compensation with inertial delay.
3. Current observations could be valid while fusion with old unobserved tails failed the learned width guard, leading back to the old predicted pair. In replay frame8, the fused gap at y100 was 25.78725px versus required26.24523px; both curves were smooth. Current left/right qualities were0.8560/0.8427. Rejecting the mixed pair did not imply absence of white paint.

## Changes and side-effect controls

- Compare previous independent candidates after mapping with reliable measured flow (motion confidence >=0.5). Missing/unreliable flow retains conservative coordinate consistency checks.
- Use reliable measured motion directly for geometry. Inertial velocity remains a fallback; motion fades during missing observations. Initial implementation failed the long-occlusion drift regression; restored damping for reliable-flow loss cases too, then all tests passed.
- After three consistent independent BOTH-side observations with quality>=0.65 and valid geometry, replace a stale prior when p90 disagreement>6px. Retain only current observed support, not an old unsupported extension.
- If confirmed current bilateral observations pass all current pair guards but their fusion with stale tails fails, use the confirmed supported pair. Do not relax crossing, minimum spacing, learned-width or smoothness guards globally. Partial/weak temporal observations cannot trigger this replacement.
- Add per-frame observation quality, prior disagreement, confirmation count, pair-rejection category, fallback source and recovery flags to existing telemetry.
- Regional performance optimization retains every candidate and all acceptance thresholds. Compute a segment's aligned samples/tangent/centroid once and vectorize overlapping-row medians instead of repeating per-row Python medians and fits for each combination. Cache per-row paint coordinates per call and vectorize NN support reduction with the same per-row Boolean OR semantics. Cache polynomial fits only for byte-identical merged samples; still evaluate each group's tangent checks in original order.

## Tests and identical-input comparison

- 95 tests pass, final run19.648s. Five new tests cover moving bilateral recovery, inconsistent jumps, weak partial pairs, moving single-side reacquisition and old unobserved width tails. Existing long loss, crossing, true curve, straight stripe, right-upper paint and NN ranking regressions pass.
- Same100 moving frames, baseline/current: tracking74/96, both-held state26/4; left observed64/95; right observed60/88; restored42/12; invalid pairs0/0.
- Current replay's longest both-held period0.600s. Recovery frame49: applied motion3.45px, mean RGB change9.84, both OBSERVED after both HELD. Other recoveries at68/81 have nonzero image change as well. Recovery is not dependent on pausing this recorded input.
- Performance-only vectorization compared against the recovery fix before optimization: all100 states and polynomial coefficients identical (max coefficient delta0).
- Scripts: replay_moving_recovery.py, summarize_boundary_recovery.py, capture_left_debug.py, boundary_timing_capture.py. Detailed row/gap/minimum failures are saved in replay JSONL. Artifacts are local under artifacts/moving_recovery_*.

## Device measurements

Initial recovery fix, before performance optimization: 474frames/60.053s =7.876FPS,126 source sequence gaps. Left observed446, right observed455; invalid pairs0; centerline0. Both-held maximum0.200s. Recorded15 transitions back to bilateral OBSERVED with applied motion1.34..5.10px. This supports moving recovery but did not meet throughput.

Bounded timing on device over100frames: total boundary update mean75.74ms/max155.18ms; regional detection56.89ms/max136.10ms; full detection5.86ms; road motion1.89ms; fusion1.28ms. Timings are inclusive function costs and collected with lightweight wrappers; full application/NN/encoding time is outside the total.

First optimization: 553frames/60.080s=9.188FPS,47 sequence gaps; left observed539/right540; invalid0, centerline0, both-held maximum0.200s. Added the exact row-pixel/NN-support/centroid reuse above to improve remaining overhead.

Optimized source deployed into the existing container, hash checked and backend restarted: image_boundaries.py SHA256 `763f751542e192603aa69931dc2e7759b0526dadd1d35a40e0dce48d25e3407e`. No persistent new image build. Diagnostic wrappers are removed after collection, and FPS is measured without them.

- Final60s:595frames/60.033s=**9.8946 output FPS**, source sequence gaps0. Source timestamps span59.4005s for594 intervals (~10Hz); distinguish source rate from measured output wall-time rate. Strict sustained output>=10FPS is not proven by this measurement.
- Left observed582/595, right576/595; tracking588, held7; invalid pairs0, centerline0; restored22. Longest both-held interval0.300s.
- Thirteen transitions back to both OBSERVED during nonzero applied motion (1.07..4.85px). These are recovery events, not a claim of13 independently annotated road scenarios.
- Additional actual device100-frame variable capture spans13.3001source seconds (profiling overhead). Left observed94/right95, restored7, invalid0. Mean consecutive RGB change8.93 intensity units. Actual synchronized input/output contact sheet inspected: most sampled white markings are followed; frame99 still shows aligning/held and an offset right boundary. This remaining transient is explicitly unresolved.
- Actual captured recovery frame79: both HELD -> both OBSERVED while applied motion3.11px and RGB change12.37. Frames33/84 also recover with nonzero image change. This demonstrates recovery without a stationary input in the captured sequence.
- Normal-path30s recheck after removing the capture wrapper:289frames,9.639FPS,2 sequence gaps; left observed285/right286, invalid0, centerline0. This did not establish sustained10FPS. Added the identical-fit cache as a further conservative optimization; all100 replay curves remained bit-identical to the pre-optimization recovery implementation.
- Final deployed source SHA256 after identical-fit cache: `c1d625abc42d1b4d8d72625427a35d8e2883d6f54b8556055775f701f551c2e0`. Same cached container source deployment, backend restarted, no diagnostic wrapper installed.
- Final release measurement (`artifacts/moving_recovery_release_fps`):597frames/60.0027s=**9.9329FPS**,7 sequence gaps, invalid pairs0, centerline0. Left observed563/597, right572/597. Tracking581/held16; restored40. Longest both-held interval0.400s. Nineteen transitions back to bilateral OBSERVED with nonzero applied motion0.81..5.53px. Frame469 recovers from both HELD with4.07px motion; frame700 recovers from left HELD with5.08px motion. Strict sustained>=10FPS and zero transient errors remain unachieved; report the measured rate rather than rounding up as a guarantee.
- `artifacts/moving_recovery_after/actual_frames.jpg` contains synchronized raw input and actual device output samples (not recomputed predictions). Full100 frames and telemetry in boundary_debug.npz/actual_frames.jsonl. `artifacts/moving_recovery_optimized_fps/frames.jsonl` holds the595-frame measurement. Diagnostic wrapper removed afterward.
- Final multiview RGB/result/depth/BEV all received under `artifacts/moving_recovery_release_streams`. Result/BEV inspected, but compression corruption is visible, so pixel alignment conclusions rely on synchronized internal RGB/estimate capture rather than these sequential RTSP stills.

## Limits

This is video replay viewed by the camera, not a vehicle road trial. Mask-based observation state and confidence are not full manually annotated pixel accuracy. Current-pair replacement can move the display noticeably when correcting a large accumulated error; it requires repeated strong bilateral evidence to limit wrong resets. Some held frames and unsupported upper tails remain possible. No claim of universal detection or zero transient error. ROS2 build/TF checks do not apply to this standalone application update.
