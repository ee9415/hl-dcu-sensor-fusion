# Repeated history lock diagnosis — 2026-10-23

## Evidence collection

Initially left app running unchanged and captured 10 seconds: artifacts/history_audit_live contains 103 telemetry samples, 39 RGB frames and 15 result frames, no reported decoder errors. Video delivery is sparse; receive-time pairs are approximate and not NN frame synchronization. Do not use these images to time a momentary jump.

All 103 telemetry samples had boundary_restored_pair=true; 100 were left OBSERVED/right HELD and three both HELD. Right detector diagnostics reported accepted=1 while its final state was HELD. The current failure is downstream of finding a candidate.

After preserving live history evidence, restarted with bounded diagnostic capture, exported and restored the original node. Additional exact-state capture includes width_samples and last_valid_pair. Camera algorithm/configuration was not changed during diagnosis; diagnostic restarts did reset its state. No assertion that a UI capture operation itself always resets the tracker.

## Confirmed primary cause: learned-width guard and fallback lock

Exact replay of artifacts/history_audit_exact yields the same OBSERVED/HELD and restored=true. Two rows fail:

| Row at 320x180 | New fused gap px | Required minimum px | Width samples |
| --- | ---: | ---: | ---: |
| 140 | 45.5362 | 47.1597 | 5 |
| 138 | 41.5308 | 45.1653 | 5 |

_plausible_pair uses 0.55*stored width after five samples. Normal fresh minimum at these rows is six pixels. Fallback preserves the old right boundary. Width updates require both states OBSERVED, so once fallback causes HELD the conflicting width cannot update through that route.

A controlled offline ablation on identical inputs, only setting width_samples to zero (disabling learned-width guard while retaining geometric and near-width checks), produces OBSERVED/ALIGNING, restored=false and no pair rejection. This proves the learned-width guard blocks this captured fusion. It is not a proposal to remove all width protection.

Width initializes to clip(1.6*(row-65),24,240), then applies 0.9*old+0.1*measured. After five updates, 59.049% of the initial/default component remains algebraically. The five-sample switch therefore treats a partly assumed width as trustworthy. This explains how the lock can arise again even after a restart on the same scene; first initialization and subsequent motion/observations affect the result.

## Secondary confirmed code failure: disjoint supported intervals

reproduce_history_lock.py feeds identical quality-0.8 observations for 20 frames. With no history: OBSERVED, confidence ~0.8, y120..144. With old left history y150..172: ALIGNING, residual sentinel 100, confidence 4.62e-8, old y150..172 unchanged for all 20 frames. Pair remains valid and side_missing stays zero because an observation object exists.

_fuse clears support when old finite rows exist but observed rows have no overlap. It does not independently adopt the repeated disjoint observation. Reacquisition returns a candidate, but still sends it into the same fusion path. This is a deterministic synthetic reproducer, not proof that the supplied screenshots triggered this exact branch.

## Why masking/unmasking may change the result

Code clears history/width on nonpositive dt, dt>1s, or a large visual cut with invalid motion and mean absolute gray difference >45. Camera/view movement or a physical occluder can trigger these conditions; a UI overlay can also change image evidence if the camera sees that display. Merely freezing a source does not inherently reset tracking, and merely capturing a screenshot is not shown to reset anything. A reset starts selection without previous constraints; it may temporarily recover, then re-enter the width lock after learning. Capture the masking transition to establish which trigger applies to the user's operation.

## Why visible paint is not used immediately

The audited current scene has a right candidate, so insufficient AI detection is not the immediate explanation. Earlier variable capture on this scene produced left quality .6553/right .6627. Ordinary full detection accepts >=.65, whereas independent reacquisition requires >=.75. Temporal fragments additionally require slope difference <=.6, offset <=6px and three repeated samples. A wrongly located history can block short correct fragments. Low confidence does not delete displayed history under the current continuity policy, so incorrect HELD curves remain visible.

The most recent opposite-curvature assistance applies only to PARTIAL with opposite OBSERVED. It does not repair the OBSERVED/HELD state audited here.

## Recommended fix order, not implemented here

1. Distinguish observed width estimates from default initialization; initialize trusted samples from real pairs and check finite observed overlap only.
2. Retain intersection/minimum-gap protection, but allow repeated independently consistent current pairs to supersede a stale learned-width prior with bounded transition. Count rejected-observation duration separately from missing detection.
3. Add controlled adoption for repeated disjoint observations instead of silently clearing all support indefinitely; verify identity/geometry and preserve the other boundary.
4. Align reacquisition criteria with repeated independent evidence rather than relying only on a stricter .75 threshold. Do not globally lower all filters.
5. Test warm history vs cold start on identical replay, masking/unmasking, incorrect narrow pairs, parking stripes, pole/vehicle edges and 10 FPS before deployment.

Tools added are diagnostic only: audit_pair_rejection.py, reproduce_history_lock.py, plus richer bounded debug capture. Current runtime algorithm remains unchanged.
