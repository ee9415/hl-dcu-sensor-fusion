# History recovery fix and deployment — 2026-10-23

## Changes

- Initialize each first-observed width row from the actual pair, not a 90% default-width mixture. Unobserved default rows remain untrusted.
- Independently accepted observations on both sides (quality >=0.65, excluding temporal-added weak fragments) may supersede an incompatible learned width after three consecutive consistent pairs. Require >=8 matching rows per side, <=3px variation, shape/noncrossing checks and the existing near minimum gap. Only common observed rows are replaced; other widths are untouched.
- Confirmed pairs can reinitialize a side whose old finite interval overlaps fewer than three current support rows. The retained-history veto can no longer block these disjoint observations indefinitely. This reset changes the supported interval and may visibly relocate an incorrect prior; overlapping observations continue to use existing rate-limited fusion.
- Stream/time resets clear confirmation history. No centerline added. NN ranking, opposite curvature assistance and normal false-positive checks retained.

## Regression evidence

- Existing 80 tests pass; four new recovery tests pass: measured width initialization, stale-width recovery, disjoint interval recovery, and crossing/narrow-pair rejection.
- Synthetic quality-0.8 disjoint observation: previously stuck ALIGNING for 20 updates; now OBSERVED with confidence ~0.8 after confirmation and remains there at update 20.
- Exact captured input from artifacts/history_audit_exact: starts OBSERVED/HELD with restored=true and ends OBSERVED/OBSERVED, restored=false after 20 repeated inputs. The current replay report replaces the previous diagnostic output in that local artifact directory; the original rejection numbers are preserved in history_lock_20261023.md.

## Camera deployment

Uploaded source into existing cached running container, verified SHA256 and restarted backend. No new persistent image built.

- image_boundaries.py: f00f9fee71a76c73d5e065c6139f33b7d4189a4b2f742355613f816793c18885
- lane_boundaries.py: 64b09514380a82f297d7e5e3990581a81eb6b43f713805f01b896397fd0d8ff4
- artifacts/history_recovery_fps: 301 frames / 29.9944s = 10.00187 FPS, source gaps 0, invalid pairs 0, centerline valid 0.
- Restored pair 0/301. Right OBSERVED 301; left OBSERVED 295 and PARTIAL 6.
- RGB/result/depth/BEV streams all received. artifacts/history_recovery_streams/result.jpg inspected: current left/right boundaries displayed together, rather than holding the old right boundary.

## Remaining limits

Static current scene validation only. Physical masking/unmasking and moving curves have not been tested in this deployment. Repeated geometrically valid false markings can still satisfy confirmation, so this is not a general accuracy guarantee. The fix requires two reliable current observations for stale-prior replacement; it deliberately does not let a single low-confidence stripe reset the pair. Stop-line detection is unchanged.
