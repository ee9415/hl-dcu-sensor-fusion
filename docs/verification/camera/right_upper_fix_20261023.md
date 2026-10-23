# Right upper white-line association fix — 2026-10-23

## Changes and side-effect controls

- Full detector stops bridging gaps beyond 24 image rows, caps search half-width at 11 pixels, and requires cross-gap slope within 0.5 of the incoming tangent for gaps >4 rows (at least four previous samples).
- Candidate ranking adds balanced band paint support evaluated within the observed fit span, so numerous lower points cannot completely hide poor upper alignment.
- Gap gating alone was insufficient in replay. Added regional-upper refit: require >=4 supported upper rows, >=75% actual paint agreement there, full curve <=25% agreement and >=6px disagreement. Require >=6 near overlap rows and <=3px median disagreement before joining to the same lower marking.
- Refit uses regional paint-supported rows plus paint-supported lower continuation. Require >=12 rows, <=3px fit residual, smooth shape and image bounds. Existing fusion/pair tests still apply. This returns a single quadratic with actual support flags; confidence is capped at 0.7, not raised to the old full-detector score.
- Brightness, ROI and road/object filters unchanged. No centerline added. Missing upper paint or a different near marking cannot trigger replacement.

## Verification

- Existing 84 tests pass. Three new tests pass using the actual captured mask/old-prior fixture: correct upper location with/without stale history, no replacement without upper paint, no join to a different near line.
- Initial full run had one overly specific test assertion requiring the regional branch even when a correct direct full detection won; removed that branch assertion and kept pixel accuracy/geometry checks. All three new tests then pass. No algorithm change was needed for that assertion.
- Stored ten-frame replay: right x at y100 changes from 195.264..205.158 to 177.742..180.242. Visually inspected white pixels there are x178..180 in the representative frame; these are local sample checks, not full ground-truth annotation.
- Actual binary mask regression fixture: tests/data/right_upper_gap.npz; no source video embedded. replay_right_upper_fix.py records all ten frame comparisons in artifacts/right_upper_pixels/fix_replay.json.

## Deployment

Source uploaded into existing cached camera container, hash verified, backend restarted. No new persistent image built.

- image_boundaries.py SHA256: 1dbc483dff61a741a1c5e5711a53dd159d40a52c5871c876bce3d147406bda0f
- artifacts/right_upper_fix_fps: 301 frames / 29.9981 seconds = 10.00063 FPS; sequence gaps 0, invalid pairs 0, restored pairs 0, centerline valid 0.
- Both sides OBSERVED for all 301 measured frames. Right x(y100)=177.8866..180.2397.
- All four multi-view streams received. artifacts/right_upper_fix_streams/result.jpg inspected: right upper curve now follows the white marking instead of bending toward the roadside.

Static current scene verified. Longer dashed gaps, tighter genuine curves and masking/unmasking in moving footage remain broader validation work; stricter gap gating can drop unsupported long bridges, with regional detection/history remaining the fallback. No claim of universal lane detection accuracy.
