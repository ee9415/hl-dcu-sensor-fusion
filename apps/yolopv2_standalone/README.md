# YOLOPv2 standalone OAK App

한국어 실습 안내: [OAK4D Pro 개발 빌드 배포 매뉴얼](../../docs/oak4d_development_deployment_manual.md).

## Current: fixed-anchor BEV boundary pair (0.21.0, 2026-10-22)

The active algorithm jointly selects two quadratic boundaries in BEV. Both near
anchors stay fixed at forward distance zero. The requested normal width is vehicle
width plus left/right margins. The boundaries remain visible from startup through
missing paint/road evidence; absence of evidence is not a free-space confirmation.
No center path, goal, turn preview or steering command is produced.

The left quadratic searches both heading C1 and curvature C2 at its fixed anchor. Its exact
normal offset supplies samples for a separate right quadratic fit with fixed C0.
Pairs with excessive normal-width error, crossings or coordinates outside the BEV
are rejected. This is a constrained quadratic approximation, not an exact parallel
curve guarantee. The right coefficients can differ from the left coefficients.

Candidate pairs are scored using support across the WHOLE corridor, additional
pavement beside it, near-connected RGB/NN paint support and temporal/gentle-curve preferences.
A feasible full-width candidate wins over an infeasible candidate. If none fits,
the best-supported direction is approached without reducing width; state stays
blocked until the displayed pair itself fits. Smoothing regenerates paired geometry
and checks width error, anchors and per-frame displacement at every step.

### Startup settings: `lane_boundaries.json`

| Setting | Default | Meaning |
| --- | ---: | --- |
| `vehicle_width_m` | 1.8 | Requested vehicle width |
| `left_margin_m` / `right_margin_m` | 0.3 / 0.3 | Requested left/right margins |
| `max_lateral_rate_m_s` | 1.0 | Maximum displayed lateral movement per second |
| `max_forward_m` | 25.0 | Fixed forward display/search range |
| `max_curvature_coefficient` | 0.02 | Left quadratic C2 search bound |
| `max_heading_coefficient` | 0.2 | Left quadratic C1 bound, also limited by width error |
| `near_left_u` / `near_right_u` / `near_v` | 0.12 / 0.88 / 0.98 | Fixed normalized image start points |
| `far_left_u` / `far_right_u` / `far_v` | 0.46 / 0.54 / 0.60 | Projection reference at forward 40 |
| `width_tolerance_m` | 0.08 | Maximum sampled normal-width approximation error |
| `road_support_threshold` | 0.9 | Required road fraction at EVERY sampled cross-section |

Anchors: left C0 = -vehicle_width/2-left_margin, right C0 = vehicle_width/2+right_margin.
All geometry is still in **uncalibrated development BEV units** (12 lateral × 40
forward). Fields ending in `_m` express intended metric settings but do not establish
actual physical meters without calibration. Normalized near image anchors are explicitly
configured at (0.12,0.98) and (0.88,0.98); far reference points are (0.46,0.60)
and (0.54,0.60). Evidence and rendering use the same homography. These are
installation defaults, not an automatic extraction of screenshot endpoints.
Settings require app restart. Removed settings: observation_gate_m, max_missing_s,
max_lane_gap_s. Unknown settings fail explicitly rather than silently changing geometry.

### Output meaning

- `tracking`: the displayed corridor meets the sampled road-support threshold.
- `blocked`: fixed width cannot currently be confirmed inside the road mask;
  both boundaries remain drawn as continuous solid lines.
- `held`: no road evidence; retain the last pair (straight pair at initial startup).
- `invalid_timestamp`: keep the pair visible without applying motion; a finite clock
  reset re-baselines timing for the next frame.
- `boundary_valid`/side validity/`boundary_pair_valid`: geometry/display validity,
  NOT permission to drive. They remain true for the initialized virtual pair.
- `corridor_feasible` and compatibility `clearance_sufficient`: true/false/null
  for sampled fit, insufficient support, or unknown. This is not a safety classifier.
- `left_boundary_coefficients` / `right_boundary_coefficients`: `[C0,C1,C2]`, with
  forward distance x and lateral position y in `y=C0+C1*x+C2*x*x`.
- `normal_width_error_dev`, `road_coverage`: approximation and mask-support diagnostics.
- `boundary_width_min_dev/max_dev`: same-forward horizontal separation, which is
  intentionally different from normal width on bends.
- `corridor_width_m` now specifies the requested fixed normal width.

CSV serializes both coefficient arrays as JSON. Legacy centerline columns remain
empty/disabled for compatibility; the historical `CenterlineBEV` transport identifier
is retained without any centerline pixels. RGB, object/Depth and recording paths are
unchanged. The obsolete centerline/turn implementations and their tests/diagnostics
were deleted; Git history and dated verification documents retain the earlier work.

Host validation: [fixed BEV boundaries](../../docs/verification/camera/fixed_bev_boundaries_20261022.md).
Older sections below are historical and do not define current boundary behavior.

This app is intended to run entirely on an OAK4-D Pro (RVC4). It keeps the
validated `1920x1080 NV12 -> H.264` camera path at a configured 10 FPS and branches the same frames
through a letterbox resize and RGB conversion for a fixed `1x3x320x320`
YOLOPv2 inference input. CAM_A output is calibration-undistorted so the
`CAM_B/C -> StereoDepth -> ImageAlign` path can produce depth in the same CAM_A
pixel coordinate system.

## Desktop multiview (0.13.0)

### Frame telemetry CSV (0.14.0, 2026-10-16)

Connecting video in the desktop UI starts automatic CSV logging, independent
of the MP4 recording button. The UI shows path, successfully flushed frame
and object row counts, sequence gaps and connection/storage errors. Closing
the app closes the CSV files; reconnecting appends within the same session.
Each new UI run gets a unique directory beside recordings: `telemetry/<date_id>`.
Files grow until the app closes; no automatic rotation/deletion is implemented.

- `frames.csv`: frame/NN/depth sequences and monotonic timestamps, estimated
  capture wall time, PC UTC receive time, c0..c3, validity/tracking/turn state,
  object count, sequence gaps, clock offset/RTT/age, estimated reception delays.
- `objects.csv`: one row per detection, joined by device session + frame_seq;
  index is per-frame, NOT a persistent track ID. Normalized box, class ID,
  confidence, valid depth flag and camera optical X/Y/Z in meters. Z is axial
  depth, not Euclidean distance. Invalid distances are blank. Empty detection
  frames still have a frames.csv row. Invalid centerline coefficients are blank.

This is a direct TCP JSON subscriber on port 8557, not a ROS subscriber.
The read-only endpoint is unauthenticated/unencrypted, for a trusted camera LAN
only; do not expose it to the Internet. A bounded 100-frame queue per client
protects perception from slow consumers. Lost data is not replayed; gaps are
explicit. Frame timestamps are preserved separately for RGB, NN and Depth.

Clock offset is estimated by a request/reply exchange every five seconds.
Delay uses the two hosts' monotonic clocks and their estimated offset, not
wall-clock subtraction. UTC wall times are retained separately for correlation.
`receive_delay_est_ms` estimates RGB frame timestamp to telemetry reception,
NOT RTSP decode/display latency or hardware exposure-to-screen latency.
`transport_delay_est_ms` estimates telemetry publish-to-receive latency.
These are not PTP-synchronized measurements. Negative/nonphysical values,
large RTT yield blank estimates with
`invalid_clock_or_asymmetry`; raw values remain for diagnosis. Positive values
can still be biased by clock drift/asymmetric network delay. Camera timestamp
to wall-time conversion is itself an estimate. No hard accuracy bound is claimed.
Centerline coefficients remain uncalibrated development BEV units, not meters.
During turn preview they describe the road centerline, not the purple turn path.

The desktop recorder receives RGB, result, BEV and colorized Depth directly
over RTSP. The Depth preview is 640x360 at a configured 10 FPS on
`rtsp://<device-ip>:8556/depth`. It samples the aligned 1920x1080 depth at 3:1.
Red is near (0.3 m), blue is far (25 m, since 0.14.1); values outside this display range
saturate and invalid values are black. This lossy H.264 preview is not raw
metric depth. The original millimeter depth used for object XYZ is unchanged.

RemoteConnection visualization is disabled by default, and frontend port
assignment is disabled. Set `OAK_WEB_VISUALIZER=1` explicitly for diagnostic
web visualization. The base image may still print its generic Viewer URL;
this is not required for desktop Depth and does not launch a browser.

## Optional web topics

- `YOLOPv2Result` (`full` mode): a single `960x540` H.264 result stream with
  green drivable-area overlay, red lane-line overlay, and yellow detection
  boxes composited directly over CAM_A.
- `Video` (diagnostic stages): the unannotated H.264 CAM_A stream used by
  `camera`, `depth`, `manip`, and `nn` fault-isolation modes.
- `DepthAligned` (`depth` and `full` modes): CAM_B/C stereo depth aligned to
  CAM_A at `1920x1080`. Valid values are millimeters and zero means invalid
  depth.
- `SpatialDetections` (`full` mode): detections with valid CAM_A optical-frame
  `X`, `Y`, and `Z` coordinates in millimeters. The regular detection path
  retains every object; this spatial topic contains only objects that pass the
  depth-validity rules.
- `CenterlineBEV` (`full` mode): a `640x480` BEV development view showing the
  detected drivable corridor, its boundaries, and the fitted cubic virtual
  centerline.

The fixed neural-network input remains `1x3x320x320`; the larger result size
only affects visualization. Internal detection and binary-mask messages are
still generated by the postprocessor, but `full` mode exposes the combined
stream as the primary Visualizer output.

The app emits a `HEALTH {...}` log record every five seconds with frame count,
average FPS, post-processing time, detection count, and fixed model input
shape. These records are available through `oakctl app logs` even if no
inspection client is connected. Raw JSON `Buffer` topics are not published
because the current RVC4 RemoteConnection does not serialize them.

For each object, the postprocessor samples the center 50% of its RGB-aligned
depth box. Values outside 0.3-50 m are rejected, at least 20 pixels and 10% of
the ROI must remain, and the 10th-90th percentile range is median-filtered.
The aligned frame's transformed intrinsic matrix converts the result to CAM_A
optical-frame XYZ. The result video displays yellow `Z=12.4m` text for a valid
measurement and red `Z=?` when depth is insufficient.

The verified 10 FPS configuration sustains approximately 9.8 FPS after the
startup interval, with about 19-21 ms of host post-processing per frame.

## Recording

The unannotated RGB camera video and, in `full` mode, the composited result are
available as separate H.264 RTSP streams:

```text
RGB:    rtsp://<device-ip>:8555/rgb
Result: rtsp://<device-ip>:8554/yolopv2
BEV:    rtsp://<device-ip>:8553/bev
Depth:  rtsp://<device-ip>:8556/depth
```

On the project PC, start the small recorder window with:

```powershell
& 'C:\Users\qstech_gn\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' -m venv .venv
& .\.venv\Scripts\python.exe -m pip install -r .\tools\oak_recorder_requirements.txt
& .\.venv\Scripts\pythonw.exe .\tools\oak_recorder.py
```

The first two commands are only needed when creating the local environment.

The recorder uses a four-panel layout for `원본 RGB`, `인식 결과`,
`RGB 정렬 Depth`, and `BEV · 가상 중심선`. RGB, the composited result, and BEV
use live RTSP streams. Depth remains an explicit placeholder until its
PC-facing video output is added.

Use the recorder window in this order: `영상 연결`, select `원본 RGB`,
`YOLO 결과`, or both, then use `녹화 시작`, `녹화 멈춤`, and `파일 저장`.
The two available live previews start as soon as `영상 연결` succeeds and remain
visible before, during, and after recording. A single recording selection opens
a normal file-save dialog. Selecting both opens a folder dialog and creates two files:
`*_rgb.mp4` (`1920x1080`) and `*_yolopv2.mp4` (`960x540`). Both are 10 FPS.
The recorder uses H.264 in the MP4 container through Windows Media Foundation,
with MPEG-4 as a compatibility fallback. If the device address changes, pass
`--device-ip <new-device-ip>`.

## Virtual centerline

The postprocessor selects the drivable segment connected to the image center,
maps its left and right boundaries into a provisional `40 m x 12 m` BEV, and
fits the metric-form cubic `y(x) = c0 + c1*x + c2*x^2 + c3*x^3`. A cyan line is
reprojected onto `YOLOPv2Result` and rendered in `CenterlineBEV`. The health log
reports validity, coefficients, and fit residual. This BEV currently uses a
fixed source trapezoid for development; production vehicle coordinates require
camera mounting height/pitch measurement and field calibration.

The displayed centerline uses an exponential temporal filter. A single large
curve jump or up to three missing measurements keeps the preceding result, while
a persistent scene change is accepted. Extraction also requires the drivable
component to connect to the lower image center, reducing false fits on monitor
bezels and surrounding furniture. Invalid object-depth labels are limited to
four sufficiently large boxes per frame; valid distance labels are never hidden.

The connection panel also provides editable OAK IP and masked camera-password
fields plus `IP 자동검색`. `영상 연결` checks whether both RTSP streams already
exist; otherwise it starts the OAK app through `oakctl` using the entered
password. The password is cleared from the UI immediately and is never saved
to a file.

Do not point CAM_A at a monitor displaying its own Visualizer stream. That
creates an optical feedback loop in which rendered masks and boxes are captured
as new model input, causing detections and overlays to saturate. Validate with
a clean road source outside the camera's view of the result display.

## Validation workflow

The first device run must remain a development validation. From the repository
root, run it only after approval:

```powershell
& 'C:\Program Files\oakctl\bin\oakctl.exe' app run .\apps\yolopv2_standalone --detach -d 1142227768
& 'C:\Program Files\oakctl\bin\oakctl.exe' app logs 000 --tail 100 --no-follow -d 1142227768
& 'C:\Program Files\oakctl\bin\oakctl.exe' inspect topics -d 1142227768
```

For fault isolation, set `PIPELINE_STAGE` to `camera`, `depth`, `manip`, `nn`,
`static_nn`, or `full`. `static_nn` sends a deterministic 320x320 test image
through the same explicit preprocessing and inference path so its tensors can
be compared with ONNX Runtime. The default is `full`.

Use `PIPELINE_STAGE=depth` to validate CAM_A and RGB-aligned depth without
loading the YOLOPv2 network. The RVC4 path uses the DepthAI v3 `ImageAlign`
node, left-right consistency checking, subpixel disparity, and the
`ROBOTICS` StereoDepth preset. Object ROI distance and CAM_A XYZ calculation
are active in `full` mode; their hardware accuracy and runtime performance
still require the authenticated device validation.

After numerical and live-image validation, build and install the production
package, then enable automatic startup. Do not enable automatic startup before
the development run passes.

## Model preprocessing

The compiled DLC expects RGB values normalized to `[0, 1]`. When an explicit
node output is linked to `NeuralNetwork`, NNArchive `mean`, `scale`, and channel
metadata do not preprocess the message payload. The app therefore converts
each 320x320 `BGR888i` frame to an FP16 NHWC `NNData` tensor named `images`,
reverses BGR to RGB, and divides by 255 before inference. The original HubAI
download under `artifacts/` is not modified.
