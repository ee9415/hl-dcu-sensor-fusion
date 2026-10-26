# Multiview recording/CSV lifecycle — 2026-10-26

Request: do not save logs on app launch/video connection; save logs together with video recording.

- Removed logger startup from RecorderApp._begin_receivers. Preview-only connections create no CSV sessions.
- Start a fresh FrameCsvLogger only after all selected video writers open successfully. Duplicate start calls while recording are ignored.
- Stop CSV alongside video recording. Window close also stops active recording. A later recording creates a new unique telemetry directory.
- After a blocking telemetry read, check the stop flag before writing, avoiding a late row after stop was requested. Display the completed logger as 기록 중지 instead of leaving 기록 중 text.
- Existing telemetry paths and columns are retained. CSV files remain in telemetry/<date_id>; the video export dialog remains unchanged. This couples recording lifecycle, not hardware frame synchronization: video and telemetry still arrive on separate streams.

Validation: five recorder control tests pass (preview, start/stop/restart, partial writer failure, disconnected video, window close). Actual local telemetry socket-to-CSV test passes and verifies logger thread termination/status. Tests run in WSL Ubuntu Python. GUI widgets/video writers are mocked in control tests; no claim of an interactive camera recording test in this change.

Apply by restarting the desktop multiview app. Existing running recordings/processes were not interrupted. Camera-side code and ROS2 interfaces are unchanged; camera redeployment and ROS2 checks are not needed.
