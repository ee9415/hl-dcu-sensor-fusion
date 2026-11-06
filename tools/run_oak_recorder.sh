#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CV_SITE="${OAK_RECORDER_CV_SITE:-$SCRIPT_DIR/../artifacts/runtime/recorder-wsl-site}"
if [[ -d "$CV_SITE/cv2" ]]; then
    export PYTHONPATH="$CV_SITE${PYTHONPATH:+:$PYTHONPATH}"
    export OPENCV_FFMPEG_THREADS="${OPENCV_FFMPEG_THREADS:-1}"
    export OPENCV_FFMPEG_IS_THREAD_SAFE="${OPENCV_FFMPEG_IS_THREAD_SAFE:-1}"
fi
exec python3 "$SCRIPT_DIR/oak_recorder.py" "$@"
