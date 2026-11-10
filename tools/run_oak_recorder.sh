#!/usr/bin/env bash
set -euo pipefail
# WSL에서 멀티뷰 앱(oak_recorder.py)을 실행하는 스크립트.
# 시스템 패키지를 바꾸지 않고, 프로젝트 전용으로 설치한 OpenCV가 있으면 그것을 우선 사용한다.
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# 격리 설치한 OpenCV 위치(OAK_RECORDER_CV_SITE 환경 변수로 바꿀 수 있음)
CV_SITE="${OAK_RECORDER_CV_SITE:-$SCRIPT_DIR/../artifacts/runtime/recorder-wsl-site}"
if [[ -d "$CV_SITE/cv2" ]]; then
    export PYTHONPATH="$CV_SITE${PYTHONPATH:+:$PYTHONPATH}"
    # 여러 RTSP를 동시에 받을 때 안정적이도록 FFmpeg 디코더를 1스레드로 제한한다.
    export OPENCV_FFMPEG_THREADS="${OPENCV_FFMPEG_THREADS:-1}"
    export OPENCV_FFMPEG_IS_THREAD_SAFE="${OPENCV_FFMPEG_IS_THREAD_SAFE:-1}"
fi
# 받은 인자를 그대로 넘겨 멀티뷰 앱을 실행한다.
exec python3 "$SCRIPT_DIR/oak_recorder.py" "$@"
