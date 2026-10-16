"""Open the real recorder, report stream/preview readiness, and optionally keep it open."""
import argparse
import json
from pathlib import Path
import sys
import tkinter as tk

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tools"))
from oak_recorder import RecorderApp, DEFAULT_DEVICE_SERIAL


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--device-ip", default="192.168.0.4")
    parser.add_argument("--keep-open", action="store_true")
    parser.add_argument("--wait-seconds", type=int, default=60)
    parser.add_argument("--turn", choices=('left', 'right', 'road'))
    args = parser.parse_args()
    root = tk.Tk()
    app = RecorderApp(root, args.device_ip, DEFAULT_DEVICE_SERIAL,
                      None, None, None, REPO / "artifacts" / "recordings")
    result = {}

    def report():
        streams = {}
        for key, source in app.sources.items():
            connected, frame = source.snapshot()
            streams[key] = {"connected": connected,
                            "frames_received": source.frames_received,
                            "shape": list(frame.shape) if frame is not None else None,
                            "preview": bool(getattr(app.preview_labels[key], "image", None))}
        result.update(window_mapped=bool(root.winfo_ismapped()), streams=streams)
        result['turn_controls'] = hasattr(app, 'request_turn') and bool(app.turn_status.get())
        result['turn_status'] = app.turn_status.get()
        logger = app.csv_logger
        result['csv'] = {'path': str(logger.path), 'frames': logger.frames,
                         'objects': logger.objects, 'gaps': logger.gaps,
                         'status': logger.status, 'ui_text': app.csv_status.get()} if logger else None
        result["passed"] = result["window_mapped"] and all(
            s["connected"] and s["shape"] and s["preview"] for s in streams.values())
        result['passed'] = result['passed'] and logger is not None and logger.frames > 0
        if args.turn:
            result['passed'] = result['passed'] and '요청 수락' in app.turn_status.get()
        print(json.dumps(result), flush=True)
        if not args.keep_open:
            app._close()

    root.after(200, app.connect_video)
    if args.turn:
        root.after(2500, lambda: app.request_turn(args.turn))
    root.after(args.wait_seconds * 1000, report)
    root.mainloop()
    return 0 if result.get("passed") else 1


if __name__ == "__main__":
    raise SystemExit(main())
