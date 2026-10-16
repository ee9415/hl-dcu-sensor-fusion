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
                            "shape": list(frame.shape) if frame is not None else None,
                            "preview": bool(getattr(app.preview_labels[key], "image", None))}
        result.update(window_mapped=bool(root.winfo_ismapped()), streams=streams)
        result["passed"] = result["window_mapped"] and all(
            s["connected"] and s["shape"] and s["preview"] for s in streams.values())
        print(json.dumps(result), flush=True)
        if not args.keep_open:
            app._close()

    root.after(200, app.connect_video)
    root.after(20000, report)
    root.mainloop()
    return 0 if result.get("passed") else 1


if __name__ == "__main__":
    raise SystemExit(main())
