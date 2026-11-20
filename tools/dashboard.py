"""Live browser dashboard for the sensor simulation.

Starts sensor_sim.exe with --dashboard, keeps its latest data in memory and serves a web page
that shows LIDAR, camera and GPS live.

Usage (from the project folder):
    python tools/dashboard.py                 # real time, runs until Ctrl+C
    python tools/dashboard.py --speed 2       # twice as fast
    python tools/dashboard.py --duration 60   # stop after 60 simulated seconds
Then open http://localhost:8765 (it opens automatically).

Only the Python standard library is used - nothing to install.
"""

import argparse
import json
import os
import subprocess
import sys
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TOOLS_DIR)
EXE = os.path.join(ROOT, "sensor_sim.exe" if os.name == "nt" else "sensor_sim")
HTML = os.path.join(TOOLS_DIR, "dashboard.html")
MIME = {".bmp": "image/bmp", ".jpg": "image/jpeg", ".jpeg": "image/jpeg", ".png": "image/png"}


class SimState:
    """Latest data received from the simulator (shared between threads)."""

    def __init__(self):
        self.lock = threading.Lock()
        self.world = None          # raw JSON text of the world description
        self.latest = None         # raw JSON text of the latest state line
        self.status = "starting"   # starting | running | finished | stopped
        self.frame_no = -1
        self.frame_bytes = b""
        self.frame_type = "image/bmp"

    def snapshot(self):
        with self.lock:
            return self.status, self.latest


STATE = SimState()


def read_simulator(proc):
    """Reads JSON lines from the simulator's stdout."""
    for line in proc.stdout:
        line = line.strip()
        if not line.startswith("{"):
            continue
        try:
            msg = json.loads(line)
        except json.JSONDecodeError:
            continue
        kind = msg.get("type")
        if kind == "world":
            with STATE.lock:
                STATE.world = line
        elif kind == "state":
            cam = msg.get("camera", {})
            frame_bytes = None
            # Copy the camera frame now: the simulator rewrites that file two frames later.
            if cam.get("frame", -1) != STATE.frame_no and cam.get("file"):
                try:
                    with open(cam["file"], "rb") as f:
                        frame_bytes = f.read()
                except OSError:
                    frame_bytes = None
            with STATE.lock:
                STATE.latest = line
                STATE.status = "running"
                if frame_bytes:
                    STATE.frame_no = cam["frame"]
                    STATE.frame_bytes = frame_bytes
                    STATE.frame_type = MIME.get(os.path.splitext(cam["file"])[1].lower(), "image/bmp")
        elif kind == "end":
            with STATE.lock:
                STATE.status = "finished"
    proc.wait()
    with STATE.lock:
        if STATE.status != "finished":
            STATE.status = "stopped"


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):   # keep the console quiet
        pass

    def send_body(self, body, ctype, status=200):
        try:
            self.send_response(status)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass

    def do_GET(self):
        path = self.path.split("?")[0]
        if path in ("/", "/index.html"):
            with open(HTML, "rb") as f:
                self.send_body(f.read(), "text/html; charset=utf-8")
        elif path == "/state":
            status, latest = STATE.snapshot()
            body = '{"status":"%s","state":%s}' % (status, latest or "null")
            self.send_body(body.encode(), "application/json")
        elif path == "/world":
            with STATE.lock:
                world = STATE.world
            self.send_body((world or "null").encode(), "application/json")
        elif path == "/camera":
            with STATE.lock:
                data, ctype = STATE.frame_bytes, STATE.frame_type
            if data:
                self.send_body(data, ctype)
            else:
                self.send_body(b"no frame yet", "text/plain", 404)
        else:
            self.send_body(b"not found", "text/plain", 404)


def main():
    ap = argparse.ArgumentParser(description="Live browser dashboard for sensor_sim.exe")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--speed", type=float, default=1.0, help="1 = real time (default), 2 = twice as fast")
    ap.add_argument("--duration", type=float, default=0, help="simulated seconds, 0 = until Ctrl+C (default)")
    ap.add_argument("--photos", default="assets/camera", help="folder with your own camera photos")
    ap.add_argument("--seed", type=int, default=42, help="obstacle layout")
    ap.add_argument("--no-browser", action="store_true", help="do not open the browser automatically")
    ap.add_argument("--no-save", action="store_true", help="do not save the data to sim_output/")
    args = ap.parse_args()

    if not os.path.exists(EXE):
        sys.exit("sensor_sim.exe not found - build it first with:  .\\build.ps1")
    if args.speed <= 0:
        sys.exit("--speed must be > 0 for the dashboard")

    # The simulator also saves every reading to sim_output/*.csv (+ every 50th camera picture)
    save = ["--no-csv", "--save-images", "0"] if args.no_save else ["--save-images", "50"]
    cmd = [EXE, "--dashboard", *save, "--speed", str(args.speed),
           "--duration", str(args.duration), "--photos", args.photos, "--seed", str(args.seed)]
    proc = subprocess.Popen(cmd, cwd=ROOT, stdout=subprocess.PIPE, text=True, bufsize=1)
    threading.Thread(target=read_simulator, args=(proc,), daemon=True).start()

    server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    url = "http://localhost:%d" % args.port
    print("Sensor dashboard running at %s   (Ctrl+C to stop)" % url)
    if not args.no_save:
        print("Saving all sensor data to %s" % os.path.join(ROOT, "sim_output"))
    if not args.no_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        server.server_close()
        if proc.poll() is None:
            proc.terminate()
            proc.wait()


if __name__ == "__main__":
    main()
