"""Live web display and control panel for navico.py (stdlib only).

Spoke frames are pushed to the browser unchanged (base64 over Server-Sent
Events) and decoded there, so the page sees exactly what the radar sends.
"""

import base64
import json
import os
import queue
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import navico

HERE = os.path.dirname(os.path.abspath(__file__))


class Hub:
    def __init__(self, radar):
        self.radar = radar
        self.clients = set()
        self.lock = threading.Lock()
        self.recorder = None
        self.record_path = None
        radar.listeners.append(self.on_packet)

    def on_packet(self, channel, t, data):
        if channel != navico.CHANNEL_DATA:
            return
        msg = base64.b64encode(data)
        with self.lock:
            clients = list(self.clients)
        for q in clients:
            try:
                q.put_nowait(msg)
            except queue.Full:
                pass  # slow browser: drop frames rather than lag behind

    def snapshot(self):
        nav = self.radar.nav
        rec = self.recorder
        return {
            "info": self.radar.describe(),
            "state": self.radar.state,
            "raw": self.radar.raw,
            "controls": {k: v[0] for k, v in navico.CONTROLS.items()},
            "nav": None if nav is None else {
                "heading": nav.heading, "cog": nav.cog, "sog": nav.sog, "sent": nav.sent,
                "other_sender": nav.other_sender if nav.yielding() else None},
            "recording": None if rec is None else {
                "path": self.record_path, "packets": rec.count, "bytes": rec.bytes},
        }

    def start_recording(self, path):
        if self.recorder:
            raise ValueError("already recording")
        path = os.path.abspath(path or time.strftime("radar-%Y%m%d-%H%M%S.navrec"))
        self.recorder = navico.Recorder(path)
        self.record_path = path
        self.radar.listeners.append(self.recorder)

    def stop_recording(self):
        rec, self.recorder = self.recorder, None
        if rec:
            self.radar.listeners.remove(rec)
            rec.close()


def make_handler(hub):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, fmt, *args):
            pass

        def _send(self, code, body, ctype="application/json"):
            if not isinstance(body, bytes):
                body = json.dumps(body).encode()
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                with open(os.path.join(HERE, "viewer.html"), "rb") as f:
                    self._send(200, f.read(), "text/html; charset=utf-8")
            elif self.path == "/api/state":
                self._send(200, hub.snapshot())
            elif self.path == "/api/spokes":
                self._stream()
            else:
                self._send(404, {"error": "not found"})

        def _stream(self):
            q = queue.Queue(maxsize=64)
            with hub.lock:
                hub.clients.add(q)
            try:
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Connection", "close")
                self.end_headers()
                while True:
                    try:
                        msg = q.get(timeout=10)
                        self.wfile.write(b"data: " + msg + b"\n\n")
                    except queue.Empty:
                        self.wfile.write(b": keepalive\n\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass
            finally:
                with hub.lock:
                    hub.clients.discard(q)
                self.close_connection = True

        def do_POST(self):
            n = int(self.headers.get("Content-Length", 0))
            try:
                req = json.loads(self.rfile.read(n) or b"{}")
                result = self.route(req)
                self._send(200, {"ok": True, **(result or {})})
            except (ValueError, KeyError, RuntimeError, OSError) as e:
                self._send(400, {"ok": False, "error": str(e)})

        def route(self, req):
            if self.path == "/api/set":
                name = req["control"]
                if name not in navico.CONTROLS:
                    raise ValueError(f"unknown control {name}")
                frames = navico.CONTROLS[name][1](str(req["value"]))
                hub.radar.send(frames)
                return {"sent": [f.hex(" ") for f in frames]}
            if self.path == "/api/power":
                if req["state"] not in ("standby", "transmit"):
                    raise ValueError("state must be standby or transmit")
                frames = navico.power_frames(req["state"])
                hub.radar.send(frames)
                return {"sent": [f.hex(" ") for f in frames]}
            if self.path == "/api/nav":
                vals = {k: (None if req.get(k) in (None, "") else float(req[k]))
                        for k in ("heading", "cog", "sog")}
                hub.radar.set_nav(**vals)
                return None
            if self.path == "/api/record":
                if req.get("action") == "start":
                    hub.start_recording(req.get("path"))
                else:
                    hub.stop_recording()
                return None
            raise ValueError(f"unknown endpoint {self.path}")

    return Handler


def serve(radar, host, port):
    hub = Hub(radar)
    radar.start()
    server = ThreadingHTTPServer((host, port), make_handler(hub))
    server.daemon_threads = True
    print(f"serving {radar.describe()['source']} on http://{host}:{port}/ (Ctrl+C to stop)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        hub.stop_recording()
        server.server_close()
        radar.stop()
    return 0
