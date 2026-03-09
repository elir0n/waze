"""
gui/bridge.py — HTTP bridge between the C server and the browser map.

The C server speaks line-based TCP on port 8080.
This script polls it for car positions and congestion data, and exposes them
over HTTP on port 8090 so the browser can fetch them with simple AJAX.

Usage:
    python3 gui/bridge.py [--server-host 127.0.0.1] [--server-port 8080]
                          [--http-port 8090] [--interval 0.5]

Then open: http://localhost:8090/map.html
"""

import argparse
import json
import os
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

# ---------------------------------------------------------------------------
# Background poller: connects to C server, fetches POSITIONS + CONGESTION
# ---------------------------------------------------------------------------

def _tcp_query(host: str, port: int, command: bytes) -> bytes:
    """Open a TCP connection, send command, read one line response."""
    with socket.create_connection((host, port), timeout=2.0) as s:
        s.sendall(command)
        buf = b""
        while b"\n" not in buf:
            chunk = s.recv(4096)
            if not chunk:
                break
            buf += chunk
    return buf


class DataPoller(threading.Thread):
    def __init__(self, server_host: str, server_port: int, interval: float):
        super().__init__(daemon=True)
        self._host      = server_host
        self._port      = server_port
        self._interval  = interval
        self._lock      = threading.Lock()
        self._positions: list = []
        self._congestion: list = []

    # -- public getters (thread-safe) -----------------------------------------

    def get_positions(self) -> list:
        with self._lock:
            return list(self._positions)

    def get_congestion(self) -> list:
        with self._lock:
            return list(self._congestion)

    def get_metrics(self) -> dict:
        with self._lock:
            positions = self._positions
        driving = sum(1 for p in positions if p.get("state") == "driving")
        arrived = sum(1 for p in positions if p.get("state") == "arrived")
        waiting = sum(1 for p in positions if p.get("state") == "idle")
        return {
            "driving": driving,
            "arrived": arrived,
            "waiting": waiting,
            "total":   driving + arrived + waiting,
        }

    # -- background thread ----------------------------------------------------

    def run(self) -> None:
        while True:
            try:
                self._fetch_positions()
            except Exception as e:
                print(f"[bridge] positions poll error: {e}", file=sys.stderr)
            try:
                self._fetch_congestion()
            except Exception as e:
                print(f"[bridge] congestion poll error: {e}", file=sys.stderr)
            time.sleep(self._interval)

    def _fetch_positions(self) -> None:
        buf = _tcp_query(self._host, self._port, b"POSITIONS\n")
        obj = json.loads(buf.decode("utf-8").strip())
        positions = obj.get("positions", [])
        with self._lock:
            self._positions = positions

    def _fetch_congestion(self) -> None:
        buf = _tcp_query(self._host, self._port, b"CONGESTION\n")
        obj = json.loads(buf.decode("utf-8").strip())
        congestion = obj.get("congestion", [])
        with self._lock:
            self._congestion = congestion


# ---------------------------------------------------------------------------
# HTTP handler
# ---------------------------------------------------------------------------

_poller: DataPoller = None  # set in main()


class BridgeHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/positions":
            self._send_json({"positions": _poller.get_positions()})

        elif self.path == "/congestion":
            self._send_json({"congestion": _poller.get_congestion()})

        elif self.path == "/metrics":
            self._send_json(_poller.get_metrics())

        elif self.path in ("/", "/map.html"):
            html_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "map.html")
            try:
                with open(html_path, "rb") as f:
                    body = f.read()
            except OSError as e:
                self.send_response(500)
                self.end_headers()
                self.wfile.write(str(e).encode())
                return
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        else:
            self.send_response(404)
            self.end_headers()
            self.wfile.write(b"Not found")

    def _send_json(self, obj: dict) -> None:
        body = json.dumps(obj).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt, *args):
        pass  # suppress per-request access log noise


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    global _poller

    ap = argparse.ArgumentParser(description="HTTP bridge: C server → browser map")
    ap.add_argument("--server-host", default="127.0.0.1")
    ap.add_argument("--server-port", type=int, default=8080)
    ap.add_argument("--http-port",   type=int, default=8090)
    ap.add_argument("--interval",    type=float, default=0.5,
                    help="Polling interval in seconds (default 0.5)")
    args = ap.parse_args()

    _poller = DataPoller(args.server_host, args.server_port, args.interval)
    _poller.start()
    print(f"[bridge] polling {args.server_host}:{args.server_port} every {args.interval}s")

    httpd = HTTPServer(("0.0.0.0", args.http_port), BridgeHandler)
    print(f"[bridge] serving http://localhost:{args.http_port}/map.html")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[bridge] stopped")


if __name__ == "__main__":
    main()
