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
import csv
import gzip
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


USER_CAR_USER_ID  = 9999
USER_CAR_ID_START = 100_000


class DataPoller(threading.Thread):
    def __init__(self, server_host: str, server_port: int, interval: float):
        super().__init__(daemon=True)
        self._host      = server_host
        self._port      = server_port
        self._interval  = interval
        self._lock      = threading.Lock()
        self._positions: list = []
        self._congestion: list = []
        self._user_cars: dict = {}          # car_id -> {"state": str}
        self._user_car_counter: int = USER_CAR_ID_START

    # -- public getters (thread-safe) -----------------------------------------

    def get_positions(self) -> list:
        with self._lock:
            positions = list(self._positions)
            user_car_ids = set(self._user_cars.keys())
        for p in positions:
            if p.get("car_id") in user_car_ids:
                p["user_car"] = True
        return positions

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

    def register_user_car(self, src: int, dst: int) -> tuple:
        """Register a user-controlled car. Returns (car_id, eta).

        src/dst are raw OSM node IDs (as shown in the UI).
        They are translated to internal 0-based indices before
        being sent to the C server.
        """
        src_idx = _node_id_to_index.get(src)
        dst_idx = _node_id_to_index.get(dst)
        if src_idx is None or dst_idx is None:
            raise ValueError(f"Unknown node ID(s): src={src}, dst={dst}")

        with self._lock:
            car_id = self._user_car_counter
            self._user_car_counter += 1

        payload = json.dumps({
            "register_car": 1,
            "user_id": USER_CAR_USER_ID,
            "car_id": car_id,
            "start_node": src_idx,
            "destination_node": dst_idx,
            "timestamp": 0.0,
        }, separators=(',', ':')).encode() + b"\n"

        buf = _tcp_query(self._host, self._port, payload)
        resp = json.loads(buf.decode().strip())

        if resp.get("status") != "REGISTERED":
            raise ValueError(resp.get("error", "REGISTER_FAILED"))

        with self._lock:
            self._user_cars[car_id] = {"state": "driving"}

        return car_id, float(resp.get("eta", 0.0))

    # -- background thread ----------------------------------------------------

    def run(self) -> None:
        while True:
            try:
                self._tick_user_cars()
            except Exception as e:
                print(f"[bridge] user car tick error: {e}", file=sys.stderr)
            try:
                self._fetch_positions()
            except Exception as e:
                print(f"[bridge] positions poll error: {e}", file=sys.stderr)
            try:
                self._fetch_congestion()
            except Exception as e:
                print(f"[bridge] congestion poll error: {e}", file=sys.stderr)
            time.sleep(self._interval)

    def _tick_user_cars(self) -> None:
        with self._lock:
            active = {cid: info for cid, info in self._user_cars.items()
                      if info["state"] != "arrived"}

        for car_id in active:
            payload = json.dumps(
                {"tick_car": 1, "car_id": car_id, "dt": self._interval},
                separators=(',', ':')
            ).encode() + b"\n"
            try:
                buf = _tcp_query(self._host, self._port, payload)
                resp = json.loads(buf.decode().strip())
                if resp.get("cmd") == "ARRIVED":
                    with self._lock:
                        if car_id in self._user_cars:
                            self._user_cars[car_id]["state"] = "arrived"
            except Exception as e:
                print(f"[bridge] tick user car {car_id}: {e}", file=sys.stderr)

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
# Static data loaders for /edges and /nodes endpoints
# ---------------------------------------------------------------------------

_edges_gz: bytes = b""
_nodes_gz: bytes = b""
_node_id_to_index: dict = {}   # raw OSM node_id → internal 0-based index


def _load_edges_gz() -> bytes:
    base = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data")
    nodes: dict = {}
    with open(os.path.join(base, "nodes.csv"), newline="") as f:
        for row in csv.DictReader(f):
            nodes[int(row["node_id"])] = (float(row["lat"]), float(row["lon"]))
    edges = []
    with open(os.path.join(base, "edges.csv"), newline="") as f:
        for row in csv.DictReader(f):
            fn, tn = int(row["from_node"]), int(row["to_node"])
            if fn not in nodes or tn not in nodes:
                continue
            flat, flon = nodes[fn]
            tlat, tlon = nodes[tn]
            speed = float(row.get("base_speed_limit") or row.get("speed_limit") or 50)
            edges.append({
                "id":    int(row["edge_id"]),
                "flat":  round(flat,  5), "flon": round(flon,  5),
                "tlat":  round(tlat,  5), "tlon": round(tlon,  5),
                "speed": speed,
                "lanes": int(row["lanes"]),
                "rtype": row["road_type"],
            })
    raw = json.dumps({"edges": edges}, separators=(',', ':')).encode()
    return gzip.compress(raw, compresslevel=6)


def _load_nodes_gz() -> tuple:
    """Returns (gzipped_json_bytes, node_id_to_index_dict)."""
    base = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data")
    nodes = []
    id_to_index = {}
    with open(os.path.join(base, "nodes.csv"), newline="") as f:
        for idx, row in enumerate(csv.DictReader(f)):
            node_id = int(row["node_id"])
            id_to_index[node_id] = idx
            nodes.append({
                "id":  node_id,
                "lat": round(float(row["lat"]), 6),
                "lon": round(float(row["lon"]), 6),
            })
    nodes.sort(key=lambda x: x["id"])
    raw = json.dumps({"nodes": nodes}, separators=(',', ':')).encode()
    return gzip.compress(raw, compresslevel=6), id_to_index


# ---------------------------------------------------------------------------
# HTTP handler
# ---------------------------------------------------------------------------

_poller: DataPoller = None  # set in main()


class BridgeHandler(BaseHTTPRequestHandler):
    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self):
        if self.path == "/positions":
            self._send_json({"positions": _poller.get_positions()})

        elif self.path == "/congestion":
            self._send_json({"congestion": _poller.get_congestion()})

        elif self.path == "/metrics":
            self._send_json(_poller.get_metrics())

        elif self.path == "/edges":
            if not _edges_gz:
                self._send_json({"edges": []})
                return
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(_edges_gz)))
            self.send_header("Content-Encoding", "gzip")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(_edges_gz)

        elif self.path == "/nodes":
            if not _nodes_gz:
                self._send_json({"nodes": []})
                return
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(_nodes_gz)))
            self.send_header("Content-Encoding", "gzip")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(_nodes_gz)

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

    def do_POST(self):
        if self.path == "/navigate":
            length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(length)
            try:
                req = json.loads(body)
                src = int(req["src"])
                dst = int(req["dst"])
            except (KeyError, ValueError, json.JSONDecodeError) as e:
                self._send_error(400, f"Bad request: {e}")
                return
            try:
                car_id, eta = _poller.register_user_car(src, dst)
                self._send_json({"car_id": car_id, "eta": round(eta, 2), "status": "REGISTERED"})
            except Exception as e:
                self._send_error(500, str(e))
        else:
            self.send_response(404)
            self.end_headers()

    def _send_json(self, obj: dict) -> None:
        body = json.dumps(obj).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def _send_error(self, code: int, msg: str) -> None:
        body = json.dumps({"error": msg}).encode("utf-8")
        self.send_response(code)
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

    global _edges_gz
    try:
        _edges_gz = _load_edges_gz()
        print(f"[bridge] /edges ready ({len(_edges_gz):,} bytes gzipped)")
    except Exception as e:
        print(f"[bridge] WARNING: /edges unavailable: {e}", file=sys.stderr)

    global _nodes_gz, _node_id_to_index
    try:
        _nodes_gz, _node_id_to_index = _load_nodes_gz()
        print(f"[bridge] /nodes ready ({len(_nodes_gz):,} bytes gzipped, {len(_node_id_to_index)} nodes mapped)")
    except Exception as e:
        print(f"[bridge] WARNING: /nodes unavailable: {e}", file=sys.stderr)

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
