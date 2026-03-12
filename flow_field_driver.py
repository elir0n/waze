#!/usr/bin/env python3
"""
flow_field_driver.py — Multi-area flow-field batch-routing driver + HTTP backend.

Tel Aviv is divided into NX×NY geographic sectors.  Each sector has its own
pre-computed flow field (one-time Dijkstra, saved to data/).  Cars travel between
sectors, dwell on arrival, then pick a new destination sector.

Startup (first run — builds and caches flow fields):
    python3 flow_field_driver.py --cars 500 --data-dir data --nx 3 --ny 3

Subsequent runs (loads from cache — starts immediately):
    python3 flow_field_driver.py --cars 500 --data-dir data

Open:  http://localhost:8090/map.html
"""

from __future__ import annotations

import argparse
import csv
import gzip
import json
import math
import os
import random
import socket
import sys
import threading
import time
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, HTTPServer
from typing import Dict, List, Optional, Set, Tuple

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from flow_field.fields import compute_integration_field, compute_flow_field
from flow_field.loader import load_graph_to_grid, cells_in_world_box


# ---------------------------------------------------------------------------
# Node index mapping (raw CSV node_id → C-server sequential index)
# ---------------------------------------------------------------------------

_node_to_idx:     Dict[int, int]    = {}
_node_latlon_arr: "np.ndarray|None" = None   # shape (N,2): lat,lon per node
_node_ids_list:   List[int]         = []


def load_node_index_map(data_dir: str) -> Dict[int, int]:
    """
    The C server assigns sequential indices 0, 1, 2, … to nodes in the order
    they appear in nodes.csv.  Raw node_id values in the CSV (e.g. OSM IDs)
    are NOT the same as these indices.  Build the mapping here so every
    C server call uses the correct sequential index.
    """
    path = os.path.join(data_dir, "nodes.csv")
    mapping: Dict[int, int] = {}
    with open(path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for idx, row in enumerate(reader):
            mapping[int(row["node_id"])] = idx
    return mapping


def nearest_node_to(lat: float, lon: float) -> int:
    """Return the node_id of the graph node nearest to (lat, lon).
    Returns -1 if the lookup table has not been initialised yet.
    """
    if _node_latlon_arr is None:
        return -1
    diffs = _node_latlon_arr - np.array([lat, lon], dtype=np.float32)
    idx = int(np.argmin(diffs[:, 0] ** 2 + diffs[:, 1] ** 2))
    return _node_ids_list[idx]


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

JAM_THRESHOLD      = 0.8   # occ/cap ratio that triggers rerouting
JAM_CHECK_INTERVAL = 10    # check congestion every N sim steps
LOOK_AHEAD_CELLS   = 20    # cells to project along flow vector
HTTP_PORT          = 8090


# ---------------------------------------------------------------------------
# Area dataclass
# ---------------------------------------------------------------------------

@dataclass
class Area:
    area_id:     int
    cells:       List[Tuple[int, int]]   # traversable (col, row) cells — Dijkstra seed
    nodes:       List[int]               # road-graph node IDs in this sector
    flow_x:      np.ndarray              # (H, W) float32 — points toward this area
    flow_y:      np.ndarray              # (H, W) float32
    integration: np.ndarray              # (H, W) float64


# ---------------------------------------------------------------------------
# AreaManager: build / save / load N flow fields
# ---------------------------------------------------------------------------

class AreaManager:
    """
    Divides the map into NX×NY sectors and holds one flow field per sector.

    sector layout (area_id = iy*NX + ix):
      [NX*(NY-1) … NX*NY-1]   ← North row
      ...
      [0   1   …   NX-1  ]    ← South row
       W             E
    """

    def __init__(self,
                 grid,
                 node_world_pos: Dict[int, Tuple[float, float]],
                 nx: int,
                 ny: int,
                 data_dir: str):
        self.nx = nx
        self.ny = ny
        self.areas: Dict[int, Area] = {}
        self._grid_shape = (grid.height, grid.width)

        cache_path = os.path.join(
            data_dir,
            f"areas_{nx}x{ny}_cs{int(grid.bounds.cell_size)}.npz",
        )

        if os.path.exists(cache_path) and self._cache_valid(cache_path, grid, nx, ny, node_world_pos):
            print(f"[areas] Loading area cache from {cache_path} …")
            self._load(cache_path, node_world_pos, grid)
            print(f"[areas] Loaded {len(self.areas)} areas from cache.")
        else:
            print(f"[areas] Building {nx*ny} flow fields (first run — this takes a moment) …")
            self._build(grid, node_world_pos)
            self._save(cache_path, grid, node_world_pos)
            print(f"[areas] Saved area cache to {cache_path}.")

        # Log area population
        for aid, area in sorted(self.areas.items()):
            ix = aid % self.nx
            iy = aid // self.nx
            print(f"  area {aid} (col={ix},row={iy}): "
                  f"{len(area.nodes):>5d} nodes, {len(area.cells):>6d} cells")

    # ------------------------------------------------------------------ #
    # Build                                                                #
    # ------------------------------------------------------------------ #

    def _build(self, grid, node_world_pos: Dict[int, Tuple[float, float]]) -> None:
        dx = (grid.bounds.max_x - grid.bounds.min_x) / self.nx
        dy = (grid.bounds.max_y - grid.bounds.min_y) / self.ny

        for iy in range(self.ny):
            for ix in range(self.nx):
                area_id = iy * self.nx + ix
                x_lo = grid.bounds.min_x + ix * dx
                x_hi = grid.bounds.min_x + (ix + 1) * dx
                y_lo = grid.bounds.min_y + iy * dy
                y_hi = grid.bounds.min_y + (iy + 1) * dy

                cells = cells_in_world_box(grid, x_lo, y_lo, x_hi, y_hi)
                nodes = [
                    nid for nid, (x, y) in node_world_pos.items()
                    if x_lo <= x < x_hi and y_lo <= y < y_hi
                ]

                if cells:
                    integration = compute_integration_field(grid.cost, cells)
                    flow_x, flow_y = compute_flow_field(integration, grid.blocked)
                else:
                    h, w = grid.height, grid.width
                    integration = np.full((h, w), np.inf, dtype=np.float64)
                    flow_x = np.zeros((h, w), dtype=np.float32)
                    flow_y = np.zeros((h, w), dtype=np.float32)

                self.areas[area_id] = Area(
                    area_id=area_id,
                    cells=cells,
                    nodes=nodes,
                    flow_x=flow_x.astype(np.float32),
                    flow_y=flow_y.astype(np.float32),
                    integration=integration,
                )
                print(f"  [{area_id+1}/{self.nx*self.ny}] area ({ix},{iy}) done — "
                      f"{len(cells)} cells, {len(nodes)} nodes")

    # ------------------------------------------------------------------ #
    # Save / load                                                          #
    # ------------------------------------------------------------------ #

    def _save(self, path: str, grid, node_world_pos: Dict[int, Tuple[float, float]]) -> None:
        data: dict = {
            "meta": np.array([
                self.nx, self.ny, grid.bounds.cell_size,
                grid.width, grid.height,
                grid.bounds.min_x, grid.bounds.min_y,
                grid.bounds.max_x, grid.bounds.max_y,
                len(node_world_pos),   # index 9: node count for cache validation
            ], dtype=np.float64),
        }
        for aid, area in self.areas.items():
            data[f"flow_x_{aid}"]    = area.flow_x
            data[f"flow_y_{aid}"]    = area.flow_y
            data[f"nodes_{aid}"]     = np.array(area.nodes, dtype=np.int32)
            cols = np.array([c for c, r in area.cells], dtype=np.int32)
            rows = np.array([r for c, r in area.cells], dtype=np.int32)
            data[f"cells_col_{aid}"] = cols
            data[f"cells_row_{aid}"] = rows
        np.savez_compressed(path, **data)

    def _cache_valid(self, path: str, grid, nx: int, ny: int,
                     node_world_pos: Dict[int, Tuple[float, float]]) -> bool:
        try:
            npz = np.load(path)
            meta = npz["meta"]
            ok = (
                int(meta[0]) == nx and
                int(meta[1]) == ny and
                abs(meta[2] - grid.bounds.cell_size) < 0.1 and
                int(meta[3]) == grid.width and
                int(meta[4]) == grid.height
            )
            # Also check node count if saved (index 9); mismatch → stale cache
            if ok and len(meta) > 9:
                ok = ok and int(meta[9]) == len(node_world_pos)
            return ok
        except Exception:
            return False

    def _load(self,
              path: str,
              node_world_pos: Dict[int, Tuple[float, float]],
              grid) -> None:
        npz = np.load(path)
        dx = (grid.bounds.max_x - grid.bounds.min_x) / self.nx
        dy = (grid.bounds.max_y - grid.bounds.min_y) / self.ny

        for iy in range(self.ny):
            for ix in range(self.nx):
                aid = iy * self.nx + ix
                flow_x = npz[f"flow_x_{aid}"]
                flow_y = npz[f"flow_y_{aid}"]
                # Filter out node IDs that no longer exist in node_world_pos
                # (guards against a stale cache from a different data set)
                valid_ids = set(node_world_pos.keys())
                nodes = [n for n in npz[f"nodes_{aid}"].tolist() if n in valid_ids]
                cols   = npz[f"cells_col_{aid}"]
                rows_  = npz[f"cells_row_{aid}"]
                cells  = list(zip(cols.tolist(), rows_.tolist()))

                h, w = grid.height, grid.width
                integration = np.full((h, w), np.inf, dtype=np.float64)

                self.areas[aid] = Area(
                    area_id=aid,
                    cells=cells,
                    nodes=nodes,
                    flow_x=flow_x.astype(np.float32),
                    flow_y=flow_y.astype(np.float32),
                    integration=integration,
                )

    # ------------------------------------------------------------------ #
    # Helpers                                                              #
    # ------------------------------------------------------------------ #

    def node_area_id(self,
                     node_id: int,
                     node_world_pos: Dict[int, Tuple[float, float]],
                     grid) -> int:
        """Return which area_id a node belongs to."""
        x, y = node_world_pos[node_id]
        dx = (grid.bounds.max_x - grid.bounds.min_x) / self.nx
        dy = (grid.bounds.max_y - grid.bounds.min_y) / self.ny
        ix = int((x - grid.bounds.min_x) / dx)
        iy = int((y - grid.bounds.min_y) / dy)
        ix = max(0, min(ix, self.nx - 1))
        iy = max(0, min(iy, self.ny - 1))
        return iy * self.nx + ix

    def routable_area_ids(self) -> List[int]:
        """Return IDs of areas that have at least one node."""
        return [aid for aid, a in self.areas.items() if a.nodes]


# ---------------------------------------------------------------------------
# Destination selection using flow field
# ---------------------------------------------------------------------------

def pick_destination(
    cur_node: int,
    dst_area: Area,
    node_to_cell: Dict[int, Tuple[int, int]],
    look_ahead: int,
    rng: random.Random,
) -> int:
    if not dst_area.nodes:
        raise ValueError(f"Area {dst_area.area_id} has no nodes")

    cell = node_to_cell.get(cur_node)
    if cell is None:
        return rng.choice(dst_area.nodes)

    col, row = cell
    h, w = dst_area.flow_x.shape
    col = max(0, min(col, w - 1))
    row = max(0, min(row, h - 1))

    vx = float(dst_area.flow_x[row, col])
    vy = float(dst_area.flow_y[row, col])

    if abs(vx) < 1e-6 and abs(vy) < 1e-6:
        return rng.choice(dst_area.nodes)  # no flow at this cell — random fallback

    target_col = col + vx * look_ahead
    target_row = row + vy * look_ahead

    return min(
        dst_area.nodes,
        key=lambda nid: (
            (node_to_cell.get(nid, (target_col, target_row))[0] - target_col) ** 2 +
            (node_to_cell.get(nid, (target_col, target_row))[1] - target_row) ** 2
        ),
    )


# ---------------------------------------------------------------------------
# Car dataclass
# ---------------------------------------------------------------------------

@dataclass
class Car:
    car_id:      int
    cur_node:    int
    dst_node:    int
    cur_area_id: int
    dst_area_id: int
    route:       List[int]
    edge_idx:    int   = 0
    arrived:     bool  = False
    dwell_until: float = 0.0   # simulated time when dwell expires


# ---------------------------------------------------------------------------
# Buffered TCP connection
# ---------------------------------------------------------------------------

class ServerConn:
    def __init__(self, host: str, port: int, timeout: float = 30.0):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.settimeout(timeout)
        self._sock.connect((host, port))
        self._buf = b""

    def send_line(self, line: str) -> None:
        data = (line if line.endswith("\n") else line + "\n").encode()
        self._sock.sendall(data)

    def recv_line(self) -> str:
        while b"\n" not in self._buf:
            chunk = self._sock.recv(65536)
            if not chunk:
                raise ConnectionError("C server closed connection")
            self._buf += chunk
        idx = self._buf.index(b"\n")
        line = self._buf[:idx].decode(errors="replace")
        self._buf = self._buf[idx + 1:]
        return line

    def request(self, line: str) -> str:
        self.send_line(line)
        return self.recv_line()

    def close(self) -> None:
        self._sock.close()


# ---------------------------------------------------------------------------
# Route cache (batching mechanism)
# ---------------------------------------------------------------------------

_route_cache: Dict[Tuple[int, int], List[int]] = {}

# Populated at startup from nodes.csv; used by get_route / register_route


def get_route(conn: ServerConn, start: int, dest: int) -> Optional[List[int]]:
    key = (start, dest)
    if key in _route_cache:
        return _route_cache[key]

    # Translate raw node IDs to C-server sequential indices
    start_idx = _node_to_idx.get(start, start)
    dest_idx  = _node_to_idx.get(dest,  dest)

    req = json.dumps({
        "start_node": start_idx,
        "destination_node": dest_idx,
        "user_id": 0,
        "car_id": 0,
        "timestamp": time.time(),
    })
    try:
        resp_line = conn.request(req)
        data = json.loads(resp_line)
    except Exception as exc:
        print(f"[route] A* request failed: {exc}", file=sys.stderr)
        return None

    if "error" in data:
        if not hasattr(get_route, "_warned"):
            get_route._warned = True
            print(f"[route] A* error: {data['error']} for ({start}→{dest}) "
                  f"[idx {start_idx}→{dest_idx}]", file=sys.stderr)
        return None

    edges = data.get("route_edges", [])
    if not edges:
        return None

    _route_cache[key] = edges
    return edges


def evict_jammed_routes(jammed_edges: Set[int]) -> int:
    stale = [k for k, route in _route_cache.items()
             if any(e in jammed_edges for e in route)]
    for k in stale:
        del _route_cache[k]
    return len(stale)


# ---------------------------------------------------------------------------
# Register route + tick_all helpers
# ---------------------------------------------------------------------------

def register_route(conn: ServerConn, car: Car) -> bool:
    req = json.dumps({
        "register_route": 1,
        "car_id":      car.car_id,
        "start_node":  _node_to_idx.get(car.cur_node, car.cur_node),
        "dest_node":   _node_to_idx.get(car.dst_node, car.dst_node),
        "route_edges": car.route,
    })
    try:
        resp_line = conn.request(req)
        data = json.loads(resp_line)
        return data.get("status") == "REGISTERED"
    except Exception as exc:
        print(f"[reg] register_route failed for car {car.car_id}: {exc}", file=sys.stderr)
        return False


def tick_all(conn: ServerConn, dt: float) -> Optional[dict]:
    try:
        conn.send_line(f"TICK_ALL {dt:.4f}")
        resp_line = conn.recv_line()
        return json.loads(resp_line)
    except Exception as exc:
        print(f"[tick] TICK_ALL failed: {exc}", file=sys.stderr)
        return None


# ---------------------------------------------------------------------------
# Jam handling (A* rerouting only — no flow field changes)
# ---------------------------------------------------------------------------

def query_jammed_edges(conn: ServerConn) -> Set[int]:
    try:
        resp_line = conn.request("CONGESTION")
        data = json.loads(resp_line)
    except Exception:
        return set()
    jammed: Set[int] = set()
    for entry in data.get("congestion", []):
        occ = entry.get("occ", 0)
        cap = max(1, entry.get("cap", 1))
        if occ / cap > JAM_THRESHOLD:
            jammed.add(entry["id"])
    return jammed


def should_reroute(car: Car, jammed_edges: Set[int]) -> bool:
    if car.arrived or not car.route:
        return False
    current_eid = car.route[car.edge_idx] if car.edge_idx < len(car.route) else -1
    if current_eid in jammed_edges:
        return False  # already in jam — committed
    future = car.route[car.edge_idx + 1:]
    return any(e in jammed_edges for e in future)


def handle_jams(
    conn: ServerConn,
    cars: List[Car],
    area_manager: AreaManager,
    node_to_cell: Dict[int, Tuple[int, int]],
    look_ahead: int,
    sim_time: float,
    rng: random.Random,
) -> None:
    jammed = query_jammed_edges(conn)
    if not jammed:
        return

    evicted = evict_jammed_routes(jammed)
    if evicted:
        print(f"[jam] {len(jammed)} jammed edges → evicted {evicted} cache entries")

    with _positions_lock:
        pos_map = {p["car_id"]: p for p in _positions_cache["positions"]}

    rerouteed = 0
    for car in cars:
        if car.arrived or car.dwell_until > sim_time:
            continue
        if not should_reroute(car, jammed):
            continue
        dst_area = area_manager.areas[car.dst_area_id]
        if not dst_area.nodes:
            continue

        # Find the car's actual current node from its reported lat/lon so that
        # rerouting starts from where the car physically is, not the journey origin.
        pos = pos_map.get(car.car_id)
        if pos:
            nn = nearest_node_to(pos["lat"], pos["lon"])
            if nn != -1 and nn in node_to_cell:
                car.cur_node = nn

        new_dest = pick_destination(car.cur_node, dst_area, node_to_cell, look_ahead, rng)
        route = get_route(conn, car.cur_node, new_dest)
        if route:
            car.dst_node = new_dest
            car.route    = route
            car.edge_idx = 0
            register_route(conn, car)
            rerouteed += 1

    if rerouteed:
        print(f"[jam] Rerouted {rerouteed} cars.")


# ---------------------------------------------------------------------------
# Shared state for the HTTP server
# ---------------------------------------------------------------------------

_positions_cache: dict = {"positions": []}
_positions_lock = threading.Lock()


def update_positions_cache(resp: dict) -> None:
    with _positions_lock:
        _positions_cache["positions"] = resp.get("positions", [])


def get_positions_json() -> str:
    with _positions_lock:
        return json.dumps({"positions": _positions_cache["positions"]})


# ---------------------------------------------------------------------------
# HTTP server (same API as bridge.py)
# ---------------------------------------------------------------------------

_http_conn: Optional[ServerConn] = None
_http_conn_lock = threading.Lock()


def _get_http_conn(host: str, port: int) -> ServerConn:
    global _http_conn
    with _http_conn_lock:
        if _http_conn is None:
            _http_conn = ServerConn(host, port, timeout=5.0)
        return _http_conn


def _load_edges_gz(data_dir: str) -> bytes:
    nodes: Dict[int, tuple] = {}
    with open(os.path.join(data_dir, "nodes.csv"), newline="") as f:
        for row in csv.DictReader(f):
            nodes[int(row["node_id"])] = (float(row["lat"]), float(row["lon"]))
    edges = []
    with open(os.path.join(data_dir, "edges.csv"), newline="") as f:
        for row in csv.DictReader(f):
            fn, tn = int(row["from_node"]), int(row["to_node"])
            if fn not in nodes or tn not in nodes:
                continue
            flat, flon = nodes[fn]
            tlat, tlon = nodes[tn]
            speed = float(row.get("base_speed_limit") or row.get("speed_limit") or 50)
            edges.append({
                "id":   int(row["edge_id"]),
                "flat": round(flat, 5), "flon": round(flon, 5),
                "tlat": round(tlat, 5), "tlon": round(tlon, 5),
                "speed": speed,
                "lanes": int(row["lanes"]),
                "rtype": row["road_type"],
            })
    raw = json.dumps({"edges": edges}, separators=(',', ':')).encode()
    return gzip.compress(raw, compresslevel=6)


class _Handler(BaseHTTPRequestHandler):
    c_host: str = "127.0.0.1"
    c_port: int = 8080
    edges_gz: bytes = b""

    def log_message(self, fmt, *args):
        pass

    def _send(self, code: int, ctype: str, body: bytes) -> None:
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = self.path.split("?")[0]

        if path in ("/positions", "/positions/"):
            self._send(200, "application/json", get_positions_json().encode())

        elif path in ("/congestion", "/congestion/"):
            try:
                conn = _get_http_conn(self.c_host, self.c_port)
                with _http_conn_lock:
                    body = conn.request("CONGESTION").encode()
            except Exception:
                body = b'{"congestion":[]}'
            self._send(200, "application/json", body)

        elif path in ("/metrics", "/metrics/"):
            with _positions_lock:
                positions = _positions_cache["positions"]
            n_driving = sum(1 for p in positions if p.get("state") == "driving")
            n_arrived = sum(1 for p in positions if p.get("state") == "arrived")
            n_idle    = sum(1 for p in positions if p.get("state") == "idle")
            body = json.dumps({
                "driving": n_driving,
                "arrived": n_arrived,
                "waiting": n_idle,
                "total":   len(positions),
                "cache_entries": len(_route_cache),
            }).encode()
            self._send(200, "application/json", body)

        elif path in ("/edges", "/edges/"):
            gz = _Handler.edges_gz
            if not gz:
                self._send(200, "application/json", b'{"edges":[]}')
                return
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(gz)))
            self.send_header("Content-Encoding", "gzip")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(gz)

        elif path in ("/map.html", "/"):
            map_path = os.path.join(os.path.dirname(__file__), "gui", "map.html")
            try:
                with open(map_path, "rb") as f:
                    body = f.read()
                self._send(200, "text/html", body)
            except FileNotFoundError:
                self._send(404, "text/plain", b"map.html not found")

        else:
            self._send(404, "text/plain", b"Not found")


def start_http_server(http_port: int, c_host: str, c_port: int, data_dir: str) -> None:
    _Handler.c_host = c_host
    _Handler.c_port = c_port
    try:
        _Handler.edges_gz = _load_edges_gz(data_dir)
        print(f"[http] /edges ready ({len(_Handler.edges_gz):,} bytes gzipped)")
    except Exception as e:
        print(f"[http] WARNING: /edges unavailable: {e}", file=sys.stderr)
    httpd = HTTPServer(("", http_port), _Handler)
    t = threading.Thread(target=httpd.serve_forever, daemon=True)
    t.start()
    print(f"[http] Serving http://localhost:{http_port}/map.html")


# ---------------------------------------------------------------------------
# Main simulation loop
# ---------------------------------------------------------------------------

def run(args: argparse.Namespace) -> None:
    global _node_to_idx, _node_latlon_arr, _node_ids_list
    rng = random.Random(args.seed)

    # ------------------------------------------------------------------
    # 1. Load graph
    # ------------------------------------------------------------------
    print(f"[init] Loading graph from {args.data_dir}/ …")
    _node_to_idx = load_node_index_map(args.data_dir)
    print(f"[init] Node index map: {len(_node_to_idx)} entries.")

    grid, node_world_pos, node_latlon = load_graph_to_grid(
        data_dir=args.data_dir,
        cell_size=args.cell_size,
    )
    print(f"[init] Grid {grid.width}×{grid.height} loaded.")

    # Build fast nearest-node lookup (used for mid-journey rerouting)
    _node_ids_list = list(node_latlon.keys())
    _node_latlon_arr = np.array(list(node_latlon.values()), dtype=np.float32)

    node_to_cell: Dict[int, Tuple[int, int]] = {
        nid: grid.world_to_cell(*pos)
        for nid, pos in node_world_pos.items()
    }

    # ------------------------------------------------------------------
    # 2. Build / load areas
    # ------------------------------------------------------------------
    area_manager = AreaManager(
        grid, node_world_pos,
        nx=args.nx, ny=args.ny,
        data_dir=args.data_dir,
    )

    routable = area_manager.routable_area_ids()
    if len(routable) < 2:
        print("[init] ERROR: Need at least 2 non-empty areas.", file=sys.stderr)
        sys.exit(1)
    print(f"[init] {len(routable)}/{args.nx*args.ny} areas have road nodes.")

    # ------------------------------------------------------------------
    # 3. Connect to C server + start HTTP
    # ------------------------------------------------------------------
    print(f"[init] Connecting to C server at {args.host}:{args.port} …")
    conn = ServerConn(args.host, args.port, timeout=args.timeout)
    print("[init] Connected.")

    start_http_server(HTTP_PORT, args.host, args.port, args.data_dir)

    # ------------------------------------------------------------------
    # 4. Spawn cars
    # ------------------------------------------------------------------
    num_cars = args.cars
    cars: List[Car] = []
    registered = 0
    attempts = 0
    max_attempts = max(num_cars * 20, 100)

    print(f"[init] Registering {num_cars} cars …")
    while registered < num_cars and attempts < max_attempts:
        attempts += 1
        # Pick a random start area and a different destination area
        src_aid = rng.choice(routable)
        dst_candidates = [a for a in routable if a != src_aid]
        dst_aid = rng.choice(dst_candidates)

        src_area = area_manager.areas[src_aid]
        dst_area = area_manager.areas[dst_aid]

        start = rng.choice(src_area.nodes)
        dest  = pick_destination(start, dst_area, node_to_cell, args.look_ahead, rng)

        route = get_route(conn, start, dest)
        if not route:
            continue

        car = Car(
            car_id=registered,
            cur_node=start,
            dst_node=dest,
            cur_area_id=src_aid,
            dst_area_id=dst_aid,
            route=route,
        )
        if register_route(conn, car):
            cars.append(car)
            registered += 1
        elif attempts % 100 == 0:
            print(f"[init] register retries: {attempts} attempts, {registered}/{num_cars} cars ready.")

    if registered < num_cars:
        print(f"[init] WARNING: only {registered}/{num_cars} cars registered after {attempts} attempts.",
              file=sys.stderr)

    print(f"[init] {registered}/{num_cars} cars registered after {attempts} attempts. "
          f"Route cache: {len(_route_cache)} entries.\n")

    # ------------------------------------------------------------------
    # 5. Simulation loop
    # ------------------------------------------------------------------
    sim_time   = 0.0
    step       = 0
    total_time = 0.0
    car_map    = {c.car_id: c for c in cars}

    print(f"[sim] Running (dt={args.dt}s, dwell {args.dwell_min}–{args.dwell_max}s) …\n")

    while True:
        t0 = time.perf_counter()

        resp = tick_all(conn, args.dt)
        if resp is None:
            print("[sim] TICK_ALL failed — exiting.", file=sys.stderr)
            break

        update_positions_cache(resp)
        sim_time += args.dt

        # ---- Process arrivals ----
        for cid in resp.get("arrived", []):
            car = car_map.get(cid)
            if car is None or car.arrived:
                continue
            car.arrived    = True
            car.cur_area_id = car.dst_area_id
            car.cur_node   = car.dst_node
            car.dwell_until = sim_time + rng.uniform(args.dwell_min, args.dwell_max)

        # ---- Dispatch cars whose dwell has expired ----
        for car in cars:
            if not car.arrived or car.dwell_until > sim_time:
                continue

            dst_candidates = [a for a in routable if a != car.cur_area_id]
            dst_aid = rng.choice(dst_candidates)
            dst_area = area_manager.areas[dst_aid]

            dest  = pick_destination(car.cur_node, dst_area,
                                     node_to_cell, args.look_ahead, rng)
            route = get_route(conn, car.cur_node, dest)
            if not route:
                car.dwell_until = sim_time + 5.0   # retry after 5 sim-seconds
                continue

            car.dst_area_id = dst_aid
            car.dst_node    = dest
            car.route       = route
            car.edge_idx    = 0
            car.arrived     = False

            register_route(conn, car)

        # ---- Jam check ----
        if step % JAM_CHECK_INTERVAL == 0:
            handle_jams(conn, cars, area_manager, node_to_cell,
                        args.look_ahead, sim_time, rng)

        t1 = time.perf_counter()
        total_time += (t1 - t0)
        step += 1

        # ---- Periodic log ----
        if step % 25 == 0:
            positions = resp.get("positions", [])
            n_driving = sum(1 for p in positions if p.get("state") == "driving")
            n_arrived = sum(1 for p in positions if p.get("state") == "arrived")
            n_dwell   = sum(1 for c in cars if c.arrived)
            avg_ms    = total_time / step * 1000
            print(f"  step {step:>5d} | t={sim_time:>7.1f}s | "
                  f"driving={n_driving:>5d}  dwell={n_dwell:>5d}  arrived_total={n_arrived:>5d} | "
                  f"cache={len(_route_cache):>5d} | avg {avg_ms:.1f} ms/step")

        # Real-time pacing
        elapsed = time.perf_counter() - t0
        time.sleep(max(0.0, args.dt - elapsed))


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    ap = argparse.ArgumentParser(
        description="Multi-area flow-field batch routing driver",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    ap.add_argument("--host",       default="127.0.0.1")
    ap.add_argument("--port",       type=int,   default=8080)
    ap.add_argument("--cars",       type=int,   default=500)
    ap.add_argument("--data-dir",   default="data")
    ap.add_argument("--cell-size",  type=float, default=100.0)
    ap.add_argument("--dt",         type=float, default=1.0,
                    help="Simulation time step in seconds.")
    ap.add_argument("--nx",         type=int,   default=3,
                    help="Number of sectors east-west.")
    ap.add_argument("--ny",         type=int,   default=3,
                    help="Number of sectors north-south.")
    ap.add_argument("--look-ahead", type=int,   default=LOOK_AHEAD_CELLS,
                    help="Cells to project along flow vector for destination selection.")
    ap.add_argument("--dwell-min",  type=float, default=5.0,
                    help="Minimum dwell time (sim seconds) after arrival.")
    ap.add_argument("--dwell-max",  type=float, default=30.0,
                    help="Maximum dwell time (sim seconds) after arrival.")
    ap.add_argument("--seed",       type=int,   default=42)
    ap.add_argument("--timeout",    type=float, default=30.0,
                    help="TCP socket timeout in seconds.")
    args = ap.parse_args()
    run(args)


if __name__ == "__main__":
    main()