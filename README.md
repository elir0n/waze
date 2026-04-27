<div align="center">

# 🗺️ Waze — Real-Time GPS Navigation & Traffic Simulation

[![Language: C](https://img.shields.io/badge/Language-C11-blue?logo=c&logoColor=white)](https://en.wikipedia.org/wiki/C11_(C_standard_revision))
[![Python](https://img.shields.io/badge/Python-3.10+-yellow?logo=python&logoColor=white)](https://python.org)
[![License: MIT](https://img.shields.io/badge/License-MIT-green)](LICENSE)
[![Docker](https://img.shields.io/badge/Docker-ready-2496ED?logo=docker&logoColor=white)](Dockerfile)
[![Platform](https://img.shields.io/badge/Platform-Linux%20%7C%20WSL2-lightgrey?logo=linux&logoColor=white)]()

**A production-grade GPS navigation engine** built from scratch in C with Python tooling — featuring concurrent A\* routing, live traffic ingestion, congestion physics, and a real-time web map powered by Tel Aviv's OpenStreetMap road network.

*Parallel & Distributed Programming — Course Project*

---

</div>

## 📸 Screenshots

<div align="center">

| Select Source | Select Destination |
|:---:|:---:|
| ![Pick source](screenshots/pick_source.png) | ![Pick destination](screenshots/pick_destination.png) |

| ETA Confirmation | Live Navigation |
|:---:|:---:|
| ![ETA dialog](screenshots/eta_dialog.png) | ![Navigation panel](screenshots/nav_panel.png) |

</div>

---

## 🚀 Quick Start

**Using Docker (recommended — zero setup):**

```bash
# Build and launch everything (server + 500 simulated cars + web UI)
docker compose up --build

# Open the live map
open http://localhost:8090/map.html
```

> On first run, the container automatically downloads the Tel Aviv road network from OpenStreetMap (~1–2 min). Subsequent runs start immediately from the cached graph.

**Local build:**

```bash
# 1. Download the Tel Aviv road network (once)
pip install osmnx
python3 scripts/download_tel_aviv.py --out data/

# 2. Build and run the C server
make run          # starts listening on TCP port 8080

# 3. Launch simulation clients
python3 gui/bridge.py &                    # HTTP bridge on port 8090
python3 legacy/car_client.py --mode sim --cars 50 --steps 200
```

---

## ✨ Features

| Feature | Details |
|---|---|
| ⚡ **Concurrent A\* routing** | 8 parallel routing workers; TASK_REQ reads `_Atomic current_travel_time` lock-free |
| 🔄 **Live traffic ingestion** | EMA-smoothed edge weights updated by car speed reports |
| 🚗 **30,000 simulated vehicles** | Server-side physics with congestion modeling |
| 🗺️ **Real-world map** | Tel Aviv OpenStreetMap drive network (6,500 nodes / 12,470 edges) |
| 🌐 **Interactive web UI** | Leaflet.js map with car positions, congestion overlays, route planner |
| 📡 **Line-based JSON protocol** | Simple TCP API compatible with `nc` / curl |
| 🐳 **Docker Compose** | One-command deployment of the full simulation stack |
| 🔮 **Travel time prediction** | EMA-based per-edge prediction endpoint |

---

## 🏗️ Architecture

### System Overview

```
┌──────────────────────────────────────────────────────────────┐
│                        Web Browser                           │
│                   Leaflet.js  map.html                       │
└──────────────────────┬───────────────────────────────────────┘
                       │  HTTP  (port 8090)
┌──────────────────────▼───────────────────────────────────────┐
│                    HTTP Bridge (Python)                       │
│           bridge.py — REST ↔ TCP translation layer           │
│   /positions  /congestion  /navigate  /edges  /metrics       │
└──────────────────────┬───────────────────────────────────────┘
                       │  TCP  (port 8080, line-based JSON)
┌──────────────────────▼───────────────────────────────────────┐
│                  C Routing & Traffic Server                   │
│                                                              │
│  Accept Thread                                               │
│       │  spawns per-connection threads                       │
│       ▼                                                      │
│  Client Threads ──► routing_q ──► ROUTE_WORKERS (×8) ──┐    │
│                                    [_Atomic reads; lock-free] │   │
│  Client Threads ──► traffic_q ──► TRAFFIC_WORKERS (×2) ─┤   │
│                                    [write lock on graph] │   │
│                                                          ▼   │
│                                              Graph (rwlock)  │
│                                             + Vehicle Reg.   │
└──────────────────────────────────────────────────────────────┘
                       │  also TCP (port 8080)
┌──────────────────────▼───────────────────────────────────────┐
│               Simulated Car Clients (Python)                 │
│     car_client.py / flow_field_driver.py — 500–1000 cars     │
│     REGISTER → ROUTE → TICK_ALL → traffic reports → loop     │
└──────────────────────────────────────────────────────────────┘
```

### Three-Tier Concurrency Model

```
TCP socket
    │
    ▼ spawns
client_thread ──► enqueue Task ──► block on cond_var
                       │
                 ┌─────┴─────┐
                 ▼           ▼
           routing_q    traffic_q
                 │           │
         8 workers   2 workers
         (rd lock)   (wr lock)
                 │           │
                 └─────┬─────┘
                       ▼
               Graph (pthread_rwlock_t)
                       │
               task_complete()
                       │
                       ▼
               client_thread sends response
```

1. **Accept thread** — accepts TCP connections; spawns a detached `client_thread_main` per socket.
2. **Client threads** — parse one request line, create a `Task`, enqueue it, then **block** on the task's condition variable. Guarantees per-connection response ordering with zero busy-waiting.
3. **Routing workers** (`ROUTE_WORKERS = 8`) — `TASK_REQ` acquires **no lock**; `current_travel_time` is `_Atomic double`, so A\* weight reads are lock-free. `TASK_PRED` uses a **read lock**. Multiple routing workers run simultaneously without blocking each other.
4. **Traffic workers** (`TRAFFIC_WORKERS = 2`) — hold a **write lock**; EMA updates are serialized to prevent races on `ema_travel_time` and `observation_count`.

---

## 📁 Project Structure

```
waze/
├── src/                          # C — routing engine & TCP server
│   ├── main.c                    #   Entry point: load graph, start server
│   ├── server.h / server.c       #   Thread pools, vehicle registry, task dispatch
│   ├── routing.h / routing.c     #   A* pathfinding over directed graph
│   ├── min_heap.h / min_heap.c   #   Binary min-heap (A* open set)
│   ├── graph.h / graph.c         #   Graph data structures & edge weights
│   └── graph_loader.h / graph_loader.c   # CSV graph parser
│
├── gui/                          # Interactive web frontend
│   ├── map.html                  #   Leaflet.js live map (positions, congestion, routing)
│   ├── bridge.py                 #   HTTP↔TCP bridge with background polling thread
│   └── gui.py                    #   Desktop launcher (PySide6 / Chrome / browser fallback)
│
├── flow_field/                   # Sector-based flow field simulation
│   ├── fields.py                 #   Flow field computation per sector
│   ├── grid.py                   #   2D grid partitioning of the map
│   └── loader.py                 #   Cached flow field loader
│
├── scripts/                      # Data utilities
│   ├── download_tel_aviv.py      #   OSMnx downloader → nodes.csv / edges.csv
│   └── convert_graph.py          #   GraphML → CSV converter
│
├── legacy/                       # Earlier simulation clients
│   ├── car_client.py             #   Multi-car TCP simulation (sim / interactive modes)
│   ├── load_test.py              #   Throughput benchmarking tool
│   └── agents.py                 #   Agent-based car framework
│
├── generate_graph.py             # Synthetic graph generator (for testing)
├── flow_field_driver.py          # Flow-field simulation entry point
│
├── Dockerfile                    # Multi-stage build (gcc → Python slim)
├── docker-compose.yml            # Orchestration: server + sim + bridge
├── entrypoint.sh                 # Auto-download graph if missing, then boot server
├── makefile                      # C build rules
│
├── screenshots/                  # UI screenshots
│   ├── pick_source.png
│   ├── pick_destination.png
│   ├── eta_dialog.png
│   └── nav_panel.png
│
└── data/                         # Graph data (gitignored — generated at runtime)
    ├── graph.meta                #   Node and edge counts
    ├── nodes.csv                 #   node_id, lat, lon
    └── edges.csv                 #   edge_id, from, to, length, speed, type, lanes, oneway
```

---

## 🧠 Data Structures

### Graph

```
Graph
 ├── nodes[MAX_NODES = 100,000]     fixed-size array, O(1) lookup
 │    └── Node
 │         ├── node_id (int64)
 │         ├── lat, lon (double)
 │         └── out_edges → linked list of EdgeNode (adjacency list)
 │
 └── edges[]                        dynamic array, O(1) by edge_id
      └── Edge
           ├── edge_id, from_node, to_node
           ├── base_length (m), base_speed_limit (km/h)
           ├── current_travel_time (s)      ← used by A*
           ├── ema_travel_time (s)          ← smoothed history
           ├── observation_count
           ├── road_type, lanes, is_oneway
           └── capacity                     ← lanes × ⌊length / headway⌋
```

Adjacency list (not matrix) — optimal for sparse road graphs where average out-degree ≈ 2–4.

### Vehicle State

```
VehicleState
 ├── car_id, user_id
 ├── route_edges[]        list of edge IDs for current route
 ├── edge_idx             current position in route
 ├── pos (0.0–1.0)        fractional position along current edge
 ├── speed (km/h)         recomputed each tick from congestion
 └── state                CAR_IDLE | CAR_DRIVING | CAR_ARRIVED
```

Up to **30,000 vehicles** stored server-side. `TICK_ALL` pre-computes the occupancy array once before advancing any car — O(E) not O(V²).

---

## ⚙️ Algorithms

### A\* Routing

`routing.c` implements A\* with a **binary min-heap** supporting O(log N) decrease-key.

| Component | Detail |
|---|---|
| **Edge weight** | `w(e) = current_travel_time(e)` — live EMA-smoothed travel time |
| **Heuristic** | `h(n) = geo_distance(n, goal) / v_max` — equirectangular, admissible |
| **Complexity** | O((V + E) log V) |
| **vs Dijkstra** | Heuristic prunes most of the search space; critical for concurrent load |

Path reconstruction follows parent pointers back from the goal, returning both the **edge-ID path** (for traffic reports) and **node-ID path** (for display).

### Traffic EMA (Exponential Moving Average)

When a car reports its speed on an edge:

```
T_measured  =  base_length / reported_speed

alpha  =  1.0    (first observation — bootstrap immediately)
alpha  =  0.2    (subsequent — smooth noise, react to trends)

T_ema  =  alpha × T_measured  +  (1 − alpha) × T_ema

current_travel_time  ←  T_ema   (next A* query uses this)
```

20% weight on new observations: reacts quickly to sustained congestion while filtering transient spikes.

### Congestion Physics

On each `TICK_ALL dt`:

```
capacity     =  lanes × ⌊edge_length / headway(road_class)⌋

  road class headways:
    motorway / trunk      →  25 m/car
    primary / secondary   →  15 m/car
    residential / other   →   8 m/car

occupancy    =  cars currently on edge  (O(E) pre-pass)
cong_factor  =  max(jam_floor, 1.0 − occupancy / capacity)
speed        =  base_speed × cong_factor
advance      =  (speed × dt) / edge_length
```

---

## 📡 Protocol

Line-based TCP on port **8080**. Each message is a single newline-terminated JSON object (or legacy plain-text for a few commands).

### Requests

| Task | Key fields | Description |
|---|---|---|
| Route request | `start_node`, `destination_node` | Compute optimal A\* path |
| Traffic update | `edge_id`, `speed`, `position_on_edge` | Report car speed → update EMA |
| Register car | `register_car: 1`, `start_node`, `destination_node` | Server allocates car + computes route |
| Tick car | `tick_car: 1`, `car_id`, `dt` | Advance single car by `dt` seconds |
| `TICK_ALL <dt>` | plain text | Advance all cars efficiently (O(E) occupancy pass) |
| `POSITIONS` | plain text | Fetch all car positions |
| `CONGESTION` | plain text | Fetch edges with occupancy > 30% capacity |
| `PRED <edge_id>` | plain text | Predict travel time for an edge |

### Responses

```jsonc
// Route response
{"user_id": 1, "car_id": 42, "route_edges": [101, 205, 88, ...], "eta": 347.5}

// Positions response
{"positions": [{"car_id": 42, "lat": 32.07, "lon": 34.78, "state": "driving"}, ...]}

// Congestion response
{"congestion": [{"edge_id": 101, "occupancy": 12, "capacity": 8}, ...]}

// ACK / error
{"status": "ACK", "user_id": 1, "car_id": 42}
{"error": "NO_PATH"}
```

---

## 🔨 Build & Configuration

### Building the Server

```bash
make              # build ./server
make run          # build + run
make clean        # remove binary

# Override worker counts at compile time
make CFLAGS="-Wall -Wextra -std=c11 -O2 -DROUTE_WORKERS=16 -DTRAFFIC_WORKERS=4"
```

Compiler: `gcc -Wall -Wextra -std=c11 -O2`, linked with `-lm -pthread`.

### Generating Graph Data

```bash
# Real Tel Aviv network (6,500 nodes / 12,470 edges in data/)
pip install osmnx
python3 scripts/download_tel_aviv.py --out data/

# Synthetic graph (fast, for testing)
python3 generate_graph.py --nodes 1000 --edges 3000 --out data/
```

### Manual Testing

```bash
# Raw TCP with netcat
nc 127.0.0.1 8080
{"start_node": 0, "destination_node": 999, "user_id": 1, "car_id": 1, "timestamp": 0}

# Simulation clients
python3 legacy/car_client.py --mode sim --cars 20 --steps 200 --sim-workers 8
python3 legacy/car_client.py --mode interactive

# Load test
python3 legacy/load_test.py --num-nodes 6500 --num-edges 12470
```

---

## 📊 Performance Benchmarks

**Setup**: Tel Aviv OSM graph, 6,500 nodes / 12,470 edges; 32 concurrent REQ clients × 10 rounds, 8 UPD clients × 20 rounds. Run `python3 benchmark.py` to regenerate.

| Route Workers | Throughput (ops/s) | Elapsed (s) | p50 ms | p99 ms | Speedup |
|:---:|---:|---:|---:|---:|---:|
| 1 | 82.4 | 19.42 | 393.4 | 610.5 | 1.00× |
| 2 | 80.5 | 19.87 | 402.8 | 624.1 | 0.98× |
| 4 | 87.7 | 18.24 | 391.9 | 611.2 | 1.06× |
| 8 | 81.0 | 19.76 | 399.6 | 620.4 | 0.98× |

**Key observations:**
- Throughput is roughly flat 1–8 workers on this graph: 32 clients each send requests **sequentially** (send → wait → send), so the bottleneck is client round-trip time, not routing CPU.
- The routing worker pool matters most for batch requests and large graphs where each A\* call is CPU-bound for tens of milliseconds.
- **TASK_REQ** workers now run lock-free (read `_Atomic current_travel_time`) with per-worker pre-allocated scratch buffers — eliminating all per-request `malloc` overhead inside A\*.

---

## 🐳 Docker Deployment

```yaml
# docker-compose.yml services
server:      # C routing daemon — downloads OSM graph on first run
flow-sim:    # Flow-field simulation with 1000 cars (default)
bridge:      # HTTP bridge (car-sim profile)
sim:         # Python car simulation (car-sim profile)
```

```bash
# Full stack (server + flow field simulation + web UI)
docker compose up --build

# With browser-based per-car simulation
docker compose --profile car-sim up --build

# Just the server (bring your own client)
docker compose up server
```

The `data/` volume persists the downloaded graph between restarts.

---

## 🔧 Component Reference

| File | Language | Responsibility |
|---|---|---|
| [src/server.c](src/server.c) | C | TCP server, thread pools, vehicle registry, congestion physics |
| [src/routing.c](src/routing.c) | C | A\* pathfinding with binary min-heap |
| [src/graph.c](src/graph.c) | C | Graph data structure, edge weights, A\* heuristic |
| [src/graph_loader.c](src/graph_loader.c) | C | CSV parser for nodes/edges, OSM ID remapping |
| [src/min_heap.c](src/min_heap.c) | C | Binary min-heap with O(log N) decrease-key |
| [gui/bridge.py](gui/bridge.py) | Python | HTTP↔TCP bridge, background poller, REST API |
| [gui/map.html](gui/map.html) | JS/HTML | Leaflet.js live map: positions, congestion, route planner |
| [flow_field/](flow_field/) | Python | Sector-based flow field computation for large-scale sim |
| [scripts/download_tel_aviv.py](scripts/download_tel_aviv.py) | Python | OSMnx downloader for real-world graph |
| [generate_graph.py](generate_graph.py) | Python | Synthetic random graph generator |

---

## 👥 Authors

**Eliron Picard** · **Roy Meiri**

*Parallel & Distributed Programming — 2025*
