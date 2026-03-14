# Waze — Technical Report

Course: Parallel & Distributed Programming

---

## 1. System Architecture

The system is split into three communicating services:

```
[Simulated Clients]
      │  TCP (port 8080, JSON)
      ▼
[Graph & Routing Service]  ←──►  [Traffic Ingestion Service]
      │                                    │
      └──────────── shared graph (rwlock) ──┘
```

### Components

| Service | Implementation | Responsibility |
|---|---|---|
| **Graph & Routing Service** | C (`src/server.c`, `src/routing.c`) | Stores the road graph, answers route requests via A\*, exposes TCP endpoint |
| **Traffic Ingestion Service** | C (`src/server.c`) | Receives speed reports from cars, applies EMA to edge travel times |
| **Simulated Clients** | Python (`legacy/car_client.py`) | Cars that request routes, follow them, and send periodic traffic reports |
| **HTTP Bridge** | Python (`gui/bridge.py`) | Translates TCP server state into REST endpoints for the browser |
| **GUI** | Leaflet.js (`gui/map.html`) | Live map showing car positions, road congestion, and real-time metrics |

All services communicate over **TCP on port 8080** (line-based, JSON payloads). The HTTP bridge additionally exposes **port 8090** for the browser.

### Concurrency Model

```
TCP client  ──►  client_thread  ──►  routing_q  ──►  ROUTE_WORKERS (8)  ─┐
                                                                           ├──► graph (rwlock)
TCP client  ──►  client_thread  ──►  traffic_q  ──►  TRAFFIC_WORKERS (2) ─┘
```

1. **Accept thread** — accepts TCP connections, spawns a detached client thread per socket.
2. **Client threads** — parse one request at a time, push a `Task` to the appropriate queue, then block on a condition variable until a worker signals completion. This guarantees ordered responses per connection.
3. **Routing worker pool** (`ROUTE_WORKERS = 8`) — threads hold a **read lock** on the graph; multiple routing queries execute in parallel.
4. **Traffic worker pool** (`TRAFFIC_WORKERS = 2`) — threads hold a **write lock** on the graph; traffic updates are serialized to prevent race conditions on edge weights.

---

## 2. Data Structures

### Graph

```
Graph
 ├── nodes[MAX_NODES = 100 000]    fixed-size array
 │    └── Node { node_id, lat, lon, out_edges → [EdgeNode] }   (adjacency list)
 └── edges[]                       dynamic array
      └── Edge { edge_id, from_node, to_node,
                 base_length, base_speed_limit,
                 current_travel_time,
                 ema_travel_time, observation_count,
                 road_type, lanes, is_oneway }
```

**Adjacency list** (not a matrix): for each node, a linked list of outgoing edge IDs. This is efficient for sparse road graphs where the average node has only 2–4 neighbours.

Node `lat`/`lon` coordinates are stored for the A\* heuristic and for drawing.

`ema_travel_time` and `observation_count` serve as historical statistics: they accumulate past measurements and are used by the prediction endpoint.

### Car (Vehicle State)

```
VehicleState {
    car_id, user_id
    route_edges[]        // list of edge IDs for the current route
    edge_idx             // index into route_edges (current position)
    pos                  // position along current edge, 0.0–1.0
    speed                // current speed (km/h), computed from congestion
    state                // CAR_IDLE | CAR_DRIVING | CAR_ARRIVED
}
```

Cars are stored server-side in a fixed registry (`MAX_CARS = 30 000`). Each car advances in discrete time steps (`TICK_ALL dt`), sends speed reports back to the Traffic Ingestion Service, and requests a new route when it arrives.

---

## 3. Algorithms

### A\* Routing

`routing.c` implements A\* over the directed graph using a **binary min-heap** with `decreaseKey` for O(log N) updates.

**Edge weight**: `w(e) = current_travel_time(e)` — the EMA-smoothed travel time, updated live by the Traffic Ingestion Service.

**Heuristic**: equirectangular distance from node `n` to the goal, divided by the maximum speed in the graph:

```
h(n) = geo_distance(n, goal) / v_max
```

This is **admissible** (never overestimates), so A\* is guaranteed to find the optimal path.

**Why A\* over Dijkstra**: on a geographic road network the heuristic eliminates most of the search space, giving significantly faster queries — especially important for handling many concurrent requests.

### Traffic EMA (Exponential Moving Average)

When a car reports its speed on an edge:

```
T_measured = base_length / reported_speed
alpha      = 1.0   (first observation)
alpha      = 0.2   (subsequent observations)
T_ema      = alpha * T_measured + (1 - alpha) * T_ema
```

A 20% weight on new observations smooths noise while still reacting quickly to sustained congestion. The updated `T_ema` is written back to `current_travel_time`, so the next A\* query immediately uses the new weight.

### Traffic Prediction Heuristic

The `PRED <edge_id>` command returns the EMA-smoothed travel time as the predicted near-future travel time. For edges with no observations yet, it falls back to `base_length / base_speed_limit`. This is a rule-based heuristic: the EMA of past observations is used directly as the prediction.

### Vehicle Physics

On each `TICK_ALL dt`:

```
occupancy    = cars currently on edge
capacity     = lanes × CARS_PER_LANE (5)
cong_factor  = max(0.1, 1.0 − occupancy / capacity)
speed        = base_speed × cong_factor
advance      = (speed × dt) / edge_length
```

`TICK_ALL` pre-computes a single occupancy array for all edges before advancing any car — O(N) instead of O(N²).

---

## 4. Build and Run

### Step 1 — Download the Tel Aviv map

Requires Python with `osmnx` installed (`pip install osmnx`):

```bash
python3 scripts/download_tel_aviv.py --out data/
```

This downloads the Tel Aviv drive network from OpenStreetMap and writes `data/nodes.csv`, `data/edges.csv`, and `data/graph.meta`. One-time download (~1–2 min).

### Step 2 — Run with Docker

```bash
docker compose bulid
```

```bash
docker compose up
```

Builds the C server, starts 500 simulated cars, and exposes the live map at **http://localhost:8090/map.html**.

> If `data/` already contains the graph files from Step 1, the container skips the download and starts immediately.

---

## 5. User Navigation

The live map includes an interactive navigation panel that lets you plan and follow a route alongside the simulated traffic.

### Selecting Source & Destination

Pick origin and destination nodes using **Browse list** (searchable node list) or **Click map** (click directly on the map to snap to the nearest node). Selected nodes are marked with a red dot on the map.

![Navigation panel with source selected](screenshots/pick_source.png)

![Both source and destination selected](screenshots/pick_destination.png)

### Route & ETA Confirmation

After clicking **Navigate**, the server computes the optimal A\* route and displays the estimated travel time. Click **Start Journey** to begin, or **Cancel** to go back.

![ETA confirmation dialog](screenshots/eta_dialog.png)

### Following Your Car

Once the journey starts, your car appears as a pulsing icon on the map. The map centers on the source node and tracks your car in real time among the simulated traffic. When you arrive, a status message confirms it.

Click **New Trip** to reset and plan another route.

![Live map overview](screenshots/nav_panel.png)

---

## 6. Experiments

### 6.1 Effect of Threading on Routing Throughput

**Setup**: 2000-node / 6000-edge graph; 32 concurrent clients each sending 50 routing requests (1600 total). Server compiled with varying `ROUTE_WORKERS`.

| Route Workers | Throughput (ops/s) | Elapsed (s) | p50 latency (ms) | p99 latency (ms) |
|:---:|---:|---:|---:|---:|
| 1 | 202 | 7.90 | 138 | 378 |
| 2 | 393 | 4.08 | 78 | 123 |
| 4 | 322 | 4.98 | 55 | 281 |
| 8 | 148 | 10.80 | 220 | 405 |

**Observations**:
- Going from 1 → 2 workers nearly doubles throughput (×1.94), confirming that parallel routing under the reader lock works correctly.
- Beyond 2 workers the gains diminish and reverse on this small synthetic graph. On a small graph, each A\* query completes in microseconds; the overhead of the task-queue mutex and thread scheduling dominates. On a larger real-world graph (Tel Aviv, ~50 000 nodes), routing queries are more expensive and additional workers would continue to improve throughput.
- p50 latency with 4 workers (55 ms) is lower than with 2 workers (78 ms), suggesting the server handles bursty load better with more threads even when raw throughput is similar.

---

## Authors

- **Eliron Picard**
- **Roy Meiri**
