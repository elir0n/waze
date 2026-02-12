# 🚗 Concurrent Routing Server (Waze)

A multithreaded routing server written in **C**, inspired by Waze-style navigation systems.
The server maintains a directed road graph, supports concurrent clients, computes shortest paths using **A\***, adapts travel times in real time using traffic reports, and serves a simple heuristic traffic prediction.
It also includes a CLI simulation and interactive client that act as real users: requesting routes, moving across edges, and sending periodic traffic updates.

---

## ✨ Features

- ⚡ **Concurrent TCP server** (thread-per-client)
- 🧭 **A\*** routing with geometric heuristic
- 🚦 **Live traffic updates** with EMA smoothing
- 🔮 **Heuristic traffic prediction** (EMA-based)
- 🔐 **Thread-safe graph access** with read/write locks
- 🚗 **CLI simulation** with parallel cars + traffic reporting
- 🧪 **Synthetic graph generator** for scalable testing
- 📈 **Parallel load testing client**

---

## 📁 Project Structure

```
.
├── src/
│   ├── main.c               # Server entry point
│   ├── server.c             # TCP server & concurrency logic
│   ├── graph.c              # Graph data structure
│   ├── graph_loader.c       # CSV/meta graph loader
│   ├── routing.c            # A* routing implementation
│   └── min_heap.c           # Priority queue for A*
├── data/                    # Generated graph data (ignored by git)
│   ├── graph.meta
│   ├── nodes.csv
│   └── edges.csv
├── generate_graph.py        # Synthetic graph generator
├── cli_sim.py               # CLI simulation + interactive client
├── load_test.py             # Parallel load testing client
├── Makefile
└── README.md
```

⚠️ The `data/` directory is generated locally and ignored by Git.

---

## 🛠️ Build & Run

From the project root:

```bash
make
./server
```

Or:

```bash
make run
```

- The server listens on **TCP port 8080**
- Graph data is loaded from the `data/` directory

---

## 📊 Graph Input Format

The server expects the following files inside `data/`:

### graph.meta

```
num_nodes <N>
num_edges <M>
```

### nodes.csv

```
node_id,x,y
```

### edges.csv

```
edge_id,from_node,to_node,base_length,base_speed_limit
```

The graph is **directed**. Node coordinates are used for the A* heuristic.

---

## 🧬 Generating Graph Data

A Python script is provided to generate synthetic graphs.

Example (1000 nodes, 3000 edges):

```bash
python3 generate_graph.py --nodes 1000 --edges 3000
```

This generates `graph.meta`, `nodes.csv`, and `edges.csv` directly in the `data/` directory.

---

## 🔌 Client Protocol

The server uses a simple **line-based TCP protocol**. Both `cli_sim.py` (simulation + interactive client) and `load_test.py` use this same protocol. You can also connect manually using `nc` (netcat):

```bash
nc 127.0.0.1 8080
```

### 🧭 Routing Request

```json
{"user_id":1,"car_id":1,"start_node":10,"destination_node":42,"timestamp":12.5}
```

✅ Response on success:

```json
{"user_id":1,"car_id":1,"route_edges":[100,233,912],"eta":47.31}
```

❌ If no route exists:

```json
{"error":"NO_ROUTE","user_id":1,"car_id":1}
```

### 🚦 Traffic Update

```json
{"user_id":1,"car_id":1,"timestamp":13.0,"edge_id":233,"position_on_edge":0.45,"speed":16.2}
```

Response:

```json
{"status":"ACK","user_id":1,"car_id":1}
```
Note: in this implementation, `car_id` is set equal to `user_id`; both are kept in the protocol for future scalability (multiple cars/sessions per user).

Traffic updates adjust the travel time using an **EMA**.

### 🔮 Traffic Prediction (Heuristic)

```
PRED <edge_id>
```

Response:

```
PRED <edge_id> <predicted_travel_time>
```

The prediction is a simple heuristic: the server returns the edge’s EMA travel time (or the current travel time if there is no history).

---

## 🧵 Concurrency Model

- Each client connection runs in its **own thread**
- Routing requests are pushed into a **routing queue** and handled by a **routing worker pool**
- Traffic reports are pushed into a **traffic queue** and handled by a **traffic worker pool**
- Routing workers use a **read lock**; traffic workers use a **write lock**
- Shared graph data is protected by a global `pthread_rwlock_t`

This allows:

- Multiple routing queries to run in parallel
- Safe and consistent traffic updates

---

## 🚗 Simulation (CLI)

The simulation spawns multiple cars, each with its own TCP connection, and runs a discrete-time loop. Cars request routes, move along edges, periodically report traffic updates, and can reroute mid-trip.

Run the server (in one terminal):

```bash
make
./server
```

Run the simulation (in another terminal):

```bash
python3 cli_sim.py --mode sim --cars 20 --steps 200 --sim-workers 8 --reroute-every-steps 5
```

Interactive routing (manual REQ):

```bash
python3 cli_sim.py --mode interactive
```

Interactive prediction:

```
pred <edge_id>
```

At the end of simulation mode, a short summary is printed (arrived/driving/waiting and average drive/wait steps).

---

## 📈 Load Testing

A Python-based load test client is provided:

```bash
python3 load_test.py --num-nodes <N> --num-edges <M>
```

The load test issues concurrent routing and update requests to verify correctness and stability under parallel load.

---

## 📝 Notes

- The graph is directed; some routes may not exist
- All graph operations are thread-safe
- Designed to remain stable under concurrent read/write workloads

---

## 👨‍💻 Authors

- **Eliron Picard**
- **Roy Meiri**
