# 🚗 Concurrent Routing Server (Waze)

A multithreaded routing server written in **C**, inspired by Waze-style navigation systems.
The server maintains a directed road graph, supports concurrent clients, and computes shortest paths using the **A\*** algorithm while dynamically adapting to traffic updates.

---

## ✨ Features

- ⚡ **Concurrent TCP server** (one thread per client)
- 🧭 **A* shortest-path routing** with geometric heuristic
- 🚦 **Live traffic updates** using Exponential Moving Average (EMA)
- 🔐 **Thread-safe graph** access with reader–writer locks
- 🧪 **Synthetic graph generation** for scalable testing
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
├── load_test.py             # Parallel load testing client
├── Makefile
└── README.md
```

⚠️ The `data/` directory is generated locally and ignored by Git.

---

## 🛠️ Compilation & Execution

From the project root:

```bash
make
./server
```

Or simply:

```bash
make run
```

- 📡 The server listens on **TCP port 8080**
- 📂 Graph data is loaded from the `data/` directory at startup

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

The graph is directed. Node coordinates are used for the A* heuristic.

---

## 🧬 Generating Graph Data

A Python script is provided to generate synthetic graphs for testing.

Example (1000 nodes, 3000 edges):

```bash
./generate_graph.py --nodes 1000 --edges 3000
```

This generates `graph.meta`, `nodes.csv`, and `edges.csv` directly in the `data/` directory.

---

## 🔌 Client Protocol

The server uses a simple **line-based TCP protocol**.
Clients may connect using standard tools such as `nc` (netcat):

```bash
nc 127.0.0.1 8080
```

### 🧭 Routing Request

```
REQ <source_node> <destination_node>
```

✅ Response on success:

```
ROUTE <total_cost> <edge_count> <edge_id_1> <edge_id_2> ...
```

❌ If no route exists:

```
ERR NO_ROUTE
```

### 🚦 Traffic Update

```
UPD <edge_id> <speed>
```

Response:

```
ACK
```

Traffic updates adjust the travel time using an **EMA**.

---

## 🧵 Concurrency Model

- Each client connection runs in its **own thread**
- Routing requests (REQ) acquire a **read lock**
- Traffic updates (UPD) acquire a **write lock**
- Shared graph data is protected by a global `pthread_rwlock_t`

This allows:

- Multiple routing queries to run in parallel
- Safe and consistent traffic updates

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