# Waze Navigation Server — Project Report

---

## 1. Application Overview

The project is a real-time TCP navigation server modelled after the Waze navigation system. It operates on the Tel Aviv road network (OpenStreetMap data, 6,500 nodes / 12,470 edges) and serves multiple simultaneous clients over a line-based JSON/TCP protocol on port 8080.

**What it does:**

- **Route computation** (`TASK_REQ` / `batch_routes`): given a source and destination node ID, returns the shortest-time path as an ordered list of edge IDs and an estimated travel time (ETA). Batch mode accepts up to 64 (source, destination) pairs in a single request and uses a hierarchical section-based shortcut to answer them faster.
- **Traffic updates** (`TASK_UPD`) and **movement instructions** (`TASK_TICK`): connected cars report their current `edge_id` and `speed` via `TASK_UPD`; the server converts the speed report to a measured travel time, blends it into a per-edge Exponential Moving Average (EMA), and acknowledges the update. Separately, cars send `TASK_TICK` (`{"tick_car":1,"car_id":…,"dt":…}`) to request advancement along their route; the server advances the car by `dt` seconds (accounting for congestion), then instructs the client where to go: it returns `{"cmd":"MOVE","edge_id":…,"position":…,"lat":…,"lon":…}` while the car is still en route, or `{"cmd":"ARRIVED","lat":…,"lon":…}` when the destination is reached.
- **Travel-time prediction** (`TASK_PRED`): returns the current EMA travel time for a given edge.
- **Vehicle simulation**: the server tracks up to 30,000 vehicles (`MAX_CARS`). Each vehicle holds a car ID, a pre-computed route (list of edge IDs), and a fractional position along its current edge. `TASK_TICK` advances a single car; `TASK_TICK_ALL` advances all active cars by `dt` seconds in one parallelised pass.
- **Congestion modelling**: each edge has a physical capacity based on its length, lane count, and road class. When occupancy exceeds the capacity, vehicle speed degrades linearly toward a road-class-specific minimum speed floor.
- **Position / congestion queries**: `POSITIONS` returns lat/lon for all active cars; `CONGESTION` returns edges where occupancy exceeds 30% of capacity, with coordinates for map rendering.

**How it is used:**

The server is built with `make` and run as `./server [--workers N] [--port P]`. Clients connect over TCP and send newline-terminated JSON (or legacy plain-text) messages. The Python scripts in `legacy/` (`car_client.py`, `load_test.py`) demonstrate both interactive and load-test usage. The GUI in `gui/` renders car positions and congestion on a Leaflet.js map via a Python HTTP bridge.

---

## 2. Architecture

### 2.1 Graph (`graph.h`, `graph.c`)

```c
typedef struct {
    int edge_id;
    double base_length, base_speed_limit;
    char road_type[32];
    int lanes, is_oneway;
    _Atomic double current_travel_time;   // live weight, lock-free reads
    double ema_travel_time;               // protected by per-stripe mutex
    int observation_count;
} Edge;

typedef struct EdgeNode { int edge_id; struct EdgeNode* next; } EdgeNode;

typedef struct {
    int node_id; double lat, lon;
    EdgeNode* out_edges;                  // singly-linked adjacency list
} Node;

typedef struct {
    Node  nodes[MAX_NODES];               // fixed array, MAX_NODES = 100,000
    Edge* edges;                          // dynamic array
    int   num_nodes, num_edges;
    double max_speed_limit;
} Graph;
```

**Why this layout:** The fixed `nodes[]` array gives O(1) node access by ID with no pointer indirection. Each node's adjacency list is a singly-linked list of `EdgeNode`s — simple to build incrementally at load time and efficient to traverse during A\* neighbour expansion (O(degree) per node). Edge attributes live in a separate dynamic array indexed by `edge_id`, giving O(1) edge lookup. Separating `current_travel_time` as `_Atomic double` from the mutable `ema_travel_time` (protected by a mutex) is the key design decision that lets routing workers read live weights without any lock.

### 2.2 A\* Pathfinding (`routing.c`, `min_heap.c`)

`find_route_a_star_path()` implements A\* with two distinct execution paths depending on whether a `RouteContext` is provided by the caller.

**Fast path (used by all worker threads at runtime):**

The caller passes a per-worker `RouteContext`, which is a struct of pre-allocated scratch arrays:

```c
typedef struct {
    double* g_score, *f_score, *h_key;  // per-node scores and heap keys
    int*    parent, *node_path, *path_edges;
    int*    h_heap, *h_pos;             // flat binary min-heap arrays
    unsigned int* node_gen;             // generation tag per node
    unsigned int  gen;                  // current generation counter
    int h_size, capacity;
} RouteContext;
```

Two optimisations make repeated calls cheap:

1. **Generation counter (O(1) reset):** instead of `memset`-ing the entire `g_score`, `parent`, etc. arrays before each A\* call (O(V)), the counter `gen` is incremented by one. A node `v` is considered "seen in this call" iff `node_gen[v] == gen`. Only the start node needs explicit initialisation. On the 6,500-node graph this eliminates ~26,000 stores per call; on the maximum 100,000-node graph it saves ~400,000 stores.

2. **Flat binary min-heap:** the open set is a position-tracked binary min-heap backed by three parallel arrays: `h_heap[i]` (node ID at heap slot `i`), `h_key[node_id]` (f-score), and `h_pos[node_id]` (current heap index). There are no per-node heap allocations. `decrease-key` is performed in-place via `fh_sift_up(ctx, h_pos[v])` — no separate operation needed.

**Heuristic:** `heuristic(g, v, goal)` returns the straight-line travel time using the equirectangular approximation divided by `g->max_speed_limit`. This is admissible (never overestimates), so A\* always finds the optimal path.

**Edge weight:** `get_edge_weight(g, eid)` returns `atomic_load(&g->edges[eid].current_travel_time)`. This is the live EMA-smoothed travel time, so routes always reflect current congestion.

**Fallback path (no RouteContext):** used only at startup for precompute tasks when a context could not be allocated. Falls back to `malloc`/`free` per call and the legacy `MinHeap` from `min_heap.c` (pointer-per-node, O(V) initialisation).

**Complexity:** O((V + E) log V) worst case; the admissible heuristic prunes the search substantially on real road networks.

### 2.3 Traffic Update — EMA (`server.c`, `traffic_worker_main`)

When a car reports `speed` on `edge_id`:

```
measured            = base_length / speed          // speed → travel time (s)
alpha               = 1.0  if observation_count == 0  // bootstrap
                    = 0.2  otherwise               // subsequent smoothing
ema_travel_time     = alpha * measured + (1 − alpha) * ema_travel_time
current_travel_time = ema_travel_time              // atomic store
observation_count++
```

The read-modify-write on `ema_travel_time` and `observation_count` is protected by `edge_stripe_mu[edge_id % N_EDGE_STRIPES]` (64 stripe mutexes). The final write to `current_travel_time` is a C11 `_Atomic double` store, making the new weight immediately visible to all A\* workers without any lock acquisition.

**Why EMA:** the smoothing constant α = 0.2 keeps ~80% of the previous estimate, providing noise resistance while still converging to sustained congestion within a few reports. α = 1.0 on the first observation bootstraps the estimate from the base speed limit.

### 2.4 Congestion Model (`road_class`, `edge_capacity`, `jam_floor`)

```
capacity  = lanes × max(1, floor(base_length / meters_per_car(road_class)))
cong_frac = max(jam_floor(road_class), 1 − occupancy / capacity)
speed     = base_speed × cong_frac
```

Road class (0 = residential → 5 = motorway) controls two parameters:
- `meters_per_car`: physical headway at capacity (8 m residential → 25 m motorway).
- `jam_floor`: minimum speed fraction (0.05 residential → 0.30 motorway).

**Why:** this linear degradation model is simple, parameter-free per edge, and physically motivated. Motorways have longer headways and higher jam floors because they are designed for higher sustained speeds even under congestion.

### 2.5 Hierarchical Section Routing

At startup, the bounding box of all nodes is divided into a **5 × 5 geographic grid** (25 sections). For each section the **hub node** is the node geometrically closest to the section centroid. A\* is run between every valid ordered hub pair (up to 25 × 24 = 600, with empty sections skipped), and the resulting edge lists are stored in `section_paths[i][j]`.

At runtime, a `batch_routes` request groups its (src, dst) pairs by `(src_section, dst_section)`. For each cross-section group:

```
Phase 1: A*(src[k]   → hub_src)  — individual per car, via segment worker pool
Phase 2: section_paths[src_sec][dst_sec]  — O(1) lookup, same for all cars in group
Phase 3: A*(hub_dst  → dst[k])  — individual per car, via segment worker pool
full_path = P1 + P2 + P3
```

Same-section pairs or missing precomputed paths fall back to a direct A\* call.

**Why:** Phase 2 is computed once at startup and shared by all cars going from the same source section to the same destination section. This amortises the cost of the longest part of the route (the inter-section highway segment) across all cars in the group.

### 2.6 Server State and Task Queues

All server state is held in a single `ServerState` struct:

```c
typedef struct {
    Graph*           g;
    pthread_rwlock_t graph_lock;         // still used for TASK_PRED and TASK_PRECOMPUTE
    VehicleRegistry  vehicles;           // mutex-protected
    TaskQueue        routing_q;          // condvar queue for routing/tick/register tasks
    TaskQueue        traffic_q;          // condvar queue for EMA update tasks
    SectionPath      section_paths[25][25];
    int              section_hubs[25];
    int*             node_sections;
    SegTaskQueue     seg_q;              // condvar queue for segment workers
    pthread_mutex_t  edge_stripe_mu[64]; // per-stripe EMA protection
    ...
} ServerState;
```

Each `Task` carries a `pthread_mutex_t` / `pthread_cond_t` pair. Client threads block on `t->cv` after enqueuing; workers call `task_complete(t, resp)` to signal completion. This preserves per-connection response ordering without busy-waiting.

---

## 3. Parallelism Analysis

### 3.1 What is parallelised and why

| Component | Threads | Synchronisation | Why it can run in parallel |
|---|---|---|---|
| **Routing worker pool** | `route_workers` (default 8, configurable via `--workers`) | None on graph reads — `current_travel_time` is `_Atomic double` | Each A\* call operates on its own `RouteContext` scratch space; graph edges are read-only after load (except `current_travel_time`) |
| **Startup precompute** | Same routing workers, `TASK_PRECOMPUTE` tasks | `precompute_remaining` counter + condvar | All 272+ valid (i,j) section-hub paths are fully independent |
| **Batch cross-section groups** | One routing worker per group dispatched via `routing_q` | Task `mu`/`cv` per group | Different (src_sec, dst_sec) groups use disjoint hub pairs |
| **Segment worker pool (Phase 1 + 3)** | `SEG_WORKERS = 8` persistent threads, `SegTaskQueue seg_q` | `_Atomic int pending` decremented per task; dispatcher spins on `sched_yield` | All Phase-1 and Phase-3 A\* calls within a batch group are independent; each worker owns a `RouteContext` |
| **`TICK_ALL` vehicle advancement** | `N_TICK_WORKERS = 4` threads created per call, non-overlapping slot ranges | `__atomic_fetch_add/sub(&occ[eid], 1, ATOMIC_RELAXED)` for occupancy transitions | Cars on disjoint slot ranges never share `VehicleState`; occupancy updates are heuristic so relaxed atomics suffice |
| **Traffic EMA updates** | `TRAFFIC_WORKERS = 2` persistent threads, `traffic_q` | `edge_stripe_mu[edge_id % 64]` per update | Workers updating edges in different stripe buckets (62/64 chance) proceed concurrently; only same-bucket updates serialise |
| **Per-connection client threads** | One detached thread per TCP connection | TCP socket is per-thread | Each client's request stream is independent; a 4,096-byte `RecvBuf` amortises `recv()` syscalls |

### 3.2 What remains serial and why

| Component | Why it must be serial |
|---|---|
| **EMA read-modify-write within a stripe bucket** | The three-field update (`ema_travel_time`, `current_travel_time`, `observation_count`) on the same edge is not atomic as a unit; the stripe mutex serialises writers on the same bucket. This is the minimum serialisation required — it does not affect workers on different edges. |
| **Occupancy pre-pass in `handle_tick_all`** | The `occ[]` array is built in a single O(E) serial pass over all active cars before tick workers start. This is necessary because the tick workers read `occ[]` during advancement; building it in parallel would require synchronisation on every edge increment. |
| **`VehicleRegistry` mutation** | `TASK_REGISTER` and single-car `TASK_TICK` hold `vehicles.mu` while scanning and updating the car slot array. These operations are inherently sequential on shared state. |
| **Response building in `TICK_ALL`** | After the 4 tick threads join, the JSON response (positions + arrived list) is built in a single pass. The response is only valid once all cars have advanced, so it cannot be interleaved with ticking. |
| **Graph load and section setup** | Done once before any connections are accepted. No benefit to parallelising one-time startup work. |

### 3.3 How parallel and serial parts interact

```
TCP accept loop (single accept thread)
    │
    └─► client_thread_main (one detached thread per connection)
            │  RecvBuf: one recv() per 4096 bytes, not per line
            │  Parse JSON line; dispatch by type
            │
            ├─ TASK_REQ / TASK_REGISTER / TASK_TICK / TICK_ALL / ...
            │      → queue_push(&routing_q, t)
            │      → pthread_cond_wait(&t->cv)  ← blocks until worker signals
            │
            └─ TASK_UPD
                   → queue_push(&traffic_q, t)
                   → pthread_cond_wait(&t->cv)

routing_q ──► routing_worker [0..N-1]  (each owns one RouteContext)
    │
    ├─ TASK_REQ:   find_route_a_star_path(ctx)  — no lock, _Atomic reads
    │
    ├─ TASK_PRED:  pthread_rwlock_rdlock → read ema_travel_time → unlock
    │
    ├─ TASK_PRECOMPUTE: pthread_rwlock_rdlock → A* → unlock
    │                   decrement precompute_remaining; if 0 → condvar broadcast
    │
    └─ TASK_BATCH_SECTION:
            │  push 2N SegTasks to seg_q  (pending = 2N, atomic)
            │
            seg_worker [0..7]  (each owns one RouteContext)
                pop SegTask; A*(no lock); __atomic_fetch_sub(&pending, 1)
            │
            spin sched_yield until pending == 0
            stitch P1 + P2 + P3 → task_complete → signal client_thread

    ├─ TASK_TICK_ALL:
            │  serial O(N) pre-pass → build occ[]
            │  pthread_create × 4 (tick_worker_fn, non-overlapping ranges)
            │    each tick worker: atomic load/add/sub on occ[]
            │  pthread_join × 4
            │  serial response build → task_complete

traffic_q ──► traffic_worker [0..1]
    ├─ validate edge_id / speed
    ├─ stripe = edge_id % 64
    ├─ lock edge_stripe_mu[stripe]
    │    EMA update (ema_travel_time, observation_count)
    │    atomic_store(&current_travel_time, ema_travel_time)
    └─ unlock → task_complete → signal client_thread
```

**Synchronisation summary:**

- Routing workers and segment workers read `current_travel_time` as an `_Atomic double` load — no lock, no contention with traffic workers.
- Prediction workers take a `pthread_rwlock_rdlock` to read `ema_travel_time`; this lock is also held by precompute workers at startup (both are read locks, so they do not block each other).
- Traffic workers take `edge_stripe_mu[stripe]` for the EMA RMW, then do a lock-free atomic store to `current_travel_time`. The stripe mutex is released before `task_complete`.
- Tick workers use `__ATOMIC_RELAXED` add/subtract on `occ[]` — this is safe because the occupancy value is only a heuristic input to the congestion speed formula; slight over-/under-counting between workers is acceptable.

---

## 4. Benchmarks

### 4.1 Setup

- **Graph:** Tel Aviv OpenStreetMap, 6,500 nodes / 12,470 edges
- **Load:** 128 concurrent REQ clients × 50 route rounds each; 0 UPD clients (pure routing scalability test); `route_repeats=10` — each request runs A\* 10 times before responding, making routing CPU-bound and amplifying the parallelism signal
- **Method:** `benchmark.py` starts the server once per worker count, runs one discarded warmup pass, then runs 3 timed passes against the warm server and reports the median throughput, elapsed time, and latency percentiles
- **Machine:** WSL2 on Linux 5.15 (Windows host), AMD Ryzen 7 8845HS, **16 logical cores**
- **Worker counts tested:** 1, 2, 3, 4, 6, 8, 12, 16, 20, 24, 28, 32

### 4.2 Results

```
 Workers |  Throughput (ops/s) |   Elapsed |   p50 ms |   p99 ms |  Speedup
--------------------------------------------------------------------------
       1 |              362.5  |   17.65s  |    349.3 |    451.6 |    1.00x
       2 |              710.7  |    9.01s  |    176.7 |    217.4 |    1.96x
       3 |             1018.4  |    6.28s  |    123.1 |    149.4 |    2.81x
       4 |             1309.2  |    4.89s  |     95.0 |    116.7 |    3.61x
       6 |             1954.9  |    3.27s  |     62.7 |     84.2 |    5.39x
       8 |             2537.8  |    2.52s  |     47.6 |     62.7 |    7.00x
      12 |             3449.1  |    1.86s  |     31.5 |     44.8 |    9.51x
      16 |             4086.9  |    1.57s  |     13.3 |     31.7 |   11.27x  ← peak
      20 |             3779.9  |    1.69s  |     13.9 |     41.4 |   10.43x
      24 |             3797.2  |    1.69s  |     15.1 |     45.5 |   10.47x
      28 |             3707.0  |    1.73s  |     13.4 |     46.4 |   10.23x
      32 |             3700.4  |    1.73s  |     14.9 |     50.7 |   10.21x
--------------------------------------------------------------------------
Baseline (1 worker): 362.5 ops/s
```

ASCII throughput chart (each `█` ≈ 350 ops/s):

```
 1 worker  | █                     362.5
 2 workers | ██                    710.7
 3 workers | ███                  1018.4
 4 workers | ████                 1309.2
 6 workers | █████▌               1954.9
 8 workers | ███████▎             2537.8
12 workers | █████████▊           3449.1
16 workers | ███████████▋         4086.9  ← peak (= core count)
20 workers | ██████████▊          3779.9
24 workers | ██████████▊          3797.2
28 workers | ██████████▌          3707.0
32 workers | ██████████▌          3700.4
```

### 4.3 Analysis

**Throughput grows nearly linearly from 1→8 workers, continues improving to a peak of 11.27× at 16 workers, then plateaus — exactly matching the machine's 16-core count.**

**Near-linear region (1→8 workers):**

Each A\* call operates on its own `RouteContext` scratch space (pre-allocated `g_score`, `f_score`, `h_heap`, `h_pos`, `h_key`, `parent` arrays). Graph edge weights are read as `_Atomic double` loads with no lock. Because routing workers share no mutable state during path search, concurrent workers process independent client requests with minimal coordination overhead. The 3.61× speedup at 4 workers (versus an ideal 4×) and 7.00× at 8 workers (versus ideal 8×) reflect the small serial fraction: task queue locking, TCP receive/send, and JSON parse/format. The `route_repeats=10` flag amplifies the CPU-bound A\* fraction, pushing efficiency close to ideal.

**Approaching the core-count ceiling (8→16 workers):**

The speedup continues climbing — from 7.00× at 8 to 11.27× at 16 — as all 16 physical cores become saturated. Applying Amdahl's law to the peak measurement:

```
S = 1 / (f + (1−f)/N)
11.27 = 1 / (f + (1−f)/16)  →  f ≈ 0.028
```

Only ~2.8% of request processing is serial. This is possible because:
- Routing workers read `current_travel_time` as `_Atomic double` — no lock, no contention with traffic workers.
- Task queue operations (the only lock routing workers hold) are brief and infrequent.
- The 64-bucket striped EMA mutex (`edge_stripe_mu[edge_id % 64]`) means traffic workers virtually never block routing workers (≈1/64 collision probability).

**Plateau and slight regression beyond 16 workers (20→32 workers):**

Above 16 workers performance plateaus at ~3,750–3,800 ops/s and does not improve. This is the hardware ceiling: the Ryzen 7 8845HS has 16 logical processors, so adding more threads does not expose additional parallelism. The slight regression from the 16-worker peak (4,087 → 3,780 ops/s) reflects increased OS scheduling overhead and slightly higher task-queue mutex contention as more threads compete to dequeue work.

**Where parallelism helps most:**

1. **Routing worker pool** (this benchmark's primary lever): each of the `route_workers` threads owns a `RouteContext`; concurrent A\* calls never touch the same scratch arrays. The generation counter (`ctx->gen++`) replaces an O(V) `memset` with a single increment, so spinning up a new search is O(1). With `route_repeats=10`, A\* CPU time dominates — scaling is close to ideal up to the core limit.

2. **Startup precompute:** All routing workers drain the `TASK_PRECOMPUTE` queue concurrently. The 272+ valid (section_i, section_j) hub-to-hub paths are fully independent; precompute wall time scales as ~1/W.

3. **`TICK_ALL` on large fleets:** `handle_tick_all` spawns `N_TICK_WORKERS = 4` threads covering non-overlapping vehicle-slot ranges. Occupancy transitions use `__atomic_fetch_add/sub(RELAXED)` — no locks. On 30,000 active vehicles, tick wall time is reduced by up to 4×.

4. **Batch routing (segment pool):** `handle_batch_section` pushes 2N `SegTask` items onto `seg_q`; 8 persistent segment workers drain the queue concurrently. The `_Atomic int pending` counter avoids a `pthread_join` and eliminates per-batch thread creation overhead.

**Scalability conclusion:**

The server scales effectively with worker count up to the physical core limit. Throughput increases from 362.5 ops/s at 1 worker to 4,086.9 ops/s at 16 workers — an **11.27× improvement** — before plateauing as the hardware is exhausted. The curve follows the expected Amdahl profile: near-linear up to the core count (serial fraction ≈ 2.8%), then a flat ceiling rather than degradation. There is no performance regression from synchronisation: the lock-free `_Atomic double` reads and the striped EMA mutex eliminate the contention bottlenecks that would otherwise appear at high thread counts.
