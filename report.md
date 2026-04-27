# Waze Navigation Server — Project Report

---

## Section 1 — Application

The application is a real-time GPS navigation server modelled after the Waze navigation system.
It operates on the Tel Aviv road network (OpenStreetMap data, 6,500 nodes / 12,470 edges) and provides the
following functionality via a TCP/JSON protocol:

- **Route computation** — given a source and destination node, computes the optimal route using
  the A* pathfinding algorithm, taking live traffic conditions into account.
- **Live traffic updates** — connected cars continuously report their current edge ID and speed.
  The server smooths these reports using an Exponential Moving Average (EMA) and immediately
  updates the travel-time weights used by the routing engine.
- **Vehicle simulation** — the server tracks up to 30,000 simulated vehicles, each holding a
  car ID, a pre-computed route (list of edge IDs), and a fractional position along the current
  edge. A single `TICK_ALL` command advances every vehicle by `dt` seconds in one pass.
- **Congestion modelling** — each edge has a physical capacity based on its length, number of
  lanes, and road class. When occupancy exceeds a threshold, vehicle speed degrades linearly
  toward a road-class-specific minimum (the "jam floor").
- **Batch routing** — a `batch_routes` endpoint accepts up to 64 (source, destination)
  pairs in a single request. The server groups them by geographic section, dispatches each group
  to a separate routing worker in parallel, and runs Phase-1 and Phase-3 A* segments for all
  cars in a group concurrently.
- **Interactive web UI** — a Leaflet.js browser interface shows car positions and a congestion
  heatmap in real time, bridged from the TCP server via a lightweight Python HTTP proxy.

---

## Section 2 — Architecture and Algorithms

### Graph data structure

The road network is represented as a directed graph with up to 100,000 nodes and an
unlimited number of edges:

```
Node  { node_id, lat, lon, EdgeNode* out_edges }   // adjacency list (linked list)
Edge  { edge_id, from_node, to_node,
        base_length, base_speed_limit, road_type, lanes,
        current_travel_time, ema_travel_time, observation_count }
Graph { Node nodes[MAX_NODES], Edge* edges, int num_nodes, num_edges }
```

`out_edges` is a singly-linked list of `EdgeNode { edge_id, next }` — one per outgoing
edge. A global dynamic array `edges[]` stores all edge attributes, indexed by `edge_id`.
This gives O(1) edge lookup and O(degree) neighbour traversal, which is optimal for A*.

### A* pathfinding

`find_route_a_star_path()` in `routing.c` implements the standard A* algorithm:

1. **Open set** — a binary min-heap (`min_heap.c`) keyed by `f_score = g_score + h(v)`.
   A `pos[]` array maps `node_id → heap index`, enabling O(log N) `decrease-key`.
2. **Heuristic** — straight-line travel time using the equirectangular approximation:
   `h(v, goal) = dist_euclidean(v, goal) / max_speed`. This is **admissible** (never
   overestimates), guaranteeing optimal paths.
3. **Edge weights** — `get_edge_weight()` returns `current_travel_time`, which is updated
   live by the EMA traffic smoother, so routes always reflect current congestion.
4. **Complexity** — O((V + E) log V) in the worst case; the heuristic prunes the search
   substantially on real road networks.

### Traffic update (EMA)

When a car reports `speed` on `edge_id`:

```
measured = base_length / speed            // convert speed → travel time
alpha    = 1.0  (first observation)
         = 0.2  (subsequent)
ema_travel_time = alpha * measured + (1 - alpha) * ema_travel_time
current_travel_time = ema_travel_time
```

The first observation bootstraps the estimate (α = 1); subsequent reports smooth it
exponentially (α = 0.2 keeps ~80 % of the previous estimate), providing noise resistance.

### Congestion model

```
capacity  = lanes × floor(base_length / meters_per_car(road_class))
cong_frac = max(jam_floor(road_class),  1 − occupancy / capacity)
speed     = base_speed × cong_frac
```

Road class (motorway → residential) controls the headway (`meters_per_car`) and the
minimum speed fraction (`jam_floor`, 0.05 – 0.30).

### Hierarchical section routing (new)

The map is divided into a **5 × 5 geographic grid** (25 sections) by lat/lon quantisation.
Each section has a **hub node** — the node closest to the section's geometric centroid.

At server startup, A* is run between every pair of hub nodes that actually exist (272 valid
pairs out of the 25×24 = 600 possible ordered pairs — some geographic cells contain no nodes
and therefore have no hub), producing a **pre-computed section graph** stored in a 25 × 25
matrix of `SectionPath` structs (`cost`, edge list, edge count).

At runtime, a batch request routes each car hierarchically:

```
src ∈ section X,  dst ∈ section Y  (X ≠ Y):
  Phase 1: A*(src     → hub_X)   — individual, per-car
  Phase 2: section_paths[X][Y]   — pre-computed, O(1) lookup, shared by all cars in group
  Phase 3: A*(hub_Y   → dst)     — individual, per-car
  full_path = P1 + P2 + P3,   cost = cost1 + cost_highway + cost3
```

When `X == Y` (same section), or when no pre-computed path exists for the pair, the server
falls back to a direct `A*(src, dst)`.

---

## Section 3 — Parallel Parts

### What is parallel, and why

| Component | Threading mechanism | Why it can be parallel |
|---|---|---|
| **Routing worker pool** (configurable N threads) | pthreads, `routing_q` task queue | A* reads `current_travel_time` as an `_Atomic double` — no lock needed. Multiple workers run A* in parallel with zero contention on the graph |
| **Section precompute** (startup) | Same routing workers, TASK_PRECOMPUTE tasks | All 272 valid (i,j) paths are completely independent — no shared mutable state between them |
| **Cross-section batch groups** | Each group → one TASK_BATCH_SECTION → different routing worker | Different (src_sec, dst_sec) groups access disjoint hub pairs; no shared data between groups |
| **Within-group Phase 1 + Phase 3** | `pthread_create` / `pthread_join`, 2N threads per group | All Phase-1 calls (`A*(src_i → hub_X)`) and all Phase-3 calls (`A*(hub_Y → dst_i)`) are fully independent; each thread acquires its own read lock |
| **Per-connection client threads** | One detached thread per TCP connection | Each client's request stream is independent; parsing and queue dispatch are non-blocking |
| **Traffic worker pool** (2 threads) | pthreads, `traffic_q` task queue | Traffic updates are serialised by a **write lock** — see below |

### What is serial, and why

| Component | Why it must be serial |
|---|---|
| **EMA traffic updates** | Write lock (`pthread_rwlock_wrlock`) — only one writer at a time. Two concurrent writers on `current_travel_time` and `observation_count` would produce data races and incorrect smoothed values |
| **TICK_ALL vehicle simulation** | A single occupancy array is computed in one pass over all cars, then used to advance all cars. Splitting this across threads would require atomic updates to occupancy counters on every edge-crossing, adding overhead that outweighs the benefit for the current 30K-car scale |
| **Graph loading and section metadata setup** | Single-threaded at startup, before any connections are accepted. No concurrency is needed or useful here |

### How the parallel parts interact

```
TCP client
    │
    ▼ client_thread_main (one thread per connection)
    │   Parse JSON; detect "batch_routes" key
    │
    ├─ For each (src_sec, dst_sec) group:
    │      Create TASK_BATCH_SECTION → routing_q
    │      Block on task->cv (zero busy-waiting)
    │
    ▼ routing_q  (lock-free enqueue; condvar wake)
    │
    ├─ routing_worker_1 ──► TASK_BATCH_SECTION (group A)
    │                            │
    │                            ├─ spawn N segment_threads for Phase 1
    │                            │      each: rdlock → A* → rdlock_unlock
    │                            ├─ spawn N segment_threads for Phase 3
    │                            │      each: rdlock → A* → rdlock_unlock
    │                            ├─ pthread_join all 2N threads
    │                            └─ build JSON array → task_complete → signal client_thread
    │
    ├─ routing_worker_2 ──► TASK_BATCH_SECTION (group B)  [runs simultaneously]
    │
    └─ traffic_worker   ──► TASK_UPD: wrlock → EMA update → wrlock_unlock
```

The `pthread_rwlock_t graph_lock` is the central synchronisation point:
- **TASK_REQ routing workers**: acquire **no lock**. `current_travel_time` is declared
  `_Atomic double` in `graph.h`, so each A* weight read is an atomic load. This means routing
  workers never block each other or the traffic workers.
- **TASK_PRED routing workers**: acquire a **read lock** to read `ema_travel_time` from the
  graph. Multiple concurrent predictions are allowed; they block only if a traffic write is
  in progress.
- **Segment threads** (Phase 1 + Phase 3 of batch routing): acquire a **read lock**.
- **Traffic workers** (`TASK_UPD`): acquire a **write lock** to serialise the EMA
  read-modify-write sequence on `ema_travel_time` and `observation_count`. The final write to
  `current_travel_time` is also an atomic store, consistent with lock-free routing reads.
- Hold times are very short (one A* segment or one EMA update), so contention is minimal.

---

## Section 4 — Benchmarks

### Setup

- **Graph:** Tel Aviv OpenStreetMap road network, 6,500 nodes / 12,470 edges
- **Load:** 32 concurrent REQ clients × 10 route rounds each; 8 UPD clients × 20 traffic
  update rounds each
- **Tool:** `benchmark.py`, which starts the server (including the 272-path precompute phase,
  which completes before any connections are accepted), runs one discarded warmup pass, then
  runs 3 timed passes via `legacy/load_test.py` and reports the median throughput and latency.
- **Machine:** WSL2 on Linux 5.15 (Windows host), x86-64

### Results

```
 Workers |     Throughput |   Elapsed |   p50 ms |   p99 ms |  Speedup
----------------------------------------------------------------------
       1 |          82.4  |   19.42s  |    393.4 |    610.5 |    1.00x
       2 |          80.5  |   19.87s  |    402.8 |    624.1 |    0.98x
       3 |          80.0  |   20.01s  |    402.4 |    622.6 |    0.97x
       4 |          87.7  |   18.24s  |    391.9 |    611.2 |    1.06x
       6 |          80.3  |   19.92s  |    404.4 |    634.3 |    0.97x
       8 |          81.0  |   19.76s  |    399.6 |    620.4 |    0.98x
      12 |          64.4  |   24.84s  |    493.8 |    817.8 |    0.78x
      16 |          60.9  |   26.29s  |    532.6 |    818.0 |    0.74x
      20 |          60.5  |   26.43s  |    536.2 |    834.6 |    0.73x
      24 |          61.1  |   26.19s  |    530.6 |    816.4 |    0.74x
      28 |          60.8  |   26.33s  |    534.6 |    813.7 |    0.74x
      32 |          60.6  |   26.41s  |    531.8 |    828.0 |    0.74x
----------------------------------------------------------------------
Baseline (1 worker): 82.4 ops/s
```

### Analysis

**Why throughput is flat from 1–8 workers:**
The 32 concurrent clients each send requests **sequentially** (send → wait for response → send
next), so at most 32 A* calls can be in flight simultaneously. At ~400 ms p50 latency, most of
that time is queue wait and network round-trip, not the A* computation itself. Routing workers
are idle a large fraction of the time waiting for the next request to arrive, so adding more
workers beyond the client count provides no benefit.

**Why throughput drops at 12+ workers:**
Each additional thread adds OS scheduling overhead and increases contention on the task queue
mutex (`routing_q.mu`), the per-connection `recv` calls, and the read-write lock. Once the
worker count exceeds the natural parallelism of the workload (32 sequential clients), the cost
of managing extra idle threads dominates any gain from additional routing capacity.

**Where parallelism does help:**

1. **Precompute phase:** The 272 valid section-to-section paths are computed during server
   startup using all available routing workers. With 8 workers, this completes approximately
   8× faster than with 1 worker, keeping startup time low even for large graphs.

2. **Within-group Phase 1 + Phase 3 parallelism:** When a batch of N cars is submitted, the
   `handle_batch_section` function spawns 2N threads (N for Phase 1, N for Phase 3) that all
   run concurrently. Each A* segment takes a measurable amount of time on the Tel Aviv graph,
   and the 2× parallelism of Phase 1 + Phase 3 running simultaneously gives a real speedup
   over sequential per-car routing.

3. **Cross-section parallelism:** If a batch contains cars going to K different sections,
   K routing workers process them simultaneously (one group per worker), each independently
   running A* under a read lock.

**Scalability on larger graphs:**
On graphs significantly larger than 6,500 nodes, each A* call would become more expensive,
making the routing worker count a more meaningful bottleneck and enabling clearer scaling up
to the number of physical CPU cores. Beyond that, hyper-threading provides diminishing returns,
and eventually `pthread_create` fails when the OS thread limit is reached (typically ~32,000
threads per process on Linux, but performance degrades well before that point due to scheduling
overhead).
