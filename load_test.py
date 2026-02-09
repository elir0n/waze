#!/usr/bin/env python3
import argparse
import random
import socket
import threading
import time
from dataclasses import dataclass
from typing import Optional, Tuple, List

# --------- helpers ---------

def connect(host: str, port: int, timeout: float) -> socket.socket:
    s = socket.create_connection((host, port), timeout=timeout)
    s.settimeout(timeout)
    return s

def send_line(sock: socket.socket, line: str) -> None:
    if not line.endswith("\n"):
        line += "\n"
    sock.sendall(line.encode("utf-8"))

def recv_line(sock: socket.socket) -> str:
    # Read until '\n'
    chunks = []
    while True:
        b = sock.recv(1)
        if not b:
            raise ConnectionError("Server closed connection")
        chunks.append(b)
        if b == b"\n":
            break
    return b"".join(chunks).decode("utf-8", errors="replace")

def parse_route(resp: str) -> Tuple[float, int, List[int]]:
    parts = resp.strip().split()
    if len(parts) < 3:
        raise ValueError(f"Not a ROUTE response: {resp!r}")

    if parts[0] == "ROUTE":
        cost = float(parts[1])
        edge_count = int(parts[2])
        edge_ids = [int(x) for x in parts[3:]]
    elif parts[0] == "ROUTE2":
        cost = float(parts[1])
        node_count = int(parts[2])
        idx_edges = 3 + node_count
        if idx_edges >= len(parts):
            raise ValueError(f"ROUTE2 missing edge_count: {resp!r}")
        edge_count = int(parts[idx_edges])
        edge_ids = [int(x) for x in parts[idx_edges + 1 :]]
    else:
        raise ValueError(f"Not a ROUTE response: {resp!r}")

    if len(edge_ids) != edge_count:
        raise ValueError(f"edge_count mismatch: declared={edge_count} got={len(edge_ids)} resp={resp!r}")
    return cost, edge_count, edge_ids

# --------- stats ---------

@dataclass
class Stats:
    ok: int = 0
    err: int = 0
    timeouts: int = 0
    other_fail: int = 0
    latencies_ms: Optional[List[float]] = None

    def __post_init__(self):
        if self.latencies_ms is None:
            self.latencies_ms = []

def percentile(xs: List[float], p: float) -> float:
    if not xs:
        return float("nan")
    xs_sorted = sorted(xs)
    k = int(round((p / 100.0) * (len(xs_sorted) - 1)))
    return xs_sorted[max(0, min(k, len(xs_sorted) - 1))]

# --------- worker logic ---------

def req_worker(
    host: str,
    port: int,
    timeout: float,
    num_nodes: int,
    rounds: int,
    think_ms: int,
    stats: Stats,
    seed: int
) -> None:
    rnd = random.Random(seed)
    try:
        sock = connect(host, port, timeout)
    except Exception:
        stats.other_fail += 1
        return

    with sock:
        for _ in range(rounds):
            src = rnd.randrange(0, max(1, num_nodes))
            dst = rnd.randrange(0, max(1, num_nodes))
            t0 = time.perf_counter()
            try:
                send_line(sock, f"REQ {src} {dst}")
                resp = recv_line(sock)
                t1 = time.perf_counter()
                stats.latencies_ms.append((t1 - t0) * 1000.0)

                if resp.startswith(("ROUTE ", "ROUTE2 ")):
                    # Validate structure
                    parse_route(resp)
                    stats.ok += 1
                elif resp.startswith("ERR "):
                    # ERR NO_ROUTE is valid depending on graph connectivity
                    stats.err += 1
                else:
                    stats.other_fail += 1
            except socket.timeout:
                stats.timeouts += 1
            except Exception:
                stats.other_fail += 1

            if think_ms > 0:
                time.sleep(think_ms / 1000.0)

def upd_worker(
    host: str,
    port: int,
    timeout: float,
    num_edges: int,
    rounds: int,
    think_ms: int,
    stats: Stats,
    seed: int
) -> None:
    rnd = random.Random(seed)
    try:
        sock = connect(host, port, timeout)
    except Exception:
        stats.other_fail += 1
        return

    with sock:
        for _ in range(rounds):
            if num_edges <= 0:
                stats.err += 1
                return
            edge_id = rnd.randrange(0, num_edges)
            # Speed in some reasonable range; you can change
            speed = rnd.uniform(1.0, 30.0)

            t0 = time.perf_counter()
            try:
                send_line(sock, f"UPD {edge_id} {speed:.3f}")
                resp = recv_line(sock)
                t1 = time.perf_counter()
                stats.latencies_ms.append((t1 - t0) * 1000.0)

                if resp.strip() == "ACK":
                    stats.ok += 1
                elif resp.startswith("ERR "):
                    stats.err += 1
                else:
                    stats.other_fail += 1
            except socket.timeout:
                stats.timeouts += 1
            except Exception:
                stats.other_fail += 1

            if think_ms > 0:
                time.sleep(think_ms / 1000.0)

# --------- main ---------

def main():
    ap = argparse.ArgumentParser(description="Load test for your Waze server (REQ/UPD, parallel).")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--timeout", type=float, default=3.0)

    ap.add_argument("--num-nodes", type=int, required=True, help="Graph num_nodes (from graph.meta).")
    ap.add_argument("--num-edges", type=int, required=True, help="Graph num_edges (from graph.meta).")

    ap.add_argument("--req-clients", type=int, default=32, help="Concurrent REQ clients.")
    ap.add_argument("--upd-clients", type=int, default=8, help="Concurrent UPD clients.")

    ap.add_argument("--req-rounds", type=int, default=50, help="REQ commands per REQ client.")
    ap.add_argument("--upd-rounds", type=int, default=100, help="UPD commands per UPD client.")

    ap.add_argument("--think-ms", type=int, default=0, help="Sleep between commands per client.")
    ap.add_argument("--seed", type=int, default=1)

    args = ap.parse_args()

    print(f"Target: {args.host}:{args.port}")
    print(f"Clients: REQ={args.req_clients} (x{args.req_rounds}), UPD={args.upd_clients} (x{args.upd_rounds})")
    print(f"Graph: nodes={args.num_nodes}, edges={args.num_edges}")

    req_stats = Stats()
    upd_stats = Stats()

    threads: List[threading.Thread] = []

    start = time.perf_counter()

    for i in range(args.req_clients):
        t = threading.Thread(
            target=req_worker,
            args=(args.host, args.port, args.timeout, args.num_nodes, args.req_rounds, args.think_ms, req_stats, args.seed + 1000 + i),
            daemon=True,
        )
        threads.append(t)

    for i in range(args.upd_clients):
        t = threading.Thread(
            target=upd_worker,
            args=(args.host, args.port, args.timeout, args.num_edges, args.upd_rounds, args.think_ms, upd_stats, args.seed + 2000 + i),
            daemon=True,
        )
        threads.append(t)

    for t in threads:
        t.start()
    for t in threads:
        t.join()

    end = time.perf_counter()
    elapsed = end - start

    def report(name: str, s: Stats):
        total = s.ok + s.err + s.timeouts + s.other_fail
        lats = s.latencies_ms
        print(f"\n[{name}] total={total} ok={s.ok} err={s.err} timeouts={s.timeouts} other_fail={s.other_fail}")
        if lats:
            print(f"[{name}] latency ms: p50={percentile(lats,50):.2f} p90={percentile(lats,90):.2f} p99={percentile(lats,99):.2f} max={max(lats):.2f}")
            print(f"[{name}] throughput: {total/elapsed:.2f} ops/sec (elapsed {elapsed:.2f}s)")

    report("REQ", req_stats)
    report("UPD", upd_stats)

    # Simple pass/fail heuristics
    if req_stats.other_fail or upd_stats.other_fail:
        raise SystemExit("FAIL: Some operations had unexpected failures (other_fail > 0).")
    if req_stats.timeouts or upd_stats.timeouts:
        raise SystemExit("WARN: Some operations timed out; server might be overloaded or deadlocked.")
    print("\nPASS: Completed parallel REQ/UPD load with no unexpected failures.")

if __name__ == "__main__":
    main()
