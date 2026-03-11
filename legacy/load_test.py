import argparse
import json
import random
import socket
import threading
import time
from dataclasses import dataclass
from typing import List, Optional


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
    chunks = []
    while True:
        b = sock.recv(1)
        if not b:
            raise ConnectionError("Server closed connection")
        chunks.append(b)
        if b == b"\n":
            break
    return b"".join(chunks).decode("utf-8", errors="replace")


def send_json(sock: socket.socket, payload: dict) -> None:
    send_line(sock, json.dumps(payload, separators=(",", ":")))


def recv_json(sock: socket.socket) -> dict:
    return json.loads(recv_line(sock).strip())


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
    seed: int,
) -> None:
    rnd = random.Random(seed)
    try:
        sock = connect(host, port, timeout)
    except Exception:
        stats.other_fail += 1
        return

    user_id = seed
    car_id = seed

    with sock:
        for i in range(rounds):
            src = rnd.randrange(0, max(1, num_nodes))
            dst = rnd.randrange(0, max(1, num_nodes))
            req = {
                "user_id": user_id,
                "car_id": car_id,
                "start_node": src,
                "destination_node": dst,
                "timestamp": float(i),
            }

            t0 = time.perf_counter()
            try:
                send_json(sock, req)
                resp = recv_json(sock)
                t1 = time.perf_counter()
                stats.latencies_ms.append((t1 - t0) * 1000.0)

                if "error" in resp:
                    stats.err += 1
                elif (
                    resp.get("user_id") == user_id
                    and resp.get("car_id") == car_id
                    and isinstance(resp.get("route_edges"), list)
                    and "eta" in resp
                ):
                    stats.ok += 1
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
    seed: int,
) -> None:
    rnd = random.Random(seed)
    try:
        sock = connect(host, port, timeout)
    except Exception:
        stats.other_fail += 1
        return

    user_id = seed
    car_id = seed

    with sock:
        for i in range(rounds):
            if num_edges <= 0:
                stats.err += 1
                return
            edge_id = rnd.randrange(0, num_edges)
            speed = rnd.uniform(1.0, 30.0)
            pos = rnd.uniform(0.0, 1.0)

            report = {
                "user_id": user_id,
                "car_id": car_id,
                "timestamp": float(i),
                "edge_id": edge_id,
                "position_on_edge": pos,
                "speed": speed,
            }

            t0 = time.perf_counter()
            try:
                send_json(sock, report)
                resp = recv_json(sock)
                t1 = time.perf_counter()
                stats.latencies_ms.append((t1 - t0) * 1000.0)

                if resp.get("status") == "ACK":
                    stats.ok += 1
                elif "error" in resp:
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

def main() -> None:
    ap = argparse.ArgumentParser(description="Load test for your Waze server (JSON route/traffic protocol, parallel).")
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

    if req_stats.other_fail or upd_stats.other_fail:
        raise SystemExit("FAIL: Some operations had unexpected failures (other_fail > 0).")
    if req_stats.timeouts or upd_stats.timeouts:
        raise SystemExit("WARN: Some operations timed out; server might be overloaded or deadlocked.")
    print("\nPASS: Completed parallel REQ/UPD load with no unexpected failures.")


if __name__ == "__main__":
    main()
