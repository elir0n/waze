#!/usr/bin/env python3
"""
Scalability benchmark: start server with 1, 2, 3, ... routing workers,
run load_test.py against each, collect throughput/latency, print table.

Strategy: for each worker count, start the server once, run one warmup
pass (discarded), then run --runs timed passes against the same server
and report the median. This avoids cold-cache bias from restarting the
server between every measurement.
"""

import argparse
import os
import re
import socket
import subprocess
import sys
import tempfile
import time


# ── helpers ──────────────────────────────────────────────────────────────────

def read_graph_meta(path="data/graph.meta"):
    num_nodes = num_edges = 0
    try:
        with open(path) as f:
            for line in f:
                m = re.match(r"num_nodes\s+(\d+)", line)
                if m:
                    num_nodes = int(m.group(1))
                m = re.match(r"num_edges\s+(\d+)", line)
                if m:
                    num_edges = int(m.group(1))
    except FileNotFoundError:
        print(f"[benchmark] ERROR: {path} not found. Generate a graph first.", file=sys.stderr)
        sys.exit(1)
    return num_nodes, num_edges


def wait_for_port(host, port, timeout=10.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            with socket.create_connection((host, port), timeout=0.3):
                return True
        except OSError:
            time.sleep(0.1)
    return False


def start_server(workers, port=8080):
    log = tempfile.NamedTemporaryFile(mode='w', suffix='.log', delete=False)
    proc = subprocess.Popen(
        ["./server", "--workers", str(workers), "--port", str(port)],
        stdout=subprocess.DEVNULL,
        stderr=log,
    )
    proc._log_path = log.name
    log.close()
    return proc


def run_load_test(num_nodes, num_edges, port=8080,
                  req_clients=32, req_rounds=10,
                  upd_clients=8,  upd_rounds=20,
                  timeout=180.0):
    cmd = [
        sys.executable, "legacy/load_test.py",
        "--host", "127.0.0.1",
        "--port", str(port),
        "--num-nodes", str(num_nodes),
        "--num-edges", str(num_edges),
        "--req-clients", str(req_clients),
        "--req-rounds",  str(req_rounds),
        "--upd-clients", str(upd_clients),
        "--upd-rounds",  str(upd_rounds),
        "--timeout", "5.0",
    ]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return result.stdout + result.stderr
    except subprocess.TimeoutExpired:
        return None


def parse_load_test_output(out):
    """Extract throughput, latency from load_test.py stdout."""
    throughput = p50 = p90 = p99 = elapsed = None
    for line in out.splitlines():
        m = re.search(r"throughput:\s+([\d.]+)\s+ops/sec\s+\(elapsed\s+([\d.]+)s\)", line)
        if m and "[REQ]" in line:
            throughput = float(m.group(1))
            elapsed    = float(m.group(2))
        m = re.search(r"latency ms:\s+p50=([\d.]+).*?p90=([\d.]+).*?p99=([\d.]+)", line)
        if m and "[REQ]" in line:
            p50 = float(m.group(1))
            p90 = float(m.group(2))
            p99 = float(m.group(3))
    return throughput, elapsed, p50, p90, p99


# ── main ─────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="Waze server scalability benchmark")
    ap.add_argument("--port",        type=int, default=8080)
    ap.add_argument("--req-clients", type=int, default=32)
    ap.add_argument("--req-rounds",  type=int, default=10)
    ap.add_argument("--upd-clients", type=int, default=8)
    ap.add_argument("--upd-rounds",  type=int, default=20)
    ap.add_argument("--max-workers", type=int, default=32)
    ap.add_argument("--runs",        type=int, default=3,
                    help="Timed runs per worker count after one warmup (median reported)")
    ap.add_argument("--startup-timeout", type=float, default=60.0,
                    help="Seconds to wait for server to start (longer due to precompute)")
    args = ap.parse_args()

    num_nodes, num_edges = read_graph_meta()
    print(f"Graph: {num_nodes} nodes, {num_edges} edges")
    print(f"Load: {args.req_clients} REQ clients × {args.req_rounds} rounds, "
          f"{args.upd_clients} UPD clients × {args.upd_rounds} rounds  "
          f"(1 warmup + median of {args.runs} timed runs per worker count)\n")

    worker_counts = []
    n = 1
    while n <= args.max_workers:
        worker_counts.append(n)
        if n < 4:
            n += 1
        elif n < 8:
            n += 2
        else:
            n += 4

    results = []
    baseline_tput = None

    header = f"{'Workers':>8} | {'Throughput':>14} | {'Elapsed':>9} | {'p50 ms':>8} | {'p99 ms':>8} | {'Speedup':>8}"
    sep    = "-" * len(header)
    print(header)
    print(sep)

    for w in worker_counts:
        # Start server once for this worker count
        proc = start_server(w, args.port)
        ready = wait_for_port("127.0.0.1", args.port, timeout=args.startup_timeout)

        if not ready:
            print(f"{'':>8}  Server with {w} workers failed to start in "
                  f"{args.startup_timeout}s — stopping.")
            proc.terminate()
            proc.wait()
            try:
                with open(proc._log_path) as f:
                    print(f"[server stderr]\n{f.read()}", file=sys.stderr)
                os.unlink(proc._log_path)
            except OSError:
                pass
            break

        # Warmup run — discarded
        run_load_test(num_nodes, num_edges, args.port,
                      args.req_clients, args.req_rounds,
                      args.upd_clients, args.upd_rounds)

        # Timed runs against the warm server
        run_tputs, run_elapsed, run_p50, run_p99 = [], [], [], []
        failed = False

        for _ in range(args.runs):
            out = run_load_test(num_nodes, num_edges, args.port,
                                args.req_clients, args.req_rounds,
                                args.upd_clients, args.upd_rounds)
            if out is None:
                failed = True
                break

            tput, elapsed, p50, p90, p99 = parse_load_test_output(out)
            if tput is None:
                failed = True
                break

            run_tputs.append(tput)
            run_elapsed.append(elapsed)
            run_p50.append(p50)
            run_p99.append(p99)

        proc.terminate()
        proc.wait()
        try:
            os.unlink(proc._log_path)
        except OSError:
            pass

        if failed or not run_tputs:
            print(f"{w:>8} | {'FAILED':>14}")
            break

        # Report median across timed runs
        run_tputs.sort();  run_elapsed.sort(); run_p50.sort(); run_p99.sort()
        mid = len(run_tputs) // 2
        tput    = run_tputs[mid]
        elapsed = run_elapsed[mid]
        p50     = run_p50[mid]
        p99     = run_p99[mid]

        if baseline_tput is None:
            baseline_tput = tput

        speedup = tput / baseline_tput if baseline_tput else 1.0
        results.append((w, tput, elapsed, p50, p99, speedup))

        print(f"{w:>8} | {tput:>12.1f}   | {elapsed:>7.2f}s | {p50:>8.1f} | {p99:>8.1f} | {speedup:>7.2f}x")

    print(sep)
    print(f"\nBaseline (1 worker) throughput: {baseline_tput:.1f} ops/s" if baseline_tput else "")
    return results


if __name__ == "__main__":
    main()
