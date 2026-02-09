#!/usr/bin/env python3
import argparse
import csv
import random
import socket
import time
import threading
from dataclasses import dataclass, field
from typing import Dict, List, Tuple, Optional

# ---------------- network helpers ----------------

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

# ---------------- graph loading ----------------

def load_graph_meta(path: str) -> Tuple[int, int]:
    num_nodes = None
    num_edges = None
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) != 2:
                continue
            if parts[0] == "num_nodes":
                num_nodes = int(parts[1])
            elif parts[0] == "num_edges":
                num_edges = int(parts[1])
    if num_nodes is None or num_edges is None:
        raise ValueError(f"Invalid graph.meta: {path}")
    return num_nodes, num_edges

@dataclass
class EdgeInfo:
    edge_id: int
    from_node: int
    to_node: int
    length: float
    speed_limit: float


def load_edges(path: str) -> Dict[int, EdgeInfo]:
    edges: Dict[int, EdgeInfo] = {}
    with open(path, "r", encoding="utf-8") as f:
        r = csv.DictReader(f)
        for row in r:
            edge_id = int(row["edge_id"])
            edges[edge_id] = EdgeInfo(
                edge_id=edge_id,
                from_node=int(row["from_node"]),
                to_node=int(row["to_node"]),
                length=float(row["base_length"]),
                speed_limit=float(row["base_speed_limit"]),
            )
    return edges

# ---------------- protocol parsing ----------------

def parse_route(resp: str) -> Tuple[float, List[int], List[int]]:
    parts = resp.strip().split()
    if len(parts) < 3:
        raise ValueError(f"Not a ROUTE response: {resp!r}")

    if parts[0] == "ROUTE2":
        cost = float(parts[1])
        node_count = int(parts[2])
        nodes = [int(x) for x in parts[3:3 + node_count]]
        idx_edges = 3 + node_count
        if idx_edges >= len(parts):
            raise ValueError(f"ROUTE2 missing edge_count: {resp!r}")
        edge_count = int(parts[idx_edges])
        edges = [int(x) for x in parts[idx_edges + 1 :]]
        if len(edges) != edge_count:
            raise ValueError("edge_count mismatch")
        return cost, nodes, edges

    if parts[0] == "ROUTE":
        cost = float(parts[1])
        edge_count = int(parts[2])
        edges = [int(x) for x in parts[3:]]
        if len(edges) != edge_count:
            raise ValueError("edge_count mismatch")
        return cost, [], edges

    raise ValueError(f"Not a ROUTE response: {resp!r}")

# ---------------- simulation ----------------

@dataclass
class Car:
    car_id: int
    user_id: int
    state: str = "WAITING_FOR_ROUTE"
    route_edges: List[int] = field(default_factory=list)
    current_edge_index: int = 0
    position_on_edge: float = 0.0
    speed: float = 0.0
    desired_speed: float = 0.0
    speed_hold_steps: int = 0
    cooldown_steps: int = 0
    total_wait_steps: int = 0
    total_drive_steps: int = 0
    arrival_step: Optional[int] = None
    src: Optional[int] = None
    dst: Optional[int] = None

    def reset_route(self, edges: List[int], src: int, dst: int) -> None:
        self.route_edges = edges
        self.current_edge_index = 0
        self.position_on_edge = 0.0
        self.state = "DRIVING" if edges else "WAITING_FOR_ROUTE"
        self.src = src
        self.dst = dst

    def current_edge(self) -> Optional[int]:
        if self.current_edge_index >= len(self.route_edges):
            return None
        return self.route_edges[self.current_edge_index]


@dataclass
class JamState:
    jam_factor: Dict[int, float] = field(default_factory=dict)
    jam_remaining: Dict[int, int] = field(default_factory=dict)
    edge_occupancy: Dict[int, int] = field(default_factory=dict)
    lock: threading.Lock = field(default_factory=threading.Lock)

    def maybe_start_jam(self, edge_id: int, rnd: random.Random, args: argparse.Namespace) -> None:
        with self.lock:
            if edge_id in self.jam_remaining:
                return
            occ = self.edge_occupancy.get(edge_id, 0)
            if occ < args.jam_min_cars:
                return
            if rnd.random() > args.jam_prob:
                return
            factor = rnd.uniform(args.jam_min_factor, args.jam_max_factor)
            steps = rnd.randint(args.jam_min_steps, args.jam_max_steps)
            self.jam_factor[edge_id] = factor
            self.jam_remaining[edge_id] = steps

    def get_factor(self, edge_id: int) -> float:
        with self.lock:
            return self.jam_factor.get(edge_id, 1.0)

    def tick(self) -> None:
        with self.lock:
            if not self.jam_remaining:
                return
            to_clear = []
            for edge_id in list(self.jam_remaining.keys()):
                self.jam_remaining[edge_id] -= 1
                if self.jam_remaining[edge_id] <= 0:
                    to_clear.append(edge_id)
            for edge_id in to_clear:
                self.jam_remaining.pop(edge_id, None)
                self.jam_factor.pop(edge_id, None)

    def update_occupancy(self, cars: List[Car]) -> None:
        counts: Dict[int, int] = {}
        for c in cars:
            if c.state != "DRIVING":
                continue
            eid = c.current_edge()
            if eid is None:
                continue
            counts[eid] = counts.get(eid, 0) + 1
        with self.lock:
            self.edge_occupancy = counts


def pick_dst(rnd: random.Random, num_nodes: int, src: int) -> int:
    if num_nodes <= 1:
        return src
    dst = src
    while dst == src:
        dst = rnd.randrange(0, num_nodes)
    return dst


def request_route(sock: socket.socket, src: int, dst: int) -> Tuple[bool, Optional[Tuple[float, List[int], List[int]]]]:
    send_line(sock, f"REQ {src} {dst}")
    resp = recv_line(sock)
    if resp.startswith("ERR "):
        return False, None
    return True, parse_route(resp)


def send_update(sock: socket.socket, edge_id: int, speed: float) -> bool:
    send_line(sock, f"UPD {edge_id} {speed:.3f}")
    resp = recv_line(sock)
    return resp.strip() == "ACK"

def send_update_with_pos(sock: socket.socket, edge_id: int, speed: float, pos: float) -> bool:
    send_line(sock, f"UPD {edge_id} {speed:.3f} {pos:.3f}")
    resp = recv_line(sock)
    return resp.strip() == "ACK"


def simulate_loop(args: argparse.Namespace) -> None:
    num_nodes, _ = load_graph_meta(f"{args.graph_dir}/graph.meta")
    edges = load_edges(f"{args.graph_dir}/edges.csv")

    cars: List[Car] = [Car(car_id=i, user_id=i) for i in range(args.cars)]
    jam_state = JamState()
    error_flag = {"error": None}
    step_counter = {"step": 0}

    def barrier_action() -> None:
        jam_state.update_occupancy(cars)
        jam_state.tick()
        step_counter["step"] += 1
        step = step_counter["step"]
        if args.log_every > 0 and step % args.log_every == 0:
            driving = sum(1 for c in cars if c.state == "DRIVING")
            arrived = sum(1 for c in cars if c.state == "ARRIVED")
            waiting = sum(1 for c in cars if c.state == "WAITING_FOR_ROUTE")
            print(f"step {step}: driving={driving} arrived={arrived} waiting={waiting}")
        if args.sleep_ms > 0:
            time.sleep(args.sleep_ms / 1000.0)

    barrier = threading.Barrier(args.cars, action=barrier_action)

    def car_worker(car: Car) -> None:
        rnd = random.Random(args.seed + 1000 + car.car_id)
        try:
            sock = connect(args.host, args.port, args.timeout)
        except Exception as e:
            error_flag["error"] = f"connect failed for car {car.car_id}: {e}"
            barrier.abort()
            return

        with sock:
            for _ in range(args.steps):
                if car.cooldown_steps > 0:
                    car.cooldown_steps -= 1

                if car.state in ("WAITING_FOR_ROUTE", "ARRIVED") and car.cooldown_steps == 0:
                    src = rnd.randrange(0, max(1, num_nodes))
                    dst = pick_dst(rnd, num_nodes, src)
                    ok, route = request_route(sock, src, dst)
                    if not ok or route is None:
                        car.state = "WAITING_FOR_ROUTE"
                        car.cooldown_steps = args.reroute_cooldown_steps
                    else:
                        cost, _, edge_ids = route
                        car.reset_route(edge_ids, src, dst)
                        car.cooldown_steps = args.route_cooldown_steps
                        if args.verbose:
                            step = step_counter["step"]
                            print(f"step {step}: car {car.car_id} route {src}->{dst} eta={cost:.3f} edges={len(edge_ids)}")

                if car.state == "DRIVING":
                    edge_id = car.current_edge()
                    if edge_id is None:
                        car.state = "ARRIVED"
                    else:
                        e = edges.get(edge_id)
                        if not e:
                            car.state = "WAITING_FOR_ROUTE"
                        else:
                            jam_state.maybe_start_jam(edge_id, rnd, args)
                            jam_factor = jam_state.get_factor(edge_id)

                            if car.speed_hold_steps <= 0:
                                base_speed = e.speed_limit * rnd.uniform(args.min_speed_factor, args.max_speed_factor)
                                car.desired_speed = base_speed
                                car.speed_hold_steps = rnd.randint(args.speed_hold_min, args.speed_hold_max)
                            speed = max(0.1, car.desired_speed * jam_factor)
                            if speed > e.speed_limit:
                                speed = e.speed_limit
                            car.speed = speed

                            if e.length <= 0:
                                car.position_on_edge = 1.0
                            else:
                                advance = (speed * args.dt) / e.length
                                car.position_on_edge += advance

                            step = step_counter["step"]
                            if args.report_every > 0 and step % args.report_every == 0:
                                send_update_with_pos(sock, edge_id, speed, car.position_on_edge)

                            while car.position_on_edge >= 1.0 and car.state == "DRIVING":
                                car.position_on_edge -= 1.0
                                car.current_edge_index += 1
                                if car.current_edge_index >= len(car.route_edges):
                                    car.state = "ARRIVED"
                                    car.cooldown_steps = args.arrival_cooldown_steps
                                    car.arrival_step = step
                                    print(f"step {step}: car {car.car_id} arrived {car.src}->{car.dst}")
                                    break

                try:
                    barrier.wait()
                except threading.BrokenBarrierError:
                    return

                if car.speed_hold_steps > 0:
                    car.speed_hold_steps -= 1

                if car.state == "DRIVING":
                    car.total_drive_steps += 1
                else:
                    car.total_wait_steps += 1

    threads: List[threading.Thread] = []
    for car in cars:
        t = threading.Thread(target=car_worker, args=(car,), daemon=True)
        threads.append(t)
        t.start()

    for t in threads:
        t.join()

    if error_flag["error"]:
        raise SystemExit(error_flag["error"])

    arrived = [c for c in cars if c.arrival_step is not None]
    driving = [c for c in cars if c.state == "DRIVING"]
    waiting = [c for c in cars if c.state != "DRIVING" and c.arrival_step is None]

    def avg(xs: List[int]) -> float:
        return (sum(xs) / len(xs)) if xs else 0.0

    print("\nSimulation summary:")
    print(f"cars_total={len(cars)} arrived={len(arrived)} driving={len(driving)} waiting={len(waiting)}")
    print(f"avg_drive_steps={avg([c.total_drive_steps for c in cars]):.2f}")
    print(f"avg_wait_steps={avg([c.total_wait_steps for c in cars]):.2f}")
    if arrived:
        print(f"avg_steps_to_arrive={avg([c.arrival_step for c in arrived]):.2f}")


def interactive_mode(args: argparse.Namespace) -> None:
    num_nodes, _ = load_graph_meta(f"{args.graph_dir}/graph.meta")
    sock = connect(args.host, args.port, args.timeout)
    with sock:
        print(f"Graph nodes: 0..{num_nodes - 1}")
        while True:
            raw = input("Enter src dst (or 'q'): ").strip()
            if raw.lower() in ("q", "quit", "exit"):
                break
            parts = raw.split()
            if len(parts) != 2:
                print("Please enter two integers: <src> <dst>")
                continue
            try:
                src = int(parts[0])
                dst = int(parts[1])
            except ValueError:
                print("Invalid integers.")
                continue

            try:
                ok, route = request_route(sock, src, dst)
                if not ok or route is None:
                    print("ERR NO_ROUTE")
                    continue
                cost, nodes, edges = route
                print(f"ETA: {cost:.3f}")
                if nodes:
                    print("Nodes:", " ".join(map(str, nodes)))
                print("Edges:", " ".join(map(str, edges)))
            except Exception as e:
                print(f"Request failed: {e}")


def main() -> None:
    ap = argparse.ArgumentParser(description="CLI client + simulation loop for Waze server")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--timeout", type=float, default=3.0)
    ap.add_argument("--graph-dir", default="data")

    ap.add_argument("--mode", choices=["sim", "interactive"], default="sim")

    ap.add_argument("--cars", type=int, default=10)
    ap.add_argument("--steps", type=int, default=200)
    ap.add_argument("--dt", type=float, default=1.0)
    ap.add_argument("--report-every", type=int, default=5)
    ap.add_argument("--sleep-ms", type=int, default=0)
    ap.add_argument("--log-every", type=int, default=10)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--verbose", action="store_true")

    ap.add_argument("--min-speed-factor", type=float, default=0.4)
    ap.add_argument("--max-speed-factor", type=float, default=1.0)
    ap.add_argument("--jam-prob", type=float, default=0.02)
    ap.add_argument("--jam-min-factor", type=float, default=0.2)
    ap.add_argument("--jam-max-factor", type=float, default=0.6)
    ap.add_argument("--jam-min-steps", type=int, default=5)
    ap.add_argument("--jam-max-steps", type=int, default=20)
    ap.add_argument("--jam-min-cars", type=int, default=3)
    ap.add_argument("--speed-hold-min", type=int, default=3)
    ap.add_argument("--speed-hold-max", type=int, default=10)
    ap.add_argument("--route-cooldown-steps", type=int, default=0)
    ap.add_argument("--reroute-cooldown-steps", type=int, default=3)
    ap.add_argument("--arrival-cooldown-steps", type=int, default=5)

    args = ap.parse_args()

    if args.mode == "interactive":
        interactive_mode(args)
    else:
        simulate_loop(args)

if __name__ == "__main__":
    main()
