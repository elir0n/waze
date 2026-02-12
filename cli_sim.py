import argparse
import csv
import json
import random
import socket
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple


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


def send_json(sock: socket.socket, payload: dict) -> None:
    send_line(sock, json.dumps(payload, separators=(",", ":")))


def recv_json(sock: socket.socket) -> dict:
    raw = recv_line(sock).strip()
    return json.loads(raw)


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


# ---------------- protocol ----------------

def request_route(
    sock: socket.socket,
    user_id: int,
    car_id: int,
    src: int,
    dst: int,
    timestamp: float,
) -> Tuple[bool, Optional[Tuple[float, List[int]]]]:
    req = {
        "user_id": user_id,
        "car_id": car_id,
        "start_node": src,
        "destination_node": dst,
        "timestamp": timestamp,
    }
    send_json(sock, req)
    resp = recv_json(sock)

    if "error" in resp:
        return False, None

    if (
        "user_id" not in resp
        or "car_id" not in resp
        or "route_edges" not in resp
        or "eta" not in resp
    ):
        return False, None

    if resp["user_id"] != user_id or resp["car_id"] != car_id:
        return False, None

    route_edges = [int(eid) for eid in resp["route_edges"]]
    eta = float(resp["eta"])
    return True, (eta, route_edges)


def send_traffic_report(
    sock: socket.socket,
    user_id: int,
    car_id: int,
    timestamp: float,
    edge_id: int,
    position_on_edge: float,
    speed: float,
) -> bool:
    report = {
        "user_id": user_id,
        "car_id": car_id,
        "timestamp": timestamp,
        "edge_id": edge_id,
        "position_on_edge": position_on_edge,
        "speed": speed,
    }
    send_json(sock, report)
    resp = recv_json(sock)
    return resp.get("status") == "ACK"


def request_pred(sock: socket.socket, edge_id: int) -> Tuple[bool, Optional[float]]:
    send_line(sock, f"PRED {edge_id}")
    resp = recv_line(sock).strip()
    parts = resp.split()
    if len(parts) == 3 and parts[0] == "PRED":
        return True, float(parts[2])
    return False, None


# ---------------- simulation ----------------

@dataclass
class Car:
    car_id: int
    user_id: int
    sock: socket.socket
    rng: random.Random
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

    def maybe_start_jam(self, edge_id: int, rnd: random.Random, args: argparse.Namespace) -> None:
        if edge_id in self.jam_remaining:
            return
        occ = self.edge_occupancy.get(edge_id, 0)
        if occ < args.jam_min_cars:
            return
        if rnd.random() > args.jam_prob:
            return
        self.jam_factor[edge_id] = rnd.uniform(args.jam_min_factor, args.jam_max_factor)
        self.jam_remaining[edge_id] = rnd.randint(args.jam_min_steps, args.jam_max_steps)

    def get_factor(self, edge_id: int) -> float:
        return self.jam_factor.get(edge_id, 1.0)

    def tick(self) -> None:
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
        self.edge_occupancy = counts


def pick_dst(rnd: random.Random, num_nodes: int, src: int) -> int:
    if num_nodes <= 1:
        return src
    dst = src
    while dst == src:
        dst = rnd.randrange(0, num_nodes)
    return dst


def maybe_reroute_midway(
    car: Car,
    step: int,
    sim_time: float,
    edges: Dict[int, EdgeInfo],
    args: argparse.Namespace,
) -> None:
    if args.reroute_every_steps <= 0 or (step % args.reroute_every_steps) != 0:
        return
    if car.state != "DRIVING" or car.dst is None:
        return

    current_edge_id = car.current_edge()
    if current_edge_id is None:
        return
    edge = edges.get(current_edge_id)
    if edge is None:
        return

    reroute_src = edge.to_node
    if reroute_src == car.dst:
        return

    ok, route = request_route(car.sock, car.user_id, car.car_id, reroute_src, car.dst, sim_time)
    if not ok or route is None:
        return

    _, new_tail = route
    if not new_tail:
        return

    # Keep current edge, replace the rest so reroute takes effect immediately after this edge.
    car.route_edges = car.route_edges[: car.current_edge_index + 1] + new_tail


def process_car_step(
    car: Car,
    step: int,
    sim_time: float,
    num_nodes: int,
    edges: Dict[int, EdgeInfo],
    jam_state: JamState,
    args: argparse.Namespace,
) -> None:
    rnd = car.rng

    if car.cooldown_steps > 0:
        car.cooldown_steps -= 1

    if car.state in ("WAITING_FOR_ROUTE", "ARRIVED") and car.cooldown_steps == 0:
        src = rnd.randrange(0, max(1, num_nodes))
        dst = pick_dst(rnd, num_nodes, src)
        ok, route = request_route(car.sock, car.user_id, car.car_id, src, dst, sim_time)
        if not ok or route is None:
            car.state = "WAITING_FOR_ROUTE"
            car.cooldown_steps = args.reroute_cooldown_steps
        else:
            eta, edge_ids = route
            car.reset_route(edge_ids, src, dst)
            car.cooldown_steps = args.route_cooldown_steps
            if args.verbose:
                print(f"step {step}: car {car.car_id} route {src}->{dst} eta={eta:.3f} edges={len(edge_ids)}")

    if car.state == "DRIVING":
        edge_id = car.current_edge()
        if edge_id is None:
            car.state = "ARRIVED"
            return

        edge = edges.get(edge_id)
        if edge is None:
            car.state = "WAITING_FOR_ROUTE"
            return

        maybe_reroute_midway(car, step, sim_time, edges, args)

        jam_state.maybe_start_jam(edge_id, rnd, args)
        jam_factor = jam_state.get_factor(edge_id)

        if car.speed_hold_steps <= 0:
            base_speed = edge.speed_limit * rnd.uniform(args.min_speed_factor, args.max_speed_factor)
            car.desired_speed = base_speed
            car.speed_hold_steps = rnd.randint(args.speed_hold_min, args.speed_hold_max)

        speed = max(0.1, car.desired_speed * jam_factor)
        if speed > edge.speed_limit:
            speed = edge.speed_limit
        car.speed = speed

        if edge.length <= 0:
            car.position_on_edge = 1.0
        else:
            advance = (speed * args.dt) / edge.length
            car.position_on_edge += advance

        if args.report_every > 0 and step % args.report_every == 0:
            _ = send_traffic_report(
                car.sock,
                car.user_id,
                car.car_id,
                sim_time,
                edge_id,
                max(0.0, min(1.0, car.position_on_edge)),
                speed,
            )

        while car.position_on_edge >= 1.0 and car.state == "DRIVING":
            car.position_on_edge -= 1.0
            car.current_edge_index += 1
            if car.current_edge_index >= len(car.route_edges):
                car.state = "ARRIVED"
                car.cooldown_steps = args.arrival_cooldown_steps
                car.arrival_step = step
                print(f"step {step}: car {car.car_id} arrived {car.src}->{car.dst}")
                break

    if car.speed_hold_steps > 0:
        car.speed_hold_steps -= 1

    if car.state == "DRIVING":
        car.total_drive_steps += 1
    else:
        car.total_wait_steps += 1


def simulate_loop(args: argparse.Namespace) -> None:
    num_nodes, _ = load_graph_meta(f"{args.graph_dir}/graph.meta")
    edges = load_edges(f"{args.graph_dir}/edges.csv")

    cars: List[Car] = []
    for i in range(args.cars):
        sock = connect(args.host, args.port, args.timeout)
        cars.append(Car(car_id=i, user_id=i, sock=sock, rng=random.Random(args.seed + 1000 + i)))

    jam_state = JamState()
    sim_workers = args.sim_workers if args.sim_workers > 0 else max(1, min(args.cars, 32))

    try:
        with ThreadPoolExecutor(max_workers=sim_workers) as pool:
            for step in range(args.steps):
                sim_time = step * args.dt

                jam_state.update_occupancy(cars)
                jam_state.tick()

                futures = [
                    pool.submit(process_car_step, car, step, sim_time, num_nodes, edges, jam_state, args)
                    for car in cars
                ]
                for f in futures:
                    f.result()

                if args.log_every > 0 and step % args.log_every == 0:
                    driving = sum(1 for c in cars if c.state == "DRIVING")
                    currently_arrived = sum(1 for c in cars if c.state == "ARRIVED")
                    waiting = sum(1 for c in cars if c.state == "WAITING_FOR_ROUTE")
                    print(
                        f"step {step}: driving={driving} currently_arrived={currently_arrived} waiting={waiting}"
                    )

                if args.sleep_ms > 0:
                    time.sleep(args.sleep_ms / 1000.0)
    finally:
        for c in cars:
            try:
                c.sock.close()
            except Exception:
                pass

    ever_arrived = [c for c in cars if c.arrival_step is not None]
    driving = [c for c in cars if c.state == "DRIVING"]
    currently_arrived = [c for c in cars if c.state == "ARRIVED"]
    waiting = [c for c in cars if c.state == "WAITING_FOR_ROUTE"]

    def avg(xs: List[int]) -> float:
        return (sum(xs) / len(xs)) if xs else 0.0

    print("\nSimulation summary:")
    print(
        "cars_total="
        f"{len(cars)} ever_arrived={len(ever_arrived)} currently_arrived={len(currently_arrived)} "
        f"driving={len(driving)} waiting={len(waiting)}"
    )
    print(f"avg_drive_steps={avg([c.total_drive_steps for c in cars]):.2f}")
    print(f"avg_wait_steps={avg([c.total_wait_steps for c in cars]):.2f}")
    if ever_arrived:
        print(f"avg_steps_to_arrive={avg([c.arrival_step for c in ever_arrived]):.2f}")


def interactive_mode(args: argparse.Namespace) -> None:
    num_nodes, _ = load_graph_meta(f"{args.graph_dir}/graph.meta")
    sock = connect(args.host, args.port, args.timeout)
    with sock:
        print(f"Graph nodes: 0..{num_nodes - 1}")
        user_id = 1
        car_id = 1
        while True:
            raw = input("Enter src dst, or 'pred <edge_id>', or 'q': ").strip()
            if raw.lower() in ("q", "quit", "exit"):
                break
            if raw.lower().startswith("pred "):
                parts = raw.split()
                if len(parts) != 2:
                    print("Usage: pred <edge_id>")
                    continue
                try:
                    edge_id = int(parts[1])
                except ValueError:
                    print("Invalid edge_id.")
                    continue
                ok, pred = request_pred(sock, edge_id)
                if not ok or pred is None:
                    print("ERR PRED")
                else:
                    print(f"Predicted travel time: {pred:.3f}")
                continue

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

            ok, route = request_route(sock, user_id, car_id, src, dst, time.time())
            if not ok or route is None:
                print("ERR NO_ROUTE")
                continue
            eta, edge_ids = route
            print(f"ETA: {eta:.3f}")
            print("Edges:", " ".join(map(str, edge_ids)))


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

    ap.add_argument("--sim-workers", type=int, default=8)
    ap.add_argument("--reroute-every-steps", type=int, default=5)

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
