import argparse
import json
import random
import socket
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
from typing import Optional, Tuple


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


# ---------------- protocol ----------------

def request_route(
    sock: socket.socket,
    user_id: int,
    car_id: int,
    src: int,
    dst: int,
    timestamp: float,
) -> Tuple[bool, Optional[Tuple[float, list]]]:
    """Ask the server for a route (legacy TASK_REQ). Used by interactive mode."""
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
    if not all(k in resp for k in ("user_id", "car_id", "route_edges", "eta")):
        return False, None
    if resp["user_id"] != user_id or resp["car_id"] != car_id:
        return False, None

    route_edges = [int(eid) for eid in resp["route_edges"]]
    eta = float(resp["eta"])
    return True, (eta, route_edges)


def request_pred(sock: socket.socket, edge_id: int) -> Tuple[bool, Optional[float]]:
    """Ask the server for the predicted travel time of a given edge."""
    send_line(sock, f"PRED {edge_id}")
    resp = recv_line(sock).strip()
    parts = resp.split()
    if len(parts) == 3 and parts[0] == "PRED":
        return True, float(parts[2])
    return False, None


def register_car(
    sock: socket.socket,
    user_id: int,
    car_id: int,
    src: int,
    dst: int,
) -> Tuple[bool, Optional[dict]]:
    """Register a car with the server; server computes route and becomes authority."""
    req = {
        "register_car": 1,
        "user_id": user_id,
        "car_id": car_id,
        "start_node": src,
        "destination_node": dst,
        "timestamp": 0.0,
    }
    send_json(sock, req)
    resp = recv_json(sock)
    if "error" in resp or resp.get("status") != "REGISTERED":
        return False, None
    return True, resp


def tick_car(sock: socket.socket, car_id: int, dt: float) -> dict:
    """Ask the server to advance this car by dt seconds. Returns movement command."""
    req = {"tick_car": 1, "car_id": car_id, "dt": dt}
    send_json(sock, req)
    return recv_json(sock)


# ---------------- simulation ----------------

@dataclass
class Car:
    car_id:  int
    user_id: int
    sock:    socket.socket
    rng:     random.Random
    state:   str = "WAITING"   # WAITING | DRIVING | ARRIVED
    arrivals: int = 0


def pick_dst(rnd: random.Random, num_nodes: int, src: int) -> int:
    if num_nodes <= 1:
        return src
    dst = src
    while dst == src:
        dst = rnd.randrange(0, num_nodes)
    return dst


def _reconnect(car: Car, args: argparse.Namespace) -> bool:
    """Close the current socket and open a new one. Returns True on success."""
    try:
        car.sock.close()
    except Exception:
        pass
    try:
        car.sock = connect(args.host, args.port, args.timeout)
        return True
    except Exception as e:
        if args.verbose:
            print(f"car {car.car_id}: reconnect failed: {e}")
        return False


def step_car(car: Car, num_nodes: int, args: argparse.Namespace) -> None:
    """Advance one car by one simulation step."""
    if car.state in ("WAITING", "ARRIVED"):
        src = car.rng.randrange(0, max(1, num_nodes))
        dst = pick_dst(car.rng, num_nodes, src)
        try:
            ok, info = register_car(car.sock, car.user_id, car.car_id, src, dst)
        except Exception as e:
            if args.verbose:
                print(f"car {car.car_id}: register error ({e}), reconnecting")
            _reconnect(car, args)
            car.state = "WAITING"
            return
        if ok:
            car.state = "DRIVING"
            if args.verbose and info:
                print(f"car {car.car_id}: registered {src}->{dst} "
                      f"route_len={info.get('route_len',0)} eta={info.get('eta',0):.2f}")
        else:
            car.state = "WAITING"
        return

    # DRIVING: ask server to tick
    try:
        resp = tick_car(car.sock, car.car_id, args.dt)
    except Exception as e:
        if args.verbose:
            print(f"car {car.car_id}: tick error ({e}), reconnecting")
        _reconnect(car, args)
        car.state = "WAITING"
        return

    cmd = resp.get("cmd", "")
    if cmd == "ARRIVED":
        car.state = "ARRIVED"
        car.arrivals += 1
        print(f"car {car.car_id}: arrived (trip #{car.arrivals})")
    elif cmd == "MOVE":
        car.state = "DRIVING"
    elif cmd in ("WAITING", "UNKNOWN_CAR"):
        car.state = "WAITING"
    else:
        car.state = "WAITING"


def simulate_loop(args: argparse.Namespace) -> None:
    num_nodes, _ = load_graph_meta(f"{args.graph_dir}/graph.meta")

    cars = []
    for i in range(args.cars):
        try:
            sock = connect(args.host, args.port, args.timeout)
        except Exception as e:
            print(f"car {i}: initial connect failed ({e}), will retry during simulation")
            # Create a dummy closed socket; _reconnect will fix it on first step
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.close()
        cars.append(Car(car_id=i, user_id=i, sock=sock, rng=random.Random(args.seed + 1000 + i)))

    sim_workers = args.sim_workers if args.sim_workers > 0 else max(1, min(args.cars, 32))

    try:
        with ThreadPoolExecutor(max_workers=sim_workers) as pool:
            for step in range(args.steps):
                futures = [
                    pool.submit(step_car, car, num_nodes, args)
                    for car in cars
                ]
                for f in futures:
                    f.result()

                if args.log_every > 0 and step % args.log_every == 0:
                    driving = sum(1 for c in cars if c.state == "DRIVING")
                    arrived = sum(1 for c in cars if c.state == "ARRIVED")
                    waiting = sum(1 for c in cars if c.state == "WAITING")
                    print(f"step {step}: driving={driving} arrived={arrived} waiting={waiting}")

                if args.sleep_ms > 0:
                    time.sleep(args.sleep_ms / 1000.0)
    finally:
        for c in cars:
            try:
                c.sock.close()
            except Exception:
                pass

    total_arrivals = sum(c.arrivals for c in cars)
    driving = sum(1 for c in cars if c.state == "DRIVING")
    waiting = sum(1 for c in cars if c.state == "WAITING")
    print("\nSimulation summary:")
    print(f"cars={len(cars)} total_arrivals={total_arrivals} driving={driving} waiting={waiting}")


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
    ap.add_argument("--timeout", type=float, default=10.0)
    ap.add_argument("--graph-dir", default="data")

    ap.add_argument("--mode", choices=["sim", "interactive"], default="sim")

    ap.add_argument("--cars", type=int, default=10)
    ap.add_argument("--steps", type=int, default=200)
    ap.add_argument("--dt", type=float, default=1.0)
    ap.add_argument("--sleep-ms", type=int, default=0)
    ap.add_argument("--log-every", type=int, default=10)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--verbose", action="store_true")
    ap.add_argument("--sim-workers", type=int, default=8)

    args = ap.parse_args()

    if args.mode == "interactive":
        interactive_mode(args)
    else:
        simulate_loop(args)


if __name__ == "__main__":
    main()
