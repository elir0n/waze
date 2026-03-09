import csv
import random
import argparse
import math
import os

ROAD_TYPES = ["motorway", "primary", "secondary", "tertiary", "residential", "service"]

def haversine_m(lat1, lon1, lat2, lon2):
    deg_to_rad = math.pi / 180.0
    r = 6371000.0
    p1 = lat1 * deg_to_rad
    p2 = lat2 * deg_to_rad
    dlat = (lat2 - lat1) * deg_to_rad
    dlon = (lon2 - lon1) * deg_to_rad
    a = math.sin(dlat / 2.0) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlon / 2.0) ** 2
    return 2.0 * r * math.asin(math.sqrt(a))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--nodes", type=int, default=1000)
    ap.add_argument("--edges", type=int, default=3000)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--out", default="data")
    args = ap.parse_args()

    random.seed(args.seed)

    out_dir = args.out
    os.makedirs(out_dir, exist_ok=True)

    N = args.nodes
    M = args.edges

    if M < N - 1:
        raise ValueError("edges must be >= nodes-1 to ensure connectivity")

    # ---------------- nodes ----------------
    nodes = []
    center_lat = 32.0853
    center_lon = 34.7818
    for i in range(N):
        lat = center_lat + random.uniform(-0.05, 0.05)
        lon = center_lon + random.uniform(-0.05, 0.05)
        nodes.append((i, lat, lon))

    # ---------------- edges ----------------
    edges = []

    # spanning tree (guaranteed connectivity)
    for i in range(1, N):
        j = random.randrange(0, i)
        dx = nodes[i][1] - nodes[j][1]
        dy = nodes[i][2] - nodes[j][2]
        length = math.hypot(dx, dy) # geometric distance
        speed = random.choice([30, 40, 50, 60])
        road_type = random.choice(ROAD_TYPES)
        lanes = random.randint(1, 4)
        is_oneway = random.choice([0, 1])
        edges.append((len(edges), j, i, length, speed, road_type, lanes, is_oneway))

    # random extra edges
    while len(edges) < M:
        u = random.randrange(0, N)
        v = random.randrange(0, N)
        if u == v:
            continue
        length = haversine_m(nodes[u][1], nodes[u][2], nodes[v][1], nodes[v][2])
        speed = random.choice([30, 40, 50, 60])
        road_type = random.choice(ROAD_TYPES)
        lanes = random.randint(1, 4)
        is_oneway = random.choice([0, 1])
        edges.append((len(edges), u, v, length, speed, road_type, lanes, is_oneway))

    # ---------------- write files ----------------
    nodes_path = os.path.join(out_dir, "nodes.csv")
    edges_path = os.path.join(out_dir, "edges.csv")
    meta_path  = os.path.join(out_dir, "graph.meta")

    with open(nodes_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["node_id", "lat", "lon"])
        for n in nodes:
            w.writerow(n)

    with open(edges_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow([
            "edge_id",
            "from_node",
            "to_node",
            "base_length",
            "base_speed_limit",
            "road_type",
            "lanes",
            "is_oneway",
        ])
        for e in edges:
            w.writerow(e)

    with open(meta_path, "w") as f:
        f.write(f"num_nodes {N}\n")
        f.write(f"num_edges {M}\n")

    print(f"Generated graph in {out_dir}/")
    print(f"  nodes={N}, edges={M}")

if __name__ == "__main__":
    main()
