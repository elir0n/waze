import csv
import random
import argparse
import math
import os

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
    for i in range(N):
        x = random.uniform(0, 1000)
        y = random.uniform(0, 1000)
        nodes.append((i, x, y))

    # ---------------- edges ----------------
    edges = []

    # Step 1: spanning tree (guaranteed connectivity)
    for i in range(1, N):
        j = random.randrange(0, i)
        dx = nodes[i][1] - nodes[j][1]
        dy = nodes[i][2] - nodes[j][2]
        length = math.hypot(dx, dy)
        speed = random.choice([30, 40, 50, 60])
        edges.append((len(edges), j, i, length, speed))

    # Step 2: random extra edges
    while len(edges) < M:
        u = random.randrange(0, N)
        v = random.randrange(0, N)
        if u == v:
            continue
        dx = nodes[u][1] - nodes[v][1]
        dy = nodes[u][2] - nodes[v][2]
        length = math.hypot(dx, dy)
        speed = random.choice([30, 40, 50, 60])
        edges.append((len(edges), u, v, length, speed))

    # ---------------- write files ----------------
    nodes_path = os.path.join(out_dir, "nodes.csv")
    edges_path = os.path.join(out_dir, "edges.csv")
    meta_path  = os.path.join(out_dir, "graph.meta")

    with open(nodes_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["node_id", "x", "y"])
        for n in nodes:
            w.writerow(n)

    with open(edges_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["edge_id", "from_node", "to_node", "base_length", "base_speed_limit"])
        for e in edges:
            w.writerow(e)

    with open(meta_path, "w") as f:
        f.write(f"num_nodes {N}\n")
        f.write(f"num_edges {M}\n")

    print(f"Generated graph in {out_dir}/")
    print(f"  nodes={N}, edges={M}")

if __name__ == "__main__":
    main()
