"""
Download the Tel Aviv drive network from OpenStreetMap via osmnx
and write nodes.csv / edges.csv / graph.meta to --out directory.

Used by entrypoint.sh on first container start.
"""
import argparse
import csv
import os

import osmnx as ox


def parse_speed_kmh(raw):
    if raw is None:
        return 50.0
    if isinstance(raw, list):
        raw = raw[0] if raw else None
    if raw is None:
        return 50.0
    num = []
    for ch in str(raw):
        if ch.isdigit() or ch == ".":
            num.append(ch)
        elif num:
            break
    if not num:
        return 50.0
    try:
        v = float("".join(num))
        return v if v > 0.0 else 50.0
    except ValueError:
        return 50.0


def parse_lanes(raw):
    if raw is None:
        return 1
    if isinstance(raw, list):
        raw = raw[0] if raw else None
    if raw is None:
        return 1
    token = str(raw).strip().split(";")[0].split("|")[0].strip()
    try:
        v = int(token)
        return v if v > 0 else 1
    except ValueError:
        return 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--place", default="Tel Aviv-Yafo, Israel")
    ap.add_argument("--out", default="data")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)

    print(f"[init] Downloading '{args.place}' from OpenStreetMap...")
    G = ox.graph_from_place(args.place, network_type="drive")
    print(f"[init] Downloaded: {G.number_of_nodes()} nodes, {G.number_of_edges()} edges")

    with open(os.path.join(args.out, "nodes.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["node_id", "lat", "lon"])
        for node, data in G.nodes(data=True):
            w.writerow([node, data["y"], data["x"]])

    with open(os.path.join(args.out, "edges.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["edge_id", "from_node", "to_node", "base_length",
                    "base_speed_limit", "road_type", "lanes", "is_oneway"])
        edge_id = 0
        for u, v, data in G.edges(data=True):
            oneway = data.get("oneway", False)
            if isinstance(oneway, str):
                oneway = 1 if oneway.lower() in ("yes", "true", "1") else 0
            else:
                oneway = 1 if bool(oneway) else 0
            w.writerow([
                edge_id, u, v,
                float(data.get("length", 1.0)),
                parse_speed_kmh(data.get("maxspeed")),
                str(data.get("highway", "unknown")),
                parse_lanes(data.get("lanes")),
                oneway,
            ])
            edge_id += 1

    with open(os.path.join(args.out, "graph.meta"), "w") as f:
        f.write(f"num_nodes {G.number_of_nodes()}\n")
        f.write(f"num_edges {G.number_of_edges()}\n")

    print(f"[init] Graph written to {args.out}/")


if __name__ == "__main__":
    main()
