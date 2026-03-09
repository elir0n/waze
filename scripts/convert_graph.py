import osmnx as ox
import csv

G = ox.load_graphml("../data/tel_aviv.graphml")

def parse_speed_kmh(raw):
    if raw is None:
        return 50.0
    if isinstance(raw, list):
        raw = raw[0] if raw else None
    if raw is None:
        return 50.0
    s = str(raw)
    num = []
    for ch in s:
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
    s = str(raw).strip()
    token = s.split(";")[0].split("|")[0].strip()
    try:
        v = int(token)
        return v if v > 0 else 1
    except ValueError:
        return 1

def parse_road_type(data):
    return str(data.get("highway", "unknown"))

def parse_oneway(data):
    v = data.get("oneway", False)
    if isinstance(v, str):
        return 1 if v.lower() in ("yes", "true", "1") else 0
    return 1 if bool(v) else 0

# nodes
with open("../data/nodes.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["node_id", "lat", "lon"])

    for node, data in G.nodes(data=True):
        writer.writerow([node, data["y"], data["x"]])

# edges
with open("../data/edges.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow([
        "edge_id",
        "from_node",
        "to_node",
        "base_length",
        "base_speed_limit",
        "road_type",
        "lanes",
        "is_oneway",
    ])

    edge_id = 0
    for u, v, data in G.edges(data=True):
        writer.writerow([
            edge_id,
            u,
            v,
            float(data.get("length", 1.0)),
            parse_speed_kmh(data.get("maxspeed")),
            parse_road_type(data),
            parse_lanes(data.get("lanes")),
            parse_oneway(data),
        ])
        edge_id += 1

with open("../data/graph.meta", "w") as f:
    f.write(f"num_nodes {G.number_of_nodes()}\n")
    f.write(f"num_edges {G.number_of_edges()}\n")

print("Export finished")
