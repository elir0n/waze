"""
flow_field/loader.py — Load nodes.csv / edges.csv and build a Grid.

OSM-to-grid pipeline
--------------------
1. Read all node (lat, lon) pairs → determine grid bounds.
2. Construct Grid with those bounds and the requested cell size.
3. For every edge:
     a. Compute road desirability from (speed_kph, lanes, road_type).
     b. Project the from-node and to-node world positions.
     c. Rasterize the segment via Bresenham into the grid.
     d. For two-way roads (is_oneway == 0) also rasterize the reverse
        segment (from to-node to from-node) — same desirability.
4. Call grid.finalize_cost_field() to convert desirability → cost.

The resulting Grid object has:
    grid.cost       — float32 (height × width), inf = blocked
    grid.blocked    — bool    (height × width)
    grid.integration, grid.flow_x, grid.flow_y — all zeros/inf until
        fields.build_fields() is called with target cells.

Projection
----------
We use a simple equirectangular approximation valid for city-scale areas
(≲ 50 km).  The reference origin is the south-west corner of the node set.
At latitude ~32° the east–west distortion is cos(32°) ≈ 0.848, which we
fold in correctly.  For larger areas a proper UTM projection should be used.
"""

from __future__ import annotations

import csv
import os
from typing import Dict, List, Optional, Tuple

from .grid import Grid, GridBounds, bounds_from_latlon, latlon_to_local


# ---------------------------------------------------------------------------
# Public loader
# ---------------------------------------------------------------------------

def load_graph_to_grid(
    data_dir: str = "data",
    cell_size: float = 50.0,
    padding_m: float = 200.0,
) -> Tuple[Grid, Dict[int, Tuple[float, float]], Dict[int, Tuple[float, float]]]:
    """
    Read nodes.csv + edges.csv from *data_dir* and build a Grid.

    Args:
        data_dir  : directory containing nodes.csv, edges.csv, graph.meta
        cell_size : grid resolution in metres per cell (default 50 m)
        padding_m : extra border around the node bounding box (metres)

    Returns:
        (grid, node_world_pos, node_latlon)

        grid           : Grid with cost field finalised; ready for fields.build_fields()
        node_world_pos : dict {node_id: (x_m, y_m)} in local metres
        node_latlon    : dict {node_id: (lat, lon)}  original coordinates
    """
    nodes_path = os.path.join(data_dir, "nodes.csv")
    edges_path = os.path.join(data_dir, "edges.csv")

    # ---- Step 1: read nodes ----
    node_latlon: Dict[int, Tuple[float, float]] = {}
    with open(nodes_path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            nid = int(row["node_id"])
            lat = float(row["lat"])
            lon = float(row["lon"])
            node_latlon[nid] = (lat, lon)

    lats = [v[0] for v in node_latlon.values()]
    lons = [v[1] for v in node_latlon.values()]

    # ---- Step 2: grid bounds and projection ----
    bounds = bounds_from_latlon(lats, lons, cell_size, padding_m)
    ref_lat = min(lats)
    ref_lon = min(lons)

    node_world_pos: Dict[int, Tuple[float, float]] = {}
    for nid, (lat, lon) in node_latlon.items():
        x, y = latlon_to_local(lat, lon, ref_lat, ref_lon)
        node_world_pos[nid] = (x, y)

    grid = Grid(bounds)

    # ---- Step 3: rasterize edges ----
    with open(edges_path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            from_id     = int(row["from_node"])
            to_id       = int(row["to_node"])
            speed_kph   = float(row["base_speed_limit"])
            road_type   = row["road_type"].strip()
            lanes       = int(row["lanes"])
            is_oneway   = int(row["is_oneway"])

            if from_id not in node_world_pos or to_id not in node_world_pos:
                continue  # skip edges with missing nodes

            x0, y0 = node_world_pos[from_id]
            x1, y1 = node_world_pos[to_id]

            desirability = Grid.road_desirability(speed_kph, lanes, road_type)

            grid.rasterize_segment(x0, y0, x1, y1, desirability)
            if not is_oneway:
                # Rasterize reverse direction so bidirectional roads fully cover both
                grid.rasterize_segment(x1, y1, x0, y0, desirability)

    # ---- Step 4: finalise ----
    grid.finalize_cost_field()

    return grid, node_world_pos, node_latlon


# ---------------------------------------------------------------------------
# Area helpers
# ---------------------------------------------------------------------------

def cells_in_latlon_box(
    grid: Grid,
    node_world_pos: Dict[int, Tuple[float, float]],
    node_latlon: Dict[int, Tuple[float, float]],
    lat_min: float, lat_max: float,
    lon_min: float, lon_max: float,
    ref_lat: Optional[float] = None,
    ref_lon: Optional[float] = None,
) -> List[Tuple[int, int]]:
    """
    Return all *traversable* grid cells whose centres fall inside a
    lat/lon bounding box.

    Useful for defining Area B (destination region) from geographic bounds.
    """
    all_lats = [v[0] for v in node_latlon.values()]
    all_lons = [v[1] for v in node_latlon.values()]
    ref_lat = ref_lat if ref_lat is not None else min(all_lats)
    ref_lon = ref_lon if ref_lon is not None else min(all_lons)

    # Convert box corners to world metres
    from .grid import latlon_to_local
    x_min, y_min = latlon_to_local(lat_min, lon_min, ref_lat, ref_lon)
    x_max, y_max = latlon_to_local(lat_max, lon_max, ref_lat, ref_lon)

    c_lo, r_lo = grid.world_to_cell(x_min, y_min)
    c_hi, r_hi = grid.world_to_cell(x_max, y_max)

    # Normalise ordering (lat/lon box may not map to sorted col/row)
    c_lo, c_hi = min(c_lo, c_hi), max(c_lo, c_hi)
    r_lo, r_hi = min(r_lo, r_hi), max(r_lo, r_hi)

    cells = []
    for r in range(max(0, r_lo), min(grid.height, r_hi + 1)):
        for c in range(max(0, c_lo), min(grid.width, c_hi + 1)):
            if not grid.blocked[r, c]:
                cells.append((c, r))
    return cells


def cells_in_world_box(
    grid: Grid,
    x_min: float, y_min: float,
    x_max: float, y_max: float,
) -> List[Tuple[int, int]]:
    """
    Return all traversable cells whose centres fall inside a world-metre box.
    """
    c_lo, r_lo = grid.world_to_cell(x_min, y_min)
    c_hi, r_hi = grid.world_to_cell(x_max, y_max)

    c_lo, c_hi = (min(c_lo, c_hi), max(c_lo, c_hi))
    r_lo, r_hi = (min(r_lo, r_hi), max(r_lo, r_hi))

    cells = []
    for r in range(max(0, r_lo), min(grid.height, r_hi + 1)):
        for c in range(max(0, c_lo), min(grid.width, c_hi + 1)):
            if not grid.blocked[r, c]:
                cells.append((c, r))
    return cells
