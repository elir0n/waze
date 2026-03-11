"""
flow_field/demo.py — Self-contained demonstration of the full pipeline.

Run with:
    python3 -m flow_field.demo                         # synthetic grid
    python3 -m flow_field.demo --data-dir data         # real CSV data
    python3 -m flow_field.demo --data-dir data --plot  # with matplotlib visualisation

The demo works in two modes:

1. Synthetic mode (default, no data files needed):
   Builds a small 80×60 cell grid with a simple road layout, runs the full
   flow-field pipeline, spawns 200 agents from Area A, and drives them toward
   Area B.  Prints per-step statistics.

2. Real-data mode (--data-dir):
   Loads nodes.csv + edges.csv from the specified directory, builds a grid,
   computes the flow field from the graph's SW quarter (Area A) to NE quarter
   (Area B), and runs the simulation.

The --plot flag (requires matplotlib) renders:
   - Cost field heat-map
   - Integration field
   - Flow-field arrows
   - Agent positions per frame
"""

from __future__ import annotations

import argparse
import math
import random
import sys
import time
from typing import List, Optional, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# Allow running as `python3 flow_field/demo.py` from the project root
# ---------------------------------------------------------------------------
if __package__ is None:
    import os
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from flow_field.grid   import Grid, GridBounds
from flow_field.fields import build_fields
from flow_field.loader import (
    load_graph_to_grid, cells_in_world_box
)
# agents.py was moved to legacy/ alongside this file
import sys as _sys, os as _os
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
from agents import (
    Agent, SpatialHash, update_agents, sample_flow_bilinear
)


# ===========================================================================
# Synthetic road-grid builder
# ===========================================================================

def build_synthetic_grid(width: int = 80, height: int = 60,
                          cell_size: float = 50.0) -> Grid:
    """
    Build a simple synthetic grid without any CSV data.

    Road layout:
      - A horizontal "motorway" across the middle (y ≈ height/2)
      - A vertical "primary" road at x ≈ width/3
      - A vertical "secondary" at x ≈ 2*width/3
      - A grid of "residential" streets every 10 cells in x and y

    This lets the demo run with no external data files.
    """
    bounds = GridBounds(
        min_x=0.0, min_y=0.0,
        max_x=width  * cell_size,
        max_y=height * cell_size,
        cell_size=cell_size,
    )
    grid = Grid(bounds)

    def paint_hline(row: int, road_type: str, speed: float, lanes: int,
                    c_from: int = 0, c_to: Optional[int] = None) -> None:
        if c_to is None:
            c_to = width - 1
        d = Grid.road_desirability(speed, lanes, road_type)
        x0 = bounds.min_x + c_from * cell_size
        x1 = bounds.min_x + c_to   * cell_size
        y  = bounds.min_y + row    * cell_size
        grid.rasterize_segment(x0, y, x1, y, d)

    def paint_vline(col: int, road_type: str, speed: float, lanes: int,
                    r_from: int = 0, r_to: Optional[int] = None) -> None:
        if r_to is None:
            r_to = height - 1
        d = Grid.road_desirability(speed, lanes, road_type)
        x  = bounds.min_x + col    * cell_size
        y0 = bounds.min_y + r_from * cell_size
        y1 = bounds.min_y + r_to   * cell_size
        grid.rasterize_segment(x, y0, x, y1, d)

    # Motorway across the middle
    paint_hline(height // 2, "motorway", 120.0, 4)
    # Primary verticals
    paint_vline(width // 3,      "primary", 80.0, 3)
    paint_vline(2 * width // 3,  "primary", 80.0, 3)
    # Secondary at 1/4 and 3/4
    paint_vline(width // 4,       "secondary", 60.0, 2)
    paint_vline(3 * width // 4,   "secondary", 60.0, 2)
    # Residential grid every 10 cells
    for r in range(0, height, 10):
        paint_hline(r, "residential", 30.0, 1)
    for c in range(0, width, 10):
        paint_vline(c, "residential", 30.0, 1)

    grid.finalize_cost_field()
    return grid


# ===========================================================================
# Area helpers for the synthetic grid
# ===========================================================================

def synthetic_area_a(grid: Grid) -> List[Tuple[int, int]]:
    """Left quarter of the synthetic grid (Area A — origin)."""
    cells = []
    for r in range(grid.height):
        for c in range(grid.width // 4):
            if not grid.blocked[r, c]:
                cells.append((c, r))
    return cells


def synthetic_area_b(grid: Grid) -> List[Tuple[int, int]]:
    """Right quarter of the synthetic grid (Area B — destination)."""
    cells = []
    for r in range(grid.height):
        for c in range(3 * grid.width // 4, grid.width):
            if not grid.blocked[r, c]:
                cells.append((c, r))
    return cells


# ===========================================================================
# Visualisation (optional matplotlib)
# ===========================================================================

def visualise(grid: Grid, agents: List[Agent],
              step: int, pause: float = 0.05) -> None:
    """Render cost, integration, flow, and agent positions."""
    import matplotlib.pyplot as plt
    import matplotlib.colors as mcolors

    if not hasattr(visualise, "_fig"):
        fig, axes = plt.subplots(1, 3, figsize=(16, 5))
        visualise._fig  = fig
        visualise._axes = axes
        plt.ion()
        plt.tight_layout()

    fig  = visualise._fig
    axes = visualise._axes

    # ---- cost ----
    ax = axes[0]
    ax.cla()
    cost_vis = np.where(np.isinf(grid.cost), np.nan, grid.cost)
    im = ax.imshow(cost_vis, origin="lower", cmap="YlOrRd_r", aspect="auto")
    ax.set_title("Cost field (darker = cheaper)")

    # ---- integration ----
    ax = axes[1]
    ax.cla()
    intg_vis = np.where(np.isinf(grid.integration), np.nan, grid.integration)
    ax.imshow(intg_vis, origin="lower", cmap="plasma_r", aspect="auto")
    ax.set_title("Integration field")

    # ---- flow + agents ----
    ax = axes[2]
    ax.cla()
    ax.imshow(np.where(grid.blocked, 0.0, 1.0), origin="lower",
              cmap="Greys", vmin=0, vmax=1, aspect="auto", alpha=0.4)

    # Subsample flow arrows (every 4th cell)
    step_c, step_r = 4, 4
    rows = np.arange(0, grid.height, step_r)
    cols = np.arange(0, grid.width,  step_c)
    CC, RR = np.meshgrid(cols, rows)
    FX = grid.flow_x[RR, CC]
    FY = grid.flow_y[RR, CC]
    valid = (FX != 0) | (FY != 0)
    ax.quiver(CC[valid], RR[valid], FX[valid], FY[valid],
              color="steelblue", scale=20, width=0.003, alpha=0.7)

    # Agents by mode
    flow_agents = [(a.col, a.row) for a in agents if a.mode == "FLOW"]
    lm_agents   = [(a.col, a.row) for a in agents if a.mode == "LAST_MILE"]
    arr_agents  = [(a.col, a.row) for a in agents if a.mode == "ARRIVED"]

    for pts, color, label in [
        (flow_agents, "dodgerblue", "FLOW"),
        (lm_agents,   "orange",     "LAST_MILE"),
        (arr_agents,  "limegreen",  "ARRIVED"),
    ]:
        if pts:
            xs, ys = zip(*pts)
            ax.scatter(xs, ys, c=color, s=4, label=label, zorder=5)

    ax.legend(loc="upper left", fontsize=6)
    ax.set_title(f"Agents — step {step}")

    fig.canvas.draw()
    fig.canvas.flush_events()
    plt.pause(pause)


# ===========================================================================
# Main simulation loop
# ===========================================================================

def run_demo(args: argparse.Namespace) -> None:

    rng = random.Random(args.seed)

    # ------------------------------------------------------------------
    # 1. Build grid
    # ------------------------------------------------------------------
    if args.data_dir:
        print(f"[demo] Loading graph from {args.data_dir}/ …")
        t0 = time.perf_counter()
        grid, node_world_pos, node_latlon = load_graph_to_grid(
            data_dir=args.data_dir,
            cell_size=args.cell_size,
        )
        t1 = time.perf_counter()
        print(f"[demo] Grid {grid.width}×{grid.height} ({grid.width*grid.height:,} cells) "
              f"loaded in {t1-t0:.2f}s")

        # Define Area A (SW corner) and Area B (NE corner) from world bounds
        w_m = (grid.bounds.max_x - grid.bounds.min_x)
        h_m = (grid.bounds.max_y - grid.bounds.min_y)

        area_a_cells = cells_in_world_box(
            grid,
            grid.bounds.min_x,               grid.bounds.min_y,
            grid.bounds.min_x + w_m * 0.25,  grid.bounds.min_y + h_m * 0.25,
        )
        area_b_cells = cells_in_world_box(
            grid,
            grid.bounds.min_x + w_m * 0.75,  grid.bounds.min_y + h_m * 0.75,
            grid.bounds.max_x,                grid.bounds.max_y,
        )

    else:
        print("[demo] Building synthetic grid …")
        grid = build_synthetic_grid(cell_size=args.cell_size)
        area_a_cells = synthetic_area_a(grid)
        area_b_cells = synthetic_area_b(grid)

    traversable = int(np.sum(~grid.blocked))
    blocked_pct  = 100.0 * np.sum(grid.blocked) / grid.blocked.size
    print(f"[demo] Traversable: {traversable:,}  Blocked: {blocked_pct:.1f}%")
    print(f"[demo] Area A cells: {len(area_a_cells)}  Area B cells: {len(area_b_cells)}")

    if not area_b_cells:
        print("[demo] ERROR: Area B has no traversable cells. "
              "Increase cell_size or check the data.", file=sys.stderr)
        sys.exit(1)

    # ------------------------------------------------------------------
    # 2. Compute integration + flow fields
    # ------------------------------------------------------------------
    print("[demo] Computing integration field (Dijkstra) …")
    t0 = time.perf_counter()
    build_fields(grid, area_b_cells)
    t1 = time.perf_counter()

    reachable = np.sum(np.isfinite(grid.integration))
    print(f"[demo] Integration field done in {t1-t0:.3f}s  "
          f"({reachable:,} / {traversable:,} cells reachable from Area B)")

    # ------------------------------------------------------------------
    # 3. Spawn agents in Area A
    # ------------------------------------------------------------------
    num_agents = args.agents
    if not area_a_cells:
        print("[demo] ERROR: Area A has no traversable cells.", file=sys.stderr)
        sys.exit(1)

    # Each agent gets its own random destination cell within Area B.
    # This avoids all agents cramming toward a single point.
    agents: List[Agent] = []
    for idx in range(num_agents):
        sc, sr = rng.choice(area_a_cells)
        dc, dr = rng.choice(area_b_cells)
        jitter = 0.4
        agents.append(Agent(
            agent_id=idx,
            col=sc + rng.uniform(-jitter, jitter),
            row=sr + rng.uniform(-jitter, jitter),
            dst_col=float(dc),
            dst_row=float(dr),
            speed=rng.uniform(1.5, 2.5),
            last_mile_radius=8.0,
        ))

    spatial_hash = SpatialHash(bucket_size=3.0)

    # ------------------------------------------------------------------
    # 4. Simulation loop
    # ------------------------------------------------------------------
    print(f"[demo] Simulating {num_agents} agents for {args.steps} steps "
          f"(dt={args.dt}s) …\n")

    total_time = 0.0
    for step in range(args.steps):
        t0 = time.perf_counter()
        update_agents(
            agents, grid.flow_x, grid.flow_y, grid.blocked,
            args.dt, spatial_hash,
            integration=grid.integration,
            separation_radius=2.0,
            separation_strength=0.5,
            flow_weight=0.85,
            sep_weight=0.15,
        )
        t1 = time.perf_counter()
        total_time += (t1 - t0)

        if (step + 1) % args.log_every == 0 or step == 0:
            n_flow = sum(1 for a in agents if a.mode == "FLOW")
            n_lm   = sum(1 for a in agents if a.mode == "LAST_MILE")
            n_arr  = sum(1 for a in agents if a.mode == "ARRIVED")
            avg_ms = (total_time / (step + 1)) * 1000
            print(f"  step {step+1:>4d}:  FLOW={n_flow:>5d}  LAST_MILE={n_lm:>5d}  "
                  f"ARRIVED={n_arr:>5d}    avg {avg_ms:.3f} ms/step")

        if args.plot:
            visualise(grid, agents, step + 1, pause=0.02)

        # Stop early if everyone arrived
        if all(a.mode == "ARRIVED" for a in agents):
            print(f"\n[demo] All agents arrived after {step+1} steps.")
            break

    # ------------------------------------------------------------------
    # 5. Summary
    # ------------------------------------------------------------------
    n_arr = sum(1 for a in agents if a.mode == "ARRIVED")
    total_steps = min(step + 1, args.steps)
    print(f"\n[demo] Finished:  {n_arr}/{num_agents} arrived in {total_steps} steps.")
    print(f"[demo] Total sim time: {total_time*1000:.1f} ms "
          f"over {total_steps} steps  "
          f"= {total_time*1000/total_steps:.3f} ms/step "
          f"({num_agents} agents → "
          f"{total_time*1e6/total_steps/num_agents:.3f} µs/agent/step)")

    if args.plot:
        import matplotlib.pyplot as plt
        print("[demo] Close the plot window to exit.")
        plt.ioff()
        plt.show()


# ===========================================================================
# Entry point
# ===========================================================================

def main() -> None:
    ap = argparse.ArgumentParser(
        description="Flow Field Pathfinding demo",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    ap.add_argument("--data-dir",  default=None,
                    help="CSV data directory (nodes.csv + edges.csv). "
                         "Omit to use the built-in synthetic grid.")
    ap.add_argument("--cell-size", type=float, default=50.0,
                    help="Grid resolution in metres per cell.")
    ap.add_argument("--agents",    type=int,   default=500,
                    help="Number of agents to simulate.")
    ap.add_argument("--steps",     type=int,   default=300,
                    help="Number of simulation steps.")
    ap.add_argument("--dt",        type=float, default=1.0,
                    help="Time step in seconds.")
    ap.add_argument("--log-every", type=int,   default=25,
                    help="Print status every N steps.")
    ap.add_argument("--seed",      type=int,   default=42)
    ap.add_argument("--plot",      action="store_true",
                    help="Show live matplotlib visualisation (requires matplotlib).")
    args = ap.parse_args()
    run_demo(args)


if __name__ == "__main__":
    main()
