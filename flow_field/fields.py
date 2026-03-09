"""
flow_field/fields.py — Cost field, Integration field (Dijkstra), Flow field.

Pipeline
--------
1. cost field      — raw per-cell traversal cost (seconds/cell).  Built in grid.py.
2. integration field — accumulated minimum cost to reach destination region.
                       Computed here with multi-source Dijkstra.
3. flow field       — unit direction vectors toward the descent of the integration
                      field.  Computed here with a vectorized NumPy pass.

Design notes
------------
* We use Dijkstra (not BFS) because cell costs differ.  BFS is only correct for
  uniform-cost grids; using it here would give wrong shortest paths.
* The integration field is computed once from *all* destination cells simultaneously
  (multi-source Dijkstra with all seeds at distance 0).  This is equivalent to
  placing a "super-source" with zero-cost edges to every destination cell.
* The flow field derivation is fully vectorised: for each of the 8 directions we
  shift the integration array, compare with the running minimum, and record the
  winner direction.  No Python loop over individual cells.
* The edge cost between two adjacent cells is the average of their costs times the
  Euclidean distance weight (1.0 for cardinal, √2 for diagonal).  This gives a
  physically meaningful "total seconds to cross" metric.
"""

from __future__ import annotations

import heapq
from typing import List, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# 8-directional neighbour table  (dcol, drow, Euclidean_weight)
# ---------------------------------------------------------------------------
_NEIGHBORS_8: List[Tuple[int, int, float]] = [
    (-1,  0, 1.0),          # West
    ( 1,  0, 1.0),          # East
    ( 0, -1, 1.0),          # South
    ( 0,  1, 1.0),          # North
    (-1, -1, 1.4142135),    # SW
    ( 1, -1, 1.4142135),    # SE
    (-1,  1, 1.4142135),    # NW
    ( 1,  1, 1.4142135),    # NE
]


# ---------------------------------------------------------------------------
# Integration field
# ---------------------------------------------------------------------------

def compute_integration_field(
    cost: np.ndarray,
    target_cells: List[Tuple[int, int]],
) -> np.ndarray:
    """
    Multi-source Dijkstra from every cell in *target_cells* outward.

    integration[row, col] = minimum total traversal cost (seconds) to reach
                            any destination cell from (col, row).

    All destination cells start with integration = 0.0 and the wavefront
    expands through traversable neighbours (cost < inf), accumulating cost.

    Args:
        cost         : (height, width) float32 array.  inf = blocked.
        target_cells : list of (col, row) pairs that form Area B.

    Returns:
        (height, width) float64 integration array.

    Complexity: O(N log N) where N = number of traversable cells.
    """
    height, width = cost.shape
    integration = np.full((height, width), np.inf, dtype=np.float64)

    # Min-heap entries: (accumulated_cost, col, row)
    heap: List[Tuple[float, int, int]] = []

    for col, row in target_cells:
        if 0 <= col < width and 0 <= row < height and cost[row, col] < np.inf:
            if integration[row, col] > 0.0:  # seed only once
                integration[row, col] = 0.0
                heapq.heappush(heap, (0.0, col, row))

    while heap:
        dist, col, row = heapq.heappop(heap)

        # Stale entry — a shorter path was already found
        if dist > integration[row, col]:
            continue

        for dcol, drow, w in _NEIGHBORS_8:
            nc, nr = col + dcol, row + drow
            if not (0 <= nc < width and 0 <= nr < height):
                continue
            ncost = cost[nr, nc]
            if ncost >= np.inf:
                continue
            # Edge cost: average of the two cell costs × diagonal weight
            edge_cost = 0.5 * (float(cost[row, col]) + float(ncost)) * w
            new_dist = dist + edge_cost
            if new_dist < integration[nr, nc]:
                integration[nr, nc] = new_dist
                heapq.heappush(heap, (new_dist, nc, nr))

    return integration


# ---------------------------------------------------------------------------
# Flow field
# ---------------------------------------------------------------------------

def compute_flow_field(
    integration: np.ndarray,
    blocked: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Derive normalised flow vectors from the integration field.

    For every non-blocked cell, find the neighbour (among 8) with the
    smallest integration value and set the flow direction toward it.

    Implementation is fully vectorised: we shift the integration array in
    each of the 8 directions, track the per-cell minimum across all shifts,
    and record the corresponding (dcol, drow) displacement.

    Args:
        integration : (height, width) float64 integration field.
        blocked     : (height, width) bool array.  True = blocked.

    Returns:
        (flow_x, flow_y) as float32 arrays of shape (height, width).
        Blocked and unreachable cells have (0, 0).
        All other vectors are unit-length in grid-cell space.
    """
    height, width = integration.shape

    best_val = np.full((height, width), np.inf, dtype=np.float64)
    best_dx  = np.zeros((height, width), dtype=np.float32)
    best_dy  = np.zeros((height, width), dtype=np.float32)

    for dcol, drow, _ in _NEIGHBORS_8:
        # Shift integration so that shifted[r, c] = integration of (c+dcol, r+drow)
        # np.roll wraps edges — we fix that by filling wrapped regions with inf.
        shifted = np.roll(np.roll(integration, -drow, axis=0), -dcol, axis=1)

        # Invalidate the border rows/cols that were wrapped by np.roll
        if drow > 0:
            shifted[-drow:, :] = np.inf
        elif drow < 0:
            shifted[:-drow, :] = np.inf
        if dcol > 0:
            shifted[:, -dcol:] = np.inf
        elif dcol < 0:
            shifted[:, :-dcol] = np.inf

        better = shifted < best_val
        best_val[better] = shifted[better]
        best_dx[better]  = float(dcol)
        best_dy[better]  = float(drow)

    # Normalise
    mag = np.sqrt(best_dx ** 2 + best_dy ** 2)
    valid = (~blocked) & (mag > 0) & np.isfinite(integration)

    flow_x = np.zeros((height, width), dtype=np.float32)
    flow_y = np.zeros((height, width), dtype=np.float32)
    flow_x[valid] = best_dx[valid] / mag[valid]
    flow_y[valid] = best_dy[valid] / mag[valid]

    return flow_x, flow_y


# ---------------------------------------------------------------------------
# Convenience: run the full pipeline on a Grid object
# ---------------------------------------------------------------------------

def build_fields(grid, target_cells: List[Tuple[int, int]]) -> None:
    """
    Run the integration + flow pipeline and store results in *grid* in place.

    Args:
        grid         : a flow_field.grid.Grid instance whose cost field has
                       already been finalized (grid.finalize_cost_field() called).
        target_cells : list of (col, row) destination cells (Area B).
    """
    grid.integration = compute_integration_field(grid.cost, target_cells)
    grid.flow_x, grid.flow_y = compute_flow_field(grid.integration, grid.blocked)
