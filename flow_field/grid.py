"""
flow_field/grid.py — Uniform 2D grid over the map area.

Responsibilities:
  - Project lat/lon → local meters (equirectangular approximation, city-scale)
  - Define grid bounds and cell indexing
  - Rasterize road segments (Bresenham) into grid cells
  - Accumulate road desirability scores per cell
  - Expose the finalized cost field for downstream pipeline stages

Memory layout (row-major NumPy):
    index = row * width + col
    arr[row, col]   ← row = y axis (North), col = x axis (East)

All position units are grid-space floats unless noted as "world" (meters).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# Road-type desirability factors  (0 < factor ≤ 1, higher = more desirable)
# ---------------------------------------------------------------------------
ROAD_TYPE_FACTORS: dict[str, float] = {
    "motorway":       1.00,
    "motorway_link":  0.90,
    "trunk":          0.90,
    "trunk_link":     0.85,
    "primary":        0.80,
    "primary_link":   0.75,
    "secondary":      0.70,
    "secondary_link": 0.65,
    "tertiary":       0.60,
    "tertiary_link":  0.55,
    "unclassified":   0.50,
    "residential":    0.40,
    "living_street":  0.30,
    "service":        0.25,
    "track":          0.15,
    "path":           0.10,
}
DEFAULT_ROAD_TYPE_FACTOR = 0.40

# Reference speed for normalization (km/h) — treated as "ideal motorway"
MAX_SPEED_KPH: float = 130.0

# Maximum lane count used for lane normalization
MAX_LANES: int = 4

# Earth radius (metres)
_R_EARTH = 6_371_000.0


# ---------------------------------------------------------------------------
# GridBounds
# ---------------------------------------------------------------------------

@dataclass
class GridBounds:
    """
    Axis-aligned bounding box in local-metre coordinates, plus cell size.

    The "local metre" origin is the southwest corner (min_x, min_y).
    Positive x = East, positive y = North.
    """
    min_x: float
    min_y: float
    max_x: float
    max_y: float
    cell_size: float  # metres per cell (square cells)

    @property
    def width(self) -> int:
        """Number of columns (East–West)."""
        return max(1, int(math.ceil((self.max_x - self.min_x) / self.cell_size)))

    @property
    def height(self) -> int:
        """Number of rows (South–North)."""
        return max(1, int(math.ceil((self.max_y - self.min_y) / self.cell_size)))


# ---------------------------------------------------------------------------
# Coordinate helpers (module-level, so they can be used without a Grid instance)
# ---------------------------------------------------------------------------

def latlon_to_local(lat: float, lon: float,
                    ref_lat: float, ref_lon: float) -> Tuple[float, float]:
    """
    Equirectangular projection → local metres.

    Args:
        lat, lon  : point to project
        ref_lat   : latitude of the grid origin (south edge)
        ref_lon   : longitude of the grid origin (west edge)

    Returns:
        (x_m, y_m) in metres relative to the reference point.
    """
    x = _R_EARTH * math.radians(lon - ref_lon) * math.cos(math.radians(ref_lat))
    y = _R_EARTH * math.radians(lat - ref_lat)
    return x, y


def bounds_from_latlon(lats: List[float], lons: List[float],
                       cell_size: float,
                       padding_m: float = 200.0) -> GridBounds:
    """
    Compute GridBounds that contain all given lat/lon points.

    Args:
        lats, lons  : coordinate lists (equal length)
        cell_size   : desired cell size in metres
        padding_m   : extra margin around the tight bounding box

    Returns:
        GridBounds with the southwest corner as the local origin.
    """
    ref_lat = min(lats)
    ref_lon = min(lons)

    xs = []
    ys = []
    for lat, lon in zip(lats, lons):
        x, y = latlon_to_local(lat, lon, ref_lat, ref_lon)
        xs.append(x)
        ys.append(y)

    return GridBounds(
        min_x=min(xs) - padding_m,
        min_y=min(ys) - padding_m,
        max_x=max(xs) + padding_m,
        max_y=max(ys) + padding_m,
        cell_size=cell_size,
    )


# ---------------------------------------------------------------------------
# Grid
# ---------------------------------------------------------------------------

class Grid:
    """
    Uniform 2D grid storing road-derived cost and flow data.

    Stages:
      1. Construct with GridBounds.
      2. Call rasterize_segment() for every road edge.
      3. Call finalize_cost_field() once all edges are ingested.
      4. Downstream: integration field and flow field are stored here too
         (written by fields.py after construction).

    Array indexing: arr[row, col]  →  flat index = row * width + col
    """

    def __init__(self, bounds: GridBounds) -> None:
        self.bounds = bounds
        self.cell_size = bounds.cell_size
        self.width = bounds.width
        self.height = bounds.height

        # ------------------------------------------------------------------
        # Primary fields (allocated here, filled by pipeline)
        # ------------------------------------------------------------------

        # Best desirability seen per cell during rasterization (0 = no road)
        self._best_desirability = np.zeros((self.height, self.width), dtype=np.float32)

        # Cost field: traversal cost in seconds per metre.
        # cost[r,c] = cell_size / effective_speed_ms
        # Blocked cells have cost = +inf.
        self.cost = np.full((self.height, self.width), np.inf, dtype=np.float32)

        # Blocked mask: True = no usable road through this cell.
        self.blocked = np.ones((self.height, self.width), dtype=np.bool_)

        # Integration field (written by compute_integration_field):
        #   integration[r,c] = min total cost to reach destination region.
        self.integration = np.full((self.height, self.width), np.inf, dtype=np.float64)

        # Flow field vectors (written by compute_flow_field):
        #   (flow_x[r,c], flow_y[r,c]) is the normalised direction a traversable
        #   cell should steer toward. Units: grid cells (not metres).
        self.flow_x = np.zeros((self.height, self.width), dtype=np.float32)
        self.flow_y = np.zeros((self.height, self.width), dtype=np.float32)

    # ------------------------------------------------------------------
    # Coordinate conversion
    # ------------------------------------------------------------------

    def world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        """World metres → integer grid cell (col, row). May be out of bounds."""
        col = int((x - self.bounds.min_x) / self.cell_size)
        row = int((y - self.bounds.min_y) / self.cell_size)
        return col, row

    def world_to_cell_f(self, x: float, y: float) -> Tuple[float, float]:
        """World metres → continuous grid coordinates (col_f, row_f)."""
        col_f = (x - self.bounds.min_x) / self.cell_size
        row_f = (y - self.bounds.min_y) / self.cell_size
        return col_f, row_f

    def cell_to_world(self, col: int, row: int) -> Tuple[float, float]:
        """Grid cell centre → world metres."""
        x = self.bounds.min_x + (col + 0.5) * self.cell_size
        y = self.bounds.min_y + (row + 0.5) * self.cell_size
        return x, y

    def in_bounds(self, col: int, row: int) -> bool:
        return 0 <= col < self.width and 0 <= row < self.height

    # ------------------------------------------------------------------
    # Rasterization
    # ------------------------------------------------------------------

    def rasterize_segment(self,
                          x0: float, y0: float,
                          x1: float, y1: float,
                          desirability: float) -> None:
        """
        Paint a road segment (world metres) into the grid.

        Uses Bresenham's line algorithm in grid space.
        Each covered cell gets the maximum desirability it has seen so far,
        so the best road class "wins" when multiple roads cross a cell.

        Args:
            x0, y0      : start point in world metres
            x1, y1      : end point   in world metres
            desirability: value in (0, 1] — higher is better / cheaper to traverse
        """
        c0, r0 = self.world_to_cell(x0, y0)
        c1, r1 = self.world_to_cell(x1, y1)

        for col, row in _bresenham(c0, r0, c1, r1):
            if self.in_bounds(col, row):
                if desirability > self._best_desirability[row, col]:
                    self._best_desirability[row, col] = desirability

    def finalize_cost_field(self) -> None:
        """
        Convert accumulated desirability scores into the cost field.

        Cost formula (time-based):
            effective_speed (m/s) = desirability * MAX_SPEED_KPH / 3.6
            cost (s/cell)         = cell_size / effective_speed

        This makes cost physically meaningful as traversal time in seconds.
        Cells with no road keep cost = inf and blocked = True.
        """
        road_mask = self._best_desirability > 0.0
        self.blocked = ~road_mask

        eff_speed_ms = (
            self._best_desirability[road_mask].astype(np.float32)
            * (MAX_SPEED_KPH / 3.6)
        )
        # Avoid divide-by-zero (desirability > 0 guaranteed here)
        self.cost[road_mask] = self.cell_size / eff_speed_ms
        self.cost[~road_mask] = np.inf

    # ------------------------------------------------------------------
    # Convenience: desirability score from road attributes
    # ------------------------------------------------------------------

    @staticmethod
    def road_desirability(speed_kph: float,
                          lanes: int,
                          road_type: str) -> float:
        """
        Combine road attributes into a single desirability score in (0, 1].

        Formula:
            speed_f = clamp(speed_kph / MAX_SPEED_KPH, 0.05, 1.0)
            lane_f  = clamp(lanes / MAX_LANES,          0.25, 1.0)
            type_f  = ROAD_TYPE_FACTORS.get(road_type, DEFAULT)
            desirability = (speed_f + lane_f + type_f) / 3

        Using an arithmetic mean keeps the formula simple, bounded in (0,1],
        and easy to tune by adjusting individual factor tables.
        """
        speed_f = min(1.0, max(0.05, speed_kph / MAX_SPEED_KPH))
        lane_f  = min(1.0, max(0.25, lanes / MAX_LANES))
        type_f  = ROAD_TYPE_FACTORS.get(road_type, DEFAULT_ROAD_TYPE_FACTOR)
        return (speed_f + lane_f + type_f) / 3.0


# ---------------------------------------------------------------------------
# Bresenham's line (module-private)
# ---------------------------------------------------------------------------

def _bresenham(c0: int, r0: int, c1: int, r1: int) -> List[Tuple[int, int]]:
    """
    Enumerate integer grid cells along the line from (c0,r0) to (c1,r1).
    Standard Bresenham algorithm; O(max(|dc|, |dr|)) cells.
    """
    cells: List[Tuple[int, int]] = []
    dc = abs(c1 - c0)
    dr = abs(r1 - r0)
    sc = 1 if c1 > c0 else -1
    sr = 1 if r1 > r0 else -1
    err = dc - dr
    c, r = c0, r0
    while True:
        cells.append((c, r))
        if c == c1 and r == r1:
            break
        e2 = 2 * err
        if e2 > -dr:
            err -= dr
            c += sc
        if e2 < dc:
            err += dc
            r += sr
    return cells
