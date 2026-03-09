"""
flow_field/agents.py — Agent state, bilinear flow sampling, spatial hash,
                       local separation, hybrid last-mile transition.

Agent movement model
--------------------
Each agent has a continuous position in *grid-space* (col_f, row_f).
Every time-step dt the agent:

  1. Determines its movement mode (FLOW | LAST_MILE | ARRIVED).
  2. Computes a base steering direction:
       FLOW mode      → bilinear sample of (flow_x, flow_y) at its position.
       LAST_MILE mode → direct unit vector from agent to its destination.
  3. Adds a lightweight separation force from nearby agents (spatial hash).
  4. Combines and renormalises: final_dir = flow_weight·flow + sep_weight·sep
  5. Advances position:   col += final_dir_x · speed · dt
                          row += final_dir_y · speed · dt
  6. Rejects the step if the target cell is blocked (agent stays put).

Bilinear interpolation
----------------------
The flow field stores one vector per cell.  We treat those vectors as
samples at the *centres* of cells, i.e. the sample at cell (c, r) sits at
continuous coordinate (c + 0.5, r + 0.5).

For an agent at continuous position (p_col, p_row):
  shifted_col = p_col - 0.5       ← align to cell-centre grid
  shifted_row = p_row - 0.5
  c0, r0      = floor(shifted_col), floor(shifted_row)
  fx = shifted_col - c0           ← fractional offset in [0, 1)
  fy = shifted_row - r0

  v = (1-fx)(1-fy)·v[r0,c0]   +   fx(1-fy)·v[r0,c1]
    + (1-fx)fy ·v[r1,c0]   +   fx·fy   ·v[r1,c1]

  where c1 = c0+1, r1 = r0+1 (clamped to grid bounds).

This gives smooth, continuous steering directions with no visible snapping
between cells.

Spatial hash for separation
----------------------------
All active agents are inserted into a uniform spatial hash (bucket_size
defaults to 2 cell-widths).  To find neighbours of an agent at (c, r) we
only check the 3×3 = 9 surrounding buckets.  This is O(1) amortised per
agent (assuming roughly uniform density) compared with O(N) naive search.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np


# ---------------------------------------------------------------------------
# Agent
# ---------------------------------------------------------------------------

@dataclass
class Agent:
    """
    Single simulation agent.

    Positions are in continuous grid-space (col_f, row_f).
    Destination likewise stored as grid-space floats.
    """
    agent_id:       int
    col:            float          # current column (grid-space)
    row:            float          # current row    (grid-space)
    dst_col:        float          # destination column
    dst_row:        float          # destination row
    speed:          float = 2.0    # cells per second
    mode:           str   = "FLOW" # "FLOW" | "LAST_MILE" | "ARRIVED"

    # Distance threshold (grid cells) to switch from FLOW to LAST_MILE
    last_mile_radius: float = 8.0


# ---------------------------------------------------------------------------
# Spatial hash
# ---------------------------------------------------------------------------

class SpatialHash:
    """
    Uniform-grid spatial hash for O(1)-per-agent local neighbour queries.

    bucket_size : size of one bucket in grid-cell units.
                  Should equal (or be slightly larger than) the separation
                  radius so that all potential neighbours fit in the 3×3
                  neighbourhood of buckets.
    """

    def __init__(self, bucket_size: float = 3.0) -> None:
        self.bucket_size = bucket_size
        self._buckets: Dict[Tuple[int, int], List[int]] = {}

    # ---- internal ----

    def _key(self, col: float, row: float) -> Tuple[int, int]:
        return (
            int(math.floor(col / self.bucket_size)),
            int(math.floor(row / self.bucket_size)),
        )

    # ---- public API ----

    def clear(self) -> None:
        self._buckets.clear()

    def insert(self, agent_id: int, col: float, row: float) -> None:
        key = self._key(col, row)
        bucket = self._buckets.get(key)
        if bucket is None:
            self._buckets[key] = [agent_id]
        else:
            bucket.append(agent_id)

    def query_neighbors(self, col: float, row: float) -> List[int]:
        """
        Return all agent IDs in the 3×3 neighbourhood of buckets around
        (col, row).  Includes the agent itself — callers must filter by ID.
        """
        bx, by = self._key(col, row)
        result: List[int] = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                bucket = self._buckets.get((bx + dx, by + dy))
                if bucket:
                    result.extend(bucket)
        return result


# ---------------------------------------------------------------------------
# Bilinear flow sampling
# ---------------------------------------------------------------------------

def sample_flow_bilinear(
    flow_x: np.ndarray,
    flow_y: np.ndarray,
    col: float,
    row: float,
) -> Tuple[float, float]:
    """
    Bilinearly interpolate the flow field at continuous position (col, row).

    The flow vectors are treated as cell-centre samples.  We shift the
    query position by –0.5 to align with those centres, then do standard
    bi-linear interpolation over the four surrounding sample points.

    Returns:
        (vx, vy) — normalised direction vector.  (0, 0) if the field is zero
        at this position (blocked or unreachable area).
    """
    height, width = flow_x.shape

    # Align to cell-centre lattice
    sc = col - 0.5
    sr = row - 0.5

    # Clamp to valid sampling range
    sc = max(0.0, min(sc, width  - 1.001))
    sr = max(0.0, min(sr, height - 1.001))

    c0 = int(math.floor(sc))
    r0 = int(math.floor(sr))
    c1 = min(c0 + 1, width  - 1)
    r1 = min(r0 + 1, height - 1)

    fx = sc - c0   # fractional East  (0…1)
    fy = sr - r0   # fractional North (0…1)

    # Bilinear blend over the four corners
    w00 = (1.0 - fx) * (1.0 - fy)
    w10 =        fx  * (1.0 - fy)
    w01 = (1.0 - fx) *        fy
    w11 =        fx  *        fy

    vx = (w00 * flow_x[r0, c0] + w10 * flow_x[r0, c1] +
          w01 * flow_x[r1, c0] + w11 * flow_x[r1, c1])
    vy = (w00 * flow_y[r0, c0] + w10 * flow_y[r0, c1] +
          w01 * flow_y[r1, c0] + w11 * flow_y[r1, c1])

    # Renormalise (bilinear blending can shrink the magnitude)
    mag = math.sqrt(vx * vx + vy * vy)
    if mag > 1e-6:
        vx /= mag
        vy /= mag
    return vx, vy


# ---------------------------------------------------------------------------
# Separation force
# ---------------------------------------------------------------------------

def _separation_force(
    agent: Agent,
    neighbor_agents: List[Agent],
    radius: float,
    strength: float,
) -> Tuple[float, float]:
    """
    Lightweight linear-falloff repulsion from neighbours within *radius*.

    Returns (sx, sy) — unnormalised separation force vector.
    Not normalised here so the caller can weight it against the flow vector.
    """
    sx = sy = 0.0
    for other in neighbor_agents:
        if other.agent_id == agent.agent_id:
            continue
        dx = agent.col - other.col
        dy = agent.row - other.row
        dist_sq = dx * dx + dy * dy
        if 0.0 < dist_sq < radius * radius:
            dist = math.sqrt(dist_sq)
            # Linear falloff: strongest when dist→0, zero at dist=radius
            f = strength * (1.0 - dist / radius)
            sx += (dx / dist) * f
            sy += (dy / dist) * f
    return sx, sy


# ---------------------------------------------------------------------------
# Per-frame batch update
# ---------------------------------------------------------------------------

def update_agents(
    agents:             List[Agent],
    flow_x:             np.ndarray,
    flow_y:             np.ndarray,
    blocked:            np.ndarray,
    dt:                 float,
    spatial_hash:       SpatialHash,
    *,
    integration:        Optional[np.ndarray] = None,
    separation_radius:  float = 2.0,
    separation_strength: float = 0.5,
    flow_weight:        float = 0.85,
    sep_weight:         float = 0.15,
) -> None:
    """
    Advance every agent by *dt* seconds.

    Steps:
      1. Rebuild the spatial hash from current positions (O(N)).
      2. For each active agent:
           a. Evaluate mode transition (FLOW → LAST_MILE → ARRIVED).
           b. Sample steering direction (flow field or direct vector).
           c. Compute separation force from the spatial hash.
           d. Blend and renormalise the final direction.
           e. Attempt to move; reject if the target cell is blocked.

    The spatial hash is rebuilt every frame because agents move every frame.
    The total cost is O(N) build + O(k) per agent (k = average neighbours),
    giving effectively O(N) total, which scales well.

    Args:
        agents             : list of Agent objects to update in place.
        flow_x, flow_y     : flow field arrays (height × width).
        blocked            : blocked mask (height × width).
        dt                 : time step in seconds.
        spatial_hash       : pre-allocated SpatialHash (cleared and rebuilt here).
        separation_radius  : neighbourhood radius for repulsion (grid cells).
        separation_strength: repulsion magnitude.
        flow_weight        : blend weight for the flow/last-mile component.
        sep_weight         : blend weight for the separation component.
    """
    height, width = flow_x.shape

    # ---- 1. Rebuild spatial hash ----
    spatial_hash.clear()
    id_map: Dict[int, Agent] = {}
    for agent in agents:
        id_map[agent.agent_id] = agent
        if agent.mode != "ARRIVED":
            spatial_hash.insert(agent.agent_id, agent.col, agent.row)

    # ---- 2. Update each agent ----
    for agent in agents:
        if agent.mode == "ARRIVED":
            continue

        # Distance to destination (grid cells)
        ddx = agent.dst_col - agent.col
        ddy = agent.dst_row - agent.row
        dist = math.sqrt(ddx * ddx + ddy * ddy)

        # Arrival check — use a generous radius so agents don't get stuck
        # trying to reach the exact centre through blocked cells.
        arrival_radius = max(1.5, agent.last_mile_radius * 0.2)
        if dist < arrival_radius:
            agent.mode = "ARRIVED"
            continue

        # Mode transition: enter last-mile when close enough OR when the
        # agent has reached the destination region (integration == 0).
        # The integration field is flat inside Area B, so flow vectors there
        # are meaningless — we must switch to direct steering.
        if dist < agent.last_mile_radius:
            agent.mode = "LAST_MILE"
        elif integration is not None:
            # Use round() so positions like 69.999 map to cell 70, not 69.
            cur_c = max(0, min(round(agent.col), flow_x.shape[1] - 1))
            cur_r = max(0, min(round(agent.row), flow_x.shape[0] - 1))
            if integration[cur_r, cur_c] == 0.0:
                agent.mode = "LAST_MILE"

        # ---- Steering direction ----
        if agent.mode == "LAST_MILE":
            # Direct vector toward exact destination
            vx = ddx / dist
            vy = ddy / dist
        else:
            # Bilinear sample from shared flow field (O(1))
            vx, vy = sample_flow_bilinear(flow_x, flow_y, agent.col, agent.row)
            # Fallback to direct steering if agent is in an unreachable zone
            if abs(vx) < 1e-6 and abs(vy) < 1e-6:
                vx = ddx / dist
                vy = ddy / dist

        # ---- Separation force ----
        neighbor_ids = spatial_hash.query_neighbors(agent.col, agent.row)
        neighbors = [id_map[nid] for nid in neighbor_ids if nid in id_map]
        sx, sy = _separation_force(agent, neighbors,
                                   separation_radius, separation_strength)

        # ---- Blend ----
        fx = vx * flow_weight + sx * sep_weight
        fy = vy * flow_weight + sy * sep_weight

        # Renormalise combined direction
        mag = math.sqrt(fx * fx + fy * fy)
        if mag > 1e-6:
            fx /= mag
            fy /= mag

        # ---- Move ----
        new_col = agent.col + fx * agent.speed * dt
        new_row = agent.row + fy * agent.speed * dt

        # Clamp to grid
        new_col = max(0.0, min(new_col, width  - 1.0))
        new_row = max(0.0, min(new_row, height - 1.0))

        # Blocked-cell rejection with sliding fallback.
        # Use round() so positions like 69.999 correctly resolve to cell 70.
        # Try full move → slide along X only → slide along Y only → stay.
        def _try_move(nc: float, nr: float) -> bool:
            nc_i = max(0, min(round(nc), width  - 1))
            nr_i = max(0, min(round(nr), height - 1))
            if not blocked[nr_i, nc_i]:
                agent.col = max(0.0, min(nc, width  - 1.0))
                agent.row = max(0.0, min(nr, height - 1.0))
                return True
            return False

        if not _try_move(new_col, new_row):
            # Try sliding along each axis independently
            if not _try_move(new_col, agent.row):
                _try_move(agent.col, new_row)
        # If all three fail, agent stays put this frame


# ---------------------------------------------------------------------------
# Factory: create agents from world-metre coordinates
# ---------------------------------------------------------------------------

def make_agents(
    start_positions_m: List[Tuple[float, float]],
    destination_m: Tuple[float, float],
    grid,
    speed: float = 2.0,
    last_mile_radius: float = 8.0,
) -> List[Agent]:
    """
    Construct Agent objects from world-metre start positions and a shared
    destination point in world metres.

    Args:
        start_positions_m : list of (x_m, y_m) origin positions
        destination_m     : (x_m, y_m) single destination (centre of Area B)
        grid              : a flow_field.grid.Grid instance
        speed             : agent speed in grid cells per second
        last_mile_radius  : cells to destination at which LAST_MILE activates

    Returns:
        list of Agent objects ready for update_agents().
    """
    dst_col, dst_row = grid.world_to_cell_f(*destination_m)
    agents = []
    for idx, (x, y) in enumerate(start_positions_m):
        col, row = grid.world_to_cell_f(x, y)
        agents.append(Agent(
            agent_id=idx,
            col=col,
            row=row,
            dst_col=dst_col,
            dst_row=dst_row,
            speed=speed,
            last_mile_radius=last_mile_radius,
        ))
    return agents
