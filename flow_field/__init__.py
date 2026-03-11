"""
flow_field — Flow Field Pathfinding for large-scale agent simulation.

Modules
-------
grid     : Grid, GridBounds, coordinate projection, road rasterization.
fields   : compute_integration_field (Dijkstra), compute_flow_field, build_fields.
loader   : load_graph_to_grid, cells_in_latlon_box, cells_in_world_box.
"""

from .grid   import Grid, GridBounds, bounds_from_latlon, latlon_to_local
from .fields import compute_integration_field, compute_flow_field, build_fields
from .loader import load_graph_to_grid, cells_in_world_box, cells_in_latlon_box

__all__ = [
    # grid
    "Grid", "GridBounds", "bounds_from_latlon", "latlon_to_local",
    # fields
    "compute_integration_field", "compute_flow_field", "build_fields",
    # loader
    "load_graph_to_grid", "cells_in_world_box", "cells_in_latlon_box",
]
