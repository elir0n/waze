#ifndef ROUTING_H
#define ROUTING_H

#include "graph.h"

/* Routing API */
void find_route_a_star(Graph* graph, int start_id, int target_id);
/**
 * A* that returns total cost and edge path.
 *  - out_cost: total travel time
 *  - out_edges: edge_ids along the path (src -> dst order)
 *  - max_edges: capacity of out_edges
 *  - out_edge_count: number of edges written
 * Returns 0 on success, 1 if no path, non-zero on error.
 */
int find_route_a_star_path(Graph* graph,
                           int start_id,
                           int target_id,
                           double* out_cost,
                           int* out_edges,
                           int max_edges,
                           int* out_edge_count);

#endif
