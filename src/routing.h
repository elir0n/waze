#ifndef ROUTING_H
#define ROUTING_H

#include "graph.h"

/*
 * Per-worker scratch space.  Allocated once at worker startup via
 * route_context_create(); passed to find_route_a_star_path() so that A*
 * avoids four malloc/free pairs per request.
 *
 * Contains:
 *   - flat arrays for g_score, f_score, parent, node_path
 *   - a flat binary min-heap (h_heap/h_key/h_pos) that never allocates
 *     individual heap nodes
 */
typedef struct {
    double* g_score;    /* g_score[node_id] */
    double* f_score;    /* f_score[node_id] */
    int*    parent;     /* parent[node_id]   */
    int*    node_path;  /* node_path[i] during path reconstruction */
    int*    path_edges; /* output edge-id buffer (capacity == num_nodes) */
    /* flat binary min-heap */
    int*    h_heap;     /* h_heap[i] = node_id at heap position i */
    double* h_key;      /* h_key[node_id] = current priority */
    int*    h_pos;      /* h_pos[node_id] = position in h_heap */
    int     h_size;
    int     capacity;   /* num_nodes at alloc time */
} RouteContext;

RouteContext* route_context_create(int num_nodes);
void          route_context_free(RouteContext* ctx);

/* Routing API */
void find_route_a_star(Graph* graph, int start_id, int target_id);

/**
 * A* that returns total cost and edge path.
 *  - out_cost: total travel time
 *  - out_edges: edge_ids along the path (src -> dst order)
 *  - max_edges: capacity of out_edges
 *  - out_edge_count: number of edges written
 *  - out_nodes / max_nodes / out_node_count: optional node path
 *  - ctx: optional per-worker scratch space (NULL = allocate internally)
 * Returns 0 on success, 1 if no path, non-zero on error.
 */
int find_route_a_star_path(Graph* graph,
                           int start_id,
                           int target_id,
                           double* out_cost,
                           int* out_edges,
                           int max_edges,
                           int* out_edge_count,
                           int* out_nodes,
                           int max_nodes,
                           int* out_node_count,
                           RouteContext* ctx);

#endif
