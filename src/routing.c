#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include <string.h>
#include "graph.h"
#include "min_heap.h"
#include "routing.h"

/*
// heuristic is on Euclidean distance
double get_heuristic(Node* a, Node* b) {
    return sqrt(pow(a->x - b->x, 2) + pow(a->y - b->y, 2));
}

// finel print
void print_path(int current_node, int* parent) {
    if (current_node == -1) {
        return;
    }
    print_path(parent[current_node], parent);
    printf("%d -> ", current_node);
}

// A* Search
void find_route_a_star(Graph* graph, int start_id, int target_id) {
    int V = graph->num_nodes;
    
    double* g_score = (double*)malloc(V * sizeof(double));
    double* f_score = (double*)malloc(V * sizeof(double));
    int* parent = (int*)malloc(V * sizeof(int));

    MinHeap* minHeap = createMinHeap(V);

    // initialize
    for (int i = 0; i < V; i++) {
        g_score[i] = DBL_MAX;
        f_score[i] = DBL_MAX;
        parent[i] = -1;
        
        // initialize each node to infinity
        minHeap->array[i] = newMinHeapNode(i, DBL_MAX);
        minHeap->pos[i] = i;
    }
    minHeap->size = V; 

    //  start node
    g_score[start_id] = 0.0;
    f_score[start_id] = get_heuristic(&graph->nodes[start_id], &graph->nodes[target_id]);
    decreaseKey(minHeap, start_id, f_score[start_id]);

    printf("Starting A* Search from node %d to %d...\n", start_id, target_id);

    while (!isEmpty(minHeap)) {
        // extractMin
        MinHeapNode* minNode = extractMin(minHeap);
        int u = minNode->node_id;

        
        if (g_score[u] == DBL_MAX) break;

        // in the target
        if (u == target_id) {
            printf("\nDestination reached! Cost: %.2f\n", g_score[u]);
            printf("Path: ");
            print_path(target_id, parent);
            printf("END\n");
            
            free(g_score); free(f_score); free(parent);
            freeMinHeap(minHeap);free(minNode);
            return;
        }

        // explore neighbors
        EdgeNode* curr_edge = graph->nodes[u].out_edges;
        while (curr_edge != NULL) {
            int v = curr_edge->edge_data.to_node;
            double weight = curr_edge->edge_data.current_travel_time;

            // g score
            double tentative_g = g_score[u] + weight;

            if (tentative_g < g_score[v]) {
                // found a better path
                g_score[v] = tentative_g;
                double h = get_heuristic(&graph->nodes[v], &graph->nodes[target_id]);
                f_score[v] = g_score[v] + h;
                parent[v] = u;

                // update in min heap
                if (isInMinHeap(minHeap, v)) {
                    decreaseKey(minHeap, v, f_score[v]);
                }
            }
            curr_edge = curr_edge->next;
        }
    }

    printf("No path found.\n");
    free(g_score); free(f_score); free(parent); 
 
}
    */



/*
 * Print path from start to current_node using parent[] array
 */
static void print_path(int current_node, const int* parent)
{
    if (current_node == -1) return;
    print_path(parent[current_node], parent);
    printf("%d ", current_node);
}

/*
 * A* Search
 * Finds route from start_id to target_id using:
 *  - g_score: cost-so-far
 *  - f_score: g_score + heuristic
 *
 * Graph neighbors:
 *  - adjacency list: g->nodes[u].out_edges (list of edge_ids)
 * Edge weight:
 *  - g->edges[edge_id].current_travel_time via get_edge_weight()
 */
void find_route_a_star(Graph* graph, int start_id, int target_id)
{
    if (!graph) {
        fprintf(stderr, "find_route_a_star: graph is NULL\n");
        return;
    }

    if (start_id < 0 || start_id >= graph->num_nodes ||
        target_id < 0 || target_id >= graph->num_nodes) {
        fprintf(stderr, "find_route_a_star: invalid start/target\n");
        return;
    }

    int V = graph->num_nodes;

    double* g_score = (double*)malloc(sizeof(double) * V);
    double* f_score = (double*)malloc(sizeof(double) * V);
    int* parent      = (int*)malloc(sizeof(int) * V);

    if (!g_score || !f_score || !parent) {
        fprintf(stderr, "find_route_a_star: malloc failed\n");
        free(g_score); free(f_score); free(parent);
        return;
    }

    MinHeap* minHeap = createMinHeap(V);
    if (!minHeap) {
        fprintf(stderr, "find_route_a_star: createMinHeap failed\n");
        free(g_score); free(f_score); free(parent);
        return;
    }

    /* Initialize */
    for (int i = 0; i < V; i++) {
        g_score[i] = DBL_MAX;
        f_score[i] = DBL_MAX;
        parent[i] = -1;

        minHeap->array[i] = newMinHeapNode(i, DBL_MAX);
        minHeap->pos[i] = i;
    }
    minHeap->size = V;

    /* Start node */
    g_score[start_id] = 0.0;
    f_score[start_id] = heuristic(graph, start_id, target_id);
    decreaseKey(minHeap, start_id, f_score[start_id]);

    printf("Starting A* Search from %d to %d...\n", start_id, target_id);

    while (!isEmpty(minHeap)) {
        MinHeapNode* minNode = extractMin(minHeap);
        if (!minNode) break;

        int u = minNode->node_id;
        double u_f = minNode->dist;

        /* If u is unreachable */
        if (u_f == DBL_MAX) {
            free(minNode);
            break;
        }

        /* Goal reached */
        if (u == target_id) {
            printf("Destination reached! Cost: %.4f\n", g_score[u]);
            printf("Path: ");
            print_path(target_id, parent);
            printf("\n");

            free(minNode);
            free(g_score); free(f_score); free(parent);
            freeMinHeap(minHeap);
            return;
        }

        /* Explore neighbors via adjacency list */
        EdgeNode* curr = graph->nodes[u].out_edges;
        while (curr != NULL) {
            int edge_id = curr->edge_id;

            /* edge_id must be valid */
            if (edge_id < 0 || edge_id >= graph->num_edges) {
                curr = curr->next;
                continue;
            }

            int v = graph->edges[edge_id].to_node;         /* neighbor */
            double w = get_edge_weight(graph, edge_id);     /* weight */

            if (v < 0 || v >= V) {
                curr = curr->next;
                continue;
            }

            if (g_score[u] != DBL_MAX) {
                double tentative_g = g_score[u] + w;

                if (tentative_g < g_score[v]) {
                    g_score[v] = tentative_g;
                    double h = heuristic(graph, v, target_id);
                    f_score[v] = tentative_g + h;
                    parent[v] = u;

                    if (isInMinHeap(minHeap, v)) {
                        decreaseKey(minHeap, v, f_score[v]);
                    }
                }
            }

            curr = curr->next;
        }

        free(minNode);
    }

    printf("No path found.\n");

    free(g_score); free(f_score); free(parent);
    freeMinHeap(minHeap);
}

/* Helper: find edge_id for directed edge from 'from' to 'to'. Returns -1 if not found. */
static int find_edge_id(Graph* g, int from, int to)
{
    EdgeNode* cur = g->nodes[from].out_edges;
    while (cur) {
        int eid = cur->edge_id;
        if (eid >= 0 && eid < g->num_edges && g->edges[eid].to_node == to) {
            return eid;
        }
        cur = cur->next;
    }
    return -1;
}

/**
 * A* variant that returns cost and edge path instead of printing.
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
                           int* out_node_count)
{
    if (!graph || !out_cost || !out_edges || !out_edge_count) return 10;

    if (start_id < 0 || start_id >= graph->num_nodes ||
        target_id < 0 || target_id >= graph->num_nodes) {
        return 11;
    }

    int V = graph->num_nodes;

    double* g_score = (double*)malloc(sizeof(double) * V);
    double* f_score = (double*)malloc(sizeof(double) * V);
    int* parent      = (int*)malloc(sizeof(int) * V);

    if (!g_score || !f_score || !parent) {
        free(g_score); free(f_score); free(parent);
        return 12;
    }

    MinHeap* minHeap = createMinHeap(V);
    if (!minHeap) {
        free(g_score); free(f_score); free(parent);
        return 13;
    }

    for (int i = 0; i < V; i++) {
        g_score[i] = DBL_MAX;
        f_score[i] = DBL_MAX;
        parent[i] = -1;

        minHeap->array[i] = newMinHeapNode(i, DBL_MAX);
        minHeap->pos[i] = i;
    }
    minHeap->size = V;

    g_score[start_id] = 0.0;
    f_score[start_id] = heuristic(graph, start_id, target_id);
    decreaseKey(minHeap, start_id, f_score[start_id]);

    int found = 0;

    while (!isEmpty(minHeap)) {
        MinHeapNode* minNode = extractMin(minHeap);
        if (!minNode) break;

        int u = minNode->node_id;
        double u_f = minNode->dist;
        free(minNode);

        if (u_f == DBL_MAX) break;

        if (u == target_id) {
            found = 1;
            break;
        }

        EdgeNode* curr = graph->nodes[u].out_edges;
        while (curr != NULL) {
            int edge_id = curr->edge_id;

            if (edge_id < 0 || edge_id >= graph->num_edges) {
                curr = curr->next;
                continue;
            }

            int v = graph->edges[edge_id].to_node;         /* neighbor */
            double w = get_edge_weight(graph, edge_id);     /* weight */

            if (v < 0 || v >= V) {
                curr = curr->next;
                continue;
            }

            if (g_score[u] != DBL_MAX) {
                double tentative_g = g_score[u] + w;

                if (tentative_g < g_score[v]) {
                    g_score[v] = tentative_g;
                    double h = heuristic(graph, v, target_id);
                    f_score[v] = tentative_g + h;
                    parent[v] = u;

                    if (isInMinHeap(minHeap, v)) {
                        decreaseKey(minHeap, v, f_score[v]);
                    }
                }
            }

            curr = curr->next;
        }
    }

    if (!found) {
        free(g_score); free(f_score); free(parent);
        freeMinHeap(minHeap);
        return 1; /* no path */
    }

    /* Reconstruct node path from target back to start */
    int* node_path = (int*)malloc(sizeof(int) * V);
    if (!node_path) {
        free(g_score); free(f_score); free(parent);
        freeMinHeap(minHeap);
        return 14;
    }

    int path_len = 0;
    for (int v = target_id; v != -1; v = parent[v]) {
        node_path[path_len++] = v;
    }

    /* Reverse to get start -> target */
    for (int i = 0; i < path_len / 2; i++) {
        int tmp = node_path[i];
        node_path[i] = node_path[path_len - 1 - i];
        node_path[path_len - 1 - i] = tmp;
    }

    /* Copy node path to caller if requested */
    if (out_nodes && out_node_count) {
        if (path_len > max_nodes) {
            free(node_path);
            free(g_score); free(f_score); free(parent);
            freeMinHeap(minHeap);
            return 17; /* node buffer too small */
        }
        for (int i = 0; i < path_len; i++) {
            out_nodes[i] = node_path[i];
        }
        *out_node_count = path_len;
    }

    /* Convert node path to edge ids */
    int edge_count = 0;
    for (int i = 0; i < path_len - 1; i++) {
        int from = node_path[i];
        int to = node_path[i + 1];
        int eid = find_edge_id(graph, from, to);
        if (eid < 0) {
            free(node_path);
            free(g_score); free(f_score); free(parent);
            freeMinHeap(minHeap);
            return 15;
        }

        if (edge_count >= max_edges) {
            free(node_path);
            free(g_score); free(f_score); free(parent);
            freeMinHeap(minHeap);
            return 16; /* path too long for out_edges capacity */
        }

        out_edges[edge_count++] = eid;
    }

    free(node_path);

    *out_cost = g_score[target_id];
    *out_edge_count = edge_count;

    free(g_score); free(f_score); free(parent);
    freeMinHeap(minHeap);
    return 0;
}
