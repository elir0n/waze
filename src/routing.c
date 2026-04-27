#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include <string.h>
#include "graph.h"
#include "min_heap.h"
#include "routing.h"

/* ── RouteContext lifecycle ─────────────────────────────────────────────── */

RouteContext* route_context_create(int num_nodes)
{
    RouteContext* ctx = (RouteContext*)malloc(sizeof(RouteContext));
    if (!ctx) return NULL;

    ctx->capacity    = num_nodes;
    ctx->g_score     = (double*)malloc(sizeof(double) * num_nodes);
    ctx->f_score     = (double*)malloc(sizeof(double) * num_nodes);
    ctx->parent      = (int*)   malloc(sizeof(int)    * num_nodes);
    ctx->node_path   = (int*)   malloc(sizeof(int)    * num_nodes);
    ctx->path_edges  = (int*)   malloc(sizeof(int)    * num_nodes);
    ctx->h_heap      = (int*)   malloc(sizeof(int)    * num_nodes);
    ctx->h_key       = (double*)malloc(sizeof(double) * num_nodes);
    ctx->h_pos       = (int*)   malloc(sizeof(int)    * num_nodes);

    if (!ctx->g_score || !ctx->f_score || !ctx->parent || !ctx->node_path ||
        !ctx->path_edges || !ctx->h_heap || !ctx->h_key || !ctx->h_pos) {
        route_context_free(ctx);
        return NULL;
    }
    return ctx;
}

void route_context_free(RouteContext* ctx)
{
    if (!ctx) return;
    free(ctx->g_score);
    free(ctx->f_score);
    free(ctx->parent);
    free(ctx->node_path);
    free(ctx->path_edges);
    free(ctx->h_heap);
    free(ctx->h_key);
    free(ctx->h_pos);
    free(ctx);
}

/* ── Flat binary min-heap helpers (no per-node malloc) ──────────────────── */

static void fh_reset(RouteContext* ctx, int n)
{
    for (int i = 0; i < n; i++) {
        ctx->h_heap[i] = i;
        ctx->h_key[i]  = DBL_MAX;
        ctx->h_pos[i]  = i;
    }
    ctx->h_size = n;
}

static void fh_swap(RouteContext* ctx, int i, int j)
{
    int ni = ctx->h_heap[i], nj = ctx->h_heap[j];
    ctx->h_heap[i] = nj;
    ctx->h_heap[j] = ni;
    ctx->h_pos[nj] = i;
    ctx->h_pos[ni] = j;
}

static void fh_sift_up(RouteContext* ctx, int i)
{
    while (i > 0) {
        int p = (i - 1) / 2;
        if (ctx->h_key[ctx->h_heap[i]] < ctx->h_key[ctx->h_heap[p]]) {
            fh_swap(ctx, i, p);
            i = p;
        } else break;
    }
}

static void fh_sift_down(RouteContext* ctx, int i)
{
    int n = ctx->h_size;
    while (1) {
        int sm = i, l = 2*i+1, r = 2*i+2;
        if (l < n && ctx->h_key[ctx->h_heap[l]] < ctx->h_key[ctx->h_heap[sm]]) sm = l;
        if (r < n && ctx->h_key[ctx->h_heap[r]] < ctx->h_key[ctx->h_heap[sm]]) sm = r;
        if (sm == i) break;
        fh_swap(ctx, i, sm);
        i = sm;
    }
}

/* Returns node_id of the minimum element; removes it from heap. Returns -1 if empty. */
static int fh_extract_min(RouteContext* ctx, double* out_key)
{
    if (ctx->h_size == 0) return -1;
    int root = ctx->h_heap[0];
    *out_key  = ctx->h_key[root];
    /* Move last element to root and sift down */
    ctx->h_size--;
    if (ctx->h_size > 0) {
        ctx->h_heap[0] = ctx->h_heap[ctx->h_size];
        ctx->h_pos[ctx->h_heap[0]] = 0;
        fh_sift_down(ctx, 0);
    }
    ctx->h_pos[root] = ctx->capacity; /* sentinel: extracted */
    return root;
}

static void fh_decrease_key(RouteContext* ctx, int node_id, double key)
{
    int i = ctx->h_pos[node_id];
    if (i >= ctx->h_size) return; /* already extracted */
    ctx->h_key[node_id] = key;
    fh_sift_up(ctx, i);
}

/* ── Path-finding helpers ─────────────────────────────────────────────────── */

static void print_path(int current_node, const int* parent)
{
    if (current_node == -1) return;
    print_path(parent[current_node], parent);
    printf("%d ", current_node);
}

/* Helper: find edge_id for directed edge from → to. Returns -1 if not found. */
static int find_edge_id(Graph* g, int from, int to)
{
    EdgeNode* cur = g->nodes[from].out_edges;
    while (cur) {
        int eid = cur->edge_id;
        if (eid >= 0 && eid < g->num_edges && g->edges[eid].to_node == to)
            return eid;
        cur = cur->next;
    }
    return -1;
}

/* ── A* (printing variant) ───────────────────────────────────────────────── */

void find_route_a_star(Graph* graph, int start_id, int target_id)
{
    if (!graph) { fprintf(stderr, "find_route_a_star: graph is NULL\n"); return; }
    if (start_id < 0 || start_id >= graph->num_nodes ||
        target_id < 0 || target_id >= graph->num_nodes) {
        fprintf(stderr, "find_route_a_star: invalid start/target\n");
        return;
    }

    int V = graph->num_nodes;
    double* g_score = (double*)malloc(sizeof(double) * V);
    double* f_score = (double*)malloc(sizeof(double) * V);
    int*    parent  = (int*)   malloc(sizeof(int)    * V);
    if (!g_score || !f_score || !parent) {
        free(g_score); free(f_score); free(parent);
        return;
    }

    MinHeap* minHeap = createMinHeap(V);
    if (!minHeap) { free(g_score); free(f_score); free(parent); return; }

    for (int i = 0; i < V; i++) {
        g_score[i] = f_score[i] = DBL_MAX;
        parent[i]  = -1;
        minHeap->array[i] = newMinHeapNode(i, DBL_MAX);
        minHeap->pos[i]   = i;
    }
    minHeap->size = V;

    g_score[start_id] = 0.0;
    f_score[start_id] = heuristic(graph, start_id, target_id);
    decreaseKey(minHeap, start_id, f_score[start_id]);

    printf("Starting A* Search from %d to %d...\n", start_id, target_id);

    while (!isEmpty(minHeap)) {
        MinHeapNode* minNode = extractMin(minHeap);
        if (!minNode) break;
        int    u   = minNode->node_id;
        double u_f = minNode->dist;
        if (u_f == DBL_MAX) { free(minNode); break; }

        if (u == target_id) {
            printf("Destination reached! Cost: %.4f\n", g_score[u]);
            printf("Path: ");
            print_path(target_id, parent);
            printf("\n");
            free(minNode);
            break;
        }

        EdgeNode* curr = graph->nodes[u].out_edges;
        while (curr) {
            int eid = curr->edge_id;
            if (eid >= 0 && eid < graph->num_edges) {
                int v = graph->edges[eid].to_node;
                if (v >= 0 && v < V && g_score[u] != DBL_MAX) {
                    double tg = g_score[u] + get_edge_weight(graph, eid);
                    if (tg < g_score[v]) {
                        g_score[v] = tg;
                        f_score[v] = tg + heuristic(graph, v, target_id);
                        parent[v]  = u;
                        if (isInMinHeap(minHeap, v))
                            decreaseKey(minHeap, v, f_score[v]);
                    }
                }
            }
            curr = curr->next;
        }
        free(minNode);
    }

    free(g_score); free(f_score); free(parent);
    freeMinHeap(minHeap);
}

/* ── A* path-returning variant ───────────────────────────────────────────── */

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
                           RouteContext* ctx)
{
    if (!graph || !out_cost || !out_edges || !out_edge_count) return 10;
    if (start_id < 0 || start_id >= graph->num_nodes ||
        target_id < 0 || target_id >= graph->num_nodes)
        return 11;

    int V = graph->num_nodes;

    /* ── Scratch space: use caller's ctx when provided, else malloc ── */
    int ctx_owned = (ctx == NULL);
    double* g_score;
    double* f_score;
    int*    parent;
    int*    node_path;

    if (ctx_owned) {
        g_score   = (double*)malloc(sizeof(double) * V);
        f_score   = (double*)malloc(sizeof(double) * V);
        parent    = (int*)   malloc(sizeof(int)    * V);
        node_path = (int*)   malloc(sizeof(int)    * V);
        if (!g_score || !f_score || !parent || !node_path) {
            free(g_score); free(f_score); free(parent); free(node_path);
            return 12;
        }
    } else {
        g_score   = ctx->g_score;
        f_score   = ctx->f_score;
        parent    = ctx->parent;
        node_path = ctx->node_path;
    }

    /* ── Initialise scores ── */
    for (int i = 0; i < V; i++) {
        g_score[i] = DBL_MAX;
        f_score[i] = DBL_MAX;
        parent[i]  = -1;
    }

    g_score[start_id] = 0.0;
    f_score[start_id] = heuristic(graph, start_id, target_id);

    int found = 0;

    /* ── Main loop: two code paths (flat heap vs legacy heap) ── */
    if (!ctx_owned) {
        /* ── Fast path: flat heap, no per-node malloc ── */
        fh_reset(ctx, V);
        fh_decrease_key(ctx, start_id, f_score[start_id]);

        while (ctx->h_size > 0) {
            double u_f;
            int u = fh_extract_min(ctx, &u_f);
            if (u < 0 || u_f == DBL_MAX) break;

            if (u == target_id) { found = 1; break; }

            EdgeNode* curr = graph->nodes[u].out_edges;
            while (curr) {
                int eid = curr->edge_id;
                if (eid >= 0 && eid < graph->num_edges) {
                    int    v = graph->edges[eid].to_node;
                    double w = get_edge_weight(graph, eid);
                    if (v >= 0 && v < V && g_score[u] != DBL_MAX) {
                        double tg = g_score[u] + w;
                        if (tg < g_score[v]) {
                            g_score[v] = tg;
                            f_score[v] = tg + heuristic(graph, v, target_id);
                            parent[v]  = u;
                            fh_decrease_key(ctx, v, f_score[v]);
                        }
                    }
                }
                curr = curr->next;
            }
        }
    } else {
        /* ── Fallback path: legacy MinHeap ── */
        MinHeap* minHeap = createMinHeap(V);
        if (!minHeap) {
            free(g_score); free(f_score); free(parent); free(node_path);
            return 13;
        }
        for (int i = 0; i < V; i++) {
            minHeap->array[i] = newMinHeapNode(i, DBL_MAX);
            minHeap->pos[i]   = i;
        }
        minHeap->size = V;
        decreaseKey(minHeap, start_id, f_score[start_id]);

        while (!isEmpty(minHeap)) {
            MinHeapNode* minNode = extractMin(minHeap);
            if (!minNode) break;
            int    u   = minNode->node_id;
            double u_f = minNode->dist;
            free(minNode);
            if (u_f == DBL_MAX) break;

            if (u == target_id) { found = 1; break; }

            EdgeNode* curr = graph->nodes[u].out_edges;
            while (curr) {
                int eid = curr->edge_id;
                if (eid >= 0 && eid < graph->num_edges) {
                    int    v = graph->edges[eid].to_node;
                    double w = get_edge_weight(graph, eid);
                    if (v >= 0 && v < V && g_score[u] != DBL_MAX) {
                        double tg = g_score[u] + w;
                        if (tg < g_score[v]) {
                            g_score[v] = tg;
                            f_score[v] = tg + heuristic(graph, v, target_id);
                            parent[v]  = u;
                            if (isInMinHeap(minHeap, v))
                                decreaseKey(minHeap, v, f_score[v]);
                        }
                    }
                }
                curr = curr->next;
            }
        }
        freeMinHeap(minHeap);
    }

    if (!found) {
        if (ctx_owned) { free(g_score); free(f_score); free(parent); free(node_path); }
        return 1;
    }

    /* ── Reconstruct node path ── */
    int path_len = 0;
    for (int v = target_id; v != -1; v = parent[v])
        node_path[path_len++] = v;

    for (int i = 0; i < path_len / 2; i++) {
        int tmp = node_path[i];
        node_path[i]             = node_path[path_len - 1 - i];
        node_path[path_len-1-i]  = tmp;
    }

    if (out_nodes && out_node_count) {
        if (path_len > max_nodes) {
            if (ctx_owned) { free(g_score); free(f_score); free(parent); free(node_path); }
            return 17;
        }
        for (int i = 0; i < path_len; i++) out_nodes[i] = node_path[i];
        *out_node_count = path_len;
    }

    /* ── Convert node path to edge ids ── */
    int edge_count = 0;
    for (int i = 0; i < path_len - 1; i++) {
        int eid = find_edge_id(graph, node_path[i], node_path[i+1]);
        if (eid < 0) {
            if (ctx_owned) { free(g_score); free(f_score); free(parent); free(node_path); }
            return 15;
        }
        if (edge_count >= max_edges) {
            if (ctx_owned) { free(g_score); free(f_score); free(parent); free(node_path); }
            return 16;
        }
        out_edges[edge_count++] = eid;
    }

    *out_cost       = g_score[target_id];
    *out_edge_count = edge_count;

    if (ctx_owned) { free(g_score); free(f_score); free(parent); free(node_path); }
    return 0;
}
