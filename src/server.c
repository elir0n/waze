#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <ctype.h>

#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <pthread.h>

#include "server.h"
#include "routing.h"

/* ---------------- configuration ---------------- */

#ifndef ROUTE_WORKERS
#define ROUTE_WORKERS 8
#endif

#ifndef TRAFFIC_WORKERS
#define TRAFFIC_WORKERS 2
#endif

#ifndef MAX_CARS
#define MAX_CARS 30000
#endif

#define MAX_ROUTE_WORKERS 256
#define SECTION_ROWS       5
#define SECTION_COLS       5
#define NUM_SECTIONS      25    /* SECTION_ROWS * SECTION_COLS */
#define MAX_BATCH         64    /* max cars per batch_routes request */
#define MAX_SEG_EDGES  16384    /* max edges per A* segment result */
#define N_TICK_WORKERS     4    /* parallel threads for handle_tick_all second pass */
#define SEG_WORKERS        8    /* persistent segment worker pool size */
#define N_EDGE_STRIPES    64    /* striped mutex buckets for traffic EMA updates */

typedef struct {
    double cost;
    int*   edges;       /* malloc'd; NULL if no path */
    int    edge_count;
} SectionPath;

/* SegTask: one A* segment job for the segment worker pool */
typedef struct SegTask {
    int              src;
    int              dst;
    Graph*           g;
    int*             out_edges;       /* pre-allocated by caller */
    int              out_edge_count;
    double           out_cost;
    int              rc;
    int*             pending;         /* atomic counter; decremented on finish */
    struct SegTask*  next;
} SegTask;

typedef struct {
    SegTask*        head;
    SegTask*        tail;
    pthread_mutex_t mu;
    pthread_cond_t  cv;
} SegTaskQueue;

typedef struct {
    SegTaskQueue*  q;
    RouteContext*  ctx;
} SegWorkerArg;

/* Congestion physics:
 * Road class (0–5) is derived from road_type and controls two parameters:
 *   meters_per_car — physical headway (m) at which the road reaches capacity:
 *                    8 m (residential) → 25 m (motorway)
 *   min_speed_frac — speed floor (jam minimum): 0.05 (residential) → 0.30 (motorway)
 *
 * Capacity = lanes * floor(base_length / meters_per_car(cls)), minimum 1 per lane.
 * This ensures a 500m road holds proportionally more cars than a 50m road.
 *
 * Speed drops linearly from base_speed (0 cars) to min_speed_frac*base_speed (at capacity).
 *
 * Congestion is reported to clients only when occupancy exceeds 30% of capacity. */

static int road_class(const char* road_type) {
    if (!road_type) return 0;
    if (strncmp(road_type, "motorway",  8) == 0) return 5;
    if (strncmp(road_type, "trunk",     5) == 0) return 4;
    if (strncmp(road_type, "primary",   7) == 0) return 3;
    if (strncmp(road_type, "secondary", 9) == 0) return 2;
    if (strncmp(road_type, "tertiary",  8) == 0) return 1;
    return 0;  /* residential, living_street, unclassified, road, … */
}

/* Average headway (metres/car) at capacity, by road class.
 * cls 0 residential: 8m, cls1 tertiary: 10m, cls2 secondary: 12m,
 * cls3 primary: 15m, cls4 trunk: 20m, cls5 motorway: 25m */
static double meters_per_car(int cls) {
    static const double tbl[6] = { 8.0, 10.0, 12.0, 15.0, 20.0, 25.0 };
    return tbl[(cls >= 0 && cls <= 5) ? cls : 0];
}

/* Physical capacity: how many cars fit on this edge (lanes × floor(length / headway)).
 * Minimum 1 per lane so very short OSM stub edges are handled correctly. */
static int edge_capacity(double length_m, int lanes, int cls) {
    int per_lane = (int)(length_m / meters_per_car(cls));
    if (per_lane < 1) per_lane = 1;
    int cap = lanes * per_lane;
    return (cap > 0) ? cap : 1;
}

/* Returns minimum speed fraction (jam floor) for a given road class. */
static double jam_floor(int cls) { return 0.05 + cls * 0.05; }

/* ---------------- vehicle state ---------------- */

typedef enum {
    CAR_IDLE    = 0,
    CAR_DRIVING = 1,
    CAR_ARRIVED = 2
} CarState;

typedef struct {
    int      car_id;
    int      user_id;
    int      active;        /* 0 = slot unused */
    CarState state;
    int*     route_edges;   /* malloc'd; NULL when idle */
    int      route_len;
    int      edge_idx;
    double   pos;           /* 0.0–1.0 along current edge */
    int      src_node;
    int      dst_node;
} VehicleState;

typedef struct {
    VehicleState    cars[MAX_CARS];
    int             count;
    int             max_active_slot; /* highest slot ever assigned + 1; bounds all loops */
    pthread_mutex_t mu;
} VehicleRegistry;

/* ---------------- helpers ---------------- */

static void trim_crlf(char* s) {
    size_t n = strlen(s);
    while (n > 0 && (s[n-1] == '\n' || s[n-1] == '\r')) {
        s[n-1] = '\0';
        n--;
    }
}

/* Per-connection receive buffer — amortises recv() syscalls. */
typedef struct { char data[4096]; int start; int end; } RecvBuf;

/* Buffered line reader: reads up to 4096 bytes at a time, scans for '\n'.
 * Returns line length (including '\n'), 0 on EOF, -1 on error. */
static int recv_line(int client_fd, char* buf, int cap, RecvBuf* rb) {
    int pos = 0;
    while (pos < cap - 1) {
        if (rb->start >= rb->end) {
            int r = (int)recv(client_fd, rb->data, sizeof(rb->data), 0);
            if (r == 0) { if (pos == 0) return 0; break; }
            if (r < 0)  return -1;
            rb->start = 0;
            rb->end   = r;
        }
        char c = rb->data[rb->start++];
        buf[pos++] = c;
        if (c == '\n') break;
    }
    buf[pos] = '\0';
    return pos;
}

static int send_all(int client_fd, const char* s) {
    int len = (int)strlen(s);
    int sent = 0;
    while (sent < len) {
        int r = (int)send(client_fd, s + sent, len - sent, 0);
        if (r <= 0) return -1;
        sent += r;
    }
    return 0;
}

static int json_extract_int(const char* json, const char* key, int* out) {
    char pat[128];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char* p = strstr(json, pat);
    if (!p) return 0;
    p = strchr(p, ':');
    if (!p) return 0;
    p++;
    while (*p && isspace((unsigned char)*p)) p++;
    char* endp = NULL;
    long v = strtol(p, &endp, 10);
    if (endp == p) return 0;
    *out = (int)v;
    return 1;
}

static int json_extract_double(const char* json, const char* key, double* out) {
    char pat[128];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char* p = strstr(json, pat);
    if (!p) return 0;
    p = strchr(p, ':');
    if (!p) return 0;
    p++;
    while (*p && isspace((unsigned char)*p)) p++;
    char* endp = NULL;
    double v = strtod(p, &endp);
    if (endp == p) return 0;
    *out = v;
    return 1;
}

/* Parse a JSON integer array: "key":[1,2,3]  → allocates *out, sets *count.
 * Caller must free(*out).  Returns 1 on success, 0 on failure. */
static int json_extract_int_array(const char* json, const char* key,
                                   int** out, int* count)
{
    char pat[128];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char* p = strstr(json, pat);
    if (!p) return 0;
    p = strchr(p, ':');
    if (!p) return 0;
    p++;
    while (*p && isspace((unsigned char)*p)) p++;
    if (*p != '[') return 0;
    p++;

    /* Count commas to estimate capacity */
    int cap = 16;
    for (const char* q = p; *q && *q != ']'; q++)
        if (*q == ',') cap++;

    int* arr = (int*)malloc(sizeof(int) * (size_t)cap);
    if (!arr) return 0;
    int n = 0;

    while (*p && *p != ']') {
        while (*p && isspace((unsigned char)*p)) p++;
        if (*p == ']') break;
        char* endp = NULL;
        long v = strtol(p, &endp, 10);
        if (endp == p) { free(arr); return 0; }
        if (n < cap) arr[n++] = (int)v;
        p = endp;
        while (*p && isspace((unsigned char)*p)) p++;
        if (*p == ',') p++;
    }

    *out   = arr;
    *count = n;
    return 1;
}

static char* build_error_response(const char* code, int user_id, int car_id) {
    char* resp = (char*)malloc(160);
    if (!resp) return strdup("{\"error\":\"NO_MEM\"}\n");
    if (user_id >= 0 && car_id >= 0) {
        snprintf(resp, 160, "{\"error\":\"%s\",\"user_id\":%d,\"car_id\":%d}\n", code, user_id, car_id);
    } else {
        snprintf(resp, 160, "{\"error\":\"%s\"}\n", code);
    }
    return resp;
}

/* ---------------- task + queues ---------------- */

typedef enum {
    TASK_REQ        = 1,
    TASK_UPD        = 2,
    TASK_PRED       = 3,
    TASK_REGISTER   = 4,
    TASK_TICK       = 5,
    TASK_POSITIONS  = 6,
    TASK_CONGESTION = 7,
    TASK_TICK_ALL   = 8,
    TASK_REG_ROUTE     = 9,
    TASK_PRECOMPUTE    = 11,   /* startup: compute one section→section path */
    TASK_BATCH_SECTION = 12    /* runtime: process one (src_sec,dst_sec) group */
} TaskType;

typedef struct Task {
    TaskType type;

    Graph* g;
    pthread_rwlock_t* graph_lock;

    int client_fd;

    /* REQ / REGISTER payload */
    int user_id;
    int car_id;
    double timestamp;
    int src;
    int dst;

    /* UPD payload */
    int edge_id;
    double position;
    double speed;

    /* PRED payload */
    int pred_edge_id;

    /* TICK payload */
    double dt;

    /* REG_ROUTE payload */
    int* reg_route_edges;
    int  reg_route_len;

    /* PRECOMPUTE payload */
    int section_src;
    int section_dst;

    /* BATCH_SECTION payload */
    int  batch_size;
    int* batch_srcs;
    int* batch_dsts;
    int* batch_user_ids;
    int* batch_car_ids;

    /* result */
    char* response;     /* malloc'ed string to send back */
    int done;           /* 0/1 */

    pthread_mutex_t mu;
    pthread_cond_t  cv;

    struct Task* next;
} Task;

typedef struct {
    Task* head;
    Task* tail;
    pthread_mutex_t mu;
    pthread_cond_t  cv;
} TaskQueue;

static void queue_init(TaskQueue* q) {
    q->head = q->tail = NULL;
    pthread_mutex_init(&q->mu, NULL);
    pthread_cond_init(&q->cv, NULL);
}

static void queue_push(TaskQueue* q, Task* t) {
    t->next = NULL;
    pthread_mutex_lock(&q->mu);
    if (!q->tail) {
        q->head = q->tail = t;
    } else {
        q->tail->next = t;
        q->tail = t;
    }
    pthread_cond_signal(&q->cv);
    pthread_mutex_unlock(&q->mu);
}

static Task* queue_pop(TaskQueue* q) {
    pthread_mutex_lock(&q->mu);
    while (!q->head) {
        pthread_cond_wait(&q->cv, &q->mu);
    }
    Task* t = q->head;
    q->head = t->next;
    if (!q->head) q->tail = NULL;
    pthread_mutex_unlock(&q->mu);
    t->next = NULL;
    return t;
}

/* Complete a task and wake the waiting client thread */
static void task_complete(Task* t, char* resp) {
    pthread_mutex_lock(&t->mu);
    t->response = resp;
    t->done = 1;
    pthread_cond_signal(&t->cv);
    pthread_mutex_unlock(&t->mu);
}

static Task* task_create(Graph* g, pthread_rwlock_t* lock, int client_fd) {
    Task* t = (Task*)calloc(1, sizeof(Task));
    if (!t) return NULL;
    t->g = g;
    t->graph_lock = lock;
    t->client_fd = client_fd;
    t->response = NULL;
    t->done = 0;
    pthread_mutex_init(&t->mu, NULL);
    pthread_cond_init(&t->cv, NULL);
    return t;
}

static void task_destroy(Task* t) {
    if (!t) return;
    free(t->response);
    free(t->reg_route_edges);
    free(t->batch_srcs);
    free(t->batch_dsts);
    free(t->batch_user_ids);
    free(t->batch_car_ids);
    pthread_mutex_destroy(&t->mu);
    pthread_cond_destroy(&t->cv);
    free(t);
}

/* ---------------- segment worker pool ---------------- */

static void seg_queue_init(SegTaskQueue* q) {
    q->head = q->tail = NULL;
    pthread_mutex_init(&q->mu, NULL);
    pthread_cond_init(&q->cv, NULL);
}

static void seg_queue_push(SegTaskQueue* q, SegTask* t) {
    t->next = NULL;
    pthread_mutex_lock(&q->mu);
    if (!q->tail) q->head = q->tail = t;
    else          { q->tail->next = t; q->tail = t; }
    pthread_cond_signal(&q->cv);
    pthread_mutex_unlock(&q->mu);
}

static SegTask* seg_queue_pop(SegTaskQueue* q) {
    pthread_mutex_lock(&q->mu);
    while (!q->head) pthread_cond_wait(&q->cv, &q->mu);
    SegTask* t = q->head;
    q->head = t->next;
    if (!q->head) q->tail = NULL;
    pthread_mutex_unlock(&q->mu);
    t->next = NULL;
    return t;
}

static void* seg_worker_main(void* arg) {
    SegWorkerArg* wa = (SegWorkerArg*)arg;
    while (1) {
        SegTask* t = seg_queue_pop(wa->q);
        /* No graph_lock needed: current_travel_time is _Atomic;
         * base_length, speed_limit, road_type, lanes are immutable after load. */
        t->rc = find_route_a_star_path(t->g, t->src, t->dst,
                                       &t->out_cost,
                                       t->out_edges, MAX_SEG_EDGES,
                                       &t->out_edge_count,
                                       NULL, 0, NULL,
                                       wa->ctx);
        __atomic_fetch_sub(t->pending, 1, __ATOMIC_RELEASE);
    }
    return NULL;
}

/* ---------------- protocol execution (workers) ---------------- */

static char* build_route_response(Graph* g, int user_id, int car_id, int src, int dst) {
    if (src < 0 || src >= g->num_nodes || dst < 0 || dst >= g->num_nodes) {
        return build_error_response("BAD_NODES", user_id, car_id);
    }

    int max_edges = (g->num_nodes > 0) ? g->num_nodes : 1;
    int* path_edges = (int*)malloc(sizeof(int) * max_edges);
    int* path_nodes = (int*)malloc(sizeof(int) * g->num_nodes);
    if (!path_edges || !path_nodes) {
        free(path_edges);
        free(path_nodes);
        return build_error_response("NO_MEM", user_id, car_id);
    }

    double cost = 0.0;
    int edge_count = 0;
    int node_count = 0;
    int rc = find_route_a_star_path(g, src, dst,
                                    &cost,
                                    path_edges, max_edges, &edge_count,
                                    path_nodes, g->num_nodes, &node_count,
                                    NULL);

    if (rc == 1) {
        free(path_edges);
        free(path_nodes);
        return build_error_response("NO_ROUTE", user_id, car_id);
    }
    if (rc != 0) {
        free(path_edges);
        free(path_nodes);
        return build_error_response("ROUTE_FAIL", user_id, car_id);
    }

    if (edge_count < 0 || edge_count > max_edges) {
        free(path_edges);
        free(path_nodes);
        return build_error_response("ROUTE_FAIL", user_id, car_id);
    }

    if (node_count < 0 || node_count > g->num_nodes) {
        free(path_edges);
        free(path_nodes);
        return build_error_response("ROUTE_FAIL", user_id, car_id);
    }

    size_t buf_sz = 128 + (size_t)edge_count * 16;
    char* resp = (char*)malloc(buf_sz);
    if (!resp) {
        free(path_edges);
        free(path_nodes);
        return build_error_response("NO_MEM", user_id, car_id);
    }

    int n = snprintf(resp, buf_sz, "{\"user_id\":%d,\"car_id\":%d,\"route_edges\":[", user_id, car_id);
    for (int i = 0; i < edge_count && n > 0 && (size_t)n < buf_sz; i++) {
        n += snprintf(resp + n, buf_sz - (size_t)n, "%s%d", (i == 0 ? "" : ","), path_edges[i]);
    }
    if (n > 0 && (size_t)n < buf_sz) {
        snprintf(resp + n, buf_sz - (size_t)n, "],\"eta\":%.3f}\n", cost);
    } else {
        free(path_edges);
        free(path_nodes);
        free(resp);
        return build_error_response("ROUTE_FAIL", user_id, car_id);
    }

    free(path_edges);
    free(path_nodes);
    return resp;
}


static char* build_pred_response(Graph* g, int edge_id) {
    if (edge_id < 0 || edge_id >= g->num_edges) {
        return strdup("ERR BAD_EDGE\n");
    }
    Edge* e = &g->edges[edge_id];
    double pred = (e->observation_count > 0) ? e->ema_travel_time : e->current_travel_time;

    char* resp = (char*)malloc(64);
    if (!resp) return strdup("ERR NO_MEM\n");
    snprintf(resp, 64, "PRED %d %.3f\n", edge_id, pred);
    return resp;
}

/* ---------------- server shared state ---------------- */

typedef struct {
    Graph*           g;
    pthread_rwlock_t graph_lock;

    VehicleRegistry  vehicles;

    TaskQueue routing_q;
    TaskQueue traffic_q;

    pthread_t routing_workers[MAX_ROUTE_WORKERS];
    pthread_t traffic_workers[TRAFFIC_WORKERS];
    int       num_route_workers;

    /* hierarchical section routing */
    SectionPath section_paths[NUM_SECTIONS][NUM_SECTIONS];
    int         section_hubs[NUM_SECTIONS];
    int*        node_sections;   /* node_sections[node_id] = section_id */
    double      lat_min, lat_max, lon_min, lon_max;

    /* precompute barrier */
    int             precompute_remaining;
    pthread_mutex_t precompute_mu;
    pthread_cond_t  precompute_cv;

    /* segment worker pool */
    SegTaskQueue    seg_q;
    pthread_t       seg_workers[SEG_WORKERS];
    SegWorkerArg    seg_wargs[SEG_WORKERS];

    /* per-stripe edge mutexes — replace global graph_lock write lock */
    pthread_mutex_t edge_stripe_mu[N_EDGE_STRIPES];
} ServerState;

/* Per-routing-worker argument: shared state + pre-allocated scratch space. */
typedef struct {
    ServerState*  st;
    RouteContext* ctx;
} RoutingWorkerArg;

/* ---------------- vehicle handlers ---------------- */

static char* handle_register(ServerState* st,
                              int user_id, int car_id,
                              int src, int dst)
{
    if (src < 0 || src >= st->g->num_nodes ||
        dst < 0 || dst >= st->g->num_nodes) {
        return build_error_response("BAD_NODES", user_id, car_id);
    }

    /* Compute route under graph read lock */
    int max_edges = st->g->num_nodes;
    int* path_edges = (int*)malloc(sizeof(int) * max_edges);
    if (!path_edges) return build_error_response("NO_MEM", user_id, car_id);

    double cost = 0.0;
    int edge_count = 0;

    pthread_rwlock_rdlock(&st->graph_lock);
    int rc = find_route_a_star_path(st->g, src, dst,
                                    &cost,
                                    path_edges, max_edges, &edge_count,
                                    NULL, 0, NULL,
                                    NULL);
    pthread_rwlock_unlock(&st->graph_lock);

    if (rc == 1) { free(path_edges); return build_error_response("NO_ROUTE",   user_id, car_id); }
    if (rc != 0) { free(path_edges); return build_error_response("ROUTE_FAIL", user_id, car_id); }

    /* Register car in vehicle registry */
    pthread_mutex_lock(&st->vehicles.mu);

    int slot = -1;
    /* Reuse existing slot for same car_id */
    for (int i = 0; i < st->vehicles.max_active_slot; i++) {
        if (st->vehicles.cars[i].active &&
            st->vehicles.cars[i].car_id == car_id) {
            slot = i;
            break;
        }
    }
    if (slot < 0) {
        /* Find a free slot */
        for (int i = 0; i < MAX_CARS; i++) {
            if (!st->vehicles.cars[i].active) {
                slot = i;
                break;
            }
        }
    }

    if (slot < 0) {
        pthread_mutex_unlock(&st->vehicles.mu);
        free(path_edges);
        return build_error_response("MAX_CARS", user_id, car_id);
    }

    VehicleState* v = &st->vehicles.cars[slot];
    free(v->route_edges);        /* free any previous route */
    v->car_id      = car_id;
    v->user_id     = user_id;
    v->active      = 1;
    v->state       = CAR_DRIVING;
    v->route_edges = path_edges; /* ownership transferred */
    v->route_len   = edge_count;
    v->edge_idx    = 0;
    v->pos         = 0.0;
    v->src_node    = src;
    v->dst_node    = dst;

    if (slot + 1 > st->vehicles.max_active_slot)
        st->vehicles.max_active_slot = slot + 1;

    pthread_mutex_unlock(&st->vehicles.mu);

    char* resp = (char*)malloc(128);
    if (!resp) return build_error_response("NO_MEM", user_id, car_id);
    snprintf(resp, 128,
             "{\"car_id\":%d,\"status\":\"REGISTERED\","
             "\"route_len\":%d,\"eta\":%.3f}\n",
             car_id, edge_count, cost);
    return resp;
}

/* Count how many DRIVING cars are currently on a given edge.
 * Must be called while vehicles.mu is already held. */
static int count_cars_on_edge(VehicleRegistry* reg, int edge_id)
{
    int count = 0;
    for (int i = 0; i < reg->max_active_slot; i++) {
        VehicleState* v = &reg->cars[i];
        if (!v->active || v->state != CAR_DRIVING) continue;
        if (v->edge_idx < v->route_len && v->route_edges[v->edge_idx] == edge_id)
            count++;
    }
    return count;
}

static char* handle_tick(ServerState* st, int car_id, double dt)
{
    if (dt <= 0.0) dt = 1.0;

    pthread_mutex_lock(&st->vehicles.mu);

    VehicleState* v = NULL;
    for (int i = 0; i < st->vehicles.max_active_slot; i++) {
        if (st->vehicles.cars[i].active &&
            st->vehicles.cars[i].car_id == car_id) {
            v = &st->vehicles.cars[i];
            break;
        }
    }

    if (!v) {
        pthread_mutex_unlock(&st->vehicles.mu);
        char* err = (char*)malloc(64);
        if (err) snprintf(err, 64, "{\"car_id\":%d,\"cmd\":\"UNKNOWN_CAR\"}\n", car_id);
        return err ? err : build_error_response("UNKNOWN_CAR", -1, car_id);
    }

    if (v->state == CAR_IDLE) {
        pthread_mutex_unlock(&st->vehicles.mu);
        char* resp = (char*)malloc(64);
        if (resp) snprintf(resp, 64, "{\"car_id\":%d,\"cmd\":\"WAITING\"}\n", car_id);
        return resp;
    }

    if (v->state == CAR_ARRIVED) {
        double lat = st->g->nodes[v->dst_node].lat;
        double lon = st->g->nodes[v->dst_node].lon;
        pthread_mutex_unlock(&st->vehicles.mu);
        char* resp = (char*)malloc(128);
        if (resp)
            snprintf(resp, 128,
                     "{\"car_id\":%d,\"cmd\":\"ARRIVED\","
                     "\"lat\":%.6f,\"lon\":%.6f}\n",
                     car_id, lat, lon);
        return resp;
    }

    /* CAR_DRIVING: advance along route */
    double remaining_dt = dt;
    while (v->state == CAR_DRIVING && remaining_dt > 0.0) {
        if (v->edge_idx >= v->route_len) {
            v->state = CAR_ARRIVED;
            break;
        }

        int eid = v->route_edges[v->edge_idx];
        /* base_length, base_speed_limit, lanes are immutable after graph load — no lock needed */
        double len        = st->g->edges[eid].base_length;
        double base_speed = st->g->edges[eid].base_speed_limit / 3.6; /* km/h → m/s */
        if (len        <= 0.0) len        = 1.0;
        if (base_speed <= 0.0) base_speed = 1.0;

        /* Congestion: speed drops linearly from base_speed (0 cars) to
         * jam_floor * base_speed (at capacity). Both capacity and floor depend
         * on road class (derived from road_type) and lane count. */
        int    cls         = road_class(st->g->edges[eid].road_type);
        int    capacity    = edge_capacity(st->g->edges[eid].base_length,
                                           st->g->edges[eid].lanes, cls);
        int    occupancy   = count_cars_on_edge(&st->vehicles, eid);
        double cong_factor = 1.0 - (double)occupancy / (double)capacity;
        if (cong_factor < jam_floor(cls)) cong_factor = jam_floor(cls);
        double speed = base_speed * cong_factor;

        double advance = (speed * remaining_dt) / len;

        if (v->pos + advance < 1.0) {
            v->pos += advance;
            remaining_dt = 0.0;
        } else {
            /* Time to reach end of this edge */
            double remaining_frac = 1.0 - v->pos;
            double time_to_cross  = (remaining_frac * len) / speed;
            remaining_dt -= time_to_cross;
            v->pos = 0.0;
            v->edge_idx++;
            if (v->edge_idx >= v->route_len) {
                v->state = CAR_ARRIVED;
            }
        }
    }

    /* Compute lat/lon by linear interpolation on current edge */
    double lat, lon;
    if (v->state == CAR_ARRIVED) {
        lat = st->g->nodes[v->dst_node].lat;
        lon = st->g->nodes[v->dst_node].lon;
    } else {
        int eid  = v->route_edges[v->edge_idx];
        int from = st->g->edges[eid].from_node;
        int to   = st->g->edges[eid].to_node;
        double t = v->pos;
        lat = st->g->nodes[from].lat * (1.0 - t) + st->g->nodes[to].lat * t;
        lon = st->g->nodes[from].lon * (1.0 - t) + st->g->nodes[to].lon * t;
    }

    int       out_edge_id = (v->state == CAR_DRIVING) ? v->route_edges[v->edge_idx] : -1;
    double    out_pos     = v->pos;
    CarState  out_state   = v->state;

    pthread_mutex_unlock(&st->vehicles.mu);

    char* resp = (char*)malloc(192);
    if (!resp) return build_error_response("NO_MEM", -1, car_id);

    if (out_state == CAR_ARRIVED) {
        snprintf(resp, 192,
                 "{\"car_id\":%d,\"cmd\":\"ARRIVED\","
                 "\"lat\":%.6f,\"lon\":%.6f}\n",
                 car_id, lat, lon);
    } else {
        snprintf(resp, 192,
                 "{\"car_id\":%d,\"cmd\":\"MOVE\","
                 "\"edge_id\":%d,\"position\":%.4f,"
                 "\"lat\":%.6f,\"lon\":%.6f}\n",
                 car_id, out_edge_id, out_pos, lat, lon);
    }
    return resp;
}

static char* handle_positions(ServerState* st)
{
    pthread_mutex_lock(&st->vehicles.mu);

    size_t buf_sz = 32 + (size_t)MAX_CARS * 96;
    char* resp = (char*)malloc(buf_sz);
    if (!resp) {
        pthread_mutex_unlock(&st->vehicles.mu);
        return strdup("{\"error\":\"NO_MEM\"}\n");
    }

    int n = snprintf(resp, buf_sz, "{\"positions\":[");
    int first = 1;

    for (int i = 0; i < st->vehicles.max_active_slot && n > 0 && (size_t)n < buf_sz; i++) {
        VehicleState* v = &st->vehicles.cars[i];
        if (!v->active) continue;

        double lat, lon;
        const char* state_str;

        if (v->state == CAR_ARRIVED) {
            lat = st->g->nodes[v->dst_node].lat;
            lon = st->g->nodes[v->dst_node].lon;
            state_str = "arrived";
        } else if (v->state == CAR_DRIVING && v->edge_idx < v->route_len) {
            int eid  = v->route_edges[v->edge_idx];
            int from = st->g->edges[eid].from_node;
            int to   = st->g->edges[eid].to_node;
            double t = v->pos;
            lat = st->g->nodes[from].lat * (1.0 - t) + st->g->nodes[to].lat * t;
            lon = st->g->nodes[from].lon * (1.0 - t) + st->g->nodes[to].lon * t;
            state_str = "driving";
        } else {
            lat = st->g->nodes[v->src_node].lat;
            lon = st->g->nodes[v->src_node].lon;
            state_str = "idle";
        }

        n += snprintf(resp + n, buf_sz - (size_t)n,
                      "%s{\"car_id\":%d,\"lat\":%.6f,\"lon\":%.6f,\"state\":\"%s\"}",
                      first ? "" : ",",
                      v->car_id, lat, lon, state_str);
        first = 0;
    }

    pthread_mutex_unlock(&st->vehicles.mu);

    if (n > 0 && (size_t)n < buf_sz) {
        snprintf(resp + n, buf_sz - (size_t)n, "]}\n");
    }
    return resp;
}

static char* handle_congestion(ServerState* st)
{
    pthread_mutex_lock(&st->vehicles.mu);

    int num_edges = st->g->num_edges;
    int* occ = (int*)calloc((size_t)num_edges, sizeof(int));
    if (!occ) {
        pthread_mutex_unlock(&st->vehicles.mu);
        return strdup("{\"congestion\":[]}\n");
    }

    for (int i = 0; i < st->vehicles.max_active_slot; i++) {
        VehicleState* v = &st->vehicles.cars[i];
        if (!v->active || v->state != CAR_DRIVING) continue;
        if (v->edge_idx < v->route_len) {
            int eid = v->route_edges[v->edge_idx];
            if (eid >= 0 && eid < num_edges)
                occ[eid]++;
        }
    }

    size_t buf_sz = 32 + (size_t)MAX_CARS * 160;
    char* resp = (char*)malloc(buf_sz);
    if (!resp) {
        free(occ);
        pthread_mutex_unlock(&st->vehicles.mu);
        return strdup("{\"congestion\":[]}\n");
    }

    int n = snprintf(resp, buf_sz, "{\"congestion\":[");
    int first = 1;

    for (int eid = 0; eid < num_edges && n > 0 && (size_t)n < buf_sz; eid++) {
        if (occ[eid] == 0) continue;
        Edge* e = &st->g->edges[eid];
        int from = e->from_node;
        int to   = e->to_node;
        int cls  = road_class(e->road_type);
        int cap  = edge_capacity(e->base_length, e->lanes, cls);
        if ((double)occ[eid] / (double)cap < 0.3) continue;

        n += snprintf(resp + n, buf_sz - (size_t)n,
                      "%s{\"id\":%d,"
                      "\"from_lat\":%.6f,\"from_lon\":%.6f,"
                      "\"to_lat\":%.6f,\"to_lon\":%.6f,"
                      "\"occ\":%d,\"cap\":%d}",
                      first ? "" : ",",
                      eid,
                      st->g->nodes[from].lat, st->g->nodes[from].lon,
                      st->g->nodes[to].lat,   st->g->nodes[to].lon,
                      occ[eid], cap);
        first = 0;
    }

    free(occ);
    pthread_mutex_unlock(&st->vehicles.mu);

    if (n > 0 && (size_t)n < buf_sz)
        snprintf(resp + n, buf_sz - (size_t)n, "]}\n");

    return resp;
}

/* Register a pre-computed route (bypasses A*).
 * route_edges ownership is transferred to the vehicle slot. */
static char* handle_register_route(ServerState* st,
                                    int car_id, int src, int dst,
                                    int* route_edges, int route_len)
{
    pthread_mutex_lock(&st->vehicles.mu);

    int slot = -1;
    for (int i = 0; i < st->vehicles.max_active_slot; i++) {
        if (st->vehicles.cars[i].active &&
            st->vehicles.cars[i].car_id == car_id) {
            slot = i;
            break;
        }
    }
    if (slot < 0) {
        for (int i = 0; i < MAX_CARS; i++) {
            if (!st->vehicles.cars[i].active) {
                slot = i;
                break;
            }
        }
    }

    if (slot < 0) {
        pthread_mutex_unlock(&st->vehicles.mu);
        free(route_edges);
        return build_error_response("MAX_CARS", -1, car_id);
    }

    VehicleState* v = &st->vehicles.cars[slot];
    free(v->route_edges);
    v->car_id      = car_id;
    v->user_id     = -1;
    v->active      = 1;
    v->state       = CAR_DRIVING;
    v->route_edges = route_edges;  /* ownership transferred */
    v->route_len   = route_len;
    v->edge_idx    = 0;
    v->pos         = 0.0;
    v->src_node    = src;
    v->dst_node    = dst;

    if (slot + 1 > st->vehicles.max_active_slot)
        st->vehicles.max_active_slot = slot + 1;

    pthread_mutex_unlock(&st->vehicles.mu);

    /* Estimate ETA from route edges (sum of current_travel_time) */
    double eta = 0.0;
    for (int i = 0; i < route_len; i++) {
        int eid = route_edges[i];
        if (eid >= 0 && eid < st->g->num_edges)
            eta += st->g->edges[eid].current_travel_time;
    }

    char* resp = (char*)malloc(128);
    if (!resp) return build_error_response("NO_MEM", -1, car_id);
    snprintf(resp, 128,
             "{\"car_id\":%d,\"status\":\"REGISTERED\","
             "\"route_len\":%d,\"eta\":%.3f}\n",
             car_id, route_len, eta);
    return resp;
}

typedef struct {
    ServerState* st;
    int          start;      /* first vehicle slot (inclusive) */
    int          end;        /* last vehicle slot (exclusive) */
    int*         occ;        /* shared occupancy array; use atomic ops for updates */
    int          num_edges;
    double       dt;
} TickArg;

static void* tick_worker_fn(void* arg) {
    TickArg* a       = (TickArg*)arg;
    ServerState* st  = a->st;
    int num_edges    = a->num_edges;
    int* occ         = a->occ;
    double dt        = a->dt;

    for (int i = a->start; i < a->end; i++) {
        VehicleState* v = &st->vehicles.cars[i];
        if (!v->active || v->state != CAR_DRIVING) continue;

        double remaining_dt = dt;
        while (v->state == CAR_DRIVING && remaining_dt > 0.0) {
            if (v->edge_idx >= v->route_len) {
                v->state = CAR_ARRIVED;
                break;
            }

            int eid = v->route_edges[v->edge_idx];
            double len        = st->g->edges[eid].base_length;
            double base_speed = st->g->edges[eid].base_speed_limit / 3.6;
            if (len        <= 0.0) len        = 1.0;
            if (base_speed <= 0.0) base_speed = 1.0;

            int    cls      = road_class(st->g->edges[eid].road_type);
            int    capacity = edge_capacity(st->g->edges[eid].base_length,
                                            st->g->edges[eid].lanes, cls);
            int    occupancy = (eid >= 0 && eid < num_edges)
                               ? __atomic_load_n(&occ[eid], __ATOMIC_RELAXED) : 0;
            double cong = 1.0 - (double)occupancy / (double)capacity;
            if (cong < jam_floor(cls)) cong = jam_floor(cls);
            double speed   = base_speed * cong;
            double advance = (speed * remaining_dt) / len;

            if (v->pos + advance < 1.0) {
                v->pos += advance;
                remaining_dt = 0.0;
            } else {
                double remaining_frac = 1.0 - v->pos;
                double time_to_cross  = (remaining_frac * len) / speed;
                remaining_dt -= time_to_cross;

                if (eid >= 0 && eid < num_edges)
                    __atomic_fetch_sub(&occ[eid], 1, __ATOMIC_RELAXED);

                v->pos = 0.0;
                v->edge_idx++;

                if (v->edge_idx >= v->route_len) {
                    v->state = CAR_ARRIVED;
                } else {
                    int neid = v->route_edges[v->edge_idx];
                    if (neid >= 0 && neid < num_edges)
                        __atomic_fetch_add(&occ[neid], 1, __ATOMIC_RELAXED);
                }
            }
        }
    }
    return NULL;
}

/* Advance ALL active cars by dt seconds in a single call.
 * Pre-computes occupancy array once (O(N)) to avoid O(N²) cost. */
static char* handle_tick_all(ServerState* st, double dt)
{
    if (dt <= 0.0) dt = 1.0;

    pthread_mutex_lock(&st->vehicles.mu);

    int num_edges = st->g->num_edges;

    /* Build occupancy array in one pass */
    int* occ = (int*)calloc((size_t)num_edges, sizeof(int));
    if (!occ) {
        pthread_mutex_unlock(&st->vehicles.mu);
        return strdup("{\"positions\":[],\"arrived\":[]}\n");
    }
    for (int i = 0; i < st->vehicles.max_active_slot; i++) {
        VehicleState* v = &st->vehicles.cars[i];
        if (!v->active || v->state != CAR_DRIVING) continue;
        if (v->edge_idx < v->route_len) {
            int eid = v->route_edges[v->edge_idx];
            if (eid >= 0 && eid < num_edges)
                occ[eid]++;
        }
    }

    /* Tick every active car — N_TICK_WORKERS threads, non-overlapping ranges */
    int max_slot = st->vehicles.max_active_slot;
    pthread_t tick_tids[N_TICK_WORKERS];
    TickArg   tick_args[N_TICK_WORKERS];
    int chunk = (max_slot + N_TICK_WORKERS - 1) / N_TICK_WORKERS;
    for (int w = 0; w < N_TICK_WORKERS; w++) {
        tick_args[w].st        = st;
        tick_args[w].start     = w * chunk;
        tick_args[w].end       = (w + 1) * chunk < max_slot ? (w + 1) * chunk : max_slot;
        tick_args[w].occ       = occ;
        tick_args[w].num_edges = num_edges;
        tick_args[w].dt        = dt;
        pthread_create(&tick_tids[w], NULL, tick_worker_fn, &tick_args[w]);
    }
    for (int w = 0; w < N_TICK_WORKERS; w++)
        pthread_join(tick_tids[w], NULL);

    free(occ);

    /* Build response: positions array + arrived list */
    size_t buf_sz = 64 + (size_t)MAX_CARS * 100;
    char* resp = (char*)malloc(buf_sz);
    if (!resp) {
        pthread_mutex_unlock(&st->vehicles.mu);
        return strdup("{\"positions\":[],\"arrived\":[]}\n");
    }

    int n = snprintf(resp, buf_sz, "{\"positions\":[");
    int first_pos = 1;

    /* Temporary arrived list — at most MAX_CARS entries */
    int* arrived_ids = (int*)malloc(sizeof(int) * MAX_CARS);
    int  arrived_n   = 0;

    for (int i = 0; i < st->vehicles.max_active_slot && n > 0 && (size_t)n < buf_sz; i++) {
        VehicleState* v = &st->vehicles.cars[i];
        if (!v->active) continue;

        double lat, lon;
        const char* state_str;

        if (v->state == CAR_ARRIVED) {
            lat = st->g->nodes[v->dst_node].lat;
            lon = st->g->nodes[v->dst_node].lon;
            state_str = "arrived";
            if (arrived_ids && arrived_n < MAX_CARS)
                arrived_ids[arrived_n++] = v->car_id;
        } else if (v->state == CAR_DRIVING && v->edge_idx < v->route_len) {
            int eid  = v->route_edges[v->edge_idx];
            int from = st->g->edges[eid].from_node;
            int to   = st->g->edges[eid].to_node;
            double t = v->pos;
            lat = st->g->nodes[from].lat * (1.0 - t) + st->g->nodes[to].lat * t;
            lon = st->g->nodes[from].lon * (1.0 - t) + st->g->nodes[to].lon * t;
            state_str = "driving";
        } else {
            lat = st->g->nodes[v->src_node].lat;
            lon = st->g->nodes[v->src_node].lon;
            state_str = "idle";
        }

        n += snprintf(resp + n, buf_sz - (size_t)n,
                      "%s{\"car_id\":%d,\"lat\":%.6f,\"lon\":%.6f,\"state\":\"%s\"}",
                      first_pos ? "" : ",",
                      v->car_id, lat, lon, state_str);
        first_pos = 0;
    }

    pthread_mutex_unlock(&st->vehicles.mu);

    /* Append arrived array */
    if (n > 0 && (size_t)n < buf_sz)
        n += snprintf(resp + n, buf_sz - (size_t)n, "],\"arrived\":[");

    if (arrived_ids) {
        for (int i = 0; i < arrived_n && n > 0 && (size_t)n < buf_sz; i++) {
            n += snprintf(resp + n, buf_sz - (size_t)n,
                          "%s%d", (i == 0 ? "" : ","), arrived_ids[i]);
        }
        free(arrived_ids);
    }

    if (n > 0 && (size_t)n < buf_sz)
        snprintf(resp + n, buf_sz - (size_t)n, "]}\n");

    return resp;
}

/* ---------------- worker threads ----------------</ */

/* Build a route-JSON string from concatenated edge segments.
 * Returns malloc'd string; caller frees. */
static char* build_segment_response(int user_id, int car_id,
                                    int* p1_edges, int p1_len, double p1_cost,
                                    int* p2_edges, int p2_len, double p2_cost,
                                    int* p3_edges, int p3_len, double p3_cost)
{
    int total = p1_len + p2_len + p3_len;
    double eta = p1_cost + p2_cost + p3_cost;
    size_t buf_sz = 128 + (size_t)total * 16;
    char* resp = (char*)malloc(buf_sz);
    if (!resp) return build_error_response("NO_MEM", user_id, car_id);

    int n = snprintf(resp, buf_sz,
                     "{\"user_id\":%d,\"car_id\":%d,\"route_edges\":[", user_id, car_id);
    int first = 1;
    for (int i = 0; i < p1_len; i++) {
        n += snprintf(resp + n, buf_sz - (size_t)n, "%s%d", first ? "" : ",", p1_edges[i]);
        first = 0;
    }
    for (int i = 0; i < p2_len; i++) {
        n += snprintf(resp + n, buf_sz - (size_t)n, "%s%d", first ? "" : ",", p2_edges[i]);
        first = 0;
    }
    for (int i = 0; i < p3_len; i++) {
        n += snprintf(resp + n, buf_sz - (size_t)n, "%s%d", first ? "" : ",", p3_edges[i]);
        first = 0;
    }
    snprintf(resp + n, buf_sz - (size_t)n, "],\"eta\":%.3f}\n", eta);
    return resp;
}

/* Process one (src_sec, dst_sec) group via the segment worker pool.
 * Submits 2N SegTasks (Phase1 + Phase3), waits for completion, then
 * stitches results with the pre-computed Phase2 highway segment.
 * Returns a malloc'd JSON string: "[route_json_0,...,route_json_N-1]" */
static char* handle_batch_section(ServerState* st, Task* t) {
    int N        = t->batch_size;
    int src_sec  = t->section_src;
    int dst_sec  = t->section_dst;
    int hub_src  = st->section_hubs[src_sec];
    int hub_dst  = st->section_hubs[dst_sec];
    SectionPath* hw = &st->section_paths[src_sec][dst_sec];

    SegTask* tasks = (SegTask*)malloc(2 * (size_t)N * sizeof(SegTask));
    if (!tasks) return strdup("[{\"error\":\"NO_MEM\"}]\n");

    /* Shared atomic counter — decremented by each seg worker on finish */
    int pending = 2 * N;

    for (int i = 0; i < N; i++) {
        /* Phase 1: src[i] → hub_src */
        tasks[i].src           = t->batch_srcs[i];
        tasks[i].dst           = hub_src;
        tasks[i].g             = st->g;
        tasks[i].out_edges     = (int*)malloc(MAX_SEG_EDGES * sizeof(int));
        tasks[i].out_edge_count = 0;
        tasks[i].out_cost      = 0.0;
        tasks[i].rc            = -1;
        tasks[i].pending       = &pending;
        seg_queue_push(&st->seg_q, &tasks[i]);

        /* Phase 3: hub_dst → dst[i] */
        tasks[N + i].src           = hub_dst;
        tasks[N + i].dst           = t->batch_dsts[i];
        tasks[N + i].g             = st->g;
        tasks[N + i].out_edges     = (int*)malloc(MAX_SEG_EDGES * sizeof(int));
        tasks[N + i].out_edge_count = 0;
        tasks[N + i].out_cost      = 0.0;
        tasks[N + i].rc            = -1;
        tasks[N + i].pending       = &pending;
        seg_queue_push(&st->seg_q, &tasks[N + i]);
    }

    /* Wait for all 2N tasks — sched_yield keeps this thread cooperative */
    while (__atomic_load_n(&pending, __ATOMIC_ACQUIRE) > 0)
        sched_yield();

    /* Combine per-car results into JSON array string */
    size_t total_sz = 64;
    for (int i = 0; i < N; i++) {
        int edges_est = tasks[i].out_edge_count + (hw->edges ? hw->edge_count : 0)
                        + tasks[N + i].out_edge_count;
        total_sz += 128 + (size_t)edges_est * 16;
    }
    char* out = (char*)malloc(total_sz);
    if (!out) {
        for (int i = 0; i < 2 * N; i++) free(tasks[i].out_edges);
        free(tasks);
        return strdup("[{\"error\":\"NO_MEM\"}]\n");
    }

    int pos = 0;
    pos += snprintf(out + pos, total_sz - (size_t)pos, "[");
    for (int i = 0; i < N; i++) {
        if (i > 0) pos += snprintf(out + pos, total_sz - (size_t)pos, ",");

        if (tasks[i].rc != 0 || tasks[N + i].rc != 0 || !hw->edges) {
            char* err = build_error_response("NO_ROUTE",
                                             t->batch_user_ids[i], t->batch_car_ids[i]);
            int elen = (int)strlen(err);
            if (elen > 0 && err[elen - 1] == '\n') err[elen - 1] = '\0';
            pos += snprintf(out + pos, total_sz - (size_t)pos, "%s", err);
            free(err);
        } else {
            char* route = build_segment_response(
                t->batch_user_ids[i], t->batch_car_ids[i],
                tasks[i].out_edges,     tasks[i].out_edge_count,     tasks[i].out_cost,
                hw->edges,              hw->edge_count,               hw->cost,
                tasks[N+i].out_edges,   tasks[N+i].out_edge_count,   tasks[N+i].out_cost);
            int rlen = (int)strlen(route);
            if (rlen > 0 && route[rlen - 1] == '\n') route[rlen - 1] = '\0';
            pos += snprintf(out + pos, total_sz - (size_t)pos, "%s", route);
            free(route);
        }
    }
    snprintf(out + pos, total_sz - (size_t)pos, "]\n");

    for (int i = 0; i < 2 * N; i++) free(tasks[i].out_edges);
    free(tasks);
    return out;
}

/* -------- section metadata computation -------- */

static void compute_sections(ServerState* st) {
    Graph* g = st->g;
    if (g->num_nodes <= 0) return;

    st->lat_min = st->lat_max = g->nodes[0].lat;
    st->lon_min = st->lon_max = g->nodes[0].lon;
    for (int i = 1; i < g->num_nodes; i++) {
        if (g->nodes[i].lat < st->lat_min) st->lat_min = g->nodes[i].lat;
        if (g->nodes[i].lat > st->lat_max) st->lat_max = g->nodes[i].lat;
        if (g->nodes[i].lon < st->lon_min) st->lon_min = g->nodes[i].lon;
        if (g->nodes[i].lon > st->lon_max) st->lon_max = g->nodes[i].lon;
    }

    double lat_range = st->lat_max - st->lat_min + 1e-9;
    double lon_range = st->lon_max - st->lon_min + 1e-9;

    st->node_sections = (int*)malloc((size_t)g->num_nodes * sizeof(int));
    if (!st->node_sections) return;

    for (int i = 0; i < g->num_nodes; i++) {
        int row = (int)((g->nodes[i].lat - st->lat_min) / lat_range * SECTION_ROWS);
        int col = (int)((g->nodes[i].lon - st->lon_min) / lon_range * SECTION_COLS);
        if (row < 0) row = 0;
        if (row >= SECTION_ROWS) row = SECTION_ROWS - 1;
        if (col < 0) col = 0;
        if (col >= SECTION_COLS) col = SECTION_COLS - 1;
        st->node_sections[i] = row * SECTION_COLS + col;
    }

    /* Find hub node per section: closest to section centroid */
    for (int s = 0; s < NUM_SECTIONS; s++) st->section_hubs[s] = -1;

    double centroid_lat[NUM_SECTIONS], centroid_lon[NUM_SECTIONS];
    int    sec_count[NUM_SECTIONS];
    for (int s = 0; s < NUM_SECTIONS; s++) {
        centroid_lat[s] = centroid_lon[s] = 0.0;
        sec_count[s] = 0;
    }
    for (int i = 0; i < g->num_nodes; i++) {
        int s = st->node_sections[i];
        centroid_lat[s] += g->nodes[i].lat;
        centroid_lon[s] += g->nodes[i].lon;
        sec_count[s]++;
    }
    for (int s = 0; s < NUM_SECTIONS; s++) {
        if (sec_count[s] > 0) {
            centroid_lat[s] /= sec_count[s];
            centroid_lon[s] /= sec_count[s];
        }
    }
    /* For each node, update hub if it's closer to its section centroid */
    double best_dist[NUM_SECTIONS];
    for (int s = 0; s < NUM_SECTIONS; s++) best_dist[s] = 1e18;
    for (int i = 0; i < g->num_nodes; i++) {
        int s = st->node_sections[i];
        double dlat = g->nodes[i].lat - centroid_lat[s];
        double dlon = g->nodes[i].lon - centroid_lon[s];
        double d = dlat * dlat + dlon * dlon;
        if (d < best_dist[s]) {
            best_dist[s] = d;
            st->section_hubs[s] = i;
        }
    }
}

/* Push all (i,j) precompute tasks to routing_q; returns count pushed. */
static int push_precompute_tasks(ServerState* st) {
    int count = 0;
    for (int i = 0; i < NUM_SECTIONS; i++) {
        if (st->section_hubs[i] < 0) continue;
        for (int j = 0; j < NUM_SECTIONS; j++) {
            if (i == j) continue;
            if (st->section_hubs[j] < 0) continue;

            Task* t = task_create(st->g, &st->graph_lock, -1);
            if (!t) continue;
            t->type        = TASK_PRECOMPUTE;
            t->src         = st->section_hubs[i];
            t->dst         = st->section_hubs[j];
            t->section_src = i;
            t->section_dst = j;
            queue_push(&st->routing_q, t);
            count++;
        }
    }
    return count;
}

/* -------- batch request handler (called from client thread) -------- */

/* Parse up to MAX_BATCH objects from "batch_routes":[{...},{...},...].
 * Extracts start_node, destination_node, user_id, car_id into arrays.
 * Returns count parsed, -1 on failure. */
static int parse_batch_routes(const char* line,
                               int* srcs, int* dsts,
                               int* user_ids, int* car_ids)
{
    const char* p = strstr(line, "\"batch_routes\"");
    if (!p) return -1;
    p = strchr(p, '[');
    if (!p) return -1;
    p++;

    int count = 0;
    while (*p && *p != ']' && count < MAX_BATCH) {
        /* Find next '{' */
        while (*p && *p != '{' && *p != ']') p++;
        if (*p != '{') break;
        const char* obj_start = p;

        /* Find matching '}' (no nesting) */
        const char* obj_end = strchr(p, '}');
        if (!obj_end) break;

        /* Extract fields from this object */
        char obj[512];
        int obj_len = (int)(obj_end - obj_start + 1);
        if (obj_len >= (int)sizeof(obj)) { p = obj_end + 1; continue; }
        memcpy(obj, obj_start, (size_t)obj_len);
        obj[obj_len] = '\0';

        int sn = -1, dn = -1, uid = -1, cid = -1;
        json_extract_int(obj, "start_node",       &sn);
        json_extract_int(obj, "destination_node", &dn);
        json_extract_int(obj, "user_id",           &uid);
        json_extract_int(obj, "car_id",            &cid);

        if (sn >= 0 && dn >= 0) {
            srcs[count]     = sn;
            dsts[count]     = dn;
            user_ids[count] = uid;
            car_ids[count]  = cid;
            count++;
        }
        p = obj_end + 1;
    }
    return count;
}

static void handle_batch_route_request(ServerState* st, int client_fd, const char* line) {
    int srcs[MAX_BATCH], dsts[MAX_BATCH], uids[MAX_BATCH], cids[MAX_BATCH];
    int total = parse_batch_routes(line, srcs, dsts, uids, cids);
    if (total <= 0) {
        send_all(client_fd, "{\"error\":\"BAD_BATCH\"}\n");
        return;
    }

    /* Validate node IDs */
    for (int i = 0; i < total; i++) {
        if (srcs[i] < 0 || srcs[i] >= st->g->num_nodes ||
            dsts[i] < 0 || dsts[i] >= st->g->num_nodes) {
            send_all(client_fd, "{\"error\":\"BAD_NODES\"}\n");
            return;
        }
    }

    /* Group items by (src_sec, dst_sec).  We allow up to NUM_SECTIONS*NUM_SECTIONS groups. */
    typedef struct {
        int src_sec, dst_sec;
        int indices[MAX_BATCH]; /* original indices into srcs/dsts */
        int count;
    } Group;

    Group groups[NUM_SECTIONS * NUM_SECTIONS];
    int   num_groups = 0;

    /* Map (src_sec*NUM_SECTIONS+dst_sec) → group index */
    int group_map[NUM_SECTIONS * NUM_SECTIONS];
    for (int k = 0; k < NUM_SECTIONS * NUM_SECTIONS; k++) group_map[k] = -1;

    for (int i = 0; i < total; i++) {
        int ss = (st->node_sections && srcs[i] < st->g->num_nodes)
                    ? st->node_sections[srcs[i]] : 0;
        int ds = (st->node_sections && dsts[i] < st->g->num_nodes)
                    ? st->node_sections[dsts[i]] : 0;
        int key = ss * NUM_SECTIONS + ds;
        if (group_map[key] < 0) {
            group_map[key] = num_groups;
            groups[num_groups].src_sec = ss;
            groups[num_groups].dst_sec = ds;
            groups[num_groups].count   = 0;
            num_groups++;
        }
        int g_idx = group_map[key];
        groups[g_idx].indices[groups[g_idx].count++] = i;
    }

    /* Create one task per group and push to routing_q */
    Task** tasks = (Task**)malloc((size_t)num_groups * sizeof(Task*));
    if (!tasks) { send_all(client_fd, "{\"error\":\"NO_MEM\"}\n"); return; }

    for (int g = 0; g < num_groups; g++) {
        Group* gr = &groups[g];
        int N = gr->count;

        int ss = gr->src_sec, ds = gr->dst_sec;

        /* Same section or no highway → dispatch as individual TASK_REQ tasks */
        if (ss == ds || !st->section_paths[ss][ds].edges) {
            /* We'll handle them as individual TASK_REQs collected into a mini-array.
             * For simplicity, create N separate tasks and store them in the group slot.
             * We reuse a single "batch task" that is actually a TASK_BATCH_SECTION
             * with ss==ds handled in handle_batch_section by falling through to A*. */
        }

        Task* t = task_create(st->g, &st->graph_lock, -1);
        if (!t) { tasks[g] = NULL; continue; }

        t->type        = TASK_BATCH_SECTION;
        t->section_src = ss;
        t->section_dst = ds;
        t->batch_size  = N;
        t->batch_srcs     = (int*)malloc((size_t)N * sizeof(int));
        t->batch_dsts     = (int*)malloc((size_t)N * sizeof(int));
        t->batch_user_ids = (int*)malloc((size_t)N * sizeof(int));
        t->batch_car_ids  = (int*)malloc((size_t)N * sizeof(int));

        if (!t->batch_srcs || !t->batch_dsts || !t->batch_user_ids || !t->batch_car_ids) {
            task_destroy(t);
            tasks[g] = NULL;
            continue;
        }
        for (int k = 0; k < N; k++) {
            int idx = gr->indices[k];
            t->batch_srcs[k]     = srcs[idx];
            t->batch_dsts[k]     = dsts[idx];
            t->batch_user_ids[k] = uids[idx];
            t->batch_car_ids[k]  = cids[idx];
        }
        tasks[g] = t;
        queue_push(&st->routing_q, t);
    }

    /* Wait for all group tasks */
    /* Collect per-car results ordered by original index */
    /* group task response = JSON array "[r0,r1,...]" for that group's cars */
    char* car_results[MAX_BATCH];
    for (int i = 0; i < total; i++) car_results[i] = NULL;

    for (int g = 0; g < num_groups; g++) {
        if (!tasks[g]) continue;
        Task* t = tasks[g];
        pthread_mutex_lock(&t->mu);
        while (!t->done) pthread_cond_wait(&t->cv, &t->mu);
        char* resp = t->response;
        t->response = NULL; /* take ownership */
        pthread_mutex_unlock(&t->mu);

        /* resp is "[r0,r1,...]" — parse per-car results back out */
        if (resp) {
            /* Distribute results to original indices */
            Group* gr = &groups[g];
            const char* rp = resp;
            /* skip '[' */
            while (*rp && *rp != '[') rp++;
            if (*rp == '[') rp++;
            int k = 0;
            while (*rp && *rp != ']' && k < gr->count) {
                while (*rp && (*rp == ',' || *rp == ' ')) rp++;
                if (*rp == ']' || *rp == '\0') break;
                /* Find end of this JSON object */
                const char* obj_start = rp;
                int depth = 0;
                while (*rp) {
                    if (*rp == '{') depth++;
                    else if (*rp == '}') { depth--; if (depth == 0) { rp++; break; } }
                    rp++;
                }
                int idx = gr->indices[k];
                int len = (int)(rp - obj_start);
                car_results[idx] = (char*)malloc((size_t)len + 2);
                if (car_results[idx]) {
                    memcpy(car_results[idx], obj_start, (size_t)len);
                    car_results[idx][len]     = '\0';
                }
                k++;
            }
            free(resp);
        }
        task_destroy(t);
    }
    free(tasks);

    /* Build final {"results":[...]} */
    size_t out_sz = 32;
    for (int i = 0; i < total; i++) out_sz += car_results[i] ? strlen(car_results[i]) + 4 : 24;
    char* out = (char*)malloc(out_sz);
    if (!out) {
        for (int i = 0; i < total; i++) free(car_results[i]);
        send_all(client_fd, "{\"error\":\"NO_MEM\"}\n");
        return;
    }

    int pos = snprintf(out, out_sz, "{\"results\":[");
    for (int i = 0; i < total; i++) {
        if (i > 0) pos += snprintf(out + pos, out_sz - (size_t)pos, ",");
        if (car_results[i]) {
            pos += snprintf(out + pos, out_sz - (size_t)pos, "%s", car_results[i]);
            free(car_results[i]);
        } else {
            pos += snprintf(out + pos, out_sz - (size_t)pos, "{\"error\":\"FAIL\"}");
        }
    }
    snprintf(out + pos, out_sz - (size_t)pos, "]}\n");
    send_all(client_fd, out);
    free(out);
}

static void* routing_worker_main(void* arg) {
    RoutingWorkerArg* warg = (RoutingWorkerArg*)arg;
    ServerState*  st  = warg->st;
    RouteContext* ctx = warg->ctx; /* pre-allocated scratch; may be NULL if alloc failed */

    while (1) {
        Task* t = queue_pop(&st->routing_q);
        char* resp = NULL;

        if (t->type == TASK_REQ) {
            /* current_travel_time is _Atomic — no lock needed for A*. */
            if (t->src < 0 || t->src >= st->g->num_nodes ||
                t->dst < 0 || t->dst >= st->g->num_nodes) {
                resp = build_error_response("BAD_NODES", t->user_id, t->car_id);
            } else if (ctx) {
                /* Fast path: use pre-allocated per-worker scratch (no malloc). */
                double cost = 0.0;
                int edge_count = 0;
                int rc = find_route_a_star_path(st->g, t->src, t->dst,
                                                &cost,
                                                ctx->path_edges, ctx->capacity, &edge_count,
                                                NULL, 0, NULL,
                                                ctx);
                if (rc == 1) {
                    resp = build_error_response("NO_ROUTE", t->user_id, t->car_id);
                } else if (rc != 0 || edge_count < 0 || edge_count > ctx->capacity) {
                    resp = build_error_response("ROUTE_FAIL", t->user_id, t->car_id);
                } else {
                    size_t buf_sz = 128 + (size_t)edge_count * 16;
                    resp = (char*)malloc(buf_sz);
                    if (!resp) {
                        resp = build_error_response("NO_MEM", t->user_id, t->car_id);
                    } else {
                        int n = snprintf(resp, buf_sz,
                                         "{\"user_id\":%d,\"car_id\":%d,\"route_edges\":[",
                                         t->user_id, t->car_id);
                        for (int i = 0; i < edge_count && n > 0 && (size_t)n < buf_sz; i++)
                            n += snprintf(resp + n, buf_sz - (size_t)n,
                                          "%s%d", (i == 0 ? "" : ","), ctx->path_edges[i]);
                        if (n > 0 && (size_t)n < buf_sz)
                            snprintf(resp + n, buf_sz - (size_t)n, "],\"eta\":%.3f}\n", cost);
                    }
                }
            } else {
                /* Fallback: ctx alloc failed at startup — use per-call malloc. */
                int max_edges = st->g->num_nodes > 0 ? st->g->num_nodes : 1;
                int* path_edges = (int*)malloc(sizeof(int) * max_edges);
                if (!path_edges) {
                    resp = build_error_response("NO_MEM", t->user_id, t->car_id);
                } else {
                    double cost = 0.0;
                    int edge_count = 0;
                    int rc = find_route_a_star_path(st->g, t->src, t->dst,
                                                    &cost,
                                                    path_edges, max_edges, &edge_count,
                                                    NULL, 0, NULL,
                                                    NULL);
                    if (rc == 1) {
                        resp = build_error_response("NO_ROUTE", t->user_id, t->car_id);
                    } else if (rc != 0 || edge_count < 0 || edge_count > max_edges) {
                        resp = build_error_response("ROUTE_FAIL", t->user_id, t->car_id);
                    } else {
                        size_t buf_sz = 128 + (size_t)edge_count * 16;
                        resp = (char*)malloc(buf_sz);
                        if (!resp) {
                            resp = build_error_response("NO_MEM", t->user_id, t->car_id);
                        } else {
                            int n = snprintf(resp, buf_sz,
                                             "{\"user_id\":%d,\"car_id\":%d,\"route_edges\":[",
                                             t->user_id, t->car_id);
                            for (int i = 0; i < edge_count && n > 0 && (size_t)n < buf_sz; i++)
                                n += snprintf(resp + n, buf_sz - (size_t)n,
                                              "%s%d", (i == 0 ? "" : ","), path_edges[i]);
                            if (n > 0 && (size_t)n < buf_sz)
                                snprintf(resp + n, buf_sz - (size_t)n, "],\"eta\":%.3f}\n", cost);
                        }
                    }
                    free(path_edges);
                }
            }

        } else if (t->type == TASK_PRED) {
            pthread_rwlock_rdlock(&st->graph_lock);
            resp = build_pred_response(st->g, t->pred_edge_id);
            pthread_rwlock_unlock(&st->graph_lock);

        } else if (t->type == TASK_REGISTER) {
            resp = handle_register(st, t->user_id, t->car_id, t->src, t->dst);

        } else if (t->type == TASK_TICK) {
            resp = handle_tick(st, t->car_id, t->dt);

        } else if (t->type == TASK_POSITIONS) {
            resp = handle_positions(st);

        } else if (t->type == TASK_CONGESTION) {
            resp = handle_congestion(st);

        } else if (t->type == TASK_TICK_ALL) {
            resp = handle_tick_all(st, t->dt);

        } else if (t->type == TASK_REG_ROUTE) {
            /* Ownership of reg_route_edges transfers to handle_register_route */
            resp = handle_register_route(st, t->car_id, t->src, t->dst,
                                         t->reg_route_edges, t->reg_route_len);
            t->reg_route_edges = NULL; /* avoid double-free in task_destroy */

        } else if (t->type == TASK_PRECOMPUTE) {
            pthread_rwlock_rdlock(&st->graph_lock);
            {
                int si = t->section_src, sj = t->section_dst;
                int* edges = (int*)malloc(MAX_SEG_EDGES * sizeof(int));
                double cost = 0.0;
                int edge_count = 0;
                int rc = 0;
                if (edges) {
                    rc = find_route_a_star_path(st->g, t->src, t->dst,
                                                &cost, edges, MAX_SEG_EDGES,
                                                &edge_count, NULL, 0, NULL,
                                                NULL);
                }
                if (edges && rc == 0) {
                    st->section_paths[si][sj].cost       = cost;
                    st->section_paths[si][sj].edges      = edges;
                    st->section_paths[si][sj].edge_count = edge_count;
                } else {
                    free(edges);
                    st->section_paths[si][sj].edges = NULL;
                }
            }
            pthread_rwlock_unlock(&st->graph_lock);
            pthread_mutex_lock(&st->precompute_mu);
            st->precompute_remaining--;
            if (st->precompute_remaining == 0)
                pthread_cond_broadcast(&st->precompute_cv);
            pthread_mutex_unlock(&st->precompute_mu);
            /* No client waiting — free task directly */
            task_destroy(t);
            continue;

        } else if (t->type == TASK_BATCH_SECTION) {
            /* handle_batch_section spawns its own threads for phases 1+3.
             * For same-section pairs (or missing highway), fall back to
             * individual A* calls per car. */
            int ss = t->section_src, ds = t->section_dst;
            if (ss == ds || !st->section_paths[ss][ds].edges) {
                /* Same section or no precomputed path: direct A* per car */
                int N = t->batch_size;
                size_t total_sz = 32;
                for (int i = 0; i < N; i++) total_sz += 256 + (size_t)st->g->num_nodes * 16;
                char* out = (char*)malloc(total_sz);
                int pos = 0;
                if (out) pos = snprintf(out, total_sz, "[");
                for (int i = 0; i < N; i++) {
                    if (i > 0 && out) pos += snprintf(out + pos, total_sz - (size_t)pos, ",");
                    pthread_rwlock_rdlock(&st->graph_lock);
                    char* r = build_route_response(st->g,
                                                   t->batch_user_ids[i], t->batch_car_ids[i],
                                                   t->batch_srcs[i], t->batch_dsts[i]);
                    pthread_rwlock_unlock(&st->graph_lock);
                    if (r && out) {
                        int rlen = (int)strlen(r);
                        if (rlen > 0 && r[rlen - 1] == '\n') r[rlen - 1] = '\0';
                        pos += snprintf(out + pos, total_sz - (size_t)pos, "%s", r);
                    }
                    free(r);
                }
                if (out) snprintf(out + pos, total_sz - (size_t)pos, "]\n");
                resp = out;
            } else {
                resp = handle_batch_section(st, t);
            }

        } else {
            resp = build_error_response("INTERNAL", t->user_id, t->car_id);
        }

        task_complete(t, resp);
    }
    return NULL;
}

static void* traffic_worker_main(void* arg) {
    ServerState* st = (ServerState*)arg;

    while (1) {
        Task* t = queue_pop(&st->traffic_q);

        /* Validate before acquiring the write lock (num_edges is immutable). */
        if (t->edge_id < 0 || t->edge_id >= st->g->num_edges) {
            task_complete(t, build_error_response("BAD_EDGE", t->user_id, t->car_id));
            continue;
        }
        if (t->speed <= 0.0) {
            task_complete(t, build_error_response("BAD_SPEED", t->user_id, t->car_id));
            continue;
        }

        double speed = t->speed < 1e-6 ? 1e-6 : t->speed;

        /* Per-stripe mutex: only edges in the same stripe contend.
         * current_travel_time is _Atomic so routing workers read it lock-free. */
        int stripe = t->edge_id % N_EDGE_STRIPES;
        pthread_mutex_lock(&st->edge_stripe_mu[stripe]);
        Edge* e = &st->g->edges[t->edge_id];
        double alpha   = (e->observation_count == 0) ? 1.0 : 0.2;
        double measured = e->base_length / speed;
        e->ema_travel_time     = alpha * measured + (1.0 - alpha) * e->ema_travel_time;
        e->current_travel_time = e->ema_travel_time;
        e->observation_count++;
        pthread_mutex_unlock(&st->edge_stripe_mu[stripe]);

        /* Build response after releasing the lock. */
        char* resp = (char*)malloc(96);
        if (resp)
            snprintf(resp, 96, "{\"status\":\"ACK\",\"user_id\":%d,\"car_id\":%d}\n",
                     t->user_id, t->car_id);
        else
            resp = build_error_response("NO_MEM", t->user_id, t->car_id);

        task_complete(t, resp);
    }
    return NULL;
}

/* ---------------- per-client network thread ---------------- */

typedef struct {
    ServerState* st;
    int client_fd;
    RecvBuf rb;
} ClientCtx;

static void* client_thread_main(void* arg) {
    ClientCtx* ctx = (ClientCtx*)arg;
    ServerState* st = ctx->st;
    int client_fd = ctx->client_fd;
    ctx->rb.start = ctx->rb.end = 0;

    fprintf(stderr, "Client connected (fd=%d).\n", client_fd);

    char line[16384]; /* must fit max batch_routes request: 64 pairs ~5 KB */
    while (1) {
        int r = recv_line(client_fd, line, (int)sizeof(line), &ctx->rb);
        if (r == 0) break;
        if (r < 0) {
            fprintf(stderr, "recv error (fd=%d): %s\n", client_fd, strerror(errno));
            break;
        }

        trim_crlf(line);
        if (line[0] == '\0') {
            send_all(client_fd, "{\"error\":\"EMPTY\"}\n");
            continue;
        }

        Task* t = task_create(st->g, &st->graph_lock, client_fd);
        if (!t) {
            send_all(client_fd, "{\"error\":\"NO_MEM\"}\n");
            continue;
        }

        int src, dst;
        int edge_id;
        int user_id = -1;
        int car_id  = -1;
        double speed;
        double position;
        double timestamp;
        int flag;
        double dt_val;

        /* --- BATCH_ROUTES: {"batch_routes":[...]} --- */
        if (strstr(line, "\"batch_routes\"")) {
            task_destroy(t);
            handle_batch_route_request(st, client_fd, line);
            continue;

        /* --- REGISTER: {"register_car":1,"user_id":...,"car_id":...,"start_node":...,"destination_node":...} --- */
        } else if (json_extract_int(line, "register_car", &flag) && flag &&
            json_extract_int(line, "user_id", &user_id) &&
            json_extract_int(line, "car_id",  &car_id)  &&
            json_extract_int(line, "start_node", &src)  &&
            json_extract_int(line, "destination_node", &dst)) {
            t->type    = TASK_REGISTER;
            t->user_id = user_id;
            t->car_id  = car_id;
            t->src     = src;
            t->dst     = dst;
            queue_push(&st->routing_q, t);

        /* --- TICK: {"tick_car":1,"car_id":...,"dt":...} --- */
        } else if (json_extract_int(line, "tick_car", &flag) && flag &&
                   json_extract_int(line, "car_id", &car_id) &&
                   json_extract_double(line, "dt", &dt_val)) {
            t->type   = TASK_TICK;
            t->car_id = car_id;
            t->dt     = dt_val;
            queue_push(&st->routing_q, t);

        /* --- TICK_ALL: "TICK_ALL <dt>" plain text --- */
        } else if (strncmp(line, "TICK_ALL", 8) == 0 && isspace((unsigned char)line[8])) {
            dt_val = 1.0;
            sscanf(line + 8, " %lf", &dt_val);
            t->type = TASK_TICK_ALL;
            t->dt   = dt_val;
            queue_push(&st->routing_q, t);

        /* --- REGISTER_ROUTE: {"register_route":1,"car_id":...,"start_node":...,"dest_node":...,"route_edges":[...]} --- */
        } else if (json_extract_int(line, "register_route", &flag) && flag &&
                   json_extract_int(line, "car_id",   &car_id) &&
                   json_extract_int(line, "start_node", &src)  &&
                   json_extract_int(line, "dest_node",  &dst)) {
            int*  re  = NULL;
            int   rn  = 0;
            json_extract_int_array(line, "route_edges", &re, &rn);
            t->type            = TASK_REG_ROUTE;
            t->car_id          = car_id;
            t->src             = src;
            t->dst             = dst;
            t->reg_route_edges = re;
            t->reg_route_len   = rn;
            queue_push(&st->routing_q, t);

        /* --- POSITIONS (plain text) --- */
        } else if (strcmp(line, "POSITIONS") == 0) {
            t->type = TASK_POSITIONS;
            queue_push(&st->routing_q, t);

        /* --- CONGESTION (plain text) --- */
        } else if (strcmp(line, "CONGESTION") == 0) {
            t->type = TASK_CONGESTION;
            queue_push(&st->routing_q, t);

        /* --- existing: JSON routing request --- */
        } else if (json_extract_int(line, "start_node", &src) &&
                   json_extract_int(line, "destination_node", &dst) &&
                   json_extract_int(line, "user_id", &user_id) &&
                   json_extract_int(line, "car_id", &car_id) &&
                   json_extract_double(line, "timestamp", &timestamp)) {
            t->type = TASK_REQ;
            t->user_id = user_id;
            t->car_id = car_id;
            t->timestamp = timestamp;
            t->src = src;
            t->dst = dst;
            queue_push(&st->routing_q, t);

        /* --- existing: JSON traffic update --- */
        } else if (json_extract_int(line, "edge_id", &edge_id) &&
                   json_extract_double(line, "speed", &speed) &&
                   json_extract_double(line, "position_on_edge", &position) &&
                   json_extract_int(line, "user_id", &user_id) &&
                   json_extract_int(line, "car_id", &car_id) &&
                   json_extract_double(line, "timestamp", &timestamp)) {
            t->type = TASK_UPD;
            t->user_id = user_id;
            t->car_id = car_id;
            t->timestamp = timestamp;
            t->edge_id = edge_id;
            t->position = position;
            t->speed = speed;
            queue_push(&st->traffic_q, t);

        /* --- legacy plain-text --- */
        } else if (sscanf(line, "REQ %d %d", &src, &dst) == 2) {
            t->type = TASK_REQ;
            t->user_id = -1;
            t->car_id = -1;
            t->src = src;
            t->dst = dst;
            queue_push(&st->routing_q, t);

        } else if (sscanf(line, "UPD %d %lf %lf", &edge_id, &speed, &position) >= 2) {
            t->type = TASK_UPD;
            t->user_id = -1;
            t->car_id = -1;
            t->edge_id = edge_id;
            t->speed = speed;
            queue_push(&st->traffic_q, t);

        } else if (sscanf(line, "PRED %d", &edge_id) == 1) {
            t->type = TASK_PRED;
            t->pred_edge_id = edge_id;
            queue_push(&st->routing_q, t);

        } else {
            task_destroy(t);
            send_all(client_fd, "{\"error\":\"UNKNOWN_CMD\"}\n");
            continue;
        }

        /* Wait for worker to finish this task (preserves per-connection order) */
        pthread_mutex_lock(&t->mu);
        while (!t->done) {
            pthread_cond_wait(&t->cv, &t->mu);
        }
        char* resp = t->response;
        pthread_mutex_unlock(&t->mu);

        if (resp) {
            send_all(client_fd, resp);
        } else {
            send_all(client_fd, "{\"error\":\"INTERNAL\"}\n");
        }

        task_destroy(t);
    }

    fprintf(stderr, "Client disconnected (fd=%d).\n", client_fd);
    close(client_fd);
    free(ctx);
    return NULL;
}

/* ---------------- server_run ---------------- */

int server_run(Graph* g, int port, int route_workers) {
    if (route_workers < 1) route_workers = 1;
    if (route_workers > MAX_ROUTE_WORKERS) route_workers = MAX_ROUTE_WORKERS;

    static ServerState st;
    memset(&st, 0, sizeof(st));
    st.g = g;
    st.num_route_workers = route_workers;

    queue_init(&st.routing_q);
    queue_init(&st.traffic_q);

    /* Init vehicle registry */
    memset(&st.vehicles, 0, sizeof(st.vehicles));
    if (pthread_mutex_init(&st.vehicles.mu, NULL) != 0) {
        fprintf(stderr, "pthread_mutex_init vehicles failed\n");
        return 8;
    }

    if (pthread_rwlock_init(&st.graph_lock, NULL) != 0) {
        fprintf(stderr, "pthread_rwlock_init failed\n");
        return 5;
    }

    for (int i = 0; i < N_EDGE_STRIPES; i++) {
        if (pthread_mutex_init(&st.edge_stripe_mu[i], NULL) != 0) {
            fprintf(stderr, "pthread_mutex_init edge stripe %d failed\n", i);
            return 5;
        }
    }

    if (pthread_mutex_init(&st.precompute_mu, NULL) != 0 ||
        pthread_cond_init(&st.precompute_cv, NULL)  != 0) {
        fprintf(stderr, "pthread_mutex/cond_init precompute failed\n");
        return 9;
    }

    /* Start worker pools — each routing worker gets its own RouteContext. */
    static RoutingWorkerArg route_wargs[MAX_ROUTE_WORKERS];
    for (int i = 0; i < route_workers; i++) {
        route_wargs[i].st  = &st;
        route_wargs[i].ctx = route_context_create(g->num_nodes);
        if (!route_wargs[i].ctx)
            fprintf(stderr, "Warning: routing worker %d: route_context_create failed; "
                            "falling back to per-request malloc\n", i);
        if (pthread_create(&st.routing_workers[i], NULL, routing_worker_main,
                           &route_wargs[i]) != 0) {
            fprintf(stderr, "pthread_create routing worker %d failed\n", i);
            return 6;
        }
        pthread_detach(st.routing_workers[i]);
    }
    for (int i = 0; i < TRAFFIC_WORKERS; i++) {
        if (pthread_create(&st.traffic_workers[i], NULL, traffic_worker_main, &st) != 0) {
            fprintf(stderr, "pthread_create traffic worker failed\n");
            return 7;
        }
        pthread_detach(st.traffic_workers[i]);
    }

    /* Start segment worker pool */
    seg_queue_init(&st.seg_q);
    for (int i = 0; i < SEG_WORKERS; i++) {
        st.seg_wargs[i].q   = &st.seg_q;
        st.seg_wargs[i].ctx = route_context_create(g->num_nodes);
        if (!st.seg_wargs[i].ctx)
            fprintf(stderr, "Warning: seg worker %d: route_context_create failed\n", i);
        if (pthread_create(&st.seg_workers[i], NULL, seg_worker_main,
                           &st.seg_wargs[i]) != 0) {
            fprintf(stderr, "pthread_create seg worker %d failed\n", i);
            return 7;
        }
        pthread_detach(st.seg_workers[i]);
    }

    /* Compute geographic sections and hub nodes */
    compute_sections(&st);

    /* Count how many precompute tasks will be pushed, set counter first,
     * then push (so workers can decrement safely from the start). */
    {
        int n_precompute = 0;
        for (int i = 0; i < NUM_SECTIONS; i++) {
            if (st.section_hubs[i] < 0) continue;
            for (int j = 0; j < NUM_SECTIONS; j++) {
                if (i == j || st.section_hubs[j] < 0) continue;
                n_precompute++;
            }
        }
        if (n_precompute > 0) {
            st.precompute_remaining = n_precompute;
            push_precompute_tasks(&st);
            pthread_mutex_lock(&st.precompute_mu);
            while (st.precompute_remaining > 0)
                pthread_cond_wait(&st.precompute_cv, &st.precompute_mu);
            pthread_mutex_unlock(&st.precompute_mu);
            fprintf(stderr, "Precomputed %d section paths.\n", n_precompute);
        }
    }

    int listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd < 0) {
        perror("socket");
        return 2;
    }

    int opt = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons((unsigned short)port);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(listen_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(listen_fd);
        return 3;
    }

    if (listen(listen_fd, 1024) < 0) {
        perror("listen");
        close(listen_fd);
        return 4;
    }

    fprintf(stderr, "Server listening on port %d...\n", port);

    while (1) {
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        int client_fd = accept(listen_fd, (struct sockaddr*)&client_addr, &client_len);
        if (client_fd < 0) {
            perror("accept");
            continue;
        }

        ClientCtx* ctx = (ClientCtx*)malloc(sizeof(ClientCtx));
        if (!ctx) {
            fprintf(stderr, "malloc failed\n");
            close(client_fd);
            continue;
        }
        ctx->st = &st;
        ctx->client_fd = client_fd;

        pthread_t tid;
        if (pthread_create(&tid, NULL, client_thread_main, ctx) != 0) {
            fprintf(stderr, "pthread_create client thread failed\n");
            close(client_fd);
            free(ctx);
            continue;
        }
        pthread_detach(tid);
    }

    close(listen_fd);

    /* Cleanup vehicle registry */
    pthread_mutex_destroy(&st.vehicles.mu);
    for (int i = 0; i < MAX_CARS; i++) {
        free(st.vehicles.cars[i].route_edges);
    }

    pthread_rwlock_destroy(&st.graph_lock);
    return 0;
}
