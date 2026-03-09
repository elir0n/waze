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

/* Congestion model:
 * Road class (0–5) is derived from road_type and controls two parameters:
 *   cars_per_lane  = 3 + class*2   (density before jamming: 3 on residential → 13 on motorway)
 *   min_speed_frac = 0.05 + class*0.05  (speed floor: 5% on residential → 30% on motorway)
 * Capacity = lanes * cars_per_lane.
 * Speed drops linearly from base_speed (0 cars) to min_speed_frac*base_speed (at capacity). */

static int road_class(const char* road_type) {
    if (!road_type) return 0;
    if (strncmp(road_type, "motorway",  8) == 0) return 5;
    if (strncmp(road_type, "trunk",     5) == 0) return 4;
    if (strncmp(road_type, "primary",   7) == 0) return 3;
    if (strncmp(road_type, "secondary", 9) == 0) return 2;
    if (strncmp(road_type, "tertiary",  8) == 0) return 1;
    return 0;  /* residential, living_street, unclassified, road, … */
}

/* Returns cars-per-lane capacity for a given road class. */
static int cars_per_lane(int cls) { return 3 + cls * 2; }

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

/* Reads one line ending with '\n' into buf (null-terminated).
   Returns length, 0 if connection closed, -1 on error. */
static int recv_line(int client_fd, char* buf, int cap) {
    int pos = 0;
    while (pos < cap - 1) {
        char c;
        int r = (int)recv(client_fd, &c, 1, 0);
        if (r == 0) { /* peer closed */
            if (pos == 0) return 0;
            break;
        }
        if (r < 0) return -1;

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
    TASK_REG_ROUTE  = 9
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
    pthread_mutex_destroy(&t->mu);
    pthread_cond_destroy(&t->cv);
    free(t);
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
                                    path_nodes, g->num_nodes, &node_count);

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

static char* apply_update(Graph* g, int user_id, int car_id, int edge_id, double speed) {
    if (edge_id < 0 || edge_id >= g->num_edges) {
        return build_error_response("BAD_EDGE", user_id, car_id);
    }
    if (speed <= 0.0) {
        return build_error_response("BAD_SPEED", user_id, car_id);
    }

    const double min_speed = 1e-6;
    if (speed < min_speed) speed = min_speed;

    Edge* e = &g->edges[edge_id];
    const double alpha = (e->observation_count == 0) ? 1.0 : 0.2;
    double measured = e->base_length / speed;

    e->ema_travel_time = alpha * measured + (1.0 - alpha) * e->ema_travel_time;
    e->current_travel_time = e->ema_travel_time;
    e->observation_count++;

    char* ack = (char*)malloc(96);
    if (!ack) return build_error_response("NO_MEM", user_id, car_id);
    snprintf(ack, 96, "{\"status\":\"ACK\",\"user_id\":%d,\"car_id\":%d}\n", user_id, car_id);
    return ack;
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

    pthread_t routing_workers[ROUTE_WORKERS];
    pthread_t traffic_workers[TRAFFIC_WORKERS];
} ServerState;

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
                                    NULL, 0, NULL);
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
        int    capacity    = st->g->edges[eid].lanes * cars_per_lane(cls);
        if (capacity <= 0) capacity = cars_per_lane(cls);
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
        if (occ[eid] < 2) continue;  /* single car is not congestion */
        Edge* e = &st->g->edges[eid];
        int from = e->from_node;
        int to   = e->to_node;
        int cls  = road_class(e->road_type);
        int cap  = e->lanes * cars_per_lane(cls);
        if (cap <= 0) cap = cars_per_lane(cls);

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

    /* Tick every active car */
    for (int i = 0; i < st->vehicles.max_active_slot; i++) {
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
            int    capacity = st->g->edges[eid].lanes * cars_per_lane(cls);
            if (capacity <= 0) capacity = cars_per_lane(cls);
            int    occupancy = (eid >= 0 && eid < num_edges) ? occ[eid] : 0;
            double cong = 1.0 - (double)occupancy / (double)capacity;
            if (cong < jam_floor(cls)) cong = jam_floor(cls);
            double speed = base_speed * cong;

            double advance = (speed * remaining_dt) / len;

            if (v->pos + advance < 1.0) {
                v->pos += advance;
                remaining_dt = 0.0;
            } else {
                double remaining_frac = 1.0 - v->pos;
                double time_to_cross  = (remaining_frac * len) / speed;
                remaining_dt -= time_to_cross;

                /* Update occupancy: leaving old edge */
                if (eid >= 0 && eid < num_edges && occ[eid] > 0)
                    occ[eid]--;

                v->pos = 0.0;
                v->edge_idx++;

                if (v->edge_idx >= v->route_len) {
                    v->state = CAR_ARRIVED;
                } else {
                    /* Update occupancy: entering new edge */
                    int neid = v->route_edges[v->edge_idx];
                    if (neid >= 0 && neid < num_edges)
                        occ[neid]++;
                }
            }
        }
    }

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

/* ---------------- worker threads ---------------- */

static void* routing_worker_main(void* arg) {
    ServerState* st = (ServerState*)arg;

    while (1) {
        Task* t = queue_pop(&st->routing_q);
        char* resp = NULL;

        if (t->type == TASK_REQ) {
            pthread_rwlock_rdlock(&st->graph_lock);
            resp = build_route_response(st->g, t->user_id, t->car_id, t->src, t->dst);
            pthread_rwlock_unlock(&st->graph_lock);

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
        pthread_rwlock_wrlock(&st->graph_lock);
        char* resp = apply_update(st->g, t->user_id, t->car_id, t->edge_id, t->speed);
        pthread_rwlock_unlock(&st->graph_lock);

        task_complete(t, resp);
    }
    return NULL;
}

/* ---------------- per-client network thread ---------------- */

typedef struct {
    ServerState* st;
    int client_fd;
} ClientCtx;

static void* client_thread_main(void* arg) {
    ClientCtx* ctx = (ClientCtx*)arg;
    ServerState* st = ctx->st;
    int client_fd = ctx->client_fd;

    fprintf(stderr, "Client connected (fd=%d).\n", client_fd);

    char line[1024];
    while (1) {
        int r = recv_line(client_fd, line, (int)sizeof(line));
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

        /* --- REGISTER: {"register_car":1,"user_id":...,"car_id":...,"start_node":...,"destination_node":...} --- */
        if (json_extract_int(line, "register_car", &flag) && flag &&
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

int server_run(Graph* g, int port) {
    static ServerState st;
    memset(&st, 0, sizeof(st));
    st.g = g;

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

    /* Start worker pools */
    for (int i = 0; i < ROUTE_WORKERS; i++) {
        if (pthread_create(&st.routing_workers[i], NULL, routing_worker_main, &st) != 0) {
            fprintf(stderr, "pthread_create routing worker failed\n");
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
