#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

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

/* ---------------- task + queues ---------------- */

typedef enum {
    TASK_REQ = 1,
    TASK_UPD = 2
} TaskType;

typedef struct Task {
    TaskType type;

    Graph* g;
    pthread_rwlock_t* graph_lock;

    int client_fd;

    /* REQ payload */
    int src;
    int dst;

    /* UPD payload */
    int edge_id;
    double speed;

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
    pthread_mutex_destroy(&t->mu);
    pthread_cond_destroy(&t->cv);
    free(t);
}

/* ---------------- protocol execution (workers) ---------------- */

static char* build_route_response(Graph* g, int src, int dst) {
    if (src < 0 || src >= g->num_nodes || dst < 0 || dst >= g->num_nodes) {
        return strdup("ERR BAD_NODES\n");
    }

    int max_edges = (g->num_nodes > 0) ? g->num_nodes : 1;
    int* path_edges = (int*)malloc(sizeof(int) * max_edges);
    int* path_nodes = (int*)malloc(sizeof(int) * g->num_nodes);
    if (!path_edges || !path_nodes) {
        free(path_edges);
        free(path_nodes);
        return strdup("ERR NO_MEM\n");
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
        return strdup("ERR NO_ROUTE\n");
    }
    if (rc != 0) {
        free(path_edges);
        free(path_nodes);
        return strdup("ERR ROUTE_FAIL\n");
    }

    /* Safety: ensure edge_count fits what we allocated */
    if (edge_count < 0 || edge_count > max_edges) {
        free(path_edges);
        free(path_nodes);
        return strdup("ERR ROUTE_FAIL\n");
    }

    if (node_count < 0 || node_count > g->num_nodes) {
        free(path_edges);
        free(path_nodes);
        return strdup("ERR ROUTE_FAIL\n");
    }

    size_t buf_sz = 64 + (size_t)edge_count * 16 + (size_t)node_count * 16;
    char* resp = (char*)malloc(buf_sz);
    if (!resp) {
        free(path_edges);
        free(path_nodes);
        return strdup("ERR NO_MEM\n");
    }

    int n = snprintf(resp, buf_sz, "ROUTE2 %.3f %d", cost, node_count);
    for (int i = 0; i < node_count && n > 0 && (size_t)n < buf_sz; i++) {
        n += snprintf(resp + n, buf_sz - (size_t)n, " %d", path_nodes[i]);
    }
    n += snprintf(resp + n, buf_sz - (size_t)n, " %d", edge_count);
    for (int i = 0; i < edge_count && n > 0 && (size_t)n < buf_sz; i++) {
        n += snprintf(resp + n, buf_sz - (size_t)n, " %d", path_edges[i]);
    }
    if (n > 0 && (size_t)n < buf_sz) {
        snprintf(resp + n, buf_sz - (size_t)n, "\n");
    } else {
        free(path_edges);
        free(path_nodes);
        free(resp);
        return strdup("ERR ROUTE_FAIL\n");
    }

    free(path_edges);
    free(path_nodes);
    return resp;
}

static char* apply_update(Graph* g, int edge_id, double speed) {
    if (edge_id < 0 || edge_id >= g->num_edges) {
        return strdup("ERR BAD_EDGE\n");
    }
    if (speed <= 0.0) {
        return strdup("ERR BAD_SPEED\n");
    }

    const double min_speed = 1e-6;
    if (speed < min_speed) speed = min_speed;

    Edge* e = &g->edges[edge_id];
    const double alpha = (e->observation_count == 0) ? 1.0 : 0.2;
    double measured = e->base_length / speed;

    e->ema_travel_time = alpha * measured + (1.0 - alpha) * e->ema_travel_time;
    e->current_travel_time = e->ema_travel_time;
    e->observation_count++;

    return strdup("ACK\n");
}

/* ---------------- server shared state ---------------- */

typedef struct {
    Graph* g;
    pthread_rwlock_t graph_lock;

    TaskQueue routing_q;
    TaskQueue traffic_q;

    pthread_t routing_workers[ROUTE_WORKERS];
    pthread_t traffic_workers[TRAFFIC_WORKERS];
} ServerState;

/* ---------------- worker threads ---------------- */

static void* routing_worker_main(void* arg) {
    ServerState* st = (ServerState*)arg;

    while (1) {
        Task* t = queue_pop(&st->routing_q);
        /* Execute REQ under read lock */
        pthread_rwlock_rdlock(&st->graph_lock);
        char* resp = build_route_response(st->g, t->src, t->dst);
        pthread_rwlock_unlock(&st->graph_lock);

        task_complete(t, resp);
        /* IMPORTANT: client thread destroys task after sending */
    }
    return NULL;
}

static void* traffic_worker_main(void* arg) {
    ServerState* st = (ServerState*)arg;

    while (1) {
        Task* t = queue_pop(&st->traffic_q);
        /* Execute UPD under write lock */
        pthread_rwlock_wrlock(&st->graph_lock);
        char* resp = apply_update(st->g, t->edge_id, t->speed);
        pthread_rwlock_unlock(&st->graph_lock);

        task_complete(t, resp);
        /* client thread destroys task after sending */
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
            send_all(client_fd, "ERR EMPTY\n");
            continue;
        }

        Task* t = task_create(st->g, &st->graph_lock, client_fd);
        if (!t) {
            send_all(client_fd, "ERR NO_MEM\n");
            continue;
        }

        int src, dst;
        int edge_id;
        double speed;

        if (sscanf(line, "REQ %d %d", &src, &dst) == 2) {
            t->type = TASK_REQ;
            t->src = src;
            t->dst = dst;

            queue_push(&st->routing_q, t);

        } else if (sscanf(line, "UPD %d %lf", &edge_id, &speed) == 2) {
            t->type = TASK_UPD;
            t->edge_id = edge_id;
            t->speed = speed;

            queue_push(&st->traffic_q, t);

        } else {
            task_destroy(t);
            send_all(client_fd, "ERR UNKNOWN_CMD\n");
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
            send_all(client_fd, "ERR INTERNAL\n");
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
    ServerState st;
    memset(&st, 0, sizeof(st));
    st.g = g;

    queue_init(&st.routing_q);
    queue_init(&st.traffic_q);

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

    if (listen(listen_fd, 64) < 0) {
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

    /* Unreachable in this assignment version */
    close(listen_fd);
    pthread_rwlock_destroy(&st.graph_lock);
    return 0;
}
