#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>
#include <pthread.h>
#include <stdio.h>

#include "structs.h"

// Request queue, this is used in a thread for the requests made to the server
//by the rovers
void init_request_queue(struct RequestQueue* queue, int capacity) {
    if (!queue || capacity <= 0) return;
    
    queue->requests = malloc(capacity * sizeof(struct RoverRequest));
    queue->capacity = capacity;
    queue->front = 0;
    queue->rear = -1;
    queue->size = 0;
    pthread_rwlock_init(&queue->rwlock, NULL);
}

bool queue_is_empty(struct RequestQueue* queue) {
    if (!queue) return true;
    return queue->size == 0;
}

void push_request(struct RequestQueue* queue, struct RoverRequest request) {
    if (!queue || queue->size >= queue->capacity) return;
    
    pthread_rwlock_wrlock(&queue->rwlock);
    queue->rear = (queue->rear + 1) % queue->capacity;
    queue->requests[queue->rear] = request;
    queue->size++;
    pthread_rwlock_unlock(&queue->rwlock);
}

struct RoverRequest pop_next_request(struct RequestQueue* queue) {
    struct RoverRequest empty_request = {0};
    
    if (!queue || queue_is_empty(queue)) {
        return empty_request;
    }
    
    pthread_rwlock_rdlock(&queue->rwlock);
    struct RoverRequest request = queue->requests[queue->front];
    queue->front = (queue->front + 1) % queue->capacity;
    queue->size--;
    pthread_rwlock_unlock(&queue->rwlock);
    
    return request;
}

void free_request_queue(struct RequestQueue* queue) {
    if (!queue) return;
    
    pthread_rwlock_destroy(&queue->rwlock);
    free(queue->requests);
    queue->requests = NULL;
    queue->capacity = 0;
    queue->size = 0;
    queue->front = 0;
    queue->rear = -1;
}

// add a request to the global queue at runtime
int add_runtime_request(int rover_id, int req_flag, struct Location position, double battery_level) {
    struct RequestQueue* queue = get_request_queue();
    if (!queue) {
        return -1; // System not initialized
    }
    
    struct RoverRequest request = {
        request.rover_id = rover_id,
        request.req_flag = req_flag,
        request.position = position,
        request.battery_level = battery_level,
        request.timestamp = (double)time (NULL),
        request.yaw_rads = position.heading_rads
    };
    
    fprintf(stderr, "at push in queue management: rover x %.2f, y %.2f, yaw_rads %.2f\n",request.position.x_dim, request.position.y_dim, request.position.heading_rads);
    push_request(queue, request);
    return 0;
}
