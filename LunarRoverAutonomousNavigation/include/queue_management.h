#ifndef QUEUE_MANAGEMENT_H
#define QUEUE_MANAGEMENT_H

#include "structs.h"
#ifdef __cplusplus
extern "C" {
#endif

// Request queue functions
void init_request_queue(struct RequestQueue* queue, int capacity);
bool queue_is_empty(struct RequestQueue* queue);
struct RoverRequest pop_next_request(struct RequestQueue* queue);
void push_request(struct RequestQueue* queue, struct RoverRequest request);
void free_request_queue(struct RequestQueue* queue);
int add_runtime_request(int rover_id, int req_flag, struct Location position, double battery_level);

#ifdef __cplusplus
}
#endif

#endif
