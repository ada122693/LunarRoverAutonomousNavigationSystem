#ifndef PROAVO_PQUEUE_H
#define PROAVO_PQUEUE_H

#include "structs.h"

#ifdef __cplusplus
extern "C" {
#endif

/** priority queue push */
void heap_push(struct VehicleState *node, struct VehicleState **pq, int *pq_count);

/** priority queue pop */
struct VehicleState *heap_pop(struct VehicleState **pq, int *pq_count);
#ifdef __cplusplus
}
#endif
#endif
