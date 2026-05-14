
#include "proavo_pqueue.h"
#include "structs.h"
#include <algorithm>

static inline bool compare_min_cost(struct VehicleState *a, struct VehicleState *b) {
  return a->f_cost > b->f_cost;
}

/* counts handled in the algo function*/
extern "C" void heap_push(struct VehicleState *node, struct VehicleState **pq, int *pq_count) {
  pq[*pq_count] = node;
  (*pq_count)++;
  std::push_heap(pq, pq + *pq_count, compare_min_cost);
}
/*counts handled in the algo function*/
extern "C" struct VehicleState *heap_pop(struct VehicleState **pq, int *pq_count) {
  std::pop_heap(pq, pq + *pq_count, compare_min_cost);
  (*pq_count)--;
  return pq[*pq_count];
}
