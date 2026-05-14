#ifndef DECONF_PQUEUE_H
#define DECONF_PQUEUE_H

#include "structs.h"

#ifdef __cplusplus
extern "C" {
#endif

// Generic heap operations for InstructionPackage arrays (pass queue as parameter)
void instrpack_heap_push(struct InstructionPackage* package, struct InstructionPackage** pq, int* pq_count);
struct InstructionPackage* instrpack_heap_pop(struct InstructionPackage** pq, int* pq_count);

// Memory management for InstructionPackage queues
struct InstructionPackage** instrpack_queue_alloc(int capacity);
void instrpack_queue_free(struct InstructionPackage** pq);

/** initialize deconfliction priority queue */
void deconf_pqueue_init(int num_rovers);

/** cleanup deconfliction priority queue */
void deconf_pqueue_cleanup(void);

/** push instruction package to deconfliction queue */
void deconf_pqueue_push(struct InstructionPackage *package);

/** pop highest priority instruction package from deconfliction queue */
struct InstructionPackage *deconf_pqueue_pop(void);

/**check if deconfliction queue is empty */
bool deconf_pqueue_is_empty(void);

/** get size of deconfliction queue */
int deconf_pqueue_size(void);

/** get deconf queue array (for direct generic function access) */
struct InstructionPackage** deconf_pqueue_get_array(void);
int* deconf_pqueue_get_count_ptr(void);

/**push to replan queue (uses deconf_pqueue) */
void replan_pqueue_push(struct InstructionPackage* package);

#ifdef __cplusplus
}
#endif

#endif
