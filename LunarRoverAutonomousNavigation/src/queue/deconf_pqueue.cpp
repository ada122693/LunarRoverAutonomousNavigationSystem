#include "deconf_pqueue.h"
#include "structs.h"
#include "rover_config.h"
#include <algorithm>
#include <cstdio>
#include <ctime>

// Comparison function for InstructionPackage (max-heap by priority, then by count)
static inline bool compare_instrpack(struct InstructionPackage* a, struct InstructionPackage* b) {
    if (a->metadata.priority != b->metadata.priority) {
        return a->metadata.priority < b->metadata.priority; // max-heap behavior
    }
    return a->metadata.count < b->metadata.count;
}

// Generic heap operations for InstructionPackage arrays
extern "C" void instrpack_heap_push(struct InstructionPackage* package, struct InstructionPackage** pq, int* pq_count) {
    if (!package || !pq || !pq_count) return;
    
    // Get actual number of rovers for capacity check
    int capacity = get_actual_num_rovers();
    if (capacity == 0) capacity = 10; // Fallback to MAX_ROVERS
    
    // Only check for duplicates if queue is at capacity
    if (*pq_count >= capacity) {
        // Queue is full, check for duplicate rover_id
        for (int i = 0; i < *pq_count; i++) {
            if (pq[i] != NULL && pq[i]->metadata.rover_id == package->metadata.rover_id) {
                // Duplicate found - compare timestamps
                fprintf(stderr, "ERROR: Duplicate InstructionPackage detected for rover_id %d (queue at capacity)\n", package->metadata.rover_id);
                if (package->metadata.timestamp > pq[i]->metadata.timestamp) {
                    // New package is more recent, replace the old one
                    fprintf(stderr, "  Keeping newer package (timestamp: %.2f), discarding older (timestamp: %.2f)\n",
                            package->metadata.timestamp, pq[i]->metadata.timestamp);
                    pq[i] = package;
                    // Re-heapify since we replaced an element
                    std::make_heap(pq, pq + *pq_count, compare_instrpack);
                    return;
                } else {
                    // Old package is more recent, discard the new one
                    fprintf(stderr, "  Discarding newer package (timestamp: %.2f), keeping older (timestamp: %.2f)\n",
                            package->metadata.timestamp, pq[i]->metadata.timestamp);
                    return;
                }
            }
        }
        // No duplicate found but queue is full - reject the push
        fprintf(stderr, "ERROR: Queue at capacity (%d), cannot add new InstructionPackage for rover_id %d\n", capacity, package->metadata.rover_id);
        return;
    }
    
    // Queue has space, add normally
    pq[*pq_count] = package;
    (*pq_count)++;
    std::push_heap(pq, pq + *pq_count, compare_instrpack);
}

extern "C" struct InstructionPackage* instrpack_heap_pop(struct InstructionPackage** pq, int* pq_count) {
    if (!pq || !pq_count || *pq_count == 0) return NULL;
    
    std::pop_heap(pq, pq + *pq_count, compare_instrpack);
    (*pq_count)--;
    return pq[*pq_count];
}

// Memory management for InstructionPackage queues (capacity based on actual number of rovers)
extern "C" struct InstructionPackage** instrpack_queue_alloc(int capacity) {
    if (capacity <= 0) return NULL;
    return (struct InstructionPackage**)malloc(capacity * sizeof(struct InstructionPackage*));
}

extern "C" void instrpack_queue_free(struct InstructionPackage** pq) {
    if (pq) free(pq);
}

//so much nesting :(
extern "C" void free_instruction_package(struct InstructionPackage* package) {
    if (!package) return;
    struct Instruction* instr = package->instructions;
    while (instr != NULL) {
        struct Instruction* next = instr->next;
        free(instr);
        instr = next;
    }
    if (package->has_safe_haven) free(package->has_safe_haven);
    if (package->hooks) free(package->hooks);
    if (package->sectors_traversal) {
        if (package->sectors_traversal->sectors) free(package->sectors_traversal->sectors);
        if (package->sectors_traversal->num_reservations) free(package->sectors_traversal->num_reservations);
        if (package->sectors_traversal->sharesTail) free(package->sectors_traversal->sharesTail);
        free(package->sectors_traversal);
    }
    free(package);
}

// Deconfliction priority queue (pending routes)
static struct InstructionPackage** g_deconf_pq = NULL;
static int g_deconf_capacity = 0;
static int g_deconf_count = 0;

extern "C" void deconf_pqueue_init(int num_rovers) {
    if (g_deconf_pq != NULL) return;
    
    g_deconf_pq = instrpack_queue_alloc(num_rovers);
    g_deconf_capacity = num_rovers;
    g_deconf_count = 0;
}

extern "C" void deconf_pqueue_cleanup(void) {
    if (g_deconf_pq == NULL) return;
    // the most nested structs possibly?
    for (int i = 0; i < g_deconf_count; i++) {
        if (g_deconf_pq[i] != NULL) {
            free_instruction_package(g_deconf_pq[i]);
        }
    }
    instrpack_queue_free(g_deconf_pq);
    g_deconf_pq = NULL;
    g_deconf_capacity = 0;
    g_deconf_count = 0;
}

extern "C" void deconf_pqueue_push(struct InstructionPackage* package) {
    instrpack_heap_push(package, g_deconf_pq, &g_deconf_count);
}

extern "C" struct InstructionPackage* deconf_pqueue_pop(void) {
    return instrpack_heap_pop(g_deconf_pq, &g_deconf_count);
}

extern "C" bool deconf_pqueue_is_empty(void) {
    return g_deconf_count == 0;
}

extern "C" int deconf_pqueue_size(void) {
    return g_deconf_count;
}

// Getter for deconf queue array and count (for direct generic function access)
extern "C" struct InstructionPackage** deconf_pqueue_get_array(void) {
    return g_deconf_pq;
}

extern "C" int* deconf_pqueue_get_count_ptr(void) {
    return &g_deconf_count;
}

// Replan queue (uses deconf_pqueue for failed routes that need replanning)
extern "C" void replan_pqueue_push(struct InstructionPackage* package) {
    deconf_pqueue_push(package);
}
