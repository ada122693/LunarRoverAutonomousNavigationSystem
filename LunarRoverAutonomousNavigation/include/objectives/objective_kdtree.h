#ifndef OBJECTIVE_KDTREE_H
#define OBJECTIVE_KDTREE_H

#include "structs.h"

#ifdef __cplusplus
extern "C" {
#endif

// Forward declaration for compatibility
struct ObjectiveKDTree;

// KD-tree functions (C wrapper interface)
void objective_kdtree_add(struct ObjectiveKDTree* tree, struct Objective* objective);
void objective_kdtree_build(struct ObjectiveKDTree* tree);
void objective_kdtree_remove(struct ObjectiveKDTree* tree, struct Objective* objective);

// Additional functions for priority-specific search
struct Objective* objective_kdtree_find_nearest_in_priority(int priority, struct Location location);
struct Objective* objective_kdtree_find_next_nearest_in_priority(int priority, struct Location location, struct Objective* exclude_obj);
void objective_kdtree_push(struct Objective* obj);
void objective_kdtree_cleanup(void);

#ifdef __cplusplus
}
#endif

#endif
