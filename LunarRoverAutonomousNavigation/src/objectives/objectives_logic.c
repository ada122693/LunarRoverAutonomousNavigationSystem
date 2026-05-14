#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "structs.h"
#include "constants/arrsize_global.h"
#include "constants/constants.h"

#include "objectives/objective_kdtree.h"
#include "objectives/power_station_kdtree.h"

// Objectives logical operations
//MAX_BLACKLISTED_ORIGINS - the maximum amount of origin locations that can be blacklisted for an objective
//LOCATION_EPSILON - a cushion for locations to eliminate tiny differences


static double calculate_objective_score(struct Location rover_pos, struct Objective* obj, int priority_weight);
static struct Objective* find_best_objective(struct Location rover_pos);
static void shift_trees_if_needed(void);
static void remove_objective_from_tree(struct Objective* obj);
static bool is_origin_blacklisted_for_objective(struct Objective* obj, struct Location origin);


// Global variables for objectives system
struct Objective objectives[MAX_OBJECTIVES];
struct PowerStation power_stations[MAX_OBJECTIVES];
int power_station_count = 0;
int objective_counts[MAX_OBJECTIVE_PRIORITY_LEVELS] = {0};
int total_objective_count = 0;

// For tree shifting to prevent starvation
static int current_highest_priority = 0;


/* finding best objective */

// Helper function to calculate distance-weighted score
static double calculate_objective_score(struct Location rover_pos, struct Objective* obj, int priority_weight) {
    double distance = sqrt(pow(rover_pos.x_dim - obj->location.x_dim, 2) + 
                         pow(rover_pos.y_dim - obj->location.y_dim, 2));
    
    // Weight by priority and inverse distance (closer = higher score)
    // Add small epsilon to avoid division by zero
    double distance_score = (distance > 0.0) ? 
                           (DISTANCE_WEIGHT_MULTIPLIER / (distance + DISTANCE_EPSILON)) : 
                           (DISTANCE_WEIGHT_MULTIPLIER / distance);
    double priority_score = (MAX_OBJECTIVE_PRIORITY_LEVELS - priority_weight) * PRIORITY_WEIGHT_MULTIPLIER;
    
    return priority_score + distance_score;
}

// Find best objective based on distance and priority
static struct Objective* find_best_objective(struct Location rover_pos) {
    struct Objective* best_obj = NULL;
    double best_score = -1.0;
    
    // Check from current highest priority downwards
    for (int priority = current_highest_priority; priority < MAX_OBJECTIVE_PRIORITY_LEVELS; priority++) {
        if (objective_counts[priority] == 0) {
            continue; // Skip empty priority levels
        }
        
        // Find nearest objective in this priority level
        struct Objective* nearest = objective_kdtree_find_nearest_in_priority(priority, rover_pos);
        if (nearest != NULL) {
            // Check if this objective is blacklisted for the current rover position
            if (is_origin_blacklisted_for_objective(nearest, rover_pos)) {
                // Find the next-nearest objective in this priority level
                nearest = objective_kdtree_find_next_nearest_in_priority(priority, rover_pos, nearest);
                if (nearest == NULL) {
                    continue; // No non-blacklisted objectives in this priority level
                }
            }
            
            double score = calculate_objective_score(rover_pos, nearest, priority);
            if (score > best_score) {
                best_score = score;
                best_obj = nearest;
            }
        }
    }
    
    return best_obj;
}

// Shift trees to prevent starvation
static void shift_trees_if_needed(void) {
    // Find the next non-empty priority level
    while (current_highest_priority < MAX_OBJECTIVE_PRIORITY_LEVELS && 
           objective_counts[current_highest_priority] == 0) {
        current_highest_priority++;
    }
    
    // If we've exhausted all priority levels, reset to 0
    if (current_highest_priority >= MAX_OBJECTIVE_PRIORITY_LEVELS) {
        current_highest_priority = 0;
    }
}

// Route planner integration functions
bool check_objectives(struct RouteRequest* request) {
    if (total_objective_count == 0) return false;
    
    // Shift trees if current priority is empty
    shift_trees_if_needed();
    
    // Find best objective based on distance and priority
    struct Objective* next_obj = find_best_objective(request->rover->position);
    if (next_obj == NULL) return false;
    
    // Assign objective pointer and destination to request (DO NOT REMOVE FROM TREE)
    request->objective = next_obj;
    request->rover->objective_position = next_obj->location;
    return true; // Success - objective assigned
}

/* remove stuff */

// Function for planner to call when objective validation succeeds
void validate_and_remove_objective(struct Objective* objective) {
    // Move objective from original tree to reserved tree
    remove_objective_from_tree(objective);
}

// Remove objective from its KD-tree
static void remove_objective_from_tree(struct Objective* obj) {
    int priority = obj->priority;
    if (priority >= 0 && priority < MAX_OBJECTIVE_PRIORITY_LEVELS) {
        objective_kdtree_remove(NULL, obj);
        objective_counts[priority]--;
        total_objective_count--;
    }
}


/* blacklist stuff */

void objective_kdtree_push_and_blacklist(struct RouteRequest* request) {
    struct Objective* obj = request->objective;
    struct Location origin = request->rover->position;
    
    if (!obj) return;
    
    // Check if origin is already blacklisted
    for (int i = 0; i < obj->blacklisted_count; i++) {
        if (fabs(obj->blacklisted_origins[i].x_dim - origin.x_dim) < LOCATION_EPSILON && 
            fabs(obj->blacklisted_origins[i].y_dim - origin.y_dim) < LOCATION_EPSILON) {
            return; // Already blacklisted
        }
    }
    
    // Add origin to blacklist
    if (obj->blacklisted_count < MAX_BLACKLISTED_ORIGINS) {
        obj->blacklisted_origins[obj->blacklisted_count] = origin;
        obj->blacklisted_count++;
    }
    
    // Pull from reserved tree and add back to priority tree
    if (obj->priority >= 0 && obj->priority < MAX_OBJECTIVE_PRIORITY_LEVELS) {
        objective_kdtree_push(obj);
    }

    //cleanup request
    request->rover->objective_position = (struct Location){0.0,0.0,0.0};
}


bool is_origin_blacklisted_for_objective(struct Objective* obj, struct Location origin) {
    if (!obj) return false;
    
    for (int i = 0; i < obj->blacklisted_count; i++) {
        if (fabs(obj->blacklisted_origins[i].x_dim - origin.x_dim) < LOCATION_EPSILON && 
            fabs(obj->blacklisted_origins[i].y_dim - origin.y_dim) < LOCATION_EPSILON) {
            return true;
        }
    }
    return false;
}

void blacklist_origin_for_failed_objective(struct RouteRequest* request) {
    struct Objective* obj = request->objective;
    struct Location origin = request->rover->position;
    
    if (!obj) return;
    
    if (obj->blacklisted_count >= MAX_BLACKLISTED_ORIGINS) return;
    
    for (int j = 0; j < obj->blacklisted_count; j++) {
        if (fabs(obj->blacklisted_origins[j].x_dim - origin.x_dim) < LOCATION_EPSILON && 
            fabs(obj->blacklisted_origins[j].y_dim - origin.y_dim) < LOCATION_EPSILON) {
            return; 
        }
    }
    
    // add to blacklist
    obj->blacklisted_origins[obj->blacklisted_count] = origin;
    obj->blacklisted_count++;

    //unset
    request->rover->objective_position = (struct Location){0.0,0.0,0.0};
}
