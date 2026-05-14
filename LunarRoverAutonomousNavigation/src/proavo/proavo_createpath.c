
 //NO_POSSIBLE_PATH
 //struct Rover {position, objective_position}
 //Using state node pool for memory management - pool_reset() is called on failure

#include "structs.h"
#include "proavo_pool.h"
#include "proavo/proavo_algo.h"

 struct VehicleState* proavo_createpath(struct Rover *rover) {
    // Reset pool for new pathfinding operation
    pool_reset();

    //check if graph exists between the two points
    //if not, request SLAM

    //take the graph and construct data types properly

    //construct VehicleState
    struct Location curr_location = rover->position;
    struct VehicleState* curr_state = pool_alloc_vehicle_state();
    if (curr_state == NULL) {
        pool_reset();
        return NULL;
    }
    curr_state->position = curr_location;
    curr_state->accumulated_path_cost = 0.0;
    curr_state->total_time_cost = 0.0;
    curr_state->travel_time = 0.0;
    curr_state->f_cost = 0.0;
    curr_state->safe_haven = true;
    curr_state->parent_node = NULL;

    struct Location obj_location = rover->objective_position;
    //run the algorithm
    struct VehicleState* result = proavo_algo(curr_state, obj_location);
    
    // If pathfinding failed, reset pool to free allocated nodes
    if (result == NULL) {
        pool_reset();
        return NULL;
    }
    
    return result;
 }
