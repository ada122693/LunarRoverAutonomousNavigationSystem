#include <stdint.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include <string.h>


#include "structs.h"
#include "constants/constants.h"
#include "constants/config.h"
#include "arrsize_global.h"

#include "gridmap/gridmap.h"

#include "proavo_pqueue.h"
#include "proavo/proavo_reedsshepp.h"
#include "proavo_pool.h"
#include "proavo/proavo_cost.h"

//
// CONSTANTS THIS NEEDS :
// ACCEPTABLE_MARGIN; //cost margin
// int MAX_OPEN_SET_SIZE;
// int MAX_CLOSED_SET_SIZE;
// float MAX_X_DIM; //this must be set by prior function based on the size of the abstraction of the lidar map
// float MAX_Y_DIM;
// float STEER_STEP_RADS;
// float PRIM_LENGTH;
// double HEADING_GRANULARITY;
// float POTENTIAL_STEERING[]; //this one is precomputed with calculus and whatnot
// int POTENTIAL_STEERING_LEN; //number of turns/steering angles/straight etc
// double ACCEPTABLE_DISTANCE_MARGIN_OBJECTIVE;
// double ACCEPTABLE_HEADING_MARGIN_OBJECTIVE;
//


//memory pre-allocation
struct VehicleState state_node_pool[MAX_NODES_IN_POOL];
static int pool_index = 0;

#define GRID_RESOLUTION PRIM_LENGTH
#define HEADING_RESOLUTION HEADING_GRANULARITY

struct VehicleState* pool_alloc_vehicle_state(void) {
    if (pool_index >= MAX_NODES_IN_POOL) {
        return NULL;
    }
    return &state_node_pool[pool_index++];
}

void pool_reset(void) {
    pool_index = 0;
}

int pool_get_index(void) {
    return pool_index;
}

void pool_rollback(int index) {
    if (index >= 0 && index < pool_index) {
        pool_index = index;
    }
}

//closed set is a 3d lookup pool of (x,y,theta)
static bool visited[GRID_X_CELLS][GRID_Y_CELLS][GRID_HEADING_BINS];

static int get_position_index(double value) {
    int idx = (int)((value + X_DIM_SIZE/2.0) / PRIM_LENGTH + 0.5);
    if (idx < 0) idx = 0;
    if (idx >= GRID_X_CELLS) idx = GRID_X_CELLS - 1;
    return idx;
}

static int get_y_position_index(double value) {
    int idx = (int)((value + Y_DIM_SIZE/2.0) / PRIM_LENGTH + 0.5);
    if (idx < 0) idx = 0;
    if (idx >= GRID_Y_CELLS) idx = GRID_Y_CELLS - 1;
    return idx;
}

static int get_heading_index(double value) {
    while (value < 0) value += 2.0 * M_PI;
    while (value >= 2.0 * M_PI) value -= 2.0 * M_PI;
    int idx = (int)(value / HEADING_GRANULARITY + 0.5);
    if (idx < 0) idx = 0;
    if (idx >= GRID_HEADING_BINS) idx = GRID_HEADING_BINS - 1;
    return idx;
}

static bool reached_objective(struct VehicleState* state, struct Location objective) {
    double dx = state->position.x_dim - objective.x_dim;
    double dy = state->position.y_dim - objective.y_dim;
    double dtheta = fabs(state->position.heading_rads - objective.heading_rads);

    double distance = sqrt(dx*dx + dy*dy);
    if (dtheta > M_PI) dtheta = 2.0*M_PI - dtheta;

    return (distance < ACCEPTABLE_DISTANCE_MARGIN_OBJECTIVE) && (dtheta < ACCEPTABLE_HEADING_MARGIN_OBJECTIVE);
}

// generate new state with kinematics
static struct VehicleState* create_successor (struct VehicleState* current, double steer_angle) {
    struct VehicleState* next = pool_alloc_vehicle_state();
    if (next == NULL) { return NULL; }

    next->position.x_dim = current->position.x_dim + PRIM_LENGTH * cosf (current->position.heading_rads);
    next->position.y_dim = current->position.y_dim + PRIM_LENGTH * sinf (current->position.heading_rads);

    //normalize [0,2PI]
    next->position.heading_rads = current->position.heading_rads + (PRIM_LENGTH / ROVER_WHEELBASE) * tanf(steer_angle);
    if (next->position.heading_rads < 0) next->position.heading_rads += 2.0 * M_PI;
    if (next->position.heading_rads >= 2.0 * M_PI) next->position.heading_rads -= 2.0 * M_PI;

    next->parent_node = current;
    return next;
}

//main hybrid A* algo
struct VehicleState* proavo_algo (struct VehicleState* start_node, struct Location objective) {
    memset(visited, 0, sizeof(visited));
    //this is a priority queue
    struct VehicleState* open_list[MAX_OPEN_SET_SIZE];
    int open_list_count = 0;

    //add s_start to open list
    heap_push ((struct VehicleState*)start_node, open_list, &open_list_count);

    fprintf(stderr, "collision at start? %d\n", check_collision(start_node->position.x_dim, start_node->position.y_dim));
fprintf(stderr, "start pos: %.2f, %.2f\n", start_node->position.x_dim, start_node->position.y_dim);
fprintf(stderr, "obj pos: %.2f, %.2f\n", objective.x_dim, objective.y_dim);
fprintf(stderr, "map inside start? %d\n", !check_collision(start_node->position.x_dim, start_node->position.y_dim));

    while (open_list_count > 0) {
        //pop s
        struct VehicleState* curr_state = (struct VehicleState*)heap_pop (open_list, &open_list_count);

        //check visited, then set this as visited
        if (visited[get_position_index(curr_state->position.x_dim)][get_y_position_index(curr_state->position.y_dim)][get_heading_index(curr_state->position.heading_rads)]) { continue; }
        visited[get_position_index(curr_state->position.x_dim)] [get_y_position_index(curr_state->position.y_dim)] [get_heading_index(curr_state->position.heading_rads)] = true;

        if (reached_objective(curr_state, objective)) {
            fprintf(stderr, "is the prev node valid and real?\n");
            //ERROR CHECK: REMOVE:
            if (curr_state->parent_node == NULL) {
                fprintf(stderr, "ERROR: prev node is NULL\n");
                return NULL;
            } else if (check_collision(curr_state->parent_node->position.x_dim, curr_state->parent_node->position.y_dim)) {
                fprintf(stderr, "ERROR: prev node is in collision\n");
            }
            return curr_state;
        }

        //reerds-shepp
        //this should check the path
        struct VehicleState* rs_end_node = getRSShot(curr_state, objective);
        if (rs_end_node != NULL) {
            return rs_end_node;
        }

        //expand s
        for (int i = 0; i < POTENTIAL_STEERING_LEN; i++) {
            int pre_alloc_index = pool_get_index();
            struct VehicleState* successor = create_successor (curr_state, POTENTIAL_STEERING[i]);

            if (successor == NULL || check_collision (successor->position.x_dim, successor->position.y_dim)) {
                pool_rollback(pre_alloc_index);
                continue;
            }

            successor->safe_haven = false;
            //cost: g(s') = g(s) + cost)s, s')
            //calculate the energy cost for successor, if this function returns false the tipover risk is too high so continue
            //need the slope for the calculation
            double slope = get_slope (successor->position.x_dim, successor->position.y_dim);
            if (!update_energy_cost(curr_state,successor,slope, (double)POTENTIAL_STEERING[i], 0.0)) {
                pool_rollback(pre_alloc_index);
                continue;
            }

            //rs heuristic: reeds-shepp estimate
            //distance*avg energy cost * weight
            double heuristic_cost = rs_heuristic (successor->position, objective) * EST_METER_COST * RS_COST_WEIGHT;

            successor->f_cost = heuristic_cost + successor->accumulated_path_cost;

            //push s' to open
            fprintf(stderr, "pushing node to open with x %f y %f yaw %f\n", successor->position.x_dim, successor->position.y_dim, successor->position.heading_rads);
            heap_push ((struct VehicleState*)successor, open_list, &open_list_count);
        }

    }
    fprintf(stderr, "this should be a fail\n");
    return NULL;
}
