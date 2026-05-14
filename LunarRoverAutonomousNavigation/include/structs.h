#include <stdbool.h>
#include <pthread.h>

#include "constants/config.h"
#include "arrsize_global.h"
#include "dead_rover.h"

#ifndef STRUCTS_H
#define STRUCTS_H

#ifdef __cplusplus
extern "C" {
#endif

//basic

// Positions in space
struct Location {
    double x_dim;
    double y_dim;
    double heading_rads;
};

// Pose for position-based instructions
struct Pose {
    double x;
    double y;
    double z;
};

// Quaternion for orientation
struct Quaternion {
    double x;
    double y;
    double z;
    double w;
};

// pathfinding

// A* vehicle state
struct VehicleState {
    struct Location position;
    double accumulated_path_cost;  // joules
    double total_time_cost;         // seconds
    double travel_time;
    double f_cost;
    bool safe_haven;                // where the agent can stop during deconfliction
    struct VehicleState* parent_node;
};

//deconf

// Deconfliction interval tracking
struct safe_wait_intervals {
    int next_changed_interval;
    int prev_changed_interval;
};

// Sector traversal information
struct SectorTraversal {
    int* sectors;
    int* num_reservations;
    bool* sharesTail;
    int count;
};


//nav

// Wait hook information (indexed by sector)
struct WaitHookInfo {
    int instruction_index;  // which instruction has the wait
    double wait_duration;  // -1 for not possible
};

// POSE-BASED INSTRUCTION FORMAT (currently active)
struct Instruction {
    struct Pose pose_position;
    struct Quaternion pose_orientation;
    struct Instruction *next;
};

// Instruction metadata
struct InstructionMetadata {
    int count;
    double total_time;
    int path_length_in_sectors;
    double joule_cost;

    struct Location start_location;
    struct Location end_location;
    double battery_level; //joule
    double timestamp; 

    bool valid;
    int rover_id;
    int req_flag;
    int priority;

    int route_status;
    int route_error;

    int num_waits;
    int wait_slot_allowance;
};

// Complete instruction package
struct InstructionPackage {
    struct Instruction* instructions;
    struct InstructionMetadata metadata;
    bool* has_safe_haven;
    struct WaitHookInfo* hooks;
    struct SectorTraversal* sectors_traversal;
    int first_wait; // Wait duration from objective start_time to when rover actually starts moving (in time slots)
};

// rover

// Rover request for routing
struct RoverRequest {
    int rover_id;
    int req_flag;
    struct Location position;
    double battery_level;
    double timestamp;
    double yaw_rads;
};

// Rover structure for managing rover state
struct Rover {
    int rover_id;
    int req_flag;
    int priority;
    
    struct Location position;
    struct Location objective_position;
    double battery_level; //joule
    double timestamp;
    
    // Route status flags
    int route_status;
    int route_error;
    
    // Assigned instructions
    struct InstructionPackage* instructions;
};

// Route structure for path planning
struct Route {
    struct InstructionPackage* instructions;
    double est_batt_drain;
    int priority;
    bool valid;
};

// List of routes for evaluation
struct RouteList {
    struct Route** routes;
    int size;
    int capacity;
};

// Objective structure
struct Objective {
    int id;
    int priority;
    struct Location location;
    struct Location blacklisted_origins[MAX_BLACKLISTED_ORIGINS];
    int blacklisted_count;
};

// Power station structure
struct PowerStation {
    int id;
    struct Location location;
};

// Route request structure
struct RouteRequest {
    struct Rover* rover;
    int priority;
    struct Objective* objective; // Pointer to the actual objective
};

// queues

// Request queue structure
struct RequestQueue {
    struct RoverRequest* requests;
    int front;
    int rear;
    int size;
    int capacity;
    pthread_rwlock_t rwlock;
};

// Priority queue for instruction packages
struct InstructionPackageQueue {
    struct InstructionPackage** packages;
    int size;
    int capacity;
};

// functions related to structs

// Generic InstructionPackage heap operations (pass queue as parameter)
void instrpack_heap_push(struct InstructionPackage* package, struct InstructionPackage** pq, int* pq_count);
struct InstructionPackage* instrpack_heap_pop(struct InstructionPackage** pq, int* pq_count);
struct InstructionPackage** instrpack_queue_alloc(int capacity);
void instrpack_queue_free(struct InstructionPackage** pq);

void free_instruction_package(struct InstructionPackage* package);

// Deconfliction priority queue (wrapper functions for global queue)
void deconf_pqueue_init(int num_rovers);
void deconf_pqueue_cleanup(void);
void deconf_pqueue_push(struct InstructionPackage *package);
struct InstructionPackage *deconf_pqueue_pop(void);
bool deconf_pqueue_is_empty(void);
int deconf_pqueue_size(void);
void replan_pqueue_push(struct InstructionPackage* package);

// queue functions for requests
void init_request_queue(struct RequestQueue* queue, int capacity);
bool queue_is_empty(struct RequestQueue* queue);
struct RoverRequest pop_next_request(struct RequestQueue* queue);
void push_request(struct RequestQueue* queue, struct RoverRequest request);
void free_request_queue(struct RequestQueue* queue);

// power station routing
bool find_power_route(struct RouteRequest *request);

// route failure handling
void no_path_found(struct RouteRequest *request);

// system initialization
int system_initialization(void);
struct RequestQueue* get_request_queue(void);
void system_cleanup(void);


#ifdef __cplusplus
}
#endif

#endif
