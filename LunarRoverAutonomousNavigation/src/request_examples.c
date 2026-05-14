#include "structs.h"
#include "constants/constants.h"

// Example function to create a route request
void create_route_request(int rover_id, struct Location current_pos, struct Location objective) {
    struct RequestQueue* queue = get_request_queue();
    if (!queue) return;
    
    struct RoverRequest request = {
        .rover_id = rover_id,
        .req_flag = ROUTE_REQUEST,
        .position = current_pos,
        .battery_level = 40000.0, //40k joules, roughly 75% for the default specs
        .timestamp = 0.0,
        .yaw_rads = current_pos.heading_rads
    };
    
    push_request(queue, request);
}

// Example function to create a low battery request
void create_low_battery_request(int rover_id, struct Location current_pos, double battery_level) {
    struct RequestQueue* queue = get_request_queue();
    if (!queue) return;

    //these are hardcoded just for testing purposes
    //real requests shouldn't make requests this way
    const double CRITICAL_JOULE_THRESHOLD = 10656.0; //roughly 20% of given specs (53280)
    const double LOW_JOULE_THRESHOLD = 18648.0; //roughly 35%
    
    struct RoverRequest request = {
        .rover_id = rover_id,
        .req_flag = (battery_level < CRITICAL_JOULE_THRESHOLD) ? CRITICAL_BATTERY : LOW_BATTERY,
        .position = current_pos,
        .battery_level = battery_level,
        .timestamp = 0.0,
        .yaw_rads = current_pos.heading_rads
    };
    
    push_request(queue, request);
}

// Example function to simulate incoming requests from rovers
void simulate_rover_requests(void) {
    // Example: Rover 1 needs a new route
    struct Location pos1 = {10.0, 15.0, 0.5};
    struct Location obj1 = {50.0, 60.0, 1.0};
    create_route_request(1, pos1, obj1);
    
    // Example: Rover 2 has low battery
    struct Location pos2 = {20.0, 25.0, 0.8};
    create_low_battery_request(2, pos2, 25.0);
    
    // Example: Rover 3 needs a route
    struct Location pos3 = {30.0, 35.0, 1.2};
    struct Location obj3 = {70.0, 80.0, 0.3};
    create_route_request(3, pos3, obj3);
}
