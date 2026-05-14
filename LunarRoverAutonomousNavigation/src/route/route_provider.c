#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>

#include "constants/constants.h"
#include "structs.h"
#include "constants/status_codes.h"

#include "battery/batterymath.h"
#include "path_storage.h"

//checks and provides routes

// Search for matching instructions from saved
struct Route* search_for_instructions(struct RouteRequest *route_request) {
    if (route_request == NULL || route_request->rover == NULL) {
        return NULL;
    }

    struct InstructionPackage* existing_path = find_matching_path(&route_request->rover->position, &route_request->rover->objective_position);
    if (existing_path == NULL) {
        return NULL;
    }

    // Create a Route wrapper for the found instruction package
    struct Route* route = malloc(sizeof(struct Route));
    if (route == NULL) {
        return NULL;
    }

    route->instructions = existing_path;
    route->est_batt_drain = existing_path->metadata.joule_cost;
    route->priority = route_request->priority;
    route->valid = true;

    return route;
}

//checks if path is feasible -> if so assign
bool assign_route(struct Rover *rover, struct InstructionPackage *instructions) {
    if (rover == NULL || instructions == NULL) {
        return false;
    }
    fprintf(stderr, "battery: %.2f joule_cost: %.2f count: %d\n", rover->battery_level, instructions->metadata.joule_cost, instructions->metadata.count);

    int est_safe_waits = safe_waits_motor_calc(rover->battery_level, instructions->metadata.joule_cost, instructions->metadata.count);

    //already protected against low values, returns 0 if below margin
    if (est_safe_waits <= 0) {
        return false;
    }

    // Set metadata from rover
    instructions->metadata.rover_id = rover->rover_id;
    instructions->metadata.priority = rover->priority;
    instructions->metadata.timestamp = rover->timestamp;
    instructions->metadata.req_flag = rover->req_flag;
    instructions->metadata.route_status = rover->route_status;
    instructions->metadata.route_error = rover->route_error;
    instructions->metadata.battery_level = rover->battery_level;

    // Assign route
    rover->instructions = instructions;
    rover->instructions->metadata.wait_slot_allowance = est_safe_waits;
    return true;
}

int route_provider(struct RouteRequest *route_request) {
    struct Route *proposed_route = search_for_instructions(route_request);

    if (proposed_route != NULL && route_request->priority == MAX_PRIORITY) {
        if (assign_route(route_request->rover, proposed_route->instructions)) {
            route_request->rover->route_status = ROUTE_ASSIGNED;
            return 0;
        }
    }

    if (proposed_route != NULL) {
        if (assign_route(route_request->rover, proposed_route->instructions)) {
            route_request->rover->route_status = ROUTE_ASSIGNED;
            return 0;
        }
    }

    route_request->rover->route_status = NEED_NEW_INSTRUCTIONS;
    return NEED_NEW_INSTRUCTIONS;
}
