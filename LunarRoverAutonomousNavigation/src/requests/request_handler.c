#include <stdlib.h>
#include <stdio.h>

#include "structs.h"
#include "constants/constants.h"
#include "constants/status_codes.h"

#include "objectives/power_station_kdtree.h"
#include "objectives/objective_kdtree.h"
#include "objectives/objectives_logic.h"
#include "deconf_pqueue.h"
#include "path_storage.h"
#include "proavo/proavo_createpath.h"
#include "nav/nav_main.h"
#include "request_handler.h"
#include "route/route_planner.h"
#include "route/route_provider.h"


struct RouteRequest* plan_route(struct Rover *rover) {
    struct RouteRequest* request = route_planner(rover);
    if (rover->route_status == NO_SOLUTION_KILL_ROVER || rover->route_status == IDLE) {
        if (rover->route_status == NO_SOLUTION_KILL_ROVER) {
            //kill the rover
            set_rover_kill(rover);
            //manage fallout
            dead_rover(rover);

        } else {
            //get idle instructions
            set_rover_idle(rover);
            send_rover_idle(rover);
        }
        free(request);
        return NULL;
    }
    return request;
}


int rover_request_handler(struct Rover *rover) {
    struct RouteRequest* request = plan_route(rover);
    
    if (request == NULL) {
        return 0;
    }
    fprintf(stderr, "Route planned for rover %d, route_status=%d\n", rover->rover_id, rover->route_status);

    if (rover->route_status == NEED_NEW_ROUTE) {
        //this needs to be handled
        fprintf(stderr, "No possible route for rover %d or no more objectives or power stations\n", rover->rover_id);
    }

    if (rover->route_status == NEED_NEW_INSTRUCTIONS) {
        // create a path and generate instructions
        const struct Location *rovpos = &rover->position;
        const struct Location *objpos = &rover->objective_position;
        
        if (request->objective != NULL) {
            fprintf(stderr, "Creating path from rover %d to objective %d\n", rover->rover_id, request->objective->id);
        } else {
            fprintf(stderr, "Creating path from rover %d to power station\n", rover->rover_id);
        }
        struct VehicleState* path = proavo_createpath(rover);
        if (path == NULL) {
            //no possible path
            rover->route_error = NO_PATH_FOUND;

		if (request->objective != NULL) {
            //blacklist the origin point so it doesn't try the objective again and push the objective back
            //onto the appropriate kd tree
            objective_kdtree_push_and_blacklist(request);
            }
            
            //push back to request queue
            struct RoverRequest retry_request = {
            retry_request.rover_id = rover->rover_id,
            retry_request.req_flag = rover->req_flag,
            retry_request.position = rover->position,
            retry_request.battery_level = rover->battery_level,
            retry_request.timestamp = rover->timestamp,
            retry_request.yaw_rads = rover->position.heading_rads
            };
            push_request(get_request_queue(), retry_request);
            free(request);

            return 0;
        }
        fprintf(stderr, "Path found! Generating instructions...\n");

        //pass in pointer to found path from createpath
        //use nav to generate instructions and metadata
        struct InstructionPackage* instructions = nav_main(path, rover->position, rover->objective_position);
        if (instructions == NULL) {
            fprintf(stderr, "Failed to generate instructions\n");
            
            if (request->objective != NULL) {
            objective_kdtree_push_and_blacklist(request);
            }

            //push back to request queue
            struct RoverRequest retry_request = {
            retry_request.rover_id = rover->rover_id,
            retry_request.req_flag = rover->req_flag,
            retry_request.position = rover->position,
            retry_request.battery_level = rover->battery_level,
            retry_request.timestamp = rover->timestamp,
            retry_request.yaw_rads = rover->position.heading_rads
            };
            push_request(get_request_queue(), retry_request);
            free(request);
            return 0;
        }
        // Save the generated path for future use
        save_current_path(instructions, rovpos, objpos);

        //assign the instructions to the rover
        if (!assign_route(rover, instructions)) {
            //push back to request queue

            //blacklist the origin point so it doesn't try the objective again and push the objective back
            //onto the appropriate kd tree
            if (request->objective != NULL) {
            objective_kdtree_push_and_blacklist(request);
            }
            struct RoverRequest retry_request = {
            retry_request.rover_id = rover->rover_id,
            retry_request.req_flag = rover->req_flag,
            retry_request.position = rover->position,
            retry_request.battery_level = rover->battery_level,
            retry_request.timestamp = rover->timestamp,
            retry_request.yaw_rads = rover->position.heading_rads
            };
            push_request(get_request_queue(), retry_request);

            free(request);
            return 0;
        }
        instrpack_heap_push(instructions, deconf_pqueue_get_array(), deconf_pqueue_get_count_ptr());
        fprintf(stderr, "Instructions generated\n");
    }
    
    free(request);
    return 0;
}
