 #include <stdio.h>

#include "constants/constants.h"
#include "constants/config.h"
#include "constants/status_codes.h"

#include "route/route_planner.h"
#include "route/find_power_route.h"
#include "route/route_provider.h"

#include "objectives/power_station_kdtree.h"
#include "objectives/objectives_logic.h"
#include "objectives/objectives_loader.h"

void no_path_found(struct RouteRequest *request) {
    // Simply blacklist the origin in the objective struct and unassign
    blacklist_origin_for_failed_objective(request);
    
    // Reset rover state to plan again
    request->rover->req_flag = ROVER_NO_OBJECTIVE;  // Reset to need objective state
    request->rover->route_status = NEED_NEW_ROUTE;   // Indicate new route needed
}

struct RouteRequest* route_planner(struct Rover *rover) {
    fprintf(stderr, "Planning route for rover %d, req_flag: %d\n", rover->rover_id, rover->req_flag);
    struct RouteRequest *request;
    request = malloc(sizeof(struct RouteRequest));
    if (request == NULL) {
        return NULL;
    }
    request->rover = rover;
    request->objective = NULL;

    if (rover->req_flag == POWER_EMERGENCY || rover->req_flag == POWER_REQUEST) {
        fprintf(stderr, "Power request, priority: %d\n", rover->req_flag == POWER_EMERGENCY ? MAX_PRIORITY : HIGH_PRIORITY);
        if (rover->req_flag == POWER_EMERGENCY) {
            request->priority = MAX_PRIORITY;
        } else if (rover->req_flag == POWER_REQUEST) {
            request->priority = HIGH_PRIORITY;
        }
        
        power_station_kdtree_clear_temp_blacklist();
        
        for (int i = 0; i < MAX_POWER_STATION_SEARCHES; i++) {
            struct PowerStation* closest_station = power_station_kdtree_find_nearest_excluding_blacklist(
                request->rover->position.x_dim, 
                request->rover->position.y_dim
            );
            
            if (closest_station == NULL) {
                fprintf(stderr, "No power station found\n");
                //either means A. magical walls have risen from the ground around power stations or B. not possible to reach within left battery (more likely)
                break;
            }
            
            // stops trying the station again in the loop
            //this one is TEMPORARY
            power_station_kdtree_add_to_temp_blacklist(closest_station);

            // Set rover's objective position and try to route
            request->rover->objective_position = closest_station->location;

            if (find_power_route(request)) {
                route_provider(request);
                if (rover->route_status == ROUTE_ASSIGNED || rover->route_status == NEED_NEW_INSTRUCTIONS) {
                    // success
                    if (request->objective != NULL) {
                        fprintf(stderr, "power route for rover %d to station %d found with code %d\n", rover->rover_id, request->objective->id, rover->route_status);
                    } else {
                        fprintf(stderr, "power route for rover %d found with code %d\n", rover->rover_id, rover->route_status);
                    }
                    break;
                }
            } else {
                fprintf(stderr, "power route failed\n");
            }

        }
        //since its low battery, we use a temporary blacklist to avoid checking the same power station multiple times
            //but future robots may have enough power to access it
        power_station_kdtree_clear_temp_blacklist();
        // there isn't a reasonable solution
        if (rover->route_status != ROUTE_ASSIGNED && rover->route_status != NEED_NEW_INSTRUCTIONS && request->priority == MAX_PRIORITY) {
            rover->route_status = NO_SOLUTION_KILL_ROVER;
        } else if (rover->route_status != ROUTE_ASSIGNED && rover->route_status != NEED_NEW_INSTRUCTIONS && request->priority == HIGH_PRIORITY) {
            rover->route_status = NEED_NEW_ROUTE;
            request->priority = HIGH_PRIORITY;
            rover->req_flag = POWER_EMERGENCY;
        }
        return request;
    }

    if (rover->req_flag == ROVER_NO_OBJECTIVE) {
        fprintf(stderr, "Determining objective, objective count = %d\n", get_objective_count());
        // Check if objectives are available before trying to assign
        if ((get_objective_count() > 0) && (check_objectives(request))) {
            // Get next objective from priority queue
            // Objective destination was set by check_objectives
            request->priority = NORMAL_PRIORITY;
            route_provider(request);
            
            // Remove objective from tree after successful route assignment
            if (request->rover->route_status == ROUTE_ASSIGNED || request->rover->route_status == NEED_NEW_INSTRUCTIONS) {
                validate_and_remove_objective(request->objective);
            } else {
                //insufficient power for objective, this means there's a possible path but the rover cannot take it
                //regardless of the fact that it has relatively high battery percentage
                //the objective is far from this location, right now I'm going to blacklist as a safety behavior
                //but different behavior, like charging first as a first segment of the route rather than as a separate objective
                //could improve throughput if that is determined safe
                blacklist_origin_for_failed_objective(request);
                // Reset rover state to plan again
                request->rover->req_flag = ROVER_NO_OBJECTIVE;  // Reset to need objective state
                request->rover->route_status = NEED_NEW_ROUTE;   // Indicate new route needed
            }
            return request;
        } else {
            // No objectives available, or isssue with objoective assignment, try to find power station
            request->priority = LOW_PRIORITY;
            if (find_power_route(request)) {
                route_provider(request);
                if (request->rover->route_status == ROUTE_ASSIGNED || request->rover->route_status == NEED_NEW_INSTRUCTIONS) {
                    return request;
                } else {
                    // Reset rover state to plan again
                    request->rover->req_flag = ROVER_NO_OBJECTIVE;  // Reset to need objective state
                    request->rover->route_status = IDLE;
                    return request; //its not important for this to be given a power route right now
                }
            } else {
                fprintf(stderr, "power route failed, setting IDLE\n");
                rover->route_status = IDLE;
                return request;
            }
        }
    }
    return request;
}
