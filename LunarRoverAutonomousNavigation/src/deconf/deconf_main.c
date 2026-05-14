#include <stddef.h>
#include <stdbool.h>
#include <stdio.h>

#include "constants/constants.h"
#include "status_codes.h"
#include "structs.h"

#include "deconf_pqueue.h"
#include "deconf/deconf_algo.h"

//deconf loop that takes a pqueue of pending routes, either writes waits to them
//or invalidates it if invalid



static void add_back_to_request_queue(struct InstructionPackage *route) {
    struct RoverRequest request = {
    request.rover_id = route->metadata.rover_id,
    request.req_flag = route->metadata.req_flag,
    request.position = route->metadata.start_location,
    request.battery_level = route->metadata.battery_level,
    request.timestamp = route->metadata.timestamp,
    request.yaw_rads = route->metadata.start_location.heading_rads
    };
    
    // Push to global request queue
    push_request(get_request_queue(), request);
}

int deconf_main(struct InstructionPackage **pending, struct InstructionPackage **completed, int* completed_count, int start_time) {
    fprintf(stderr, "debug log: deconf_main.c starting deconfliction\n");
    // Get queue array and count for generic heap operations
    struct InstructionPackage** deconf_pq = deconf_pqueue_get_array();
    int* deconf_count = deconf_pqueue_get_count_ptr();
    struct InstructionPackage** completed_pq = completed; // Use the passed completed queue

    // Use the generic heap operations
    struct InstructionPackage *route = instrpack_heap_pop(deconf_pq, deconf_count);
    int try_count = 0;

    // Set the objective start time for all routes in this batch
    int objective_start_time = start_time;

    while (route != NULL && try_count < MAX_DECONF_TRIES) {
        try_count++;
        // Calculate actual start time including first_wait
        int actual_start_time = objective_start_time + route->first_wait;

        // Check if initial first_wait hasn't been cleaned
        //quiet error
        if (route->first_wait != 0) {
            route->metadata.route_error = INACCURATE_INFO_QUIET;
            route->first_wait = 0;
        }

        //lay the route on the schedule and attempt to find conflicts and add waits
        int addr = 0;
        int *time_till_potential_valid = &addr;
        int cumulative_initial_wait = 0;
        if (!deconf_standard(route, time_till_potential_valid, cumulative_initial_wait, actual_start_time)) {
            fprintf(stderr, "debug log: deconf_main.c deconfliction try 1 failed\n");
            //check if it was a battery issue
            if (route->metadata.route_error == INSUFFICIENT_BATTERY) {
                add_back_to_request_queue(route);
                free_instruction_package(route);
                route = instrpack_heap_pop(deconf_pq, deconf_count);
                break;
            }

            //definitely a conflict now
            //need to delay departure
            if (*time_till_potential_valid > 0) {
                // Check if this wait would exceed allowance
                if (*time_till_potential_valid > route->metadata.wait_slot_allowance) {
                    route->metadata.valid = false;
                    route->metadata.route_error = INSUFFICIENT_BATTERY;
                    add_back_to_request_queue(route);
                    free_instruction_package(route);
                    route = instrpack_heap_pop(deconf_pq, deconf_count);
                    continue;
                }

                //only try again so many times
                //this is at the entry point so it can be busy, but it can also be expensive to check here
                //in the case where its actually unsolveable at the end
                int i;
                for (i = 0; i < MAX_DECONF_RETRIES; i++) {
                    cumulative_initial_wait += *time_till_potential_valid;
                    actual_start_time = objective_start_time + route->first_wait + cumulative_initial_wait;
                    if (deconf_standard(route, time_till_potential_valid, cumulative_initial_wait, actual_start_time)) {
                        // Set first_wait to the cumulative initial wait
                        route->first_wait = cumulative_initial_wait;
                        instrpack_heap_push(route, completed_pq, completed_count);
                        break;
                    } else {
                        // deconf_standard failed - check if due to insufficient battery
                        if (route->metadata.route_error == INSUFFICIENT_BATTERY) {
                            add_back_to_request_queue(route);
                            free_instruction_package(route);
                            route = instrpack_heap_pop(deconf_pq, deconf_count);
                            break;
                        }
                    }
                    
                    // Check if the new time_till_potential_valid exceeds allowance
                    if (*time_till_potential_valid + cumulative_initial_wait > route->metadata.wait_slot_allowance) {
                        route->metadata.valid = false;
                        route->metadata.route_error = INSUFFICIENT_BATTERY;
                        add_back_to_request_queue(route);
                        free_instruction_package(route);
                        route = instrpack_heap_pop(deconf_pq, deconf_count);
                        break;
                    }

                    cumulative_initial_wait += *time_till_potential_valid;
                    *time_till_potential_valid = 0;
                }

                if (i == MAX_DECONF_RETRIES - 1) {
                    // Max retries reached, mark as invalid
                    route->metadata.valid = false;
                    route->metadata.route_error = PATH_VALIDATION_FAILED;
                    add_back_to_request_queue(route);
                    free_instruction_package(route);
                    route = instrpack_heap_pop(deconf_pq, deconf_count);
                    continue;
                }
            } else {
                route->metadata.valid = false;
                route->metadata.route_error = (route->metadata.route_error == INSUFFICIENT_BATTERY) ? INSUFFICIENT_BATTERY : SCHEDULING_CONFLICT;
                add_back_to_request_queue(route);
                free_instruction_package(route);
                route = instrpack_heap_pop(deconf_pq, deconf_count);
                continue;
            }
        } else {
            fprintf(stderr, "debug log: deconf_main.c deconfliction try %d succeeded\n", try_count);
            // Route successfully deconflicted - set first_wait and push to completed queue
            route->first_wait = cumulative_initial_wait;
            route->metadata.valid = true;
            instrpack_heap_push(route, completed_pq, completed_count);
        }

        route = instrpack_heap_pop(deconf_pq, deconf_count);
        try_count++;
    }

    return 0;
}
