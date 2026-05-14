//switch stuff and reading from the pqueue and all that
#include <stdlib.h>
#include <stddef.h>
#include <stdio.h>

#include "structs.h"
#include "constants/constants.h"
#include "constants/config.h"
#include "constants/status_codes.h"
#include "request_handler.h"


int process_requests(void) {
    struct RequestQueue* queue = get_request_queue();
    if (!queue || queue_is_empty(queue)) {
        return 0;
    }
    
    fprintf(stderr, "Processing requests...\n");
    struct RoverRequest req = pop_next_request(queue);

        //make rover struct
        struct Rover* rover = malloc(sizeof(struct Rover));
        if (rover == NULL) { return -1; }
        
        // Initialize rover with request data
        rover->rover_id = req.rover_id;
        rover->req_flag = req.req_flag;
        rover->position = req.position;
        rover->battery_level = req.battery_level;
        rover->timestamp = req.timestamp;
        rover->route_status = NEED_NEW_ROUTE;  // Proper initialization - rover needs a route
        rover->route_error = 0;  // No error initially (0 is not a valid error code, errors are negative)
        rover->instructions = NULL;

        switch (req.req_flag) {
            /*case DEAD_ROVER:
                death_in_flight(rover);
                break;*/
            case CRITICAL_BATTERY:
                rover->req_flag = POWER_REQUEST; /*POWER_EMERGENCY; //this is for the actual power emergency, temporarily sending regular requests
                //because I haven't fully added the inserts yet in deconf, but the algorithm is made respecting it*/
                rover_request_handler(rover);
                break;
            case LOW_BATTERY:
                rover->req_flag = POWER_REQUEST;
                rover_request_handler(rover);
                break;
            case ROUTE_REQUEST:
                rover->req_flag = ROVER_NO_OBJECTIVE;
                rover_request_handler(rover);
                break;
        }

        free(rover); //ur free now little rover
    return 0;
}
