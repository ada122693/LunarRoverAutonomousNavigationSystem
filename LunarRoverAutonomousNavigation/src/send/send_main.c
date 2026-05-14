#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "structs.h"
#include "constants/constants.h"
#include "constants/config.h"

#include "deconf_pqueue.h"
#include "rover_action_client.h"
#include "rover_config.h"

#include "io/platform_specific_instructions_converter.h"
#include "io/io_send_instructions.h"
#include "dead_rover.h"
#include "dispatcher.h"
#include "deconf/deconf_main.h"


const uint64_t MAX_TRANSMISSION_TIME = 0.5; //500 ms, this is generous as its typically 200ms max

// External function declaration for get_time_slot_offset
extern int get_time_slot_offset(void);

// External function declaration for get_first_slot_wall_time
extern double get_first_slot_wall_time(void);

// suboptimal deconfliction time to set a start time for scheduling the agents
// not actually max
static double calculate_max_deconf_time(int sectors_per_agent) {
    int max_time_slots = SECTOR_TIME_SLOTS;

    // Operations per sector-slot iteration (conservative estimate)
    const int ops_per_iteration = 10; // array accesses, comparisons, arithmetic

    // Time per operation (conservative estimate in nanoseconds)
    const double time_per_op_ns = 100.0;

    // Total operations: sectors × time_slots × ops_per_iteration
    double total_ops = sectors_per_agent * max_time_slots * ops_per_iteration;

    // Convert to seconds
    return total_ops * time_per_op_ns / 1e9;
}

//for deconfliction and sending out rover schedules
//TO SWITCH TO SEGMENT-BASED INSTRUCTIONS, YOU NEED TO MODIFY EVERYTHIN STEP 2 AND AFTER TO FIT YOUR IMPLEMENTATION
//YOU CANNOT USE THE SAME FUNCTIONS AND METHOD HERE AS SEGMENT-BASED REQUIRES SOME SPECIFIC WORK WITH STRUCTURES
//THAT POSE-BASED DOES NOT
//ALSO THIS CALLS PLATFORM-SPECIFIC FUNCTIONS

int create_and_send_rover_schedules(void) {
    int pending_count = deconf_pqueue_size(); 
    if (pending_count == 0) {
        return 0;
    }
    fprintf(stderr, "Creating Schedules for rovers\n");

    // Create local completed queue scoped to this function
    int actual_num_rovers = get_actual_num_rovers();
    if (actual_num_rovers == 0) actual_num_rovers = 10; // Fallback
    
    struct InstructionPackage** local_completed_pq = instrpack_queue_alloc(actual_num_rovers);
    int local_completed_count = 0;
    int local_completed_capacity = actual_num_rovers;

    //find a good start time on the schedule:
    // calculate suboptimal case deconfliction time per agent
    int bad_case_sectors_per_agent = SECTOR_TIME_SLOTS / 10;

    double max_deconf_time = calculate_max_deconf_time(bad_case_sectors_per_agent);

    double time_delay = (double)(pending_count * (MAX_TRANSMISSION_TIME + max_deconf_time));

    // Calculate elapsed time from first slot wall time
    struct timespec current_time;
    clock_gettime(CLOCK_REALTIME, &current_time);
    double current_wall_time = current_time.tv_sec + current_time.tv_nsec / 1e9;
    double first_slot_wall_time = get_first_slot_wall_time();
    double elapsed_since_first_slot = current_wall_time - first_slot_wall_time;
    
    // Target dispatch time is elapsed time + delay
    double target_dispatch_time = elapsed_since_first_slot + time_delay;
    
    // Convert to slots relative to the grid (which starts at 0)
    int start_time_slots = (int)(target_dispatch_time / TIME_SLOT_SIZE);

    fprintf(stderr, "debug log: send_main.c time calculated\n");
    // run deconfliction - pending → deconfliction → replan/completed
    //this adds stuff to completed when its complete
    //it reruns until its queue is empty
    //DO NOT LOOP THIS, if it fails to find a schedule, it needs to be allowed to fail
    //it retries an appropriate amount of times and sends the stuff back to replan from the start
    //it cannot be allowed to block the rest of the program
    deconf_main(deconf_pqueue_get_array(), local_completed_pq, &local_completed_count, start_time_slots);
    fprintf(stderr, "debug log: send_main.c deconfliction complete\n");
    // convert completed packages to full instructions (with wait hooks embedded)
    //currently this step is consolidated with the io step, because I'm using pose-based
    //and nav2 needs waits and instructions separate to construct the actual
    //platform specific, instead of simply 0 velocity like segment-based instructions

    // struct CompleteInstructionPackage *full_packages = malloc(sizeof(struct CompleteInstructionPackage) * MAX_COMPLETED_PACKAGES);
    // if (full_packages == NULL) {
    //     return -1; // Memory allocation failed
    // }
    // int completed_count = 0;
    // for (int i = 0; completed[i] != NULL && i < MAX_COMPLETED_PACKAGES; i++) {
    //     full_packages[i] = construct_full_instructions_generic(completed[i]);
    //     completed_count++;
    // }
    
    // platform-specific format
    int completed_count = local_completed_count;
    if (completed_count == 0) {
        fprintf(stderr, "debug log: send_main.c no completed packages to send\n");
        instrpack_queue_free(local_completed_pq);
        return 0;
    }

    struct PlatformSpecificInstructions **platform_packages = malloc(sizeof(struct PlatformSpecificInstructions*) * completed_count);
    if (platform_packages == NULL) {
        fprintf(stderr, "debug log: send_main.c failed to allocate memory for platform packages\n");
        instrpack_queue_free(local_completed_pq);
        return -1;
    }

    int actual_count = 0;
    struct InstructionPackage *pkg = instrpack_heap_pop(local_completed_pq, &local_completed_count);
    fprintf(stderr, "debug log: send_main.c converting completed packages to platform specific instructions\n");
    while (pkg != NULL && actual_count < completed_count) {
        fprintf(stderr, "debug log: send_main.c converting package %d\n", actual_count);
        platform_packages[actual_count] = convert_to_platform_specific(pkg);
        actual_count++;
        free_instruction_package(pkg);
        pkg = instrpack_heap_pop(local_completed_pq, &local_completed_count);
    }

    fprintf(stderr, "Sending instructions and schedules to rovers\n");
    // Send to platform
    int send_result = 0;
    for (int i = 0; i < actual_count; i++) {
        // Calculate actual wall time dispatch time
        // start_time_slots is relative to grid (which starts at first_slot_wall_time)
        double actual_dispatch_time = first_slot_wall_time + target_dispatch_time;

        int result = send_instructions(platform_packages[i], actual_dispatch_time);
        if (result != 0) {
            send_result = result; // Track if any send fails
        }
    }


    for (int i = 0; i < actual_count; i++) {
        if (platform_packages[i] != NULL) {
            freethepackage(platform_packages[i]);
        }
    }
    free(platform_packages);
    // Free the local completed queue
    instrpack_queue_free(local_completed_pq);

    fprintf(stderr, "Completed!\n");
    return send_result;
}
