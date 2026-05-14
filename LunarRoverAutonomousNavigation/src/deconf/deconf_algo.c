//safe_wait:
//safe_wait_array[instruction_index] = num_time_slots
//safe_wait_pointer_array[] = safe_wait_pointer (in reverse order)
//
//


#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "constants/constants.h"
#include "constants/config.h"
#include "arrsize_global.h"
#include "structs.h"
#include "status_codes.h"

#include "deconf_pqueue.h"
#include "deconf/deconf_algo.h"
#include "resgrid.h"

struct safe_wait_cell {
    int prev_safe_wait_index;
    //DOES NOT INCLUDE START TIME OFFSET, IS RELATIVE NOT ABSOLUTE
    int cumulative_time_slots;
    bool is_safe;
};

static void publish_reservations (struct safe_wait_cell* safe_wait, struct InstructionPackage *route, int start_time) {
    int prev_slots = start_time;
    for (int i = 0; i < route->sectors_traversal->count; i++) {
        int sector = route->sectors_traversal->sectors[i];
        int slots = safe_wait[i].cumulative_time_slots;

        int waits_used = safe_wait[i].cumulative_time_slots - route->sectors_traversal->num_reservations[i];
        
        if (route->hooks[i].instruction_index >= 0) {
            // Modify existing wait hook for this instruction
            route->hooks[i].wait_duration += waits_used * TIME_SLOT_SIZE; // Convert slots to seconds
            //keep track
            route->metadata.num_waits++;
        }

        // Mark graph slots as reserved
        for (int j = 0; j <= slots; j++) {
            g_reservation_grid[sector][prev_slots + j] = true;
        }

        // Update intervals array
        // Find interval start by scanning backwards from first reserved slot
        int first_reserved = prev_slots;
        int interval_start = first_reserved;
        while (interval_start > 0 && g_intervals[sector][interval_start - 1] != MAX_TIME_SLOTS) {
            interval_start -= g_intervals[sector][interval_start - 1];
        }

        // Find interval end by scanning forwards from last reserved slot
        int last_reserved = prev_slots + slots;
        int interval_end = last_reserved;
        while (interval_end < MAX_TIME_SLOTS && g_intervals[sector][interval_end] != MAX_TIME_SLOTS) {
            interval_end += g_intervals[sector][interval_end];
        }
        
        if (interval_start < first_reserved) {
            g_intervals[sector][interval_start] = first_reserved - interval_start;
        }
        g_intervals[sector][first_reserved] = last_reserved - first_reserved + 1;
        if (last_reserved + 1 < interval_end) {
            g_intervals[sector][last_reserved + 1] = interval_end - (last_reserved + 1);
        }

        prev_slots += slots;
    }
}

static int find_next_free_interval(int sector, int start_time) {
    //find the next free interval
    int sum = 0;
    if (g_reservation_grid[sector][start_time] && sum < MAX_TIME_SLOTS) {
        //if busy, its the next
        sum = g_intervals[sector][start_time];
        if (sum >= MAX_TIME_SLOTS) {
            return -1;
        }
        return sum;
    } else {
        //if free, next after next
        int interval = g_intervals[sector][start_time];
        sum += interval;
        if (sum + interval >= MAX_TIME_SLOTS) {
            return -1;
        }
        interval = g_intervals[sector][start_time + interval];
        sum += interval;
        if (sum >= MAX_TIME_SLOTS) {
            return -1;
        }
        return sum;
    }
    
    return -1;
}

 //looks at how much time must be scheduled to traverse a sector (would typically be 1 if constants are set)
//and sees if it can schedule it, if not it checks to see if the amount of time that spot is busy can be waited for
//in the closest to that point sector with slack
bool deconf_standard(struct InstructionPackage *route, int *time_till_potential_valid, int cumulative_initial_wait, int actual_start_time) {
    fprintf(stderr, "debug log: deconf_standard rover_id=%d, start_time=%d, cumulative_initial_wait=%d\n",
            route->metadata.rover_id, actual_start_time, cumulative_initial_wait);

    //max free slots at each sector the instructions traverse
    struct safe_wait_cell safe_wait[route->sectors_traversal->count + 1];
    safe_wait[0].prev_safe_wait_index = -1;
    safe_wait[0].cumulative_time_slots = 0;
    safe_wait[0].is_safe = false;

    int start_pos = route->sectors_traversal->sectors[0];
    int start_time = actual_start_time;

    fprintf(stderr, "debug log: deconf_standard start_pos=%d, g_reservation_grid[%d][%d]=%d\n",
            start_pos, start_pos, start_time, g_reservation_grid[start_pos][start_time]);

    if (g_reservation_grid[start_pos][start_time]) {
        *time_till_potential_valid = find_next_free_interval(route->sectors_traversal->sectors[0], start_time);
        fprintf(stderr, "debug log: deconf_standard start position already reserved, time_till_potential_valid=%d\n",
                *time_till_potential_valid);
        return false;
    }

    for (int position = 0, iter_gate = 0; position < route->sectors_traversal->count && iter_gate < MAX_ITER_DECONFLICT; position++, iter_gate++) {
        int sector = route->sectors_traversal->sectors[position];
        int num_reservations = route->sectors_traversal->num_reservations[position];

        int cumulative_time = safe_wait[position].cumulative_time_slots;
        //this can't happen on the first cell so it won't result in -1
        if (route->sectors_traversal->sharesTail[position]) {
            cumulative_time--;
        }
        if (cumulative_time < 0) {
            cumulative_time = 0;
        }

        int last_prev_safe_wait_index = safe_wait[position].prev_safe_wait_index;

        int slots_at_status = g_intervals[sector][cumulative_time + start_time];

        fprintf(stderr, "debug log: deconf_standard position=%d, sector=%d, cumulative_time=%d, num_reservations=%d, slots_at_status=%d\n",
                position, sector, cumulative_time, num_reservations, slots_at_status);

        bool free;

        //interval is free for enough time, reserve it
        if (!g_reservation_grid[sector][cumulative_time + start_time]) {
            if (slots_at_status >= num_reservations) {
                fprintf(stderr, "debug log: deconf_standard slot free, reserving directly\n");
                safe_wait[position + 1].prev_safe_wait_index = (safe_wait[position].is_safe) ? position : last_prev_safe_wait_index;
                safe_wait[position + 1].cumulative_time_slots = cumulative_time + num_reservations;
                safe_wait[position + 1].is_safe = route->has_safe_haven[sector];
                continue;
            } else {
                fprintf(stderr, "debug log: deconf_standard slot free but not enough slots, need to wait\n");
                free = true;
            }
        } else {
            fprintf(stderr, "debug log: deconf_standard slot already reserved, need to wait\n");
            free = false;
        }
        
        //interval is not, find a place to wait closest to current step and get rid of everything after

        //finding how much time to wait
        int time_to_wait;
        for (time_to_wait = cumulative_time + slots_at_status; time_to_wait + start_time < MAX_TIME_SLOTS; free = !free) {
            if (free) {
                int free_interval = g_intervals[sector][time_to_wait + start_time] - time_to_wait;
                time_to_wait += free_interval;
                if (free_interval >= num_reservations) {
                    break;
                }
            } else {
                time_to_wait = g_intervals[sector][time_to_wait + start_time];
            }
        }

        bool backtrack_found = false;

        fprintf(stderr, "debug log: deconf_standard attempting backtrack, last_prev_safe_wait_index=%d\n",
                last_prev_safe_wait_index);

        while (last_prev_safe_wait_index > 0) {
            int cumulative_time = safe_wait[last_prev_safe_wait_index].cumulative_time_slots;
            int slots_after = g_intervals[route->sectors_traversal->sectors[last_prev_safe_wait_index - 1]][cumulative_time] - (cumulative_time + time_to_wait);
            if (slots_after >= 0) {
                fprintf(stderr, "debug log: deconf_standard backtrack found at index=%d, slots_after=%d\n",
                        last_prev_safe_wait_index, slots_after);
                safe_wait[last_prev_safe_wait_index].cumulative_time_slots = cumulative_time + time_to_wait;
                safe_wait[last_prev_safe_wait_index].is_safe = slots_after > 0;

                //change position of loop to found valid backtrack location, start operating at +1
                //since safe_wait is indexed starting at 1, and need to start next iteration at
                position = last_prev_safe_wait_index - 1;
                backtrack_found = true;
                break;
            }
            last_prev_safe_wait_index = safe_wait[last_prev_safe_wait_index].prev_safe_wait_index;
        }

        if (!backtrack_found) {
            //no acceptable time
            //very very unlikely to reach here, unless nearly the entire grid is occupied
            //find the interval at which the rover would have to wait for it to count as a different insertion
            fprintf(stderr, "debug log: deconf_standard backtrack failed, grid likely full\n");
            *time_till_potential_valid = find_next_free_interval(route->sectors_traversal->sectors[0], start_time);
            return false;
        }
    }


    // Check if total wait slots consumed (including cumulative initial wait) exceeds battery allowance
    if (safe_wait[route->sectors_traversal->count].cumulative_time_slots + cumulative_initial_wait > route->metadata.wait_slot_allowance) {
        fprintf(stderr, "debug log: deconf_standard battery check failed, cumulative=%d, allowance=%d\n",
                safe_wait[route->sectors_traversal->count].cumulative_time_slots + cumulative_initial_wait,
                route->metadata.wait_slot_allowance);
        route->metadata.route_error = INSUFFICIENT_BATTERY;
        route->metadata.valid = false;
        return false;
    }
    

    fprintf(stderr, "debug log: deconf_standard success, publishing reservations\n");
    publish_reservations (safe_wait, route, start_time);

    return true;
}
