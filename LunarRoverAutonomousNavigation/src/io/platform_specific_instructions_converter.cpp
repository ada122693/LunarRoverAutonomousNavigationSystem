//this converts POSE-BASED instructions to platform-specific instructions
//for my current testing implementation
//before sending, another program will modify the timestamp in the first instruction so
//the initial wait synchronizes to the actual schedule

#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "robot_interfaces/action/path_schedule_plan.hpp"

#include "structs.h"
#include "constants/constants.h"
#include "constants/config.h"

#include "io/platform_specific_instructions_converter.h"

PlatformSpecificInstructions* convert_to_platform_specific(struct InstructionPackage *package) {
    auto curr_instr = package->instructions;

    int instruction_index = 0;

    PlatformSpecificInstructions* result = new PlatformSpecificInstructions();

    // Set rover_id in the outer struct
    result->rover_id = package->metadata.rover_id;

    // add initial wait
    //initial wait * time slot size
    double initial_wait_in_sec = (double)package->first_wait * TIME_SLOT_SIZE;
    result->goal.wait_durations.push_back(initial_wait_in_sec);

    // Create first segment
    nav_msgs::msg::Path segment;
    segment.header.frame_id = "map";
    // segment.header.stamp will be set by the calling program

    int sector_index = 0;
    int next_wait_instruction_index = -1;

    // Find first valid and used hook to get initial next_wait_instruction_index
    while (sector_index < package->sectors_traversal->count &&
           package->hooks[sector_index].wait_duration <= 0.0) {
        sector_index++;
    }

    // if the index == count, there's no hooks
    if (sector_index < package->sectors_traversal->count) {
        next_wait_instruction_index = package->hooks[sector_index].instruction_index;
    }

    while (curr_instr != nullptr) {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = segment.header;

        pose_stamped.pose.position.x = curr_instr->pose_position.x;
        pose_stamped.pose.position.y = curr_instr->pose_position.y;
        pose_stamped.pose.position.z = curr_instr->pose_position.z;
        pose_stamped.pose.orientation.x = curr_instr->pose_orientation.x;
        pose_stamped.pose.orientation.y = curr_instr->pose_orientation.y;
        pose_stamped.pose.orientation.z = curr_instr->pose_orientation.z;
        pose_stamped.pose.orientation.w = curr_instr->pose_orientation.w;

        // Check if current instruction has a wait
        if (instruction_index == next_wait_instruction_index) {
            double wait_duration = package->hooks[sector_index].wait_duration;

            // Move to next sector with a hook
            //this is the same time complexity as checking the address of the next sector
            //not any difference in anything, just only returns valid AND used waits
            //and checking for instruction index of it
            //instead of returning valid wait hooks and then checking for instruction index of it
            //and if its used
            sector_index++;
            while (sector_index < package->sectors_traversal->count && package->hooks[sector_index].wait_duration <= 0.0) {
                sector_index++;
            }

            //if the sector index reaches the amount of sectors, there isn't another wait
            if (sector_index < package->sectors_traversal->count) {
                next_wait_instruction_index = package->hooks[sector_index].instruction_index;
            } else {
                // set it to -1 so none of this triggers
                next_wait_instruction_index = -1;
            }

            if (wait_duration > 0.0) {
                // Has wait - split segment
                segment.poses.push_back(pose_stamped);
                result->goal.segments.push_back(segment);
                result->goal.wait_durations.push_back(wait_duration);
                segment.poses.clear();
                continue;
            }
        }

        // Add current pose to current segment
        segment.poses.push_back(pose_stamped);

        curr_instr = curr_instr->next;
        instruction_index++;
    }

    // Add the final segment
    if (!segment.poses.empty()) {
        result->goal.segments.push_back(segment);
    }

    return result;
}

void freethepackage (PlatformSpecificInstructions* instance) {
    delete instance;
}
