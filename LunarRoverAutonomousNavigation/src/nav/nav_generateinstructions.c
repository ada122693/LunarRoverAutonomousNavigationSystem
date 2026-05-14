
//this is the interval over time implementation
//it is currently inactive, to switch back you need to follow the directions
//in the README in ## Instructions

//RE-ENABLE THIS TO USE, SEE README
//#include "nav/generate_instructions.h"

 //each instruction should be according to specific directions over intervals
 //a straight path should be one instruction with wait hooks
 //since marginal velocity is low and this system is assumed to be working
 //with agents that have a velocity controller
 //paths were calculated with a marginal velocity, so this is best
 //it could be changed though, with relatively little changes to the pathfinding code
 //but you would also have to change the instructions to add in accel/decel

 //current instructions are based on time, ex. "go forward for 5 seconds"

 //from constants:
 //DPOS_THRESH = 0.01m //don't consider under a cm for changes, still shows eventually when it accumulates
 //DHEADING_THRESH = 0.01rad //don't consider under a rad for changes, still shows eventually when it accumulates
 //they should be lower than the vehicle's tolerance

//RE-ENABLE THE FOLLOWING TO USE, SEE README

 /*
struct Instruction {
    double dheading;
    double velocity;
    double duration;
    struct Instruction *next;
}; 
struct WaitHookInfo {
    double beforeDur;
    double wait_duration;
    int instruction_index;
};

#include "structs.h"
#include <stdbool.h>
#include <stddef.h>

struct Instruction* generate_instructions(struct InstructionMetadata *metadata, struct VehicleState *start_node, int *sector_start_array, bool **has_safe_haven_out, struct WaitHookInfo **hooks_out, int sectors_traversal_count) {
    struct VehicleState *current = start_node;
    // "parent node" is the next node cause its already flipped
    struct VehicleState *next = current->parent_node;

    int currentSector = 1;
    int node = 0;
    int nextNode = sector_start_array[1];

    *has_safe_haven_out = malloc(sectors_traversal_count * sizeof(bool));
    *hooks_out = malloc(sectors_traversal_count * sizeof(struct WaitHookInfo));

    bool forward = false;
    bool prev_forward = false;
    bool safe_haven = false;

    double total_joules = 0.0;
    
    double accum_dtheta = 0.0;

    struct Instruction *curr_instruction = (struct Instruction *)malloc(sizeof(struct Instruction));
    if (curr_instruction == NULL) {return NULL;}
    curr_instruction->next = NULL;
    curr_instruction->dheading = 0.0;
    curr_instruction->velocity = 0.0;
    curr_instruction->duration = 0.0;
    struct Instruction *first_instruction = curr_instruction;
    int instruction_count = 0;

    while (next != NULL) {        
        double dheading = next->position.heading_rads - current->position.heading_rads;
        if (dheading > M_PI) {dheading -= 2.0*M_PI;}
        if (dheading < -M_PI) {dheading += 2.0*M_PI;}

        double dx = next->position.x_dim - current->position.x_dim;
        double dy = next->position.y_dim - current->position.y_dim;
        
        double project_movement = dx * cos(current->position.heading_rads) + dy * sin(current->position.heading_rads);
        if (project_movement > 0) {
            forward = true;
        } else {
            forward = false;
        }

        accum_dtheta += dheading;

        //info for wait hooks
        if (!safe_haven && current->safe_haven) {
            (*hooks_out)[currentSector - 1].instruction_index = instruction_count;
            (*hooks_out)[currentSector - 1].wait_duration = 0.0;
            safe_haven = true;
        }

        //new instruction for significant changes
        if (fabs(accum_dtheta) > DHEADING_THRESH || forward != prev_forward) {
            struct Instruction *next_instruction = (struct Instruction *)malloc(sizeof(struct Instruction));
            if (next_instruction == NULL) {return NULL;}
            curr_instruction->next = next_instruction;

            curr_instruction = next_instruction;

            curr_instruction->dheading = accum_dtheta;
            curr_instruction->velocity = (forward) ? MARGINAL_VELOCITY : -MARGINAL_VELOCITY;
            curr_instruction->duration = 0.0;

            accum_dtheta = 0.0;
            prev_forward = forward;
            instruction_count++;
        }

        curr_instruction->duration += next->travel_time;

        //keep track and update on new sectors
        if (node == nextNode && currentSector < sectors_traversal_count) {
            nextNode = sector_start_array[currentSector];
            (*has_safe_haven_out)[currentSector - 1] = safe_haven;
            safe_haven = false;
            currentSector++;
        }
        total_joules += current->f_cost;
        //"parent node" is the next node cause its already flipped
        current = current->parent_node;
        node++;
    }
    curr_instruction->next = NULL;

    metadata->joule_cost = total_joules;
    metadata->count = instruction_count;

    //get rid of the dummy node
    struct Instruction *final = first_instruction->next;
    free(first_instruction);
    return final;
}
*/