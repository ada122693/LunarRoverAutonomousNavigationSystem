//a replacement for generateinstructions which functions through a system of path nodes
//this is intended for systems like the yahboom microros 
//
//its output is very similar to the regular path, except it creates wait hooks at intervals where its safe to wait
//this uses a DIFFERENT VERSION OF Instruction
//so depending on if you use this or the other, change appropriately as well as change calls
//to re-enable segment-based, see the readme

#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "structs.h"

#include "nav/generate_instructions_global.h"

 struct Instruction* generate_instructions_global(struct InstructionMetadata *metadata, struct VehicleState *start_node, int *sector_start_array, bool **has_safe_haven_out, struct WaitHookInfo **hooks_out, int sectors_traversal_count) {

    *has_safe_haven_out = NULL;
    *hooks_out = NULL;

    if (sectors_traversal_count <= 0) {
        fprintf(stderr, "generate_instructions_global: sectors_traversal_count <= 0, returning NULL\n");
        return NULL;
    }

    struct VehicleState *current = start_node;
    // "parent node" is the next node cause its already flipped
    struct VehicleState *next = current->parent_node;

    int currentSector = 1;
    int node = 0;
    int nextNode = (sectors_traversal_count > 1) ? sector_start_array[1] : 0;

    *has_safe_haven_out = malloc(sectors_traversal_count * sizeof(bool));
    if (*has_safe_haven_out == NULL) {
        fprintf(stderr, "generate_instructions_global: malloc failed for has_safe_haven_out\n");
        return NULL;
    }
    *hooks_out = malloc(sectors_traversal_count * sizeof(struct WaitHookInfo));
    if (*hooks_out == NULL) {
        fprintf(stderr, "generate_instructions_global: malloc failed for hooks_out\n");
        free(*has_safe_haven_out);
        *has_safe_haven_out = NULL;
        return NULL;
    }

    // Initialize hooks with -1 for sectors without safe havens
    for (int i = 0; i < sectors_traversal_count; i++) {
        (*hooks_out)[i].instruction_index = -1;
        (*hooks_out)[i].wait_duration = -1.0;
    }

    bool safe_haven = false;

    struct Instruction *first_instruction = NULL;
    struct Instruction *curr_instruction = NULL;
    int instruction_count = 0;

    int safety = 0;
    while (next != NULL && safety++ < 100000) {

        // Allocate new instruction for this node
        struct Instruction *new_instruction = (struct Instruction *)malloc(sizeof(struct Instruction));
        if (new_instruction == NULL) {
            fprintf(stderr, "generate_instructions_global: malloc failed for new_instruction\n");
            // Free all allocated instructions
            while (first_instruction != NULL) {
                struct Instruction *to_free = first_instruction;
                first_instruction = first_instruction->next;
                free(to_free);
            }
            free(*hooks_out);
            *hooks_out = NULL;
            free(*has_safe_haven_out);
            *has_safe_haven_out = NULL;
            return NULL;
        }
        new_instruction->next = NULL;
        new_instruction->pose_position = (struct Pose){0.0, 0.0, 0.0};
        new_instruction->pose_orientation = (struct Quaternion){0.0, 0.0, 0.0, 1.0};

        // Link to list
        if (first_instruction == NULL) {
            first_instruction = new_instruction;
        } else {
            curr_instruction->next = new_instruction;
        }
        curr_instruction = new_instruction;

        //runs once per sector as safe_haven is set to false when a new sector is reached
        if (!safe_haven && current->safe_haven) {
            if (currentSector - 1 >= sectors_traversal_count) {
                fprintf(stderr, "generate_instructions_global: ERROR currentSector-1=%d >= sectors_traversal_count=%d\n", currentSector - 1, sectors_traversal_count);
            }
            (*hooks_out)[currentSector - 1].instruction_index = instruction_count;
            (*hooks_out)[currentSector - 1].wait_duration = 0.0;
            safe_haven = true;
        }

        //set the pose data
        curr_instruction->pose_position.x = current->position.x_dim;
        curr_instruction->pose_position.y = current->position.y_dim;
        curr_instruction->pose_position.z = 0.0; // irrelevant unless wheeled rovers can now fly
        curr_instruction->pose_orientation.x = 0.0; // irrelevant
        curr_instruction->pose_orientation.y = 0.0;
        curr_instruction->pose_orientation.z = sin(current->position.heading_rads / 2.0); // sin(yaw/2)
        curr_instruction->pose_orientation.w = cos(current->position.heading_rads / 2.0); // cos(yaw/2)

        //keep track and update on new sectors
        //node == nextNode checks for when the node == the node where the next sector starts
        //not the next existing node.
        if (node == nextNode && currentSector < sectors_traversal_count) {
            if (currentSector - 1 >= sectors_traversal_count) {
                fprintf(stderr, "generate_instructions_global: ERROR currentSector-1=%d >= sectors_traversal_count=%d\n", currentSector - 1, sectors_traversal_count);
            }
            (*has_safe_haven_out)[currentSector - 1] = safe_haven;
            safe_haven = false;
            currentSector++;
            if (currentSector < sectors_traversal_count) {
                nextNode = sector_start_array[currentSector];
            }
        }

        node++;
        current = next;
        next = current->parent_node;
        instruction_count++;
    }
    if (safety >= 100000) {
        fprintf(stderr, "ERROR generate_instructions_global: malformed path, safety limit reached\n");
        return NULL;
    }
    
    metadata->joule_cost = current->accumulated_path_cost;
    metadata->count = instruction_count;

    return first_instruction;
    
}
