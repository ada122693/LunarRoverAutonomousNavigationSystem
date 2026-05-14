#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>

#include "structs.h"
#include "nav/generate_instructions_global.h"
#include "nav/nav_reservationgraph.h"

 //pose-based instructions (currently active)
//segment-based instructions (currently disabled), see README ## Instructions section


struct VehicleState* reverse_to_path(struct VehicleState* foundPathNode, struct InstructionMetadata* metadata) {
    struct VehicleState *next = NULL;
    struct VehicleState *curr = foundPathNode;
    struct VehicleState *prev = NULL;
    double count = 0;
    int safety = 0;

    while (curr != NULL && safety++ < 100000) {
        count++;

        prev = curr->parent_node;

        curr->parent_node = next;

        next = curr;
        curr = prev;
    }

    if (safety >= 100000) {
        fprintf(stderr, "ERROR reverse_to_path: Cycle detected in path, safety limit reached\n");
        //this issue WAS SOLVED but dealing with a node pool + all that no need to risk it
        return NULL;
    }

    metadata->count = count;
    metadata->total_time = foundPathNode->total_time_cost;

    return next;
}

struct InstructionPackage* nav_main(struct VehicleState *foundPathNode, struct Location start, struct Location end) {
    struct InstructionPackage* package = malloc(sizeof(struct InstructionPackage));
    if (package == NULL) {
        return NULL;
    }

    //take tail node
    //call function that populates a list by backtracking through parents
    package->metadata = (struct InstructionMetadata){0, 0};
    package->metadata.start_location = start;
    package->metadata.end_location = end;
    package->metadata.num_waits = 0;
    package->first_wait = 0; // Initialize first wait to 0
    package->metadata.rover_id = -1;  // Unassigned rover
    package->metadata.req_flag = -1;  // placeholder - overwritten by caller (request_handler.c)
    package->metadata.route_status = -1;  // placeholder - overwritten by caller (request_handler.c)
    package->metadata.route_error = 0;  // No error (0 is not a valid error code, errors are negative)
    package->metadata.battery_level = -1;  // Unknown battery level
    package->metadata.timestamp = -1;  // Unknown timestamp
    struct VehicleState *orderedPath = reverse_to_path(foundPathNode, &package->metadata);
    if (orderedPath == NULL) {
        fprintf(stderr, "ERROR nav_main: reverse_to_path failed (cycle detected)\n");
        free(package);
        return NULL;
    }
    fprintf(stderr, "nav_main: reverse_to_path succeeded\n");

    int *sector_start_array = NULL;
    //turn into sector-time graph for deconf
    struct SectorTraversal* traversal = generate_reservationgraph(orderedPath, foundPathNode, &package->metadata, &sector_start_array);
    if (traversal == NULL) {
        // Clean up
        if (sector_start_array != NULL) {
            free(sector_start_array);
        }
        // Note: generate_reservationgraph handles cleanup of its internal allocations
        // Note: orderedPath is pool-allocated, do not free() it
        free(package);
        return NULL;
    }
    package->sectors_traversal = traversal;
    package->metadata.path_length_in_sectors = package->sectors_traversal->count;

    //This is the segment based instructions that are not currently in-use. to switch from pose instructions, see
    //the readme ## Instructions section to see how to switch
    /*
    //generic instruction struct
    package->instructions = generate_instruction(&package->metadata, orderedPath, sector_start_array, &package->has_safe_haven, &package->hooks);
    */
    //pose-based instruction struct
    package->instructions = generate_instructions_global(&package->metadata, orderedPath, sector_start_array, &package->has_safe_haven, &package->hooks, package->sectors_traversal->count);
    fprintf(stderr, "[nav_main]instruction count = %d\n", package->metadata.count);
    if (package->instructions == NULL) {
    	fprintf(stderr, "[nav_main] null instructions, cleaning up\n");
        // Clean up
        free(sector_start_array);
        if (package->has_safe_haven) free(package->has_safe_haven);
        if (package->hooks) free(package->hooks);
        if (package->sectors_traversal) {
            if (package->sectors_traversal->sectors) free(package->sectors_traversal->sectors);
            if (package->sectors_traversal->num_reservations) free(package->sectors_traversal->num_reservations);
            if (package->sectors_traversal->sharesTail) free(package->sectors_traversal->sharesTail);
            free(package->sectors_traversal);
        }
        // Note: orderedPath is pool-allocated, do not free() it
        free(package);
        return NULL;
    }

    free(sector_start_array);


    // Note: orderedPath is pool-allocated via proavo_createpath, do not free() it
    // The pool will be reset on the next proavo_createpath call

    return package;
}
