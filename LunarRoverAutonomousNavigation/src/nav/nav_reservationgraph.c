#include <stdbool.h>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>

#include "constants/constants.h"
#include "constants/config.h"
#include "structs.h"


struct SectorStartNodes {
    int count;
    struct VehicleState* node;
    struct SectorStartNodes* prev;
};


//ensure instructions contain the id from here in a lookupable way


//sector height = some multiple of x, sector width = some multiple of y

struct SectorTraversal * generate_reservationgraph(struct VehicleState *startNode, struct VehicleState *endNode, struct InstructionMetadata *metadata, int ** sector_start_array) {
    *sector_start_array = NULL;
    double start_pos_x = startNode->position.x_dim;
    double start_pos_y = startNode->position.y_dim;

    double end_pos_x = endNode->position.x_dim;
    double end_pos_y = endNode->position.y_dim;

    double total_x_sectors = (end_pos_x - start_pos_x) / SECTOR_WIDTH;
    double total_y_sectors = (end_pos_y - start_pos_y) / SECTOR_HEIGHT;

    int sum_sectors = 0;
    int sector_count = 0;
    double sum_time_diff = 0;
    
    struct VehicleState *current = startNode;

    struct SectorStartNodes *sector_start_nodes = malloc(sizeof(struct SectorStartNodes));    
    if (sector_start_nodes == NULL) {
        return NULL;
    }
    sector_start_nodes->node = startNode;
    sector_start_nodes->count = 0;
    sector_start_nodes->prev = NULL;

    //for sectors_traversal
    int *sectors = malloc(metadata->count * sizeof(int));
    if (sectors == NULL) {
        free(sector_start_nodes);
        return NULL;
    }
    int *num_reservations = malloc(metadata->count * sizeof(int));
    if (num_reservations == NULL) {
        free(sectors);
        free(sector_start_nodes);
        return NULL;
    }
    bool *sharesTail = malloc(metadata->count * sizeof(bool));
    if (sharesTail == NULL) {
        free(num_reservations);
        free(sectors);
        free(sector_start_nodes);
        return NULL;
    }

    int currPathNode = 0;
    int prev_sector = -1;
    int num_sectors_y = (int)(Y_DIM_SIZE / SECTOR_HEIGHT);
    int safety = 100000;
    while (current != NULL && safety > 0) {
        safety--;
        int sector_x = (int)((current->position.x_dim + X_DIM_SIZE/2.0) / SECTOR_WIDTH);
        int sector_y = (int)((current->position.y_dim + Y_DIM_SIZE/2.0) / SECTOR_HEIGHT);
        int sector = sector_x * num_sectors_y + sector_y;
        if (sector != prev_sector) {

            struct SectorStartNodes *new_node = malloc(sizeof(struct SectorStartNodes));
            if (new_node == NULL) {
                // Clean up previously allocated memory
                struct SectorStartNodes *curr = sector_start_nodes;
                while (curr != NULL) {
                    struct SectorStartNodes *to_free = curr;
                    curr = curr->prev;
                    free(to_free);
                }
                free(sharesTail);
                free(num_reservations);
                free(sectors);
                return NULL;
            }

            new_node->node = current;
            new_node->count = currPathNode;
            new_node->prev = sector_start_nodes;

            sector_start_nodes = new_node;

            sector_count++;
            
            //this is to account for precision, a time slot is certainly larger than the time of
            //a single step
            num_reservations[sector_count - 1] = (int)((sum_time_diff + TIME_SLOT_SIZE - 1) / TIME_SLOT_SIZE);
            double remainder = fmod(sum_time_diff, TIME_SLOT_SIZE);
            sharesTail[sector_count - 1] = (remainder > 1e-9); // true unless remainder is near zero 
            sectors[sector_count - 1] = sector;

            sum_sectors++;
            sum_time_diff = 0;
        }
        sum_time_diff += current->travel_time;
        prev_sector = sector;

        //"parent node" is the next node cause its already flipped
        current = current->parent_node;
        currPathNode++;
    }
    if (safety <= 0) {
        fprintf(stderr, "ERROR generate_reservationgraph: malformed path, safety limit reached\n");
        free(sharesTail);
        free(num_reservations);
        free(sectors);
        return NULL;
    }

    *sector_start_array = malloc(sector_count * sizeof(int));
    if (*sector_start_array == NULL) {
        // Clean up previously allocated memory
        struct SectorStartNodes *curr = sector_start_nodes;
        while (curr != NULL) {
            struct SectorStartNodes *to_free = curr;
            curr = curr->prev;
            free(to_free);
        }
        free(sharesTail);
        free(num_reservations);
        free(sectors);
        return NULL;
    }
    struct SectorStartNodes *curr = sector_start_nodes;

    for (int i = sector_count - 1; i >= 0 && curr != NULL; i--) {
        (*sector_start_array)[i] = curr->count;
        struct SectorStartNodes *to_free = curr;
        curr = curr->prev;
        free(to_free);
    }

    //construct sectors_traversal
    if (sector_count == 0) {
        free(sectors);
        free(num_reservations);
        free(sharesTail);
        sectors = NULL;
        num_reservations = NULL;
        sharesTail = NULL;
    } else {
        sectors = realloc(sectors, sector_count * sizeof(int));
        num_reservations = realloc(num_reservations, sector_count * sizeof(int));
        sharesTail = realloc(sharesTail, sector_count * sizeof(bool));
    }
    struct SectorTraversal* sectors_traversal = malloc(sizeof(struct SectorTraversal));
    if (sectors_traversal == NULL) {
        free(*sector_start_array);
        *sector_start_array = NULL;
        free(sectors);
        free(num_reservations);
        free(sharesTail);
        return NULL;
    }
    sectors_traversal->sectors = sectors;
    sectors_traversal->num_reservations = num_reservations;
    sectors_traversal->sharesTail = sharesTail;
    sectors_traversal->count = sector_count;

    return sectors_traversal;
}
