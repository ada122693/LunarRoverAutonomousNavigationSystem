#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "structs.h"
#include "constants/constants.h"
#include "constants/arrsize_global.h"
#include "config.h"

#include "objectives/power_station_kdtree.h"
#include "objectives/objective_kdtree.h"
#include "objectives/objectives_logic.h"

void add_power_station(int id, struct Location location);
void add_objective(int id, int priority, struct Location location);

// External variables (defined in objectives_logic.c)
extern struct Objective objectives[MAX_OBJECTIVES];
extern struct PowerStation power_stations[MAX_OBJECTIVES];
extern int power_station_count;
extern int total_objective_count;
extern int objective_counts[MAX_OBJECTIVE_PRIORITY_LEVELS];

// Initialize objectives system
static void init_objectives_system(void) {
    // Initialize power station KD-tree
    power_station_kdtree_init();
}

void init_objectives_from_file(const char* filename) {
    fprintf(stderr, "Loading objectives from file: %s\n", filename);
    FILE* file = fopen(filename, "r");
    if (file == NULL) {
        fprintf(stderr, "Failed to open objectives file: %s\n", filename);
        return;
    }
    
    // Initialize system if not already done
    if (total_objective_count == 0) {
        init_objectives_system();
    }
    
    char line[200];
    
    while (fgets(line, sizeof(line), file) != NULL && total_objective_count < MAX_OBJECTIVES) {
        // Remove newline
        line[strcspn(line, "\n")] = '\0';
        
        if (strlen(line) == 0 || line[0] == '#') {
            continue; // Skip empty lines and comments
        }
        
        // Parse format: id,priority,x,y,is_power_station
        // description not parsed but the last string ,description
        char* token = strtok(line, ",");
        if (token != NULL) {
            int id = atoi(token);
            // Validate id is non-negative and reasonable
            if (id < 0 || id >= 1000) continue;

            token = strtok(NULL, ",");
            if (token != NULL) {
                int priority = atoi(token);
                // Validate priority is within valid range
                if (priority < 0 || priority >= MAX_OBJECTIVE_PRIORITY_LEVELS) continue;

                token = strtok(NULL, ",");
                if (token != NULL) {
                    double x = atof(token);
                    // Validate x coordinate is within world dimensions
                    if (x < -X_DIM_SIZE + PRIM_LENGTH || x > X_DIM_SIZE + PRIM_LENGTH) continue;

                    token = strtok(NULL, ",");
                    if (token != NULL) {
                        double y = atof(token);
                        // Validate y coordinate is within world dimensions
                        if (y < -Y_DIM_SIZE + PRIM_LENGTH || y > Y_DIM_SIZE + PRIM_LENGTH) continue;

                        token = strtok(NULL, ",");
                        if (token != NULL) {
                            int ps_val = atoi(token);
                            // Validate is_power_station is 0 or 1
                            bool is_power_station = (ps_val == 1);

                            token = strtok(NULL, ",");
                            double heading = (token != NULL) ? atof(token) : 0.0;

                            // Create location
                            struct Location loc = {x, y, heading};

                            if (is_power_station) {
                                add_power_station(id, loc);
                            } else {
                                add_objective(id, priority, loc);
                            }
                        }
                    }
                }
            }
        }
    }
    
    // Build the KD-trees for efficient nearest neighbor searches
    power_station_kdtree_build();
    objective_kdtree_build(NULL);
    
    fclose(file);
    fprintf(stderr, "Loaded %d objectives and %d power stations\n", total_objective_count, power_station_count);
}

void add_objective(int id, int priority, struct Location location) {
    if (total_objective_count >= MAX_OBJECTIVES) {
        return; // Full
    }

    // Clamp priority to valid range
    if (priority < 0) priority = 0;
    if (priority >= MAX_OBJECTIVE_PRIORITY_LEVELS) priority = MAX_OBJECTIVE_PRIORITY_LEVELS - 1;

    // Initialize system if not already done
    if (total_objective_count == 0) {
        init_objectives_system();
    }

    objectives[total_objective_count].id = id;
    objectives[total_objective_count].priority = priority;
    objectives[total_objective_count].location = location;

    // Add to appropriate KD-tree based on priority
    objective_kdtree_add(NULL, &objectives[total_objective_count]);
    objective_counts[priority]++;
    total_objective_count++;
}

void add_power_station(int id, struct Location location) {
    if (power_station_count >= MAX_OBJECTIVES) {
        return; // Full
    }

    // Initialize system if not already done
    if (total_objective_count == 0) {
        init_objectives_system();
    }

    power_stations[power_station_count].id = id;
    power_stations[power_station_count].location = location;

    // Add to KD-tree
    power_station_kdtree_add(&power_stations[power_station_count]);
    power_station_count++;
}

static struct Objective* get_objectives(void) {
    return objectives;
}

int get_objective_count(void) {
    return total_objective_count;
}

static bool is_power_station_location(struct Location loc) {
    // Simple linear search since we removed hashmap
    // Could be optimized with KD-tree if needed
    for (int i = 0; i < power_station_count; i++) {
        if (power_stations[i].location.x_dim == loc.x_dim && 
            power_stations[i].location.y_dim == loc.y_dim) {
            return true;
        }
    }
    return false;
}
