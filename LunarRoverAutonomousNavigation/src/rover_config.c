#include "rover_config.h"
#include "config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

// Static variable to store the actual number of rovers from config
static int g_actual_num_rovers = 0;

int load_rover_config(const char* filename, struct RoverConfig* configs, int* num_rovers) {
    FILE* file = fopen(filename, "r");
    if (!file) {
        fprintf(stderr, "Failed to open rover config file: %s\n", filename);
        return -1;
    }

    char line[256];
    int current_id = -1;
    double current_x = 0.0;
    double current_y = 0.0;
    *num_rovers = 0;

    while (fgets(line, sizeof(line), file) && *num_rovers < MAX_ROVERS) {
        // Remove trailing newline
        line[strcspn(line, "\n")] = '\0';
        
        // Skip comments and empty lines
        if (line[0] == '#' || line[0] == '\0') {
            continue;
        }

        // Trim whitespace
        char* start = line;
        while (isspace(*start)) start++;
        if (*start == '\0') continue;

        // Parse rover_id
        if (strncmp(start, "rover_id", 8) == 0) {
            char* eq = strchr(start, '=');
            if (eq) {
                int parsed_id = atoi(eq + 1);
                // Validate rover_id is non-negative and reasonable
                if (parsed_id >= 0 && parsed_id < 1000) {
                    current_id = parsed_id;
                }
            }
        }
        // Parse rover_start_coords
        else if (strncmp(start, "rover_start_coords", 18) == 0) {
            char* paren = strchr(start, '(');
            if (paren) {
                double parsed_x, parsed_y;
                if (sscanf(paren, "(%lf,%lf)", &parsed_x, &parsed_y) == 2) {
                    // Validate coordinates are within world dimensions (0 to X_DIM_SIZE, 0 to Y_DIM_SIZE)
                    // Allow small margin for floating point errors
                    if (parsed_x >= -10.0 && parsed_x <= X_DIM_SIZE + 10.0 &&
                        parsed_y >= -10.0 && parsed_y <= Y_DIM_SIZE + 10.0) {
                        current_x = parsed_x;
                        current_y = parsed_y;
                    }
                }

                // Save the rover config
                if (current_id != -1) {
                    configs[*num_rovers].rover_id = current_id;
                    configs[*num_rovers].start_x = current_x;
                    configs[*num_rovers].start_y = current_y;
                    (*num_rovers)++;

                    // Reset for next rover
                    current_id = -1;
                    current_x = 0.0;
                    current_y = 0.0;
                }
            }
        }
    }

    fclose(file);
    
    // Store the actual number of rovers for later retrieval
    g_actual_num_rovers = *num_rovers;
    
    return 0;
}

struct RoverConfig* get_rover_config(int rover_id, struct RoverConfig* configs, int num_rovers) {
    for (int i = 0; i < num_rovers; i++) {
        if (configs[i].rover_id == rover_id) {
            return &configs[i];
        }
    }
    return NULL;
}

int get_actual_num_rovers(void) {
    return g_actual_num_rovers;
}
