#ifndef ROVER_CONFIG_H
#define ROVER_CONFIG_H

#include "structs.h"

#ifdef __cplusplus
extern "C" {
#endif

struct RoverConfig {
    int rover_id;
    double start_x;
    double start_y;
};

// Parse rovers.txt and populate rover configs
int load_rover_config(const char* filename, struct RoverConfig* configs, int* num_rovers);
struct RoverConfig* get_rover_config(int rover_id, struct RoverConfig* configs, int num_rovers);
int get_actual_num_rovers(void);


#ifdef __cplusplus
}
#endif

#endif
