#ifndef GRIDMAP_H
#define GRIDMAP_H

#ifdef __cplusplus
extern "C" {
#endif

// Initialize the grid map
void init_gridmap(void);

//returns true if collision
bool check_collision (double x, double y);

//in rads
double get_slope (double x, double y);

#ifdef __cplusplus
}
#endif

#endif
