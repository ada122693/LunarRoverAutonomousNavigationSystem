#ifndef PROAVO_ALGO_H
#define PROAVO_ALGO_H

#include "structs.h"

// Main hybrid A* algorithm function
struct VehicleState* proavo_algo(struct VehicleState* start_node, struct Location objective);

#endif
