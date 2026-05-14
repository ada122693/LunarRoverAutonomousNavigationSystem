#ifndef ROUTE_PLANNER_H
#define ROUTE_PLANNER_H

#include "structs.h"
#ifdef __cplusplus
extern "C" {
#endif

//uses the request flag
struct RouteRequest* route_planner(struct Rover* rover);

void no_path_found(struct RouteRequest* request);

#ifdef __cplusplus
}
#endif

#endif
