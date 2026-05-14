#ifndef OBJECTIVES_LOGIC_H
#define OBJECTIVES_LOGIC_H

#include <stdbool.h>
#include <stdlib.h>
#include "structs.h"

#ifdef __cplusplus
extern "C" {
#endif

// Global objectives system variables
extern struct Objective objectives[];
extern struct PowerStation power_stations[];
extern int power_station_count;
extern int total_objective_count;
extern int objective_counts[];

//weighted best objective based on distance and priority excluding
//blacklisted origin locations
bool check_objectives(struct RouteRequest* request);

//when objective is validated, this handles after
void validate_and_remove_objective(struct Objective* objective);

//for when validate was already done and it still fails
void objective_kdtree_push_and_blacklist(struct RouteRequest* request);
//when validate_and_remove_objective wasn't called this is used when validation fails
void blacklist_origin_for_failed_objective(struct RouteRequest* request);

#ifdef __cplusplus
}
#endif

#endif

