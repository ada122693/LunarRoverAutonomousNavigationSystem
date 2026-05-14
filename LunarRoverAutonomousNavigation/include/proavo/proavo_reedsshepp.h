#include "structs.h"

#ifndef PROAVO_REEDSSHEPP_H
#define PROAVO_REEDSSHEPP_H

#ifdef __cplusplus
extern "C" {
#endif

/** rs for heuristic cost calculation */
double rs_heuristic(struct Location origin, struct Location destination);

/**get the reeds-shepp path */
struct VehicleState* getRSShot(struct VehicleState* current, struct Location destination);

#ifdef __cplusplus
}
#endif

#endif
