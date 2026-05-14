#ifndef PROAVO_COST_H
#define PROAVO_COST_H

#include <stdbool.h>
#include "structs.h"

#ifdef __cplusplus
extern "C" {
#endif

extern const double CALCULATE_POWER_FAIL_RETURN;
extern const double TIME_PENALTY_SEC;

bool update_energy_cost(struct VehicleState* current, struct VehicleState* successor, 
                       double slope_rads, double steer_angle, double steer_vel);

#ifdef __cplusplus
}
#endif

#endif
