#ifndef PROAVO_POOL_H
#define PROAVO_POOL_H

#include "structs.h"

//proavo uses a pool for VehicleStruct

#ifdef __cplusplus
extern "C" {
#endif

// Allocate a VehicleState from the node pool
struct VehicleState* pool_alloc_vehicle_state(void);

void pool_reset(void);

int pool_get_index(void);

void pool_rollback(int index);

#ifdef __cplusplus
}
#endif

#endif
