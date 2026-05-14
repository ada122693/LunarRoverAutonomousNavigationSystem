#ifndef NAV_RESERVATIONGRAPH_H
#define NAV_RESERVATIONGRAPH_H

#include "structs.h"

#ifdef __cplusplus
extern "C" {
#endif
//this allows for deconfliction to work
//its a graph of a path over discretized spacetime
struct SectorTraversal* generate_reservationgraph(struct VehicleState* startNode, struct VehicleState* endNode, struct InstructionMetadata* metadata, int** sector_start_array);

#ifdef __cplusplus
}
#endif

#endif
