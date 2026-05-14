#ifndef NAV_MAIN_H
#define NAV_MAIN_H

#include "structs.h"

struct InstructionPackage* nav_main(struct VehicleState *foundPathNode, struct Location start, struct Location end);

#endif
