#ifndef GENERATE_INSTRUCTIONS_GLOBAL_H
#define GENERATE_INSTRUCTIONS_GLOBAL_H

#include "structs.h"

struct Instruction* generate_instructions_global(struct InstructionMetadata *metadata, struct VehicleState *start_node, int *sector_start_array, bool **has_safe_haven_out, struct WaitHookInfo **hooks_out, int sectors_traversal_count);

#endif
