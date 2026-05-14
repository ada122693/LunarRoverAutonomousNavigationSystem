#ifndef DECONF_MAIN_H
#define DECONF_MAIN_H

#include "structs.h"

#ifdef __cplusplus
extern "C" {
#endif

	int deconf_main (struct InstructionPackage** pending, struct InstructionPackage** completed, int* completed_count, int start_time);
#ifdef __cplusplus
}
#endif
#endif
