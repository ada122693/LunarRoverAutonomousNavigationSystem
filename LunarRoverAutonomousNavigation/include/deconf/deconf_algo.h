#ifndef DECONF_ALGO_H
#define DECONF_ALGO_H

#include "structs.h"

#ifdef __cplusplus
extern "C" {
#endif
	bool deconf_standard (struct InstructionPackage* route, int* time_till_potential_valid, int cumulative_initial_wait, int actual_start_time);
#ifdef __cplusplus
}
#endif
#endif