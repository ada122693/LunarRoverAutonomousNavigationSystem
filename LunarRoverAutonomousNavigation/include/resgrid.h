#ifndef RES_GRID_H
#define RES_GRID_H

#include "arrsize_global.h"

#ifdef __cplusplus
extern "C" {
#endif

	extern bool g_reservation_grid[SECTORS_ARR_SIZE][MAX_TIME_SLOTS_ARR_SIZE];
	extern int g_intervals[SECTORS_ARR_SIZE][MAX_TIME_SLOTS_ARR_SIZE];

#ifdef __cplusplus
}
#endif
#endif