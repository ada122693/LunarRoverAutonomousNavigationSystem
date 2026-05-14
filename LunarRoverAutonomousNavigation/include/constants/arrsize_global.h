//this is for the global array
//its needed to set the size of the spacetime graph (see initialization.c)
//it must be the SAME as its corresponding value in config.c

#ifndef ARRSIZE_GLOBAL_H
#define ARRSIZE_GLOBAL_H
#ifdef __cplusplus
extern "C" {
#endif
	enum arrsize {
		SECTORS_ARR_SIZE = 100000,
		MAX_TIME_SLOTS_ARR_SIZE = 1000
	};
	enum maxobjectives {
		//intended to be a finite limit, can be changed but is intended to put a cap on the number of objectives
		//that are processed
		MAX_OBJECTIVES = 50,
		//this is just for clarity with the logic
		MAX_BLACKLISTED_ORIGINS = MAX_OBJECTIVES,

		//keep this within reason, there will be this amount of kdtrees so don't make a new priority level for every
		//minute preference, set them based on need
		MAX_OBJECTIVE_PRIORITY_LEVELS = 5
	};

	enum poolandsetspathfinding {
		// Maximum number of nodes in pathfinding pool
		MAX_NODES_IN_POOL = 50000,
		//max nodes in pathfinding pool is enum in arrsize_global.h
		// Maximum open set size for pathfinding
		MAX_OPEN_SET_SIZE = 20000,
		// Maximum closed set size for pathfinding
		MAX_CLOSED_SET_SIZE = 20000
	};

	/* discrete grid dimensions for pathfinding visited array */
	// Calculated from world dimensions and pathfinding resolution
	enum grid {
		//x dim size / prim length
		GRID_X_CELLS = 400,
		//y dim size / prim length
		GRID_Y_CELLS = 400,
		//2.0 * PI * HEADING_GRANULARITY
		GRID_HEADING_BINS = 49
	};

#ifdef __cplusplus
}
#endif
#endif
