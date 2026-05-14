#ifndef CONFIG_H
#define CONFIG_H

#include <stdbool.h>
#include <stdint.h>


#define MAX_ROVERS 10

#ifdef __cplusplus
extern "C" {
#endif
	/* energy and battery*/

	extern const double BATTERY_SAFETY_MARGIN;
	extern const double MAX_SAFE_WAIT_BATTERY_DRAIN;
	extern const double ENERGY_TO_BATTERY_DRAIN_RATIO;
	extern const double BATTERY_CAPACITY;
	extern const double PASSIVE_BATTERY_DRAIN_RATE;

	/* timing */

	extern const double TIME_SLOT_SIZE;
	extern const int TOTAL_TIME;
	extern const int MAX_TIME_SLOTS;

	/* program timing */

#define PROACTIVE_MONITORING_INTERVAL_DEF 100000000ULL
	extern const uint64_t PROACTIVE_MONITORING_INTERVAL;
	extern const int SCHEDULING_PACE;

	/* pathfinding */

	extern const double GRID_RESOLUTION;
	extern const double HEADING_RESOLUTION;

	/* occupancy grid */
	//in meters (of world)
	extern const double MAX_ELEVATION;

	/* system behavior */

	extern const int MAX_CALCULATED_SAFE_WAITS;
	extern const int MIN_CALCULATED_SAFE_WAITS;
	extern const int RETRY_PRIORITY_START;
	extern const int REQUEST_PROCESS_DELAY_US;
	extern const int MAX_POWER_STATION_SEARCHES;

	/* spatial */

	extern const double SECTOR_WIDTH;
	extern const double SECTOR_HEIGHT;

	/* world dimensions */

	extern const double X_DIM_SIZE;
	extern const double Y_DIM_SIZE;
	extern const int SECTORS_X;
	extern const int SECTORS_Y;
	extern const int SECTORS;
	extern const int SECTOR_TIME_SLOTS;

	/* debug and testing */

	extern const bool DEBUG_ENABLED;
	extern const bool PATH_STORAGE_ENABLED;
	extern const int MAX_COMPLETED_PACKAGES;

	extern const char* SCAN_FILE_PATH;
	extern const char* PATH_FILENAME;
	extern const char* ROVERS_FILENAME;
	extern const char* OBJECTIVES_FILENAME;
	extern const char* GRID_FILE_PATH;

	/* pathfinding tolerances and margins */

	extern const double ACCEPTABLE_DISTANCE_MARGIN_OBJECTIVE;
	extern const double ACCEPTABLE_HEADING_MARGIN_OBJECTIVE;
	extern const double PRIM_LENGTH;
	extern const double HEADING_GRANULARITY;

	/* steering angles for pathfinding */

	extern const double POTENTIAL_STEERING[];
	extern const int POTENTIAL_STEERING_LEN;

	/* Reeds-Shepp heuristic threshold */

	extern const double HEURISTIC_SAFE_SLOPE;

	/* motor specifications for energy validation */

	extern const double MOTOR_RATED_POWER;
	extern const int NUM_MOTORS;
	extern const double MOTOR_VOLTAGE;
	extern const double MOTOR_CURRENT_TYPICAL;
	extern const double MOTOR_CURRENT_STALL;
	extern const double BATTERY_CAPACITY_WH;
	extern const double BATTERY_CAPACITY_JOULES;
	extern const double MOTOR_EFFICIENCY;

#ifdef __cplusplus
}
#endif

#endif
