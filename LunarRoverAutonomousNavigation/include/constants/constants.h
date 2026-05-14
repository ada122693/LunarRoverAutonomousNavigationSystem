#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <stdbool.h>
#include <stdint.h>

/* mathematical constants */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


#ifdef __cplusplus
extern "C" {
#endif

	extern const uint64_t NANOS_PER_SEC;
	extern const uint64_t NANOS_PER_MILLI;
	extern const uint64_t NANOS_PER_MICRO;

	/* system capacity limits */

	extern const int MAX_FAILED_START_POINTS;
	extern const int MAX_LINE_LENGTH;
	extern const int MAX_DESCRIPTION_LENGTH;

	/* queue and memory limits */

	extern const int DEFAULT_REQUEST_QUEUE_CAPACITY;
	extern const int DEFAULT_INSTRUCTION_QUEUE_CAPACITY;

	/* retry and timeout */

	extern const int MAX_DECONF_RETRIES;
	extern const int MAX_ITER_DECONFLICT;
	extern const int MAX_DECONF_TRIES;

	/* timing */

	extern const int MAX_ALLOWED_JITTER_TICKS;


	/* rover physical constants */

	extern const double ROVER_TRACK;
	extern const double ROVER_WHEELBASE;
	extern const double COG_HEIGHT;
	extern const double ROVER_RADIUS;
	extern const double MIN_TURN_RADIUS;
	extern const double VEHICLE_MASS_KG;
	extern const double LUNAR_GRAVITY;
	extern const double LUNAR_WEIGHT;
	extern const double EST_METER_COST;
	extern const double ROLLING_RESISTANCE_COEFF;
	extern const double K_TRACTION;
	extern const double THERMAL_LOSS_MARGIN;
	extern const double K_STEERING;
	extern const double NOMINAL_VELOCITY;

	/* pathfinding constants */

	extern const double ACCEPTABLE_DISTANCE_MARGIN_OBJECTIVE;
	extern const double ACCEPTABLE_HEADING_MARGIN_OBJECTIVE;
	extern const double PRIM_LENGTH;
	extern const double HEADING_GRANULARITY;
	extern const double RS_COST_WEIGHT;

	/* safety thresholds */

	extern const double TIP_MARGIN_HEURISTIC;
	extern const double BASELINE_LOSS;
	extern const double HARDER_TURN_LOSS;
	extern const double SAFE_HAVEN_SLOPE_THRESHOLD;
	extern const double CRITICAL_BATTERY_THRESHOLD;
	extern const double MAX_ALLOWED_SLOPE;
	extern const double MAX_ALLOWED_ROUGHNESS;

	/* objective scoring constants */

	extern const double DISTANCE_EPSILON;
	extern const double LOCATION_EPSILON;
	extern const double PRIORITY_WEIGHT_MULTIPLIER;
	extern const double DISTANCE_WEIGHT_MULTIPLIER;

	/* time sync */
#define TIME_BEFORE_SYNC_MS_DEF 1800000ULL
	extern const double TIME_BEFORE_SYNC_MS;

#ifdef __cplusplus
}
#endif
#endif
