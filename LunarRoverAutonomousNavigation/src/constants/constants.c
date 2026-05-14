#include "constants.h"

/* math */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

const uint64_t NANOS_PER_SEC = 1000000000ULL;
const uint64_t NANOS_PER_MILLI = 1000000ULL;
const uint64_t NANOS_PER_MICRO = 1000ULL;

//MEASUREMENT UNITS ON THIS PAGE ARE CURRENTLY NOT SI,

/* system capacity limits */

// Maximum number of failed start points to track
const int MAX_FAILED_START_POINTS = 20;
// Maximum line length for file parsing
const int MAX_LINE_LENGTH = 200;
// Maximum description length for objectives and power stations
const int MAX_DESCRIPTION_LENGTH = 100;

/* queue and memory limits */

// Default request queue capacity
const int DEFAULT_REQUEST_QUEUE_CAPACITY = 100;
// Default instruction package queue capacity
const int DEFAULT_INSTRUCTION_QUEUE_CAPACITY = 50;


/* deconf*/
//max times deconf can retry one route
const int MAX_DECONF_RETRIES = 15;
const int MAX_ITER_DECONFLICT = 10000;
//max times deconf can run on a queue
//its set low because it'll never be given more than there are rovers
const int MAX_DECONF_TRIES = 10;

/* timing */
// Maximum allowed jitter ticks before system halts for safety
const int MAX_ALLOWED_JITTER_TICKS = 1000;

/* priority levels */

const int MAX_PRIORITY = 4;
const int HIGH_PRIORITY = 3;
const int NORMAL_PRIORITY = 2;
const int LOW_PRIORITY = 1;

/* request flags */

const int POWER_EMERGENCY = 0;
const int POWER_REQUEST = 1;
const int ROVER_NO_OBJECTIVE = 2;
const int ROUTE_REQUEST = 3;
const int CRITICAL_BATTERY = 4;
const int LOW_BATTERY = 5;

/* route status flags */

const int ROUTE_ASSIGNED = 0;
const int NEED_NEW_INSTRUCTIONS = 1;
const int NEED_NEW_ROUTE = 2;
const int NO_SOLUTION_KILL_ROVER = 3;
const int IDLE = 4;
const int NO_PATH_FOUND = 5;

/* error codes */

const int SCHEDULING_CONFLICT = -1;
const int INSUFFICIENT_BATTERY = -2;
const int PATH_VALIDATION_FAILED = -3;
const int INACCURATE_INFO_QUIET = -4;

/* rover physical constants */

// Rover physical dimensions and properties
const double ROVER_TRACK = 0.160; // distance between wheels (meters
const double ROVER_WHEELBASE = 0.105; // Distance between axles (meters)
const double COG_HEIGHT = 0.085; // Center of gravity height (meters)
const double ROVER_RADIUS = 0.3; //not the actual radius, make it bigger so collision checks work
// Minimum turning radius (meters) = wheelbase / tan(max steering angle)
const double MIN_TURN_RADIUS = 0.182;
const double VEHICLE_MASS_KG = 4.0; // Vehicle mass in kg
const double LUNAR_GRAVITY = 9.81; // lunar gravity in m/s squared
//EARTH FOR DEMO
//vehicle mass kg * lunar gravity
const double LUNAR_WEIGHT = 39.24;
const double ROLLING_RESISTANCE_COEFF = 0.015; // HARD GROUND - lunar dust could be much much more
const double K_TRACTION = 200.0; //HARD GROUND
const double THERMAL_LOSS_MARGIN = 1.2; // not really any way to know this for real I guess that's why it exists
const double K_STEERING = 0.05; //HARD GROUND
const double NOMINAL_VELOCITY = 0.30; // Nominal velocity in m/s
const double EST_METER_COST = 20.0;

/* pathfinding constants */


const double RS_COST_WEIGHT = 15.0; // Reeds-Shepp cost weight

/* safety thresholds */

// Safety and threshold constants
const double TIP_MARGIN_HEURISTIC = 0.87; // Tipover margin heuristic
const double BASELINE_LOSS = 0.5; // Baseline loss for cornering
const double HARDER_TURN_LOSS = 2.0; // Harder turn loss factor
const double SAFE_HAVEN_SLOPE_THRESHOLD = 0.1; // Safe haven slope threshold (radians)
const double CRITICAL_BATTERY_THRESHOLD = 20.0; // Critical battery level percentage
const double MAX_ALLOWED_SLOPE = 0.2618; // max slope rads, for collision checks
const double MAX_ALLOWED_ROUGHNESS = 0.5; // max steps in height, for collision checks checks stuff slope may not get

/* objective scoring constants */

const double DISTANCE_EPSILON = 0.001; // Small epsilon to avoid division by zero
const double LOCATION_EPSILON = 0.001; // Small epsilon for location comparisons
const double PRIORITY_WEIGHT_MULTIPLIER = 10.0; // Weight for priority in objective scoring
const double DISTANCE_WEIGHT_MULTIPLIER = 1.0; // Weight for distance in objective scoring


//Time sync interval to ensure wall time drift isn't problematic
//ITS EQUIVALENT MUST BE THE SAME IN constants.h
//#define TIME_BEFORE_SYNC_MS_DEF
const double TIME_BEFORE_SYNC_MS = 1800000; // 1,800,000 ms = 1800 seconds, roughly 30 minutes.
