#include "config.h"
#include "constants.h"
#include <stdint.h>

/* energy and battery */
// Joules safety margin for battery operations
const double BATTERY_SAFETY_MARGIN = 10.0;
// Maximum joules for safe waiting - USER CONFIGURABLE
const double MAX_SAFE_WAIT_BATTERY_DRAIN = 50.0;
// Conversion factor: energy -> battery drain
const double ENERGY_TO_BATTERY_DRAIN_RATIO = 0.8;
// Total battery capacity in joules (2000mAh @ 7.4V = 14.8Wh = 53280J)
const double BATTERY_CAPACITY = 53280.0;
// Joules per second when idle (0.5W = typical for embedded system with ROS2 comms)
const double PASSIVE_BATTERY_DRAIN_RATE = 0.5;

/* timing */
// Total number of time slots in schedule
const int TOTAL_TIME = 1000;

/* program timing */
// Proactive monitoring interval in nanoseconds (1s = 1,000,000,000 ns)
//MUST CHANGE ITS #define EQUIVALENT UPON CHANGING AS WELL
//TO BE IDENTICAL IF YOU CHANGE THIS HERE
//IN config.h, PROACTIVE_MONITORING_INTERVAL_DEF
const uint64_t PROACTIVE_MONITORING_INTERVAL = 100000000ULL; // 100ms in ns
// Scheduling pace - how often to run scheduling (in timer ticks)
const int SCHEDULING_PACE = 10; // Run scheduling every 10 timer ticks

/* pathfinding */
// Grid resolution for pathfinding (meters)
const double GRID_RESOLUTION = 1.0;
// Heading resolution for pathfinding (radians)
const double HEADING_RESOLUTION = 0.1;

/* occupancy grid */
//for testing in gazebo, maximum elevation in the gazebo world
//in meters
//10 for the blank png, 5 for the terrain.png one
const double MAX_ELEVATION = 5.0;

/* system behavior */
// Maximum safe waits calculated from battery (reasonable upper cap)
const int MAX_CALCULATED_SAFE_WAITS = 20;
// Minimum safe waits (ensure at least 1 wait)
const int MIN_CALCULATED_SAFE_WAITS = 1;
// Retry priority starting value for failed objectives
const int RETRY_PRIORITY_START = 2;
// Delay for request processing loop in microseconds
const int REQUEST_PROCESS_DELAY_US = 10000; // 10ms
// Maximum number of power stations to search before giving up
const int MAX_POWER_STATION_SEARCHES = 5;

/* world dimensions */
const double X_DIM_SIZE = 20.0; // 20 meters //20 IN SIMULATION, 40 inch in demo
const double Y_DIM_SIZE = 20.0; // 20 meters //20 IN SIMULATION, 30 inch in demo
// discrete grid
//X_DIM_SIZE / SECTOR_WIDTH
//num of each sector dimension
const int SECTORS_X = 40;
const int SECTORS_Y = 40;
//total num of sectors
//MUST BE CHANGED IN arrsize_global.h TO BE THE SAME
const int SECTORS = 1600;
const int SECTOR_TIME_SLOTS = 1000;
// maximum number of time slots
// MUST BE CHANGED IN arrsize_global.h TO BE THE SAME
const int MAX_TIME_SLOTS = 1000;
// duration of each time slot in seconds
const double TIME_SLOT_SIZE = 1.0;

/* spatial */
// Sector width for reservation grid
// at least 3x prim_length but should be WELL above that
const double SECTOR_WIDTH = 0.5;
// Sector height for reservation grid
const double SECTOR_HEIGHT = 0.5;

/* debug and testing */
// Enable/disable debug output
const bool DEBUG_ENABLED = false;
// Enable/disable path storage for testing
const bool PATH_STORAGE_ENABLED = true;
// Maximum completed packages for processing
const int MAX_COMPLETED_PACKAGES = 50;

/**file stuff */
//pcd path
//const char* GRID_FILE_PATH = "/home/yahboom/SAVED/yahboom_map.pgm";
//const char* GRID_FILE_PATH = "/home/yahboom/yahboomcar_ros2_ws/yahboomcar_ws/install/LunarRoverAutonomousNavigation/lib/LunarRoverAutonomousNavigation/flat_world.png";
const char* GRID_FILE_PATH = "/home/yahboom/terrain.png";
//path storage -paths as in pathfinding not filepath here
const char* PATH_FILENAME = "/home/yahboom/yahboomcar_ros2_ws/yahboomcar_ws/install/LunarRoverAutonomousNavigation/lib/LunarRoverAutonomousNavigation/stored_paths.dat";
//rover config
const char* ROVERS_FILENAME = "/home/yahboom/yahboomcar_ros2_ws/yahboomcar_ws/install/LunarRoverAutonomousNavigation/lib/LunarRoverAutonomousNavigation/rovers.txt";
//objectives file
const char* OBJECTIVES_FILENAME = "/home/yahboom/yahboomcar_ros2_ws/yahboomcar_ws/install/LunarRoverAutonomousNavigation/lib/LunarRoverAutonomousNavigation/example_objectives.txt";

// Pathfinding tolerances and margins
const double ACCEPTABLE_DISTANCE_MARGIN_OBJECTIVE = 0.05; // Acceptable distance to objective (meters)
const double ACCEPTABLE_HEADING_MARGIN_OBJECTIVE = 0.1; // Acceptable heading margin (radians)
const double PRIM_LENGTH = 0.05; // Step length between nodes (meters)
const double HEADING_GRANULARITY = M_PI/24; // Heading granularity for pathfinding (radians)

// Steering angles for pathfinding (radians) - differential drive robot
// Reduced set since Reeds-Shepp handles reversing
//NOT CURRENTLY CALCULATED
const double POTENTIAL_STEERING[] = {
    0.0,              // Straight
    0.523599,     // 20 degrees
    -0.523599,    // 20 degrees right
    0.174533,     // 10 degrees left
    -0.174533     // 10 degrees right
};
const int POTENTIAL_STEERING_LEN = 5; // Number of steering angles

// Safe slope threshold for Reeds-Shepp heuristic (radians)
const double HEURISTIC_SAFE_SLOPE = 0.2;

/* motor specifications for energy validation */
const double MOTOR_RATED_POWER = 9.0; // Watts per motor
const int NUM_MOTORS = 2; // Number of motors
const double MOTOR_VOLTAGE = 12.0; // Volts
const double MOTOR_CURRENT_TYPICAL = 0.9; // Amps per motor (typical operation)
const double MOTOR_CURRENT_STALL = 2.5; // Amps per motor (stall)
const double BATTERY_CAPACITY_WH = 22.2; // Watt-hours (2000mAh @ 7.4V)
const double BATTERY_CAPACITY_JOULES = 79920.0; // Joules (14.8Wh * 3600s)
const double MOTOR_EFFICIENCY = 0.85; // Motor efficiency (0-1)
