//error codes as enums
/* status_codes.h*/
#ifndef STATUS_CODES_H
#define STATUS_CODES_H
#ifdef __cplusplus
extern "C" {
#endif

	enum priority_codes {
		MAX_PRIORITY = 4,
		HIGH_PRIORITY = 3,
		NORMAL_PRIORITY = 2,
		LOW_PRIORITY = 1
	};

	enum req_flags {
		POWER_EMERGENCY = 0,
		POWER_REQUEST = 1,
		ROVER_NO_OBJECTIVE = 2,
		ROUTE_REQUEST = 3,
		CRITICAL_BATTERY = 4,
		LOW_BATTERY = 5
	};

	enum route_status {
		ROUTE_ASSIGNED = 0,
		NEED_NEW_INSTRUCTIONS = 1,
		NEED_NEW_ROUTE = 2,
		NO_SOLUTION_KILL_ROVER = 3,
		IDLE = 4,
		NO_PATH_FOUND = 5
	};

	enum error_codes {
		SCHEDULING_CONFLICT = -1,
		INSUFFICIENT_BATTERY = -2,
		PATH_VALIDATION_FAILED = -3,
		INACCURATE_INFO_QUIET = -4,
	};

#ifdef __cplusplus
}
#endif

#endif