#ifndef REQUEST_HANDLER_H
#define REQUEST_HANDLER_H

#include "structs.h"
#ifdef __cplusplus
extern "C" {
#endif

	int rover_request_handler (struct Rover* rover);
	struct RouteRequest* plan_route (struct Rover* rover);

#ifdef __cplusplus
}
#endif
#endif
