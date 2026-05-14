#ifndef ROUTE_PROVIDER_H
#define ROUTE_PROVIDER_H

#include "structs.h"
#ifdef __cplusplus
extern "C" {
#endif

	void route_provider (struct RouteRequest* request);
	bool assign_route(struct Rover *rover, struct InstructionPackage *instructions);

#ifdef __cplusplus
}
#endif
#endif