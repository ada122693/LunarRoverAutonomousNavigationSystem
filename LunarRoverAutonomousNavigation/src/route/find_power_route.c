#include "structs.h"

#include "route/find_power_route.h"
#include "objectives/power_station_kdtree.h"

bool find_power_route(struct RouteRequest *request) {
    // Use KD-tree (O(log n) vs O(n))
    struct PowerStation* closest_station = power_station_kdtree_find_nearest(
        request->rover->position.x_dim,
        request->rover->position.y_dim
    );

    if (closest_station == NULL) {
        return false; // No power stations available
    }

    // set rover objective position to the closest power station
    request->rover->objective_position = closest_station->location;

    return true; // Success
}
