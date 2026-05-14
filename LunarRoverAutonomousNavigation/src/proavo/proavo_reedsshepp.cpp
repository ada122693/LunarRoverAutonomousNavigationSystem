#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <cstdlib>
#include <cmath>
#include <cstdio>

#include "constants/constants.h"
#include "constants/config.h"
#include "structs.h"
#include "proavo/proavo_pool.h"
#include "gridmap/gridmap.h"
#include "proavo/proavo_reedsshepp.h"
#include "proavo/proavo_cost.h"

std::shared_ptr<ompl::base::ReedsSheppStateSpace> rsSpace = std::make_shared<ompl::base::ReedsSheppStateSpace>(MIN_TURN_RADIUS);

extern "C" double rs_heuristic (struct Location origin, struct Location destination) {
    ompl::base::ScopedState<> o (rsSpace), d (rsSpace);
    auto* originState = o->as<ompl::base::SE2StateSpace::StateType> ();
    originState ->setX (origin.x_dim);
    originState ->setY (origin.y_dim);
    originState ->setYaw (origin.heading_rads);
    auto* destState = d->as<ompl::base::SE2StateSpace::StateType> ();
    destState->setX (destination.x_dim);
    destState->setY (destination.y_dim);
    destState->setYaw(destination.heading_rads);
    return rsSpace->distance(o.get(), d.get());
}

extern "C" double ReedsSheppCost(struct Location origin, struct Location destination) {
    return rs_heuristic(origin, destination);
}

extern "C" struct VehicleState* getRSShot(struct VehicleState* current, struct Location destination) {
    ompl::base::ScopedState<> tempState(rsSpace);
    ompl::base::ScopedState<> o(rsSpace), d(rsSpace);
    auto* originState = o->as<ompl::base::SE2StateSpace::StateType>();
    originState->setX(current->position.x_dim);
    originState->setY(current->position.y_dim);
    originState->setYaw(current->position.heading_rads);
    auto* destState = d->as<ompl::base::SE2StateSpace::StateType>();
    destState->setX(destination.x_dim);
    destState->setY(destination.y_dim);
    destState->setYaw(destination.heading_rads);
    

    double totalDist = rsSpace->distance(o.get(), d.get());
    int numSteps = (int)(totalDist / PRIM_LENGTH) + 1;

    /**check collisions*/
    for (int i = 1; i <= numSteps; ++i) {
        rsSpace->interpolate(o.get(), d.get(), (double)i / numSteps, tempState.get());
        auto* s = tempState->as<ompl::base::SE2StateSpace::StateType>();

        if (check_collision(s->getX(), s->getY())) {
    fprintf(stderr, "RS collision at %.2f %.2f\n", s->getX(), s->getY());
    		return nullptr; //collision
	}
        // cautious slope check
        double slope = get_slope(s->getX(), s->getY());
        if (slope > HEURISTIC_SAFE_SLOPE) {
            return nullptr;
        }
    }
    fprintf(stderr, "rs found with no collisions");

    /**no collisions*/
    int start_index = pool_get_index();
    struct VehicleState* prev = NULL;
    struct VehicleState* first = NULL;
    for (int i = 1; i <= numSteps; i++) {
        rsSpace->interpolate(o.get(), d.get(), (double)i / numSteps, tempState.get());
        auto* s = tempState->as<ompl::base::SE2StateSpace::StateType>();

        struct VehicleState* next = pool_alloc_vehicle_state();
        if (!next) {
            pool_rollback(start_index);
            fprintf(stderr, "pool issue\n");
            return nullptr;
        }

        next->position.x_dim = s->getX();
        next->position.y_dim = s->getY();
        next->position.heading_rads = s->getYaw();
        next->parent_node = (prev == NULL) ? current : prev;
if (prev == NULL) { first = next; }

        struct VehicleState* real_prev = (prev == NULL) ? current : prev;
        // find turn angle
        double delta_theta = next->position.heading_rads - real_prev->position.heading_rads;
        while (delta_theta > M_PI) delta_theta -= 2.0 * M_PI;
        while (delta_theta < -M_PI) delta_theta += 2.0 * M_PI;
        double steering_estimate = atan2(delta_theta * ROVER_WHEELBASE, PRIM_LENGTH);

        // g(n) cost
        double slope = get_slope(next->position.x_dim, next->position.y_dim);
        if (!update_energy_cost(real_prev, next, slope, steering_estimate, 0.0)) {
            pool_rollback(start_index);
            fprintf(stderr, "tipover\n");
            return nullptr;  /** this shouldn't be reached, if it is then the threshold for tipover is wrong */
        }

        prev = next;
    }
    fprintf(stderr, "returning rs curve at node with x=%.3f y=%.3f yaw=%.3f\n", prev->position.x_dim, prev->position.y_dim, prev->position.heading_rads);
    return prev;
}
