#ifndef ROBOT_INITIALIZATION_H
#define ROBOT_INITIALIZATION_H

#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "robot_interfaces/action/path_schedule_plan.hpp"

using PathSchedulePlan = robot_interfaces::action::PathSchedulePlan;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize robot ID cache by loading from rover configuration
 * Must be called before get_robot_ids()
 * return 0 on success, -1 on failure
 */
int init_robot_initialization(void);

/**
 * Get list of robot IDs from cache
 * Must call init_robot_initialization() first
 * return Vector of robot IDs
 */
std::vector<int> get_robot_ids(void);

/**
 * Cleanup robot initialization resources
 */
void cleanup_robot_initialization(void);

/**
 * Dispatch a schedule to a specific rover
 * id Rover ID
 * goal PathSchedulePlan goal to send
 */
void dispatch_schedule(int id, const PathSchedulePlan::Goal& goal);

#ifdef __cplusplus
}
#endif

#endif
