#ifndef ROVER_ACTION_CLIENT_H
#define ROVER_ACTION_CLIENT_H


#ifdef __cplusplus
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
rclcpp::Node* get_rover_action_node(void);
rclcpp::Publisher<std_msgs::msg::Bool>* get_rover_done_publisher(int rover_id);
extern "C" {
#endif

// Initialize rover action clients from config file
int init_rover_action_clients(const char* config_file);

// Sync wall time for chrono-based waiting
void sync_wall_time(void);

// Get action client for a specific rover (returns opaque pointer)
void* get_rover_action_client(int rover_id);

// Shutdown all rover action clients
void shutdown_rover_action_clients(void);

// Wrapper to call rclcpp::shutdown()
void rclcpp_shutdown_wrapper(void);


#ifdef __cplusplus
}
#endif

#endif
