 #include <unistd.h>
 #include <signal.h>
 #include <stdbool.h>
 #include <stdio.h>
 #include <stdint.h>
 #include "constants/constants.h"
 #include "constants/config.h"
 #include "initialization.h"
 #include "dispatcher.h"
 #include "platform.h"
 #include "io/ros2_service_wrapper.h"
 #include "rover_action_client.h"


static volatile bool running = true;
void shutdown_handler(int sig) {
  fprintf(stderr, "[SIGNAL] Received signal, shutting down...\n");
  running = false;
}

int main(void) {
  fprintf(stderr, "[MAIN] Starting main\n");
  signal(SIGTERM, shutdown_handler);
  signal(SIGINT, shutdown_handler);

  // Initialize system data structures
  fprintf(stderr, "[STATUS] Initializing system...\n");
  if (system_initialization() != 0) {
    fprintf(stderr, "Failed to initialize system\n");
    return 1;
  }

  // Initialize ROS2 service server for external requests
  fprintf(stderr, "[STATUS] Calling init_ros2_service_server\n");
  if (init_ros2_service_server() != 0) {
    fprintf(stderr, "Failed to initialize ROS2 service server\n");
  }
  fprintf(stderr, "[STATUS] init_ros2_service_server returned\n");

  fprintf(stderr, "[STATUS] Creating timer\n");
  platform_timer_t* timer = platform_timer_create(PROACTIVE_MONITORING_INTERVAL);
  if (!timer) {
    fprintf(stderr, "Failed to create timer\n");
    system_cleanup();
    return 1;
  }
  fprintf(stderr, "[STATUS] Timer created\n");

  fprintf(stderr, "[STATUS] Starting system loop\n");
  run_system_loop(timer, &running);
  fprintf(stderr, "[STATUS] System loop exited\n");

  fprintf(stderr, "[STATUS] Cleaning up...\n");
  platform_timer_destroy(timer);
  fprintf(stderr, "[STATUS] Timer destroyed\n");

  fprintf(stderr, "[MAIN] Calling shutdown_ros2_service_server\n");
  shutdown_ros2_service_server();
  fprintf(stderr, "[MAIN] shutdown_ros2_service_server returned\n");
  fprintf(stderr, "[MAIN] Calling shutdown_rover_action_clients\n");
  shutdown_rover_action_clients();
  fprintf(stderr, "[MAIN] shutdown_rover_action_clients returned\n");
  extern void rclcpp_shutdown_wrapper(void);
  fprintf(stderr, "[MAIN] Calling rclcpp_shutdown_wrapper\n");
  rclcpp_shutdown_wrapper();
  fprintf(stderr, "[MAIN] rclcpp_shutdown_wrapper returned\n");
  fprintf(stderr, "[MAIN] Cleanup complete, exiting\n");
  return 0;
}
