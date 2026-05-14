#ifndef ROS2_SERVICE_WRAPPER_H
#define ROS2_SERVICE_WRAPPER_H

#ifdef __cplusplus
extern "C" {
#endif

// Initialize the ROS2 service server (call from C)
int init_ros2_service_server(void);

// Shutdown the ROS2 service server (call from C)
void shutdown_ros2_service_server(void);

#ifdef __cplusplus
}
#endif

#endif
