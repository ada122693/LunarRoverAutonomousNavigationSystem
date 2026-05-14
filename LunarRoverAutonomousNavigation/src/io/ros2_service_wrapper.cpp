#include <memory>
#include <thread>
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include "robot_interfaces/srv/add_request.hpp"

#include "constants/config.h"

#include "queue_management.h"
#include "io/ros2_service_wrapper.h"



class RequestServiceServer : public rclcpp::Node {
public:
    RequestServiceServer() : Node("request_service_server") {
        service_ = this->create_service<robot_interfaces::srv::AddRequest>(
            "add_request",
            std::bind(&RequestServiceServer::handle_add_request, this,
                      std::placeholders::_1, std::placeholders::_2)
        );
    }

private:
    void handle_add_request(
        const std::shared_ptr<robot_interfaces::srv::AddRequest::Request> request,
        std::shared_ptr<robot_interfaces::srv::AddRequest::Response> response)
    {
        // Validate rover_id is non-negative and reasonable
        if (request->rover_id < 0 || request->rover_id >= 10000) {
            response->success = false;
            return;
        }

        // Validate coordinates are within world dimensions
        if (request->x < -10.0 || request->x > X_DIM_SIZE + 10.0 ||
            request->y < -10.0 || request->y > Y_DIM_SIZE + 10.0) {
            response->success = false;
            return;
        }

        // Validate battery level is non-negative and reasonable (max 200000J)
        if (request->battery_level < 0.0 || request->battery_level > 200000.0) {
            response->success = false;
            return;
        }

        // Validate req_flag is within expected range
        if (request->req_flag < 0 || request->req_flag > 10) {
            response->success = false;
            return;
        }

        struct Location position;
        position.x_dim = request->x;
        position.y_dim = request->y;
        position.heading_rads = request->yaw_rads;

        int result = add_runtime_request(
            request->rover_id,
            request->req_flag,
            position,
            request->battery_level
        );

        response->success = (result == 0);
    }

    rclcpp::Service<robot_interfaces::srv::AddRequest>::SharedPtr service_;
};

static std::shared_ptr<RequestServiceServer> g_service_node = nullptr;
static std::shared_ptr<std::thread> g_ros_spin_thread = nullptr;

extern "C" {

int init_ros2_service_server() {
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }
    
    g_service_node = std::make_shared<RequestServiceServer>();
    
    g_ros_spin_thread = std::make_shared<std::thread>([]() {
        rclcpp::spin(g_service_node);
    });
    
    return 0;
}

void shutdown_ros2_service_server() {
    fprintf(stderr, "[SERVICE] shutdown_ros2_service_server called\n");
    if (g_service_node) {
        fprintf(stderr, "[SERVICE] Clearing service node\n");
        g_service_node = nullptr;
    }
    
    if (g_ros_spin_thread && g_ros_spin_thread->joinable()) {
        fprintf(stderr, "[SERVICE] Joining ROS spin thread\n");
        g_ros_spin_thread->join();
        fprintf(stderr, "[SERVICE] ROS spin thread joined\n");
        g_ros_spin_thread = nullptr;
    } else {
        fprintf(stderr, "[SERVICE] ROS spin thread not joinable or null\n");
    }
    fprintf(stderr, "[SERVICE] shutdown_ros2_service_server complete\n");
}

}
