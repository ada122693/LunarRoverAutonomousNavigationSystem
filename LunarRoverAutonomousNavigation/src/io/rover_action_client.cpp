#include "rover_action_client.h"
#include "rover_config.h"
#include "constants/constants.h"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_interfaces/action/path_schedule_plan.hpp>
#include <unordered_map>
#include <memory>
#include <chrono>
#include <std_msgs/msg/bool.hpp>


using PathSchedulePlan = robot_interfaces::action::PathSchedulePlan;

static std::unordered_map<int, std::shared_ptr<rclcpp_action::Client<PathSchedulePlan>>> g_action_clients;
static std::shared_ptr<rclcpp::Node> g_node;
static std::thread g_spin_thread;
static std::unordered_map<int, std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>>> g_done_publishers;

extern "C" {

void sync_wall_time(void) {
    // Sync ROS2 clock with system wall time for accurate chrono-based waiting
    // This ensures that when rovers wait for specific dispatch times, they use wall clock
    auto now = std::chrono::system_clock::now();
    auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();

    // Set the ROS2 clock to use system time (wall time)
    // This is done by ensuring use_sim_time is false, which uses system clock
    if (g_node) {
        if (!g_node->has_parameter("use_sim_time")) {
            g_node->declare_parameter("use_sim_time", false);
        } else {
            g_node->set_parameter(rclcpp::Parameter("use_sim_time", false));
        }
        RCLCPP_INFO(g_node->get_logger(), "Wall time sync enabled: use_sim_time set to false");
        RCLCPP_INFO(g_node->get_logger(), "Current wall time: %ld ns", now_ns);
    }
}

int init_rover_action_clients(const char* config_file) {
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }

    // create a node for action clients
    g_node = std::make_shared<rclcpp::Node>("rover_action_client_manager");

    // sync wall time for chrono-based waiting
    sync_wall_time();

    // Load rover config
    struct RoverConfig configs[MAX_ROVERS];
    int num_rovers = 0;
    if (load_rover_config(config_file, configs, &num_rovers) != 0) {
        fprintf(stderr, "Failed to load rover config\n");
        return -1;
    }

    // Create action client for each rover
    for (int i = 0; i < num_rovers; i++) {
        std::string action_name = "execute_path_schedule";
        auto client = rclcpp_action::create_client<PathSchedulePlan>(g_node, action_name);
        g_action_clients[configs[i].rover_id] = client;
        printf("Created action client for rover %d: %s\n", configs[i].rover_id, action_name.c_str());
    }
    for (int i = 0; i < num_rovers; i++) {
    auto client = rclcpp_action::create_client<PathSchedulePlan>(g_node, "execute_path_schedule");
    g_action_clients[configs[i].rover_id] = client;

    // for publishing to shims
    auto pub = g_node->create_publisher<std_msgs::msg::Bool>(
        "rover_" + std::to_string(configs[i].rover_id) + "_route_done", 10);
    g_done_publishers[configs[i].rover_id] = pub;
}

    // Spin the node in a separate thread
    g_spin_thread = std::thread([]() {
        rclcpp::spin(g_node);
    });

    return 0;
}

void* get_rover_action_client(int rover_id) {
    auto it = g_action_clients.find(rover_id);
    if (it != g_action_clients.end()) {
        return it->second.get();
    }
    return nullptr;
}

void rclcpp_shutdown_wrapper(void) {
    fprintf(stderr, "[RCLCPP] rclcpp_shutdown_wrapper called\n");
    rclcpp::shutdown();
    fprintf(stderr, "[RCLCPP] rclcpp::shutdown() returned\n");
}

void shutdown_rover_action_clients(void) {
    fprintf(stderr, "[ACTION] shutdown_rover_action_clients called\n");
    g_action_clients.clear();
    fprintf(stderr, "[ACTION] Action clients cleared\n");
    if (g_spin_thread.joinable()) {
        fprintf(stderr, "[ACTION] Joining action client spin thread\n");
        g_spin_thread.join();
        fprintf(stderr, "[ACTION] Action client spin thread joined\n");
    } else {
        fprintf(stderr, "[ACTION] Action client spin thread not joinable\n");
    }
    if (g_node) {
        fprintf(stderr, "[ACTION] Clearing action client node\n");
        g_node = nullptr;
    }
    fprintf(stderr, "[ACTION] shutdown_rover_action_clients complete\n");
}


}

rclcpp::Node* get_rover_action_node(void) {
    return g_node.get();

    
}
rclcpp::Publisher<std_msgs::msg::Bool>* get_rover_done_publisher(int rover_id) {
    auto it = g_done_publishers.find(rover_id);
    if (it != g_done_publishers.end()) return it->second.get();
    return nullptr;
}