#include <string>
#include <map>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "robot_interfaces/action/path_schedule_plan.hpp"

#include "rover_config.h"
#include "constants/constants.h"

#include "robot/robot_initialization.h"

//this initializes the robot interfaces so I can send data to and from them

using PathSchedulePlan = robot_interfaces::action::PathSchedulePlan;

// Static storage for robot IDs
static std::vector<int> g_robot_ids;
static bool g_robot_ids_loaded = false;

// PathScheduleCentralServer for managing rover action clients
class PathScheduleCentralServer : public rclcpp::Node {
public:
    PathScheduleCentralServer() : Node("path_schedule_central_server") {
        std::vector<int> robot_ids = get_robot_ids();

        for (int id : robot_ids) {
            //construct network address
            std::string action_name = "execute_path_schedule";

            //initialize io pipe
            auto client = rclcpp_action::create_client<PathSchedulePlan>(this, action_name);
            rover_clients_[id] = client;
        }
    }

    //to push a package
    void dispatch_schedule(int id, const PathSchedulePlan::Goal& goal) {
        auto client = rover_clients_[id];
        if (client->wait_for_action_server(std::chrono::seconds(1))) {
            client->async_send_goal(goal);
        }
    }


private:
    std::map<int, rclcpp_action::Client<PathSchedulePlan>::SharedPtr> rover_clients_;
};

static std::shared_ptr<PathScheduleCentralServer> g_central_server;
static std::thread g_spin_thread;

extern "C" {

int init_robot_initialization(void) {
    if (g_robot_ids_loaded) {
        return 0; // Already initialized
    }
    
    // Load rover config to get robot IDs
    struct RoverConfig configs[MAX_ROVERS];
    int num_rovers = 0;
    if (load_rover_config(ROVERS_FILENAME, configs, &num_rovers) != 0) {
        return -1;
    }
    
    g_robot_ids.clear();
    for (int i = 0; i < num_rovers; i++) {
        g_robot_ids.push_back(configs[i].rover_id);
    }
    g_robot_ids_loaded = true;
    
    // Initialize the central server
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }
    g_central_server = std::make_shared<PathScheduleCentralServer>();

    // Spin the node in a separate thread
    g_spin_thread = std::thread([]() {
        rclcpp::spin(g_central_server);
    });

    return 0;
}

std::vector<int> get_robot_ids(void) {
    // Return cached robot IDs (must be initialized first)
    return g_robot_ids;
}

void cleanup_robot_initialization(void) {
    g_robot_ids.clear();
    g_robot_ids_loaded = false;
    g_central_server.reset();
    rclcpp::shutdown();
    if (g_spin_thread.joinable()) {
        g_spin_thread.join();
    }
}

void dispatch_schedule(int id, const PathSchedulePlan::Goal& goal) {
    if (g_central_server) {
        g_central_server->dispatch_schedule(id, goal);
    }
}

}