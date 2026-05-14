#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <robot_interfaces/srv/add_request.hpp>
#include <robot_interfaces/action/path_schedule_plan.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <cmath>
#include <std_msgs/msg/bool.hpp>

using PathSchedulePlan = robot_interfaces::action::PathSchedulePlan;


const int ROUTE_REQUEST = 3;
const int CRITICAL_BATTERY = 4;
const int LOW_BATTERY = 5;

const int X_DIM_SIZE = 10;
const int Y_DIM_SIZE = 10;

const double TOTAL_JOULE_CAPACITY = 95040.0; //Vnom * 2.0Ah * seconds in an hour, 3600
const double CRITICAL_JOULE_THRESHOLD = 19008.0; //20%ish
const double LOW_JOULE_THRESHOLD = 33264.0; //35%ish

class RoverShim : public rclcpp::Node {
public:

    RoverShim(int rover_id) : Node("rover_shim_" + std::to_string(rover_id)), my_id(rover_id) {
        // Battery subscriber
        battery_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "battery_voltage", 10,
            std::bind(&RoverShim::battery_callback, this, std::placeholders::_1)
        );

        // action client for PathSchedulePlan to track when goal is complete
        action_client_ = rclcpp_action::create_client<PathSchedulePlan>(
            this, "execute_path_schedule"
        );

        // Odom subscriber for getting position
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&RoverShim::odom_callback, this, std::placeholders::_1)
        );

        // Service client for adding requests
        service_client_ = this->create_client<robot_interfaces::srv::AddRequest>("add_request");
    initial_request_timer_ = this->create_wall_timer(
		std::chrono::seconds(2),
            	[this]() {
                if (!odom_received_) {
                    return; // keep waiting, don't cancel
                }
                this->make_request(this->get_req_type());
                initial_request_timer_->cancel();
            }
        );

        route_done_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "rover_" + std::to_string(my_id) + "_route_done", 1,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (msg->data) {
                std::this_thread::sleep_for(std::chrono::seconds(10));
                make_request(get_req_type());
                }
            });
    }

    int get_req_type() {
        if (curr_joules < CRITICAL_JOULE_THRESHOLD) {
            return CRITICAL_BATTERY;
        } else if (curr_joules < LOW_JOULE_THRESHOLD) {
            return LOW_BATTERY;
        }
        return ROUTE_REQUEST;
    }

    void make_request(int req_flag) {
        if (!service_client_->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Service not available");
            return;
        }

        auto request = std::make_shared<robot_interfaces::srv::AddRequest::Request>();

        // Use odom position tracking
        request->rover_id = my_id;
        request->req_flag = req_flag;

        // Clamp coordinates to world dimensions before sending
        double x = curr_x;
        double y = curr_y;
        double x_max = X_DIM_SIZE + 10.0;
        double y_max = Y_DIM_SIZE + 10.0;
        if (x < -10.0) x = -10.0;
        if (x > x_max) x = x_max;
        if (y < -10.0) y = -10.0;
        if (y > y_max) y = y_max;
        request->x = x;
        request->y = y;

        request->yaw_rads = curr_yaw;

        // Clamp battery level to reasonable bounds
        double battery = curr_joules;
        if (battery < 0.0) battery = 0.0;
        if (battery > 200000.0) battery = 200000.0;
        request->battery_level = battery;

        request->timestamp = this->now().seconds();

        auto result = service_client_->async_send_request(request);
    }

private:

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr route_done_sub_;
    double curr_x = 0.0;
    double curr_y = 0.0;
    double curr_yaw = 0.0;
    double odom_start_x_ = 0.0;
    double odom_start_y_ = 0.0;
    double odom_start_yaw_ = 0.0;
    bool odom_received_ = false;
    bool first_odom_ = true;
    int my_id;
    int odom_skip_count_ = 0;
    double curr_joules = TOTAL_JOULE_CAPACITY; //default to full
    static constexpr double MAX_CAPACITY_AH = 2.2; // 2200mAh is 2.2Ah

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr battery_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp_action::Client<PathSchedulePlan>::SharedPtr action_client_;
    rclcpp::Client<robot_interfaces::srv::AddRequest>::SharedPtr service_client_;
    rclcpp::TimerBase::SharedPtr initial_request_timer_;

    void battery_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        double voltage = msg->data;
        //MAX POWER FOR DEMOS
        curr_joules = TOTAL_JOULE_CAPACITY;//voltage * MAX_CAPACITY_AH * 3600.0; // Convert to joules
    }
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        //forcing it to clamp to diff. Should be done dynamically.
        //but this is just for the demo

        // quaternion to yaw
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        
        // Yaw from quaternion
        double siny_cosp = 2.0 * (qw * qz + qx * qy);
        double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
        if (first_odom_) {
            if (++odom_skip_count_ < 5) return;  
        odom_start_x_ = msg->pose.pose.position.x;
        odom_start_y_ = msg->pose.pose.position.y;
            odom_start_yaw_ = std::atan2(siny_cosp, cosy_cosp);
        first_odom_ = false;
        odom_received_ = true;
    }
    curr_x = 0 + (msg->pose.pose.position.x - odom_start_x_);
curr_y = 0 + (msg->pose.pose.position.y - odom_start_y_);
        double raw_yaw = std::atan2(siny_cosp, cosy_cosp);
    curr_yaw = raw_yaw - odom_start_yaw_;
    }
};

int main(int argc, char** argv) {
    if (argc < 2) { return 1; }
    int rover_id = std::atoi(argv[1]);
    
    //initialize the ros
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RoverShim>(rover_id);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
