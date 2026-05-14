#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include "robot_interfaces/action/path_schedule_plan.hpp"

#include "io/platform_specific_instructions_converter.h"
#include "io/io_send_instructions.h"
#include "rover_action_client.h"

extern "C" int send_instructions(struct PlatformSpecificInstructions* instructions, double actual_dispatch_time) {
    void* action_client = get_rover_action_client(instructions->rover_id);
    if (action_client == NULL) {
        fprintf(stderr, "[SEND] Failed to get action client for rover %d\n", instructions->rover_id);
        return -1;
    }

    auto* client = static_cast<rclcpp_action::Client<robot_interfaces::action::PathSchedulePlan>*>(action_client);
    fprintf(stderr, "[SEND] Waiting for action server for rover %d...\n", instructions->rover_id);
    if (!client->wait_for_action_server(std::chrono::seconds(5))) {
        fprintf(stderr, "[SEND] Action server not available for rover %d\n", instructions->rover_id);
        return -1;
    }
    fprintf(stderr, "[SEND] Action server available for rover %d\n", instructions->rover_id);

    // give the robot the time for it to dispatch
    instructions->goal.dispatch_time = actual_dispatch_time;
    fprintf(stderr, "[SEND] Sending goal to rover %d with dispatch_time: %.2f, %zu segments\n",
            instructions->rover_id, actual_dispatch_time, instructions->goal.segments.size());

    // send package
    auto send_goal_options = rclcpp_action::Client<robot_interfaces::action::PathSchedulePlan>::SendGoalOptions();
    int rover_id = instructions->rover_id;
    send_goal_options.result_callback = 
    [rover_id](const rclcpp_action::ClientGoalHandle<robot_interfaces::action::PathSchedulePlan>::WrappedResult& result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            auto* pub = get_rover_done_publisher(rover_id);
            if (pub) {
                auto msg = std_msgs::msg::Bool();
                msg.data = true;
                pub->publish(msg);
            }
        }
    };
    client->async_send_goal(instructions->goal, send_goal_options);
    fprintf(stderr, "[SEND] Goal sent to rover %d\n", instructions->rover_id);

    return 0;
}
