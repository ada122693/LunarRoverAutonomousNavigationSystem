#include <memory>
#include <chrono>
#include <thread>
#include <cmath>
#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <robot_interfaces/action/path_schedule_plan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>

using namespace std::chrono_literals;
using PathSchedulePlan = robot_interfaces::action::PathSchedulePlan;
using GoalHandlePSP = rclcpp_action::ServerGoalHandle<PathSchedulePlan>;

static constexpr double V_NOMINAL = 0.15;
static constexpr double ARC_RATE_HZ = 20.0;
static constexpr double STOP_COAST_S = 0.08;
static constexpr double HEADING_THRESH = 0.015;
static constexpr double DECIMATE_DIST = 0.05;
static constexpr double DECIMATE_HEADING = 0.05;
static constexpr double TURN_TUNING = 1.0;
namespace bt_nodes
{

static double yaw_from_pose(const geometry_msgs::msg::Pose & pose)
{
  const auto & q = pose.orientation;
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

class PathSchedulePlanServer : public rclcpp::Node
{
public:
  PathSchedulePlanServer()
  : Node("path_schedule_plan_server")
  {
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    auto odom_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions odom_opts;
    odom_opts.callback_group = odom_group;

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            curr_x_ = msg->pose.pose.position.x;
            curr_y_ = msg->pose.pose.position.y;
            auto & q = msg->pose.pose.orientation;
            curr_yaw_ = std::atan2(2.0*(q.w*q.z + q.x*q.y),
                                   1.0 - 2.0*(q.y*q.y + q.z*q.z));
        }, odom_opts);

    action_server_ = rclcpp_action::create_server<PathSchedulePlan>(
      this,
      "/execute_path_schedule",
      std::bind(&PathSchedulePlanServer::handle_goal,     this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PathSchedulePlanServer::handle_cancel,   this, std::placeholders::_1),
      std::bind(&PathSchedulePlanServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Haphazard ready on /execute_path_schedule");
  }

private:
  rclcpp_action::Server<PathSchedulePlan>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  double curr_x_ = 0.0, curr_y_ = 0.0, curr_yaw_ = 0.0;
  std::atomic<bool> holding_{false};

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const PathSchedulePlan::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(),
      "Received goal — dispatch: %.2f  segments: %zu  waits: %zu",
      goal->dispatch_time, goal->segments.size(), goal->wait_durations.size());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandlePSP>)
  {
    RCLCPP_INFO(this->get_logger(), "Cancel received — stopping robot");
    holding_ = false;
    publish_stop();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandlePSP> goal_handle)
  {
    std::thread{std::bind(&PathSchedulePlanServer::execute, this,
                          std::placeholders::_1), goal_handle}.detach();
  }

  void publish_stop()
  {
    cmd_pub_->publish(geometry_msgs::msg::Twist{});
    std::this_thread::sleep_for(std::chrono::duration<double>(STOP_COAST_S));
  }

  void execute_segment(const nav_msgs::msg::Path & path)
{
    const auto & poses = path.poses;
    if (poses.empty()) return;

    auto period = std::chrono::duration<double>(1.0 / ARC_RATE_HZ);
    size_t i = 1;

    while (i < poses.size()) {
        const auto & target = poses[i].pose;
        double threshold = 0.08;

        double dx = target.position.x - curr_x_;
        double dy = target.position.y - curr_y_;
        double dist = std::hypot(dx, dy);

        // skip waypoints we've already passed
        if (dist < threshold) {
            i++;
            continue;
        }

        double target_yaw = std::atan2(dy, dx);
        double yaw_err = target_yaw - curr_yaw_;
        yaw_err = std::atan2(std::sin(yaw_err), std::cos(yaw_err));

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = std::min(V_NOMINAL, dist * 1.5) * (1.0 - std::min(1.0, std::abs(yaw_err) / 1.0));
        if (std::abs(yaw_err) < 0.15) {
            cmd.angular.z = 0.0;
        } else {
            cmd.angular.z = std::max(-1.5, std::min(1.5, yaw_err * 1.5));
        }
        cmd_pub_->publish(cmd);
        std::this_thread::sleep_for(period);

    }
    publish_stop();
}
  void execute(const std::shared_ptr<GoalHandlePSP> goal_handle)
  {
    holding_ = false;
    auto goal     = goal_handle->get_goal();
    auto result   = std::make_shared<PathSchedulePlan::Result>();
    auto feedback = std::make_shared<PathSchedulePlan::Feedback>();

    {
      auto now_sec = std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count();
      double delay = goal->dispatch_time - now_sec;
      if (delay > 0.0) {
        RCLCPP_INFO(this->get_logger(), "Sleeping %.3fs until dispatch", delay);
        std::this_thread::sleep_for(std::chrono::duration<double>(delay));
      }
    }

    size_t num_segments = goal->segments.size();
    size_t num_waits    = goal->wait_durations.size();

    for (size_t i = 0; i < num_segments; ++i) {
      if (goal_handle->is_canceling()) {
        publish_stop();
        result->success = false;
        goal_handle->canceled(result);
        return;
      }

      if (i < num_waits && goal->wait_durations[i] > 0.0) {
        RCLCPP_INFO(this->get_logger(),
          "Segment %zu: holding %.3fs", i, goal->wait_durations[i]);
        std::this_thread::sleep_for(std::chrono::duration<double>(goal->wait_durations[i]));
      }

      feedback->current_segment = static_cast<uint32_t>(i);
      goal_handle->publish_feedback(feedback);

      RCLCPP_INFO(this->get_logger(), "Executing segment %zu (%zu poses)",
        i, goal->segments[i].poses.size());

      execute_segment(goal->segments[i]);
    }

    publish_stop();
    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Path schedule complete");

    double target_yaw = curr_yaw_;
    auto hold_period = std::chrono::duration<double>(1.0 / ARC_RATE_HZ);
    holding_ = true;
    while (holding_) {
      geometry_msgs::msg::Twist correction{};
      double yaw_err = target_yaw - curr_yaw_;
      yaw_err = std::atan2(std::sin(yaw_err), std::cos(yaw_err));
      correction.angular.z = yaw_err * 2.0;
      cmd_pub_->publish(correction);
      std::this_thread::sleep_for(hold_period);
    }
  }
};

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<bt_nodes::PathSchedulePlanServer>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
