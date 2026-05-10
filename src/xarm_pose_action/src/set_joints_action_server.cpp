#include <algorithm>
#include <atomic>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "xarm_pose_action/action/set_joints.hpp"

namespace
{
double bounded_or_default(double value, double fallback, double min_value, double max_value)
{
  if (value <= 0.0) {
    return fallback;
  }
  return std::clamp(value, min_value, max_value);
}

int positive_or_default(int value, int fallback)
{
  return value > 0 ? value : fallback;
}

std::string nonempty_or_default(const std::string& value, const std::string& fallback)
{
  return value.empty() ? fallback : value;
}
}  // namespace

class SetJointsActionServer : public rclcpp::Node
{
public:
  using SetJoints = xarm_pose_action::action::SetJoints;
  using GoalHandleSetJoints = rclcpp_action::ServerGoalHandle<SetJoints>;

  explicit SetJointsActionServer(const rclcpp::NodeOptions& options)
      : Node("set_joints_action_server", options)
  {
    default_planning_group_ = get_or_declare_parameter<std::string>("planning_group", "xarm6");
    default_execute_ = get_or_declare_parameter<bool>("execute", true);
    default_velocity_scaling_ = get_or_declare_parameter<double>("velocity_scaling", 0.3);
    default_acceleration_scaling_ = get_or_declare_parameter<double>("acceleration_scaling", 0.1);
    default_planning_time_ = get_or_declare_parameter<double>("planning_time", 5.0);
    default_planning_attempts_ = get_or_declare_parameter<int>("planning_attempts", 5);
    action_name_ = get_or_declare_parameter<std::string>("action_name", "set_joints");

    action_server_ = rclcpp_action::create_server<SetJoints>(
        this,
        action_name_,
        std::bind(&SetJointsActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&SetJointsActionServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&SetJointsActionServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "SetJoints action server ready on '%s'", action_name_.c_str());
  }

private:
  template <typename T>
  T get_or_declare_parameter(const std::string& name, const T& default_value)
  {
    T value{};
    if (has_parameter(name)) {
      get_parameter(name, value);
      return value;
    }
    return declare_parameter<T>(name, default_value);
  }

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID&,
      std::shared_ptr<const SetJoints::Goal> goal)
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    if (goal_active_) {
      RCLCPP_WARN(get_logger(), "Rejecting SetJoints goal because another goal is active");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->joint_values.empty()) {
      RCLCPP_WARN(get_logger(), "Rejecting SetJoints goal with empty joint_values");
      return rclcpp_action::GoalResponse::REJECT;
    }
    goal_active_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleSetJoints>)
  {
    RCLCPP_INFO(get_logger(), "Cancel requested for SetJoints goal");
    cancel_requested_.store(true);
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleSetJoints> goal_handle)
  {
    std::thread{std::bind(&SetJointsActionServer::execute_goal, this, goal_handle)}.detach();
  }

  void publish_feedback(
      const std::shared_ptr<GoalHandleSetJoints>& goal_handle,
      const std::string& state,
      double progress)
  {
    auto feedback = std::make_shared<SetJoints::Feedback>();
    feedback->state = state;
    feedback->progress = progress;
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(get_logger(), "%s", state.c_str());
  }

  void finish_goal()
  {
    cancel_requested_.store(false);
    std::lock_guard<std::mutex> lock(goal_mutex_);
    goal_active_ = false;
  }

  void log_plan(const moveit::planning_interface::MoveGroupInterface::Plan& plan)
  {
    const auto& trajectory = plan.trajectory.joint_trajectory;
    RCLCPP_INFO(get_logger(), "Planned trajectory has %zu points", trajectory.points.size());
    if (trajectory.points.empty()) {
      return;
    }

    const auto& last_point = trajectory.points.back();
    for (std::size_t i = 0; i < trajectory.joint_names.size() && i < last_point.positions.size(); ++i) {
      RCLCPP_INFO(
          get_logger(),
          "Final trajectory %s = %.6f rad",
          trajectory.joint_names[i].c_str(),
          last_point.positions[i]);
    }
  }

  void execute_goal(const std::shared_ptr<GoalHandleSetJoints> goal_handle)
  {
    auto result = std::make_shared<SetJoints::Result>();
    result->success = false;
    result->moveit_error_code = moveit_msgs::msg::MoveItErrorCodes::FAILURE;

    const auto goal = goal_handle->get_goal();
    const std::string planning_group = nonempty_or_default(goal->planning_group, default_planning_group_);
    const bool execute_motion = goal->execute;
    RCLCPP_INFO(get_logger(), "SetJoints goal execute=%s", execute_motion ? "true" : "false");
    const double velocity_scaling = bounded_or_default(
        goal->velocity_scaling, default_velocity_scaling_, 0.001, 1.0);
    const double acceleration_scaling = bounded_or_default(
        goal->acceleration_scaling, default_acceleration_scaling_, 0.001, 1.0);
    const double planning_time = bounded_or_default(goal->planning_time, default_planning_time_, 0.1, 120.0);
    const int planning_attempts = positive_or_default(goal->planning_attempts, default_planning_attempts_);

    try {
      publish_feedback(goal_handle, "Creating MoveIt interface", 0.05);
      auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
          shared_from_this(), planning_group);
      move_group->setMaxVelocityScalingFactor(velocity_scaling);
      move_group->setMaxAccelerationScalingFactor(acceleration_scaling);
      move_group->setPlanningTime(planning_time);
      move_group->setNumPlanningAttempts(planning_attempts);
      move_group->setStartStateToCurrentState();

      const std::vector<std::string> joint_names = move_group->getJointNames();
      if (goal->joint_values.size() != joint_names.size()) {
        result->message = "Joint target size does not match planning group";
        result->moveit_error_code = moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
        RCLCPP_ERROR(
            get_logger(),
            "%s: got %zu values for %zu joints",
            result->message.c_str(),
            goal->joint_values.size(),
            joint_names.size());
        goal_handle->abort(result);
        finish_goal();
        return;
      }

      std::vector<double> joint_targets;
      joint_targets.reserve(joint_names.size());
      for (std::size_t i = 0; i < joint_names.size(); ++i) {
        joint_targets.push_back(goal->joint_values[i]);
        RCLCPP_INFO(get_logger(), "Joint target %s = %.6f rad", joint_names[i].c_str(), goal->joint_values[i]);
      }

      if (goal_handle->is_canceling() || cancel_requested_.load()) {
        result->message = "Canceled before planning";
        goal_handle->canceled(result);
        finish_goal();
        return;
      }

      publish_feedback(goal_handle, "Planning joint target", 0.25);
      const bool target_ok = move_group->setJointValueTarget(joint_names, joint_targets);
      if (!target_ok) {
        result->message = "Joint target rejected by MoveIt";
        result->moveit_error_code = moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
        goal_handle->abort(result);
        finish_goal();
        return;
      }

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      RCLCPP_INFO(get_logger(), "Calling move_group->plan()");
      const auto plan_code = move_group->plan(plan);
      RCLCPP_INFO(get_logger(), "move_group->plan() returned %d", plan_code.val);
      result->moveit_error_code = plan_code.val;
      log_plan(plan);
      if (plan_code != moveit::core::MoveItErrorCode::SUCCESS) {
        result->message = "Joint planning failed";
        goal_handle->abort(result);
        finish_goal();
        return;
      }

      if (!execute_motion) {
        result->success = true;
        result->message = "Planning succeeded; execution disabled";
        publish_feedback(goal_handle, result->message, 1.0);
        goal_handle->succeed(result);
        finish_goal();
        return;
      }

      if (goal_handle->is_canceling() || cancel_requested_.load()) {
        result->message = "Canceled before execution";
        goal_handle->canceled(result);
        finish_goal();
        return;
      }

      publish_feedback(goal_handle, "Executing trajectory", 0.75);
      RCLCPP_INFO(get_logger(), "Calling move_group->execute()");
      const auto exec_code = move_group->execute(plan);
      RCLCPP_INFO(get_logger(), "move_group->execute() returned %d", exec_code.val);
      result->moveit_error_code = exec_code.val;
      result->success = exec_code == moveit::core::MoveItErrorCode::SUCCESS;
      result->message = result->success ? "Motion executed successfully" : "Trajectory execution failed";

      publish_feedback(goal_handle, result->message, result->success ? 1.0 : 0.9);
      if (result->success) {
        goal_handle->succeed(result);
      } else {
        goal_handle->abort(result);
      }
    } catch (const std::exception& ex) {
      result->message = std::string("SetJoints exception: ") + ex.what();
      RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
      goal_handle->abort(result);
    }

    finish_goal();
  }

  rclcpp_action::Server<SetJoints>::SharedPtr action_server_;
  std::mutex goal_mutex_;
  bool goal_active_{false};
  std::atomic_bool cancel_requested_{false};

  std::string default_planning_group_;
  bool default_execute_;
  double default_velocity_scaling_;
  double default_acceleration_scaling_;
  double default_planning_time_;
  int default_planning_attempts_;
  std::string action_name_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SetJointsActionServer>(
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
