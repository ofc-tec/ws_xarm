#include <algorithm>
#include <atomic>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "xarm_pose_action/action/set_pose.hpp"

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

class SetPoseActionServer : public rclcpp::Node
{
public:
  using SetPose = xarm_pose_action::action::SetPose;
  using GoalHandleSetPose = rclcpp_action::ServerGoalHandle<SetPose>;

  explicit SetPoseActionServer(const rclcpp::NodeOptions& options)
      : Node("set_pose_action_server", options)
  {
    default_planning_group_ = get_or_declare_parameter<std::string>("planning_group", "xarm6");
    default_end_effector_link_ = get_or_declare_parameter<std::string>("end_effector_link", "");
    default_execute_ = get_or_declare_parameter<bool>("execute", true);
    default_cartesian_ = get_or_declare_parameter<bool>("cartesian", false);
    default_velocity_scaling_ = get_or_declare_parameter<double>("velocity_scaling", 0.3);
    default_acceleration_scaling_ = get_or_declare_parameter<double>("acceleration_scaling", 0.1);
    default_position_tolerance_ = get_or_declare_parameter<double>("position_tolerance", 0.01);
    default_orientation_tolerance_ = get_or_declare_parameter<double>("orientation_tolerance", 0.05);
    default_planning_time_ = get_or_declare_parameter<double>("planning_time", 5.0);
    default_planning_attempts_ = get_or_declare_parameter<int>("planning_attempts", 5);
    cartesian_eef_step_ = get_or_declare_parameter<double>("cartesian_eef_step", 0.005);
    jump_threshold_ = get_or_declare_parameter<double>("jump_threshold", 0.0);
    action_name_ = get_or_declare_parameter<std::string>("action_name", "set_pose");

  }

  void initialize()
  {
    RCLCPP_INFO(
        get_logger(),
        "Creating MoveIt interface for planning group '%s'",
        default_planning_group_.c_str());
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), default_planning_group_);
    if (!default_end_effector_link_.empty()) {
      move_group_->setEndEffectorLink(default_end_effector_link_);
    }
    RCLCPP_INFO(
        get_logger(),
        "MoveIt interface ready for planning group '%s', planning frame '%s', eef '%s'",
        default_planning_group_.c_str(),
        move_group_->getPlanningFrame().c_str(),
        move_group_->getEndEffectorLink().c_str());

    action_server_ = rclcpp_action::create_server<SetPose>(
        this,
        action_name_,
        std::bind(&SetPoseActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&SetPoseActionServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&SetPoseActionServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "SetPose action server ready on '%s'", action_name_.c_str());
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
      std::shared_ptr<const SetPose::Goal> goal)
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    if (goal_active_) {
      RCLCPP_WARN(get_logger(), "Rejecting SetPose goal because another goal is active");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->target_pose.header.frame_id.empty()) {
      RCLCPP_WARN(get_logger(), "Rejecting SetPose goal with empty target_pose.header.frame_id");
      return rclcpp_action::GoalResponse::REJECT;
    }
    goal_active_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleSetPose>)
  {
    RCLCPP_INFO(get_logger(), "Cancel requested for SetPose goal");
    cancel_requested_.store(true);
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleSetPose> goal_handle)
  {
    std::thread{std::bind(&SetPoseActionServer::execute_goal, this, goal_handle)}.detach();
  }

  void publish_feedback(
      const std::shared_ptr<GoalHandleSetPose>& goal_handle,
      const std::string& state,
      double progress)
  {
    auto feedback = std::make_shared<SetPose::Feedback>();
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

  bool target_reached(
      const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& move_group,
      const std::string& end_effector_link,
      const geometry_msgs::msg::Pose& target_pose,
      double position_tolerance,
      double angular_tolerance)
  {
    const auto current_pose_stamped = move_group->getCurrentPose(end_effector_link);
    const auto& current_pose = current_pose_stamped.pose;

    const double dx = current_pose.position.x - target_pose.position.x;
    const double dy = current_pose.position.y - target_pose.position.y;
    const double dz = current_pose.position.z - target_pose.position.z;
    const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    const double dot =
        current_pose.orientation.x * target_pose.orientation.x +
        current_pose.orientation.y * target_pose.orientation.y +
        current_pose.orientation.z * target_pose.orientation.z +
        current_pose.orientation.w * target_pose.orientation.w;
    const double clamped_dot = std::clamp(std::fabs(dot), 0.0, 1.0);
    const double angular_distance = 2.0 * std::acos(clamped_dot);

    RCLCPP_INFO(
        get_logger(),
        "Final pose error: position %.4f m, angular %.4f rad",
        distance,
        angular_distance);
    return distance <= position_tolerance && angular_distance <= angular_tolerance;
  }

  void execute_goal(const std::shared_ptr<GoalHandleSetPose> goal_handle)
  {
    auto result = std::make_shared<SetPose::Result>();
    result->success = false;
    result->moveit_error_code = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    result->cartesian_fraction = 0.0;

    const auto goal = goal_handle->get_goal();
    const std::string planning_group = nonempty_or_default(goal->planning_group, default_planning_group_);
    const std::string end_effector_link = nonempty_or_default(goal->end_effector_link, default_end_effector_link_);
    const bool execute_motion = goal->execute;
    const bool cartesian = goal->cartesian;
    const bool position_only = goal->position_only;
    const double velocity_scaling = bounded_or_default(
        goal->velocity_scaling, default_velocity_scaling_, 0.001, 1.0);
    const double acceleration_scaling = bounded_or_default(
        goal->acceleration_scaling, default_acceleration_scaling_, 0.001, 1.0);
    const double position_tolerance = bounded_or_default(
        goal->position_tolerance, default_position_tolerance_, 0.0001, 1.0);
    const double orientation_tolerance = bounded_or_default(
        goal->orientation_tolerance, default_orientation_tolerance_, 0.0001, 3.14159);
    const double planning_time = bounded_or_default(goal->planning_time, default_planning_time_, 0.1, 120.0);
    const int planning_attempts = positive_or_default(goal->planning_attempts, default_planning_attempts_);

    try {
      std::lock_guard<std::mutex> commander_lock(commander_mutex_);
      publish_feedback(goal_handle, "Using MoveIt interface", 0.05);
      if (planning_group != default_planning_group_) {
        result->message = "SetPose server was initialized for a different planning group";
        result->moveit_error_code = moveit_msgs::msg::MoveItErrorCodes::INVALID_GROUP_NAME;
        RCLCPP_ERROR(
            get_logger(),
            "%s: requested '%s', available '%s'",
            result->message.c_str(),
            planning_group.c_str(),
            default_planning_group_.c_str());
        goal_handle->abort(result);
        finish_goal();
        return;
      }
      auto move_group = move_group_;
      move_group->setMaxVelocityScalingFactor(velocity_scaling);
      move_group->setMaxAccelerationScalingFactor(acceleration_scaling);
      move_group->setPlanningTime(planning_time);
      move_group->setNumPlanningAttempts(planning_attempts);
      move_group->setStartStateToCurrentState();
      move_group->setPoseReferenceFrame(goal->target_pose.header.frame_id);
      move_group->setGoalPositionTolerance(position_tolerance);
      move_group->setGoalOrientationTolerance(orientation_tolerance);

      if (!end_effector_link.empty()) {
        move_group->setEndEffectorLink(end_effector_link);
      }

      RCLCPP_INFO(
          get_logger(),
          "Planning group '%s', planning frame '%s', eef '%s'",
          planning_group.c_str(),
          move_group->getPlanningFrame().c_str(),
          move_group->getEndEffectorLink().c_str());
      RCLCPP_INFO(
          get_logger(),
          "Goal tolerances: position %.4f m, orientation %.4f rad",
          position_tolerance,
          orientation_tolerance);

      if (goal_handle->is_canceling() || cancel_requested_.load()) {
        result->message = "Canceled before planning";
        goal_handle->canceled(result);
        finish_goal();
        return;
      }

      publish_feedback(goal_handle, cartesian ? "Planning Cartesian path" : "Planning pose target", 0.25);
      moveit::core::MoveItErrorCode plan_code;
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      moveit_msgs::msg::RobotTrajectory cartesian_trajectory;

      if (cartesian) {
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(goal->target_pose.pose);
        result->cartesian_fraction = move_group->computeCartesianPath(
            waypoints, cartesian_eef_step_, jump_threshold_, cartesian_trajectory);
        plan_code = result->cartesian_fraction >= 0.9
                        ? moveit::core::MoveItErrorCode::SUCCESS
                        : moveit::core::MoveItErrorCode::PLANNING_FAILED;
      } else if (position_only) {
        const auto& position = goal->target_pose.pose.position;
        RCLCPP_INFO(
            get_logger(),
            "Calling move_group->setPositionTarget(%.3f, %.3f, %.3f)",
            position.x,
            position.y,
            position.z);
        const bool target_ok = move_group->setPositionTarget(position.x, position.y, position.z, end_effector_link);
        RCLCPP_INFO(get_logger(), "move_group->setPositionTarget() returned %s", target_ok ? "true" : "false");
        if (!target_ok) {
          result->message = "Position target rejected by MoveIt";
          result->moveit_error_code = moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
          goal_handle->abort(result);
          finish_goal();
          return;
        }
        RCLCPP_INFO(get_logger(), "Calling move_group->plan()");
        plan_code = move_group->plan(plan);
        RCLCPP_INFO(get_logger(), "move_group->plan() returned %d", plan_code.val);
      } else {
        RCLCPP_INFO(get_logger(), "Calling move_group->setPoseTarget()");
        const bool target_ok = move_group->setPoseTarget(goal->target_pose.pose, end_effector_link);
        RCLCPP_INFO(get_logger(), "move_group->setPoseTarget() returned %s", target_ok ? "true" : "false");
        if (!target_ok) {
          result->message = "Target pose rejected by MoveIt";
          result->moveit_error_code = moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
          goal_handle->abort(result);
          finish_goal();
          return;
        }
        RCLCPP_INFO(get_logger(), "Calling move_group->plan()");
        plan_code = move_group->plan(plan);
        RCLCPP_INFO(get_logger(), "move_group->plan() returned %d", plan_code.val);
      }

      result->moveit_error_code = plan_code.val;
      if (plan_code != moveit::core::MoveItErrorCode::SUCCESS) {
        result->message = cartesian ? "Cartesian planning failed" : "Pose planning failed";
        goal_handle->abort(result);
        finish_goal();
        return;
      }

      if (goal_handle->is_canceling() || cancel_requested_.load()) {
        result->message = "Canceled after planning";
        goal_handle->canceled(result);
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
      moveit::core::MoveItErrorCode exec_code;
      if (cartesian) {
        exec_code = move_group->execute(cartesian_trajectory);
      } else {
        RCLCPP_INFO(get_logger(), "Calling move_group->move()");
        exec_code = move_group->move();
        RCLCPP_INFO(get_logger(), "move_group->move() returned %d", exec_code.val);
      }
      result->moveit_error_code = exec_code.val;
      result->success = exec_code == moveit::core::MoveItErrorCode::SUCCESS;
      if (!result->success && target_reached(
          move_group,
          end_effector_link,
          goal->target_pose.pose,
          position_tolerance,
          position_only ? 3.15 : orientation_tolerance)) {
        RCLCPP_WARN(get_logger(), "MoveIt reported execution failure, but end effector is within target tolerance");
        result->success = true;
        result->moveit_error_code = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
        result->message = "Motion reached target despite execution status";
      } else {
        result->message = result->success ? "Motion executed successfully" : "Trajectory execution failed";
      }

      publish_feedback(goal_handle, result->message, result->success ? 1.0 : 0.9);
      if (result->success) {
        goal_handle->succeed(result);
      } else {
        goal_handle->abort(result);
      }
    } catch (const std::exception& ex) {
      result->message = std::string("SetPose exception: ") + ex.what();
      RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
      goal_handle->abort(result);
    }

    finish_goal();
  }

  rclcpp_action::Server<SetPose>::SharedPtr action_server_;
  std::mutex goal_mutex_;
  std::mutex commander_mutex_;
  bool goal_active_{false};
  std::atomic_bool cancel_requested_{false};
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  std::string default_planning_group_;
  std::string default_end_effector_link_;
  bool default_execute_;
  bool default_cartesian_;
  double default_velocity_scaling_;
  double default_acceleration_scaling_;
  double default_position_tolerance_;
  double default_orientation_tolerance_;
  double default_planning_time_;
  int default_planning_attempts_;
  double cartesian_eef_step_;
  double jump_threshold_;
  std::string action_name_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SetPoseActionServer>(
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  node->initialize();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
