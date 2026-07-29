// Copyright 2025 HarvestX Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mg400_plugin/motion_api/command_queue.hpp"

#include <algorithm>

namespace mg400_plugin
{

void CommandQueue::configure(
  const mg400_interface::MotionCommander::SharedPtr commander,
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_if,
  const rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_if,
  const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_if,
  const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_if,
  const rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_if,
  const mg400_interface::MG400Interface::SharedPtr mg400_if)
{
  if (!this->configure_base(
      commander, node_base_if, node_clock_if,
      node_logging_if, node_services_if, node_waitables_if, mg400_if))
  {
    return;
  }

  // Initialize TF manager if not already initialized
  TFManager & tf_manager = TFManager::getInstance();
  if (!tf_manager.isInitialized()) {
    tf_manager.initialize(node_clock_if);
  }

  using namespace std::placeholders;  // NOLINT

  this->action_server_ =
    rclcpp_action::create_server<ActionT>(
    this->node_base_if_,
    this->node_clock_if_,
    this->node_logging_if_,
    this->node_waitable_if_,
    "command_queue",
    std::bind(&CommandQueue::handle_goal, this, _1, _2),
    std::bind(&CommandQueue::handle_cancel, this, _1),
    std::bind(&CommandQueue::handle_accepted, this, _1),
    rcl_action_server_get_default_options(),
    this->node_base_if_->get_default_callback_group());
}

rclcpp_action::GoalResponse CommandQueue::handle_goal(
  const rclcpp_action::GoalUUID & uuid, ActionT::Goal::ConstSharedPtr goal)
{
  if (!this->mg400_interface_->ok()) {
    RCLCPP_ERROR(
      this->node_logging_if_->get_logger(), "MG400 is not connected");
    return rclcpp_action::GoalResponse::REJECT;
  }

  // check if the requested goal is inside the mg400 range
  if (!this->validateTarget(goal->commands)) {
    RCLCPP_ERROR(this->node_logging_if_->get_logger(), "The targets are outside of the range.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  auto lease = this->tryAcquireRegularMotionLease();
  if (!lease) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  GoalReservations::Entry reservation{std::move(*lease), std::monostate{}};
  if (!this->goal_reservations_.reserve(
      mg400_plugin_base::makeActionGoalKey(uuid), std::move(reservation)))
  {
    RCLCPP_ERROR(this->node_logging_if_->get_logger(), "Duplicate Action goal UUID");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CommandQueue::handle_cancel(
  const std::shared_ptr<GoalHandle>/*unused*/)
{
  RCLCPP_INFO(
    this->node_logging_if_->get_logger(), "Received request to cancel goal");
  // There is no verified queue stop operation here. Keep the regular-motion
  // lease until every submitted command has actually finished.
  return rclcpp_action::CancelResponse::ACCEPT;
}

void CommandQueue::handle_accepted(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  auto reservation = this->goal_reservations_.take(
    mg400_plugin_base::makeActionGoalKey(goal_handle->get_goal_id()));
  if (!reservation || !reservation->lease.isCurrent()) {
    RCLCPP_ERROR(this->node_logging_if_->get_logger(), "Accepted goal has no current motion lease");
    auto result = std::make_shared<ActionT::Result>();
    result->result = false;
    goal_handle->abort(result);
    return;
  }
  auto execution = std::make_shared<GoalReservations::Entry>(std::move(*reservation));
  try {
    std::thread{
      [this, goal_handle, execution]() {
        try {
          this->execute(goal_handle, *execution);
        } catch (const std::exception & error) {
          RCLCPP_ERROR(this->node_logging_if_->get_logger(), "%s", error.what());
          auto result = std::make_shared<ActionT::Result>();
          result->result = false;
          goal_handle->abort(result);
        } catch (...) {
          RCLCPP_ERROR(this->node_logging_if_->get_logger(), "Unhandled queue execution error");
          auto result = std::make_shared<ActionT::Result>();
          result->result = false;
          goal_handle->abort(result);
        }
      }}.detach();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      this->node_logging_if_->get_logger(), "Failed to start execution: %s",
      error.what());
    auto result = std::make_shared<ActionT::Result>();
    result->result = false;
    goal_handle->abort(result);
  }
}


void CommandQueue::execute(
  const std::shared_ptr<GoalHandle> goal_handle,
  const GoalReservations::Entry & /*reservation*/)
{
  rclcpp::Rate control_freq(10);  // Hz

  const auto & goal = goal_handle->get_goal();

  auto feedback = std::make_shared<ActionT::Feedback>();
  auto result = std::make_shared<ActionT::Result>();
  result->result = false;

  const auto update_pose_and_angles =
    [&](geometry_msgs::msg::PoseStamped & pose, std::array<double, 4> & angles) -> void
    {
      pose.header.stamp = this->node_clock_if_->get_clock()->now();
      pose.header.frame_id =
        this->mg400_interface_->realtime_tcp_interface->frame_id_prefix + "mg400_origin_link";
      this->mg400_interface_->realtime_tcp_interface->getCurrentEndPose(pose.pose);
      this->mg400_interface_->realtime_tcp_interface->getCurrentJointStates(angles);
    };

  using RobotMode = mg400_msgs::msg::RobotMode;
  using namespace std::chrono_literals;   // NOLINT
  constexpr size_t enable_sync_every = 40;

  const auto append_error_id_to_result = [&]() -> void
    {
      try {
        const std::array<std::vector<int>, 6> error_id =
          this->mg400_interface_->dashboard_commander->getErrorId();
        this->mg400_interface_->dashboard_commander->convertToErrorIdMsg(
          error_id, result->error_id);
      } catch (const std::exception & e) {
        RCLCPP_WARN(this->node_logging_if_->get_logger(), "Failed to get Error ID: %s", e.what());
      }
    };

  const auto wait_until_running_and_enable = [&]() -> bool
    {
      // TODO(anyone): Should calculate timeout with expectation goal time
      const auto timeout = rclcpp::Duration(60s);
      const auto start = this->node_clock_if_->get_clock()->now();
      update_pose_and_angles(feedback->current_pose, feedback->current_angles);

      while (!this->mg400_interface_->realtime_tcp_interface->isRobotMode(RobotMode::RUNNING)) {
        if (this->node_clock_if_->get_clock()->now() - start > rclcpp::Duration(5s)) {
          RCLCPP_ERROR(
            this->node_logging_if_->get_logger(),
            "execution timeout: Robot mode did not become RUNNING.");
          append_error_id_to_result();
          return false;
        }

        if (this->mg400_interface_->realtime_tcp_interface->isRobotMode(RobotMode::ERROR)) {
          RCLCPP_ERROR(
            this->node_logging_if_->get_logger(),
            "Robot Mode Error while checking becoming RUNNING");
          append_error_id_to_result();
          return false;
        }

        control_freq.sleep();
      }

      rclcpp::Time enable_start_time;
      bool enable_mode_confirmed = false;
      const auto enable_duration_threshold = rclcpp::Duration::from_seconds(0.3);

      while (!enable_mode_confirmed) {
        if (!this->mg400_interface_->realtime_tcp_interface->isRobotMode(RobotMode::ENABLE)) {
          enable_start_time = rclcpp::Time();

          if (!this->mg400_interface_->ok()) {
            RCLCPP_ERROR(this->node_logging_if_->get_logger(), "MG400 Connection Error");
            append_error_id_to_result();
            return false;
          }
          if (this->mg400_interface_->realtime_tcp_interface->isRobotMode(RobotMode::ERROR)) {
            RCLCPP_ERROR(
              this->node_logging_if_->get_logger(),
              "Robot Mode Error while checking goal");
            append_error_id_to_result();
            return false;
          }
          if (this->node_clock_if_->get_clock()->now() - start > timeout) {
            RCLCPP_ERROR(this->node_logging_if_->get_logger(), "execution timeout");
            append_error_id_to_result();
            return false;
          }
        } else {
          if (enable_start_time.nanoseconds() == 0) {
            enable_start_time = this->node_clock_if_->get_clock()->now();
          } else if (this->node_clock_if_->get_clock()->now() - enable_start_time >=
            enable_duration_threshold)
          {
            enable_mode_confirmed = true;
          }
        }

        update_pose_and_angles(feedback->current_pose, feedback->current_angles);
        goal_handle->publish_feedback(feedback);
        control_freq.sleep();
      }
      return true;
    };

  geometry_msgs::msg::PoseStamped current_pose;
  std::array<double, 4> current_angles;
  update_pose_and_angles(current_pose, current_angles);

  const auto equal_poses = [](const geometry_msgs::msg::PoseStamped & a,
      const geometry_msgs::msg::PoseStamped & b,
      const double pos_tol = 1e-3,
      const double yaw_tol = 1e-3) -> bool
    {
      const double dx = a.pose.position.x - b.pose.position.x;
      const double dy = a.pose.position.y - b.pose.position.y;
      const double dz = a.pose.position.z - b.pose.position.z;
      const double yaw_a = tf2::getYaw(a.pose.orientation);
      const double yaw_b = tf2::getYaw(b.pose.orientation);
      return std::fabs(dx) <= pos_tol &&
             std::fabs(dy) <= pos_tol &&
             std::fabs(dz) <= pos_tol &&
             std::fabs(yaw_a - yaw_b) <= yaw_tol;
    };

  const auto equal_joints = [](const std::array<double, 4> & a,
      const std::array<double, 4> & b,
      const double tol = 2e-3) -> bool
    {
      for (size_t i = 0; i < a.size(); ++i) {
        if (std::fabs(a[i] - b[i]) > tol) {
          return false;
        }
      }
      return true;
    };

  const auto is_same_as_current = [&](const mg400_msgs::msg::Command & command) -> bool
    {
      geometry_msgs::msg::PoseStamped transformed_pose;
      switch (command.command_type) {
        case mg400_msgs::msg::Command::CT_MOV_J:
          if (!transformPoseToOrigin(command.mov_j_params.pose, transformed_pose)) {
            return false;
          }
          return equal_poses(transformed_pose, current_pose);
        case mg400_msgs::msg::Command::CT_MOV_L:
          if (!transformPoseToOrigin(command.mov_l_params.pose, transformed_pose)) {
            return false;
          }
          return equal_poses(transformed_pose, current_pose);
        case mg400_msgs::msg::Command::CT_JOINT_MOV_J:
          return equal_joints(command.joint_mov_j_params.joint_angles, current_angles);
        case mg400_msgs::msg::Command::CT_MOV_JIO:
          if (!transformPoseToOrigin(command.mov_jio_params.pose, transformed_pose)) {
            return false;
          }
          return equal_poses(transformed_pose, current_pose);
        case mg400_msgs::msg::Command::CT_MOV_LIO:
          if (!transformPoseToOrigin(command.mov_lio_params.pose, transformed_pose)) {
            return false;
          }
          return equal_poses(transformed_pose, current_pose);
        default:
          return false;
      }
    };

  size_t first_non_matching_index = 0;
  while (first_non_matching_index < goal->commands.size()) {
    if (!is_same_as_current(goal->commands[first_non_matching_index])) {
      break;
    }
    ++first_non_matching_index;
  }

  int sent_command_count = 0;
  for (size_t batch_start = first_non_matching_index;
    batch_start < goal->commands.size();
    batch_start += enable_sync_every)
  {
    const size_t batch_end = std::min(batch_start + enable_sync_every, goal->commands.size());
    const int sent_in_batch = this->sendCommand(goal->commands, batch_start, batch_end);
    sent_command_count += sent_in_batch;
    if (sent_in_batch > 0 && !wait_until_running_and_enable()) {
      goal_handle->abort(result);
      return;
    }
  }

  if (sent_command_count == 0) {
    RCLCPP_WARN(
      this->node_logging_if_->get_logger(),
      "No new command was sent to the robot because all targets are same as the current state.");
    result->result = true;
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
    } else {
      goal_handle->succeed(result);
    }
    return;
  }

  RCLCPP_INFO(this->node_logging_if_->get_logger(), "Execution succeeded");
  result->result = true;
  if (goal_handle->is_canceling()) {
    goal_handle->canceled(result);
  } else {
    goal_handle->succeed(result);
  }
}

void CommandQueue::sendMovJ(const mg400_msgs::msg::MovJ & params)
{
  geometry_msgs::msg::PoseStamped tf_pose;
  if (!transformPoseToOrigin(params.pose, tf_pose)) {
    RCLCPP_ERROR(this->node_logging_if_->get_logger(), "Failed to transform pose in sendMovJ.");
    return;
  }

  int8_t speed_j = plugin_utils::clampSpeedJ(
    params.set_speed_j, params.speed_j, this->node_logging_if_->get_logger());
  int8_t acc_j = plugin_utils::clampAccJ(
    params.set_acc_j, params.acc_j, this->node_logging_if_->get_logger());
  int8_t cp = plugin_utils::clampCP(
    params.set_cp, params.cp, this->node_logging_if_->get_logger());

  this->commander_->movJ(
    tf_pose.pose.position.x,
    tf_pose.pose.position.y,
    tf_pose.pose.position.z,
    tf2::getYaw(tf_pose.pose.orientation),
    speed_j, acc_j, cp
  );
}

void CommandQueue::sendMovL(const mg400_msgs::msg::MovL & params)
{
  geometry_msgs::msg::PoseStamped tf_pose;
  if (!transformPoseToOrigin(params.pose, tf_pose)) {
    RCLCPP_ERROR(this->node_logging_if_->get_logger(), "Failed to transform pose in sendMovL.");
    return;
  }

  int8_t speed_l = plugin_utils::clampSpeedL(
    params.set_speed_l, params.speed_l, this->node_logging_if_->get_logger());
  int8_t acc_l = plugin_utils::clampAccL(
    params.set_acc_l, params.acc_l, this->node_logging_if_->get_logger());
  int8_t cp = plugin_utils::clampCP(
    params.set_cp, params.cp, this->node_logging_if_->get_logger());

  this->commander_->movL(
    tf_pose.pose.position.x,
    tf_pose.pose.position.y,
    tf_pose.pose.position.z,
    tf2::getYaw(tf_pose.pose.orientation),
    speed_l, acc_l, cp);
}

void CommandQueue::sendJointMovJ(const mg400_msgs::msg::JointMovJ & params)
{
  int8_t speed_j = plugin_utils::clampSpeedJ(
    params.set_speed_j, params.speed_j, this->node_logging_if_->get_logger());
  int8_t acc_j = plugin_utils::clampAccJ(
    params.set_acc_j, params.acc_j, this->node_logging_if_->get_logger());
  int8_t cp = plugin_utils::clampCP(
    params.set_cp, params.cp, this->node_logging_if_->get_logger());

  this->commander_->jointMovJ(
    params.joint_angles[0],
    params.joint_angles[1],
    params.joint_angles[2],
    params.joint_angles[3],
    speed_j, acc_j, cp);
}

void CommandQueue::sendMovJIO(const mg400_msgs::msg::MovJIO & params)
{
  geometry_msgs::msg::PoseStamped tf_pose;
  if (!transformPoseToOrigin(params.pose, tf_pose)) {
    RCLCPP_ERROR(this->node_logging_if_->get_logger(), "Failed to transform pose in sendMovJIO.");
    return;
  }

  int8_t speed_j = plugin_utils::clampSpeedJ(
    params.set_speed_j, params.speed_j, this->node_logging_if_->get_logger());
  int8_t acc_j = plugin_utils::clampAccJ(
    params.set_acc_j, params.acc_j, this->node_logging_if_->get_logger());

  int8_t cp = -1;  // This means the cp option is always disabled.
  // This is due to a bug in the MG400 TCP/IP API.
  // According to the MG400 manual, MovLIO and MovJIO accepts the cp option,
  // but MG400 firmware version 1.6 does not support it.

  this->commander_->movJIO(
    tf_pose.pose.position.x,
    tf_pose.pose.position.y,
    tf_pose.pose.position.z,
    tf2::getYaw(tf_pose.pose.orientation),
    params.mode,
    params.distance,
    params.index,
    params.status,
    speed_j, acc_j, cp);
}

void CommandQueue::sendMovLIO(const mg400_msgs::msg::MovLIO & params)
{
  geometry_msgs::msg::PoseStamped tf_pose;
  if (!transformPoseToOrigin(params.pose, tf_pose)) {
    RCLCPP_ERROR(this->node_logging_if_->get_logger(), "Failed to transform pose in sendMovLIO.");
    return;
  }

  int8_t speed_l = plugin_utils::clampSpeedL(
    params.set_speed_l, params.speed_l, this->node_logging_if_->get_logger());
  int8_t acc_l = plugin_utils::clampAccL(
    params.set_acc_l, params.acc_l, this->node_logging_if_->get_logger());

  int8_t cp = -1;  // This means the cp option is always disabled.
  // This is due to a bug in the MG400 TCP/IP API.
  // According to the MG400 manual, MovLIO and MovJIO accepts the cp option,
  // but MG400 firmware version 1.6 does not support it.

  this->commander_->movLIO(
    tf_pose.pose.position.x,
    tf_pose.pose.position.y,
    tf_pose.pose.position.z,
    tf2::getYaw(tf_pose.pose.orientation),
    params.mode,
    params.distance,
    params.index,
    params.status,
    speed_l, acc_l, cp);
}

int CommandQueue::sendCommand(
  const std::vector<mg400_msgs::msg::Command> & commands,
  size_t start_index,
  size_t end_index)
{
  int sent_command_count = 0;
  for (size_t i = start_index; i < end_index; ++i) {
    const auto & command = commands[i];
    switch (command.command_type) {
      case mg400_msgs::msg::Command::CT_MOV_J:
        sendMovJ(command.mov_j_params);
        sent_command_count++;
        break;

      case mg400_msgs::msg::Command::CT_MOV_L:
        sendMovL(command.mov_l_params);
        sent_command_count++;
        break;

      case mg400_msgs::msg::Command::CT_JOINT_MOV_J:
        sendJointMovJ(command.joint_mov_j_params);
        sent_command_count++;
        break;

      case mg400_msgs::msg::Command::CT_MOV_JIO:
        sendMovJIO(command.mov_jio_params);
        sent_command_count++;
        break;

      case mg400_msgs::msg::Command::CT_MOV_LIO:
        sendMovLIO(command.mov_lio_params);
        sent_command_count++;
        break;

      default:
        RCLCPP_WARN(
          this->node_logging_if_->get_logger(),
          "Unknown command type: %d", command.command_type);
        break;
    }
  }
  return sent_command_count;
}

bool CommandQueue::transformPoseToOrigin(
  const geometry_msgs::msg::PoseStamped & input_pose,
  geometry_msgs::msg::PoseStamped & output_pose)
{
  try {
    // Use the TFManager to get the transform
    TFManager & tf_manager = TFManager::getInstance();
    auto tf_buffer = tf_manager.getBuffer();
    auto transform = tf_buffer->lookupTransform(
      this->mg400_interface_->realtime_tcp_interface->frame_id_prefix + "mg400_origin_link",
      input_pose.header.frame_id,
      rclcpp::Time(0)
    );
    tf2::doTransform(input_pose, output_pose, transform);
    return true;
  } catch (const tf2::TransformException & e) {
    RCLCPP_ERROR(this->node_logging_if_->get_logger(), "TF transform failed: %s", e.what());
    return false;
  }
}

bool CommandQueue::validateIK(const geometry_msgs::msg::PoseStamped & pose)
{
  geometry_msgs::msg::PoseStamped tf_pose;
  if (!transformPoseToOrigin(pose, tf_pose)) {
    RCLCPP_ERROR(this->node_logging_if_->get_logger(), "Failed to transform pose in validateI.");
    return false;
  }

  std::vector<double> tool_vec = {
    tf_pose.pose.position.x,
    tf_pose.pose.position.y,
    tf_pose.pose.position.z,
    tf2::getYaw(tf_pose.pose.orientation)
  };

  try {
    mg400_ik_util_.InverseKinematics(tool_vec);
    return true;
  } catch (const std::exception & e) {
    return false;
  }
}

bool CommandQueue::validateAngles(const std::array<double, 4> & angles)
{
  return mg400_ik_util_.InMG400Range({angles[0], angles[1], angles[2], angles[3]});
}

bool CommandQueue::validateTarget(const std::vector<mg400_msgs::msg::Command> & commands)
{
  for (size_t i = 0; i < commands.size(); ++i) {
    const auto & command = commands[i];
    switch (command.command_type) {
      case mg400_msgs::msg::Command::CT_MOV_J:
        if (!validateIK(command.mov_j_params.pose)) {
          const auto & pos = command.mov_j_params.pose.pose.position;
          const auto & orient = command.mov_j_params.pose.pose.orientation;
          RCLCPP_ERROR(
            node_logging_if_->get_logger(),
            "Command %zu (MovJ): Target pose unreachable - pos(%.3f, %.3f, %.3f), orient(%.3f, %.3f, %.3f, %.3f)",
            i, pos.x, pos.y, pos.z, orient.x, orient.y, orient.z, orient.w);
          return false;
        }
        break;

      case mg400_msgs::msg::Command::CT_MOV_L:
        if (!validateIK(command.mov_l_params.pose)) {
          const auto & pos = command.mov_l_params.pose.pose.position;
          const auto & orient = command.mov_l_params.pose.pose.orientation;
          RCLCPP_ERROR(
            node_logging_if_->get_logger(),
            "Command %zu (MovL): Target pose unreachable - pos(%.3f, %.3f, %.3f), orient(%.3f, %.3f, %.3f, %.3f)",
            i, pos.x, pos.y, pos.z, orient.x, orient.y, orient.z, orient.w);
          return false;
        }
        break;

      case mg400_msgs::msg::Command::CT_JOINT_MOV_J:
        if (!validateAngles(command.joint_mov_j_params.joint_angles)) {
          const auto & angles = command.joint_mov_j_params.joint_angles;
          RCLCPP_ERROR(
            node_logging_if_->get_logger(),
            "Command %zu (JointMovJ): Joint angles out of range - [%.3f, %.3f, %.3f, %.3f]",
            i, angles[0], angles[1], angles[2], angles[3]);
          return false;
        }
        break;

      case mg400_msgs::msg::Command::CT_MOV_JIO:
        if (!validateIK(command.mov_jio_params.pose)) {
          const auto & pos = command.mov_jio_params.pose.pose.position;
          const auto & orient = command.mov_jio_params.pose.pose.orientation;
          RCLCPP_ERROR(
            node_logging_if_->get_logger(),
            "Command %zu (MovJIO): Target pose unreachable - pos(%.3f, %.3f, %.3f), orient(%.3f, %.3f, %.3f, %.3f)",
            i, pos.x, pos.y, pos.z, orient.x, orient.y, orient.z, orient.w);
          return false;
        }
        break;

      case mg400_msgs::msg::Command::CT_MOV_LIO:
        if (!validateIK(command.mov_lio_params.pose)) {
          const auto & pos = command.mov_lio_params.pose.pose.position;
          const auto & orient = command.mov_lio_params.pose.pose.orientation;
          RCLCPP_ERROR(
            node_logging_if_->get_logger(),
            "Command %zu (MovLIO): Target pose unreachable - pos(%.3f, %.3f, %.3f), orient(%.3f, %.3f, %.3f, %.3f)",
            i, pos.x, pos.y, pos.z, orient.x, orient.y, orient.z, orient.w);
          return false;
        }
        break;

      default:
        RCLCPP_WARN(
          node_logging_if_->get_logger(), "Unknown command type: %d",
          command.command_type);
        break;
    }
  }

  return true;
}

}  // namespace mg400_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  mg400_plugin::CommandQueue,
  mg400_plugin_base::MotionApiPluginBase)
