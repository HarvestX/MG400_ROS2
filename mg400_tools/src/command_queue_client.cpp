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

#include <mg400_tools/command_queue_client.hpp>

namespace mg400_tools
{

CommandQueueClient::CommandQueueClient(const rclcpp::NodeOptions options)
: rclcpp::Node("command_queue_client", options)
{
  this->param_name_command_config_path_ = "command_config_path";
  auto default_command_config_path =
    ament_index_cpp::get_package_share_directory("mg400_tools") + "/config/commands.yaml";
  this->declare_parameter<std::string>(
    this->param_name_command_config_path_, default_command_config_path);

  using namespace std::chrono_literals;   // NOLINT;
  std::string action_name = std::string(this->get_namespace()) + "/command_queue";
  RCLCPP_INFO(this->get_logger(), "Action server: %s", action_name.c_str());
  this->client_ = rclcpp_action::create_client<CommandQueue>(this, action_name);
  this->init_timer_ = this->create_wall_timer(0s, std::bind(&CommandQueueClient::onInit, this));
}

void CommandQueueClient::onInit()
{
  this->init_timer_->cancel();
  if (!waitForActionServer()) {
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Action server available.");
  this->sendGoal();
}

bool CommandQueueClient::waitForActionServer()
{
  using namespace std::chrono_literals;   // NOLINT
  if (!this->client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available");
    rclcpp::shutdown();
    return false;
  }
  return true;
}

YAML::Node CommandQueueClient::loadConfig()
{
  std::string command_config_path =
    this->get_parameter(this->param_name_command_config_path_).as_string();
  RCLCPP_INFO(this->get_logger(), "Loaded command config: %s", command_config_path.c_str());
  return YAML::LoadFile(command_config_path);
}

void CommandQueueClient::sendGoal()
{
  YAML::Node config = loadConfig();
  auto goal_msg = CommandQueue::Goal();

  parseCommandsFromConfig(config, goal_msg);
  sendGoalWithCallbacks(goal_msg);
}

void CommandQueueClient::parseCommandsFromConfig(
  const YAML::Node & config,
  CommandQueue::Goal & goal_msg)
{
  for (const auto & cmd_node : config["commands"]) {
    mg400_msgs::msg::Command cmd;
    std::string type = cmd_node["type"].as<std::string>();

    if (!parseCommand(cmd_node, type, cmd)) {
      continue;
    }

    goal_msg.commands.emplace_back(cmd);
  }
}

bool CommandQueueClient::parseCommand(
  const YAML::Node & cmd_node, const std::string & type,
  mg400_msgs::msg::Command & cmd)
{
  if (type == "JointMovJ") {
    return parseJointMovJCommand(cmd_node, cmd);
  } else if (type == "MovJ") {
    return parseMovJCommand(cmd_node, cmd);
  } else if (type == "MovL") {
    return parseMovLCommand(cmd_node, cmd);
  } else if (type == "MovJIO") {
    return parseMovJIOCommand(cmd_node, cmd);
  } else if (type == "MovLIO") {
    return parseMovLIOCommand(cmd_node, cmd);
  } else {
    RCLCPP_WARN(this->get_logger(), "Unknown command type: %s", type.c_str());
    return false;
  }
}

bool CommandQueueClient::parseJointMovJCommand(
  const YAML::Node & cmd_node,
  mg400_msgs::msg::Command & cmd)
{
  cmd.command_type = cmd.CT_JOINT_MOV_J;

  cmd.joint_mov_j_params.set_speed_j = true;
  cmd.joint_mov_j_params.speed_j = static_cast<uint8_t>(cmd_node["speed_j"].as<int>());
  cmd.joint_mov_j_params.set_acc_j = true;
  cmd.joint_mov_j_params.acc_j = static_cast<uint8_t>(cmd_node["acc_j"].as<int>());
  cmd.joint_mov_j_params.set_cp = true;
  cmd.joint_mov_j_params.cp = static_cast<uint8_t>(cmd_node["cp"].as<int>());

  auto angles = cmd_node["angles_deg"];
  for (size_t i = 0; i < 4; ++i) {
    cmd.joint_mov_j_params.joint_angles[i] = angles[i].as<double>() * M_PI / 180.0;
    RCLCPP_INFO(this->get_logger(), "%f", cmd.joint_mov_j_params.joint_angles[i]);
  }

  return true;
}

bool CommandQueueClient::parseMovJCommand(
  const YAML::Node & cmd_node,
  mg400_msgs::msg::Command & cmd)
{
  cmd.command_type = cmd.CT_MOV_J;

  cmd.mov_j_params.set_speed_j = true;
  cmd.mov_j_params.speed_j = static_cast<uint8_t>(cmd_node["speed_j"].as<int>());
  cmd.mov_j_params.set_acc_j = true;
  cmd.mov_j_params.acc_j = static_cast<uint8_t>(cmd_node["acc_j"].as<int>());
  cmd.mov_j_params.set_cp = true;
  cmd.mov_j_params.cp = static_cast<uint8_t>(cmd_node["cp"].as<int>());

  setPoseFromConfig(cmd_node, cmd.mov_j_params.pose);

  return true;
}

bool CommandQueueClient::parseMovLCommand(
  const YAML::Node & cmd_node,
  mg400_msgs::msg::Command & cmd)
{
  cmd.command_type = cmd.CT_MOV_L;

  cmd.mov_l_params.set_speed_l = true;
  cmd.mov_l_params.speed_l = static_cast<uint8_t>(cmd_node["speed_l"].as<int>());
  cmd.mov_l_params.set_acc_l = true;
  cmd.mov_l_params.acc_l = static_cast<uint8_t>(cmd_node["acc_l"].as<int>());
  cmd.mov_l_params.set_cp = true;
  cmd.mov_l_params.cp = static_cast<uint8_t>(cmd_node["cp"].as<int>());

  setPoseFromConfig(cmd_node, cmd.mov_l_params.pose);

  return true;
}

bool CommandQueueClient::parseMovJIOCommand(
  const YAML::Node & cmd_node,
  mg400_msgs::msg::Command & cmd)
{
  cmd.command_type = cmd.CT_MOV_JIO;

  cmd.mov_jio_params.set_speed_j = true;
  cmd.mov_jio_params.speed_j = static_cast<uint8_t>(cmd_node["speed_j"].as<int>());
  cmd.mov_jio_params.set_acc_j = true;
  cmd.mov_jio_params.acc_j = static_cast<uint8_t>(cmd_node["acc_j"].as<int>());

  // cmd.mov_lio_params.set_cp = false;
  // cmd.mov_lio_params.cp = static_cast<uint8_t>(cmd_node["cp"].as<int>());
  // ----------
  // The cp options are temporarily commented out for the following reason:
  // According to the MG400 manual, MovLIO and MovJIO accept the cp option,
  // but MG400 firmware version 1.6 does not support it.

  setPoseFromConfig(cmd_node, cmd.mov_jio_params.pose);
  setIOParameters(
    cmd_node, cmd.mov_jio_params.mode, cmd.mov_jio_params.distance,
    cmd.mov_jio_params.index, cmd.mov_jio_params.status);

  return true;
}

bool CommandQueueClient::parseMovLIOCommand(
  const YAML::Node & cmd_node,
  mg400_msgs::msg::Command & cmd)
{
  cmd.command_type = cmd.CT_MOV_LIO;

  cmd.mov_lio_params.set_speed_l = true;
  cmd.mov_lio_params.speed_l = static_cast<uint8_t>(cmd_node["speed_l"].as<int>());
  cmd.mov_lio_params.set_acc_l = true;
  cmd.mov_lio_params.acc_l = static_cast<uint8_t>(cmd_node["acc_l"].as<int>());

  // cmd.mov_lio_params.set_cp = false;
  // cmd.mov_lio_params.cp = static_cast<uint8_t>(cmd_node["cp"].as<int>());
  // ----------
  // The cp options are temporarily commented out for the following reason:
  // According to the MG400 manual, MovLIO and MovJIO accept the cp option,
  // but MG400 firmware version 1.6 does not support it.

  setPoseFromConfig(cmd_node, cmd.mov_lio_params.pose);
  setIOParameters(
    cmd_node, cmd.mov_lio_params.mode, cmd.mov_lio_params.distance,
    cmd.mov_lio_params.index, cmd.mov_lio_params.status);

  return true;
}

void CommandQueueClient::setPoseFromConfig(
  const YAML::Node & cmd_node,
  geometry_msgs::msg::PoseStamped & pose)
{
  const auto & pos = cmd_node["pos_m"];
  const auto & yaw = cmd_node["yaw_deg"].as<double>() * M_PI / 180.0;
  const auto & frame_id = cmd_node["frame_id"].as<std::string>();

  pose.header.stamp = this->get_clock()->now();
  pose.header.frame_id = frame_id;
  pose.pose.position.x = pos[0].as<double>();
  pose.pose.position.y = pos[1].as<double>();
  pose.pose.position.z = pos[2].as<double>();

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.orientation.w = q.w();
}

void CommandQueueClient::setIOParameters(
  const YAML::Node & cmd_node, mg400_msgs::msg::DistanceMode & mode, int & distance,
  mg400_msgs::msg::DOIndex & index, mg400_msgs::msg::DOStatus & status)
{
  if (cmd_node["distance_mode"].as<std::string>() == "percentage") {
    mode.mode = mg400_msgs::msg::DistanceMode::PERCENTAGE;
  } else {
    mode.mode = mg400_msgs::msg::DistanceMode::FROM_START_OR_TARGET;
  }

  distance = cmd_node["distance"].as<int>();
  index.index = cmd_node["index"].as<mg400_msgs::msg::DOIndex::_index_type>();
  status.status = cmd_node["status"].as<mg400_msgs::msg::DOStatus::_status_type>();
}

void CommandQueueClient::sendGoalWithCallbacks(const CommandQueue::Goal & goal_msg)
{
  auto send_goal_options = rclcpp_action::Client<CommandQueue>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    std::bind(&CommandQueueClient::goalResponseCallback, this, std::placeholders::_1);

  send_goal_options.feedback_callback =
    std::bind(
    &CommandQueueClient::feedbackCallback, this, std::placeholders::_1,
    std::placeholders::_2);

  send_goal_options.result_callback =
    std::bind(&CommandQueueClient::resultCallback, this, std::placeholders::_1);

  RCLCPP_INFO(this->get_logger(), "Sending goal with %zu commands.", goal_msg.commands.size());
  this->client_->async_send_goal(goal_msg, send_goal_options);
}

void CommandQueueClient::goalResponseCallback(const GoalHandleCommandQueue::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    rclcpp::shutdown();
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
}

void CommandQueueClient::feedbackCallback(
  GoalHandleCommandQueue::SharedPtr /*goal_handle*/,
  const std::shared_ptr<const CommandQueue::Feedback> feedback)
{
  logCurrentJointAngles(feedback);
  logCurrentPose(feedback);
}

void CommandQueueClient::logCurrentJointAngles(
  const std::shared_ptr<const CommandQueue::Feedback> feedback)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Current joint angles: [%.2f°, %.2f°, %.2f°, %.2f°]",
    feedback->current_angles[0] * 180.0 / M_PI,
    feedback->current_angles[1] * 180.0 / M_PI,
    feedback->current_angles[2] * 180.0 / M_PI,
    feedback->current_angles[3] * 180.0 / M_PI
  );
}

void CommandQueueClient::logCurrentPose(
  const std::shared_ptr<const CommandQueue::Feedback> feedback)
{
  tf2::Quaternion q(
    feedback->current_pose.pose.orientation.x,
    feedback->current_pose.pose.orientation.y,
    feedback->current_pose.pose.orientation.z,
    feedback->current_pose.pose.orientation.w
  );

  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  RCLCPP_INFO(
    this->get_logger(),
    "Current pose: [x=%.2f, y=%.2f, z=%.2f, yaw=%.2f°]",
    feedback->current_pose.pose.position.x,
    feedback->current_pose.pose.position.y,
    feedback->current_pose.pose.position.z,
    yaw * 180.0 / M_PI
  );
}

void CommandQueueClient::resultCallback(const GoalHandleCommandQueue::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
      handleSuccessResult(result.result);
      break;

    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      handleErrorResult(result.result);
      break;

    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(this->get_logger(), "Goal was canceled");
      handleCancelResult(result.result);
      break;

    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      break;
  }
  rclcpp::shutdown();
}

void CommandQueueClient::handleSuccessResult(
  const std::shared_ptr<const CommandQueue::Result> result)
{
  RCLCPP_INFO(this->get_logger(), "Successfully completed joint trajectory execution");

  if (result) {
    if (result->result) {
      RCLCPP_INFO(this->get_logger(), "Action result: SUCCESS");
    } else {
      RCLCPP_WARN(this->get_logger(), "Action completed but result flag is false");
    }
    this->showErrorID(result->error_id);
  }
}

void CommandQueueClient::handleErrorResult(const std::shared_ptr<const CommandQueue::Result> result)
{
  RCLCPP_ERROR(this->get_logger(), "Joint trajectory execution failed");

  if (result) {
    RCLCPP_ERROR(this->get_logger(), "Action result: FAILED");
    this->showErrorID(result->error_id);
  }
}

void CommandQueueClient::handleCancelResult(
  const std::shared_ptr<const CommandQueue::Result> result)
{
  RCLCPP_WARN(this->get_logger(), "Joint trajectory execution was canceled");

  if (result) {
    this->showErrorID(result->error_id);
  }
}

void CommandQueueClient::showErrorID(mg400_msgs::msg::ErrorID error_id)
{
  if (!error_id.controller.ids.empty()) {
    RCLCPP_INFO(this->get_logger(), "Controller Error IDs:");
    for (auto id : error_id.controller.ids) {
      RCLCPP_INFO(this->get_logger(), "  %d", id);
    }
  }
}

void CommandQueueClient::cancelGoal()
{
  if (this->goal_handle_) {
    RCLCPP_INFO(this->get_logger(), "Canceling current goal");
    this->client_->async_cancel_goal(this->goal_handle_);
  } else {
    RCLCPP_WARN(this->get_logger(), "No active goal to cancel");
  }
}

}  // namespace mg400_tools

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<mg400_tools::CommandQueueClient>(options));
  rclcpp::shutdown();
  return 0;
}
