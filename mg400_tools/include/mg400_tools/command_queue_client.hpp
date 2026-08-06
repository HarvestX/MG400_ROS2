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

#ifndef MG400_TOOLS__COMMAND_QUEUE_CLIENT_HPP_
#define MG400_TOOLS__COMMAND_QUEUE_CLIENT_HPP_

#include <memory>
#include <string>
#include <vector>
#include <array>
#include <filesystem>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <yaml-cpp/yaml.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <mg400_msgs/action/command_queue.hpp>
#include <mg400_msgs/msg/command.hpp>
#include <mg400_msgs/msg/joint_mov_j.hpp>
#include <mg400_msgs/msg/mov_j.hpp>
#include <mg400_msgs/msg/mov_l.hpp>
#include <mg400_msgs/msg/mov_jio.hpp>
#include <mg400_msgs/msg/mov_lio.hpp>
#include <mg400_msgs/msg/error_id.hpp>
#include <mg400_msgs/msg/distance_mode.hpp>
#include <mg400_msgs/msg/do_index.hpp>
#include <mg400_msgs/msg/do_status.hpp>

namespace mg400_tools
{

class CommandQueueClient : public rclcpp::Node
{
public:
  explicit CommandQueueClient(const rclcpp::NodeOptions options);
  void cancelGoal();

private:
  using CommandQueue = mg400_msgs::action::CommandQueue;
  using GoalHandleCommandQueue = rclcpp_action::ClientGoalHandle<CommandQueue>;

  std::string param_name_command_config_path_;

  void onInit();
  bool waitForActionServer();
  YAML::Node loadConfig();
  void sendGoal();
  void sendGoalWithCallbacks(const CommandQueue::Goal & goal_msg);

  void parseCommandsFromConfig(const YAML::Node & config, CommandQueue::Goal & goal_msg);
  bool parseCommand(
    const YAML::Node & cmd_node, const std::string & type,
    mg400_msgs::msg::Command & cmd);

  bool parseJointMovJCommand(const YAML::Node & cmd_node, mg400_msgs::msg::Command & cmd);
  bool parseMovJCommand(const YAML::Node & cmd_node, mg400_msgs::msg::Command & cmd);
  bool parseMovLCommand(const YAML::Node & cmd_node, mg400_msgs::msg::Command & cmd);
  bool parseMovJIOCommand(const YAML::Node & cmd_node, mg400_msgs::msg::Command & cmd);
  bool parseMovLIOCommand(const YAML::Node & cmd_node, mg400_msgs::msg::Command & cmd);

  void setPoseFromConfig(const YAML::Node & cmd_node, geometry_msgs::msg::PoseStamped & pose);
  void setIOParameters(
    const YAML::Node & cmd_node, mg400_msgs::msg::DistanceMode & mode,
    int & distance, mg400_msgs::msg::DOIndex & index,
    mg400_msgs::msg::DOStatus & status);

  void goalResponseCallback(const GoalHandleCommandQueue::SharedPtr & goal_handle);
  void feedbackCallback(
    GoalHandleCommandQueue::SharedPtr,
    const std::shared_ptr<const CommandQueue::Feedback> feedback);
  void resultCallback(const GoalHandleCommandQueue::WrappedResult & result);

  void logCurrentJointAngles(const std::shared_ptr<const CommandQueue::Feedback> feedback);
  void logCurrentPose(const std::shared_ptr<const CommandQueue::Feedback> feedback);

  void handleSuccessResult(const std::shared_ptr<const CommandQueue::Result> result);
  void handleErrorResult(const std::shared_ptr<const CommandQueue::Result> result);
  void handleCancelResult(const std::shared_ptr<const CommandQueue::Result> result);
  void showErrorID(mg400_msgs::msg::ErrorID error_id);

  rclcpp_action::Client<CommandQueue>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr init_timer_;
  GoalHandleCommandQueue::SharedPtr goal_handle_;
};

}  // namespace mg400_tools

#endif  // MG400_TOOLS__COMMAND_QUEUE_CLIENT_HPP_
