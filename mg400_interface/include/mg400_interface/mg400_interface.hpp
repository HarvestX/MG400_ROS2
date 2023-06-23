// Copyright 2022 HarvestX Inc.
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

#pragma once

#include <string>
#include <memory>

#include "mg400_interface/tcp_interface/dashboard_tcp_interface.hpp"
#include "mg400_interface/tcp_interface/motion_tcp_interface.hpp"
#include "mg400_interface/tcp_interface/realtime_feedback_tcp_interface.hpp"

#include "mg400_interface/commander/dashboard_commander.hpp"
#include "mg400_interface/commander/motion_commander.hpp"

#include "mg400_interface/joint_handler.hpp"
#include "mg400_interface/error_msg_generator.hpp"

#include <rclcpp/rclcpp.hpp>


namespace mg400_interface
{
class MG400Interface
{
public:
  using UniquePtr = std::unique_ptr<MG400Interface>;
  using SharedPtr = std::shared_ptr<MG400Interface>;

  DashboardCommander::SharedPtr dashboard_commander;
  MotionCommander::SharedPtr motion_commander;
  RealtimeFeedbackTcpInterface::SharedPtr realtime_tcp_interface;

  std::unique_ptr<ErrorMsgGenerator> controller_error_msg_generator;
  std::unique_ptr<ErrorMsgGenerator> servo_error_msg_generator;

private:
  const std::string IP;

  DashboardTcpInterface::UniquePtr dashboard_tcp_if_;
  MotionTcpInterface::UniquePtr motion_tcp_if_;

public:
  MG400Interface() = delete;
  explicit MG400Interface(const std::string &);

  bool configure(const std::string & = "");

  bool activate();
  bool deactivate();
  bool ok();

private:
  static const rclcpp::Logger getLogger() noexcept;
  bool isConnected();
};
}  // namespace mg400_interface
