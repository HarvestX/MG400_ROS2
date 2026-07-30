// Copyright 2026 HarvestX Inc.
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

#include "mg400_interface/servo_mode/servo_stop_strategy.hpp"

#include <algorithm>
#include <exception>
#include <stdexcept>
#include <thread>
#include <utility>

namespace mg400_interface
{

ResetRobotStopStrategy::ResetRobotStopStrategy(
  ResetRobotFunction reset_robot, RobotModeReader robot_mode_reader)
: ResetRobotStopStrategy(
    std::move(reset_robot), std::move(robot_mode_reader), Options{})
{
}

ResetRobotStopStrategy::ResetRobotStopStrategy(
  ResetRobotFunction reset_robot, RobotModeReader robot_mode_reader,
  const Options & options)
: reset_robot_(std::move(reset_robot)),
  robot_mode_reader_(std::move(robot_mode_reader)),
  options_(options)
{
  if (!this->reset_robot_) {
    throw std::invalid_argument("ResetRobot stop strategy requires a ResetRobot function");
  }
  if (!this->robot_mode_reader_) {
    throw std::invalid_argument("ResetRobot stop strategy requires a RobotMode reader");
  }
  if (this->options_.confirmation_poll_period <= std::chrono::nanoseconds::zero()) {
    throw std::invalid_argument("stop confirmation poll period must be positive");
  }
}

ServoStopStrategy::Result ResetRobotStopStrategy::stop(
  const Clock::time_point & deadline)
{
  try {
    this->reset_robot_();
  } catch (const std::exception & error) {
    return Result{Status::RESET_FAILED, std::string("ResetRobot failed: ") + error.what()};
  } catch (...) {
    return Result{Status::RESET_FAILED, "ResetRobot failed with an unknown exception"};
  }

  while (Clock::now() < deadline) {
    std::uint64_t robot_mode = 0;
    try {
      if (this->robot_mode_reader_(robot_mode) &&
        robot_mode == this->options_.successful_robot_mode)
      {
        return Result{Status::SUCCESS, "ResetRobot succeeded and RobotMode is ENABLE"};
      }
    } catch (const std::exception &) {
      // A transient read failure is retried until the common steady-clock deadline.
    } catch (...) {
      // Treat unknown reader failures as transient until the deadline as well.
    }

    const auto wake_at = std::min(
      deadline, Clock::now() + this->options_.confirmation_poll_period);
    std::this_thread::sleep_until(wake_at);
  }

  return Result{
    Status::CONFIRMATION_TIMEOUT,
    "ResetRobot succeeded but RobotMode=ENABLE was not confirmed before the deadline"};
}

}  // namespace mg400_interface
