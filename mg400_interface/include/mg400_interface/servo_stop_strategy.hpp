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

#ifndef MG400_INTERFACE__SERVO_STOP_STRATEGY_HPP_
#define MG400_INTERFACE__SERVO_STOP_STRATEGY_HPP_

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace mg400_interface
{

class ServoStopStrategy
{
public:
  using SharedPtr = std::shared_ptr<ServoStopStrategy>;
  using Clock = std::chrono::steady_clock;

  enum class Status
  {
    SUCCESS,
    RESET_FAILED,
    CONFIRMATION_TIMEOUT,
  };

  struct Result
  {
    Status status;
    std::string message;

    bool success() const noexcept {return status == Status::SUCCESS;}
  };

  virtual ~ServoStopStrategy() = default;
  virtual Result stop(const Clock::time_point & deadline) = 0;
};

/// Provisional stop implementation: ResetRobot(), then observe RobotMode=ENABLE.
class ResetRobotStopStrategy : public ServoStopStrategy
{
public:
  using ResetRobotFunction = std::function<void ()>;
  using RobotModeReader = std::function<bool (std::uint64_t &)>;

  struct Options
  {
    std::chrono::nanoseconds confirmation_poll_period = std::chrono::milliseconds(10);
    std::uint64_t successful_robot_mode = 5;  // MG400 RobotMode::ENABLE.
  };

  ResetRobotStopStrategy(
    ResetRobotFunction reset_robot, RobotModeReader robot_mode_reader);
  ResetRobotStopStrategy(
    ResetRobotFunction reset_robot, RobotModeReader robot_mode_reader,
    const Options & options);

  Result stop(const Clock::time_point & deadline) override;

private:
  ResetRobotFunction reset_robot_;
  RobotModeReader robot_mode_reader_;
  Options options_;
};

}  // namespace mg400_interface

#endif  // MG400_INTERFACE__SERVO_STOP_STRATEGY_HPP_
