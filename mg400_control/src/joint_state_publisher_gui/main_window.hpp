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

#include <QWidget>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

QT_BEGIN_NAMESPACE
namespace Ui
{
class MainWindow;
}
QT_END_NAMESPACE


class MainWindow : public QWidget, public rclcpp::Node
{
  Q_OBJECT

public:
  static constexpr double J1_MIN = -2.82;
  static constexpr double J1_MAX = 2.82;
  static constexpr double J2_MIN = -0.52;
  static constexpr double J2_MAX = 1.57;
  static constexpr double J3_MIN = 0;
  static constexpr double J3_MAX = 1.57;
  static constexpr double J4_MIN = -3.14;
  static constexpr double J4_MAX = 3.14;

private:
  Ui::MainWindow * ui_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double j1_;
  double j2_;
  double j3_;
  double j4_;

public:
  explicit MainWindow(
    const rclcpp::NodeOptions & node_options,
    QWidget * parent = nullptr);
  ~MainWindow() override;

private:
  void randomBtn();
  void centerBtn();
  void j1ValueChange(int);
  void j2ValueChange(int);
  void j3ValueChange(int);
  void j4ValueChange(int);
  void publishJointStates();
};
