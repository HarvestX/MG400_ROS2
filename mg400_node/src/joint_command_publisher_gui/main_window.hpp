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

#ifndef __MG400_NODE_JOINT_COMMAND_PUBLISHER_GUI_MAIN_WINDOW_HPP__
#define __MG400_NODE_JOINT_COMMAND_PUBLISHER_GUI_MAIN_WINDOW_HPP__

#include <QWidget>

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <mg400_common/mg400_ik_util.hpp>
#include <mg400_interface/joint_handler.hpp>

QT_BEGIN_NAMESPACE
namespace Ui
{
class MainWindow;
}
QT_END_NAMESPACE


class MainWindow : public QWidget, public rclcpp::Node
{
  Q_OBJECT

private:
  Ui::MainWindow * ui_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double j1_;
  double j2_;
  double j3_;
  double j4_;

  const std::string prefix_;

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

#endif
