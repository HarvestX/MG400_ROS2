// Copyright 2026
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

#ifndef __MG400_RVIZ_PLUGIN_PANEL_MG400_MONITOR_HPP__
#define __MG400_RVIZ_PLUGIN_PANEL_MG400_MONITOR_HPP__

#include <mutex>

#ifndef Q_MOC_RUN
#include <QtWidgets>
#include <rclcpp/rclcpp.hpp>
#endif

#include <mg400_msgs/msg/realtime_feedback.hpp>

#include "mg400_rviz_plugin/panel_base.hpp"

namespace mg400_rviz_plugin
{
class Mg400MonitorPanel : public Mg400PanelBase
{
  Q_OBJECT

private:
  using RealtimeFeedback = mg400_msgs::msg::RealtimeFeedback;

protected:
  QLabel * label_feedback_status_;
  QLabel * label_feedback_robot_mode_;
  QLabel * label_tcp_actual_;
  QLabel * label_tcp_target_;
  QLabel * label_joint_actual_;
  QLabel * label_joint_target_;
  QTableWidget * realtime_feedback_table_;

  rclcpp::Subscription<RealtimeFeedback>::SharedPtr realtime_feedback_sub_;
  std::mutex realtime_feedback_mutex_;
  RealtimeFeedback latest_realtime_feedback_;
  rclcpp::Time latest_realtime_feedback_received_time_;
  bool has_realtime_feedback_;

public:
  explicit Mg400MonitorPanel(QWidget * parent = nullptr);

public Q_SLOTS:
  void tick();

protected:
  void onRosNodeInitialized() override;
  void onNamespaceChanged() override;
  void setupRosInterfaces();
  QWidget * createMonitorSection();
  void updateMonitorLabels();
  void clearRealtimeFeedbackTable();
  void updateRealtimeFeedbackTable(const RealtimeFeedback & feedback);
};
}  // namespace mg400_rviz_plugin

#endif
