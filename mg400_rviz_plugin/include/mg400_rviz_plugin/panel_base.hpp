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

#ifndef __MG400_RVIZ_PLUGIN_PANEL_BASE_HPP__
#define __MG400_RVIZ_PLUGIN_PANEL_BASE_HPP__

#include <string>

#ifndef Q_MOC_RUN
#include <QtWidgets>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>
#endif

namespace mg400_rviz_plugin
{
class Mg400PanelBase : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit Mg400PanelBase(QWidget * parent = nullptr);

  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

protected Q_SLOTS:
  void applyNamespaceEdit();

protected:
  QHBoxLayout * createNamespaceLayout();

  rclcpp::Node::SharedPtr rawNode() const;
  rclcpp::CallbackGroup::SharedPtr callbackGroup() const;
  rclcpp::executors::SingleThreadedExecutor & callbackGroupExecutor();

  const std::string & robotNamespace() const;
  std::string makeName(const std::string & name) const;
  std::string makeTopic(const std::string & name) const;
  std::string makeService(const std::string & name) const;
  std::string makeAction(const std::string & name) const;

  virtual void onRosNodeInitialized();
  virtual void onNamespaceChanged();
  virtual void loadPanelConfig(const rviz_common::Config & config);
  virtual void savePanelConfig(rviz_common::Config config) const;

private:
  static std::string normalizeNamespace(const std::string & robot_namespace);
  void setRobotNamespace(const std::string & robot_namespace);

  QLineEdit * namespace_edit_;
  QPushButton * namespace_apply_button_;

  std::string robot_namespace_;
  rclcpp::Node::SharedPtr raw_node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
};
}  // namespace mg400_rviz_plugin

#endif
