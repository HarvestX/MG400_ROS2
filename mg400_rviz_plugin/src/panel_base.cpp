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

#include "mg400_rviz_plugin/panel_base.hpp"

#include <algorithm>
#include <cctype>

namespace mg400_rviz_plugin
{
Mg400PanelBase::Mg400PanelBase(QWidget * parent)
: rviz_common::Panel(parent),
  namespace_edit_(new QLineEdit(this)),
  namespace_apply_button_(new QPushButton("Apply", this)),
  robot_namespace_("/mg400")
{
  namespace_edit_->setText(QString::fromStdString(robot_namespace_));
  namespace_edit_->setPlaceholderText("/mg400");
  namespace_apply_button_->setMaximumWidth(80);

  connect(
    namespace_edit_, SIGNAL(editingFinished()),
    this, SLOT(applyNamespaceEdit()));
  connect(
    namespace_apply_button_, SIGNAL(clicked()),
    this, SLOT(applyNamespaceEdit()));
}

void Mg400PanelBase::onInitialize()
{
  rviz_common::Panel::onInitialize();

  const auto node_abstraction = this->getDisplayContext()->getRosNodeAbstraction().lock();
  if (!node_abstraction) {
    return;
  }

  raw_node_ = node_abstraction->get_raw_node();
  callback_group_ = raw_node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(
    callback_group_, raw_node_->get_node_base_interface());

  this->onRosNodeInitialized();
}

void Mg400PanelBase::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);

  QString loaded_namespace;
  if (config.mapGetString("namespace", &loaded_namespace)) {
    const auto previous_namespace = robot_namespace_;
    this->setRobotNamespace(loaded_namespace.toStdString());
    if (raw_node_ && robot_namespace_ != previous_namespace) {
      this->onNamespaceChanged();
    }
  }

  this->loadPanelConfig(config);
}

void Mg400PanelBase::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("namespace", QString::fromStdString(robot_namespace_));
  this->savePanelConfig(config);
}

void Mg400PanelBase::applyNamespaceEdit()
{
  const auto next_namespace = normalizeNamespace(namespace_edit_->text().toStdString());
  if (next_namespace == robot_namespace_) {
    namespace_edit_->setText(QString::fromStdString(robot_namespace_));
    return;
  }

  robot_namespace_ = next_namespace;
  namespace_edit_->setText(QString::fromStdString(robot_namespace_));

  if (raw_node_) {
    this->onNamespaceChanged();
  }
  Q_EMIT configChanged();
}

QHBoxLayout * Mg400PanelBase::createNamespaceLayout()
{
  auto * layout = new QHBoxLayout;
  layout->addWidget(new QLabel("Namespace:", this));
  layout->addWidget(namespace_edit_);
  layout->addWidget(namespace_apply_button_);
  return layout;
}

rclcpp::Node::SharedPtr Mg400PanelBase::rawNode() const
{
  return raw_node_;
}

rclcpp::CallbackGroup::SharedPtr Mg400PanelBase::callbackGroup() const
{
  return callback_group_;
}

rclcpp::executors::SingleThreadedExecutor & Mg400PanelBase::callbackGroupExecutor()
{
  return callback_group_executor_;
}

const std::string & Mg400PanelBase::robotNamespace() const
{
  return robot_namespace_;
}

std::string Mg400PanelBase::makeName(const std::string & name) const
{
  auto relative_name = name;
  relative_name.erase(
    relative_name.begin(),
    std::find_if(
      relative_name.begin(), relative_name.end(),
      [](const char value) {return value != '/';}));

  if (robot_namespace_.empty()) {
    return "/" + relative_name;
  }
  return robot_namespace_ + "/" + relative_name;
}

std::string Mg400PanelBase::makeTopic(const std::string & name) const
{
  return this->makeName(name);
}

std::string Mg400PanelBase::makeService(const std::string & name) const
{
  return this->makeName(name);
}

std::string Mg400PanelBase::makeAction(const std::string & name) const
{
  return this->makeName(name);
}

void Mg400PanelBase::onRosNodeInitialized()
{
}

void Mg400PanelBase::onNamespaceChanged()
{
}

void Mg400PanelBase::loadPanelConfig(const rviz_common::Config &)
{
}

void Mg400PanelBase::savePanelConfig(rviz_common::Config) const
{
}

std::string Mg400PanelBase::normalizeNamespace(const std::string & robot_namespace)
{
  auto normalized = robot_namespace;

  normalized.erase(
    normalized.begin(),
    std::find_if(
      normalized.begin(), normalized.end(),
      [](const char value) {return !std::isspace(static_cast<unsigned char>(value));}));
  normalized.erase(
    std::find_if(
      normalized.rbegin(), normalized.rend(),
      [](const char value) {return !std::isspace(static_cast<unsigned char>(value));}).base(),
    normalized.end());

  if (normalized.empty() || normalized == "/") {
    return "";
  }
  if (normalized.front() != '/') {
    normalized.insert(normalized.begin(), '/');
  }
  while (normalized.size() > 1 && normalized.back() == '/') {
    normalized.pop_back();
  }

  return normalized;
}

void Mg400PanelBase::setRobotNamespace(const std::string & robot_namespace)
{
  robot_namespace_ = normalizeNamespace(robot_namespace);
  namespace_edit_->setText(QString::fromStdString(robot_namespace_));
}
}  // namespace mg400_rviz_plugin
