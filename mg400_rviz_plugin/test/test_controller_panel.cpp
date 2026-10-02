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

#include <algorithm>
#include <array>
#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <thread>

#include <gtest/gtest.h>
#include <mg400_interface/joint_handler.hpp>
#include <pluginlib/class_loader.hpp>

#include "mg400_rviz_plugin/panel_mg400_controller.hpp"

using namespace std::chrono_literals;  // NOLINT
using Action = mg400_msgs::action::JointMovJ;
using MovJ = mg400_msgs::action::MovJ;
using RobotMode = mg400_msgs::msg::RobotMode;
using JointState = sensor_msgs::msg::JointState;
using Enable = mg400_msgs::srv::EnableRobot;
using Disable = mg400_msgs::srv::DisableRobot;
using ClearError = mg400_msgs::srv::ClearError;
using SetCollision = mg400_msgs::srv::SetCollisionLevel;

class TestPanel : public mg400_rviz_plugin::Mg400ControllerPanel
{
public:
  using Mg400ControllerPanel::initializeRos;
};

class ControllerPanelTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("joint_panel_test");
    executor_.add_node(node_);
    panel_ = std::make_unique<TestPanel>();
    send_ = panel_->findChild<QPushButton *>("send_joint_mov_j");
    copy_ = panel_->findChild<QPushButton *>("use_current");
    status_ = panel_->findChild<QLabel *>("status");
    ASSERT_NE(nullptr, send_);
    ASSERT_NE(nullptr, copy_);
    ASSERT_NE(nullptr, status_);
    EXPECT_FALSE(send_->isEnabled());
    EXPECT_FALSE(copy_->isEnabled());
    panel_->initializeRos(node_);
    mode_pub_ = node_->create_publisher<RobotMode>("/mg400/robot_mode", 1);
    joints_pub_ = node_->create_publisher<JointState>("/mg400/joint_states", 1);
    server_ = rclcpp_action::create_server<Action>(
      node_, "/mg400/joint_mov_j",
      [this](const rclcpp_action::GoalUUID &, Action::Goal::ConstSharedPtr goal) {
        ++goal_count_;
        last_goal_ = *goal;
        return reject_ ? rclcpp_action::GoalResponse::REJECT :
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](std::shared_ptr<rclcpp_action::ServerGoalHandle<Action>>) {
        return rclcpp_action::CancelResponse::REJECT;
      },
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<Action>> handle) {handle_ = handle;});
    movj_server_ = rclcpp_action::create_server<MovJ>(
      node_, "/mg400/mov_j",
      [this](const rclcpp_action::GoalUUID &, MovJ::Goal::ConstSharedPtr goal) {
        ++movj_count_;
        last_movj_goal_ = *goal;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](std::shared_ptr<rclcpp_action::ServerGoalHandle<MovJ>>) {
        return rclcpp_action::CancelResponse::REJECT;
      },
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<MovJ>> handle) {
        movj_handle_ = handle;
      });
    enable_server_ = node_->create_service<Enable>(
      "/mg400/enable_robot",
      [this](Enable::Request::SharedPtr request, Enable::Response::SharedPtr response) {
        ++enable_count_;
        last_enable_request_ = *request;
        response->result = true;
      });
    disable_server_ = node_->create_service<Disable>(
      "/mg400/disable_robot",
      [this](Disable::Request::SharedPtr, Disable::Response::SharedPtr response) {
        ++disable_count_;
        response->result = true;
      });
    clear_server_ = node_->create_service<ClearError>(
      "/mg400/clear_error",
      [this](ClearError::Request::SharedPtr, ClearError::Response::SharedPtr response) {
        ++clear_count_;
        response->result = clear_success_;
        response->error_id = clear_success_ ? 0 : 42;
      });
    collision_server_ = node_->create_service<SetCollision>(
      "/mg400/set_collision_level",
      [this](SetCollision::Request::SharedPtr request, SetCollision::Response::SharedPtr response) {
        ++collision_count_;
        last_collision_request_ = *request;
        response->result = collision_success_;
        response->error_id = collision_success_ ? 0 : 42;
      });
    ASSERT_TRUE(
      waitFor(
        [this]() {
          return mode_pub_->get_subscription_count() > 0 &&
          joints_pub_->get_subscription_count() > 0;
        }));
  }

  bool waitFor(const std::function<bool()> & predicate)
  {
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    do {
      executor_.spin_some();
      QApplication::processEvents();
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(1ms);
    } while (std::chrono::steady_clock::now() < deadline);
    return false;
  }

  void setMode(uint64_t mode, const QString & name)
  {
    RobotMode msg;
    msg.robot_mode = mode;
    mode_pub_->publish(msg);
    ASSERT_TRUE(
      waitFor(
        [&]() {
          return panel_->findChild<QLabel *>("robot_mode")->text() == "Robot Mode: " + name;
        }));
  }

  void setGoals(const std::array<const char *, 4> & values)
  {
    for (size_t i = 0; i < values.size(); ++i) {
      panel_->findChild<QLineEdit *>(QString("goal_j%1").arg(i + 1))->setText(values[i]);
    }
  }

  void setPoseGoals(const std::array<const char *, 4> & values)
  {
    const std::array<const char *, 4> names = {"goal_x", "goal_y", "goal_z", "goal_r"};
    for (size_t i = 0; i < values.size(); ++i) {
      panel_->findChild<QLineEdit *>(names[i])->setText(values[i]);
    }
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::unique_ptr<TestPanel> panel_;
  rclcpp::Publisher<RobotMode>::SharedPtr mode_pub_;
  rclcpp::Publisher<JointState>::SharedPtr joints_pub_;
  rclcpp_action::Server<Action>::SharedPtr server_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<Action>> handle_;
  Action::Goal last_goal_;
  rclcpp_action::Server<MovJ>::SharedPtr movj_server_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<MovJ>> movj_handle_;
  MovJ::Goal last_movj_goal_;
  size_t movj_count_ = 0;
  rclcpp::Service<Enable>::SharedPtr enable_server_;
  rclcpp::Service<Disable>::SharedPtr disable_server_;
  rclcpp::Service<ClearError>::SharedPtr clear_server_;
  rclcpp::Service<SetCollision>::SharedPtr collision_server_;
  Enable::Request last_enable_request_;
  SetCollision::Request last_collision_request_;
  size_t enable_count_ = 0;
  size_t disable_count_ = 0;
  size_t clear_count_ = 0;
  size_t collision_count_ = 0;
  bool clear_success_ = true;
  bool collision_success_ = true;
  bool reject_ = false;
  size_t goal_count_ = 0;
  QPushButton * send_;
  QPushButton * copy_;
  QLabel * status_;
};

TEST_F(ControllerPanelTest, MapsPhysicalJointsAndCopiesDegrees)
{
  const std::array<double, 4> angles = {0.1, 0.2, 0.5, -0.4};
  auto msg = mg400_interface::JointHandler::getJointState(angles, "test_");
  std::reverse(msg->name.begin(), msg->name.end());
  std::reverse(msg->position.begin(), msg->position.end());
  joints_pub_->publish(*msg);
  ASSERT_TRUE(waitFor([this]() {return copy_->isEnabled();}));
  copy_->click();
  for (size_t i = 0; i < angles.size(); ++i) {
    const double degrees = mg400_interface::rad2degree(angles[i]);
    const auto current = panel_->findChild<QLabel *>(QString("current_j%1").arg(i + 1));
    const auto goal = panel_->findChild<QLineEdit *>(QString("goal_j%1").arg(i + 1));
    EXPECT_NEAR(degrees, current->text().toDouble(), 0.001);
    EXPECT_NEAR(degrees, goal->text().toDouble(), 0.000001);
  }
  EXPECT_EQ(0u, goal_count_);
}

TEST_F(ControllerPanelTest, IgnoresMalformedJointStates)
{
  auto msg = mg400_interface::JointHandler::getJointState({0.1, 0.2, 0.3, 0.4}, "");
  msg->position.resize(2);
  joints_pub_->publish(*msg);
  setMode(RobotMode::DISABLED, "DISABLED");
  EXPECT_FALSE(copy_->isEnabled());
  msg = mg400_interface::JointHandler::getJointState({0.1, 0.2, 0.3, 0.4}, "");
  msg->position[6] = std::numeric_limits<double>::quiet_NaN();
  joints_pub_->publish(*msg);
  setMode(RobotMode::ENABLE, "ENABLE");
  EXPECT_FALSE(copy_->isEnabled());
}

TEST_F(ControllerPanelTest, RejectsInvalidTargetsAndUnavailableRobot)
{
  setMode(RobotMode::ENABLE, "ENABLE");
  EXPECT_FALSE(send_->isEnabled());
  setGoals({"10", "20", "30", "-40"});
  ASSERT_TRUE(waitFor([this]() {return send_->isEnabled();}));
  for (const auto * invalid : {"", "abc", "nan", "inf", "1e309", "161"}) {
    setGoals({invalid, "20", "30", "-40"});
    EXPECT_FALSE(send_->isEnabled()) << invalid;
  }
  // Each axis alone is inside its limits, but J3 - J2 exceeds the coupled limit.
  setGoals({"0", "-10", "100", "0"});
  EXPECT_FALSE(send_->isEnabled());
  setGoals({"0", "20", "30", "360"});
  EXPECT_FALSE(send_->isEnabled());
  setGoals({"10", "20", "30", "-40"});
  setMode(RobotMode::DISABLED, "DISABLED");
  EXPECT_FALSE(send_->isEnabled());
  setMode(RobotMode::RUNNING, "RUNNING");
  EXPECT_FALSE(send_->isEnabled());
  setMode(RobotMode::ERROR, "ERROR");
  EXPECT_FALSE(send_->isEnabled());
  setMode(RobotMode::ENABLE, "ENABLE");
  server_.reset();
  ASSERT_TRUE(waitFor([this]() {return !send_->isEnabled();}));
  EXPECT_EQ(0u, goal_count_);
}

TEST_F(ControllerPanelTest, SendsRadiansAndProcessesFeedbackAndSuccess)
{
  setPoseGoals({"100", "-50", "200", "90"});
  setGoals({"10", "20", "30", "-40"});
  setMode(RobotMode::ENABLE, "ENABLE");
  ASSERT_TRUE(waitFor([this]() {return send_->isEnabled();}));
  send_->click();
  EXPECT_FALSE(send_->isEnabled());
  EXPECT_FALSE(panel_->findChild<QPushButton *>("send_mov_j")->isEnabled());
  send_->click();
  ASSERT_TRUE(waitFor([this]() {return handle_ && status_->text().contains("accepted");}));
  EXPECT_EQ(1u, goal_count_);
  const std::array<double, 4> degrees = {10, 20, 30, -40};
  for (size_t i = 0; i < degrees.size(); ++i) {
    EXPECT_NEAR(mg400_interface::degree2rad(degrees[i]), last_goal_.joint_angles[i], 1e-12);
  }
  EXPECT_FALSE(last_goal_.set_speed_j);
  EXPECT_FALSE(last_goal_.set_acc_j);
  EXPECT_FALSE(last_goal_.set_cp);
  auto feedback = std::make_shared<Action::Feedback>();
  feedback->current_angles = {0.1, 0.2, 0.3, 0.4};
  handle_->publish_feedback(feedback);
  ASSERT_TRUE(
    waitFor(
      [this]() {
        return panel_->findChild<QLabel *>("current_j3")->text() == "17.189";
      }));
  EXPECT_FALSE(copy_->isEnabled());
  auto result = std::make_shared<Action::Result>();
  result->result = true;
  handle_->succeed(result);
  ASSERT_TRUE(waitFor([this]() {return status_->text() == "JointMovJ succeeded.";}));
  EXPECT_TRUE(send_->isEnabled());
}

TEST_F(ControllerPanelTest, SharedRobotButtonsStayAboveAllTabs)
{
  auto * tabs = panel_->findChild<QTabWidget *>("motion_tabs");
  ASSERT_NE(nullptr, tabs);
  ASSERT_EQ(3, tabs->count());
  EXPECT_EQ("MovJ", tabs->tabText(0));
  EXPECT_EQ("JointMovJ", tabs->tabText(1));
  EXPECT_EQ("Collision", tabs->tabText(2));
  EXPECT_TRUE(tabs->widget(2)->isAncestorOf(panel_->findChild<QComboBox *>("collision_level")));
  EXPECT_TRUE(tabs->widget(2)->isAncestorOf(panel_->findChild<QPushButton *>("set_collision_level")));
  EXPECT_TRUE(tabs->widget(0)->isAncestorOf(panel_->findChild<QPushButton *>("send_mov_j")));
  EXPECT_TRUE(tabs->widget(1)->isAncestorOf(send_));
  EXPECT_TRUE(tabs->widget(0)->isAncestorOf(panel_->findChild<QPushButton *>("use_current_pose")));
  EXPECT_TRUE(tabs->widget(1)->isAncestorOf(copy_));
  panel_->resize(460, 460);
  panel_->show();
  QApplication::processEvents();
  for (const auto * name : {"enable_robot", "disable_robot", "clear_error"}) {
    auto * button = panel_->findChild<QPushButton *>(name);
    ASSERT_NE(nullptr, button);
    EXPECT_FALSE(tabs->isAncestorOf(button));
    EXPECT_LT(button->mapTo(panel_.get(), QPoint()).y(), tabs->y());
    for (int i = 0; i < tabs->count(); ++i) {
      tabs->setCurrentIndex(i);
      EXPECT_TRUE(button->isVisible());
    }
  }
  for (const auto * name :
    {"enable_load", "enable_center_x", "enable_center_y", "enable_center_z"})
  {
    auto * input = panel_->findChild<QWidget *>(name);
    ASSERT_NE(nullptr, input);
    EXPECT_FALSE(tabs->isAncestorOf(input));
    EXPECT_LT(input->mapTo(panel_.get(), QPoint()).y(), tabs->y());
    EXPECT_TRUE(input->isVisible());
  }
}

TEST_F(ControllerPanelTest, CopiesCurrentPoseInMillimetersAndDegrees)
{
  auto msg = mg400_interface::JointHandler::getJointState({0, 0, 0, M_PI_2}, "");
  joints_pub_->publish(*msg);
  auto * copy_pose = panel_->findChild<QPushButton *>("use_current_pose");
  ASSERT_TRUE(waitFor([&]() {return copy_pose->isEnabled();}));
  copy_pose->click();
  EXPECT_NEAR(284.5, panel_->findChild<QLineEdit *>("goal_x")->text().toDouble(), 1e-6);
  EXPECT_NEAR(0, panel_->findChild<QLineEdit *>("goal_y")->text().toDouble(), 1e-6);
  EXPECT_NEAR(122, panel_->findChild<QLineEdit *>("goal_z")->text().toDouble(), 1e-6);
  EXPECT_NEAR(90, panel_->findChild<QLineEdit *>("goal_r")->text().toDouble(), 1e-6);
  EXPECT_EQ(0u, movj_count_);
  EXPECT_EQ(0u, goal_count_);
}

TEST_F(ControllerPanelTest, MovJConvertsUnitsAndBlocksJointMovJUntilCompletion)
{
  setPoseGoals({"100", "-50", "200", "90"});
  setGoals({"10", "20", "30", "-40"});
  setMode(RobotMode::ENABLE, "ENABLE");
  auto * send_pose = panel_->findChild<QPushButton *>("send_mov_j");
  ASSERT_TRUE(waitFor([&]() {return send_pose->isEnabled() && send_->isEnabled();}));
  send_pose->click();
  EXPECT_FALSE(send_pose->isEnabled());
  EXPECT_FALSE(send_->isEnabled());
  panel_->findChild<QTabWidget *>("motion_tabs")->setCurrentIndex(1);
  send_->click();
  ASSERT_TRUE(waitFor([this]() {return movj_handle_ && status_->text().contains("accepted");}));
  EXPECT_EQ(1u, movj_count_);
  EXPECT_EQ(0u, goal_count_);
  EXPECT_EQ("mg400_origin_link", last_movj_goal_.pose.header.frame_id);
  EXPECT_DOUBLE_EQ(0.1, last_movj_goal_.pose.pose.position.x);
  EXPECT_DOUBLE_EQ(-0.05, last_movj_goal_.pose.pose.position.y);
  EXPECT_DOUBLE_EQ(0.2, last_movj_goal_.pose.pose.position.z);
  EXPECT_NEAR(std::sqrt(0.5), last_movj_goal_.pose.pose.orientation.z, 1e-12);
  EXPECT_NEAR(std::sqrt(0.5), last_movj_goal_.pose.pose.orientation.w, 1e-12);
  EXPECT_FALSE(last_movj_goal_.set_speed_j);
  EXPECT_FALSE(last_movj_goal_.set_acc_j);
  EXPECT_FALSE(last_movj_goal_.set_cp);
  auto feedback = std::make_shared<MovJ::Feedback>();
  feedback->current_pose = last_movj_goal_.pose;
  feedback->current_pose.pose.position.x = 0.21;
  movj_handle_->publish_feedback(feedback);
  ASSERT_TRUE(
    waitFor(
      [this]() {
        return panel_->findChild<QLabel *>("current_x")->text() == "210.000";
      }));
  auto result = std::make_shared<MovJ::Result>();
  result->result = true;
  movj_handle_->succeed(result);
  ASSERT_TRUE(waitFor([this]() {return status_->text() == "MovJ succeeded.";}));
  EXPECT_TRUE(send_pose->isEnabled());
  EXPECT_TRUE(send_->isEnabled());
}

TEST_F(ControllerPanelTest, RejectsInvalidPoseInput)
{
  setMode(RobotMode::ENABLE, "ENABLE");
  auto * send_pose = panel_->findChild<QPushButton *>("send_mov_j");
  setPoseGoals({"100", "-50", "200", "90"});
  ASSERT_TRUE(waitFor([&]() {return send_pose->isEnabled();}));
  for (const auto * invalid : {"", "abc", "nan", "inf", "1e309"}) {
    setPoseGoals({"100", "-50", "200", invalid});
    EXPECT_FALSE(send_pose->isEnabled());
  }
  EXPECT_EQ(0u, movj_count_);
}

TEST_F(ControllerPanelTest, RobotServicesAreAsynchronousAndReportErrors)
{
  auto * enable = panel_->findChild<QPushButton *>("enable_robot");
  auto * disable = panel_->findChild<QPushButton *>("disable_robot");
  auto * clear = panel_->findChild<QPushButton *>("clear_error");
  auto * status = panel_->findChild<QLabel *>("service_status");
  setMode(RobotMode::DISABLED, "DISABLED");
  ASSERT_TRUE(waitFor([&]() {return enable->isEnabled();}));
  EXPECT_FALSE(disable->isEnabled());
  enable->click();
  EXPECT_EQ(0u, enable_count_);  // The button returns before the service is executed.
  EXPECT_FALSE(enable->isEnabled());
  enable->click();
  ASSERT_TRUE(waitFor([&]() {return status->text() == "Enable succeeded.";}));
  EXPECT_EQ(1u, enable_count_);
  EXPECT_EQ(Enable::Request::FOUR_PARAM, last_enable_request_.num_of_params);
  EXPECT_DOUBLE_EQ(0.0, last_enable_request_.load);
  EXPECT_DOUBLE_EQ(0.0, last_enable_request_.center_x);
  EXPECT_DOUBLE_EQ(0.0, last_enable_request_.center_y);
  EXPECT_DOUBLE_EQ(0.0, last_enable_request_.center_z);
  setMode(RobotMode::RUNNING, "RUNNING");
  EXPECT_TRUE(disable->isEnabled());
  EXPECT_FALSE(clear->isEnabled());
  disable->click();
  ASSERT_TRUE(waitFor([&]() {return status->text() == "Disable succeeded.";}));
  EXPECT_EQ(1u, disable_count_);
  setMode(RobotMode::ERROR, "ERROR");
  EXPECT_FALSE(enable->isEnabled());
  EXPECT_TRUE(clear->isEnabled());
  clear_success_ = false;
  clear->click();
  ASSERT_TRUE(waitFor([&]() {return status->text() == "Clear Error failed (error ID: 42).";}));
  EXPECT_EQ(1u, clear_count_);
  clear_success_ = true;
  clear->click();
  ASSERT_TRUE(waitFor([&]() {return status->text() == "Clear Error succeeded.";}));
  EXPECT_EQ(2u, clear_count_);
  clear_server_.reset();
  ASSERT_TRUE(waitFor([&]() {return !clear->isEnabled();}));
}

TEST_F(ControllerPanelTest, EnablesWithPayloadAndLocksInputsUntilDisabled)
{
  auto * enable = panel_->findChild<QPushButton *>("enable_robot");
  auto * status = panel_->findChild<QLabel *>("service_status");
  const QStringList names =
  {"enable_load", "enable_center_x", "enable_center_y", "enable_center_z"};
  const std::array<double, 4> values = {0.123, 45.678, -12.345, 10.001};
  EXPECT_EQ(nullptr, panel_->findChild<QCheckBox *>("set_enable_payload"));
  for (const auto & name : names) {
    auto * input = panel_->findChild<QLineEdit *>(name);
    ASSERT_NE(nullptr, input);
    EXPECT_TRUE(input->isEnabled());
  }
  setMode(RobotMode::DISABLED, "DISABLED");
  for (size_t i = 0; i < values.size(); ++i) {
    auto * input = panel_->findChild<QLineEdit *>(names[i]);
    EXPECT_TRUE(input->isEnabled());
    input->setText(QString::number(values[i], 'f', 3));
  }
  ASSERT_TRUE(waitFor([&]() {return enable->isEnabled();}));
  enable->click();
  for (const auto & name : names) {
    EXPECT_FALSE(panel_->findChild<QLineEdit *>(name)->isEnabled());
  }
  ASSERT_TRUE(waitFor([&]() {return status->text() == "Enable succeeded.";}));
  ASSERT_EQ(1u, enable_count_);
  EXPECT_EQ(Enable::Request::FOUR_PARAM, last_enable_request_.num_of_params);
  EXPECT_DOUBLE_EQ(values[0], last_enable_request_.load);
  EXPECT_DOUBLE_EQ(values[1], last_enable_request_.center_x);
  EXPECT_DOUBLE_EQ(values[2], last_enable_request_.center_y);
  EXPECT_DOUBLE_EQ(values[3], last_enable_request_.center_z);
  for (const auto mode : {RobotMode::ENABLE, RobotMode::RUNNING}) {
    setMode(mode, mode == RobotMode::ENABLE ? "ENABLE" : "RUNNING");
    for (const auto & name : names) {
      EXPECT_FALSE(panel_->findChild<QLineEdit *>(name)->isEnabled());
    }
  }
  setMode(RobotMode::DISABLED, "DISABLED");
  for (size_t i = 0; i < values.size(); ++i) {
    auto * input = panel_->findChild<QLineEdit *>(names[i]);
    EXPECT_TRUE(input->isEnabled());
    EXPECT_DOUBLE_EQ(values[i], input->text().toDouble());
  }
  panel_->findChild<QLineEdit *>("enable_load")->setText("0.25");
  enable->click();
  ASSERT_TRUE(waitFor([&]() {return status->text() == "Enable succeeded.";}));
  EXPECT_EQ(2u, enable_count_);
  EXPECT_EQ(Enable::Request::FOUR_PARAM, last_enable_request_.num_of_params);
  EXPECT_DOUBLE_EQ(0.25, last_enable_request_.load);
  EXPECT_DOUBLE_EQ(values[1], last_enable_request_.center_x);
  EXPECT_DOUBLE_EQ(values[2], last_enable_request_.center_y);
  EXPECT_DOUBLE_EQ(values[3], last_enable_request_.center_z);
}

TEST_F(ControllerPanelTest, RejectsInvalidPayloadBeforeEnabling)
{
  auto * enable = panel_->findChild<QPushButton *>("enable_robot");
  setMode(RobotMode::DISABLED, "DISABLED");
  ASSERT_TRUE(waitFor([&]() {return enable->isEnabled();}));
  const QStringList names =
  {"enable_load", "enable_center_x", "enable_center_y", "enable_center_z"};
  for (const auto & name : names) {
    auto * input = panel_->findChild<QLineEdit *>(name);
    for (const auto * invalid : {"", "abc", "nan", "inf", "1e309", "0.0001", "500.001",
        "-500.001"})
    {
      input->setText(invalid);
      EXPECT_FALSE(enable->isEnabled()) << name.toStdString() << ": " << invalid;
      enable->click();
    }
    if (name == "enable_load") {
      for (const auto * invalid : {"-0.001", "0.751"}) {
        input->setText(invalid);
        EXPECT_FALSE(enable->isEnabled());
        enable->click();
      }
      input->setText("0.75");
      EXPECT_TRUE(enable->isEnabled());
    } else {
      for (const auto * boundary : {"-500", "500"}) {
        input->setText(boundary);
        EXPECT_TRUE(enable->isEnabled());
      }
    }
    input->setText("0");
    EXPECT_TRUE(enable->isEnabled());
  }
  EXPECT_EQ(0u, enable_count_);
  panel_->findChild<QLineEdit *>("enable_load")->clear();
  EXPECT_FALSE(enable->isEnabled());
  enable->click();
  EXPECT_EQ(0u, enable_count_);
  panel_->findChild<QLineEdit *>("enable_load")->setText("0.25");
  EXPECT_TRUE(enable->isEnabled());
  enable->click();
  auto * status = panel_->findChild<QLabel *>("service_status");
  ASSERT_TRUE(waitFor([&]() {return status->text() == "Enable succeeded.";}));
  EXPECT_EQ(1u, enable_count_);
  EXPECT_EQ(Enable::Request::FOUR_PARAM, last_enable_request_.num_of_params);
  EXPECT_DOUBLE_EQ(0.25, last_enable_request_.load);
}

TEST_F(ControllerPanelTest, CollisionLevelRequiresApplyAndReportsServiceResults)
{
  auto * level = panel_->findChild<QComboBox *>("collision_level");
  auto * apply = panel_->findChild<QPushButton *>("set_collision_level");
  auto * status = panel_->findChild<QLabel *>("service_status");
  auto * tabs = panel_->findChild<QTabWidget *>("motion_tabs");
  ASSERT_NE(nullptr, level);
  ASSERT_NE(nullptr, apply);
  EXPECT_EQ(6, level->count());
  EXPECT_FALSE(level->isEditable());
  EXPECT_EQ(-1, level->currentIndex());
  EXPECT_FALSE(apply->isEnabled());
  setGoals({"10", "20", "30", "-40"});
  setPoseGoals({"100", "-50", "200", "90"});
  setMode(RobotMode::ENABLE, "ENABLE");
  EXPECT_FALSE(apply->isEnabled());
  for (int value = 0; value <= 5; ++value) {
    const int index = level->findData(value);
    ASSERT_GE(index, 0);
    level->setCurrentIndex(index);
    tabs->setCurrentIndex(2);
    tabs->setCurrentIndex(0);
    tabs->setCurrentIndex(2);
    ASSERT_TRUE(waitFor([&]() {return apply->isEnabled();}));
    EXPECT_EQ(static_cast<size_t>(value), collision_count_);
    apply->click();
    EXPECT_EQ(static_cast<size_t>(value), collision_count_);
    EXPECT_FALSE(apply->isEnabled());
    EXPECT_FALSE(level->isEnabled());
    EXPECT_FALSE(send_->isEnabled());
    EXPECT_FALSE(panel_->findChild<QPushButton *>("send_mov_j")->isEnabled());
    EXPECT_FALSE(panel_->findChild<QPushButton *>("clear_error")->isEnabled());
    apply->click();
    ASSERT_TRUE(
      waitFor(
        [&]() {
          return status->text() == QString("Collision level %1 succeeded.").arg(value);
        }));
    EXPECT_EQ(static_cast<size_t>(value + 1), collision_count_);
    EXPECT_EQ(value, last_collision_request_.level.level);
    EXPECT_TRUE(apply->isEnabled());
    EXPECT_TRUE(level->isEnabled());
    EXPECT_TRUE(send_->isEnabled());
  }
  collision_success_ = false;
  level->setCurrentIndex(level->findData(3));
  apply->click();
  ASSERT_TRUE(
    waitFor(
      [&]() {
        return status->text() == "Collision level 3 failed (error ID: 42).";
      }));
  EXPECT_EQ(7u, collision_count_);
  EXPECT_TRUE(apply->isEnabled());
  EXPECT_TRUE(level->isEnabled());
  EXPECT_EQ(0u, goal_count_);
  EXPECT_EQ(0u, movj_count_);
}

TEST_F(ControllerPanelTest, CollisionLevelWaitsForIdleRobotAndAvailableService)
{
  auto * level = panel_->findChild<QComboBox *>("collision_level");
  auto * apply = panel_->findChild<QPushButton *>("set_collision_level");
  level->setCurrentIndex(level->findData(2));
  EXPECT_FALSE(apply->isEnabled());
  setMode(RobotMode::RUNNING, "RUNNING");
  EXPECT_FALSE(apply->isEnabled());
  EXPECT_FALSE(level->isEnabled());
  setMode(RobotMode::ERROR, "ERROR");
  EXPECT_FALSE(apply->isEnabled());
  setMode(RobotMode::PAUSE, "PAUSE");
  EXPECT_FALSE(apply->isEnabled());
  setMode(RobotMode::DISABLED, "DISABLED");
  ASSERT_TRUE(waitFor([&]() {return apply->isEnabled();}));
  auto * enable = panel_->findChild<QPushButton *>("enable_robot");
  enable->click();
  EXPECT_FALSE(apply->isEnabled());
  EXPECT_FALSE(level->isEnabled());
  apply->click();
  ASSERT_TRUE(waitFor([&]() {return apply->isEnabled();}));
  setMode(RobotMode::ENABLE, "ENABLE");
  setGoals({"10", "20", "30", "-40"});
  ASSERT_TRUE(waitFor([this]() {return send_->isEnabled();}));
  send_->click();
  EXPECT_FALSE(apply->isEnabled());
  EXPECT_FALSE(level->isEnabled());
  apply->click();
  ASSERT_TRUE(waitFor([this]() {return handle_ && status_->text().contains("accepted");}));
  EXPECT_FALSE(apply->isEnabled());
  EXPECT_TRUE(panel_->findChild<QPushButton *>("disable_robot")->isEnabled());
  auto result = std::make_shared<Action::Result>();
  result->result = true;
  handle_->succeed(result);
  ASSERT_TRUE(waitFor([&]() {return apply->isEnabled();}));
  EXPECT_TRUE(level->isEnabled());
  collision_server_.reset();
  ASSERT_TRUE(waitFor([&]() {return !apply->isEnabled();}));
  EXPECT_EQ(0u, collision_count_);
}

TEST_F(ControllerPanelTest, RecoversAfterRejectionAndDisplaysErrors)
{
  reject_ = true;
  setGoals({"10", "20", "30", "-40"});
  setMode(RobotMode::ENABLE, "ENABLE");
  ASSERT_TRUE(waitFor([this]() {return send_->isEnabled();}));
  send_->click();
  ASSERT_TRUE(waitFor([this]() {return status_->text().contains("rejected");}));
  EXPECT_TRUE(send_->isEnabled());
  reject_ = false;
  send_->click();
  ASSERT_TRUE(waitFor([this]() {return handle_ && status_->text().contains("accepted");}));
  auto result = std::make_shared<Action::Result>();
  result->result = false;
  result->error_id.controller.ids = {18};
  result->error_id.servo[2].ids = {42};
  handle_->abort(result);
  ASSERT_TRUE(waitFor([this]() {return status_->text().contains("aborted");}));
  EXPECT_TRUE(status_->text().contains("controller: 18"));
  EXPECT_TRUE(status_->text().contains("servo 3: 42"));
  EXPECT_TRUE(send_->isEnabled());
}

TEST(ControllerPluginTest, LoadsThroughPluginlib)
{
  pluginlib::ClassLoader<rviz_common::Panel> loader("rviz_common", "rviz_common::Panel");
  auto panel = loader.createSharedInstance("mg400_rviz_plugin/Mg400Controller");
  EXPECT_NE(nullptr, panel->findChild<QPushButton *>("send_joint_mov_j"));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  QApplication application(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
