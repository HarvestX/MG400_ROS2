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

#include "main_window.hpp"
#include "ui_main_window.h"
#include "mg400_control/convert.hpp"
#include <QApplication>

MainWindow::MainWindow(
  const rclcpp::NodeOptions & node_options,
  QWidget * parent
)
: QWidget(parent),
  rclcpp::Node("joint_state_publisher_gui", node_options),
  ui_(new Ui::MainWindow)
{
  using namespace std::chrono_literals;
  this->joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "joint_states",
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile()
  );
  this->timer_ = this->create_wall_timer(100ms, std::bind(&MainWindow::publishJointStates, this));

  this->ui_->setupUi(this);

  this->ui_->j1_txt->setText("0.000");
  this->ui_->j2_txt->setText("0.000");
  this->ui_->j3_txt->setText("0.000");
  this->ui_->j4_txt->setText("0.000");

  this->j1_ = 0.000;
  this->j2_ = 0.000;
  this->j3_ = 0.000;
  this->j4_ = 0.000;

  this->ui_->j1_slider->setValue(std::abs(J1_MIN) / (J1_MAX - J1_MIN) * 1000);
  this->ui_->j2_slider->setValue(std::abs(J2_MIN) / (J2_MAX - J2_MIN) * 1000);
  this->ui_->j3_slider->setValue(std::abs(J3_MIN) / (J3_MAX - J3_MIN) * 1000);
  this->ui_->j4_slider->setValue(std::abs(J4_MIN) / (J4_MAX - J4_MIN) * 1000);

  this->connect(this->ui_->j1_slider, &QSlider::valueChanged, this, &MainWindow::j1ValueChange);
  this->connect(this->ui_->j2_slider, &QSlider::valueChanged, this, &MainWindow::j2ValueChange);
  this->connect(this->ui_->j3_slider, &QSlider::valueChanged, this, &MainWindow::j3ValueChange);
  this->connect(this->ui_->j4_slider, &QSlider::valueChanged, this, &MainWindow::j4ValueChange);

  this->connect(this->ui_->random_btn, &QPushButton::clicked, this, &MainWindow::randomBtn);
  this->connect(this->ui_->center_btn, &QPushButton::clicked, this, &MainWindow::centerBtn);
}

MainWindow::~MainWindow()
{
  delete this->ui_;
}


void MainWindow::randomBtn()
{
  int32_t val;

  val = random() % 1000;
  j1ValueChange(val);
  ui_->j1_slider->setValue(val);

  val = random() % 1000;
  j2ValueChange(val);
  ui_->j2_slider->setValue(val);

  val = random() % 1000;
  j3ValueChange(val);
  ui_->j3_slider->setValue(val);

  val = random() % 1000;
  j4ValueChange(val);
  ui_->j4_slider->setValue(val);
}

void MainWindow::centerBtn()
{
  auto get_center = [](const double & min, const double & max) -> double {
      return std::abs(min) / (max - min) * 1000;
    };

  int j1_center = get_center(J1_MIN, J1_MAX);
  int j2_center = get_center(J2_MIN, J2_MAX);
  int j3_center = get_center(J3_MIN, J3_MAX);
  int j4_center = get_center(J4_MIN, J4_MAX);

  this->j1ValueChange(j1_center);
  this->j2ValueChange(j2_center);
  this->j3ValueChange(j3_center);
  this->j4ValueChange(j4_center);

  this->ui_->j1_slider->setValue(j1_center);
  this->ui_->j2_slider->setValue(j2_center);
  this->ui_->j3_slider->setValue(j3_center);
  this->ui_->j4_slider->setValue(j4_center);
}

void MainWindow::j1ValueChange(int value)
{
  char txt[50];
  this->j1_ = J1_MIN + (J1_MAX - J1_MIN) * value * 1e-3;
  sprintf(txt, "%0.3f", this->j1_);
  ui_->j1_txt->setText(txt);
}

void MainWindow::j2ValueChange(int value)
{
  char txt[50];
  this->j2_ = J2_MIN + (J2_MAX - J2_MIN) * value * 1e-3;
  sprintf(txt, "%0.3f", this->j2_);
  ui_->j2_txt->setText(txt);
}

void MainWindow::j3ValueChange(int value)
{
  char txt[50];
  this->j3_ = J3_MIN + (J3_MAX - J3_MIN) * value * 1e-3;
  sprintf(txt, "%0.3f", this->j3_);
  ui_->j3_txt->setText(txt);
}

void MainWindow::j4ValueChange(int value)
{
  char txt[50];
  this->j4_ = J4_MIN + (J4_MAX - J4_MIN) * value * 1e-3;
  sprintf(txt, "%0.3f", this->j4_);
  ui_->j4_txt->setText(txt);
}

void MainWindow::publishJointStates()
{
  this->joint_state_pub_->publish(
    convert::toJointState(
      this->j1_,
      this->j2_,
      this->j3_,
      this->j4_
  ));
}
