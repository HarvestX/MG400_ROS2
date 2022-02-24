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
#include <QApplication>

MainWindow::MainWindow(
  const rclcpp::NodeOptions & node_options,
  QWidget * parent
)
: QWidget(parent),
  rclcpp::Node("joint_state_publisher_gui", node_options),
  ui_(new Ui::MainWindow)
{
  this->joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "joint_states",
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile()
  );

  ui_->setupUi(this);

  ui_->j1_txt->setText("0.000");
  ui_->j2_txt->setText("0.000");
  ui_->j3_txt->setText("0.000");
  ui_->j4_txt->setText("0.000");
  j1_ = 0.000;
  j2_ = 0.000;
  j3_ = 0.000;
  j4_ = 0.000;

  ui_->j1_slider->setValue(std::abs(J1_MIN) / (J1_MAX - J1_MIN) * 1000);
  ui_->j2_slider->setValue(std::abs(J2_MIN) / (J2_MAX - J2_MIN) * 1000);
  ui_->j3_slider->setValue(std::abs(J3_MIN) / (J3_MAX - J3_MIN) * 1000);
  ui_->j4_slider->setValue(std::abs(J4_MIN) / (J4_MAX - J4_MIN) * 1000);
  connect(ui_->j1_slider, &QSlider::valueChanged, this, &MainWindow::j1ValueChange);
  connect(ui_->j2_slider, &QSlider::valueChanged, this, &MainWindow::j2ValueChange);
  connect(ui_->j3_slider, &QSlider::valueChanged, this, &MainWindow::j3ValueChange);
  connect(ui_->j4_slider, &QSlider::valueChanged, this, &MainWindow::j4ValueChange);

  connect(ui_->random_btn, &QPushButton::clicked, this, &MainWindow::randomBtn);
  connect(ui_->center_btn, &QPushButton::clicked, this, &MainWindow::centerBtn);
}

MainWindow::~MainWindow()
{
  delete this->ui_;
}


void MainWindow::randomBtn()
{

}

void MainWindow::centerBtn()
{

}

void MainWindow::j1ValueChange(int value)
{

}

void MainWindow::j2ValueChange(int value)
{
}

void MainWindow::j3ValueChange(int value)
{

}

void MainWindow::j4ValueChange(int value)
{

}

void MainWindow::publishJointStates()
{
  // TODO: Implement here
}
