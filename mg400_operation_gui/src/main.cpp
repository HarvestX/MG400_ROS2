// Copyright 2026 Yuki Yamamoto
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

#include <QApplication>
#include <QTimer>

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "mg400_operation_gui/main_window.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  QApplication application(argc, argv);

  auto window = std::make_shared<mg400_operation_gui::MainWindow>();
  auto node = std::static_pointer_cast<rclcpp::Node>(window);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  QTimer ros_timer;
  QObject::connect(
    &ros_timer, &QTimer::timeout, [&application, &executor]() {
      if (!rclcpp::ok()) {
        application.quit();
        return;
      }
      executor.spin_some();
    });
  ros_timer.start(10);

  window->show();
  const int result = application.exec();

  ros_timer.stop();
  executor.remove_node(node);
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return result;
}
