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

#include <QApplication>
#include "main_window.hpp"

int main(int argc, char * argv[])
{
  // Initialize client library
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Generate node
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  QApplication app(argc, argv);
  MainWindow main_window(options);
  main_window.show();

  return QApplication::exec();
}
