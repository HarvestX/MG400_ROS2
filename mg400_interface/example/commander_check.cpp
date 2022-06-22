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

#include <string>
#include "mg400_interface/tcp_interface/dashboard_tcp_interface.hpp"
#include "mg400_interface/commander/dashboard_commander.hpp"


void show_result(const bool res, const std::string & command)
{
  if (res) {
    std::cout << command << " successfully called!" << std::endl;
  } else {
    std::cerr << command << " failed" << std::endl;
  }
}


int main(int argc, char ** argv)
{
  using namespace std::chrono_literals;

  std::string ip = "127.0.0.1";
  if (argc == 2) {
    ip = argv[1];
  }

  std::cout << "Connecting to :" << ip << std::endl;


  auto db_tcp_if =
    std::make_unique<mg400_interface::DashboardTcpInterface>(ip);

  db_tcp_if->init();

  while (!db_tcp_if->isConnected()) {
    std::cout << "Waiting for the connection..." << std::endl;
    rclcpp::sleep_for(1s);
  }

  auto db_commander =
    std::make_unique<mg400_interface::DashboardCommander>(db_tcp_if.get());

  auto res = db_commander->getErrorId();
  for (auto eids : res) {
    for (auto eid : eids) {
      std::cout << eid << ", ";
    }
    std::cout << std::endl;
  }

  const bool enable_res = db_commander->enableRobot();
  show_result(enable_res, "EnableRobot");

  rclcpp::sleep_for(2s);

  const bool clearerr_res = db_commander->clearError();
  show_result(clearerr_res, "ClearError");

  rclcpp::sleep_for(2s);

  const bool resetrobo_res = db_commander->resetRobot();
  show_result(resetrobo_res, "ResetRobot");

  rclcpp::sleep_for(2s);

  const bool speedfactor_res = db_commander->speedFactor(10);
  show_result(speedfactor_res, "SpeedFactor");

  rclcpp::sleep_for(2s);

  const bool disable_res = db_commander->disableRobot();
  show_result(disable_res, "DisableRobot");

  rclcpp::sleep_for(1s);

  return EXIT_SUCCESS;
}
