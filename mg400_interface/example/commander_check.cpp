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


#include "mg400_interface/tcp_interface/dashboard_tcp_interface.hpp"
#include "mg400_interface/commander/dashboard_commander.hpp"


int main(int argc, char ** argv)
{
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
    using namespace std::chrono_literals;
    rclcpp::sleep_for(1s);
  }

  auto db_commander =
    std::make_unique<mg400_interface::DashboardCommander>(db_tcp_if.get());


  db_commander->enableRobot();

  {
    using namespace std::chrono_literals;
    rclcpp::sleep_for(1s);
  }

  db_commander->disableRobot();

  return EXIT_SUCCESS;
}
