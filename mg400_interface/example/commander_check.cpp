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
#include <memory>
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

class CommanderCheckNode : public rclcpp::Node
{
public:
  std::unique_ptr<mg400_interface::DashboardCommander> db_commander;

private:
  const std::string ip_address;
  std::unique_ptr<mg400_interface::DashboardTcpInterface> db_tcp_if_;

public:
  explicit CommanderCheckNode(const rclcpp::NodeOptions & options)
  : Node("commander_check_node", options),
    ip_address(this->declare_parameter("ip_address", "127.0.0.1"))
  {
    this->db_tcp_if_ =
      std::make_unique<mg400_interface::DashboardTcpInterface>(
      this->ip_address);
  }

  bool activate()
  {
    this->db_tcp_if_->init();
    RCLCPP_INFO(
      this->get_logger(),
      "Connecting to: %s", this->ip_address.c_str());

    using namespace std::chrono_literals;
    while (!this->db_tcp_if_->isConnected()) {
      std::cout << "Waiting for the connection..." << std::endl;
      rclcpp::sleep_for(1s);
    }

    const bool ret = this->db_tcp_if_->isConnected();
    if (ret) {
      this->db_commander =
        std::make_unique<mg400_interface::DashboardCommander>(db_tcp_if_.get());
    }

    return ret;
  }

  bool deactivate()
  {
    this->db_commander = nullptr;
    return true;
  }
};


int main(int argc, char ** argv)
{
  using namespace std::chrono_literals;
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto ck_node = std::make_unique<CommanderCheckNode>(options);
  ck_node->activate();

  const bool enable_res = ck_node->db_commander->enableRobot();
  show_result(enable_res, "EnableRobot");
  rclcpp::sleep_for(2s);

  const bool res = ck_node->db_commander->tool(0);
  show_result(res, "Tool");
  rclcpp::sleep_for();

  const bool disable_res = ck_node->db_commander->disableRobot();
  show_result(disable_res, "DisableRobot");
  rclcpp::sleep_for(1s);

  ck_node->deactivate();

  return EXIT_SUCCESS;
}
