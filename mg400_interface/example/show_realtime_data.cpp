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

#include <cinttypes>
#include "mg400_interface/tcp_interface/realtime_feedback_tcp_interface.hpp"
#include "mg400_interface/tcp_interface/dashboard_tcp_interface.hpp"

int main(int argc, char ** argv)
{
  std::string ip = "127.0.0.1";
  if (argc == 2) {
    ip = argv[1];
  }

  std::cout << "Connecting to: " << ip << std::endl;

  auto rt_tcp_if =
    std::make_unique<mg400_interface::RealtimeFeedbackTcpInterface>(ip);
  auto db_tcp_if =
    std::make_unique<mg400_interface::DashboardTcpInterface>(ip);

  rt_tcp_if->init();
  db_tcp_if->init();

  while (!rt_tcp_if->isConnected() || !db_tcp_if->isConnected()) {
    std::cout << "Waiting for the connection..." << std::endl;
    using namespace std::chrono_literals;
    rclcpp::sleep_for(1s);
  }

  while (true) {
    auto data = rt_tcp_if->getRealtimeData();
    if (!data) {continue};

    auto tmp = std::system("clear");
    (void)tmp;  // for compiler warning

    printf(
      "len:\t\t\t\t%" PRIu16 "\n",
      data->len);
    printf(
      "digital inputs:\t\t\t%" PRIu64 "\n",
      data->digital_inputs);
    printf(
      "digital outputs:\t\t%" PRIu64 "\n",
      data->digital_outputs);
    printf(
      "robot mode:\t\t\t%" PRIu64 "\n",
      data->robot_mode);
    printf(
      "test value:\t\t\t%" PRIu64 "\n",
      data->test_value);
    printf(
      "speed_scaling:\t\t\t%.3lf\n",
      data->speed_scaling);
    printf(
      "q_target:\t\t\t"
      "[%.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf]\n",
      data->q_target[0], data->q_target[1],
      data->q_target[2], data->q_target[3],
      data->q_target[4], data->q_target[5]);
    printf(
      "qd_target:\t\t\t"
      "[%.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf]\n",
      data->qd_target[0], data->qd_target[1],
      data->qd_target[2], data->qd_target[3],
      data->qd_target[4], data->qd_target[5]);
    printf(
      "qdd_target:\t\t\t"
      "[%.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf]\n",
      data->qdd_target[0], data->qdd_target[1],
      data->qdd_target[2], data->qdd_target[3],
      data->qdd_target[4], data->qdd_target[5]);
    printf(
      "i_target:\t\t\t"
      "[%.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf]\n",
      data->i_target[0], data->i_target[1],
      data->i_target[2], data->i_target[3],
      data->i_target[4], data->i_target[5]);
    printf(
      "m_target:\t\t\t"
      "[%.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf]\n",
      data->m_target[0], data->m_target[1],
      data->m_target[2], data->m_target[3],
      data->m_target[4], data->m_target[5]);
    printf(
      "q_actual:\t\t\t"
      "[%.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf]\n",
      data->q_actual[0], data->q_actual[1],
      data->q_actual[2], data->q_actual[3],
      data->q_actual[4], data->q_actual[5]);
    printf(
      "qd_actual:\t\t\t"
      "[%.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf]\n",
      data->qd_actual[0], data->qd_actual[1],
      data->qd_actual[2], data->qd_actual[3],
      data->qd_actual[4], data->qd_actual[5]);
    printf(
      "i_actual:\t\t\t"
      "[%.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf]\n",
      data->i_actual[0], data->i_actual[1],
      data->i_actual[2], data->i_actual[3],
      data->i_actual[4], data->i_actual[5]);
    printf(
      "actual_i_TCP_force:\t\t"
      "[%.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf]\n",
      data->actual_i_TCP_force[0], data->actual_i_TCP_force[1],
      data->actual_i_TCP_force[2], data->actual_i_TCP_force[3],
      data->actual_i_TCP_force[4], data->actual_i_TCP_force[5]);
    printf(
      "tool_vector_actual:\t\t"
      "[%.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf]\n",
      data->tool_vector_actual[0],
      data->tool_vector_actual[1],
      data->tool_vector_actual[2],
      data->tool_vector_actual[3],
      data->tool_vector_actual[4],
      data->tool_vector_actual[5]);
    printf(
      "TCP_speed_actual:\t\t"
      "[%.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf]\n",
      data->TCP_speed_actual[0],
      data->TCP_speed_actual[1],
      data->TCP_speed_actual[2],
      data->TCP_speed_actual[3],
      data->TCP_speed_actual[4],
      data->TCP_speed_actual[5]);
    printf(
      "TCP_force:\t\t\t"
      "[%.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf]\n",
      data->TCP_force[0], data->TCP_force[1],
      data->TCP_force[2], data->TCP_force[3],
      data->TCP_force[4], data->TCP_force[5]);
    printf(
      "tool_vector_target:\t\t"
      "[%.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf]\n",
      data->tool_vector_target[0],
      data->tool_vector_target[1],
      data->tool_vector_target[2],
      data->tool_vector_actual[3],
      data->tool_vector_target[4],
      data->tool_vector_target[5]);
    printf(
      "TCP_speed_target:\t\t"
      "[%.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf]\n",
      data->TCP_speed_target[0],
      data->TCP_speed_target[1],
      data->TCP_speed_target[2],
      data->TCP_speed_target[3],
      data->TCP_speed_target[4],
      data->TCP_speed_target[5]);
    printf(
      "load:\t\t\t\t%.3lf\n",
      data->load);
    printf(
      "center_x:\t\t\t%.3lf\n",
      data->center_x);
    printf(
      "center_y:\t\t\t%.3lf\n",
      data->center_y);
    printf(
      "center_z:\t\t\t%.3lf\n",
      data->center_z);
    using namespace std::chrono_literals;
    rclcpp::sleep_for(100ms);
  }

  return EXIT_SUCCESS;
}
