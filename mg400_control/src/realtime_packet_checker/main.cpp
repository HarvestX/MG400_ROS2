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
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "mg400_control/mg400_interface/commander.hpp"


using namespace std::chrono_literals;
class RealtimePacketChecker : public rclcpp::Node
{
private:
  std::unique_ptr<mg400_interface::Commander> commander_;
  rclcpp::TimerBase::SharedPtr timer_;

public:
  RealtimePacketChecker(
    const rclcpp::NodeOptions & options
  )
  : Node("mg400_realtime_packet_checker", options)
  {
    std::string ip;
    this->declare_parameter<std::string>("ip_address", "127.0.0.1");
    this->get_parameter("ip_address", ip);
    RCLCPP_INFO(
      this->get_logger(),
      "IP address: %s", ip.c_str());

    commander_ = std::make_unique<mg400_interface::Commander>(ip);
    commander_->init();

    this->timer_ = this->create_wall_timer(
      100ms,
      std::bind(&RealtimePacketChecker::packet_visualizer, this));
  }

  void packet_visualizer()
  {
    auto tmp = std::system("clear");
    (void)tmp;  // for unused warnings
    auto real_time_data =
      std::make_unique<mg400_interface::RealTimeData>(
      this->commander_->getRealTimeData());

    std::cout << std::endl;
    printf(
      "len:\t\t\t\t%" PRIu16 "\n",
      real_time_data->len);
    printf(
      "digital input bits:\t\t%" PRIu64 "\n",
      real_time_data->digital_input_bits);
    printf(
      "digital outputs:\t\t%" PRIu64 "\n",
      real_time_data->digital_outputs);
    printf(
      "robot mode:\t\t\t%" PRIu64 "\n",
      real_time_data->robot_mode);
    printf(
      "controller timer:\t\t%" PRIu64 "\n",
      real_time_data->controller_timer);
    printf(
      "run time:\t\t\t%" PRIu64 "\n",
      real_time_data->run_time);
    printf(
      "test value:\t\t\t%" PRIu64 "\n",
      real_time_data->test_value);
    printf(
      "safety mode:\t\t\t%.3lf\n",
      real_time_data->safety_mode);
    printf(
      "speed_scaling:\t\t\t%.3lf\n",
      real_time_data->speed_scaling);
    printf(
      "linear_momentum_norm:\t\t%.3lf\n",
      real_time_data->linear_momentum_norm);
    printf(
      "v_main:\t\t\t\t%.3lf\n",
      real_time_data->v_main);
    printf(
      "v_robot:\t\t\t%.3lf\n",
      real_time_data->v_robot);
    printf(
      "i_robot:\t\t\t%.3lf\n",
      real_time_data->i_robot);
    printf(
      "program_state:\t\t\t%.3lf\n",
      real_time_data->program_state);
    printf(
      "safety_status:\t\t\t%.3lf\n",
      real_time_data->safety_status);
    printf(
      "tool_accelerometer_values:\t[%.3lf, %.3lf, %.3lf]\n",
      real_time_data->tool_accelerometer_values[0],
      real_time_data->tool_accelerometer_values[1],
      real_time_data->tool_accelerometer_values[2]);
    printf(
      "elbow_position:\t\t\t[%.3lf, %.3lf, %.3lf]\n",
      real_time_data->elbow_position[0],
      real_time_data->elbow_position[1],
      real_time_data->elbow_position[2]);
    printf(
      "elbow_velocity:\t\t\t[%.3lf, %.3lf, %.3lf]\n",
      real_time_data->elbow_velocity[0],
      real_time_data->elbow_velocity[1],
      real_time_data->elbow_velocity[2]);
    printf(
      "q_target:\t\t\t"
      "[%.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf]\n",
      real_time_data->q_target[0], real_time_data->q_target[1],
      real_time_data->q_target[2], real_time_data->q_target[3],
      real_time_data->q_target[4], real_time_data->q_target[5]);
    printf(
      "qd_target:\t\t\t"
      "[%.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf]\n",
      real_time_data->qd_target[0], real_time_data->qd_target[1],
      real_time_data->qd_target[2], real_time_data->qd_target[3],
      real_time_data->qd_target[4], real_time_data->qd_target[5]);
    printf(
      "qdd_target:\t\t\t"
      "[%.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf]\n",
      real_time_data->qdd_target[0], real_time_data->qdd_target[1],
      real_time_data->qdd_target[2], real_time_data->qdd_target[3],
      real_time_data->qdd_target[4], real_time_data->qdd_target[5]);
    printf(
      "i_target:\t\t\t"
      "[%.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf]\n",
      real_time_data->i_target[0], real_time_data->i_target[1],
      real_time_data->i_target[2], real_time_data->i_target[3],
      real_time_data->i_target[4], real_time_data->i_target[5]);
    printf(
      "m_target:\t\t\t"
      "[%.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf]\n",
      real_time_data->m_target[0], real_time_data->m_target[1],
      real_time_data->m_target[2], real_time_data->m_target[3],
      real_time_data->m_target[4], real_time_data->m_target[5]);
    printf(
      "q_actual:\t\t\t"
      "[%.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf]\n",
      real_time_data->q_actual[0], real_time_data->q_actual[1],
      real_time_data->q_actual[2], real_time_data->q_actual[3],
      real_time_data->q_actual[4], real_time_data->q_actual[5]);
    printf(
      "qd_actual:\t\t\t"
      "[%.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf]\n",
      real_time_data->qd_actual[0], real_time_data->qd_actual[1],
      real_time_data->qd_actual[2], real_time_data->qd_actual[3],
      real_time_data->qd_actual[4], real_time_data->qd_actual[5]);
    printf(
      "i_actual:\t\t\t"
      "[%.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf]\n",
      real_time_data->i_actual[0], real_time_data->i_actual[1],
      real_time_data->i_actual[2], real_time_data->i_actual[3],
      real_time_data->i_actual[4], real_time_data->i_actual[5]);
    printf(
      "i_control:\t\t\t"
      "[%.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf]\n",
      real_time_data->i_control[0], real_time_data->i_control[1],
      real_time_data->i_control[2], real_time_data->i_control[3],
      real_time_data->i_control[4], real_time_data->i_control[5]);
    printf(
      "tool_vector_actual:\t\t"
      "[%.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf]\n",
      real_time_data->tool_vector_actual[0],
      real_time_data->tool_vector_actual[1],
      real_time_data->tool_vector_actual[2],
      real_time_data->tool_vector_actual[3],
      real_time_data->tool_vector_actual[4],
      real_time_data->tool_vector_actual[5]);
    printf(
      "TCP_speed_actual:\t\t"
      "[%.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf]\n",
      real_time_data->TCP_speed_actual[0],
      real_time_data->TCP_speed_actual[1],
      real_time_data->TCP_speed_actual[2],
      real_time_data->TCP_speed_actual[3],
      real_time_data->TCP_speed_actual[4],
      real_time_data->TCP_speed_actual[5]);
    printf(
      "TCP_force:\t\t\t"
      "[%.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf]\n",
      real_time_data->TCP_force[0], real_time_data->TCP_force[1],
      real_time_data->TCP_force[2], real_time_data->TCP_force[3],
      real_time_data->TCP_force[4], real_time_data->TCP_force[5]);
    printf(
      "tool_vector_target:\t\t"
      "[%.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf]\n",
      real_time_data->tool_vector_target[0],
      real_time_data->tool_vector_target[1],
      real_time_data->tool_vector_target[2],
      real_time_data->tool_vector_actual[3],
      real_time_data->tool_vector_target[4],
      real_time_data->tool_vector_target[5]);
    printf(
      "TCP_speed_target:\t\t"
      "[%.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf]\n",
      real_time_data->TCP_speed_target[0],
      real_time_data->TCP_speed_target[1],
      real_time_data->TCP_speed_target[2],
      real_time_data->TCP_speed_target[3],
      real_time_data->TCP_speed_target[4],
      real_time_data->TCP_speed_target[5]);
    printf(
      "motor_temperatures:\t\t"
      "[%.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf]\n",
      real_time_data->motor_temperatures[0],
      real_time_data->motor_temperatures[1],
      real_time_data->motor_temperatures[2],
      real_time_data->motor_temperatures[3],
      real_time_data->motor_temperatures[4],
      real_time_data->motor_temperatures[5]);
    printf(
      "joint_modes:\t\t\t"
      "[%.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf]\n",
      real_time_data->joint_modes[0], real_time_data->joint_modes[1],
      real_time_data->joint_modes[2], real_time_data->joint_modes[3],
      real_time_data->joint_modes[4], real_time_data->joint_modes[5]);
    printf(
      "v_actual:\t\t\t"
      "[%.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf]\n",
      real_time_data->v_actual[0], real_time_data->v_actual[1],
      real_time_data->v_actual[2], real_time_data->v_actual[3],
      real_time_data->v_actual[4], real_time_data->v_actual[5]);
  }
};


int main(int argc, char * argv[])
{
  // Initialize client library
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Generate node
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  auto node = std::make_shared<RealtimePacketChecker>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
