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

#pragma once

#include <rclcpp/rclcpp.hpp>
#include "mg400_control/mg400_control/tcp_socket.hpp"

namespace mg400_control
{

#pragma pack(push, 1)
struct RealTimeData
{
  uint16_t len;                        // 0000 ~ 0001 //
  uint16_t reserve[3];                 // 0002 ~ 0007 //
  uint64_t digital_input_bits;         // 0008 ~ 0015 //
  uint64_t digital_outputs;            // 0016 ~ 0023 //
  uint64_t robot_mode;                 // 0024 ~ 0031 //
  uint64_t controller_timer;           // 0032 ~ 0039 //
  uint64_t run_time;                   // 0040 ~ 0047 //
  // 0048 ~ 0095 //
  uint64_t test_value;                 // 0048 ~ 0055 //
  double safety_mode;                  // 0056 ~ 0063 //
  double speed_scaling;                // 0064 ~ 0071 //
  double linear_momentum_norm;         // 0072 ~ 0079 //
  double v_main;                       // 0080 ~ 0087 //
  double v_robot;                      // 0088 ~ 0095 //
  // 0096 ~ 0143 //
  double i_robot;                      // 0096 ~ 0103 //
  double program_state;                // 0104 ~ 0111 //
  double safety_status;                // 0112 ~ 0119 //
  double tool_accelerometer_values[3]; // 0120 ~ 0143 //
  // 0144 ~ 0191 //
  double elbow_position[3];            // 0144 ~ 0167 //
  double elbow_velocity[3];            // 0168 ~ 0191 //
  // 0192 ~ 1439 //
  double q_target[6];                  // 0192 ~ 0239 //
  double qd_target[6];                 // 0240 ~ 0287 //
  double qdd_target[6];                // 0288 ~ 0335 //
  double i_target[6];                  // 0336 ~ 0383 //
  double m_target[6];                  // 0384 ~ 0431 //
  double q_actual[6];                  // 0432 ~ 0479 //
  double qd_actual[6];                 // 0480 ~ 0527 //
  double i_actual[6];                  // 0528 ~ 0575 //
  double i_control[6];                 // 0576 ~ 0623 //
  double tool_vector_actual[6];        // 0624 ~ 0671 //
  double TCP_speed_actual[6];          // 0672 ~ 0719 //
  double TCP_force[6];                 // 0720 ~ 0767 //
  double Tool_vector_target[6];        // 0768 ~ 0815 //
  double TCP_speed_target[6];          // 0816 ~ 0863 //
  double motor_temperatures[6];        // 0864 ~ 0911 //
  double joint_modes[6];               // 0912 ~ 0959 //
  double v_actual[6];                  // 960  ~ 1007 //
  double dummy[9][6];                  // 1008 ~ 1439 //
};
#pragma pack(pop)


class Commander
{
private:
  std::mutex mutex_;
  double current_joints_[6];
  double tool_vector_[6];
  RealTimeData real_time_data_;
  std::atomic<bool> is_running_;
  std::unique_ptr<std::thread> thread_;
  std::shared_ptr<TcpClient> real_time_tcp_;
  std::shared_ptr<TcpClient> dash_board_tcp_;

public:
  explicit Commander(const std::string &);
  ~Commander();

  void getCurrentJointStates(double *);

  void getToolVectorActual(double *);

  void recvTask() noexcept;

  void init() noexcept;

  bool isEnabled() const noexcept;

  bool isConnected() const noexcept;

  void enableRobot();

  void disableRobot();

  void clearError();

  void resetRobot();

  void speedFactor(int);

  void movJ(double, double, double, double, double, double);

  void movL(double, double, double, double, double, double);

  void jointMovJ(double, double, double, double, double, double);

  void moveJog(const std::string &);

  void relMovJ(double, double, double, double, double, double);

  void relMovL(double, double, double);

  void servoJ(double, double, double, double, double, double);

  void servoP(double, double, double, double, double, double);

  void dashSendCmd(const char *, uint32_t);

  void realSendCmd(const char *, uint32_t);

private:
  static inline double rad2Deg(double rad)
  {
    return rad * 180.0 / M_PI;
  }

  static inline double deg2Rad(double deg)
  {
    return deg * M_PI / 180.0;
  }
};
} // namespace mg400_control
