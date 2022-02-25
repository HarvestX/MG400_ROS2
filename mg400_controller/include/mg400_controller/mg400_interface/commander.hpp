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
#include "mg400_controller/mg400_interface/tcp_socket.hpp"

namespace mg400_interface
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

static const rclcpp::Logger LOGGER = rclcpp::get_logger("MG400Commander");

class Commander
{
private:
  std::mutex mutex_;
  double current_joint_[6];
  double tool_vector_[6];
  RealTimeData real_time_data_;
  std::atomic<bool> is_running_;
  std::unique_ptr<std::thread> thread_;
  std::shared_ptr<TcpClient> real_time_tcp_;
  std::shared_ptr<TcpClient> dash_board_tcp_;

public:
  explicit Commander(const std::string & ip)
  : current_joint_{}, tool_vector_{}, real_time_data_{}, is_running_(false)
  {
    this->is_running_ = false;
    this->real_time_tcp_ = std::make_shared<TcpClient>(ip, 30003);
    this->dash_board_tcp_ = std::make_shared<TcpClient>(ip, 29999);
  }

  ~Commander()
  {
    this->is_running_ = false;
    this->thread_->join();
  }

  void getCurrentJointStates(double * joint)
  {
    this->mutex_.lock();
    memcpy(joint, this->current_joint_, sizeof(this->current_joint_));
    mutex_.unlock();
  }

  void getToolVectorActual(double * val)
  {
    this->mutex_.lock();
    memcpy(val, this->tool_vector_, sizeof(tool_vector_));
    this->mutex_.unlock();
  }

  void recvTask() noexcept
  {
    while (this->is_running_) {
      try {
        if (this->dash_board_tcp_->isConnected() && this->real_time_tcp_->isConnected()) {
          if (this->real_time_tcp_->recv(
              &this->real_time_data_, sizeof(this->real_time_data_),
              5000))
          {
            if (this->real_time_data_.len != 1440) {
              continue;
            }

            this->mutex_.lock();
            for (uint64_t i = 0; i < 6; ++i) {
              current_joint_[i] = this->deg2Rad(
                this->real_time_data_.q_actual[i]
              );
            }
            memcpy(this->tool_vector_, this->real_time_data_, sizeof(this->tool_vector_));
            this->mutex_.unlock();
          } else {
            // timeout
            RCLCPP_WARN(
              LOGGER,
              "Tcp recv timeout"
            );
          }
        } else {
          try {
            this->real_time_tcp_->connect();
            this->dash_board_tcp_->connect();
          } catch (const TcpClientException & err) {
            RCLCPP_ERROR(
              LOGGER,
              "Tcp recv error : %s",
              err.what()
            );
            this->sleep(3);
          }
        }
      } catch (const TcpClientException & err) {
        dash_board_tcp_->disConnect();
        RCLCPP_ERROR(
          LOGGER,
          "Tcp recv error : %s", err.what()
        );
      }
    }
  }

  void init() noexcept
  {
    try {
      this->is_running_ = true;
      this->thread_ = std::make_unique<std::thread>(
        &Commander::recvTask, this
      );
    } catch (const TcpClientException & err) {
      RCLCPP_ERROR(
        LOGGER,
        "Commander : %s",
        err.what()
      );
    }
  }

  bool isEnable() const
  {
    return this->real_time_data_.robot_mode == 5;
  }

  bool isConnected() const
  {
    return this->dash_board_tcp_->isConnected() && this->real_time_tcp_->isConnected();
  }

  void enableRobot()
  {
    const char * cmd = "EnableRobot()";
    this->dash_board_tcp_->send(cmd, strlen(cmd));
  }

  void disableRobot()
  {
    const char * cmd = "DisableRobot()";
    this->dash_board_tcp_->send(cmd, strlen(cmd));
  }

  void clearError()
  {
    const char * cmd = "ClearError()";
    this->dash_board_tcp_->send(cmd, strlen(cmd));
  }

  void resetRobot()
  {
    const char * cmd = "ResetRobot()";
    this->dash_board_tcp_->send(cmd, strlen(cmd));
  }

  void speedFactor(int ratio)
  {
    char cmd[100];
    sprintf(cmd, "SpeedFactor(%d)", ratio);
    this->dash_board_tcp_(cmd, strlen(cmd));
  }

  void movJ(double x, double y, double z, double a, double b, double c)
  {
    char cmd[100];
    sprintf(cmd, "MovJ(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)", x, y, z, a, b, c);
    this->real_time_tcp_->tcpSend(cmd, strlen(cmd));
  }

  void movL(double x, double y, double z, double a, double b, double c)
  {
    char cmd[100];
    sprintf(cmd, "MovL(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)", x, y, z, a, b, c);
    this->real_time_tcp_->tcpSend(cmd, strlen(cmd));
  }

  void jointMovJ(double j1, double j2, double j3, double j4, double j5, double j6)
  {
    char cmd[100];
    sprintf(cmd, "JointMovJ(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)", j1, j2, j3, j4, j5, j6);
    this->real_time_tcp_->tcpSend(cmd, strlen(cmd));
  }

  void moveJog(const std::string & axis)
  {
    char cmd[100];
    sprintf(cmd, "MoveJog(%s)", axis.c_str());
    this->real_time_tcp_->tcpSend(cmd, strlen(cmd));
  }

  void relMovJ(
    double offset1, double offset2, double offset3, double offset4, double offset5,
    double offset6)
  {
    char cmd[100];
    sprintf(
      cmd, "RelMovJ(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)", offset1, offset2, offset3, offset4,
      offset5,
      offset6);
    this->real_time_tcp_->tcpSend(cmd, strlen(cmd));
  }

  void relMovL(double x, double y, double z)
  {
    char cmd[100];
    sprintf(cmd, "RelMovL(%0.3f,%0.3f,%0.3f)", x, y, z);
    this->real_time_tcp_->tcpSend(cmd, strlen(cmd));
  }

  void servoJ(double j1, double j2, double j3, double j4, double j5, double j6)
  {
    char cmd[100];
    sprintf(cmd, "ServoJ(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)", j1, j2, j3, j4, j5, j6);
    this->real_time_tcp_->tcpSend(cmd, strlen(cmd));
  }

  void servoP(double x, double y, double z, double a, double b, double c)
  {
    char cmd[100];
    sprintf(cmd, "ServoP(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)", x, y, z, a, b, c);
    this->real_time_tcp_->tcpSend(cmd, strlen(cmd));
  }

  void dashSendCmd(const char * cmd, uint32_t len)
  {
    this->dash_board_tcp_->tcpSend(cmd, strlen(cmd));
  }

  void realSendCmd(const char * cmd, uint32_t len)
  {
    this->real_time_tcp_->tcpSend(cmd, strlen(cmd));
  }

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
} // namespace mg400_interface
