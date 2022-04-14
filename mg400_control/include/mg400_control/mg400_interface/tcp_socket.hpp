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

#include <unistd.h>
#include <string>
#include <rclcpp/rclcpp.hpp>

namespace mg400_interface
{

class TcpClientException : public std::logic_error
{
public:
  TcpClientException(
    const std::string & what
  )
  : std::logic_error(what)
  {}
};

class TcpClient
{
private:
  int fd_;
  uint16_t port_;
  std::string ip_;
  std::atomic<bool> is_connected_;

public:
  TcpClient(
    std::string,
    uint16_t);

  ~TcpClient();

  void close();
  void connect();
  void disConnect();
  bool isConnected() const;
  void send(const void *, uint32_t);
  bool recv(void *, uint32_t, uint32_t);
  std::string toString();
};
}  // namespace mg400_interface
