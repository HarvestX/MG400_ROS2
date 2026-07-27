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

#include "mg400_interface/tcp_interface/motion_tcp_interface.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <exception>
#include <utility>
#include <vector>

#include "mg400_interface/commander/response_parser.hpp"

namespace mg400_interface
{

MotionTcpInterface::MotionTcpInterface(const std::string & ip)
: MotionTcpInterface(ip, DEFAULT_PORT)
{}

MotionTcpInterface::MotionTcpInterface(
  const std::string & ip, uint16_t port, const MotionTcpInterfaceOptions & options)
: options_(options),
  is_running_(false),
  tcp_socket_(std::make_shared<TcpSocketHandler>(ip, port)),
  next_sequence_id_(1),
  connection_generation_(0),
  has_latest_response_(false),
  has_latest_error_(false),
  dropped_completed_response_count_(0)
{
  if (this->options_.pending_capacity == 0 || this->options_.completed_capacity == 0) {
    throw std::invalid_argument("Motion response queue capacities must be greater than zero");
  }
  if (this->options_.maximum_response_length == 0) {
    throw std::invalid_argument("maximum_response_length must be greater than zero");
  }
  if (this->options_.command_timeout <= std::chrono::nanoseconds::zero() ||
    this->options_.receive_poll_interval <= std::chrono::nanoseconds::zero() ||
    this->options_.reconnect_interval <= std::chrono::nanoseconds::zero() ||
    this->options_.connect_timeout <= std::chrono::nanoseconds::zero())
  {
    throw std::invalid_argument("Motion TCP timeouts and intervals must be greater than zero");
  }
}

MotionTcpInterface::~MotionTcpInterface()
{
  this->disConnect();
}

rclcpp::Logger MotionTcpInterface::getLogger()
{
  return rclcpp::get_logger("Motion Tcp Interface");
}

void MotionTcpInterface::init() noexcept
{
  std::lock_guard<std::mutex> lock(this->mutex_lifecycle_);
  if (this->is_running_.exchange(true)) {
    return;
  }

  try {
    this->thread_ = std::make_unique<std::thread>(&MotionTcpInterface::receiveLoop, this);
  } catch (const std::exception & err) {
    this->is_running_.store(false);
    RCLCPP_ERROR(this->getLogger(), "Failed to start motion receive thread: %s", err.what());
  }
}

bool MotionTcpInterface::isConnected() const
{
  return this->tcp_socket_->isConnected();
}

bool MotionTcpInterface::waitUntilConnected(const std::chrono::nanoseconds & timeout)
{
  std::unique_lock<std::mutex> lock(this->mutex_state_);
  return this->cv_connection_.wait_for(
    lock, timeout, [this]() {
      return this->tcp_socket_->isConnected() || !this->is_running_.load();
    }) && this->tcp_socket_->isConnected();
}

void MotionTcpInterface::disConnect()
{
  std::lock_guard<std::mutex> lifecycle_lock(this->mutex_lifecycle_);
  this->is_running_.store(false);
  this->cv_stop_.notify_all();

  // Serialize against registration/send, then wake recvSome().
  {
    std::lock_guard<std::mutex> send_lock(this->mutex_send_);
    this->tcp_socket_->disConnect();
  }
  this->cv_connection_.notify_all();
  this->cv_response_.notify_all();

  if (this->thread_ && this->thread_->joinable()) {
    this->thread_->join();
  }
  this->thread_.reset();
  this->failAllPending(MotionResponseResult::DISCONNECTED, std::chrono::steady_clock::now());
}

void MotionTcpInterface::sendCommand(const std::string & command)
{
  std::lock_guard<std::mutex> send_lock(this->mutex_send_);
  if (!this->is_running_.load() || !this->tcp_socket_->isConnected()) {
    throw TcpSocketException("motion tcp is disconnected");
  }

  const auto now = std::chrono::steady_clock::now();
  uint64_t sequence_id = 0;
  {
    std::lock_guard<std::mutex> state_lock(this->mutex_state_);
    if (this->pending_commands_.size() >= this->options_.pending_capacity) {
      throw MotionCommandQueueFullException("motion pending command queue is full");
    }

    sequence_id = this->next_sequence_id_++;
    this->pending_commands_.push_back(
      PendingCommand{
        sequence_id,
        this->connection_generation_,
        command,
        takeCommandName(command),
        now,
        now + this->options_.command_timeout});
  }

  // Pending registration intentionally precedes send(). A response can be
  // available as soon as the kernel accepts the bytes, so registering after
  // send would race the sole receiver thread.
  try {
    this->tcp_socket_->send(command.data(), static_cast<uint32_t>(command.size()));
  } catch (const TcpSocketException & err) {
    this->handleConnectionLoss(err.what());
    throw;
  }
}

bool MotionTcpInterface::tryTakeResponse(MotionResponse & response)
{
  std::lock_guard<std::mutex> lock(this->mutex_state_);
  if (this->completed_responses_.empty()) {
    return false;
  }
  response = std::move(this->completed_responses_.front());
  this->completed_responses_.pop_front();
  return true;
}

bool MotionTcpInterface::waitForResponse(
  MotionResponse & response, const std::chrono::nanoseconds & timeout)
{
  std::unique_lock<std::mutex> lock(this->mutex_state_);
  if (!this->cv_response_.wait_for(
      lock, timeout,
      [this]() {
        return !this->completed_responses_.empty() || !this->is_running_.load();
      }) || this->completed_responses_.empty())
  {
    return false;
  }
  response = std::move(this->completed_responses_.front());
  this->completed_responses_.pop_front();
  return true;
}

bool MotionTcpInterface::getLatestResponse(MotionResponse & response) const
{
  std::lock_guard<std::mutex> lock(this->mutex_state_);
  if (!this->has_latest_response_) {
    return false;
  }
  response = this->latest_response_;
  return true;
}

bool MotionTcpInterface::getLatestError(MotionResponse & response) const
{
  std::lock_guard<std::mutex> lock(this->mutex_state_);
  if (!this->has_latest_error_) {
    return false;
  }
  response = this->latest_error_;
  return true;
}

size_t MotionTcpInterface::getPendingCommandCount() const
{
  std::lock_guard<std::mutex> lock(this->mutex_state_);
  return this->pending_commands_.size();
}

uint64_t MotionTcpInterface::getDroppedCompletedResponseCount() const
{
  std::lock_guard<std::mutex> lock(this->mutex_state_);
  return this->dropped_completed_response_count_;
}

size_t MotionTcpInterface::getCompletedResponseCount() const
{
  std::lock_guard<std::mutex> lock(this->mutex_state_);
  return this->completed_responses_.size();
}

void MotionTcpInterface::receiveLoop()
{
  std::string receive_buffer;
  receive_buffer.reserve(std::min(this->options_.maximum_response_length, size_t{1024}));
  std::array<char, RECEIVE_CHUNK_SIZE> chunk{};

  while (this->is_running_.load()) {
    if (!this->tcp_socket_->isConnected()) {
      receive_buffer.clear();
      if (!this->connectSocket()) {
        this->waitBeforeReconnect();
      }
      continue;
    }

    try {
      const uint32_t received = this->tcp_socket_->recvSome(
        chunk.data(), static_cast<uint32_t>(chunk.size()), this->options_.receive_poll_interval);
      if (received > 0 &&
        !this->processReceivedBytes(receive_buffer, chunk.data(), received))
      {
        receive_buffer.clear();
        continue;
      }
      if (this->expireTimedOutCommands()) {
        receive_buffer.clear();
      }
    } catch (const TcpSocketException & err) {
      receive_buffer.clear();
      this->handleConnectionLoss(err.what());
    }
  }
}

bool MotionTcpInterface::connectSocket()
{
  try {
    std::lock_guard<std::mutex> send_lock(this->mutex_send_);
    if (!this->is_running_.load()) {
      return false;
    }
    this->tcp_socket_->connect(this->options_.connect_timeout);
    {
      std::lock_guard<std::mutex> state_lock(this->mutex_state_);
      ++this->connection_generation_;
    }
    this->cv_connection_.notify_all();
    return true;
  } catch (const TcpSocketException & err) {
    if (this->is_running_.load()) {
      RCLCPP_DEBUG(this->getLogger(), "Motion TCP connect failed: %s", err.what());
    }
    return false;
  }
}

void MotionTcpInterface::waitBeforeReconnect()
{
  std::unique_lock<std::mutex> lock(this->mutex_stop_);
  this->cv_stop_.wait_for(
    lock, this->options_.reconnect_interval,
    [this]() {return !this->is_running_.load();});
}

bool MotionTcpInterface::processReceivedBytes(
  std::string & receive_buffer, const char * data, size_t size)
{
  receive_buffer.append(data, size);
  while (true) {
    const auto terminator = receive_buffer.find(';');
    if (terminator == std::string::npos) {
      if (receive_buffer.size() > this->options_.maximum_response_length) {
        this->completeFrontAsProtocolError(receive_buffer);
        this->handleProtocolFailure("motion response exceeded maximum length before terminator");
        return false;
      }
      return true;
    }

    if (terminator + 1 > this->options_.maximum_response_length) {
      this->completeFrontAsProtocolError(receive_buffer.substr(0, terminator + 1));
      this->handleProtocolFailure("motion response exceeded maximum length");
      return false;
    }

    const std::string frame = receive_buffer.substr(0, terminator + 1);
    receive_buffer.erase(0, terminator + 1);
    if (!this->processResponseFrame(frame)) {
      return false;
    }
  }
}

bool MotionTcpInterface::processResponseFrame(const std::string & raw_frame)
{
  const auto first = raw_frame.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return true;
  }
  const std::string frame = raw_frame.substr(first);
  const auto received_at = std::chrono::steady_clock::now();

  DashboardResponse parsed{};
  bool parsed_ok = false;
  try {
    parsed_ok = ResponseParser::parseResponse(frame, parsed);
  } catch (const std::exception &) {
    parsed_ok = false;
  }

  MotionResponse response;
  bool protocol_failure = false;
  {
    std::lock_guard<std::mutex> lock(this->mutex_state_);
    if (this->pending_commands_.empty()) {
      response.raw_response = frame;
      response.received_at = received_at;
      response.error_code = parsed_ok ? parsed.error_id : MotionResponse::ERROR_CODE_UNAVAILABLE;
      response.result = MotionResponseResult::PARSE_ERROR;
      this->latest_response_ = response;
      this->latest_error_ = response;
      this->has_latest_response_ = true;
      this->has_latest_error_ = true;
      protocol_failure = true;
    } else {
      const PendingCommand pending = std::move(this->pending_commands_.front());
      this->pending_commands_.pop_front();
      response.sequence_id = pending.sequence_id;
      response.command = pending.command;
      response.raw_response = frame;
      response.sent_at = pending.sent_at;
      response.received_at = received_at;
      response.error_code = parsed_ok ? parsed.error_id : MotionResponse::ERROR_CODE_UNAVAILABLE;

      // The protocol has no request ID. The documented response echoes the
      // submitted command, but the guide does not explicitly guarantee reply
      // ordering. FIFO correlation is isolated here and validated by command
      // name; a mismatch resets the connection instead of guessing.
      if (pending.connection_generation != this->connection_generation_ || !parsed_ok ||
        !commandNamesEqual(pending.command_name, takeCommandName(parsed.func_name)))
      {
        response.result = MotionResponseResult::PARSE_ERROR;
        protocol_failure = true;
      } else if (parsed.error_id == 0) {
        response.result = MotionResponseResult::SUCCESS;
      } else {
        response.result = MotionResponseResult::CONTROLLER_ERROR;
      }
      this->addCompletedResponse(response);
    }
  }
  this->cv_response_.notify_all();

  if (protocol_failure) {
    this->handleProtocolFailure("invalid or out-of-order motion response");
    return false;
  }
  return true;
}

bool MotionTcpInterface::expireTimedOutCommands()
{
  const auto now = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(this->mutex_state_);
    const auto expired = std::find_if(
      this->pending_commands_.begin(), this->pending_commands_.end(),
      [now](const PendingCommand & pending) {return pending.timeout_at <= now;});
    if (expired == this->pending_commands_.end()) {
      return false;
    }
  }

  // Without request IDs, retaining the connection after removing a timed-out
  // FIFO entry could attach its late response to the next command. Reset the
  // connection generation; expired entries become TIMEOUT and the rest become
  // DISCONNECTED.
  this->tcp_socket_->disConnect();
  std::vector<MotionResponse> responses;
  {
    std::lock_guard<std::mutex> lock(this->mutex_state_);
    while (!this->pending_commands_.empty()) {
      const PendingCommand pending = std::move(this->pending_commands_.front());
      this->pending_commands_.pop_front();
      MotionResponse response;
      response.sequence_id = pending.sequence_id;
      response.command = pending.command;
      response.sent_at = pending.sent_at;
      response.received_at = now;
      response.result = pending.timeout_at <= now ?
        MotionResponseResult::TIMEOUT : MotionResponseResult::DISCONNECTED;
      responses.push_back(std::move(response));
    }
    for (const auto & response : responses) {
      this->addCompletedResponse(response);
    }
  }
  this->cv_response_.notify_all();
  this->cv_connection_.notify_all();
  return true;
}

void MotionTcpInterface::handleConnectionLoss(const std::string & message)
{
  this->tcp_socket_->disConnect();
  this->failAllPending(MotionResponseResult::DISCONNECTED, std::chrono::steady_clock::now());
  this->cv_connection_.notify_all();
  if (this->is_running_.load()) {
    RCLCPP_WARN(this->getLogger(), "Motion TCP connection lost: %s", message.c_str());
  }
}

void MotionTcpInterface::handleProtocolFailure(const std::string & message)
{
  this->tcp_socket_->disConnect();
  this->failAllPending(MotionResponseResult::DISCONNECTED, std::chrono::steady_clock::now());
  this->cv_connection_.notify_all();
  RCLCPP_ERROR(this->getLogger(), "Motion TCP protocol failure: %s", message.c_str());
}

void MotionTcpInterface::completeFrontAsProtocolError(const std::string & raw_response)
{
  const auto completed_at = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(this->mutex_state_);
    MotionResponse response;
    response.raw_response = raw_response;
    response.received_at = completed_at;
    response.result = MotionResponseResult::PARSE_ERROR;
    if (!this->pending_commands_.empty()) {
      const PendingCommand pending = std::move(this->pending_commands_.front());
      this->pending_commands_.pop_front();
      response.sequence_id = pending.sequence_id;
      response.command = pending.command;
      response.sent_at = pending.sent_at;
      this->addCompletedResponse(response);
    } else {
      this->latest_response_ = response;
      this->latest_error_ = response;
      this->has_latest_response_ = true;
      this->has_latest_error_ = true;
    }
  }
  this->cv_response_.notify_all();
}

void MotionTcpInterface::failAllPending(
  MotionResponseResult result, const std::chrono::steady_clock::time_point & completed_at)
{
  bool added = false;
  {
    std::lock_guard<std::mutex> lock(this->mutex_state_);
    while (!this->pending_commands_.empty()) {
      const PendingCommand pending = std::move(this->pending_commands_.front());
      this->pending_commands_.pop_front();
      MotionResponse response;
      response.sequence_id = pending.sequence_id;
      response.command = pending.command;
      response.sent_at = pending.sent_at;
      response.received_at = completed_at;
      response.result = result;
      this->addCompletedResponse(response);
      added = true;
    }
  }
  if (added) {
    this->cv_response_.notify_all();
  }
}

void MotionTcpInterface::addCompletedResponse(const MotionResponse & response)
{
  // mutex_state_ must be held. Existing fire-and-forget users do not drain the
  // completion queue. Keep receiving indefinitely by replacing the oldest
  // completion at the explicit bound, while making every loss observable via
  // a monotonic diagnostic counter (and periodic warnings).
  if (this->completed_responses_.size() >= this->options_.completed_capacity) {
    this->completed_responses_.pop_front();
    ++this->dropped_completed_response_count_;
    if (this->dropped_completed_response_count_ == 1 ||
      this->dropped_completed_response_count_ % 100 == 0)
    {
      RCLCPP_WARN(
        this->getLogger(),
        "Motion completed response queue overflow; dropped oldest response (total=%lu)",
        static_cast<unsigned long>(this->dropped_completed_response_count_));
    }
  }
  this->completed_responses_.push_back(response);
  this->latest_response_ = response;
  this->has_latest_response_ = true;
  if (response.result != MotionResponseResult::SUCCESS) {
    this->latest_error_ = response;
    this->has_latest_error_ = true;
  }
}

std::string MotionTcpInterface::takeCommandName(const std::string & command)
{
  const auto first = command.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return std::string();
  }
  const auto bracket = command.find('(', first);
  if (bracket == std::string::npos) {
    return std::string();
  }
  const auto last = command.find_last_not_of(" \t\r\n", bracket - 1);
  if (last == std::string::npos || last < first) {
    return std::string();
  }
  return command.substr(first, last - first + 1);
}

bool MotionTcpInterface::commandNamesEqual(const std::string & left, const std::string & right)
{
  if (left.size() != right.size() || left.empty()) {
    return false;
  }
  return std::equal(
    left.begin(), left.end(), right.begin(),
    [](char lhs, char rhs) {
      return std::tolower(static_cast<unsigned char>(lhs)) ==
      std::tolower(static_cast<unsigned char>(rhs));
    });
}

}  // namespace mg400_interface
