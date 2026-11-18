// Copyright 2026 HarvestX Inc.
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

#ifndef MG400_INTERFACE__SOFTWARE_FAULT_HPP_
#define MG400_INTERFACE__SOFTWARE_FAULT_HPP_

#include <atomic>
#include <cstdio>
#include <exception>
#include <typeinfo>

namespace mg400_interface
{

// Shared by the receiver, callbacks and connection policy. No automatic reset:
// reconstruction starts a new instance. This latch does not prohibit process
// restart or replace the caller's responsibility to notify the supervising node.
class SoftwareFault
{
public:
  bool failed() const noexcept {return failed_.load();}

  template<typename Operation>
  bool run(const char * boundary, Operation && operation) noexcept
  {
    if (failed()) {
      return false;
    }
    try {
      operation();
      return !failed();
    } catch (const std::exception & error) {
      record(boundary, typeid(error).name(), error.what());
    } catch (...) {
      record(boundary, "unknown", "non-standard exception");
    }
    return false;
  }

private:
  std::atomic<bool> failed_{false};

  void record(const char * boundary, const char * type, const char * message) noexcept
  {
    // Latch before attempting diagnostics. Avoid allocating strings or calling
    // ROS publishers from a failure handler (including allocation failures).
    failed_.store(true);
    std::fprintf(
      stderr, "MG400 software exception: boundary=%s type=%s what=%s; "
      "software fault latched in this interface instance\n",
      boundary, type, message);
    std::fflush(stderr);
  }
};

}  // namespace mg400_interface

#endif  // MG400_INTERFACE__SOFTWARE_FAULT_HPP_
