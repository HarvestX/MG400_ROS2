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

#ifndef MG400_INTERFACE__EXCEPTION_CONTEXT_HPP_
#define MG400_INTERFACE__EXCEPTION_CONTEXT_HPP_

#include <cerrno>
#include <cstdio>
#include <exception>
#include <functional>
#include <thread>
#include <typeinfo>
#include <utility>

namespace mg400_interface
{

// Record the failing operation's call site, not an inferred throw location.
// Keep the original exception and control flow; stopping belongs to the boundary.
template<typename Operation>
decltype(auto) withExceptionContext(
  const char * operation, const char * file, int line, Operation && invoke)
{
  try {
    return std::forward<Operation>(invoke)();
  } catch (const std::exception & error) {
    const int saved_errno = errno;
    std::fprintf(
      stderr,
      "MG400 exception: thread=%zu operation=%s operation_callsite=%s:%d type=%s what=%s\n",
      std::hash<std::thread::id>{}(std::this_thread::get_id()),
      operation, file, line, typeid(error).name(), error.what());
    // Flush to the configured stderr sink before unwinding further. This is not
    // proof of durable storage; the deployed log collection path must be tested.
    std::fflush(stderr);
    errno = saved_errno;
    throw;
  } catch (...) {
    const int saved_errno = errno;
    std::fprintf(
      stderr,
      "MG400 exception: thread=%zu operation=%s operation_callsite=%s:%d "
      "type=unknown what=non-standard-exception\n",
      std::hash<std::thread::id>{}(std::this_thread::get_id()), operation, file, line);
    std::fflush(stderr);
    errno = saved_errno;
    throw;
  }
}

}  // namespace mg400_interface
#endif  // MG400_INTERFACE__EXCEPTION_CONTEXT_HPP_
