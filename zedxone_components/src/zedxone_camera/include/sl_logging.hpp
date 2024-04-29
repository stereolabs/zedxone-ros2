// Copyright 2024 Stereolabs
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SL_LOGGING_HPP
#define SL_LOGGING_HPP

#include <rcutils/logging_macros.h>

// General
#define DEBUG_GEN(...) \
  if (_debugGeneral) RCLCPP_DEBUG (get_logger(), __VA_ARGS__)
#define DEBUG_ONCE_GEN(...) \
  if (_debugGeneral) RCLCPP_DEBUG_ONCE (get_logger(), __VA_ARGS__)
#define DEBUG_STREAM_GEN(stream_arg) \
  if (_debugGeneral) RCLCPP_DEBUG_STREAM (get_logger(), stream_arg)
#define DEBUG_STREAM_THROTTLE_GEN(duration, stream_arg) \
  if (_debugGeneral) { \
    rclcpp::Clock steady_clock(RCL_STEADY_TIME); \
    RCLCPP_DEBUG_STREAM_THROTTLE( \
      get_logger(), steady_clock, duration, \
      stream_arg); \
  }

#endif // SL_LOGGING_HPP
