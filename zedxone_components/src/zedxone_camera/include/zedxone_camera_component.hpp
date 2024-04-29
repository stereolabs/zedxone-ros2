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

#ifndef ZEDXONE_CAMERA_COMPONENT_HPP
#define ZEDXONE_CAMERA_COMPONENT_HPP

#include "visibility_control.hpp"

#include "sl_types.hpp"
#include "sl_logging.hpp"

#include "ArgusCapture.hpp"

namespace stereolabs
{

class ZedXOneCamera : public rclcpp::Node
{
public:
  ZEDXONE_COMPONENTS_PUBLIC
  explicit ZedXOneCamera(const rclcpp::NodeOptions & options);

  virtual ~ZedXOneCamera();

protected:
private:
  // ZED X One camera object
  std::unique_ptr<oc::ArgusV4l2Capture> _cam;

  // ----> Parameters
  // Debug
  bool _debugGeneral = false;
  // <---- Parameters

  // ----> QoS
  // https://github.com/ros2/ros2/wiki/About-Quality-of-Service-Settings
  rclcpp::QoS _qos;
  rclcpp::PublisherOptions _pubOpt;
  rclcpp::SubscriptionOptions _subOpt;
  // <---- QoS
};

} // namespace stereolabs

#endif