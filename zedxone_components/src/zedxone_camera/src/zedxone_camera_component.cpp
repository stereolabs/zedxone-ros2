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

#include "zedxone_camera_component.hpp"

namespace stereolabs
{

// ----> Global constants
const int QOS_QUEUE_SIZE = 10;
// <---- Global constants

ZedXOneCamera::ZedXOneCamera(const rclcpp::NodeOptions & options)
: Node("zedxone_node", options),
  _qos(QOS_QUEUE_SIZE)
{
  RCLCPP_INFO(get_logger(), "********************************");
  RCLCPP_INFO(get_logger(), "   ZED X One Camera Component ");
  RCLCPP_INFO(get_logger(), "********************************");
  RCLCPP_INFO(get_logger(), " * namespace: %s", get_namespace());
  RCLCPP_INFO(get_logger(), " * node name: %s", get_name());
  RCLCPP_INFO(get_logger(), "********************************");

  // Create the camera object
  _cam = std::make_unique<oc::ArgusV4l2Capture>();

  if (!_cam) {
    RCLCPP_FATAL(
      get_logger(), "Error creating the 'ArgusV4l2Capture'. Aborting ZED X One component.");
    exit(EXIT_FAILURE);
  }

  // ----> Publishers/Subscribers options
#ifndef FOUND_FOXY
  _pubOpt.qos_overriding_options =
    rclcpp::QosOverridingOptions::with_default_policies();
  _subOpt.qos_overriding_options =
    rclcpp::QosOverridingOptions::with_default_policies();
#endif
  // <---- Publishers/Subscribers options
}

ZedXOneCamera::~ZedXOneCamera()
{

}

} // namespace stereolabs

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(stereolabs::ZedXOneCamera)
