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

#include <image_transport/camera_publisher.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/image_transport.hpp>

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
  bool openCamera();
  void callback_frameGrab();

  void initParameters();
  void initDebugParams();
  void initCamParams();

  template<typename T>
  void getParam(
    std::string paramName, T defValue, T & outVal,
    std::string log_info = std::string(), bool dynamic = false);

private:
  // ZED X One camera object
  std::unique_ptr<oc::ArgusVirtualCapture> _cam;

  // ----> Parameters
  // Debug
  int _argusVerbose = 0;
  bool _debugGeneral = false;

  // Camera
  std::string _model;
  int _deviceIdx = 0;
  int _fps = 30;
  std::string _resolution = "HD1080";
  bool _swapRB = false;
  std::string _pxFormat;
  int _cam_timeout_msec = 2000;
  // <---- Parameters

  // ----> Running parameters
  bool _debugMode = false;
  int _width;
  int _height;
  oc::PixelMode _pxMode;
  // <---- Running parameters

  // ----> Running variables
  //rclcpp::TimerBase::SharedPtr _frameGrabTimer;  // Timer to grab camera frames
  std::thread _grabThread;
  image_transport::Publisher _pubImg;    // Publisher for camera stream without camera information
  // <---- Running variables

  // ----> QoS
  // https://github.com/ros2/ros2/wiki/About-Quality-of-Service-Settings
  rclcpp::QoS _qos;
  rclcpp::PublisherOptions _pubOpt;
  rclcpp::SubscriptionOptions _subOpt;
  // <---- QoS
};

} // namespace stereolabs

#endif
