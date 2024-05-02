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
#include "sl_tools.hpp"

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace std::chrono_literals;
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

  // ----> Publishers/Subscribers options
#ifndef FOUND_FOXY
  _pubOpt.qos_overriding_options =
    rclcpp::QosOverridingOptions::with_default_policies();
  _subOpt.qos_overriding_options =
    rclcpp::QosOverridingOptions::with_default_policies();
#endif
  // <---- Publishers/Subscribers options

  // Parameters initialization
  initParameters();

  // Open Camera
  if (!openCamera()) {
    exit(EXIT_FAILURE);
  }

  // ----> Create publishers
  _pubImg = image_transport::create_publisher(this, "image", _qos.get_rmw_qos_profile());
  RCLCPP_INFO_STREAM(get_logger(), "Advertised on topic: " << _pubImg.getTopic());

  // // ----> Start grab timer
  // int msec = static_cast<int>(1000. / (_fps));
  // _frameGrabTimer =
  //   create_wall_timer(
  //   std::chrono::duration_cast<std::chrono::milliseconds>(
  //     std::chrono::milliseconds(msec)),
  //   std::bind(&ZedXOneCamera::callback_frameGrab, this));
  // // <---- Start grab timer

  // Start Grab thread
  _grabThread = std::thread(&ZedXOneCamera::callback_frameGrab, this);
}

ZedXOneCamera::~ZedXOneCamera()
{

}

template<typename T>
void ZedXOneCamera::getParam(
  std::string paramName, T defValue, T & outVal,
  std::string log_info, bool dynamic)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = !dynamic;

  declare_parameter(paramName, rclcpp::ParameterValue(defValue), descriptor);

  if (!get_parameter(paramName, outVal)) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "The parameter '" <<
        paramName <<
        "' is not available or is not valid, using the default value: " <<
        defValue);
  }

  if (!log_info.empty()) {
    RCLCPP_INFO_STREAM(get_logger(), log_info << outVal);
  }
}

void ZedXOneCamera::initParameters()
{
  // Debug
  initDebugParams();

  // Camera
  initCamParams();
}

void ZedXOneCamera::initDebugParams()
{
  rclcpp::Parameter paramVal;

  RCLCPP_INFO(get_logger(), "*** DEBUG parameters ***");

  getParam("debug.grab_verbose_level", _argusVerbose, _argusVerbose, " * Grabber Verbose: ");

  getParam("debug.general", _debugGeneral, _debugGeneral);
  RCLCPP_INFO(
    get_logger(), " * Debug General: %s",
    _debugGeneral ? "TRUE" : "FALSE");

  // ************************************************** //

  _debugMode = _debugGeneral;

  if (_debugMode) {
    rcutils_ret_t res = rcutils_logging_set_logger_level(
      get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

    if (res != RCUTILS_RET_OK) {
      RCLCPP_INFO(get_logger(), "Error setting DEBUG level for logger");
    } else {
      RCLCPP_INFO(get_logger(), " + Debug Mode enabled +");
    }
  } else {
    rcutils_ret_t res = rcutils_logging_set_logger_level(
      get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);

    if (res != RCUTILS_RET_OK) {
      RCLCPP_INFO(get_logger(), "Error setting INFO level for logger");
    }
  }

  DEBUG_STREAM_GEN("[ROS 2] Using RMW_IMPLEMENTATION " << rmw_get_implementation_identifier());
}

void ZedXOneCamera::initCamParams()
{
  rclcpp::Parameter paramVal;

  RCLCPP_INFO(get_logger(), "*** CAMERA parameters ***");

  getParam("camera.camera_model", _model, _model);
  if (_model != "GS" && _model != "4K") {
    RCLCPP_FATAL_STREAM(
      get_logger(),
      "Wrong camera model parameter. Expected 'GS' or '4K', retrieved '" << _model << "'");
    RCLCPP_FATAL(get_logger(), "Please check the value of the parameter 'camera.model'");
    exit(EXIT_FAILURE);
  }
  RCLCPP_INFO_STREAM(get_logger(), " * Model: " << _model);

  getParam("camera.idx", _deviceIdx, _deviceIdx, " * Device IDX: ");

  getParam("camera.resolution", _resolution, _resolution);
  if (_resolution == "SVGA") {
    _width = SVGA_W;
    _height = SVGA_H;
  } else if (_resolution == "HD1080") {
    _width = HD1080_W;
    _height = HD1080_H;
  } else if (_resolution == "HD1200") {
    _width = HD1200_W;
    _height = HD1200_H;
  } else if (_resolution == "4K") {
    if (_model != "4K") {
      RCLCPP_FATAL(get_logger(), "'4K' resolution is only available with '4k' camera model.");
      RCLCPP_FATAL(get_logger(), "Please check the value of the parameter 'camera.resolution'");
      exit(EXIT_FAILURE);
    }
    _width = HD4K_W;
    _height = HD4K_H;
  } else {
    RCLCPP_FATAL_STREAM(get_logger(), "Invalid resolution parameter: '" << _resolution << "'");
    RCLCPP_FATAL(get_logger(), "Please check the value of the parameter 'camera.resolution'");
    exit(EXIT_FAILURE);
  }
  RCLCPP_INFO_STREAM(
    get_logger(), " * Resolution: " << _resolution << " [" << _width << "x" << _height << "]");

  getParam("camera.framerate", _fps, _fps, " * Framerate: ");
  getParam("camera.timeout_msec", _cam_timeout_msec, _cam_timeout_msec, " * Timeous [msec]: ");

  getParam("camera.swap_rb", _swapRB, _swapRB);
  RCLCPP_INFO_STREAM(
    get_logger(), " * Swap RB: " << (_swapRB ? "TRUE" : "FALSE"));

  getParam("camera.pixel_format", _pxFormat, _pxFormat);
  if (_pxFormat == "COLOR_RGB") {
    _pxMode = oc::PixelMode::COLOR_RGB;
  } else if (_pxFormat == "COLOR_RGBA") {
    _pxMode = oc::PixelMode::COLOR_RGBA;
  } else if (_pxFormat == "RAW_BAYER") {
    if (_model == "4K") {
      _pxMode = oc::PixelMode::RAW12;
    } else {
      _pxMode = oc::PixelMode::RAW10;
    }
  } else {
    RCLCPP_FATAL_STREAM(get_logger(), "Invalid pixel format parameter: '" << _pxFormat << "'");
    RCLCPP_FATAL(get_logger(), "Please check the value of the parameter 'camera.pixel_format'");
    exit(EXIT_FAILURE);
  }
  RCLCPP_INFO_STREAM(
    get_logger(), " * Pixel Format: " << _pxFormat);
}

bool ZedXOneCamera::openCamera()
{
  // Create the camera object
  if (_pxFormat == "RAW_BAYER") {
    _cam = std::make_unique<oc::ArgusV4l2Capture>();
  } else {
    _cam = std::make_unique<oc::ArgusBayerCapture>();
  }

  if (!_cam) {
    RCLCPP_FATAL(
      get_logger(), "Error creating the 'ArgusV4l2Capture'. Aborting ZED X One component.");
    return false;
  }

  // ----> Camera configuration
  oc::ArgusCameraConfig config;
  config.mDeviceId = _deviceIdx;
  config.mFPS = _fps;
  config.mHeight = _height;
  config.mSwapRB = _swapRB;
  config.mWidth = _width;
  config.verbose_level = _argusVerbose;
  config.mode = _pxMode;
  // <---- Camera configuration

  oc::ARGUS_STATE res;

  res = _cam->openCamera(config);
  if (res != oc::ARGUS_STATE::OK) {
    RCLCPP_FATAL_STREAM(get_logger(), "Failed to open the camera: " << oc::ARGUS_STATE2str(res));
    return false;
  }

  return true;
}

void ZedXOneCamera::callback_frameGrab()
{
  while (1) {
    // ----> Check if a new frame is available
    auto start = std::chrono::system_clock::now();
    while (!_cam->isNewFrame()) {
      auto end = std::chrono::system_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
      if (elapsed.count() > _cam_timeout_msec) {
        RCLCPP_FATAL(get_logger(), "Camera timeout. Disconnected?");
        exit(EXIT_FAILURE);     // TODO Recover camera?
      }
      rclcpp::sleep_for(1ms);
    }
    // <---- Check if a new frame is available

    size_t data_size = _cam->getWidth() * _cam->getHeight() * _cam->getNumberOfChannels();

    uint64_t ts_nsec = _cam->getImageTimestampinUs() * 1000;

    std::shared_ptr<sensor_msgs::msg::Image> msg = std::make_shared<sensor_msgs::msg::Image>();
    msg->header.frame_id = _model;
    msg->header.stamp = sl_tools::slTime2Ros(ts_nsec);
    msg->encoding = sensor_msgs::image_encodings::BGRA8; // TODO Switch on different encodings
    msg->width = _width;
    msg->height = _height;
    msg->step = _width * _cam->getNumberOfChannels();
    msg->data = std::vector<uint8_t>(_cam->getPixels(), _cam->getPixels() + data_size);

    int num = 1;  // for endianness detection
    msg->is_bigendian = !(*reinterpret_cast<char *>(&num) == 1);

    _pubImg.publish(msg);
  }
}
} // namespace stereolabs

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(stereolabs::ZedXOneCamera)
