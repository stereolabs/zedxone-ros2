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

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

using namespace std::chrono_literals;
namespace stereolabs
{

// ----> Global constants
const int QOS_QUEUE_SIZE = 10;
// <---- Global constants

ZedXOneCamera::ZedXOneCamera(const rclcpp::NodeOptions & options)
: Node("zedxone_node", options),
  _stopNode(false),
  _grabFreqStopWatch(get_clock()),
  _diagUpdater(this),
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

  // ----> Diagnostic variables
  _grabPeriodMean_sec = std::make_unique<sl_tools::WinAvg>(static_cast<size_t>(_fps));

  _diagUpdater.add(
    "ZED X One Diagnostic", this,
    &ZedXOneCamera::callback_updateDiagnostic);
  std::string hw_id = std::string("Stereolabs camera: ") + _model;
  _diagUpdater.setHardwareID(hw_id);
  // <---- Statistic variables

  // ----> Create messages
  int num = 1;    // for endianness detection

  _imgTrMsg = std::make_unique<sensor_msgs::msg::Image>();
  _imgTrMsg->header.frame_id = _model;
  _imgTrMsg->encoding = sensor_msgs::image_encodings::BGRA8;   // TODO Switch on different encodings
  _imgTrMsg->width = _width;
  _imgTrMsg->height = _height;
  _imgTrMsg->step = _width * _cam->getNumberOfChannels();
  _imgTrMsg->is_bigendian = !(*reinterpret_cast<char *>(&num) == 1);
  // <---- Create messages

  // ----> Create publishers
  _pubImgTransp = image_transport::create_publisher(this, "image", _qos.get_rmw_qos_profile());
  RCLCPP_INFO_STREAM(get_logger(), "Advertised on topic: " << _pubImgTransp.getTopic());
  // <---- Create publishers

#ifdef USE_THREAD
  // Start Grab thread
  _grabThread = std::thread(&ZedXOneCamera::callback_frameGrab, this);
#else
  // ----> Start grab timer
  int msec = static_cast<int>(1000. / (_fps));
  _frameGrabTimer =
    create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::milliseconds(msec)),
    std::bind(&ZedXOneCamera::callback_frameGrab, this));
  // <---- Start grab timer
#endif


}

ZedXOneCamera::~ZedXOneCamera()
{
  _stopNode = true;

#ifdef USE_THREAD
  if (_grabThread.joinable()) {
    _grabThread.join();
  }
#else
  _frameGrabTimer.reset();
#endif

  _cam.reset();
  _imgTrMsg.reset();
  _diagUpdater.force_update();
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

  getParam("debug.diagnostic", _debugDiagnostic, _debugDiagnostic);
  RCLCPP_INFO(
    get_logger(), " * Debug Diagnostic: %s",
    _debugDiagnostic ? "TRUE" : "FALSE");
  // ************************************************** //

  _debugMode = _debugGeneral | _debugDiagnostic;

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

#ifdef USE_THREAD
  DEBUG_GEN("Grab thread started");
  while (!_stopNode)
#endif
  {

#ifdef USE_THREAD
    if (!_cam->isNewFrame()) {
      rclcpp::sleep_for(5ms);
      continue;
    }
#else
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
#endif

    // Data retrieve
    auto px_data = _cam->getPixels(); // This is required to not block the grabber
    uint64_t ts_nsec = _cam->getImageTimestampinUs() * 1000;
    size_t data_size = _cam->getWidth() * _cam->getHeight() * _cam->getNumberOfChannels();

    // ----> Image message
    _imgTranspSubs = _pubImgTransp.getNumSubscribers();
    if (_imgTranspSubs > 0) {
      _imgTrMsg->header.stamp = sl_tools::slTime2Ros(ts_nsec);
      _imgTrMsg->data = std::vector<uint8_t>(px_data, px_data + data_size);

      _pubImgTransp.publish(*_imgTrMsg);
    }
    // <---- Image message

    // ----> Grab freq calculation
    double elapsed_sec = _grabFreqStopWatch.toc();
    _grabPeriodMean_sec->addValue(elapsed_sec);
    _grabFreqStopWatch.tic();

    DEBUG_STREAM_DIAG(
      "Grab period: " << _grabPeriodMean_sec->getAvg() << " sec - Freq: " << 1.0 /
        _grabPeriodMean_sec->getAvg());
    // <---- Grab freq calculation
  }

#ifdef USE_THREAD
  _diagUpdater.force_update();
  DEBUG_GEN("Grab thread finished");
#endif
}

void ZedXOneCamera::callback_updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  DEBUG_DIAG("*** Update Diagnostic ***");

  if (_stopNode) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::STALE,
      "Node stopped");
  }

  if (_cam && _cam->isOpened()) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::OK,
      "Camera grabbing");
  } else {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::STALE,
      "Camera stopped");
  }

  double freq = 1. / _grabPeriodMean_sec->getAvg();
  double freq_perc = 100. * freq / _fps;
  stat.addf("Capture", "Mean Frequency: %.1f Hz (%.1f%%)", freq, freq_perc);

  stat.addf("Image Transp. Pub.", "Subscribers: %d", _imgTranspSubs);
}

} // namespace stereolabs

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(stereolabs::ZedXOneCamera)
