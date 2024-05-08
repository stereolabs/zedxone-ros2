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
using namespace std::placeholders;
namespace stereolabs
{

// ----> Global constants
const int QOS_QUEUE_SIZE = 10;
// <---- Global constants

ZedXOneCamera::ZedXOneCamera(const rclcpp::NodeOptions & options)
: Node("zedxone_node", options),
  _stopNode(false),
  _setDynParams(true),
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
  switch (_pxMode) {
    case oc::PixelMode::COLOR_RGB:
      if (_swapRB) {
        _imgTrMsg->encoding = sensor_msgs::image_encodings::RGB8;
      } else {
        _imgTrMsg->encoding = sensor_msgs::image_encodings::BGR8;
      }
      break;
    case oc::PixelMode::COLOR_RGBA:
      if (_swapRB) {
        _imgTrMsg->encoding = sensor_msgs::image_encodings::RGBA8;
      } else {
        _imgTrMsg->encoding = sensor_msgs::image_encodings::BGRA8;
      }
      break;
    case oc::PixelMode::RAW10:
    case oc::PixelMode::RAW12:
      _imgTrMsg->encoding = sensor_msgs::image_encodings::BAYER_BGGR16;
      break;
  }

  _imgTrMsg->width = _width;
  _imgTrMsg->height = _height;
  _imgTrMsg->step = _width * _cam->getNumberOfChannels() * _cam->getPixelDepth();
  _imgTrMsg->is_bigendian = !(*reinterpret_cast<char *>(&num) == 1);

  DEBUG_GEN("*** Image Format ***");
  DEBUG_STREAM_GEN(" * Width: " << _imgTrMsg->width);
  DEBUG_STREAM_GEN(" * Height: " << _imgTrMsg->height);
  DEBUG_STREAM_GEN(" * Channels: " << _cam->getNumberOfChannels());
  DEBUG_STREAM_GEN(" * Depth: " << _cam->getPixelDepth());
  DEBUG_STREAM_GEN(" * Step: " << _imgTrMsg->step);
  DEBUG_STREAM_GEN(" * Size: " << _imgTrMsg->step * _imgTrMsg->height);
  DEBUG_STREAM_GEN(" * Format : " << _pxFormat);
  // <---- Create messages

  // ----> Create publishers
  RCLCPP_INFO(get_logger(), "*** Publishers ***");
  _pubImgTransp = image_transport::create_publisher(this, "image", _qos.get_rmw_qos_profile());
  RCLCPP_INFO_STREAM(get_logger(), "Advertised on topic: " << _pubImgTransp.getTopic());
  // <---- Create publishers

  // Set default values for dynamic parameters
  updateDynamicControls();

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

  // Dynamic parameters callback
  _paramChangeCallbackHandle = add_on_set_parameters_callback(
    std::bind(&ZedXOneCamera::callback_paramChange, this, _1));
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

  getParam("debug.controls", _debugControls, _debugControls);
  RCLCPP_INFO(
    get_logger(), " * Debug Controls: %s",
    _debugControls ? "TRUE" : "FALSE");
  // ************************************************** //

  _debugMode = _debugGeneral | _debugDiagnostic | _debugControls;

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
  getParam("camera.timeout_msec", _camTimeout_msec, _camTimeout_msec, " * Timeous [msec]: ");

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

  // ====================================
  // Dynamic parameters

  getParam("camera.dynamic.auto_exposure", _autoExposure, _autoExposure, std::string(), true);
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * [Dyn] Automatic Exposure: " << (_autoExposure ? "TRUE" : "FALSE"));

  getParam(
    "camera.dynamic.exposure_range_min", _exposureRange_min, _exposureRange_min,
    " * [Dyn] Exposure range min.: ", true);
  getParam(
    "camera.dynamic.exposure_range_max", _exposureRange_max, _exposureRange_max,
    " * [Dyn] Exposure range max.: ", true);

  getParam(
    "camera.dynamic.manual_exposure_usec", _manualExposure_usec, _manualExposure_usec,
    " * [Dyn] Manual Exposure [usec]: ", true);

  getParam(
    "camera.dynamic.auto_analog_gain", _autoAnalogGain, _autoAnalogGain, std::string(),
    true);
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * [Dyn] Automatic Analog gain: " << (_autoAnalogGain ? "TRUE" : "FALSE"));

  getParam(
    "camera.dynamic.analog_frame_gain_range_min", _analogFrameGainRange_min,
    _analogFrameGainRange_min, " * [Dyn] Analog Gain range min.: ", true);
  getParam(
    "camera.dynamic.analog_frame_gain_range_max", _analogFrameGainRange_max,
    _analogFrameGainRange_max, " * [Dyn] Analog Gain range max.: ", true);

  getParam(
    "camera.dynamic.manual_analog_gain_db", _manualAnalogGain_db, _manualAnalogGain_db,
    " * [Dyn] Manual Analog Gain [dB]: ", true);

  getParam(
    "camera.dynamic.auto_digital_gain", _autoDigitalGain, _autoDigitalGain,
    std::string(), true);
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * [Dyn] Automatic Digital gain: " << (_autoDigitalGain ? "TRUE" : "FALSE"));

  getParam(
    "camera.dynamic.digital_frame_gain_range_min", _digitalFrameGainRange_min,
    _digitalFrameGainRange_min, " * [Dyn] Digital Gain range min.: ", true);
  getParam(
    "camera.dynamic.digital_frame_gain_range_max", _digitalFrameGainRange_max,
    _digitalFrameGainRange_max, " * [Dyn] Digital Gain range max.: ", true);

  getParam(
    "camera.dynamic.manual_digital_gain_value", _manualDigitalGainValue,
    _manualDigitalGainValue, " * [Dyn] Manual Digital Gain: ", true);

  getParam("camera.dynamic.auto_wb", _autoWB, _autoWB, std::string(), true);
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * [Dyn] Automatic White Balance: " << (_autoWB ? "TRUE" : "FALSE"));

  getParam(
    "camera.dynamic.manual_wb", _manualWB, _manualWB, " * [Dyn] Manual White Balance [°]: ",
    true);

  std::string val_str = "AUTO";
  getParam("camera.dynamic.ae_anti_banding", val_str, val_str, std::string(), true);

  if (val_str == "OFF") {
    _aeAntiBanding = oc::AEANTIBANDING::OFF;
  } else if (val_str == "50Hz") {
    _aeAntiBanding = oc::AEANTIBANDING::HZ50;
  } else if (val_str == "60Hz") {
    _aeAntiBanding = oc::AEANTIBANDING::HZ60;
  } else {
    _aeAntiBanding = oc::AEANTIBANDING::AUTO;
    val_str = "AUTO";
  }
  RCLCPP_INFO_STREAM(get_logger(), " * [Dyn] Anti Banding: " << val_str);

  getParam(
    "camera.dynamic.color_saturation", _colorSaturation, _colorSaturation, " * [Dyn] Saturation: ",
    true);
  getParam("camera.dynamic.denoising", _denoising, _denoising, " * [Dyn] Denoising: ", true);
  getParam(
    "camera.dynamic.exposure_compensation", _exposureCompensation, _exposureCompensation,
    " * [Dyn] Exposure Compensation: ", true);
  getParam("camera.dynamic.sharpening", _sharpening, _sharpening, " * [Dyn] Sharpening: ", true);

  getParam(
    "camera.dynamic.tone_mapping_r_gamma", _toneMapping_R_gamma, _toneMapping_R_gamma,
    " * [Dyn] Tone Mapping Gamma RED: ", true);
  getParam(
    "camera.dynamic.tone_mapping_g_gamma", _toneMapping_G_gamma, _toneMapping_G_gamma,
    " * [Dyn] Tone Mapping Gamma GREEN: ", true);
  getParam(
    "camera.dynamic.tone_mapping_b_gamma", _toneMapping_B_gamma, _toneMapping_B_gamma,
    " * [Dyn] Tone Mapping Gamma BLUE: ", true);

  getParam(
    "camera.dynamic.aec_agc_roi_x", _aecAgcRoi_x, _aecAgcRoi_x, " * [Dyn] AEC/AGC ROI X: ",
    true);
  getParam(
    "camera.dynamic.aec_agc_roi_y", _aecAgcRoi_y, _aecAgcRoi_y, " * [Dyn] AEC/AGC ROI Y: ",
    true);
  getParam(
    "camera.dynamic.aec_agc_roi_w", _aecAgcRoi_w, _aecAgcRoi_w, " * [Dyn] AEC/AGC ROI Width: ",
    true);
  getParam(
    "camera.dynamic.aec_agc_roi_h", _aecAgcRoi_h, _aecAgcRoi_h, " * [Dyn] AEC/AGC ROI Height: ",
    true);

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
      if (elapsed.count() > _camTimeout_msec) {
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

    // Update Dynamic controls if required
    if (_setDynParams) {
      _setDynParams = false;
      updateDynamicControls();
    }
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

rcl_interfaces::msg::SetParametersResult ZedXOneCamera::callback_paramChange(
  std::vector<rclcpp::Parameter> parameters)
{
  DEBUG_STREAM_GEN("Parameter change callback");

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  DEBUG_STREAM_GEN("Modifying " << parameters.size() << " parameters");

  int count = 0;
  int count_ok = 0;

  for (const rclcpp::Parameter & param : parameters) {
    count++;

    DEBUG_STREAM_CONTROLS("Param #" << count << ": " << param.get_name());

    if (param.get_name() == "camera.dynamic.auto_exposure") {
      rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_BOOL;
      if (param.get_type() != correctType) {
        result.successful = false;
        result.reason =
          param.get_name() + " must be a " + rclcpp::to_string(correctType);
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      _autoExposure = param.as_bool();

      RCLCPP_INFO_STREAM(
        get_logger(), "Parameter '" << param.get_name() <<
          "' correctly set to " <<
          _autoExposure);
      count_ok++;
    }

    if (param.get_name() == "camera.dynamic.exposure_range_min") {
      rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_INTEGER;
      if (param.get_type() != correctType) {
        result.successful = false;
        result.reason =
          param.get_name() + " must be a " + rclcpp::to_string(correctType);
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      _exposureRange_min = param.as_int();

      RCLCPP_INFO_STREAM(
        get_logger(), "Parameter '" << param.get_name() <<
          "' correctly set to " <<
          _exposureRange_min);
      count_ok++;
    }

    if (param.get_name() == "camera.dynamic.exposure_range_max") {
      rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_INTEGER;
      if (param.get_type() != correctType) {
        result.successful = false;
        result.reason =
          param.get_name() + " must be a " + rclcpp::to_string(correctType);
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      _exposureRange_max = param.as_int();

      RCLCPP_INFO_STREAM(
        get_logger(), "Parameter '" << param.get_name() <<
          "' correctly set to " <<
          _exposureRange_max);
      count_ok++;
    }

    if (param.get_name() == "camera.dynamic.manual_exposure_usec") {
      rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_INTEGER;
      if (param.get_type() != correctType) {
        result.successful = false;
        result.reason =
          param.get_name() + " must be a " + rclcpp::to_string(correctType);
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      _manualExposure_usec = param.as_int();

      RCLCPP_INFO_STREAM(
        get_logger(), "Parameter '" << param.get_name() <<
          "' correctly set to " <<
          _manualExposure_usec);
      count_ok++;
    }

    if (param.get_name() == "camera.dynamic.auto_analog_gain") {
      rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_BOOL;
      if (param.get_type() != correctType) {
        result.successful = false;
        result.reason =
          param.get_name() + " must be a " + rclcpp::to_string(correctType);
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      _autoAnalogGain = param.as_bool();

      RCLCPP_INFO_STREAM(
        get_logger(), "Parameter '" << param.get_name() <<
          "' correctly set to " <<
          _autoAnalogGain);
      count_ok++;
    }

    if (param.get_name() == "camera.dynamic.analog_frame_gain_range_min") {
      rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_DOUBLE;
      if (param.get_type() != correctType) {
        result.successful = false;
        result.reason =
          param.get_name() + " must be a " + rclcpp::to_string(correctType);
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      _analogFrameGainRange_min = param.as_double();

      RCLCPP_INFO_STREAM(
        get_logger(), "Parameter '" << param.get_name() <<
          "' correctly set to " <<
          _analogFrameGainRange_min);
      count_ok++;
    }

    if (param.get_name() == "camera.dynamic.analog_frame_gain_range_max") {
      rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_DOUBLE;
      if (param.get_type() != correctType) {
        result.successful = false;
        result.reason =
          param.get_name() + " must be a " + rclcpp::to_string(correctType);
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      _analogFrameGainRange_max = param.as_double();

      RCLCPP_INFO_STREAM(
        get_logger(), "Parameter '" << param.get_name() <<
          "' correctly set to " <<
          _analogFrameGainRange_max);
      count_ok++;
    }

    if (param.get_name() == "camera.dynamic.manual_analog_gain_db") {
      rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_DOUBLE;
      if (param.get_type() != correctType) {
        result.successful = false;
        result.reason =
          param.get_name() + " must be a " + rclcpp::to_string(correctType);
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      _manualAnalogGain_db = param.as_double();

      RCLCPP_INFO_STREAM(
        get_logger(), "Parameter '" << param.get_name() <<
          "' correctly set to " <<
          _manualAnalogGain_db);
      count_ok++;
    }


  }

  if (result.successful) {
    RCLCPP_INFO_STREAM(
      get_logger(), "Correctly set " << count_ok << "/" <<
        parameters.size() <<
        " parameters");
    _setDynParams = true;
  }

  return result;
}

void ZedXOneCamera::updateDynamicControls()
{
  DEBUG_CONTROLS("*** Dynamic Camera Controls ***");
  int res;

  // ----> Exposure
  if (_autoExposure) {
    res = _cam->setAutomaticExposure();
    if (res != 0) {
      RCLCPP_WARN(get_logger(), "Failed to enable automatic Exposure");
    } else {
      DEBUG_CONTROLS("Automatic Exposure enabled");
    }
  } else {
    res = _cam->setFrameExposureRange(_exposureRange_min, _exposureRange_max);
    if (res != 0) {
      RCLCPP_WARN(get_logger(), "Failed to set Exposure Range");
    } else {
      DEBUG_CONTROLS("Set Frame Exposure Range OK");
    }
    res = _cam->setManualExposure(_manualExposure_usec);
    if (res != 0) {
      RCLCPP_WARN(get_logger(), "Failed to set Exposure");
    } else {
      DEBUG_CONTROLS("Set Exposure OK");
    }
  }
  // <---- Exposure

  // ----> Analog Gain
  if (_autoAnalogGain) {
    res = _cam->setAutomaticAnalogGain();
    if (res != 0) {
      RCLCPP_WARN(get_logger(), "Failed to enable automatic Analog Gain");
    } else {
      DEBUG_CONTROLS("Automatic Analog Gain enabled");
    }
  } else {
    res = _cam->setAnalogFrameGainRange(_analogFrameGainRange_min, _analogFrameGainRange_max);
    if (res != 0) {
      RCLCPP_WARN(get_logger(), "Failed to set Analog Gain Range");
    } else {
      DEBUG_CONTROLS("Set Frame Analog Gain Range OK");
    }
    res = _cam->setManualAnalogGainReal(_manualAnalogGain_db);
    if (res != 0) {
      RCLCPP_WARN(get_logger(), "Failed to set Analog Gain");
    } else {
      DEBUG_CONTROLS("Set Analog Gain OK");
    }
  }
  // <---- Analog Gain

  // ----> Digital Gain
  if (_autoDigitalGain) {
    res = _cam->setAutomaticDigitalGain();
    if (res != 0) {
      RCLCPP_WARN(get_logger(), "Failed to enable automatic Digital Gain");
    } else {
      DEBUG_CONTROLS("Automatic Digital Gain enabled");
    }
  } else {
    res = _cam->setDigitalFrameGainRange(
      static_cast<float>(_digitalFrameGainRange_min),
      static_cast<float>(_digitalFrameGainRange_max));
    if (res != 0) {
      RCLCPP_WARN(get_logger(), "Failed to set Digital Gain Range");
    } else {
      DEBUG_CONTROLS("Set Frame Digital Gain Range OK");
    }
    res = _cam->setManualDigitalGainReal(_manualDigitalGainValue);
    if (res != 0) {
      RCLCPP_WARN(get_logger(), "Failed to set Digital Gain");
    } else {
      DEBUG_CONTROLS("Set Digital Gain OK");
    }
  }
  // <---- Digital Gain

  // ----> White Balance
  if (_autoWB) {
    res = _cam->setAutomaticWhiteBalance(1);
    if (res != 0) {
      RCLCPP_WARN(get_logger(), "Failed to enable automatic White Balance");
    } else {
      DEBUG_CONTROLS("Automatic White Balance enabled");
    }
  } else {
    res = _cam->setManualWhiteBalance(static_cast<uint32_t>(_manualWB));
    if (res != 0) {
      RCLCPP_WARN(get_logger(), "Failed to set White Balance");
    } else {
      DEBUG_CONTROLS("Set White Balance OK");
    }
  }
  // <---- White Balance

  // ----> Anti Banding
  res = _cam->setAEAntiBanding(_aeAntiBanding);
  if (res != 0) {
    RCLCPP_WARN(get_logger(), "Failed to set AE Anti Banding");
  } else {
    DEBUG_CONTROLS("Set AE Anti Banding OK");
  }
  // <---- Anti Banding

  // ----> Saturation
  res = _cam->setColorSaturation(_colorSaturation);
  if (res != 0) {
    RCLCPP_WARN(get_logger(), "Failed to set Saturation");
  } else {
    DEBUG_CONTROLS("Set Saturation OK");
  }
  // <---- Saturation

  // ----> Denoising
  res = _cam->setDenoisingValue(_denoising);
  if (res != 0) {
    RCLCPP_WARN(get_logger(), "Failed to set Denoising");
  } else {
    DEBUG_CONTROLS("Set Denoising OK");
  }
  // <---- Denoising

  // ----> Exposure Compensation
  res = _cam->setExposureCompensation(_exposureCompensation);
  if (res != 0) {
    RCLCPP_WARN(get_logger(), "Failed to set Exposure Compensation");
  } else {
    DEBUG_CONTROLS("Set Exposure Compensation OK");
  }
  // <---- Exposure Compensation

  // ----> Sharpening
  res = _cam->setSharpening(_sharpening);
  if (res != 0) {
    RCLCPP_WARN(get_logger(), "Failed to set Sharpening");
  } else {
    DEBUG_CONTROLS("Set Sharpening OK");
  }
  // <---- Sharpening

  // ----> Tone Mapping RED
  res = _cam->setToneMappingFromGamma(0, _toneMapping_R_gamma);
  if (res != 0) {
    RCLCPP_WARN(get_logger(), "Failed to set Tone Mapping RED");
  } else {
    DEBUG_CONTROLS("Set Tone Mapping RED OK");
  }
  // <---- Tone Mapping RED

  // ----> Tone Mapping GREEN
  res = _cam->setToneMappingFromGamma(1, _toneMapping_G_gamma);
  if (res != 0) {
    RCLCPP_WARN(get_logger(), "Failed to set Tone Mapping GREEN");
  } else {
    DEBUG_CONTROLS("Set Tone Mapping GREEN OK");
  }
  // <---- Tone Mapping GREEN

  // ----> Tone Mapping BLUE
  res = _cam->setToneMappingFromGamma(2, _toneMapping_B_gamma);
  if (res != 0) {
    RCLCPP_WARN(get_logger(), "Failed to set Tone Mapping BLUE");
  } else {
    DEBUG_CONTROLS("Set Tone Mapping BLUE OK");
  }
  // <---- Tone Mapping BLUE

  // ----> AEC AGC ROI
  if (_aecAgcRoi_x == -1 ||
    _aecAgcRoi_y == -1 ||
    _aecAgcRoi_w == -1 ||
    _aecAgcRoi_h == -1)
  {
    res = _cam->resetROIforAECAGC();
    if (res != 0) {
      RCLCPP_WARN(get_logger(), "Failed to reset AEC/AGC ROI");
    } else {
      DEBUG_CONTROLS("Reset AEC/AGC ROI OK");
    }
  } else {
    oc::Rect roi;
    roi.x = _aecAgcRoi_x;
    roi.y = _aecAgcRoi_y;
    roi.width = _aecAgcRoi_w;
    roi.height = _aecAgcRoi_h;
    res = _cam->setROIforAECAGC(roi);
    if (res != 0) {
      RCLCPP_WARN(get_logger(), "Failed to set AEC/AGC ROI");
    } else {
      DEBUG_CONTROLS("Set AEC/AGC ROI OK");
    }
  }
  // <---- AEC AGC ROI
}

} // namespace stereolabs

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(stereolabs::ZedXOneCamera)
