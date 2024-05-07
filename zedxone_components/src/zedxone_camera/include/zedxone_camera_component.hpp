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

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/publisher.hpp>

#include "sl_types.hpp"
#include "sl_logging.hpp"
#include "sl_tools.hpp"

#include "ArgusCapture.hpp"

#define USE_THREAD // if defined grab is performed in a thread, otherwise a timer is used

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

  void callback_updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper & stat);
  rcl_interfaces::msg::SetParametersResult callback_paramChange(
    std::vector<rclcpp::Parameter> parameters);

  void initParameters();
  void initDebugParams();
  void initCamParams();

  void updateDynamicControls();

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
  bool _debugDiagnostic = false;
  bool _debugControls = false;

  // Camera
  std::string _model;
  int _deviceIdx = 0;
  int _fps = 30;
  std::string _resolution = "HD1080";
  bool _swapRB = false;
  std::string _pxFormat;
  int _camTimeout_msec = 2000;

  // Dynamic
  OnSetParametersCallbackHandle::SharedPtr _paramChangeCallbackHandle;

  bool _autoExposure;               // Enable Automatic Exposure
  int _exposureRange_min;           // Minimum value for Automatic Exposure
  int _exposureRange_max;           // Maximum value for Automatic Exposure
  int _manualExposure_usec = 2000;  // Manual Exposure time

  bool _autoAnalogGain;               // Enable Automatic Analog Gain
  float _analogFrameGainRange_min;    // Minimum value for Automatic Analog Gain
  float _analogFrameGainRange_max;    // Maximum value for Automatic Analog Gain
  float _manualAnalogGain_db;         // Manual Analog Gain

  bool _autoDigitalGain;              // Enable Automatic Digital Gain
  int _digitalFrameGainRange_min;     // Minimum value for Automatic Digital Gain
  int _digitalFrameGainRange_max;     // Maximum value for Automatic Digital Gain
  int _manualDigitalGainValue;        // Manual Digital Gain [1,256]

  bool _autoWB;   // Enable Automatic White Balance
  int _manualWB;  // Manual White Balance [2800,12000]

  oc::AEANTIBANDING _aeAntiBanding = oc::AEANTIBANDING::AUTO; // * Exposure anti banding - OFF, AUTO, 50Hz, 60Hz
  float _colorSaturation;          // * Color Saturation [0.0,2.0]
  float _denoising;                 // * Image Denoising [0.0,1.0]
  float _exposureCompensation;     // * Exposure Compensation [-2.0,2.0]
  float _sharpening;                // * Image Sharpening [0.0,1.0]

  int _aecAgcRoi_x;   // * AEC-AGC ROI top left x coordinate
  int _aecAgcRoi_y;   // * AEC-AGC ROI top left y coordinate
  int _aecAgcRoi_w;   // * AEC-AGC ROI width
  int _aecAgcRoi_h;   // * AEC-AGC ROI height

  float _toneMapping_R_gamma;     // [1.5,3.5]
  float _toneMapping_G_gamma;     // [1.5,3.5]
  float _toneMapping_B_gamma;     // [1.5,3.5]
  // <---- Parameters

  // ----> Running parameters
  bool _debugMode = false;
  int _width;
  int _height;
  oc::PixelMode _pxMode;
  std::atomic<bool> _setDynParams;
  // <---- Running parameters

  // ----> Running variables
#ifdef USE_THREAD
  std::thread _grabThread;
#else
  rclcpp::TimerBase::SharedPtr _frameGrabTimer;  // Timer to grab camera frames
#endif

  std::atomic<bool> _stopNode;
  image_transport::Publisher _pubImgTransp; // Publisher for camera stream over image_transport
  // <---- Running variables

  // ----> Diagnostic variables
  sl_tools::StopWatch _grabFreqStopWatch;
  std::unique_ptr<sl_tools::WinAvg> _grabPeriodMean_sec;

  int _imgTranspSubs = 0;

  diagnostic_updater::Updater _diagUpdater;  // Diagnostic Updater
  // <---- Diagnostic variables

  // ----> QoS
  // https://github.com/ros2/ros2/wiki/About-Quality-of-Service-Settings
  rclcpp::QoS _qos;
  rclcpp::PublisherOptions _pubOpt;
  rclcpp::SubscriptionOptions _subOpt;
  // <---- QoS

  // ----> Messages
  sensor_msgs::msg::Image::UniquePtr _imgTrMsg;
  // <---- Messages
};

} // namespace stereolabs

#endif
