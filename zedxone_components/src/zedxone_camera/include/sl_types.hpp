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

#ifndef SL_TYPES_HPP
#define ZEDXONE_CAMERA_COMPONENT_HPP

#include <rclcpp/rclcpp.hpp>
#include <image_transport/camera_publisher.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>

namespace stereolabs
{
// ----> Camera resolutions
const int SVGA_W = 960;
const int SVGA_H = 600;
const int HD1080_W = 1920;
const int HD1080_H = 1080;
const int HD1200_W = 1920;
const int HD1200_H = 1200;
const int HD4K_W = 3840;
const int HD4K_H = 2160;
// <---- Camera resolutions

} // namespace stereolabs

#endif //  SL_TYPES_HPP
