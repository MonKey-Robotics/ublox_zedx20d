// Copyright 2026 Australian Robotics Supplies & Technology
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

#ifndef UBLOX_HEADING_IMU_NODE__HEADING_CONVERSION_HPP_
#define UBLOX_HEADING_IMU_NODE__HEADING_CONVERSION_HPP_

#include <cmath>

namespace ublox_heading_imu
{

// UBX-NAV-DAHEADING relPosHeading raw units are 1e-5 degrees
constexpr double DAHEADING_SCALE_DEG = 1e-5;

// Normalize an angle in radians to the range (-pi, pi]
inline double normalize_angle(double angle_rad)
{
  angle_rad = std::fmod(angle_rad, 2.0 * M_PI);
  if (angle_rad > M_PI) {
    angle_rad -= 2.0 * M_PI;
  } else if (angle_rad <= -M_PI) {
    angle_rad += 2.0 * M_PI;
  }
  return angle_rad;
}

// Convert a UBX heading (degrees clockwise from True North, NED / compass
// convention) to a REP-103 ENU yaw (radians counter-clockwise from East):
//   yaw_enu = pi/2 - heading_ned
// offset_deg is added to the heading before conversion; it corrects for an
// antenna baseline that is not aligned with the vehicle forward direction
// when the device-side CFG_NAVSPG_DAHEADING_OFFSET is not used.
inline double ned_heading_deg_to_enu_yaw_rad(double heading_deg, double offset_deg = 0.0)
{
  double heading_rad = (heading_deg + offset_deg) * M_PI / 180.0;
  return normalize_angle(M_PI_2 - heading_rad);
}

// Yaw variance (rad^2) from the raw UBX accHeading field (1e-5 degrees),
// clamped below by min_std_dev_rad
inline double heading_acc_to_yaw_variance(uint32_t acc_heading_raw, double min_std_dev_rad)
{
  double std_dev_rad = acc_heading_raw * DAHEADING_SCALE_DEG * M_PI / 180.0;
  if (std_dev_rad < min_std_dev_rad) {
    std_dev_rad = min_std_dev_rad;
  }
  return std_dev_rad * std_dev_rad;
}

}  // namespace ublox_heading_imu

#endif  // UBLOX_HEADING_IMU_NODE__HEADING_CONVERSION_HPP_
