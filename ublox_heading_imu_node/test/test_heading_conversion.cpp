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

#include <gtest/gtest.h>
#include <cmath>

#include "ublox_heading_imu_node/heading_conversion.hpp"

using ublox_heading_imu::heading_acc_to_yaw_variance;
using ublox_heading_imu::ned_heading_deg_to_enu_yaw_rad;
using ublox_heading_imu::normalize_angle;

constexpr double TOL = 1e-9;

TEST(HeadingConversion, cardinal_north) {
  // NED heading 0 deg (North) -> ENU yaw +pi/2
  EXPECT_NEAR(ned_heading_deg_to_enu_yaw_rad(0.0), M_PI_2, TOL);
}

TEST(HeadingConversion, cardinal_east) {
  // NED heading 90 deg (East) -> ENU yaw 0
  EXPECT_NEAR(ned_heading_deg_to_enu_yaw_rad(90.0), 0.0, TOL);
}

TEST(HeadingConversion, cardinal_south) {
  // NED heading 180 deg (South) -> ENU yaw -pi/2
  EXPECT_NEAR(ned_heading_deg_to_enu_yaw_rad(180.0), -M_PI_2, TOL);
}

TEST(HeadingConversion, cardinal_west) {
  // NED heading 270 deg (West) -> ENU yaw +/-pi (wrapped)
  EXPECT_NEAR(std::abs(ned_heading_deg_to_enu_yaw_rad(270.0)), M_PI, TOL);
}

TEST(HeadingConversion, offset_applied_before_conversion) {
  // baseline heading 0 deg with a +90 deg athwartships offset reads East
  EXPECT_NEAR(ned_heading_deg_to_enu_yaw_rad(0.0, 90.0), 0.0, TOL);
  // and a -90 deg offset reads West
  EXPECT_NEAR(std::abs(ned_heading_deg_to_enu_yaw_rad(0.0, -90.0)), M_PI, TOL);
}

TEST(HeadingConversion, offset_wraps) {
  // heading 350 deg + offset 90 deg = 440 deg = 80 deg -> yaw 10 deg
  EXPECT_NEAR(ned_heading_deg_to_enu_yaw_rad(350.0, 90.0), 10.0 * M_PI / 180.0, TOL);
}

TEST(HeadingConversion, output_range) {
  for (double heading = -720.0; heading <= 720.0; heading += 7.5) {
    double yaw = ned_heading_deg_to_enu_yaw_rad(heading);
    EXPECT_GT(yaw, -M_PI - TOL);
    EXPECT_LE(yaw, M_PI + TOL);
  }
}

TEST(NormalizeAngle, boundaries) {
  EXPECT_NEAR(normalize_angle(M_PI), M_PI, TOL);
  EXPECT_NEAR(normalize_angle(-M_PI), M_PI, TOL);
  EXPECT_NEAR(normalize_angle(3.0 * M_PI), M_PI, TOL);
  EXPECT_NEAR(normalize_angle(0.0), 0.0, TOL);
}

TEST(HeadingAccToYawVariance, scaling) {
  // acc_heading raw 100000 = 1.0 deg standard deviation
  double expected_std = 1.0 * M_PI / 180.0;
  EXPECT_NEAR(heading_acc_to_yaw_variance(100000, 0.0), expected_std * expected_std, TOL);
}

TEST(HeadingAccToYawVariance, floor_applies) {
  // a tiny reported accuracy is clamped to the configured floor
  double floor_std = 0.01;
  EXPECT_NEAR(heading_acc_to_yaw_variance(1, floor_std), floor_std * floor_std, TOL);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
