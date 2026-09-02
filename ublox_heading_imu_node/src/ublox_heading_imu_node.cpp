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

#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "ublox_heading_imu_node/heading_conversion.hpp"
#include "ublox_heading_imu_node/visibility_control.h"
#include "ublox_ubx_msgs/msg/carr_soln.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_da_heading.hpp"

namespace ublox_heading_imu
{

// Variance used for the unobserved roll/pitch elements of the orientation
// covariance - orientation is present (yaw), so element 0 must not be -1
static constexpr double UNOBSERVED_VARIANCE = 1e6;

// Converts UBX-NAV-DAHEADING dual-antenna heading (ZED-X20D) into a
// sensor_msgs/Imu with a yaw-only orientation for robot_localization.
// UBX heading is degrees clockwise from True North (NED / compass
// convention); REP-103 yaw is radians counter-clockwise from East (ENU).
class UbloxHeadingImuNode : public rclcpp::Node
{
public:
  UBLOX_HEADING_IMU_NODE_PUBLIC
  explicit UbloxHeadingImuNode(const rclcpp::NodeOptions & options)
  : Node("ublox_heading_imu", options)
  {
    RCLCPP_INFO(this->get_logger(), "starting %s", get_name());

    // empty keeps the frame_id of the incoming UBX message
    frame_id_ = this->declare_parameter<std::string>("frame_id", "");
    // added to the reported NED heading before conversion; prefer the
    // device-side CFG_NAVSPG_DAHEADING_OFFSET and leave this at zero
    heading_offset_deg_ = this->declare_parameter<double>("heading_offset_deg", 0.0);
    // minimum carrier solution status to publish: 0 = any, 1 = RTK float,
    // 2 = RTK fixed
    min_carr_soln_ = this->declare_parameter<int>(
      "min_carr_soln",
      ublox_ubx_msgs::msg::CarrSoln::CARRIER_SOLUTION_PHASE_WITH_FIXED_AMBIGUITIES);
    // floor for the yaw standard deviation derived from acc_heading
    min_heading_std_dev_rad_ = this->declare_parameter<double>("min_heading_std_dev_rad", 0.001);

    auto qos = rclcpp::SensorDataQoS();
    rclcpp::PublisherOptions pub_options;
    pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("heading/imu", qos, pub_options);

    ubx_nav_da_heading_sub_ = this->create_subscription<ublox_ubx_msgs::msg::UBXNavDAHeading>(
      "ubx_nav_da_heading", qos,
      std::bind(&UbloxHeadingImuNode::nav_da_heading_callback, this, std::placeholders::_1));
  }

  UBLOX_HEADING_IMU_NODE_LOCAL
  ~UbloxHeadingImuNode() {RCLCPP_INFO(this->get_logger(), "finished");}

private:
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Subscription<ublox_ubx_msgs::msg::UBXNavDAHeading>::SharedPtr ubx_nav_da_heading_sub_;

  std::string frame_id_;
  double heading_offset_deg_;
  int min_carr_soln_;
  double min_heading_std_dev_rad_;

  UBLOX_HEADING_IMU_NODE_LOCAL
  void nav_da_heading_callback(const ublox_ubx_msgs::msg::UBXNavDAHeading::SharedPtr msg)
  {
    // never publish a heading the receiver has not marked valid
    if (!msg->gnss_fix_ok || !msg->rel_pos_heading_valid) {
      RCLCPP_DEBUG(
        this->get_logger(), "heading not valid (gnss_fix_ok: %d rel_pos_heading_valid: %d)",
        msg->gnss_fix_ok, msg->rel_pos_heading_valid);
      return;
    }
    if (msg->carr_soln.status < min_carr_soln_) {
      RCLCPP_DEBUG(
        this->get_logger(), "carr_soln %u below min_carr_soln %d",
        msg->carr_soln.status, min_carr_soln_);
      return;
    }

    double heading_deg = msg->rel_pos_heading * DAHEADING_SCALE_DEG;
    double yaw = ned_heading_deg_to_enu_yaw_rad(heading_deg, heading_offset_deg_);

    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header = msg->header;
    if (!frame_id_.empty()) {
      imu_msg.header.frame_id = frame_id_;
    }

    // yaw-only quaternion
    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = std::sin(yaw / 2.0);
    imu_msg.orientation.w = std::cos(yaw / 2.0);

    // roll/pitch are unobserved; yaw variance comes from the reported accuracy
    imu_msg.orientation_covariance[0] = UNOBSERVED_VARIANCE;
    imu_msg.orientation_covariance[4] = UNOBSERVED_VARIANCE;
    imu_msg.orientation_covariance[8] =
      heading_acc_to_yaw_variance(msg->acc_heading, min_heading_std_dev_rad_);

    // no angular velocity or linear acceleration measurements in this source
    imu_msg.angular_velocity_covariance[0] = -1.0;
    imu_msg.linear_acceleration_covariance[0] = -1.0;

    imu_pub_->publish(imu_msg);

    RCLCPP_DEBUG(
      this->get_logger(), "published heading %.5f deg -> yaw %.5f rad", heading_deg, yaw);
  }
};

}  // namespace ublox_heading_imu

RCLCPP_COMPONENTS_REGISTER_NODE(ublox_heading_imu::UbloxHeadingImuNode)
