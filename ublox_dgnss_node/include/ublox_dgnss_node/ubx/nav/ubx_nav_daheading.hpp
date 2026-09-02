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

#ifndef UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_DAHEADING_HPP_
#define UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_DAHEADING_HPP_

#include <unistd.h>
#include <memory>
#include <tuple>
#include <string>
#include "ublox_dgnss_node/ubx/ubx.hpp"
#include "ublox_dgnss_node/ubx/utils.hpp"

namespace ubx::nav::daheading
{

enum carrier_solution_status_t : u1_t {no_carrier_range_solution = 0,
  carrier_phase_solution_with_floating_ambiguities = 1,
  carrier_phase_solution_with_fixed_ambiguities = 2 };

struct status_flags_t
{
  union {
    x4_t all;
    struct
    {
      l_t gnssFixOK : 1;                // 1 = A valid fix (i.e within DOP & accuracy masks)
      l_t diffSoln : 1;                 // 1 = differential corrections were applied
      l_t relPosValid : 1;              // 1 = relative position components, accuracy and
                                        // baseline are valid
      carrier_solution_status_t carrSoln : 2;  // carrier phase range solution
      l_t reserved : 1;                 // reserved
      l_t relPosHeadingValid : 1;       // 1 = relPosHeading is valid
    } bits;
  };
};

// UBX-NAV-DAHEADING - dual-antenna heading (ZED-X20D, HDG 2.00 firmware).
// Only payload version 0x02 (60 bytes) is parsed; other versions set
// parse_valid false so the node can drop the frame instead of mis-parsing.
class NavDAHeadingPayload : UBXPayload
{
public:
  static const msg_class_t MSG_CLASS = UBX_NAV;
  static const msg_id_t MSG_ID = UBX_NAV_DAHEADING;
  static const u1_t PAYLOAD_VERSION = 0x02;
  static const u2_t PAYLOAD_SIZE = 60;

  u1_t version;      // message version (0x02 for this version)
  u4_t iTOW;      // ms - GPS Time of week of the navigation epoch.
  i4_t relPosN;      // mm - North component of relative position vector from antenna 1 to antenna 2
  i4_t relPosE;      // mm - East component of relative position vector from antenna 1 to antenna 2
  i4_t relPosD;      // mm - Down component of relative position vector from antenna 1 to antenna 2
  i4_t relPosLength;    // mm - Length of the relative position vector
  i4_t relPosHeading;    // deg scale 1e-5 - clockwise angle from True North to the projection of
                         // the antenna 1 to antenna 2 baseline, range 0 to 360 deg; adjusted by
                         // CFG_NAVSPG_DAHEADING_OFFSET
  u4_t accN;      // mm - Accuracy of relative position North component
  u4_t accE;      // mm - Accuracy of relative position East component
  u4_t accD;      // mm - Accuracy of relative position Down component
  u4_t accLength;      // mm - Accuracy of length of the relative position vector
  u4_t accHeading;      // deg scale 1e-5 - Accuracy of heading of the relative position vector
  status_flags_t flags;     // navigation status flags
  bool parse_valid;     // true only when a known payload version was fully parsed

public:
  NavDAHeadingPayload()
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
    version = 0;
    iTOW = 0;
    relPosN = relPosE = relPosD = relPosLength = relPosHeading = 0;
    accN = accE = accD = accLength = accHeading = 0;
    flags.all = 0;
    parse_valid = false;
  }
  NavDAHeadingPayload(ch_t * payload_polled, u2_t size)
  : NavDAHeadingPayload()
  {
    payload_.clear();
    payload_.reserve(size);
    payload_.resize(size);
    memcpy(payload_.data(), payload_polled, size);
    if (size < 1) {
      return;
    }
    version = buf_offset<u1_t>(&payload_, 0);
    if (version != PAYLOAD_VERSION || size < PAYLOAD_SIZE) {
      return;
    }
    iTOW = buf_offset<u4_t>(&payload_, 4);
    relPosN = buf_offset<i4_t>(&payload_, 8);
    relPosE = buf_offset<i4_t>(&payload_, 12);
    relPosD = buf_offset<i4_t>(&payload_, 16);
    relPosLength = buf_offset<i4_t>(&payload_, 20);
    relPosHeading = buf_offset<i4_t>(&payload_, 24);
    accN = buf_offset<u4_t>(&payload_, 32);
    accE = buf_offset<u4_t>(&payload_, 36);
    accD = buf_offset<u4_t>(&payload_, 40);
    accLength = buf_offset<u4_t>(&payload_, 44);
    accHeading = buf_offset<u4_t>(&payload_, 48);
    flags = buf_offset<status_flags_t>(&payload_, 56);
    parse_valid = true;
  }
  std::tuple<u1_t *, size_t> make_poll_payload()
  {
    payload_.clear();
    return std::make_tuple(payload_.data(), payload_.size());
  }
  std::string to_string()
  {
    std::ostringstream oss;
    oss << std::fixed;
    oss << "ver: " << +version;
    oss << " iTOW: " << iTOW;
    oss << " relPos - N: " << +relPosN;
    oss << " E: " << +relPosE;
    oss << " D: " << +relPosD;
    oss << " length: " << +relPosLength;
    oss << std::setprecision(5);
    oss << " heading: " << +relPosHeading * 1e-5;
    oss << std::setprecision(0);
    oss << " acc - N: " << +accN;
    oss << " E: " << +accE;
    oss << " D: " << +accD;
    oss << " length: " << +accLength;
    oss << std::setprecision(5);
    oss << " heading: " << +accHeading * 1e-5;
    oss << " flags - ";
    oss << std::setprecision(0);
    oss << " gnssFixOK: " << +flags.bits.gnssFixOK;
    oss << " diffSoln: " << +flags.bits.diffSoln;
    oss << " relPosValid: " << +flags.bits.relPosValid;
    oss << " carrSoln: " << +flags.bits.carrSoln;
    oss << " relPosHeadingValid: " << +flags.bits.relPosHeadingValid;
    oss << " parseValid: " << +parse_valid;

    return oss.str();
  }
};
}  // namespace ubx::nav::daheading
#endif  // UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_DAHEADING_HPP_
