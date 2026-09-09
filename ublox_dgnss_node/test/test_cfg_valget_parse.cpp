// Copyright 2026 Monkey Robotics
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

#include <vector>

#include "ublox_dgnss_node/ubx/ubx_cfg.hpp"

// CFG-VALGET reply payload: version, layer, position(U2), then key(U4) + packed value.
// Before the fix the value_t union was initialised from a single byte, so 1000 came back
// as 232 and -9000 as 0xD8.
TEST(CfgValGetParse, multi_byte_values_round_trip) {
  std::vector<ubx::u1_t> payload = {
    0x01, 0x00, 0x00, 0x00,
    // CFG_RATE_MEAS 0x30210001 (U2) = 1000 = 0x03E8 little endian
    0x01, 0x00, 0x21, 0x30, 0xE8, 0x03,
    // CFG_NAVSPG_DAHEADING_OFFSET 0x401100e4 (I4) = -9000 = 0xFFFFDCD8
    0xE4, 0x00, 0x11, 0x40, 0xD8, 0xDC, 0xFF, 0xFF,
    // CFG_USBOUTPROT_NMEA 0x10780002 (L) = 0
    0x02, 0x00, 0x78, 0x10, 0x00,
  };
  ubx::cfg::CfgValGetPayload p(
    reinterpret_cast<ubx::ch_t *>(payload.data()), static_cast<ubx::u2_t>(payload.size()));

  ASSERT_EQ(p.cfg_data.size(), 3u);
  EXPECT_EQ(p.cfg_data[0].ubx_key_id.all, 0x30210001u);
  EXPECT_EQ(p.cfg_data[0].ubx_value.u2, 1000u);
  EXPECT_EQ(p.cfg_data[1].ubx_key_id.all, 0x401100e4u);
  EXPECT_EQ(p.cfg_data[1].ubx_value.i4, -9000);
  EXPECT_EQ(p.cfg_data[2].ubx_key_id.all, 0x10780002u);
  EXPECT_EQ(p.cfg_data[2].ubx_value.l, 0u);
}

TEST(CfgValGetParse, packed_bytes_are_complete) {
  std::vector<ubx::u1_t> payload = {
    0x01, 0x00, 0x00, 0x00,
    // CFG_UART2_BAUDRATE 0x40530001 (U4) = 38400 = 0x00009600
    0x01, 0x00, 0x53, 0x40, 0x00, 0x96, 0x00, 0x00,
  };
  ubx::cfg::CfgValGetPayload p(
    reinterpret_cast<ubx::ch_t *>(payload.data()), static_cast<ubx::u2_t>(payload.size()));
  ASSERT_EQ(p.cfg_data.size(), 1u);
  EXPECT_EQ(p.cfg_data[0].ubx_value.u4, 38400u);
  EXPECT_EQ(p.cfg_data[0].ubx_value.bytes[1], 0x96);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
