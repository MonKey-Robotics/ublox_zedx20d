# ZED-X20D port notes

Status of the X20D support added to this fork: what is verified against primary
documentation, and what still needs confirmation on real hardware. Search the tree for
`ASSUMED(X20D)` to find every hardware-pending assumption.

## Verified against primary documents (no hardware involved)

Source of record: *u-blox X20 HDG 2.00 Interface Description*, UBXDOC-304424225-21470 (R01).

| Fact | Value | Where used |
|---|---|---|
| USB VID / default PID | 0x1546 / 0x01ab (`CFG-USB-PRODUCT_ID` default 427) | `device_family.cpp` |
| UBX-NAV-DAHEADING identity | class 0x01, id 0x45, length 60, version 0x02 | `ubx_msg.hpp`, `ubx_nav_daheading.hpp` |
| Payload layout | version U1@0, iTOW U4@4, relPosN/E/D I4@8/12/16 (mm), relPosLength I4@20 (mm), relPosHeading I4@24 (1e-5 deg), accN/E/D/Length U4@32-44 (mm), accHeading U4@48 (1e-5 deg), flags X4@56 | `ubx_nav_daheading.hpp` |
| flags bits | 0 gnssFixOK, 1 diffSoln, 2 relPosValid, 4:3 carrSoln, 6 relPosHeadingValid | `ubx_nav_daheading.hpp` |
| `CFG_MSGOUT_UBX_NAV_DAHEADING_USB` | 0x209103e2, U1 | `ubx_cfg_item_map.hpp` |
| `CFG_NAVSPG_DAHEADING_OFFSET` | 0x401100e4, I4, scale 1e-2 deg, applied by the device to relPosHeading only | `ubx_cfg_item_map.hpp` |
| Absent from the HDG 2.00 config database | all `CFG-TMODE-*`, ESF / sensor fusion, NAV-ODO, `CFG-SFIMU/SFODO-*`, `CFG-SIGNAL-NAVIC-*`, `UBX-RXM-RTCM`, all `CFG-MSGOUT-RTCM_3X-*`, `CFG-UART2INPROT-UBX/NMEA` (UART2 input is RTCM3X/SPARTN only) | `@exclude`/`@only` annotations → `x20d_ubx_config.toml` |
| Present in HDG 2.00 (kept in the TOML) | `CFG-SIGNAL-PLAN` (default SP2), `CFG-MSGOUT-UBX_NAV_SVIN_USB`, RXM RAWX/SFRBX/COR/MEASX, `CFG-UARTxOUTPROT-RTCM3X` | `x20d_ubx_config.toml` |
| Board USB topology | ArduSimple simpleRTK4 Dual POWER+GPS USB-C is the module's **native USB** (the second USB-C is an FTDI bridge to the XBee socket only) | connection approach |
| Antenna convention | GPS1 = master (antenna 1), GPS2 = slave; heading is the clockwise angle from True North of the GPS1 to GPS2 baseline projection | `ublox_heading_imu_node` |

pyubx2 (`semuconsulting/pyubx2`) agrees with all of the above and additionally defines a
64-byte **version 0x01** payload with four high-precision I1 sub-fields at offset 32. That
variant is pre-production (not in the HDG 2.00 interface description) and is deliberately
not parsed: `NavDAHeadingPayload` accepts version 0x02 only and the node drops anything
else with a throttled warning.

## Still assumed - confirm when hardware arrives

- The X20D actually enumerates as `1546:01ab` with **2 CDC-ACM interfaces** (control +
  data) like the X20P main interface, and exposes **no** vendor-specific UART bridge PIDs
  analogous to the X20P's 0x050c/0x050d. `product_ids` in `device_family.cpp` lists only
  0x01ab; if `lsusb` shows bridge PIDs they must be added there and blocked the way the
  X20P ones are (`usb.cpp`).
- `reliable_iserial = false` (conservative, F9-style). If the board reports a stable
  factory iSerial, flip it.
- The x20d TOML include-list was derived by diffing this driver's key list against the
  interface description; the live device's VALGET/NAK behaviour is the final authority.
- `CFG_RATE_NAV`/`CFG_RATE_MEAS` limits for DAHEADING output (data sheet: 1 Hz default,
  up to 10 Hz).
- The sign of the athwartships mounting offset (+90 vs -90 in 0.01 deg raw units) for
  `CFG_NAVSPG_DAHEADING_OFFSET`.

The full procedure is in [x20d-bringup.md](x20d-bringup.md).

## Corrections to earlier working assumptions

- The USB PID did not need to be guessed: the interface description publishes the
  `CFG-USB-PRODUCT_ID` default (427 = 0x01ab). It collides with the X20P main interface
  by design; the driver already treats the family as declared (F9P/F9R share 0x01a9).
- `NAV-SVIN` msgout keys and `CFG-SIGNAL-PLAN` **do** exist on the HDG firmware, so they
  are not excluded, even though survey-in itself (TMODE) is absent.
