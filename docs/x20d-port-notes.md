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

## Confirmed on hardware (2026-09-04, simpleRTK4 Dual, HDG 2.00)

- `lsusb`: `1546:01ab`, 2 interfaces (CDC control + CDC data), endpoints 0x83 comms-in,
  0x01 data-out, 0x82 data-in - the standard CDC-ACM path, **no** UART-bridge PIDs.
  `iSerial` is empty, so `reliable_iserial = false` is right and `DEVICE_SERIAL_STRING`
  must stay empty.
- `UBX-MON-VER`: `MOD=ZED-X20D`, `FWVER=HDG 2.00`, `PROTVER=57.02`.
- NAV-DAHEADING v2 parse verified end to end: `atan2(relPosE, relPosN)` and
  `sqrt(N^2+E^2+D^2)` reproduce `relPosHeading` and `relPosLength` exactly; a 1 m bench
  baseline was reported as 979 mm.
- All 92 keys of `x20d_ubx_config.toml` answer `CFG-VALGET`; the 8 launch-file keys answer
  `CFG-VALSET` individually and as one frame (57 B, 64 B and 97 B frames all ACK).
- `CFG_NAVSPG_DAHEADING_OFFSET` accepts -18000 < v < 18000 (+/-180 deg in raw 0.01 deg);
  anything outside NAKs.
- **The HDG firmware NAKs any CFG-VALSET that arrives while the previous one is still
  being processed**: six small frames back to back answer `NNNNNA`, the same six one at a
  time answer `AAAAAA`. The driver's startup batching counted `n = 10` over iterated
  config items, i.e. 1-2 keys per frame in a 2 ms burst, so most launch parameters were
  lost on every other start. Fixed: one frame per 64 keys (the UBX maximum) in both the
  startup and the runtime sender.
- MON-RF / MON-SPAN are only ever emitted for antenna 1 (`recInf.msgSource = 1`) over USB,
  even while the heading is valid - their absence for antenna 2 means nothing.
- Heading validity vs `CFG_RATE_MEAS` (root port, no USB events at any rate): 1000 ms holds
  `relPosHeadingValid = 1`, `carrSoln = 2`, accuracy 0.6-0.9 deg indefinitely and re-fixes
  within seconds after a rate change. 333 ms (3 Hz) loses the heading within ~10 s, 200 ms
  falls back to 1 Hz output with the heading lost, 100 ms streams ~8 Hz with `carrSoln = 0`;
  250 ms held for one 15 s window at 3.4 deg accuracy, i.e. coasting. Behind the bus-powered
  hub chain the module additionally dropped off the bus when set to 100 ms (kernel:
  re-enumerating as low-speed, descriptor read errors) - that part was power. Launch default
  is 1000 ms; 500 ms (2 Hz) was also verified for 45 s at 0.61-0.63 deg, so 1-2 Hz is the
  usable range.
- Bench geometry: antenna 1 (RF1) on the left, antenna 2 (RF2) on the right, 1 m apart,
  "vehicle" facing west. Raw `relPosHeading` with offset 0 read 2-6 deg (the left-to-right
  baseline points north), and with `CFG_NAVSPG_DAHEADING_OFFSET = -9000` it read 270 deg =
  west. That is the athwartships case documented in the bring-up guide: RF1 left / RF2 right
  needs -9000, the mirror mount needs +9000.

## Still open

- The startup CFG-VALGET fetch is also sent in bursts; one DEBUG run logged
  `Missing response` for six keys and the driver ran degraded for them (harmless, but an
  ACK-paced sender would remove the race for both VALGET and VALSET).
- Offset sign on the robot: the bench had the baseline along the heading; an athwartships
  mount needs +/-9000 as described in the bring-up guide.

The full procedure is in [x20d-bringup.md](x20d-bringup.md).

## Corrections to earlier working assumptions

- The USB PID did not need to be guessed (and lsusb later confirmed it): the interface description publishes the
  `CFG-USB-PRODUCT_ID` default (427 = 0x01ab). It collides with the X20P main interface
  by design; the driver already treats the family as declared (F9P/F9R share 0x01a9).
- `NAV-SVIN` msgout keys and `CFG-SIGNAL-PLAN` **do** exist on the HDG firmware, so they
  are not excluded, even though survey-in itself (TMODE) is absent.
