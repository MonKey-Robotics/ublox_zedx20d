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

## Confirmed on hardware (2026-09-08/09, robot, cold start)

- **After the receiver has been unpowered for hours it ACKs the startup CFG-VALSET and does
  not apply it.** DEBUG run of 2026-09-08 09:58 (`component_container_mt_7248_...`): the
  single 8-key VALSET was ACK-ACKed 267 ms after it was sent - *after* 16 of the 18 CFG-VALGET
  ACKs of the parameter sweep - with zero NAKs, every VALGET answered, and the default NMEA
  set (`GGA/GLL/GSA/GSV/RMC/VTG/THS`, ~46 sentences/s) kept streaming; `CFG_MSGOUT_UBX_NAV_*`
  stayed 0, so `gps/fix` and `heading/imu` never published. Three driver restarts over
  11 minutes on 2026-09-09 (09:27, 09:29, 09:32) behaved identically; a USB unplug/replug
  cleared it at once and on 2026-09-08 it cleared by itself after ~30 min with no USB event.
  ModemManager was probing `ttyACM0` during the *successful* run, so it is not the cause.
- The old startup path (`ublox_send_user_params_async`, fire-and-forget, ACK-ACK logged at
  DEBUG, ACK-NAK a WARN with the retry commented out, no readback, no watchdog) could not
  see this. It is replaced by the config engine (`config_engine.hpp`): MON-VER handshake,
  one CFG-VALSET in flight, CFG-VALGET readback of every user key from the RAM layer, and a
  ladder when the readback disagrees - rung a transactionless retries with backoff
  (0.5/1/2/4 s), rung b the same keys as a configuration transaction (start ... apply; the
  interface description says a transaction from another source aborts a foreign one and
  unlocks the config database), rung c UBX-CFG-RST controlled software reset with hot start
  (`resetMode` from `CONFIG_ENGINE_RESET_MODE`, default 0x01; 0x00 is the hardware reset,
  closer to the replug), rung d one attempt every 30 s at ERROR level. Watch the log for
  `user configuration verified on device (8 keys)` (healthy) or `config NOT applied on
  device (rung a, attempt n/4): KEY expected X got Y` and the rung lines. **Which rung
  clears a real cold start is still to be observed** - record it here.
- **CFG-VALGET parser bug** (fixed): `CfgValGetPayload` built the 8-byte `value_t` from a
  single byte, so every multi-byte value read back truncated to its low byte
  (`CFG_RATE_MEAS` 1000 -> 232, `CFG_NAVSPG_DAHEADING_OFFSET` -9000 -> 0xD8). Nobody
  noticed because user keys were never read back; `test_cfg_valget_parse` pins it.
- `$GNTHS` keeps arriving once per second with `CFG_USBOUTPROT_NMEA=false` - the heading
  sentence bypasses the port protocol switch on HDG 2.00. Harmless; the engine's NMEA
  watchdog tolerates 3 sentences/s (`CONFIG_ENGINE_NMEA_WATCHDOG_PER_S`; a 5 s window
  sometimes counts 6 THS sentences, and 1.0 tripped a false re-verify on the bench).
- On a USB re-attach the old `reset_device_parameters()` dropped every `ros2 param set`
  value (it tested the *status*, which is `PARAM_VALSET` after a send); it now keeps every
  key whose *source* is the user and the engine re-applies them.

## Still open

- **The process aborts on exit** (`exit code -6`, `malloc_consolidate(): unaligned fastbin
  chunk detected`, a core in the CWD) - seen on every session since 2026-09-04, i.e. before
  the config engine. `Connection::shutdown()` used to run three times from two threads
  (rclcpp `on_shutdown` hook, node destructor, `~Connection`) and the core of 2026-09-09
  12:58 showed the second `libusb_close()` on the same handle; that is now guarded
  (idempotent, mutex), and the next core shows the corruption being detected later, in
  Fast-DDS static teardown (`TypeObjectFactory::delete_instance` -> `free`). So the heap is
  corrupted earlier by something else; candidates: `ublox_in_callback` writes `buf[len] = 0`
  one past `actual_length`, and the transfer cleanup in `close_devh()`. Needs an ASan build
  in the dev container. Harmless for operation (it only happens at exit) but it leaves
  60-100 MB cores behind.
- The startup CFG-VALGET sweep is still sent as a burst of ~18 frames; it now runs only
  after the user keys are verified, so it no longer races the VALSET, but one DEBUG run
  had logged `Missing response` for six keys (degraded mode, harmless). Pacing it one
  request at a time is the remaining clean-up.
- Offset sign on the robot: the bench had the baseline along the heading; an athwartships
  mount needs +/-9000 as described in the bring-up guide.

The full procedure is in [x20d-bringup.md](x20d-bringup.md).

## Corrections to earlier working assumptions

- The USB PID did not need to be guessed (and lsusb later confirmed it): the interface description publishes the
  `CFG-USB-PRODUCT_ID` default (427 = 0x01ab). It collides with the X20P main interface
  by design; the driver already treats the family as declared (F9P/F9R share 0x01a9).
- `NAV-SVIN` msgout keys and `CFG-SIGNAL-PLAN` **do** exist on the HDG firmware, so they
  are not excluded, even though survey-in itself (TMODE) is absent.
