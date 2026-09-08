# ZED-X20D bring-up guide

Hardware bring-up and verification procedure for the dual-antenna heading support.
Steps 1-4 were completed on a simpleRTK4 Dual on 2026-09-04 (see
[x20d-port-notes.md](x20d-port-notes.md)); re-run them on any new board or firmware.

## Requirements

- ZED-X20D with firmware **HDG 2.00 or later** (`UBX_20_HDG200` line). Verify with
  `UBX-MON-VER` in u-center 2 before anything else; earlier firmware uses a different
  (version 0x01) `NAV-DAHEADING` payload that this driver deliberately drops.
- Two identical antennas (e.g. ANN-MB2), identical cable lengths, mounted rigidly and
  oriented the same way. Baseline of 1 m or more for sub-degree heading; 0.3 m is the
  practical minimum.
- Heading definition: clockwise angle from True North of the baseline from **antenna 1
  (GPS1, master)** to **antenna 2 (GPS2, slave)**. On the ArduSimple simpleRTK4 Dual,
  the POWER+GPS USB-C port is the module's native USB - use that port.
- udev: install the `1546:01ab` rule from the top-level README. Its
  `ENV{ID_MM_DEVICE_IGNORE}="1"` is not optional - without it ModemManager opens
  `/dev/ttyACM0` exclusively and the libusb claim fails with `LIBUSB_ERROR_BUSY`.
- USB power: connect the board to a root port or a powered hub. Behind a bus-powered hub
  chain the module dropped off the bus (re-enumerating as low-speed with descriptor
  errors) as soon as the measurement rate was raised to 10 Hz.
- Measurement rate: keep `CFG_RATE_MEAS` at 1000 ms (the launch default). On a root port
  the dual-antenna fix holds at 1 Hz and 2 Hz (0.6-0.9 deg) but is lost within seconds at
  3 Hz and above; 4 Hz only coasts briefly at ~3.4 deg. Use 500 ms if the EKF needs 2 Hz. Changing the rate at runtime
  (`ros2 param set /ublox_dgnss CFG_RATE_MEAS ...`) is fine and the fix returns within
  seconds after going back to 1000 ms.
- Only one process may own the device. A stale driver holding the USB interface makes a
  new one fail with `LIBUSB_ERROR_BUSY`; a driver that is started while the device is
  absent logs `Starting USB initialization` every 10 ms until it appears.

## Checklist

1. **USB enumeration** - `lsusb -v -d 1546:`
   - Expect a single `1546:01ab` device with 2 interfaces (CDC control + CDC data).
   - If additional vendor-specific PIDs appear (analogous to the X20P's 0x050c/0x050d
     UART bridges), record them: they need to be added to `product_ids` in
     `device_family.cpp` and blocked like the X20P ones in `usb.cpp`.
   - Note whether iSerial is present and stable across replugs; if so, set
     `reliable_iserial = true` for X20D.
2. **First connection** -
   `ros2 launch ublox_dgnss ublox_x20d_rover_heading.launch.py`
   - The node must log `Device family: X20D ...` and reach CONNECTED.
   - Sweep the startup log for `not recognised` warnings and `NAK` messages from the
     config sweep; every offender needs an `@exclude: X20D` in `ubx_cfg_item_map.hpp`
     followed by `python3 ublox_dgnss/scripts/generate_toml_from_existing.py`.
   - `ros2 topic echo /ubx_nav_hp_pos_llh` proves position before touching heading.
3. **Config keys live** -
   `ros2 param get /ublox_dgnss CFG_MSGOUT_UBX_NAV_DAHEADING_USB` (expect 1 from the
   launch file) and `ros2 param get /ublox_dgnss CFG_NAVSPG_DAHEADING_OFFSET`.
   A NAK/timeout on either means the key id assumption failed - re-check against the
   interface description.
4. **Heading payload** - `ros2 topic echo /ubx_nav_da_heading`
   - `version` must be 2. Capture a raw `.ubx` log in parallel (u-center 2 or
     `ubxtool`), decode with `pyubx2`, and compare field-by-field with the ROS message,
     especially `rel_pos_heading` and `acc_heading` scaling (both 1e-5 deg).
   - Cross-check the heading value against u-center 2's dual-antenna view.
5. **Offset sign** - with `CFG_NAVSPG_DAHEADING_OFFSET` at 0, point the vehicle at a
   known bearing and read `rel_pos_heading`. For an athwartships baseline the reading
   is offset ~90 deg from the vehicle heading; set the launch argument
   `daheading_offset` (raw 0.01 deg units, so +90 deg = `9000`) so the reported heading
   equals the vehicle heading. Re-verify at N/E/S/W against a compass, and confirm
   `heading/imu` yaw follows REP-103 (0 = East, counter-clockwise positive).
6. **Validity gating** -
   - Cover one antenna: `rel_pos_heading_valid` must drop and `heading/imu` must stop
     publishing.
   - Feed RTCM corrections (NTRIP): `carr_soln.status` should reach 2 and, with the
     default `min_carr_soln = 2`, `heading/imu` should resume.
7. **Rates** - only if a faster heading is really needed: raise `CFG_RATE_MEAS` in steps
   and at each step confirm `rel_pos_heading_valid` stays 1 for a full minute (not just
   the first seconds) and `dmesg` shows no USB re-enumeration. On HDG 2.00 expect 1 Hz to
   be the usable rate.
8. Record the results in [x20d-port-notes.md](x20d-port-notes.md).

## robot_localization notes

`ublox_heading_imu_node` publishes `sensor_msgs/Imu` on `heading/imu` with:

- a yaw-only orientation (`yaw_enu = pi/2 - heading_ned`, normalized to (-pi, pi]),
- `orientation_covariance[8]` from `acc_heading` (floored by `min_heading_std_dev_rad`),
  large values on the unobserved roll/pitch diagonal,
- `angular_velocity_covariance[0] = -1` and `linear_acceleration_covariance[0] = -1`
  so the EKF ignores those blocks.

Feed it to the EKF as an `imu0`-style input with only the yaw element of the pose
enabled. Nothing is ever published while the receiver marks the heading invalid, so a
stale heading cannot reach the filter.
