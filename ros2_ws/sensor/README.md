# rtk_driver

Dual-antenna RTK GNSS driver. **NOT a ROS 2 node** -- it is a non-ROS C++
process (CLAUDE.md 0.2) that publishes to the **RT-plane Zenoh**, not to ROS
topics. It builds with plain CMake (no rclcpp / ament / colcon).

> The directory is `ros2_ws/sensor/` only because `ros2_ws/` holds all robot-side
> C++ assets (0.2); the "not ROS" property is enforced by this package's
> CMakeLists linking no ROS target, not by where it lives.

## What it does

Parses **NMEA-0183** from the Unicore UM982-class dual-antenna module (WCH CH343
USB serial, `/dev/ttyACM0`, 115200 8N1, 20 Hz) and resolves a single authoritative
heading through the `11 S3.3` L1/L2/L3 state machine, then publishes the RT-plane
`GnssHeading` under `xbrain/{rid}/rt/gnss/heading` wrapped in the `11 S3.0` envelope.

| Level | Source | Admission (11 S3.3.1) | i_heading |
|---|---|---|---|
| L1 | `dual_antenna` (TRA) | baseline fixed (QF=4) && cov <= 0.02 rad && age <= 0.2 s | 1.0 |
| L2 | `cog` (RMC) | fix in {rtk_fixed, rtk_float} && speed >= 0.5 m/s sustained 0.5 s | 0.4 |
| L3 | `none` | neither -- heading frozen at last-known (H-2) | 0.0 |

`heading_valid` is the SOLE downstream criterion (H-1). Heading is in ENU
(east = 0, CCW); the module's true-north clockwise degrees are converted by
`heading_enu = wrap(pi/2 - heading_ned)`.

The driver is also the **sole `ClockStatus.sync` judge** (CLK-A1); the
`rt/clock/status` 1 Hz publish is a follow-up (B4b).

## Layout

- `include/sensor/`, `src/` -- the ROS-free, zenoh-free core:
  `nmea_parser` (GGA/TRA/RMC) -> `heading_resolver` (L1/L2/L3) ->
  `gnss_heading` (DTO + JSON) -> `rtk_driver` (wiring + envelope).
- `publish_sink.h` -- the Zenoh publish **seam** (abstract); the real sink lands
  when `zenoh-c` is installed (`11 S2.4.1`, version not yet locked).
- `src/main.cc` -- today a serial **smoke tool** (parse + print). It does NOT
  build a `RtkDriver`, because the safety thresholds must come from
  `configs/rtk_driver.yaml` via a C++ config loader that does not exist yet, and
  hardcoding them would be the silent default `3.1` forbids.

Config: `configs/rtk_driver.yaml` (V6 config root). Runtime reads the freeze
product `/run/xbrain/resolved/rtk_driver.yaml`, never the source (`10 S5.4.1`).

## Build / test (ON THE ORIN -- aarch64)

The ORIN has no colcon / ROS2; build with plain CMake. Do NOT build on the x86
host -- the binary must be aarch64.

```sh
ssh jack@192.168.1.7
cd /opt/xbrain_v6/ros2_ws/sensor
cmake -B build && cmake --build build
ctest --test-dir build --output-on-failure
```

Four offline test suites (no ROS, no hardware): `test_nmea_parser`,
`test_gnss_heading`, `test_heading_resolver`, `test_rtk_driver`.

Serial smoke test with the real module:

```sh
./build/rtk_driver /dev/ttyACM0 115200 40   # parse 40 sentences, print facts
```
