/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rtk_config.h
 * Brief: Load resolved rtk_driver.yaml into DriverConfig + serial params (3.1)
 *
 * Description:
 * Maps the resolved config (data/run/resolved/rtk_driver.yaml, flat top-level
 * sections) onto the structs the driver needs. Every safety threshold is pulled
 * with a require_* accessor, so a missing or null value THROWS with its key path
 * and the process refuses to start (CLAUDE.md 3.1) -- there is no default here.
 *
 * Boundary: this maps the FILE fields only. The runtime identity (rid / src /
 * boot) is not in the file -- rid comes from common.robot_id, src is the fixed
 * "rtk_driver", boot is read from the OS at startup -- so the caller passes those
 * in. heading_stddev is stored in the file as DEGREES (human-facing) and
 * converted to radians here, the single conversion point.
 *
 * rtcm.* (11 S3.2A): a source whose `enabled` is false may leave its other keys
 * null -- nothing reads them, and the NTRIP port / mountpoint are still waiting
 * on the account details (V-RTCM-2). An enabled source must have every key.
 * rtcm.sink.queue_max_bytes below one maximum RTCM frame (1029 bytes) is
 * refused: such a queue would drop every large frame the moment it arrived.
 */
#ifndef HACHIST_XBRAIN_V6_SENSOR_RTK_CONFIG_H_
#define HACHIST_XBRAIN_V6_SENSOR_RTK_CONFIG_H_

#include <string>

#include "sensor/correction_link.h"
#include "sensor/rtcm_sink.h"
#include "sensor/rtk_driver.h"

namespace sensor {

// Everything main needs from the config file: the DriverConfig for RtkDriver
// plus the serial params for opening the port (serial is I/O, not a safety
// threshold, but it still lives in the one config file).
struct RtkConfig {
  DriverConfig driver;
  std::string serial_port;
  int serial_baud;
  SinkConfig sink;                 // rtcm.sink.*  (11 S3.2A RTG-6)
  CorrectionConfig corrections;    // rtcm.lan / ntrip / health / selector
};

// Parse the resolved yaml at `path` and fold in the runtime identity. Throws
// std::runtime_error (with the offending key path) on any missing / null /
// unparseable value -- fail-stop, never fail-silent (3.1).
RtkConfig load_rtk_config(const std::string& path, const std::string& rid,
                        const std::string& src, const std::string& boot);

}  // namespace sensor

#endif  // HACHIST_XBRAIN_V6_SENSOR_RTK_CONFIG_H_
