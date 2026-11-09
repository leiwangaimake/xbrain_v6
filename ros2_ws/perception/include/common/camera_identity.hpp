/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: camera_identity.hpp
 * Brief: PSC-3: is the one connected camera the configured one (serial number, literally)?
 *
 * Description:
 * 19 section 6, PSC-3: "the camera is connected and its serial number is the configured one",
 * refused at startup; A19-BOOT-2 tests it with a device list that reports another serial.
 * check_camera_identity() is that decision alone, on plain strings: the capture module reads the
 * serial numbers of the devices the SDK lists and asks here, so the rule can be tested without a
 * camera or the SDK.
 *
 * The rule, unchanged from what the capture module did inline: a serial must be configured;
 * exactly one device is listed (none: no camera; several: this process would not know which one
 * the calibration belongs to, also when one of them matches); its serial equals the configured
 * one as a whole string -- no prefix, no case folding, no trimming.
 */

#pragma once

#include <string>
#include <vector>

namespace perception {

struct CameraIdentityVerdict {
    bool ok = false;
    std::string reason;   // why not; empty when ok
};

inline CameraIdentityVerdict check_camera_identity(const std::string& configured_serial,
                                                   const std::vector<std::string>& listed_serials) {
    if (configured_serial.empty()) return {false, "no camera serial is configured"};
    if (listed_serials.size() != 1)
        return {false, "expected exactly one Orbbec device, found " + std::to_string(listed_serials.size())};
    if (listed_serials.front() != configured_serial)
        return {false, "Gemini serial mismatch expected=" + configured_serial + " actual=" + listed_serials.front()};
    return {true, {}};
}

}  // namespace perception
