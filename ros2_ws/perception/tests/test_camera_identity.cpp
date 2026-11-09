/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_camera_identity.cpp
 * Brief: A19-BOOT-2 / PSC-3: a device list that reports another serial refuses the start.
 *
 * Description:
 * What it checks. check_camera_identity() with the device lists a mocked SDK could return:
 * the configured camera alone is accepted; another serial is refused and both serials are in
 * the reason (the contract's mutant: the comparison skipped); a serial that only starts with,
 * ends with or differs in case or blanks from the configured one is another serial; no device
 * and more than one device are refused, also when one or all of them carry the configured
 * serial; no configured serial is refused whatever is listed.
 * The capture module's own use of the function (it reads the list from the SDK) is not run here:
 * that needs a camera.
 */

#include "common/camera_identity.hpp"

#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using perception::check_camera_identity;
int checks = 0;
void check(bool value) {
    ++checks;
    if (!value) throw std::runtime_error("camera identity check " + std::to_string(checks));
}
const std::string kSerial = "CHL5663000DS";
bool refused(const std::string& configured, const std::vector<std::string>& listed, const std::string& why) {
    const auto verdict = check_camera_identity(configured, listed);
    return !verdict.ok && verdict.reason.find(why) != std::string::npos;
}
}  // namespace

int main() {
    try {
        const auto good = check_camera_identity(kSerial, {kSerial});
        check(good.ok && good.reason.empty());
        // A19-BOOT-2: the SDK lists one camera with another serial.
        const auto other = check_camera_identity(kSerial, {"CHL5663000XX"});
        check(!other.ok && other.reason == "Gemini serial mismatch expected=CHL5663000DS actual=CHL5663000XX");
        const std::vector<std::string> near = {"CHL5663000D", "CHL5663000DSS", "HL5663000DS", "chl5663000ds", " CHL5663000DS",
                                               "CHL5663000DS ", "CHL5663000DS\n", "", "0"};
        for (const std::string& serial : near) check(refused(kSerial, {serial}, "serial mismatch expected=CHL5663000DS actual=" + serial));
        // The roles are not interchangeable: a longer configured serial does not accept its prefix.
        check(refused("CHL5663000DSS", {kSerial}, "serial mismatch"));
        // No camera, and more than one: also when the configured one is among them.
        check(refused(kSerial, {}, "exactly one Orbbec device, found 0"));
        check(refused(kSerial, {kSerial, "CHL5663000XX"}, "exactly one Orbbec device, found 2"));
        check(refused(kSerial, {"CHL5663000XX", kSerial}, "found 2"));
        check(refused(kSerial, {kSerial, kSerial}, "found 2"));
        check(refused(kSerial, {kSerial, kSerial, kSerial}, "found 3"));
        // Nothing configured: refused whatever is listed, an empty serial included.
        check(refused("", {kSerial}, "no camera serial is configured"));
        check(refused("", {""}, "no camera serial is configured"));
        check(refused("", {}, "no camera serial is configured"));
        std::cout << "CAMERA_IDENTITY_PASS checks=" << checks << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "CAMERA_IDENTITY_FAIL " << error.what() << '\n';
        return 1;
    }
}
