/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_object_geometry_setup.cpp
 * Brief: Host test of the Objects geometry capability setup and its frame-level reason (B4-3).
 *
 * Description:
 * What it checks. Ready setup derives rays from the registration with base <- colour = base <-
 * depth o depth <- colour; "capability not available" keeps the service running with objects
 * unlocatable, while configuration or program faults are NOT capability outcomes; the frame-level
 * Objects reason enters through the Status owner's edges, holds across heartbeats and recovers
 * only on a ready frame. No SDK, OpenCV or device.
 */

#include "service/object_geometry_setup.hpp"
#include "service/profile_reason_state.hpp"

#include "output/rns_reason_edges.hpp"

#include <cmath>
#include <iostream>
#include <string>

using perception::RgbDepthRegistration;
using perception::rns::ObjectGeometryOptions;
using perception::rns::ReasonEdge;
using perception::rns::ReasonEdges;
using perception::rns::ReasonEdgeSeverity;
using perception::service::ObjectGeometryCapabilityError;
using perception::service::ObjectGeometrySession;
using perception::service::ProfileReasonState;
using perception::service::setup_object_geometry;

namespace {
int checks = 0, failures = 0;
#define CHECK(cond) do { ++checks; if (!(cond)) { ++failures; std::cerr << "FAIL " << __LINE__ << ": " #cond "\n"; } } while (0)

// base_link <- depth optical: camera 0.8 m high, level, looking along +x.
const std::string kYaml =
    "perception:\n"
    "  extrinsic_base_cam:\n"
    "    translation: [0.0, 0.0, 0.8]\n"
    "    rotation_xyzw: [-0.5, 0.5, -0.5, 0.5]\n"
    "    calib_record: null\n"
    "    calib_sha256: null\n"
    "    residual_max_m: null\n";
const ObjectGeometryOptions kOptions{5, 0.01, 0.02};

ObjectGeometrySession valid_session() {
    ObjectGeometrySession s;
    s.present = true;
    s.depth_to_colour_R = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    s.depth_to_colour_t_m = {-0.0238F, 0.0F, 0.0F};
    s.registration = std::make_shared<const RgbDepthRegistration>(64, 40, 50.0, 50.0, 31.5, 19.5,
                                                                  perception::StandardDistortion{-0.03, 0.03, -0.01, 0, 0});
    s.registration_status = "active";
    return s;
}

enum class Outcome { Ready, Capability, Fault };
Outcome classify(const ObjectGeometrySession& s, const std::string& yaml, const ObjectGeometryOptions& o, std::string* what = nullptr) {
    try {
        (void)setup_object_geometry(s, yaml, o);
        return Outcome::Ready;
    } catch (const ObjectGeometryCapabilityError& e) {
        if (what) *what = e.what();
        return Outcome::Capability;
    } catch (const std::exception& e) {
        if (what) *what = e.what();
        return Outcome::Fault;
    }
}
}  // namespace

int main() {
    // ---- ready: rays from the registration, base <- colour = base <- depth o depth <- colour ----
    {
        const auto s = valid_session();
        const auto ready = setup_object_geometry(s, kYaml, kOptions);
        CHECK(ready.builder && ready.registration == s.registration);
        // colour optical origin in base: depth origin (0,0,0.8) shifted by the inverse extrinsic
        // p_d = p_c - t  => colour origin at depth (+0.0238, 0, 0) => base y = -0.0238, x = 0, z = 0.8
        CHECK(std::fabs(ready.base_from_colour.t[0]) < 1e-9 && std::fabs(ready.base_from_colour.t[1] + 0.0238) < 1e-6 &&
              std::fabs(ready.base_from_colour.t[2] - 0.8) < 1e-9);
        // optical z (forward) maps to base x
        CHECK(std::fabs(ready.base_from_colour.R[6 - 6 + 2] - 1.0) < 1e-9 && ready.base_from_colour_R[2] == 1.0F);
    }
    // ---- capability not available (service continues, objects unlocatable) ----
    {
        std::string what;
        auto s = valid_session();
        s.present = false;
        CHECK(classify(s, kYaml, kOptions, &what) == Outcome::Capability && what.find("no session RGB-D calibration") != std::string::npos);
        s = valid_session();
        s.registration.reset();
        s.registration_status = "unavailable: firmware 1.9.0 not measured";
        CHECK(classify(s, kYaml, kOptions, &what) == Outcome::Capability && what.find("firmware 1.9.0 not measured") != std::string::npos);
        s = valid_session();
        s.depth_to_colour_R[0] = 1.5F;  // not a rotation
        CHECK(classify(s, kYaml, kOptions, &what) == Outcome::Capability && what.find("invalid SDK depth->colour extrinsic") != std::string::npos);
        s = valid_session();
        s.depth_to_colour_t_m[1] = std::nanf("");
        CHECK(classify(s, kYaml, kOptions) == Outcome::Capability);
    }
    // ---- configuration / program faults are NOT capability outcomes ----
    {
        std::string what;
        const auto s = valid_session();
        CHECK(classify(s, "perception:\n  profile:\n    n_bins: 5\n", kOptions, &what) == Outcome::Fault &&
              what.find("extrinsic_base_cam") != std::string::npos);
        const std::string bad_quaternion =
            "perception:\n  extrinsic_base_cam:\n    translation: [0, 0, 0.8]\n    rotation_xyzw: [0, 0, 0, 2]\n"
            "    calib_record: null\n    calib_sha256: null\n    residual_max_m: null\n";
        CHECK(classify(s, bad_quaternion, kOptions, &what) == Outcome::Fault && what.find("rotation_xyzw") != std::string::npos);
        CHECK(classify(s, kYaml, ObjectGeometryOptions{0, 0.01, 0.02}, &what) == Outcome::Fault &&
              what.find("builder rejected validated inputs") != std::string::npos);
    }
    // ---- frame-level Objects reason through the Status owner's edges ----
    {
        const std::uint64_t kFixed = 1U << 0, kObj = 1U << 1;
        ProfileReasonState objects(kObj);
        ReasonEdges edges(0, 8, 64);
        std::size_t warns = 0, infos = 0;
        std::int64_t now = 0;
        const auto poll = [&] {
            now += 10;
            (void)edges.update(kFixed | objects.current(), now);
            ReasonEdge e;
            while (edges.pop(e)) (e.severity == ReasonEdgeSeverity::Warn ? warns : infos) += 1;
        };
        poll();                                           // fixed reason, no Objects frame yet: no object reason
        CHECK(warns == 1 && (edges.active() & kObj) == 0);
        CHECK(objects.commit_frame(kObj));                 // capability unavailable at this Objects frame
        poll();
        CHECK(warns == 2 && (edges.active() & kObj));
        for (int i = 0; i < 500; ++i) poll();             // heartbeats, no new Objects frame: no pseudo recovery
        CHECK(infos == 0 && (edges.active() & kObj));
        for (int i = 0; i < 30; ++i) { CHECK(objects.commit_frame(kObj)); poll(); }  // persistent: never withdrawn
        CHECK(infos == 0 && warns == 2);
        CHECK(objects.commit_frame(0));                    // a ready Objects frame: real recovery edge
        poll();
        CHECK(infos == 1 && (edges.active() & kObj) == 0);
        CHECK(!objects.commit_frame(kFixed));               // only the tracked Objects bit is accepted
    }
    std::cout << "{\"result\":\"" << (failures ? "FAIL" : "PASS") << "\",\"checks\":" << checks << ",\"failures\":" << failures
              << ",\"scope\":\"object_geometry_setup_host\",\"sdk\":false,\"camera\":false}" << std::endl;
    return failures ? 1 : 0;
}
