/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_service_config.cpp
 * Brief: Host test of the formal R6 service startup configuration and its self-checks.
 *
 * Description:
 * What it checks. With the dev fixture as the positive case: PSC-1, the MANIFEST, PSC-2 null
 * discipline and exact key tables (including the extrinsic and geometry setup sections), the
 * implementation section (dual_model.enabled is a required YAML bool that must be true), the
 * runtime switch table (retired keys are refused), the optional switch profile_thread_cpu
 * (absent: the runtime digest of a configuration is the one it had before the switch existed;
 * reading the configuration does not judge the caller's CPU set, on Linux with this test
 * narrowed to one core; the startup plan from the inherited CPU set), the inherited environment
 * and the PSC-5 key registry. No camera, SDK, model file, network or /run write is used.
 */

#include "service/service_config.hpp"

#include "common/config.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <unistd.h>
#include <utility>

namespace fs = std::filesystem;
using namespace perception::service;

namespace {
int checks = 0, failures = 0, rejections = 0;
#define CHECK(cond) do { ++checks; if (!(cond)) { ++failures; std::cerr << "FAIL " << __LINE__ << ": " #cond "\n"; } } while (0)

const char* kBoot = "0123abcd-0000-4000-8000-00000000abcd";

std::string read_file(const fs::path& p) {
    std::ifstream in(p, std::ios::binary);
    std::ostringstream s;
    s << in.rdbuf();
    return s.str();
}
void write_file(const fs::path& p, const std::string& text) {
    std::ofstream out(p, std::ios::binary | std::ios::trunc);
    out << text;
}
std::string replace_once(std::string s, const std::string& from, const std::string& to) {
    const auto at = s.find(from);
    if (at == std::string::npos) { std::cerr << "fixture pattern missing: " << from << "\n"; std::exit(2); }
    return s.replace(at, from.size(), to);
}

struct Env {
    fs::path dir;
    std::string fixture;
    Env() {
        char tmpl[] = "/tmp/svc_config_test.XXXXXX";
        dir = fs::canonical(::mkdtemp(tmpl));
        fixture = read_file(fs::path(FIXTURE_DIR) / "perception.dev_candR.yaml");
    }
    ~Env() { std::error_code ec; fs::remove_all(dir, ec); }
    std::string prefix() const { return dir.string() + "/"; }

    std::string manifest(const std::string& yaml_path, const std::string& sha, const std::string& boot = kBoot,
                         const std::string& rid = "xbrain-01", bool with_perception = true) const {
        std::string perception = with_perception
            ? ",\"perception\":{\"path\":\"" + yaml_path + "\",\"refs\":0,\"sha256\":\"" + sha + "\"}" : "";
        return "{\"schema\":\"xbrain.config.manifest/1\",\"gen_ts\":1.0,\"boot_id\":\"" + boot +
               "\",\"config_rev\":\"2026-09-17T00:00:00Z+gtest\",\"robot_id\":\"" + rid +
               "\",\"site_id\":\"dev\",\"common_digest\":\"0123456789abcdef\",\"processes\":{\"p1_motion\":"
               "{\"path\":\"/opt/xbrain_v6/data/run/resolved/p1_motion.yaml\",\"refs\":1,\"sha256\":\"x\"}" + perception + "}}";
    }
    // Writes yaml + a MANIFEST binding it (unless manifest_text is given).
    LoadInputs stage(const std::string& yaml, std::string manifest_text = "") const {
        const auto yaml_path = prefix() + "perception.yaml";
        write_file(yaml_path, yaml);
        if (manifest_text.empty()) manifest_text = manifest(yaml_path, yolo::sha256_hex(yaml));
        write_file(prefix() + "MANIFEST.json", manifest_text);
        LoadInputs in;
        in.resolved_prefix = prefix();
        in.resolved_path = yaml_path;
        in.manifest_path = prefix() + "MANIFEST.json";
        in.host.boot_id = kBoot;
        return in;
    }
};

void expect_reject(const std::string& name, const std::function<void()>& run,
                  const std::string& check, const std::string& key_contains,
                  const std::string& why_contains = "") {
    ++checks;
    try {
        run();
        ++failures;
        std::cerr << "FAIL accepted: " << name << "\n";
    } catch (const StartupCheckError& e) {
        if (e.check() != check || e.key().find(key_contains) == std::string::npos ||
            std::string(e.what()).find(why_contains) == std::string::npos) {
            ++failures;
            std::cerr << "FAIL wrong rejection " << name << ": " << e.what() << "\n";
        } else {
            ++rejections;
        }
    } catch (const std::exception& e) {
        ++failures;
        std::cerr << "FAIL non-startup exception " << name << ": " << e.what() << "\n";
    }
}
}  // namespace

int main() {
    Env env;
    const auto& base = env.fixture;

    // ---- positive: dev fixture ----
    {
        const auto in = env.stage(base);
        const auto c = load_service_config(in);
        CHECK(c.rid == "xbrain-01" && c.boot8 == "0123abcd");
        CHECK(c.resolved_sha256 == yolo::sha256_hex(base) && c.manifest.perception_sha256 == c.resolved_sha256);
        CHECK(c.contract.profile.n_bins == 181 && c.contract.profile.range_max_m == 6.0);
        CHECK(!c.contract.profile.dfree_cap_fallback_m && !c.contract.profile.fallback_align_err_m);
        CHECK(c.contract.profile.plane_fit_min_inlier_frac == 0.5);
        CHECK(c.contract.extrinsic.mode == ExtrinsicMode::Absent && !c.contract.extrinsic.translation);
        CHECK(c.contract.latency_fallback_ms == 23 && c.contract.status_window_s == 10.0);
        CHECK(c.contract.med2.rtsp_port == 18083 && !c.contract.med2.pub_port && c.contract.med2.bitrate_kbps == 4000);
        CHECK(c.contract.rt_endpoint == "tcp/127.0.0.1:7449" && c.contract.gen_endpoint == "tcp/127.0.0.1:7447");
        CHECK(c.runtime.size() == runtime_switch_keys().size() && c.runtime.size() == 37);
        CHECK(c.runtime_digest.size() == 64);
        CHECK(!c.pipeline["runtime"] && !c.pipeline["media"] && c.pipeline["camera"] && c.pipeline["dual_model"]);
        CHECK(c.pipeline["hevc"]["sink"].as<std::string>() == "med2_loopback");
        CHECK(c.pipeline["hevc"]["rtsp_listen"].as<std::string>() == "rtsp://127.0.0.1:18083/rgb");
        CHECK(c.pipeline["hevc"]["bitrate"].as<int>() == 4000000 && c.pipeline["hevc"]["codec"].as<std::string>() == "nvv4l2h265enc");
        CHECK(!c.pipeline["rtsp_forward"] && !c.pipeline["visualization"]);
        CHECK(c.ros.enable && !c.ros.tf_enable && c.ros.odom_frame == "odom" && c.ros.base_frame == "base_link" && !c.pipeline["ros"]);
        bool seen_unset = false, seen_delay = false, seen_release = false, seen_release_log = false;
        for (const auto& s : c.runtime) {
            if (s.key == "coco_mask_resize") seen_unset = !s.set;
            if (s.key == "jit_bind_delay_ms") seen_delay = s.set && s.value == "10.6921890700000084";
            if (s.key == "objects_release_delay_ms")
                seen_release = s.set && s.value == "0" && s.env == "PERCEPTION_OBJECTS_RELEASE_DELAY_MS";
            if (s.key == "objects_release_log")
                seen_release_log = s.set && s.value == "0" && s.env == "PERCEPTION_OBJECTS_RELEASE_LOG";
        }
        CHECK(seen_unset && seen_delay && seen_release && seen_release_log);
        // deterministic digest
        CHECK(load_service_config(in).runtime_digest == c.runtime_digest);
    }
    // fallback-branch keys may carry values
    {
        auto y = replace_once(base, "    fallback_align_err_m: null", "    fallback_align_err_m: 0.03");
        y = replace_once(y, "    dfree_cap_fallback_m: null", "    dfree_cap_fallback_m: 2.0");
        const auto c = load_service_config(env.stage(y));
        CHECK(c.contract.profile.fallback_align_err_m && *c.contract.profile.fallback_align_err_m == 0.03);
    }
    // INT8 build files (2026-10-10): accepted by absolute path, refused relative (PSC-2)
    {
        const std::string sha(64, 'a');
        const auto with_int8 = [&](const std::string& ranges) {
            return replace_once(base, "      int8: false", "      int8: true\n      int8_ranges: " + ranges +
                                                             "\n      int8_ranges_sha256: " + sha);
        };
        const auto c = load_service_config(env.stage(with_int8("/opt/xbrain_v6/models/r.cache")));
        CHECK(c.pipeline["inference"]["int8_ranges"].as<std::string>() == "/opt/xbrain_v6/models/r.cache");
        expect_reject("relative inference.int8_ranges", [&] {
            load_service_config(env.stage(with_int8("models/r.cache")));
        }, "PSC-2", "inference.int8_ranges", "absolute");
        expect_reject("relative dual_model.int8_fp16_layers", [&] {
            load_service_config(env.stage(replace_once(base, "      workspace_mb: 1024",
                                                       "      workspace_mb: 1024\n      int8_fp16_layers: l.txt")));
        }, "PSC-2", "dual_model.int8_fp16_layers", "absolute");
    }
    // configured transform (flow and block vectors), uncertified record fields
    {
        const std::string setup = "  native_geometry_setup:\n    schema_version: 1\n";
        auto flow = replace_once(base, "    translation:      [null, null, null]", "    translation:      [0.35, 0.0, 0.645]");
        flow = replace_once(flow, "    rotation_xyzw:    [null, null, null, null]", "    rotation_xyzw:    [0, 0, 0, 1]");
        flow = replace_once(flow, "  med2:", setup + "  med2:");
        auto c = load_service_config(env.stage(flow));
        CHECK(c.contract.extrinsic.mode == ExtrinsicMode::TransformConfigured && (*c.contract.extrinsic.translation)[2] == 0.645);
        auto block = replace_once(base, "    translation:      [null, null, null]", "    translation:\n    - 0.35\n    - 0.0\n    - 0.645");
        block = replace_once(block, "    rotation_xyzw:    [null, null, null, null]", "    rotation_xyzw:\n    - 0\n    - 0\n    - 0\n    - 1");
        block = replace_once(block, "  med2:", setup + "  med2:");
        c = load_service_config(env.stage(block));
        CHECK(c.contract.extrinsic.mode == ExtrinsicMode::TransformConfigured && (*c.contract.extrinsic.rotation_xyzw)[3] == 1);
        expect_reject("transform without native_geometry_setup", [&] {
            auto y = replace_once(base, "    translation:      [null, null, null]", "    translation:      [0.35, 0.0, 0.645]");
            y = replace_once(y, "    rotation_xyzw:    [null, null, null, null]", "    rotation_xyzw:    [0, 0, 0, 1]");
            load_service_config(env.stage(y));
        }, "PSC-2", "native_geometry_setup");
        expect_reject("non-unit quaternion", [&] {
            load_service_config(env.stage(replace_once(flow, "[0, 0, 0, 1]", "[0.0, 0.130526, 0.0, 0.991445]")));
        }, "PSC-2", "rotation_xyzw");
        // record triple: all set is accepted (verdict comes later), partial or relative is refused
        const auto with_record = [&](const std::string& path, bool sha, bool residual) {
            auto y = replace_once(flow, "    calib_record:     null", "    calib_record:     " + path);
            if (sha) y = replace_once(y, "    calib_sha256:     null", "    calib_sha256:     " + std::string(64, 'a'));
            if (residual) y = replace_once(y, "    residual_max_m:   null", "    residual_max_m:   0.01");
            return y;
        };
        c = load_service_config(env.stage(with_record("/var/lib/xbrain/calib/record.yaml", true, true)));
        CHECK(c.contract.extrinsic.calib_record && *c.contract.extrinsic.calib_record == "/var/lib/xbrain/calib/record.yaml" &&
              c.contract.extrinsic.calib_sha256 && c.contract.extrinsic.residual_max_m && *c.contract.extrinsic.residual_max_m == 0.01);
        expect_reject("partial record triple (path only)", [&] {
            load_service_config(env.stage(with_record("/var/lib/xbrain/calib/record.yaml", false, false)));
        }, "PSC-2", "extrinsic_base_cam", "all null or all set");
        expect_reject("partial record triple (no residual)", [&] {
            load_service_config(env.stage(with_record("/var/lib/xbrain/calib/record.yaml", true, false)));
        }, "PSC-2", "extrinsic_base_cam", "all null or all set");
        expect_reject("relative record path", [&] {
            load_service_config(env.stage(with_record("calib/record.yaml", true, true)));
        }, "PSC-2", "calib_record", "absolute");
        // Each record field is checked on its own, before the all-or-none rule;
        // this is the only admission of these values (the geometry parser
        // accepts the keys but does not read them).
        for (const auto& bad : {std::string("abc"), std::string(64, 'z')})
            expect_reject("calib_sha256 " + bad.substr(0, 3), [&] {
                load_service_config(env.stage(replace_once(flow, "    calib_sha256:     null", "    calib_sha256:     " + bad)));
            }, "PSC-2", "calib_sha256");
        for (const auto* bad : {"-1", "nan", "inf", "true", "[]"})
            expect_reject(std::string("residual_max_m ") + bad, [&] {
                load_service_config(env.stage(replace_once(flow, "    residual_max_m:   null", std::string("    residual_max_m:   ") + bad)));
            }, "PSC-2", "residual_max_m");
    }
    // a comment line inside the pipeline at column 2 must not end the section
    {
        const auto y = replace_once(base, "    runtime:\n", "  # operator note inside pipeline\n    runtime:\n");
        CHECK(load_service_config(env.stage(y)).runtime.size() == runtime_switch_keys().size());
    }

    // ---- PSC-1 ----
    // The service defaults name the platform freeze output root; the former /run location is
    // outside it and refused before any file is opened.
    {
        const std::string root = "/opt/xbrain_v6/data/run/resolved/";
        CHECK(std::string(kResolvedPrefix) == root);
        CHECK(std::string(kResolvedConfigPath) == root + "perception.yaml");
        CHECK(std::string(kManifestPath) == root + "MANIFEST.json");
        const LoadInputs service_defaults;
        CHECK(service_defaults.resolved_prefix == root && service_defaults.resolved_path == root + "perception.yaml" &&
              service_defaults.manifest_path == root + "MANIFEST.json");
    }
    expect_reject("former /run location", [&] {
        LoadInputs in;
        in.resolved_path = "/run/xbrain/resolved/perception.yaml";
        in.host.boot_id = kBoot;
        load_service_config(in);
    }, "PSC-1", "/run/xbrain/resolved/perception.yaml", "/opt/xbrain_v6/data/run/resolved/");
    expect_reject("config outside resolved prefix", [&] {
        auto in = env.stage(base);
        const fs::path outside = env.dir.parent_path() / ("svc_outside_" + std::to_string(::getpid()) + ".yaml");
        write_file(outside, base);
        in.resolved_path = outside.string();
        try { load_service_config(in); } catch (...) { fs::remove(outside); throw; }
        fs::remove(outside);
    }, "PSC-1", "svc_outside_");
    expect_reject("symlink to source", [&] {
        auto in = env.stage(base);
        const fs::path source = env.dir.parent_path() / ("svc_source_" + std::to_string(::getpid()) + ".yaml");
        write_file(source, base);
        fs::remove(in.resolved_path);
        fs::create_symlink(source, in.resolved_path);
        try { load_service_config(in); } catch (...) { fs::remove(source); throw; }
        fs::remove(source);
    }, "PSC-1", "perception.yaml");
    expect_reject("missing resolved file", [&] {
        auto in = env.stage(base);
        fs::remove(in.resolved_path);
        load_service_config(in);
    }, "PSC-1", "perception.yaml");

    // ---- MANIFEST ----
    const auto yaml_path = env.prefix() + "perception.yaml";
    expect_reject("boot mismatch", [&] {
        load_service_config(env.stage(base, env.manifest(yaml_path, yolo::sha256_hex(base), "ffffffff-0000-4000-8000-00000000abcd")));
    }, "MANIFEST", "boot_id");
    expect_reject("sha mismatch", [&] {
        load_service_config(env.stage(base, env.manifest(yaml_path, yolo::sha256_hex(base + "\n"))));
    }, "MANIFEST", "sha256");
    expect_reject("perception not in snapshot", [&] {
        load_service_config(env.stage(base, env.manifest(yaml_path, yolo::sha256_hex(base), kBoot, "xbrain-01", false)));
    }, "MANIFEST", "processes.perception");
    expect_reject("invalid robot id", [&] {
        load_service_config(env.stage(base, env.manifest(yaml_path, yolo::sha256_hex(base), kBoot, "Robot01")));
    }, "MANIFEST", "robot_id");
    expect_reject("manifest path names another file", [&] {
        load_service_config(env.stage(base, env.manifest(kResolvedConfigPath, yolo::sha256_hex(base))));
    }, "MANIFEST", "path");
    expect_reject("malformed host boot id", [&] {
        auto in = env.stage(base);
        in.host.boot_id = "not-a-boot-id";
        load_service_config(in);
    }, "MANIFEST", "boot_id");

    // ---- PSC-2 null discipline and exact key tables ----
    for (const auto* key : {"h_tol_m", "z_pass_m", "tau_T", "plane_fit_min_inlier_frac", "seg_max_age_ms"}) {
        expect_reject(std::string("null ") + key, [&] {
            std::string y = base;
            const auto at = y.find(std::string("    ") + key + ":");
            const auto eol = y.find('\n', at);
            y.replace(at, eol - at, std::string("    ") + key + ": null");
            load_service_config(env.stage(y));
        }, "PSC-2", std::string("perception.profile.") + key);
    }
    // seg_max_age_ms must stay below the 10 s segmentation window minus 1 s (extended self-check).
    for (const auto* value : {"9000", "20000"}) {
        expect_reject(std::string("seg_max_age_ms ") + value, [&] {
            std::string y = base;
            const auto at = y.find("    seg_max_age_ms:");
            const auto eol = y.find('\n', at);
            y.replace(at, eol - at, std::string("    seg_max_age_ms: ") + value);
            load_service_config(env.stage(y));
        }, "PSC-2", "perception.profile.seg_max_age_ms");
    }
    {
        std::string y = base;
        const auto at = y.find("    seg_max_age_ms:");
        const auto eol = y.find('\n', at);
        y.replace(at, eol - at, "    seg_max_age_ms: 8999");
        CHECK(load_service_config(env.stage(y)).contract.profile.seg_max_age_ms == 8999);
    }
    expect_reject("null depth_quality_fair (not self-exempted)", [&] {
        load_service_config(env.stage(replace_once(base, "    depth_quality_fair: 0.6", "    depth_quality_fair: null")));
    }, "PSC-2", "depth_quality_fair");
    expect_reject("missing key", [&] {
        load_service_config(env.stage(replace_once(base, "    eps_neg:          0.15\n", "")));
    }, "PSC-2", "perception.profile.eps_neg");
    expect_reject("unknown profile key", [&] {
        load_service_config(env.stage(replace_once(base, "    eps_neg:", "    extrinsic_calibrated: true\n    eps_neg:")));
    }, "PSC-2", "perception.profile.extrinsic_calibrated");
    expect_reject("common top-level key", [&] {
        load_service_config(env.stage("common:\n  robot_id: x\n" + base));
    }, "PSC-2", "common");
    expect_reject("range_max 6.5 (A19-BOOT-5)", [&] {
        load_service_config(env.stage(replace_once(base, "    range_max_m:      6.0", "    range_max_m:      6.5")));
    }, "PSC-6", "range_max_m");
    expect_reject("partial transform (translation only)", [&] {
        load_service_config(env.stage(replace_once(base, "    translation:      [null, null, null]", "    translation:      [0.1, 0.2, 0.3]")));
    }, "PSC-2", "extrinsic_base_cam", "both null or both set");
    expect_reject("partial transform (rotation only)", [&] {
        load_service_config(env.stage(replace_once(base, "    rotation_xyzw:    [null, null, null, null]", "    rotation_xyzw:    [0, 0, 0, 1]")));
    }, "PSC-2", "extrinsic_base_cam", "both null or both set");
    expect_reject("partial vector", [&] {
        load_service_config(env.stage(replace_once(base, "    translation:      [null, null, null]", "    translation:      [0.1, null, 0.3]")));
    }, "PSC-2", "translation");
    expect_reject("record without transform", [&] {
        load_service_config(env.stage(replace_once(base, "    residual_max_m:   null", "    residual_max_m:   0.02")));
    }, "PSC-2", "extrinsic_base_cam", "without a transform");
    expect_reject("asymmetric angle domain (upstream sample value)", [&] {
        load_service_config(env.stage(replace_once(base, "    angle_min_rad:    -0.78543 ", "    angle_min_rad:    -0.785398 ")));
    }, "PSC-2", "angle_min_rad");
    expect_reject("even bin count", [&] {
        load_service_config(env.stage(replace_once(base, "    n_bins:           181", "    n_bins:           180")));
    }, "PSC-2", "n_bins");
    expect_reject("debug point cloud", [&] {
        load_service_config(env.stage(replace_once(base, "    pointcloud_enable: false", "    pointcloud_enable: true")));
    }, "PSC-2", "pointcloud_enable");
    expect_reject("legacy keys", [&] {
        load_service_config(env.stage(replace_once(base, "    legacy_keys_enable: false", "    legacy_keys_enable: true")));
    }, "PSC-2", "legacy_keys_enable");
    expect_reject("window not 10 s", [&] {
        load_service_config(env.stage(replace_once(base, "    window_s:         10.0", "    window_s:         5.0")));
    }, "PSC-2", "window_s");
    expect_reject("wildcard router bind", [&] {
        load_service_config(env.stage(replace_once(base, "\"tcp/127.0.0.1:7449\"", "\"tcp/0.0.0.0:7449\"")));
    }, "PSC-2", "rt_endpoint");
    expect_reject("same router for both planes", [&] {
        load_service_config(env.stage(replace_once(base, "\"tcp/127.0.0.1:7447\"", "\"tcp/127.0.0.1:7449\"")));
    }, "PSC-2", "perception.zenoh");
    expect_reject("pub port equals loopback port", [&] {
        load_service_config(env.stage(replace_once(base, "    rgbd_pub_port:    null", "    rgbd_pub_port:    18083")));
    }, "PSC-2", "rgbd_pub_port");

    // ---- implementation section ----
    expect_reject("missing pipeline", [&] {
        load_service_config(env.stage(base.substr(0, base.find("  pipeline:\n"))));
    }, "PSC-2", "perception.pipeline");
    expect_reject("duplicate pipeline", [&] {
        load_service_config(env.stage(base + "  pipeline:\n    log:\n      level: INFO\n"));
    }, "PSC-2", "perception.pipeline");
    expect_reject("retired hevc section", [&] {
        load_service_config(env.stage(replace_once(base, "    media:\n", "    hevc:\n      enable: true\n    media:\n")));
    }, "PSC-2", "perception.pipeline.hevc");
    expect_reject("relative engine path", [&] {
        load_service_config(env.stage(replace_once(base, "\"/home/nvidia/perception_test_338Le/models/yolo11n-seg-dla0-fp16.engine\"",
                                                "\"models/yolo11n-seg-dla0-fp16.engine\"")));
    }, "PSC-2", "trt_engine_path");
    // dual_model.enabled (level-3 trim): a required YAML bool that must be true,
    // refused at PSC-2 before any dual-model JSON, engine or model file is read.
    // Supersedes the B8-1 regression "dual model off reads no files": that
    // branch no longer exists.
    const std::string dual_on = "    dual_model:\n      enabled: true\n";
    expect_reject("dual model disabled", [&] {
        load_service_config(env.stage(replace_once(base, dual_on, "    dual_model:\n      enabled: false\n")));
    }, "PSC-2", "perception.pipeline.dual_model.enabled", "must be true");
    expect_reject("dual model enabled key missing", [&] {
        load_service_config(env.stage(replace_once(base, dual_on, "    dual_model:\n")));
    }, "PSC-2", "perception.pipeline.dual_model.enabled", "missing key");
    expect_reject("dual model enabled null", [&] {
        load_service_config(env.stage(replace_once(base, dual_on, "    dual_model:\n      enabled: null\n")));
    }, "PSC-2", "perception.pipeline.dual_model.enabled", "null");
    expect_reject("dual model enabled not a YAML bool", [&] {
        load_service_config(env.stage(replace_once(base, dual_on, "    dual_model:\n      enabled: 1\n")));
    }, "PSC-2", "perception.pipeline.dual_model.enabled", "must be a YAML bool");
    expect_reject("dual model section not a map", [&] {
        const auto from = base.find(dual_on);
        const auto to = base.find("    queue:\n", from);
        if (from == std::string::npos || to == std::string::npos) throw std::runtime_error("fixture dual_model block not found");
        load_service_config(env.stage(base.substr(0, from) + "    dual_model: true\n" + base.substr(to)));
    }, "PSC-2", "perception.pipeline.dual_model", "must be a map");
    // Read as yolo::Config reads it: other YAML spellings of true are accepted.
    for (const char* spelling : {"True", "yes", "on"})
        CHECK(load_service_config(env.stage(replace_once(base, dual_on, std::string("    dual_model:\n      enabled: ") + spelling + "\n")))
                  .pipeline["dual_model"]["enabled"].as<bool>());
    // Negative control for "before any read": the runtime JSON below does not
    // exist. With enabled: true PSC-2 accepts it (load_service_config reads no
    // dual-model file), so the refusal with enabled: false is the key's, never
    // the missing file's.
    {
        const auto absent = env.prefix() + "absent/dual_model_runtime_config.json";
        const auto moved = replace_once(base, "\"/home/nvidia/perception_test_338Le/config/dual_model_runtime_config.json\"",
                                       "\"" + absent + "\"");
        CHECK(!fs::exists(absent));
        const auto c = load_service_config(env.stage(moved));
        CHECK(c.pipeline["dual_model"]["runtime_config_path"].as<std::string>() == absent);
        expect_reject("dual model disabled, runtime JSON absent", [&] {
            load_service_config(env.stage(replace_once(moved, dual_on, "    dual_model:\n      enabled: false\n")));
        }, "PSC-2", "perception.pipeline.dual_model.enabled", "must be true");
    }
    // PSC-3 depth stream (19 section 11: 640x400 Y16@30). The camera block is the
    // only place with a 4-space "depth:" under pipeline.camera; locate it there.
    const auto with_depth = [&](const std::string& from, const std::string& to) {
        std::string y = base;
        const auto cam = y.find("      depth:\n        # Vendor-designated");
        if (cam == std::string::npos) throw std::runtime_error("fixture camera.depth block not found");
        return y.substr(0, cam) + replace_once(y.substr(cam), from, to);
    };
    expect_reject("depth width not 640 (PSC-3)", [&] {
        load_service_config(env.stage(with_depth("        width: 640", "        width: 848")));
    }, "PSC-3", "pipeline.camera.depth.width");
    expect_reject("depth height not 400 (PSC-3)", [&] {
        load_service_config(env.stage(with_depth("        height: 400", "        height: 530")));
    }, "PSC-3", "pipeline.camera.depth.height");
    expect_reject("depth fps not 30 (PSC-3)", [&] {
        load_service_config(env.stage(with_depth("        fps: 30", "        fps: 15")));
    }, "PSC-3", "pipeline.camera.depth.fps");
    expect_reject("depth format not y16 (PSC-3)", [&] {
        load_service_config(env.stage(with_depth("        device_format: y16", "        device_format: mjpg")));
    }, "PSC-3", "pipeline.camera.depth.device_format");
    expect_reject("main output not 1280x800", [&] {
        std::string y = base;
        const auto out = y.find("      output:");
        y = y.substr(0, out) + replace_once(y.substr(out), "        width: 1280", "        width: 1920");
        load_service_config(env.stage(y));
    }, "PSC-3", "camera.output.width");
    // B8-6: 19 section 13.1 default MED-2 (H.264 1280x720@15, 2000 kbps) loads
    {
        auto y = replace_once(base, "      codec: nvv4l2h265enc", "      codec: nvv4l2h264enc");
        y = replace_once(y, "    height: 800\n    fps: 30\n", "    height: 720\n    fps: 15\n");
        y = replace_once(y, "    bitrate_kbps: 4000", "    bitrate_kbps: 2000");
        const auto c = load_service_config(env.stage(y));
        CHECK(c.pipeline["hevc"]["codec"].as<std::string>() == "nvv4l2h264enc" &&
              c.pipeline["hevc"]["width"].as<int>() == 1280 && c.pipeline["hevc"]["height"].as<int>() == 720 &&
              c.pipeline["hevc"]["fps"].as<int>() == 15 && c.pipeline["hevc"]["bitrate"].as<int>() == 2000000);
    }
    expect_reject("software encoder", [&] {
        load_service_config(env.stage(replace_once(base, "      codec: nvv4l2h265enc", "      codec: x264enc")));
    }, "PSC-2", "media.codec");
    expect_reject("preview rate not dividing 30", [&] {
        load_service_config(env.stage(replace_once(base, "    height: 800\n    fps: 30\n", "    height: 720\n    fps: 20\n")));
    }, "PSC-2", "perception.med2");
    expect_reject("preview crop not centred", [&] {
        load_service_config(env.stage(replace_once(base, "    height: 800\n    fps: 30\n", "    height: 721\n    fps: 15\n")));
    }, "PSC-2", "perception.med2");
    expect_reject("preview scaling", [&] {
        load_service_config(env.stage(replace_once(base, "    width: 1280\n    height: 800\n    fps: 30\n    bitrate_kbps",
                                                "    width: 1920\n    height: 800\n    fps: 30\n    bitrate_kbps")));
    }, "PSC-2", "perception.med2");

    expect_reject("ros section missing key", [&] {
        load_service_config(env.stage(replace_once(base, "      base_frame: base_link\n", "")));
    }, "PSC-2", "perception.pipeline.ros");
    expect_reject("tf without executor", [&] {
        auto y = replace_once(base, "      enable: true\n      tf_enable: false", "      enable: false\n      tf_enable: true");
        load_service_config(env.stage(y));
    }, "PSC-2", "tf_enable");
    expect_reject("tf enabled (odom-frame velocity unsupported)", [&] {
        load_service_config(env.stage(replace_once(base, "      enable: true\n      tf_enable: false", "      enable: true\n      tf_enable: true")));
    }, "PSC-2", "ros.tf_enable", "odom-frame velocity");
    expect_reject("ros flag not boolean", [&] {
        load_service_config(env.stage(replace_once(base, "      enable: true\n      tf_enable", "      enable: yes\n      tf_enable")));
    }, "PSC-2", "ros.enable");

    // ---- runtime switches ----
    expect_reject("non-production switch (c16 media nice)", [&] {
        load_service_config(env.stage(replace_once(base, "    runtime:\n", "    runtime:\n      media_nice: \"10\"\n")));
    }, "RUNTIME", "media_nice");
    expect_reject("missing switch", [&] {
        load_service_config(env.stage(replace_once(base, "      jit_frame_move: \"1\"\n", "")));
    }, "RUNTIME", "jit_frame_move");
    expect_reject("diagnostic value", [&] {
        load_service_config(env.stage(replace_once(base, "      preprocess_impl: fused", "      preprocess_impl: shadow")));
    }, "RUNTIME", "preprocess_impl");
    expect_reject("null switch", [&] {
        load_service_config(env.stage(replace_once(base, "      async_log: \"1\"", "      async_log: null")));
    }, "RUNTIME", "async_log");
    // traversable_seg (issues.md 3.1, 2026-10-08): only "off" or "on".
    expect_reject("T switch value", [&] {
        load_service_config(env.stage(replace_once(base, "      traversable_seg: \"on\"", "      traversable_seg: \"auto\"")));
    }, "RUNTIME", "traversable_seg");
    // The two guards of 2026-10-10: only the listed values.
    expect_reject("padding guard value", [&] {
        load_service_config(env.stage(replace_once(base, "      active12_padding_anchor_guard: \"1\"", "      active12_padding_anchor_guard: \"on\"")));
    }, "RUNTIME", "active12_padding_anchor_guard");
    expect_reject("publish guard value", [&] {
        load_service_config(env.stage(replace_once(base, "      semantic_publish_guard_ms: \"20\"", "      semantic_publish_guard_ms: \"12\"")));
    }, "RUNTIME", "semantic_publish_guard_ms");
    expect_reject("missing publish guard", [&] {
        load_service_config(env.stage(replace_once(base, "      semantic_publish_guard_ms: \"20\"\n", "")));
    }, "RUNTIME", "semantic_publish_guard_ms");
    // Paced Objects release (2026-10-10): listed delays only, both keys mandatory, and a delay
    // other than "0" needs the two-slot JIT loop (the only one that records the ready time).
    {
        const std::string off = "      objects_release_delay_ms: \"0\"";
        const auto paced = replace_once(base, off, "      objects_release_delay_ms: \"43\"");
        bool seen = false;
        for (const auto& s : load_service_config(env.stage(paced)).runtime)
            if (s.key == "objects_release_delay_ms") seen = s.set && s.value == "43";
        CHECK(seen);
        expect_reject("release delay value", [&] {
            load_service_config(env.stage(replace_once(base, off, "      objects_release_delay_ms: \"44\"")));
        }, "RUNTIME", "objects_release_delay_ms");
        expect_reject("release delay with a unit", [&] {
            load_service_config(env.stage(replace_once(base, off, "      objects_release_delay_ms: \"43ms\"")));
        }, "RUNTIME", "objects_release_delay_ms");
        expect_reject("missing release delay", [&] {
            load_service_config(env.stage(replace_once(base, off + "\n", "")));
        }, "RUNTIME", "objects_release_delay_ms");
        expect_reject("release log value", [&] {
            load_service_config(env.stage(replace_once(base, "      objects_release_log: \"0\"", "      objects_release_log: \"2\"")));
        }, "RUNTIME", "objects_release_log");
        expect_reject("missing release log", [&] {
            load_service_config(env.stage(replace_once(base, "      objects_release_log: \"0\"\n", "")));
        }, "RUNTIME", "objects_release_log");
        expect_reject("release delay on the serial loop", [&] {
            load_service_config(env.stage(replace_once(paced, "      two_slot_jit: \"1\"", "      two_slot_jit: \"0\"")));
        }, "RUNTIME", "objects_release_delay_ms");
        expect_reject("release delay with the loop unset", [&] {
            load_service_config(env.stage(replace_once(paced, "      two_slot_jit: \"1\"", "      two_slot_jit: unset")));
        }, "RUNTIME", "objects_release_delay_ms");
        // Delay "0" or unset is accepted on either loop.
        const auto serial = replace_once(base, "      two_slot_jit: \"1\"", "      two_slot_jit: \"0\"");
        CHECK(load_service_config(env.stage(serial)).runtime.size() == runtime_switch_keys().size());
        CHECK(load_service_config(env.stage(replace_once(serial, off, "      objects_release_delay_ms: unset"))).runtime.size() ==
              runtime_switch_keys().size());
    }
    // Minimum submit interval (2026-10-20): "0" (off) or a whole number of milliseconds 1..49 in
    // one spelling (50 and more would put almost every interval on or over the 50 ms line),
    // mandatory, handed on as PERCEPTION_INFER_MIN_INTERVAL_MS; a cap needs the two-slot JIT loop
    // (the only one that paces) with async submit (a serial next submit would hold every result
    // back by the wait). "unset" leaves the variable absent (off).
    {
        const std::string off = "      infer_min_interval_ms: \"0\"";
        const auto setting_of = [&](const std::string& text) {
            for (const auto& s : load_service_config(env.stage(text)).runtime)
                if (s.key == "infer_min_interval_ms") return s;
            return RuntimeSetting{};
        };
        const auto zero = setting_of(base);
        CHECK(zero.set && zero.value == "0" && zero.env == "PERCEPTION_INFER_MIN_INTERVAL_MS");
        const auto unset = setting_of(replace_once(base, off, "      infer_min_interval_ms: unset"));
        CHECK(unset.key == "infer_min_interval_ms" && !unset.set && unset.env == "PERCEPTION_INFER_MIN_INTERVAL_MS");
        for (const char* value : {"45", "1", "49"}) {
            const auto capped = setting_of(replace_once(base, off, std::string("      infer_min_interval_ms: \"") + value + "\""));
            CHECK(capped.set && capped.value == value && capped.env == "PERCEPTION_INFER_MIN_INTERVAL_MS");
        }
        // Passed on as configured: "45" and "0" are set, unset removes the variable.
        apply_runtime_environment(load_service_config(env.stage(replace_once(base, off, "      infer_min_interval_ms: \"45\""))));
        const char* passed = std::getenv("PERCEPTION_INFER_MIN_INTERVAL_MS");
        CHECK(passed && std::string(passed) == "45");
        apply_runtime_environment(load_service_config(env.stage(base)));
        passed = std::getenv("PERCEPTION_INFER_MIN_INTERVAL_MS");
        CHECK(passed && std::string(passed) == "0");
        apply_runtime_environment(load_service_config(env.stage(replace_once(base, off, "      infer_min_interval_ms: unset"))));
        CHECK(std::getenv("PERCEPTION_INFER_MIN_INTERVAL_MS") == nullptr);
        for (const char* bad : {"50", "100", "101", "-1", "4.5", "abc", "045", "00", "1000", "45ms", " 45", "+45", "0x2d",
                                "4e1"}) {
            expect_reject("minimum submit interval value", [&] {
                load_service_config(env.stage(replace_once(base, off, std::string("      infer_min_interval_ms: \"") + bad + "\"")));
            }, "RUNTIME", "infer_min_interval_ms");
        }
        expect_reject("missing minimum submit interval", [&] {
            load_service_config(env.stage(replace_once(base, off + "\n", "")));
        }, "RUNTIME", "infer_min_interval_ms");
        const auto paced = replace_once(base, off, "      infer_min_interval_ms: \"45\"");
        expect_reject("minimum submit interval on the serial loop", [&] {
            load_service_config(env.stage(replace_once(paced, "      two_slot_jit: \"1\"", "      two_slot_jit: \"0\"")));
        }, "RUNTIME", "infer_min_interval_ms", "two_slot_jit");
        expect_reject("minimum submit interval with the loop unset", [&] {
            load_service_config(env.stage(replace_once(paced, "      two_slot_jit: \"1\"", "      two_slot_jit: unset")));
        }, "RUNTIME", "infer_min_interval_ms", "two_slot_jit");
        expect_reject("minimum submit interval with a serial next submit", [&] {
            load_service_config(env.stage(replace_once(paced, "      jit_async_submit: \"1\"", "      jit_async_submit: \"0\"")));
        }, "RUNTIME", "infer_min_interval_ms", "jit_async_submit");
        expect_reject("minimum submit interval with async submit unset", [&] {
            load_service_config(env.stage(replace_once(paced, "      jit_async_submit: \"1\"", "      jit_async_submit: unset")));
        }, "RUNTIME", "infer_min_interval_ms", "jit_async_submit");
        // "0" or unset is accepted on either loop and with either submit.
        const auto serial = replace_once(base, "      two_slot_jit: \"1\"", "      two_slot_jit: \"0\"");
        const auto serial_zero = setting_of(serial);
        CHECK(serial_zero.set && serial_zero.value == "0");
        CHECK(!setting_of(replace_once(serial, off, "      infer_min_interval_ms: unset")).set);
        const auto sync_submit = replace_once(base, "      jit_async_submit: \"1\"", "      jit_async_submit: \"0\"");
        const auto sync_zero = setting_of(sync_submit);
        CHECK(sync_zero.set && sync_zero.value == "0");
        CHECK(!setting_of(replace_once(sync_submit, off, "      infer_min_interval_ms: unset")).set);
    }
    // Placement of the camera SDK's stream threads (2026-10-13): "off" or exactly one of the
    // CPUs perception may ever use (2, 3, the lent 7, the borrow core 1); mandatory like every
    // switch. Cores 0, 4, 5 and 6 cannot be spelled.
    {
        const std::string off = "      capture_sdk_threads_cpu: \"off\"";
        for (const char* cpu : {"1", "2", "3", "7"}) {
            bool seen = false;
            const auto placed = replace_once(base, off, std::string("      capture_sdk_threads_cpu: \"") + cpu + "\"");
            for (const auto& s : load_service_config(env.stage(placed)).runtime)
                if (s.key == "capture_sdk_threads_cpu")
                    seen = s.set && s.value == cpu && s.env == "PERCEPTION_CAPTURE_SDK_THREADS_CPU";
            CHECK(seen);
        }
        bool off_seen = false;
        for (const auto& s : load_service_config(env.stage(base)).runtime)
            if (s.key == "capture_sdk_threads_cpu") off_seen = s.set && s.value == "off";
        CHECK(off_seen);
        for (const char* bad : {"0", "4", "5", "6", "8", "on", "2,3", "2-3", "01"}) {
            expect_reject("SDK thread CPU value", [&] {
                load_service_config(env.stage(replace_once(base, off, std::string("      capture_sdk_threads_cpu: \"") + bad + "\"")));
            }, "RUNTIME", "capture_sdk_threads_cpu");
        }
        expect_reject("missing SDK thread CPU", [&] {
            load_service_config(env.stage(replace_once(base, off + "\n", "")));
        }, "RUNTIME", "capture_sdk_threads_cpu");
    }
    // The optional Profile core (profile_thread_cpu). Absent: no runtime setting and no digest
    // line, so the digests are the ones the configurations had before the switch existed (pinned:
    // computed with the service configuration before the switch). Present: "off" or a core, only
    // with capture_affinity_pin2 "1" and a core for the SDK's threads, never that core or the
    // capture core; the inherited set is checked by profile_thread_plan.
    {
        const std::string sdk_off = "      capture_sdk_threads_cpu: \"off\"";
        const std::string pin2 = "      capture_affinity_pin2: \"1\"";
        const auto with_sdk = [&](const std::string& cpu) {
            return replace_once(base, sdk_off, "      capture_sdk_threads_cpu: \"" + cpu + "\"");
        };
        const auto with_key = [&](const std::string& text, const std::string& line) {
            const auto at = text.find("      capture_sdk_threads_cpu: ");
            const auto end = text.find('\n', at);
            return text.substr(0, end + 1) + line + "\n" + text.substr(end + 1);
        };
        const auto setting = [&](const ServiceConfig& c, bool& found) {
            found = false;
            RuntimeSetting out;
            for (const auto& s : c.runtime)
                if (s.key == "profile_thread_cpu") { found = true; out = s; }
            return out;
        };
        bool found = true;
        const auto absent = load_service_config(env.stage(base));
        (void)setting(absent, found);
        CHECK(!found);
        CHECK(absent.runtime.size() == runtime_switch_keys().size() && absent.runtime.size() == 37);
        CHECK(absent.runtime_digest == "14293d8159eba0ed045841af6c3276e0d960a07c929475f353e9bcaa2afcd830");
        const auto core1 = with_sdk("1");
        const auto absent1 = load_service_config(env.stage(core1));
        CHECK(absent1.runtime_digest == "79ad456bdb3450de7c8ef1900650eecf4e1271c8f871c3a5efca83fbfb5f312f");
        CHECK(profile_thread_cpu(absent1) == -1);
        // Absent: nothing is read or judged, also with an unreadable or unusable set.
        CHECK(!profile_thread_plan(absent1, false, 0).ok);
        CHECK(!profile_thread_plan(absent1, true, 12).ok);
        // The production form: SDK threads on core 1, the Profile thread on core 3.
        const auto production = with_key(core1, "      profile_thread_cpu: \"3\"");
        const auto p = load_service_config(env.stage(production));
        const auto s3 = setting(p, found);
        CHECK(found && s3.set && s3.value == "3" && s3.env.empty());
        CHECK(p.runtime.size() == runtime_switch_keys().size() + 1);
        CHECK(p.runtime_digest != absent1.runtime_digest);
        CHECK(p.runtime_digest == load_service_config(env.stage(production)).runtime_digest);
        CHECK(profile_thread_cpu(p) == 3);
        // Service-applied: no variable is set, and the environment step passes.
        apply_runtime_environment(p);
        CHECK(std::getenv("PERCEPTION_PROFILE_THREAD_CPU") == nullptr);
        // Inherited set {1,2,3,7} (the platform unit) and {2,3,7} (cores 2, 3, 7 with the SDK
        // core outside): Profile {3}, the other threads {2,7}.
        for (const std::uint64_t inherited : {std::uint64_t{0x8E}, std::uint64_t{0x8C}}) {
            const auto plan = profile_thread_plan(p, true, inherited);
            CHECK(plan.ok && plan.profile_mask == 0x08 && plan.others_mask == 0x84 && plan.sdk_cpu == 1 &&
                  plan.capture_cpu == 2 && plan.inherited_mask == inherited);
        }
        const std::pair<std::uint64_t, const char*> refused_sets[] = {
            {0x0C, "set 12 (cores 2,3): others_too_few"}, {0x86, "(cores 1,2,7): profile_cpu_not_inherited"},
            {0x8A, "(cores 1,3,7): capture_cpu_not_left"}, {0x71, "(cores 0,4,5,6): profile_cpu_not_inherited"}};
        for (const auto& refused : refused_sets) {
            const std::uint64_t inherited = refused.first;
            const char* const why = refused.second;
            expect_reject(std::string("Profile core with inherited set ") + std::to_string(inherited),
                          [&] { (void)profile_thread_plan(p, true, inherited); }, "RUNTIME", "profile_thread_cpu", why);
        }
        expect_reject("Profile core with an unreadable set", [&] { (void)profile_thread_plan(p, false, 0x8E); },
                      "RUNTIME", "profile_thread_cpu", "could not be read");
        // "off" is a configured value (one digest line) that places nothing.
        const auto off_cfg = load_service_config(env.stage(with_key(core1, "      profile_thread_cpu: \"off\"")));
        const auto soff = setting(off_cfg, found);
        CHECK(found && soff.set && soff.value == "off");
        CHECK(profile_thread_cpu(off_cfg) == -1 && !profile_thread_plan(off_cfg, false, 0).ok);
        CHECK(off_cfg.runtime_digest != absent1.runtime_digest && off_cfg.runtime_digest != p.runtime_digest);
        // The other spellable cores, each with an SDK core of its own.
        const std::pair<const char*, const char*> cores[] = {{"1", "7"}, {"7", "1"}, {"3", "7"}};
        for (const auto& [cpu, sdk] : cores) {
            const auto c = load_service_config(env.stage(with_key(with_sdk(sdk), std::string("      profile_thread_cpu: \"") + cpu + "\"")));
            CHECK(profile_thread_cpu(c) == std::atoi(cpu));
        }
        for (const char* bad : {"0", "4", "5", "6", "8", "03", "3,7", "2-3", " 3", "on", "unset"}) {
            expect_reject(std::string("Profile core value ") + bad, [&] {
                load_service_config(env.stage(with_key(core1, std::string("      profile_thread_cpu: \"") + bad + "\"")));
            }, "RUNTIME", "profile_thread_cpu", "not allowed");
        }
        expect_reject("Profile core null", [&] {
            load_service_config(env.stage(with_key(core1, "      profile_thread_cpu: null")));
        }, "RUNTIME", "profile_thread_cpu", "null");
        for (const char* mode : {"\"0\"", "unset"}) {
            expect_reject(std::string("Profile core without the capture pin: ") + mode, [&] {
                load_service_config(env.stage(replace_once(production, pin2, std::string("      capture_affinity_pin2: ") + mode)));
            }, "RUNTIME", "profile_thread_cpu", "capture_affinity_pin2");
        }
        for (const char* sdk : {"\"off\"", "unset"}) {
            expect_reject(std::string("Profile core without an SDK core: ") + sdk, [&] {
                load_service_config(env.stage(with_key(replace_once(base, sdk_off, std::string("      capture_sdk_threads_cpu: ") + sdk),
                                                       "      profile_thread_cpu: \"3\"")));
            }, "RUNTIME", "profile_thread_cpu", "needs a core for capture_sdk_threads_cpu");
        }
        for (const char* cpu : {"1", "3", "7"}) {
            expect_reject(std::string("Profile core on the SDK core ") + cpu, [&] {
                load_service_config(env.stage(with_key(with_sdk(cpu), std::string("      profile_thread_cpu: \"") + cpu + "\"")));
            }, "RUNTIME", "profile_thread_cpu", "is the core of capture_sdk_threads_cpu");
        }
        expect_reject("Profile core on the capture core", [&] {
            load_service_config(env.stage(with_key(core1, "      profile_thread_cpu: \"2\"")));
        }, "RUNTIME", "profile_thread_cpu", "core of the capture_affinity_pin2 pin");
        // The SDK's threads on the capture core could not be told from the capture thread's children.
        expect_reject("Profile core with the SDK core on the capture core", [&] {
            load_service_config(env.stage(with_key(with_sdk("2"), "      profile_thread_cpu: \"3\"")));
        }, "RUNTIME", "profile_thread_cpu", "another core than the capture_affinity_pin2 pin");
#ifdef __linux__
        // Reading the configuration does not judge this process's CPU set (--check-config and the
        // seal tool may run on other cores than the service): narrowed to one core that is none of
        // 1, 2, 3, 7 when there is one, else to its lowest core, the production form still loads.
        const auto own = perception::read_capture_affinity();
        CHECK(own.valid && own.mask != 0);
        const std::uint64_t outside = own.mask & ~std::uint64_t{0x8E};   // none of 1, 2, 3, 7
        const std::uint64_t pool = outside ? outside : own.mask;
        const std::uint64_t one = pool & (~pool + 1);                     // its lowest core
        CHECK(perception::set_capture_affinity(one) == 0 && perception::read_capture_affinity().mask == one);
        bool loaded_narrowed = false;
        try {
            loaded_narrowed = load_service_config(env.stage(production)).runtime_digest == p.runtime_digest;
        } catch (const StartupCheckError&) {
        }
        CHECK(perception::set_capture_affinity(own.mask) == 0 && perception::read_capture_affinity().mask == own.mask);
        CHECK(loaded_narrowed);
#endif
    }
    // The optional coverage tickets (coverage_tickets, 2026-10-27). Absent: no setting, no digest
    // line, and the variable is unset (the grid keeps the static prior tickets). Present: "prior" or
    // "fitted", exported to PERCEPTION_COVERAGE_TICKETS for build_profile_geometry.
    {
        const auto with_tickets = [&](const std::string& value) {
            const auto at = base.find("      capture_sdk_threads_cpu: ");
            const auto end = base.find('\n', at);
            return base.substr(0, end + 1) + "      coverage_tickets: " + value + "\n" + base.substr(end + 1);
        };
        const auto absent = load_service_config(env.stage(base));
        for (const auto& s : absent.runtime) CHECK(s.key != "coverage_tickets");
        CHECK(absent.runtime_digest == "14293d8159eba0ed045841af6c3276e0d960a07c929475f353e9bcaa2afcd830");
        std::string previous_digest;
        for (const char* value : {"prior", "fitted"}) {
            const auto c = load_service_config(env.stage(with_tickets(std::string("\"") + value + "\"")));
            bool seen = false;
            for (const auto& s : c.runtime)
                if (s.key == "coverage_tickets") seen = s.set && s.value == value && s.env == "PERCEPTION_COVERAGE_TICKETS";
            CHECK(seen && c.runtime.size() == absent.runtime.size() + 1);
            CHECK(c.runtime_digest != absent.runtime_digest && c.runtime_digest != previous_digest);
            previous_digest = c.runtime_digest;
            apply_runtime_environment(c);
            const char* exported = std::getenv("PERCEPTION_COVERAGE_TICKETS");
            CHECK(exported && std::string(exported) == value);
        }
        apply_runtime_environment(absent);
        CHECK(std::getenv("PERCEPTION_COVERAGE_TICKETS") == nullptr);
        for (const char* bad : {"\"on\"", "\"Fitted\"", "\"fitted \"", "\"1\"", "unset"}) {
            expect_reject(std::string("coverage tickets value ") + bad, [&] {
                load_service_config(env.stage(with_tickets(bad)));
            }, "RUNTIME", "coverage_tickets");
        }
    }
    // Calibration of the capture clock (2026-10-14): a whole number of microseconds in
    // 10000..100000 with one spelling per value; mandatory, and "unset" is refused too (no
    // frameset would get a capture time).
    {
        const std::string measured = "      capture_min_path_us: \"30767\"";
        for (const char* value : {"30767", "10000", "100000", "46039"}) {
            bool seen = false;
            const auto text = replace_once(base, measured, std::string("      capture_min_path_us: \"") + value + "\"");
            for (const auto& s : load_service_config(env.stage(text)).runtime)
                if (s.key == "capture_min_path_us")
                    seen = s.set && s.value == value && s.env == "PERCEPTION_CAPTURE_MIN_PATH_US";
            CHECK(seen);
        }
        for (const char* bad : {"0", "9999", "100001", "030767", "30767.0", "30.767", "-30767", "3e4", "23", " 30767", "0x7fff"}) {
            expect_reject("capture clock calibration value", [&] {
                load_service_config(env.stage(replace_once(base, measured, std::string("      capture_min_path_us: \"") + bad + "\"")));
            }, "RUNTIME", "capture_min_path_us");
        }
        expect_reject("capture clock calibration unset", [&] {
            load_service_config(env.stage(replace_once(base, measured, "      capture_min_path_us: unset")));
        }, "RUNTIME", "capture_min_path_us");
        expect_reject("missing capture clock calibration", [&] {
            load_service_config(env.stage(replace_once(base, measured + "\n", "")));
        }, "RUNTIME", "capture_min_path_us");
    }
    expect_reject("lead out of range", [&] {
        load_service_config(env.stage(replace_once(base, "      dla_active_lead_us: \"2000\"", "      dla_active_lead_us: \"60000\"")));
    }, "RUNTIME", "dla_active_lead_us");
    // Retired with the safe traversable mask (level-3 trim): a runtime section
    // written before the trim still carries these keys at their old places and
    // their old production values; it is refused, not silently accepted.
    CHECK(std::find(runtime_switch_keys().begin(), runtime_switch_keys().end(), "safe_mask_seed") == runtime_switch_keys().end());
    CHECK(std::find(runtime_switch_keys().begin(), runtime_switch_keys().end(), "safe_mask_build") == runtime_switch_keys().end());
    expect_reject("retired switch safe_mask_seed", [&] {
        load_service_config(env.stage(replace_once(base, "      depth_scan_fast: \"1\"\n",
                                                "      depth_scan_fast: \"1\"\n      safe_mask_seed: unset\n")));
    }, "RUNTIME", "perception.pipeline.runtime.safe_mask_seed", "unknown or non-production switch");
    expect_reject("retired switch safe_mask_build", [&] {
        load_service_config(env.stage(replace_once(base, "      jit_second_chance: \"0\"\n",
                                                "      jit_second_chance: \"0\"\n      safe_mask_build: \"0\"\n")));
    }, "RUNTIME", "perception.pipeline.runtime.safe_mask_build", "unknown or non-production switch");

    // ---- environment ----
    {
        char a[] = "PATH=/usr/bin";
        char b[] = "PERCEPTION_JIT_STALL_TRACE=1";
        char* bad[] = {a, b, nullptr};
        expect_reject("inherited switch", [&] { reject_inherited_runtime_environment(bad); }, "ENV", "PERCEPTION_JIT_STALL_TRACE");
        char c[] = "RNS_NATIVE_CAMERA_SERIAL=X";
        char* bad2[] = {a, c, nullptr};
        expect_reject("inherited test-shell variable", [&] { reject_inherited_runtime_environment(bad2); }, "ENV", "RNS_NATIVE_CAMERA_SERIAL");
        char* good[] = {a, nullptr};
        reject_inherited_runtime_environment(good);
        ++checks;
        const auto cfg = load_service_config(env.stage(base));
        ::setenv("PERCEPTION_COCO_MASK_RESIZE", "full", 1);
        ::setenv("PERCEPTION_MEDIA_NICE", "10", 1);
        apply_runtime_environment(cfg);
        CHECK(std::getenv("PERCEPTION_COCO_MASK_RESIZE") == nullptr);
        CHECK(std::getenv("PERCEPTION_MEDIA_NICE") == nullptr);
        CHECK(std::string(std::getenv("PERCEPTION_JIT_BIND_DELAY_MS")) == "10.6921890700000084");
        CHECK(std::string(std::getenv("PERCEPTION_CAPTURE_AFFINITY_PIN2")) == "1");
    }

    // ---- PSC-5 registry ----
    {
        const auto keys = declared_key_registry("xbrain-01");
        CHECK(keys.size() == 6);
        // A19-BOOT-4 / PSC-5: the registry is exactly these six keys, in this order: the three RT keys
        // of 11 section 2.2.1 (robot-id prefix) and the three bare GEN event keys of 11 section 1.1.6 PC-2.
        const std::vector<std::string> expected{
            "xbrain/xbrain-01/rt/perception/profile", "xbrain/xbrain-01/rt/perception/objects",
            "xbrain/xbrain-01/rt/perception/status", "event/info/perception",
            "event/warn/perception", "event/fault/perception"};
        CHECK(keys == expected);
        for (const auto& k : keys) CHECK(k.find('*') == std::string::npos);
        expect_reject("registry invalid rid", [&] { declared_key_registry("X/Y"); }, "PSC-5", "X/Y");

        // A19-BOOT-4 / PSC-5 at startup: what the publishers are given is the registered set,
        // literally. The service calls require_declared_keys(rid, publisher_keys(rid).all()).
        const auto pub = publisher_keys("xbrain-01");
        CHECK(pub.profile == expected[0] && pub.objects == expected[1] && pub.status == expected[2]);
        CHECK(pub.info == expected[3] && pub.warn == expected[4] && pub.fault == expected[5]);
        CHECK(pub.all() == expected);
        require_declared_keys("xbrain-01", pub.all());
        require_declared_keys("xbrain-01", {expected[5], expected[0], expected[3], expected[1], expected[4], expected[2]});
        ++checks;
        expect_reject("publisher keys invalid rid", [&] { publisher_keys("X/Y"); }, "PSC-5", "X/Y");
        {
            const auto other = publisher_keys("dev");
            CHECK(other.profile == "xbrain/dev/rt/perception/profile" && other.status == "xbrain/dev/rt/perception/status");
            CHECK(other.info == pub.info && other.warn == pub.warn && other.fault == pub.fault);
        }
        expect_reject("declared keys invalid rid", [&] { require_declared_keys("X/Y", pub.all()); }, "PSC-5", "X/Y");
        // One key that is not registered: next to the six, in place of one, with a registered
        // prefix (the mutant of the contract: compared by prefix), under another robot id.
        for (const std::string& extra : {std::string("xbrain/xbrain-01/rt/perception/targets"),
                                         std::string("xbrain/xbrain-01/rt/perception/profile/extra"),
                                         std::string("xbrain/xbrain-01/rt/perception/profiles"),
                                         std::string("xbrain/xbrain-01/rt/perception"),
                                         std::string("xbrain/xbrain-02/rt/perception/profile"),
                                         std::string("xbrain/xbrain-01/event/info/perception")}) {
            auto added = pub.all();
            added.push_back(extra);
            expect_reject("unregistered key added: " + extra, [&] { require_declared_keys("xbrain-01", added); }, "PSC-5", extra, "not registered");
            auto replaced = pub.all();
            replaced[0] = extra;
            expect_reject("unregistered key in place: " + extra, [&] { require_declared_keys("xbrain-01", replaced); }, "PSC-5", extra, "not registered");
        }
        expect_reject("wildcard key", [&] { require_declared_keys("xbrain-01", {"xbrain/xbrain-01/rt/perception/*"}); }, "PSC-5", "perception/*", "wildcard");
        {
            auto twice = pub.all();
            twice.push_back(pub.status);
            expect_reject("key declared twice", [&] { require_declared_keys("xbrain-01", twice); }, "PSC-5", pub.status, "twice");
            auto missing = pub.all();
            missing.pop_back();
            expect_reject("registered key not declared", [&] { require_declared_keys("xbrain-01", missing); }, "PSC-5", pub.fault, "not declared");
            expect_reject("nothing declared", [&] { require_declared_keys("xbrain-01", {}); }, "PSC-5", pub.profile, "not declared");
        }
        // The keys of one robot are not the keys of another.
        expect_reject("keys of another robot", [&] { require_declared_keys("xbrain-02", pub.all()); }, "PSC-5", pub.profile, "not registered");
    }

    std::cout << "{\"result\":\"" << (failures ? "FAIL" : "PASS") << "\",\"checks\":" << checks
              << ",\"failures\":" << failures << ",\"rejections\":" << rejections
              << ",\"scope\":\"service_config_host\",\"camera\":false,\"sdk\":false,\"production_acceptance\":false}\n";
    return failures ? 1 : 0;
}
