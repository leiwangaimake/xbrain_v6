/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: service_config.hpp
 * Brief: Formal R6 perception service startup configuration and self-checks.
 *
 * Description:
 * Reads ONLY the resolved product (19 section 8.1) and its MANIFEST. Contract
 * keys (19 section 8.2) are read with the pinned upstream yaml_lite through the
 * bounded local vector adapter; the implementation section perception.pipeline
 * (legacy model/tracking/depth/velocity keys plus explicit runtime switches)
 * is handed to the existing yolo::Config. No key has a code default. Every
 * refusal names the self-check and the dotted key path.
 *
 * Working rules (reply 2026-09-17): rid comes from
 * MANIFEST.robot_id; MANIFEST.boot_id must equal this boot; the perception
 * entry must bind this file by path and SHA-256; only fallback-branch keys may
 * be null; extrinsic transform all-null is a legal no-geometry mode, partial is
 * refused; runtime switches are explicit config, never inherited environment.
 */

#pragma once

#include "common/profile_thread_placement.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace perception::service {

// The resolved product directory the platform config freeze writes (19 section 8.1; the freeze
// output root RESOLVED_ROOT_DEFAULT): the service reads its configuration and MANIFEST only from
// here, and the boot id from the kernel.
inline constexpr const char* kResolvedPrefix = "/opt/xbrain_v6/data/run/resolved/";
inline constexpr const char* kResolvedConfigPath = "/opt/xbrain_v6/data/run/resolved/perception.yaml";
inline constexpr const char* kManifestPath = "/opt/xbrain_v6/data/run/resolved/MANIFEST.json";
inline constexpr const char* kBootIdPath = "/proc/sys/kernel/random/boot_id";

// check: "PSC-1".."PSC-7", "MANIFEST", "RUNTIME", "ENV". key: dotted path/file.
class StartupCheckError : public std::runtime_error {
public:
    StartupCheckError(std::string check, std::string key, const std::string& why);
    const std::string& check() const noexcept { return check_; }
    const std::string& key() const noexcept { return key_; }
private:
    std::string check_, key_;
};

// The running host as the MANIFEST must describe it: a MANIFEST from another boot is stale.
struct HostIdentity {
    std::string boot_id;  // full kernel boot_id, lowercase UUID text
};
HostIdentity read_host_identity(const std::string& boot_id_path = kBootIdPath);
// First 8 hex digits of boot_id (11 section 3.0). Throws on malformed boot_id.
std::string envelope_boot(const HostIdentity&);

// What the MANIFEST says about this product: robot id (the envelope rid), boot id, config
// revision and common digest (reported in config_digest), and the path and SHA-256 under which
// it binds perception.yaml (must match the file actually read).
struct ManifestIdentity {
    std::string robot_id, boot_id, config_rev, common_digest;
    std::string perception_path, perception_sha256;
};

// perception.extrinsic_base_cam: either entirely null (Absent: the unconfigured Profile, no
// geometry) or a complete transform; a partial one is refused at startup. The record reference
// (calib_record, calib_sha256, residual_max_m) is optional as a whole and certifies the transform.
enum class ExtrinsicMode { Absent, TransformConfigured };
struct ExtrinsicConfig {
    ExtrinsicMode mode = ExtrinsicMode::Absent;
    std::optional<std::array<double, 3>> translation;
    std::optional<std::array<double, 4>> rotation_xyzw;
    std::optional<std::string> calib_record, calib_sha256;
    std::optional<double> residual_max_m;
};

// perception.profile (19 section 8.2): bin layout (angles, count, range, cell size), the ground
// classification and coverage thresholds, the T and S parameters (seg_max_age_ms, tau_T,
// v_ego_max_mps, omega_max_rps, m_jitter_m) and the ground-fit gates. All are required.
struct ProfileConfig {
    double angle_min_rad = 0, angle_step_rad = 0;
    std::size_t n_bins = 0;
    double range_max_m = 0, dr_m = 0, blind_near_lo_m = 0, h_tol_m = 0, z_pass_m = 0;
    double cover_min = 0, omega_max_rps = 0;
    std::int64_t seg_max_age_ms = 0;
    double plane_fit_min_inlier_frac = 0, plane_fit_resid_max_m = 0;
    // Fallback-branch keys: null closes only the fallback branch (r6 reply L60).
    std::optional<double> dfree_cap_fallback_m, fallback_tilt_tol_deg,
        fallback_tilt_err_max_deg, fallback_align_err_m;
    double tau_T = 0, slope_max_deg = 0, sigma_max_m = 0, eps_neg = 0, w_h_m = 0;
    double v_ego_max_mps = 0, m_jitter_m = 0;
};

// perception.med2: the local RTSP preview stream (MED-2), its raster, rate and encoder settings.
struct Med2Config {
    int rtsp_port = 0;
    std::optional<int> pub_port;  // null: external forwarding not configured
    int width = 0, height = 0, fps = 0, bitrate_kbps = 0, gop = 0;
};

// Every contract section the service itself interprets (as opposed to perception.pipeline,
// which goes to the legacy yolo::Config): Profile, Status depth-quality and window, latency
// fallback, extrinsic, MED-2, Objects staleness, feature flags and the two zenoh endpoints.
struct ContractConfig {
    ProfileConfig profile;
    double depth_quality_fair = 0, depth_quality_poor = 0;
    double status_window_s = 0;
    std::int64_t latency_fallback_ms = 0;
    ExtrinsicConfig extrinsic;
    Med2Config med2;
    std::int64_t objects_stale_ms = 0;
    bool pointcloud_enable = false, legacy_keys_enable = false;
    std::string rt_endpoint, gen_endpoint;
};

// One production runtime switch. value empty <=> configured "unset" (the
// environment variable is left absent, which differs from "0" for some).
// env empty <=> applied directly by the service (process_opencv_threads).
struct RuntimeSetting {
    std::string key, env, value;
    bool set = false;
};

// ROS executor (19 section 1.1 item 5): TF listener + capture command only;
// perception data never goes to ROS topics (P19-4). Source: perception.pipeline.ros.
struct RosConfig {
    bool enable = false, tf_enable = false;
    std::string odom_frame, base_frame;
};

// The complete, validated startup configuration. Paths and SHA-256 of both input files are kept
// for config_digest and the startup log; rid and boot8 form the envelope identity.
struct ServiceConfig {
    std::string resolved_path, resolved_sha256, manifest_path, manifest_sha256;
    ManifestIdentity manifest;
    std::string rid, boot8;
    ContractConfig contract;
    // Adapted contract text (pipeline removed) for the existing geometry parser.
    std::string contract_yaml;
    // yolo::Config document: perception.pipeline minus runtime/media, plus the
    // hevc block synthesized from perception.med2 and pipeline.media.
    YAML::Node pipeline;
    std::string media_codec, media_path;
    RosConfig ros;
    std::vector<RuntimeSetting> runtime;
    std::string runtime_digest;  // sha256 of the canonical "ENV=value|unset" lines
};

// Where to read from. The service always uses the defaults; tests point them at staged copies.
struct LoadInputs {
    std::string resolved_path = kResolvedConfigPath;
    std::string manifest_path = kManifestPath;
    HostIdentity host;
    // Host unit tests only: canonical directory standing in for kResolvedPrefix.
    // The service entry never changes it and asserts so.
    std::string resolved_prefix = kResolvedPrefix;
};

// Full startup configuration check (PSC-1, PSC-2, PSC-6, MANIFEST, runtime
// table). Throws StartupCheckError. Reads the two files once each.
ServiceConfig load_service_config(const LoadInputs&);

// Names of every runtime switch the formal service may set, in table order.
const std::vector<std::string>& runtime_switch_keys();
// Refuse any inherited PERCEPTION_* / RNS_* variable (they would silently change
// the verified combination). envp is the process environment.
void reject_inherited_runtime_environment(char** envp);
// setenv/unsetenv each production switch and verify by readback. Call before
// the logger or any pipeline module is constructed.
void apply_runtime_environment(const ServiceConfig&);
// process_opencv_threads of the runtime table (0 = not set, library default).
// Applied by the service right after the runtime environment; every tool that
// must reproduce the service's inference path (PSC-4 seal) applies it the same way.
int process_open_cv_threads(const ServiceConfig&);
// profile_thread_cpu of the runtime table: the core, or -1 when absent or "off".
int profile_thread_cpu(const ServiceConfig&);
// The masks of the Profile thread placement (profile_thread_placement.hpp) from the service's
// inherited CPU set, read on the main thread before anything is narrowed. Called when the service
// starts, after --check-config would have returned: that check may run on other cores. Not ok
// (and nothing judged) when the switch is absent or "off"; otherwise refuses the start
// (StartupCheckError "RUNTIME") when the set is unreadable or the plan's rules fail.
ProfileThreadPlan profile_thread_plan(const ServiceConfig&, bool inherited_valid, std::uint64_t inherited_mask);

// PSC-5: the exact contract keys this process may declare (no wildcard).
std::vector<std::string> declared_key_registry(const std::string& rid);

// The key each of the service's six publishers is given (perception_service_main.cpp hands
// exactly these to the transports). Written out per channel, apart from the registry above,
// so that the startup check compares what is declared with what is registered.
struct PublisherKeys {
    std::string profile, objects, status, info, warn, fault;
    std::vector<std::string> all() const { return {profile, objects, status, info, warn, fault}; }
};
PublisherKeys publisher_keys(const std::string& rid);

// PSC-5 / A19-BOOT-4: refuses the start (StartupCheckError "PSC-5", the offending key) unless
// `declared` is the registered key set of this robot, literally (declared_keys.hpp).
void require_declared_keys(const std::string& rid, const std::vector<std::string>& declared);

}  // namespace perception::service
