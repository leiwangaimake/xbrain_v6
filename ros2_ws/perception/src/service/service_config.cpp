/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: service_config.cpp
 * Brief: Formal R6 perception service startup configuration and self-checks.
 *
 * Description:
 * What problem this solves. Implements service_config.hpp: read the resolved configuration and
 * its MANIFEST exactly once, check every key, and hand three things to the service: the typed
 * contract (Profile/Objects/Status/extrinsic/MED-2/Zenoh), the yolo::Config document for the
 * existing pipeline, and the table of production runtime switches.
 *
 * Why so strict. A wrong key that silently falls back to a default would change what RNS
 * receives without any visible sign. So: exact key sets, no code defaults, null only where the
 * contract allows it, and every refusal names the self-check (PSC-n, MANIFEST, RUNTIME, ENV)
 * and the dotted key path, which is what the operator sees in the exit message.
 */
#include "service/service_config.hpp"

#include "common/capture_affinity_control.hpp"
#include "common/capture_time_state.hpp"
#include "common/config.hpp"
#include "common/jit_schedule.hpp"
#include "common/resolved_yaml.hpp"
#include "service/declared_keys.hpp"

#include <xbrain/config/yaml_lite.h>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <climits>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <sys/stat.h>
#include <utility>

namespace perception::service {
namespace {
using xbrain::config::YamlNode;
// Setup files are small; the bound keeps a wrong path from being slurped.
constexpr std::size_t kMaxFileBytes = 1024U * 1024U;
constexpr double kPi = 3.1415926535897932384626433832795;

[[noreturn]] void fail(const std::string& check, const std::string& key, const std::string& why) {
    throw StartupCheckError(check, key, why);
}

// Read a regular file once, bounded; errors carry the calling self-check name.
std::string read_bounded(const std::string& path, const char* check) {
    struct stat st {};
    if (::stat(path.c_str(), &st) != 0) fail(check, path, "file does not exist");
    if (!S_ISREG(st.st_mode)) fail(check, path, "not a regular file");
    if (st.st_size < 0 || static_cast<std::uint64_t>(st.st_size) > kMaxFileBytes)
        fail(check, path, "file larger than 1 MiB setup bound");
    std::ifstream in(path, std::ios::binary);
    if (!in) fail(check, path, "cannot open file");
    std::ostringstream buffer;
    buffer << in.rdbuf();
    if (!in.good() && !in.eof()) fail(check, path, "read failed");
    return buffer.str();
}

std::string trim(const std::string& s) { return xbrain::config::detail::trim(s); }

// Exactly `size` lowercase hex digits (SHA-256 values, boot id): one accepted spelling only, so
// two texts of the same value can never both pass a string comparison.
bool lower_hex(const std::string& s, std::size_t size) {
    return s.size() == size && std::all_of(s.begin(), s.end(), [](unsigned char c) {
        return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
    });
}

// Robot id as it appears in every key: [a-z0-9_-]{1,32}.
bool valid_rid(const std::string& rid) {
    return !rid.empty() && rid.size() <= 32 && std::all_of(rid.begin(), rid.end(), [](unsigned char c) {
        return (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_' || c == '-';
    });
}

// ---- resolved text split: perception.pipeline is the implementation section ----
struct Split { std::string contract, pipeline; };

// The resolved file holds contract keys (read by the pinned yaml_lite) and the
// implementation section perception.pipeline (a yaml-cpp document for the
// existing yolo::Config, with structures yaml_lite cannot read). Split the text
// by indentation: every line under "  pipeline:" goes, with its 4-space prefix
// removed, to the pipeline document; everything else stays contract text.
Split split_pipeline(const std::string& text) {
    std::istringstream input(text);
    std::string line;
    std::string top;
    bool in_pipeline = false, seen = false;
    std::ostringstream contract, pipeline;
    // Line by line: once "  pipeline:" is seen under "perception:", every following line indented
    // deeper than 2 (or blank) belongs to it; the first line back at indent <= 2 ends it. Children
    // must be indented by at least 4 so that removing the 4-space prefix keeps valid YAML.
    while (std::getline(input, line)) {
        const std::string content = xbrain::config::detail::strip_comment(line);
        const std::string t = trim(content);
        const int indent = xbrain::config::detail::count_indent(content);
        if (in_pipeline) {
            if (t.empty() || indent > 2) {
                if (t.empty()) { pipeline << "\n"; continue; }
                if (indent < 4) fail("PSC-2", "perception.pipeline", "child indentation must be at least 4 spaces");
                pipeline << line.substr(4) << "\n";
                continue;
            }
            in_pipeline = false;
        }
        if (!t.empty() && indent == 0) top = t.substr(0, t.find(':'));
        if (!t.empty() && indent == 2 && top == "perception" && t.rfind("pipeline:", 0) == 0) {
            if (trim(t.substr(std::strlen("pipeline:"))) != "")
                fail("PSC-2", "perception.pipeline", "must be a map");
            if (seen) fail("PSC-2", "perception.pipeline", "duplicate key");
            seen = in_pipeline = true;
            continue;
        }
        contract << line << "\n";
    }
    if (!seen) fail("PSC-2", "perception.pipeline", "missing implementation section");
    return {contract.str(), pipeline.str()};
}

// ---- contract readers (yaml_lite) ----
// Each reader fails on a missing or null key: no key has a code default. Only
// the optional* readers accept null, for the keys the contract lets be null.
const YamlNode& node(const YamlNode& root, const std::string& path) {
    try {
        return root.at(path);
    } catch (const std::exception&) {
        fail("PSC-2", path, "missing key");
    }
}

// Exactly these keys: an unknown (e.g. misspelled) key is refused, not ignored.
void exact_keys(const YamlNode& root, const std::string& path, const std::set<std::string>& keys) {
    const auto& n = node(root, path);
    if (!n.is_map()) fail("PSC-2", path, "must be a map");
    for (const auto& item : n.items())
        if (!keys.count(item.first)) fail("PSC-2", path + "." + item.first, "unknown key");
    for (const auto& key : keys)
        if (!n.items().count(key)) fail("PSC-2", path + "." + key, "missing key");
}

bool is_null(const YamlNode& root, const std::string& path) {
    const auto& n = node(root, path);
    if (n.is_map()) fail("PSC-2", path, "must be a scalar");
    return n.is_null();
}

// A required finite number; NaN or inf would pass every later range comparison unnoticed.
double number(const YamlNode& root, const std::string& path) {
    if (is_null(root, path)) fail("PSC-2", path, "null (not configured)");
    double v = 0;
    try { v = root.require_double(path); }
    catch (const std::exception&) { fail("PSC-2", path, "not a number"); }
    if (!std::isfinite(v)) fail("PSC-2", path, "must be finite");
    return v;
}

std::optional<double> optional_number(const YamlNode& root, const std::string& path) {
    if (is_null(root, path)) return std::nullopt;
    return number(root, path);
}

std::int64_t integer(const YamlNode& root, const std::string& path, std::int64_t lo, std::int64_t hi) {
    if (is_null(root, path)) fail("PSC-2", path, "null (not configured)");
    long v = 0;
    try { v = root.require_int(path); }
    catch (const std::exception&) { fail("PSC-2", path, "not an integer"); }
    if (v < lo || v > hi) fail("PSC-2", path, "integer outside [" + std::to_string(lo) + "," + std::to_string(hi) + "]");
    return v;
}

// Only the literal words true/false (no yes/on/1 variants).
bool boolean(const YamlNode& root, const std::string& path) {
    if (is_null(root, path)) fail("PSC-2", path, "null (not configured)");
    const auto s = root.require_string(path);
    if (s == "true") return true;
    if (s == "false") return false;
    fail("PSC-2", path, "must be exactly true or false");
}

std::string string_value(const YamlNode& root, const std::string& path) {
    if (is_null(root, path)) fail("PSC-2", path, "null (not configured)");
    auto s = root.require_string(path);
    if (s.empty()) fail("PSC-2", path, "empty string");
    return s;
}

// [lo, hi], or (lo, hi] with lo_open.
void range(double v, const std::string& path, double lo, double hi, bool lo_open = false) {
    if (v < lo || v > hi || (lo_open && v == lo))
        fail("PSC-2", path, "value outside allowed numeric domain");
}

// "[a, b, ...]" of N finite numbers. All items null = not configured (legal for
// the no-geometry mode); a partly null vector is refused.
template <std::size_t N>
std::optional<std::array<double, N>> vector(const YamlNode& root, const std::string& path) {
    if (is_null(root, path)) return std::nullopt;
    const auto s = root.require_string(path);
    if (s.size() < 2 || s.front() != '[' || s.back() != ']') fail("PSC-2", path, "expected [..] vector");
    std::array<std::optional<double>, N> items;
    std::size_t begin = 1, count = 0, nulls = 0;
    while (begin <= s.size() - 1) {
        auto end = s.find(',', begin);
        if (end == std::string::npos) end = s.size() - 1;
        const auto item = trim(s.substr(begin, end - begin));
        if (count == N) fail("PSC-2", path, "wrong vector length");
        if (item == "null" || item == "~") {
            ++nulls;
        } else {
            std::size_t used = 0;
            double v = 0;
            try { v = std::stod(item, &used); }
            catch (const std::exception&) { fail("PSC-2", path, "vector item not numeric"); }
            if (used != item.size() || !std::isfinite(v)) fail("PSC-2", path, "vector item must be one finite number");
            items[count] = v;
        }
        ++count;
        begin = end + 1;
    }
    if (count != N) fail("PSC-2", path, "wrong vector length");
    if (nulls == N) return std::nullopt;
    if (nulls != 0) fail("PSC-2", path, "partially null vector");
    std::array<double, N> out{};
    for (std::size_t i = 0; i < N; ++i) out[i] = *items[i];
    return out;
}

// The service connects only to routers on this host: tcp/127.0.0.1:<1..65535>.
bool local_tcp_endpoint(const std::string& value) {
    // LINT-ALLOW(endpoint): loopback VALIDATOR for a configured value, not a hardcoded endpoint.
    const std::string prefix = "tcp/127.0.0.1:";
    if (value.rfind(prefix, 0) != 0) return false;
    const auto port = value.substr(prefix.size());
    if (port.empty() || port.size() > 5 || port[0] == '0' ||
        !std::all_of(port.begin(), port.end(), [](unsigned char c) { return std::isdigit(c); }))
        return false;
    const long p = std::strtol(port.c_str(), nullptr, 10);
    return p >= 1 && p <= 65535;
}

ProfileConfig read_profile(const YamlNode& root) {
    const std::string p = "perception.profile.";
    exact_keys(root, "perception.profile", {
        "angle_min_rad", "angle_step_rad", "n_bins", "range_max_m", "dr_m", "blind_near_lo_m",
        "h_tol_m", "z_pass_m", "cover_min", "omega_max_rps", "seg_max_age_ms",
        "dfree_cap_fallback_m", "plane_fit_min_inlier_frac", "plane_fit_resid_max_m",
        "fallback_tilt_tol_deg", "fallback_tilt_err_max_deg", "fallback_align_err_m",
        "tau_T", "slope_max_deg", "sigma_max_m", "eps_neg", "w_h_m", "v_ego_max_mps", "m_jitter_m"});
    ProfileConfig c;
    // Bin domain (11 section 3.1B.1 angle convention): n_bins odd and
    // angle_min = -(n - 1) / 2 * step, both checked in the float32 wire domain
    // RNS will reconstruct the bin angles from.
    c.n_bins = static_cast<std::size_t>(integer(root, p + "n_bins", 1, 1023));
    if (c.n_bins % 2 != 1) fail("PSC-2", p + "n_bins", "must be odd (symmetric bin domain)");
    const double raw_min = number(root, p + "angle_min_rad");
    const double raw_step = number(root, p + "angle_step_rad");
    range(raw_step, p + "angle_step_rad", 0, kPi, true);
    range(raw_min, p + "angle_min_rad", -kPi, 0);
    c.angle_step_rad = static_cast<double>(static_cast<float>(raw_step));
    c.angle_min_rad = static_cast<double>(static_cast<float>(raw_min));
    const auto first = static_cast<float>(-0.5 * static_cast<double>(c.n_bins - 1) * c.angle_step_rad);
    if (c.angle_min_rad != static_cast<double>(first))
        fail("PSC-2", p + "angle_min_rad", "must be the symmetric first-bin centre in the float32 wire domain");
    // PSC-6: range within the camera's recommended range (11 section 3.1B.1).
    c.range_max_m = number(root, p + "range_max_m");
    if (!(c.range_max_m > 0.0)) fail("PSC-6", p + "range_max_m", "must be positive");
    if (c.range_max_m > 6.0) fail("PSC-6", p + "range_max_m", "exceeds 6.0 m (Gemini 338Le recommended range)");
    // Cell size, blind range and the height / coverage / rate thresholds: each within the domain
    // the geometry is defined for. A zero where it would divide or make a degenerate grid (dr,
    // tolerances, coverage, tau_T) is excluded with an open lower bound.
    c.dr_m = number(root, p + "dr_m");                      range(c.dr_m, p + "dr_m", 0, c.range_max_m, true);
    c.blind_near_lo_m = number(root, p + "blind_near_lo_m"); range(c.blind_near_lo_m, p + "blind_near_lo_m", 0, c.range_max_m);
    c.h_tol_m = number(root, p + "h_tol_m");                range(c.h_tol_m, p + "h_tol_m", 0, 10, true);
    c.z_pass_m = number(root, p + "z_pass_m");              range(c.z_pass_m, p + "z_pass_m", 0, 100, true);
    c.cover_min = number(root, p + "cover_min");            range(c.cover_min, p + "cover_min", 0, 1, true);
    c.omega_max_rps = number(root, p + "omega_max_rps");    range(c.omega_max_rps, p + "omega_max_rps", 0, 100);
    c.seg_max_age_ms = integer(root, p + "seg_max_age_ms", 1, 60000);
    // Extended self-check (user decision 2026-09-30; like B3-4, not contract text): the
    // frame-level T age limit stays clearly below the 10 s traversable_seg_available window
    // (19 section 5.1), so a false flag always coincides with t_seg null (19 section 3.3,
    // 11 section 3.2.4). The 1 s margin covers the Profile delay between the two clocks.
    if (c.seg_max_age_ms >= 9000)
        fail("PSC-2", p + "seg_max_age_ms", "must be below 9000 ms (10 s segmentation window minus 1 s)");
    // Fallback-branch keys may be null: null closes only that branch (r6 L60).
    c.dfree_cap_fallback_m = optional_number(root, p + "dfree_cap_fallback_m");
    if (c.dfree_cap_fallback_m) range(*c.dfree_cap_fallback_m, p + "dfree_cap_fallback_m", 0, c.range_max_m);
    c.plane_fit_min_inlier_frac = number(root, p + "plane_fit_min_inlier_frac");
    range(c.plane_fit_min_inlier_frac, p + "plane_fit_min_inlier_frac", 0, 1, true);
    c.plane_fit_resid_max_m = number(root, p + "plane_fit_resid_max_m");
    range(c.plane_fit_resid_max_m, p + "plane_fit_resid_max_m", 0, 10);
    for (auto* key : {"fallback_tilt_tol_deg", "fallback_tilt_err_max_deg"}) {
        const auto v = optional_number(root, p + key);
        if (v) range(*v, p + key, 0, 90);
        (std::string(key) == "fallback_tilt_tol_deg" ? c.fallback_tilt_tol_deg : c.fallback_tilt_err_max_deg) = v;
    }
    c.fallback_align_err_m = optional_number(root, p + "fallback_align_err_m");
    if (c.fallback_align_err_m) range(*c.fallback_align_err_m, p + "fallback_align_err_m", 0, 10);
    // T and S parameters (19 section 3.3 / 3.4A): the traversable-fraction threshold, slope and
    // roughness limits, the negative-evidence margin, the height window, and the platform motion
    // bounds that inflate stale evidence.
    c.tau_T = number(root, p + "tau_T");                    range(c.tau_T, p + "tau_T", 0, 1, true);
    c.slope_max_deg = number(root, p + "slope_max_deg");    range(c.slope_max_deg, p + "slope_max_deg", 0, 90);
    c.sigma_max_m = number(root, p + "sigma_max_m");        range(c.sigma_max_m, p + "sigma_max_m", 0, 10);
    c.eps_neg = number(root, p + "eps_neg");                range(c.eps_neg, p + "eps_neg", 0, 1);
    c.w_h_m = number(root, p + "w_h_m");                    range(c.w_h_m, p + "w_h_m", 0, c.range_max_m);
    c.v_ego_max_mps = number(root, p + "v_ego_max_mps");    range(c.v_ego_max_mps, p + "v_ego_max_mps", 0, 100);
    c.m_jitter_m = number(root, p + "m_jitter_m");          range(c.m_jitter_m, p + "m_jitter_m", 0, 10);
    return c;
}

ExtrinsicConfig read_extrinsic(const YamlNode& root) {
    const std::string p = "perception.extrinsic_base_cam.";
    exact_keys(root, "perception.extrinsic_base_cam",
              {"translation", "rotation_xyzw", "calib_record", "calib_sha256", "residual_max_m"});
    ExtrinsicConfig e;
    // Transform: both null (no geometry) or both set; record fields all or none
    // and only with a transform; the record path absolute (Q11).
    e.translation = vector<3>(root, p + "translation");
    e.rotation_xyzw = vector<4>(root, p + "rotation_xyzw");
    if (!is_null(root, p + "calib_record")) e.calib_record = string_value(root, p + "calib_record");
    if (!is_null(root, p + "calib_sha256")) {
        e.calib_sha256 = string_value(root, p + "calib_sha256");
        if (!lower_hex(*e.calib_sha256, 64)) fail("PSC-2", p + "calib_sha256", "expected 64 lowercase hex digits");
    }
    e.residual_max_m = optional_number(root, p + "residual_max_m");
    if (e.residual_max_m) range(*e.residual_max_m, p + "residual_max_m", 0, 10);
    if (e.translation.has_value() != e.rotation_xyzw.has_value())
        fail("PSC-2", "perception.extrinsic_base_cam", "translation and rotation_xyzw must be both null or both set");
    if (!e.translation) {
        if (e.calib_record || e.calib_sha256 || e.residual_max_m)
            fail("PSC-2", "perception.extrinsic_base_cam", "calibration record fields without a transform");
        e.mode = ExtrinsicMode::Absent;
        return e;
    }
    const int record_fields = int(e.calib_record.has_value()) + int(e.calib_sha256.has_value()) + int(e.residual_max_m.has_value());
    if (record_fields != 0 && record_fields != 3)
        fail("PSC-2", "perception.extrinsic_base_cam", "calib_record, calib_sha256 and residual_max_m must be all null or all set");
    if (e.calib_record && (e.calib_record->empty() || e.calib_record->front() != '/'))
        fail("PSC-2", p + "calib_record", "must be an absolute path (reply Q11)");
    // Not normalised here: a silently normalised rotation would no longer equal
    // the one in the calibration record.
    const auto& q = *e.rotation_xyzw;
    const double n2 = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    if (std::abs(n2 - 1.0) > 1e-9) fail("PSC-2", p + "rotation_xyzw", "must be unit length (not auto-normalized)");
    e.mode = ExtrinsicMode::TransformConfigured;
    return e;
}

ContractConfig read_contract(const YamlNode& root) {
    if (!root.is_map()) fail("PSC-2", "<root>", "resolved config must be a map");
    for (const auto& item : root.items())
        if (item.first != "perception") fail("PSC-2", item.first, "unknown top-level key (process config holds perception.* only)");
    const auto& perception = node(root, "perception");
    if (!perception.is_map()) fail("PSC-2", "perception", "must be a map");
    // The process config holds perception.* only, with a fixed section set.
    const std::set<std::string> sections = {"profile", "objects", "status", "capture",
        "extrinsic_base_cam", "med2", "objects_inject", "debug", "zenoh", "native_geometry_setup"};
    for (const auto& item : perception.items())
        if (!sections.count(item.first)) fail("PSC-2", "perception." + item.first, "unknown section");

    ContractConfig c;
    c.profile = read_profile(root);
    // Objects depth-quality bands: fractions of valid depth in the object region (0..1).
    exact_keys(root, "perception.objects", {"depth_quality_fair", "depth_quality_poor"});
    c.depth_quality_fair = number(root, "perception.objects.depth_quality_fair");
    range(c.depth_quality_fair, "perception.objects.depth_quality_fair", 0, 1);
    c.depth_quality_poor = number(root, "perception.objects.depth_quality_poor");
    range(c.depth_quality_poor, "perception.objects.depth_quality_poor", 0, 1);
    exact_keys(root, "perception.status", {"window_s"});
    // The health windows are implemented for exactly 10 s; another value would
    // be accepted but not honoured, so it is refused.
    c.status_window_s = number(root, "perception.status.window_s");
    if (c.status_window_s != 10.0)
        fail("PSC-2", "perception.status.window_s", "implemented health windows are exactly 10 s");
    exact_keys(root, "perception.capture", {"latency_fallback_ms"});
    // Declared link estimate for capture times (19 section 2.4, not measured).
    c.latency_fallback_ms = integer(root, "perception.capture.latency_fallback_ms", 0, 1000);
    c.extrinsic = read_extrinsic(root);
    if (c.extrinsic.mode == ExtrinsicMode::TransformConfigured && !perception.items().count("native_geometry_setup"))
        fail("PSC-2", "perception.native_geometry_setup", "required when a transform is configured");

    // MED-2 preview stream (19 section 13.1): loopback RTSP port and encoding.
    exact_keys(root, "perception.med2", {"rgbd_rtsp_port", "rgbd_pub_port", "width", "height", "fps", "bitrate_kbps", "gop"});
    c.med2.rtsp_port = static_cast<int>(integer(root, "perception.med2.rgbd_rtsp_port", 1, 65535));
    if (!is_null(root, "perception.med2.rgbd_pub_port")) {
        c.med2.pub_port = static_cast<int>(integer(root, "perception.med2.rgbd_pub_port", 1, 65535));
        if (*c.med2.pub_port == c.med2.rtsp_port)
            fail("PSC-2", "perception.med2.rgbd_pub_port", "must differ from the loopback RTSP port");
    }
    c.med2.width = static_cast<int>(integer(root, "perception.med2.width", 1, 8192));
    c.med2.height = static_cast<int>(integer(root, "perception.med2.height", 1, 8192));
    c.med2.fps = static_cast<int>(integer(root, "perception.med2.fps", 1, 240));
    c.med2.bitrate_kbps = static_cast<int>(integer(root, "perception.med2.bitrate_kbps", 1, 1000000));
    c.med2.gop = static_cast<int>(integer(root, "perception.med2.gop", 1, 1000));

    // Age limit of an Objects snapshot the S channel may still inject into the Profile (3.4A).
    exact_keys(root, "perception.objects_inject", {"objects_stale_ms"});
    c.objects_stale_ms = integer(root, "perception.objects_inject.objects_stale_ms", 1, 60000);
    // Declared debug switches this service does not implement must be false.
    exact_keys(root, "perception.debug", {"pointcloud_enable", "legacy_keys_enable"});
    c.pointcloud_enable = boolean(root, "perception.debug.pointcloud_enable");
    c.legacy_keys_enable = boolean(root, "perception.debug.legacy_keys_enable");
    if (c.pointcloud_enable)
        fail("PSC-2", "perception.debug.pointcloud_enable", "debug point cloud is not implemented by this service");
    if (c.legacy_keys_enable)
        fail("PSC-2", "perception.debug.legacy_keys_enable", "legacy keys are not implemented by this service");
    // RT and GEN planes: two different routers, both on this host.
    exact_keys(root, "perception.zenoh", {"rt_endpoint", "gen_endpoint"});
    c.rt_endpoint = string_value(root, "perception.zenoh.rt_endpoint");
    c.gen_endpoint = string_value(root, "perception.zenoh.gen_endpoint");
    for (const auto& [path, value] : {std::pair<std::string, std::string>{"perception.zenoh.rt_endpoint", c.rt_endpoint},
                                      {"perception.zenoh.gen_endpoint", c.gen_endpoint}})
        if (!local_tcp_endpoint(value)) fail("PSC-2", path, "must be tcp/127.0.0.1:<port>");
    if (c.rt_endpoint == c.gen_endpoint)
        fail("PSC-2", "perception.zenoh", "RT and GEN planes must use different routers");
    return c;
}

// ---- MANIFEST ----
ManifestIdentity read_manifest(const std::string& text, const std::string& path,
                              const LoadInputs& in, const std::string& resolved_sha256) {
    YAML::Node m;
    try { m = YAML::Load(text); }
    catch (const std::exception& e) { fail("MANIFEST", path, std::string("not parseable JSON: ") + e.what()); }
    if (!m.IsMap()) fail("MANIFEST", path, "must be a JSON object");
    // A required non-empty string field; the error names its MANIFEST path.
    const auto str = [&](const YAML::Node& parent, const char* key, const std::string& where) {
        const auto v = parent[key];
        if (!v || !v.IsScalar()) fail("MANIFEST", where + "." + key, "missing string");
        const auto s = v.as<std::string>();
        if (s.empty()) fail("MANIFEST", where + "." + key, "empty string");
        return s;
    };
    // The MANIFEST binds the snapshot: schema, robot id, THIS boot (a snapshot
    // from an earlier boot is stale), and the perception entry's path and sha256
    // must name exactly the file that was loaded.
    if (str(m, "schema", "MANIFEST") != "xbrain.config.manifest/1")
        fail("MANIFEST", "MANIFEST.schema", "unsupported manifest schema");
    ManifestIdentity id;
    id.robot_id = str(m, "robot_id", "MANIFEST");
    if (!valid_rid(id.robot_id)) fail("MANIFEST", "MANIFEST.robot_id", "must match [a-z0-9_-]{1,32}");
    id.boot_id = str(m, "boot_id", "MANIFEST");
    if (id.boot_id != in.host.boot_id)
        fail("MANIFEST", "MANIFEST.boot_id", "snapshot is not from the current boot");
    id.config_rev = str(m, "config_rev", "MANIFEST");
    id.common_digest = str(m, "common_digest", "MANIFEST");
    const auto processes = m["processes"];
    if (!processes || !processes.IsMap()) fail("MANIFEST", "MANIFEST.processes", "missing object");
    const auto perception = processes["perception"];
    if (!perception || !perception.IsMap())
        fail("MANIFEST", "MANIFEST.processes.perception", "freeze snapshot does not include perception");
    id.perception_path = str(perception, "path", "MANIFEST.processes.perception");
    id.perception_sha256 = str(perception, "sha256", "MANIFEST.processes.perception");
    if (id.perception_path != in.resolved_path)
        fail("MANIFEST", "MANIFEST.processes.perception.path", "does not name the loaded resolved file");
    if (id.perception_sha256 != resolved_sha256)
        fail("MANIFEST", "MANIFEST.processes.perception.sha256", "does not match the loaded resolved file bytes");
    return id;
}

// ---- runtime switches (cand_R environment -> explicit implementation config) ----
// The verified cand_R combination was a set of environment variables; here each
// is an explicit key in perception.pipeline.runtime with its allowed values, so
// the running combination is always the configured (and digested) one.
// Retired with the safe traversable mask (level-3 trim, PSC-4 re-seal):
// safe_mask_seed and safe_mask_build have no row and no reader any more, so a
// runtime section that still carries either key is refused as unknown.
struct SwitchSpec {
    const char* key;
    const char* env;
    std::vector<std::string> values;  // empty => validated by a numeric rule below
};

const std::vector<SwitchSpec>& switch_table() {
    static const std::vector<SwitchSpec> table = {
        // neon (2026-10-06): fused with an own NEON resize for the sizes it supports, same bits.
        {"preprocess_impl", "PERCEPTION_PREPROCESS_IMPL", {"legacy", "fused", "neon"}},
        {"coco_mask_resize", "PERCEPTION_COCO_MASK_RESIZE", {"roi", "full"}},
        {"coco_mask_roi2", "PERCEPTION_COCO_MASK_ROI2", {"0", "1"}},
        {"active12_mask_rows", "PERCEPTION_ACTIVE12_MASK_ROWS", {"0", "1"}},
        {"depth_scan_fast", "PERCEPTION_DEPTH_SCAN_FAST", {"0", "1"}},
        {"async_log", "PERCEPTION_ASYNC_LOG", {"0", "1"}},
        {"inference_profile", "PERCEPTION_INFERENCE_PROFILE", {"0", "1"}},
        {"mask_output_copy", "PERCEPTION_MASK_OUTPUT_COPY", {"clone", "retain"}},
        {"post_role_swap", "PERCEPTION_POST_ROLE_SWAP", {"0", "1"}},
        {"active12_mask_binary_impl", "PERCEPTION_ACTIVE12_MASK_BINARY_IMPL", {"legacy", "compare"}},
        {"active12_mask_impl", "PERCEPTION_ACTIVE12_MASK_IMPL", {"serial", "parallel2"}},
        {"two_slot_jit", "PERCEPTION_TWO_SLOT_JIT", {"0", "1"}},
        {"jit_async_submit", "PERCEPTION_JIT_ASYNC_SUBMIT", {"0", "1"}},
        {"jit_bind_delay_ms", "PERCEPTION_JIT_BIND_DELAY_MS", {}},
        {"jit_fresh_bind", "PERCEPTION_JIT_FRESH_BIND", {"0", "1"}},
        {"jit_second_chance", "PERCEPTION_JIT_SECOND_CHANCE", {"0", "1"}},
        {"jit_frame_move", "PERCEPTION_JIT_FRAME_MOVE", {"0", "1"}},
        {"jit_probe_log", "PERCEPTION_JIT_PROBE_LOG", {"full", "compact", "off"}},
        {"jit_empty_acquire", "PERCEPTION_JIT_EMPTY_ACQUIRE", {"0", "1", "2"}},
        {"dla_active_lead_us", "PERCEPTION_DLA_ACTIVE_LEAD_US", {}},
        // Minimum interval between two dual-model device submissions in milliseconds (2026-10-20),
        // see pace_wait_ns in jit_schedule.hpp. A cap above the pair time (back to back about
        // 24.4 Hz) releases DLA, GPU and CPU time to the platform, and each ms of it above the pair
        // time removes one ms of host-jitter margin under the 50 ms line of 19 section 6.3. A value
        // is accepted per measurement window (>50 ms share, infer_gap / infer_rate_low, Objects
        // age). "0" submits back to back as before; otherwise a whole number 1..49 in one spelling,
        // and only with two_slot_jit "1" and jit_async_submit "1" (checked below): only the JIT
        // loop paces, and its serial-next path would hold every result back by the whole wait.
        {"infer_min_interval_ms", "PERCEPTION_INFER_MIN_INTERVAL_MS", {}},
        {"inference_handoff_first", "PERCEPTION_INFERENCE_HANDOFF_FIRST", {"0", "1"}},
        {"capture_affinity_pin2", "PERCEPTION_CAPTURE_AFFINITY_PIN2", {"0", "1"}},
        // Placement of the camera SDK's stream threads (2026-10-13, issues.md 9.1), see
        // sdk_stream_thread_placement.hpp: "off", or the one CPU on which the threads the SDK
        // creates during the pipeline start are born and stay. Only cores perception may ever
        // use are spellable: 2, 3, the lent 7 and the borrow core 1.
        {"capture_sdk_threads_cpu", "PERCEPTION_CAPTURE_SDK_THREADS_CPU", {"off", "1", "2", "3", "7"}},
        // Capture time (2026-10-14, 19 section 2.4 / PD-11), see capture_clock.hpp and
        // capture_time_state.hpp: the calibrated shortest "camera frame timestamp -> pickup" of
        // this camera, link and SDK in whole microseconds (10000..100000; 30767 measured on the
        // Gemini 338Le over Ethernet, SDK 2.9.3). The one calibration of the capture clock; it
        // cannot be unset, the service stamps no frameset without it.
        {"capture_min_path_us", "PERCEPTION_CAPTURE_MIN_PATH_US", {}},
        {"gemini_opencv_threads", "PERCEPTION_GEMINI_OPENCV_THREADS", {"1", "2"}},
        {"depth_sort_impl", "PERCEPTION_DEPTH_SORT_IMPL", {"stdsort", "codesort"}},
        // "2" (2026-10-08): the same two lanes with a shared, largest-first split (same outputs);
        // "alt" (diagnostic): fixed split on even frame ids, shared on odd ones, for a same-scene
        // comparison of the two splits in one run.
        {"parallel_depth_sample", "PERCEPTION_PARALLEL_DEPTH_SAMPLE", {"0", "1", "2", "alt"}},
        // Hardware MJPG decode (2026-10-05), see HwMjpegDecoder: nvv4l2 is the measured and example
        // default, approved by the integration lead on 2026-10-08 (issues.md 3.5 A, formal
        // configuration); sdk keeps the SDK software decode.
        {"gemini_mjpeg_decode", "PERCEPTION_GEMINI_MJPEG_DECODE", {"sdk", "nvv4l2"}},
        // Diagnostic (2026-10-05): one SUP_FRAME line per Supervisor frame with the wall and thread
        // CPU time of its stages; "0" in production. It used to be an inherited variable only, which
        // the formal service refuses, so the Supervisor stage could not be measured in service runs.
        {"sup_frame_log", "PERCEPTION_SUP_FRAME_LOG", {"0", "1"}},
        // T channel (2026-10-08, issues.md 3.1): "off" during the transition (the integration lead's
        // ruling, formal config): no class-92 mask slot, producer or Profile T owner, the Profile
        // carries no T (t_seg null, bit0 clear) and Status raises no_traversable_seg. "on" runs the
        // plan-2f T channel (development measurements until the enable-T batch).
        {"traversable_seg", "PERCEPTION_TRAVERSABLE_SEG", {"off", "on"}},
        // Padding-anchor guard of the Active12 decoder (2026-10-10), see Active12PaddingGuard in
        // postprocess.hpp: "1" drops candidates whose anchor centre lies in the letterbox padding
        // (an INT8 engine with quantized first layers reports traversable_area there) and counts
        // them in ACTIVE12_PADDING_GUARD / ACTIVE_MASK_SUMMARY.
        {"active12_padding_anchor_guard", "PERCEPTION_ACTIVE12_PADDING_ANCHOR_GUARD", {"0", "1"}},
        // S publish guard in milliseconds (2026-10-10), see rns_semantic.hpp: an Objects snapshot
        // within this margin of objects_stale_ms at the Profile's check is treated as stale, so
        // the frame is published geometry-only instead of being refused as a whole when the
        // snapshot expires before the put. "0" is the plain bound.
        {"semantic_publish_guard_ms", "PERCEPTION_SEMANTIC_PUBLISH_GUARD_MS", {"0", "5", "10", "15", "20", "25", "30"}},
        // Paced Objects release in milliseconds (2026-10-10), see rns_objects_release.hpp: a
        // finished Objects body is published on a thread of its own once this long has passed since
        // the frame's outputs were seen ready (since those of the oldest body it replaced, if any);
        // the body published is always the latest handed over (2026-10-16, latest wins). With a
        // delay longer than the frame period plus the time from ready to hand-over only every other
        // body is published (rns_objects_release.hpp). "0" keeps the publish inside the Objects callback. Any other value needs two_slot_jit "1"
        // (checked below): only that loop records the ready time.
        {"objects_release_delay_ms", "PERCEPTION_OBJECTS_RELEASE_DELAY_MS",
         {"0", "20", "25", "30", "33", "35", "38", "40", "43", "45", "50"}},
        // Diagnostic: one OBJ_RELEASE line per released, overwritten or discarded Objects body.
        {"objects_release_log", "PERCEPTION_OBJECTS_RELEASE_LOG", {"0", "1"}},
        // Object geometry points (2026-10-08, see rns_object_geometry.hpp): exact double precision,
        // or single precision with every output widened by 1 mm (more conservative, not bit-identical).
        {"object_geometry_points", "PERCEPTION_OBJECT_GEOMETRY_POINTS", {"exact", "float"}},
        // Applied by the service itself (cv::setNumThreads before init), never an
        // environment variable: the cand_R shell set the process pool to 2.
        {"process_opencv_threads", "", {"1", "2"}},
    };
    return table;
}

// Optional production switches: absent leaves nothing set and adds no line to the runtime digest,
// so a configuration written before such a switch existed keeps its digest and seal; present, it
// is read, checked and handed on like a table switch (no "unset" spelling).
const std::vector<SwitchSpec>& optional_switch_table() {
    static const std::vector<SwitchSpec> table = {
        // The Profile thread alone on one core, see profile_thread_placement.hpp: "off", or that
        // core (applied by the service itself). Only with capture_affinity_pin2 "1" and a core
        // for capture_sdk_threads_cpu other than the capture core, and neither of those two cores
        // itself (checked below); the service's own CPU set is checked when it starts
        // (profile_thread_plan), not here.
        {"profile_thread_cpu", "", {"off", "1", "2", "3", "7"}},
        // Profile coverage tickets (2026-10-27), see GridOptions::fitted_tickets: "prior" (or absent)
        // keeps the static prior-plane tickets; "fitted" draws each frame's tickets from its fitted
        // plane, so a small tilt of the ground against the calibration prior no longer ends FREE near
        // 2 m (.26 outdoors: 0.67 degrees, ground fully seen to 3.3 m). Read by build_profile_geometry.
        {"coverage_tickets", "PERCEPTION_COVERAGE_TICKETS", {"prior", "fitted"}},
    };
    return table;
}

// Diagnostic/experiment switches never set by the formal service. They are unset at startup
// even when present, so a variable left over from a developer shell or an older unit file
// cannot turn on tracing or change behaviour in production.
const std::vector<const char*>& diagnostic_switches() {
    static const std::vector<const char*> names = {
        "PERCEPTION_ACTIVE12_MASK_LOG",
        "PERCEPTION_BT_MASK_SHADOW_JSONL", "PERCEPTION_BT_MASK_SHADOW_MAX_UPDATES",
        "PERCEPTION_CAPTURE_DECODE_CPU_TRACE", "PERCEPTION_CAPTURE_PUSH_TRACE",
        "PERCEPTION_DEPTH_LANE_TRACE", "PERCEPTION_DEPTH_PROFILE",
        "PERCEPTION_DIAGNOSTIC_SUPPRESS_HEVC", "PERCEPTION_EXECUTION_CPU_TRACE",
        "PERCEPTION_JIT_STALL_TRACE", "PERCEPTION_MEDIA_NICE", "PERCEPTION_POST_DUMP_DIR",
        "PERCEPTION_POST_DUMP_FRAMES", "PERCEPTION_POST_HANDOFF_TRACE",
        "PERCEPTION_TRT_PAIR_ONLY_PROFILE", "RNS_NATIVE_CAMERA_SERIAL"};
    return names;
}

// A present, non-null yaml-cpp scalar (yaml-cpp has several spellings of null).
std::string scalar_text(const YAML::Node& n, const std::string& path, const char* check) {
    if (!n) fail(check, path, "missing key");
    if (n.IsNull()) fail(check, path, "null (not configured)");
    if (!n.IsScalar()) fail(check, path, "must be a scalar");
    if (n.Tag() == "tag:yaml.org,2002:null" || n.Scalar().empty() || n.Scalar() == "~" || n.Scalar() == "null")
        fail(check, path, "null (not configured)");
    return n.Scalar();
}

std::vector<RuntimeSetting> read_runtime(const YAML::Node& runtime) {
    const std::string base = "perception.pipeline.runtime";
    if (!runtime || !runtime.IsMap()) fail("RUNTIME", base, "missing map");
    std::set<std::string> known;
    for (const auto& spec : switch_table()) known.insert(spec.key);
    for (const auto& spec : optional_switch_table()) known.insert(spec.key);
    for (const auto& item : runtime) {
        const auto key = item.first.as<std::string>();
        if (!known.count(key)) fail("RUNTIME", base + "." + key, "unknown or non-production switch");
    }
    // Every switch must be present (no default); "unset" leaves the variable
    // absent, anything else must be an allowed value.
    std::vector<RuntimeSetting> out;
    for (const auto& spec : switch_table()) {
        const std::string path = base + "." + spec.key;
        const auto value = scalar_text(runtime[spec.key], path, "RUNTIME");
        RuntimeSetting s{spec.key, spec.env, "", false};
        if (value != "unset") {
            bool ok = std::find(spec.values.begin(), spec.values.end(), value) != spec.values.end();
            // The numeric switches: jit_bind_delay_ms is a non-negative decimal (digits and
            // a point only, so no exponent or sign); dla_active_lead_us a whole number of
            // microseconds up to 50000 written without leading zeros (one spelling per value);
            // capture_min_path_us a whole number of microseconds, 10000..100000, one spelling;
            // infer_min_interval_ms "0" or a whole number of milliseconds 1..49, one spelling.
            if (spec.values.empty()) {
                if (std::string(spec.key) == "capture_min_path_us") {
                    ok = valid_capture_min_path_text(value);
                } else if (std::string(spec.key) == "infer_min_interval_ms") {
                    ok = valid_infer_min_interval_text(value);
                } else if (std::string(spec.key) == "jit_bind_delay_ms") {
                    char* end = nullptr;
                    errno = 0;
                    const double v = std::strtod(value.c_str(), &end);
                    ok = errno == 0 && end == value.c_str() + value.size() && std::isfinite(v) && v >= 0 &&
                         value.find_first_not_of("0123456789.") == std::string::npos;
                } else {
                    ok = !value.empty() && value.size() <= 5 &&
                         std::all_of(value.begin(), value.end(), [](unsigned char c) { return std::isdigit(c); }) &&
                         std::strtol(value.c_str(), nullptr, 10) <= 50000 && (value == "0" || value[0] != '0');
                }
            }
            if (!ok) fail("RUNTIME", path, "value '" + value + "' not allowed in production");
            s.value = value;
            s.set = true;
        }
        out.push_back(std::move(s));
    }
    for (const auto& spec : optional_switch_table()) {
        if (!runtime[spec.key]) continue;
        const std::string path = base + "." + spec.key;
        const auto value = scalar_text(runtime[spec.key], path, "RUNTIME");
        if (std::find(spec.values.begin(), spec.values.end(), value) == spec.values.end())
            fail("RUNTIME", path, "value '" + value + "' not allowed in production");
        out.push_back(RuntimeSetting{spec.key, spec.env, value, true});
    }
    // The paced release starts from a ready time that exists on the two-slot JIT loop only.
    const auto value_of = [&](const char* key) {
        for (const auto& s : out)
            if (s.key == key) return s.set ? s.value : std::string();
        return std::string();
    };
    // The capture clock has no default calibration: "unset" would leave every frameset unstamped.
    if (value_of("capture_min_path_us").empty())
        fail("RUNTIME", base + ".capture_min_path_us", "value 'unset' not allowed: the capture clock needs its calibration");
    const std::string release_delay = value_of("objects_release_delay_ms");
    if (!release_delay.empty() && release_delay != "0" && value_of("two_slot_jit") != "1")
        fail("RUNTIME", base + ".objects_release_delay_ms",
             "value '" + release_delay + "' needs two_slot_jit \"1\" (the serial loop records no ready time)");
    // Likewise the minimum submit interval: the serial loop would silently run without it, and
    // the JIT loop's serial-next path would hold every result back by the whole wait.
    const std::string min_interval = value_of("infer_min_interval_ms");
    if (!min_interval.empty() && min_interval != "0" && value_of("two_slot_jit") != "1")
        fail("RUNTIME", base + ".infer_min_interval_ms",
             "value '" + min_interval + "' needs two_slot_jit \"1\" (only the JIT loop paces its submissions)");
    if (!min_interval.empty() && min_interval != "0" && value_of("jit_async_submit") != "1")
        fail("RUNTIME", base + ".infer_min_interval_ms",
             "value '" + min_interval + "' needs jit_async_submit \"1\" (a serial next submit would delay every result by the wait)");
    // The Profile core is placed after the capture thread's pin, apart from the SDK's stream
    // threads, which are told from the other threads by their core.
    const std::string profile_cpu = value_of("profile_thread_cpu");
    if (!profile_cpu.empty() && profile_cpu != "off") {
        const std::string path = base + ".profile_thread_cpu";
        if (value_of("capture_affinity_pin2") != "1")
            fail("RUNTIME", path, "value '" + profile_cpu + "' needs capture_affinity_pin2 \"1\" (placed after that pin)");
        const std::string sdk_cpu = value_of("capture_sdk_threads_cpu");
        if (sdk_cpu.empty() || sdk_cpu == "off")
            fail("RUNTIME", path, "value '" + profile_cpu + "' needs a core for capture_sdk_threads_cpu (the SDK's threads are told apart by it)");
        if (sdk_cpu == std::to_string(kCapturePinCpu))
            fail("RUNTIME", path, "value '" + profile_cpu + "' needs capture_sdk_threads_cpu on another core than the capture_affinity_pin2 pin (the SDK's threads are told apart by their core)");
        if (sdk_cpu == profile_cpu)
            fail("RUNTIME", path, "value '" + profile_cpu + "' is the core of capture_sdk_threads_cpu");
        if (profile_cpu == std::to_string(kCapturePinCpu))
            fail("RUNTIME", path, "value '" + profile_cpu + "' is the core of the capture_affinity_pin2 pin");
    }
    return out;
}

bool absolute_path(const YAML::Node& n) {
    return n && n.IsScalar() && !n.Scalar().empty() && n.Scalar()[0] == '/';
}

YAML::Node read_pipeline(const std::string& text, const ContractConfig& contract, ServiceConfig& out) {
    YAML::Node p;
    try { p = YAML::Load(text); }
    catch (const std::exception& e) { fail("PSC-2", "perception.pipeline", std::string("not parseable: ") + e.what()); }
    if (!p.IsMap()) fail("PSC-2", "perception.pipeline", "must be a map");
    // A fixed section set: sections retired from the legacy node (outputs, safe mask, ...) are
    // refused, so an old configuration cannot look accepted while part of it is ignored.
    const std::set<std::string> required = {"camera", "inference", "dual_model", "queue", "log", "tracking",
                                            "depth", "fusion", "capture", "velocity", "runtime", "media", "ros"};
    const std::set<std::string> optional = {"odom"};
    for (const auto& item : p) {
        const auto key = item.first.as<std::string>();
        if (!required.count(key) && !optional.count(key))
            fail("PSC-2", "perception.pipeline." + key, "unknown or retired implementation section");
    }
    for (const auto& key : required)
        if (!p[key]) fail("PSC-2", "perception.pipeline." + key, "missing section");

    // PSC-2: the service runs COCO80 and Active12 together (class schema 94; T
    // needs Active12 class 92). dual_model.enabled is a required YAML bool, read
    // the way yolo::Config reads it, and must be true. A missing, null, non-bool
    // or false value is refused here, before any dual-model JSON, engine or
    // model file is read. Read through a const node so the check cannot add a
    // key to the pipeline document that is digested for PSC-4.
    {
        const YAML::Node& pipeline = p;
        const YAML::Node dual = pipeline["dual_model"];
        if (!dual.IsMap()) fail("PSC-2", "perception.pipeline.dual_model", "must be a map");
        const std::string key = "perception.pipeline.dual_model.enabled";
        const YAML::Node flag = dual["enabled"];
        (void)scalar_text(flag, key, "PSC-2");
        bool enabled = false;
        if (!YAML::convert<bool>::decode(flag, enabled)) fail("PSC-2", key, "must be a YAML bool");
        if (!enabled) fail("PSC-2", key, "must be true: the service runs COCO80 and Active12 together");
    }

    // PSC-3: the one camera and the stream sizes the geometry and models are
    // built for.
    const auto camera = p["camera"];
    if (scalar_text(camera["driver"], "perception.pipeline.camera.driver", "PSC-3") != "orbbec_gemini338le")
        fail("PSC-3", "perception.pipeline.camera.driver", "formal service requires orbbec_gemini338le");
    scalar_text(camera["serial"], "perception.pipeline.camera.serial", "PSC-3");
    const auto dims = [&](const char* section, int w, int h, int fps) {
        const auto s = camera[section];
        const std::string base = std::string("perception.pipeline.camera.") + section;
        if (!s || !s.IsMap()) fail("PSC-3", base, "missing map");
        for (const auto& [k, v] : {std::pair<const char*, int>{"width", w}, {"height", h}, {"fps", fps}})
            if (scalar_text(s[k], base + "." + k, "PSC-3") != std::to_string(v))
                fail("PSC-3", base + "." + k, "must be " + std::to_string(v));
    };
    dims("output", 1280, 800, 30);
    dims("depth", 640, 400, 30);
    if (scalar_text(camera["depth"]["device_format"], "perception.pipeline.camera.depth.device_format", "PSC-3") != "y16")
        fail("PSC-3", "perception.pipeline.camera.depth.device_format", "must be y16");

    // Model and engine files by absolute path only (no working-dir dependence).
    for (const auto& [section, key] : {std::pair<const char*, const char*>{"inference", "model_path"},
                                       {"inference", "trt_engine_path"}, {"dual_model", "runtime_config_path"},
                                       {"dual_model", "class_schema_path"}, {"dual_model", "active_model_path"},
                                       {"dual_model", "active_engine_path"}}) {
        if (!absolute_path(p[section][key]))
            fail("PSC-2", std::string("perception.pipeline.") + section + "." + key, "must be an absolute path");
    }
    // The optional INT8 build files (2026-10-10) likewise, when given; read through a const node
    // so the check cannot add a key to the digested pipeline document.
    {
        const YAML::Node& pipeline = p;
        for (const char* section : {"inference", "dual_model"}) {
            for (const char* key : {"int8_ranges", "int8_fp16_layers"}) {
                const YAML::Node value = pipeline[section][key];
                if (value && !value.IsNull() && !absolute_path(value))
                    fail("PSC-2", std::string("perception.pipeline.") + section + "." + key, "must be an absolute path");
            }
        }
    }

    const auto media = p["media"];
    if (!media.IsMap() || media.size() != 2 || !media["codec"] || !media["path"])
        fail("PSC-2", "perception.pipeline.media", "must hold exactly codec and path");
    out.media_codec = scalar_text(media["codec"], "perception.pipeline.media.codec", "PSC-2");
    out.media_path = scalar_text(media["path"], "perception.pipeline.media.path", "PSC-2");
    // 19 section 13.1: H.264 NVENC is the MED-2 default; H.265 is kept only to
    // reproduce earlier diagnostic runs, never a formal MED-2 (reply batch 8, B8-6).
    if (out.media_codec != "nvv4l2h264enc" && out.media_codec != "nvv4l2h265enc")
        fail("PSC-2", "perception.pipeline.media.codec", "must be nvv4l2h264enc (19 section 13.1) or nvv4l2h265enc (diagnostic)");
    if (out.media_path.size() < 2 || out.media_path[0] != '/' ||
        !std::all_of(out.media_path.begin() + 1, out.media_path.end(), [](unsigned char c) {
            return std::isalnum(c) || c == '_' || c == '-';
        }))
        fail("PSC-2", "perception.pipeline.media.path", "must be /<name> with [A-Za-z0-9_-]");
    const auto& med2 = contract.med2;
    // Implemented preview adaptation: centre crop of the 1280x800@30 camera output
    // (same width, even height difference) at a rate dividing 30. The 19 section
    // 13.1 default 1280x720@15 is one such preview; no scaling is implemented.
    if (med2.width != 1280 || med2.height > 800 || (800 - med2.height) % 2 != 0 || 30 % med2.fps != 0)
        fail("PSC-2", "perception.med2",
             "preview must be a centre crop of the 1280x800@30 output (width 1280, height <= 800 with an even "
             "difference) at a rate dividing 30; no scaling is implemented");

    out.runtime = read_runtime(p["runtime"]);

    // ROS executor switches (TF listener + capture command only, P19-4).
    const auto ros = p["ros"];
    if (!ros.IsMap() || ros.size() != 4 || !ros["enable"] || !ros["tf_enable"] || !ros["odom_frame"] || !ros["base_frame"])
        fail("PSC-2", "perception.pipeline.ros", "must hold exactly enable, tf_enable, odom_frame, base_frame");
    const auto flag = [&](const char* key) {
        const auto text = scalar_text(ros[key], std::string("perception.pipeline.ros.") + key, "PSC-2");
        if (text != "true" && text != "false") fail("PSC-2", std::string("perception.pipeline.ros.") + key, "must be true or false");
        return text == "true";
    };
    out.ros.enable = flag("enable");
    out.ros.tf_enable = flag("tf_enable");
    out.ros.odom_frame = scalar_text(ros["odom_frame"], "perception.pipeline.ros.odom_frame", "PSC-2");
    out.ros.base_frame = scalar_text(ros["base_frame"], "perception.pipeline.ros.base_frame", "PSC-2");
    if (out.ros.tf_enable && !out.ros.enable)
        fail("PSC-2", "perception.pipeline.ros.tf_enable", "TF requires the ROS executor");
    // Batch-4 reply B4-1: with a TF source the Supervisor estimates velocity in
    // the odom frame, but the Objects path supports only camera-frame raw
    // velocity (ego_removed, 19 section 4.2, is not implemented). Refuse the
    // configuration instead of publishing a wrongly rotated velocity.
    if (out.ros.tf_enable)
        fail("PSC-2", "perception.pipeline.ros.tf_enable",
             "odom-frame velocity is not supported by the Objects path (ego_removed not implemented)");
    for (const auto& [key, frame] : {std::pair<const char*, std::string>{"odom_frame", out.ros.odom_frame}, {"base_frame", out.ros.base_frame}})
        if (frame.find_first_not_of("abcdefghijklmnopqrstuvwxyz0123456789_/") != std::string::npos)
            fail("PSC-2", std::string("perception.pipeline.ros.") + key, "invalid TF frame name");

    // The yolo::Config document: the pipeline sections minus runtime/media/ros,
    // plus an hevc block synthesized from perception.med2 and pipeline.media
    // (loopback RTSP sink of the MED-2 preview).
    YAML::Node doc(YAML::NodeType::Map);
    for (const auto& item : p) {
        const auto key = item.first.as<std::string>();
        if (key != "runtime" && key != "media" && key != "ros") doc[key] = YAML::Clone(item.second);
    }
    YAML::Node hevc(YAML::NodeType::Map);
    hevc["enable"] = true;
    hevc["sink"] = "med2_loopback";
    hevc["rtsp_listen"] = "rtsp://127.0.0.1:" + std::to_string(med2.rtsp_port) + out.media_path;
    hevc["width"] = med2.width;
    hevc["height"] = med2.height;
    hevc["fps"] = med2.fps;
    hevc["bitrate"] = med2.bitrate_kbps * 1000;
    hevc["codec"] = out.media_codec;
    hevc["gop"] = med2.gop;
    doc["hevc"] = hevc;
    // Retired legacy sections are not configurable: read_pipeline() above refuses
    // them, and no code reads them any more.
    return doc;
}
}  // namespace

StartupCheckError::StartupCheckError(std::string check, std::string key, const std::string& why)
    : std::runtime_error("[" + check + "] " + key + ": " + why), check_(std::move(check)), key_(std::move(key)) {}

// The running kernel's boot id, read once and checked for shape (envelope_boot throws).
HostIdentity read_host_identity(const std::string& boot_id_path) {
    const auto text = trim(read_bounded(boot_id_path, "MANIFEST"));
    HostIdentity id{text};
    envelope_boot(id);
    return id;
}

// The envelope boot field: first 8 hex digits of a well-formed kernel boot_id.
std::string envelope_boot(const HostIdentity& host) {
    const auto& b = host.boot_id;
    const bool shape = b.size() == 36 && b[8] == '-' && b[13] == '-' && b[18] == '-' && b[23] == '-';
    std::string compact;
    for (std::size_t i = 0; i < b.size(); ++i) if (i != 8 && i != 13 && i != 18 && i != 23) compact += b[i];
    if (!shape || !lower_hex(compact, 32)) fail("MANIFEST", kBootIdPath, "malformed kernel boot_id");
    return b.substr(0, 8);
}

ServiceConfig load_service_config(const LoadInputs& in) {
    ServiceConfig out;
    out.resolved_path = in.resolved_path;
    out.manifest_path = in.manifest_path;
    // PSC-1: the resolved product by its canonical location, never a source file.
    const std::string prefix = in.resolved_prefix;
    if (prefix.empty() || prefix.back() != '/' || prefix[0] != '/')
        fail("PSC-1", prefix, "resolved prefix must be an absolute directory ending in /");
    if (in.resolved_path.rfind(prefix, 0) != 0)
        fail("PSC-1", in.resolved_path, "config must be the resolved product under " + prefix);
    char canonical[PATH_MAX];
    if (!::realpath(in.resolved_path.c_str(), canonical))
        fail("PSC-1", in.resolved_path, "resolved product does not exist");
    if (std::string(canonical).rfind(prefix, 0) != 0)
        fail("PSC-1", in.resolved_path, "resolves outside " + prefix + " (symlink to a source?)");
    if (in.manifest_path.rfind(prefix, 0) != 0)
        fail("PSC-1", in.manifest_path, "MANIFEST must be in " + prefix);
    envelope_boot(in.host);

    // Each file is read once; the hashes are of exactly the bytes parsed.
    const auto text = read_bounded(in.resolved_path, "PSC-1");
    out.resolved_sha256 = yolo::sha256_hex(text);
    const auto manifest_text = read_bounded(in.manifest_path, "MANIFEST");
    out.manifest_sha256 = yolo::sha256_hex(manifest_text);
    out.manifest = read_manifest(manifest_text, in.manifest_path, in, out.resolved_sha256);
    out.rid = out.manifest.robot_id;
    out.boot8 = envelope_boot(in.host);

    // PSC-2: contract text through the vector adapter and yaml_lite, pipeline
    // text through yaml-cpp.
    const auto split = split_pipeline(text);
    try {
        out.contract_yaml = config::adapt_resolved_yaml_vectors(split.contract);
    } catch (const std::exception& e) {
        fail("PSC-2", "perception", e.what());
    }
    YamlNode root;
    try { root = xbrain::config::parse_yaml(out.contract_yaml); }
    catch (const std::exception& e) { fail("PSC-2", "perception", e.what()); }
    out.contract = read_contract(root);
    out.pipeline = read_pipeline(split.pipeline, out.contract, out);

    // Digest of the effective switch combination, reported in config_digest.
    std::string canonical_runtime;
    for (const auto& s : out.runtime)
        canonical_runtime += (s.env.empty() ? "service:" + s.key : s.env) + "=" + (s.set ? s.value : "<unset>") + "\n";
    out.runtime_digest = yolo::sha256_hex(canonical_runtime);
    return out;
}

// The production switch names in table order (for tools and tests that enumerate them).
const std::vector<std::string>& runtime_switch_keys() {
    static const std::vector<std::string> keys = [] {
        std::vector<std::string> k;
        for (const auto& spec : switch_table()) k.push_back(spec.key);
        return k;
    }();
    return keys;
}

// Any PERCEPTION_* or RNS_* variable in the environment the service was started with is refused:
// the production combination comes only from the configuration, never from the caller's shell.
void reject_inherited_runtime_environment(char** envp) {
    for (char** e = envp; e && *e; ++e) {
        const std::string entry(*e);
        const auto name = entry.substr(0, entry.find('='));
        if (name.rfind("PERCEPTION_", 0) == 0 || name.rfind("RNS_", 0) == 0)
            fail("ENV", name, "inherited runtime switch refused; production switches come from perception.pipeline.runtime");
    }
}

// Diagnostic switches are always removed; production switches are set or unset
// as configured and read back, before any module reads them. An absent optional switch with a
// variable is unset.
void apply_runtime_environment(const ServiceConfig& config) {
    for (const auto* name : diagnostic_switches())
        if (::unsetenv(name) != 0) fail("ENV", name, "unsetenv failed");
    for (const auto& spec : optional_switch_table()) {
        if (spec.env[0] == '\0') continue;  // service-applied switch
        const bool present = std::any_of(config.runtime.begin(), config.runtime.end(),
                                         [&](const RuntimeSetting& s) { return s.key == spec.key; });
        if (!present && ::unsetenv(spec.env) != 0) fail("ENV", spec.env, "unsetenv failed");
    }
    for (const auto& s : config.runtime) {
        if (s.env.empty()) continue;  // service-applied switch
        const int rc = s.set ? ::setenv(s.env.c_str(), s.value.c_str(), 1) : ::unsetenv(s.env.c_str());
        if (rc != 0) fail("ENV", s.env, "setenv/unsetenv failed");
        const char* back = std::getenv(s.env.c_str());
        if (s.set ? (!back || s.value != back) : back != nullptr) fail("ENV", s.env, "readback mismatch");
    }
}

int process_open_cv_threads(const ServiceConfig& config) {
    for (const auto& r : config.runtime)
        if (r.key == "process_opencv_threads" && r.set) return std::stoi(r.value);
    return 0;
}

int profile_thread_cpu(const ServiceConfig& config) {
    for (const auto& r : config.runtime)
        if (r.key == "profile_thread_cpu" && r.set && r.value != "off") return std::stoi(r.value);
    return -1;
}

ProfileThreadPlan profile_thread_plan(const ServiceConfig& config, bool inherited_valid, std::uint64_t inherited_mask) {
    ProfileThreadPlan plan;
    const int profile_cpu = profile_thread_cpu(config);
    if (profile_cpu < 0) return plan;
    const std::string path = "perception.pipeline.runtime.profile_thread_cpu";
    if (!inherited_valid) fail("RUNTIME", path, "the inherited CPU set of the service could not be read");
    int sdk_cpu = -1;
    for (const auto& r : config.runtime)
        if (r.key == "capture_sdk_threads_cpu" && r.set && r.value != "off") sdk_cpu = std::stoi(r.value);
    plan = plan_profile_thread_placement(inherited_mask, profile_cpu, sdk_cpu, kCapturePinCpu);
    if (!plan.ok) {
        std::string cores;
        for (int cpu = 0; cpu < 64; ++cpu)
            if (inherited_mask & (std::uint64_t{1} << cpu)) cores += (cores.empty() ? "" : ",") + std::to_string(cpu);
        fail("RUNTIME", path, std::string("value '") + std::to_string(profile_cpu) + "' with the service's CPU set " +
             std::to_string(inherited_mask) + " (cores " + (cores.empty() ? "none" : cores) + "): " + plan.reason);
    }
    return plan;
}

// PSC-5: the three RT data keys of this robot (11 section 2.2.1, xbrain/{rid}/rt/perception/...)
// and the three bare GEN event keys (11 section 1.1.6 PC-2, event/{severity}/perception), nothing
// else (no wildcard).
std::vector<std::string> declared_key_registry(const std::string& rid) {
    if (!valid_rid(rid)) fail("PSC-5", rid, "invalid robot id");
    const std::string base = "xbrain/" + rid + "/";
    return {base + "rt/perception/profile", base + "rt/perception/objects", base + "rt/perception/status",
            "event/info/perception", "event/warn/perception", "event/fault/perception"};
}

PublisherKeys publisher_keys(const std::string& rid) {
    if (!valid_rid(rid)) fail("PSC-5", rid, "invalid robot id");
    const std::string rt = "xbrain/" + rid + "/rt/perception/", event = "event/";
    PublisherKeys keys;
    keys.profile = rt + "profile";
    keys.objects = rt + "objects";
    keys.status = rt + "status";
    keys.info = event + "info/perception";
    keys.warn = event + "warn/perception";
    keys.fault = event + "fault/perception";
    return keys;
}

void require_declared_keys(const std::string& rid, const std::vector<std::string>& declared) {
    const auto verdict = check_declared_keys(declared_key_registry(rid), declared);
    if (!verdict.ok) fail("PSC-5", verdict.key, verdict.reason);
}

}  // namespace perception::service
