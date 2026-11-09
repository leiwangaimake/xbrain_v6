/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_geometry_config.cpp
 * Brief: Parse the geometry setup keys into ProjectionBinding, GeometryOptions and GridOptions.
 *
 * Description:
 * What problem this solves. Implements rns_geometry_config.hpp. Every value is read explicitly
 * (no defaults), unknown keys under the two geometry sections are refused, and the final option
 * structs are passed through the owners' own validators, so a configuration that parses but
 * could not run is refused at setup with a dotted key path.
 */
#include "common/rns_geometry_config.hpp"
#include "common/resolved_yaml.hpp"

#include <xbrain/config/yaml_lite.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace perception::config {
namespace {
using xbrain::config::YamlNode;
constexpr double pi = 3.1415926535897932384626433832795;
const std::string profile = "perception.profile.";
const std::string extrinsic = "perception.extrinsic_base_cam.";
const std::string internal = "perception.native_geometry_setup.";

[[noreturn]] void invalid(const std::string& path, const std::string& why) {
    throw std::invalid_argument(path + ": " + why);
}
void require(bool ok, const std::string& path, const char* why) {
    if (!ok) invalid(path, why);
}
std::string trim(const std::string& s) { return xbrain::config::detail::trim(s); }
double scalar(const YamlNode& root, const std::string& path) {
    const double v = root.require_double(path);
    require(std::isfinite(v), path, "must be finite");
    return v;
}
double positive(const YamlNode& root, const std::string& path) {
    const double v = scalar(root, path);
    require(v > 0, path, "must be positive");
    return v;
}
double nonnegative(const YamlNode& root, const std::string& path) {
    const double v = scalar(root, path);
    require(v >= 0, path, "must be nonnegative");
    return v;
}
std::size_t integer(const YamlNode& root, const std::string& path, bool allow_zero=false) {
    const long v = root.require_int(path);
    require(v >= (allow_zero ? 0 : 1), path, "integer outside allowed range");
    return static_cast<std::size_t>(v);
}
// The fit seed as decimal text: the reader's integers (long) cannot hold every uint64.
std::uint64_t seed(const YamlNode& root, const std::string& path) {
    const auto s = root.require_string(path);
    require(!s.empty() && std::all_of(s.begin(), s.end(), [](unsigned char c) {
        return c >= '0' && c <= '9';
    }), path, "seed must be a decimal uint64");
    try { return std::stoull(s); }
    catch (const std::exception&) { invalid(path, "seed outside uint64 range"); }
}
// "[a, b, ...]" of exactly N finite numbers (after the vector adapter).
template<std::size_t N>
std::array<double, N> vector(const YamlNode& root, const std::string& path) {
    const auto s = root.require_string(path);
    require(s.size() >= 2 && s.front() == '[' && s.back() == ']', path, "expected numeric vector");
    const auto inside = s.substr(1, s.size()-2);
    std::array<double, N> result{};
    std::size_t begin = 0;
    for (std::size_t i = 0; i < N; ++i) {
        const auto end = inside.find(',', begin);
        require((i+1 == N) == (end == std::string::npos), path, "wrong vector length");
        const auto item = trim(inside.substr(begin, end == std::string::npos ? end : end-begin));
        std::size_t consumed = 0;
        try { result[i] = std::stod(item, &consumed); }
        catch (const std::exception&) { invalid(path+"["+std::to_string(i)+"]", "not numeric"); }
        require(consumed == item.size() && std::isfinite(result[i]),
                path+"["+std::to_string(i)+"]", "must be a finite number");
        begin = end == std::string::npos ? inside.size() : end+1;
    }
    return result;
}
// Walk a dotted path; nullptr when any segment is absent (never throws for that).
const YamlNode* optional_node(const YamlNode& root, const std::string& path) {
    const YamlNode* node = &root;
    std::size_t begin = 0;
    while (begin < path.size()) {
        require(node->is_map(), path, "parent must be a map");
        const auto end = path.find('.', begin);
        const auto it = node->items().find(path.substr(begin, end == std::string::npos ? end : end-begin));
        if (it == node->items().end()) return nullptr;
        node = &it->second;
        if (end == std::string::npos) return node;
        begin = end+1;
    }
    return node;
}
// Unknown keys are refused: a misspelled knob must not fall back silently.
void reject_unknown(const YamlNode& root, const std::string& path, const std::set<std::string>& allowed) {
    const auto& node = root.at(path);
    require(node.is_map(), path, "must be a map");
    for (const auto& item : node.items())
        require(allowed.count(item.first) != 0, path+"."+item.first, "unknown setup key");
}
// The calib_* record keys are allowed but not read here: ServiceConfig
// (read_extrinsic) validates them and service::CalibrationVerifier reads them.
BaseFromCamera base_from_camera(const YamlNode& root) {
    reject_unknown(root, "perception.extrinsic_base_cam", {
        "translation", "rotation_xyzw", "calib_record", "calib_sha256", "residual_max_m"});
    BaseFromCamera out;
    const auto t = vector<3>(root, extrinsic+"translation");
    out.t = {t[0], t[1], t[2]};
    const auto q = vector<4>(root, extrinsic+"rotation_xyzw");
    const double x=q[0], y=q[1], z=q[2], w=q[3];
    // Unit quaternion (x, y, z, w) -> row-major rotation matrix; not normalised
    // here, so the rotation equals the one in the calibration record.
    const double norm2 = x*x+y*y+z*z+w*w;
    require(std::isfinite(norm2) && std::abs(norm2-1.0) <= 1e-9,
            extrinsic+"rotation_xyzw", "must be unit length (not auto-normalized)");
    out.R = {1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w),
        2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w),
        2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)};
    return out;
}
}  // namespace

BaseFromCamera parse_base_from_camera(const std::string& resolved_yaml) {
    return base_from_camera(xbrain::config::parse_yaml(adapt_resolved_yaml_vectors(resolved_yaml)));
}

NativeGeometrySetup parse_native_geometry_setup(const std::string& resolved_yaml,
                                            const native_depth::Metadata& native_profile) {
    const auto root = xbrain::config::parse_yaml(adapt_resolved_yaml_vectors(resolved_yaml));
    reject_unknown(root, "perception.extrinsic_base_cam", {
        "translation", "rotation_xyzw", "calib_record", "calib_sha256", "residual_max_m"});
    reject_unknown(root, "perception.native_geometry_setup", {
        "schema_version", "prior_normal", "prior_d_m", "fit_near_min_m", "fit_near_max_m",
        "fit_prior_band_m", "fit_inlier_distance_m", "fit_min_xy_stddev_m", "fit_min_up_dot",
        "fit_max_prior_angle_deg", "fit_min_inliers", "fit_hypotheses", "fit_seed"});
    // extrinsic_calibrated is a verdict, never a configuration value.
    for (const auto& path : {std::string("perception.extrinsic_calibrated"), profile+"extrinsic_calibrated"})
        require(optional_node(root, path) == nullptr, path, "calibration status cannot be configured");
    require(root.require_int(internal+"schema_version") == 1, internal+"schema_version", "unsupported local schema");

    // Projection: the bound stream's own metadata plus the configured transform.
    NativeGeometrySetup out;
    auto& b = out.projection;
    b.intrinsics = native_profile.intrinsics;
    b.distortion = native_profile.distortion;
    b.unit_m = native_profile.unit_m;
    const auto base_camera = base_from_camera(root);
    b.t_base_camera = base_camera.t;
    b.R_base_camera = base_camera.R;
    require(geometry::valid_projection_binding(b), "native_profile/perception.extrinsic_base_cam",
            "invalid native 640x400 metadata or rigid transform");

    // Geometry options: bin domain (float32 wire domain, symmetric), range and
    // height classes from the profile contract, prior plane and fit knobs from
    // the internal section.
    auto& g = out.geometry;
    g.n_bins = integer(root, profile+"n_bins");
    require(g.n_bins <= geometry::kGeometryMaxBins && g.n_bins%2 == 1,
            profile+"n_bins", "requires odd bounded symmetric bin count");
    const double raw_min = scalar(root, profile+"angle_min_rad");
    const double raw_step = positive(root, profile+"angle_step_rad");
    require(std::abs(raw_min) <= pi && raw_step <= 2*pi, profile+"angle_min_rad/angle_step_rad", "angular range invalid");
    g.angle_min_rad = static_cast<double>(static_cast<float>(raw_min));
    g.angle_step_rad = static_cast<double>(static_cast<float>(raw_step));
    const auto first = static_cast<float>(-0.5*static_cast<double>(g.n_bins-1)*g.angle_step_rad);
    require(g.angle_min_rad == static_cast<double>(first), profile+"angle_min_rad",
            "first bin must be symmetric center in the float32 wire domain");
    g.range_max_m = positive(root, profile+"range_max_m");
    require(g.range_max_m <= 6, profile+"range_max_m", "must not exceed 6 metres");
    g.blind_near_lo_m = nonnegative(root, profile+"blind_near_lo_m");
    g.h_tol_m = positive(root, profile+"h_tol_m");
    g.z_pass_m = positive(root, profile+"z_pass_m");
    require(g.z_pass_m <= std::numeric_limits<float>::max(), profile+"z_pass_m", "wire float32 overflow");
    g.eps_neg = nonnegative(root, profile+"eps_neg");
    g.w_h_m = nonnegative(root, profile+"w_h_m");
    const auto n = vector<3>(root, internal+"prior_normal");
    g.prior.normal = {n[0], n[1], n[2]};
    g.prior.d = scalar(root, internal+"prior_d_m");
    g.fit_near_min_m = nonnegative(root, internal+"fit_near_min_m");
    g.fit_near_max_m = positive(root, internal+"fit_near_max_m");
    g.fit_prior_band_m = positive(root, internal+"fit_prior_band_m");
    g.fit.inlier_distance_m = positive(root, internal+"fit_inlier_distance_m");
    g.fit.min_inlier_fraction = positive(root, profile+"plane_fit_min_inlier_frac");
    g.fit.max_inlier_rms_m = nonnegative(root, profile+"plane_fit_resid_max_m");
    g.fit.min_xy_stddev_m = positive(root, internal+"fit_min_xy_stddev_m");
    g.fit.min_up_dot = nonnegative(root, internal+"fit_min_up_dot");
    g.fit.max_prior_angle_rad = nonnegative(root, internal+"fit_max_prior_angle_deg")*pi/180;
    g.fit.min_inliers = integer(root, internal+"fit_min_inliers");
    g.fit.hypotheses = integer(root, internal+"fit_hypotheses");
    g.fit.seed = seed(root, internal+"fit_seed");
    require(geometry::valid_geometry_options(g), "perception.profile/perception.native_geometry_setup",
            "invalid prior plane, fit options or geometry domain");

    // Grid options, and the same storage bounds NormalGrid enforces.
    out.grid.dr_m = positive(root, profile+"dr_m");
    out.grid.cover_min = positive(root, profile+"cover_min");
    out.grid.slope_max_rad = nonnegative(root, profile+"slope_max_deg")*pi/180;
    out.grid.sigma_max_m = nonnegative(root, profile+"sigma_max_m");
    require(geometry::valid_grid_options(out.grid), "perception.profile", "invalid grid thresholds");
    const double cells = std::ceil(g.range_max_m/out.grid.dr_m);
    require(std::isfinite(cells) && cells >= 1 && cells <= geometry::kMaxRadialCells,
            profile+"dr_m", "radial grid capacity exceeded");
    require(g.n_bins <= geometry::kMaxGridCells/static_cast<std::size_t>(cells),
            profile+"n_bins/dr_m", "total grid capacity exceeded");
    return out;
}
}  // namespace perception::config
