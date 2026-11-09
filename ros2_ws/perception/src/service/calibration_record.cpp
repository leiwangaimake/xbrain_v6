/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: calibration_record.cpp
 * Brief: Strict record parser/formatter and the CalibrationVerifier verdict chain.
 *
 * Description:
 * What problem this solves. Implements calibration_record.hpp. The parser is deliberately
 * strict (exact key sets, printable ASCII, no comments or quotes, round-trip numbers) so one
 * record has exactly one byte form: the sha256 in the configuration then pins both the file and
 * its meaning. The verifier walks the verdict rules in a fixed order and stops at the first
 * failure, reporting its code and a one-line detail for the startup log and config_digest.
 */
#include "service/calibration_record.hpp"

#include "common/config.hpp"
#include "common/resolved_yaml.hpp"

#include <xbrain/config/yaml_lite.h>

#include <sys/stat.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>

namespace perception::service {
namespace {
using xbrain::config::YamlNode;
constexpr const char* kRoot = "calib_record";

// Every parse error names the dotted record path, e.g. "calib_record.scope: ...".
[[noreturn]] void invalid(const std::string& path, const std::string& why) {
    throw std::invalid_argument(path + ": " + why);
}
// Shorthand for the many single-condition rules below.
void require(bool ok, const std::string& path, const char* why) {
    if (!ok) invalid(path, why);
}
std::string at(const char* suffix) { return std::string(kRoot) + "." + suffix; }

// The node at a dotted path, or an error naming that path (yaml_lite's own message would not).
const YamlNode& child(const YamlNode& root, const std::string& path) {
    try {
        return root.at(path);
    } catch (const std::exception&) {
        invalid(path, "missing key");
    }
}
// The map at path has exactly these keys: unknown ones are errors, not ignored,
// so a misspelled field cannot silently fall back to a default.
void exact_keys(const YamlNode& root, const std::string& path, const std::set<std::string>& keys) {
    const auto& n = child(root, path);
    require(n.is_map(), path, "must be a map");
    std::set<std::string> seen;
    for (const auto& item : n.items()) {
        require(keys.count(item.first) != 0, path + "." + item.first, "unknown key");
        seen.insert(item.first);
    }
    for (const auto& k : keys) require(seen.count(k) != 0, path + "." + k, "missing key");
}
// A non-empty, bounded, printable-ASCII scalar without surrounding whitespace.
std::string text(const YamlNode& root, const std::string& path, std::size_t max_len) {
    const auto& n = child(root, path);
    require(n.is_scalar() && !n.is_null(), path, "must be a non-null scalar");
    const auto s = root.require_string(path);
    require(!s.empty() && s.size() <= max_len, path, "empty or too long");
    for (unsigned char c : s) require(c >= 0x20 && c < 0x7f, path, "printable ASCII only");
    require(s == xbrain::config::detail::trim(s), path, "leading/trailing whitespace");
    return s;
}
// An identifier-like value ([A-Za-z0-9._-]), for ids, serials and tool names.
std::string token(const YamlNode& root, const std::string& path, std::size_t max_len) {
    const auto s = text(root, path, max_len);
    for (unsigned char c : s)
        require(std::isalnum(c) || c == '_' || c == '-' || c == '.', path, "allowed characters: [A-Za-z0-9._-]");
    return s;
}
// A closed vocabulary value: anything outside the set is refused, never mapped to a default.
std::string one_of(const YamlNode& root, const std::string& path, const std::set<std::string>& values) {
    const auto s = text(root, path, 64);
    require(values.count(s) != 0, path, "unsupported value");
    return s;
}
// The whole text must be one finite number (no trailing characters).
double finite(const std::string& item, const std::string& path) {
    std::size_t used = 0;
    double v = 0;
    try {
        v = std::stod(item, &used);
    } catch (const std::exception&) {
        invalid(path, "not a number");
    }
    require(used == item.size() && std::isfinite(v), path, "must be one finite number");
    return v;
}
// A finite number from a bounded scalar.
double number(const YamlNode& root, const std::string& path) {
    return finite(text(root, path, 64), path);
}
// A whole-text integer within [lo, hi] (e.g. "12abc" or "1e3" is refused).
long integer(const YamlNode& root, const std::string& path, long lo, long hi) {
    const auto s = text(root, path, 32);
    std::size_t used = 0;
    long v = 0;
    try {
        v = std::stol(s, &used);
    } catch (const std::exception&) {
        invalid(path, "not an integer");
    }
    require(used == s.size() && v >= lo && v <= hi, path, "integer out of range");
    return v;
}
// "[a, b, c]" with exactly N finite numbers.
template <std::size_t N>
std::array<double, N> vector(const YamlNode& root, const std::string& path) {
    const auto s = text(root, path, 512);
    require(s.size() >= 2 && s.front() == '[' && s.back() == ']', path, "expected [..] vector");
    std::array<double, N> out{};
    // Split on commas between the brackets; each element must be one finite number and the
    // count must be exactly N (a short or long vector is an error, not padded or cut).
    std::size_t begin = 1, count = 0;
    while (begin <= s.size() - 1) {
        auto end = s.find(',', begin);
        if (end == std::string::npos) end = s.size() - 1;
        require(count < N, path, "wrong vector length");
        out[count++] = finite(xbrain::config::detail::trim(s.substr(begin, end - begin)), path);
        begin = end + 1;
    }
    require(count == N, path, "wrong vector length");
    return out;
}
// The fixed creation-time form; only its shape is checked (it is an audit label, not a clock).
bool utc_stamp(const std::string& s) {
    // YYYY-MM-DDTHH:MM:SSZ
    if (s.size() != 20 || s[4] != '-' || s[7] != '-' || s[10] != 'T' || s[13] != ':' || s[16] != ':' || s[19] != 'Z')
        return false;
    for (std::size_t i : {0U, 1U, 2U, 3U, 5U, 6U, 8U, 9U, 11U, 12U, 14U, 15U, 17U, 18U})
        if (!std::isdigit(static_cast<unsigned char>(s[i]))) return false;
    return true;
}
// %.17g: the shortest text guaranteed to parse back to the same double.
std::string round_trip(double v) {
    char buf[40];
    const int n = std::snprintf(buf, sizeof(buf), "%.17g", v);
    if (n <= 0 || static_cast<std::size_t>(n) >= sizeof(buf)) throw std::runtime_error("number format failed");
    return buf;
}
}  // namespace

CalibrationRecord parse_calibration_record(const std::string& bytes) {
    require(bytes.size() <= kCalibrationRecordMaxBytes, kRoot, "record exceeds 64 KiB");
    // Raw-byte rules (the parsed value no longer shows what yaml_lite stripped):
    // no comments and no quoting, so every record round-trips byte-exactly.
    // One byte form per record: plain printable text only.
    for (unsigned char c : bytes) {
        require(c == '\n' || c == '\r' || (c >= 0x20 && c < 0x7f), kRoot, "record must be printable ASCII text");
        require(c != '#' && c != '"' && c != '\'', kRoot, "comments and quotes are not allowed in a record");
    }
    // yaml_lite has no sequence node: the adapter turns the two vector leaves
    // into one flow-style scalar and rejects duplicate keys, aliases and other
    // sequences first (resolved_yaml.hpp).
    const std::map<std::string, std::size_t> vectors = {
        {at("extrinsic_base_cam.translation"), 3}, {at("extrinsic_base_cam.rotation_xyzw"), 4}};
    const auto root = xbrain::config::parse_yaml(perception::config::adapt_yaml_vectors(bytes, vectors, kRoot, kCalibrationRecordMaxBytes));
    require(root.is_map(), "<root>", "record must be a map");
    for (const auto& item : root.items()) require(item.first == kRoot, item.first, "unknown top-level key");
    exact_keys(root, kRoot, {"schema_version", "record_id", "scope", "created_utc", "camera", "base_frame",
                            "extrinsic_base_cam", "method", "plane_fit", "tool"});
    CalibrationRecord r;
    // Only version 1 is understood: a newer record is refused, never half read.
    r.schema_version = static_cast<int>(integer(root, at("schema_version"), 1, 1000));
    require(r.schema_version == 1, at("schema_version"), "unsupported record schema version");
    r.record_id = token(root, at("record_id"), 64);
    // Scope and base frame must agree (vehicle <-> base_link, bench <-> bench).
    r.scope = one_of(root, at("scope"), {"vehicle", "bench"}) == "bench" ? CalibrationScope::Bench : CalibrationScope::Vehicle;
    // Creation time, UTC, second resolution (audit only).
    r.created_utc = text(root, at("created_utc"), 20);
    require(utc_stamp(r.created_utc), at("created_utc"), "expected YYYY-MM-DDTHH:MM:SSZ");

    // The camera the record belongs to; the serial is compared at verification.
    exact_keys(root, at("camera"), {"driver", "serial", "depth_stream"});
    r.camera_driver = one_of(root, at("camera.driver"), {"orbbec_gemini338le"});
    r.camera_serial = token(root, at("camera.serial"), 64);
    r.depth_stream = one_of(root, at("camera.depth_stream"), {kCalibrationDepthStream});

    exact_keys(root, at("base_frame"), {"name", "origin"});
    r.base_frame = one_of(root, at("base_frame.name"), {"base_link", "bench"});
    r.base_origin = text(root, at("base_frame.origin"), 256);
    require((r.scope == CalibrationScope::Bench) == (r.base_frame == "bench"), at("base_frame.name"),
            "vehicle scope uses base_link; bench scope uses bench");

    // The transform itself: translation within 10 m, a unit quaternion.
    exact_keys(root, at("extrinsic_base_cam"), {"translation", "rotation_xyzw"});
    r.translation = vector<3>(root, at("extrinsic_base_cam.translation"));
    for (double v : r.translation)
        require(std::abs(v) <= 10.0, at("extrinsic_base_cam.translation"), "component beyond 10 m");
    r.rotation_xyzw = vector<4>(root, at("extrinsic_base_cam.rotation_xyzw"));
    const auto& q = r.rotation_xyzw;
    const double n2 = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    require(std::abs(n2 - 1.0) <= 1e-9, at("extrinsic_base_cam.rotation_xyzw"), "must be unit length");

    // How each part was measured; a vehicle record may not use bench axes.
    exact_keys(root, at("method"), {"plane", "yaw", "xy"});
    r.method_plane = one_of(root, at("method.plane"), {"depth_plane_fit"});
    r.method_yaw = one_of(root, at("method.yaw"), {"straight_edge_reference", "bench_axis_definition"});
    r.method_xy = one_of(root, at("method.xy"), {"tape_measure", "bench_axis_definition"});
    if (r.scope == CalibrationScope::Vehicle) {
        require(r.method_yaw != "bench_axis_definition", at("method.yaw"), "bench axis definition in a vehicle record");
        require(r.method_xy != "bench_axis_definition", at("method.xy"), "bench axis definition in a vehicle record");
    }

    // Ground-plane fit statistics; the RMS is checked against residual_max_m.
    exact_keys(root, at("plane_fit"), {"frames", "inliers", "inlier_fraction", "plane_fit_inlier_rms_m"});
    r.plane_frames = integer(root, at("plane_fit.frames"), 1, 100000);
    r.plane_inliers = integer(root, at("plane_fit.inliers"), 3, std::numeric_limits<long>::max());
    r.plane_inlier_fraction = number(root, at("plane_fit.inlier_fraction"));
    require(r.plane_inlier_fraction > 0 && r.plane_inlier_fraction <= 1, at("plane_fit.inlier_fraction"),
            "must be in (0, 1]");
    r.plane_fit_inlier_rms_m = number(root, at("plane_fit.plane_fit_inlier_rms_m"));
    require(r.plane_fit_inlier_rms_m >= 0 && r.plane_fit_inlier_rms_m <= 1, at("plane_fit.plane_fit_inlier_rms_m"),
            "must be in [0, 1] metres");

    // Which tool wrote the record, for traceability of how the values were produced.
    exact_keys(root, at("tool"), {"name", "version"});
    r.tool_name = token(root, at("tool.name"), 64);
    r.tool_version = token(root, at("tool.version"), 32);
    return r;
}

// The canonical byte form the calibration tools write; parse(format(r)) == r.
std::string format_calibration_record(const CalibrationRecord& r) {
    std::ostringstream o;
    // Vectors in the flow form the parser accepts, numbers in round-trip form.
    const auto vec = [](const double* v, std::size_t n) {
        std::string s = "[";
        for (std::size_t i = 0; i < n; ++i) s += (i ? ", " : "") + round_trip(v[i]);
        return s + "]";
    };
    // Fixed key order and indentation: the same record always gives the same bytes, so its
    // SHA-256 in the configuration is stable across re-formatting.
    o << "calib_record:\n"
      << "  schema_version: " << r.schema_version << "\n"
      << "  record_id: " << r.record_id << "\n"
      << "  scope: " << (r.scope == CalibrationScope::Bench ? "bench" : "vehicle") << "\n"
      << "  created_utc: " << r.created_utc << "\n"
      << "  camera:\n"
      << "    driver: " << r.camera_driver << "\n"
      << "    serial: " << r.camera_serial << "\n"
      << "    depth_stream: " << r.depth_stream << "\n"
      << "  base_frame:\n"
      << "    name: " << r.base_frame << "\n"
      << "    origin: " << r.base_origin << "\n"
      << "  extrinsic_base_cam:\n"
      << "    translation: " << vec(r.translation.data(), 3) << "\n"
      << "    rotation_xyzw: " << vec(r.rotation_xyzw.data(), 4) << "\n"
      << "  method:\n"
      << "    plane: " << r.method_plane << "\n"
      << "    yaw: " << r.method_yaw << "\n"
      << "    xy: " << r.method_xy << "\n"
      << "  plane_fit:\n"
      << "    frames: " << r.plane_frames << "\n"
      << "    inliers: " << r.plane_inliers << "\n"
      << "    inlier_fraction: " << round_trip(r.plane_inlier_fraction) << "\n"
      << "    plane_fit_inlier_rms_m: " << round_trip(r.plane_fit_inlier_rms_m) << "\n"
      << "  tool:\n"
      << "    name: " << r.tool_name << "\n"
      << "    version: " << r.tool_version << "\n";
    return o.str();
}

// Stable names for logs and config_digest.
const char* calibration_code_name(CalibrationCode code) noexcept {
    switch (code) {
    case CalibrationCode::NoTransform: return "no_transform";
    case CalibrationCode::TransformOnly: return "transform_only";
    case CalibrationCode::Verified: return "verified";
    case CalibrationCode::RecordUnreadable: return "record_unreadable";
    case CalibrationCode::ShaMismatch: return "sha_mismatch";
    case CalibrationCode::SchemaInvalid: return "schema_invalid";
    case CalibrationCode::CameraMismatch: return "camera_mismatch";
    case CalibrationCode::ExtrinsicMismatch: return "extrinsic_mismatch";
    case CalibrationCode::ResidualExceeded: return "residual_exceeded";
    case CalibrationCode::BenchScope: return "bench_scope";
    }
    return "unknown";
}

// Read the file ONCE into memory (absolute path, regular file, bounded size);
// the same bytes are hashed and parsed, so they cannot differ between the two.
bool read_calibration_file(const std::string& path, std::size_t max_bytes, std::string& bytes, std::string& why) {
    bytes.clear();
    if (path.empty() || path.front() != '/') {
        why = "path must be absolute";
        return false;
    }
    struct stat st {};
    if (::stat(path.c_str(), &st) != 0 || !S_ISREG(st.st_mode)) {
        why = "not a readable regular file";
        return false;
    }
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        why = "open failed";
        return false;
    }
    // Read one byte more than allowed to detect an oversize file.
    std::string data;
    data.resize(max_bytes + 1);
    in.read(&data[0], static_cast<std::streamsize>(data.size()));
    const auto got = static_cast<std::size_t>(in.gcount());
    if (in.bad()) {
        why = "read failed";
        return false;
    }
    if (got > max_bytes) {
        why = "record exceeds size bound";
        return false;
    }
    data.resize(got);
    bytes.swap(data);
    return true;
}

CalibrationVerdict CalibrationVerifier::verify(const ExtrinsicConfig& e, const std::string& camera_serial,
                                               const CalibrationFileReader& reader) {
    // Verdict chain; the first failed rule decides the code.
    CalibrationVerdict v;
    // No configured transform at all: the unconfigured Profile runs, nothing to verify.
    if (e.mode != ExtrinsicMode::TransformConfigured || !e.translation || !e.rotation_xyzw) {
        v.code_ = CalibrationCode::NoTransform;
        return v;
    }
    // A transform without any record reference: usable, uncertified (Q12).
    if (!e.calib_record && !e.calib_sha256 && !e.residual_max_m) {
        v.code_ = CalibrationCode::TransformOnly;
        return v;
    }
    // Detail is kept on one line for the log and the event detail.
    const auto fail = [&v](CalibrationCode code, std::string detail) {
        v.code_ = code;
        for (auto& c : detail)
            if (c == '\n' || c == '\r') c = ' ';
        v.detail_ = std::move(detail);
        return v;
    };
    // A partial reference (some of the three keys set) is a configuration mistake, never a
    // weaker certification.
    if (!e.calib_record || !e.calib_sha256 || !e.residual_max_m)
        return fail(CalibrationCode::SchemaInvalid, "calib_record, calib_sha256 and residual_max_m must all be set");
    v.record_path_ = *e.calib_record;
    std::string bytes, why;
    if (!reader || !reader(*e.calib_record, kCalibrationRecordMaxBytes, bytes, why))
        return fail(CalibrationCode::RecordUnreadable, why.empty() ? "reader failed" : why);
    // sha256 over ALL raw bytes: the configuration pins exactly this file.
    v.record_sha256_ = yolo::sha256_hex(bytes);
    if (v.record_sha256_ != *e.calib_sha256)
        return fail(CalibrationCode::ShaMismatch, "record sha256 differs from perception.extrinsic_base_cam.calib_sha256");
    CalibrationRecord record;
    try {
        record = parse_calibration_record(bytes);
    } catch (const std::exception& ex) {
        return fail(CalibrationCode::SchemaInvalid, ex.what());
    }
    v.record_id_ = record.record_id;
    v.inlier_rms_m_ = record.plane_fit_inlier_rms_m;
    // A record of another camera cannot certify this one.
    if (record.camera_serial != camera_serial)
        return fail(CalibrationCode::CameraMismatch, "record camera.serial differs from perception.pipeline.camera.serial");
    // Exact equality: both sides are the IEEE doubles of the same decimal text.
    if (record.translation != *e.translation || record.rotation_xyzw != *e.rotation_xyzw)
        return fail(CalibrationCode::ExtrinsicMismatch, "record transform differs from perception.extrinsic_base_cam");
    // Plane-fit quality in metres against the configured bound ("!(<=)" also
    // rejects NaN).
    if (!(record.plane_fit_inlier_rms_m <= *e.residual_max_m))
        return fail(CalibrationCode::ResidualExceeded, "plane_fit_inlier_rms_m " + round_trip(record.plane_fit_inlier_rms_m) +
                                                           " > residual_max_m " + round_trip(*e.residual_max_m));
    // Everything matches, but a bench record cannot certify the vehicle (Q13).
    if (record.scope == CalibrationScope::Bench)
        return fail(CalibrationCode::BenchScope, "bench record verifies components only (Q13); not vehicle calibration");
    v.code_ = CalibrationCode::Verified;
    return v;
}

}  // namespace perception::service
