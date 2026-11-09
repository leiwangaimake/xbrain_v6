/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: calibration_record.hpp
 * Brief: Versioned camera-extrinsic calibration record and the extrinsic_calibrated verdict.
 *
 * Description:
 * 19 section 7.3, reply 2026-09-17 Q11/Q12.
 * Why this exists. extrinsic_calibrated=false makes RNS refuse autonomous navigation
 * (11 section 3.1B.4), so a true value must be earned by evidence, never set by hand.
 * The record schema is a perception-side implementation convention (Q11), not
 * an upstream-frozen format. Verdict rule, all of which must hold:
 *   - perception.extrinsic_base_cam transform AND record path/sha/threshold set;
 *   - the record file (absolute path, regular file, <= 64 KiB) is read ONCE;
 *     sha256 over ALL its raw bytes equals calib_sha256 (no self-contained hash);
 *   - the SAME bytes parse as schema_version 1;
 *   - record camera serial equals perception.pipeline.camera.serial;
 *   - record transform equals the configured transform exactly (IEEE double of
 *     each decimal text; tools write 17 significant digits);
 *   - plane_fit.plane_fit_inlier_rms_m <= residual_max_m (metres, plane inlier
 *     RMS; CAL-13 pixel reprojection error is NOT a substitute);
 *   - scope is vehicle. A bench record verifies components only (Q13) and is
 *     reported BenchScope with calibrated=false until ruled otherwise.
 * There is no configuration key or API that forces calibrated=true: the only
 * producer of a true verdict is verify_calibration().
 */
#pragma once

#include "service/service_config.hpp"

#include <array>
#include <cstddef>
#include <functional>
#include <optional>
#include <string>

namespace perception::service {

// A record is a small text file; the bound keeps a wrong path from being slurped.
inline constexpr std::size_t kCalibrationRecordMaxBytes = 64U * 1024U;
// The only depth stream the geometry is built for (native 640x400 Y16 at 30 fps).
inline constexpr const char* kCalibrationDepthStream = "640x400_y16_30";

// Vehicle: base_link on the robot. Bench: a bench frame (component check only).
enum class CalibrationScope { Vehicle, Bench };

// The parsed record, field for field (schema_version 1).
struct CalibrationRecord {
    int schema_version = 0;
    std::string record_id;
    CalibrationScope scope = CalibrationScope::Vehicle;
    std::string created_utc;
    std::string camera_driver, camera_serial, depth_stream;
    std::string base_frame, base_origin;
    std::array<double, 3> translation{};
    std::array<double, 4> rotation_xyzw{};
    std::string method_plane, method_yaw, method_xy;
    long plane_frames = 0, plane_inliers = 0;
    double plane_inlier_fraction = 0, plane_fit_inlier_rms_m = 0;
    std::string tool_name, tool_version;
};

// Parses record bytes. Throws std::invalid_argument("<dotted path>: why").
CalibrationRecord parse_calibration_record(const std::string& bytes);
// Canonical text a calibration tool writes for `record` (round-trip doubles).
std::string format_calibration_record(const CalibrationRecord& record);

enum class CalibrationCode {
    NoTransform,        // extrinsic absent: no geometry at all
    TransformOnly,      // transform configured, record fields null (Q12 compute, uncertified)
    Verified,           // the only calibrated=true outcome
    RecordUnreadable, ShaMismatch, SchemaInvalid, CameraMismatch,
    ExtrinsicMismatch, ResidualExceeded, BenchScope
};
const char* calibration_code_name(CalibrationCode) noexcept;

class CalibrationVerdict {
public:
    bool calibrated() const noexcept { return code_ == CalibrationCode::Verified; }
    CalibrationCode code() const noexcept { return code_; }
    // Single-line reason (empty for NoTransform/TransformOnly/Verified).
    const std::string& detail() const noexcept { return detail_; }
    const std::string& record_path() const noexcept { return record_path_; }
    // Actual sha256 of the bytes read (empty when not read).
    const std::string& record_sha256() const noexcept { return record_sha256_; }
    const std::string& record_id() const noexcept { return record_id_; }
    std::optional<double> inlier_rms() const noexcept { return inlier_rms_m_; }

private:
    friend struct CalibrationVerifier;
    CalibrationVerdict() = default;
    CalibrationCode code_ = CalibrationCode::NoTransform;
    std::string detail_, record_path_, record_sha256_, record_id_;
    std::optional<double> inlier_rms_m_;
};

// Reads at most max_bytes from an absolute path; returns false with `why` set.
using CalibrationFileReader =
    std::function<bool(const std::string& path, std::size_t max_bytes, std::string& bytes, std::string& why)>;
bool read_calibration_file(const std::string& path, std::size_t max_bytes, std::string& bytes, std::string& why);

struct CalibrationVerifier {
    static CalibrationVerdict verify(const ExtrinsicConfig& extrinsic, const std::string& camera_serial,
                                     const CalibrationFileReader& reader = read_calibration_file);
};

inline CalibrationVerdict verify_calibration(const ExtrinsicConfig& extrinsic, const std::string& camera_serial,
                                            const CalibrationFileReader& reader = read_calibration_file) {
    return CalibrationVerifier::verify(extrinsic, camera_serial, reader);
}

}  // namespace perception::service
