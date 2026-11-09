/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_contract.hpp
 * Brief: The three R6 perception bodies (Profile, Objects, Status) and their bounded JSON encoders.
 *
 * Description:
 * What problem this solves. RNS accepts or rejects each perception message as a whole
 * (11 section 3.1B), so a body that breaks one field rule must never be sent. This file is the
 * single definition of the wire form: plain structs the owners fill, and encoders that check
 * every 11 section 3.1B.1 (Profile), 3.1B.2 (Objects) and 3.1B.3 (Status) rule before writing
 * JSON into caller-owned storage.
 *
 * Boundaries. Transport-independent: no Zenoh, no clock, no allocation while encoding (the RT
 * publishing path calls it). Units: base_link metres, with d_free/d_block/range_max_m/
 * blind_near_m as horizontal radial distance hypot(x, y); CLOCK_MONOTONIC milliseconds.
 * Calibration and frame identity are facts the caller supplies; the encoder only checks that
 * the body is self-consistent.
 *
 * Nullable fields (11 v2.3). Unknown is null, never a sentinel or an old value: blind_near_m,
 * the four Objects geometry fields together, velocity_xy, and Status statistics without samples.
 */
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace perception::rns {
using Millis = std::int64_t;
// Profile src bits (11 section 3.1B.1, src per-bit definition v2.1):
//   kT  bit0  traversable segmentation endorses [blind_near_m, d_free);
//   kG  bit1  geometry had a valid sample in this bin in this frame;
//   kS  bit2  d_block was produced or tightened by semantic injection;
//   kNegative bit3  d_block is a negative-obstacle boundary (h_block <= 0 or null).
constexpr std::uint8_t kT = 1, kG = 2, kS = 4, kNegative = 8;
// Storage limits, NOT measured camera geometry or a truncation policy.
constexpr std::size_t kMaxBins = 1024, kMaxObjects = 512;

// InvalidInput: a contract rule failed. Capacity: the output buffer is too small.
enum class Error { None, InvalidInput, Capacity };
// Result of one encode; size excludes the terminating NUL and is 0 on failure.
struct Encoded {
    Error error;
    std::size_t size;
    explicit operator bool() const { return error == Error::None; }
};
// One angular bin. d_free null = no confirmed FREE; d_block null = no obstacle
// within range_max_m (never +inf, JSON has no such literal); h_block = visible
// height of the blocking cluster (a lower bound). terrain 0..4 (unknown, hard,
// soft, rough, slope); slope_deg is the local slope change of the FREE segment
// relative to the fitted ground plane, not an absolute slope (11 v2.1).
struct Bin {
    std::optional<float> d_free, d_block, h_block;
    std::uint8_t src = 0, conf = 0, terrain = 0;
    std::optional<int> slope_deg;
};
// ProfileMsg (11 section 3.1B.1). t_capture is its identity time for acceptance
// and ordering; t_publish is stamped just before put (cadence measurement).
// angle_min_rad is the CENTRE angle of bin 0; bins are angle_step_rad apart.
struct Profile {
    Millis t_capture_mono_ms = -1, t_publish_mono_ms = -1;
    bool extrinsic_calibrated = false;
    // Segmentation frame behind the T evidence; null = no T evidence, all bit0 clear.
    std::optional<Millis> t_seg_mono_ms;
    float z_pass_m = 0, angle_min_rad = 0, angle_step_rad = 0;
    float range_max_m = 0;
    // R6: no verifiable ground means unknown, never a configured/old distance.
    // A null blind range requires every d_free=null and every kT bit clear.
    std::optional<float> blind_near_m;
    std::vector<Bin> bins;
    // First slice supports pose_used=null only. It does not infer odometry.
};
// base_link x/y in metres.
struct Point { float x, y; };
struct ObjectGeometry {
    // One measurement bundle: no independently nullable footprint/height/range.
    // r_near is derived from this same footprint during encoding.
    std::vector<Point> footprint_xy;
    float z_min, z_max;
    ObjectGeometry(std::vector<Point> footprint, float minimum, float maximum)
        : footprint_xy(std::move(footprint)), z_min(minimum), z_max(maximum) {}
};
struct Object {
    std::int64_t track_id = -1;
    int class_id = -1;
    std::string class_name;
    float confidence = 0;
    // Closed set: proxy | confirmed.
    std::string semantic_status;
    // R6: absence preserves the detection, with all four geometry fields null.
    // A present bundle is measured base_link geometry, not an image-box fallback.
    std::optional<ObjectGeometry> geometry;
    // Unknown is null, NOT an invented zero speed. Numeric-but-invalid raw
    // estimates are retained. Null requires false + warming_up/timestamp_gap.
    std::optional<std::array<float, 2>> velocity_xy;
    // velocity_frame raw | ego_removed; velocity_status is the five-value closed
    // set of 11 section 3.1B.2 v1.7; depth_quality good | fair | poor.
    std::string velocity_frame, velocity_status, depth_quality;
    bool velocity_valid = false;
    std::uint64_t stable_frames = 0;
    // last_seen must equal the message's t_capture: only observed objects are sent.
    Millis first_seen_mono_ms = -1, last_seen_mono_ms = -1;
};
// ObjectsMsg (11 section 3.1B.2): the complete object list of one inference frame.
struct Objects {
    Millis t_capture_mono_ms = -1, t_publish_mono_ms = -1;
    bool extrinsic_calibrated = false;
    std::vector<Object> objects;
};
// StatusMsg (11 section 3.1B.3), the 1 Hz heartbeat. It has no capture time:
// t_publish is its identity time (11 section 3.1B.5 acceptance rule).
struct Status {
    Millis t_publish_mono_ms = -1;
    double fps_depth = 0, fps_infer = 0;
    // R6: no samples are unknown, not a fabricated healthy zero.
    std::optional<double> latency_ms_p50, latency_ms_p99;
    std::optional<double> infer_gap_ms_p99, infer_gap_ms_max;
    std::optional<double> invalid_pixel_ratio;
    int ground_seg_level = 0;
    bool extrinsic_calibrated = false, traversable_seg_available = false;
    std::vector<std::string> degraded_reasons;
};

// No allocations in encoding: caller owns inputs and reusable output storage.
// Failure clears output[0] and returns size=0; partial JSON must never be sent.
Encoded encode(const Profile&, char* output, std::size_t capacity);
Encoded encode(const Objects&, char* output, std::size_t capacity);
Encoded encode(const Status&, char* output, std::size_t capacity);
// Explicit publish-time override, without copying or mutating the input body.
// Intended for the publisher's late-stamping adapter; all other checks remain.
Encoded encode_at_publish(const Profile&, Millis publish, char*, std::size_t);
Encoded encode_at_publish(const Objects&, Millis publish, char*, std::size_t);
Encoded encode_at_publish(const Status&, Millis publish, char*, std::size_t);

// r_near (11 section 3.1B.2 v1.7): minimum Euclidean distance from the base_link
// origin to the polygon, measured to edges, 0 when the origin is inside; NaN for
// a polygon valid_footprint rejects.
double footprint_near(const std::vector<Point>& ccw_convex);
// The exact footprint rule the Objects encoder applies (3..12 finite vertices,
// no repeated consecutive vertex, CCW convex with at least one strict turn).
bool valid_footprint(const std::vector<Point>& footprint);
}  // namespace perception::rns
