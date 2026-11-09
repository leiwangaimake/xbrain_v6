/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_contract.cpp
 * Brief: Contract validation and bounded, allocation-free JSON encoding of Profile/Objects/Status.
 *
 * Description:
 * What problem this solves. Implements the encoders declared in rns_contract.hpp. Each encoder
 * first runs the full 11 section 3.1B rule set for its body (valid(...) below) and only then
 * writes; RNS rejects a whole message on any rule violation, so checking here turns a silent
 * downstream rejection into a local, counted encode failure.
 *
 * Why a hand-written writer. The RT publishing path must not allocate, and the message sizes
 * are bounded (kMaxBins, kMaxObjects), so the JSON is written straight into caller-owned
 * storage. Numbers use std::to_chars (locale-independent, shortest round-trip form), which a
 * printf-style formatter would not guarantee.
 *
 * Failure rule. On any error the output is reset to an empty string and size 0 is returned:
 * partial JSON must never reach the transport.
 */
#include "output/rns_contract.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstring>
#include <limits>
#include <string_view>
#include <system_error>

namespace perception::rns {
namespace {
bool finite(double x) { return std::isfinite(x); }
bool positive(double x) { return finite(x) && x > 0; }
bool nonnegative(double x) { return finite(x) && x >= 0; }
// The identity (capture) time must exist and publish cannot precede it.
bool stamp(Millis capture, Millis publish) { return capture >= 0 && publish >= capture; }
bool optional_finite(const std::optional<float>& x) { return !x || finite(*x); }

// Strict UTF-8 (no overlong forms, no surrogates, at most U+10FFFF). JSON text
// must be valid UTF-8; class names come from a model schema file, and one bad
// byte would make the whole message unparseable at RNS.
bool utf8(std::string_view s) {
    for (std::size_t i = 0; i < s.size();) {
        const auto c = static_cast<unsigned char>(s[i++]);
        // ASCII needs no further bytes.
        if (c < 0x80) continue;
        // The lead byte fixes the sequence length and the smallest code point that
        // length may encode; anything smaller is an overlong form (0xc0/0xc1 and
        // 0xf5.. can never start a valid sequence).
        unsigned need = 0, value = 0, minimum = 0;
        if (c >= 0xc2 && c <= 0xdf) { need = 1; value = c & 31; minimum = 0x80; }
        else if (c >= 0xe0 && c <= 0xef) { need = 2; value = c & 15; minimum = 0x800; }
        else if (c >= 0xf0 && c <= 0xf4) { need = 3; value = c & 7; minimum = 0x10000; }
        else return false;
        if (s.size() - i < need) return false;
        // Continuation bytes are 10xxxxxx, 6 payload bits each.
        for (unsigned j = 0; j < need; ++j) {
            const auto v = static_cast<unsigned char>(s[i++]);
            if ((v & 0xc0) != 0x80) return false;
            value = (value << 6) | (v & 63);
        }
        // Overlong, beyond Unicode, or a UTF-16 surrogate half.
        if (value < minimum || value > 0x10ffff || (value >= 0xd800 && value <= 0xdfff)) return false;
    }
    return true;
}

// Append-only writer into caller storage. The first error sticks and every later
// call is a no-op, so each encoder is straight-line code checked once in finish().
class Writer {
public:
    // A null or empty buffer cannot even hold the NUL: capacity error from the start.
    Writer(char* output, std::size_t capacity) : out_(output), capacity_(capacity) {
        if (!output || !capacity) error_ = Error::Capacity;
        else output[0] = '\0';
    }
    // ">=" keeps one byte free for the terminating NUL.
    void raw(std::string_view s) {
        if (error_ != Error::None) return;
        if (s.size() >= capacity_ - used_) { error_ = Error::Capacity; return; }
        std::memcpy(out_ + used_, s.data(), s.size()); used_ += s.size();
    }
    // JSON string: quote and backslash escaped, control characters as six-byte unicode escapes,
    // everything else (already checked UTF-8) copied byte for byte.
    void quoted(std::string_view s) {
        if (!utf8(s)) { error_ = Error::InvalidInput; return; }
        raw("\"");
        constexpr char hex[] = "0123456789abcdef";
        for (char c : s) {
            const auto u = static_cast<unsigned char>(c);
            if (c == '"' || c == '\\') { const char e[] = {'\\', c}; raw({e, 2}); }
            else if (u < 32) { const char e[] = {'\\', 'u', '0', '0', hex[u >> 4], hex[u & 15]}; raw({e, 6}); }
            else raw({&c, 1});
        }
        raw("\"");
    }
    // NaN and infinity have no JSON literal, so they are an input error, never
    // written as a sentinel. max_digits10 makes the text round-trip to the same
    // double (float fields widen exactly).
    void number(double v) {
        if (!finite(v)) { error_ = Error::InvalidInput; return; }
        char temp[64];
        const auto result = std::to_chars(temp, temp + sizeof(temp), v, std::chars_format::general,
                                          std::numeric_limits<double>::max_digits10);
        if (result.ec != std::errc{}) { error_ = Error::InvalidInput; return; }
        raw({temp, static_cast<std::size_t>(result.ptr - temp)});
    }
    // Integers are written exactly, never through double, so 64-bit times and
    // ids keep every digit.
    template <typename T> void integer(T v) {
        char temp[32];
        const auto result = std::to_chars(temp, temp + sizeof(temp), v);
        if (result.ec != std::errc{}) { error_ = Error::InvalidInput; return; }
        raw({temp, static_cast<std::size_t>(result.ptr - temp)});
    }
    void boolean(bool b) { raw(b ? "true" : "false"); }
    // Absent value = JSON null (11 v2.3: unknown is null, never a stand-in value).
    template <typename T> void nullable(const std::optional<T>& n) { if (n) number(*n); else raw("null"); }
    // Terminate the text, or on any error reset it to "" and report size 0.
    Encoded finish(bool valid) {
        if (!valid) error_ = Error::InvalidInput;
        if (out_ && capacity_) out_[error_ == Error::None ? used_ : 0] = '\0';
        return {error_, error_ == Error::None ? used_ : 0};
    }
private:
    char* out_;
    // used_ = bytes written so far, excluding the NUL.
    std::size_t capacity_, used_ = 0;
    Error error_ = Error::None;
};

// Profile rules (11 section 3.1B.1). Header: capture before publish, 1..kMaxBins
// bins, positive pass height and step, range in (0, 6] m (range_max_m must not
// exceed the sensor's recommended range; 6.0 is the contract's value, beyond it
// depth error grows with z^2), a known blind range inside the range, and the
// segmentation frame not after the capture.
bool valid(const Profile& p, Millis publish) {
    if (!stamp(p.t_capture_mono_ms, publish) || p.bins.empty() || p.bins.size() > kMaxBins ||
        !positive(p.z_pass_m) || !finite(p.angle_min_rad) || !positive(p.angle_step_rad) ||
        !positive(p.range_max_m) || p.range_max_m > 6 ||
        (p.blind_near_m && (!nonnegative(*p.blind_near_m) || *p.blind_near_m > p.range_max_m)) ||
        (p.t_seg_mono_ms && (*p.t_seg_mono_ms < 0 || *p.t_seg_mono_ms > p.t_capture_mono_ms))) return false;
    for (const auto& b : p.bins) {
        // v2.3 I: no verifiable ground (null blind range) allows no FREE and no bit0.
        if (!p.blind_near_m && (b.d_free || (b.src & kT))) return false;
        // Field ranges: finite floats, only the four defined src bits, terrain
        // 0..4, slope 0..90 degrees.
        if (!optional_finite(b.d_free) || !optional_finite(b.d_block) || !optional_finite(b.h_block) ||
            b.src > 15 || b.terrain > 4 || (b.slope_deg && (*b.slope_deg < 0 || *b.slope_deg > 90))) return false;
        // Distances within [0, range_max_m], and FREE never beyond the first block:
        // everything behind d_block is UNKNOWN (11 occlusion rule).
        if (b.d_free && (*b.d_free < 0 || *b.d_free > p.range_max_m)) return false;
        if (b.d_block && (*b.d_block < 0 || *b.d_block > p.range_max_m)) return false;
        if (b.d_free && b.d_block && *b.d_free > *b.d_block) return false;
        // bit1 = 0 (no geometry sample, v2.2): no FREE, no height, no T or NEG;
        // a block can then only come from semantic injection (bit2).
        if (!(b.src & kG) && (b.d_free || b.h_block || (b.src & (kT | kNegative)))) return false;
        if (!(b.src & kG) && b.d_block && !(b.src & kS)) return false;
        // No block: no height and no S/NEG bits (they describe a block).
        if (!b.d_block && (b.h_block || (b.src & (kS | kNegative)))) return false;
        // A semantic block carries no height and is not a negative obstacle.
        if ((b.src & kS) && ((b.src & kNegative) || b.h_block)) return false;
        // NEG: height <= 0 or null; otherwise a height is never negative.
        if ((b.src & kNegative) && b.h_block && *b.h_block > 0) return false;
        if (!(b.src & kNegative) && b.h_block && *b.h_block < 0) return false;
        // bit0 T: needs a segmentation frame, a known blind range and a FREE
        // endpoint strictly beyond it (d_free = blind_near_m means bit0 = 0).
        if ((b.src & kT) && (!p.t_seg_mono_ms || !p.blind_near_m || !b.d_free ||
                           *b.d_free <= *p.blind_near_m)) return false;
        // slope_deg describes the FREE segment, so it needs one.
        if (!b.d_free && b.slope_deg) return false;
    }
    return true;
}

// z of (b - a) x (c - a) in double: > 0 when c is left of a->b.
double cross(Point a, Point b, Point c) {
    return (static_cast<double>(b.x) - a.x) * (static_cast<double>(c.y) - a.y) -
           (static_cast<double>(b.y) - a.y) * (static_cast<double>(c.x) - a.x);
}
// Footprint rule: 3..12 finite vertices (12 is RNS's per-cycle real-time bound,
// 11 section 3.1B.2), no repeated consecutive vertex, CCW convex, not all collinear.
bool polygon(const std::vector<Point>& p) {
    if (p.size() < 3 || p.size() > 12) return false;
    for (const auto& v : p) if (!finite(v.x) || !finite(v.y)) return false;
    bool non_collinear = false;
    for (std::size_t i = 0; i < p.size(); ++i) {
        const auto& a = p[i]; const auto& b = p[(i + 1) % p.size()];
        if (a.x == b.x && a.y == b.y) return false;
        // All vertices must lie on the left of every edge: rejects CW,
        // concavity, self-intersection and backtracking, not just local turns.
        for (const auto& c : p) {
            const double value = cross(a, b, c);
            if (value < 0) return false;
            non_collinear |= value > 0;
        }
    }
    return non_collinear;
}
// Objects rules (11 section 3.1B.2).
bool valid(const Objects& m, Millis publish) {
    if (!stamp(m.t_capture_mono_ms, publish) || m.objects.size() > kMaxObjects) return false;
    for (std::size_t i = 0; i < m.objects.size(); ++i) {
        const auto& o = m.objects[i];
        // Identity and class: non-negative ids, a UTF-8 class name that is not the
        // traversable class (forbidden as an object, v2.1: T travels in the profile),
        // confidence in [0, 1], observation times ending at this capture.
        if (o.track_id < 0 || o.class_id < 0 || o.class_name.empty() || o.class_name == "traversable_area" ||
            !utf8(o.class_name) || !nonnegative(o.confidence) || o.confidence > 1 ||
            o.first_seen_mono_ms < 0 || o.last_seen_mono_ms < o.first_seen_mono_ms ||
            o.last_seen_mono_ms != m.t_capture_mono_ms) return false;
        // Geometry is one bundle (v2.3 IV); an unlocatable object carries no velocity.
        if (o.geometry) {
            const auto& g = *o.geometry;
            if (!polygon(g.footprint_xy) || !finite(g.z_min) || !finite(g.z_max) || g.z_min > g.z_max) return false;
        } else if (o.velocity_xy || o.velocity_valid) return false;
        // velocity_xy null means no estimate: then valid=false and status
        // warming_up or timestamp_gap (v2.3 II).
        if (o.velocity_xy) {
            if (!finite((*o.velocity_xy)[0]) || !finite((*o.velocity_xy)[1])) return false;
        } else if (o.velocity_valid || (o.velocity_status != "warming_up" && o.velocity_status != "timestamp_gap"))
            return false;
        // Closed value sets. ego_removed is refused without a calibrated extrinsic:
        // removing ego motion needs the camera -> base_link transform.
        if (o.semantic_status != "confirmed" && o.semantic_status != "proxy") return false;
        if (o.velocity_frame != "raw" && o.velocity_frame != "ego_removed") return false;
        if (!m.extrinsic_calibrated && o.velocity_frame == "ego_removed") return false;
        if (o.velocity_status != "warming_up" && o.velocity_status != "motion_unconfirmed" &&
            o.velocity_status != "moving" && o.velocity_status != "static" && o.velocity_status != "timestamp_gap") return false;
        if (o.depth_quality != "good" && o.depth_quality != "fair" && o.depth_quality != "poor") return false;
        // Track ids are unique within one message.
        for (std::size_t j = 0; j < i; ++j) if (m.objects[j].track_id == o.track_id) return false;
    }
    return true;
}
// Status rules (11 section 3.1B.3): rates non-negative; each statistic is null when
// its window had no samples (v2.3 III), p50/p99 null together and ordered, a gap
// p99 needs its max; invalid_pixel_ratio is a fraction; reasons are UTF-8.
bool valid(const Status& s, Millis publish) {
    const auto optional_nonnegative = [](const std::optional<double>& x) { return !x || nonnegative(*x); };
    if (publish < 0 || !nonnegative(s.fps_depth) || !nonnegative(s.fps_infer) ||
        !optional_nonnegative(s.latency_ms_p50) || !optional_nonnegative(s.latency_ms_p99) ||
        s.latency_ms_p50.has_value() != s.latency_ms_p99.has_value() ||
        (s.latency_ms_p50 && *s.latency_ms_p50 > *s.latency_ms_p99) ||
        !optional_nonnegative(s.infer_gap_ms_p99) || !optional_nonnegative(s.infer_gap_ms_max) ||
        (s.infer_gap_ms_p99 && (!s.infer_gap_ms_max || *s.infer_gap_ms_p99 > *s.infer_gap_ms_max)) ||
        !optional_nonnegative(s.invalid_pixel_ratio) ||
        (s.invalid_pixel_ratio && *s.invalid_pixel_ratio > 1) || s.ground_seg_level < 0) return false;
    for (const auto& r : s.degraded_reasons) if (!utf8(r)) return false;
    return true;
}
// Common head of Profile and Objects. extrinsic_calibrated=false makes RNS refuse
// autonomous navigation (11 section 3.1B.4), so it is always written explicitly.
void header(Writer& w, const char* schema, Millis capture, Millis publish, bool calibrated) {
    w.raw("{\"schema\":"); w.quoted(schema);
    w.raw(",\"t_capture_mono_ms\":"); w.integer(capture);
    w.raw(",\"t_publish_mono_ms\":"); w.integer(publish);
    w.raw(",\"frame\":\"base_link\",\"extrinsic_calibrated\":"); w.boolean(calibrated);
}
}  // namespace

// The encoder's own footprint rule, exported so the producers (object geometry, fusion) can
// handle a bad polygon themselves (fallback rectangle, unlocatable, or a refused frame) instead
// of the whole Objects message failing to encode.
bool valid_footprint(const std::vector<Point>& p) { return polygon(p); }

double footprint_near(const std::vector<Point>& p) {
    // NaN makes the encoder fail (non-finite number) rather than send a guess.
    if (!polygon(p)) return std::numeric_limits<double>::quiet_NaN();
    double best = std::numeric_limits<double>::infinity();
    bool inside = true;
    for (std::size_t i = 0; i < p.size(); ++i) {
        const Point a = p[i], b = p[(i + 1) % p.size()];
        // For a CCW convex polygon the origin is inside iff it is left of (or on)
        // every edge.
        inside &= cross(a, b, {0, 0}) >= 0;
        // Distance from the origin to the segment a-b: project, clamp to the
        // segment, measure (edges, not only vertices, per the r_near definition).
        const double dx = static_cast<double>(b.x) - a.x, dy = static_cast<double>(b.y) - a.y;
        const double t = std::clamp(-(a.x * dx + a.y * dy) / (dx * dx + dy * dy), 0.0, 1.0);
        best = std::min(best, std::hypot(a.x + t * dx, a.y + t * dy));
    }
    // The origin (base_link) inside the footprint means the object already reaches the robot.
    return inside ? 0 : best;
}

// The three encoders validate first and write nothing for a rejected body.
Encoded encode_at_publish(const Profile& p, Millis publish, char* output, std::size_t capacity) {
    Writer w(output, capacity);
    if (!valid(p, publish)) return w.finish(false);
    header(w, "perception_profile_v1", p.t_capture_mono_ms, publish, p.extrinsic_calibrated);
    // pose_used is always null: this delivery does not use odometry.
    w.raw(",\"pose_used\":null,\"t_seg_mono_ms\":");
    if (p.t_seg_mono_ms) w.integer(*p.t_seg_mono_ms); else w.raw("null");
    w.raw(",\"z_pass_m\":"); w.number(p.z_pass_m);
    w.raw(",\"angle_min_rad\":"); w.number(p.angle_min_rad);
    w.raw(",\"angle_step_rad\":"); w.number(p.angle_step_rad);
    // n_bins is written from the array size, so the count and the seven arrays cannot disagree.
    w.raw(",\"n_bins\":"); w.integer(p.bins.size());
    w.raw(",\"range_max_m\":"); w.number(p.range_max_m);
    w.raw(",\"blind_near_m\":"); w.nullable(p.blind_near_m);
    // Seven parallel arrays of n_bins entries, in the contract's order.
    const char* keys[] = {"d_free", "d_block", "h_block", "src", "conf", "terrain", "slope_deg"};
    for (unsigned k = 0; k < 7; ++k) {
        w.raw(","); w.quoted(keys[k]); w.raw(":[");
        for (std::size_t i = 0; i < p.bins.size(); ++i) {
            if (i) w.raw(",");
            const auto& b = p.bins[i];
            switch (k) {
                case 0: w.nullable(b.d_free); break;
                case 1: w.nullable(b.d_block); break;
                case 2: w.nullable(b.h_block); break;
                case 3: w.integer(b.src); break;
                case 4: w.integer(b.conf); break;
                case 5: w.integer(b.terrain); break;
                // slope_deg: integer degrees or null (no -128 style sentinel).
                default: if (b.slope_deg) w.integer(*b.slope_deg); else w.raw("null");
            }
        }
        w.raw("]");
    }
    w.raw("}"); return w.finish(true);
}

Encoded encode_at_publish(const Objects& m, Millis publish, char* output, std::size_t capacity) {
    Writer w(output, capacity);
    if (!valid(m, publish)) return w.finish(false);
    header(w, "perception_objects_v1", m.t_capture_mono_ms, publish, m.extrinsic_calibrated);
    // One JSON object per entry, fields in the order of the contract sample.
    w.raw(",\"objects\":[");
    for (std::size_t i = 0; i < m.objects.size(); ++i) {
        if (i) w.raw(",");
        const auto& o = m.objects[i];
        w.raw("{\"track_id\":"); w.integer(o.track_id);
        w.raw(",\"class_id\":"); w.integer(o.class_id);
        w.raw(",\"class_name\":"); w.quoted(o.class_name);
        w.raw(",\"confidence\":"); w.number(o.confidence);
        w.raw(",\"semantic_status\":"); w.quoted(o.semantic_status);
        if (o.geometry) {
            const auto& g = *o.geometry;
            w.raw(",\"footprint_xy\":[");
            for (std::size_t j = 0; j < g.footprint_xy.size(); ++j) {
                if (j) w.raw(",");
                w.raw("["); w.number(g.footprint_xy[j].x);
                w.raw(","); w.number(g.footprint_xy[j].y); w.raw("]");
            }
            w.raw("],\"z_min\":"); w.number(g.z_min);
            w.raw(",\"z_max\":"); w.number(g.z_max);
            // r_near is derived from the same footprint, so the two cannot disagree.
            w.raw(",\"r_near\":"); w.number(footprint_near(g.footprint_xy));
        } else {
            // Unlocatable: all four geometry fields null together (v2.3 IV).
            w.raw(",\"footprint_xy\":null,\"z_min\":null,\"z_max\":null,\"r_near\":null");
        }
        w.raw(",\"velocity_xy\":");
        if (o.velocity_xy) {
            w.raw("["); w.number((*o.velocity_xy)[0]); w.raw(","); w.number((*o.velocity_xy)[1]); w.raw("]");
        } else {
            w.raw("null");
        }
        w.raw(",\"velocity_frame\":"); w.quoted(o.velocity_frame);
        w.raw(",\"velocity_status\":"); w.quoted(o.velocity_status);
        w.raw(",\"velocity_valid\":"); w.boolean(o.velocity_valid);
        w.raw(",\"stable_frames\":"); w.integer(o.stable_frames);
        w.raw(",\"first_seen_mono_ms\":"); w.integer(o.first_seen_mono_ms);
        w.raw(",\"last_seen_mono_ms\":"); w.integer(o.last_seen_mono_ms);
        w.raw(",\"depth_quality\":"); w.quoted(o.depth_quality); w.raw("}");
    }
    w.raw("]}"); return w.finish(true);
}

Encoded encode_at_publish(const Status& s, Millis publish, char* output, std::size_t capacity) {
    Writer w(output, capacity);
    if (!valid(s, publish)) return w.finish(false);
    // No capture time and no frame: Status is identified by its publish time.
    w.raw("{\"schema\":\"perception_status_v1\",\"t_publish_mono_ms\":"); w.integer(publish);
    w.raw(",\"fps_depth\":"); w.number(s.fps_depth);
    w.raw(",\"fps_infer\":"); w.number(s.fps_infer);
    w.raw(",\"latency_ms_p50\":"); w.nullable(s.latency_ms_p50);
    w.raw(",\"latency_ms_p99\":"); w.nullable(s.latency_ms_p99);
    w.raw(",\"infer_gap_ms_p99\":"); w.nullable(s.infer_gap_ms_p99);
    w.raw(",\"infer_gap_ms_max\":"); w.nullable(s.infer_gap_ms_max);
    w.raw(",\"invalid_pixel_ratio\":"); w.nullable(s.invalid_pixel_ratio);
    w.raw(",\"ground_seg_level\":"); w.integer(s.ground_seg_level);
    w.raw(",\"extrinsic_calibrated\":"); w.boolean(s.extrinsic_calibrated);
    w.raw(",\"traversable_seg_available\":"); w.boolean(s.traversable_seg_available);
    // Reason codes as a JSON string array; empty means no degradation reported.
    w.raw(",\"degraded_reasons\":[");
    for (std::size_t i = 0; i < s.degraded_reasons.size(); ++i) { if (i) w.raw(","); w.quoted(s.degraded_reasons[i]); }
    w.raw("]}"); return w.finish(true);
}
// Encode with the body's own t_publish (tests, tools). The publisher calls
// encode_at_publish with the send-time stamp instead.
Encoded encode(const Profile& p, char* out, std::size_t capacity) {
    return encode_at_publish(p, p.t_publish_mono_ms, out, capacity);
}
Encoded encode(const Objects& p, char* out, std::size_t capacity) {
    return encode_at_publish(p, p.t_publish_mono_ms, out, capacity);
}
Encoded encode(const Status& p, char* out, std::size_t capacity) {
    return encode_at_publish(p, p.t_publish_mono_ms, out, capacity);
}
}  // namespace perception::rns
