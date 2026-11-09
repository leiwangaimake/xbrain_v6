/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rns_enc_strict.cpp
 * Brief: A19-ENC-1: accepted wire text is strict JSON without inf/nan literals; non-finite inputs are refused.
 *
 * Description:
 * What it checks. 19 section 14 row A19-ENC-1: serialized output passes a strict JSON parse with
 * zero failures and the whole text holds no inf / nan literal. The rule behind it is 19 section 3.5
 * and the encoding convention of 11 section 3.1B.1: a missing value is always null (d_block null =
 * no obstacle within range_max_m, never +inf; h_block null, never NaN), because JSON has no such
 * literals and a receiver would reject the message or read a wrong number.
 * How. Every body the encoder accepts here goes through tests/strict_json.hpp: validate() (the
 * strict parse) and first_non_finite_token() (the literal scan outside strings). For Profile the
 * float arrays are also read back entry by entry: null exactly where the body has no value.
 * Groups, each printed with the case id:
 *   validator / scan controls  texts the two checks must reject and must accept, so that a checker
 *                              that accepts everything cannot pass;
 *   Profile / Objects / Status bodies of every shape the contract allows, including a bin that
 *                              geometry sampled without finding a block (the d_blk = +inf case of
 *                              the design), values that print with an exponent, 181 and kMaxBins
 *                              bins, and every boolean both false and true (calibrated messages);
 *   negative matrix            each float slot x {+inf, -inf, NaN} is refused with InvalidInput,
 *                              size 0 and an empty output;
 *   final packets              the bytes PublisherChannel hands to the transport (envelope with
 *                              space-padded late fields around the body), with ts_sync false and
 *                              true, and no put for a NaN body.
 * Mandated mutant (19 section 14): the serializer writes inf for a bin without a block -> the
 * strict parse of the Profile bodies fails -> red.
 * Scope. Host only: no device, network or SDK. The base bodies repeat the fixtures of
 * tests/test_rns_contract.cpp, which checks the field rules themselves; nothing here is a
 * receive-side acceptance.
 */

#include "output/rns_contract.hpp"
#include "output/rns_publisher.hpp"
#include "strict_json.hpp"

#include <algorithm>
#include <array>
#include <cfloat>
#include <climits>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <initializer_list>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

using namespace perception::rns;
using namespace std::string_view_literals;
namespace json = perception::rns::test_json;

namespace {
constexpr auto kNpos = std::string_view::npos;
int failures = 0, checks = 0, accepted_bodies = 0, refused_inputs = 0, packets = 0;
// The text or body under test, printed with every failure so that a red line names its fixture.
std::string subject;
#define CHECK(c) do { ++checks; if (!(c)) { ++failures; \
    std::printf("FAIL %s:%d %s [%s]\n", __FILE__, __LINE__, #c, subject.c_str()); } } while (0)

// One printed verdict per group, so the regression log maps to the contract row.
struct Group {
    explicit Group(const char* group) : name(group), failures_before(failures) {}
    ~Group() { std::printf("A19-ENC-1 %s: %s\n", name, failures == failures_before ? "PASS" : "FAIL"); }
    const char* name;
    int failures_before;
};

// Large enough for kMaxBins bins and kMaxObjects objects.
constexpr std::size_t kWireBytes = std::size_t{1} << 20;
std::vector<char> wire(kWireBytes);

// The text around a byte offset, for a failure line.
std::string around(std::string_view text, std::size_t at) {
    return std::string(text.substr(at > 28 ? at - 28 : 0, 56));
}

// Both halves of A19-ENC-1 on one text: the strict parse and the literal scan.
void strict(std::string_view text) {
    const auto verdict = json::validate(text);
    CHECK(verdict.ok);
    if (!verdict.ok) {
        std::printf("  strict parse stops at byte %zu (%s): ...%s...\n", verdict.offset, verdict.reason,
                    around(text, verdict.offset).c_str());
    }
    const auto token = json::first_non_finite_token(text);
    CHECK(token == kNpos);
    if (token != kNpos) std::printf("  inf/nan token at byte %zu: ...%s...\n", token, around(text, token).c_str());
}

// Encode a body the contract allows and hold its text against A19-ENC-1. Returns the text
// (empty when the encoder refused the body) for the caller's own field checks.
template <class T> std::string_view accepted(const T& body, const std::string& name) {
    subject = name; ++accepted_bodies;
    const auto result = encode(body, wire.data(), wire.size());
    CHECK(result.error == Error::None && result.size > 0);
    if (!result) return {};
    CHECK(result.size == std::strlen(wire.data()));
    const std::string_view text(wire.data(), result.size);
    strict(text);
    return text;
}

// A body with one non-finite float must not be written at all: InvalidInput, size 0 and an empty
// output (11 section 3.1B.1: never a sentinel; rns_contract.hpp: partial JSON is never sent).
template <class T> void refused(const T& body, const std::string& name) {
    subject = name; ++refused_inputs;
    std::strcpy(wire.data(), "old contents");
    const auto result = encode(body, wire.data(), wire.size());
    CHECK(!result && result.error == Error::InvalidInput && result.size == 0 && wire[0] == '\0');
    if (result) {
        // Show what would have gone to the transport and what a strict receiver makes of it.
        const std::string_view text(wire.data(), result.size);
        const auto verdict = json::validate(text);
        const auto token = json::first_non_finite_token(text);
        const auto at = verdict.ok ? token : verdict.offset;
        std::printf("  the encoder wrote %zu bytes; strict parse %s (%s): ...%s...\n", result.size,
                    verdict.ok ? "passes" : "fails", verdict.ok ? "finite stand-in value" : verdict.reason,
                    at == kNpos ? "" : around(text, at).c_str());
    }
}

// The value text of one member, e.g. "z_pass_m" -> "0.75". Only for scalar members.
std::string_view member(std::string_view text, std::string_view key) {
    const std::string open = "\"" + std::string(key) + "\":";
    const auto at = text.find(open);
    if (at == kNpos) return {};
    const auto begin = at + open.size();
    return text.substr(begin, std::min(text.find_first_of(",}", begin), text.size()) - begin);
}

// The entries of one Profile array, e.g. "d_block" -> {"2.5", "null", "null"}. Profile arrays
// hold only numbers and null, so the next ']' closes the array.
std::vector<std::string_view> entries(std::string_view text, std::string_view key) {
    std::vector<std::string_view> out;
    const std::string open = "\"" + std::string(key) + "\":[";
    auto at = text.find(open);
    if (at == kNpos) return out;
    at += open.size();
    const auto end = text.find(']', at);
    if (end == kNpos) return out;
    while (at < end) {
        const auto comma = std::min(text.find(',', at), end);
        out.push_back(text.substr(at, comma - at));
        at = comma + 1;
    }
    return out;
}
bool exponent(std::string_view token) { return token.find('e') != kNpos; }
bool exponent_at(std::string_view text, std::string_view key, std::size_t index) {
    const auto got = entries(text, key);
    return index < got.size() && exponent(got[index]);
}

// 19 section 3.5 on one float array: null exactly where the body has no value (never a stand-in
// number), and a present value reads back as the same float. The writer prints the widened
// double with 17 significant digits, so the read-back is exact; no decimal text is asserted.
void null_pattern(std::string_view text, const Profile& p, std::string_view key, std::optional<float> Bin::*field) {
    const auto got = entries(text, key);
    CHECK(got.size() == p.bins.size());
    if (got.size() != p.bins.size()) return;
    std::size_t wrong_null = 0, wrong_value = 0, first = kNpos;
    for (std::size_t i = 0; i < got.size(); ++i) {
        const auto& want = p.bins[i].*field;
        bool bad = false;
        if (!want) {
            bad = got[i] != "null"; wrong_null += bad;
        } else {
            const std::string token(got[i]);
            bad = got[i] == "null" || std::strtod(token.c_str(), nullptr) != static_cast<double>(*want);
            wrong_value += bad;
        }
        if (bad && first == kNpos) first = i;
    }
    CHECK(wrong_null == 0);
    CHECK(wrong_value == 0);
    if (first != kNpos) {
        std::printf("  %.*s: first wrong entry is bin %zu, text %.*s\n", static_cast<int>(key.size()), key.data(),
                    first, static_cast<int>(got[first].size()), got[first].data());
    }
}
// All of a Profile text that A19-ENC-1 and 19 section 3.5 speak about.
void profile_nulls(std::string_view text, const Profile& p) {
    null_pattern(text, p, "d_free", &Bin::d_free);
    null_pattern(text, p, "d_block", &Bin::d_block);
    null_pattern(text, p, "h_block", &Bin::h_block);
    // slope_deg has no data without a FREE segment: null, not a -128 style sentinel.
    const auto slope = entries(text, "slope_deg");
    CHECK(slope.size() == p.bins.size());
    std::size_t wrong_slope = 0;
    for (std::size_t i = 0; i < slope.size() && i < p.bins.size(); ++i) {
        wrong_slope += (slope[i] == "null") != !p.bins[i].slope_deg;
    }
    CHECK(wrong_slope == 0);
    for (const auto* key : {"src", "conf", "terrain"}) CHECK(entries(text, key).size() == p.bins.size());
    CHECK(member(text, "n_bins") == std::to_string(p.bins.size()));
    CHECK((member(text, "blind_near_m") == "null") == !p.blind_near_m);
}

// ---- Bodies. The three base bodies repeat tests/test_rns_contract.cpp. ----
Profile base_profile() {
    Profile p;
    p.t_capture_mono_ms = 1000; p.t_publish_mono_ms = 1020;
    p.z_pass_m = .75F; p.range_max_m = 6; p.blind_near_m = .5F;
    p.angle_min_rad = -.01F; p.angle_step_rad = .01F;
    p.bins.resize(3);
    p.bins[0].d_block = 2.5F; p.bins[0].src = kS;   // S-only block, no geometry sample
    // bins[1]: nothing observed, every field null.
    p.bins[2].src = kG;   // geometry sampled the bin and found no block: the design's d_blk = +inf
    return p;
}
// The bin shapes the contract allows (11 section 3.1B.1, src per-bit rules). ground = false
// leaves out d_free and slope_deg: a frame without a verifiable ground carries no FREE.
constexpr unsigned kBinKinds = 10;
Bin bin_kind(unsigned kind, bool ground) {
    Bin b;
    switch (kind) {
        case 0: break;                                 // nothing observed: every field null
        case 1: b.src = kS; b.d_block = 2.5F; break;   // S-only block
        case 2: b.src = kG; break;                     // geometry sample, no block, no FREE
        case 3:                                        // FREE up to the range limit, no block
            b.src = kG; b.d_free = 6; b.slope_deg = 0; b.terrain = 1; b.conf = 255; break;
        case 4:                                        // positive block with its visible height
            b.src = kG; b.d_free = 1.5F; b.d_block = 2.5F; b.h_block = .4F; b.slope_deg = 12; b.conf = 200; break;
        case 5:                                        // negative obstacle with a measured depth
            b.src = kG | kNegative; b.d_free = .75F; b.d_block = 1.5F; b.h_block = -.3F; break;
        case 6: b.src = kG | kNegative; b.d_block = 1.5F; break;   // negative, depth not measurable
        case 7: b.src = kG | kS; b.d_free = 1; b.d_block = 2; break;   // semantic block: no height
        case 8:                                        // block exactly at range_max_m = 6.0
            b.src = kG; b.d_free = 6; b.d_block = 6; b.h_block = .75F; b.terrain = 4; b.slope_deg = 90; break;
        default: b.src = kG; b.d_free = 0; b.d_block = 0; b.h_block = -0.0F; break;   // zeros, and "-0"
    }
    if (!ground) { b.d_free.reset(); b.slope_deg.reset(); }
    return b;
}
// A frame with the header of the contract sample (+-45 degrees, 0.5 degree bins) whose bins cycle
// through every shape above.
Profile frame(std::size_t bins, bool ground) {
    Profile p;
    p.t_capture_mono_ms = 1000; p.t_publish_mono_ms = 1020;
    p.z_pass_m = .75F; p.range_max_m = 6;
    p.angle_min_rad = -.785398F; p.angle_step_rad = .008727F;
    if (ground) p.blind_near_m = .59F;
    p.bins.reserve(bins);
    for (std::size_t i = 0; i < bins; ++i) p.bins.push_back(bin_kind(static_cast<unsigned>(i % kBinKinds), ground));
    return p;
}
// Tiny floats in every Profile float slot: std::to_chars prints them with an exponent.
Profile exponent_profile() {
    auto p = base_profile();
    p.z_pass_m = 1e-5F; p.angle_min_rad = -1e-5F; p.angle_step_rad = FLT_MIN;
    p.blind_near_m = std::numeric_limits<float>::denorm_min();
    p.bins[2].d_free = 1e-5F; p.bins[2].d_block = 1e-5F; p.bins[2].h_block = FLT_MIN;
    return p;
}
Objects base_objects() {
    Objects m; m.t_capture_mono_ms = 1100; m.t_publish_mono_ms = 1200;
    Object o;
    // The class name of test_rns_contract.cpp: two CJK characters as raw UTF-8, then a quote, a
    // line feed and a control byte, which the writer must escape.
    o.track_id = 9; o.class_id = 0; o.class_name = "\xe7\xae\xb1\xe4\xbd\x93\"\n\x01";
    o.confidence = .9F; o.semantic_status = "confirmed";
    o.geometry = ObjectGeometry{{{2, -1}, {3, -1}, {3, 1}, {2, 1}}, .1F, 1};
    o.velocity_xy = std::array<float, 2>{0, 0};
    o.velocity_frame = "raw"; o.velocity_status = "warming_up"; o.depth_quality = "fair";
    o.first_seen_mono_ms = 1000; o.last_seen_mono_ms = 1100;
    m.objects.push_back(o);
    return m;
}
// 0: raw velocity estimate; 1: velocity unknown (null); 2: unlocated, all four geometry fields null.
Objects r6_objects(unsigned variant) {
    auto m = base_objects(); auto& o = m.objects[0];
    if (variant == 0) {
        o.velocity_xy = std::array<float, 2>{.5F, -.125F}; o.velocity_status = "motion_unconfirmed";
    } else {
        o.velocity_xy.reset();
        if (variant == 2) { o.geometry.reset(); o.velocity_status = "timestamp_gap"; o.depth_quality = "poor"; }
    }
    return m;
}
Status base_status() {
    Status s; s.t_publish_mono_ms = 1200; s.fps_depth = 30; s.fps_infer = 25;
    s.latency_ms_p50 = 100; s.latency_ms_p99 = 180;
    s.infer_gap_ms_p99 = 47; s.infer_gap_ms_max = 62;
    s.invalid_pixel_ratio = .1;
    s.degraded_reasons = {"ground_fit_fallback", "ground_free_withdrawn"};
    return s;
}
// The three messages once the extrinsic is calibrated (11 section 3.1B.4). Every boolean of the
// body is true here and false in the bodies above, so both literals of the writer are exercised.
Profile calibrated_profile() { auto p = base_profile(); p.extrinsic_calibrated = true; return p; }
Objects calibrated_objects() {
    auto m = r6_objects(0); auto& o = m.objects[0];
    m.extrinsic_calibrated = true;   // ego_removed is refused without it
    o.velocity_frame = "ego_removed"; o.velocity_status = "moving"; o.velocity_valid = true;
    return m;
}
Status calibrated_status() {
    auto s = base_status(); s.extrinsic_calibrated = true; s.traversable_seg_available = true;
    return s;
}

// ---- The checkers' own controls: a validator that accepts everything must not pass. ----
void validator_controls() {
    const Group group("strict JSON validator controls");
    const std::string deep_ok = std::string(json::kMaxDepth, '[') + std::string(json::kMaxDepth, ']');
    const std::string too_deep = "[" + deep_ok + "]";
    const std::string_view rejected[] = {
        // The literals the case is about, as bare tokens and in the two mutant shapes of d_block.
        "[inf]", "[-inf]", "[+inf]", "[nan]", "[-nan]", "[NaN]", "[Infinity]", "[-Infinity]", "[nan(ind)]", "[1.#INF]",
        "{\"d_block\":[2.5,inf,inf]}", "{\"d_block\":[2.5,null,inf]}", "{\"h_block\":[null,nan]}",
        // Legal number grammar, but a receiver reads infinity.
        "[1e999]", "[-1e999]",
        // Numbers outside the grammar.
        "[01]", "[-01]", "[.5]", "[5.]", "[+1]", "[1e]", "[1e+]", "[-]", "[0x10]", "[1.2.3]", "[--1]",
        // Structure: trailing or missing commas, bad keys, repeated keys (also after unescaping).
        "[1,]", "[,1]", "[1 2]", "{\"a\":1,}", "{,}", "{a:1}", "{\"a\" 1}", "{\"a\":}", "{\"a\"}", "{1:2}",
        "{\"a\":1,\"a\":2}", "{\"a\":1,\"\\u0061\":2}", "{\"a\":{\"b\":1,\"b\":2}}",
        // Not exactly one value.
        "", " ", "[", "]", "{", "}", "[1]x", "[1] [2]", "{}{}", "[1]\0"sv, "\xef\xbb\xbf[1]",
        // Bare words and comments.
        "nul", "True", "[TRUE]", "[NULL]", "[truee]", "'a'", "[1] // note", "/* note */ [1]",
        // Strings: not closed, raw control characters, bad escapes, half surrogate pairs.
        "\"abc", "\"\\\"", "\"a\nb\"", "\"a\tb\"", "\"a\x01" "b\"", "\"\\q\"", "\"\\x41\"", "\"\\u12g4\"", "\"\\u123\"",
        "\"\\ud800\"", "\"\\udc00\"", "\"\\ud800\\u0041\"",
        // Bytes that are not UTF-8: overlong, surrogate, truncated, stray lead and continuation.
        "\"\xc0\xaf\"", "\"\xed\xa0\x80\"", "\"\xe4\xb8\"", "\"\xff\"", "\"\x80\"",
        too_deep,
    };
    const std::string_view passed[] = {
        "{}", "[]", "0", "-0", "-0.0", "0.5", "true", "false", "null", "\"\"",
        // The exponent forms std::to_chars prints, up to the ends of the double range.
        "1e5", "1E+5", "1e-05", "9.9999997473787516e-06", "1.1754943508222875e-38", "1.4012984643248171e-45",
        "1e+21", "1.7976931348623157e+308", "4.9406564584124654e-324",
        // Whitespace around tokens, as in the publisher's space-padded late fields.
        " \t\r\n[ 1 , 2 ]\n", "{ \"a\" : [ true , false , null ] }",
        "{\"seq\":1                   ,\"ts_sync\":false,\"t_publish_mono_ms\":30000              }",
        // Strings: every short escape, a unicode escape with a surrogate pair, raw 2-, 3- and 4-byte UTF-8.
        "\"\\\"\\\\\\/\\b\\f\\n\\r\\t\\u00e9\\ud83d\\ude00\"", "\"\xc3\xa9 \xe7\xae\xb1 \xf0\x9f\x98\x80\"",
        // The words are text inside a string.
        "\"inf nan Infinity\"", "{\"infer_gap_ms_p99\":null,\"degraded_reasons\":[\"infer_gap\"]}",
        // The same key in different objects, and keys that differ after unescaping.
        "{\"\":1}", "{\"a\":1,\"A\":2}", "{\"a\":{\"b\":1},\"b\":2}", "[{\"a\":1},{\"a\":2}]",
        "{\"a\\u0062\":1,\"a\":2}",
        deep_ok,
    };
    for (const auto text : rejected) {
        subject = "must be rejected: " + std::string(text.substr(0, 48));
        CHECK(!json::validate(text).ok);
    }
    for (const auto text : passed) {
        subject = "must be accepted: " + std::string(text.substr(0, 48));
        const auto verdict = json::validate(text);
        CHECK(verdict.ok);
        if (!verdict.ok) std::printf("  rejected at byte %zu (%s)\n", verdict.offset, verdict.reason);
    }
    // The verdict points at the offending byte: the first inf, the repeated key, the trailing byte.
    subject = "reported offsets";
    CHECK(json::validate("{\"d_block\":[2.5,inf,inf]}").offset == 16);
    CHECK(json::validate("{\"a\":1,\"a\":2}").offset == 7);
    CHECK(json::validate("[1]x").offset == 3);
    std::printf("  %zu texts rejected, %zu accepted\n", std::size(rejected), std::size(passed));
}

void scan_controls() {
    const Group group("inf/nan token scan controls");
    struct Hit { std::string_view text; std::size_t at; };
    const Hit hits[] = {
        {"[inf]", 1}, {"[-inf]", 2}, {"[NaN]", 1}, {"[nan(ind)]", 1}, {"[Infinity]", 1}, {"[-Infinity]", 2},
        {"[1.#INF]", 4}, {"[1.#QNAN]", 4}, {"[2.5,null,nan]", 10}, {"{\"infer_gap\":inf}", 13},
        // The escaped backslash does not escape the closing quote: inf is outside the string.
        {"[\"a\\\\\",inf]", 7},
    };
    const std::string_view clean[] = {
        "", "[1e5,1E-5,true,false,null]", "\"inf\"", "[\"nan\",\"\\\\\"]",
        // An escaped quote does not close the string: inf is still inside it.
        "[\"a\\\"inf\"]",
        // The Status key and reason a plain substring search for inf would flag.
        "{\"infer_gap_ms_p99\":null,\"degraded_reasons\":[\"infer_gap\",\"inf\",\"NaN\"]}",
    };
    for (const auto& hit : hits) {
        subject = "must be flagged: " + std::string(hit.text);
        CHECK(json::first_non_finite_token(hit.text) == hit.at);
    }
    for (const auto text : clean) {
        subject = "must not be flagged: " + std::string(text);
        CHECK(json::first_non_finite_token(text) == kNpos);
    }
    std::printf("  %zu texts flagged, %zu clean\n", std::size(hits), std::size(clean));
}

// ---- Accepted bodies. ----
void profile_bodies() {
    const Group group("Profile bodies strict JSON, no inf/nan, null where no value");
    // The mandated mutant's body: bins[2] has a geometry sample (bit1) and no block.
    auto p = base_profile();
    auto text = accepted(p, "profile: S-only block, unobserved bin, G bin without block");
    profile_nulls(text, p);
    const auto block = entries(text, "d_block");
    CHECK(block.size() == 3 && block[0] != "null" && block[1] == "null" && block[2] == "null");

    p = base_profile(); p.bins[2].d_free = 6;
    profile_nulls(accepted(p, "profile: geometry-only FREE up to range_max_m 6.0"), p);

    // T evidence (bit0) needs a segmentation frame and a FREE end beyond the blind range.
    p = base_profile(); p.t_seg_mono_ms = 990;
    p.bins[2].src = kG | kT; p.bins[2].d_free = 3.25F; p.bins[2].slope_deg = 5; p.bins[2].terrain = 1;
    text = accepted(p, "profile: T-backed FREE, no block");
    profile_nulls(text, p);
    CHECK(member(text, "t_seg_mono_ms") == "990");

    p = base_profile(); p.bins[0].src = kG | kNegative; p.bins[0].h_block = -.3F;
    profile_nulls(accepted(p, "profile: negative block with a negative h_block"), p);
    p.bins[0].h_block.reset();
    profile_nulls(accepted(p, "profile: negative block with h_block null"), p);

    // No verifiable ground: blind_near_m null (11 v2.3), every d_free null.
    p = base_profile(); p.blind_near_m.reset();
    profile_nulls(accepted(p, "profile: blind_near_m null"), p);
    p.blind_near_m = 0;
    text = accepted(p, "profile: blind_near_m measured 0");
    profile_nulls(text, p);
    CHECK(member(text, "extrinsic_calibrated") == "false");

    // Calibrated extrinsic: the writer's true literal (false in every body above).
    p = calibrated_profile();
    text = accepted(p, "profile: extrinsic_calibrated true");
    profile_nulls(text, p);
    CHECK(member(text, "extrinsic_calibrated") == "true");

    p = exponent_profile();
    text = accepted(p, "profile: exponent-printing values in every float slot");
    profile_nulls(text, p);
    // The fixture really exercised the exponent form of the writer.
    CHECK(exponent(member(text, "z_pass_m")) && exponent(member(text, "angle_min_rad")) &&
          exponent(member(text, "angle_step_rad")) && exponent(member(text, "blind_near_m")));
    CHECK(exponent_at(text, "d_free", 2) && exponent_at(text, "d_block", 2) && exponent_at(text, "h_block", 2));

    // Whole frames: 181 bins is the contract's sector, kMaxBins the storage limit.
    for (const std::size_t bins : {std::size_t{1}, std::size_t{181}, kMaxBins}) {
        for (const bool ground : {true, false}) {
            p = frame(bins, ground);
            profile_nulls(accepted(p, "profile: " + std::to_string(bins) + " bins of every shape, " +
                                          (ground ? "ground known" : "no verifiable ground")), p);
        }
    }
}

void objects_bodies() {
    const Group group("Objects bodies strict JSON, no inf/nan");
    auto text = accepted(base_objects(), "objects: one located object, class name needing escapes");
    // The quote is backslash-escaped; the line feed and 0x01 go out as six-byte unicode escapes.
    CHECK(text.find("\\\"\\u000a\\u0001") != kNpos);
    accepted(r6_objects(0), "objects: raw velocity estimate");
    text = accepted(r6_objects(1), "objects: velocity unknown");
    CHECK(member(text, "velocity_xy") == "null");
    text = accepted(r6_objects(2), "objects: unlocated");
    CHECK(text.find("\"footprint_xy\":null,\"z_min\":null,\"z_max\":null,\"r_near\":null") != kNpos);

    auto m = base_objects(); m.objects.clear();
    CHECK(accepted(m, "objects: empty list").find("\"objects\":[]") != kNpos);

    m = r6_objects(0); auto second = r6_objects(2).objects[0]; second.track_id = 10;
    m.objects.push_back(second);
    accepted(m, "objects: located and unlocated in one message");

    // Calibrated message with an ego-removed, valid velocity: both booleans of Objects are true.
    text = accepted(calibrated_objects(), "objects: calibrated, ego-removed valid velocity");
    CHECK(member(text, "extrinsic_calibrated") == "true" && member(text, "velocity_valid") == "true");

    // Extremes of every numeric field; the words inf / NaN and all escapes inside the class name.
    // The backslash stands before an s: left unescaped it would not form a legal JSON escape.
    constexpr float kMax = FLT_MAX;
    m = r6_objects(0); auto& o = m.objects[0];
    o.track_id = std::numeric_limits<std::int64_t>::max(); o.class_id = INT_MAX;
    o.class_name = "inf NaN -Infinity \" \\s / \b\f\n\r\t\x1f\x7f \xc3\xa9 \xe7\xae\xb1 \xf0\x9f\x98\x80";
    o.confidence = std::numeric_limits<float>::denorm_min();
    o.geometry = ObjectGeometry{{{-kMax, -kMax}, {kMax, -kMax}, {kMax, kMax}, {-kMax, kMax}}, -kMax, kMax};
    o.velocity_xy = std::array<float, 2>{1e-5F, -kMax};
    o.stable_frames = std::numeric_limits<std::uint64_t>::max();
    text = accepted(m, "objects: extreme finite values");
    CHECK(exponent(member(text, "confidence")) && exponent(member(text, "z_min")) && exponent(member(text, "z_max")));
    CHECK(text.find("\\\" \\\\s / \\u0008\\u000c") != kNpos);

    // A footprint a few hundredths of a millimetre wide: vertices and the derived r_near print with an exponent.
    m = r6_objects(0);
    m.objects[0].geometry = ObjectGeometry{{{1e-5F, 0}, {2e-5F, 0}, {2e-5F, 1e-5F}}, 0, 1e-5F};
    text = accepted(m, "objects: tiny footprint");
    CHECK(exponent(member(text, "r_near")));

    // The storage limit, located and unlocated objects alternating.
    m = r6_objects(0); const auto located = m.objects[0]; const auto unlocated = r6_objects(2).objects[0];
    m.objects.clear();
    for (std::size_t i = 0; i < kMaxObjects; ++i) {
        m.objects.push_back(i % 2 ? unlocated : located);
        m.objects.back().track_id = static_cast<std::int64_t>(i);
    }
    accepted(m, "objects: kMaxObjects objects");
}

void status_bodies() {
    const Group group("Status bodies strict JSON, no inf/nan");
    Status s; s.t_publish_mono_ms = 1200;
    auto text = accepted(s, "status: every statistic null");
    for (const auto* key : {"latency_ms_p50", "latency_ms_p99", "infer_gap_ms_p99", "infer_gap_ms_max",
                            "invalid_pixel_ratio"}) {
        CHECK(member(text, key) == "null");
    }
    // The key infer_gap_ms_p99 contains "inf": a plain substring search would flag every Status.
    CHECK(text.find("inf") != kNpos && json::first_non_finite_token(text) == kNpos);

    text = accepted(base_status(), "status: numeric statistics");
    CHECK(member(text, "extrinsic_calibrated") == "false" && member(text, "traversable_seg_available") == "false");
    // Calibrated, with the traversable segmentation available: both booleans of Status are true.
    text = accepted(calibrated_status(), "status: calibrated, traversable segmentation available");
    CHECK(member(text, "extrinsic_calibrated") == "true" && member(text, "traversable_seg_available") == "true");

    // The ends of the double range, and reasons that hold the words and need escapes.
    s = base_status();
    s.fps_depth = 1e21; s.fps_infer = DBL_MAX;
    s.latency_ms_p50 = std::numeric_limits<double>::denorm_min(); s.latency_ms_p99 = 1e-7;
    s.infer_gap_ms_p99 = 1e300; s.infer_gap_ms_max = DBL_MAX;
    s.invalid_pixel_ratio = DBL_MIN; s.ground_seg_level = INT_MAX;
    s.degraded_reasons = {"infer_gap", "infer_rate_low", "inf", "-inf", "NaN", "Infinity", "nan(ind)",
                          "quote\" backslash\\ control\x01 \xe7\xae\xb1"};
    text = accepted(s, "status: extreme finite values, reasons with inf/nan words and escapes");
    for (const auto* key : {"fps_depth", "fps_infer", "latency_ms_p50", "latency_ms_p99", "infer_gap_ms_p99",
                            "infer_gap_ms_max", "invalid_pixel_ratio"}) {
        CHECK(exponent(member(text, key)));
    }
}

// ---- Negative matrix: each float slot x {+inf, -inf, NaN}. ----
struct NonFinite { const char* name; double value; };
const NonFinite kNonFinite[] = {
    {"+inf", std::numeric_limits<double>::infinity()},
    {"-inf", -std::numeric_limits<double>::infinity()},
    {"NaN", std::numeric_limits<double>::quiet_NaN()},
};
template <class T> struct Slot { const char* name; void (*set)(T&, double); };
template <class T> void matrix(const std::string& message, const T& base, std::initializer_list<Slot<T>> slots) {
    // The unmodified body is accepted, so each refusal below is caused by the one injected value.
    accepted(base, message + ": base body of the negative matrix");
    for (const auto& slot : slots) {
        for (const auto& bad : kNonFinite) {
            auto body = base;
            slot.set(body, bad.value);
            refused(body, message + " " + slot.name + " = " + bad.name);
        }
    }
}
float narrow(double v) { return static_cast<float>(v); }   // +-inf and NaN stay what they are

void negative_matrix() {
    const Group group("negative matrix, non-finite float refused with InvalidInput and empty output");
    // bins: 0 S-only block, 2 G-only FREE, 3 positive block with height, 4 negative block with depth.
    auto p = base_profile();
    p.bins.resize(5);
    p.bins[2].d_free = 2;
    p.bins[3].src = kG; p.bins[3].d_free = 1; p.bins[3].d_block = 2.5F; p.bins[3].h_block = .4F;
    p.bins[4].src = kG | kNegative; p.bins[4].d_block = 1.5F; p.bins[4].h_block = -.3F;
    matrix<Profile>("profile", p, {
        {"z_pass_m", [](Profile& b, double v) { b.z_pass_m = narrow(v); }},
        {"angle_min_rad", [](Profile& b, double v) { b.angle_min_rad = narrow(v); }},
        {"angle_step_rad", [](Profile& b, double v) { b.angle_step_rad = narrow(v); }},
        {"range_max_m", [](Profile& b, double v) { b.range_max_m = narrow(v); }},
        {"blind_near_m", [](Profile& b, double v) { b.blind_near_m = narrow(v); }},
        {"d_free (G-only FREE bin)", [](Profile& b, double v) { b.bins[2].d_free = narrow(v); }},
        {"d_free (bin with a block)", [](Profile& b, double v) { b.bins[3].d_free = narrow(v); }},
        {"d_block (S-only bin)", [](Profile& b, double v) { b.bins[0].d_block = narrow(v); }},
        {"d_block (geometry bin)", [](Profile& b, double v) { b.bins[3].d_block = narrow(v); }},
        {"h_block (positive block)", [](Profile& b, double v) { b.bins[3].h_block = narrow(v); }},
        {"h_block (negative block)", [](Profile& b, double v) { b.bins[4].h_block = narrow(v); }},
    });
    matrix<Objects>("objects", r6_objects(0), {
        {"confidence", [](Objects& b, double v) { b.objects[0].confidence = narrow(v); }},
        {"footprint_xy[0].x", [](Objects& b, double v) { b.objects[0].geometry->footprint_xy[0].x = narrow(v); }},
        {"footprint_xy[0].y", [](Objects& b, double v) { b.objects[0].geometry->footprint_xy[0].y = narrow(v); }},
        {"footprint_xy[2].x", [](Objects& b, double v) { b.objects[0].geometry->footprint_xy[2].x = narrow(v); }},
        {"footprint_xy[2].y", [](Objects& b, double v) { b.objects[0].geometry->footprint_xy[2].y = narrow(v); }},
        {"z_min", [](Objects& b, double v) { b.objects[0].geometry->z_min = narrow(v); }},
        {"z_max", [](Objects& b, double v) { b.objects[0].geometry->z_max = narrow(v); }},
        {"velocity_xy[0]", [](Objects& b, double v) { (*b.objects[0].velocity_xy)[0] = narrow(v); }},
        {"velocity_xy[1]", [](Objects& b, double v) { (*b.objects[0].velocity_xy)[1] = narrow(v); }},
    });
    matrix<Status>("status", base_status(), {
        {"fps_depth", [](Status& b, double v) { b.fps_depth = v; }},
        {"fps_infer", [](Status& b, double v) { b.fps_infer = v; }},
        {"latency_ms_p50", [](Status& b, double v) { b.latency_ms_p50 = v; }},
        {"latency_ms_p99", [](Status& b, double v) { b.latency_ms_p99 = v; }},
        {"infer_gap_ms_p99", [](Status& b, double v) { b.infer_gap_ms_p99 = v; }},
        {"infer_gap_ms_max", [](Status& b, double v) { b.infer_gap_ms_max = v; }},
        {"invalid_pixel_ratio", [](Status& b, double v) { b.invalid_pixel_ratio = v; }},
    });
}

// ---- Final packets: what PublisherChannel hands to the transport. ----
struct FixedClock final : PublishClock {
    Millis wall = 1700000000123, mono = 30000;
    Millis wall_millis() noexcept override { return wall; }
    Millis mono_millis() noexcept override { return mono; }
};
// Keeps the bytes of the last put and moves the clock on, so that two Status never share a millisecond.
struct CaptureTransport final : RnsTransport {
    explicit CaptureTransport(FixedClock& c) : clock(c) {}
    FixedClock& clock;
    std::string bytes;
    std::size_t calls = 0;
    // What ts_sync of the envelope must read: false until a fresh ClockStatus with sync = true.
    bool synced = false;
    PutReceipt put(std::string_view, std::string_view packet, PublishQos) override {
        ++calls; bytes.assign(packet.data(), packet.size()); clock.mono += 2;
        return {PutOutcome::Accepted, 0};
    }
};
// Publish one body and hold the bytes the transport received against A19-ENC-1.
template <class T>
void packet(PublisherChannel& channel, CaptureTransport& sink, const T& body, const char* schema,
            const std::string& name) {
    subject = name;
    const auto calls = sink.calls;
    const auto result = channel.publish(body);
    CHECK(result.error == PublishError::None && sink.calls == calls + 1);
    if (!result || sink.calls != calls + 1) return;
    ++packets;
    strict(sink.bytes);
    // The body sits in "data" of the 11 section 3.0 envelope; seq is a space-padded late field.
    CHECK(sink.bytes.find(std::string("\"data\":{\"schema\":\"") + schema + "\"") != kNpos);
    CHECK(sink.bytes.find(" ,\"src\":\"perception\"") != kNpos);
    // ts_sync is a five-byte slot: "false", or "true" followed by one space of padding.
    auto sync = member(sink.bytes, "ts_sync");
    while (!sync.empty() && sync.back() == ' ') sync.remove_suffix(1);
    CHECK(sync == (sink.synced ? "true" : "false"));
}
// The publisher refuses to repeat a capture, so every packet gets its own capture time.
Profile captured_at(Profile p, Millis capture) { p.t_capture_mono_ms = capture; return p; }
Objects captured_at(Objects m, Millis capture) {
    m.t_capture_mono_ms = capture;
    for (auto& o : m.objects) o.last_seen_mono_ms = capture;
    return m;
}

void final_packets() {
    const Group group("final packets from PublisherChannel strict JSON, no inf/nan");
    ProcessEnvelope envelope("robot_1", "abcdef01", 5000);
    FixedClock clock; CaptureTransport sink(clock); ObjectsCadence cadence(0, 512);
    PublisherChannel profile(Topic::Profile, envelope, clock, sink, kWireBytes, kWireBytes);
    PublisherChannel objects(Topic::Objects, envelope, clock, sink, kWireBytes, kWireBytes, &cadence);
    PublisherChannel status(Topic::Status, envelope, clock, sink, kWireBytes, kWireBytes);
    subject = "publisher channels"; CHECK(profile.valid() && objects.valid() && status.valid());

    Millis capture = 20000;
    const char* schema = "perception_profile_v1";
    packet(profile, sink, captured_at(base_profile(), ++capture), schema, "packet: profile, G bin without block");
    packet(profile, sink, captured_at(exponent_profile(), ++capture), schema, "packet: profile with exponents");
    packet(profile, sink, captured_at(frame(181, true), ++capture), schema, "packet: profile of 181 bins");
    packet(profile, sink, captured_at(frame(181, false), ++capture), schema, "packet: profile, no verifiable ground");
    packet(profile, sink, captured_at(frame(kMaxBins, true), ++capture), schema, "packet: profile of kMaxBins bins");
    // A non-finite value never reaches the transport: InvalidBody and no put.
    auto bad = captured_at(base_profile(), ++capture);
    bad.bins[2].d_block = std::numeric_limits<float>::quiet_NaN();
    subject = "packet: profile with a NaN d_block";
    const auto calls = sink.calls;
    CHECK(profile.publish(bad).error == PublishError::InvalidBody && sink.calls == calls);

    for (unsigned variant = 0; variant < 3; ++variant) {
        packet(objects, sink, captured_at(r6_objects(variant), ++capture), "perception_objects_v1",
               "packet: objects variant " + std::to_string(variant));
    }
    Status s; s.t_publish_mono_ms = 1200;
    packet(status, sink, s, "perception_status_v1", "packet: status with every statistic null");
    packet(status, sink, base_status(), "perception_status_v1", "packet: status with numeric statistics");

    // A fresh ClockStatus with sync = true (CLK-A2): from here the envelope carries ts_sync true,
    // the other value of that slot. The bodies are the calibrated ones, with their booleans true.
    subject = "clock status"; CHECK(envelope.note_clock_status(true, clock.mono));
    sink.synced = true;
    packet(profile, sink, captured_at(calibrated_profile(), ++capture), schema,
           "packet: calibrated profile, ts_sync true");
    packet(objects, sink, captured_at(calibrated_objects(), ++capture), "perception_objects_v1",
           "packet: calibrated objects, ts_sync true");
    packet(status, sink, calibrated_status(), "perception_status_v1", "packet: calibrated status, ts_sync true");
}
}  // namespace

int main() {
    validator_controls();
    scan_controls();
    profile_bodies();
    objects_bodies();
    status_bodies();
    negative_matrix();
    final_packets();
    std::printf("{\"result\":\"%s\",\"case\":\"A19-ENC-1\",\"checks\":%d,\"failures\":%d,\"accepted_bodies\":%d,"
                "\"refused_inputs\":%d,\"final_packets\":%d}\n",
                failures ? "FAIL" : "PASS", checks, failures, accepted_bodies, refused_inputs, packets);
    return failures ? 1 : 0;
}
