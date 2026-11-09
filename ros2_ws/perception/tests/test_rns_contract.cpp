/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rns_contract.cpp
 * Brief: Host-only R4 positive and negative fixtures of the wire contract, with an allocation guard.
 *
 * Description:
 * What it checks. The encoders and validators of Profile, Objects and Status accept exactly what
 * the contract allows: nullable fields, an atomic geometry bundle (no partial-null footprint, z or
 * range; empty or malformed present geometry is invalid, not null), identity, timestamp, class and
 * finite-data checks that unknown geometry does not waive, located and unlocalized detections in
 * one result, bounded null encoding, double precision kept, and an R6 unknown blind range that
 * never authorises FREE or T.
 */

#include "output/rns_contract.hpp"
#include <array>
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <new>
#include <type_traits>

static std::size_t allocations = 0;
void* operator new(std::size_t n) { ++allocations; if (auto* p = std::malloc(n ? n : 1)) return p; throw std::bad_alloc(); }
void* operator new[](std::size_t n) { return ::operator new(n); }
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }

using namespace perception::rns;
Profile profile() {
    Profile p;
    p.t_capture_mono_ms = 1000; p.t_publish_mono_ms = 1020;
    p.z_pass_m = .75F; p.range_max_m = 6; p.blind_near_m = .5F;
    p.angle_min_rad = -.01F; p.angle_step_rad = .01F;
    p.bins.resize(3);
    // No geometry + semantic boundary; invalid; fallback with samples but no FREE.
    p.bins[0].d_block = 2.5F; p.bins[0].src = kS;
    p.bins[2].src = kG;
    return p;
}
Objects objects() {
    Objects m; m.t_capture_mono_ms = 1100; m.t_publish_mono_ms = 1200;
    Object o;
    o.track_id = 9; o.class_id = 0; o.class_name = "箱体\"\n\x01";
    o.confidence = .9F; o.semantic_status = "confirmed";
    o.geometry = ObjectGeometry{{{2, -1}, {3, -1}, {3, 1}, {2, 1}}, .1F, 1};
    o.velocity_xy = std::array<float, 2>{0, 0}; // Explicit old fixture measurement, not a default.
    o.velocity_frame = "raw"; o.velocity_status = "warming_up";
    o.depth_quality = "fair";
    o.first_seen_mono_ms = 1000; o.last_seen_mono_ms = 1100;
    m.objects.push_back(o); return m;
}
Objects r6_objects(unsigned variant) {
    auto m = objects(); auto& o = m.objects[0];
    if (variant == 0) {
        o.velocity_xy = std::array<float, 2>{.5F, -.125F};
        o.velocity_status = "motion_unconfirmed";
    } else {
        o.velocity_xy.reset();
        if (variant == 2) {
            o.geometry.reset(); o.velocity_status = "timestamp_gap"; o.depth_quality = "poor";
        }
    }
    return m;
}
Status status() {
    Status s; s.t_publish_mono_ms = 1200; s.fps_depth = 30; s.fps_infer = 25;
    s.latency_ms_p50 = 100; s.latency_ms_p99 = 180;
    s.infer_gap_ms_p99 = 47; s.infer_gap_ms_max = 62;
    s.invalid_pixel_ratio = .1;
    s.degraded_reasons = {"ground_fit_fallback", "ground_free_withdrawn"}; return s;
}
template <class T> void good(const T& t) {
    std::array<char, 65536> out{};
    const auto before = allocations;
    const auto result = encode(t, out.data(), out.size());
    assert(result && result.size == std::strlen(out.data()));
    assert(allocations == before);  // Serialization itself must not allocate.
    assert(out[0] == '{' && out[result.size - 1] == '}');
}
template <class T> void bad(const T& t) {
    char out[2048] = "old contents";
    const auto result = encode(t, out, sizeof(out));
    assert(!result && result.error == Error::InvalidInput && result.size == 0 && out[0] == '\0');
}
template <class T> void publish_override(const T& t) {
    std::array<char, 65536> before{}, overridden{};
    assert(encode(t, before.data(), before.size()));
    assert(encode_at_publish(t, t.t_publish_mono_ms, overridden.data(), overridden.size()));
    assert(std::strcmp(before.data(), overridden.data()) == 0);
    auto changed = t; changed.t_publish_mono_ms = 1300;
    assert(encode(changed, before.data(), before.size()));
    assert(encode_at_publish(t, 1300, overridden.data(), overridden.size()));
    assert(std::strcmp(before.data(), overridden.data()) == 0);
    assert(!encode_at_publish(t, -1, overridden.data(), overridden.size()) && overridden[0] == 0);
}
void nullable_objects() {
    static_assert(!std::is_default_constructible_v<ObjectGeometry>,
                  "a located measurement must explicitly supply footprint and both heights");
    Object unknown;
    assert(!unknown.geometry && !unknown.velocity_xy && !unknown.velocity_valid);
    // The geometry bundle is atomic; partial-null footprint/z/range has no
    // representation. Empty/malformed present geometry is invalid, not null.
    auto m = objects(); m.objects[0].geometry->footprint_xy.clear(); bad(m);
    m = objects(); m.objects[0].geometry->z_min = std::numeric_limits<float>::quiet_NaN(); bad(m);
    m = objects(); m.objects[0].geometry->z_max = std::numeric_limits<float>::infinity(); bad(m);
    m = objects(); m.objects[0].geometry->z_min = 2; bad(m);
    // A19-NULL-2 (19 section 14; 11 section 3.1B.2), wire side, from here to its PASS line:
    // variants 1 and 2 carry no velocity estimate and must be written as "velocity_xy":null with
    // "velocity_valid":false and their no-estimate status; null is accepted under warming_up and
    // timestamp_gap only, and refused with velocity_valid true or under a status that claims an
    // estimate. (Variant 0 and the geometry asserts in the first loop belong to other rows.)
    for (unsigned variant = 0; variant < 3; ++variant) {
        m = r6_objects(variant); good(m); publish_override(m);
        char out[4096]{}; assert(encode(m, out, sizeof(out)));
        assert(std::strstr(out, "\"track_id\":9") && std::strstr(out, "\"semantic_status\":\"confirmed\""));
        assert(std::strstr(out, "\"objects\":[{") && !std::strstr(out, "\"objects\":[]"));
        assert(std::strstr(out, "\"velocity_valid\":false"));
        if (variant == 0) {
            assert(std::strstr(out, "\"velocity_xy\":[0.5,-0.125]"));
        } else {
            assert(std::strstr(out, "\"velocity_xy\":null"));
            assert(std::strstr(out, variant == 1 ? "\"velocity_status\":\"warming_up\""
                                                 : "\"velocity_status\":\"timestamp_gap\""));
        }
        if (variant == 2) {
            assert(std::strstr(out, "\"footprint_xy\":null,\"z_min\":null,\"z_max\":null,\"r_near\":null"));
        } else {
            assert(std::strstr(out, "\"footprint_xy\":[[") && std::strstr(out, "\"r_near\":2"));
        }
    }
    for (const auto* status : {"warming_up", "timestamp_gap"}) {
        m = r6_objects(1); m.objects[0].velocity_status = status; good(m);
        m.objects[0].velocity_valid = true; bad(m);
        m = r6_objects(2); m.objects[0].velocity_status = status; good(m);
        m.objects[0].velocity_valid = true; bad(m);
    }
    for (const auto* status : {"moving", "static", "motion_unconfirmed", "unknown"}) {
        m = r6_objects(1); m.objects[0].velocity_status = status; bad(m);
        m = r6_objects(2); m.objects[0].velocity_status = status; bad(m);
    }
    std::cout << "A19-NULL-2 wire: no velocity estimate -> velocity_xy null, velocity_valid false, "
                 "warming_up / timestamp_gap; null with velocity_valid true or an estimate status refused PASS\n";
    for (const auto speed : {std::array<float, 2>{0, 0}, std::array<float, 2>{.5F, -.125F}}) {
        m = r6_objects(2); m.objects[0].velocity_xy = speed; bad(m);
        m = r6_objects(0); m.objects[0].velocity_xy = speed; good(m); // numeric raw false remains numeric
    }
    m = r6_objects(0); (*m.objects[0].velocity_xy)[0] = std::numeric_limits<float>::infinity(); bad(m);
    m = r6_objects(0); (*m.objects[0].velocity_xy)[1] = std::numeric_limits<float>::quiet_NaN(); bad(m);
    m = r6_objects(0); m.extrinsic_calibrated = true;
    m.objects[0].velocity_frame = "ego_removed"; m.objects[0].velocity_status = "moving";
    m.objects[0].velocity_valid = true; good(m);
    // Unknown geometry does not waive identity, timestamp, class or finite-data checks.
    m = r6_objects(2); m.objects[0].last_seen_mono_ms = 1099; bad(m);
    m = r6_objects(2); m.objects[0].first_seen_mono_ms = 1101; bad(m);
    m = r6_objects(2); m.objects[0].confidence = std::numeric_limits<float>::quiet_NaN(); bad(m);
    m = r6_objects(2); m.objects[0].class_name = "traversable_area"; bad(m);
    m = r6_objects(2); m.objects[0].semantic_status = "unknown"; bad(m);
    m = r6_objects(2); m.objects[0].velocity_frame = "ego_removed"; bad(m);
    m = r6_objects(2); m.objects[0].depth_quality = "unknown"; bad(m);
    // One complete new result may include located and unlocalized detections.
    m = r6_objects(0); auto missing = r6_objects(2).objects[0]; missing.track_id = 10;
    m.objects.push_back(missing); good(m);
    char out[4096]{}; assert(encode(m, out, sizeof(out)));
    assert(std::strstr(out, "\"track_id\":9") && std::strstr(out, "\"track_id\":10"));
    m.objects[1].track_id = 9; bad(m);
    // Null encoding remains bounded and late-stamp overrides never mutate the source.
    m = r6_objects(2); char small[8] = "before";
    const auto result = encode(m, small, sizeof(small));
    assert(result.error == Error::Capacity && result.size == 0 && small[0] == 0);
}
void nullable_status() {
    Status s; s.t_publish_mono_ms = 1200;
    good(s); publish_override(s);
    char out[4096]{}; assert(encode(s, out, sizeof(out)));
    for (const auto* field : {"\"invalid_pixel_ratio\":null", "\"latency_ms_p50\":null",
            "\"latency_ms_p99\":null", "\"infer_gap_ms_p99\":null", "\"infer_gap_ms_max\":null"})
        assert(std::strstr(out, field));
    s.invalid_pixel_ratio = 1; good(s); s.invalid_pixel_ratio = 0; good(s);
    s.invalid_pixel_ratio = std::numeric_limits<double>::quiet_NaN(); bad(s);
    s.invalid_pixel_ratio = std::numeric_limits<double>::infinity(); bad(s);
    s.invalid_pixel_ratio = -.1; bad(s); s.invalid_pixel_ratio = 1.1; bad(s);
    s.invalid_pixel_ratio.reset(); s.latency_ms_p50 = 1; bad(s);
    s.latency_ms_p99 = 2; good(s); s.latency_ms_p99 = .5; bad(s);
    s.latency_ms_p50.reset(); bad(s); s.latency_ms_p99.reset(); good(s);
    s.infer_gap_ms_max = 10000; good(s); // observed open gap, even if quantile unavailable
    s.infer_gap_ms_p99 = 10000; good(s); s.infer_gap_ms_max = 9999; bad(s);
    s.infer_gap_ms_max.reset(); bad(s); s.infer_gap_ms_p99.reset(); good(s);
    // The optional writer retains double precision rather than narrowing to float.
    s.invalid_pixel_ratio = 0.12345678901234566;
    assert(encode(s, out, sizeof(out)));
    assert(std::strstr(out, "\"invalid_pixel_ratio\":0.12345678901234566"));
}
int main(int argc, char** argv) {
    if (argc == 2 && std::string(argv[1]) == "--emit-fixtures") {
        std::array<char, 65536> out{};
        assert(encode(profile(), out.data(), out.size())); std::cout << out.data() << '\n';
        assert(encode(objects(), out.data(), out.size())); std::cout << out.data() << '\n';
        assert(encode(status(), out.data(), out.size())); std::cout << out.data() << '\n';
        return 0;
    }
    if (argc == 2 && std::string(argv[1]) == "--emit-r6-object-fixtures") {
        constexpr const char* cases[] = {"objects_raw_estimate", "objects_velocity_unknown", "objects_unlocalized"};
        std::array<char, 65536> out{};
        for (unsigned i = 0; i < 3; ++i) {
            const auto body = r6_objects(i); good(body);
            assert(encode(body, out.data(), out.size()));
            std::cout << "{\"case\":\"" << cases[i] << "\",\"body\":" << out.data() << "}\n";
        }
        return 0;
    }
    nullable_objects();
    good(profile()); good(objects()); good(status()); nullable_status();
    publish_override(profile()); publish_override(objects()); publish_override(status());
    auto p = profile(); p.bins[0].d_free = 2; bad(p);  // S alone never FREE.
    p = profile(); p.bins[0].h_block = .4F; bad(p);
    p = profile(); p.bins[0].src = 0; bad(p);
    p = profile(); p.bins[0].src = kG | kT; p.bins[0].d_free = 2; bad(p); // no T timestamp
    p.t_seg_mono_ms = 990; good(p);
    p.bins[0].d_free = 3; bad(p);  // PROF-1
    p = profile(); p.bins[2].d_free = 6; good(p);  // Geometry-only no segmentation.
    p.bins[2].d_free = 7; bad(p);
    p = profile(); p.bins[0].d_block = std::numeric_limits<float>::infinity(); bad(p);
    p = profile(); p.bins[0].d_block = std::numeric_limits<float>::quiet_NaN(); bad(p);
    p = profile(); p.bins[1].src = kNegative; bad(p);
    p = profile(); p.bins[0].src = kG | kNegative; p.bins[0].h_block = -.3F; good(p);
    p.bins[0].h_block = .3F; bad(p);
    p = profile(); p.bins[1].slope_deg = -128; bad(p);
    p = profile(); p.t_publish_mono_ms = 999; bad(p);
    p = profile(); p.t_seg_mono_ms = 1001; bad(p);
    p = profile(); p.bins.resize(kMaxBins + 1); bad(p);
    p = profile(); p.bins.clear(); bad(p);

    // R6 unknown blind range is representable, but never authorizes FREE/T.
    p = profile(); p.blind_near_m.reset(); good(p); publish_override(p);
    char withdrawn[2048]{};
    assert(encode(p, withdrawn, sizeof(withdrawn)));
    assert(std::strstr(withdrawn, "\"blind_near_m\":null"));
    p.bins[2].d_free = 0; bad(p); // Even a zero endpoint is a non-null FREE claim.
    p.bins[2].d_free = 1; bad(p);
    p.bins[2].d_free.reset(); p.bins[2].src = kG | kT; p.t_seg_mono_ms = 990; bad(p);
    p.bins[2].d_free = 1; bad(p);
    p.blind_near_m = 0.5F; good(p); // Otherwise-valid T must not bypass null-blind check.
    p = profile(); p.blind_near_m = -1; bad(p);
    p.blind_near_m = 7; bad(p);
    p.blind_near_m = std::numeric_limits<float>::quiet_NaN(); bad(p);
    p.blind_near_m = std::numeric_limits<float>::infinity(); bad(p);
    p.blind_near_m = 0; good(p); // Measured zero and unknown are distinct wire values.

    auto m = objects(); assert(footprint_near(m.objects[0].geometry->footprint_xy) == 2);
    m.objects[0].class_name = "traversable_area"; bad(m);
    m = objects(); m.objects[0].geometry->footprint_xy.resize(2); bad(m);
    m = objects(); m.objects[0].geometry->footprint_xy = {{0,0}, {1,0}, {2,0}}; bad(m);
    m = objects(); m.objects[0].geometry->footprint_xy = {{2,1}, {3,1}, {3,-1}, {2,-1}}; bad(m);
    m = objects(); m.objects[0].geometry->footprint_xy = {{2,-1}, {3,1}, {3,-1}, {2,1}}; bad(m);
    m = objects(); m.objects[0].geometry->footprint_xy = {{-1,-1}, {1,-1}, {1,1}, {-1,1}};
    assert(footprint_near(m.objects[0].geometry->footprint_xy) == 0); good(m);
    m = objects(); m.objects[0].velocity_frame = "ego_removed"; bad(m);
    m.extrinsic_calibrated = true; good(m); // Caller still owes exact-time TF proof.
    m = objects(); m.objects[0].last_seen_mono_ms = 1099; bad(m);
    m = objects(); m.objects[0].class_name = std::string("\xc0\xaf", 2); bad(m);
    m = objects(); m.objects[0].velocity_status = "unknown"; bad(m);
    m = objects(); m.objects.push_back(m.objects[0]); bad(m);
    m = objects(); m.objects.clear(); good(m);  // A new complete empty inference is legal.
    auto s = status(); s.infer_gap_ms_p99 = 80; bad(s);
    s = status(); s.invalid_pixel_ratio = 1.1; bad(s);

    char small[8] = "before";
    const auto limited = encode(profile(), small, sizeof(small));
    assert(limited.error == Error::Capacity && limited.size == 0 && small[0] == '\0');
    assert(encode(profile(), nullptr, 0).error == Error::Capacity);
    char full[65536]; const auto exact = encode(profile(), full, sizeof(full)); assert(exact);
    std::vector<char> boundary(exact.size + 1);
    assert(encode(profile(), boundary.data(), boundary.size()));
    assert(encode(profile(), boundary.data(), boundary.size() - 1).error == Error::Capacity);
    std::cout << "RNS payload positive/negative and no-allocation checks PASS\n";
}
