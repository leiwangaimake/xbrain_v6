/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rns_capture_estimate.cpp
 * Brief: Unit tests for the checked hand-over of a frame's capture stamp to Profile and Objects.
 *
 * Description:
 * What it checks. Since 2026-10-14 the capture thread stamps every frameset from the camera's
 * frame timestamp and carries the stamp in the SourceId (19 section 2.4, PD-11 measured);
 * estimate_capture() hands exactly that stamp on. Synthetic cases cover: the stamp is taken as
 * it is, in both grades (measured and the declared fallback grade with its Status literal), and
 * the host-ready time no longer decides it; a frame without a stamp gets none (NoCaptureTime,
 * no constant stands in); a stamp after the frame's own pickup is refused; configuration and
 * clock-epoch validation; source validation at the 64-bit limits; the explicit adapter handed
 * to the Profile path; and that nothing is mutated or allocated.
 * Trap. Clock forwarding does not latch errors; monotonic history is the consumer's check, so no
 * latch is expected here. No SDK, device, real clock or publish is involved.
 */

/* Pure synthetic estimate checks. No SDK, device, actual clock or publish. */
#include "output/rns_capture_estimate.hpp"
#include <array>
#include <atomic>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <new>
#include <string>
#include <type_traits>
#include <utility>

namespace {
std::atomic<std::size_t> allocations{0};
std::size_t checks = 0;
void* alloc(std::size_t n) {
    if (auto* p = std::malloc(n ? n : 1)) { ++allocations; return p; }
    throw std::bad_alloc();
}
void* aligned(std::size_t n, std::size_t alignment) {
    void* p = nullptr;
    if (posix_memalign(&p, alignment, n ? n : 1)) throw std::bad_alloc();
    ++allocations;
    return p;
}
[[noreturn]] void fail(const char* expr, int line) {
    std::cerr << "capture estimate CHECK " << line << ": " << expr << '\n';
    std::abort();
}
#define CHECK(x) do { ++checks; if (!(x)) fail(#x, __LINE__); } while (false)
}
void* operator new(std::size_t n) { return alloc(n); }
void* operator new[](std::size_t n) { return alloc(n); }
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }
void* operator new(std::size_t n, std::align_val_t a) { return aligned(n, static_cast<std::size_t>(a)); }
void* operator new[](std::size_t n, std::align_val_t a) { return aligned(n, static_cast<std::size_t>(a)); }
void operator delete(void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete(void* p, std::size_t, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t, std::align_val_t) noexcept { std::free(p); }

namespace {
using namespace perception::rns;
namespace nd = perception::native_depth;
using perception::geometry::CaptureStamp;
using Status = CaptureEstimateStatus;
constexpr auto maximum = std::numeric_limits<std::int64_t>::max();
constexpr auto umaximum = std::numeric_limits<std::uint64_t>::max();

static_assert(noexcept(estimate_capture(std::declval<const nd::Metadata&>(),
    std::declval<const CaptureEstimateConfig&>(), 1, 0)));
static_assert(!std::is_copy_constructible_v<EstimatedCaptureClock>);
static_assert(!std::is_move_constructible_v<EstimatedCaptureClock>);

bool same(const nd::SourceId& a, const nd::SourceId& b) {
    return a.stream_id == b.stream_id && a.sequence == b.sequence &&
        a.color_index == b.color_index && a.depth_index == b.depth_index &&
        a.color_device_us == b.color_device_us && a.depth_device_us == b.depth_device_us &&
        a.host_ready_mono_ns == b.host_ready_mono_ns &&
        a.capture_mono_ns == b.capture_mono_ns && a.capture_estimated == b.capture_estimated;
}
// A frame taken at host time 1000 that the capture thread stamped 900 (measured grade).
nd::Metadata metadata() {
    nd::Metadata m;
    m.source = {1,2,0,4,5,6,1000,900,false};
    m.intrinsics = {100,200,300,400,640,400};
    m.unit_m = .001F;
    m.distortion.reported = true;
    m.distortion.model = 7;
    m.distortion.coefficients = {1,2,3,4,5,6,7,8};
    return m;
}
CaptureEstimateConfig config() {
    return {42,"synthetic/config: capture clock (NOT calibration)"};
}
void rejected(const CaptureEstimateResult& result, Status status) {
    CHECK(result.status == status);
    CHECK(!result.estimated && result.reason.empty() && result.audit_source.empty());
    CHECK(result.capture.clock_epoch == 0 && result.capture.capture_mono_ns == -1);
    CHECK(same(result.capture.source, nd::SourceId{}));
}

// The capture time is the frame's own stamp, with the source identity copied. The measured grade
// carries no reason; the fallback grade carries t_capture_estimated. The host-ready time does not
// move the stamp (a frame that waited 0.35 s before it was taken keeps its capture time); zero
// values are valid.
void test_stamp_is_handed_on() {
    const auto m = metadata();
    const auto c = config();
    const auto r = estimate_capture(m,c,42,1100);
    CHECK(r.status == Status::Ready && !r.estimated && r.reason.empty());
    CHECK(r.capture.capture_mono_ns == 900 && r.capture.clock_epoch == 42);
    CHECK(same(r.capture.source,m.source));
    CHECK(r.audit_source == c.audit_source);
    auto fallback = m;
    fallback.source.capture_estimated = true;
    const auto e = estimate_capture(fallback,c,42,1100);
    CHECK(e.status == Status::Ready && e.estimated && e.reason == "t_capture_estimated" && e.reason == kCaptureEstimatedReason);
    CHECK(e.capture.capture_mono_ns == 900 && same(e.capture.source,fallback.source));
    // Taken late: the pickup moves by 350 ms, the capture time does not.
    auto late = m;
    late.source.host_ready_mono_ns = 350000900;
    const auto l = estimate_capture(late,c,42,350001000);
    CHECK(l.status == Status::Ready && l.capture.capture_mono_ns == 900);
    auto z = m;
    z.source.host_ready_mono_ns = 0;
    z.source.capture_mono_ns = 0;
    z.source.color_index = z.source.depth_index = 0;
    const auto zero = estimate_capture(z,c,42,0);
    CHECK(zero.status == Status::Ready && !zero.estimated);
    CHECK(zero.capture.capture_mono_ns == 0 && same(zero.capture.source,z.source));
}

// A frame without a stamp gets no capture time, whatever its host-ready time: nothing stands in
// (the old rule would have answered host-ready minus 23 ms). The fallback flag alone is no stamp.
// A stamp later than the frame's own pickup is refused.
void test_no_stamp_and_stamp_after_pickup() {
    auto m = metadata();
    m.source.capture_mono_ns = -1;
    rejected(estimate_capture(m,config(),42,1100),Status::NoCaptureTime);
    m.source.capture_estimated = true;
    rejected(estimate_capture(m,config(),42,1100),Status::NoCaptureTime);
    m.source.capture_mono_ns = std::numeric_limits<std::int64_t>::min();
    rejected(estimate_capture(m,config(),42,1100),Status::NoCaptureTime);
    m = metadata();
    m.source.capture_mono_ns = 1001;
    rejected(estimate_capture(m,config(),42,1100),Status::ClockRejected);
    m.source.capture_mono_ns = 1000;     // exposure and pickup at the same instant: the limit, accepted
    CHECK(estimate_capture(m,config(),42,1100).capture.capture_mono_ns == 1000);
}

// A zero epoch, a bad or too long audit-source text, or a mismatched epoch is refused.
void test_config_and_epoch() {
    const auto m = metadata();
    auto c = config();
    c.clock_epoch = 0;
    rejected(estimate_capture(m,c,42,1000),Status::InvalidConfig);
    for (const auto text : {std::string_view{},std::string_view{"   "},
                           std::string_view{"a\nb"},std::string_view{"a\0b",3},
                           std::string_view{"a\tb"},std::string_view{"a\x7f"}}) {
        c = config(); c.audit_source = text;
        rejected(estimate_capture(m,c,42,1000),Status::InvalidConfig);
    }
    std::array<char,kCaptureEstimateAuditSourceMax+1> long_id;
    long_id.fill('x');
    c = config(); c.audit_source = {long_id.data(),long_id.size()};
    rejected(estimate_capture(m,c,42,1000),Status::InvalidConfig);
    c.audit_source = {long_id.data(),long_id.size()-1};
    CHECK(valid_capture_estimate_config(c));
    rejected(estimate_capture(m,config(),0,1000),Status::InvalidEpoch);
    rejected(estimate_capture(m,config(),41,1000),Status::InvalidEpoch);
}

// Zero or negative source fields and a clock reading before host-ready are refused; extreme
// 64-bit values pass through exactly, and device timestamps are never cast to signed.
void test_source_and_limits() {
    auto m = metadata();
    for (const auto member : {&nd::SourceId::stream_id,&nd::SourceId::sequence,
        &nd::SourceId::color_device_us,&nd::SourceId::depth_device_us}) {
        m = metadata(); m.source.*member = 0;
        rejected(estimate_capture(m,config(),42,1000),Status::InvalidSource);
    }
    m = metadata(); m.source.host_ready_mono_ns = -1;
    rejected(estimate_capture(m,config(),42,1000),Status::InvalidSource);
    m.source.host_ready_mono_ns = std::numeric_limits<std::int64_t>::min();
    rejected(estimate_capture(m,config(),42,1000),Status::InvalidSource);
    m = metadata();
    rejected(estimate_capture(m,config(),42,-1),Status::ClockRejected);
    rejected(estimate_capture(m,config(),42,std::numeric_limits<std::int64_t>::min()),Status::ClockRejected);
    rejected(estimate_capture(m,config(),42,999),Status::ClockRejected);
    m.source = {umaximum,umaximum,umaximum,umaximum,umaximum,umaximum,maximum,maximum,false};
    auto c = config(); c.clock_epoch = umaximum;
    auto r = estimate_capture(m,c,umaximum,maximum);
    CHECK(r.status == Status::Ready && r.capture.capture_mono_ns == maximum);
    CHECK(same(r.capture.source,m.source)); // Device timestamps never cast into signed mono.
    m.source.capture_mono_ns = 0;
    r = estimate_capture(m,c,umaximum,maximum);
    CHECK(r.status == Status::Ready && r.capture.capture_mono_ns == 0);
}

struct Clock {
    std::int64_t value = 1100;
    mutable std::size_t calls = 0;
    static std::int64_t now(const void* context) noexcept {
        const auto& self = *static_cast<const Clock*>(context);
        ++self.calls;
        return self.value;
    }
};

// The clock adapter forwards now and capture, fails closed on a bad clock or setup without calling
// the clock, and owns a copy of its audit-source text.
void test_explicit_adapter() {
    Clock clock;
    EstimatedCaptureClock adapter(config(),&clock,42,&Clock::now);
    CHECK(adapter.valid() && adapter.config().clock_epoch == 42);
    const auto native = adapter.native_clock();
    CHECK(native.context == &adapter && native.epoch == 42 && native.now_ns && native.capture);
    CHECK(native.now_ns(native.context) == 1100 && clock.calls == 1);
    CaptureStamp out;
    CHECK(native.capture(native.context,metadata(),out) && out.capture_mono_ns == 900);
    CHECK(clock.calls == 2 && same(out.source,metadata().source));
    clock.value = -7;
    CHECK(native.now_ns(native.context) == -1);
    CHECK(!native.capture(native.context,metadata(),out));
    CHECK(out.capture_mono_ns == -1 && out.clock_epoch == 0 && same(out.source,nd::SourceId{}));
    CHECK(!native.capture(nullptr,metadata(),out) && native.now_ns(nullptr) == -1);
    clock.value = 999;
    rejected(adapter.estimate(metadata()),Status::ClockRejected);
    clock.value = 1100;
    CHECK(native.capture(native.context,metadata(),out));
    // A frame without a stamp: the adapter gives none and leaves no older stamp behind.
    auto unstamped = metadata();
    unstamped.source.capture_mono_ns = -1;
    CHECK(!native.capture(native.context,unstamped,out));
    CHECK(out.capture_mono_ns == -1 && out.clock_epoch == 0 && same(out.source,nd::SourceId{}));
    rejected(adapter.estimate(unstamped),Status::NoCaptureTime);
    // Clock forwarding does not latch errors; monotonic history is the consumer's check.
    EstimatedCaptureClock absent(config(),&clock,42,nullptr);
    EstimatedCaptureClock no_context(config(),nullptr,42,&Clock::now);
    EstimatedCaptureClock wrong_epoch(config(),&clock,43,&Clock::now);
    EstimatedCaptureClock no_config({},&clock,42,&Clock::now);
    const auto calls = clock.calls;
    for (const auto* bad : {&absent,&no_context,&wrong_epoch,&no_config}) {
        CHECK(!bad->valid());
        const auto invalid = bad->native_clock();
        CHECK(!invalid.context && !invalid.epoch && !invalid.now_ns && !invalid.capture);
        CHECK(bad->estimate(metadata()).status != Status::Ready);
    }
    CHECK(clock.calls == calls);
    char borrowed[] = "synthetic/audit";
    EstimatedCaptureClock owned({42,borrowed},&clock,42,&Clock::now);
    borrowed[0] = 'x';
    CHECK(owned.valid() && owned.config().audit_source == "synthetic/audit");
    CHECK(owned.config().audit_source.data() != borrowed);
    EstimatedCaptureClock temporary({42,std::string("temporary/synthetic/config")},
                                   &clock,42,&Clock::now);
    CHECK(temporary.valid() && temporary.config().audit_source == "temporary/synthetic/config");
    CHECK(temporary.estimate(metadata()).audit_source == "temporary/synthetic/config");
}

// 10000 estimates allocate nothing and never modify the input metadata or config.
void test_no_mutation_or_allocation() {
    auto m = metadata();
    const auto original = m;
    const auto c = config();
    Clock clock;
    EstimatedCaptureClock adapter(c,&clock,42,&Clock::now);
    const auto native = adapter.native_clock();
    const auto before = allocations.load();
    for (int i=0;i<10000;++i) {
        CHECK(estimate_capture(m,c,42,1000).status == Status::Ready);
        CHECK(adapter.estimate(m).capture.capture_mono_ns == 900);
        CaptureStamp out;
        CHECK(native.capture(native.context,m,out));
        CHECK(estimate_capture(m,c,41,1000).status == Status::InvalidEpoch);
    }
    CHECK(allocations.load() == before);
    CHECK(same(m.source,original.source));
    CHECK(m.intrinsics.fx == original.intrinsics.fx && m.intrinsics.fy == original.intrinsics.fy &&
        m.intrinsics.cx == original.intrinsics.cx && m.intrinsics.cy == original.intrinsics.cy &&
        m.intrinsics.width == original.intrinsics.width && m.intrinsics.height == original.intrinsics.height);
    CHECK(m.unit_m == original.unit_m && m.distortion.reported == original.distortion.reported &&
        m.distortion.model == original.distortion.model &&
        m.distortion.coefficients == original.distortion.coefficients);
    CHECK(c.clock_epoch == 42 && c.audit_source == config().audit_source);
}
}  // namespace

int main() {
    test_stamp_is_handed_on();
    test_no_stamp_and_stamp_after_pickup();
    test_config_and_epoch();
    test_source_and_limits();
    test_explicit_adapter();
    test_no_mutation_or_allocation();
    std::cout << "RNS capture stamp hand-over PASS: groups=6 checks=" << checks
              << " [stamp+grades, no-stamp+after-pickup, config+epoch, source+64bit, adapter+lifetime, no-mutation+no-allocation(10000)]"
              << "; synthetic-only, no device/actual-clock/publish\n";
}
