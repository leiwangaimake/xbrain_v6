/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rns_profile_timing_log.cpp
 * Brief: The Profile thread's timing lines through ProfileTimingHandoff: same text, no heap call on the fast line.
 *
 * Description:
 * What it checks (19 P19-3 / A19-ALLOC-1 for the 10 s timing lines; service/profile_timing_handoff.hpp).
 *   1. Text: for a full ledger (stages, T, S; one route group without frames), a bare one (the
 *      unconfigured loop's) and an empty one, at periodic and at stop, the lines formatted from
 *      the frozen figures equal, byte for byte, the lines of a frozen copy of ProfileTiming::log()
 *      as it was until 2026-10-14 (ReferenceTiming below); and through the real logger each logged
 *      line ends with exactly that text, after its PROFILE_TIMING_OBSERVED line.
 *   2. Handoff rules: four observations wait, the fifth is dropped and counted, the next carries
 *      the gap (seq) and the count (dropped=1) and all the dropped one's samples (cumulative);
 *      what a take returns is the frozen state, not the live ledger; close() appends what waits,
 *      then one when=stop group, and nothing can be taken after it; a consumer racing close()
 *      (300 rounds, the taker logging slowly) never duplicates or loses a group, and the stop
 *      group is always logged last (the taker logs before it lets go); 1000 offers make no heap
 *      call on the offering thread.
 *   3. Both real Profile loops (run_profile_steady with T and S owners, run_profile_unconfigured)
 *      for 31.5 s on real threads, frames every 50 ms: across their three 10 s marks the Profile
 *      threads make no heap call between their first put and the stop request (counted at malloc,
 *      calloc, realloc, free and the aligned forms on glibc; at operator new / delete elsewhere);
 *      a Status-like thread takes and logs the configured loop's groups (seq 1-3, its own thread
 *      id on the lines, the Profile thread's in owner_thread); the unconfigured loop's groups are
 *      not taken and its own thread logs them at stop, then when=stop; every stop group appears
 *      once and last; the counts only grow; the stop count equals the loop's puts.
 * What it does not check. Jitter or rate (the quantile scans still run on the Profile thread),
 * zenoh-c's own heap calls (tests/test_rns_zenoh_alloc.cpp), and the async logger's writer.
 * Pure host test, real time: -O2, never with the sanitizers (they replace malloc themselves).
 */

#include "rns_profile_thread.hpp"

#include "common/logger.hpp"
#include "service/profile_timing.hpp"
#include "service/profile_timing_handoff.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <new>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

// ---- heap calls of watched threads, with their time ----
namespace {
thread_local int t_watch = -1;  // >= 0: this thread's heap calls are recorded under that index
struct HeapEvent {
    std::int64_t ns;
    int thread;
    bool release;
};
constexpr std::size_t kHeapEvents = std::size_t{1} << 16;
HeapEvent heap_events[kHeapEvents];
std::atomic<std::size_t> heap_event_count{0};
inline void note_heap(bool release) noexcept {
    if (t_watch < 0) return;
    const std::size_t i = heap_event_count.fetch_add(1, std::memory_order_relaxed);
    if (i >= kHeapEvents) return;  // counted in heap_event_count, checked by the test
    timespec ts{};
    ::clock_gettime(CLOCK_MONOTONIC, &ts);
    heap_events[i] = {static_cast<std::int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec, t_watch, release};
}
[[noreturn]] void fail(const char* expr, int line) {
    std::cerr << "test_rns_profile_timing_log CHECK " << line << ": " << expr << '\n';
    std::abort();
}
#define CHECK(x) do { if (!(x)) fail(#x, __LINE__); } while (false)
}  // namespace

#if defined(__linux__) && defined(__GLIBC__)
// glibc: the malloc family itself (operator new, std::string and the logger all end here).
extern "C" {
void* __libc_malloc(std::size_t size) noexcept;
void* __libc_calloc(std::size_t count, std::size_t size) noexcept;
void* __libc_realloc(void* old, std::size_t size) noexcept;
void* __libc_memalign(std::size_t alignment, std::size_t size) noexcept;
void __libc_free(void* p) noexcept;
void* malloc(std::size_t size) noexcept { note_heap(false); return __libc_malloc(size); }
void* calloc(std::size_t count, std::size_t size) noexcept { note_heap(false); return __libc_calloc(count, size); }
void* realloc(void* old, std::size_t size) noexcept { note_heap(false); return __libc_realloc(old, size); }
void free(void* p) noexcept {
    if (p) note_heap(true);
    __libc_free(p);
}
void* memalign(std::size_t alignment, std::size_t size) noexcept { note_heap(false); return __libc_memalign(alignment, size); }
void* aligned_alloc(std::size_t alignment, std::size_t size) noexcept { note_heap(false); return __libc_memalign(alignment, size); }
int posix_memalign(void** out, std::size_t alignment, std::size_t size) noexcept {
    if (alignment == 0 || (alignment & (alignment - 1)) != 0 || alignment % sizeof(void*) != 0) return EINVAL;
    note_heap(false);
    void* const p = __libc_memalign(alignment, size);
    if (!p) return ENOMEM;
    *out = p;
    return 0;
}
}  // extern "C"
constexpr const char* kHeapLayer = "malloc/free (glibc)";
#else
// Elsewhere (the development Mac): operator new / delete only; a direct malloc is not seen.
namespace {
void* counted_new(std::size_t n) {
    note_heap(false);
    if (void* p = std::malloc(n ? n : 1)) return p;
    throw std::bad_alloc();
}
void* counted_aligned(std::size_t n, std::size_t a) {
    note_heap(false);
    void* p = nullptr;
    if (posix_memalign(&p, a, n ? n : 1) == 0) return p;
    throw std::bad_alloc();
}
void counted_delete(void* p) noexcept {
    if (p) note_heap(true);
    std::free(p);
}
}  // namespace
void* operator new(std::size_t n) { return counted_new(n); }
void* operator new[](std::size_t n) { return counted_new(n); }
void* operator new(std::size_t n, const std::nothrow_t&) noexcept {
    try { return counted_new(n); } catch (...) { return nullptr; }
}
void* operator new[](std::size_t n, const std::nothrow_t&) noexcept {
    try { return counted_new(n); } catch (...) { return nullptr; }
}
void* operator new(std::size_t n, std::align_val_t a) { return counted_aligned(n, static_cast<std::size_t>(a)); }
void* operator new[](std::size_t n, std::align_val_t a) { return counted_aligned(n, static_cast<std::size_t>(a)); }
void operator delete(void* p) noexcept { counted_delete(p); }
void operator delete[](void* p) noexcept { counted_delete(p); }
void operator delete(void* p, std::size_t) noexcept { counted_delete(p); }
void operator delete[](void* p, std::size_t) noexcept { counted_delete(p); }
void operator delete(void* p, const std::nothrow_t&) noexcept { counted_delete(p); }
void operator delete[](void* p, const std::nothrow_t&) noexcept { counted_delete(p); }
void operator delete(void* p, std::align_val_t) noexcept { counted_delete(p); }
void operator delete[](void* p, std::align_val_t) noexcept { counted_delete(p); }
void operator delete(void* p, std::size_t, std::align_val_t) noexcept { counted_delete(p); }
void operator delete[](void* p, std::size_t, std::align_val_t) noexcept { counted_delete(p); }
constexpr const char* kHeapLayer = "operator new/delete (not glibc: a direct malloc is not seen)";
#endif

namespace {
using namespace perception::test_profile;
using svc::ProfileTiming;
using svc::ProfileTimingFigures;
using svc::ProfileTimingHandoff;
using svc::ProfileTimingOwner;
using svc::ProfileTimingWhen;
constexpr std::int64_t kMs = 1000000;

// Heap calls of watched thread `thread` with a time in (from, to).
std::size_t heap_calls(int thread, std::int64_t from, std::int64_t to, bool release) {
    std::size_t n = 0;
    const std::size_t count = std::min(heap_event_count.load(), kHeapEvents);
    for (std::size_t i = 0; i < count; ++i)
        n += heap_events[i].thread == thread && heap_events[i].release == release && heap_events[i].ns > from &&
             heap_events[i].ns < to;
    return n;
}

// ---- the old formatter, frozen ----
// ProfileTiming::log() as it was at edb29e3 (and since 2026-10-13 in perception_service_main.cpp),
// copied verbatim with LOG_INFO_M replaced by REFERENCE_LOG, which keeps the text the logger was
// given for each line. The reference the new formatter must equal byte for byte.
#define REFERENCE_LOG(module, message)                       \
    do {                                                     \
        std::ostringstream reference_stream;                 \
        reference_stream << message;                         \
        lines.push_back(reference_stream.str());             \
    } while (false)
struct ReferenceTiming : ProfileTiming {
    // The periodic PROFILE_TIMING (and PROFILE_SEMANTIC_AGE) log lines.
    void log(const char* when, std::vector<std::string>& lines) const {
        const auto ms = [](std::int64_t ns) { return ns < 0 ? -1.0 : static_cast<double>(ns) / 1e6; };
        // Not labelled G-P1: the start is the SDK FrameSet wait return (complete
        // RGB-D set), whose mapping to "depth callback return" is unconfirmed (B3-5).
        REFERENCE_LOG("Service", "PROFILE_TIMING when=" << when << " frameset_return_to_put_return_n=" << gp1.count()
            << " frameset_return_to_put_return_p50_le_ms=" << ms(gp1.quantile_upper_ns(0.5))
            << " frameset_return_to_put_return_p99_le_ms=" << ms(gp1.quantile_upper_ns(0.99))
            << " frameset_return_to_put_return_max_ms=" << ms(gp1.max_ns())
            << " frameset_return_to_put_return_over14=" << gp1.over(0) << " frameset_return_to_put_return_over60=" << gp1.over(1)
            << " put_n=" << put.count() << " put_p99_le_ms=" << ms(put.quantile_upper_ns(0.99)) << " put_max_ms="
            << ms(put.max_ns()) << " put_over1ms=" << put.over(0) << " proc_p50_le_ms=" << ms(processing.quantile_upper_ns(0.5))
            << " proc_p99_le_ms=" << ms(processing.quantile_upper_ns(0.99)) << " proc_max_ms=" << ms(processing.max_ns())
            << " proc_over14=" << processing.over(0) << " proc_cpu_n=" << processing_cpu.count()
            << " proc_cpu_p50_le_ms=" << ms(processing_cpu.quantile_upper_ns(0.5))
            << " proc_cpu_p99_le_ms=" << ms(processing_cpu.quantile_upper_ns(0.99))
            << " proc_cpu_max_ms=" << ms(processing_cpu.max_ns()) << " proc_cpu_unavailable=" << cpu_unavailable
            << " audit_mismatch=" << audit_mismatch);
        // One line per route that had frames; fixed notation, so a parser never meets an exponent.
        for (std::size_t group = 0; group < kRouteGroups && !stages.empty(); ++group) {
            const auto* h = &stages[group * kPerGroup];
            if (h[kStages - 1].count() == 0) continue;
            std::ostringstream line;
            line << std::fixed << std::setprecision(3);
            line << "PROFILE_STAGE_TIMING when=" << when << " route=" << kRouteGroupNames[group] << " clock=wall";
            for (std::size_t i = 0; i < kStages; ++i)
                line << ' ' << kStageNames[i] << "_n=" << h[i].count()
                     << ' ' << kStageNames[i] << "_p10_le_ms=" << ms(h[i].quantile_upper_ns(0.1))
                     << ' ' << kStageNames[i] << "_p50_le_ms=" << ms(h[i].quantile_upper_ns(0.5))
                     << ' ' << kStageNames[i] << "_p90_le_ms=" << ms(h[i].quantile_upper_ns(0.9));
            line << " cpu_n=" << h[kStages].count() << " cpu_p10_le_ms=" << ms(h[kStages].quantile_upper_ns(0.1))
                 << " cpu_p50_le_ms=" << ms(h[kStages].quantile_upper_ns(0.5))
                 << " cpu_p90_le_ms=" << ms(h[kStages].quantile_upper_ns(0.9));
            REFERENCE_LOG("Service", line.str());
        }
        if (semantic_age_check)
            REFERENCE_LOG("Service", "PROFILE_SEMANTIC_AGE when=" << when
                << " n=" << semantic_age_check->count()
                << " at_capture_p50_le_ms=" << ms(semantic_age_capture->quantile_upper_ns(0.5))
                << " at_capture_p99_le_ms=" << ms(semantic_age_capture->quantile_upper_ns(0.99))
                << " at_capture_max_ms=" << ms(semantic_age_capture->max_ns())
                << " at_check_p50_le_ms=" << ms(semantic_age_check->quantile_upper_ns(0.5))
                << " at_check_p99_le_ms=" << ms(semantic_age_check->quantile_upper_ns(0.99))
                << " at_check_max_ms=" << ms(semantic_age_check->max_ns())
                << " at_check_over_stale=" << semantic_age_check->over(0)
                << " at_check_over_2x_stale=" << semantic_age_check->over(1)
                << " margin_n=" << semantic_margin_n
                << " margin_mean_m=" << (semantic_margin_n ? semantic_margin_sum_m / static_cast<double>(semantic_margin_n) : 0.0)
                << " margin_max_m=" << semantic_margin_max_m
                << " unavailable=" << semantic_age_unavailable
                << " check_to_publish_n=" << semantic_check_to_publish->count()
                << " check_to_publish_p50_le_ms=" << ms(semantic_check_to_publish->quantile_upper_ns(0.5))
                << " check_to_publish_p99_le_ms=" << ms(semantic_check_to_publish->quantile_upper_ns(0.99))
                << " check_to_publish_max_ms=" << ms(semantic_check_to_publish->max_ns())
                << " check_to_publish_over5=" << semantic_check_to_publish->over(0)
                << " check_to_publish_over15=" << semantic_check_to_publish->over(1)
                << " fresh_check_to_publish_n=" << semantic_fresh_check_to_publish->count()
                << " fresh_check_to_publish_p50_le_ms=" << ms(semantic_fresh_check_to_publish->quantile_upper_ns(0.5))
                << " fresh_check_to_publish_p99_le_ms=" << ms(semantic_fresh_check_to_publish->quantile_upper_ns(0.99))
                << " fresh_check_to_publish_max_ms=" << ms(semantic_fresh_check_to_publish->max_ns()));
        if (t_stage)
            REFERENCE_LOG("Service", "PROFILE_T_TIMING when=" << when << " n=" << t_stage->count()
                << " stage_p50_le_ms=" << ms(t_stage->quantile_upper_ns(0.5))
                << " stage_p99_le_ms=" << ms(t_stage->quantile_upper_ns(0.99))
                << " stage_max_ms=" << ms(t_stage->max_ns()) << " stage_over5=" << t_stage->over(0)
                << " stage_over14=" << t_stage->over(1) << " age_n=" << t_age->count()
                << " age_p50_le_ms=" << ms(t_age->quantile_upper_ns(0.5))
                << " age_p99_le_ms=" << ms(t_age->quantile_upper_ns(0.99))
                << " age_max_ms=" << ms(t_age->max_ns()) << " age_over_max=" << t_age->over(0));
    }
};
#undef REFERENCE_LOG

std::vector<std::string> new_lines(const ProfileTiming& timing, ProfileTimingWhen when) {
    ProfileTimingFigures f;
    svc::freeze_profile_timing(timing, f);
    f.when = when;
    std::vector<std::string> lines;
    svc::append_profile_timing_lines(f, lines);
    return lines;
}
std::vector<std::string> old_lines(const ReferenceTiming& timing, const char* when) {
    std::vector<std::string> lines;
    timing.log(when, lines);
    return lines;
}

// A deterministic spread of samples: from a few microseconds to past every histogram's last bin
// (so quantiles fall into the overflow and report the exact maximum), and a negative one.
struct Samples {
    std::uint64_t state = 0x2545f4914f6cdd1dULL;
    std::int64_t next(std::int64_t max_ns) {
        state = state * 6364136223846793005ULL + 1442695040888963407ULL;
        return static_cast<std::int64_t>((state >> 11) % static_cast<std::uint64_t>(max_ns));
    }
};
void fill(ReferenceTiming& t, std::size_t n, bool full) {
    Samples s;
    for (std::size_t i = 0; i < n; ++i) {
        rns::NativePutAudit a;
        a.clock_valid = i % 37 != 0;  // some audits do not belong to their frame
        const std::int64_t host = 1000 * kMs, taken = host + s.next(3 * kMs), built = taken + s.next(30 * kMs);
        a.put_begin_ns = built + s.next(kMs / 2);
        a.put_end_ns = a.put_begin_ns + s.next(i % 50 == 0 ? 30 * kMs : 2 * kMs);
        t.observe(host - s.next(130 * kMs), taken, built, a);
        t.observe_cpu(i % 41 == 0 ? -1 : s.next(40 * kMs));
        if (!full) continue;
        t.observe_t(s.next(i % 30 == 0 ? 150 * kMs : 12 * kMs), s.next(1200 * kMs));
        t.observe_semantic_age(i % 23 == 0 ? -1 : s.next(250 * kMs), s.next(250 * kMs),
                               static_cast<double>(s.next(1000)) / 7919.0);
        t.observe_semantic_publish(1000, 1000 + s.next(20 * kMs), i % 3 != 0);
        for (std::size_t k = 0; k < ProfileTiming::kPerGroup; ++k) {
            t.stages[k].add(s.next(k == 2 ? 120 * kMs : 25 * kMs));                       // normal
            if (i % 4 == 0) t.stages[2 * ProfileTiming::kPerGroup + k].add(s.next(9 * kMs));  // fit failed
        }
        if (i == 7) t.stages[0].add(-5);  // invalid, never counted
    }
}

void test_text() {
    std::size_t compared = 0;
    for (int kind = 0; kind < 3; ++kind) {
        ReferenceTiming t;
        if (kind == 0) {
            t.enable_stages();
            t.enable_t(300 * kMs);
            t.enable_semantic(100 * kMs);
            fill(t, 4000, true);
        } else if (kind == 1) {
            fill(t, 900, false);
        }
        for (const auto when : {ProfileTimingWhen::Periodic, ProfileTimingWhen::Stop}) {
            const auto expected = old_lines(t, when == ProfileTimingWhen::Stop ? "stop" : "periodic");
            const auto got = new_lines(t, when);
            CHECK(got.size() == expected.size());
            for (std::size_t i = 0; i < got.size(); ++i) {
                if (got[i] != expected[i]) std::cerr << "old: " << expected[i] << "\nnew: " << got[i] << '\n';
                CHECK(got[i] == expected[i]);
            }
            compared += got.size();
        }
        if (kind == 0) CHECK(old_lines(t, "stop").size() == 5);  // TIMING, 2 stage routes, S, T
    }
    // Through the real logger (synchronous mode), as the Status thread logs a taken group.
    ReferenceTiming t;
    t.enable_stages();
    t.enable_t(300 * kMs);
    t.enable_semantic(100 * kMs);
    fill(t, 600, true);
    ProfileTimingHandoff handoff;
    CHECK(handoff.offer(t, ProfileTimingOwner::Configured));
    std::vector<std::string> lines;
    std::ostringstream captured;
    auto* const saved = std::cout.rdbuf(captured.rdbuf());
    const auto taken = handoff.take_lines(lines, [](const std::string& line) { LOG_INFO_M("Service", line); });
    std::cout.rdbuf(saved);
    CHECK(taken == 1);
    const auto expected = old_lines(t, "periodic");
    std::istringstream logged(captured.str());
    std::vector<std::string> logged_lines;
    for (std::string l; std::getline(logged, l);) logged_lines.push_back(l);
    CHECK(logged_lines.size() == expected.size() + 1);
    CHECK(logged_lines[0].find("[INFO] [Service] [") != std::string::npos);
    CHECK(logged_lines[0].find("] PROFILE_TIMING_OBSERVED when=periodic seq=1 owner=configured owner_thread=") !=
          std::string::npos);
    for (std::size_t i = 0; i < expected.size(); ++i) {
        const auto& l = logged_lines[i + 1];
        CHECK(l.find("[INFO] [Service] [") != std::string::npos);
        CHECK(l.size() > expected[i].size() + 2 && l.compare(l.size() - expected[i].size() - 2, 2, "] ") == 0 &&
              l.compare(l.size() - expected[i].size(), expected[i].size(), expected[i]) == 0);
    }
    std::cout << "text: " << compared << " lines equal the frozen ProfileTiming::log() byte for byte (full, bare, "
              << "empty ledger x periodic/stop); through the logger " << expected.size() << " lines plus OBSERVED\n";
}

// ---- handoff rules ----
// The numbers of one taken group: seq, dropped, when, and PROFILE_TIMING's frameset_..._n.
struct Group {
    long long seq = -1, dropped = -1, n = -1;
    std::string when, owner, owner_thread;
};
void no_emit(const std::string&) {}
std::string word_after(const std::string& line, std::string_view key) {
    const auto at = line.find(key);
    if (at == std::string::npos) return {};
    const auto from = at + key.size();
    return line.substr(from, line.find(' ', from) - from);
}
std::vector<Group> groups(const std::vector<std::string>& lines) {
    std::vector<Group> out;
    for (const auto& l : lines) {
        if (l.find("PROFILE_TIMING_OBSERVED ") != std::string::npos) {
            Group g;
            g.seq = std::atoll(word_after(l, " seq=").c_str());
            g.dropped = std::atoll(word_after(l, " dropped=").c_str());
            g.when = word_after(l, " when=");
            g.owner = word_after(l, " owner=");
            g.owner_thread = word_after(l, " owner_thread=");
            out.push_back(g);
        } else if (l.find("PROFILE_TIMING when=") != std::string::npos && !out.empty()) {
            out.back().n = std::atoll(word_after(l, " frameset_return_to_put_return_n=").c_str());
        }
    }
    return out;
}
void add_one(ProfileTiming& t) {
    rns::NativePutAudit a;
    a.clock_valid = true;
    a.put_begin_ns = 20 * kMs;
    a.put_end_ns = 21 * kMs;
    t.observe(1 * kMs, 2 * kMs, 10 * kMs, a);
}

void test_handoff_rules() {
    {  // full: four wait, the fifth is dropped; the next carries the gap, the count and its samples
        ProfileTiming t;
        ProfileTimingHandoff h;
        for (int i = 1; i <= 5; ++i) {
            add_one(t);
            CHECK(h.offer(t, ProfileTimingOwner::Unconfigured) == (i <= 4));
        }
        CHECK(h.dropped() == 1);
        std::vector<std::string> lines;
        CHECK(h.take_lines(lines, no_emit) == 4);
        auto g = groups(lines);
        CHECK(g.size() == 4);
        for (int i = 0; i < 4; ++i)
            CHECK(g[i].seq == i + 1 && g[i].dropped == 0 && g[i].n == i + 1 && g[i].when == "periodic" &&
                  g[i].owner == "unconfigured");
        add_one(t);
        CHECK(h.offer(t, ProfileTimingOwner::Unconfigured));
        lines.clear();
        CHECK(h.take_lines(lines, no_emit) == 1);
        g = groups(lines);
        CHECK(g.size() == 1 && g[0].seq == 6 && g[0].dropped == 1 && g[0].n == 6);  // cumulative, not a window
        lines.clear();
        CHECK(h.take_lines(lines, no_emit) == 0 && lines.empty());
    }
    {  // a take returns the frozen figures, not the live ledger
        ProfileTiming t;
        ProfileTimingHandoff h;
        add_one(t);
        CHECK(h.offer(t, ProfileTimingOwner::Configured));
        for (int i = 0; i < 100; ++i) add_one(t);
        std::vector<std::string> lines;
        CHECK(h.take_lines(lines, no_emit) == 1 && groups(lines)[0].n == 1);
    }
    {  // close(): what waits, then one stop group; nothing to take afterwards
        ProfileTiming t;
        ProfileTimingHandoff h;
        add_one(t);
        CHECK(h.offer(t, ProfileTimingOwner::Configured));
        add_one(t);
        CHECK(h.offer(t, ProfileTimingOwner::Configured));
        add_one(t);
        std::vector<std::string> lines;
        h.close(t, ProfileTimingOwner::Configured, lines);
        const auto g = groups(lines);
        CHECK(g.size() == 3);
        CHECK(g[0].seq == 1 && g[0].when == "periodic" && g[0].n == 1);
        CHECK(g[1].seq == 2 && g[1].when == "periodic" && g[1].n == 2);
        CHECK(g[2].seq == 3 && g[2].when == "stop" && g[2].n == 3);
        CHECK(lines.back().find("PROFILE_TIMING when=stop") != std::string::npos);
        add_one(t);
        CHECK(h.offer(t, ProfileTimingOwner::Configured));  // misuse after close: nothing takes it
        std::vector<std::string> after;
        CHECK(h.take_lines(after, no_emit) == 0 && after.empty());
    }
    // A consumer racing close(): no group twice, none lost, the stop group from close() and logged last.
    std::size_t by_consumer = 0, by_close = 0, mixed = 0;  // who took the three periodic groups
    for (int round = 0; round < 300; ++round) {
        ProfileTiming t;
        ProfileTimingHandoff h;
        std::vector<std::string> taken, closing;
        // The log as the logger would order it: the taker's lines as it emits them (slowly, 5 us a
        // line), the closing thread's after close() returns, as the loops log them.
        std::mutex log_mutex;
        std::vector<std::string> log;
        std::atomic<bool> closed{false}, started{false};
        std::thread consumer([&] {
            std::vector<std::string> lines;
            const auto emit = [&](const std::string& line) {
                std::this_thread::sleep_for(std::chrono::microseconds(5));
                std::lock_guard<std::mutex> lock(log_mutex);
                log.push_back(line);
            };
            started.store(true, std::memory_order_release);
            while (!closed.load(std::memory_order_acquire)) {
                h.take_lines(lines, emit);
                taken.insert(taken.end(), lines.begin(), lines.end());
                if (round % 3) std::this_thread::sleep_for(std::chrono::microseconds(25 * (round % 3)));
            }
        });
        while (!started.load(std::memory_order_acquire)) std::this_thread::yield();
        for (int i = 0; i < 3; ++i) {
            add_one(t);
            (void)h.offer(t, ProfileTimingOwner::Configured);
            if (round % 2) std::this_thread::sleep_for(std::chrono::microseconds(round % 40));  // vary where close() lands
        }
        add_one(t);
        h.close(t, ProfileTimingOwner::Configured, closing);
        {
            std::lock_guard<std::mutex> lock(log_mutex);
            log.insert(log.end(), closing.begin(), closing.end());
        }
        closed.store(true, std::memory_order_release);
        consumer.join();
        std::vector<long long> logged_seqs;
        for (const auto& g : groups(log)) logged_seqs.push_back(g.seq);
        CHECK((logged_seqs == std::vector<long long>{1, 2, 3, 4}));  // the stop group logged last
        const auto a = groups(taken), b = groups(closing);
        CHECK(a.size() + b.size() == 4 && !b.empty());
        std::vector<long long> seqs;
        for (const auto& g : a) { seqs.push_back(g.seq); CHECK(g.when == "periodic"); }
        for (const auto& g : b) seqs.push_back(g.seq);
        CHECK((seqs == std::vector<long long>{1, 2, 3, 4}));
        CHECK(b.back().when == "stop" && b.back().n == 4);
        by_consumer += b.size() == 1;
        by_close += a.empty();
        mixed += !a.empty() && b.size() > 1;
    }
    // offer() on a watched thread: no heap call, ever (the figures are frozen into a fixed slot).
    {
        ReferenceTiming t;
        t.enable_stages();
        t.enable_t(300 * kMs);
        t.enable_semantic(100 * kMs);
        fill(t, 300, true);
        ProfileTimingHandoff h;
        std::vector<std::string> lines;
        lines.reserve(16);
        std::int64_t from = 0, to = 0;
        std::vector<double> offer_us;
        offer_us.reserve(4000);
        std::thread producer([&] {
            t_watch = 7;
            from = mono_ns();
            for (int accepted = 0; accepted < 1000;) {
                const auto a = mono_ns();
                const bool ok = h.offer(t, ProfileTimingOwner::Configured);
                const auto b = mono_ns();
                if (offer_us.size() < offer_us.capacity()) offer_us.push_back(static_cast<double>(b - a) / 1e3);
                if (ok) ++accepted;
                else std::this_thread::yield();
            }
            to = mono_ns();
            t_watch = -1;
        });
        std::size_t taken = 0;
        while (taken < 1000) {
            lines.clear();
            taken += h.take_lines(lines, no_emit);
        }
        producer.join();
        CHECK(heap_calls(7, from - 1, to + 1, false) == 0 && heap_calls(7, from - 1, to + 1, true) == 0);
        std::sort(offer_us.begin(), offer_us.end());
        std::cout << "handoff: full/drop/cumulative/frozen/close rules hold; close() racing a taker 300x: the "
                  << "periodic groups taken all by the taker " << by_consumer << ", all by close() " << by_close
                  << ", split " << mixed << ", never twice or lost; 1000 offers accepted ("
                  << h.dropped() << " dropped while the taker lagged), 0 heap calls on the offering thread ("
                  << kHeapLayer << "); one offer() of a full ledger (freeze incl. quantile scans) p50 "
                  << offer_us[offer_us.size() / 2] << " us max " << offer_us.back() << " us; sizeof(figures) "
                  << sizeof(ProfileTimingFigures) << " B\n";
    }
}

// ---- both real Profile loops across three 10 s marks ----
// The unconfigured loop's owners: its own mailbox, channel, transport and handoff.
struct UnconfiguredOwners {
    nd::Mailbox mailbox{geo::kGeometryPixels};
    svc::ProfileCounters counters;
    ProfileTimingHandoff timing_handoff;
    rns::SystemPublishClock publish_clock;
    rns::ProcessEnvelope envelope{"robot_1", "abcdef01", 5000};
    RecordingTransport transport;
    rns::PublisherChannel channel{rns::Topic::Profile, envelope, publish_clock, transport, 131072, 262144};
    int clock_context = 0;
    rns::EstimatedCaptureClock capture_clock{{1, "profile thread host test"}, &clock_context, 1, mono_now};
    std::uint64_t stream = mailbox.begin_stream(), sequence = 0;
    svc::ProfileUnconfiguredInputs inputs() {
        return {mailbox, geo::kGeometryPixels, {.5F, -155.0F / 128, 1.0F / 128, 6.F, 311, 1}, capture_clock.native_clock(),
                channel, nullptr, {&transport, &RecordingTransport::read_audit}, timing_handoff};
    }
};
std::string thread_text(std::thread::id id) {
    std::ostringstream out;
    out << id;
    return out.str();
}

void test_real_loops() {
    constexpr std::int64_t kFramePeriodMs = 50, kRunMs = 31500, kLagMs = 30;
    ProfileOwners c(300 * kMs, 100 * kMs, 15 * kMs);
    c.light_floor();  // both loops take this raster; a step well inside the 50 ms frame period on the 1.8 too
    UnconfiguredOwners u;
    // Two stop flags: the configured loop is stopped and joined first, so the two stop groups,
    // logged by the loops' own threads, never interleave in the log.
    std::atomic<bool> c_finish{false}, u_finish{false}, stop{false};
    std::string c_error, u_error;
    std::ostringstream captured;
    auto* const saved = std::cout.rdbuf(captured.rdbuf());  // only the logger writes until restored
    std::thread configured([&] {
        t_watch = 0;
        try { svc::run_profile_steady(c_finish, c.inputs(), c.counters); } catch (const std::exception& e) { c_error = e.what(); }
    });
    std::thread unconfigured([&] {
        t_watch = 1;
        try { svc::run_profile_unconfigured(u_finish, u.inputs(), u.counters); } catch (const std::exception& e) { u_error = e.what(); }
    });
    // The Status owner's part: every 10 ms take the configured loop's groups and log them.
    std::vector<std::string> status_taken;
    std::thread status([&] {
        std::vector<std::string> lines;
        while (!stop.load(std::memory_order_acquire)) {
            if (c.timing_handoff.take_lines(lines, [](const std::string& line) { LOG_INFO_M("Service", line); }))
                status_taken.insert(status_taken.end(), lines.begin(), lines.end());
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });
    const std::string configured_id = thread_text(configured.get_id()), unconfigured_id = thread_text(unconfigured.get_id()),
        status_id = thread_text(status.get_id());
    for (int i = 0; i < 500 && !c.t_ready.load(); ++i) std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // The camera, and the slow line on every other frame (S then T, the previous frame's stamp).
    const rns::Objects empty;
    const auto t0 = std::chrono::steady_clock::now();
    geo::CaptureStamp previous{};
    for (std::int64_t k = 0; k * kFramePeriodMs < kRunMs; ++k) {
        std::this_thread::sleep_until(t0 + std::chrono::milliseconds(kFramePeriodMs * k));
        const std::int64_t now = mono_ns();
        geo::CaptureStamp stamp;
        (void)c.offer(now, kLagMs * kMs, stamp);
        nd::View v;
        v.metadata.intrinsics = c.b.intrinsics;
        v.metadata.distortion = c.b.distortion;
        v.metadata.unit_m = c.b.unit_m;
        v.metadata.source = stamp.source;
        v.metadata.source.stream_id = u.stream;
        v.metadata.source.sequence = ++u.sequence;
        v.pre_d2c = true;
        v.pixels = c.raster.data();
        v.bytes = c.raster.size() * sizeof(c.raster[0]);
        (void)u.mailbox.try_offer(v);
        if (k % 2 == 1) {
            c.s_slot.publish(empty, previous, svc::kSemanticBaseRevision);
            (void)c.t_slot.publish(c.mask.data(), 1280, 800, 1280, previous, svc::kTProjectionRevision);
        }
        previous = stamp;
    }
    const std::int64_t c_finish_ns = mono_ns();
    c_finish.store(true, std::memory_order_release);
    configured.join();
    const std::int64_t u_finish_ns = mono_ns();
    u_finish.store(true, std::memory_order_release);
    unconfigured.join();
    stop.store(true, std::memory_order_release);
    status.join();
    std::cout.rdbuf(saved);

    CHECK(c_error.empty() && u_error.empty());
    CHECK(c.transport.size() > 0 && u.transport.size() > 0);
    CHECK(heap_event_count.load() <= kHeapEvents);
    const std::int64_t c_from = c.transport[0].put_begin_ns, u_from = u.transport[0].put_begin_ns;
    const std::size_t c_alloc = heap_calls(0, c_from, c_finish_ns, false), c_free = heap_calls(0, c_from, c_finish_ns, true);
    const std::size_t u_alloc = heap_calls(1, u_from, u_finish_ns, false), u_free = heap_calls(1, u_from, u_finish_ns, true);
    const std::size_t c_setup = heap_calls(0, 0, c_from, false), c_stop = heap_calls(0, c_finish_ns, INT64_MAX, false);
    std::cout << "real loops: " << kRunMs / 1000.0 << " s, frames every " << kFramePeriodMs << " ms; configured puts="
              << c.counters.puts.load() << " t_fresh=" << c.counters.t_fresh.load() << ", unconfigured puts="
              << u.counters.puts.load() << "\n  heap calls (" << kHeapLayer << ") from the first put to the stop request: "
              << "configured " << c_alloc << " alloc / " << c_free << " free, unconfigured " << u_alloc << " / " << u_free
              << "; outside it on the configured thread: setup " << c_setup << ", stop " << c_stop << "\n";
    CHECK(c_alloc == 0 && c_free == 0 && u_alloc == 0 && u_free == 0);
    // What was logged, in order, with the logging thread of each line.
    std::istringstream text(captured.str());
    std::vector<std::string> logged;
    for (std::string l; std::getline(text, l);) logged.push_back(l);
    const auto by_owner = [&](const char* owner) {
        std::vector<std::string> mine;
        bool in_group = false;
        for (const auto& l : logged) {
            if (l.find("PROFILE_TIMING_OBSERVED ") != std::string::npos)
                in_group = l.find(std::string(" owner=") + owner + " ") != std::string::npos;
            if (in_group && l.find("PROFILE_") != std::string::npos) mine.push_back(l);
        }
        return mine;
    };
    const auto c_lines = by_owner("configured"), u_lines = by_owner("unconfigured");
    const auto cg = groups(c_lines), ug = groups(u_lines);
    // Configured: seq 1-3 periodic from the Status-like thread, then seq 4 stop from the Profile thread.
    CHECK(cg.size() == 4 && ug.size() == 4);
    for (std::size_t i = 0; i < 4; ++i) {
        CHECK(cg[i].seq == static_cast<long long>(i + 1) && ug[i].seq == static_cast<long long>(i + 1));
        CHECK(cg[i].dropped == 0 && ug[i].dropped == 0);
        CHECK(cg[i].when == (i < 3 ? "periodic" : "stop") && ug[i].when == (i < 3 ? "periodic" : "stop"));
        CHECK(cg[i].owner_thread == configured_id && ug[i].owner_thread == unconfigured_id);
        if (i) CHECK(cg[i].n >= cg[i - 1].n && ug[i].n > ug[i - 1].n);
    }
    CHECK(cg[3].n == static_cast<long long>(c.counters.puts.load()) && ug[3].n == static_cast<long long>(u.counters.puts.load()));
    CHECK(groups(status_taken).size() == 3);
    const auto logged_by = [&](const std::string& line, const std::string& id) {
        return line.find("] [" + id + "] ") != std::string::npos;
    };
    std::size_t c_periodic_by_status = 0, c_stop_by_profile = 0, u_by_profile = 0, stage_lines = 0, t_lines = 0, s_lines = 0;
    for (const auto& l : c_lines) {
        const bool stop_line = l.find("when=stop") != std::string::npos;
        if (!stop_line) c_periodic_by_status += logged_by(l, status_id);
        else c_stop_by_profile += logged_by(l, configured_id);
        stage_lines += l.find("PROFILE_STAGE_TIMING when=") != std::string::npos;
        t_lines += l.find("PROFILE_T_TIMING when=") != std::string::npos;
        s_lines += l.find("PROFILE_SEMANTIC_AGE when=") != std::string::npos;
    }
    for (const auto& l : u_lines) {
        u_by_profile += logged_by(l, unconfigured_id);
        CHECK(l.find("PROFILE_STAGE_TIMING") == std::string::npos && l.find("PROFILE_T_TIMING") == std::string::npos);
    }
    CHECK(c_periodic_by_status == c_lines.size() - c_stop_by_profile && c_stop_by_profile == c_lines.size() / 4);
    CHECK(u_by_profile == u_lines.size());
    CHECK(stage_lines == 4 && t_lines == 4 && s_lines == 4);
    // The stop group of each loop is the last of its lines in the log.
    CHECK(c_lines.back().find("when=stop") != std::string::npos && u_lines.back().find("when=stop") != std::string::npos);
    CHECK(c.timing_handoff.dropped() == 0 && u.timing_handoff.dropped() == 0);
    std::cout << "  groups: configured seq 1-3 periodic logged by the Status-like thread, seq 4 stop by its own thread; "
              << "unconfigured seq 1-3 periodic and seq 4 stop, all by its own thread at stop; dropped 0\n";
}
}  // namespace

int main() {
    // The real logger in its synchronous mode: what it writes is read back from std::cout.
    ::unsetenv("PERCEPTION_ASYNC_LOG");
    test_text();
    test_handoff_rules();
    test_real_loops();
    std::cout << "profile timing log PASS\n";
    return 0;
}
