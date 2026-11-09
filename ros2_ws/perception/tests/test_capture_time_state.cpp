/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_capture_time_state.cpp
 * Brief: The capture clock's runtime switch and the two Status reasons made from its counters.
 *
 * Description:
 * What it checks (common/capture_time_state.hpp), without a camera. The capture thread's side is
 * written as it is in orbbec_capture.cpp: a withheld frameset counts and sets withheld_now; a
 * stamped one counts, sets estimated_now to its grade and clears withheld_now.
 *   switch    capture_min_path_us has one spelling per value, 10000..100000 whole microseconds,
 *             and no default: a missing or malformed value throws instead of becoming a number.
 *   reasons   seen the way the Status thread sees them, a look every 10 ms:
 *             - nothing withheld and nothing estimated raises nothing, however many framesets
 *               are stamped;
 *             - the first second of a device stream (30 framesets withheld, one every 33 ms) is
 *               ONE episode of capture_time_unavailable without a break, ending 1.5 s after the
 *               last withheld frameset: no edge per frameset;
 *             - a single withheld frameset is still up when the next 1 Hz Status goes out;
 *             - no recovery without evidence (GPT question 16.4): one withheld frameset and then
 *               no frame at all keeps the reason up for as long as nothing is stamped; the first
 *               later stamped frameset lets it go;
 *             - t_capture_estimated stays up while framesets are stamped in the estimated grade
 *               and goes only after one is stamped in the measured grade; independent of the
 *               other reason;
 *             - framesets withheld before the Status thread's first look are reported by it.
 *   limit     the run of withheld framesets as the capture thread keeps it (GPT question 16.3,
 *             Fable question 11.2): one line when it starts and one when it ends; the settling
 *             second opens the run but does not start the deadline; the deadline is 5 s of host
 *             time from the first non-settling withheld frameset, checked on withheld framesets,
 *             on frame wait timeouts (overdue) and on the stamped frameset that would end the run,
 *             which cannot clear it after the deadline; sparse framesets end it on time too; a run
 *             that ended within the deadline leaves no trace in the next one.
 */

#include "common/capture_time_state.hpp"

#include <iostream>
#include <stdexcept>
#include <string>

namespace {
using perception::CaptureTimeCounters;
using perception::CaptureTimeReasonWatch;
using perception::CaptureTimeWithheldRun;
using Step = CaptureTimeWithheldRun::Step;

int checks = 0;
void check(bool ok, int line) {
    ++checks;
    if (!ok) throw std::runtime_error("capture time state line " + std::to_string(line));
}
#define CHECK(x) check(static_cast<bool>(x), __LINE__)

bool throws(const char* text) {
    try {
        (void)perception::parse_capture_min_path_us(text);
    } catch (const std::invalid_argument&) {
        return true;
    }
    return false;
}

// The capture thread's side (orbbec_capture.cpp).
void withhold(CaptureTimeCounters& c) {
    c.withheld.fetch_add(1);
    c.withheld_now = true;
}
void stamp(CaptureTimeCounters& c, bool estimated) {
    c.stamped.fetch_add(1);
    if (estimated) c.estimated.fetch_add(1);
    c.estimated_now = estimated;
    c.withheld_now = false;
}

void test_switch() {
    CHECK(perception::parse_capture_min_path_us("30767") == 30767);
    CHECK(perception::parse_capture_min_path_us("10000") == 10000);
    CHECK(perception::parse_capture_min_path_us("100000") == 100000);
    CHECK(throws(nullptr));
    for (const char* bad : {"", "0", "23", "9999", "100001", "030767", "30767 ", " 30767", "30767.0", "3e4", "-30767",
                            "+30767", "0x7fff", "30,767", "1000000", "unset", "off"})
        CHECK(throws(bad) && !perception::valid_capture_min_path_text(bad));
    CHECK(perception::valid_capture_min_path_text("46039"));
    CHECK(&perception::capture_time_counters() == &perception::capture_time_counters());
}

void test_nothing_raised_while_all_is_stamped() {
    CaptureTimeCounters counters;
    CaptureTimeReasonWatch watch;
    for (std::int64_t now = 0; now < 20000; now += 10) {
        if (now % 30 == 0) stamp(counters, false);
        const auto r = watch.poll(counters, now);
        CHECK(!r.estimated && !r.unavailable);
    }
}

// The first second of a device stream as the Status thread sees it.
void test_start_is_one_episode() {
    CaptureTimeCounters counters;
    CaptureTimeReasonWatch watch;
    int edges = 0, last_withheld_ms = -1;
    bool previous = false;
    std::int64_t first_up = -1, last_up = -1;
    for (std::int64_t now = 0; now < 6000; now += 10) {
        // framesets at 1000, 1033, ... ms: the first 30 are withheld, the rest stamped
        if (now >= 1000 && (now - 1000) % 33 < 10) {
            const auto frame = (now - 1000) / 33;
            if (frame < 30) {
                withhold(counters);
                counters.settling.fetch_add(1);
                last_withheld_ms = static_cast<int>(now);
            } else {
                stamp(counters, false);
            }
        }
        const auto r = watch.poll(counters, now);
        CHECK(!r.estimated);
        if (r.unavailable != previous) ++edges;
        previous = r.unavailable;
        if (r.unavailable) {
            if (first_up < 0) first_up = now;
            last_up = now;
        }
    }
    CHECK(edges == 2);                                   // up once, down once
    CHECK(first_up >= 1000 && first_up < 1010);
    CHECK(last_up == last_withheld_ms + perception::kCaptureTimeReasonHoldMs);
    CHECK(counters.withheld.load() == 30 && counters.settling.load() == 30);
}

void test_one_frameset_reaches_a_status() {
    for (std::int64_t at : {5, 400, 999}) {             // wherever in the second it happens
        CaptureTimeCounters c;
        CaptureTimeReasonWatch w;
        bool on_a_status = false;
        for (std::int64_t now = 0; now < 4000; now += 10) {
            if (now <= at && at < now + 10) withhold(c);
            if (now > at && now % 33 == 0) stamp(c, false);  // frames go on, stamped again
            const auto r = w.poll(c, now);
            if (now % 1000 == 0 && now > at) on_a_status = on_a_status || r.unavailable;   // the 1 Hz Status
        }
        CHECK(on_a_status);
    }
}

// GPT question 16.4: one withheld frameset and then no frame at all. Time passing is not a
// recovery: the reason stays up as long as nothing is stamped, and the first later stamped
// frameset lets it go (the hold after the last withheld one has long passed).
void test_no_recovery_without_a_stamped_frame() {
    CaptureTimeCounters counters;
    CaptureTimeReasonWatch watch;
    bool cleared = false;
    for (std::int64_t now = 0; now <= 8000; now += 10) {
        if (now == 100) withhold(counters);
        const auto r = watch.poll(counters, now);
        if (now >= 100) cleared = cleared || !r.unavailable;
    }
    CHECK(!cleared);
    stamp(counters, false);
    CHECK(!watch.poll(counters, 8010).unavailable);
    // Within the hold a stamped frameset does not cut the episode short.
    withhold(counters);
    CHECK(watch.poll(counters, 9000).unavailable);
    stamp(counters, false);
    CHECK(watch.poll(counters, 9010).unavailable);
    CHECK(watch.poll(counters, 9000 + perception::kCaptureTimeReasonHoldMs).unavailable);
    CHECK(!watch.poll(counters, 9001 + perception::kCaptureTimeReasonHoldMs).unavailable);
}

void test_estimated_is_its_own_reason() {
    CaptureTimeCounters counters;
    CaptureTimeReasonWatch watch;
    CHECK(!watch.poll(counters, 0).estimated);
    stamp(counters, true);
    auto r = watch.poll(counters, 100);
    CHECK(r.estimated && !r.unavailable);
    // Still estimated framesets, then none at all: it stays up long after the hold.
    for (std::int64_t now = 200; now < 6000; now += 10) {
        if (now < 3000 && now % 33 == 0) stamp(counters, true);
        CHECK(watch.poll(counters, now).estimated);
    }
    // One measured stamp lets it go once the hold after the last estimated one has passed.
    stamp(counters, false);
    CHECK(!watch.poll(counters, 6000).estimated);
    // Independent of the other reason.
    withhold(counters);
    r = watch.poll(counters, 7000);
    CHECK(!r.estimated && r.unavailable);
    stamp(counters, true);
    r = watch.poll(counters, 7100);
    CHECK(r.estimated && r.unavailable);                  // unavailable: within its hold
    r = watch.poll(counters, 7000 + perception::kCaptureTimeReasonHoldMs + 1);
    CHECK(r.estimated && !r.unavailable);
    // A clock that goes back does not end a reason early.
    stamp(counters, false);
    r = watch.poll(counters, 7050);
    CHECK(r.estimated);
}

void test_withheld_before_the_first_look() {
    CaptureTimeCounters counters;
    for (int k = 0; k < 12; ++k) withhold(counters);
    CaptureTimeReasonWatch watch;
    CHECK(watch.poll(counters, 777).unavailable);
    CHECK(watch.poll(counters, 777 + perception::kCaptureTimeReasonHoldMs + 1).unavailable);   // nothing stamped yet
    stamp(counters, false);
    CHECK(!watch.poll(counters, 777 + perception::kCaptureTimeReasonHoldMs + 2).unavailable);
}

void test_withheld_run_and_limit() {
    constexpr std::int64_t kPeriod = 33366700, kSecond = INT64_C(1000000000);
    CHECK(perception::kCaptureTimeWithheldLimitNs == 5 * kSecond);
    // The first second of a device stream: 40 settling framesets (as on the device, 2026-10-14),
    // then stamped. One Started, 39 Continues, one Resumed; the deadline never starts.
    CaptureTimeWithheldRun run;
    std::int64_t t = 7 * kSecond;
    CHECK(run.withheld(t, true) == Step::Started && run.since_ns() == t && run.limit_from_ns() < 0);
    for (int k = 1; k < 40; ++k) CHECK(run.withheld(t + k * kPeriod / 2, true) == Step::Continues);
    CHECK(run.framesets() == 40 && run.limit_from_ns() < 0 && !run.overdue(t + 60 * kSecond));
    CHECK(run.stamped(t + 40 * kPeriod / 2) == Step::Resumed && run.ended_framesets() == 40 && run.framesets() == 0 && run.since_ns() == t);
    CHECK(run.stamped(t + 41 * kPeriod / 2) == Step::Stamped && run.ended_framesets() == 40);
    // Settling, then lost (Fable 11.2): the deadline starts at the first non-settling frameset,
    // so the SDK's mapping has its full 5 s after the settling second.
    CaptureTimeWithheldRun lost;
    t = 100 * kSecond;
    int too_long = -1;
    for (int k = 0; k < 400 && too_long < 0; ++k) {
        const auto step = lost.withheld(t + k * kPeriod, k < 30);
        if (k == 0) CHECK(step == Step::Started);
        if (step == Step::TooLong) too_long = k;
    }
    CHECK(lost.limit_from_ns() == t + 30 * kPeriod);
    CHECK(too_long == 30 + 150);                          // 150 x 33.37 ms = 5.005 s, 149 x = 4.972 s
    // GPT 16.3: withheld until 4.9 s, then a stamped frameset at 5.8 s does not clear the run.
    CaptureTimeWithheldRun late;
    CHECK(late.withheld(0, false) == Step::Started);
    CHECK(late.withheld(4900000000, false) == Step::Continues);
    CHECK(late.stamped(5800000000) == Step::TooLong && late.framesets() == 2);
    // GPT 16.3: withheld until 4.9 s, then frame wait timeouts: overdue from 5 s on, on host time.
    CaptureTimeWithheldRun waits;
    CHECK(waits.withheld(0, false) == Step::Started);
    CHECK(waits.withheld(4900000000, false) == Step::Continues);
    CHECK(!waits.overdue(4999999999) && waits.overdue(5 * kSecond) && waits.overdue(9800000000));
    // Host time, not a frame count: framesets only now and then still end it at 5 s.
    CaptureTimeWithheldRun sparse;
    CHECK(sparse.withheld(0, false) == Step::Started);
    CHECK(sparse.withheld(2 * kSecond, false) == Step::Continues);
    CHECK(sparse.withheld(5 * kSecond - 1, false) == Step::Continues);
    CHECK(sparse.withheld(5 * kSecond, false) == Step::TooLong);
    // A run that ends within the deadline leaves nothing behind: the next one has its own start.
    CaptureTimeWithheldRun again;
    CHECK(again.withheld(0, false) == Step::Started);
    CHECK(again.withheld(4 * kSecond, false) == Step::Continues);
    CHECK(again.stamped(4 * kSecond + 1) == Step::Resumed && again.limit_from_ns() < 0 && !again.overdue(100 * kSecond));
    CHECK(again.withheld(8 * kSecond, false) == Step::Started && again.since_ns() == 8 * kSecond);
    CHECK(again.withheld(12 * kSecond, false) == Step::Continues);
    CHECK(again.withheld(13 * kSecond, false) == Step::TooLong);
    // No run open: nothing is ever overdue.
    CaptureTimeWithheldRun idle;
    CHECK(!idle.overdue(INT64_C(1) << 60) && idle.stamped(10) == Step::Stamped);
}
}  // namespace

int main() {
    try {
        test_switch();
        test_nothing_raised_while_all_is_stamped();
        test_start_is_one_episode();
        test_one_frameset_reaches_a_status();
        test_no_recovery_without_a_stamped_frame();
        test_estimated_is_its_own_reason();
        test_withheld_before_the_first_look();
        test_withheld_run_and_limit();
        std::cout << "capture time state: 8 cases, " << checks << " checks PASS\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
