/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_sdk_stream_thread_placement.cpp
 * Brief: Unit tests for placing the camera SDK's stream threads on one CPU and checking them.
 *
 * Description:
 * What it checks. PERCEPTION_CAPTURE_SDK_THREADS_CPU narrows the calling thread to the target CPU
 * for the pipeline start only, accepts the placement only when exactly the expected number of
 * tasks was born meanwhile, all with exactly the target and none with another set, restores the
 * caller, and later verifies every member: the same task (tid and start time), still on the
 * target.
 * With mocked system calls: strict parsing (only off, 1, 2, 3, 7), the start time in a stat
 * line, the disabled no-op, the order list - narrow - list - restore, a new task with another
 * set, with a superset of the target or with an unreadable set (each refuses the placement,
 * also when the count on the target is right), one that already ended (refused: it may have
 * been one of the expected threads while a child filled the count), too few,
 * too many (a child born between the caller's own threads; tids that wrapped), an unsorted
 * listing, a start time that cannot be read, every failure of begin and of the restore (the
 * caller is restored even when its set cannot be read first), whether the caller is then
 * confirmed restored and that no later placement begins on a thread left narrowed, verify
 * (ended for good, a tid reused by another task, an SDK that restarts a stream, moved, superset,
 * read errors), survey, errno left alone, every field of the log line, that no set but the
 * target and the caller's own is ever requested, the end guard (also on a throw, also with a
 * throwing sink) and the periodic watch (period, log on change of answer, reason or number of
 * ended members, the published state, the one survey). A final Linux-only case runs the real
 * system calls on threads of this test, also from a thread other than the caller, on a CPU of
 * the runner's own set chosen among 2, 3 and 7 (never another core).
 * Trap. end() restores before it judges, except on a thread other than begin()'s, whose set it
 * never touches; the OS may refuse the restore, and caller_restore_unconfirmed() says so. A
 * member that ended never comes back: a tid that appears again is another task.
 */

#include "common/sdk_stream_thread_placement.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {
using namespace perception;
int checks = 0;
void check(bool value) {
    ++checks;
    if (!value) throw std::runtime_error("sdk thread placement check " + std::to_string(checks));
}

constexpr std::uint64_t kOwn = 0x8C;      // CPUs 2, 3, 7
constexpr std::uint64_t kTarget = 0x02;   // CPU 1
constexpr std::uint64_t kStart = 5000;    // start time of every mocked task unless set otherwise
struct Mock {
    std::uint64_t self_mask = kOwn, set_adds = 0;
    std::int64_t self_tid = 100;
    bool read_valid = true, set_takes_effect = true;
    int reads = 0, sets = 0, lists = 0, mask_reads = 0, start_reads = 0;
    int fail_set_at = 0, fail_list_at = 0, invalid_read_at = 0, foreign_tid_at = 0, set_adds_at = 0;
    std::vector<std::uint64_t> requested;
    std::vector<std::string> order;
    std::vector<std::vector<std::int64_t>> listings;   // one per list call; the last one repeats
    std::map<std::int64_t, std::pair<int, std::uint64_t>> masks;    // a task exists when it has an entry
    std::map<std::int64_t, std::pair<int, std::uint64_t>> starts;   // start answers other than {0, kStart}
} mock;
void reset() {
    mock = Mock{};
    sdk_thread_placement_left_narrowed().store(-1);   // process-wide; every case starts clean
}

CaptureAffinitySnapshot read_mock() noexcept {
    ++mock.reads;
    mock.order.push_back("read");
    errno = EIO;   // the placement must not leak a system call's errno
    CaptureAffinitySnapshot out;
    out.tid = mock.reads == mock.foreign_tid_at ? 999 : mock.self_tid;   // the tid is there even when the rest is not
    out.mask = mock.self_mask;   // a read that failed half way may hold the set and still be invalid
    if (!mock.read_valid || mock.reads == mock.invalid_read_at) { out.reason = "mock_invalid"; return out; }
    out.valid = true;
    out.reason = "ok";
    out.observed_ns = 1000 + mock.reads;
    out.mask = mock.self_mask;
    out.policy = out.priority = out.nice = 0;
    return out;
}
int set_mock(std::uint64_t mask) noexcept {
    ++mock.sets;
    mock.requested.push_back(mask);
    mock.order.push_back("set");
    errno = EPERM;
    if (mock.sets == mock.fail_set_at) return EPERM;
    if (mock.set_takes_effect) mock.self_mask = mask | (mock.sets == mock.set_adds_at ? mock.set_adds : 0);
    return 0;
}
bool list_mock(std::vector<std::int64_t>& out) noexcept {
    ++mock.lists;
    mock.order.push_back("list");
    errno = ENOENT;
    if (mock.lists == mock.fail_list_at || mock.listings.empty()) return false;
    const auto index = std::min<std::size_t>(static_cast<std::size_t>(mock.lists) - 1, mock.listings.size() - 1);
    out = mock.listings[index];
    return true;
}
int mask_mock(std::int64_t tid, std::uint64_t& mask) noexcept {
    ++mock.mask_reads;
    errno = ESRCH;
    const auto it = mock.masks.find(tid);
    if (it == mock.masks.end()) { mask = 0; return ESRCH; }
    mask = it->second.second;
    return it->second.first;
}
int start_mock(std::int64_t tid, std::uint64_t& ticks) noexcept {
    ++mock.start_reads;
    errno = ENOENT;
    ticks = 0;
    if (mock.masks.find(tid) == mock.masks.end()) return ESRCH;
    const auto it = mock.starts.find(tid);
    if (it == mock.starts.end()) { ticks = kStart; return 0; }
    ticks = it->second.second;
    return it->second.first;
}
SdkThreadPlacementOps operations() { return {read_mock, set_mock, list_mock, mask_mock, start_mock}; }
bool reason(const SdkThreadPlacementRecord& record, const char* expected) {
    return std::string(record.reason) == expected;
}

// The process before the start: the caller (100) and five other tasks. During the start
// `on_target` new tasks appear on the target (tids 200, 201, ...). With extras, three more that
// are not on the target: 300
// with the process's own set (created by another thread), 302 with the target AND other CPUs,
// 303 whose set cannot be read.
const std::vector<std::int64_t> kBefore = {100, 101, 102, 103, 104, 105};
void stage_start(std::size_t on_target, bool extras = false) {
    std::vector<std::int64_t> after = kBefore;
    for (std::size_t i = 0; i < on_target; ++i) {
        after.push_back(200 + static_cast<std::int64_t>(i));
        mock.masks[200 + static_cast<std::int64_t>(i)] = {0, kTarget};
    }
    if (extras) {
        for (const std::int64_t tid : {300, 302, 303}) after.push_back(tid);
        mock.masks[300] = {0, kOwn};
        mock.masks[302] = {0, kTarget | kOwn};
        mock.masks[303] = {EINVAL, kTarget};
    }
    for (const auto tid : kBefore) mock.masks[tid] = {0, kOwn};
    mock.listings = {kBefore, after};
}
// The process before the start and exactly these new tasks on the target.
void stage_born(const std::vector<std::int64_t>& born) {
    std::vector<std::int64_t> after = kBefore;
    for (const auto tid : born) { after.push_back(tid); mock.masks[tid] = {0, kTarget}; }
    for (const auto tid : kBefore) mock.masks[tid] = {0, kOwn};
    mock.listings = {kBefore, after};
}
bool only_target_or_own_requested() {
    for (const auto mask : mock.requested)
        if (mask != kTarget && mask != kOwn) return false;
    return true;
}
// The family the record holds, which is logged in order of tid.
bool family_is(const SdkThreadPlacementRecord& record, const std::vector<std::int64_t>& tids) {
    if (record.family != tids.size() || tids.size() > record.family_tids.size()) return false;
    return std::vector<std::int64_t>(record.family_tids.begin(), record.family_tids.begin() + static_cast<std::ptrdiff_t>(tids.size())) == tids;
}

void parsing() {
    check(parse_sdk_stream_threads_cpu(nullptr) == -1 && parse_sdk_stream_threads_cpu("off") == -1);
    check(parse_sdk_stream_threads_cpu("1") == 1 && parse_sdk_stream_threads_cpu("2") == 2);
    check(parse_sdk_stream_threads_cpu("3") == 3 && parse_sdk_stream_threads_cpu("7") == 7);
    // Cores 0, 4, 5 and 6 are not perception's; no list, range, padding or other spelling.
    for (const char* bad : {"", "0", "4", "5", "6", "8", "01", " 1", "1 ", "on", "OFF", "2,3", "2-3", "-1", "17"}) {
        bool refused = false;
        try { (void)parse_sdk_stream_threads_cpu(bad); } catch (const std::invalid_argument&) { refused = true; }
        check(refused);
    }
}

// The start time is field 22 of the stat line, counted from the last ')' of the name.
void task_start_parsing() {
    const std::string tail = " S 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 ";   // fields 3 to 21
    std::uint64_t ticks = 7;
    errno = EDOM;
    check(parse_task_start(("77 (cap_loop)" + tail + "424242 99 100\n").c_str(), ticks) && ticks == 424242 && errno == EDOM);
    check(parse_task_start(("77 (a b) (c))" + tail + "31 0").c_str(), ticks) && ticks == 31);   // a name with spaces and brackets
    check(parse_task_start(("77 (x)" + tail + "12").c_str(), ticks) && ticks == 12);            // the line ends with the field
    check(parse_task_start(("77 (x)" + tail + "0 1").c_str(), ticks) && ticks == 0);
    const std::vector<std::string> malformed = {
        "77 x S 1 2", "77 (x)", "77 (x) S 1 2 3", "77 (x)" + tail, "77 (x)" + tail + "x12 3", "77 (x)" + tail + "12x 3",
        "77 (x)" + tail + "-5 3", "77 (x)" + tail + "99999999999999999999999999 3"};
    for (const std::string& bad : malformed) {
        ticks = 7;
        errno = EDOM;
        check(!parse_task_start(bad.c_str(), ticks) && ticks == 0 && errno == EDOM);
    }
    check(!parse_task_start(nullptr, ticks));
}

void disabled_is_a_no_op() {
    reset();
    stage_start(6);
    SdkStreamThreadPlacement placement(-1, 6, operations());
    check(!placement.enabled() && !placement.needs_end() && !placement.placed() && !placement.caller_restore_unconfirmed());
    check(!placement.begin().success && !placement.end().success);
    check(!placement.verify().success && !placement.survey().success);
    check(mock.reads == 0 && mock.sets == 0 && mock.lists == 0 && mock.mask_reads == 0 && mock.start_reads == 0);
    check(!placement.caller_restore_unconfirmed());
}

void placed_family() {
    reset();
    stage_start(6);
    SdkStreamThreadPlacement placement(1, 6, operations());
    check(!placement.caller_restore_unconfirmed());   // nothing was narrowed yet
    errno = EDOM;
    const auto& begun = placement.begin();
    check(errno == EDOM && begun.success && reason(begun, "ok") && placement.needs_end() && !placement.placed());
    check(placement.caller_restore_unconfirmed());    // narrowed, and not restored yet
    check(begun.caller_tid == 100 && begun.original_mask == kOwn && begun.narrowed_mask == kTarget);
    check(begun.tasks_before == 6 && begun.target_cpu == 1 && begun.expected == 6);
    // read own set, list, narrow, read back -- in that order.
    check((mock.order == std::vector<std::string>{"read", "list", "set", "read"}));
    check(mock.self_mask == kTarget && mock.requested == std::vector<std::uint64_t>{kTarget});
    (void)placement.begin();   // a second begin changes nothing
    check(mock.sets == 1 && mock.reads == 2);

    mock.order.clear();
    errno = EDOM;
    const auto& ended = placement.end();
    check(errno == EDOM && ended.success && reason(ended, "ok") && placement.placed() && !placement.needs_end());
    check(!placement.caller_restore_unconfirmed() && sdk_thread_placement_left_narrowed().load() == -1);
    // list first, then restore: read, set, read back.
    check((mock.order == std::vector<std::string>{"list", "read", "set", "read"}));
    check(mock.self_mask == kOwn && ended.restored_mask == kOwn && ended.restore_errno == 0);
    check(ended.tasks_before == 6 && ended.tasks_after == 12 && ended.born == 6 && ended.born_ended == 0);
    check(ended.born_elsewhere == 0 && ended.unreadable == 0 && ended.expected == 6);
    check(family_is(ended, {200, 201, 202, 203, 204, 205}));
    check(only_target_or_own_requested() && mock.sets == 2);
    (void)placement.end();     // a second end changes nothing
    check(mock.sets == 2 && &placement.family() == &ended);

    // verify: the set and the start time of every member, nothing else touched.
    const int reads = mock.reads, sets = mock.sets, lists = mock.lists, mask_reads = mock.mask_reads;
    const int start_reads = mock.start_reads;
    errno = EDOM;
    const auto& fine = placement.verify();
    check(errno == EDOM && fine.success && reason(fine, "ok") && fine.checked == 6);
    check(fine.gone == 0 && fine.moved == 0 && fine.unreadable == 0 && fine.first_bad_tid == -1 && fine.family == 6);
    check(mock.reads == reads && mock.sets == sets && mock.lists == lists);
    check(mock.mask_reads == mask_reads + 6 && mock.start_reads == start_reads + 6);
    // One member with another set, one with the target and more, one whose set or start time
    // cannot be read: a fault while that lasts, and the answer follows the present state.
    mock.masks[201] = {0, kOwn};
    const auto& moved = placement.verify();
    check(!moved.success && reason(moved, "family_mask_changed") && moved.moved == 1 && moved.unreadable == 0 &&
          moved.gone == 0 && moved.first_bad_tid == 201 && moved.first_bad_mask == kOwn);
    mock.masks[201] = {0, kTarget | 0x4};
    check(reason(placement.verify(), "family_mask_changed") && placement.verify().first_bad_mask == (kTarget | 0x4));
    mock.masks[201] = {EINVAL, kTarget};
    const auto& unreadable = placement.verify();
    check(reason(unreadable, "family_read_failed") && unreadable.unreadable == 1 && unreadable.moved == 0 && unreadable.first_bad_tid == 201);
    mock.masks[201] = {0, kTarget};
    mock.starts[204] = {EIO, 0};
    check(reason(placement.verify(), "family_read_failed") && placement.verify().first_bad_tid == 204);
    mock.starts.erase(204);
    mock.masks[203] = {0, kOwn};
    mock.masks[205] = {EINVAL, kTarget};
    const auto& both = placement.verify();   // a moved member is named before an unreadable one
    check(reason(both, "family_mask_changed") && both.moved == 1 && both.unreadable == 1 && both.first_bad_tid == 203);
    mock.masks[203] = {0, kTarget};
    mock.masks[205] = {0, kTarget};
    check(placement.verify().success);

    // survey: the family and two descendants have exactly the target; a task with the target
    // and more, and one whose set cannot be read, do not count.
    std::vector<std::int64_t> now = mock.listings[1];
    for (const std::int64_t tid : {302, 303, 400, 401}) now.push_back(tid);
    mock.masks[302] = {0, kTarget | kOwn};
    mock.masks[303] = {EINVAL, kTarget};
    mock.masks[400] = {0, kTarget};
    mock.masks[401] = {0, kTarget};
    mock.listings.push_back(now);
    errno = EDOM;
    const auto& counted = placement.survey();
    check(errno == EDOM && counted.success && counted.on_target == 8 && counted.tasks_after == 16 && counted.family == 6);
    mock.fail_list_at = mock.lists + 1;
    errno = EDOM;
    check(reason(placement.survey(), "list_tasks_failed") && errno == EDOM);
    check(only_target_or_own_requested() && mock.sets == 2 && mock.self_mask == kOwn);

    // A member that ended is a fault for good. A task with that tid again -- same set, even the
    // same start time -- is not read any more.
    mock.masks.erase(203);
    const auto& gone = placement.verify();
    check(!gone.success && reason(gone, "family_thread_gone") && gone.gone == 1 && gone.checked == 6 && gone.first_bad_tid == 203);
    mock.masks[203] = {0, kTarget};
    const int before = mock.mask_reads;
    const auto& still = placement.verify();
    check(reason(still, "family_thread_gone") && still.checked == 5 && still.gone == 1 && mock.mask_reads == before + 5);
    // An ended member is named before a moved one; both are counted.
    mock.masks[200] = {0, kOwn};
    const auto& two = placement.verify();
    check(reason(two, "family_thread_gone") && two.gone == 1 && two.moved == 1 && two.first_bad_tid == 200 && two.first_bad_mask == kOwn);
}

// Every field of the log line carries its own value.
void log_line() {
    SdkThreadPlacementRecord r;
    r.event = "verify";
    r.target_cpu = 7;
    r.success = false;
    r.reason = "family_thread_gone";
    r.caller_tid = 11;
    r.original_mask = 12;
    r.narrowed_mask = 13;
    r.restored_mask = 14;
    r.set_errno = 15;
    r.restore_errno = 16;
    r.tasks_before = 17;
    r.tasks_after = 18;
    r.born = 19;
    r.born_ended = 20;
    r.born_elsewhere = 21;
    r.family = 3;
    r.expected = 22;
    r.checked = 23;
    r.gone = 24;
    r.moved = 25;
    r.unreadable = 26;
    r.on_target = 27;
    r.first_bad_tid = 28;
    r.first_bad_mask = 29;
    r.family_tids = {31, 32, 33, 34};
    std::ostringstream line;
    line << r;
    check(line.str() ==
          "SDK_THREAD_PLACEMENT schema=2 mask_encoding=cpu_bitset_u64 event=verify target_cpu=7 success=0 "
          "reason=family_thread_gone caller_tid=11 original_mask=12 narrowed_mask=13 restored_mask=14 set_errno=15 "
          "restore_errno=16 tasks_before=17 tasks_after=18 born=19 born_ended=20 born_elsewhere=21 family=3 expected=22 "
          "checked=23 gone=24 moved=25 unreadable=26 on_target=27 first_bad_tid=28 first_bad_mask=29 family_tids=31,32,33");
    r.family = 40;   // more than the record holds: the held ones are shown
    r.success = true;
    std::ostringstream wide;
    wide << r;
    check(wide.str().find(" success=1 ") != std::string::npos && wide.str().find(" family=40 ") != std::string::npos);
    check(wide.str().find(" family_tids=31,32,33,34,0,0,0,0,0,0,0,0,0,0,0,0") != std::string::npos);
    std::ostringstream none;
    none << SdkThreadPlacementRecord{};
    check(none.str().find(" event=begin ") != std::string::npos && none.str().find(" family_tids=none") != std::string::npos);
}

void family_identity() {
    // Fewer than expected on the target: a stream thread was created by some other thread.
    for (const std::size_t on_target : {std::size_t{0}, std::size_t{4}, std::size_t{5}}) {
        reset();
        stage_start(on_target);
        SdkStreamThreadPlacement placement(1, 6, operations());
        check(placement.begin().success);
        const auto& ended = placement.end();
        check(!ended.success && reason(ended, "family_too_small") && ended.family == on_target && !placement.placed());
        check(mock.self_mask == kOwn && ended.restored_mask == kOwn && !placement.caller_restore_unconfirmed());
        check(reason(placement.verify(), "not_placed") && mock.mask_reads == static_cast<int>(on_target));
    }
    // More than expected: a family thread already created a child, and which of them are the
    // caller's own is not known. Not verified, however many there are; the record holds the
    // first sixteen for the log.
    for (const std::size_t on_target : {std::size_t{7}, std::size_t{8}, std::size_t{20}}) {
        reset();
        stage_start(on_target);
        SdkStreamThreadPlacement placement(1, 6, operations());
        check(placement.begin().success);
        const auto& ended = placement.end();
        check(!ended.success && reason(ended, "family_too_large") && !placement.placed() && ended.family == on_target);
        for (std::size_t i = 0; i < ended.family_tids.size(); ++i)
            check(ended.family_tids[i] == (i < on_target ? 200 + static_cast<std::int64_t>(i) : 0));
        check(ended.born == on_target && mock.self_mask == kOwn && !placement.caller_restore_unconfirmed());
        const int before = mock.mask_reads;
        check(reason(placement.verify(), "not_placed") && mock.mask_reads == before);
    }
    // A child of the first stream's threads (203) born before the second stream's own threads
    // (204 to 206): seven on the target, and no way to tell that 203 is the child. Taking the
    // first six by tid would leave out the caller's own 206.
    reset();
    stage_born({200, 201, 202, 203, 204, 205, 206});
    SdkStreamThreadPlacement interleaved(1, 6, operations());
    check(interleaved.begin().success && reason(interleaved.end(), "family_too_large") && !interleaved.placed());
    mock.masks[206] = {0, kOwn};
    check(!interleaved.verify().success);
    // Tids that wrapped during the start (the caller's own 32761 to 32766, the child 300): the same.
    reset();
    stage_born({32761, 32762, 32763, 300, 32764, 32765, 32766});
    SdkStreamThreadPlacement wrapped(1, 6, operations());
    check(wrapped.begin().success && reason(wrapped.end(), "family_too_large") && !wrapped.placed());
    check(family_is(wrapped.family(), {300, 32761, 32762, 32763, 32764, 32765, 32766}));

    // A new task that is not on the target refuses the placement, also when six are: it may be
    // a stream thread that another thread created while a child filled the count. Its own set,
    // a superset of the target, a set that cannot be read.
    struct Stray { std::pair<int, std::uint64_t> mask; };
    for (const Stray& stray : {Stray{{0, kOwn}}, Stray{{0, kTarget | kOwn}}, Stray{{EIO, kTarget}}, Stray{{EINVAL, 0}}}) {
        reset();
        stage_born({200, 201, 202, 203, 204, 205, 206});
        mock.masks[205] = stray.mask;   // one of the caller's own; 206 is a child that fills the count
        SdkStreamThreadPlacement placement(1, 6, operations());
        check(placement.begin().success);
        const auto& ended = placement.end();
        check(!ended.success && reason(ended, "born_off_target") && ended.born_elsewhere == 1 && ended.family == 6 &&
              ended.born == 7 && !placement.placed() && mock.self_mask == kOwn);
        check(reason(placement.verify(), "not_placed"));
    }
    reset();
    stage_start(6, true);   // the six, and three strays
    SdkStreamThreadPlacement with_strays(1, 6, operations());
    check(with_strays.begin().success);
    const auto& strays = with_strays.end();
    check(reason(strays, "born_off_target") && strays.born_elsewhere == 3 && strays.born_ended == 0 && strays.born == 9 && strays.family == 6);

    // The listing need not be sorted, neither the one taken before the start: an old task is
    // not taken for a new one, and the family is held in order of tid.
    reset();
    stage_start(6);
    mock.listings[0] = {105, 100, 104, 101, 103, 102};
    mock.listings[1] = {205, 103, 201, 100, 204, 101, 200, 102, 203, 104, 202, 105};
    for (const auto tid : kBefore) mock.masks[tid] = {0, kTarget};   // were it "new", it would count
    SdkStreamThreadPlacement unsorted(1, 6, operations());
    check(unsorted.begin().success);
    const auto& mixed = unsorted.end();
    check(mixed.success && mixed.born == 6 && mixed.born_ended == 0 && family_is(mixed, {200, 201, 202, 203, 204, 205}));

    // A member's tid now belongs to another task (another start time): the member has ended,
    // whatever that task's set is and whether it can be read.
    const std::vector<std::pair<int, std::uint64_t>> successors = {{0, kTarget}, {0, kOwn}, {EINVAL, kTarget}};
    for (const auto& successor : successors) {
        reset();
        stage_born({200, 201, 202, 203, 204, 205});
        mock.starts[202] = {0, 77};   // a member with its own start time
        SdkStreamThreadPlacement reused(1, 6, operations());
        check(reused.begin().success && reused.end().success && reused.verify().success);
        mock.starts[202] = {0, kStart};   // the start time the other members have: still another task
        mock.masks[202] = successor;
        const auto& replaced = reused.verify();
        check(reason(replaced, "family_thread_gone") && replaced.gone == 1 && replaced.moved == 0 &&
              replaced.unreadable == 0 && replaced.first_bad_tid == 202);
        mock.starts[202] = {0, 77};       // even its old start time does not bring it back
        mock.masks[202] = {0, kTarget};
        check(reason(reused.verify(), "family_thread_gone") && reused.verify().checked == 5);
    }
    // The task ends between the two reads of one check.
    reset();
    stage_born({200, 201, 202, 203, 204, 205});
    SdkStreamThreadPlacement racing(1, 6, operations());
    check(racing.begin().success && racing.end().success);
    mock.starts[204] = {ESRCH, 0};
    check(reason(racing.verify(), "family_thread_gone") && racing.verify().first_bad_tid == 204);

    // The SDK restarts a stream by itself: three members end and three new threads appear,
    // which are not members whatever their set. The survey still counts six on the target.
    reset();
    stage_born({200, 201, 202, 203, 204, 205});
    SdkStreamThreadPlacement restarted(1, 6, operations());
    check(restarted.begin().success && restarted.end().success && restarted.verify().success);
    for (const std::int64_t tid : {203, 204, 205}) mock.masks.erase(tid);
    for (const std::int64_t tid : {500, 501, 502}) mock.masks[tid] = {0, kTarget};
    const auto& after_restart = restarted.verify();
    check(reason(after_restart, "family_thread_gone") && after_restart.gone == 3 && after_restart.checked == 6 && after_restart.first_bad_tid == 203);
    mock.listings.push_back({100, 101, 102, 103, 104, 105, 200, 201, 202, 500, 501, 502});
    check(restarted.survey().on_target == 6 && !restarted.verify().success && restarted.verify().checked == 3);

    // At the start: a new task on the target whose start time cannot be read is not a member
    // that could be verified, so the placement does not stand -- also when six others are there.
    reset();
    stage_born({200, 201, 202, 203, 204, 205, 206});
    mock.starts[206] = {EIO, 0};
    SdkStreamThreadPlacement unreadable(1, 6, operations());
    check(unreadable.begin().success);
    const auto& no_identity = unreadable.end();
    check(!no_identity.success && reason(no_identity, "family_read_failed") && no_identity.unreadable == 1 &&
          no_identity.family == 6 && !unreadable.placed() && mock.self_mask == kOwn);

    // A new task that has ended by the time it is read refuses the placement: it may have been
    // one of the caller's own threads. The caller's 205 ends right after the listing, and the
    // child 206 makes the count six again -- then whatever replaces 205 would never be looked at.
    reset();
    stage_born({200, 201, 202, 203, 204, 205, 206});
    mock.masks.erase(205);
    SdkStreamThreadPlacement substituted(1, 6, operations());
    check(substituted.begin().success);
    const auto& vanished = substituted.end();
    check(!vanished.success && reason(vanished, "born_ended") && vanished.born == 7 && vanished.born_ended == 1 &&
          vanished.family == 6 && !substituted.placed() && mock.self_mask == kOwn && !substituted.caller_restore_unconfirmed());
    check(reason(substituted.verify(), "not_placed"));
    // The same when it ends between the read of its set and the read of its start time, and when
    // six are left without it (it may have been a seventh, short-lived one: not known).
    reset();
    stage_born({200, 201, 202, 203, 204, 205, 206});
    mock.starts[206] = {ESRCH, 0};
    SdkStreamThreadPlacement ended_meanwhile(1, 6, operations());
    check(ended_meanwhile.begin().success);
    const auto& between_reads = ended_meanwhile.end();
    check(!between_reads.success && reason(between_reads, "born_ended") && between_reads.born_ended == 1 && between_reads.family == 6);
    reset();
    stage_born({200, 201, 202, 203, 204, 205});
    mock.listings[1].push_back(301);   // listed, never readable: it had ended
    SdkStreamThreadPlacement short_lived(1, 6, operations());
    check(short_lived.begin().success && reason(short_lived.end(), "born_ended") && short_lived.family().family == 6 && !short_lived.placed());
    // With too few left as well, the ended one is what is reported.
    reset();
    stage_born({200, 201, 202, 203, 204, 205});
    mock.masks.erase(203);
    SdkStreamThreadPlacement too_few(1, 6, operations());
    check(too_few.begin().success && reason(too_few.end(), "born_ended") && too_few.family().family == 5);
    // A stray thread is reported before an ended one.
    reset();
    stage_born({200, 201, 202, 203, 204, 205, 206});
    mock.masks[204] = {0, kOwn};
    mock.masks.erase(205);
    SdkStreamThreadPlacement stray_and_ended(1, 6, operations());
    check(stray_and_ended.begin().success && reason(stray_and_ended.end(), "born_off_target"));

    // The largest family; expected counts the placement cannot hold, and a CPU beyond the bit
    // set, are refused untouched.
    reset();
    stage_start(SdkThreadPlacementRecord::kMaxFamily);
    SdkStreamThreadPlacement full(1, SdkThreadPlacementRecord::kMaxFamily, operations());
    check(full.begin().success && full.end().success && full.family().family == 16 && full.family().family_tids[15] == 215);
    check(full.verify().success && full.verify().checked == 16);
    mock.masks[215] = {0, kOwn};
    check(full.verify().first_bad_tid == 215);
    for (const auto& bad : {std::pair<int, std::size_t>{1, 0}, std::pair<int, std::size_t>{1, 17}, std::pair<int, std::size_t>{64, 6}}) {
        reset();
        stage_start(6);
        SdkStreamThreadPlacement placement(bad.first, bad.second, operations());
        check(reason(placement.begin(), "invalid_configuration") && !placement.needs_end() && !placement.caller_restore_unconfirmed());
        check(mock.reads == 0 && mock.sets == 0 && mock.lists == 0);
    }
}

void begin_failures() {
    // Nothing is narrowed when the caller's set cannot be read, is the target already, or the
    // tasks cannot be listed; end() has then nothing to do.
    struct Case { const char* reason; void (*arrange)(); };
    const Case untouched[] = {
        {"initial_read_failed", [] { mock.read_valid = false; }},
        {"already_confined_to_target", [] { mock.self_mask = kTarget; }},
        {"list_tasks_failed", [] { mock.fail_list_at = 1; }},
    };
    for (const auto& c : untouched) {
        reset();
        stage_start(6);
        c.arrange();
        const std::uint64_t own = mock.self_mask;
        SdkStreamThreadPlacement placement(1, 6, operations());
        const auto& begun = placement.begin();
        check(!begun.success && reason(begun, c.reason) && !placement.needs_end() && !placement.caller_restore_unconfirmed());
        check(mock.sets == 0 && mock.self_mask == own);
        check(!placement.end().success && mock.sets == 0 && !placement.placed() && !placement.caller_restore_unconfirmed());
    }
    // The narrowing itself fails or does not read back exactly: end() still restores, and
    // reports begin. A set that reads back as the target plus more CPUs is not the target.
    const Case narrowed[] = {
        {"setaffinity_failed", [] { mock.fail_set_at = 1; }},
        {"readback_failed", [] { mock.invalid_read_at = 2; }},
        {"thread_identity_changed", [] { mock.foreign_tid_at = 2; }},
        {"readback_mask_mismatch", [] { mock.set_takes_effect = false; }},
        {"readback_mask_mismatch", [] { mock.set_adds_at = 1; mock.set_adds = 0x4; }},
    };
    for (const auto& c : narrowed) {
        reset();
        stage_start(6);
        c.arrange();
        SdkStreamThreadPlacement placement(1, 6, operations());
        const auto& begun = placement.begin();
        check(!begun.success && reason(begun, c.reason) && placement.needs_end() && placement.caller_restore_unconfirmed());
        mock.set_takes_effect = true;
        const auto& ended = placement.end();
        check(!ended.success && reason(ended, "begin_failed") && !placement.placed());
        check(mock.self_mask == kOwn && mock.requested.back() == kOwn && only_target_or_own_requested());
        check(!placement.caller_restore_unconfirmed());   // no family, but the caller has its own set back
    }
}

void end_failures() {
    // unconfirmed: whether it is NOT established that the caller has its own set back. The
    // capture start stops on it before it creates any thread.
    struct Case { const char* reason; void (*arrange)(); bool restored, unconfirmed; };
    const Case cases[] = {
        // reads: 1 begin, 2 begin readback, 3 end before restore, 4 end readback. sets: 1 narrow, 2 restore.
        {"restore_setaffinity_failed", [] { mock.fail_set_at = 2; }, false, true},
        {"restore_readback_failed", [] { mock.invalid_read_at = 4; }, true, true},
        {"restore_readback_failed", [] { mock.foreign_tid_at = 4; }, true, true},
        {"restore_readback_mask_mismatch", [] { mock.set_takes_effect = false; }, false, true},
        {"restore_readback_mask_mismatch", [] { mock.set_adds_at = 2; mock.set_adds = 0x1; }, false, true},
        {"list_tasks_failed", [] { mock.fail_list_at = 2; }, true, false},
    };
    for (const auto& c : cases) {
        reset();
        stage_start(6);
        SdkStreamThreadPlacement placement(1, 6, operations());
        check(placement.begin().success);
        c.arrange();
        const auto& ended = placement.end();
        check(!ended.success && reason(ended, c.reason) && !placement.placed() && !placement.needs_end());
        check(mock.sets == 2 && mock.requested.back() == kOwn);   // the restore was attempted
        check((mock.self_mask == kOwn) == c.restored);
        check(placement.caller_restore_unconfirmed() == c.unconfirmed);
        check(reason(placement.verify(), "not_placed"));
        check(placement.caller_restore_unconfirmed() == c.unconfirmed);   // nothing later changes it
        // The thread that was left narrowed is remembered for the rest of the process.
        check(sdk_thread_placement_left_narrowed().load() == (c.unconfirmed ? 100 : -1));
    }
    // The caller's set cannot be read just before the restore (same thread): it is restored all
    // the same, and with the family in order the placement stands.
    reset();
    stage_start(6);
    SdkStreamThreadPlacement unreadable(1, 6, operations());
    check(unreadable.begin().success);
    mock.invalid_read_at = 3;
    const auto& restored = unreadable.end();
    check(mock.sets == 2 && mock.requested.back() == kOwn && mock.self_mask == kOwn);
    check(restored.success && restored.restored_mask == kOwn && unreadable.placed() && !unreadable.caller_restore_unconfirmed());
    // end() on another thread than begin(): that thread's set is not touched, and the thread
    // that was narrowed is not known to be restored.
    reset();
    stage_start(6);
    SdkStreamThreadPlacement placement(1, 6, operations());
    check(placement.begin().success);
    mock.foreign_tid_at = 3;
    const auto& ended = placement.end();
    check(!ended.success && reason(ended, "restore_thread_identity_mismatch") && mock.sets == 1);
    check(placement.caller_restore_unconfirmed());
    // The narrowing never took effect and the restore call fails as well: a fault, no family
    // -- but the caller's set reads as its own, so nothing it creates later is misplaced.
    reset();
    stage_start(6);
    mock.set_takes_effect = false;
    SdkStreamThreadPlacement never_narrowed(1, 6, operations());
    check(!never_narrowed.begin().success && never_narrowed.caller_restore_unconfirmed());
    mock.fail_set_at = 2;
    const auto& refused = never_narrowed.end();
    check(!refused.success && reason(refused, "restore_setaffinity_failed") && refused.restore_errno == EPERM);
    check(refused.restored_mask == kOwn && !never_narrowed.caller_restore_unconfirmed() && !never_narrowed.placed());
    check(sdk_thread_placement_left_narrowed().load() == -1);
    // The restore failed although the family is complete: the family does not make it right.
    reset();
    stage_start(6);
    SdkStreamThreadPlacement narrowed_for_good(1, 6, operations());
    check(narrowed_for_good.begin().success);
    mock.fail_set_at = 2;
    const auto& stuck = narrowed_for_good.end();
    check(stuck.family == 6 && stuck.restore_errno == EPERM && stuck.restored_mask == kTarget && mock.self_mask == kTarget);
    check(!stuck.success && !narrowed_for_good.placed() && narrowed_for_good.caller_restore_unconfirmed());
    // A later start in the same process, on that thread: its set now reads as the target (or as
    // anything else), and that is not its own set. No placement begins, nothing is touched, and
    // the thread still counts as not restored -- whatever it reads as, valid or not.
    for (const int variant : {0, 1, 2}) {
        mock.reads = mock.sets = mock.lists = 0;
        mock.fail_set_at = 0;
        if (variant == 1) mock.self_mask = kOwn;       // somebody widened it meanwhile: still not known to be right
        if (variant == 2) mock.read_valid = false;
        SdkStreamThreadPlacement again(1, 6, operations());
        check(!again.caller_restore_unconfirmed());    // not known before begin() has looked
        const auto& refused_again = again.begin();
        check(!refused_again.success && reason(refused_again, "caller_left_narrowed") && refused_again.caller_tid == 100);
        check(again.caller_restore_unconfirmed() && !again.needs_end() && !again.placed());
        check(mock.sets == 0 && mock.lists == 0 && mock.reads == 1);
        (void)again.begin();
        check(!again.end().success && mock.sets == 0 && mock.reads == 1 && again.caller_restore_unconfirmed());
        check(reason(again.verify(), "not_placed"));
        mock.read_valid = true;
    }
    // Another thread of the process is not affected.
    mock.self_tid = 101;
    mock.self_mask = kOwn;
    mock.reads = mock.sets = mock.lists = 0;
    mock.listings = {kBefore, {100, 101, 102, 103, 104, 105, 200, 201, 202, 203, 204, 205}};
    SdkStreamThreadPlacement elsewhere(1, 6, operations());
    check(elsewhere.begin().success && elsewhere.end().success && !elsewhere.caller_restore_unconfirmed());
    check(sdk_thread_placement_left_narrowed().load() == 100);
}

// The guard the capture puts around the pipeline start.
void end_guard() {
    struct Fault {};
    reset();
    stage_start(6);
    int logged = 0;
    {
        SdkStreamThreadPlacement placement(1, 6, operations());
        try {
            SdkThreadPlacementEndGuard guard(&placement, [&](const SdkThreadPlacementRecord& record) {
                ++logged;
                check(std::string(record.event) == "end" && record.success && mock.self_mask == kOwn);
            });
            check(placement.begin().success && mock.self_mask == kTarget);
            throw Fault{};   // the pipeline start throws
        } catch (const Fault&) {
        }
        check(mock.self_mask == kOwn && !placement.needs_end() && placement.placed() && logged == 1);
        check(!placement.caller_restore_unconfirmed());
    }
    // A sink that throws does not escape the guard, and the caller is restored before it runs.
    reset();
    stage_start(6);
    {
        SdkStreamThreadPlacement placement(1, 6, operations());
        {
            SdkThreadPlacementEndGuard guard(&placement, [&](const SdkThreadPlacementRecord&) {
                check(mock.self_mask == kOwn);
                throw std::runtime_error("log failed");
            });
            check(placement.begin().success);
        }
        check(mock.self_mask == kOwn && placement.placed());
    }
    // Nothing to end: begin never ran, the placement is switched off, or there is none.
    reset();
    stage_start(6);
    logged = 0;
    {
        SdkStreamThreadPlacement idle(1, 6, operations()), off(-1, 6, operations());
        const auto count = [&](const SdkThreadPlacementRecord&) { ++logged; };
        { SdkThreadPlacementEndGuard guard(&idle, count); }
        { SdkThreadPlacementEndGuard guard(&off, count); (void)off.begin(); }
        { SdkThreadPlacementEndGuard guard(static_cast<SdkStreamThreadPlacement*>(nullptr), count); }
    }
    check(logged == 0 && mock.sets == 0 && mock.reads == 0);
    // What the capture start asks after the guard: with a restore the OS refused, also when the
    // pipeline start threw, the caller is not confirmed restored and no thread may be created.
    for (const bool start_throws : {false, true}) {
        reset();
        stage_start(6);
        SdkStreamThreadPlacement placement(1, 6, operations());
        std::string end_reason;
        try {
            SdkThreadPlacementEndGuard guard(&placement, [&](const SdkThreadPlacementRecord& record) { end_reason = record.reason; });
            check(placement.begin().success);
            mock.fail_set_at = 2;
            if (start_throws) throw Fault{};
        } catch (const Fault&) {
        }
        check(end_reason == "restore_setaffinity_failed" && placement.caller_restore_unconfirmed() && mock.self_mask == kTarget);
        // The start is tried again in the same process: no bracket, and still no thread may be created.
        SdkStreamThreadPlacement retry(1, 6, operations());
        const int sets = mock.sets;
        {
            SdkThreadPlacementEndGuard guard(&retry, [&](const SdkThreadPlacementRecord&) { end_reason = "ended"; });
            check(reason(retry.begin(), "caller_left_narrowed"));
        }
        check(retry.caller_restore_unconfirmed() && mock.sets == sets && end_reason == "restore_setaffinity_failed");
    }
    reset();
    errno = EDOM;
    check(!emit_sdk_thread_placement(SdkThreadPlacementRecord{}, [](const SdkThreadPlacementRecord&) { errno = EIO; throw 1; }));
    check(errno == EDOM);
}

// The periodic check of the capture loop.
void watch() {
    const auto state = [] { return static_cast<SdkThreadPlacementState>(sdk_thread_placement_state().load()); };
    const std::int64_t second = SdkThreadPlacementWatch::kVerifyPeriodNs;
    check(SdkThreadPlacementWatch::kSurveyAfterNs == 5 * second);
    std::vector<std::string> log;
    const auto sink = [&](const SdkThreadPlacementRecord& record) {
        log.push_back(std::string(record.event) + ":" + record.reason);
    };
    // No placement or switched off: nothing is read, logged or published.
    reset();
    publish_sdk_thread_placement(SdkThreadPlacementState::Disabled);
    SdkStreamThreadPlacement off(-1, 6, operations());
    SdkThreadPlacementWatch none(nullptr), idle(&off);
    none.tick(0, sink);
    idle.tick(0, sink);
    idle.tick(100 * second, sink);
    check(log.empty() && mock.mask_reads == 0 && mock.lists == 0 && state() == SdkThreadPlacementState::Disabled);

    stage_born({200, 201, 202, 203, 204, 205});
    SdkStreamThreadPlacement placement(1, 6, operations());
    check(placement.begin().success && placement.end().success);
    SdkThreadPlacementWatch watching(&placement);
    const std::int64_t t0 = 5000 * second;   // any origin
    int reads = mock.mask_reads;
    watching.tick(t0, sink);                 // the first tick checks and reports
    check(log == std::vector<std::string>{"verify:ok"} && mock.mask_reads == reads + 6 && state() == SdkThreadPlacementState::Placed);
    watching.tick(t0 + second - 1, sink);    // within the period: nothing
    check(mock.mask_reads == reads + 6 && log.size() == 1);
    watching.tick(t0 + second, sink);        // a period later: checked, same answer, not logged again
    check(mock.mask_reads == reads + 12 && log.size() == 1);
    mock.masks[202] = {0, kOwn};             // a member with another set
    watching.tick(t0 + 2 * second, sink);
    check(log.size() == 2 && log.back() == "verify:family_mask_changed" && state() == SdkThreadPlacementState::Fault);
    watching.tick(t0 + 3 * second, sink);    // still failing, same reason: not logged again
    check(log.size() == 2 && state() == SdkThreadPlacementState::Fault);
    mock.masks[202] = {EINVAL, kTarget};     // still failing, another reason: logged
    watching.tick(t0 + 4 * second, sink);
    check(log.size() == 3 && log.back() == "verify:family_read_failed" && state() == SdkThreadPlacementState::Fault);
    mock.masks[202] = {0, kTarget};          // in order again; and the survey is due
    const int lists = mock.lists;
    watching.tick(t0 + 5 * second, sink);
    check(log.size() == 5 && log[3] == "verify:ok" && log[4] == "survey:ok" && state() == SdkThreadPlacementState::Placed);
    check(mock.lists == lists + 1);
    watching.tick(t0 + 6 * second, sink);    // the survey runs once
    watching.tick(t0 + 200 * second, sink);  // however long it runs, an unchanged answer is not logged
    check(log.size() == 5 && mock.lists == lists + 1 && state() == SdkThreadPlacementState::Placed);
    mock.masks.erase(201);                   // a member ends
    watching.tick(t0 + 201 * second, sink);
    check(log.size() == 6 && log.back() == "verify:family_thread_gone" && state() == SdkThreadPlacementState::Fault);
    mock.masks[201] = {0, kTarget};          // a task with that tid again: the member is still gone, nothing new to log
    watching.tick(t0 + 202 * second, sink);
    check(log.size() == 6 && state() == SdkThreadPlacementState::Fault);
    mock.masks.erase(204);                   // a second member ends: same reason, but it is news
    watching.tick(t0 + 203 * second, sink);
    check(log.size() == 7 && log.back() == "verify:family_thread_gone");
    watching.tick(t0 + 204 * second, sink);
    check(log.size() == 7 && state() == SdkThreadPlacementState::Fault);
    // A survey that cannot list the tasks is handed to the sink as a failure.
    reset();
    stage_start(6);
    SdkStreamThreadPlacement second_placement(1, 6, operations());
    check(second_placement.begin().success && second_placement.end().success);
    SdkThreadPlacementWatch failing_survey(&second_placement);
    log.clear();
    failing_survey.tick(0, sink);
    mock.fail_list_at = mock.lists + 1;
    failing_survey.tick(SdkThreadPlacementWatch::kSurveyAfterNs, sink);
    check(log == (std::vector<std::string>{"verify:ok", "survey:list_tasks_failed"}));
    // A placement that failed at the start: Fault from the first tick on, reported once.
    reset();
    stage_start(4);
    SdkStreamThreadPlacement failed(1, 6, operations());
    check(failed.begin().success && !failed.end().success);
    SdkThreadPlacementWatch after_failure(&failed);
    log.clear();
    after_failure.tick(0, sink);
    after_failure.tick(second, sink);
    after_failure.tick(4 * second, sink);
    check(log == std::vector<std::string>{"verify:not_placed"} && state() == SdkThreadPlacementState::Fault);
    // A sink that throws does not stop the watch or the published state.
    SdkThreadPlacementWatch throwing(&failed);
    throwing.tick(0, [](const SdkThreadPlacementRecord&) { throw std::runtime_error("log failed"); });
    check(state() == SdkThreadPlacementState::Fault);
    publish_sdk_thread_placement(SdkThreadPlacementState::Disabled);
}

#ifdef __linux__
// Real system calls on threads of this test. The target is a CPU of the runner's own set among
// 2, 3 and 7 that is not the whole set; without one the case is skipped.
void actual_threads() {
    const auto own = read_capture_affinity();
    check(own.valid);
    int target = -1;
    for (const int cpu : {2, 3, 7})
        if ((own.mask & (std::uint64_t{1} << cpu)) && own.mask != (std::uint64_t{1} << cpu)) { target = cpu; break; }
    if (target < 0) {
        std::cout << "SDK_THREAD_PLACEMENT actual case skipped: own set " << own.mask << " has no usable CPU among 2, 3, 7\n";
        return;
    }
    const std::uint64_t bit = std::uint64_t{1} << target;
    // The start time of a task: this thread's own, twice the same; none for a task of another
    // process (the parent of this test), whatever exists under that number. A thread created
    // later has its OWN, later start time, not the process's, and reads the same from inside
    // (a thread other than the first one reading under /proc/self/task) as from here.
    std::uint64_t mine = 0, again = 1, foreign = 1;
    check(read_task_start(own.tid, mine) == 0 && mine > 0 && read_task_start(own.tid, again) == 0 && again == mine);
    check(read_task_start(static_cast<std::int64_t>(::getppid()), foreign) == ESRCH && foreign == 0);
    check(read_task_start(0, foreign) == ESRCH && read_task_start(-1, foreign) == ESRCH);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));   // several start ticks (10 ms) later
    {
        std::mutex gate;
        std::condition_variable turn;
        bool ready = false, leave = false;
        std::int64_t late_tid = -1;
        std::uint64_t seen_inside = 0;
        int code_inside = -1;
        std::thread late([&] {
            const auto self = read_capture_affinity();
            std::uint64_t ticks = 0;
            const int code = read_task_start(self.tid, ticks);
            std::unique_lock<std::mutex> lock(gate);
            late_tid = self.tid;
            seen_inside = ticks;
            code_inside = code;
            ready = true;
            turn.notify_all();
            turn.wait(lock, [&] { return leave; });
        });
        std::uint64_t seen_outside = 0;
        {
            std::unique_lock<std::mutex> lock(gate);
            turn.wait(lock, [&] { return ready; });
            check(code_inside == 0 && late_tid > 0 && late_tid != own.tid);
            check(read_task_start(late_tid, seen_outside) == 0 && seen_outside == seen_inside && seen_inside > mine);
            leave = true;
        }
        turn.notify_all();
        late.join();
    }

    std::mutex mutex;
    std::condition_variable wake;
    // A child born during the bracket: one more than expected, and nothing says which is the
    // child. The placement does not stand; the caller has its own set back all the same.
    {
        bool child_running = false, done = false;
        std::thread child;
        SdkStreamThreadPlacement ambiguous(target, 1);
        check(ambiguous.begin().success);
        std::thread parent([&] {
            child = std::thread([&] {
                std::unique_lock<std::mutex> lock(mutex);
                child_running = true;
                wake.notify_all();
                wake.wait(lock, [&] { return done; });
            });
            std::unique_lock<std::mutex> lock(mutex);
            wake.wait(lock, [&] { return done; });
        });
        { std::unique_lock<std::mutex> lock(mutex); wake.wait(lock, [&] { return child_running; }); }
        const auto& ended = ambiguous.end();
        check(!ended.success && reason(ended, "family_too_large") && ended.family == 2 && ended.born == 2 && ended.born_elsewhere == 0);
        check(read_capture_affinity().mask == own.mask && !ambiguous.caller_restore_unconfirmed() && !ambiguous.placed());
        { std::lock_guard<std::mutex> lock(mutex); done = true; }
        wake.notify_all();
        parent.join();
        child.join();
    }
    // A thread that ANOTHER thread creates during the bracket is not on the target. The count on
    // the target is right, and still the placement does not stand.
    {
        int stage = 0;   // 1: the helper creates its thread; 2: that thread runs; 3: all end
        std::thread stray;
        std::thread helper([&] {   // exists before the bracket, with the process's own set
            std::unique_lock<std::mutex> lock(mutex);
            wake.wait(lock, [&] { return stage >= 1; });
            stray = std::thread([&] {
                std::unique_lock<std::mutex> inner(mutex);
                stage = 2;
                wake.notify_all();
                wake.wait(inner, [&] { return stage >= 3; });
            });
            wake.wait(lock, [&] { return stage >= 3; });
        });
        std::this_thread::sleep_for(std::chrono::milliseconds(20));   // the helper is listed before the bracket
        SdkStreamThreadPlacement strayed(target, 1);
        check(strayed.begin().success);
        std::thread own_thread([&] { std::unique_lock<std::mutex> lock(mutex); wake.wait(lock, [&] { return stage >= 3; }); });
        { std::unique_lock<std::mutex> lock(mutex); stage = 1; wake.notify_all(); wake.wait(lock, [&] { return stage >= 2; }); }
        const auto& ended = strayed.end();
        check(!ended.success && reason(ended, "born_off_target") && ended.born == 2 && ended.born_elsewhere == 1 && ended.family == 1);
        check(read_capture_affinity().mask == own.mask && !strayed.placed());
        { std::lock_guard<std::mutex> lock(mutex); stage = 3; }
        wake.notify_all();
        own_thread.join();
        helper.join();
        stray.join();
    }

    int release = 0;   // 1: the third thread creates a child; 2: the first thread ends; 3: the second widens itself; 4: all end
    std::atomic<bool> child_running{false};
    std::atomic<int> widen_result{-1};
    std::atomic<std::uint64_t> child_mask{0};
    std::vector<std::thread> threads;
    std::thread child;   // created by a family thread after the bracket: on the target, not a member
    SdkStreamThreadPlacement placement(target, 3);
    const auto& begun = placement.begin();
    check(begun.success && begun.narrowed_mask == bit && placement.caller_restore_unconfirmed());
    for (int i = 0; i < 3; ++i) {
        threads.emplace_back([&, i] {
            std::unique_lock<std::mutex> lock(mutex);
            if (i == 0) { wake.wait(lock, [&] { return release >= 2; }); return; }
            if (i == 1) {
                wake.wait(lock, [&] { return release >= 3; });
                widen_result.store(set_capture_affinity(own.mask));
            }
            if (i == 2) {
                wake.wait(lock, [&] { return release >= 1; });
                child = std::thread([&] {
                    child_mask.store(read_capture_affinity().mask);
                    child_running.store(true);
                    std::unique_lock<std::mutex> inner(mutex);
                    wake.wait(inner, [&] { return release >= 4; });
                });
            }
            wake.wait(lock, [&] { return release >= 4; });
        });
    }
    const auto& ended = placement.end();
    const auto after = read_capture_affinity();
    check(after.valid && after.mask == own.mask && !placement.caller_restore_unconfirmed());   // the caller has its own set back
    check(ended.success && placement.placed() && ended.born == 3 && ended.born_elsewhere == 0 && ended.family == 3 && ended.unreadable == 0);
    check(sdk_thread_placement_left_narrowed().load() == -1);
    const auto& fine = placement.verify();
    check(fine.success && fine.checked == 3);
    // The same check from a thread other than the one that placed them, as the capture loop does.
    bool elsewhere_ok = false;
    std::thread checker([&] { const auto& record = placement.verify(); elsewhere_ok = record.success && record.checked == 3; });
    checker.join();
    check(elsewhere_ok);
    // A thread created now is not on the target: the caller is no longer narrowed.
    std::uint64_t later_mask = 0;
    std::thread later([&] { later_mask = read_capture_affinity().mask; });
    later.join();
    check(later_mask == own.mask);
    // A thread of another process is "not ours", whatever its set: the parent of this test.
    std::uint64_t foreign_mask = 0;
    check(read_task_affinity(static_cast<std::int64_t>(::getppid()), foreign_mask) == ESRCH);
    // A family thread creates a child: on the target by inheritance, counted by the survey, not a member.
    { std::lock_guard<std::mutex> lock(mutex); release = 1; }
    wake.notify_all();
    while (!child_running.load()) std::this_thread::yield();
    check(child_mask.load() == bit && placement.verify().success && placement.verify().checked == 3);
    const auto& counted = placement.survey();
    check(counted.success && counted.on_target == 4);
    // A member ends: a fault for good.
    { std::lock_guard<std::mutex> lock(mutex); release = 2; }
    wake.notify_all();
    threads[0].join();
    // join() can return a moment before the kernel has dropped the task: wait for it.
    for (int spin = 0; spin < 5000 && placement.verify().gone == 0; ++spin)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    const auto& gone = placement.verify();
    check(!gone.success && reason(gone, "family_thread_gone") && gone.gone == 1 && gone.moved == 0);
    { std::lock_guard<std::mutex> lock(mutex); release = 3; }
    wake.notify_all();
    for (int spin = 0; spin < 5000 && placement.verify().moved == 0; ++spin)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    const auto& moved = placement.verify();
    check(widen_result.load() == 0 && reason(moved, "family_thread_gone") && moved.gone == 1 && moved.moved == 1 && moved.checked == 2);
    { std::lock_guard<std::mutex> lock(mutex); release = 4; }
    wake.notify_all();
    threads[1].join();
    threads[2].join();
    child.join();
    std::cout << "SDK_THREAD_PLACEMENT actual case target_cpu=" << target << " own_mask=" << own.mask << " ok\n";
}
#else
void actual_threads() { std::cout << "SDK_THREAD_PLACEMENT actual case skipped: not Linux\n"; }
#endif
}  // namespace

int main() {
    try {
        parsing();
        task_start_parsing();
        disabled_is_a_no_op();
        placed_family();
        log_line();
        family_identity();
        begin_failures();
        end_failures();
        end_guard();
        watch();
        check(sdk_thread_placement_state().load() == static_cast<int>(SdkThreadPlacementState::Disabled));
        check(sdk_thread_placement_left_narrowed().load() == -1);
        actual_threads();
        std::cout << "SDK_STREAM_THREAD_PLACEMENT_PASS checks=" << checks << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "SDK_STREAM_THREAD_PLACEMENT_FAIL " << error.what() << '\n';
        return 1;
    }
}
