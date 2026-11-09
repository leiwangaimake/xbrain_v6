/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_profile_thread_placement.cpp
 * Brief: Unit tests for placing the Profile thread alone on one core and the other threads off it.
 *
 * Description:
 * What it checks. The plan from the inherited set and the three cores: the platform set
 * {1,2,3,7} and the set {2,3,7} (SDK core outside) with SDK core 1 and Profile core 3 both give
 * the Profile thread {3} and the other threads {2,7}; every refusal rule. The role of one task
 * (capture thread, Profile thread, SDK family, other). With mocked system calls: nothing is
 * listed before both tids are known or with a refused plan; the apply step places every task
 * (an unsorted listing), asks for no set but the two of the plan and leaves the capture thread
 * and the SDK family alone; a recheck only once a second, only for tasks not seen before (a seen
 * task whose set changed is not touched again), logged only when it moved or failed something;
 * tasks that end before their read or set, failed sets and read-backs (not retried); a listing
 * that fails (reported once, then recovered); a Profile thread that is not listed; errno left
 * alone; every field of the two log lines. A final Linux-only case runs the real system calls on
 * threads of this test, with threads born later from the Profile, capture and main threads, on
 * four cores of the runner's set (widened to cores 1, 2, 3 and 7 where its cgroup allows them;
 * skipped with fewer than four).
 * Trap. A thread is seen once: a later change of its set is not undone, and a failed one is not
 * retried. The capture thread is told by its tid, the SDK family by its set.
 */

#include "common/profile_thread_placement.hpp"

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstring>
#include <iostream>
#include <map>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {
using namespace perception;
int checks = 0;
void check(bool value) {
    ++checks;
    if (!value) throw std::runtime_error("profile thread placement check " + std::to_string(checks));
}
bool reason(const ProfileThreadPlan& plan, const char* expected) { return std::string(plan.reason) == expected; }
bool reason(const ProfileThreadPlacementRecord& record, const char* expected) {
    return std::string(record.reason) == expected;
}
template<class T> std::string text(const T& value) {
    std::ostringstream out;
    out << value;
    return out.str();
}

constexpr std::uint64_t kPlatform = 0x8E;   // cores 1, 2, 3, 7
constexpr std::uint64_t kLent = 0x8C;       // cores 2, 3, 7
constexpr std::uint64_t kProfile = 0x08;    // core 3
constexpr std::uint64_t kOthers = 0x84;     // cores 2, 7
constexpr std::uint64_t kSdk = 0x02;        // core 1
constexpr std::uint64_t kCapture = 0x04;    // core 2

void plans() {
    for (const std::uint64_t inherited : {kPlatform, kLent}) {
        const auto plan = plan_profile_thread_placement(inherited, 3, 1, 2);
        check(plan.ok && reason(plan, "ok") && plan.profile_mask == kProfile && plan.others_mask == kOthers);
        check(plan.profile_cpu == 3 && plan.sdk_cpu == 1 && plan.capture_cpu == 2 && plan.inherited_mask == inherited);
    }
    // Other cores of the same rules: Profile 1 with the SDK on 7, Profile 7 with the SDK on 1.
    const auto p1 = plan_profile_thread_placement(kPlatform, 1, 7, 2);
    check(p1.ok && p1.profile_mask == 0x02 && p1.others_mask == 0x0C);
    const auto p7 = plan_profile_thread_placement(kPlatform, 7, 1, 2);
    check(p7.ok && p7.profile_mask == 0x80 && p7.others_mask == 0x0C);
    struct Refused { std::uint64_t inherited; int profile, sdk, capture; const char* why; };
    const Refused refused[] = {
        {kPlatform, -1, 1, 2, "profile_cpu_invalid"}, {kPlatform, 64, 1, 2, "profile_cpu_invalid"},
        {kPlatform, 3, -1, 2, "sdk_cpu_invalid"}, {kPlatform, 3, 1, 64, "capture_cpu_invalid"},
        {0x86, 3, 1, 2, "profile_cpu_not_inherited"},          // {1,2,7}
        {0x0F, 3, 3, 2, "profile_cpu_is_sdk_cpu"},
        {kPlatform, 2, 1, 2, "profile_cpu_is_capture_cpu"},
        {0x8A, 3, 1, 2, "capture_cpu_not_left"},               // {1,3,7}
        {kPlatform, 3, 2, 2, "capture_cpu_not_left"},          // the SDK on the capture core
        {0x0C, 3, 1, 2, "others_too_few"},                     // {2,3}: only the capture core left
        {0x0E, 3, 1, 2, "others_too_few"},                     // {1,2,3}
    };
    for (const auto& r : refused) {
        const auto plan = plan_profile_thread_placement(r.inherited, r.profile, r.sdk, r.capture);
        check(!plan.ok && reason(plan, r.why) && plan.profile_mask == 0 && plan.others_mask == 0);
    }
    check(text(plan_profile_thread_placement(kPlatform, 3, 1, 2)) ==
          "PROFILE_THREAD_PLACEMENT schema=1 mask_encoding=cpu_bitset_u64 event=plan profile_cpu=3 sdk_cpu=1 "
          "capture_cpu=2 inherited_mask=142 profile_mask=8 others_mask=132 success=1 reason=ok");
}

void roles() {
    const auto plan = plan_profile_thread_placement(kPlatform, 3, 1, 2);
    const auto capture = profile_thread_target(plan, 101, kCapture, 102, 101);
    check(capture.role == ProfileThreadRole::Capture && capture.mask == 0);
    const auto profile = profile_thread_target(plan, 102, kPlatform, 102, 101);
    check(profile.role == ProfileThreadRole::Profile && profile.mask == kProfile);
    // By tid first: the Profile thread on the SDK core is still the Profile thread.
    check(profile_thread_target(plan, 102, kSdk, 102, 101).role == ProfileThreadRole::Profile);
    const auto family = profile_thread_target(plan, 103, kSdk, 102, 101);
    check(family.role == ProfileThreadRole::SdkFamily && family.mask == 0);
    // The SDK core together with another core is not the family.
    for (const std::uint64_t current : {kPlatform, kProfile, kCapture, std::uint64_t{0x03}}) {
        const auto other = profile_thread_target(plan, 109, current, 102, 101);
        check(other.role == ProfileThreadRole::Other && other.mask == kOthers);
    }
}

struct Mock {
    std::map<std::int64_t, std::uint64_t> masks;   // the tasks of the mocked process
    std::map<std::int64_t, int> read_errno, set_errno;
    std::set<std::int64_t> wrong_readback;          // the set is taken, another one reads back
    bool list_fails = false;
    int lists = 0;
    std::vector<std::pair<std::int64_t, std::uint64_t>> sets;
} mock;

bool list_mock(std::vector<std::int64_t>& out) noexcept {
    ++mock.lists;
    errno = ENOENT;   // the placement must not leak a system call's errno
    out.clear();
    if (mock.list_fails) return false;
    for (const auto& task : mock.masks) out.push_back(task.first);
    std::reverse(out.begin(), out.end());   // the placement sorts itself
    return !out.empty();
}
int read_mock(std::int64_t tid, std::uint64_t& mask) noexcept {
    errno = EIO;
    mask = 0;
    const auto failing = mock.read_errno.find(tid);
    if (failing != mock.read_errno.end()) return failing->second;
    const auto it = mock.masks.find(tid);
    if (it == mock.masks.end()) return ESRCH;
    mask = it->second;
    if (mock.wrong_readback.count(tid) && !mock.sets.empty() && mock.sets.back().first == tid) mask |= 0x01;
    return 0;
}
int set_mock(std::int64_t tid, std::uint64_t mask) noexcept {
    errno = EPERM;
    mock.sets.emplace_back(tid, mask);
    const auto failing = mock.set_errno.find(tid);
    if (failing != mock.set_errno.end()) return failing->second;
    const auto it = mock.masks.find(tid);
    if (it == mock.masks.end()) return ESRCH;
    it->second = mask;
    return 0;
}
ProfileThreadPlacementOps operations() { return {list_mock, read_mock, set_mock}; }

// The process at the capture pin: main 100, capture thread 101 (pinned), Profile thread 102,
// six SDK threads 103-108 on core 1, two other threads 109 (inherited set) and 110 (already right),
// and 111, another thread on the Profile core.
void stage_process() {
    mock = Mock{};
    mock.masks = {{100, kPlatform}, {101, kCapture}, {102, kPlatform}, {109, kPlatform}, {110, kOthers}, {111, kProfile}};
    for (std::int64_t tid = 103; tid <= 108; ++tid) mock.masks[tid] = kSdk;
}

void waits_for_both_tids() {
    stage_process();
    ProfileThreadPlacement placement(plan_profile_thread_placement(kPlatform, 3, 1, 2), operations());
    check(placement.poll(-1, -1, 0) == nullptr && placement.poll(101, -1, 0) == nullptr &&
          placement.poll(-1, 102, 0) == nullptr && placement.poll(0, 102, 0) == nullptr);
    check(mock.lists == 0 && mock.sets.empty() && !placement.applied());
    // A refused plan never lists anything.
    ProfileThreadPlacement refused(plan_profile_thread_placement(0x0C, 3, 1, 2), operations());
    check(refused.poll(101, 102, 0) == nullptr && refused.poll(101, 102, 5000) == nullptr && mock.lists == 0);
}

void apply_and_recheck() {
    stage_process();
    ProfileThreadPlacement placement(plan_profile_thread_placement(kPlatform, 3, 1, 2), operations());
    errno = 1234;
    const auto* applied = placement.poll(101, 102, 10000);
    check(errno == 1234 && applied && placement.applied() && mock.lists == 1);
    check(applied->success && reason(*applied, "ok") && applied->listed && std::string(applied->event) == "apply");
    check(applied->tasks == 12 && applied->fresh == 12 && applied->moved_profile == 1 && applied->moved_other == 3 &&
          applied->unchanged == 1 && applied->capture == 1 && applied->sdk == 6 && applied->ended == 0 && applied->failed == 0);
    check(mock.masks[100] == kOthers && mock.masks[101] == kCapture && mock.masks[102] == kProfile &&
          mock.masks[109] == kOthers && mock.masks[110] == kOthers && mock.masks[111] == kOthers);
    for (std::int64_t tid = 103; tid <= 108; ++tid) check(mock.masks[tid] == kSdk);
    // Only the two sets of the plan are ever asked for, never for the capture thread or the family.
    for (const auto& set : mock.sets) {
        check(set.second == kProfile || set.second == kOthers);
        check(set.first != 101 && (set.first < 103 || set.first > 108));
    }
    check(mock.sets.size() == 4);
    check(text(*applied) ==
          "PROFILE_THREAD_PLACEMENT schema=1 mask_encoding=cpu_bitset_u64 event=apply round=1 profile_cpu=3 sdk_cpu=1 "
          "capture_cpu=2 inherited_mask=142 profile_mask=8 others_mask=132 capture_tid=101 profile_tid=102 tasks=12 "
          "new=12 moved_profile=1 moved_other=3 unchanged=1 capture=1 sdk=6 ended=0 failed=0 first_fail_tid=-1 "
          "first_fail_errno=0 first_fail=none listed=1 success=1 reason=ok");
    // Not yet due; then due with nothing new: listed, nothing changed, nothing to log.
    check(placement.poll(101, 102, 10999) == nullptr && mock.lists == 1);
    check(placement.poll(101, 102, 11000) == nullptr && mock.lists == 2 && mock.sets.size() == 4);
    // Born since: from the Profile thread (120, its core) and from the capture thread (121, the
    // capture core): moved; from an SDK thread (122) and from the main thread (123): already
    // right. 124 ends before its read, 127 before its set; 125's set fails, 126 reads back
    // another set. 110 has ended; 109's set was changed by someone else (not touched again).
    mock.masks.erase(110);
    mock.masks[109] = kPlatform;
    mock.masks[120] = kProfile;
    mock.masks[121] = kCapture;
    mock.masks[122] = kSdk;
    mock.masks[123] = kOthers;
    mock.masks[124] = kPlatform;
    mock.read_errno[124] = ESRCH;
    mock.masks[125] = kPlatform;
    mock.set_errno[125] = EPERM;
    mock.masks[126] = kPlatform;
    mock.wrong_readback.insert(126);
    mock.masks[127] = kPlatform;
    mock.set_errno[127] = ESRCH;
    const std::size_t sets_before = mock.sets.size();
    const auto* rechecked = placement.poll(101, 102, 12000);
    check(rechecked && !rechecked->success && reason(*rechecked, "set_failed") && std::string(rechecked->event) == "recheck");
    check(rechecked->round == 3 && rechecked->tasks == 19 && rechecked->fresh == 8 && rechecked->moved_profile == 0 &&
          rechecked->moved_other == 2 && rechecked->unchanged == 1 && rechecked->sdk == 1 && rechecked->capture == 0 &&
          rechecked->ended == 2 && rechecked->failed == 2);
    check(rechecked->first_fail_tid == 125 && rechecked->first_fail_errno == EPERM && std::string(rechecked->first_fail) == "set_failed");
    check(mock.masks[120] == kOthers && mock.masks[121] == kOthers && mock.masks[122] == kSdk && mock.masks[109] == kPlatform);
    check(mock.sets.size() == sets_before + 5);   // 120, 121, 125, 126, 127
    // 125 and 126 were seen: never retried. 124 and 127 ended: looked at again, still ending.
    // Nothing moved or failed: nothing to log.
    check(placement.poll(101, 102, 12500) == nullptr && placement.poll(101, 102, 13000) == nullptr);
    check(mock.sets.size() == sets_before + 6);   // 127 again
    // A listing that fails is reported once.
    mock.list_fails = true;
    const auto* unlisted = placement.poll(101, 102, 14000);
    check(unlisted && !unlisted->listed && !unlisted->success && reason(*unlisted, "list_tasks_failed") && unlisted->tasks == 0);
    check(placement.poll(101, 102, 15000) == nullptr);
    // Listed again: what is new is placed.
    mock.list_fails = false;
    mock.masks[130] = kPlatform;
    const auto* recovered = placement.poll(101, 102, 16000);
    check(recovered && recovered->success && recovered->listed && recovered->moved_other == 1 && recovered->fresh == 3);
    check(mock.masks[130] == kOthers);
}

void apply_failures() {
    // The listing fails at the apply step: logged; the next recheck that lists places everything.
    stage_process();
    mock.list_fails = true;
    ProfileThreadPlacement placement(plan_profile_thread_placement(kPlatform, 3, 1, 2), operations());
    const auto* applied = placement.poll(101, 102, 0);
    check(applied && !applied->success && !applied->listed && reason(*applied, "list_tasks_failed") && mock.sets.empty());
    check(placement.poll(101, 102, 1000) == nullptr);   // still failing: already reported
    mock.list_fails = false;
    const auto* placed = placement.poll(101, 102, 2000);
    check(placed && placed->success && placed->fresh == 12 && placed->moved_profile == 1 && placed->moved_other == 3);
    // The Profile thread is not listed (it has ended): the others are placed all the same.
    stage_process();
    mock.masks.erase(102);
    ProfileThreadPlacement lost(plan_profile_thread_placement(kPlatform, 3, 1, 2), operations());
    const auto* without = lost.poll(101, 102, 0);
    check(without && !without->success && reason(*without, "profile_thread_not_listed") && without->moved_other == 3 &&
          without->moved_profile == 0);
    // A read that fails: counted, the task is not touched and not retried.
    stage_process();
    mock.read_errno[109] = EACCES;
    ProfileThreadPlacement unreadable(plan_profile_thread_placement(kPlatform, 3, 1, 2), operations());
    const auto* read_failed = unreadable.poll(101, 102, 0);
    check(read_failed && !read_failed->success && reason(*read_failed, "read_failed") && read_failed->first_fail_tid == 109 &&
          read_failed->first_fail_errno == EACCES && read_failed->moved_other == 2 && mock.masks[109] == kPlatform);
    mock.read_errno.clear();
    check(unreadable.poll(101, 102, 1000) == nullptr && mock.masks[109] == kPlatform);
}

#ifdef __linux__
// Real system calls on threads of this test. The test widens itself to cores 1, 2, 3 and 7 where
// its cgroup allows them (the kernel keeps only the allowed ones) and takes four cores of what it
// gets: the SDK core, the capture core, the Profile core and one more; with fewer it is skipped.
void actual_threads() {
    const auto own = read_capture_affinity();
    check(own.valid && own.mask != 0);
    check(set_capture_affinity(own.mask | kPlatform) == 0);
    const auto widened = read_capture_affinity();
    check(widened.valid && (widened.mask & own.mask) == own.mask);
    // The production cores when all four are there, else the four lowest of the set.
    std::vector<int> cores;
    if ((widened.mask & kPlatform) == kPlatform) {
        cores = {1, 2, 3, 7};
    } else {
        for (int cpu = 0; cpu < 64 && cores.size() < 4; ++cpu)
            if (widened.mask & (std::uint64_t{1} << cpu)) cores.push_back(cpu);
    }
    if (cores.size() < 4) {
        check(set_capture_affinity(own.mask) == 0);
        std::cout << "PROFILE_THREAD_PLACEMENT actual case skipped: set " << widened.mask << " has fewer than four cores\n";
        return;
    }
    const int sdk_cpu = cores[0], capture_cpu = cores[1], profile_cpu = cores[2];
    const auto plan = plan_profile_thread_placement(widened.mask, profile_cpu, sdk_cpu, capture_cpu);
    check(plan.ok);
    const std::uint64_t others = plan.others_mask;
    std::mutex mutex;
    std::condition_variable wake;
    int stage = 0;   // 1: the Profile and capture threads each create a child; 2: all end
    std::atomic<std::int64_t> capture_tid{-1}, profile_tid{-1}, sdk_tid{-1}, other_tid{-1};
    std::atomic<std::int64_t> profile_child{-1}, capture_child{-1};
    std::atomic<int> ready{0};
    std::thread profile_child_thread, capture_child_thread;
    const auto wait_for = [&](int value) {
        std::unique_lock<std::mutex> lock(mutex);
        wake.wait(lock, [&] { return stage >= value; });
    };
    const auto child = [&](std::atomic<std::int64_t>& tid) {
        return std::thread([&] { tid.store(read_capture_affinity().tid); wait_for(2); });
    };
    std::thread capture([&] {
        if (set_capture_affinity(std::uint64_t{1} << capture_cpu) != 0) return;
        capture_tid.store(read_capture_affinity().tid);
        ++ready;
        wait_for(1);
        capture_child_thread = child(capture_child);
        wait_for(2);
    });
    std::thread profile([&] {
        profile_tid.store(read_capture_affinity().tid);
        ++ready;
        wait_for(1);
        profile_child_thread = child(profile_child);
        wait_for(2);
    });
    std::thread sdk([&] {
        if (set_capture_affinity(std::uint64_t{1} << sdk_cpu) != 0) return;
        sdk_tid.store(read_capture_affinity().tid);
        ++ready;
        wait_for(2);
    });
    std::thread other([&] { other_tid.store(read_capture_affinity().tid); ++ready; wait_for(2); });
    std::atomic<std::int64_t> main_child{-1};
    std::thread main_child_thread;
    // Every thread is released and joined, and this thread takes its own set back, also when a
    // check fails (the failure is then reported, not an abort on a joinable thread).
    const auto release_all = [&] {
        { std::lock_guard<std::mutex> lock(mutex); stage = 2; }
        wake.notify_all();
        for (std::thread* thread : {&capture, &profile, &sdk, &other, &main_child_thread})
            if (thread->joinable()) thread->join();
        if (profile_child_thread.joinable()) profile_child_thread.join();
        if (capture_child_thread.joinable()) capture_child_thread.join();
        return set_capture_affinity(own.mask) == 0 && read_capture_affinity().mask == own.mask;
    };
    try {
        for (int spin = 0; spin < 5000 && ready.load() < 4; ++spin) std::this_thread::sleep_for(std::chrono::milliseconds(1));
        check(ready.load() == 4);
        const std::int64_t main_tid = read_capture_affinity().tid;
        const auto mask_of = [](std::int64_t tid) {
            std::uint64_t mask = 0;
            return read_task_affinity(tid, mask) == 0 ? mask : 0;
        };
        ProfileThreadPlacement placement(plan);
        const auto* applied = placement.poll(capture_tid.load(), profile_tid.load(), 0);
        check(applied && applied->success && applied->tasks == 5 && applied->moved_profile == 1 && applied->moved_other == 2 &&
              applied->capture == 1 && applied->sdk == 1 && applied->failed == 0);
        check(mask_of(profile_tid.load()) == plan.profile_mask && mask_of(other_tid.load()) == others && mask_of(main_tid) == others);
        check(mask_of(capture_tid.load()) == (std::uint64_t{1} << capture_cpu) && mask_of(sdk_tid.load()) == (std::uint64_t{1} << sdk_cpu));
        // Born later: from the Profile thread and from the capture thread (their cores, moved by
        // the recheck), and from the main thread (narrowed by the apply step: already right).
        main_child_thread = child(main_child);
        { std::lock_guard<std::mutex> lock(mutex); stage = 1; }
        wake.notify_all();
        for (int spin = 0; spin < 5000 && (profile_child.load() < 0 || capture_child.load() < 0 || main_child.load() < 0); ++spin)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        check(mask_of(profile_child.load()) == plan.profile_mask && mask_of(capture_child.load()) == (std::uint64_t{1} << capture_cpu));
        check(mask_of(main_child.load()) == others);
        check(placement.poll(capture_tid.load(), profile_tid.load(), 999) == nullptr);
        const auto* rechecked = placement.poll(capture_tid.load(), profile_tid.load(), 1000);
        check(rechecked && rechecked->success && rechecked->fresh == 3 && rechecked->moved_other == 2 && rechecked->unchanged == 1);
        check(mask_of(profile_child.load()) == others && mask_of(capture_child.load()) == others && mask_of(main_child.load()) == others);
        check(mask_of(profile_tid.load()) == plan.profile_mask && mask_of(capture_tid.load()) == (std::uint64_t{1} << capture_cpu));
        check(placement.poll(capture_tid.load(), profile_tid.load(), 2000) == nullptr);
    } catch (...) {
        (void)release_all();
        throw;
    }
    // The placement narrowed this (the main) thread; it takes its own set back.
    check(release_all());
    std::cout << "PROFILE_THREAD_PLACEMENT actual case sdk_cpu=" << sdk_cpu << " capture_cpu=" << capture_cpu
              << " profile_cpu=" << profile_cpu << " others_mask=" << others << " own_mask=" << own.mask << " ok\n";
}
#else
void actual_threads() { std::cout << "PROFILE_THREAD_PLACEMENT actual case skipped: not Linux\n"; }
#endif
}  // namespace

int main() {
    try {
        plans();
        roles();
        waits_for_both_tids();
        apply_and_recheck();
        apply_failures();
        actual_threads();
        std::cout << "PROFILE_THREAD_PLACEMENT_PASS checks=" << checks << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "PROFILE_THREAD_PLACEMENT_FAIL " << error.what() << '\n';
        return 1;
    }
}
