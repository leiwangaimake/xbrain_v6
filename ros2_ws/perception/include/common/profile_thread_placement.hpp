/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: profile_thread_placement.hpp
 * Brief: Optional placement of the Profile thread alone on one core, every other service thread off it.
 *
 * Description:
 * Runtime switch profile_thread_cpu (applied by the service itself, no environment variable):
 *   absent, "off"   nothing is read, set or logged;
 *   "1" "3" "7"     the Profile thread (svc_profile) is allowed exactly that core, and every
 *                   other thread of the process the inherited set without that core and without
 *                   the core of the camera SDK's stream threads (capture_sdk_threads_cpu).
 * Two kinds of thread are left as they are: the capture thread (cap_loop), which pins itself
 * after its first frame (capture_affinity_control.hpp), and every thread whose allowed set is
 * exactly the SDK core (the SDK's stream threads and the threads they create,
 * sdk_stream_thread_placement.hpp).
 *
 * Two checks. When the configuration is read (also by --check-config and the seal tool, which
 * may run on other cores than the service): the value is "off", "1", "2", "3" or "7", the capture
 * pin is on, the SDK core is set and is not the capture core, and the Profile core is neither the
 * SDK core nor the capture core (so "2" is refused). When the service starts
 * (plan_profile_thread_placement, pure): the inherited set is the set of the main thread before
 * anything is narrowed; the Profile core must be in it, and what remains for the other threads
 * (inherited set minus the Profile core minus the SDK core) must hold the capture core and at
 * least one more core. Anything else refuses the start.
 *
 * When. Only after the capture thread has pinned itself (it publishes its tid once its
 * CAPTURE_AFFINITY_CONTROL apply line reads back the pinned set): by then the SDK placement has
 * ended (it runs inside the capture start, before the capture thread exists), and the capture
 * thread was born with the full inherited set that its own pin requires. Narrowing any thread
 * earlier would hand a narrowed set to threads not yet created, among them the capture thread.
 * The Profile thread publishes its own tid when it starts; both tids are needed before the
 * first step.
 *
 * Steps (ProfileThreadPlacement::poll, from the Status thread's 10 ms loop):
 *   apply    once both tids are known: every task of the process is read and placed. The threads
 *            that create threads later (the main thread, the owners) are narrowed with the rest,
 *            so what they create is born on the other set;
 *   recheck  once a second after that: only tasks not seen before are read and placed. A thread
 *            born later inherits the set of the thread that created it, so a thread created by
 *            an already placed thread is already right; one created by the Profile thread
 *            (its core) or by the capture thread (the capture core) is moved here.
 * Every set is read back. One PROFILE_THREAD_PLACEMENT line for the plan at startup, one for the
 * apply step, and one for each recheck that moved or failed something or could not list the
 * tasks (the first such listing only). A task seen once is never touched again, also when its set
 * changes later. A task that ends between the listing and its read or set is not counted as
 * placed. A failed read, set or read-back is logged and the service runs on (the thread keeps
 * its set); it is not retried.
 * It never changes policy, priority or nice. Identification is by tid of this process (checked
 * with tgkill, as in sdk_stream_thread_placement.hpp); a tid that ends and is given to a new task
 * between two rechecks is not told apart.
 */

#pragma once

#include "common/capture_affinity_control.hpp"
#include "common/sdk_stream_thread_placement.hpp"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <ostream>
#include <vector>
#ifdef __linux__
#include <sched.h>
#include <sys/syscall.h>
#include <unistd.h>
#endif

namespace perception {

// The Profile thread's tid, published by that thread when it starts; -1 before.
inline std::atomic<std::int64_t>& profile_thread_tid() noexcept {
    static std::atomic<std::int64_t> tid{-1};
    return tid;
}

// On the Profile thread, once, when it starts.
inline void publish_profile_thread_tid() noexcept {
#ifdef __linux__
    CaptureAffinityErrno keep_errno;
    const long tid = ::syscall(SYS_gettid);
    if (tid > 0) profile_thread_tid().store(tid, std::memory_order_release);
#endif
}

inline std::uint64_t cpu_bit(int cpu) noexcept {
    return cpu >= 0 && cpu < 64 ? std::uint64_t{1} << cpu : 0;
}

struct ProfileThreadPlan {
    bool ok = false;
    const char* reason = "not_planned";
    int profile_cpu = -1, sdk_cpu = -1, capture_cpu = -1;
    std::uint64_t inherited_mask = 0, profile_mask = 0, others_mask = 0;
};

// The masks of the placement from the inherited set and the three cores; ok false with the
// first rule that fails.
inline ProfileThreadPlan plan_profile_thread_placement(std::uint64_t inherited, int profile_cpu, int sdk_cpu,
                                                       int capture_cpu) noexcept {
    ProfileThreadPlan plan;
    plan.profile_cpu = profile_cpu;
    plan.sdk_cpu = sdk_cpu;
    plan.capture_cpu = capture_cpu;
    plan.inherited_mask = inherited;
    if (!cpu_bit(profile_cpu)) { plan.reason = "profile_cpu_invalid"; return plan; }
    if (!cpu_bit(sdk_cpu)) { plan.reason = "sdk_cpu_invalid"; return plan; }
    if (!cpu_bit(capture_cpu)) { plan.reason = "capture_cpu_invalid"; return plan; }
    if (!(inherited & cpu_bit(profile_cpu))) { plan.reason = "profile_cpu_not_inherited"; return plan; }
    if (profile_cpu == sdk_cpu) { plan.reason = "profile_cpu_is_sdk_cpu"; return plan; }
    if (profile_cpu == capture_cpu) { plan.reason = "profile_cpu_is_capture_cpu"; return plan; }
    const std::uint64_t others = inherited & ~cpu_bit(profile_cpu) & ~cpu_bit(sdk_cpu);
    if (!(others & cpu_bit(capture_cpu))) { plan.reason = "capture_cpu_not_left"; return plan; }
    if (!(others & ~cpu_bit(capture_cpu))) { plan.reason = "others_too_few"; return plan; }
    plan.profile_mask = cpu_bit(profile_cpu);
    plan.others_mask = others;
    plan.ok = true;
    plan.reason = "ok";
    return plan;
}

enum class ProfileThreadRole { Capture, Profile, SdkFamily, Other };

struct ProfileThreadTarget {
    ProfileThreadRole role = ProfileThreadRole::Other;
    std::uint64_t mask = 0;   // the set this task is to have; 0 = left as it is
};

// What one task is to be allowed, from its tid and its current set.
inline ProfileThreadTarget profile_thread_target(const ProfileThreadPlan& plan, std::int64_t tid, std::uint64_t current,
                                                 std::int64_t profile_tid, std::int64_t capture_tid) noexcept {
    if (tid == capture_tid) return {ProfileThreadRole::Capture, 0};
    if (tid == profile_tid) return {ProfileThreadRole::Profile, plan.profile_mask};
    if (current == cpu_bit(plan.sdk_cpu)) return {ProfileThreadRole::SdkFamily, 0};
    return {ProfileThreadRole::Other, plan.others_mask};
}

// Sets the allowed CPUs of one task of this process: 0, ESRCH when this process has no such
// task (any more), or another errno.
inline int set_task_affinity(std::int64_t tid, std::uint64_t bits) noexcept {
    CaptureAffinityErrno keep_errno;
#ifdef __linux__
    if (tid <= 0) return ESRCH;
    if (!bits) return EINVAL;
    if (::syscall(SYS_tgkill, ::getpid(), static_cast<pid_t>(tid), 0) != 0) return errno;
    cpu_set_t set;
    CPU_ZERO(&set);
    for (int cpu = 0; cpu < 64; ++cpu)
        if (bits & (std::uint64_t{1} << cpu)) CPU_SET(cpu, &set);
    return ::sched_setaffinity(static_cast<pid_t>(tid), sizeof(set), &set) == 0 ? 0 : errno;
#else
    (void)tid;
    (void)bits;
    return ENOTSUP;
#endif
}

// System calls behind the placement; a test replaces them.
struct ProfileThreadPlacementOps {
    bool (*list_tasks)(std::vector<std::int64_t>&) noexcept = list_process_tasks;
    int (*task_mask)(std::int64_t, std::uint64_t&) noexcept = read_task_affinity;
    int (*set_task_mask)(std::int64_t, std::uint64_t) noexcept = set_task_affinity;
};

struct ProfileThreadPlacementRecord {
    const char* event = "apply";
    std::uint64_t round = 0;
    ProfileThreadPlan plan;
    std::int64_t capture_tid = -1, profile_tid = -1;
    // tasks listed; new: not seen before; moved to the Profile core / to the other set; already
    // right; left as they are (capture thread, SDK family); ended before they were read or set;
    // failed reads, sets or read-backs.
    std::size_t tasks = 0, fresh = 0, moved_profile = 0, moved_other = 0, unchanged = 0, capture = 0, sdk = 0,
        ended = 0, failed = 0;
    std::int64_t first_fail_tid = -1;
    int first_fail_errno = 0;
    const char* first_fail = "none";
    bool listed = false;   // the tasks of the process could be listed
    bool success = false;
    const char* reason = "not_attempted";
};

class ProfileThreadPlacement {
public:
    static constexpr std::int64_t kRecheckMs = 1000;

    explicit ProfileThreadPlacement(const ProfileThreadPlan& plan, ProfileThreadPlacementOps ops = {})
        : plan_(plan), ops_(ops) {
        tasks_.reserve(256);
        seen_.reserve(256);
        next_seen_.reserve(256);
    }
    ProfileThreadPlacement(const ProfileThreadPlacement&) = delete;
    ProfileThreadPlacement& operator=(const ProfileThreadPlacement&) = delete;

    bool applied() const noexcept { return applied_; }

    // From one periodic thread. Returns the record of a step to log, or nullptr: nothing due,
    // or a recheck that found nothing to change.
    const ProfileThreadPlacementRecord* poll(std::int64_t capture_tid, std::int64_t profile_tid,
                                             std::int64_t now_ms) noexcept {
        if (!plan_.ok) return nullptr;
        if (!applied_) {
            if (capture_tid <= 0 || profile_tid <= 0) return nullptr;
            applied_ = true;
            capture_tid_ = capture_tid;
            profile_tid_ = profile_tid;
            next_ms_ = now_ms + kRecheckMs;
            step("apply");
            listed_last_ = record_.listed;
            return &record_;
        }
        if (now_ms < next_ms_) return nullptr;
        next_ms_ = now_ms + kRecheckMs;
        step("recheck");
        // A listing that keeps failing is reported once, when it starts to fail.
        const bool report = record_.moved_profile + record_.moved_other + record_.failed > 0 ||
                            (!record_.listed && listed_last_);
        listed_last_ = record_.listed;
        return report ? &record_ : nullptr;
    }

private:
    void fail(std::int64_t tid, int code, const char* what) noexcept {
        if (record_.failed++ == 0) {
            record_.first_fail_tid = tid;
            record_.first_fail_errno = code;
            record_.first_fail = what;
        }
    }

    void step(const char* event) noexcept {
        CaptureAffinityErrno keep_errno;
        record_ = ProfileThreadPlacementRecord{};
        record_.event = event;
        record_.round = ++round_;
        record_.plan = plan_;
        record_.capture_tid = capture_tid_;
        record_.profile_tid = profile_tid_;
        if (!ops_.list_tasks(tasks_)) {
            record_.reason = "list_tasks_failed";
            return;
        }
        record_.listed = true;
        try {
            place_listed();
        } catch (...) {   // a list that could not grow: what was placed stays, the next recheck looks again
            fail(-1, ENOMEM, "step_failed");
            record_.reason = "step_failed";
            return;
        }
        if (record_.failed) record_.reason = record_.first_fail;
        else if (!profile_listed_) record_.reason = "profile_thread_not_listed";
        else { record_.success = true; record_.reason = "ok"; }
    }

    void place_listed() {
        std::sort(tasks_.begin(), tasks_.end());
        record_.tasks = tasks_.size();
        profile_listed_ = false;
        next_seen_.clear();
        for (const std::int64_t tid : tasks_) {
            if (tid == profile_tid_) profile_listed_ = true;
            if (std::binary_search(seen_.begin(), seen_.end(), tid)) {
                next_seen_.push_back(tid);
                continue;
            }
            ++record_.fresh;
            std::uint64_t current = 0;
            const int read_code = ops_.task_mask(tid, current);
            if (read_code == ESRCH) { ++record_.ended; continue; }
            if (read_code != 0) { fail(tid, read_code, "read_failed"); next_seen_.push_back(tid); continue; }
            const auto target = profile_thread_target(plan_, tid, current, profile_tid_, capture_tid_);
            next_seen_.push_back(tid);
            if (!target.mask) {
                ++(target.role == ProfileThreadRole::Capture ? record_.capture : record_.sdk);
                continue;
            }
            if (current == target.mask) { ++record_.unchanged; continue; }
            const int set_code = ops_.set_task_mask(tid, target.mask);
            if (set_code == ESRCH) { ++record_.ended; next_seen_.pop_back(); continue; }
            if (set_code != 0) { fail(tid, set_code, "set_failed"); continue; }
            std::uint64_t back = 0;
            const int back_code = ops_.task_mask(tid, back);
            if (back_code == ESRCH) { ++record_.ended; next_seen_.pop_back(); continue; }
            if (back_code != 0 || back != target.mask) {
                fail(tid, back_code, "readback_mismatch");
                continue;
            }
            ++(target.role == ProfileThreadRole::Profile ? record_.moved_profile : record_.moved_other);
        }
        seen_.swap(next_seen_);
    }

    ProfileThreadPlan plan_;
    ProfileThreadPlacementOps ops_;
    bool applied_ = false, listed_last_ = false, profile_listed_ = false;
    std::int64_t capture_tid_ = -1, profile_tid_ = -1, next_ms_ = 0;
    std::uint64_t round_ = 0;
    std::vector<std::int64_t> tasks_, seen_, next_seen_;
    ProfileThreadPlacementRecord record_;
};

// The plan, logged once at startup when the placement is configured.
inline std::ostream& operator<<(std::ostream& out, const ProfileThreadPlan& p) {
    return out << "PROFILE_THREAD_PLACEMENT schema=1 mask_encoding=cpu_bitset_u64 event=plan profile_cpu="
        << p.profile_cpu << " sdk_cpu=" << p.sdk_cpu << " capture_cpu=" << p.capture_cpu
        << " inherited_mask=" << p.inherited_mask << " profile_mask=" << p.profile_mask
        << " others_mask=" << p.others_mask << " success=" << static_cast<int>(p.ok) << " reason=" << p.reason;
}

inline std::ostream& operator<<(std::ostream& out, const ProfileThreadPlacementRecord& r) {
    return out << "PROFILE_THREAD_PLACEMENT schema=1 mask_encoding=cpu_bitset_u64 event=" << r.event
        << " round=" << r.round << " profile_cpu=" << r.plan.profile_cpu << " sdk_cpu=" << r.plan.sdk_cpu
        << " capture_cpu=" << r.plan.capture_cpu << " inherited_mask=" << r.plan.inherited_mask
        << " profile_mask=" << r.plan.profile_mask << " others_mask=" << r.plan.others_mask
        << " capture_tid=" << r.capture_tid << " profile_tid=" << r.profile_tid << " tasks=" << r.tasks
        << " new=" << r.fresh << " moved_profile=" << r.moved_profile << " moved_other=" << r.moved_other
        << " unchanged=" << r.unchanged << " capture=" << r.capture << " sdk=" << r.sdk << " ended=" << r.ended
        << " failed=" << r.failed << " first_fail_tid=" << r.first_fail_tid
        << " first_fail_errno=" << r.first_fail_errno << " first_fail=" << r.first_fail
        << " listed=" << static_cast<int>(r.listed) << " success=" << static_cast<int>(r.success)
        << " reason=" << r.reason;
}

}  // namespace perception
