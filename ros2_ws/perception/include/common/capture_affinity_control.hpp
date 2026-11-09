/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: capture_affinity_control.hpp
 * Brief: Optional CPU pinning of the capture thread to core 2, with an audited restore.
 *
 * Description:
 * PERCEPTION_CAPTURE_AFFINITY_PIN2 (runtime switch capture_affinity_pin2, production "1"):
 *   unset  Disabled: never reads, sets or logs affinity;
 *   "0"    Observe: records the thread's affinity and scheduling at the first frame and at
 *          exit, changes nothing;
 *   "1"    Pin2: after the first complete frame, narrows the capture thread (only that task)
 *          from the inherited set, which must contain cores 2 and 3, to core 2; restores the
 *          inherited set when cap_loop exits (RAII guard).
 * Every step is read back and logged as one CAPTURE_AFFINITY_CONTROL line. A failed or
 * unverifiable first-frame step (in either mode) ends the capture loop instead of running
 * on (no silent fallback). After a successful Pin2 step the capture loop publishes its tid
 * (capture_pinned_tid): from then on the Profile thread placement may narrow the other
 * threads (profile_thread_placement.hpp).
 * It never changes scheduling policy, priority or nice, and never touches another thread.
 */

#pragma once

#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <ostream>
#include <stdexcept>
#include <utility>
#include <time.h>
#ifdef __linux__
#include <sched.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>
#endif

namespace perception {

enum class CaptureAffinityMode { Disabled, Observe, Pin2 };

// The core Pin2 narrows the capture thread to.
constexpr int kCapturePinCpu = 2;

// The capture thread's tid once its Pin2 apply succeeded and was logged; -1 before. Published
// by the capture loop, read by the Profile thread placement (profile_thread_placement.hpp).
inline std::atomic<std::int64_t>& capture_pinned_tid() noexcept {
    static std::atomic<std::int64_t> tid{-1};
    return tid;
}

inline CaptureAffinityMode parse_capture_affinity_control(const char* value) {
    if (!value) return CaptureAffinityMode::Disabled;
    if (std::strcmp(value, "0") == 0) return CaptureAffinityMode::Observe;
    if (std::strcmp(value, "1") == 0) return CaptureAffinityMode::Pin2;
    throw std::invalid_argument("PERCEPTION_CAPTURE_AFFINITY_PIN2 must be exactly 0 or 1");
}

struct CaptureAffinityErrno {
    int saved = errno;
    ~CaptureAffinityErrno() noexcept { errno = saved; }
};

struct CaptureAffinitySnapshot {
    bool valid = false;
    const char* reason = "not_read";
    int error = 0;
    std::int64_t tid = -1, observed_ns = -1;
    std::uint64_t mask = 0;
    int policy = -1, priority = -1, nice = -1;
};

// Current Linux task only; no process-wide enumeration or other-thread mutation.
// Snapshot time is sampled AFTER these reads, not an atomic scheduler snapshot.
inline CaptureAffinitySnapshot read_capture_affinity() noexcept {
    CaptureAffinityErrno keep_errno;
    CaptureAffinitySnapshot out;
#ifdef __linux__
    out.tid = static_cast<std::int64_t>(::syscall(SYS_gettid));
    if (out.tid <= 0) { out.reason = "gettid_failed"; out.error = errno; return out; }
    cpu_set_t mask;
    CPU_ZERO(&mask);
    if (::sched_getaffinity(0, sizeof(mask), &mask) != 0) {
        out.reason = "getaffinity_failed"; out.error = errno; return out;
    }
    for (int cpu = 0; cpu < CPU_SETSIZE; ++cpu) {
        if (!CPU_ISSET(cpu, &mask)) continue;
        if (cpu >= 64) { out.reason = "affinity_mask_outside_u64"; return out; }
        out.mask |= (std::uint64_t{1} << cpu);
    }
    if (out.mask == 0) { out.reason = "empty_affinity_mask"; return out; }
    out.policy = ::sched_getscheduler(0);
    if (out.policy < 0) { out.reason = "getscheduler_failed"; out.error = errno; return out; }
    sched_param param{};
    if (::sched_getparam(0, &param) != 0) {
        out.reason = "getparam_failed"; out.error = errno; return out;
    }
    out.priority = param.sched_priority;
    errno = 0;
    out.nice = ::getpriority(PRIO_PROCESS, static_cast<id_t>(out.tid));
    if (errno != 0) { out.reason = "getpriority_failed"; out.error = errno; return out; }
    timespec now{};
    if (::clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
        out.reason = "clock_failed"; out.error = errno; return out;
    }
    out.observed_ns = static_cast<std::int64_t>(now.tv_sec) * 1000000000LL + now.tv_nsec;
    if (out.observed_ns <= 0) { out.reason = "nonpositive_clock"; return out; }
    out.valid = true;
    out.reason = "ok";
#else
    out.reason = "unsupported_platform";
#endif
    return out;
}

// PID 0 means this calling task. Never sets scheduling policy, priority or nice.
inline int set_capture_affinity(std::uint64_t bits) noexcept {
    CaptureAffinityErrno keep_errno;
#ifdef __linux__
    if (!bits) return EINVAL;
    cpu_set_t mask;
    CPU_ZERO(&mask);
    for (int cpu = 0; cpu < 64; ++cpu)
        if (bits & (std::uint64_t{1} << cpu)) CPU_SET(cpu, &mask);
    return ::sched_setaffinity(0, sizeof(mask), &mask) == 0 ? 0 : errno;
#else
    (void)bits;
    return ENOTSUP;
#endif
}

struct CaptureAffinityOps {
    CaptureAffinitySnapshot (*read)() noexcept = read_capture_affinity;
    int (*set)(std::uint64_t) noexcept = set_capture_affinity;
};

struct CaptureAffinityRecord {
    const char* event = "apply";
    CaptureAffinityMode mode = CaptureAffinityMode::Disabled;
    std::uint64_t frame_id = 0, requested_mask = 0;
    CaptureAffinitySnapshot before, after;
    bool set_attempted = false, success = false;
    int set_errno = 0;
    const char* reason = "not_attempted";
};

inline bool same_capture_scheduling(const CaptureAffinitySnapshot& a,
                                  const CaptureAffinitySnapshot& b) noexcept {
    return a.policy == b.policy && a.priority == b.priority && a.nice == b.nice;
}

class CaptureAffinityControl {
public:
    explicit CaptureAffinityControl(CaptureAffinityMode mode, CaptureAffinityOps ops = {}) noexcept
        : mode_(mode), ops_(ops) {}
    CaptureAffinityControl(const CaptureAffinityControl&) = delete;
    CaptureAffinityControl& operator=(const CaptureAffinityControl&) = delete;

    bool needs_apply() const noexcept { return mode_ != CaptureAffinityMode::Disabled && !attempted_; }
    bool needs_restore() const noexcept {
        return !restore_attempted_ && (applied_ ||
            (mode_ == CaptureAffinityMode::Observe && apply_.success));
    }

    // Call only after the first complete successful handoff, never before the
    // common first converter call. This does NOT prove all SDK lazy workers have
    // been created: any later child-thread inheritance needs runtime telemetry.
    const CaptureAffinityRecord& apply(std::uint64_t first_frame_id) noexcept {
        CaptureAffinityErrno keep_errno;
        if (!needs_apply()) return apply_;
        attempted_ = true;
        apply_.mode = mode_;
        apply_.frame_id = first_frame_id;
        apply_.before = ops_.read();
        original_ = apply_.before;
        // Pin2 narrows this thread to core 2; Observe expects the inherited set back unchanged.
        apply_.requested_mask = mode_ == CaptureAffinityMode::Pin2 ? std::uint64_t{1} << kCapturePinCpu
                                                                   : (original_.valid ? original_.mask : 12U);
        if (!original_.valid) { apply_.reason = "initial_read_failed"; return apply_; }
        // The inherited set must contain cores 2 and 3: production runs on exactly 2-3, the U87
        // diagnostic lends one more core (2-3 plus 7). Pinning never widens the set, it only narrows
        // to core 2, so a superset is safe; a set without 2 or 3 is not the approved placement.
        if ((original_.mask & 12U) != 12U) { apply_.reason = "initial_mask_lacks_2_3"; return apply_; }
        if (mode_ == CaptureAffinityMode::Pin2) {
            apply_.set_attempted = true;
            apply_.set_errno = ops_.set(apply_.requested_mask);
            applied_ = apply_.set_errno == 0;
        }
        // Always read back after an attempted set, including failure. Never
        // report the requested mask as observed, and never silently fall back.
        apply_.after = ops_.read();
        if (apply_.set_errno != 0) apply_.reason = "setaffinity_failed";
        else if (!apply_.after.valid) apply_.reason = "readback_failed";
        else if (apply_.after.tid != original_.tid) apply_.reason = "thread_identity_changed";
        else if (apply_.after.observed_ns < original_.observed_ns) apply_.reason = "clock_regressed";
        else if (apply_.after.mask != apply_.requested_mask) apply_.reason = "readback_mask_mismatch";
        else if (!same_capture_scheduling(original_, apply_.after)) apply_.reason = "scheduling_metadata_changed";
        else { apply_.success = true; apply_.reason = "ok"; }
        return apply_;
    }

    const CaptureAffinityRecord& restore() noexcept {
        CaptureAffinityErrno keep_errno;
        if (!needs_restore()) return restore_;
        restore_attempted_ = true;
        restore_.event = "restore";
        restore_.mode = mode_;
        restore_.frame_id = apply_.frame_id;
        restore_.requested_mask = original_.mask;
        restore_.before = ops_.read();
        // The guard belongs to the capture thread. Never mutate a different
        // thread if ownership is violated, even though the setter targets self.
        if (restore_.before.tid != original_.tid) {
            restore_.reason = "restore_thread_identity_mismatch";
            return restore_;
        }
        // Observe has the same exit bracket but never invokes a setter.
        if (mode_ == CaptureAffinityMode::Pin2) {
            restore_.set_attempted = true;
            restore_.set_errno = ops_.set(original_.mask);
        }
        restore_.after = ops_.read();
        if (restore_.set_errno != 0) restore_.reason = "restore_setaffinity_failed";
        else if (!restore_.before.valid || !restore_.after.valid) restore_.reason = "restore_snapshot_failed";
        else if (restore_.after.tid != original_.tid) restore_.reason = "restore_thread_identity_mismatch";
        else if (restore_.after.observed_ns < restore_.before.observed_ns) restore_.reason = "restore_clock_regressed";
        else if (restore_.before.mask != apply_.requested_mask) restore_.reason = "restore_initial_mask_mismatch";
        else if (restore_.after.mask != original_.mask) restore_.reason = "restore_readback_mask_mismatch";
        else if (!same_capture_scheduling(original_, restore_.before) ||
                 !same_capture_scheduling(original_, restore_.after)) restore_.reason = "restore_scheduling_metadata_changed";
        else { restore_.success = true; restore_.reason = "ok"; }
        return restore_;
    }
private:
    CaptureAffinityMode mode_;
    CaptureAffinityOps ops_;
    bool attempted_ = false, applied_ = false, restore_attempted_ = false;
    CaptureAffinitySnapshot original_;
    CaptureAffinityRecord apply_, restore_;
};

template<class Sink> bool emit_capture_affinity(const CaptureAffinityRecord& record, Sink&& sink) noexcept {
    CaptureAffinityErrno keep_errno;
    try { std::forward<Sink>(sink)(record); return true; }
    catch (...) { return false; }
}

template<class Sink> class CaptureAffinityRestoreGuard {
public:
    CaptureAffinityRestoreGuard(CaptureAffinityControl& control, Sink sink)
        : control_(control), sink_(std::move(sink)) {}
    ~CaptureAffinityRestoreGuard() noexcept {
        if (control_.needs_restore()) (void)emit_capture_affinity(control_.restore(), sink_);
    }
    CaptureAffinityRestoreGuard(const CaptureAffinityRestoreGuard&) = delete;
    CaptureAffinityRestoreGuard& operator=(const CaptureAffinityRestoreGuard&) = delete;
private:
    CaptureAffinityControl& control_;
    Sink sink_;
};

inline std::ostream& operator<<(std::ostream& out, const CaptureAffinityRecord& r) {
    return out << "CAPTURE_AFFINITY_CONTROL schema=1 clock=CLOCK_MONOTONIC mask_encoding=cpu_bitset_u64"
        << " event=" << r.event << " mode=" << (r.mode == CaptureAffinityMode::Pin2 ? "pin2" : "observe")
        << " frame_id=" << r.frame_id << " tid=" << r.before.tid
        << " before_ns=" << r.before.observed_ns << " after_ns=" << r.after.observed_ns
        << " before_tid=" << r.before.tid << " after_tid=" << r.after.tid
        << " before_mask=" << r.before.mask << " requested_mask=" << r.requested_mask << " after_mask=" << r.after.mask
        << " before_policy=" << r.before.policy << " after_policy=" << r.after.policy
        << " before_priority=" << r.before.priority << " after_priority=" << r.after.priority
        << " before_nice=" << r.before.nice << " after_nice=" << r.after.nice
        << " before_valid=" << static_cast<int>(r.before.valid) << " after_valid=" << static_cast<int>(r.after.valid)
        << " before_reason=" << r.before.reason << " after_reason=" << r.after.reason
        << " before_errno=" << r.before.error << " after_errno=" << r.after.error
        << " set_attempted=" << static_cast<int>(r.set_attempted) << " set_errno=" << r.set_errno
        << " success=" << static_cast<int>(r.success) << " reason=" << r.reason;
}

} // namespace perception
