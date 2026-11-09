/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: sdk_stream_thread_placement.hpp
 * Brief: Optional placement of the camera SDK's stream threads on one CPU, with a self-check.
 *
 * Description:
 * PERCEPTION_CAPTURE_SDK_THREADS_CPU (runtime switch capture_sdk_threads_cpu):
 *   unset, "off"        Disabled: nothing is read, set or logged;
 *   "1" "2" "3" "7"     the threads the camera SDK creates while the pipeline starts are placed
 *                       on that one CPU and stay there.
 *
 * Which threads. Not "the busiest ones": the threads created by the thread that calls
 * ob::Pipeline::start(), during that call. A new thread inherits the allowed CPUs of its
 * creator, so the caller narrows itself to the target CPU just before the call and takes its own
 * set back right after it; the SDK's threads are born on the target, and so are the threads they
 * create later. For the Gemini 338Le over Ethernet with two video streams (measured 2026-10-13,
 * three starts, identical: notes "tbirth1013") that is, per stream and in this order, one sensor
 * queue thread and the RTP receive and assemble threads: six threads. The four RTP threads are
 * the high-rate ones (2000-3400 timeslices a second each); the queue threads wake once a frame
 * and each creates one frame-processor queue thread at its first frame. The SDK names none of
 * them and sets no affinity itself.
 *
 * Self-check.
 *   begin()   own set read; the target must not already be the whole set (the family could not
 *             be told apart then); the tasks of the process listed; own set narrowed and read back.
 *   end()     tasks listed again, then own set restored and read back, whatever happened. The
 *             placement stands only when what was born in this process during the bracket is
 *             exactly the expected family: `expected` new tasks, every one with exactly the
 *             target as its allowed set and a readable start time, and no new task with another
 *             set or an unreadable one, and none that has already ended when it is read (such a
 *             task cannot be told from one of the expected threads that ended while a child of
 *             another took its place in the count). Fewer: a stream thread was created by
 *             some other thread. More: a family thread already created a child, and this
 *             process cannot tell children
 *             from the caller's own threads -- neither by tid order (a child of the first
 *             stream may be born before the second stream's threads) nor otherwise -- so it
 *             does not guess. A new task off the target may be a stream thread that some other
 *             thread created, or an unrelated thread; it cannot tell which. In each of these
 *             cases the placement is reported as a fault: not verified, which is not the same
 *             as known to be wrong. Each member is kept as tid plus start time (field 22 of its
 *             stat file), so a later task that got the same tid is not taken for it.
 *   verify()  every member that has not ended is read: same task (tid and start time), allowed
 *             set exactly the target. A member with another set, or that cannot be read, is a
 *             fault while that lasts. A member that has ended is a fault for good (a tid that
 *             appears again is another task): the SDK stopped or restarted a stream by itself,
 *             and threads it creates then inherit from an SDK thread, not from the caller.
 *   survey()  how many tasks of the process have exactly the target now, whatever their
 *             origin (with target 2 that includes the capture thread, which pins itself
 *             there); reported, not judged.
 * begin(), end() and the survey fill one record each, which the caller logs as an
 * SDK_THREAD_PLACEMENT line; the periodic check is logged when its answer, its reason or the
 * number of ended members changes (SdkThreadPlacementWatch). A failed step is a fault: Status
 * raises the reason capture_threads_misplaced from sdk_thread_placement_state() while the
 * capture loop runs (the loop publishes Disabled when it ends: nobody checks any more), and the
 * capture goes on (19 section 9: publish and label, never stop silently). One exception: when
 * the caller's own set is not confirmed restored (caller_restore_unconfirmed()), every thread
 * the caller creates afterwards would inherit the single CPU, so the capture start fails before
 * it creates any. That holds for later starts in the same process as well: the thread is
 * remembered (sdk_thread_placement_left_narrowed()) and no placement begins on it again.
 *
 * What is not checked. Threads the family creates after the listing (per stream one
 * frame-processor queue thread at the first frame, short-lived log threads) are on the target by
 * inheritance and are counted by the survey, not verified one by one: this process cannot tell
 * them from other threads that are pinned to the same CPU. An SDK that creates its stream
 * threads differently (another count, another creating thread) fails the check at the start;
 * the thread births then have to be listed again (harness/thread_birth_log.c). The listing of
 * /proc/self/task is taken as complete, and a thread whose whole life lies between the two
 * listings is not seen at all. That the six are the SDK's own stream threads, and not six other
 * threads born on the target, rests on what the SDK was measured to do during the call
 * (orbbec_capture.cpp, SDK_START_THREADS), not on anything read here.
 *
 * It never changes policy, priority or nice and never sets the affinity of another thread; for
 * the calling thread it requests two sets only, the configured CPU and that thread's own
 * original set. A task that ends and whose tid is given to another task with the same start
 * tick (10 ms) is not told apart; neither is a task that existed before the start, ended during
 * it and whose tid a new task got during the same call.
 */

#pragma once

#include "common/capture_affinity_control.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ostream>
#include <stdexcept>
#include <utility>
#include <vector>
#ifdef __linux__
#include <dirent.h>
#include <fcntl.h>
#include <sched.h>
#include <sys/syscall.h>
#include <unistd.h>
#endif

namespace perception {

// The configured CPU, or -1 when the placement is switched off. Anything but the listed
// spellings is refused: cores 0, 4, 5 and 6 are not perception's to use.
inline int parse_sdk_stream_threads_cpu(const char* value) {
    if (!value || std::strcmp(value, "off") == 0) return -1;
    for (const char* allowed : {"1", "2", "3", "7"})
        if (std::strcmp(value, allowed) == 0) return allowed[0] - '0';
    throw std::invalid_argument("PERCEPTION_CAPTURE_SDK_THREADS_CPU must be exactly off, 1, 2, 3 or 7");
}

// What the Status owner reads: Disabled (switch off), Placed (the last check passed) or Fault.
enum class SdkThreadPlacementState : int { Disabled = 0, Placed = 1, Fault = 2 };
inline std::atomic<int>& sdk_thread_placement_state() noexcept {
    static std::atomic<int> state{static_cast<int>(SdkThreadPlacementState::Disabled)};
    return state;
}
inline void publish_sdk_thread_placement(SdkThreadPlacementState state) noexcept {
    sdk_thread_placement_state().store(static_cast<int>(state), std::memory_order_relaxed);
}

// The thread a placement of this process left narrowed (the restore of its own set was not
// confirmed), or -1. It is never cleared: nothing here knows that thread's set to be right again.
// One slot: it serves one calling thread per process (the service starts the camera from its
// main thread and leaves when this happens). A second thread that fails would overwrite it.
inline std::atomic<std::int64_t>& sdk_thread_placement_left_narrowed() noexcept {
    static std::atomic<std::int64_t> tid{-1};
    return tid;
}

// The tids of this process's tasks, ascending. false when they cannot be listed.
inline bool list_process_tasks(std::vector<std::int64_t>& out) noexcept {
    CaptureAffinityErrno keep_errno;
    try {
        out.clear();
#ifdef __linux__
        DIR* directory = ::opendir("/proc/self/task");
        if (!directory) return false;
        struct Close { DIR* directory; ~Close() { ::closedir(directory); } } close{directory};   // also when push_back throws
        while (const dirent* entry = ::readdir(directory)) {
            char* end = nullptr;
            const long long tid = std::strtoll(entry->d_name, &end, 10);
            if (end != entry->d_name && *end == '\0' && tid > 0) out.push_back(tid);
        }
        std::sort(out.begin(), out.end());
        return !out.empty();
#else
        return false;
#endif
    } catch (...) {
        return false;
    }
}

// Allowed CPUs of one task of this process as a bit set: 0, ESRCH when this process has no such
// task (any more), or another errno (ERANGE for a CPU beyond bit 63). sched_getaffinity alone
// would also answer for a thread of another process that got the same tid.
inline int read_task_affinity(std::int64_t tid, std::uint64_t& mask) noexcept {
    CaptureAffinityErrno keep_errno;
    mask = 0;
#ifdef __linux__
    if (tid <= 0) return ESRCH;
    if (::syscall(SYS_tgkill, ::getpid(), static_cast<pid_t>(tid), 0) != 0) return errno;
    cpu_set_t set;
    CPU_ZERO(&set);
    if (::sched_getaffinity(static_cast<pid_t>(tid), sizeof(set), &set) != 0) return errno;
    for (int cpu = 0; cpu < CPU_SETSIZE; ++cpu) {
        if (!CPU_ISSET(cpu, &set)) continue;
        if (cpu >= 64) return ERANGE;
        mask |= std::uint64_t{1} << cpu;
    }
    return 0;
#else
    (void)tid;
    return ENOTSUP;
#endif
}

// The start time (clock ticks since boot) in the text of a task's stat file: field 22. The name
// in field 2 may hold spaces and brackets, so the fields are counted from its last ')'.
inline bool parse_task_start(const char* text, std::uint64_t& ticks) noexcept {
    CaptureAffinityErrno keep_errno;
    ticks = 0;
    const char* at = text ? std::strrchr(text, ')') : nullptr;
    if (!at) return false;
    ++at;
    for (int field = 3; field <= 21; ++field) {   // state ... itrealvalue
        while (*at == ' ') ++at;
        if (*at == '\0') return false;
        while (*at != ' ' && *at != '\0') ++at;
    }
    while (*at == ' ') ++at;
    if (*at < '0' || *at > '9') return false;
    char* end = nullptr;
    errno = 0;
    const unsigned long long value = std::strtoull(at, &end, 10);
    if (errno != 0 || end == at || (*end != ' ' && *end != '\n' && *end != '\0')) return false;
    ticks = value;
    return true;
}

// Start time of one task of this process: 0, ESRCH when this process has no such task (any
// more), or another errno (EPROTO for a stat file that cannot be parsed).
inline int read_task_start(std::int64_t tid, std::uint64_t& ticks) noexcept {
    CaptureAffinityErrno keep_errno;
    ticks = 0;
#ifdef __linux__
    if (tid <= 0) return ESRCH;
    char path[64];
    std::snprintf(path, sizeof(path), "/proc/self/task/%lld/stat", static_cast<long long>(tid));
    const int fd = ::open(path, O_RDONLY | O_CLOEXEC);
    if (fd < 0) return errno == ENOENT ? ESRCH : errno;
    char text[1024];
    std::size_t size = 0;
    int code = 0;
    while (size < sizeof(text) - 1) {
        const ssize_t got = ::read(fd, text + size, sizeof(text) - 1 - size);
        if (got < 0 && errno == EINTR) continue;
        if (got < 0) code = errno;   // ESRCH: the task ended after the open
        if (got <= 0) break;
        size += static_cast<std::size_t>(got);
    }
    ::close(fd);
    if (code != 0) return code;
    text[size] = '\0';
    return parse_task_start(text, ticks) ? 0 : EPROTO;
#else
    (void)tid;
    return ENOTSUP;
#endif
}

// System calls behind the placement; a test replaces them.
struct SdkThreadPlacementOps {
    CaptureAffinitySnapshot (*read_self)() noexcept = read_capture_affinity;
    int (*set_self)(std::uint64_t) noexcept = set_capture_affinity;
    bool (*list_tasks)(std::vector<std::int64_t>&) noexcept = list_process_tasks;
    int (*task_mask)(std::int64_t, std::uint64_t&) noexcept = read_task_affinity;
    int (*task_start)(std::int64_t, std::uint64_t&) noexcept = read_task_start;
};

struct SdkThreadPlacementRecord {
    static constexpr std::size_t kMaxFamily = 16;
    const char* event = "begin";
    int target_cpu = -1;
    bool success = false;
    const char* reason = "not_attempted";
    std::int64_t caller_tid = -1;
    // The caller's own set: as found, after narrowing, after the restore (0 = not read).
    std::uint64_t original_mask = 0, narrowed_mask = 0, restored_mask = 0;
    int set_errno = 0, restore_errno = 0;
    // end(): tasks before and after the call; new ones; new ones that had ended when they were
    // read (any refuses the placement); new ones with another set or an unreadable set; the
    // family (new ones with exactly the target and a readable start time; family_tids holds
    // the first kMaxFamily of them).
    std::size_t tasks_before = 0, tasks_after = 0, born = 0, born_ended = 0, born_elsewhere = 0, family = 0,
        expected = 0;
    // verify(): members read this time, members that ended (then or earlier), members with
    // another set. unreadable: members (end(): new tasks on the target) whose set or start time
    // could not be read. survey(): on_target.
    std::size_t checked = 0, gone = 0, moved = 0, unreadable = 0, on_target = 0;
    std::int64_t first_bad_tid = -1;
    std::uint64_t first_bad_mask = 0;
    std::array<std::int64_t, kMaxFamily> family_tids{};
};

class SdkStreamThreadPlacement {
public:
    // target_cpu < 0: disabled, every call is a no-op. expected: the number of threads the SDK
    // creates on the caller's thread during the pipeline start.
    SdkStreamThreadPlacement(int target_cpu, std::size_t expected, SdkThreadPlacementOps ops = {}) noexcept
        : target_cpu_(target_cpu), expected_(expected), ops_(ops) {}
    SdkStreamThreadPlacement(const SdkStreamThreadPlacement&) = delete;
    SdkStreamThreadPlacement& operator=(const SdkStreamThreadPlacement&) = delete;

    bool enabled() const noexcept { return target_cpu_ >= 0; }
    // begin() narrowed (or may have narrowed) the caller and end() has not run yet.
    bool needs_end() const noexcept { return began_ && !ended_; }
    // It is not established that the thread begin() ran on has its own set: begin() narrowed
    // (or may have narrowed) it and end() has not run, or the read-back after end()'s restore
    // was not that thread's own set (or not a valid read, or not of that thread); or an earlier
    // placement of this process left that thread narrowed. Every thread it creates in this
    // state inherits the narrowed set.
    bool caller_restore_unconfirmed() const noexcept { return left_narrowed_ || (began_ && !restore_confirmed_); }
    // The family was established and the caller got its own set back.
    bool placed() const noexcept { return placed_; }
    const SdkThreadPlacementRecord& family() const noexcept { return end_; }

    // On the thread that is about to call the pipeline start, immediately before the call.
    const SdkThreadPlacementRecord& begin() noexcept {
        CaptureAffinityErrno keep_errno;
        if (!enabled() || began_ || left_narrowed_) return begin_;
        began_ = true;
        begin_.event = "begin";
        begin_.target_cpu = target_cpu_;
        begin_.expected = expected_;
        if (target_cpu_ >= 64 || expected_ == 0 || expected_ > SdkThreadPlacementRecord::kMaxFamily) {
            began_ = false;   // nothing was touched
            begin_.reason = "invalid_configuration";
            return begin_;
        }
        const std::uint64_t target = std::uint64_t{1} << target_cpu_;
        const auto original = ops_.read_self();
        begin_.caller_tid = original.tid;
        // An earlier placement could not give this thread its own set back. What it reads as
        // now is not known to be its own set, so nothing is built on it.
        if (original.tid > 0 && original.tid == sdk_thread_placement_left_narrowed().load(std::memory_order_relaxed)) {
            began_ = false;
            left_narrowed_ = true;
            begin_.original_mask = original.valid ? original.mask : 0;
            begin_.reason = "caller_left_narrowed";
            return begin_;
        }
        if (!original.valid) { began_ = false; begin_.reason = "initial_read_failed"; return begin_; }
        begin_.original_mask = original_mask_ = original.mask;
        caller_tid_ = original.tid;
        // A process that is confined to the target already: every new thread would carry the
        // target, and the family could not be told from the rest.
        if (original.mask == target) { began_ = false; begin_.reason = "already_confined_to_target"; return begin_; }
        if (!ops_.list_tasks(before_)) { began_ = false; begin_.reason = "list_tasks_failed"; return begin_; }
        std::sort(before_.begin(), before_.end());
        begin_.tasks_before = before_.size();
        // From here on end() has to run: the caller's set may have changed.
        begin_.set_errno = ops_.set_self(target);
        const auto narrowed = ops_.read_self();
        begin_.narrowed_mask = narrowed.valid ? narrowed.mask : 0;
        if (begin_.set_errno != 0) begin_.reason = "setaffinity_failed";
        else if (!narrowed.valid) begin_.reason = "readback_failed";
        else if (narrowed.tid != caller_tid_) begin_.reason = "thread_identity_changed";
        else if (narrowed.mask != target) begin_.reason = "readback_mask_mismatch";
        else { begin_.success = true; begin_.reason = "ok"; }
        return begin_;
    }

    // On the same thread, immediately after the pipeline start returned or threw.
    const SdkThreadPlacementRecord& end() noexcept {
        CaptureAffinityErrno keep_errno;
        if (!needs_end()) return end_;
        ended_ = true;
        end_.event = "end";
        end_.target_cpu = target_cpu_;
        end_.expected = expected_;
        end_.caller_tid = caller_tid_;
        end_.original_mask = original_mask_;
        end_.narrowed_mask = begin_.narrowed_mask;
        end_.tasks_before = before_.size();
        const std::uint64_t target = std::uint64_t{1} << target_cpu_;
        // List first, restore second: a thread the family creates after the call returned would
        // otherwise be counted. Restore before any judgement.
        const bool listed = ops_.list_tasks(after_);
        const auto now = ops_.read_self();
        // Restore unless this is positively another thread than the one begin() narrowed: its
        // set is never changed. The tid is read first and is there even when the rest of the
        // snapshot is not, so an unreadable set does not leave the caller narrowed.
        const bool same_thread = now.tid == caller_tid_;
        if (same_thread) end_.restore_errno = ops_.set_self(original_mask_);
        const auto restored = ops_.read_self();
        end_.restored_mask = restored.valid ? restored.mask : 0;
        // What counts is what this thread's set reads as now, not what the call returned.
        restore_confirmed_ = restored.valid && restored.tid == caller_tid_ && restored.mask == original_mask_;
        if (!restore_confirmed_) sdk_thread_placement_left_narrowed().store(caller_tid_, std::memory_order_relaxed);
        if (listed) {
            end_.tasks_after = after_.size();
            std::sort(after_.begin(), after_.end());   // the family is logged in order of tid
            for (const std::int64_t tid : after_) {
                if (std::binary_search(before_.begin(), before_.end(), tid)) continue;
                ++end_.born;
                std::uint64_t mask = 0, start = 0;
                const int code = ops_.task_mask(tid, mask);
                if (code == ESRCH) { ++end_.born_ended; continue; }   // ended between the listing and this read
                if (code != 0 || mask != target) { ++end_.born_elsewhere; continue; }
                const int start_code = ops_.task_start(tid, start);
                if (start_code == ESRCH) { ++end_.born_ended; continue; }   // ended between the two reads
                if (start_code != 0) { ++end_.unreadable; continue; }
                if (end_.family < SdkThreadPlacementRecord::kMaxFamily) {
                    family_start_[end_.family] = start;
                    end_.family_tids[end_.family] = tid;
                }
                ++end_.family;
            }
        }
        if (!same_thread) end_.reason = "restore_thread_identity_mismatch";
        else if (end_.restore_errno != 0) end_.reason = "restore_setaffinity_failed";
        else if (!restored.valid || restored.tid != caller_tid_) end_.reason = "restore_readback_failed";
        else if (restored.mask != original_mask_) end_.reason = "restore_readback_mask_mismatch";
        else if (!begin_.success) end_.reason = "begin_failed";
        else if (!listed) end_.reason = "list_tasks_failed";
        else if (end_.born_elsewhere != 0) end_.reason = "born_off_target";
        else if (end_.unreadable != 0) end_.reason = "family_read_failed";
        else if (end_.born_ended != 0) end_.reason = "born_ended";
        else if (end_.family < expected_) end_.reason = "family_too_small";
        else if (end_.family > expected_) end_.reason = "family_too_large";
        else { end_.success = true; end_.reason = "ok"; }
        placed_ = end_.success;
        before_.clear();
        after_.clear();
        return end_;
    }

    // Any thread, after end(): a few system calls per member that has not ended, no allocation.
    const SdkThreadPlacementRecord& verify() noexcept {
        CaptureAffinityErrno keep_errno;
        verify_ = SdkThreadPlacementRecord{};
        verify_.event = "verify";
        verify_.target_cpu = target_cpu_;
        verify_.expected = expected_;
        verify_.caller_tid = caller_tid_;
        verify_.family = end_.family;
        verify_.family_tids = end_.family_tids;
        if (!enabled()) return verify_;
        if (!placed_) { verify_.reason = "not_placed"; return verify_; }
        const std::uint64_t target = std::uint64_t{1} << target_cpu_;
        const auto bad = [this](std::int64_t tid, std::uint64_t mask) {
            if (verify_.first_bad_tid < 0) { verify_.first_bad_tid = tid; verify_.first_bad_mask = mask; }
        };
        for (std::size_t i = 0; i < end_.family; ++i) {   // placed: the family is `expected`, all held
            const std::int64_t tid = end_.family_tids[i];
            if (!gone_[i]) {
                ++verify_.checked;
                std::uint64_t mask = 0, start = 0;
                const int code = ops_.task_mask(tid, mask);
                const int start_code = code == ESRCH ? ESRCH : ops_.task_start(tid, start);
                // No such task any more, or the tid is another task's now (whatever can be read
                // of that one). Either way the member has ended, and it stays ended.
                if (code == ESRCH || start_code == ESRCH || (start_code == 0 && start != family_start_[i])) {
                    gone_[i] = true;
                } else if (code != 0 || start_code != 0) {
                    ++verify_.unreadable;
                    bad(tid, mask);
                } else if (mask != target) {
                    ++verify_.moved;
                    bad(tid, mask);
                }
            }
            if (gone_[i]) { ++verify_.gone; bad(tid, 0); }
        }
        if (verify_.gone != 0) verify_.reason = "family_thread_gone";
        else if (verify_.moved != 0) verify_.reason = "family_mask_changed";
        else if (verify_.unreadable != 0) verify_.reason = "family_read_failed";
        else { verify_.success = true; verify_.reason = "ok"; }
        return verify_;
    }

    // Any thread, after end(); lists the tasks (allocates): how many have exactly the target.
    const SdkThreadPlacementRecord& survey() noexcept {
        CaptureAffinityErrno keep_errno;
        survey_ = SdkThreadPlacementRecord{};
        survey_.event = "survey";
        survey_.target_cpu = target_cpu_;
        survey_.expected = expected_;
        survey_.caller_tid = caller_tid_;
        survey_.family = end_.family;
        survey_.family_tids = end_.family_tids;
        if (!enabled()) return survey_;
        const std::uint64_t target = std::uint64_t{1} << target_cpu_;
        if (!ops_.list_tasks(after_)) { survey_.reason = "list_tasks_failed"; return survey_; }
        survey_.tasks_after = after_.size();
        for (const std::int64_t tid : after_) {
            std::uint64_t mask = 0;
            if (ops_.task_mask(tid, mask) == 0 && mask == target) ++survey_.on_target;
        }
        after_.clear();
        survey_.success = true;
        survey_.reason = "ok";
        return survey_;
    }

private:
    int target_cpu_;
    std::size_t expected_;
    SdkThreadPlacementOps ops_;
    bool began_ = false, ended_ = false, placed_ = false, restore_confirmed_ = false, left_narrowed_ = false;
    std::int64_t caller_tid_ = -1;
    std::uint64_t original_mask_ = 0;
    std::vector<std::int64_t> before_, after_;
    std::array<std::uint64_t, SdkThreadPlacementRecord::kMaxFamily> family_start_{};
    std::array<bool, SdkThreadPlacementRecord::kMaxFamily> gone_{};
    SdkThreadPlacementRecord begin_, end_, verify_, survey_;
};

inline std::ostream& operator<<(std::ostream& out, const SdkThreadPlacementRecord& r) {
    out << "SDK_THREAD_PLACEMENT schema=2 mask_encoding=cpu_bitset_u64 event=" << r.event
        << " target_cpu=" << r.target_cpu << " success=" << static_cast<int>(r.success) << " reason=" << r.reason
        << " caller_tid=" << r.caller_tid << " original_mask=" << r.original_mask
        << " narrowed_mask=" << r.narrowed_mask << " restored_mask=" << r.restored_mask
        << " set_errno=" << r.set_errno << " restore_errno=" << r.restore_errno
        << " tasks_before=" << r.tasks_before << " tasks_after=" << r.tasks_after << " born=" << r.born
        << " born_ended=" << r.born_ended << " born_elsewhere=" << r.born_elsewhere
        << " family=" << r.family << " expected=" << r.expected
        << " checked=" << r.checked << " gone=" << r.gone << " moved=" << r.moved << " unreadable=" << r.unreadable
        << " on_target=" << r.on_target
        << " first_bad_tid=" << r.first_bad_tid << " first_bad_mask=" << r.first_bad_mask << " family_tids=";
    const std::size_t shown = std::min(r.family, r.family_tids.size());
    for (std::size_t i = 0; i < shown; ++i) out << (i ? "," : "") << r.family_tids[i];
    if (shown == 0) out << "none";
    return out;
}

// One record to a sink that may throw; false when it did.
template<class Sink> bool emit_sdk_thread_placement(const SdkThreadPlacementRecord& record, Sink&& sink) noexcept {
    CaptureAffinityErrno keep_errno;
    try { std::forward<Sink>(sink)(record); return true; }
    catch (...) { return false; }
}

// Around the pipeline start: end() runs when the scope is left, also by an exception, and its
// record goes to the sink. A null placement (switch off) does nothing.
template<class Sink> class SdkThreadPlacementEndGuard {
public:
    SdkThreadPlacementEndGuard(SdkStreamThreadPlacement* placement, Sink sink)
        : placement_(placement), sink_(std::move(sink)) {}
    ~SdkThreadPlacementEndGuard() noexcept {
        if (placement_ && placement_->needs_end()) (void)emit_sdk_thread_placement(placement_->end(), sink_);
    }
    SdkThreadPlacementEndGuard(const SdkThreadPlacementEndGuard&) = delete;
    SdkThreadPlacementEndGuard& operator=(const SdkThreadPlacementEndGuard&) = delete;
private:
    SdkStreamThreadPlacement* placement_;
    Sink sink_;
};

// The periodic check of the capture loop. tick() is called on every turn of the loop, frame or
// no frame: once a period it verifies the family, publishes Placed or Fault for Status and
// hands the record to the sink when the answer, its reason or the number of ended members is
// new; once, after the survey delay, it also counts the tasks on the target. A null placement
// (switch off) does nothing.
class SdkThreadPlacementWatch {
public:
    static constexpr std::int64_t kVerifyPeriodNs = 1000000000LL, kSurveyAfterNs = 5000000000LL;
    explicit SdkThreadPlacementWatch(SdkStreamThreadPlacement* placement) noexcept : placement_(placement) {}
    SdkThreadPlacementWatch(const SdkThreadPlacementWatch&) = delete;
    SdkThreadPlacementWatch& operator=(const SdkThreadPlacementWatch&) = delete;

    template<class Sink> void tick(std::int64_t now_ns, Sink&& sink) noexcept {
        if (!placement_ || !placement_->enabled()) return;
        if (started_ && now_ns - last_ns_ < kVerifyPeriodNs) return;
        if (!started_) first_ns_ = now_ns;
        started_ = true;
        last_ns_ = now_ns;
        const auto& record = placement_->verify();
        publish_sdk_thread_placement(record.success ? SdkThreadPlacementState::Placed : SdkThreadPlacementState::Fault);
        if (!reported_ || record.success != last_success_ || std::strcmp(record.reason, last_reason_) != 0 ||
            record.gone != last_gone_)
            (void)emit_sdk_thread_placement(record, sink);
        reported_ = true;
        last_success_ = record.success;
        last_reason_ = record.reason;
        last_gone_ = record.gone;
        if (!surveyed_ && now_ns - first_ns_ >= kSurveyAfterNs) {
            surveyed_ = true;
            (void)emit_sdk_thread_placement(placement_->survey(), sink);
        }
    }

private:
    SdkStreamThreadPlacement* placement_;
    bool started_ = false, reported_ = false, surveyed_ = false, last_success_ = false;
    const char* last_reason_ = "";
    std::size_t last_gone_ = 0;
    std::int64_t first_ns_ = 0, last_ns_ = 0;
};

} // namespace perception
