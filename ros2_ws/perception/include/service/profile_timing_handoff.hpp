/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: profile_timing_handoff.hpp
 * Brief: The Profile thread's timing figures, handed to another thread that formats and logs them.
 *
 * Description:
 * What problem this solves. Every 10 s the Profile thread used to format and log its timing lines
 * itself (ProfileTiming::log), and the logger allocates for every line: heap calls on the fast
 * line in steady state (19 P19-3, A19-ALLOC-1). Now the Profile thread only freezes the figures
 * those lines print into a preallocated slot (offer: a fixed copy of about 1.5 KB; the quantiles
 * are computed there, as before) and another thread turns waiting slots into lines
 * (take_lines: the Status owner, every 10 ms). Formatting and logging move off the fast line; the
 * quantile scans do not, so this says nothing about jitter or rate.
 * Rules. Single producer (the Profile thread), one consumer at a time. offer() never blocks,
 * allocates or logs. When all kSlots slots still wait to be taken, the new observation is dropped
 * and counted; the figures are cumulative, so the next one carries everything it had, and its
 * PROFILE_TIMING_OBSERVED line shows the gap (seq) and the count (dropped). The consumer never
 * waits for the producer, and reads only frozen slots, never the live ledger; it logs what it took
 * before it lets go of the consumer side. close(), on the Profile thread after its loop, takes
 * over: it waits for a take in progress (and its logging), appends what is still waiting and then
 * the stop group, all from the Profile thread, so the stop group is logged last and once, even
 * when the consumer thread has already ended.
 * Worked example. A run of 35 s with Status draining: the Status thread logs three groups
 * (seq 1, 2, 3, when=periodic, each about 10 ms after its observation), then at stop the Profile
 * thread logs seq 4 when=stop. With Status gone after 12 s: seq 1 logged by Status, seq 2 and 3
 * wait, and at stop the Profile thread logs 2, 3 and then 4 when=stop.
 */
#pragma once

#include "service/profile_timing.hpp"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <time.h>
#include <vector>

namespace perception::service {

class ProfileTimingHandoff {
public:
    // 40 s of observations may wait for a stalled consumer before one is dropped.
    static constexpr std::size_t kSlots = 4;

    ProfileTimingHandoff() = default;
    ProfileTimingHandoff(const ProfileTimingHandoff&) = delete;
    ProfileTimingHandoff& operator=(const ProfileTimingHandoff&) = delete;

    // Profile thread, at its 10 s mark: freezes `timing` into the oldest free slot. False, and
    // the observation dropped (counted), when every slot still waits. Never blocks, allocates or logs.
    bool offer(const ProfileTiming& timing, ProfileTimingOwner owner) noexcept {
        ++seq_;
        const std::uint64_t tail = tail_.load(std::memory_order_relaxed);
        // acquire: the consumer has finished reading a slot before it is written again.
        if (tail - head_.load(std::memory_order_acquire) >= kSlots) {
            dropped_.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
        ProfileTimingFigures& f = slots_[tail % kSlots];
        observe(f, ProfileTimingWhen::Periodic, owner);
        freeze_profile_timing(timing, f);
        tail_.store(tail + 1, std::memory_order_release);
        return true;
    }

    // The consumer (the Status owner): replaces `lines` with the lines of every waiting group, oldest
    // first, frees their slots, and hands each line to emit(const std::string&) (the logger) before
    // letting go of the consumer side, so close(), which waits for it, logs the stop group after
    // them. Returns how many groups. Never waits: 0 at once while close() runs or after it. Each
    // group is the PROFILE_TIMING_OBSERVED line, then the timing lines.
    template <typename Emit>
    std::size_t take_lines(std::vector<std::string>& lines, Emit&& emit) {
        lines.clear();
        std::unique_lock<std::mutex> lock(consumer_, std::try_to_lock);
        if (!lock.owns_lock() || closed_) return 0;
        const std::size_t taken = take_waiting(lines);
        for (const auto& line : lines) emit(line);
        return taken;
    }

    // Profile thread, once, after its loop: waits for a take in progress, appends the groups still
    // waiting, then the group of `timing` itself with when=stop. take_lines() takes nothing more.
    // The caller logs the lines.
    void close(const ProfileTiming& timing, ProfileTimingOwner owner, std::vector<std::string>& lines) {
        std::lock_guard<std::mutex> lock(consumer_);
        closed_ = true;
        (void)take_waiting(lines);
        ++seq_;
        ProfileTimingFigures last;
        observe(last, ProfileTimingWhen::Stop, owner);
        freeze_profile_timing(timing, last);
        append_group(last, lines);
    }

    // Observations dropped so far (any thread; exact once the Profile thread has stopped).
    std::uint64_t dropped() const noexcept { return dropped_.load(std::memory_order_relaxed); }

private:
    static std::int64_t monotonic_ns() noexcept {
        timespec ts{};
        if (::clock_gettime(CLOCK_MONOTONIC, &ts) != 0) return -1;
        return static_cast<std::int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
    }
    // The observation fields, on the Profile thread.
    void observe(ProfileTimingFigures& f, ProfileTimingWhen when, ProfileTimingOwner owner) noexcept {
        f.seq = seq_;
        f.dropped = dropped_.load(std::memory_order_relaxed);
        f.observed_mono_ns = monotonic_ns();
        f.owner_thread = std::this_thread::get_id();
        f.when = when;
        f.owner = owner;
    }
    static void append_group(const ProfileTimingFigures& f, std::vector<std::string>& lines) {
        lines.push_back(profile_timing_observed_line(f));
        append_profile_timing_lines(f, lines);
    }
    // Under consumer_. A slot is freed only after its lines are built: a throw leaves it waiting.
    std::size_t take_waiting(std::vector<std::string>& lines) {
        std::uint64_t head = head_.load(std::memory_order_relaxed);
        // acquire: the producer's writes of a slot are complete once tail_ shows it.
        const std::uint64_t tail = tail_.load(std::memory_order_acquire);
        std::size_t taken = 0;
        for (; head != tail; ++taken) {
            append_group(slots_[head % kSlots], lines);
            head_.store(++head, std::memory_order_release);
        }
        return taken;
    }

    std::array<ProfileTimingFigures, kSlots> slots_{};
    // Counts of slots written (producer) and taken (consumer); tail - head are waiting.
    std::atomic<std::uint64_t> head_{0}, tail_{0};
    std::atomic<std::uint64_t> dropped_{0};
    std::uint64_t seq_ = 0;  // Profile thread only
    std::mutex consumer_;    // one consumer at a time: take_lines (try) or close (waits)
    bool closed_ = false;    // under consumer_
};

}  // namespace perception::service
