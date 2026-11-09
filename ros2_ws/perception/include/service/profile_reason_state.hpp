/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: profile_reason_state.hpp
 * Brief: Frame-level Profile reasons handed from the Profile owner to the Status owner (reply B3-3).
 *
 * Description:
 * They are kept as three separate ledgers:
 *   1. current state = the complete reason set of the most recently PROCESSED
 *      Profile frame. A normal frame commits 0 (explicit recovery evidence); a
 *      failed frame commits its bits. No new frame changes nothing: a Status
 *      heartbeat, a failed heartbeat or a stalled stream is never a recovery.
 *   2. window statistics = cumulative per-reason frame counts (and processed
 *      frame count); the Status owner takes deltas per heartbeat for internal
 *      diagnostics. Taking a window never touches the current state.
 *   3. edges = produced by the Status owner's ReasonEdges from actual changes
 *      of the active set that includes (1).
 * The wire aggregation meaning (latest processed state vs window OR) is still
 * an implementation proposal pending the upstream ruling.
 * Single producer (Profile owner), any number of readers; lock-free.
 */
#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace perception::service {

class ProfileReasonState {
public:
    struct Snapshot {
        std::uint64_t current = 0;           // reasons of the latest processed frame
        std::uint64_t processed_frames = 0;  // cumulative
        std::array<std::uint64_t, 64> frames_with{};  // cumulative frames carrying bit i
    };

    explicit ProfileReasonState(std::uint64_t tracked_bits) noexcept : tracked_(tracked_bits) {}

    // Profile owner only: one call per processed frame with its COMPLETE
    // frame-level reason set. Bits outside the tracked set are refused.
    bool commit_frame(std::uint64_t bits) noexcept {
        if (bits & ~tracked_) return false;
        for (std::size_t i = 0; i < 64; ++i)
            if (bits & (std::uint64_t{1} << i)) frames_with_[i].fetch_add(1, std::memory_order_relaxed);
        current_.store(bits, std::memory_order_release);
        processed_.fetch_add(1, std::memory_order_release);
        return true;
    }

    std::uint64_t current() const noexcept { return current_.load(std::memory_order_acquire); }

    Snapshot snapshot() const noexcept {
        Snapshot s;
        s.processed_frames = processed_.load(std::memory_order_acquire);
        s.current = current_.load(std::memory_order_acquire);
        for (std::size_t i = 0; i < 64; ++i) s.frames_with[i] = frames_with_[i].load(std::memory_order_relaxed);
        return s;
    }

private:
    const std::uint64_t tracked_;
    std::atomic<std::uint64_t> current_{0}, processed_{0};
    std::array<std::atomic<std::uint64_t>, 64> frames_with_{};
};

}  // namespace perception::service
