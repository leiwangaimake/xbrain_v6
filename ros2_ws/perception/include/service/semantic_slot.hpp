/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: semantic_slot.hpp
 * Brief: the latest-wins semantic slot of 19 section 3.4A.
 *
 * Description:
 * The inference thread publishes the measured base_link footprints of one
 * Objects frame; the Profile thread copies the newest snapshot into its own
 * storage and offers it to the SemanticProjector. Latest wins: an older
 * snapshot is overwritten, never queued, and the Profile thread never waits for
 * inference. Both sides are bounded and allocation-free in steady state; the
 * lock is held only for a fixed-size copy.
 *
 * This slot carries measurements, not decisions: it neither ages nor inflates
 * the footprints (the projector does that), and it never invents a footprint
 * for an object the Objects path could not locate.
 * No SDK, OpenCV or device access.
 */

#pragma once

#include "output/rns_contract.hpp"
#include "output/rns_semantic.hpp"

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>

namespace perception::service {

// The calibration identity both sides of the slot must agree on. The service
// resolves one base <- camera transform at startup and never changes it while
// running, so one constant is the whole identity here; it is NOT a calibration
// certificate (that stays with CalibrationVerdict / extrinsic_calibrated).
inline constexpr std::uint64_t kSemanticBaseRevision = 1;

class SemanticSlot {
public:
    // Bounded by the projector's own input limit (512) and by what one frame of
    // this detector can plausibly carry; the overflow is counted, not dropped
    // silently. A class name longer than the buffer is skipped, never truncated
    // (a truncated name could gain or lose the traversable_area identity).
    static constexpr std::size_t kMaxObjects = 64;
    static constexpr std::size_t kMaxNameBytes = 48;

    struct Objects {
        // The stored objects carry an EMPTY class_name: a view into the names
        // buffer would dangle across the copy. latest() rebinds each view to
        // the reader's own copy using name_len.
        std::array<rns::SemanticObject, kMaxObjects> objects{};
        std::array<std::array<char, kMaxNameBytes>, kMaxObjects> names{};
        std::array<std::uint8_t, kMaxObjects> name_len{};
        std::size_t count = 0;
        geometry::CaptureStamp stamp;
        std::uint64_t base_revision = 0;
        bool valid = false;
    };
    // Consumer-side storage. One per Profile thread; the returned SemanticFrame
    // borrows it and stays valid until the next call on the same storage.
    struct Snapshot {
        Objects data;
        rns::SemanticFrame frame;
    };

    // Inference thread. Objects without measured geometry are skipped: an
    // unlocatable object has no footprint to inject.
    void publish(const rns::Objects& body, const geometry::CaptureStamp& stamp,
                 std::uint64_t base_revision) noexcept;
    // Profile thread. Returns nullptr when there is nothing this depth frame may
    // use. The snapshot is REFUSED here, not offered and rejected downstream:
    // the projector treats a future, foreign-stream or foreign-epoch snapshot as
    // a malformed input and the whole frame would then go unpublished, while an
    // absent snapshot simply leaves the profile geometry-only. A newer inference
    // frame than the depth frame being processed is an ordinary race (inference
    // and Profile consume the same stream independently), not a fault.
    const rns::SemanticFrame* latest(Snapshot& into, const geometry::CaptureStamp& depth) noexcept;

    std::uint64_t published() const noexcept { return published_.load(std::memory_order_relaxed); }
    std::uint64_t objects_published() const noexcept { return objects_published_.load(std::memory_order_relaxed); }
    std::uint64_t skipped_unlocatable() const noexcept { return skipped_unlocatable_.load(std::memory_order_relaxed); }
    std::uint64_t skipped_overflow() const noexcept { return skipped_overflow_.load(std::memory_order_relaxed); }
    std::uint64_t skipped_name() const noexcept { return skipped_name_.load(std::memory_order_relaxed); }
    std::uint64_t skipped_vertices() const noexcept { return skipped_vertices_.load(std::memory_order_relaxed); }
    std::uint64_t taken() const noexcept { return taken_.load(std::memory_order_relaxed); }
    std::uint64_t refused_future() const noexcept { return refused_future_.load(std::memory_order_relaxed); }
    std::uint64_t refused_stream() const noexcept { return refused_stream_.load(std::memory_order_relaxed); }
    std::uint64_t refused_epoch() const noexcept { return refused_epoch_.load(std::memory_order_relaxed); }

private:
    mutable std::mutex mutex_;
    Objects slot_;  // guarded by mutex_
    std::atomic<std::uint64_t> published_{0}, objects_published_{0}, skipped_unlocatable_{0},
        skipped_overflow_{0}, skipped_name_{0}, skipped_vertices_{0}, taken_{0},
        refused_future_{0}, refused_stream_{0}, refused_epoch_{0};
};

}  // namespace perception::service
