/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: t_mask_slot.hpp
 * Brief: Latest-wins handoff of the raw traversable mask (T) from the Objects side to the Profile thread.
 *
 * Description:
 * 19 section 3.1, P19-1/P19-2.
 * What it carries. The class-92 union of one inference frame, as a 1280x800 binary raster (0 or 1),
 * with the capture stamp of that frame and the projection revision it was made for. The producer
 * (the Objects thread, one per process) binarises while copying: any nonzero byte becomes 1, which
 * is the 0/255 -> 0/1 conversion NormalTraversability requires (it rejects any byte above 1).
 *
 * How it hands over (P19-2 literally). Two preallocated buffers, a publication counter and one
 * sequence number per buffer. The writer fills the buffer the reader is not directed to (sequence
 * odd while writing, even when done) and then advances the publication counter, which is the
 * pointer swap. The reader copies the newest buffer into its own storage and re-reads that buffer's
 * sequence; if it changed, the copy may be torn and is retried a bounded number of times, then
 * given up (the frame is then without T, never waiting). All shared words are atomics, so the
 * concurrent copy is not a data race in the C++ sense.
 *
 * What the reader refuses. A mask from another stream or clock epoch, for another projection
 * revision, captured after the depth frame, or not older-or-same by the owner's own identity rule
 * (source_older_or_same) is refused here and counted, so the frame is published as "no T" instead
 * of the owner calling it InvalidIdentity/Future (which would drop the whole Profile frame).
 * Staleness is NOT judged here: the owner reports Stale against seg_max_age_ms, one source of truth.
 *
 * Bounded, allocation-free after construction (the reader's Snapshot allocates once). No SDK,
 * OpenCV or device access.
 */

#pragma once

#include "modules/traversability.hpp"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace perception::service {

// The projection identity both sides of the slot must agree on. The service builds its colour
// projection once per process from the session calibration and never changes it while running,
// so one constant is the whole identity (same reasoning as kSemanticBaseRevision). It is not a
// calibration certificate.
inline constexpr std::uint64_t kTProjectionRevision = 1;

class TMaskSlot {
public:
    static constexpr std::uint32_t kWidth = 1280, kHeight = 800;
    static constexpr std::size_t kBytes = std::size_t{kWidth} * kHeight;
    static constexpr std::size_t kWords = kBytes / 8;
    static constexpr int kCopyAttempts = 4;

    // Consumer-side storage, one per Profile thread. Allocates its raster once. The returned
    // RawTMask borrows it and stays valid (and unchanged) until the next latest() on it.
    struct Snapshot {
        Snapshot();
        std::vector<std::uint8_t> pixels;
        geometry::RawTMask mask;
        std::uint64_t publication = 0;  // publication held in `pixels` (0: none)
        bool empty = false;             // published by publish_empty()
    };

    TMaskSlot();
    TMaskSlot(const TMaskSlot&) = delete;
    TMaskSlot& operator=(const TMaskSlot&) = delete;

    // Producer (one thread). Copies a kWidth x kHeight single-byte raster (row stride >= width),
    // any nonzero byte becoming 1. False, and nothing published, for another size, a null raster
    // or a second concurrent writer (all counted); the previous mask stays.
    bool publish(const std::uint8_t* pixels, std::uint32_t width, std::uint32_t height,
                 std::size_t stride_bytes, const geometry::CaptureStamp& stamp,
                 std::uint64_t projection_revision) noexcept;
    // Producer: an inference frame that completed without any class-92 detection. Publishes an
    // all-zero mask ("looked, nothing traversable"), counted separately. Whether this counts as a
    // valid update for traversable_seg_available is the Status side's choice (both clocks below).
    bool publish_empty(const geometry::CaptureStamp& stamp, std::uint64_t projection_revision) noexcept;

    // Profile thread. The newest mask this depth frame may use, or nullptr (then: no T this frame).
    // Never waits. Reuses the copy already in `into` when nothing newer was published.
    const geometry::RawTMask* latest(Snapshot& into, const geometry::CaptureStamp& depth,
                                     std::uint64_t projection_revision) noexcept;

    // CLOCK_MONOTONIC nanoseconds (Linux) of the last publication, -1 before the first;
    // and of the last one that was not publish_empty().
    std::int64_t last_publish_mono_ns() const noexcept { return last_publish_ns_.load(std::memory_order_acquire); }
    std::int64_t last_nonempty_publish_mono_ns() const noexcept { return last_nonempty_ns_.load(std::memory_order_acquire); }

    struct Counters {
        std::uint64_t published = 0, published_empty = 0, refused_input = 0, refused_concurrent = 0,
            taken = 0, reused = 0, missing = 0, refused_stream = 0, refused_epoch = 0,
            refused_revision = 0, refused_future = 0, refused_order = 0, torn_retries = 0, torn_giveup = 0;
    };
    Counters counters() const noexcept;

#ifdef T_MASK_SLOT_TEST_HOOK
    // Test builds only (never defined in the product): called by latest() once per copy attempt,
    // halfway through the raster copy, so a test can publish at exactly that moment.
    void (*test_mid_copy_hook)(void*) = nullptr;
    void* test_hook_context = nullptr;
#endif

private:
    // Header words: publication number, empty flag, projection revision, then the capture stamp.
    static constexpr std::size_t kStampWords = (sizeof(geometry::CaptureStamp) + 7) / 8;
    static constexpr std::size_t kHeaderWords = 3 + kStampWords;
    struct Buffer {
        std::atomic<std::uint64_t> sequence{0};  // odd while being written
        std::array<std::atomic<std::uint64_t>, kHeaderWords> header{};
        std::unique_ptr<std::atomic<std::uint64_t>[]> words;
    };
    template <typename Fill>
    bool write(const geometry::CaptureStamp& stamp, std::uint64_t projection_revision, bool empty,
               Fill fill) noexcept;

    std::array<Buffer, 2> buffers_;
    std::atomic<std::uint64_t> publications_{0};  // buffer of publication n is n & 1
    std::atomic<bool> writing_{false};
    std::atomic<std::int64_t> last_publish_ns_{-1}, last_nonempty_ns_{-1};
    std::atomic<std::uint64_t> published_{0}, published_empty_{0}, refused_input_{0},
        refused_concurrent_{0}, taken_{0}, reused_{0}, missing_{0}, refused_stream_{0},
        refused_epoch_{0}, refused_revision_{0}, refused_future_{0}, refused_order_{0},
        torn_retries_{0}, torn_giveup_{0};
};

}  // namespace perception::service
