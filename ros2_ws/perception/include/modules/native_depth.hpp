/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: native_depth.hpp
 * Brief: SDK-independent, bounded ownership transfer for pre-D2C depth frames.
 *
 * Description:
 * What problem this solves. The capture thread receives native (not colour-aligned) depth
 * frames at 30 fps; the Profile thread (the fast line of 19 section 1.1) must always work on the
 * newest one, and the capture must never wait for it. Mailbox is a single
 * latest-wins slot between the two: the producer copies a frame in (replacing an untaken one),
 * the consumer swaps it out into its own buffer. Neither side ever blocks on the other.
 *
 * Stream generations. Every camera (re)start begins a new stream id; frames and identities of
 * a previous stream are refused, so nothing from before a restart can be taken after it.
 */
#pragma once

#include "common/rgbd_source_identity.hpp"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <vector>

namespace perception::native_depth {

// A storage limit, not a requested or accepted camera stream configuration.
inline constexpr std::size_t kMaxPixels = 1280U * 800U;

// Native depth intrinsics as reported by the SDK for this stream.
struct Intrinsics {
    float fx = 0.0F;
    float fy = 0.0F;
    float cx = 0.0F;
    float cy = 0.0F;
    std::uint32_t width = 0;
    std::uint32_t height = 0;
};

struct Distortion {
    bool reported = false;
    // Raw SDK model value, including unknown models. This is provenance only:
    // accepting it does not claim the raster is undistorted or deprojectable.
    int model = 0;
    // k1, k2, k3, k4, k5, k6, p1, p2.
    std::array<float, 8> coefficients{};
};

// Everything the geometry needs to interpret one frame's pixels.
struct Metadata {
    SourceId source;
    Intrinsics intrinsics;
    Distortion distortion;
    float unit_m = 0.0F;
};

// The producer's borrowed view of one SDK frame, copied by try_offer.
struct View {
    Metadata metadata;
    // Caller keeps this tightly packed, native-endian uint16 raster readable
    // for the entire try_offer call. No SDK owner or pointer is retained.
    const void* pixels = nullptr;
    std::size_t bytes = 0;
    bool pre_d2c = false;
};

class Mailbox;

// A consumer-owned depth buffer (fixed capacity; ownership moves by swap).
class Frame {
public:
    // Allocates once. Capacity must be in [1, kMaxPixels].
    explicit Frame(std::size_t max_pixels);
    Frame(const Frame&) = delete;
    Frame& operator=(const Frame&) = delete;
    Frame(Frame&&) = delete;
    Frame& operator=(Frame&&) = delete;

    const Metadata& metadata() const noexcept { return metadata_; }
    const std::uint16_t* pixels() const noexcept { return pixels_.data(); }
    std::size_t pixel_count() const noexcept { return pixel_count_; }
    std::size_t capacity_pixels() const noexcept { return pixels_.size(); }

private:
    friend class Mailbox;
    Metadata metadata_{};
    std::vector<std::uint16_t> pixels_;
    std::size_t pixel_count_ = 0;
};

// Offer: Replaced = accepted and an untaken frame was dropped (latest wins);
// Busy = the other side held the lock, try again with the next frame.
enum class Offer { Accepted, Replaced, Busy, Invalid, Stale, Inactive };
enum class Take { Taken, Empty, Busy, Capacity, Inactive };

struct Counters {
    std::uint64_t offered = 0;
    // All successful offers; replaced is a subset of accepted.
    std::uint64_t accepted = 0;
    std::uint64_t replaced = 0;
    std::uint64_t taken = 0;
    // Busy and inactive include both offer and take calls.
    std::uint64_t busy = 0;
    std::uint64_t invalid = 0;
    std::uint64_t stale = 0;
    std::uint64_t inactive = 0;
    std::uint64_t discarded_on_reset = 0;
};

class Mailbox {
public:
    // One pending buffer. Every consumer Frame owns another fixed buffer.
    explicit Mailbox(std::size_t max_pixels);
    Mailbox(const Mailbox&) = delete;
    Mailbox& operator=(const Mailbox&) = delete;

    // Lifecycle calls may wait for a bounded copy/swap; serialize lifecycle
    // calls externally. Counters are lifetime totals, not reset per stream.
    // begin_stream throws on exhausted uint64 generation rather than reuse IDs.
    std::uint64_t begin_stream();
    void end_stream() noexcept;
    std::uint64_t active_stream() const noexcept;

    // Single producer and single consumer. Both hot paths use try_lock, never
    // wait for consumer processing, and never allocate. Accepted sequence,
    // device timestamps and indices strictly increase; first index 0 is valid.
    // The host clock is nonnegative and nondecreasing. Gaps are allowed.
    // Pixel value 0 is preserved; pixel validity belongs to downstream policy.
    Offer try_offer(const View& view) noexcept;

    // Requires exactly matching configured capacity. Failure leaves out intact.
    // Success swaps ownership; a held frame stays intact while offers continue.
    Take try_take(Frame& out) noexcept;

    // Optional consumer doorbell: returns once the lifetime accepted-offer
    // count exceeds `seen` or the timeout expires. The producer only takes a
    // momentary uncontended lock to notify; it never waits for processing.
    std::uint64_t accepted_count() const noexcept { return counts_.accepted.load(std::memory_order_acquire); }
    bool wait_for_accepted_after(std::uint64_t seen, std::chrono::milliseconds timeout) noexcept;

    // Relaxed atomic snapshot: individual values are valid but not coherent
    // across counters during concurrent activity. Read after quiescence to audit.
    Counters counters() const noexcept;

private:
    // Test-only access permits deterministic contention without timed sleeps.
    friend struct MailboxTestAccess;
    bool valid(const View& view, std::size_t& pixel_count) const noexcept;
    bool newer(const SourceId& source) const noexcept;
    void discard_pending() noexcept;

    // mutex_ guards the pending frame and identity; the doorbell pair only wakes
    // a waiting consumer and is never held while copying.
    mutable std::mutex mutex_;
    std::mutex doorbell_mutex_;
    std::condition_variable doorbell_cv_;
    Frame pending_;
    bool has_pending_ = false;
    bool has_accepted_ = false;
    SourceId last_accepted_{};
    std::uint64_t generation_ = 0;
    std::atomic<std::uint64_t> active_stream_{0};

    struct AtomicCounters {
        std::atomic<std::uint64_t> offered{0};
        std::atomic<std::uint64_t> accepted{0};
        std::atomic<std::uint64_t> replaced{0};
        std::atomic<std::uint64_t> taken{0};
        std::atomic<std::uint64_t> busy{0};
        std::atomic<std::uint64_t> invalid{0};
        std::atomic<std::uint64_t> stale{0};
        std::atomic<std::uint64_t> inactive{0};
        std::atomic<std::uint64_t> discarded_on_reset{0};
    } counts_;
};

}  // namespace perception::native_depth
