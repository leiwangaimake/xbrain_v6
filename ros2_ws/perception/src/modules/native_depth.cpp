/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: native_depth.cpp
 * Brief: Mailbox and Frame: the latest-wins depth handoff between capture and Profile.
 *
 * Description:
 * What problem this solves. Implements native_depth.hpp. Both hot paths use try_lock: a busy
 * mailbox costs the producer one frame (the next one replaces it anyway) and the consumer one
 * poll, but never a wait. Taking swaps buffers instead of copying, so the consumer gets the
 * frame without a second 0.5 MB copy and the producer's next offer goes into the old buffer.
 */
#include "modules/native_depth.hpp"

#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <utility>

namespace perception::native_depth {
namespace {

std::size_t checked_capacity(std::size_t capacity) {
    if (capacity == 0 || capacity > kMaxPixels) {
        throw std::invalid_argument("native depth capacity must be in [1, kMaxPixels]");
    }
    return capacity;
}

constexpr auto relaxed = std::memory_order_relaxed;

}  // namespace

Frame::Frame(std::size_t max_pixels) : pixels_(checked_capacity(max_pixels)) {}

Mailbox::Mailbox(std::size_t max_pixels) : pending_(max_pixels) {}

// New camera session: drop anything pending and forget the old identity, then
// publish the new stream id (release, so readers see the cleared state first).
std::uint64_t Mailbox::begin_stream() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (generation_ == std::numeric_limits<std::uint64_t>::max()) {
        throw std::overflow_error("native depth stream generation exhausted");
    }
    discard_pending();
    has_accepted_ = false;
    last_accepted_ = {};
    const auto stream = ++generation_;
    active_stream_.store(stream, std::memory_order_release);
    return stream;
}

void Mailbox::end_stream() noexcept {
    // Invalidate even if the platform mutex reports an exceptional failure.
    active_stream_.store(0, std::memory_order_release);
    try {
        std::lock_guard<std::mutex> lock(mutex_);
        discard_pending();
        has_accepted_ = false;
        last_accepted_ = {};
    } catch (...) {
        // A failed platform lock must not escape capture shutdown. No pending
        // data can be taken while inactive; the next begin_stream clears it.
    }
}

std::uint64_t Mailbox::active_stream() const noexcept {
    return active_stream_.load(std::memory_order_acquire);
}

// Caller holds mutex_. Counts the frame lost to a stream reset.
void Mailbox::discard_pending() noexcept {
    if (has_pending_) {
        counts_.discarded_on_reset.fetch_add(1, relaxed);
    }
    has_pending_ = false;
    pending_.pixel_count_ = 0;
    pending_.metadata_ = {};
}

// A usable native frame: pre-D2C, fits the buffer, byte size matches the
// raster, finite intrinsics/unit/distortion and a filled identity.
bool Mailbox::valid(const View& view, std::size_t& pixel_count) const noexcept {
    const auto& intr = view.metadata.intrinsics;
    const auto& source = view.metadata.source;
    if (!view.pre_d2c || view.pixels == nullptr || intr.width == 0 ||
        intr.height == 0 || intr.width > pending_.capacity_pixels() / intr.height) {
        return false;
    }
    pixel_count = static_cast<std::size_t>(intr.width) * intr.height;
    if (view.bytes != pixel_count * sizeof(std::uint16_t) ||
        !std::isfinite(intr.fx) || intr.fx <= 0.0F ||
        !std::isfinite(intr.fy) || intr.fy <= 0.0F ||
        !std::isfinite(intr.cx) || !std::isfinite(intr.cy) ||
        !std::isfinite(view.metadata.unit_m) || view.metadata.unit_m <= 0.0F ||
        !view.metadata.distortion.reported || source.sequence == 0 ||
        source.color_device_us == 0 || source.depth_device_us == 0 ||
        source.host_ready_mono_ns < 0) {
        return false;
    }
    for (const auto coefficient : view.metadata.distortion.coefficients) {
        if (!std::isfinite(coefficient)) {
            return false;
        }
    }
    return true;
}

// Every identity field must move forward (host time may repeat): a replayed or
// reordered frame is refused as Stale.
bool Mailbox::newer(const SourceId& source) const noexcept {
    return !has_accepted_ ||
           (source.sequence > last_accepted_.sequence &&
            source.color_index > last_accepted_.color_index &&
            source.depth_index > last_accepted_.depth_index &&
            source.color_device_us > last_accepted_.color_device_us &&
            source.depth_device_us > last_accepted_.depth_device_us &&
            source.host_ready_mono_ns >= last_accepted_.host_ready_mono_ns);
}

Offer Mailbox::try_offer(const View& view) noexcept {
    counts_.offered.fetch_add(1, relaxed);
    try {
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        if (!lock.owns_lock()) {
            counts_.busy.fetch_add(1, relaxed);
            return Offer::Busy;
        }
        const auto active = active_stream_.load(std::memory_order_acquire);
        if (active == 0) {
            counts_.inactive.fetch_add(1, relaxed);
            return Offer::Inactive;
        }
        std::size_t pixel_count = 0;
        if (!valid(view, pixel_count)) {
            counts_.invalid.fetch_add(1, relaxed);
            return Offer::Invalid;
        }
        if (view.metadata.source.stream_id != active || !newer(view.metadata.source)) {
            counts_.stale.fetch_add(1, relaxed);
            return Offer::Stale;
        }
        // Copy under the lock; the SDK buffer is not retained after return.
        std::memcpy(pending_.pixels_.data(), view.pixels, view.bytes);
        pending_.metadata_ = view.metadata;
        pending_.pixel_count_ = pixel_count;
        const bool replaced = has_pending_;
        has_pending_ = true;
        last_accepted_ = view.metadata.source;
        has_accepted_ = true;
        counts_.accepted.fetch_add(1, std::memory_order_release);
        if (replaced) {
            counts_.replaced.fetch_add(1, relaxed);
        }
        // Ring the doorbell after releasing the frame lock. Taking and dropping
        // the doorbell lock orders the count update before a waiter's check, so
        // a wake-up cannot be lost.
        lock.unlock();
        {
            std::lock_guard<std::mutex> bell(doorbell_mutex_);
        }
        doorbell_cv_.notify_one();
        return replaced ? Offer::Replaced : Offer::Accepted;
    } catch (...) {
        counts_.busy.fetch_add(1, relaxed);
        return Offer::Busy;
    }
}

// Consumer-side sleep until a new frame was accepted (or timeout).
bool Mailbox::wait_for_accepted_after(std::uint64_t seen, std::chrono::milliseconds timeout) noexcept {
    try {
        std::unique_lock<std::mutex> bell(doorbell_mutex_);
        return doorbell_cv_.wait_for(bell, timeout, [&]() {
            return counts_.accepted.load(std::memory_order_acquire) > seen;
        });
    } catch (...) {
        return false;
    }
}

Take Mailbox::try_take(Frame& out) noexcept {
    try {
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        if (!lock.owns_lock()) {
            counts_.busy.fetch_add(1, relaxed);
            return Take::Busy;
        }
        if (active_stream_.load(std::memory_order_acquire) == 0) {
            counts_.inactive.fetch_add(1, relaxed);
            return Take::Inactive;
        }
        if (out.capacity_pixels() != pending_.capacity_pixels()) {
            return Take::Capacity;
        }
        if (!has_pending_) {
            return Take::Empty;
        }
        // Swap buffers: the consumer owns the frame, the mailbox reuses its old one.
        pending_.pixels_.swap(out.pixels_);
        std::swap(pending_.metadata_, out.metadata_);
        std::swap(pending_.pixel_count_, out.pixel_count_);
        has_pending_ = false;
        counts_.taken.fetch_add(1, relaxed);
        return Take::Taken;
    } catch (...) {
        counts_.busy.fetch_add(1, relaxed);
        return Take::Busy;
    }
}

Counters Mailbox::counters() const noexcept {
    return {counts_.offered.load(relaxed),
            counts_.accepted.load(relaxed),
            counts_.replaced.load(relaxed),
            counts_.taken.load(relaxed),
            counts_.busy.load(relaxed),
            counts_.invalid.load(relaxed),
            counts_.stale.load(relaxed),
            counts_.inactive.load(relaxed),
            counts_.discarded_on_reset.load(relaxed)};
}

}  // namespace perception::native_depth
