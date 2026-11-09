/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_envelope.cpp
 * Brief: One bounded atomic sequence and one coherent ClockStatus snapshot.
 *
 * Description:
 * Implements ProcessEnvelope (see rns_envelope.hpp for the contract and the comparison with the
 * platform EnvelopeWriter).
 *
 * Why two single-word atomics and no mutex. The stamp runs inside the publisher's finalize step,
 * between reading the clock and handing the packet to Zenoh; a lock there could stall one topic
 * behind another. seq_ and clock_ are independent 64-bit words, each updated by bounded
 * compare-exchange, and the ClockStatus (receipt time, sync) pair is packed into ONE word so a
 * reader can never see the time of one status with the sync bit of another.
 *
 * Why relaxed memory order is enough. Neither atomic guards other data: the seq value itself is
 * the only thing a stamp publishes, and the clock word carries its whole snapshot. Uniqueness and
 * monotonic issue of seq come from the atomic read-modify-write, not from ordering.
 */
#include "output/rns_envelope.hpp"

namespace perception::rns {
namespace {

// docs11 rid alphabet [a-z0-9_-]{1,32}. The publisher and the event channel splice
// the rid into Zenoh keys and envelope JSON without escaping, so the alphabet is
// enforced here once instead of escaped at every use.
bool valid_rid(std::string_view rid) noexcept {
    if (rid.empty() || rid.size() > ProcessEnvelope::kMaxRidLength) return false;
    for (const unsigned char c : rid) {
        if (!((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') ||
              c == '_' || c == '-')) return false;
    }
    return true;
}

// docs11 boot: the first 8 hex digits of the host boot id, lowercase. Uppercase is
// rejected rather than folded, so two spellings of one boot never both appear.
bool valid_boot(std::string_view boot) noexcept {
    if (boot.size() != 8) return false;
    for (const unsigned char c : boot) {
        if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) return false;
    }
    return true;
}

}  // namespace

ProcessEnvelope::ProcessEnvelope(std::string rid, std::string boot,
                                 std::int64_t sync_timeout_ms)
    : sync_timeout_ms_(sync_timeout_ms) {
    valid_ = valid_rid(rid) && valid_boot(boot) && sync_timeout_ms > 0;
    // Copy only after validation: an invalid configuration leaves both buffers
    // empty, so rid()/boot() can never expose a half-validated identity.
    if (valid_) {
        rid_length_ = rid.size();
        for (std::size_t i = 0; i < rid_length_; ++i) rid_[i] = rid[i];
        for (std::size_t i = 0; i < boot_.size(); ++i) boot_[i] = boot[i];
    }
}

bool ProcessEnvelope::valid() const noexcept { return valid_; }

std::string_view ProcessEnvelope::rid() const noexcept { return {rid_.data(), rid_length_}; }

// boot_ has a fixed size, so emptiness on an invalid configuration is explicit.
std::string_view ProcessEnvelope::boot() const noexcept {
    return {boot_.data(), valid_ ? boot_.size() : 0};
}

bool ProcessEnvelope::note_clock_status(bool sync, std::int64_t received_mono_ms) noexcept {
    if (!valid_ || received_mono_ms < 0 || received_mono_ms > kMaxMonoMs) return false;
    // Pack (receipt, sync) into one word; receipt <= kMaxMonoMs keeps the shift
    // lossless and the result below kNoClock.
    const auto receipt = static_cast<std::uint64_t>(received_mono_ms);
    const auto desired = (receipt << 1) | static_cast<std::uint64_t>(sync);
    auto observed = clock_.load(std::memory_order_relaxed);
    for (unsigned attempt = 0; attempt < kMaxCasAttempts; ++attempt) {
        // Re-checked on every retry: a concurrent newer receipt must win, so an
        // older status arriving late can never make the snapshot go backwards.
        if (observed != kNoClock && receipt < (observed >> 1)) return false;
        // On failure compare_exchange_strong reloads observed; loop re-checks it.
        if (clock_.compare_exchange_strong(observed, desired, std::memory_order_relaxed,
                                          std::memory_order_relaxed)) return true;
    }
    return false;
}

bool ProcessEnvelope::stamp(std::int64_t wall_ms, std::int64_t mono_ms,
                            EnvelopeStamp& out) noexcept {
    if (!valid_ || wall_ms < 0 || mono_ms < 0 || mono_ms > kMaxMonoMs) return false;

    // All ClockStatus fields are in this one atomic snapshot; relaxed order is
    // sufficient because it guards no separately published data.
    const auto clock = clock_.load(std::memory_order_relaxed);
    bool ts_sync = false;
    if (clock != kNoClock) {
        const auto received_ms = static_cast<std::int64_t>(clock >> 1);
        // Ordering this check before subtraction prevents negative-age freshness
        // and avoids signed overflow. Wall time never participates in age.
        ts_sync = (clock & 1U) != 0 && received_ms <= mono_ms &&
                  mono_ms - received_ms < sync_timeout_ms_;
    }

    // CAS instead of fetch_add: fetch_add would wrap UINT64_MAX to 0 and reissue
    // old numbers, while 11 section 3.0 requires seq to be strictly increasing.
    auto observed = seq_.load(std::memory_order_relaxed);
    for (unsigned attempt = 0; attempt < kMaxCasAttempts; ++attempt) {
        if (observed == std::numeric_limits<std::uint64_t>::max()) return false;
        const auto next = observed + 1;
        if (seq_.compare_exchange_strong(observed, next, std::memory_order_relaxed,
                                        std::memory_order_relaxed)) {
            // out is written only after the seq is ours, so a failed stamp
            // leaves the caller's value untouched (header contract).
            out = EnvelopeStamp{next, wall_ms, mono_ms, ts_sync};
            return true;
        }
    }
    // Contention bound reached: no seq was consumed, the caller drops the message.
    return false;
}

}  // namespace perception::rns
