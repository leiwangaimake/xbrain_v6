/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_packet_slot_pool.hpp
 * Brief: Fixed packet buffers the RT transport hands to zenoh-c; zenoh-c's deleter returns them.
 *
 * Description:
 * What problem this solves. A19-ALLOC-1 (19 section 14, P19-3): the fast line allocates nothing
 * in steady state. ZenohRtTransport must give zenoh-c bytes that stay valid until zenoh-c calls
 * the deleter it was given (zenoh_commons.h, z_bytes_from_buf: "A thread-safe function, that will
 * be called on data when this_ is dropped"), which may be after the put returned and on another
 * thread. It used to malloc one copy per packet and hand over std::free. This pool replaces that
 * malloc: a few fixed slots of max_packet bytes, made once at startup; the deleter only marks
 * its slot free again.
 *
 * Rules. One owner thread calls acquire() (the transport's single publishing thread). release()
 * is the zenoh-c deleter and may run on any thread, during or after the put. Only acquire()
 * turns a slot from free to held and only release() turns it back, so one atomic flag per slot
 * is enough: no compare-exchange, no lock, no waiting, no allocation. acquire() never blocks:
 * with every slot still held by zenoh-c it returns nullptr and the caller takes its old malloc
 * path (counted there).
 *
 * Lifetime. Reference counted: one reference for the owner, one per held slot. The owner's
 * detach() drops its reference; whoever drops the last one deletes the pool, so a deleter that
 * zenoh-c runs after the transport is gone still finds its pool.
 *
 * What it does not cover. zenoh-c 1.9.0 still allocates per packet on the publishing thread
 * (z_bytes_from_buf wraps the buffer in a new Arc; z_publisher_put allocates inside). That half
 * of ALLOC-1 is the middleware's, not ours.
 * Tests: tests/test_rns_packet_slot_pool.cpp (pure), tests/test_rns_zenoh_alloc.cpp (Linux,
 * malloc counted per thread, our code and zenoh-c in separate columns).
 */
#pragma once
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <new>

namespace perception::rns {

class PacketSlotPool {
public:
    // A deleter that may run inside zenoh-c on any thread must not take a lock.
    static_assert(std::atomic<bool>::is_always_lock_free && std::atomic<std::size_t>::is_always_lock_free &&
                  std::atomic<std::uint64_t>::is_always_lock_free, "lock-free flags and counters required");
    // Upper bound of the slot count (it bounds the scan in acquire()); the transport uses 4.
    static constexpr std::size_t kMaxSlots = 16;

    // Startup only. slots * slot_bytes bytes, zero-filled here so that the pages are touched now
    // and not on the first packets. nullptr on a bad size or an allocation failure (the transport
    // constructor then throws: a startup error).
    static PacketSlotPool* create(std::size_t slot_bytes, std::size_t slots) noexcept {
        if (slot_bytes == 0 || slots == 0 || slots > kMaxSlots || slot_bytes > SIZE_MAX / slots) return nullptr;
        auto* const pool = new (std::nothrow) PacketSlotPool();
        if (!pool) return nullptr;
        pool->bytes_.reset(new (std::nothrow) std::uint8_t[slot_bytes * slots]());
        if (!pool->bytes_) {
            delete pool;
            return nullptr;
        }
        pool->slot_bytes_ = slot_bytes;
        pool->slots_ = slots;
        return pool;
    }

    // Owner thread only. The lowest free slot, or nullptr when zenoh-c still holds every slot.
    // The acquire load pairs with release()'s exchange: zenoh-c's last read of a slot happens
    // before the owner writes the next packet into it.
    std::uint8_t* acquire() noexcept {
        for (std::size_t i = 0; i < slots_; ++i) {
            if (held_[i].load(std::memory_order_acquire)) continue;
            // Nobody else turns a free slot into a held one: a plain store is enough. It reaches
            // the deleter's thread through zenoh-c's own hand-over of the bytes.
            held_[i].store(true, std::memory_order_relaxed);
            refs_.fetch_add(1, std::memory_order_relaxed);  // the owner already holds a reference
            return bytes_.get() + i * slot_bytes_;
        }
        return nullptr;
    }

    // The z_bytes_from_buf deleter (context = the pool). Any thread, during or after the put. In
    // the steady state only atomic operations here: no lock, no allocation, no logging. Marks the
    // slot free and drops that slot's reference; the one exception is the last reference after
    // the owner has detached (shutdown), which deletes the pool and its bytes. A pointer that is
    // not the start of one of this pool's slots, or a slot that is already free (a second
    // release), only counts in bad_releases(). It cannot tell a stale release of a slot that was
    // lent again since (no generation number): that sequence would break zenoh-c's "exactly one
    // release per payload" and no path of ours makes it (GPT question 16 supplement).
    static void release(void* data, void* context) noexcept {
        auto* const pool = static_cast<PacketSlotPool*>(context);
        const std::size_t index = pool->slot_of(data);
        if (index == kNoSlot || !pool->held_[index].exchange(false, std::memory_order_acq_rel)) {
            pool->bad_releases_.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        pool->drop_ref();
    }

    // Owner gives up its reference (transport destructor, or its constructor failing after the
    // pool was made). The owner must not touch the pool afterwards.
    void detach() noexcept { drop_ref(); }

    // Diagnostics, any thread: slots zenoh-c has not returned yet, and bad releases (must stay 0).
    std::size_t held() const noexcept {
        std::size_t count = 0;
        for (std::size_t i = 0; i < slots_; ++i)
            if (held_[i].load(std::memory_order_relaxed)) ++count;
        return count;
    }
    std::uint64_t bad_releases() const noexcept { return bad_releases_.load(std::memory_order_relaxed); }
    std::size_t slot_bytes() const noexcept { return slot_bytes_; }
    std::size_t slots() const noexcept { return slots_; }

private:
    static constexpr std::size_t kNoSlot = SIZE_MAX;
    PacketSlotPool() = default;
    ~PacketSlotPool() = default;
    PacketSlotPool(const PacketSlotPool&) = delete;
    PacketSlotPool& operator=(const PacketSlotPool&) = delete;

    // Index of the slot that starts exactly at data, or kNoSlot. Addresses are compared as
    // integers: data may come from anywhere.
    std::size_t slot_of(const void* data) const noexcept {
        const auto base = reinterpret_cast<std::uintptr_t>(bytes_.get());
        const auto at = reinterpret_cast<std::uintptr_t>(data);
        if (at < base || at - base >= slots_ * slot_bytes_ || (at - base) % slot_bytes_ != 0) return kNoSlot;
        return (at - base) / slot_bytes_;
    }
    // Release on every decrement, acquire before the delete: all earlier uses of the pool by
    // other threads happen before it is freed.
    void drop_ref() noexcept {
        if (refs_.fetch_sub(1, std::memory_order_acq_rel) == 1) delete this;
    }

    std::atomic<std::size_t> refs_{1};  // the owner, plus one per held slot
    std::atomic<std::uint64_t> bad_releases_{0};
    std::size_t slot_bytes_ = 0, slots_ = 0;
    std::unique_ptr<std::uint8_t[]> bytes_;                   // slots_ * slot_bytes_, fixed
    std::array<std::atomic<bool>, kMaxSlots> held_{};         // true = handed to zenoh-c, not yet released
};

}  // namespace perception::rns
