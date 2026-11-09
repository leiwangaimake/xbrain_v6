/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rns_packet_slot_pool.cpp
 * Brief: PacketSlotPool unit tests: slots, bad releases, lifetime, cross-thread release, no allocation.
 *
 * Description:
 * What it checks (A19-ALLOC-1, our half; the pool is rns_packet_slot_pool.hpp). 1: four slots
 * give four distinct, zero-filled, evenly spaced buffers, the fifth acquire is nullptr, and a
 * freed slot is the next one handed out (lowest first). 2: a second release, a foreign pointer,
 * a pointer inside or just outside a slot only count as bad releases and change nothing else.
 * 3: the owner's detach() keeps the pool alive while slots are held; the last release, on
 * another thread, frees it there. 4: an owner thread writes a pattern into each slot and a
 * second thread checks it before releasing, 1000000 times: a slot is never handed out again
 * while the other thread still reads it (run it under TSan for the memory-order half). 5: in
 * steady state acquire() and release() allocate and free nothing (operator new/delete counted).
 * What it does not check. Allocation is counted at operator new here, so a direct malloc would
 * not be seen; tests/test_rns_zenoh_alloc.cpp counts at the malloc layer on Linux, through the
 * real transport and zenoh-c. A wrong memory order is not guaranteed to show up, even under TSan.
 * Pure host test: no SDK, network or device.
 */

#include "output/rns_packet_slot_pool.hpp"

#include <array>
#include <atomic>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <new>
#include <thread>

namespace {
std::atomic<std::uint64_t> news{0}, deletes{0};
// The pool object under watch: its deletion is recorded with the thread that did it.
std::atomic<const void*> watched{nullptr};
std::atomic<int> watched_freed{0};
thread_local int watched_freed_here = 0;
void* counted_new(std::size_t n) {
    ++news;
    if (void* p = std::malloc(n ? n : 1)) return p;
    throw std::bad_alloc();
}
void* counted_aligned(std::size_t n, std::size_t a) {
    ++news;
    void* p = nullptr;
    if (posix_memalign(&p, a, n ? n : 1) == 0) return p;
    throw std::bad_alloc();
}
void counted_delete(void* p) noexcept {
    if (!p) return;
    ++deletes;
    if (p == watched.load()) {
        ++watched_freed;
        ++watched_freed_here;
    }
    std::free(p);
}
[[noreturn]] void fail(const char* expr, int line) {
    std::cerr << "test_rns_packet_slot_pool CHECK " << line << ": " << expr << '\n';
    std::abort();
}
#define CHECK(x) do { if (!(x)) fail(#x, __LINE__); } while (false)
}  // namespace
void* operator new(std::size_t n) { return counted_new(n); }
void* operator new[](std::size_t n) { return counted_new(n); }
void* operator new(std::size_t n, const std::nothrow_t&) noexcept {
    try { return counted_new(n); } catch (...) { return nullptr; }
}
void* operator new[](std::size_t n, const std::nothrow_t&) noexcept {
    try { return counted_new(n); } catch (...) { return nullptr; }
}
void* operator new(std::size_t n, std::align_val_t a) { return counted_aligned(n, static_cast<std::size_t>(a)); }
void* operator new[](std::size_t n, std::align_val_t a) { return counted_aligned(n, static_cast<std::size_t>(a)); }
void operator delete(void* p) noexcept { counted_delete(p); }
void operator delete[](void* p) noexcept { counted_delete(p); }
void operator delete(void* p, std::size_t) noexcept { counted_delete(p); }
void operator delete[](void* p, std::size_t) noexcept { counted_delete(p); }
void operator delete(void* p, const std::nothrow_t&) noexcept { counted_delete(p); }
void operator delete[](void* p, const std::nothrow_t&) noexcept { counted_delete(p); }
void operator delete(void* p, std::align_val_t) noexcept { counted_delete(p); }
void operator delete[](void* p, std::align_val_t) noexcept { counted_delete(p); }
void operator delete(void* p, std::size_t, std::align_val_t) noexcept { counted_delete(p); }
void operator delete[](void* p, std::size_t, std::align_val_t) noexcept { counted_delete(p); }

namespace {
using perception::rns::PacketSlotPool;
constexpr std::size_t kSlots = 4;      // as the transport (ZenohRtTransport::kPoolSlots)
constexpr std::size_t kSlotBytes = 64;
int groups = 0;
void pass(const char* name) {
    ++groups;
    std::cout << "test_rns_packet_slot_pool " << name << " PASS\n";
}

// Makes a pool and watches its object for deletion.
PacketSlotPool* make(std::size_t slot_bytes = kSlotBytes, std::size_t slots = kSlots) {
    auto* const pool = PacketSlotPool::create(slot_bytes, slots);
    CHECK(pool != nullptr);
    watched.store(pool);
    watched_freed.store(0);
    return pool;
}

void slots_and_order() {
    CHECK(PacketSlotPool::create(0, kSlots) == nullptr);
    CHECK(PacketSlotPool::create(kSlotBytes, 0) == nullptr);
    CHECK(PacketSlotPool::create(kSlotBytes, PacketSlotPool::kMaxSlots + 1) == nullptr);
    CHECK(PacketSlotPool::create(SIZE_MAX / 2 + 1, 2) == nullptr);
    auto* const pool = make();
    CHECK(pool->slots() == kSlots && pool->slot_bytes() == kSlotBytes && pool->held() == 0);
    std::array<std::uint8_t*, kSlots> slot{};
    for (std::size_t i = 0; i < kSlots; ++i) {
        slot[i] = pool->acquire();
        CHECK(slot[i] != nullptr && pool->held() == i + 1);
        // Lowest first, contiguous, slot_bytes apart; zero-filled at creation.
        CHECK(i == 0 || slot[i] == slot[0] + i * kSlotBytes);
        for (std::size_t b = 0; b < kSlotBytes; ++b) CHECK(slot[i][b] == 0);
    }
    CHECK(pool->acquire() == nullptr && pool->held() == kSlots);
    // A freed slot is the next one handed out, lowest free first.
    PacketSlotPool::release(slot[2], pool);
    CHECK(pool->held() == kSlots - 1);
    CHECK(pool->acquire() == slot[2] && pool->acquire() == nullptr);
    PacketSlotPool::release(slot[3], pool);
    PacketSlotPool::release(slot[0], pool);
    CHECK(pool->held() == kSlots - 2);
    CHECK(pool->acquire() == slot[0] && pool->acquire() == slot[3] && pool->acquire() == nullptr);
    for (auto* s : slot) PacketSlotPool::release(s, pool);
    CHECK(pool->held() == 0 && pool->bad_releases() == 0);
    // Every slot back: detach frees the pool at once, on this thread.
    pool->detach();
    CHECK(watched_freed.load() == 1 && watched_freed_here == 1);
    watched_freed_here = 0;
    pass("four_distinct_zeroed_slots_fifth_refused_lowest_free_first");
}

void bad_releases() {
    auto* const pool = make();
    auto* const a = pool->acquire();
    auto* const b = pool->acquire();
    CHECK(a && b && pool->held() == 2);
    std::uint8_t foreign[kSlotBytes] = {};
    const auto base = reinterpret_cast<std::uintptr_t>(a);
    void* const probes[] = {
        foreign,                                                     // not ours
        a + 1,                                                       // inside a held slot
        b + kSlotBytes - 1,                                          // last byte of a held slot
        reinterpret_cast<void*>(base - 1),                           // just before the slots
        reinterpret_cast<void*>(base + kSlots * kSlotBytes),         // just after the slots
        a + 2 * kSlotBytes,                                          // a free slot (never acquired)
        nullptr};
    std::uint64_t expected = 0;
    for (void* p : probes) {
        PacketSlotPool::release(p, pool);
        CHECK(pool->bad_releases() == ++expected && pool->held() == 2);
    }
    // A second release of the same slot: counted, and the reference count is not dropped twice
    // (the pool must survive the owner's detach while b is still held).
    PacketSlotPool::release(a, pool);
    CHECK(pool->held() == 1);
    PacketSlotPool::release(a, pool);
    CHECK(pool->bad_releases() == ++expected && pool->held() == 1);
    pool->detach();
    CHECK(watched_freed.load() == 0);
    PacketSlotPool::release(b, pool);
    CHECK(watched_freed.load() == 1);
    watched_freed_here = 0;
    pass("double_foreign_inside_outside_free_pointers_only_counted");
}

void lifetime_across_threads() {
    auto* const pool = make();
    auto* const a = pool->acquire();
    auto* const b = pool->acquire();
    CHECK(a && b);
    // The owner goes first (transport destroyed while zenoh-c still holds two slots).
    pool->detach();
    CHECK(watched_freed.load() == 0);
    PacketSlotPool::release(a, pool);
    CHECK(watched_freed.load() == 0 && watched_freed_here == 0);
    // The last release runs on another thread and frees the pool there.
    int freed_on_other = -1;
    std::thread other([&] {
        PacketSlotPool::release(b, pool);
        freed_on_other = watched_freed_here;
    });
    other.join();
    CHECK(watched_freed.load() == 1 && freed_on_other == 1 && watched_freed_here == 0);
    pass("detach_keeps_held_pool_alive_last_release_frees_it_on_its_thread");
}

void cross_thread_release() {
    constexpr std::uint64_t kRounds = 1000000;
    auto* const pool = make();
    // Single-producer single-consumer ring of handed-over slots (the "zenoh-c" side).
    constexpr std::size_t kRing = 8;
    std::array<std::uint8_t*, kRing> ring{};
    std::array<std::uint64_t, kRing> ring_seq{};
    std::atomic<std::uint64_t> head{0}, tail{0};
    std::atomic<bool> corrupt{false};
    std::thread releaser([&] {
        for (std::uint64_t taken = 0; taken < kRounds; ++taken) {
            while (head.load(std::memory_order_acquire) == taken) std::this_thread::yield();
            auto* const slot = ring[taken % kRing];
            const auto seq = ring_seq[taken % kRing];
            tail.store(taken + 1, std::memory_order_release);
            // The bytes must still be exactly what the owner wrote for this packet.
            std::uint64_t stamp = 0;
            std::memcpy(&stamp, slot, sizeof(stamp));
            bool same = stamp == seq;
            for (std::size_t i = sizeof(stamp); i < kSlotBytes; ++i)
                same = same && slot[i] == static_cast<std::uint8_t>(seq + i);
            if (!same) corrupt.store(true, std::memory_order_relaxed);
            PacketSlotPool::release(slot, pool);
        }
    });
    auto* const first = pool->acquire();  // slot 0, to name the slots below
    PacketSlotPool::release(first, pool);
    std::uint64_t refusals = 0;
    std::array<std::uint64_t, kSlots> used{};
    for (std::uint64_t seq = 0; seq < kRounds; ++seq) {
        std::uint8_t* slot = nullptr;
        while (!(slot = pool->acquire())) { ++refusals; std::this_thread::yield(); }
        ++used[static_cast<std::size_t>(slot - first) / kSlotBytes];
        std::memcpy(slot, &seq, sizeof(seq));
        for (std::size_t i = sizeof(seq); i < kSlotBytes; ++i) slot[i] = static_cast<std::uint8_t>(seq + i);
        while (seq - tail.load(std::memory_order_acquire) >= kRing) std::this_thread::yield();
        ring[seq % kRing] = slot;
        ring_seq[seq % kRing] = seq;
        head.store(seq + 1, std::memory_order_release);
    }
    releaser.join();
    CHECK(!corrupt.load() && pool->held() == 0 && pool->bad_releases() == 0);
    pool->detach();
    CHECK(watched_freed.load() == 1);
    watched_freed_here = 0;
    std::cout << "test_rns_packet_slot_pool cross_thread rounds=" << kRounds << " acquire_refusals=" << refusals
              << " slot_uses=" << used[0] << '/' << used[1] << '/' << used[2] << '/' << used[3] << '\n';
    pass("owner_writes_other_thread_reads_then_releases_never_reused_early");
}

void steady_state_no_allocation() {
    auto* const pool = make(262144, kSlots);  // the service's packet bound
    const auto news_before = news.load(), deletes_before = deletes.load();
    for (int i = 0; i < 100000; ++i) {
        auto* const a = pool->acquire();
        auto* const b = pool->acquire();
        CHECK(a && b);
        a[0] = static_cast<std::uint8_t>(i);
        PacketSlotPool::release(b, pool);
        PacketSlotPool::release(a, pool);
        (void)pool->held();
        (void)pool->bad_releases();
    }
    CHECK(news.load() == news_before && deletes.load() == deletes_before);
    pool->detach();
    CHECK(watched_freed.load() == 1);
    watched_freed_here = 0;
    pass("100000_acquire_release_cycles_no_operator_new_or_delete");
}
}  // namespace

int main() {
    slots_and_order();
    bad_releases();
    lifetime_across_threads();
    cross_thread_release();
    steady_state_no_allocation();
    std::cout << "PacketSlotPool PASS: " << groups << " groups; no SDK, network or device.\n";
    return 0;
}
