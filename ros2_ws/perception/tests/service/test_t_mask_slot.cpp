/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_t_mask_slot.cpp
 * Brief: TMaskSlot binarisation, refusals, reuse, zero allocation and torn-copy safety.
 *
 * Description:
 * Requirements covered: 19 P19-2 and A19-SLOT-1.
 * 1. Any nonzero byte becomes 1 (the class-92 union is 0/255; the T owner accepts only 0/1).
 * 2. Bad input is refused and counted; the previous mask stays.
 * 3. publish_empty gives an all-zero mask and advances only the any-publication clock.
 * 4. Refusal matrix, each case against a positive control: other stream, other epoch, other
 *    projection revision, capture after the depth frame, and newer or not-identical frame set by
 *    the owner's own identity rule.
 * 5. With nothing newer published the reader reuses its copy (no second copy).
 * 6. Steady state allocates nothing (global operator new counted).
 * 7. Deterministic interleavings (test-only hook halfway through the reader's copy):
 *    a publication into the other buffer leaves the copy valid; two publications (the second
 *    rewrites the buffer being copied) force a retry and the reader returns the newest complete
 *    publication, never the torn one; a writer that rewrites it on every attempt makes the reader
 *    give up (no T this frame) without waiting. Deleting the reader's sequence re-check fails the
 *    second case (checked on the Mac, 2026-09-30).
 * 8. A writer publishing as fast as it can against a reader copying continuously: every copy is
 *    exactly one publication (patterns cycle with the publication number, so a copy torn between
 *    two publications of the same buffer mixes two patterns).
 */

#ifdef NDEBUG
#undef NDEBUG
#endif

#include "service/t_mask_slot.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <new>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

// The replaced global operator new/delete below pair malloc with free on purpose; GCC's
// mismatched-new-delete check sees the inlined free and flags every container in this file.
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic ignored "-Wmismatched-new-delete"
#endif

namespace {

std::atomic<bool> count_allocations{false};
std::atomic<long> allocations{0};

}  // namespace

void* operator new(std::size_t size) {
    if (count_allocations.load(std::memory_order_relaxed)) allocations.fetch_add(1, std::memory_order_relaxed);
    if (void* p = std::malloc(size == 0 ? 1 : size)) return p;
    throw std::bad_alloc();
}
void operator delete(void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }

namespace {

using perception::geometry::CaptureStamp;
using perception::geometry::RawTMask;
using perception::service::TMaskSlot;

int checks = 0;

void check(bool condition, const std::string& message) {
    ++checks;
    if (!condition) throw std::runtime_error(message);
}

// Frame set number g of stream `stream`: every index and clock increases with g.
CaptureStamp stamp_of(std::uint64_t g, std::uint64_t stream = 7, std::uint64_t epoch = 1) {
    CaptureStamp s;
    s.source.stream_id = stream;
    s.source.sequence = g;
    s.source.color_index = g;
    s.source.depth_index = g;
    s.source.color_device_us = 1000 * g;
    s.source.depth_device_us = 1000 * g + 3;
    s.source.host_ready_mono_ns = static_cast<std::int64_t>(1000000 * g) + 50000000;
    s.clock_epoch = epoch;
    s.capture_mono_ns = s.source.host_ready_mono_ns - 23000000;  // host ready minus the link estimate
    return s;
}

std::vector<std::uint8_t> raster(std::uint8_t value) {
    return std::vector<std::uint8_t>(TMaskSlot::kBytes, value);
}

void binarisation() {
    TMaskSlot slot;
    std::vector<std::uint8_t> source(TMaskSlot::kBytes);
    const std::uint8_t values[] = {0, 1, 7, 128, 255, 0, 2, 0, 64};
    for (std::size_t i = 0; i < source.size(); ++i) source[i] = values[(i * 7 + i / 1280) % 9];
    check(slot.publish(source.data(), 1280, 800, 1280, stamp_of(10), 1), "publish refused a valid raster");
    TMaskSlot::Snapshot snapshot;
    const RawTMask* mask = slot.latest(snapshot, stamp_of(10), 1);
    check(mask != nullptr, "valid mask not taken");
    for (std::size_t i = 0; i < source.size(); ++i)
        check(mask->pixels[i] == (source[i] != 0 ? 1 : 0), "binarisation wrong at byte " + std::to_string(i));
    check(mask->width == 1280 && mask->height == 800 && mask->stride_bytes == 1280 && mask->bytes == TMaskSlot::kBytes &&
          mask->material == perception::geometry::MaskMaterial::RawTraversable && mask->projection_revision == 1,
          "mask header wrong");
    // A padded row stride (the class-92 canvas can be a view) is read row by row.
    std::vector<std::uint8_t> padded(1296 * 800, 9);
    for (std::size_t y = 0; y < 800; ++y)
        for (std::size_t x = 0; x < 1280; ++x) padded[y * 1296 + x] = (x + y) % 3 == 0 ? 255 : 0;
    check(slot.publish(padded.data(), 1280, 800, 1296, stamp_of(11), 1), "padded raster refused");
    mask = slot.latest(snapshot, stamp_of(11), 1);
    check(mask != nullptr, "padded mask not taken");
    for (std::size_t y = 0; y < 800; ++y)
        for (std::size_t x = 0; x < 1280; ++x)
            check(mask->pixels[y * 1280 + x] == ((x + y) % 3 == 0 ? 1 : 0), "padded row read wrong");
}

void bad_input_and_empty() {
    TMaskSlot slot;
    const auto ones = raster(255);
    check(!slot.publish(nullptr, 1280, 800, 1280, stamp_of(5), 1), "null raster accepted");
    check(!slot.publish(ones.data(), 1279, 800, 1280, stamp_of(5), 1), "wrong width accepted");
    check(!slot.publish(ones.data(), 1280, 799, 1280, stamp_of(5), 1), "wrong height accepted");
    check(!slot.publish(ones.data(), 1280, 800, 1279, stamp_of(5), 1), "short stride accepted");
    check(slot.counters().refused_input == 4 && slot.last_publish_mono_ns() == -1, "refused input not counted");
    TMaskSlot::Snapshot snapshot;
    check(slot.latest(snapshot, stamp_of(5), 1) == nullptr && slot.counters().missing == 1, "missing before any publish");
    check(slot.publish(ones.data(), 1280, 800, 1280, stamp_of(5), 1), "valid publish refused");
    const std::int64_t nonempty = slot.last_nonempty_publish_mono_ns();
    check(nonempty > 0, "nonempty clock not set");
    check(!slot.publish(ones.data(), 640, 800, 1280, stamp_of(6), 1), "bad size accepted");
    const RawTMask* mask = slot.latest(snapshot, stamp_of(6), 1);
    check(mask != nullptr && mask->stamp.source.sequence == 5 && mask->pixels[0] == 1, "previous mask did not stay");
    check(slot.publish_empty(stamp_of(7), 1), "publish_empty refused");
    mask = slot.latest(snapshot, stamp_of(7), 1);
    check(mask != nullptr && snapshot.empty && mask->stamp.source.sequence == 7, "empty mask not taken");
    for (std::size_t i = 0; i < TMaskSlot::kBytes; ++i) check(mask->pixels[i] == 0, "empty mask not all zero");
    check(slot.last_nonempty_publish_mono_ns() == nonempty && slot.last_publish_mono_ns() >= nonempty,
          "publish_empty moved the nonempty clock");
    const auto c = slot.counters();
    check(c.published == 1 && c.published_empty == 1 && c.refused_input == 5, "publish counters wrong");
}

void refusal_matrix() {
    const auto ones = raster(255);
    const CaptureStamp depth = stamp_of(100);
    struct Case {
        const char* name;
        CaptureStamp mask;
        std::uint64_t reader_revision;
        bool taken;
    };
    CaptureStamp other_stream = stamp_of(99, 8);
    CaptureStamp other_epoch = stamp_of(99, 7, 2);
    CaptureStamp future = stamp_of(99);
    future.capture_mono_ns = depth.capture_mono_ns + 1;
    CaptureStamp same_seq_other_frame = stamp_of(100);
    same_seq_other_frame.source.color_index += 1;
    CaptureStamp older_index_not_smaller = stamp_of(99);
    older_index_not_smaller.source.depth_index = depth.source.depth_index;
    const std::vector<Case> cases{
        {"same frame set (control)", stamp_of(100), 1, true},
        {"older frame set (control)", stamp_of(60), 1, true},
        {"other stream", other_stream, 1, false},
        {"other clock epoch", other_epoch, 1, false},
        {"other projection revision", stamp_of(99), 2, false},
        {"captured after the depth frame", future, 1, false},
        {"newer frame set", stamp_of(101), 1, false},
        {"same sequence, other frame set", same_seq_other_frame, 1, false},
        {"older sequence, depth index not older", older_index_not_smaller, 1, false},
    };
    for (const Case& c : cases) {
        TMaskSlot slot;
        check(slot.publish(ones.data(), 1280, 800, 1280, c.mask, 1), std::string("publish refused: ") + c.name);
        TMaskSlot::Snapshot snapshot;
        const RawTMask* mask = slot.latest(snapshot, depth, c.reader_revision);
        check((mask != nullptr) == c.taken, std::string("refusal matrix wrong: ") + c.name);
        const auto k = slot.counters();
        const std::uint64_t refused = k.refused_stream + k.refused_epoch + k.refused_revision + k.refused_future + k.refused_order;
        check(k.taken == (c.taken ? 1U : 0U) && refused == (c.taken ? 0U : 1U), std::string("refusal not counted: ") + c.name);
    }
}

void reuse_and_no_allocation() {
    TMaskSlot slot;
    TMaskSlot::Snapshot snapshot;
    const auto ones = raster(255);
    const auto zeros = raster(0);
    check(slot.publish(ones.data(), 1280, 800, 1280, stamp_of(20), 1), "publish refused");
    const RawTMask* first = slot.latest(snapshot, stamp_of(21), 1);
    const RawTMask* again = slot.latest(snapshot, stamp_of(22), 1);
    check(first != nullptr && again == first && slot.counters().reused == 1 && again->pixels[5] == 1, "copy not reused");
    // Steady state: publications and reads allocate nothing.
    allocations.store(0);
    count_allocations.store(true);
    for (std::uint64_t g = 30; g < 60; ++g) {
        (void)slot.publish((g % 2 == 0 ? ones : zeros).data(), 1280, 800, 1280, stamp_of(g), 1);
        (void)slot.publish_empty(stamp_of(g), 1);
        (void)slot.latest(snapshot, stamp_of(g + 1), 1);
        (void)slot.latest(snapshot, stamp_of(g + 1), 1);
    }
    count_allocations.store(false);
    check(allocations.load() == 0, "steady state allocated " + std::to_string(allocations.load()) + " times");
}

// Pattern of publication g, 0 or 1 per byte. It depends on g % 4: publications g and g + 2 share
// a buffer, so a copy torn between them mixes two different patterns.
inline std::uint8_t pattern(std::uint64_t g, std::size_t i) {
    return static_cast<std::uint8_t>(((i * 2654435761U) >> 11 ^ ((g % 4U) * 40503U) ^ (i >> 5)) & 1U);
}

struct HookState {
    TMaskSlot* slot = nullptr;
    const std::vector<std::vector<std::uint8_t>>* sources = nullptr;
    std::uint64_t next = 0;      // next publication number to write
    int per_call = 0;            // publications per hook call
    int calls_left = 0;          // hook calls that still publish
};

void publish_from_hook(void* context) {
    auto& h = *static_cast<HookState*>(context);
    if (h.calls_left <= 0) return;
    --h.calls_left;
    for (int i = 0; i < h.per_call; ++i, ++h.next)
        (void)h.slot->publish((*h.sources)[h.next % 4].data(), 1280, 800, 1280, stamp_of(h.next), 1);
}

void deterministic_interleavings() {
    std::vector<std::vector<std::uint8_t>> sources(4, std::vector<std::uint8_t>(TMaskSlot::kBytes));
    for (std::uint64_t r = 0; r < 4; ++r)
        for (std::size_t i = 0; i < TMaskSlot::kBytes; ++i) sources[r][i] = pattern(r, i) != 0 ? 255 : 0;
    const CaptureStamp depth = stamp_of(1000);
    const auto verify = [&](const RawTMask* mask, std::uint64_t expected, const char* what) {
        check(mask != nullptr, std::string(what) + ": no mask");
        check(mask->stamp.source.sequence == expected, std::string(what) + ": wrong publication " +
              std::to_string(mask->stamp.source.sequence) + " expected " + std::to_string(expected));
        bool same = true;
        for (std::size_t i = 0; i < TMaskSlot::kBytes && same; ++i) same = mask->pixels[i] == pattern(expected, i);
        check(same, std::string(what) + ": copy is not one complete publication");
    };
    {   // One publication during the copy lands in the other buffer: the copy of 1 stays valid.
        TMaskSlot slot;
        HookState h{&slot, &sources, 2, 1, 1};
        (void)slot.publish(sources[1].data(), 1280, 800, 1280, stamp_of(1), 1);
        slot.test_mid_copy_hook = publish_from_hook;
        slot.test_hook_context = &h;
        TMaskSlot::Snapshot snapshot;
        verify(slot.latest(snapshot, depth, 1), 1, "other-buffer publication");
        check(slot.counters().torn_retries == 0, "other-buffer publication caused a retry");
    }
    {   // Two publications during the copy: the second rewrites the buffer being copied (1 -> 3).
        TMaskSlot slot;
        HookState h{&slot, &sources, 2, 2, 1};
        (void)slot.publish(sources[1].data(), 1280, 800, 1280, stamp_of(1), 1);
        slot.test_mid_copy_hook = publish_from_hook;
        slot.test_hook_context = &h;
        TMaskSlot::Snapshot snapshot;
        verify(slot.latest(snapshot, depth, 1), 3, "rewritten buffer");
        check(slot.counters().torn_retries == 1, "rewritten buffer not retried");
    }
    {   // The writer rewrites the buffer on every attempt: the reader gives up, never waits.
        TMaskSlot slot;
        HookState h{&slot, &sources, 2, 2, TMaskSlot::kCopyAttempts};
        (void)slot.publish(sources[1].data(), 1280, 800, 1280, stamp_of(1), 1);
        slot.test_mid_copy_hook = publish_from_hook;
        slot.test_hook_context = &h;
        TMaskSlot::Snapshot snapshot;
        check(slot.latest(snapshot, depth, 1) == nullptr && snapshot.publication == 0, "give-up returned a mask");
        const auto c = slot.counters();
        check(c.torn_giveup == 1 && c.torn_retries == static_cast<std::uint64_t>(TMaskSlot::kCopyAttempts),
              "give-up not counted");
        slot.test_mid_copy_hook = nullptr;  // the writer stops: the next frame gets the newest one
        verify(slot.latest(snapshot, depth, 1), h.next - 1, "after give-up");
    }
}

void racing_writer() {
    TMaskSlot slot;
    std::vector<std::vector<std::uint8_t>> sources(4, std::vector<std::uint8_t>(TMaskSlot::kBytes));
    for (std::uint64_t r = 0; r < 4; ++r)
        for (std::size_t i = 0; i < TMaskSlot::kBytes; ++i) sources[r][i] = pattern(r, i) != 0 ? 255 : 0;
    std::atomic<bool> done{false};
    std::atomic<std::uint64_t> publications{0};
    std::thread writer([&]() {
        // As fast as it can for about 2 s: several publications land inside one reader copy.
        const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        std::uint64_t g = 0;
        while (std::chrono::steady_clock::now() < end) {
            ++g;
            (void)slot.publish(sources[g % 4].data(), 1280, 800, 1280, stamp_of(g), 1);
        }
        publications.store(g);
        done.store(true);
    });
    TMaskSlot::Snapshot snapshot;
    const CaptureStamp depth = stamp_of(100000000);
    long verified = 0, bad = 0;
    std::uint64_t last_seen = 0;
    while (!done.load() || verified == 0) {
        const RawTMask* mask = slot.latest(snapshot, depth, 1);
        if (mask == nullptr || snapshot.publication == last_seen) continue;
        last_seen = snapshot.publication;
        const std::uint64_t g = mask->stamp.source.sequence;
        bool same = g == snapshot.publication;
        for (std::size_t i = 0; i < TMaskSlot::kBytes && same; ++i) same = mask->pixels[i] == pattern(g, i);
        if (!same) ++bad;
        ++verified;
    }
    writer.join();
    const auto c = slot.counters();
    check(bad == 0, "torn or mismatched copies: " + std::to_string(bad) + " of " + std::to_string(verified));
    // Non-vacuous: copies were verified and the writer really did overwrite buffers mid-copy
    // (under a sanitizer the reader is slow and may mostly give up, which is correct behaviour).
    check(verified >= 1 && c.torn_retries + c.torn_giveup > 0,
          "race not exercised: verified=" + std::to_string(verified) + " torn_retries=" + std::to_string(c.torn_retries));
    std::cout << "racing writer: publications=" << publications.load() << " distinct_copies_verified=" << verified
              << " torn_retries=" << c.torn_retries << " torn_giveup=" << c.torn_giveup << "\n";
}

}  // namespace

int main() {
    try {
        binarisation();
        bad_input_and_empty();
        refusal_matrix();
        reuse_and_no_allocation();
        deterministic_interleavings();
        racing_writer();
        std::cout << "test_t_mask_slot: PASS checks=" << checks << "\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "test_t_mask_slot: FAIL " << e.what() << "\n";
        return 1;
    }
}
