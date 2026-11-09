/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_native_depth.cpp
 * Brief: Standalone C++17 tests of the native depth mailbox: fixed storage, provenance, restart, contention.
 *
 * Description:
 * What it checks. The mailbox passes the newest native 640x400 depth frame from capture to the
 * Profile owner. Cases cover construction and capacity, copy lifetime of the latest and a held
 * frame, the maximum raster, invalid metadata and exact rasters, ordering gaps and stream restart,
 * deterministic Busy, no hot-path allocation or storage growth, and concurrent use with no torn
 * frames and a final drain.
 */

#include "modules/native_depth.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <new>
#include <stdexcept>
#include <thread>
#include <type_traits>
#include <vector>

namespace {
std::atomic<std::uint64_t> allocation_count{0};

void* allocate(std::size_t size) {
    void* memory = std::malloc(size == 0 ? 1 : size);
    if (memory == nullptr) {
        throw std::bad_alloc();
    }
    allocation_count.fetch_add(1, std::memory_order_relaxed);
    return memory;
}

void* allocate_aligned(std::size_t size, std::size_t alignment) {
    void* memory = nullptr;
    if (posix_memalign(&memory, alignment, size == 0 ? 1 : size) != 0) {
        throw std::bad_alloc();
    }
    allocation_count.fetch_add(1, std::memory_order_relaxed);
    return memory;
}

[[noreturn]] void fail(const char* expression, int line) {
    std::cerr << "native depth CHECK failed at line " << line << ": " << expression << '\n';
    std::abort();
}

#define CHECK(expression) do { if (!(expression)) fail(#expression, __LINE__); } while (false)
}  // namespace

// Include ordinary, array, sized and over-aligned allocation families so that
// a new heap allocation on the hot path cannot silently bypass the counter.
void* operator new(std::size_t size) { return allocate(size); }
void* operator new[](std::size_t size) { return allocate(size); }
void operator delete(void* memory) noexcept { std::free(memory); }
void operator delete[](void* memory) noexcept { std::free(memory); }
void operator delete(void* memory, std::size_t) noexcept { std::free(memory); }
void operator delete[](void* memory, std::size_t) noexcept { std::free(memory); }
void* operator new(std::size_t size, std::align_val_t alignment) {
    return allocate_aligned(size, static_cast<std::size_t>(alignment));
}
void* operator new[](std::size_t size, std::align_val_t alignment) {
    return allocate_aligned(size, static_cast<std::size_t>(alignment));
}
void operator delete(void* memory, std::align_val_t) noexcept { std::free(memory); }
void operator delete[](void* memory, std::align_val_t) noexcept { std::free(memory); }
void operator delete(void* memory, std::size_t, std::align_val_t) noexcept { std::free(memory); }
void operator delete[](void* memory, std::size_t, std::align_val_t) noexcept { std::free(memory); }

namespace perception::native_depth {
struct MailboxTestAccess {
    static std::unique_lock<std::mutex> hold(Mailbox& mailbox) {
        return std::unique_lock<std::mutex>(mailbox.mutex_);
    }
    static void exhaust_generation(Mailbox& mailbox) {
        mailbox.generation_ = std::numeric_limits<std::uint64_t>::max();
    }
};
}  // namespace perception::native_depth

namespace {
using namespace perception::native_depth;
constexpr std::size_t small_capacity = 8;

View sample(std::uint64_t stream, std::uint64_t sequence, const void* pixels) {
    View view;
    auto& source = view.metadata.source;
    source.stream_id = stream;
    source.sequence = sequence;
    source.color_index = sequence - 1;
    source.depth_index = sequence - 1;
    source.color_device_us = sequence * 1000;
    source.depth_device_us = sequence * 1000 + 1;
    source.host_ready_mono_ns = static_cast<std::int64_t>(sequence * 1000000);
    view.metadata.intrinsics = {200.0F, 201.0F, 1.5F, 0.5F, 4, 2};
    view.metadata.distortion.reported = true;
    view.metadata.distortion.model = 17;  // Deliberately not interpreted here.
    view.metadata.distortion.coefficients = {1, 2, 3, 4, 5, 6, 7, 8};
    view.metadata.unit_m = 0.001F;
    view.pixels = pixels;
    view.bytes = small_capacity * sizeof(std::uint16_t);
    view.pre_d2c = true;
    return view;
}

void expect_pixels(const Frame& frame, std::uint16_t value) {
    for (std::size_t i = 0; i < frame.pixel_count(); ++i) {
        CHECK(frame.pixels()[i] == value);
    }
}

bool successful(Offer result) {
    return result == Offer::Accepted || result == Offer::Replaced;
}

// Zero or oversize capacities are refused; nothing moves while no stream is active; a too-small
// output gets Capacity and keeps both its own contents and the pending frame.
void test_construction_and_capacity() {
    static_assert(!std::is_copy_constructible_v<Frame>);
    static_assert(!std::is_move_constructible_v<Frame>);
    static_assert(!std::is_copy_constructible_v<Mailbox>);
    for (const auto capacity : {std::size_t{0}, kMaxPixels + 1,
                                std::numeric_limits<std::size_t>::max()}) {
        bool frame_rejected = false;
        bool mailbox_rejected = false;
        try { Frame invalid(capacity); }
        catch (const std::invalid_argument&) { frame_rejected = true; }
        try { Mailbox invalid(capacity); }
        catch (const std::invalid_argument&) { mailbox_rejected = true; }
        CHECK(frame_rejected && mailbox_rejected);
    }
    Frame largest(kMaxPixels);
    CHECK(largest.capacity_pixels() == kMaxPixels && largest.pixel_count() == 0);
    Frame smallest(1);
    CHECK(smallest.capacity_pixels() == 1 && smallest.pixel_count() == 0);

    Mailbox mailbox(small_capacity);
    Frame out(small_capacity);
    CHECK(out.pixel_count() == 0 && out.metadata().source.stream_id == 0);
    std::array<std::uint16_t, small_capacity> pixels{};
    CHECK(mailbox.active_stream() == 0);
    CHECK(mailbox.try_offer(sample(1, 1, pixels.data())) == Offer::Inactive);
    CHECK(mailbox.try_take(out) == Take::Inactive);
    const auto stream = mailbox.begin_stream();
    CHECK(stream != 0 && mailbox.active_stream() == stream);
    CHECK(mailbox.try_take(out) == Take::Empty);
    CHECK(mailbox.try_offer(sample(stream, 1, pixels.data())) == Offer::Accepted);

    // A capacity failure preserves both the output's existing contents and the
    // pending frame, even when the output is larger than the mailbox.
    Mailbox seed(16);
    const auto seed_stream = seed.begin_stream();
    Frame wrong(16);
    pixels.fill(44);
    CHECK(seed.try_offer(sample(seed_stream, 1, pixels.data())) == Offer::Accepted);
    CHECK(seed.try_take(wrong) == Take::Taken);
    const auto* held = wrong.pixels();
    CHECK(mailbox.try_take(wrong) == Take::Capacity);
    CHECK(wrong.pixels() == held && wrong.pixel_count() == small_capacity);
    expect_pixels(wrong, 44);
    CHECK(mailbox.try_take(out) == Take::Taken);
    expect_pixels(out, 0);
    CHECK(mailbox.try_take(smallest) == Take::Capacity);
    CHECK(smallest.pixel_count() == 0);
    CHECK(mailbox.counters().inactive == 2);
}

// An offer copies pixels and metadata at once (the source may then be freed); newer offers replace
// the pending frame without touching one already taken; an empty take keeps it.
void test_copy_lifetime_latest_and_held_frame() {
    Mailbox mailbox(small_capacity);
    Frame out(small_capacity);
    const auto stream = mailbox.begin_stream();
    {
        std::vector<std::uint16_t> source{0, 1, 2, 3, 4096, 65535, 17, 0};
        auto view = sample(stream, 1, source.data());
        CHECK(mailbox.try_offer(view) == Offer::Accepted);
        std::fill(source.begin(), source.end(), 999);
        view.metadata.intrinsics.fx = 999.0F;
        view.metadata.distortion.coefficients.fill(999.0F);
    }  // The source owner is destroyed before take.
    CHECK(mailbox.try_take(out) == Take::Taken);
    const std::array<std::uint16_t, small_capacity> expected{0, 1, 2, 3, 4096, 65535, 17, 0};
    CHECK(std::equal(expected.begin(), expected.end(), out.pixels()));
    CHECK(out.metadata().intrinsics.fx == 200.0F);
    CHECK(out.metadata().distortion.coefficients[7] == 8.0F);

    const auto* held_pixels = out.pixels();
    std::array<std::uint16_t, small_capacity> input{};
    input.fill(2);
    CHECK(mailbox.try_offer(sample(stream, 2, input.data())) == Offer::Accepted);
    input.fill(3);
    CHECK(mailbox.try_offer(sample(stream, 3, input.data())) == Offer::Replaced);
    input.fill(4);
    CHECK(mailbox.try_offer(sample(stream, 4, input.data())) == Offer::Replaced);
    CHECK(out.pixels() == held_pixels && out.metadata().source.sequence == 1);
    CHECK(std::equal(expected.begin(), expected.end(), out.pixels()));
    CHECK(mailbox.try_take(out) == Take::Taken);
    CHECK(out.metadata().source.sequence == 4);
    expect_pixels(out, 4);
    const auto* latest_pixels = out.pixels();
    CHECK(mailbox.try_take(out) == Take::Empty);
    CHECK(out.pixels() == latest_pixels && out.metadata().source.sequence == 4);
    expect_pixels(out, 4);
    const auto counts = mailbox.counters();
    CHECK(counts.offered == 4 && counts.accepted == 4 && counts.replaced == 2);
    CHECK(counts.taken == 2 && counts.busy == 0 && counts.invalid == 0);
}

// The largest raster (1280x800) passes intact; one row more is refused before any pixel is read.
void test_maximum_storage_raster() {
    Mailbox mailbox(kMaxPixels);
    Frame out(kMaxPixels);
    std::vector<std::uint16_t> pixels(kMaxPixels, 65535);
    pixels.front() = 0;
    pixels.back() = 123;
    const auto stream = mailbox.begin_stream();
    auto view = sample(stream, 1, pixels.data());
    view.metadata.intrinsics.width = 1280;
    view.metadata.intrinsics.height = 800;
    view.metadata.source.depth_index = 29;  // Color/depth indices need not match.
    view.bytes = pixels.size() * sizeof(std::uint16_t);
    CHECK(mailbox.try_offer(view) == Offer::Accepted);
    CHECK(mailbox.try_take(out) == Take::Taken);
    CHECK(out.pixel_count() == kMaxPixels && out.capacity_pixels() == kMaxPixels);
    CHECK(std::equal(pixels.begin(), pixels.end(), out.pixels()));
    CHECK(out.metadata().source.color_index == 0 && out.metadata().source.depth_index == 29);
    ++view.metadata.intrinsics.height;
    view.bytes += 1280 * sizeof(std::uint16_t);
    // Reject before touching the deliberately shorter underlying allocation.
    CHECK(mailbox.try_offer(view) == Offer::Invalid);
    CHECK(out.pixels()[kMaxPixels - 1] == 123);
}

// Bad metadata, sizes or source fields are refused without advancing the ordering; all-zero depth,
// an unknown distortion model and a raster smaller than capacity are accepted exactly.
void test_all_invalid_metadata_and_exact_raster() {
    Mailbox mailbox(small_capacity);
    Frame out(small_capacity);
    const auto stream = mailbox.begin_stream();
    std::array<std::uint16_t, small_capacity> pixels{};
    CHECK(mailbox.try_offer(sample(stream, 1, pixels.data())) == Offer::Accepted);
    const auto good = sample(stream, 2, pixels.data());
    std::uint64_t invalid_count = 0;
    const auto invalid = [&](auto mutation) {
        auto view = good;
        mutation(view);
        CHECK(mailbox.try_offer(view) == Offer::Invalid);
        ++invalid_count;
    };
    const auto nan = std::numeric_limits<float>::quiet_NaN();
    const auto infinity = std::numeric_limits<float>::infinity();
    invalid([](View& v) { v.pre_d2c = false; });
    invalid([](View& v) { v.pixels = nullptr; });
    invalid([](View& v) { v.bytes = 0; });
    invalid([](View& v) { --v.bytes; });
    invalid([](View& v) { ++v.bytes; });
    invalid([](View& v) { v.bytes = std::numeric_limits<std::size_t>::max(); });
    invalid([](View& v) { v.metadata.intrinsics.width = 0; });
    invalid([](View& v) { v.metadata.intrinsics.height = 0; });
    invalid([](View& v) { v.metadata.intrinsics.width = 5; });
    invalid([](View& v) {
        v.metadata.intrinsics.width = std::numeric_limits<std::uint32_t>::max();
        v.metadata.intrinsics.height = std::numeric_limits<std::uint32_t>::max();
    });
    for (const float bad : {0.0F, -1.0F, nan, infinity, -infinity}) {
        invalid([bad](View& v) { v.metadata.intrinsics.fx = bad; });
        invalid([bad](View& v) { v.metadata.intrinsics.fy = bad; });
        invalid([bad](View& v) { v.metadata.unit_m = bad; });
    }
    for (const float bad : {nan, infinity, -infinity}) {
        invalid([bad](View& v) { v.metadata.intrinsics.cx = bad; });
        invalid([bad](View& v) { v.metadata.intrinsics.cy = bad; });
        for (std::size_t index = 0; index < 8; ++index) {
            invalid([bad, index](View& v) { v.metadata.distortion.coefficients[index] = bad; });
        }
    }
    invalid([](View& v) { v.metadata.distortion.reported = false; });
    invalid([](View& v) { v.metadata.source.sequence = 0; });
    invalid([](View& v) { v.metadata.source.color_device_us = 0; });
    invalid([](View& v) { v.metadata.source.depth_device_us = 0; });
    invalid([](View& v) { v.metadata.source.host_ready_mono_ns = -1; });
    invalid([](View& v) {
        v.metadata.source.sequence = 100000;
        v.metadata.intrinsics.fx = 0.0F;
    });
    // Invalid high sequence and bad calibration never advance ordering state.
    CHECK(mailbox.try_offer(good) == Offer::Replaced);
    CHECK(mailbox.counters().invalid == invalid_count);
    CHECK(mailbox.try_take(out) == Take::Taken);
    CHECK(out.pixel_count() == small_capacity);
    expect_pixels(out, 0);  // All-zero depth remains an accepted raw observation.

    auto unknown = sample(stream, 3, pixels.data());
    unknown.metadata.distortion.model = std::numeric_limits<int>::max();
    CHECK(mailbox.try_offer(unknown) == Offer::Accepted);
    CHECK(mailbox.try_take(out) == Take::Taken);
    CHECK(out.metadata().distortion.model == std::numeric_limits<int>::max());

    auto compact = sample(stream, 4, pixels.data());
    compact.metadata.intrinsics.width = 2;
    compact.metadata.intrinsics.height = 1;
    compact.bytes = 2 * sizeof(std::uint16_t);
    pixels[0] = 71;
    pixels[1] = 65535;
    CHECK(mailbox.try_offer(compact) == Offer::Accepted);
    CHECK(mailbox.try_take(out) == Take::Taken);
    CHECK(out.pixel_count() == 2 && out.capacity_pixels() == small_capacity);
    CHECK(out.pixels()[0] == 71 && out.pixels()[1] == 65535);
}

// An offer must be newer in every source field (gaps allowed); a new stream discards the pending
// frame and refuses old-stream offers; end_stream makes both sides Inactive; generation exhaustion
// throws.
void test_ordering_gap_and_restart() {
    Mailbox mailbox(small_capacity);
    Frame out(small_capacity);
    std::array<std::uint16_t, small_capacity> pixels{};
    const auto first_stream = mailbox.begin_stream();
    auto initial = sample(first_stream, 1, pixels.data());
    initial.metadata.source.host_ready_mono_ns = 0;
    CHECK(initial.metadata.source.color_index == 0 && initial.metadata.source.depth_index == 0);
    CHECK(mailbox.try_offer(initial) == Offer::Accepted);
    CHECK(mailbox.try_take(out) == Take::Taken);
    const auto good = sample(first_stream, 2, pixels.data());
    std::uint64_t stale_count = 0;
    const auto stale = [&](auto mutation) {
        auto view = good;
        mutation(view);
        CHECK(mailbox.try_offer(view) == Offer::Stale);
        ++stale_count;
    };
    stale([](View& v) { v.metadata.source.sequence = 1; });
    stale([](View& v) { v.metadata.source.color_index = 0; });
    stale([](View& v) { v.metadata.source.depth_index = 0; });
    stale([](View& v) { v.metadata.source.color_device_us = 1000; });
    stale([](View& v) { v.metadata.source.depth_device_us = 1001; });
    stale([](View& v) { v.metadata.source.color_device_us = 999; });
    stale([](View& v) { v.metadata.source.depth_device_us = 1000; });
    stale([](View& v) { v.metadata.source.stream_id = 0; });
    stale([](View& v) { ++v.metadata.source.stream_id; });
    auto same_host_clock = good;
    same_host_clock.metadata.source.host_ready_mono_ns = 0;
    CHECK(mailbox.try_offer(same_host_clock) == Offer::Accepted);
    // Gaps need not imply delivery; a newer accepted source may replace pending.
    auto gap = sample(first_stream, 100, pixels.data());
    CHECK(mailbox.try_offer(gap) == Offer::Replaced);
    CHECK(mailbox.try_offer(good) == Offer::Stale);
    ++stale_count;
    auto backwards_host = sample(first_stream, 101, pixels.data());
    backwards_host.metadata.source.host_ready_mono_ns = gap.metadata.source.host_ready_mono_ns - 1;
    CHECK(mailbox.try_offer(backwards_host) == Offer::Stale);
    ++stale_count;
    auto backwards_index = sample(first_stream, 101, pixels.data());
    backwards_index.metadata.source.color_index = 1;
    CHECK(mailbox.try_offer(backwards_index) == Offer::Stale);
    ++stale_count;
    CHECK(mailbox.try_take(out) == Take::Taken);
    CHECK(out.metadata().source.sequence == 100);
    CHECK(mailbox.counters().stale == stale_count);

    CHECK(mailbox.try_offer(sample(first_stream, 101, pixels.data())) == Offer::Accepted);
    const auto second_stream = mailbox.begin_stream();
    CHECK(second_stream > first_stream && mailbox.active_stream() == second_stream);
    CHECK(mailbox.counters().discarded_on_reset == 1);
    CHECK(mailbox.try_take(out) == Take::Empty);
    CHECK(out.metadata().source.stream_id == first_stream);
    CHECK(out.metadata().source.stream_id != mailbox.active_stream());
    CHECK(mailbox.try_offer(sample(first_stream, 102, pixels.data())) == Offer::Stale);
    CHECK(mailbox.try_offer(sample(second_stream, 1, pixels.data())) == Offer::Accepted);
    mailbox.end_stream();
    CHECK(mailbox.active_stream() == 0 && mailbox.counters().discarded_on_reset == 2);
    const auto* held = out.pixels();
    CHECK(mailbox.try_take(out) == Take::Inactive);
    CHECK(out.pixels() == held && out.metadata().source.stream_id == first_stream);
    CHECK(mailbox.try_offer(sample(second_stream, 2, pixels.data())) == Offer::Inactive);
    mailbox.end_stream();
    CHECK(mailbox.counters().discarded_on_reset == 2);
    const auto third_stream = mailbox.begin_stream();
    CHECK(third_stream > second_stream);
    CHECK(mailbox.try_take(out) == Take::Empty);
    CHECK(mailbox.try_offer(sample(third_stream, 1, pixels.data())) == Offer::Accepted);
    CHECK(mailbox.try_take(out) == Take::Taken);
    CHECK(out.metadata().source.stream_id == third_stream);

    mailbox.end_stream();
    MailboxTestAccess::exhaust_generation(mailbox);
    bool exhausted = false;
    try { mailbox.begin_stream(); }
    catch (const std::overflow_error&) { exhausted = true; }
    CHECK(exhausted && mailbox.active_stream() == 0);
}

// While the lock is held both sides return Busy at once, the held frame is untouched, and a retry
// of the same sequence succeeds.
void test_deterministic_busy() {
    Mailbox mailbox(small_capacity);
    Frame out(small_capacity);
    std::array<std::uint16_t, small_capacity> pixels{};
    pixels.fill(7);
    const auto stream = mailbox.begin_stream();
    CHECK(mailbox.try_offer(sample(stream, 1, pixels.data())) == Offer::Accepted);
    CHECK(mailbox.try_take(out) == Take::Taken);
    const auto* held = out.pixels();
    auto held_lock = MailboxTestAccess::hold(mailbox);
    Offer offered = Offer::Invalid;
    Take taken = Take::Empty;
    std::thread contender([&] {
        offered = mailbox.try_offer(sample(stream, 2, pixels.data()));
        taken = mailbox.try_take(out);
    });
    contender.join();
    CHECK(offered == Offer::Busy && taken == Take::Busy);
    held_lock.unlock();
    CHECK(out.pixels() == held && out.metadata().source.sequence == 1);
    expect_pixels(out, 7);
    CHECK(mailbox.counters().busy == 2);
    // Busy must not poison the same sequence when it is retried.
    CHECK(mailbox.try_offer(sample(stream, 2, pixels.data())) == Offer::Accepted);
    CHECK(mailbox.try_take(out) == Take::Taken);
    CHECK(out.metadata().source.sequence == 2);
}

// Offer and take only swap two fixed buffers: 1000 frames, refusals and a stream restart allocate
// nothing.
void test_no_hot_path_allocations_or_storage_growth() {
    Mailbox mailbox(small_capacity);
    Frame out(small_capacity);
    Frame wrong(small_capacity + 1);
    std::array<std::uint16_t, small_capacity> pixels{};
    const auto* first_buffer = out.pixels();
    auto stream = mailbox.begin_stream();
    CHECK(mailbox.try_offer(sample(stream, 1, pixels.data())) == Offer::Accepted);
    CHECK(mailbox.try_take(out) == Take::Taken);
    const auto* second_buffer = out.pixels();
    CHECK(first_buffer != second_buffer);
    const auto before = allocation_count.load(std::memory_order_relaxed);
    for (std::uint64_t sequence = 2; sequence < 1000; ++sequence) {
        auto view = sample(stream, sequence, pixels.data());
        CHECK(successful(mailbox.try_offer(view)));
        CHECK(mailbox.try_offer(view) == Offer::Stale);
        view.pre_d2c = false;
        CHECK(mailbox.try_offer(view) == Offer::Invalid);
        CHECK(mailbox.try_take(wrong) == Take::Capacity);
        if (sequence % 3 == 0) {
            CHECK(mailbox.try_take(out) == Take::Taken);
            CHECK(mailbox.try_take(out) == Take::Empty);
        }
        CHECK(out.pixels() == first_buffer || out.pixels() == second_buffer);
        CHECK(out.capacity_pixels() == small_capacity);
        (void)mailbox.counters();
    }
    mailbox.end_stream();
    CHECK(mailbox.try_take(out) == Take::Inactive);
    stream = mailbox.begin_stream();
    CHECK(mailbox.try_offer(sample(stream, 1, pixels.data())) == Offer::Accepted);
    CHECK(mailbox.try_take(out) == Take::Taken);
    const auto after = allocation_count.load(std::memory_order_relaxed);
    CHECK(after == before);
}

// A producer and a consumer thread race 30000 frames: every taken frame is whole and newer than
// the last, the counters reconcile, and the last accepted frame is always taken.
void test_concurrent_no_torn_frames_and_final_drain() {
    constexpr std::size_t capacity = 4096;
    constexpr std::uint64_t attempts = 30000;
    Mailbox mailbox(capacity);
    Frame held(capacity);
    const auto stream = mailbox.begin_stream();
    std::atomic<bool> producer_done{false};
    std::uint64_t accepted = 0;
    std::uint64_t replaced = 0;
    std::uint64_t producer_busy = 0;
    std::uint64_t last_accepted = 0;
    std::uint64_t taken = 0;
    std::uint64_t consumer_busy = 0;
    std::uint64_t last_taken = 0;

    std::thread producer([&] {
        std::array<std::uint16_t, capacity> pixels{};
        for (std::uint64_t sequence = 1; sequence <= attempts; ++sequence) {
            pixels.fill(static_cast<std::uint16_t>(sequence));
            auto view = sample(stream, sequence, pixels.data());
            view.metadata.intrinsics.width = 64;
            view.metadata.intrinsics.height = 64;
            view.bytes = pixels.size() * sizeof(std::uint16_t);
            const auto result = mailbox.try_offer(view);
            if (successful(result)) {
                ++accepted;
                replaced += result == Offer::Replaced ? 1U : 0U;
                last_accepted = sequence;
            } else {
                CHECK(result == Offer::Busy);
                ++producer_busy;
            }
        }
        producer_done.store(true, std::memory_order_release);
    });
    std::thread consumer([&] {
        for (;;) {
            // Check done before take: Empty after observing done is a final
            // drain, unlike Empty immediately before the producer's last offer.
            const bool done = producer_done.load(std::memory_order_acquire);
            const auto result = mailbox.try_take(held);
            if (result == Take::Taken) {
                ++taken;
                const auto& source = held.metadata().source;
                CHECK(source.stream_id == stream && source.sequence > last_taken);
                CHECK(source.color_index + 1 == source.sequence);
                CHECK(source.depth_index + 1 == source.sequence);
                CHECK(source.color_device_us == source.sequence * 1000);
                CHECK(source.depth_device_us == source.sequence * 1000 + 1);
                CHECK(source.host_ready_mono_ns == static_cast<std::int64_t>(source.sequence * 1000000));
                CHECK(held.pixel_count() == capacity && held.capacity_pixels() == capacity);
                CHECK(held.metadata().distortion.coefficients[7] == 8.0F);
                last_taken = source.sequence;
                expect_pixels(held, static_cast<std::uint16_t>(last_taken));
                // Read again after allowing more producer activity. Processing
                // this held buffer must not share the mailbox's pending raster.
                std::this_thread::yield();
                expect_pixels(held, static_cast<std::uint16_t>(last_taken));
            } else if (result == Take::Busy) {
                ++consumer_busy;
            } else {
                CHECK(result == Take::Empty);
                if (done) {
                    break;
                }
                std::this_thread::yield();
            }
        }
    });
    producer.join();
    consumer.join();
    const auto counts = mailbox.counters();
    CHECK(counts.offered == attempts);
    CHECK(counts.accepted == accepted && counts.replaced == replaced);
    CHECK(counts.taken == taken && counts.busy == producer_busy + consumer_busy);
    CHECK(accepted + producer_busy == attempts);
    CHECK(accepted == taken + replaced);
    CHECK(taken > 0 && last_taken == last_accepted);
    CHECK(counts.invalid == 0 && counts.stale == 0 && counts.inactive == 0);
    CHECK(mailbox.try_take(held) == Take::Empty);
}

}  // namespace

int main() {
    test_construction_and_capacity();
    test_copy_lifetime_latest_and_held_frame();
    test_maximum_storage_raster();
    test_all_invalid_metadata_and_exact_raster();
    test_ordering_gap_and_restart();
    test_deterministic_busy();
    test_no_hot_path_allocations_or_storage_growth();
    test_concurrent_no_torn_frames_and_final_drain();
    std::cout << "native depth mailbox: ownership, bounded storage, metadata, ordering, restart, contention and concurrency PASS\n";
}
