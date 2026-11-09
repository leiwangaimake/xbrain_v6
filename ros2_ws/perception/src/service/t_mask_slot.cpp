/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: t_mask_slot.cpp
 * Brief: TMaskSlot writer (Objects side) and reader (Profile side).
 *
 * Description:
 * Implements t_mask_slot.hpp. The writer is a sequence lock over one of two buffers: sequence to
 * odd, release fence, relaxed stores of the header and the raster words, sequence to even with
 * release, then the publication counter with release. The reader loads the publication counter
 * and the buffer's sequence with acquire, copies with relaxed loads, issues an acquire fence and
 * re-reads the sequence: equal and even means the copy is one complete publication. This is the
 * standard C++ seqlock form (every shared word atomic), so no torn copy is ever used and the
 * reader never blocks the writer or waits for it.
 */
#include "service/t_mask_slot.hpp"

#include <chrono>
#include <cstring>
#include <ctime>
#include <type_traits>

namespace perception::service {

namespace {

// The stamp travels through the slot as plain 64-bit words (every shared word must be atomic for
// the seqlock to be data-race free), and each raster row is copied as whole words, so both
// layouts are checked at compile time.
static_assert(std::is_trivially_copyable_v<geometry::CaptureStamp>, "the stamp is copied as words");
static_assert(TMaskSlot::kBytes % 8 == 0 && TMaskSlot::kWidth % 8 == 0, "rows are whole words");

// CLOCK_MONOTONIC, the clock of every r6 age and of the Status cut (rns_system_clock.cpp); the
// Status owner compares these publication times with its observation instant.
std::int64_t monotonic_now_ns() noexcept {
#if defined(__linux__)
    timespec ts{};
    if (::clock_gettime(CLOCK_MONOTONIC, &ts) != 0) return -1;
    return static_cast<std::int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
#else
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
#endif
}

// Eight source bytes -> eight bytes of 0 or 1 (1 where the source byte is nonzero). The shifts
// fold each byte's bits into its lowest bit; spill into higher bits of the neighbour byte is
// masked away.
inline std::uint64_t binarise_word(std::uint64_t v) noexcept {
    v |= v >> 4;
    v |= v >> 2;
    v |= v >> 1;
    return v & 0x0101010101010101ULL;
}

}  // namespace

// The reader's private copy is allocated once on the Profile thread; latest() only copies into it.
TMaskSlot::Snapshot::Snapshot() : pixels(kBytes, 0) {}

// Two buffers: while the writer fills one, the reader can still copy the other complete one, so a
// steady 25 Hz writer rarely forces a retry. All storage is allocated here, none per frame.
TMaskSlot::TMaskSlot() {
    for (auto& buffer : buffers_) buffer.words = std::make_unique<std::atomic<std::uint64_t>[]>(kWords);
}

template <typename Fill>
bool TMaskSlot::write(const geometry::CaptureStamp& stamp, std::uint64_t projection_revision, bool empty,
                      Fill fill) noexcept {
    // One producer by contract; a second concurrent writer would break the sequence lock, so it is
    // refused (counted) instead.
    if (writing_.exchange(true, std::memory_order_acquire)) {
        refused_concurrent_.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    // Publication n goes to buffer n % 2, i.e. never into the buffer of the newest complete
    // publication the readers are being pointed at.
    const std::uint64_t publication = publications_.load(std::memory_order_relaxed) + 1;
    Buffer& buffer = buffers_[publication & 1U];
    const std::uint64_t sequence = buffer.sequence.load(std::memory_order_relaxed);
    buffer.sequence.store(sequence + 1, std::memory_order_relaxed);  // odd: being written
    std::atomic_thread_fence(std::memory_order_release);
    // Header: publication number, empty flag, projection revision, then the capture stamp.
    std::array<std::uint64_t, kStampWords> stamp_words{};
    std::memcpy(stamp_words.data(), &stamp, sizeof(stamp));
    buffer.header[0].store(publication, std::memory_order_relaxed);
    buffer.header[1].store(empty ? 1U : 0U, std::memory_order_relaxed);
    buffer.header[2].store(projection_revision, std::memory_order_relaxed);
    for (std::size_t i = 0; i < kStampWords; ++i) buffer.header[3 + i].store(stamp_words[i], std::memory_order_relaxed);
    fill(buffer.words.get());
    buffer.sequence.store(sequence + 2, std::memory_order_release);  // even: complete
    publications_.store(publication, std::memory_order_release);     // the pointer swap
    // Publication times feed Status (traversable_seg_available over a 10 s window); an empty
    // mask is a publication (accepted, issues.md 3.2 (1), 2026-10-08), and is still counted apart.
    const std::int64_t now = monotonic_now_ns();
    last_publish_ns_.store(now, std::memory_order_release);
    if (!empty) last_nonempty_ns_.store(now, std::memory_order_release);
    writing_.store(false, std::memory_order_release);
    return true;
}

bool TMaskSlot::publish(const std::uint8_t* pixels, std::uint32_t width, std::uint32_t height,
                        std::size_t stride_bytes, const geometry::CaptureStamp& stamp,
                        std::uint64_t projection_revision) noexcept {
    // Only the full colour raster is accepted (the projection maps onto exactly this raster);
    // a padded row stride is fine, it is read row by row.
    if (pixels == nullptr || width != kWidth || height != kHeight || stride_bytes < width) {
        refused_input_.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    // The union arrives as 0/255; it is stored as 0/1 so the T owner's "> 1 is invalid" check and
    // its "== 1 is traversable" lookup read it directly. Eight pixels per word.
    const bool ok = write(stamp, projection_revision, false, [&](std::atomic<std::uint64_t>* words) {
        constexpr std::size_t kRowWords = kWidth / 8;
        for (std::size_t y = 0; y < kHeight; ++y) {
            const std::uint8_t* row = pixels + y * stride_bytes;
            std::atomic<std::uint64_t>* out = words + y * kRowWords;
            for (std::size_t w = 0; w < kRowWords; ++w) {
                std::uint64_t v = 0;
                std::memcpy(&v, row + w * 8, 8);
                out[w].store(binarise_word(v), std::memory_order_relaxed);
            }
        }
    });
    if (ok) published_.fetch_add(1, std::memory_order_relaxed);
    return ok;
}

bool TMaskSlot::publish_empty(const geometry::CaptureStamp& stamp, std::uint64_t projection_revision) noexcept {
    const bool ok = write(stamp, projection_revision, true, [](std::atomic<std::uint64_t>* words) {
        for (std::size_t w = 0; w < kWords; ++w) words[w].store(0, std::memory_order_relaxed);
    });
    if (ok) published_empty_.fetch_add(1, std::memory_order_relaxed);
    return ok;
}

const geometry::RawTMask* TMaskSlot::latest(Snapshot& into, const geometry::CaptureStamp& depth,
                                            std::uint64_t projection_revision) noexcept {
    // Never published (T not started yet, or no inference frame so far): this frame has no T.
    const std::uint64_t newest = publications_.load(std::memory_order_acquire);
    if (newest == 0) {
        missing_.fetch_add(1, std::memory_order_relaxed);
        return nullptr;
    }
    // A new publication since the last copy: copy it (bounded retries, never waiting for the
    // writer). Otherwise the private copy is already the newest one and is reused as is.
    if (into.publication != newest) {
        bool copied = false;
        for (int attempt = 0; attempt < kCopyAttempts && !copied; ++attempt) {
            // Re-read the newest publication on every attempt: after a retry the writer may have
            // finished a newer one, and the newest complete buffer is the one to copy.
            const std::uint64_t n = publications_.load(std::memory_order_acquire);
            const Buffer& buffer = buffers_[n & 1U];
            // Acquire pairs with the writer's release store of the even sequence: everything the
            // writer stored before completing this buffer is visible to the loads below.
            const std::uint64_t before = buffer.sequence.load(std::memory_order_acquire);
            if ((before & 1U) != 0U) {  // the writer is in this buffer right now
                torn_retries_.fetch_add(1, std::memory_order_relaxed);
                continue;
            }
            // Copy header and raster with relaxed loads into private storage. If the writer
            // starts on this buffer meanwhile, the copy may be torn; the sequence re-check below
            // detects that, and a torn copy is never interpreted.
            std::array<std::uint64_t, kHeaderWords> header{};
            for (std::size_t i = 0; i < kHeaderWords; ++i) header[i] = buffer.header[i].load(std::memory_order_relaxed);
            std::uint8_t* out = into.pixels.data();
            for (std::size_t w = 0; w < kWords; ++w) {
#ifdef T_MASK_SLOT_TEST_HOOK
                if (w == kWords / 2 && test_mid_copy_hook != nullptr) test_mid_copy_hook(test_hook_context);
#endif
                const std::uint64_t v = buffer.words[w].load(std::memory_order_relaxed);
                std::memcpy(out + w * 8, &v, 8);
            }
            // The acquire fence orders the copy before the sequence re-read: if the sequence is
            // unchanged, no write to this buffer overlapped the copy.
            std::atomic_thread_fence(std::memory_order_acquire);
            if (buffer.sequence.load(std::memory_order_relaxed) != before) {  // overwritten meanwhile
                torn_retries_.fetch_add(1, std::memory_order_relaxed);
                continue;
            }
            // One complete publication (possibly newer than `newest`: the header says which).
            into.publication = header[0];
            into.empty = header[1] != 0U;
            into.mask.projection_revision = header[2];
            std::array<std::uint64_t, kStampWords> stamp_words{};
            for (std::size_t i = 0; i < kStampWords; ++i) stamp_words[i] = header[3 + i];
            // CaptureStamp is trivially copyable (static_assert above); it only has default
            // member initialisers, which GCC's class-memaccess check would otherwise flag.
            std::memcpy(static_cast<void*>(&into.mask.stamp), stamp_words.data(), sizeof(into.mask.stamp));
            // The borrowed view the T owner reads: the raw (not eroded, not fused) class-92 union
            // on the full colour raster, densely packed, pointing into the private copy.
            into.mask.material = geometry::MaskMaterial::RawTraversable;
            into.mask.width = kWidth;
            into.mask.height = kHeight;
            into.mask.stride_bytes = kWidth;
            into.mask.bytes = kBytes;
            into.mask.pixels = into.pixels.data();
            copied = true;
        }
        if (!copied) {
            into.publication = 0;  // the storage may hold a torn copy: never reuse it
            torn_giveup_.fetch_add(1, std::memory_order_relaxed);
            return nullptr;
        }
    } else {
        reused_.fetch_add(1, std::memory_order_relaxed);
    }
    // Identity checks on the private copy. Refused masks leave this frame without T. They are the
    // owner's own identity rules (same stream and clock epoch, the projection the owner was built
    // with, not later than the depth frame, the same or an older frame set), applied here so the
    // owner is never handed a mask it would reject, and each refusal is counted by cause.
    const auto& stamp = into.mask.stamp;
    if (stamp.source.stream_id != depth.source.stream_id) {
        refused_stream_.fetch_add(1, std::memory_order_relaxed);
        return nullptr;
    }
    if (stamp.clock_epoch != depth.clock_epoch) {
        refused_epoch_.fetch_add(1, std::memory_order_relaxed);
        return nullptr;
    }
    if (into.mask.projection_revision != projection_revision) {
        refused_revision_.fetch_add(1, std::memory_order_relaxed);
        return nullptr;
    }
    if (stamp.capture_mono_ns < 0 || stamp.capture_mono_ns > depth.capture_mono_ns) {
        refused_future_.fetch_add(1, std::memory_order_relaxed);
        return nullptr;
    }
    if (!geometry::source_older_or_same(stamp.source, depth.source)) {
        refused_order_.fetch_add(1, std::memory_order_relaxed);
        return nullptr;
    }
    taken_.fetch_add(1, std::memory_order_relaxed);
    return &into.mask;
}

// A relaxed snapshot of every counter for the periodic SERVICE_T_SUMMARY line; the counters are
// independent, so a summary taken while both threads run may mix neighbouring instants.
TMaskSlot::Counters TMaskSlot::counters() const noexcept {
    Counters c;
    c.published = published_.load(std::memory_order_relaxed);
    c.published_empty = published_empty_.load(std::memory_order_relaxed);
    c.refused_input = refused_input_.load(std::memory_order_relaxed);
    c.refused_concurrent = refused_concurrent_.load(std::memory_order_relaxed);
    c.taken = taken_.load(std::memory_order_relaxed);
    c.reused = reused_.load(std::memory_order_relaxed);
    c.missing = missing_.load(std::memory_order_relaxed);
    c.refused_stream = refused_stream_.load(std::memory_order_relaxed);
    c.refused_epoch = refused_epoch_.load(std::memory_order_relaxed);
    c.refused_revision = refused_revision_.load(std::memory_order_relaxed);
    c.refused_future = refused_future_.load(std::memory_order_relaxed);
    c.refused_order = refused_order_.load(std::memory_order_relaxed);
    c.torn_retries = torn_retries_.load(std::memory_order_relaxed);
    c.torn_giveup = torn_giveup_.load(std::memory_order_relaxed);
    return c;
}

}  // namespace perception::service
