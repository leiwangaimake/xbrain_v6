/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rns_zenoh_alloc.cpp
 * Brief: A19-ALLOC-1 on the real RT transport and zenoh-c 1.9.0: heap calls per put, ours and zenoh-c's apart.
 *
 * Description:
 * What it checks (19 section 14 A19-ALLOC-1, P19-3; acceptance rules agreed 2026-10-14). Three
 * ZenohRtTransport publishers as the service builds them (real key/QoS, max_packet 262144) put
 * packets of the measured sizes (Profile 13200, Objects 11900, Status 600 bytes) through a TCP
 * loopback to a subscriber of their own. Each publisher: 50 warm-up puts, then 1000 counted.
 * Our code must make 0 heap allocations and 0 frees on the publishing thread, with 0 malloc
 * fallbacks and 0 bad releases of the packet pool. zenoh-c's own heap calls on that thread are
 * printed in a separate column, not counted as ours (they are the middleware's half of ALLOC-1);
 * at least one per put is required as the positive control: a hook that does not see the Rust
 * allocator would otherwise report a false 0. Then the same with Profile and Objects on two
 * threads at once. Then the pool's exhaustion and late release: z_publisher_put keeps the
 * payloads instead of sending them (hold mode), four puts hold all four slots, the fifth takes
 * the malloc fallback (exactly one allocation of ours, still Accepted, in no held slot), the
 * held payloads are dropped on another thread, and the next put is in slot 0 again with no
 * allocation. Then informational only: 100 kB packets (zenoh-c fragments them) and whether
 * their slots were still held when the put returned. Last, the pool's lifetime (outlive phase):
 * a fourth transport (a second Profile publisher) is destroyed while zenoh-c still holds two of
 * its payloads, which are then dropped on another thread; its destructor must not free the
 * pool, the first release must not either, and the last release must free the pool's bytes on
 * that thread (seen by free()).
 *
 * How it counts. malloc, calloc, realloc, free, posix_memalign, aligned_alloc, memalign, valloc
 * and pvalloc are replaced in this executable and forwarded to glibc's __libc_* entry points
 * (glibc manual, "Replacing malloc"); libstdc++'s operator new and zenoh-c's Rust allocator both
 * end there. Counters are thread_local and count only while the measuring thread has switched
 * them on. Every zenoh-c function the put path calls is wrapped at link time (-Wl,--wrap, see
 * tests/rns_host/CMakeLists.txt): a heap call made inside a wrapped call goes to zenoh-c's
 * column, any other to ours. The deleter zenoh-c runs inside a wrapped call (our slot release,
 * or free() of a fallback buffer) is therefore in zenoh-c's column; the pure pool test
 * (test_rns_packet_slot_pool.cpp) covers the release itself.
 *
 * Before the pool (the transport of 0cbd03a) this test must fail: every put mallocs one packet
 * copy, so ours is 1000 per publisher ("always red before the implementation", A19-ALLOC-1). It
 * compiles against that transport too: the pool checks need pool_counters() and are skipped
 * without it.
 * Scope. Linux/glibc only, never under sanitizers (they replace malloc themselves); built only
 * with RNS_ZENOH_ROOT. zenoh-c's own threads are not counted. Not a timing acceptance.
 */

#include "output/rns_zenoh_transport.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <type_traits>
#include <vector>

#if !defined(__linux__) || !defined(__GLIBC__)
#error "test_rns_zenoh_alloc replaces glibc's malloc: Linux/glibc only (see tests/rns_host/CMakeLists.txt)"
#endif

// ---- heap counting at the malloc layer (glibc) ----
namespace {
// Plain thread_local scalars of this executable: reading them never allocates.
thread_local bool t_measuring = false;
thread_local int t_native_depth = 0;  // > 0 while inside a wrapped zenoh-c call
thread_local std::uint64_t t_ours_alloc = 0, t_ours_free = 0, t_native_alloc = 0, t_native_free = 0;
thread_local std::size_t t_ours_last_bytes = 0;
inline void note_alloc(std::size_t bytes) noexcept {
    if (!t_measuring) return;
    if (t_native_depth > 0) {
        ++t_native_alloc;
    } else {
        ++t_ours_alloc;
        t_ours_last_bytes = bytes;
    }
}
inline void note_free() noexcept {
    if (!t_measuring) return;
    if (t_native_depth > 0) ++t_native_free;
    else ++t_ours_free;
}
// Outlive phase: the pointers this thread frees while recording is on (the first kFreedCapacity
// are kept, the count goes on).
constexpr std::size_t kFreedCapacity = 256;
thread_local bool t_record_frees = false;
thread_local std::size_t t_freed_count = 0;
thread_local void* t_freed[kFreedCapacity];
inline void note_freed_pointer(void* p) noexcept {
    if (!t_record_frees) return;
    if (t_freed_count < kFreedCapacity) t_freed[t_freed_count] = p;
    ++t_freed_count;
}
}  // namespace

extern "C" {
void* __libc_malloc(std::size_t size) noexcept;
void* __libc_calloc(std::size_t count, std::size_t size) noexcept;
void* __libc_realloc(void* old, std::size_t size) noexcept;
void* __libc_memalign(std::size_t alignment, std::size_t size) noexcept;
void* __libc_valloc(std::size_t size) noexcept;
void* __libc_pvalloc(std::size_t size) noexcept;
void __libc_free(void* p) noexcept;

void* malloc(std::size_t size) noexcept {
    note_alloc(size);
    return __libc_malloc(size);
}
void* calloc(std::size_t count, std::size_t size) noexcept {
    note_alloc(count * size);
    return __libc_calloc(count, size);
}
// Any realloc counts as one allocation (it may move the block).
void* realloc(void* old, std::size_t size) noexcept {
    note_alloc(size);
    return __libc_realloc(old, size);
}
void free(void* p) noexcept {
    if (p) {
        note_free();
        note_freed_pointer(p);
    }
    __libc_free(p);
}
void* memalign(std::size_t alignment, std::size_t size) noexcept {
    note_alloc(size);
    return __libc_memalign(alignment, size);
}
void* aligned_alloc(std::size_t alignment, std::size_t size) noexcept {
    note_alloc(size);
    return __libc_memalign(alignment, size);
}
int posix_memalign(void** out, std::size_t alignment, std::size_t size) noexcept {
    if (alignment == 0 || (alignment & (alignment - 1)) != 0 || alignment % sizeof(void*) != 0) return EINVAL;
    note_alloc(size);
    void* const p = __libc_memalign(alignment, size);
    if (!p) return ENOMEM;
    *out = p;
    return 0;
}
void* valloc(std::size_t size) noexcept {
    note_alloc(size);
    return __libc_valloc(size);
}
void* pvalloc(std::size_t size) noexcept {
    note_alloc(size);
    return __libc_pvalloc(size);
}
}  // extern "C"
// ---- end of heap counting ----

// ---- zenoh-c entry points of the put path, wrapped at link time ----
namespace {
struct NativeScope {
    NativeScope() noexcept { ++t_native_depth; }
    ~NativeScope() { --t_native_depth; }
    NativeScope(const NativeScope&) = delete;
    NativeScope& operator=(const NativeScope&) = delete;
};
// Hold mode (test thread only): z_publisher_put keeps the payload instead of sending it.
constexpr std::size_t kHoldCapacity = 8;
bool hold_active = false;
std::size_t hold_count = 0;
std::array<z_owned_bytes_t, kHoldCapacity> held_payloads;
}  // namespace

extern "C" {
z_result_t __real_z_bytes_from_buf(z_owned_bytes_t*, uint8_t*, size_t, void (*)(void*, void*), void*);
z_result_t __real_z_publisher_put(const z_loaned_publisher_t*, z_moved_bytes_t*, z_publisher_put_options_t*);
void __real_z_bytes_drop(z_moved_bytes_t*);
void __real_z_internal_bytes_null(z_owned_bytes_t*);
void __real_z_publisher_put_options_default(z_publisher_put_options_t*);
const z_loaned_publisher_t* __real_z_publisher_loan(const z_owned_publisher_t*);

z_result_t __wrap_z_bytes_from_buf(z_owned_bytes_t* bytes, uint8_t* data, size_t len,
                                   void (*deleter)(void*, void*), void* context) {
    NativeScope scope;
    return __real_z_bytes_from_buf(bytes, data, len, deleter, context);
}
z_result_t __wrap_z_publisher_put(const z_loaned_publisher_t* publisher, z_moved_bytes_t* payload,
                                  z_publisher_put_options_t* options) {
    NativeScope scope;
    if (!hold_active) return __real_z_publisher_put(publisher, payload, options);
    if (hold_count == held_payloads.size()) {
        __real_z_bytes_drop(payload);
        return -1;
    }
    // As z_bytes_take: move the owned value out, leave a gravestone (zenoh_commons.h z_moved_bytes_t).
    held_payloads[hold_count++] = payload->_this;
    __real_z_internal_bytes_null(&payload->_this);
    return 0;  // the put options of the transport own nothing (defaults)
}
void __wrap_z_bytes_drop(z_moved_bytes_t* bytes) {
    NativeScope scope;
    __real_z_bytes_drop(bytes);
}
void __wrap_z_internal_bytes_null(z_owned_bytes_t* bytes) {
    NativeScope scope;
    __real_z_internal_bytes_null(bytes);
}
void __wrap_z_publisher_put_options_default(z_publisher_put_options_t* options) {
    NativeScope scope;
    __real_z_publisher_put_options_default(options);
}
const z_loaned_publisher_t* __wrap_z_publisher_loan(const z_owned_publisher_t* publisher) {
    NativeScope scope;
    return __real_z_publisher_loan(publisher);
}
}  // extern "C"
// ---- end of wrapped entry points ----

namespace {
using namespace perception::rns;
using Steady = std::chrono::steady_clock;
constexpr std::size_t kWarmup = 50, kCounted = 1000;
constexpr std::size_t kPacketCapacity = 262144;  // as perception_service_main.cpp kPacketCapacity
constexpr std::size_t kProfileBytes = 13200, kObjectsBytes = 11900, kStatusBytes = 600;
constexpr std::size_t kFragmentedBytes = 100000, kFragmentedPuts = 20;
constexpr auto kWait = std::chrono::seconds(5);

void require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}

// The transport with the packet pool (A19-ALLOC-1) has pool_counters(); 0cbd03a's has not.
template <class T, class = void>
struct HasPool : std::false_type {};
template <class T>
struct HasPool<T, std::void_t<decltype(std::declval<const T&>().pool_counters())>> : std::true_type {};

std::string ephemeral_loopback_endpoint() {
    const int socket = ::socket(AF_INET, SOCK_STREAM, 0);
    require(socket >= 0, "create loopback port reservation socket");
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = 0;  // The OS selects the port; no production port probe.
    if (::bind(socket, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
        ::close(socket);
        throw std::runtime_error("bind ephemeral loopback port");
    }
    socklen_t size = sizeof(address);
    if (::getsockname(socket, reinterpret_cast<sockaddr*>(&address), &size) != 0) {
        ::close(socket);
        throw std::runtime_error("read selected loopback port");
    }
    const auto port = ntohs(address.sin_port);
    ::close(socket);
    require(port != 0, "OS selected a nonzero port");
    return "tcp/127.0.0.1:" + std::to_string(port);
}

// The subscriber only has to exist (so zenoh-c routes and encodes for real); it counts samples.
std::atomic<std::uint64_t> received{0};
std::atomic<bool> receiver_dropped{false};
void on_sample(z_loaned_sample_t*, void*) noexcept { received.fetch_add(1, std::memory_order_relaxed); }
void on_drop(void*) noexcept { receiver_dropped.store(true); }

class Receiver {
public:
    Receiver(const std::string& endpoint, const std::string& pattern) {
        const std::string config_json =
            "{mode:'peer',listen:{endpoints:['" + endpoint +
            "']},connect:{endpoints:[]},scouting:{multicast:{enabled:false},gossip:{enabled:true}},"
            "transport:{shared_memory:{enabled:false}}}";
        z_owned_config_t config{};
        require(zc_config_from_str(&config, config_json.c_str()) == 0, "parse receiver config");
        if (z_open(&session_, z_move(config), nullptr) != 0) {
            z_drop(z_move(session_));
            throw std::runtime_error("open receiver loopback listener");
        }
        session_open_ = true;
        z_view_keyexpr_t key{};
        if (z_view_keyexpr_from_str(&key, pattern.c_str()) != 0) {
            close();
            throw std::runtime_error("construct receiver key");
        }
        z_owned_closure_sample_t callback{};
        z_closure_sample(&callback, on_sample, on_drop, nullptr);
        z_subscriber_options_t options{};
        z_subscriber_options_default(&options);
        options.allowed_origin = Z_LOCALITY_REMOTE;
        if (z_declare_subscriber(z_loan(session_), &subscriber_, z_loan(key), z_move(callback), &options) != 0) {
            close();
            throw std::runtime_error("declare receiver subscriber");
        }
        subscriber_open_ = true;
    }
    ~Receiver() { close(); }
    Receiver(const Receiver&) = delete;
    Receiver& operator=(const Receiver&) = delete;
    void close() noexcept {
        if (subscriber_open_) { z_drop(z_move(subscriber_)); subscriber_open_ = false; }
        if (session_open_) { z_drop(z_move(session_)); session_open_ = false; }
    }
private:
    z_owned_session_t session_{};
    z_owned_subscriber_t subscriber_{};
    bool session_open_ = false, subscriber_open_ = false;
};

// Patches one byte of the private copy, as the publisher's finalizer patches the stamp.
struct OneBytePatch final : PacketFinalizer {
    std::uint8_t mark = 0;
    bool finalize(char* data, std::size_t size) noexcept override {
        if (size < 2) return false;
        data[1] = static_cast<char>('0' + (++mark % 10));
        return true;
    }
};

// A JSON-shaped packet of exactly `bytes` bytes; bytes 2..9 carry a marker the test can set.
// mark(), marker_of() and held_data() serve the hold phase only, which needs the pool: against
// the transport of 0cbd03a they are unused (hence maybe_unused, for -Werror).
std::vector<char> packet_of(std::size_t bytes) {
    std::vector<char> p(bytes, 'x');
    p.front() = '{';
    p.back() = '}';
    return p;
}
[[maybe_unused]] void mark(std::vector<char>& packet, std::uint64_t marker) { std::memcpy(packet.data() + 2, &marker, sizeof(marker)); }
[[maybe_unused]] std::uint64_t marker_of(const std::uint8_t* data) {
    std::uint64_t marker = 0;
    std::memcpy(&marker, data + 2, sizeof(marker));
    return marker;
}

void reset_counts() noexcept {
    t_ours_alloc = t_ours_free = t_native_alloc = t_native_free = 0;
    t_ours_last_bytes = 0;
}

struct Stats {
    const char* phase = "";
    const char* topic = "";
    std::size_t bytes = 0, counted = 0;
    std::uint64_t ours_alloc = 0, ours_free = 0, native_alloc = 0, native_free = 0;
    std::size_t ours_last_bytes = 0;
    std::uint32_t native_alloc_min = 0, native_alloc_median = 0, native_alloc_max = 0;
    std::uint32_t native_free_min = 0, native_free_median = 0, native_free_max = 0;
    std::uint64_t accepted = 0, not_accepted = 0;
    bool pool = false;  // the transport has the packet pool
    std::uint64_t pool_fallbacks = 0, pool_bad_releases = 0, held_at_return = 0;
};

void min_median_max(std::array<std::uint32_t, kCounted>& values, std::size_t n,
                    std::uint32_t& lo, std::uint32_t& mid, std::uint32_t& hi) {
    std::sort(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(n));
    lo = values[0]; mid = values[n / 2]; hi = values[n - 1];
}

// 50 warm-up puts, then `counted` puts with the heap counters of this thread on.
template <class Transport>
Stats steady(const char* phase, const char* topic, Transport& transport, const std::string& key,
             PublishQos qos, std::size_t bytes, std::size_t counted = kCounted) {
    require(counted > 0 && counted <= kCounted, "counted puts within the per-put arrays");
    auto packet = packet_of(bytes);
    OneBytePatch finalizer;
    Stats s;
    s.phase = phase; s.topic = topic; s.bytes = bytes; s.counted = counted;
    for (std::size_t i = 0; i < kWarmup; ++i)
        require(transport.put_finalized(key, packet.data(), packet.size(), qos, finalizer).outcome != PutOutcome::Rejected,
                "warm-up put not rejected");
    std::array<std::uint32_t, kCounted> per_alloc{}, per_free{};
    // Used only with the pool (maybe_unused: against 0cbd03a's transport -Werror must still pass).
    [[maybe_unused]] std::uint64_t fallback_puts = 0, held_at_return = 0, fallbacks_before = 0;
    if constexpr (HasPool<Transport>::value) fallbacks_before = transport.pool_counters().fallbacks;
    reset_counts();
    t_measuring = true;
    for (std::size_t i = 0; i < counted; ++i) {
        const auto alloc_before = t_native_alloc, free_before = t_native_free;
        const auto receipt = transport.put_finalized(key, packet.data(), packet.size(), qos, finalizer);
        per_alloc[i] = static_cast<std::uint32_t>(t_native_alloc - alloc_before);
        per_free[i] = static_cast<std::uint32_t>(t_native_free - free_before);
        if (receipt.outcome == PutOutcome::Accepted) ++s.accepted;
        else ++s.not_accepted;
        if constexpr (HasPool<Transport>::value) {
            if (transport.last_audit().pool_fallback) ++fallback_puts;
            if (transport.pool_counters().held != 0) ++held_at_return;
        }
    }
    t_measuring = false;
    s.ours_alloc = t_ours_alloc; s.ours_free = t_ours_free;
    s.native_alloc = t_native_alloc; s.native_free = t_native_free;
    s.ours_last_bytes = t_ours_last_bytes;
    min_median_max(per_alloc, counted, s.native_alloc_min, s.native_alloc_median, s.native_alloc_max);
    min_median_max(per_free, counted, s.native_free_min, s.native_free_median, s.native_free_max);
    if constexpr (HasPool<Transport>::value) {
        const auto c = transport.pool_counters();
        s.pool = true;
        s.pool_fallbacks = c.fallbacks - fallbacks_before;
        s.pool_bad_releases = c.bad_releases;
        s.held_at_return = held_at_return;
        require(fallback_puts == s.pool_fallbacks, "audit pool_fallback flags agree with the counter");
    }
    return s;
}

void print(const Stats& s) {
    std::cout << "{\"test\":\"rns_zenoh_alloc\",\"phase\":\"" << s.phase << "\",\"topic\":\"" << s.topic
              << "\",\"bytes\":" << s.bytes << ",\"warmup\":" << kWarmup << ",\"counted\":" << s.counted
              << ",\"accepted\":" << s.accepted << ",\"not_accepted\":" << s.not_accepted
              << ",\"ours_alloc\":" << s.ours_alloc << ",\"ours_free\":" << s.ours_free
              << ",\"ours_last_alloc_bytes\":" << s.ours_last_bytes
              << ",\"zenoh_alloc\":" << s.native_alloc << ",\"zenoh_free\":" << s.native_free
              << ",\"zenoh_alloc_per_put_min_median_max\":[" << s.native_alloc_min << ',' << s.native_alloc_median
              << ',' << s.native_alloc_max << "],\"zenoh_free_per_put_min_median_max\":[" << s.native_free_min << ','
              << s.native_free_median << ',' << s.native_free_max << "],\"pool\":" << (s.pool ? "true" : "false");
    if (s.pool)
        std::cout << ",\"pool_fallbacks\":" << s.pool_fallbacks << ",\"pool_bad_releases\":" << s.pool_bad_releases
                  << ",\"slot_still_held_at_put_return\":" << s.held_at_return;
    std::cout << "}\n";
}

// The acceptance of one steady phase; every failure is collected so a red run shows all of them.
void judge(const Stats& s, std::vector<std::string>& failures) {
    const std::string at = std::string(s.phase) + "/" + s.topic + ": ";
    if (s.ours_alloc != 0) failures.push_back(at + "our code allocated " + std::to_string(s.ours_alloc) + " times (want 0)");
    if (s.ours_free != 0) failures.push_back(at + "our code freed " + std::to_string(s.ours_free) + " times (want 0)");
    if (s.native_alloc < s.counted)
        failures.push_back(at + "positive control: zenoh-c column " + std::to_string(s.native_alloc) +
                           " allocations for " + std::to_string(s.counted) + " puts (want >= 1 per put); the hook is blind");
    if (s.accepted != s.counted) failures.push_back(at + std::to_string(s.not_accepted) + " puts not Accepted");
    if (!s.pool) {
        failures.push_back(at + "transport has no packet pool (pre-ALLOC-1)");
        return;
    }
    if (s.pool_fallbacks != 0) failures.push_back(at + std::to_string(s.pool_fallbacks) + " malloc fallbacks (want 0)");
    if (s.pool_bad_releases != 0) failures.push_back(at + std::to_string(s.pool_bad_releases) + " bad slot releases (want 0)");
}

// Bounded wait until zenoh-c has returned every slot of this transport.
template <class Transport>
bool all_slots_returned(Transport& transport) {
    const auto deadline = Steady::now() + kWait;
    while (transport.pool_counters().held != 0) {
        if (Steady::now() >= deadline) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return true;
}

[[maybe_unused]] const std::uint8_t* held_data(std::size_t index, std::size_t& size) {
    z_view_slice_t view;
    if (z_bytes_get_contiguous_view(z_bytes_loan(&held_payloads[index]), &view) != 0) return nullptr;
    size = z_slice_len(z_view_slice_loan(&view));
    return z_slice_data(z_view_slice_loan(&view));
}

// Exhaustion and late release on another thread (needs the pool).
template <class Transport>
void hold_phase(Transport& transport, const std::string& key, PublishQos qos, std::vector<std::string>& failures) {
    if constexpr (!HasPool<Transport>::value) {
        (void)transport; (void)key; (void)qos;
        failures.push_back("hold: transport has no packet pool (pre-ALLOC-1); exhaustion not tested");
        std::cout << "{\"test\":\"rns_zenoh_alloc\",\"phase\":\"hold\",\"skipped\":\"no packet pool\"}\n";
    } else {
        constexpr std::size_t kSlots = Transport::kPoolSlots;
        static_assert(kSlots + 1 <= kHoldCapacity, "hold capacity covers every slot plus the fallback");
        require(all_slots_returned(transport), "hold: zenoh-c still held slots 5 s after the steady phases");
        const auto base = transport.pool_counters();
        auto packet = packet_of(kProfileBytes);
        OneBytePatch finalizer;
        hold_active = true;
        hold_count = 0;
        // Fill every slot.
        for (std::size_t i = 0; i < kSlots; ++i) {
            mark(packet, 100 + i);
            const auto receipt = transport.put_finalized(key, packet.data(), packet.size(), qos, finalizer);
            require(receipt.outcome == PutOutcome::Accepted && !transport.last_audit().pool_fallback,
                    "hold: a free slot is used while one is left");
            require(transport.pool_counters().held == i + 1 && hold_count == i + 1, "hold: each held put keeps its slot");
        }
        // The next put finds every slot held: one malloc of ours, still Accepted.
        mark(packet, 100 + kSlots);
        reset_counts();
        t_measuring = true;
        const auto receipt = transport.put_finalized(key, packet.data(), packet.size(), qos, finalizer);
        t_measuring = false;
        const std::uint64_t fallback_alloc = t_ours_alloc;
        const std::size_t fallback_bytes = t_ours_last_bytes;
        const auto after = transport.pool_counters();
        if (receipt.outcome != PutOutcome::Accepted || !transport.last_audit().pool_fallback ||
            after.fallbacks != base.fallbacks + 1 || after.held != kSlots || fallback_alloc != 1 ||
            fallback_bytes != packet.size())
            failures.push_back("hold: exhausted pool did not take exactly one counted malloc fallback (ours_alloc=" +
                               std::to_string(fallback_alloc) + " bytes=" + std::to_string(fallback_bytes) +
                               " fallbacks=" + std::to_string(after.fallbacks - base.fallbacks) + ")");
        // Every held payload is a distinct buffer with exactly the bytes put into it.
        std::array<const std::uint8_t*, kSlots + 1> data{};
        for (std::size_t i = 0; i <= kSlots; ++i) {
            std::size_t size = 0;
            data[i] = held_data(i, size);
            require(data[i] != nullptr && size == packet.size(), "hold: held payload is one contiguous packet");
            if (marker_of(data[i]) != 100 + i)
                failures.push_back("hold: held payload " + std::to_string(i) + " was overwritten by a later put");
            for (std::size_t j = 0; j < i; ++j)
                if (data[i] == data[j])
                    failures.push_back("hold: put " + std::to_string(i) + " reused the buffer of held put " + std::to_string(j));
        }
        // zenoh-c lets go on another thread: the deleter runs there.
        std::thread dropper([] {
            for (std::size_t i = 0; i < hold_count; ++i) z_drop(z_move(held_payloads[i]));
        });
        dropper.join();
        hold_count = 0;
        const auto released = transport.pool_counters();
        if (released.held != 0 || released.bad_releases != base.bad_releases)
            failures.push_back("hold: release on another thread left held=" + std::to_string(released.held) +
                               " bad_releases=" + std::to_string(released.bad_releases - base.bad_releases));
        // Slot 0 again, with no allocation.
        mark(packet, 200);
        reset_counts();
        t_measuring = true;
        const auto again = transport.put_finalized(key, packet.data(), packet.size(), qos, finalizer);
        t_measuring = false;
        std::size_t size = 0;
        const auto* const reused = hold_count == 1 ? held_data(0, size) : nullptr;
        if (again.outcome != PutOutcome::Accepted || transport.last_audit().pool_fallback || t_ours_alloc != 0 ||
            reused != data[0])
            failures.push_back("hold: after the release the next put did not take slot 0 without allocating");
        for (std::size_t i = 0; i < hold_count; ++i) z_drop(z_move(held_payloads[i]));
        hold_count = 0;
        hold_active = false;
        std::cout << "{\"test\":\"rns_zenoh_alloc\",\"phase\":\"hold\",\"slots\":" << kSlots
                  << ",\"fallback_ours_alloc\":" << fallback_alloc << ",\"fallback_alloc_bytes\":" << fallback_bytes
                  << ",\"released_on_other_thread_held_after\":" << released.held
                  << ",\"slot0_reused_without_alloc\":" << (reused == data[0] ? "true" : "false") << "}\n";
    }
}

// Informational: 100 kB packets are fragmented by zenoh-c; were their slots still held at return?
template <class Transport>
void fragmented_probe(Transport& transport, const std::string& key, PublishQos qos) {
    if constexpr (HasPool<Transport>::value) {
        auto packet = packet_of(kFragmentedBytes);
        OneBytePatch finalizer;
        std::uint64_t held_at_return = 0, accepted = 0;
        const auto before = transport.pool_counters();
        for (std::size_t i = 0; i < kFragmentedPuts; ++i) {
            if (transport.put_finalized(key, packet.data(), packet.size(), qos, finalizer).outcome == PutOutcome::Accepted)
                ++accepted;
            if (transport.pool_counters().held != 0) ++held_at_return;
        }
        const bool returned = all_slots_returned(transport);
        std::cout << "{\"test\":\"rns_zenoh_alloc\",\"phase\":\"fragmented_info\",\"bytes\":" << kFragmentedBytes
                  << ",\"puts\":" << kFragmentedPuts << ",\"accepted\":" << accepted
                  << ",\"slot_still_held_at_put_return\":" << held_at_return
                  << ",\"fallbacks\":" << transport.pool_counters().fallbacks - before.fallbacks
                  << ",\"all_returned_within_5s\":" << (returned ? "true" : "false") << "}\n";
    } else {
        (void)transport; (void)key; (void)qos;
    }
}

// Was p among the pointers this thread freed while recording? (Unused against 0cbd03a's transport.)
[[maybe_unused]] bool freed_here(const void* p) noexcept {
    const std::size_t kept = std::min(t_freed_count, kFreedCapacity);
    for (std::size_t i = 0; i < kept; ++i)
        if (t_freed[i] == p) return true;
    return false;
}

// zenoh-c still holds payloads when the transport is destroyed, and lets go of them on another
// thread (GPT question 16 supplement: the hold phase above releases while the transport lives).
// The pool must outlive the transport: its bytes are not freed by the destructor, the held
// payloads keep their bytes, the first release frees nothing, and the last release frees the
// pool's byte array on the releasing thread. Slot 0 starts the byte array, so the first held
// payload's address is the pointer that free() must see.
template <class Transport>
void outlive_phase(ZenohRtSession& producer, const std::string& key, PublishQos qos, std::vector<std::string>& failures) {
    if constexpr (!HasPool<Transport>::value) {
        (void)producer; (void)key; (void)qos;
        failures.push_back("outlive: transport has no packet pool (pre-ALLOC-1); not tested");
        std::cout << "{\"test\":\"rns_zenoh_alloc\",\"phase\":\"outlive\",\"skipped\":\"no packet pool\"}\n";
    } else {
        constexpr std::size_t kHeld = 2;
        static_assert(kHeld < Transport::kPoolSlots && kHeld <= kHoldCapacity, "held payloads stay in pool slots");
        auto packet = packet_of(kProfileBytes);
        OneBytePatch finalizer;
        std::array<const std::uint8_t*, kHeld> data{};
        bool freed_by_destructor = false;
        {
            auto transport = std::make_unique<Transport>(producer, key, qos, kPacketCapacity);
            hold_active = true;
            hold_count = 0;
            for (std::size_t i = 0; i < kHeld; ++i) {
                mark(packet, 300 + i);
                const auto receipt = transport->put_finalized(key, packet.data(), packet.size(), qos, finalizer);
                require(receipt.outcome == PutOutcome::Accepted && !transport->last_audit().pool_fallback,
                        "outlive: a held put took a pool slot");
            }
            hold_active = false;
            require(hold_count == kHeld && transport->pool_counters().held == kHeld, "outlive: zenoh-c holds both slots");
            for (std::size_t i = 0; i < kHeld; ++i) {
                std::size_t size = 0;
                data[i] = held_data(i, size);
                require(data[i] != nullptr && size == packet.size(), "outlive: held payload is one contiguous packet");
            }
            // The transport goes while zenoh-c still holds both payloads.
            t_freed_count = 0;
            t_record_frees = true;
            transport.reset();
            t_record_frees = false;
            freed_by_destructor = freed_here(data[0]);
        }
        if (freed_by_destructor) failures.push_back("outlive: the transport's destructor freed the pool while zenoh-c held slots");
        std::array<bool, kHeld> intact{};
        for (std::size_t i = 0; i < kHeld; ++i) intact[i] = !freed_by_destructor && marker_of(data[i]) == 300 + i;
        const bool all_intact = std::all_of(intact.begin(), intact.end(), [](bool b) { return b; });
        // zenoh-c lets go on another thread, one payload at a time.
        std::array<bool, kHeld> freed_by_drop{};
        std::size_t frees_seen = 0;
        std::thread dropper([&] {
            for (std::size_t i = 0; i < kHeld; ++i) {
                t_freed_count = 0;
                t_record_frees = true;
                z_drop(z_move(held_payloads[i]));
                t_record_frees = false;
                freed_by_drop[i] = freed_here(data[0]);
                frees_seen = std::max(frees_seen, t_freed_count);
            }
        });
        dropper.join();
        hold_count = 0;
        for (std::size_t i = 0; i < kHeld; ++i)
            if (!intact[i]) failures.push_back("outlive: held payload " + std::to_string(i) + " lost its bytes when the transport went");
        if (freed_by_drop[0]) failures.push_back("outlive: the first release freed the pool while a slot was still held");
        if (!freed_by_drop[1])
            failures.push_back(frees_seen > kFreedCapacity ? "outlive: more frees than recorded; pool release not observed"
                                                           : "outlive: the last release did not free the pool (leak)");
        std::cout << "{\"test\":\"rns_zenoh_alloc\",\"phase\":\"outlive\",\"held\":" << kHeld
                  << ",\"pool_freed_by_transport_destructor\":" << (freed_by_destructor ? "true" : "false")
                  << ",\"held_bytes_intact_after_destructor\":" << (all_intact ? "true" : "false")
                  << ",\"pool_freed_by_release\":[" << (freed_by_drop[0] ? "true" : "false") << ','
                  << (freed_by_drop[1] ? "true" : "false") << "],\"most_frees_in_one_release\":" << frees_seen << "}\n";
    }
}

int run() {
    require(std::string_view(ZENOH_C) == "1.9.0", "test requires verified Zenoh C 1.9.0 SDK");
    const auto endpoint = ephemeral_loopback_endpoint();
    const auto prefix = "xbrain/rns_alloc_" + std::to_string(::getpid()) + "/rt/perception/";
    Receiver receiver(endpoint, prefix + "*");
    ZenohRtSession producer(endpoint);
    const PublishQos fast{Priority::RealTime, false, true, true};
    const PublishQos health{Priority::DataHigh, true, false, true};
    const std::string profile_key = prefix + "profile", objects_key = prefix + "objects", status_key = prefix + "status";
    ZenohRtTransport profile(producer, profile_key, fast, kPacketCapacity);
    ZenohRtTransport objects(producer, objects_key, fast, kPacketCapacity);
    ZenohRtTransport status(producer, status_key, health, kPacketCapacity);
    const auto matching_deadline = Steady::now() + kWait;
    while (!(profile.has_matching_subscriber() && objects.has_matching_subscriber() && status.has_matching_subscriber())) {
        require(Steady::now() < matching_deadline, "matching subscriber timeout after 5 seconds");
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    std::vector<std::string> failures;
    // 1: one publisher after the other, on this thread.
    for (const auto& s : {steady("single", "profile", profile, profile_key, fast, kProfileBytes),
                          steady("single", "objects", objects, objects_key, fast, kObjectsBytes),
                          steady("single", "status", status, status_key, health, kStatusBytes)}) {
        print(s);
        judge(s, failures);
    }
    const auto rx_deadline = Steady::now() + kWait;
    while (received.load() == 0 && Steady::now() < rx_deadline) std::this_thread::sleep_for(std::chrono::milliseconds(2));
    if (received.load() == 0) failures.push_back("receiver got no sample: zenoh-c did not route the puts");
    // 2: Profile and Objects at the same time, each on its own thread.
    // The transports change owner only across thread start and join (single owner at a time).
    Stats concurrent_profile, concurrent_objects;
    std::string error_a, error_b;
    std::thread a([&] {
        try { concurrent_profile = steady("concurrent", "profile", profile, profile_key, fast, kProfileBytes); }
        catch (const std::exception& e) { error_a = e.what(); }
    });
    std::thread b([&] {
        try { concurrent_objects = steady("concurrent", "objects", objects, objects_key, fast, kObjectsBytes); }
        catch (const std::exception& e) { error_b = e.what(); }
    });
    a.join();
    b.join();
    require(error_a.empty() && error_b.empty(), "concurrent phase threw");
    for (const auto* s : {&concurrent_profile, &concurrent_objects}) {
        print(*s);
        judge(*s, failures);
    }
    // 3: exhaustion and late release, then the steady state again on the same publisher. A broken
    // pool may throw here; it is one more failure, and hold mode is switched off before going on.
    try {
        hold_phase(profile, profile_key, fast, failures);
    } catch (const std::exception& e) {
        failures.push_back(std::string("hold phase aborted: ") + e.what());
        for (std::size_t i = 0; i < hold_count; ++i) z_drop(z_move(held_payloads[i]));
        hold_count = 0;
        hold_active = false;
    }
    if constexpr (HasPool<ZenohRtTransport>::value) {
        const auto s = steady("after_hold", "profile", profile, profile_key, fast, kProfileBytes);
        print(s);
        judge(s, failures);
    }
    // 4: informational.
    fragmented_probe(objects, objects_key, fast);
    // 5: zenoh-c holds payloads past the destruction of their transport (a second Profile
    // publisher: the transport accepts only the three RT keys).
    try {
        outlive_phase<ZenohRtTransport>(producer, profile_key, fast, failures);
    } catch (const std::exception& e) {
        failures.push_back(std::string("outlive phase aborted: ") + e.what());
        for (std::size_t i = 0; i < hold_count; ++i) z_drop(z_move(held_payloads[i]));
        hold_count = 0;
        hold_active = false;
    }
    receiver.close();
    std::cerr << "rx_samples=" << received.load() << " receiver_closure_dropped=" << receiver_dropped.load() << '\n';
    if (!failures.empty()) {
        std::cerr << "RNS Zenoh alloc FAIL (" << failures.size() << "):\n";
        for (const auto& f : failures) std::cerr << "  " << f << '\n';
        return 1;
    }
    std::cerr << "RNS Zenoh alloc PASS: our code 0 allocations / 0 frees per put on the publishing thread "
                 "(3 publishers x 1000 + 2 concurrent x 1000 + 1000 after exhaustion); zenoh-c's own column "
                 "printed per phase; the pool outlives a destroyed transport and is freed by the last release "
                 "on another thread. Loopback only; zenoh-c's threads not counted; no timing claim.\n";
    return 0;
}
}  // namespace

int main() {
    try { return run(); }
    catch (const std::exception& error) {
        std::cerr << "RNS Zenoh alloc FAIL: " << error.what() << '\n';
        return 1;
    }
}
