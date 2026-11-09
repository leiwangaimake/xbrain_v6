/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_gen_events.cpp
 * Brief: SPSC ring of stamped reason edges and their 11 section 6.1 event encoding.
 *
 * Description:
 * offer() order: validate time and edge, refuse if the ring is full (never overwrite an older
 * event), stamp the envelope (this consumes a seq), write the slot, publish head. send_one()
 * order: copy the oldest entry, release its slot at once (the put below may block for a long
 * time and the producer must not wait for it), encode, check the send clock, put, count.
 *
 * Why every problem marks the handoff unhealthy instead of throwing. Both calls run on worker
 * threads that must keep going; the latched healthy() flag and the counters tell the owner what
 * was lost. Nothing is retried: an Unknown put may have been delivered, and a resend would
 * duplicate the event.
 *
 * Event JSON (11 section 6.1): envelope, then data {eid, sev, cat "perception", channel "normal"
 * (11 section 6.2 for this category), title, detail {type "perception.reason_changed",
 * observed_mono_ms, and the reason lists}}. The lists name reasons, never bit numbers.
 *
 * Tests. tests/test_rns_gen_events.cpp and tests/test_rns_gen_loopback.cpp.
 */
#include "output/rns_gen_events.hpp"
#include <charconv>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <utility>

namespace perception::rns {
namespace {
static_assert(std::atomic<std::size_t>::is_always_lock_free && std::atomic<bool>::is_always_lock_free,
              "GEN handoff requires lock-free index/flag atomics");
// Ring slots = capacity + 1: one slot stays empty so head == tail means empty.
std::size_t checked_queue(std::size_t n) {
    if (!n || n > GenEvents::kMaxQueue) throw std::invalid_argument("GEN queue capacity");
    return n + 1;
}
// Packet buffer size; the constructor also checks it fits every reason list.
std::size_t checked_packet(std::size_t n) {
    if (!n || n > GenEvents::kMaxPacket) throw std::invalid_argument("GEN packet capacity");
    return n;
}
// Reason names go into JSON unescaped, so they are restricted to [a-z0-9_].
bool token(std::string_view value, std::size_t max) {
    if (value.empty() || value.size() > max) return false;
    for (const char c : value)
        if (!((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_')) return false;
    return true;
}
// Bounded writer into the fixed packet buffer; any overflow sets ok = false and
// the whole event is dropped (never a truncated JSON document).
struct Writer {
    char* out; std::size_t capacity, size = 0; bool ok = true;
    void add(std::string_view text) noexcept {
        if (!ok || text.size() > capacity - size) { ok = false; return; }
        std::memcpy(out + size, text.data(), text.size()); size += text.size();
    }
    void number(std::uint64_t value) noexcept {
        char s[20]; auto r = std::to_chars(s, s + sizeof(s), value); add({s, static_cast<std::size_t>(r.ptr - s)});
    }
    // Seconds with three decimals from integer parts (as in rns_publisher).
    void seconds(Millis value) noexcept {
        number(static_cast<std::uint64_t>(value / 1000)); add(".");
        const auto fraction = value % 1000;
        const char part[] = {static_cast<char>('0' + fraction / 100),
            static_cast<char>('0' + fraction / 10 % 10), static_cast<char>('0' + fraction % 10)};
        add({part, 3});
    }
    // Callers pass only validated tokens, so no escaping is needed here.
    void quoted(std::string_view value) noexcept { add("\""); add(value); add("\""); }
};
}

GenEvents::GenEvents(ProcessEnvelope& envelope, PublishClock& clock,
                     RnsTransport& warn, RnsTransport& info, std::string nonce,
                     std::vector<std::string> names, std::size_t queue_capacity,
                     std::size_t packet_capacity)
    : envelope_(envelope), clock_(clock), warn_(warn), info_(info), nonce_(std::move(nonce)),
      names_(std::move(names)), queue_(checked_queue(queue_capacity)), packet_(checked_packet(packet_capacity)) {
    // A valid envelope, a 32-hex nonce, 1..64 reason names (one bit each).
    if (!envelope.valid() || nonce_.size() != 32 || names_.empty() || names_.size() > 64)
        throw std::invalid_argument("GEN identity/names");
    bool nonzero = false;
    for (const char c : nonce_) {
        if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) throw std::invalid_argument("GEN nonce");
        nonzero |= c != '0';
    }
    // An all-zero nonce is what an unread UUID would look like.
    if (!nonzero) throw std::invalid_argument("GEN nonce zero");
    // Names: valid tokens, no repeats (a repeat would make a list ambiguous).
    for (std::size_t i = 0; i < names_.size(); ++i) {
        if (!token(names_[i], 63)) throw std::invalid_argument("GEN reason name");
        for (std::size_t j = 0; j < i; ++j)
            if (names_[i] == names_[j]) throw std::invalid_argument("GEN repeated reason name");
    }
    // One bit per name; 1 << 64 is undefined, hence the explicit all-ones case.
    allowed_ = names_.size() == 64 ? UINT64_MAX : (std::uint64_t{1} << names_.size()) - 1;
    // Bare keys (19 section 5.2, 11 section 1.1.6 PC-2: event/{severity}/perception); the robot id
    // travels in the envelope, not in the key (the prefixed form belongs to p5_gateway).
    warn_key_ = "event/warn/perception";
    info_key_ = "event/info/perception";
    // Bound every possible bit-list, including audit fields, before admitting
    // any event. Capacity failure is configuration failure, not partial send.
    // Each name costs at most its length plus quotes and a comma, in each of the
    // seven lists; 2048 bytes cover the envelope and fixed fields.
    std::size_t reason_bytes = 0;
    for (const auto& name : names_) reason_bytes += name.size() + 3;
    if (packet_.size() < 2048 + 7 * reason_bytes) throw std::invalid_argument("GEN packet too small for names");
}
bool GenEvents::increment(std::uint64_t& count) noexcept {
    // Saturate rather than wrap; saturation itself is reported as unhealthy.
    if (count == UINT64_MAX) { healthy_.store(false, std::memory_order_release); return false; }
    ++count; return true;
}
bool GenEvents::valid_edge(const ReasonEdge& e) const noexcept {
    // The edge must be exactly what ReasonEdges produces: known bits only; something
    // eligible; raw = eligible + suppressed in each direction, disjoint; nothing both
    // added and removed; added bits active afterwards, removed bits not; severity
    // warn iff anything eligible was added.
    const auto all = e.raw_added | e.raw_removed | e.eligible_added | e.eligible_removed |
                     e.suppressed_added | e.suppressed_removed | e.active_after;
    if (all & ~allowed_ || !(e.eligible_added | e.eligible_removed) ||
        (e.raw_added & e.raw_removed) || (e.eligible_added & e.suppressed_added) ||
        (e.eligible_removed & e.suppressed_removed) ||
        (e.eligible_added | e.suppressed_added) != e.raw_added ||
        (e.eligible_removed | e.suppressed_removed) != e.raw_removed ||
        (e.active_after & e.raw_added) != e.raw_added || (e.active_after & e.raw_removed)) return false;
    return e.severity == (e.eligible_added ? ReasonEdgeSeverity::Warn : ReasonEdgeSeverity::Info);
}
GenOfferResult GenEvents::offer(const ReasonEdge& edge, Millis wall, Millis mono) noexcept {
    // Times valid and in order (offer time and edge time never go back), edge coherent.
    if (mono < 0 || mono > ProcessEnvelope::kMaxMonoMs || wall < 0 || mono < last_offer_ ||
        edge.observed_ms < 0 || edge.observed_ms > mono || edge.observed_ms < last_observed_ || !valid_edge(edge)) {
        healthy_.store(false, std::memory_order_release); increment(offered_.invalid);
        // seq 0: nothing was stamped.
        return {GenOfferCode::Invalid, 0};
    }
    // Lifetime counter exhausted: stop accepting rather than wrap.
    if (offered_.accepted == UINT64_MAX || offered_.exhausted) {
        offered_.exhausted = true; healthy_.store(false, std::memory_order_release);
        return {GenOfferCode::Exhausted, 0};
    }
    // head is ours (relaxed); tail comes from the consumer (acquire).
    const auto head = head_.load(std::memory_order_relaxed), next = (head + 1) % queue_.size();
    // Full: refuse this edge, keep the older ones, and advance the watermarks so the
    // caller's next edge is still checked for order.
    if (next == tail_.load(std::memory_order_acquire)) {
        healthy_.store(false, std::memory_order_release); increment(offered_.full);
        last_offer_ = mono; last_observed_ = edge.observed_ms;
        return {GenOfferCode::Full, 0};
    }
    // Stamp only when the edge will certainly be queued: a seq is never wasted on
    // an event that cannot be sent.
    EnvelopeStamp stamp;
    if (!envelope_.stamp(wall, mono, stamp)) {
        healthy_.store(false, std::memory_order_release); increment(offered_.stamp_failed);
        return {GenOfferCode::StampUnavailable, 0};
    }
    // Write the slot, then publish it (release pairs with the consumer's acquire).
    queue_[head] = {stamp, edge};
    last_offer_ = mono; last_observed_ = edge.observed_ms; ++offered_.accepted;
    head_.store(next, std::memory_order_release);
    return {GenOfferCode::Accepted, stamp.seq};
}
// Warn and info have separate keys (and transports); fault notices are elsewhere.
std::string_view GenEvents::key(ReasonEdgeSeverity severity) const noexcept {
    if (severity == ReasonEdgeSeverity::Warn) return warn_key_;
    if (severity == ReasonEdgeSeverity::Info) return info_key_;
    return {};
}
// Diagnostic only: head and tail are read separately, not as one snapshot.
std::size_t GenEvents::queue_depth() const noexcept {
    const auto tail = tail_.load(std::memory_order_acquire), head = head_.load(std::memory_order_acquire);
    return (head + queue_.size() - tail) % queue_.size();
}
std::size_t GenEvents::encode(const Entry& entry) noexcept {
    const auto& s = entry.stamp; const auto& e = entry.edge;
    // Envelope first (11 section 3.0), then the event data (11 section 6.1).
    Writer w{packet_.data(), packet_.size()};
    // v, rid, ts (wall seconds), mono (monotonic seconds), boot, seq, src, ts_sync:
    // all from the stamp frozen at offer time.
    w.add("{\"v\":1,\"rid\":"); w.quoted(envelope_.rid()); w.add(",\"ts\":"); w.seconds(s.wall_ms);
    w.add(",\"mono\":"); w.seconds(s.mono_ms); w.add(",\"boot\":"); w.quoted(envelope_.boot());
    w.add(",\"seq\":"); w.number(s.seq); w.add(",\"src\":\"perception\",\"ts_sync\":");
    w.add(s.ts_sync ? "true" : "false");
    // eid = p-<nonce>-<seq>; sev follows the edge (warn = something degraded, info =
    // recovery only), which is also the key the event goes to.
    w.add(",\"data\":{\"eid\":\"p-"); w.add(nonce_); w.add("-"); w.number(s.seq);
    w.add("\",\"sev\":"); w.quoted(e.severity == ReasonEdgeSeverity::Warn ? "warn" : "info");
    w.add(",\"cat\":\"perception\",\"channel\":\"normal\",\"title\":\"Perception reason state changed\",\"detail\":{\"type\":\"perception.reason_changed\",\"observed_mono_ms\":");
    // When the reason set was observed (monotonic ms), which may precede the stamp.
    w.number(static_cast<std::uint64_t>(e.observed_ms));
    // One JSON array of reason names per bit set, in reason_names() order.
    const auto list = [&](std::string_view name, std::uint64_t bits) {
        w.add(","); w.quoted(name); w.add(":["); bool comma = false;
        for (std::size_t i = 0; i < names_.size(); ++i) if (bits & (std::uint64_t{1} << i)) {
            if (comma) w.add(",");
            w.quoted(names_[i]); comma = true;
        }
        w.add("]");
    };
    // raw_*: the whole change of this beat; added/removed: what this event reports
    // after the 10 s dedupe; suppressed_*: what the dedupe held back; active: the
    // full reason set after the change.
    list("raw_added", e.raw_added); list("raw_removed", e.raw_removed);
    list("added", e.eligible_added); list("removed", e.eligible_removed);
    list("suppressed_added", e.suppressed_added); list("suppressed_removed", e.suppressed_removed);
    list("active", e.active_after); w.add("}}}");
    // No downstream dedup_key: local direction-aware dedupe already happened;
    // a generic group key could merge warn with its recovery or erase causes.
    return w.ok ? w.size : 0;
}
GenSendResult GenEvents::send_one() noexcept {
    GenSendResult result;
    const auto tail = tail_.load(std::memory_order_relaxed);
    // Empty ring: nothing to send (code Empty).
    if (tail == head_.load(std::memory_order_acquire)) return result;
    // Copy, then release the slot before the possibly blocking put.
    const Entry entry = queue_[tail];
    tail_.store((tail + 1) % queue_.size(), std::memory_order_release);
    increment(sent_.popped);
    // The result carries the stamp and edge so the caller can log what was sent.
    result.stamp = entry.stamp; result.edge = entry.edge; result.packet_bytes = encode(entry);
    // Did not fit: the entry is consumed and counted unsent (never partially sent).
    if (!result.packet_bytes) {
        healthy_.store(false, std::memory_order_release); increment(sent_.unsent);
        result.code = GenSendCode::EncodeFailed; return result;
    }
    // The send clock must not be before the event's own stamp or the last send.
    const auto before = clock_.mono_millis(); result.put_before_ms = before;
    if (before < entry.stamp.mono_ms || before < last_send_ || before > ProcessEnvelope::kMaxMonoMs) {
        healthy_.store(false, std::memory_order_release); increment(sent_.unsent);
        result.code = GenSendCode::ClockInvalid; return result;
    }
    // From here the put is attempted; its outcome is counted whatever it is.
    result.attempted = true;
    // A throwing transport is an Unknown outcome, never a crash of the GEN thread.
    try {
        auto& transport = entry.edge.severity == ReasonEdgeSeverity::Warn ? warn_ : info_;
        result.receipt = transport.put(key(entry.edge.severity), {packet_.data(), result.packet_bytes}, qos());
    } catch (...) { result.threw = true; result.receipt = {PutOutcome::Unknown, 0}; }
    // Return time for the audit; a broken return clock does not move the watermark.
    result.put_return_ms = clock_.mono_millis();
    result.clock_valid = result.put_return_ms >= before && result.put_return_ms <= ProcessEnvelope::kMaxMonoMs;
    last_send_ = result.clock_valid ? result.put_return_ms : before;
    // Local API outcome only; Unknown is not retried (it may have been delivered).
    switch (result.receipt.outcome) {
        case PutOutcome::Accepted: increment(sent_.accepted); result.code = GenSendCode::Accepted; break;
        case PutOutcome::Rejected: increment(sent_.rejected); result.code = GenSendCode::Rejected; break;
        default: increment(sent_.unknown); result.code = GenSendCode::Unknown; break;
    }
    // Anything but a clean accepted put is reported through healthy() and the counters.
    if (result.code != GenSendCode::Accepted || !result.clock_valid) healthy_.store(false, std::memory_order_release);
    // A broken return clock outranks the outcome in the code; the counters above
    // still record the outcome itself.
    if (!result.clock_valid) result.code = GenSendCode::ClockInvalid;
    return result;
}
} // namespace perception::rns
