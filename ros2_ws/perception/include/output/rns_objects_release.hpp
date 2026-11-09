/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_objects_release.hpp
 * Brief: Paced release of finished Objects bodies on a thread of their own.
 *
 * Description:
 * What problem this solves. The Objects cadence gate judges the interval between consecutive
 * publications (rns_cadence.hpp). The host time between "both models' outputs are ready" and
 * "the body is finished" varies from frame to frame, and published at once that variation is
 * the variation of the publish interval. Here the thread that builds a body hands a copy over
 * and goes back to the next frame; a release thread calls the channel for the copy when a
 * condition tied to the frame's ready time r and a fixed delay D holds, which turns most of
 * that variation into a constant.
 *
 * Structure. Two places of one body each, and nothing else (2026-10-16, R1: latest wins up to
 * the publication; GPT question 18.1 / 19 P19-2):
 *     pending    the body handed over last and not yet selected. Latest wins: the next hand-over
 *                takes its place at any time before the selection (the replaced body is counted
 *                as overwritten);
 *     in flight  the body the release thread selected and committed: exactly one call of the
 *                channel is made for it. It cannot be replaced, and a later hand-over waits in
 *                the pending place.
 * At most two bodies are inside, and no older body is ever published after a newer one was
 * handed over before its selection.
 *
 * Release rule. The release thread selects the pending body when nothing is in flight and its
 * clock reads at least the body's release time (due). due is the body's own r + D, or, when it
 * replaced a body that had not been selected, the earlier due of that body: replacements pass the
 * release time on, so a stream of hand-overs cannot keep postponing it (no starvation). The
 * selection and the commitment are one step under the lock; the call follows without the lock and
 * without anything in between but the call-time clock read, so the selected body is the latest one
 * at the moment the publication starts. due is a trigger, not the time of the channel call and
 * not a lower bound of it relative to the body's own r + D: an inherited due can come before it.
 * The call comes after due when the hand-over itself does (reason Late), and always after its
 * trigger by the scheduling of the release thread and, while a body is in flight, by the return of
 * that call and the observer, which runs on the release thread between two calls.
 * Consequence (by design, it is what "latest wins" means). With a constant frame period P, a
 * constant time h from r to the hand-over and the release thread on time, frame k is due at
 * r_k + D and frame k+1 is handed over at r_k + P + h, so it replaces frame k exactly when
 * D > P + h; then only every other body is published, the newest one at the inherited due (a still
 * larger D, a shorter period or jitter replaces more). With D < P + h every body goes out
 * (D 30 ms, P about 40 ms: nothing is replaced in normal running). A late release thread can make
 * a replacement happen also with D < P + h; overwritten counts each one.
 *
 * Records and progress (2026-10-16, GPT question 18 supplement). The observer gets the finished
 * records one batch at a time, without the lock; after each batch the release thread looks at a
 * stop, a fault and the due of the pending body before it takes the next batch, so a stream of
 * records (every replacement makes one) cannot keep a due body from being selected. An observer
 * that never returns still stops the release thread; that is not covered.
 *
 * Publication commitment. Under the lock, and only while the release is not closed, the release
 * thread selects and COMMITS the pending body: from then on exactly one call of the publish
 * function is made for it and is allowed to complete (in the channel: at most one attempt and its
 * completion), whatever happens meanwhile. Then, without the lock, it reads the call time and
 * calls the publish function; the lock is never held across a put. In the counters the body is
 * "in flight" from the commitment until its call was accounted.
 *
 * Ownership. offer() is called by ONE producer thread; the publish function is called by the
 * release thread only, which is therefore the single owner of the Objects PublisherChannel,
 * its clock, cadence ledger and journal. Waiting in the pending place takes place before the
 * channel is called: it holds no publication attempt and opens no journal generation. The body
 * given to the publish function is an owned copy that nobody else can change while the call
 * runs.
 *
 * Stop and faults. close() refuses further bodies and cancels what is NOT committed: under the
 * lock it discards the pending body (counted; it is not published to drain the stage). When close() returns, at most one body is still
 * inside, a committed one. Its call may begin after close() has returned (the commitment comes
 * before the call-time clock read and before the channel is entered), it completes and is
 * accounted once; close() does not wait for it, join() does. A publish result other than
 * accepted / rejected / unknown is a local contract fault: it is latched, admission stops, what
 * is not committed is discarded by the release thread, and faulted() lets the main loop stop
 * the service without waiting for another frame. A fault can also be latched by a call that
 * returns during a normal stop, so faulted() must be read again after join().
 *
 * Times. All on the injected clock (CLOCK_MONOTONIC ns), see ReleaseRecord. offer() reads it
 * twice: at its start (offer_begin_ns, before the copy and the lock) and under the lock at the
 * hand-over itself (handoff_ns, which so includes the copy, the wait for the lock and any
 * preemption in between). r must not be later than the first read and the second must not be
 * earlier than the first, otherwise the offer is Invalid and a fault is latched. Whether a body
 * is Late is decided by handoff_ns, never by offer_begin_ns; the selection compares the release
 * thread's own read, taken under the lock, with due, and that read is claim_ns.
 *
 * Accounting. Every admitted body ends in exactly one of: published (its committed call
 * returned and the channel had attempted a put), refused by the channel (its committed call
 * returned without an attempt), overwritten (replaced in the pending place), discarded at a stop
 * before a commitment.
 * ReleaseCounters carries the totals and the bodies still inside; unaccounted() must be 0.
 * Every end is also given to the observer (on the release thread) as a ReleaseRecord with the
 * times of that body.
 */
#pragma once
#include "output/rns_contract.hpp"
#include "output/rns_publisher.hpp"
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace perception::rns {
// How an admitted body left the release stage. Published / ChannelRefused: its committed call
// returned, with / without an attempted put. StopDiscarded: cancelled before a commitment.
enum class ReleaseEnd : std::uint8_t { Published, ChannelRefused, Overwritten, StopDiscarded };
// How a body was selected. Late: its own hand-over came at or after its due (handoff_ns >=
// due_ns): there was nothing left to wait for. Otherwise Deadline: the release thread read a
// clock value >= due while nothing was in flight (claim_ns says when).
enum class ReleaseReason : std::uint8_t { None, Deadline, Late };
// What offer() did with a body. Invalid: no usable ready time or clock (a fault is latched).
enum class OfferResult : std::uint8_t { Admitted, Closed, Faulted, Invalid, CopyFailed };

// One admitted body from hand-over to its end. Times are CLOCK_MONOTONIC ns of the injected
// clock; 0 = that step was not reached.
//   ready_ns        r, as given to offer();
//   offer_begin_ns  read at the start of offer(), before the copy and the lock;
//   handoff_ns      read under the lock when the body became the pending one;
//   deadline_ns     r + D, the body's own;
//   due_ns          its release time: deadline_ns, or the earlier due of the body it replaced;
//   claim_ns        read under the lock when the release thread selected and committed the body
//                   (its read that was >= due_ns);
//   call_ns         read by the release thread after the commitment, without the lock, just
//                   before the publish function is called;
//   return_ns       read by the release thread just after the publish function returned.
// ready_ns <= offer_begin_ns <= handoff_ns is checked by offer(); the later ones are not.
struct ReleaseRecord {
    ReleaseEnd end = ReleaseEnd::Published;
    ReleaseReason reason = ReleaseReason::None;
    std::uint64_t frame_id = 0;
    Millis capture_ms = -1;
    std::size_t objects = 0;
    std::int64_t ready_ns = 0, offer_begin_ns = 0, handoff_ns = 0, deadline_ns = 0, due_ns = 0, claim_ns = 0,
                 call_ns = 0, return_ns = 0;
    // Overwritten: the frame that took the pending place.
    std::uint64_t replaced_by = 0;
    // Published / ChannelRefused: what the channel returned.
    PublishResult result;
};

struct ReleaseCounters {
    // offer() calls and how they ended: offered = admitted + the four refusals.
    std::uint64_t offered = 0, admitted = 0, refused_closed = 0, refused_faulted = 0, refused_invalid = 0,
        refused_copy = 0;
    // Ends of admitted bodies (ReleaseEnd).
    std::uint64_t published = 0, channel_refused = 0, overwritten = 0, stop_discarded = 0;
    // Bodies still inside, 0 or 1 each: pending = handed over, not selected; in_flight =
    // selected and committed, its one call not accounted yet.
    std::uint64_t pending = 0, in_flight = 0;
    // Commitments taken, each one call of the publish function:
    // committed = published + channel_refused + in_flight = by_deadline + late.
    std::uint64_t committed = 0;
    // Selections by reason, how many of them ran on a due inherited from a replaced body, and the
    // channel outcomes of the published ones: published = accepted + rejected + unknown.
    std::uint64_t by_deadline = 0, late = 0, inherited = 0;
    std::uint64_t accepted = 0, rejected = 0, unknown = 0, contract_faults = 0;
    // Observer records dropped because the bounded list was full (the counters stay exact).
    std::uint64_t records_lost = 0;
    std::int64_t unaccounted() const noexcept {
        return static_cast<std::int64_t>(admitted) -
               static_cast<std::int64_t>(published + channel_refused + overwritten + stop_discarded + pending +
                                         in_flight);
    }
};

class ObjectsRelease {
public:
    using Clock = std::function<std::int64_t()>;                   // CLOCK_MONOTONIC ns; <= 0 = unavailable
    using Publish = std::function<PublishResult(const Objects&)>;  // release thread only
    using Observer = std::function<void(const ReleaseRecord&)>;    // release thread only; may be empty
    static constexpr std::int64_t kMaxDelayNs = 200000000;         // 200 ms
    // delay_ns in (0, kMaxDelayNs]; with 0 the caller keeps its synchronous publish and builds
    // no ObjectsRelease. thread_name is given to the release thread (15 bytes are kept).
    ObjectsRelease(std::int64_t delay_ns, Clock clock, Publish publish, Observer observer = {},
                   const char* thread_name = "obj_release");
    ~ObjectsRelease();  // close() and join()
    ObjectsRelease(const ObjectsRelease&) = delete;
    ObjectsRelease& operator=(const ObjectsRelease&) = delete;
    bool valid() const noexcept { return valid_; }
    // Starts the release thread (once). A body offered before it stays pending.
    bool start();
    // Producer thread: hand over a copy of body. ready_observed_mono_ns is the frame's r; it
    // must be positive and not later than the clock at the start of the call, otherwise the
    // offer is Invalid and a fault is latched (a frame without r cannot be paced, and "now" is
    // never used in its place). The same holds when the clock read at the hand-over itself is
    // unavailable or earlier than the one at the start.
    OfferResult offer(const Objects& body, std::uint64_t frame_id, std::int64_t ready_observed_mono_ns) noexcept;
    // Any thread: refuse further offers, discard what is not committed and wake the release
    // thread (idempotent). Does not wait for a committed call.
    void close() noexcept;
    // Any thread but the release thread: wait for the release thread to end (after close()).
    void join() noexcept;
    // Any thread: make the release thread read the clock again (an injected clock that was set).
    void wake() noexcept;
    bool faulted() const noexcept;
    std::string fault_reason() const;
    ReleaseCounters counters() const;
private:
    struct Slot {
        Objects body;
        std::uint64_t frame_id = 0;
        std::int64_t ready_ns = 0, offer_begin_ns = 0, handoff_ns = 0, deadline_ns = 0, due_ns = 0, claim_ns = 0;
        ReleaseReason reason = ReleaseReason::None;
    };
    void run() noexcept;
    // All of these need mutex_ held.
    void fault_locked(const std::string& why) noexcept;
    void record_locked(const ReleaseRecord& record) noexcept;
    ReleaseRecord describe_locked(const Slot& slot, ReleaseEnd end) const;
    void select_locked(std::int64_t now) noexcept;
    void discard_uncommitted_locked() noexcept;

    const std::int64_t delay_ns_;
    const Clock clock_;
    const Publish publish_;
    const Observer observer_;
    const std::string thread_name_;
    bool valid_ = false;
    mutable std::mutex mutex_;
    std::condition_variable wake_;
    // Guarded by mutex_. in_flight_ holds the selected, committed body (has_in_flight_); while it is
    // there the release thread reads in_flight_.body without the lock and nobody else touches it.
    Slot pending_, in_flight_;
    bool has_pending_ = false, has_in_flight_ = false, closed_ = false, faulted_ = false, started_ = false;
    std::string fault_reason_;
    ReleaseCounters counters_;
    std::vector<ReleaseRecord> records_;  // bounded; drained by the release thread
    // Producer thread only: the copy is made here, outside the lock, then swapped into pending_.
    Objects scratch_;
    std::thread thread_;
};
}  // namespace perception::rns
