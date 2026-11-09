/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_objects_release.cpp
 * Brief: Pending place, in-flight place and release thread for paced Objects publication.
 *
 * Description:
 * See rns_objects_release.hpp for the rule, the commitment and the ownership. Three body buffers
 * rotate and are never reallocated on purpose: scratch_ (the producer copies into it without the
 * lock), pending_ (the body handed over last) and in_flight_ (the selected body, which the release
 * thread publishes). A hand-over, a selection with its commitment and a discard are steps under
 * mutex_; the copy, the publish call and the observer run without it.
 */

#include "output/rns_objects_release.hpp"
#include "common/thread_name.hpp"
#include <chrono>
#include <limits>
#include <utility>

namespace perception::rns {
namespace {
// Records waiting for the observer; more than one per frame only piles up when it is slow.
constexpr std::size_t kRecordCapacity = 64;
}  // namespace

ObjectsRelease::ObjectsRelease(std::int64_t delay_ns, Clock clock, Publish publish, Observer observer,
                               const char* thread_name)
    : delay_ns_(delay_ns), clock_(std::move(clock)), publish_(std::move(publish)), observer_(std::move(observer)),
      thread_name_(thread_name != nullptr ? thread_name : "") {
    valid_ = delay_ns_ > 0 && delay_ns_ <= kMaxDelayNs && clock_ && publish_;
    records_.reserve(kRecordCapacity);
}

ObjectsRelease::~ObjectsRelease() {
    close();
    join();
}

bool ObjectsRelease::start() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!valid_ || started_ || closed_) return false;
    thread_ = std::thread([this] { run(); });
    started_ = true;
    return true;
}

void ObjectsRelease::fault_locked(const std::string& why) noexcept {
    // The flags first: a reason that cannot be stored must not hide the fault.
    const bool first = !faulted_;
    faulted_ = true;
    closed_ = true;
    if (first) {
        try { fault_reason_ = why; } catch (...) {}
    }
}

void ObjectsRelease::record_locked(const ReleaseRecord& record) noexcept {
    // Within the reserved capacity push_back does not allocate.
    if (records_.size() < kRecordCapacity) records_.push_back(record);
    else ++counters_.records_lost;
}

ReleaseRecord ObjectsRelease::describe_locked(const Slot& slot, ReleaseEnd end) const {
    ReleaseRecord record;
    record.end = end;
    record.reason = slot.reason;
    record.frame_id = slot.frame_id;
    record.capture_ms = slot.body.t_capture_mono_ms;
    record.objects = slot.body.objects.size();
    record.ready_ns = slot.ready_ns;
    record.offer_begin_ns = slot.offer_begin_ns;
    record.handoff_ns = slot.handoff_ns;
    record.deadline_ns = slot.deadline_ns;
    record.due_ns = slot.due_ns;
    record.claim_ns = slot.claim_ns;
    return record;
}

// The selection: pending -> in flight at clock value `now`, committed in the same step (the call
// follows on the release thread without the lock). A body whose hand-over was at or after its due
// is Late.
void ObjectsRelease::select_locked(std::int64_t now) noexcept {
    std::swap(in_flight_, pending_);
    has_pending_ = false;
    has_in_flight_ = true;
    in_flight_.claim_ns = now;
    ++counters_.committed;
    if (in_flight_.due_ns < in_flight_.deadline_ns) ++counters_.inherited;
    if (in_flight_.handoff_ns >= in_flight_.due_ns) {
        in_flight_.reason = ReleaseReason::Late;
        ++counters_.late;
    } else {
        in_flight_.reason = ReleaseReason::Deadline;
        ++counters_.by_deadline;
    }
}

// Ends the body that has no commitment: the pending one. A body in flight is committed and is left
// to its one call.
void ObjectsRelease::discard_uncommitted_locked() noexcept {
    if (has_pending_) {
        ++counters_.stop_discarded;
        record_locked(describe_locked(pending_, ReleaseEnd::StopDiscarded));
        has_pending_ = false;
    }
}

OfferResult ObjectsRelease::offer(const Objects& body, std::uint64_t frame_id,
                                  std::int64_t ready_observed_mono_ns) noexcept {
    // First clock read: the start of the call, before the copy and the lock.
    std::int64_t begin = 0;
    try { begin = clock_ ? clock_() : 0; } catch (...) { begin = 0; }
    // r must exist, lie on this clock and not in the future; then r + D cannot overflow either.
    // (An unavailable clock, begin <= 0, fails the second comparison.)
    const bool usable = valid_ && ready_observed_mono_ns > 0 && ready_observed_mono_ns <= begin &&
                        ready_observed_mono_ns <= std::numeric_limits<std::int64_t>::max() - delay_ns_;
    // The copy is made into the producer's own buffer: a failure leaves the slot as it was.
    bool copied = false;
    if (usable) {
        try {
            scratch_ = body;
            copied = true;
        } catch (...) {}
    }
    std::unique_lock<std::mutex> lock(mutex_);
    ++counters_.offered;
    if (faulted_) {
        ++counters_.refused_faulted;
        return OfferResult::Faulted;
    }
    if (closed_) {
        ++counters_.refused_closed;
        return OfferResult::Closed;
    }
    if (!usable) {
        ++counters_.refused_invalid;
        fault_locked("Objects release: frame " + std::to_string(frame_id) + " has no usable ready time (ready_ns=" +
                     std::to_string(ready_observed_mono_ns) + " now_ns=" + std::to_string(begin) + ")");
        lock.unlock();
        wake_.notify_all();
        return OfferResult::Invalid;
    }
    if (!copied) {
        ++counters_.refused_copy;
        fault_locked("Objects release: copying the body of frame " + std::to_string(frame_id) + " failed");
        lock.unlock();
        wake_.notify_all();
        return OfferResult::CopyFailed;
    }
    // Second clock read: the hand-over itself, after the copy and with the lock held. It is the
    // time of the state change below (this body pending, a pending predecessor overwritten).
    std::int64_t now = 0;
    try { now = clock_(); } catch (...) { now = 0; }
    if (now < begin) {
        ++counters_.refused_invalid;
        fault_locked("Objects release: clock unavailable or backwards at the hand-over of frame " +
                     std::to_string(frame_id) + " (begin_ns=" + std::to_string(begin) + " now_ns=" + std::to_string(now) + ")");
        lock.unlock();
        wake_.notify_all();
        return OfferResult::Invalid;
    }
    ++counters_.admitted;
    const std::int64_t deadline = ready_observed_mono_ns + delay_ns_;
    std::int64_t due = deadline;
    if (has_pending_) {
        // Latest wins: a body not selected yet is replaced, whatever the release thread is doing
        // (in a put, in the observer, or not scheduled since its due). Its release time passes on,
        // so that replacements cannot postpone the release.
        if (pending_.due_ns < due) due = pending_.due_ns;
        ++counters_.overwritten;
        ReleaseRecord record = describe_locked(pending_, ReleaseEnd::Overwritten);
        record.replaced_by = frame_id;
        record_locked(record);
    }
    std::swap(pending_.body, scratch_);
    pending_.frame_id = frame_id;
    pending_.ready_ns = ready_observed_mono_ns;
    pending_.offer_begin_ns = begin;
    pending_.handoff_ns = now;
    pending_.deadline_ns = deadline;
    pending_.due_ns = due;
    pending_.claim_ns = 0;
    pending_.reason = ReleaseReason::None;
    has_pending_ = true;
    lock.unlock();
    wake_.notify_all();
    return OfferResult::Admitted;
}

void ObjectsRelease::close() noexcept {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        closed_ = true;
        // What has no commitment ends here, so that when close() returns at most one body is
        // still inside: a committed one, whose single call completes on the release thread.
        discard_uncommitted_locked();
    }
    wake_.notify_all();
}

void ObjectsRelease::join() noexcept {
    if (thread_.joinable() && thread_.get_id() != std::this_thread::get_id()) thread_.join();
}

void ObjectsRelease::wake() noexcept { wake_.notify_all(); }

bool ObjectsRelease::faulted() const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    return faulted_;
}

std::string ObjectsRelease::fault_reason() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return fault_reason_;
}

ReleaseCounters ObjectsRelease::counters() const {
    std::lock_guard<std::mutex> lock(mutex_);
    ReleaseCounters out = counters_;
    out.pending = has_pending_ ? 1U : 0U;
    out.in_flight = has_in_flight_ ? 1U : 0U;
    return out;
}

void ObjectsRelease::run() noexcept {
    set_current_thread_name(thread_name_.c_str());
    std::vector<ReleaseRecord> finished;
    try { finished.reserve(kRecordCapacity); } catch (...) {}
    const auto now_ns = [this]() noexcept {
        try { return clock_(); } catch (...) { return std::int64_t{0}; }
    };
    std::unique_lock<std::mutex> lock(mutex_);
    for (;;) {
        // Finished records go to the observer without the lock, one batch at a time. After a batch
        // the loop goes on to the stop and due checks below instead of starting over, so records
        // that keep arriving (a replacement makes one) cannot keep a due body from being selected.
        if (!records_.empty()) {
            finished.swap(records_);
            lock.unlock();
            if (observer_) {
                for (const auto& record : finished) {
                    try { observer_(record); } catch (...) {}
                }
            }
            finished.clear();
            lock.lock();
        }
        if (closed_) {
            // A stop or a fault: nothing without a commitment is published (close() has already
            // done this for a stop; a fault latched by this thread or by offer() ends here).
            discard_uncommitted_locked();
            if (!records_.empty()) continue;
            break;
        }
        if (!has_pending_) {
            if (records_.empty()) wake_.wait(lock);
            continue;
        }
        const std::int64_t now = now_ns();
        if (now <= 0) {
            fault_locked("Objects release: monotonic clock unavailable");
            continue;
        }
        if (now < pending_.due_ns) {
            // A relative wait computed on our own clock; every wake-up reads the slot and the clock
            // again (a replacement can bring an earlier or the same due, never a later one). Records
            // that arrived meanwhile are handed to the observer first.
            if (records_.empty()) wake_.wait_for(lock, std::chrono::nanoseconds(pending_.due_ns - now));
            continue;
        }
        // The selection and the publication commitment, one step under the lock and only while not
        // closed: the body taken is the latest handed over, and from here exactly one call of the
        // publish function is made for it and completes, whatever close() or a fault does meanwhile.
        // in_flight_ is this thread's alone until the call is accounted below; a hand-over meanwhile
        // waits in the pending place.
        select_locked(now);
        lock.unlock();
        const std::int64_t call_ns = now_ns();
        PublishResult result;
        try {
            result = publish_(in_flight_.body);
        } catch (...) {
            result.error = PublishError::Internal;
        }
        const std::int64_t return_ns = now_ns();
        lock.lock();
        const bool attempted = result.audit.attempted;
        ReleaseRecord record = describe_locked(in_flight_, attempted ? ReleaseEnd::Published : ReleaseEnd::ChannelRefused);
        record.call_ns = call_ns;
        record.return_ns = return_ns;
        record.result = result;
        if (attempted) {
            ++counters_.published;
            switch (result.audit.receipt.outcome) {
                case PutOutcome::Accepted: ++counters_.accepted; break;
                case PutOutcome::Rejected: ++counters_.rejected; break;
                default: ++counters_.unknown; break;
            }
        } else {
            ++counters_.channel_refused;
        }
        // Transport outcomes are counted and never retried; anything else is a local fault.
        if (result.error != PublishError::None && result.error != PublishError::PutRejected &&
            result.error != PublishError::PutUnknown) {
            ++counters_.contract_faults;
            fault_locked("Objects release: publish contract fault error=" +
                         std::to_string(static_cast<int>(result.error)) + " frame_id=" +
                         std::to_string(in_flight_.frame_id));
        }
        record_locked(record);
        has_in_flight_ = false;
    }
}
}  // namespace perception::rns
