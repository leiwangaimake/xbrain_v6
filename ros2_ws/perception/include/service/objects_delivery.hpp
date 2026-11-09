/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: objects_delivery.hpp
 * Brief: The Objects callback's delivery step, the owner's publish counting and the stop summary.
 *
 * Description:
 * The Objects callback of the service does three things per inference frame, in this order:
 * write the frame's S evidence to the semantic slot, pass the Objects body on, produce the T
 * mask (19 section 3.4A; the order is unchanged by the paced release). This file is the middle
 * step. Without a paced release the body is published at once on the calling thread, as it
 * always was. With one (runtime switch objects_release_delay_ms), a copy is handed to the
 * release thread and the call returns without waiting, so neither the S evidence before it nor
 * the T mask after it is held back by the release delay; S may therefore reach a Profile before
 * the same frame's Objects body is on the wire, or without it ever being (an overwritten or
 * rejected body does not take its S evidence back; that was already so for a rejected put).
 *
 * The rest of the file is what the service's main program does around that step, kept here so
 * that the host tests run the same functions as main (perception_service_main.cpp) against the
 * real release and channel:
 *   publish_counted        one call of the Objects channel with the owner's counting and the
 *                          rule that tells a transport outcome from a local contract fault;
 *   release_fault          the main loop's poll of the release's fault latch;
 *   join_objects_release,  the stop: after the release thread has ended the fault latch is read
 *   merge_release_fault    AGAIN and merged into the exit code. A committed call can return
 *                          with a contract fault during a normal stop, after the main loop has
 *                          left; without this the service would end with exit code 0.
 */
#pragma once
#include "output/rns_objects_release.hpp"
#include <atomic>
#include <cstdint>
#include <string>

namespace perception::service {
// True = the frame was passed on: published (transport outcomes included), handed over, or
// refused because the service is stopping (the release counts it as refused_closed). False = a
// local fault: the synchronous publish reported one, or the release is faulted, the frame has
// no usable ready time, or its body could not be copied.
template <class PublishNow>
bool deliver_objects(rns::ObjectsRelease* release, PublishNow&& publish_now, const rns::Objects& body,
                     std::uint64_t frame_id, std::int64_t ready_observed_mono_ns) {
    if (release == nullptr) return publish_now(body);
    const rns::OfferResult offered = release->offer(body, frame_id, ready_observed_mono_ns);
    return offered == rns::OfferResult::Admitted || offered == rns::OfferResult::Closed;
}

// The Objects owner's counters. channel_calls counts calls of the channel's publish(), made
// before the call: a body the channel refuses before any put (prepare, clock, capacity) is a
// call too. It is NOT the number of put attempts; that is the channel's own "attempted" (and
// the journal's attempts), and with a paced release its published count. Atomics: one thread
// publishes, the main thread reads for its summary lines.
struct ObjectsPublishCounters {
    std::atomic<std::uint64_t> channel_calls{0}, accepted{0}, rejected{0}, unknown{0};
};

// Transport outcomes (accepted, rejected, unknown) are counted, never retried and do not stop
// the pipeline. Any other publisher error is a local contract fault.
inline bool transport_outcome(const rns::PublishResult& result) noexcept {
    return result || result.error == rns::PublishError::PutRejected || result.error == rns::PublishError::PutUnknown;
}

// One call of the channel for one body, counted. Returns the channel's own result; *contract_fault
// (when given) is set for a result that is not a transport outcome. Channel: anything with
// PublishResult publish(const Objects&) (rns::PublisherChannel in the service).
template <class Channel>
rns::PublishResult publish_counted(Channel& channel, ObjectsPublishCounters& counters, const rns::Objects& body,
                                   bool* contract_fault = nullptr) noexcept {
    counters.channel_calls.fetch_add(1, std::memory_order_relaxed);
    const rns::PublishResult result = channel.publish(body);
    if (result) counters.accepted.fetch_add(1, std::memory_order_relaxed);
    else if (result.error == rns::PublishError::PutRejected) counters.rejected.fetch_add(1, std::memory_order_relaxed);
    else if (result.error == rns::PublishError::PutUnknown) counters.unknown.fetch_add(1, std::memory_order_relaxed);
    if (contract_fault != nullptr) *contract_fault = !transport_outcome(result);
    return result;
}

// Main loop, every pass: true when the paced release has latched a fault (a publish contract
// fault, a frame without a usable ready time or clock, a failed copy); reason gets its cause.
// It needs no further frame to be seen. No release (delay "0"): never.
inline bool release_fault(const rns::ObjectsRelease* release, std::string& reason) {
    if (release == nullptr || !release->faulted()) return false;
    reason = release->fault_reason();
    return true;
}

// What the release looked like once its thread had ended.
struct ReleaseStopSummary {
    bool present = false;  // a paced release existed
    bool faulted = false;
    std::string reason;
    rns::ReleaseCounters counters;
};

// Stop, second step. The first step, release->close(), is taken before the pipeline stops; this
// one after it: wait for the release thread (a committed call completes), then read the fault
// latch and the counters. Nothing is inside afterwards (pending = in_flight = 0).
inline ReleaseStopSummary join_objects_release(rns::ObjectsRelease* release) {
    ReleaseStopSummary summary;
    if (release == nullptr) return summary;
    release->join();
    summary.present = true;
    summary.faulted = release->faulted();
    if (summary.faulted) summary.reason = release->fault_reason();
    summary.counters = release->counters();
    return summary;
}

// The exit code after the release thread has ended. A latched release fault makes a clean exit a
// runtime fault; an exit code that was already set is kept (the earlier cause stays the cause).
// stop_reason keeps what it said and gains the release's reason unless it already is that
// reason (the main loop saw the same fault). Stopping normally, with bodies refused as Closed or
// discarded, is not a fault and changes nothing.
inline int merge_release_fault(int exit_code, std::string& stop_reason, const ReleaseStopSummary& summary,
                               int runtime_exit_code) {
    if (!summary.present || !summary.faulted) return exit_code;
    if (stop_reason.find(summary.reason) == std::string::npos || summary.reason.empty()) {
        if (!stop_reason.empty()) stop_reason += "; ";
        stop_reason += "objects release fault at stop: " + summary.reason;
    }
    return exit_code == 0 ? runtime_exit_code : exit_code;
}
}  // namespace perception::service
