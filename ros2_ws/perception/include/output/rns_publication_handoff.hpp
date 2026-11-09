/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_publication_handoff.hpp
 * Brief: Certified SPSC -> Status-owner health/cadence/depth handoff.
 *
 * Description:
 * What problem this solves. Five producers write lock-free journals on their own threads:
 * Profile and Objects publications (PublicationJournal), depth arrivals, inference completions
 * and processed depth frames (SourceJournal). Once per Status beat the Status thread must see
 * all of them up to ONE common instant, so that fps, latency, gaps, cadence and the invalid
 * pixel ratio in one Status describe the same window. drain() moves every journal event up to
 * that instant ("the cut") into the Status thread's windows, or returns Busy / NotAdvanced and
 * changes nothing.
 *
 * In the service it is the full-source form, owned by the Status thread together with its
 * HealthWindow, ObjectsCadence mirror and DepthHealthWindow (perception_service_main.cpp).
 */
#pragma once
#include "output/rns_publication_journal.hpp"
#include "output/rns_health_window.hpp"
#include "output/rns_source_journal.hpp"
#include <array>

namespace perception::rns {
// Ready: windows advanced to observed_ms. Busy: a producer was mid-write, retry.
// NotAdvanced: the clock has not reached a new millisecond. Invalid: latched fault.
enum class HandoffCode { Ready, Busy, NotAdvanced, Invalid };
// Result of one drain(): the certified cut and how many events it consumed.
struct PublicationCut {
    HandoffCode code = HandoffCode::Invalid;
    Millis sampled_now_ms = -1, observed_ms = -1;
    std::array<std::size_t, 2> events{}; // Profile, Objects; includes completions/declarations
    std::size_t declarations = 0; // Subset of Objects events, never publication counts.
    std::array<std::size_t, 3> source_events{}; // Arrival, inference, processed depth
};

// Optional bounded audit sink, called by the sole consumer AFTER an accepted
// source event has been popped. The callback may copy into preallocated storage;
// it must not block, allocate, reenter this owner, mutate journals/windows or
// retain the reference. False faults the handoff, retaining consumed counts and
// any already copied evidence; no rollback or healthy cut is manufactured.
struct SourceDrainAudit {
    void* context = nullptr;
    bool (*record)(void*, const SourceEvent&, Millis cut_ms) noexcept = nullptr;
};

// Exactly ONE consumer owns this object AND the supplied HealthWindow and
// ObjectsCadence mirror. The publisher has a DIFFERENT ObjectsCadence instance.
// Never read/mutate the publisher's window or counters across threads.
//
// A cut is certified only when ALL configured producers' timestamp-to-commit intervals
// were stable and even across a shared MONOTONIC clock read and head snapshots.
// cut=floor(now)-1ms intentionally excludes the current partial millisecond:
// a new publication after the certification may still stamp floor(now).
// Busy/NotAdvanced never changes windows or advances their observation time.
// Native put may block with an already committed Attempt; it does not hold the
// generation odd. The pending count is not terminal Unknown.
//
// The original constructor certifies publications ONLY. The explicit full-source
// overload also certifies three independent, correctly typed source journals;
// all five generations/heads share ONE clock read before ANY window is mutated.
// The caller snapshots HealthWindow, ObjectsCadence and DepthHealthWindow at the
// returned observed_ms, not at a newer clock value. All objects must outlive this
// owner. Producers own their journals; only this owner consumes their events.
//
// Arrival/inference identity uses the full SourceId, not a millisecond-rounded
// timestamp. Rate-window time still uses the event's actual monotonic clock.
// Objects wire-capture deduplication is a separate publication contract.
//
// Reconnect/reconfigure declarations share the Objects FIFO and certificate;
// declarations never count as puts and never exclude actual health observations.
// Their timestamp is when the Objects owner applies them, not a backdated request.
// Neither constructor certifies combined production Status, actual source hook timing/ownership,
// an arbitrary paused critical section, SDK behaviour, or a 20 Hz cadence.
// A missing/invalid cut must NOT be relabelled as a fresh healthy snapshot.
class PublicationHealthHandoff {
public:
    PublicationHealthHandoff(Millis startup_ms, PublicationJournal& profile,
        PublicationJournal& objects, HealthWindow&, ObjectsCadence&) noexcept;
    PublicationHealthHandoff(Millis startup_ms, PublicationJournal& profile,
        PublicationJournal& objects, HealthWindow&, ObjectsCadence&,
        SourceJournal& arrival, SourceJournal& inference, SourceJournal& processed,
        DepthHealthWindow&) noexcept;
    // One certification attempt; see the .cpp for the exact sequence.
    PublicationCut drain(PublishClock&, SourceDrainAudit = {}) noexcept;
    // False once any journal fault or protocol violation was seen (sticky).
    bool valid() const noexcept { return valid_; }
    // The last certified cut (startup - 1 before the first).
    Millis last_cut() const noexcept { return last_cut_; }
private:
    std::array<PublicationJournal*, 2> journals_;
    std::array<SourceJournal*, 3> sources_{}; // All null in publication-only mode.
    DepthHealthWindow* depth_ = nullptr;
    HealthWindow& health_;
    ObjectsCadence& cadence_;
    // Cuts are strictly increasing and never before startup.
    Millis startup_, last_cut_;
    bool valid_;
};
} // namespace perception::rns
