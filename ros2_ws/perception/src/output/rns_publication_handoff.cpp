/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_publication_handoff.cpp
 * Brief: One certified cut: stable generations around one clock read, then drain up to it.
 *
 * Description:
 * drain() in order:
 *   1. Read every journal's generation; an odd one means a producer is between its clock
 *      sample and its commit, so return Busy (retry on the next Status loop, ~10 ms).
 *   2. Read the clock once: now. The cut is now - 1 ms, the last millisecond that is complete;
 *      an event committed later can still carry a time inside the current millisecond.
 *   3. Read every journal's head, then re-read every generation. If any changed, a producer
 *      committed during steps 2-3: Busy. If none changed, every event with a time <= cut is
 *      already at or before the recorded head: the cut is complete.
 *   4. Drain each publication journal up to its recorded head, stopping at the first event
 *      after the cut, and feed HealthWindow (attempts, completions) and the ObjectsCadence
 *      mirror (Objects attempts, declarations). Then drain the three source journals the same
 *      way into HealthWindow (arrivals, inference) and DepthHealthWindow (processed depth).
 *   5. Record the cut. The caller snapshots its windows at exactly observed_ms.
 *
 * Why every inconsistency is fatal. Each check guards an invariant the producers promise (time
 * order, one clock domain, correct kinds). A violation means the Status numbers can no longer
 * be trusted, so the handoff latches invalid and the service stops rather than publish a
 * Status built from a broken window ("status handoff cut invalid" in perception_service_main).
 * Busy and NotAdvanced are normal and change nothing.
 *
 * Tests. tests/test_rns_publication_handoff.cpp, tests/test_rns_source_handoff.cpp and
 * tests/test_rns_declaration_handoff.cpp.
 */
#include "output/rns_publication_handoff.hpp"

namespace perception::rns {
PublicationHealthHandoff::PublicationHealthHandoff(Millis start,
        PublicationJournal& profile, PublicationJournal& objects,
        HealthWindow& health, ObjectsCadence& cadence) noexcept
    // last_cut_ starts just before startup, so the first cut may be startup itself.
    : journals_{&profile, &objects}, health_(health), cadence_(cadence),
      startup_(start), last_cut_(start > 0 ? start - 1 : -1),
      // Valid only with a representable start and two healthy journals of the
      // right topics; an invalid handoff never produces a cut.
      valid_(start >= 0 && start <= ProcessEnvelope::kMaxMonoMs &&
          profile.valid() && objects.valid() && profile.topic() == Topic::Profile &&
          // exact topics, two distinct journals
          objects.topic() == Topic::Objects && &profile != &objects) {}

PublicationHealthHandoff::PublicationHealthHandoff(Millis start,
        PublicationJournal& profile, PublicationJournal& objects,
        HealthWindow& health, ObjectsCadence& cadence,
        SourceJournal& arrival, SourceJournal& inference, SourceJournal& processed,
        DepthHealthWindow& depth) noexcept
    : PublicationHealthHandoff(start, profile, objects, health, cadence) {
    // Full-source mode: three more journals, each of its own kind, all distinct.
    sources_ = {&arrival, &inference, &processed};
    // depth_ non-null is what switches drain() into full-source mode.
    depth_ = &depth;
    valid_ = valid_ && arrival.valid() && inference.valid() && processed.valid() &&
        arrival.kind() == SourceKind::DepthArrival &&
        inference.kind() == SourceKind::InferenceComplete &&
        processed.kind() == SourceKind::DepthProcessed &&
        &arrival != &inference && &arrival != &processed && &inference != &processed;
}

PublicationCut PublicationHealthHandoff::drain(PublishClock& clock, SourceDrainAudit audit) noexcept {
    // result defaults to Invalid; every early return below keeps or sets a code.
    PublicationCut result;
    if (!valid_) return result;
    // Step 1: generations before the clock read.
    std::array<std::uint64_t, 2> generations{}, heads{};
    std::array<std::uint64_t, 3> source_generations{}, source_heads{};
    for (std::size_t i = 0; i < journals_.size(); ++i) {
        // A faulted journal can never be certified again.
        if (!journals_[i]->valid()) { valid_ = false; return result; }
        generations[i] = journals_[i]->generation();
        if (generations[i] & 1U) { result.code = HandoffCode::Busy; return result; }
    }
    // Same for the three source journals (full-source mode).
    if (depth_) for (std::size_t i = 0; i < sources_.size(); ++i) {
        if (!sources_[i]->valid()) { valid_ = false; return result; }
        source_generations[i] = sources_[i]->generation();
        if (source_generations[i] & 1U) { result.code = HandoffCode::Busy; return result; }
    }
    // Step 2: the one clock read shared by all five journals.
    const auto now = clock.mono_millis();
    result.sampled_now_ms = now;
    // A clock before startup or not after the last cut is a clock fault.
    if (now < startup_ || now > ProcessEnvelope::kMaxMonoMs || now <= last_cut_) {
        valid_ = false; return result;
    }
    // now==0 has no completed nonnegative millisecond yet.
    const auto cut = now - 1;
    // Same millisecond as the previous cut (or the very first one): nothing new to
    // certify yet. Not an error; the caller simply tries again.
    if (cut < startup_ || cut == last_cut_) {
        result.code = HandoffCode::NotAdvanced; return result;
    }
    // Step 3: heads, then the generations again.
    for (std::size_t i = 0; i < journals_.size(); ++i) heads[i] = journals_[i]->published();
    if (depth_) for (std::size_t i = 0; i < sources_.size(); ++i)
        source_heads[i] = sources_[i]->published();
    // A changed generation means a commit happened in between: the heads may be
    // missing an event timed before the cut, so this attempt is abandoned.
    for (std::size_t i = 0; i < journals_.size(); ++i) {
        if (journals_[i]->generation() != generations[i]) {
            result.code = HandoffCode::Busy; return result;
        }
        if (!journals_[i]->valid()) { valid_ = false; return result; }
    }
    if (depth_) for (std::size_t i = 0; i < sources_.size(); ++i) {
        if (sources_[i]->generation() != source_generations[i]) {
            result.code = HandoffCode::Busy; return result;
        }
        if (!sources_[i]->valid()) { valid_ = false; return result; }
    }
    // Step 4a, publication journals.
    // Consume only the certified finite prefix; new producer traffic cannot
    // make this loop unbounded. Valid same-stream event times are monotonic.
    for (std::size_t i = 0; i < journals_.size(); ++i) {
        auto& journal = *journals_[i];
        // heads[i] was recorded under the certificate; later events wait.
        while (journal.consumed() < heads[i]) {
            // peek first: an event after the cut must stay in the journal.
            PublicationJournal::Event e;
            if (!journal.peek(e)) { valid_ = false; return result; }
            if (e.topic != journal.topic()) { valid_ = false; return result; }
            if (e.kind == PublicationJournal::Kind::Declaration) {
                // Declaration shares the Objects FIFO. Ordering still matters
                // when it has exactly the same millisecond as a put attempt.
                // No retroactive declaration may erase an already observed gap.
                // Only Objects declares; its time must lie between startup and now and
                // after the last published cut (never backdated into a reported window).
                if (e.topic != Topic::Objects || e.declared_ms < startup_ ||
                    e.declared_ms > now || e.declared_ms <= last_cut_) {
                    valid_ = false; return result;
                }
                // Later than the cut: leave it for the next drain.
                if (e.declared_ms > cut) break;
                if (!cadence_.declare_exclusion(e.declaration, e.declared_ms)) {
                    valid_ = false; return result;
                }
                // Applied to the mirror: release the slot and count it separately.
                PublicationJournal::Event discarded;
                if (!journal.pop(discarded)) { valid_ = false; return result; }
                ++result.events[i];
                ++result.declarations;
                // Excluding cadence does NOT exclude actual health puts,
                // arrivals, inference completions or processed-depth samples.
                continue;
            }
            // Only three kinds exist; anything else is a corrupted slot.
            if (e.kind != PublicationJournal::Kind::Attempt &&
                e.kind != PublicationJournal::Kind::Completion) {
                valid_ = false; return result;
            }
            // An Attempt is timed by put-before, a Completion by put-return.
            const bool attempt = e.kind == PublicationJournal::Event::Kind::Attempt;
            const auto time = attempt ? e.audit.put_before_ms : e.audit.put_return_ms;
            if ((attempt || e.audit.clock_valid) && time > now) {
                // A stable generation proves this committed before the clock
                // read. More than the current millisecond is a domain/protocol
                // fault, not a future sample to defer forever.
                valid_ = false; return result;
            }
            // A valid time after the cut stops this journal; it is drained next time.
            if ((attempt || e.audit.clock_valid) && time > cut) break;
            // An invalid completion clock may jump arbitrarily into the future.
            // It cannot be a watermark: settle it, fault, and never hide it as Pending.
            // A new Attempt behind an already published cut means the required
            // clock/producer protocol was violated. Do not retroactively heal it.
            if (attempt && time <= last_cut_) { valid_ = false; return result; }
            // HealthWindow pairs each Attempt with its Completion (latency, outcome).
            const auto accepted = attempt
                ? health_.observe_publication_attempt(e.topic, e.audit.seq,
                    e.audit.capture_ms, e.audit.put_before_ms, cut)
                : health_.resolve_publication(e.topic, e.audit, cut);
            // The window rejects duplicates, reorders and unmatched completions.
            if (accepted != HealthEventCode::Accepted) { valid_ = false; return result; }
            // The cadence mirror sees exactly the puts the publisher made. The publisher
            // refuses a duplicate or older capture before any Attempt (rns_publisher),
            // so New or Excluded are the only valid answers here.
            if (attempt && e.topic == Topic::Objects) {
                const auto cadence_code = cadence_.observe(e.audit.capture_ms, e.audit.put_before_ms);
                if (cadence_code != CadenceEvent::New && cadence_code != CadenceEvent::Excluded) {
                    valid_ = false; return result;
                }
            }
            // Accepted into the window: now release the slot.
            PublicationJournal::Event discarded;
            if (!journal.pop(discarded)) { valid_ = false; return result; }
            ++result.events[i];
            // Settle the actual outcome before faulting a broken return clock.
            if (!attempt && (!e.audit.clock_valid || time <= last_cut_)) {
                valid_ = false; return result;
            }
        }
    }
    // Step 4b, source journals (full-source mode only).
    // No source is ingested until the entire five-stream cut was certified.
    // SourceJournal and HealthWindow both retain full source identity. Distinct
    // SDK observations in one millisecond must not disappear through truncation.
    if (depth_) for (std::size_t i = 0; i < sources_.size(); ++i) {
        auto& journal = *sources_[i];
        // Same bounded prefix rule as for the publication journals.
        while (journal.consumed() < source_heads[i]) {
            SourceEvent e;
            // Each source journal carries exactly one kind and real times.
            if (!journal.peek(e) || e.kind != journal.kind() || e.observed_ns < 0) {
                valid_ = false; return result;
            }
            // Floor to milliseconds, the same domain as the cut.
            const auto time = e.observed_ns / 1000000;
            // mono_millis truncates a real nanosecond clock. A committed sample
            // anywhere in sampled_now's partial millisecond may be legitimate;
            // integer division avoids overflow of now*1e6 at the 64-bit edge.
            if (time > now) { valid_ = false; return result; }
            if (time > cut) break;
            // An event at or before a cut already published arrived too late: the
            // earlier Status was incomplete, so fault instead of healing it silently.
            if (time <= last_cut_) { valid_ = false; return result; }
            // Index 0 arrival and 1 inference feed the rate windows; 2 processed depth
            // feeds the 1 s invalid-pixel window.
            bool accepted = false;
            if (i == 2) {
                accepted = depth_->observe(e.depth, time, cut) == DepthWindowCode::Accepted;
            } else {
                const auto code = i == 0 ? health_.observe_depth(e.source, time, cut)
                                        : health_.observe_inference(e.source, time, cut);
                accepted = code == HealthEventCode::Accepted;
            }
            // The windows reject replays, reorders and malformed samples: fatal here.
            if (!accepted) { valid_ = false; return result; }
            // Accepted: release the slot and count it for the caller's log.
            SourceEvent discarded;
            if (!journal.pop(discarded)) { valid_ = false; return result; }
            ++result.source_events[i];
            // Optional test/diagnostic tap, called only after the event was consumed.
            if (audit.record && !audit.record(audit.context, e, cut)) {
                valid_ = false; return result;
            }
        }
    }
    // Step 5: the cut is certified and every window has been advanced to it.
    last_cut_ = cut;
    result.observed_ms = cut;
    result.code = HandoffCode::Ready;
    return result;
}
} // namespace perception::rns
