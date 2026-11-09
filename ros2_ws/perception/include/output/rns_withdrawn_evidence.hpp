/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_withdrawn_evidence.hpp
 * Brief: r5 fit-failed BIN EVIDENCE: raw depth support + independent semantic block.
 *
 * Description:
 * What problem this solves. When the per-frame ground fit fails (19 section 3.2A), every FREE
 * distance of that frame would rest on the prior plane alone. The evidence that would justify
 * trusting the prior plane (height and tilt deviation, PD-18/PD-19) is not available in this
 * delivery, so the frozen rule applies in full (A19-FIT-3): all d_free = null, no negative
 * obstacles, no geometric block that depends on the failed plane, but an independently valid
 * semantic (S) block is kept and src bit1 still reports where raw depth samples exist. Status
 * then carries both ground_fit_fallback and ground_free_withdrawn.
 *
 * What this class produces. Only the per-bin evidence of such a frame: kG where raw samples
 * exist, kS plus d_block where the S owner has a fresh distance. It is still not a complete
 * Profile or a put; the separate r6 withdrawn-Profile adapter (rns_withdrawn_profile) supplies
 * the approved null blind_near_m representation and the envelope.
 *
 * Why so much revalidation. The bins are read later by the adapter, possibly after the S owner
 * or the depth stream moved on. bins() therefore re-checks source, stream, S revision and S
 * age at the moment of reading, and returns nullptr rather than stale evidence.
 */

#pragma once
#include "output/rns_semantic.hpp"

namespace perception::rns {
// ReadyEvidenceOnly is the only status whose bins may be used; "EvidenceOnly" says it
// is not yet a publishable Profile.
enum class WithdrawnStatus { NotBuilt, ReadyEvidenceOnly, InvalidSource, InvalidSemantic };
// Summary of the last build(); counts are for logs and tests.
struct WithdrawnReport {
    WithdrawnStatus status = WithdrawnStatus::NotBuilt;
    geometry::CaptureStamp capture;
    SemanticStatus semantic_status = SemanticStatus::NotProcessed;
    std::size_t semantic_bins = 0, raw_sample_bins = 0;
    std::uint64_t raw_samples = 0;
    bool free_withdrawn = false;
    // Required diagnostic reasons when the application publishes this state.
    // No prior-plane FREE/obstacle/negative classification is enabled.
    std::array<std::string_view, 2> reasons{};
};
class WithdrawnSemanticEvidence {
public:
    // Explicit G-only mode: no semantic owner/configuration or fabricated
    // measurement. Only current raw FitFailed support is retained.
    explicit WithdrawnSemanticEvidence(const geometry::NativeGeometry&);
    // Exact projector/geometry/Mailbox owners outlive this serial consumer.
    // Projector must explicitly allow FitFailed; its default remains normal-only.
    explicit WithdrawnSemanticEvidence(const SemanticProjector&);
    // Nullable form for an adapter that decides at construction time whether
    // this deployment has an S owner; nullptr is the G-only constructor. The
    // geometry must be the projector's own.
    WithdrawnSemanticEvidence(const geometry::NativeGeometry&, const SemanticProjector*);
    WithdrawnSemanticEvidence(const WithdrawnSemanticEvidence&) = delete;
    WithdrawnSemanticEvidence& operator=(const WithdrawnSemanticEvidence&) = delete;
    // With an S owner: the capture stamp is the one the S owner processed.
    const WithdrawnReport& build(const native_depth::Mailbox&, std::int64_t now_ns) noexcept;
    // G-only caller supplies the same verified capture stamp used by its fast
    // consumer. Not a way to bypass a configured semantic owner.
    const WithdrawnReport& build(const native_depth::Mailbox&, const geometry::CaptureStamp&,
                                std::int64_t now_ns) noexcept;
    const WithdrawnReport& report() const noexcept { return report_; }
    // Exact immutable geometry owner, never a copied calibration/configuration.
    const geometry::NativeGeometry& geometry() const noexcept { return geometry_; }
    // The S owner this evidence reads, or nullptr in G-only mode. For the
    // commit-instant recheck of the snapshot lifetime by the Profile adapter.
    const SemanticProjector* semantic() const noexcept { return semantic_; }
    // Revalidated borrowed view. NULL on changed source, stream, projector
    // revision, clock rollback or expired S. No naked unvalidated bins getter.
    const std::vector<Bin>* bins(const native_depth::Mailbox&, std::int64_t now_ns) const noexcept;
    // Fixed bin storage, allocated once at construction (for memory accounting).
    std::size_t storage_bytes() const noexcept;
private:
    // Reset every bin and the report; a rejected build leaves no partial evidence.
    void clear(WithdrawnStatus) noexcept;
    const WithdrawnReport& build_current(const native_depth::Mailbox&, const geometry::CaptureStamp&,
                                       std::int64_t now_ns) noexcept;
    // The revalidation shared by build() and bins(), see the file header.
    bool current(const native_depth::Mailbox&, std::int64_t now_ns) const noexcept;
    const geometry::NativeGeometry& geometry_;
    const SemanticProjector* semantic_ = nullptr;
    const native_depth::Mailbox* mailbox_ = nullptr;
    std::vector<Bin> bins_;
    WithdrawnReport report_;
    // Watermarks of the last accepted frame: a frame is evidence at most once, and
    // stream, sequence, capture time and check time never go backwards.
    native_depth::SourceId last_;
    std::uint64_t epoch_ = 0, revision_ = 0;
    std::int64_t last_capture_ns_ = -1, checked_at_ns_ = -1;
};
} // namespace perception::rns
