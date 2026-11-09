/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_source_hooks.hpp
 * Brief: The one place an inference completion is recorded in the InferenceComplete journal.
 *
 * Description:
 * The inference module calls record_inference_completion() at its success point; the host
 * negative-control tests call the same function, so both count completions the same way.
 */
#pragma once
#include "output/rns_source_journal.hpp"

namespace perception::rns {
// Shared by the real inference success point and host negative controls.
// No detection-count argument: a successfully processed empty scene is still
// one completion. No queue/put side effects and no clock read on excluded work.
inline bool record_inference_completion(SourceJournal* journal, bool succeeded,
        const native_depth::SourceId& source) noexcept {
    return journal && succeeded &&
        journal->kind() == SourceKind::InferenceComplete && journal->record(source);
}
} // namespace perception::rns
