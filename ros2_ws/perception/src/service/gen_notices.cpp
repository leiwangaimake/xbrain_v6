/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: gen_notices.cpp
 * Brief: GenNotices: startup config digest and RT router loss/restore events.
 *
 * Description:
 * What problem this solves. Implements gen_notices.hpp. The packets are rare (one at startup,
 * one per router transition), so they are assembled with std::string on the GEN thread; the RT
 * publishing path is not involved.
 *
 * Edge rule. A router transition produces exactly one event per edge: fault when the router is
 * lost (including a first observation without a router), info when it comes back. Repeated
 * observations of the same state produce nothing, so a long outage is one fault, not a flood.
 */
#include "service/gen_notices.hpp"

#include "output/rns_gen_events.hpp"

#include <cstdio>
#include <stdexcept>
#include <utility>

namespace perception::service {
namespace {
// Milliseconds as seconds with three decimals, the envelope's ts/mono form.
std::string seconds(rns::Millis ms) {
    char text[32];
    std::snprintf(text, sizeof(text), "%lld.%03lld", static_cast<long long>(ms / 1000),
                  static_cast<long long>(ms % 1000));
    return text;
}
}  // namespace

// Titles and details are fixed or config-derived text; escaping keeps any
// quote or control character from breaking the JSON.
std::string json_string(const std::string& value) {
    std::string out = "\"";
    for (const unsigned char c : value) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (c < 0x20) {
                    char escaped[7];
                    std::snprintf(escaped, sizeof(escaped), "\\u%04x", c);
                    out += escaped;
                } else {
                    out += static_cast<char>(c);
                }
        }
    }
    return out + "\"";
}

GenNotices::GenNotices(rns::ProcessEnvelope& envelope, rns::PublishClock& clock, rns::RnsTransport& info,
                       rns::RnsTransport& fault, std::string nonce, std::string rt_endpoint)
    : envelope_(envelope), clock_(clock), info_(info), fault_(fault), nonce_(std::move(nonce)),
      rt_endpoint_(std::move(rt_endpoint)) {
    // The nonce makes event ids unique across restarts (eid = p-<nonce>-<seq>),
    // so it must be exactly 32 lowercase hex digits and not all zero.
    bool nonzero = false;
    if (!envelope_.valid() || nonce_.size() != 32) throw std::invalid_argument("GEN notice identity");
    for (const char c : nonce_) {
        if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) throw std::invalid_argument("GEN notice nonce");
        nonzero |= c != '0';
    }
    if (!nonzero) throw std::invalid_argument("GEN notice nonce zero");
    // Bare keys event/info/perception and event/fault/perception (11 section 1.1.6 PC-2);
    // the robot id is in the envelope.
    info_key_ = "event/info/perception";
    fault_key_ = "event/fault/perception";
}

NoticeResult GenNotices::send(const char* sev, rns::RnsTransport& transport, const std::string& key,
                              const std::string& kind, const std::string& title,
                              const std::string& detail_fields) {
    NoticeResult result;
    result.key = key;
    result.kind = kind;
    // No valid time or no envelope sequence: nothing is sent (never a guessed stamp).
    const auto wall = clock_.wall_millis();
    const auto mono = clock_.mono_millis();
    rns::EnvelopeStamp stamp;
    if (wall < 0 || mono < 0 || !envelope_.stamp(wall, mono, stamp)) return result;
    result.stamped = true;
    result.seq = stamp.seq;
    // Envelope (11 section 3.0) around the event body (11 section 6.1, channel
    // normal per 11 section 6.2); detail carries kind and the observation time.
    std::string packet = "{\"v\":1,\"rid\":" + json_string(std::string(envelope_.rid())) +
        ",\"ts\":" + seconds(stamp.wall_ms) + ",\"mono\":" + seconds(stamp.mono_ms) +
        ",\"boot\":" + json_string(std::string(envelope_.boot())) + ",\"seq\":" + std::to_string(stamp.seq) +
        ",\"src\":\"perception\",\"ts_sync\":" + (stamp.ts_sync ? "true" : "false") +
        ",\"data\":{\"eid\":\"p-" + nonce_ + "-" + std::to_string(stamp.seq) + "\",\"sev\":\"" + sev +
        "\",\"cat\":\"perception\",\"channel\":\"normal\",\"title\":" + json_string(title) +
        ",\"detail\":{\"kind\":" + json_string(kind) + ",\"observed_mono_ms\":" + std::to_string(stamp.mono_ms) +
        detail_fields + "}}}";
    // Same packet bound as GenEvents; an oversize notice is reported, not cut.
    if (packet.size() > rns::GenEvents::kMaxPacket) return result;
    result.encoded = true;
    result.bytes = packet.size();
    result.attempted = true;
    // One put, no retry (a retry could duplicate a delivered event); a throwing
    // transport is recorded as an Unknown outcome.
    try {
        result.receipt = transport.put(key, packet, rns::GenEvents::qos());
    } catch (...) {
        result.receipt = {rns::PutOutcome::Unknown, 0};
    }
    return result;
}

NoticeResult GenNotices::config_digest(const ConfigDigestFacts& f) {
    // 10 section 5.4.4: one config digest per process start. The fields let the platform tell
    // from the event log alone which configuration, manifest, runtime switches and calibration
    // this perception process ran with (a second call would be a caller bug).
    if (digest_sent_) throw std::logic_error("config_digest notice is sent once per process");
    digest_sent_ = true;
    return send("info", info_, info_key_, "config_digest", "Perception configuration digest",
                ",\"common_digest\":" + json_string(f.common_digest) + ",\"config_rev\":" + json_string(f.config_rev) +
                ",\"perception_config_sha256\":" + json_string(f.perception_config_sha256) +
                ",\"manifest_sha256\":" + json_string(f.manifest_sha256) +
                ",\"runtime_switch_digest\":" + json_string(f.runtime_switch_digest) +
                ",\"geometry\":" + json_string(f.geometry) + ",\"calibration\":" + json_string(f.calibration) +
                ",\"extrinsic_calibrated\":" + (f.extrinsic_calibrated ? "true" : "false") +
                ",\"calibration_record_sha256\":" + json_string(f.calibration_record_sha256) +
                ",\"calibration_detail\":" + json_string(f.calibration_detail));
}

NoticeResult GenNotices::observe_rt_routers(int routers) {
    // routers <= 0 (including a failed query, -1) counts as no router.
    const bool connected = routers > 0;
    // Lost -> Connected: one info event carrying when the loss began, so the outage length can
    // be read from the pair. Connected -> Connected and the first Connected: nothing.
    if (connected) {
        if (state_ == RtRouterState::Lost) {
            const auto lost = lost_ms_;
            state_ = RtRouterState::Connected;
            return send("info", info_, info_key_, "rt_session_restored", "Perception RT session router restored",
                        ",\"endpoint\":" + json_string(rt_endpoint_) + ",\"routers\":" + std::to_string(routers) +
                        ",\"lost_mono_ms\":" + std::to_string(lost));
        }
        state_ = RtRouterState::Connected;
        return {};
    }
    // Still lost: no repeat. First loss (or no router at the very first observation): one fault
    // event, and the loss time is kept for the restore event.
    if (state_ == RtRouterState::Lost) return {};
    state_ = RtRouterState::Lost;
    lost_ms_ = clock_.mono_millis();
    return send("fault", fault_, fault_key_, "rt_session_lost", "Perception RT session has no router",
                ",\"endpoint\":" + json_string(rt_endpoint_) + ",\"routers\":" + std::to_string(routers));
}

}  // namespace perception::service
