/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: gen_notices.hpp
 * Brief: One-shot GEN notices of the formal service, beside the reason-edge GenEvents.
 *
 * Description:
 * The notices: startup config digest, RT session router loss and restore.
 * 10 section 5.4.4: each process puts common_digest/config_rev into an existing
 * startup event with detail.kind="config_digest" (event/info/perception).
 * 19 section 9: RT session loss -> GEN fault event (event/fault/perception),
 * reply 2026-09-17 Q16. Exactly ONE owner thread (the GEN consumer) uses this
 * object and the transports it borrows. No retry: an Unknown/Rejected put is
 * reported, never repeated (a repeat could duplicate a delivered event).
 * The detail layout is this service's diagnostic schema, not an upstream-frozen
 * sub-type schema.
 */
#pragma once

#include "output/rns_envelope.hpp"
#include "output/rns_publisher.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace perception::service {

// What the startup config_digest notice reports: which configuration, manifest and
// runtime switches this process runs with, and the calibration verdict, so a log
// reader can tie every later message to one exact setup.
struct ConfigDigestFacts {
    std::string common_digest, config_rev, perception_config_sha256, manifest_sha256;
    std::string runtime_switch_digest;
    // Geometry mode ("absent" | "transform") and the calibration verdict
    // (service::calibration_code_name, extrinsic_calibrated, record sha/detail).
    std::string geometry = "absent", calibration = "no_transform";
    bool extrinsic_calibrated = false;
    std::string calibration_record_sha256, calibration_detail;
};

// What happened to one notice: stamped (envelope), encoded (fits kMaxPacket),
// attempted (put called) and the transport receipt. Empty when nothing was due.
struct NoticeResult {
    bool attempted = false, encoded = false, stamped = false;
    std::string key, kind;
    std::uint64_t seq = 0;
    rns::PutReceipt receipt;
    std::size_t bytes = 0;
};

// RT session router view; Unknown until the first observation.
enum class RtRouterState { Unknown, Connected, Lost };

class GenNotices {
public:
    // nonce: 32 lowercase hex digits, nonzero, new per process instance.
    GenNotices(rns::ProcessEnvelope&, rns::PublishClock&, rns::RnsTransport& info,
               rns::RnsTransport& fault, std::string nonce, std::string rt_endpoint);
    GenNotices(const GenNotices&) = delete;
    GenNotices& operator=(const GenNotices&) = delete;

    // Once per process (a second call is a logic error).
    NoticeResult config_digest(const ConfigDigestFacts&);
    // routers: result of the RT session router query (-1 = query failed, treated
    // as no router). Emits fault on entering Lost (including a first observation
    // without a router) and info on Lost -> Connected. Otherwise no event.
    NoticeResult observe_rt_routers(int routers);
    RtRouterState rt_state() const noexcept { return state_; }
    const std::string& info_key() const noexcept { return info_key_; }
    const std::string& fault_key() const noexcept { return fault_key_; }

private:
    // Build the 11 section 3.0 envelope + 11 section 6.1 event and put it once.
    NoticeResult send(const char* sev, rns::RnsTransport&, const std::string& key,
                      const std::string& kind, const std::string& title, const std::string& detail_fields);
    rns::ProcessEnvelope& envelope_;
    rns::PublishClock& clock_;
    rns::RnsTransport& info_;
    rns::RnsTransport& fault_;
    std::string nonce_, rt_endpoint_, info_key_, fault_key_;
    RtRouterState state_ = RtRouterState::Unknown;
    // Monotonic ms when the router was lost, reported in the restore notice.
    rns::Millis lost_ms_ = -1;
    bool digest_sent_ = false;
};

// JSON string literal (quotes included) with RFC 8259 escaping.
std::string json_string(const std::string&);

}  // namespace perception::service
