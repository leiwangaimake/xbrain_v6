/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_gen_notices.cpp
 * Brief: Host test of the GEN one-shot notices: startup config digest and RT router loss/restore.
 *
 * Description:
 * What it checks. The config_digest notice carries common_digest and config_rev (10 section
 * 5.4.4); RT router loss and restore produce one edge each. Injected transport and clock, no Zenoh
 * session.
 */

#include "service/gen_notices.hpp"

#include "output/rns_gen_events.hpp"

#include <yaml-cpp/yaml.h>

#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace perception;
using namespace perception::service;

namespace {
int checks = 0, failures = 0;
#define CHECK(cond) do { ++checks; if (!(cond)) { ++failures; std::cerr << "FAIL " << __LINE__ << ": " #cond "\n"; } } while (0)

struct Capture final : rns::RnsTransport {
    struct Put { std::string key, packet; rns::PublishQos qos; };
    std::vector<Put> puts;
    rns::PutOutcome outcome = rns::PutOutcome::Accepted;
    rns::PutReceipt put(std::string_view key, std::string_view packet, rns::PublishQos qos) override {
        puts.push_back({std::string(key), std::string(packet), qos});
        return {outcome, outcome == rns::PutOutcome::Accepted ? 0 : -1};
    }
};
struct Clock final : rns::PublishClock {
    rns::Millis wall = 1758100000123, mono = 4821337;
    rns::Millis wall_millis() noexcept override { return wall; }
    rns::Millis mono_millis() noexcept override { return mono; }
};

YAML::Node parse(const std::string& packet) { return YAML::Load(packet); }
}  // namespace

int main() {
    const std::string nonce = "0123456789abcdef0123456789abcdef";
    rns::ProcessEnvelope envelope("xbrain-01", "0123abcd", 5000);
    Clock clock;
    Capture info, fault;
    GenNotices notices(envelope, clock, info, fault, nonce, "tcp/127.0.0.1:7449");
    // Bare GEN keys (11 section 1.1.6 PC-2); the robot id is only in the envelope.
    CHECK(notices.info_key() == "event/info/perception");
    CHECK(notices.fault_key() == "event/fault/perception");

    // ---- config digest ----
    ConfigDigestFacts facts{"0123456789abcdef", "2026-09-17T00:00:00Z+g\"quoted\"\nnewline",
                            std::string(64, 'a'), std::string(64, 'b'), std::string(64, 'c'),
                            "transform", "sha_mismatch", false, std::string(64, 'd'), "record \"x\" differs\nline"};
    const auto r = notices.config_digest(facts);
    CHECK(r.attempted && r.encoded && r.stamped && r.receipt.outcome == rns::PutOutcome::Accepted);
    CHECK(info.puts.size() == 1 && fault.puts.empty());
    CHECK(info.puts[0].key == notices.info_key());
    const auto q = rns::GenEvents::qos();
    CHECK(info.puts[0].qos.priority == q.priority && info.puts[0].qos.reliable == q.reliable &&
          info.puts[0].qos.express == q.express && info.puts[0].qos.drop == q.drop && !q.drop);
    const auto doc = parse(info.puts[0].packet);
    CHECK(doc["v"].as<int>() == 1 && doc["rid"].as<std::string>() == "xbrain-01");
    CHECK(doc["ts"].as<std::string>() == "1758100000.123" && doc["mono"].as<std::string>() == "4821.337");
    CHECK(doc["boot"].as<std::string>() == "0123abcd" && doc["seq"].as<std::uint64_t>() == r.seq && r.seq == 1);
    CHECK(doc["src"].as<std::string>() == "perception" && doc["ts_sync"].as<bool>() == false);
    const auto data = doc["data"];
    CHECK(data["eid"].as<std::string>() == "p-" + nonce + "-1");
    CHECK(data["sev"].as<std::string>() == "info" && data["cat"].as<std::string>() == "perception");
    CHECK(data["channel"].as<std::string>() == "normal" && data["title"].IsScalar());
    CHECK(data["detail"]["kind"].as<std::string>() == "config_digest");
    CHECK(data["detail"]["config_rev"].as<std::string>() == facts.config_rev);  // escaping round trip
    CHECK(data["detail"]["runtime_switch_digest"].as<std::string>() == facts.runtime_switch_digest);
    CHECK(data["detail"]["geometry"].as<std::string>() == "transform" &&
          data["detail"]["calibration"].as<std::string>() == "sha_mismatch" &&
          data["detail"]["extrinsic_calibrated"].as<bool>() == false &&
          data["detail"]["calibration_record_sha256"].as<std::string>() == std::string(64, 'd') &&
          data["detail"]["calibration_detail"].as<std::string>() == facts.calibration_detail);
    CHECK(data["detail"]["observed_mono_ms"].as<long long>() == clock.mono);
    bool twice = false;
    try { notices.config_digest(facts); } catch (const std::logic_error&) { twice = true; }
    CHECK(twice && info.puts.size() == 1);

    // ---- RT router edges ----
    CHECK(!notices.observe_rt_routers(2).attempted && notices.rt_state() == RtRouterState::Connected);
    CHECK(!notices.observe_rt_routers(1).attempted);
    clock.mono += 1000;
    const auto lost = notices.observe_rt_routers(0);
    CHECK(lost.attempted && lost.key == notices.fault_key() && fault.puts.size() == 1);
    const auto ldoc = parse(fault.puts[0].packet);
    CHECK(ldoc["data"]["sev"].as<std::string>() == "fault" && ldoc["data"]["detail"]["kind"].as<std::string>() == "rt_session_lost");
    CHECK(ldoc["data"]["detail"]["endpoint"].as<std::string>() == "tcp/127.0.0.1:7449");
    CHECK(ldoc["seq"].as<std::uint64_t>() == 2);
    CHECK(!notices.observe_rt_routers(0).attempted && !notices.observe_rt_routers(-1).attempted);
    CHECK(fault.puts.size() == 1 && notices.rt_state() == RtRouterState::Lost);
    clock.mono += 2500;
    const auto back = notices.observe_rt_routers(3);
    CHECK(back.attempted && back.key == notices.info_key() && info.puts.size() == 2);
    const auto bdoc = parse(info.puts[1].packet);
    CHECK(bdoc["data"]["sev"].as<std::string>() == "info" && bdoc["data"]["detail"]["kind"].as<std::string>() == "rt_session_restored");
    CHECK(bdoc["data"]["detail"]["lost_mono_ms"].as<long long>() == clock.mono - 2500);
    CHECK(bdoc["seq"].as<std::uint64_t>() == 3 && notices.rt_state() == RtRouterState::Connected);

    // first observation without a router is a loss; a failed query counts as none
    {
        rns::ProcessEnvelope env2("xbrain-01", "0123abcd", 5000);
        Capture i2, f2;
        f2.outcome = rns::PutOutcome::Rejected;
        GenNotices n2(env2, clock, i2, f2, nonce, "tcp/127.0.0.1:7449");
        const auto first = n2.observe_rt_routers(-1);
        CHECK(first.attempted && first.receipt.outcome == rns::PutOutcome::Rejected && f2.puts.size() == 1);
        CHECK(!n2.observe_rt_routers(0).attempted && f2.puts.size() == 1);  // rejected put is not retried
    }
    // identity validation
    for (const auto* bad : {"", "0123456789abcdef0123456789abcde", "0123456789ABCDEF0123456789abcdef",
                            "00000000000000000000000000000000"}) {
        bool thrown = false;
        try { GenNotices n(envelope, clock, info, fault, bad, "x"); } catch (const std::invalid_argument&) { thrown = true; }
        CHECK(thrown);
    }
    CHECK(json_string(std::string("a\x01" "b")) == "\"a\\u0001b\"");

    std::cout << "{\"result\":\"" << (failures ? "FAIL" : "PASS") << "\",\"checks\":" << checks
              << ",\"failures\":" << failures << ",\"scope\":\"gen_notices_host\",\"zenoh\":false}\n";
    return failures ? 1 : 0;
}
