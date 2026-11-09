/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_declared_keys.cpp
 * Brief: A19-BOOT-4 / PSC-5: an unregistered key refuses the start; the comparison is literal.
 *
 * Description:
 * What it checks. check_declared_keys() accepts exactly the registered set, in any order, and
 * refuses, naming the key: a key that is not registered at all; a registered key with one more
 * segment, one more character, or cut short (the contract's own mutant: a whitelist compared by
 * prefix lets these through); the same path under another robot id or without one; every
 * wildcard form of a key expression, also where the rest is a registered prefix; a key declared
 * twice; a registered key that is not declared; an empty declaration.
 * The registry used here is the six keys of 11 section 2.2.1 for one robot, written out, not
 * taken from the service: the service's own table is compared with it in test_service_config.
 */

#include "service/declared_keys.hpp"

#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using perception::service::check_declared_keys;
int checks = 0;
void check(bool value) {
    ++checks;
    if (!value) throw std::runtime_error("declared keys check " + std::to_string(checks));
}
const std::vector<std::string> kRegistry = {
    "xbrain/xbrain-01/rt/perception/profile", "xbrain/xbrain-01/rt/perception/objects",
    "xbrain/xbrain-01/rt/perception/status",  "event/info/perception",
    "event/warn/perception", "event/fault/perception"};

bool refused(const std::vector<std::string>& declared, const std::string& key, const std::string& why) {
    const auto verdict = check_declared_keys(kRegistry, declared);
    return !verdict.ok && verdict.key == key && verdict.reason.find(why) != std::string::npos;
}
// The registered set with one key replaced or one more key added.
std::vector<std::string> with(std::size_t at, const std::string& key) {
    auto keys = kRegistry;
    if (at < keys.size()) keys[at] = key; else keys.push_back(key);
    return keys;
}
}  // namespace

int main() {
    try {
        // The registered set itself, in the registry's order and in any other.
        auto verdict = check_declared_keys(kRegistry, kRegistry);
        check(verdict.ok && verdict.key.empty() && verdict.reason.empty());
        auto shuffled = kRegistry;
        std::reverse(shuffled.begin(), shuffled.end());
        check(check_declared_keys(kRegistry, shuffled).ok);
        std::swap(shuffled[1], shuffled[4]);
        check(check_declared_keys(kRegistry, shuffled).ok);

        // A19-BOOT-4: one key that is not registered, in place of a registered one or next to all six.
        for (const std::string& extra : {std::string("xbrain/xbrain-01/rt/perception/targets"),
                                         std::string("xbrain/xbrain-01/rt/perception/profile/extra"),   // one more segment
                                         std::string("xbrain/xbrain-01/rt/perception/profiles"),        // one more character
                                         std::string("xbrain/xbrain-01/rt/perception/profil"),          // cut short
                                         std::string("xbrain/xbrain-01/rt/perception"),                 // a parent
                                         std::string("xbrain/xbrain-01/rt/perception/"),
                                         std::string("xbrain/xbrain-02/rt/perception/profile"),         // another robot
                                         std::string("xbrain/rt/perception/profile"),                   // no robot id
                                         std::string("rt/perception/profile"),
                                         std::string("xbrain/xbrain-01/event/info/perception/x"),
                                         std::string("xbrain/xbrain-01/event/debug/perception"),
                                         // GEN events are bare (11 PC-2): the robot-id form is p5_gateway's cloud layer.
                                         std::string("xbrain/xbrain-01/event/info/perception"),
                                         std::string("xbrain/xbrain-01/event/fault/perception"),
                                         std::string("event/info/perception/x"),                         // one more segment
                                         std::string("event/info/perceptio"),                            // cut short
                                         std::string("event/debug/perception"),                          // no such severity
                                         std::string("/event/info/perception"),
                                         std::string("event/info"),                                      // a parent
                                         std::string("XBRAIN/xbrain-01/rt/perception/profile"),         // another case
                                         std::string(" xbrain/xbrain-01/rt/perception/profile"),
                                         std::string("")}) {
            check(refused(with(0, extra), extra, "not registered"));
            check(refused(with(kRegistry.size(), extra), extra, "not registered"));
            check(refused({extra}, extra, "not registered"));
        }
        // No wildcard, also where everything else is a registered prefix.
        for (const std::string& wild : {std::string("xbrain/xbrain-01/rt/perception/*"), std::string("xbrain/xbrain-01/rt/**"),
                                        std::string("xbrain/*/rt/perception/profile"), std::string("xbrain/xbrain-01/rt/perception/pro$*"),
                                        std::string("**"), std::string("xbrain/xbrain-01/rt/perception/profile?"),
                                        std::string("xbrain/xbrain-01/rt/perception/profile#"),
                                        std::string("event/*/perception"), std::string("event/**")}) {
            check(refused(with(0, wild), wild, "wildcard"));
            check(refused(with(kRegistry.size(), wild), wild, "wildcard"));
        }
        // A wildcard in the REGISTRY would not license anything either: only whole equal strings count.
        check(!check_declared_keys({"xbrain/xbrain-01/rt/perception/*"}, {"xbrain/xbrain-01/rt/perception/profile"}).ok);

        // The first offender in the order given is the one named.
        check(refused({kRegistry[0], "a/b", "c/d"}, "a/b", "not registered"));
        // A key twice.
        auto twice = kRegistry;
        twice.push_back(kRegistry[2]);
        check(refused(twice, kRegistry[2], "twice"));
        check(refused({kRegistry[0], kRegistry[0]}, kRegistry[0], "twice"));
        // A registered key that is not declared: each of the six, and none at all.
        for (std::size_t missing = 0; missing < kRegistry.size(); ++missing) {
            auto keys = kRegistry;
            keys.erase(keys.begin() + static_cast<std::ptrdiff_t>(missing));
            check(refused(keys, kRegistry[missing], "not declared"));
        }
        check(refused({}, kRegistry[0], "not declared"));
        // An empty registry accepts only an empty declaration.
        check(check_declared_keys({}, {}).ok);
        check(!check_declared_keys({}, {kRegistry[0]}).ok);
        std::cout << "DECLARED_KEYS_PASS checks=" << checks << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "DECLARED_KEYS_FAIL " << error.what() << '\n';
        return 1;
    }
}
