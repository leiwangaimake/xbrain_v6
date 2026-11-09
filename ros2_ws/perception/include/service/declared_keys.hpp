/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: declared_keys.hpp
 * Brief: PSC-5: the keys a process is about to declare against the keys registered for it.
 *
 * Description:
 * 19 section 6, PSC-5: "the declared key set equals the registration of 11 section 2.2.1
 * literally, no wildcard", refused at startup. check_declared_keys() is that comparison and
 * nothing else: no device, session, file or configuration is read, so it can run before any
 * session exists and in the configuration check.
 *
 * A key is accepted only when it is one of the registered keys as a whole string. A registered
 * key followed by another segment or character, a prefix of one, the same path under another
 * robot id, or anything holding a key-expression wildcard is not registered. The two sets must
 * be equal: no key twice, and no registered key left undeclared.
 *
 * The verdict names the first offending key in the order given, then the first registered key
 * that is missing; the caller turns it into a startup refusal (service_config.cpp,
 * require_declared_keys).
 */

#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace perception::service {

struct DeclaredKeyVerdict {
    bool ok = true;
    std::string key;      // the offending key (declared or registered); empty when ok
    std::string reason;   // why; empty when ok
};

inline DeclaredKeyVerdict check_declared_keys(const std::vector<std::string>& registry,
                                              const std::vector<std::string>& declared) {
    for (std::size_t i = 0; i < declared.size(); ++i) {
        const std::string& key = declared[i];
        // A wildcard would make one declaration stand for keys nobody registered.
        if (key.find_first_of("*$?#") != std::string::npos) return {false, key, "declared key holds a wildcard"};
        bool registered = false;
        for (const std::string& known : registry) registered = registered || known == key;   // the whole key
        if (!registered) return {false, key, "declared key is not registered (11 section 2.2.1)"};
        for (std::size_t j = 0; j < i; ++j)
            if (declared[j] == key) return {false, key, "key declared twice"};
    }
    for (const std::string& known : registry) {
        bool found = false;
        for (const std::string& key : declared) found = found || key == known;
        if (!found) return {false, known, "registered key is not declared"};
    }
    return {};
}

}  // namespace perception::service
