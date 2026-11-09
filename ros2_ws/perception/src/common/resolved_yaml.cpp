/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: resolved_yaml.cpp
 * Brief: Bounded pre-parser that adapts our numeric vectors for the pinned yaml_lite reader.
 *
 * Description:
 * What problem this solves. Implements resolved_yaml.hpp. The platform's yaml_lite reader has no
 * sequence node and silently keeps the last of duplicate keys. This pass walks the document
 * line by line with the same indentation rules, rewrites only the declared numeric vectors from
 * block style ("- 1.0" lines) into one flow scalar ("[1.0,...]"), turns empty leaves into an
 * explicit null, and refuses every construct yaml_lite would misread (duplicates, tabs,
 * sequences elsewhere, aliases/tags, unresolved ${...}).
 */
#include "common/resolved_yaml.hpp"

#include <xbrain/config/yaml_lite.h>

#include <algorithm>
#include <map>
#include <cctype>
#include <cmath>
#include <set>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace perception::config {
namespace {
const std::string extrinsic = "perception.extrinsic_base_cam.";
const std::string internal = "perception.native_geometry_setup.";

[[noreturn]] void invalid(const std::string& path, const std::string& why) {
    throw std::invalid_argument(path + ": " + why);
}
void require(bool ok, const std::string& path, const char* why) {
    if (!ok) invalid(path, why);
}
std::string trim(const std::string& s) { return xbrain::config::detail::trim(s); }
std::string content(const std::string& s) { return xbrain::config::detail::strip_comment(s); }
// The three vector leaves of the resolved perception config, with lengths.
const std::map<std::string, std::size_t>& resolved_vectors() {
    static const std::map<std::string, std::size_t> vectors = {
        {extrinsic+"translation", 3}, {extrinsic+"rotation_xyzw", 4}, {internal+"prior_normal", 3}};
    return vectors;
}

}  // namespace

// yaml_lite intentionally has no sequence node. Adapt ONLY the three numeric
// vector leaves we own; keep all map/scalar parsing in upstream common. Check
// duplicate paths and unsupported indentation before its last-write-wins map.
std::string adapt_resolved_yaml_vectors(const std::string& text) {
    return adapt_yaml_vectors(text, resolved_vectors(), "resolved_config", 1024U*1024U);
}

std::string adapt_yaml_vectors(const std::string& text, const std::map<std::string, std::size_t>& vectors,
                             const std::string& context, std::size_t max_bytes) {
    require(text.size() <= max_bytes, context, "exceeds document size bound");
    require(text.find("${") == std::string::npos, context, "unresolved reference");
    std::vector<std::string> lines;
    std::istringstream input(text);
    std::string line;
    while (std::getline(input, line)) lines.push_back(line);
    // Stack of open maps (indent, dotted path, indent of their children).
    struct Parent { int indent; std::string path; int child_indent = -1; };
    std::vector<Parent> parents{{-1, "", -1}};
    std::set<std::string> seen;
    std::ostringstream output;
    for (std::size_t i = 0; i < lines.size(); ++i) {
        line = content(lines[i]);
        const auto t = trim(line);
        if (t.empty()) continue;
        require(line.find('\t') == std::string::npos, context, "tabs unsupported");
        require(t != "---" && t != "...", context, "document separators unsupported");
        const int indent = xbrain::config::detail::count_indent(line);
        const auto colon = t.find(':');
        require(colon != std::string::npos && t.front() != '-', context,
                "expected scalar/map key; unsupported sequence");
        const auto key = xbrain::config::detail::unquote(trim(t.substr(0, colon)));
        require(!key.empty() && std::all_of(key.begin(), key.end(), [](unsigned char c) {
            return std::isalnum(c) || c == '_' || c == '-';
        }), context, "unsupported key spelling");
        // Close the maps this line is not inside of, then check the indentation.
        while (parents.size() > 1 && parents.back().indent >= indent) parents.pop_back();
        require(parents.back().indent < indent, context, "invalid indentation");
        const auto path = parents.back().path.empty() ? key : parents.back().path+"."+key;
        auto& parent = parents.back();
        if (parent.child_indent < 0) parent.child_indent = indent;
        require(parent.child_indent == indent, path,
                "inconsistent map indentation or child under a scalar");
        require(seen.insert(path).second, path, "duplicate key");
        auto value = trim(t.substr(colon+1));
        require(value.empty() || (value.front() != '&' && value.front() != '*' && value.front() != '!'),
                path, "YAML aliases/tags unsupported");
        // A declared vector in block style: collect its "- item" lines.
        const auto vector = vectors.find(path);
        if (value.empty() && vector != vectors.end()) {
            std::vector<std::string> elements;
            std::size_t next = i+1;
            int sequence_indent = -1;
            for (; next < lines.size(); ++next) {
                const auto raw = content(lines[next]);
                const auto item = trim(raw);
                if (item.empty()) continue;
                const int item_indent = xbrain::config::detail::count_indent(raw);
                if (item_indent < indent || item.size() < 2 || item[0] != '-' || item[1] != ' ') break;
                require(raw.find('\t') == std::string::npos, path, "tabs unsupported");
                if (sequence_indent < 0) sequence_indent = item_indent;
                require(item_indent == sequence_indent, path, "inconsistent sequence indentation");
                const auto token = trim(item.substr(2));
                std::size_t used = 0;
                double parsed = 0;
                try { parsed = std::stod(token, &used); }
                catch (const std::exception&) { invalid(path, "block vector item is not numeric"); }
                require(used == token.size() && std::isfinite(parsed), path,
                        "each block vector item must be one finite number");
                elements.push_back(token);
            }
            if (!elements.empty()) {
                require(elements.size() == vector->second, path, "wrong block vector length");
                value = "[";
                for (std::size_t n = 0; n < elements.size(); ++n) {
                    if (n) value += ",";
                    value += elements[n];
                }
                value += "]";
                i = next-1;
            }
        }
        // In YAML an empty scalar leaf is null, not a childless map. Preserve
        // that distinction before yaml_lite turns every empty value into map.
        if (value.empty()) {
            std::size_t next = i+1;
            while (next < lines.size() && trim(content(lines[next])).empty()) ++next;
            if (next == lines.size() ||
                xbrain::config::detail::count_indent(content(lines[next])) <= indent)
                value = "null";
        }
        // Re-emit the line in canonical form; an empty value opens a map.
        output << std::string(static_cast<std::size_t>(indent), ' ') << key << ":";
        if (!value.empty()) output << " " << value;
        output << "\n";
        if (value.empty()) parents.push_back({indent, path, -1});
    }
    return output.str();
}

}  // namespace perception::config
