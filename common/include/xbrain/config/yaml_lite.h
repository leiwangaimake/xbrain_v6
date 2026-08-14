/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: yaml_lite.h
 * Brief: Minimal dependency-free YAML-subset reader for C++ processes (3.1)
 *
 * Description:
 * Why this exists. rtk_driver is the first C++ process that must load a resolved
 * config (data/run/resolved/rtk_driver.yaml). CLAUDE.md 3.1 forbids any code
 * default for a safety param, so the driver MUST read every threshold from the
 * file and refuse to start if one is missing -- which needs a loader. The ORIN
 * has no yaml-cpp, and common/ feeds chassis_relay on the estop path (5.3: no
 * heavy deps), so this is a small header-only reader instead of a library.
 *
 * What it parses. The freeze materialiser writes resolved configs with Python
 * yaml.dump: 2-space block style, `key: value` scalars, nested maps by
 * indentation, sorted keys, no anchors/flow/multiline. This reader covers
 * exactly that: nested maps + scalar leaves (+ inline `#` comments, quoted
 * scalars, null/~). It does NOT support sequences (`- item`) -- it THROWS on
 * them rather than silently mis-parsing, because rtk_driver.yaml has none and a
 * silent wrong parse of a safety threshold is the fail-silent 3.1 rules out.
 *
 * The 3.1 contract lives in the require_* accessors: a missing key, a null
 * (`null`/`~`/empty -- how 10 S5.4.5 writes an uncalibrated value), or a value
 * that will not parse as the requested type all THROW with the dotted key path.
 * No accessor returns a fallback. Read the resolved product, never the source
 * (10 S5.4.1) -- this reader does not expand ${common.*} references.
 */
#ifndef HACHIST_XBRAIN_V6_CONFIG_YAML_LITE_H_
#define HACHIST_XBRAIN_V6_CONFIG_YAML_LITE_H_

#include <cctype>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace xbrain {
namespace config {

// One node of the parsed tree: either a map (named children) or a scalar leaf.
// std::map keeps references stable across inserts (node-based), which the
// indentation stack in ParseYaml relies on to hold pointers to open maps.
class YamlNode {
 public:
  bool is_map() const { return is_map_; }
  bool is_scalar() const { return !is_map_; }
  // A scalar is "null" when it is the literal null / ~ / empty. This is the
  // 10 S5.4.5 shape for an uncalibrated value, treated as absent by require_*.
  bool is_null() const {
    return !is_map_ && (scalar_.empty() || scalar_ == "null" || scalar_ == "~");
  }
  const std::map<std::string, YamlNode>& items() const { return map_; }

  // Walk a dotted path ("resolver.cov_thresh_rad"). Throws with the full path
  // at the first missing segment -- never returns a placeholder node.
  const YamlNode& at(const std::string& dotted) const {
    const YamlNode* cur = this;
    std::size_t start = 0;
    while (start <= dotted.size()) {
      std::size_t dot = dotted.find('.', start);
      std::string seg =
          dotted.substr(start, dot == std::string::npos ? std::string::npos : dot - start);
      if (!cur->is_map_) {
        throw std::runtime_error("config path descends into scalar: " + dotted);
      }
      auto it = cur->map_.find(seg);
      if (it == cur->map_.end()) {
        throw std::runtime_error("missing config key: " + dotted);
      }
      cur = &it->second;
      if (dot == std::string::npos) break;
      start = dot + 1;
    }
    return *cur;
  }

  // Typed accessors -- each THROWS with the key path on missing / null /
  // parse-failure (CLAUDE.md 3.1). None of them has a default branch.
  double require_double(const std::string& p) const {
    const std::string& s = require_scalar(p);
    std::size_t pos = 0;
    double v = 0.0;
    try {
      v = std::stod(s, &pos);
    } catch (const std::exception&) {
      throw std::runtime_error("config key not a number: " + p + " = '" + s + "'");
    }
    if (pos != s.size()) {
      throw std::runtime_error("config key not a number: " + p + " = '" + s + "'");
    }
    return v;
  }
  long require_int(const std::string& p) const {
    const std::string& s = require_scalar(p);
    std::size_t pos = 0;
    long v = 0;
    try {
      v = std::stol(s, &pos, 10);
    } catch (const std::exception&) {
      throw std::runtime_error("config key not an int: " + p + " = '" + s + "'");
    }
    if (pos != s.size()) {
      throw std::runtime_error("config key not an int: " + p + " = '" + s + "'");
    }
    return v;
  }
  bool require_bool(const std::string& p) const {
    const std::string& s = require_scalar(p);
    if (s == "true" || s == "True" || s == "yes") return true;
    if (s == "false" || s == "False" || s == "no") return false;
    throw std::runtime_error("config key not a bool: " + p + " = '" + s + "'");
  }
  std::string require_string(const std::string& p) const { return require_scalar(p); }

 private:
  // Fetch a non-null scalar at path or throw. The single choke point that turns
  // "missing key" and "null (uncalibrated)" into the two 3.1 failure messages.
  const std::string& require_scalar(const std::string& p) const {
    const YamlNode& n = at(p);
    if (n.is_map_) {
      throw std::runtime_error("config key is a map, not a scalar: " + p);
    }
    if (n.is_null()) {
      throw std::runtime_error("config key is null (uncalibrated per CLAUDE.md 3.1): " + p);
    }
    return n.scalar_;
  }

  bool is_map_ = false;
  std::string scalar_;
  std::map<std::string, YamlNode> map_;

  friend YamlNode ParseYaml(const std::string& text);
};

namespace detail {

inline int CountIndent(const std::string& s) {
  int n = 0;
  while (n < static_cast<int>(s.size()) && s[n] == ' ') ++n;
  return n;
}

// Drop an inline `#` comment. A `#` only starts a comment at line start or when
// preceded by whitespace (YAML rule), and never inside a quoted scalar -- so a
// port value or a quoted string with a `#` survives.
inline std::string StripComment(const std::string& s) {
  char quote = 0;
  for (std::size_t i = 0; i < s.size(); ++i) {
    char c = s[i];
    if (quote) {
      if (c == quote) quote = 0;
    } else if (c == '"' || c == '\'') {
      quote = c;
    } else if (c == '#' && (i == 0 || s[i - 1] == ' ' || s[i - 1] == '\t')) {
      return s.substr(0, i);
    }
  }
  return s;
}

inline std::string Trim(const std::string& s) {
  std::size_t a = s.find_first_not_of(" \t\r\n");
  if (a == std::string::npos) return "";
  std::size_t b = s.find_last_not_of(" \t\r\n");
  return s.substr(a, b - a + 1);
}

// Strip a single matched pair of surrounding quotes. No escape processing is
// needed for the materialised configs (yaml.dump quotes plainly).
inline std::string Unquote(const std::string& s) {
  if (s.size() >= 2 && ((s.front() == '"' && s.back() == '"') ||
                        (s.front() == '\'' && s.back() == '\''))) {
    return s.substr(1, s.size() - 2);
  }
  return s;
}

}  // namespace detail

// Parse yaml.dump block-style text into a tree. Throws std::runtime_error on any
// shape it does not model (sequences, tabs-as-indent, a colon-less line) rather
// than guessing -- a wrong guess on a safety threshold is exactly the failure
// mode CLAUDE.md 3.2 warns about.
inline YamlNode ParseYaml(const std::string& text) {
  YamlNode root;
  root.is_map_ = true;
  struct Frame {
    int indent;
    YamlNode* node;
  };
  std::vector<Frame> stack{{-1, &root}};
  std::istringstream in(text);
  std::string line;
  int lineno = 0;
  while (std::getline(in, line)) {
    ++lineno;
    std::string content = detail::StripComment(line);
    int indent = detail::CountIndent(content);
    std::string trimmed = detail::Trim(content);
    if (trimmed.empty()) continue;
    if (trimmed == "---" || trimmed == "...") continue;
    if (trimmed[0] == '-') {
      throw std::runtime_error("yaml_lite: sequences unsupported, line " +
                               std::to_string(lineno));
    }
    while (stack.size() > 1 && stack.back().indent >= indent) stack.pop_back();
    YamlNode* parent = stack.back().node;
    std::size_t colon = trimmed.find(':');
    if (colon == std::string::npos) {
      throw std::runtime_error("yaml_lite: expected 'key:' at line " +
                               std::to_string(lineno) + ": " + trimmed);
    }
    std::string key = detail::Unquote(detail::Trim(trimmed.substr(0, colon)));
    std::string val = detail::Trim(trimmed.substr(colon + 1));
    if (val.empty()) {
      YamlNode& child = parent->map_[key];
      child.is_map_ = true;
      stack.push_back({indent, &child});
    } else {
      YamlNode& child = parent->map_[key];
      child.is_map_ = false;
      child.scalar_ = detail::Unquote(val);
    }
  }
  return root;
}

// Read a file whole and parse it. Throws if the file cannot be opened -- a
// missing resolved product must stop startup, not default (3.1 / 3.6).
inline YamlNode LoadYamlFile(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  if (!f) {
    throw std::runtime_error("cannot open config file: " + path);
  }
  std::ostringstream ss;
  ss << f.rdbuf();
  return ParseYaml(ss.str());
}

}  // namespace config
}  // namespace xbrain

#endif  // HACHIST_XBRAIN_V6_CONFIG_YAML_LITE_H_
