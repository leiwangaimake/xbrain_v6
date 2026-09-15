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
 * scalars, null/~) AND block sequences of scalars or of maps.
 *
 * Sequences (added 2026-09-15 for quadruped, 13 S8.2). The first version threw
 * on any `- item`: rtk_driver.yaml had none, and refusing was better than
 * guessing. quadruped.yaml has six (endpoint_candidates, the gait lists, the
 * backoff table), so "throw on sequences" stopped being caution and became
 * "the C++ side cannot read its own config at all". What is modelled is
 * exactly what yaml.safe_dump(default_flow_style=False) emits:
 *
 *     key:            |  key:               |  key: []
 *     - a             |  - port: 30003      |  key: {}
 *     - b             |    proto: tcp       |
 *
 * i.e. a dash at the key's own indent or deeper, holding either a scalar or a
 * map whose first pair sits on the dash line. Anything else -- nested
 * sequences, a dash inside flow context, a sequence entry that is itself a
 * sequence -- still THROWS. The rule did not change, only its reach: model
 * what the materialiser actually writes, refuse the rest, never guess.
 *
 * Why entries are addressed by index rather than iterated. A safety list read
 * with a for-each and an early `break` loses the "how many did I actually
 * read" answer; size() + at(i) keeps the count explicit at the call site, so a
 * loader that silently processed 1 of 4 endpoints cannot look like success.
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
  // Three shapes, checked in this order everywhere: map, sequence, scalar.
  // * is_scalar() stays "neither of the other two" so existing callers keep
  //   their meaning: a node that became a sequence must NOT answer true to
  //   is_scalar(), or require_scalar would try to read scalar_ off it.
  bool is_map() const { return is_map_; }
  bool is_seq() const { return is_seq_; }
  bool is_scalar() const { return !is_map_ && !is_seq_; }
  // A scalar is "null" when it is the literal null / ~ / empty. This is the
  // 10 S5.4.5 shape for an uncalibrated value, treated as absent by require_*.
  bool is_null() const {
    return !is_map_ && !is_seq_ &&
           (scalar_.empty() || scalar_ == "null" || scalar_ == "~");
  }
  const std::map<std::string, YamlNode>& items() const { return map_; }

  // Sequence length. Zero for a non-sequence node, which is deliberate: the
  // empty-list shape `key: []` and "key is not a list" must both mean "no
  // entries to process", and every call site that cares about the difference
  // asks is_seq() first.
  std::size_t size() const { return seq_.size(); }

  // Entry i of a sequence. Throws on a non-sequence node or an out-of-range
  // index rather than returning a default-constructed node -- an empty node
  // would read back as null and be reported as "uncalibrated", which would
  // send the operator hunting through the config for a value that is not
  // missing at all.
  const YamlNode& at_index(std::size_t i) const {
    if (!is_seq_) {
      throw std::runtime_error("config node is not a sequence");
    }
    if (i >= seq_.size()) {
      throw std::runtime_error("config sequence index out of range: " +
                               std::to_string(i) + " of " +
                               std::to_string(seq_.size()));
    }
    return seq_[i];
  }

  // Sequence at a dotted path, validated. Throws with the path when the key is
  // missing (at() does it) or when it is present but not a sequence -- the
  // second case is the realistic defect: an edit that turns a list into a
  // scalar would otherwise be read as an empty list and quietly do nothing.
  const YamlNode& require_seq(const std::string& p) const {
    const YamlNode& n = at(p);
    if (!n.is_seq_) {
      throw std::runtime_error("config key is not a sequence: " + p);
    }
    return n;
  }

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
    if (n.is_seq_) {
      throw std::runtime_error("config key is a sequence, not a scalar: " + p);
    }
    if (n.is_null()) {
      throw std::runtime_error("config key is null (uncalibrated per CLAUDE.md 3.1): " + p);
    }
    return n.scalar_;
  }

  bool is_map_ = false;
  bool is_seq_ = false;
  std::string scalar_;
  std::map<std::string, YamlNode> map_;
  std::vector<YamlNode> seq_;

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
// shape it does not model (nested sequences, tabs-as-indent, a colon-less line)
// rather than guessing -- a wrong guess on a safety threshold is exactly the
// failure mode CLAUDE.md 3.2 warns about.
//
// *** The parse state, written down because the indent arithmetic is the part
// that goes wrong silently. Two stacks would drift, so there is ONE: each frame
// remembers the indent that OPENED it plus what it opens onto. A sequence entry
// pushes TWO frames (the entry map, then nothing else) only when the dash line
// carries a `key: value`, because in yaml.dump output the rest of that map is
// indented to the column after the dash, not to the dash itself:
//
//     endpoint_candidates:      <- map frame at indent 4
//     - enabled: true           <- dash at indent 4, entry map opens at 6
//       host: 10.21.33.103      <- indent 6, belongs to the entry
//     - enabled: false          <- dash at indent 4 again: new entry
//
// So an entry map's indent is "dash column + 2", and a following dash at the
// dash column pops it. Getting this wrong merges two entries into one, which
// for endpoint_candidates would silently drop a probe target -- hence the
// explicit unit test over exactly this shape.
inline YamlNode ParseYaml(const std::string& text) {
  YamlNode root;
  root.is_map_ = true;
  struct Frame {
    int indent;      // Indent of the line that opened this frame.
    YamlNode* node;  // Map or sequence being filled.
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

    // ---- sequence entry -------------------------------------------------
    // A dash starts an entry of the nearest enclosing sequence. The sequence
    // node itself was created by the `key:` line above it (see below), so an
    // unattached dash means the document starts with a list -- a shape the
    // materialiser never writes, and one this reader refuses rather than
    // inventing a root list.
    if (trimmed[0] == '-' && (trimmed.size() == 1 || trimmed[1] == ' ')) {
      while (stack.size() > 1 && stack.back().indent > indent) stack.pop_back();
      // `key:` opened the frame as an empty map because a map and a sequence
      // look identical until this line. An EMPTY map at exactly this indent is
      // that pending node: convert it. A non-empty one is a real map and the
      // dash is then a shape error, caught by the check below.
      if (stack.back().node->is_map_ && stack.back().node->map_.empty() &&
          stack.back().node->seq_.empty() && stack.back().indent == indent) {
        stack.back().node->is_map_ = false;
        stack.back().node->is_seq_ = true;
      }
      // The frame at this indent must be the pending sequence. Anything else
      // (a map, or the root) is a dash in a place yaml.dump would not put one.
      if (!stack.back().node->is_seq_ || stack.back().indent != indent) {
        throw std::runtime_error("yaml_lite: unexpected sequence entry at line " +
                                 std::to_string(lineno));
      }
      YamlNode* seq = stack.back().node;
      std::string rest = detail::Trim(trimmed.substr(1));
      seq->seq_.push_back(YamlNode());
      YamlNode& entry = seq->seq_.back();
      if (rest.empty()) {
        // `-` alone: yaml.dump writes this only for a nested collection, which
        // is outside the modelled subset.
        throw std::runtime_error("yaml_lite: empty sequence entry at line " +
                                 std::to_string(lineno));
      }
      std::size_t colon = rest.find(':');
      // A scalar entry ("- stair_agile") has no colon; a map entry
      // ("- port: 30003") opens a map that continues on the following lines.
      if (colon == std::string::npos) {
        entry.is_map_ = false;
        entry.scalar_ = detail::Unquote(rest);
        continue;
      }
      entry.is_map_ = true;
      std::string key = detail::Unquote(detail::Trim(rest.substr(0, colon)));
      std::string val = detail::Trim(rest.substr(colon + 1));
      // *** The one number in this parser worth explaining: the entry frame is
      // recorded at dash column + 1, which is a column no line can occupy.
      // It has to sit STRICTLY between the dash column and the member column
      // so that both pops come out right with the one comparison each branch
      // already makes:
      //   - a member line ("      host: ...", column dash+2) must NOT pop it:
      //     the key branch pops while frame.indent >= line indent, and
      //     dash+1 < dash+2, so the frame survives;
      //   - the NEXT dash (column dash) MUST pop it: the dash branch pops
      //     while frame.indent > line indent, and dash+1 > dash.
      // Recording it at dash+2 pops the entry on its own second member and
      // merges every entry into one -- for endpoint_candidates that silently
      // drops probe targets, which is why this is a named test case.
      stack.push_back({indent + 1, &entry});
      YamlNode& child = entry.map_[key];
      if (val.empty()) {
        child.is_map_ = true;
        stack.push_back({indent + 2, &child});
      } else {
        child.is_map_ = false;
        child.scalar_ = detail::Unquote(val);
      }
      continue;
    }

    // ---- key line -------------------------------------------------------
    while (stack.size() > 1 && stack.back().indent >= indent) stack.pop_back();
    YamlNode* parent = stack.back().node;
    if (parent->is_seq_) {
      // A bare key at the sequence's own indent means the sequence ended
      // without the enclosing map being popped -- structurally impossible in
      // yaml.dump output, so refuse instead of attaching a key to a list.
      throw std::runtime_error("yaml_lite: key inside a sequence at line " +
                               std::to_string(lineno));
    }
    std::size_t colon = trimmed.find(':');
    if (colon == std::string::npos) {
      throw std::runtime_error("yaml_lite: expected 'key:' at line " +
                               std::to_string(lineno) + ": " + trimmed);
    }
    std::string key = detail::Unquote(detail::Trim(trimmed.substr(0, colon)));
    std::string val = detail::Trim(trimmed.substr(colon + 1));
    if (val.empty()) {
      // Ambiguous until the NEXT line: `key:` opens either a map or a
      // sequence. Open it as a map and let a following dash convert it --
      // conversion is safe because nothing can have been stored yet.
      YamlNode& child = parent->map_[key];
      child.is_map_ = true;
      stack.push_back({indent, &child});
    } else if (val == "[]") {
      // Explicit empty list. yaml.dump writes `key: []`, and reading it as the
      // scalar "[]" would make a later size() call answer 0 for the right
      // reason by accident -- but require_seq would then throw on a perfectly
      // valid empty list, so model it properly.
      YamlNode& child = parent->map_[key];
      child.is_map_ = false;
      child.is_seq_ = true;
    } else if (val == "{}") {
      // Explicit empty map (codebook_table.legacy_decimal today).
      YamlNode& child = parent->map_[key];
      child.is_map_ = true;
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
