/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_yaml_lite.cc
 * Brief: Offline unit test for the yaml_lite reader (no yaml-cpp, no ROS)
 *
 * Description:
 * Verifies the parser handles the exact shape the freeze materialiser emits
 * (nested maps, quoted scalars, inline `#` comments, null) AND that the require_*
 * accessors are fail-STOP: the load-bearing cases are the mutations of CLAUDE.md
 * 3.3 -- a null value and a missing key must THROW, not return 0.0. If either
 * silently returned a number, an uncalibrated safety threshold would sail
 * through as 0 and limit the machine to a stop with no error (the 3.1 fail-
 * silent). A sequence must also throw rather than be mis-parsed.
 *
 * 2026-09-15: block sequences became part of the modelled subset (quadruped.yaml
 * has six of them, 13 S8.2), so the "sequence throws" case narrowed to the
 * shapes still outside it. The sequence cases below are written against the
 * EXACT bytes yaml.safe_dump emits -- a dash at the parent key column with the
 * entry map indented two further -- because the realistic parser defect is an
 * off-by-one in that indent that merges consecutive entries into one. Merging
 * endpoint_candidates would silently drop probe targets and present as "the
 * chassis is unreachable", indistinguishable from a cable fault, so the entry
 * COUNT is asserted before any field value.
 */

#include "xbrain/config/yaml_lite.h"

#include <cstdio>
#include <string>

using xbrain::config::ParseYaml;
using xbrain::config::YamlNode;

static int g_failures = 0;

#define CHECK(cond)                                                \
  do {                                                             \
    if (!(cond)) {                                                 \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);  \
      ++g_failures;                                                \
    }                                                              \
  } while (0)

template <class F>
static bool Throws(F f) {
  try {
    f();
    return false;
  } catch (const std::exception&) {
    return true;
  }
}

static const char* kSample =
    "serial:\n"
    "  port: \"/dev/ttyACM0\"   # inline comment kept out of the value\n"
    "  baudrate: 115200\n"
    "# a full-line comment\n"
    "heading_stddev_deg: 1.0\n"
    "flag_on: true\n"
    "uncalibrated: null\n"
    "resolver:\n"
    "  cov_thresh_rad: 0.02\n"
    "  i_heading_l3: 0.0\n";

int main() {
  const YamlNode cfg = ParseYaml(kSample);

  // Nested map + typed scalars.
  CHECK(cfg.require_string("serial.port") == "/dev/ttyACM0");  // quotes + comment stripped
  CHECK(cfg.require_int("serial.baudrate") == 115200);
  CHECK(cfg.require_double("heading_stddev_deg") == 1.0);
  CHECK(cfg.require_bool("flag_on") == true);
  CHECK(cfg.require_double("resolver.cov_thresh_rad") == 0.02);
  CHECK(cfg.require_double("resolver.i_heading_l3") == 0.0);

  // 3.3 red mutants: null and missing must THROW (never a silent 0.0).
  CHECK(Throws([&] { cfg.require_double("uncalibrated"); }));       // null -> throw
  CHECK(Throws([&] { cfg.require_double("resolver.nope"); }));      // missing -> throw
  CHECK(Throws([&] { cfg.require_int("serial.port"); }));           // not an int -> throw
  CHECK(Throws([&] { cfg.require_double("serial"); }));             // map, not scalar -> throw

  // ---- block sequences (2026-09-15) ---------------------------------------
  // The materialiser shape, verbatim: dash at the key column, entry members two
  // columns further, sibling entries separated only by the next dash.
  static const char* kSeq =
      "chassis_link:\n"
      "  endpoint_candidates:\n"
      "  - enabled: true\n"
      "    host: 10.21.33.103\n"
      "    port: 30003\n"
      "    tls: false\n"
      "  - enabled: false\n"
      "    host: 10.21.33.103\n"
      "    port: 30004\n"
      "    tls: true\n"
      "  reconnect_backoff_s:\n"
      "  - 0.5\n"
      "  - 5.0\n"
      "  codebook_table:\n"
      "    legacy_decimal: {}\n"
      "motion:\n"
      "  prone_forbidden_gaits:\n"
      "  - stair_agile\n"
      "  - stair_standard\n"
      "  axes:\n"
      "    special_gaits: []\n";
  const YamlNode seq = ParseYaml(kSeq);
  const YamlNode& eps = seq.require_seq("chassis_link.endpoint_candidates");
  // Count first: an off-by-one in the entry indent yields 1 here, and every
  // field assertion below would still pass on that single merged entry.
  CHECK(eps.size() == 2);
  CHECK(eps.at_index(0).require_int("port") == 30003);
  CHECK(eps.at_index(0).require_bool("enabled") == true);
  CHECK(eps.at_index(0).require_bool("tls") == false);
  // The SECOND entry is what a merge destroys: same keys, different values.
  CHECK(eps.at_index(1).require_int("port") == 30004);
  CHECK(eps.at_index(1).require_bool("enabled") == false);
  CHECK(eps.at_index(1).require_bool("tls") == true);
  // Scalar sequences, and the two empty-collection spellings.
  CHECK(seq.require_seq("chassis_link.reconnect_backoff_s").size() == 2);
  CHECK(seq.require_seq("motion.prone_forbidden_gaits").size() == 2);
  CHECK(seq.require_seq("motion.prone_forbidden_gaits").at_index(1).is_scalar());
  CHECK(seq.require_seq("motion.axes.special_gaits").size() == 0);
  CHECK(seq.at("chassis_link.codebook_table.legacy_decimal").is_map());
  // The VALUE of a scalar sequence entry, which no dotted path can reach: an
  // entry has no key, so at("") walks into the scalar and throws. Until the
  // node-level accessors landed, a list of bare numbers was present in the
  // config, never read by the loader, and nothing failed -- the value simply
  // never reached the process. Asserting only size() (as this test used to)
  // cannot tell that apart from a list that is read correctly.
  {
    const YamlNode& backoff = seq.require_seq("chassis_link.reconnect_backoff_s");
    CHECK(backoff.at_index(0).as_double("backoff[0]") == 0.5);
    CHECK(backoff.at_index(1).as_double("backoff[1]") == 5.0);
    const YamlNode& gaits = seq.require_seq("motion.prone_forbidden_gaits");
    CHECK(gaits.at_index(0).as_scalar("gaits[0]") == "stair_agile");
    // A number read as an int, and the trailing-garbage rejection: stod and
    // stol both stop at the first unusable character, so "0.5" as an int must
    // throw rather than quietly become 0.
    CHECK(Throws([&] { backoff.at_index(0).as_int("backoff[0]"); }));
    CHECK(Throws([&] { gaits.at_index(0).as_double("gaits[0]"); }));
    CHECK(Throws([&] { gaits.at_index(0).as_bool("gaits[0]"); }));
    // *** The values above are refused by stod/stol themselves -- "stair_agile"
    // has no leading number at all. That is NOT the same as the length check,
    // and a mutant that deleted the length check survived the suite until this
    // case existed: a value that PARSES and then has garbage after it.
    // "0.5s" becomes 0.5, "10 Hz" becomes 10, and the unit the author thought
    // they were declaring is silently dropped.
    {
      const YamlNode g = ParseYaml("k:\n- 0.5s\n- 10 Hz\n");
      const YamlNode& l = g.require_seq("k");
      CHECK(Throws([&] { l.at_index(0).as_double("k[0]"); }));
      CHECK(Throws([&] { l.at_index(1).as_int("k[1]"); }));
    }
    // A node of the wrong SHAPE must throw rather than be coerced, and the
    // assertion has to be on as_scalar: every typed accessor would throw here
    // anyway because the empty string is not a number, which hides whether the
    // shape check exists at all.
    CHECK(Throws([&] { eps.at_index(0).as_scalar("eps[0]"); }));
    CHECK(Throws([&] { gaits.as_scalar("gaits"); }));
    CHECK(Throws([&] { seq.require_seq("motion.axes.special_gaits").as_scalar("x"); }));
  }
  // A null entry inside a list is an UNCALIBRATED value, not a zero. This is
  // CLAUDE.md 3.1 at the element level: a backoff ladder with a null rung must
  // stop the process, because the alternative is a 0.0 s rung that turns a
  // reconnect into a busy loop against the chassis.
  {
    const YamlNode n = ParseYaml("k:\n- 1.0\n- null\n- ~\n");
    const YamlNode& l = n.require_seq("k");
    CHECK(l.size() == 3);
    CHECK(l.at_index(0).as_double("k[0]") == 1.0);
    // as_scalar, not as_double: stod("null") throws on its own, so asserting
    // through a typed accessor cannot tell "the null check fired" from "the
    // number parse failed". Only the scalar form isolates the null branch --
    // the mutant that deleted it survived every typed assertion.
    CHECK(Throws([&] { l.at_index(1).as_scalar("k[1]"); }));  // literal null
    CHECK(Throws([&] { l.at_index(2).as_scalar("k[2]"); }));  // tilde
    CHECK(Throws([&] { l.at_index(1).as_double("k[1]"); }));
    CHECK(Throws([&] { l.at_index(2).as_double("k[2]"); }));
    // A bare dash is rejected by the PARSER (a nested collection is outside the
    // modelled subset), so there is no "empty entry" node to read -- that case
    // is asserted at parse time further down, not here.
  }
  // A sequence node must not answer to the scalar accessors: reading a list as
  // a string is how "the list is empty" and "the key holds one value" get
  // confused, and both of those are silent.
  CHECK(Throws([&] { seq.require_string("motion.prone_forbidden_gaits"); }));
  // ...and a scalar must not answer to require_seq, so an edit that replaces a
  // list with a single value fails loudly instead of iterating zero times.
  CHECK(Throws([&] { seq.require_seq("chassis_link.endpoint_candidates.0"); }));
  CHECK(Throws([&] { eps.at_index(0).require_seq("port"); }));
  CHECK(Throws([&] { eps.at_index(2); }));  // out of range, not a null node

  // Shapes still OUTSIDE the modelled subset must throw, not be guessed.
  CHECK(Throws([&] { ParseYaml("- a\n- b\n"); }));           // document-level list
  CHECK(Throws([&] { ParseYaml("k:\n  a: 1\n  - x\n"); }));  // dash inside a map
  CHECK(Throws([&] { ParseYaml("k:\n  -\n"); }));             // bare dash (nested coll.)
  // A key line whose nearest open frame is the SEQUENCE itself -- reachable
  // only through a malformed file (a mapping key indented under a scalar
  // entry). Without this case the guard that rejects it is dead code that no
  // mutant can kill, and the parser would attach the key to the list and lose
  // it. Found by mutation, not by reading (CLAUDE.md 3.3 / 7.2.1).
  CHECK(Throws([&] { ParseYaml("k:\n- a\n  x: 1\n"); }));

  if (g_failures == 0) {
    std::printf("ALL YAML_LITE TESTS PASSED\n");
    return 0;
  }
  std::printf("%d YAML_LITE TEST(S) FAILED\n", g_failures);
  return 1;
}
