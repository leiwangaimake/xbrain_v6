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

  // A sequence is not modelled: it must throw, not mis-parse.
  CHECK(Throws([&] { ParseYaml("items:\n  - a\n  - b\n"); }));

  if (g_failures == 0) {
    std::printf("ALL YAML_LITE TESTS PASSED\n");
    return 0;
  }
  std::printf("%d YAML_LITE TEST(S) FAILED\n", g_failures);
  return 1;
}
