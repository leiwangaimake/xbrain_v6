/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_chs_a_framer.cc
 * Brief: CHS-A reassembly against the real capture, split and glued every way
 *
 * Description:
 * The frames here are the ones the chassis actually sent (test/golden/), fed
 * through the framer in the arrangements a TCP stream really produces:
 *   * one frame split across three reads, including a split INSIDE the header
 *     and one that cuts the sync word in half -- the case a naive reader gets
 *     wrong, because it must keep a prefix that is not yet a frame;
 *   * two frames glued into one read, which a reader that parses "what it got"
 *     silently truncates to one;
 *   * a byte-at-a-time feed of the whole capture, which is the strongest shape:
 *     if the frame boundary logic depends on read sizes at all, this fails.
 *
 * Then the recovery rules, each with the failure it prevents:
 *   FR-2  leading garbage is slid past and the next frame is still found; a
 *         stream that never resyncs is reported as POISONED rather than slid
 *         forever, because the second case is a link carrying something else
 *         and sliding forever leaves the robot with no state and no error;
 *   FR-3  a header whose body never arrives is dropped after the timeout, and
 *         the timer is NOT refreshed by later calls -- refreshing makes the
 *         timeout unreachable on a link delivering one byte per poll, which is
 *         precisely the stall it guards;
 *   FR-5  a datagram must hold exactly one frame; a truncated or doubled one is
 *         dropped rather than partially consumed, since UDP gives no ordering
 *         that would make a continuation meaningful.
 *
 * Time is passed in, so the FR-3 case runs in microseconds and is not flaky.
 */

#include "quadruped/chs_a_framer.h"

#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

using quadruped::chs_a::FrameStatus;
using quadruped::chs_a::Framer;
using quadruped::chs_a::kHeaderBytes;

static int g_failures = 0;

#define CHECK(cond)                                                \
  do {                                                             \
    if (!(cond)) {                                                 \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);  \
      ++g_failures;                                                \
    }                                                              \
  } while (0)

namespace {

using Bytes = std::vector<std::uint8_t>;

Bytes FromHex(const std::string& hex) {
  Bytes out;
  out.reserve(hex.size() / 2);
  for (std::size_t i = 0; i + 1 < hex.size(); i += 2) {
    out.push_back(static_cast<std::uint8_t>(std::stoul(hex.substr(i, 2), nullptr, 16)));
  }
  return out;
}

// Loads the same golden file the codec test uses. An empty result FAILS rather
// than letting every case below pass on nothing.
std::map<std::string, Bytes> LoadGolden(const std::string& path) {
  std::map<std::string, Bytes> out;
  std::ifstream f(path);
  if (!f) {
    std::printf("FAIL cannot open golden file: %s\n", path.c_str());
    ++g_failures;
    return out;
  }
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream is(line);
    std::string tag, hex;
    std::size_t n = 0;
    if (!(is >> tag >> n >> hex)) continue;
    out[tag] = FromHex(hex);
  }
  if (out.empty()) {
    std::printf("FAIL golden file parsed to zero vectors\n");
    ++g_failures;
  }
  return out;
}

// Compare the framer's current frame against the expected bytes. Goes through
// a helper rather than a bare memcmp because frame() is nullptr whenever no
// frame is available: when an earlier CHECK in the same case fails, the bare
// form segfaults, and a crash in CI carries none of the information the FAIL
// line would have carried. Found by mutation -- the mutant that made resync
// discard everything took the suite down with SIGSEGV instead of a message.
bool FrameEquals(const Framer& fr, const Bytes& want) {
  if (fr.frame() == nullptr || fr.frame_len() != want.size()) return false;
  return std::memcmp(fr.frame(), want.data(), want.size()) == 0;
}

// Values for the cases below. They happen to match what configs/quadruped.yaml
// ships today, but this test does NOT read that file and must not be read as
// checking it: claiming agreement with a file one never opens is how a stale
// constant survives a config change (CLAUDE.md 3.2, "quoting text that was
// never honoured"). Several cases deliberately use much smaller budgets so the
// recovery paths are reachable in a few hundred bytes. That the PRODUCTION
// numbers reach the Framer is a property of the wiring, and belongs to the
// batch that does the wiring (B2), not here.
constexpr std::size_t kResyncMax = 4096;
constexpr int kAssemblyTimeoutMs = 500;

}  // namespace

int main(int argc, char** argv) {
  const std::string golden_path =
      (argc >= 2) ? argv[1] : "test/golden/chs_a_frames.txt";
  const auto golden = LoadGolden(golden_path);
  if (golden.empty()) {
    std::printf("%d CHS_A_FRAMER TEST(S) FAILED\n", g_failures);
    return 1;
  }
  const Bytes& basic = golden.at("RX_00100064_00f00000");   // 387 B
  const Bytes& fault = golden.at("RX_0010007f_00f00000");   // 126 B
  const Bytes& device = golden.at("RX_00100002_00f00000");  // 2405 B

  // ---- one frame, one read ------------------------------------------------
  {
    Framer fr(kResyncMax, kAssemblyTimeoutMs);
    CHECK(fr.Push(basic.data(), basic.size()));
    CHECK(fr.Next(0.0) == FrameStatus::kFrame);
    CHECK(fr.frame_len() == basic.size());
    CHECK(fr.asdu_len() == basic.size() - kHeaderBytes);
    CHECK(FrameEquals(fr, basic));
    CHECK(fr.Next(0.0) == FrameStatus::kNeedMore);
    CHECK(fr.frames_out() == 1);
  }

  // ---- split across three reads, including inside the header -------------
  {
    Framer fr(kResyncMax, kAssemblyTimeoutMs);
    // 2 bytes: cuts the sync word itself, so the framer holds a prefix that is
    // not yet identifiable as anything.
    CHECK(fr.Push(basic.data(), 2));
    CHECK(fr.Next(0.0) == FrameStatus::kNeedMore);
    // up to mid-header: the length field is still incomplete
    CHECK(fr.Push(basic.data() + 2, 7));
    CHECK(fr.Next(0.0) == FrameStatus::kNeedMore);
    CHECK(fr.Push(basic.data() + 9, basic.size() - 9));
    CHECK(fr.Next(0.0) == FrameStatus::kFrame);
    CHECK(fr.frame_len() == basic.size());
    CHECK(FrameEquals(fr, basic));
  }

  // ---- two frames glued into one read ------------------------------------
  {
    Framer fr(kResyncMax, kAssemblyTimeoutMs);
    Bytes glued;
    glued.insert(glued.end(), basic.begin(), basic.end());
    glued.insert(glued.end(), fault.begin(), fault.end());
    CHECK(fr.Push(glued.data(), glued.size()));
    CHECK(fr.Next(0.0) == FrameStatus::kFrame);
    CHECK(fr.frame_len() == basic.size());
    // The second frame must survive the first being handed out: this is where
    // the deferred consume either works or loses a frame per read.
    CHECK(fr.Next(0.0) == FrameStatus::kFrame);
    CHECK(fr.frame_len() == fault.size());
    CHECK(FrameEquals(fr, fault));
    CHECK(fr.Next(0.0) == FrameStatus::kNeedMore);
    CHECK(fr.frames_out() == 2);
  }

  // ---- byte at a time, three frames: the strongest arrangement -----------
  {
    Framer fr(kResyncMax, kAssemblyTimeoutMs);
    Bytes all;
    all.insert(all.end(), basic.begin(), basic.end());
    all.insert(all.end(), device.begin(), device.end());
    all.insert(all.end(), fault.begin(), fault.end());
    std::size_t got = 0;
    std::vector<std::size_t> sizes;
    for (std::size_t i = 0; i < all.size(); ++i) {
      CHECK(fr.Push(all.data() + i, 1));
      FrameStatus st = fr.Next(0.0);
      while (st == FrameStatus::kFrame) {
        ++got;
        sizes.push_back(fr.frame_len());
        st = fr.Next(0.0);
      }
    }
    CHECK(got == 3);
    if (sizes.size() == 3) {
      CHECK(sizes[0] == basic.size());
      CHECK(sizes[1] == device.size());
      CHECK(sizes[2] == fault.size());
    }
  }

  // ---- FR-2: garbage in front is slid past, the next frame still parses ---
  {
    Framer fr(kResyncMax, kAssemblyTimeoutMs);
    Bytes noisy(37, 0x5A);
    // A partial sync word in the noise: the resync must not mistake it for a
    // header and must not get stuck on it either.
    noisy.push_back(0xEB);
    noisy.push_back(0x91);
    noisy.push_back(0x00);
    noisy.insert(noisy.end(), basic.begin(), basic.end());
    CHECK(fr.Push(noisy.data(), noisy.size()));
    FrameStatus st = fr.Next(0.0);
    CHECK(st == FrameStatus::kFrame);
    CHECK(fr.frame_len() == basic.size());
    CHECK(fr.resync_bytes_total() == 40);  // 37 noise + the 3 decoy bytes
  }

  // ---- FR-2: a stream that never resyncs is POISONED, not slid forever ----
  {
    Framer fr(64, kAssemblyTimeoutMs);  // small budget so the case is quick
    const Bytes junk(200, 0x11);
    CHECK(fr.Push(junk.data(), junk.size()));
    CHECK(fr.Next(0.0) == FrameStatus::kPoisoned);
    CHECK(fr.Next(0.0) == FrameStatus::kNeedMore);
  }

  // ---- FR-2: poisoning DISCARDS what is buffered, frame or not -------------
  {
    // The arrangement matters. In the case above the resync had already shrunk
    // the buffer to three bytes, so "the buffer was cleared" and "there was
    // nothing left anyway" look identical -- a framer that skips the clear
    // passes it. Here the noise is followed by a perfectly good frame, and the
    // budget is exceeded by the noise alone: the frame IS present in the buffer
    // at the moment the link is declared poisoned.
    //
    // It must not be delivered. The contract on kPoisoned is that the caller
    // closes the connection; handing out a frame parsed from a stream we have
    // just declared untrustworthy would have the robot act on it AND reconnect.
    Framer fr(8, kAssemblyTimeoutMs);
    Bytes noisy(20, 0x5A);
    noisy.insert(noisy.end(), basic.begin(), basic.end());
    CHECK(fr.Push(noisy.data(), noisy.size()));
    CHECK(fr.Next(0.0) == FrameStatus::kPoisoned);
    CHECK(fr.Next(0.0) == FrameStatus::kNeedMore);
    CHECK(fr.frames_out() == 0);
  }

  // ---- FR-2: the budget boundary, pinned from both sides -------------------
  {
    // 13 S2.2 FR-2 reads "after resync_max_bytes slid without a match". That
    // makes exactly the budget still acceptable and one more fatal. Without a
    // case on each side the comparison is whichever of < and <= got typed, and
    // a later reader has no way to tell the intent from the code.
    constexpr std::size_t kBudget = 32;
    {
      Framer fr(kBudget, kAssemblyTimeoutMs);
      Bytes noisy(kBudget, 0x5A);  // exactly the budget: still acceptable
      noisy.insert(noisy.end(), fault.begin(), fault.end());
      CHECK(fr.Push(noisy.data(), noisy.size()));
      CHECK(fr.Next(0.0) == FrameStatus::kFrame);
      CHECK(fr.resync_bytes_total() == kBudget);
    }
    {
      Framer fr(kBudget, kAssemblyTimeoutMs);
      Bytes noisy(kBudget + 1, 0x5A);  // one past: poisoned
      noisy.insert(noisy.end(), fault.begin(), fault.end());
      CHECK(fr.Push(noisy.data(), noisy.size()));
      CHECK(fr.Next(0.0) == FrameStatus::kPoisoned);
    }
  }

  // ---- FR-2: a sync word split across two reads must survive the resync ---
  {
    // This is the case the "keep the last three bytes" rule exists for, and it
    // had NO coverage until a mutant that discarded the whole buffer on a failed
    // resync passed the entire suite. The arrangement: noise, then the first two
    // bytes of a frame, then the rest in a second read. A resync that keeps
    // nothing throws away EB 91 and the frame is gone for good -- on a live link
    // that is one lost report per reconnect, silently.
    Framer fr(kResyncMax, kAssemblyTimeoutMs);
    Bytes head(20, 0x5A);
    head.push_back(basic[0]);
    head.push_back(basic[1]);
    CHECK(fr.Push(head.data(), head.size()));
    CHECK(fr.Next(0.0) == FrameStatus::kNeedMore);
    CHECK(fr.Push(basic.data() + 2, basic.size() - 2));
    CHECK(fr.Next(0.0) == FrameStatus::kFrame);
    CHECK(fr.frame_len() == basic.size());
    CHECK(FrameEquals(fr, basic));
  }

  // ---- FR-2: the budget is per noise RUN, not a lifetime total --------------
  {
    // Three rounds of 40 noise bytes with a good frame after each. The total
    // slid (120) exceeds the budget (100); no single run does. A counter that
    // is never reset on a good frame declares a healthy link poisoned after
    // enough hours of small noise -- the failure looks like a chassis that
    // stops answering, and it gets worse the longer the robot runs, which is
    // the worst shape a bug can have.
    Framer fr(100, kAssemblyTimeoutMs);
    for (int round = 0; round < 3; ++round) {
      Bytes noisy(40, 0x5A);
      noisy.insert(noisy.end(), fault.begin(), fault.end());
      CHECK(fr.Push(noisy.data(), noisy.size()));
      CHECK(fr.Next(0.0) == FrameStatus::kFrame);
      CHECK(fr.frame_len() == fault.size());
    }
    CHECK(fr.resync_bytes_total() == 120);  // the TOTAL still counts them all
    CHECK(fr.frames_out() == 3);
  }

  // ---- FR-3: a header with no body is dropped, and the timer is not reset -
  {
    Framer fr(kResyncMax, kAssemblyTimeoutMs);
    // Header plus one payload byte: the framer knows how many bytes it wants
    // and will never get them.
    CHECK(fr.Push(basic.data(), kHeaderBytes + 1));
    CHECK(fr.Next(10.0) == FrameStatus::kNeedMore);   // timer starts at t=10
    CHECK(fr.Next(10.2) == FrameStatus::kNeedMore);   // still inside 500 ms
    // *** The mutation this kills: refreshing partial_since on every call. With
    // a refresh, the elapsed time below is 0.3 s, not 0.6 s, and the drop
    // never happens -- on a stalled link that means waiting forever.
    CHECK(fr.Next(10.6) == FrameStatus::kDropped);
    // The boundary itself, from both sides. 13 S2.2 FR-3 says a partial frame
    // "older than" the timeout is dropped, so elapsed == timeout is NOT yet
    // older and must keep waiting. Without this pair the comparison is whichever
    // of < and <= got typed.
    //
    // These three numbers are chosen so the arithmetic is exact: 500 ms renders
    // as 0.5, and 10.0, 10.5 and their difference are all exactly representable
    // in a double. A boundary case built on, say, a 100 ms timeout would be
    // comparing 0.09999999999999998 against 0.1 and would be a coin flip.
    CHECK(fr.dropped_frames() == 1);
    // ...and the framer is usable afterwards: a fresh frame parses.
    CHECK(fr.Push(fault.data(), fault.size()));
    CHECK(fr.Next(11.0) == FrameStatus::kFrame);
    CHECK(fr.frame_len() == fault.size());
  }

  // ---- FR-3: elapsed == timeout still waits; one ulp past it drops --------
  {
    Framer fr(kResyncMax, kAssemblyTimeoutMs);  // 500 ms -> exactly 0.5
    CHECK(fr.Push(basic.data(), kHeaderBytes + 1));
    CHECK(fr.Next(10.0) == FrameStatus::kNeedMore);  // timer starts at 10.0
    CHECK(fr.Next(10.5) == FrameStatus::kNeedMore);  // exactly at the timeout
    CHECK(fr.dropped_frames() == 0);
    CHECK(fr.Next(10.5000001) == FrameStatus::kDropped);  // past it
  }

  // ---- Reset drops buffered bytes but keeps the totals --------------------
  {
    Framer fr(kResyncMax, kAssemblyTimeoutMs);
    CHECK(fr.Push(basic.data(), 40));  // partial
    CHECK(fr.Next(0.0) == FrameStatus::kNeedMore);
    fr.Reset();
    // Bytes from the old connection must never be read as the start of the new
    // one, so the remainder of that frame must NOT complete anything.
    CHECK(fr.Push(basic.data() + 40, basic.size() - 40));
    const FrameStatus st = fr.Next(0.0);
    CHECK(st != FrameStatus::kFrame);
  }

  // ---- FR-5: one datagram, one frame -------------------------------------
  {
    Framer fr(kResyncMax, kAssemblyTimeoutMs);
    CHECK(fr.PushDatagram(basic.data(), basic.size()) == FrameStatus::kFrame);
    CHECK(fr.frame_len() == basic.size());
    // Truncated: dropped, not buffered for a continuation that UDP cannot
    // promise to deliver next.
    CHECK(fr.PushDatagram(basic.data(), basic.size() - 5) == FrameStatus::kDropped);
    // Two frames in one datagram: also dropped, because consuming the first
    // and discarding the rest would silently lose a report.
    Bytes doubled;
    doubled.insert(doubled.end(), basic.begin(), basic.end());
    doubled.insert(doubled.end(), fault.begin(), fault.end());
    CHECK(fr.PushDatagram(doubled.data(), doubled.size()) == FrameStatus::kDropped);
    // Garbage: dropped.
    const Bytes junk(40, 0x00);
    CHECK(fr.PushDatagram(junk.data(), junk.size()) == FrameStatus::kDropped);
    CHECK(fr.dropped_frames() == 3);
  }

  if (g_failures == 0) {
    std::printf("ALL CHS_A_FRAMER TESTS PASSED\n");
    return 0;
  }
  std::printf("%d CHS_A_FRAMER TEST(S) FAILED\n", g_failures);
  return 1;
}
