/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_tx_owner.cc
 * Brief: Offline test for the single outbound owner -- CA-4 / TX-4..TX-7
 *
 * Description:
 * The property under test is not "bytes come out" but "a frame is never
 * interleaved with another frame", because that is the failure that presents as
 * random 0xE001/0xE002 which 13 S7.5 tells the reader to blame on our own
 * encoder. The fake writer therefore records the ORDER of writes, and the
 * assertions read that order rather than a byte count.
 *
 * Load-bearing cases, each with the mutation that would make it fail (3.3):
 *   * realtime skips on contention -- mutate Send to spin for the realtime
 *     caller and this goes red. Without it, a TxOwner that always spun would
 *     pass every other test here while violating CPP-3 in the field, where it
 *     shows up as a 100 Hz loop missing its deadline under estop load.
 *   * a skipped frame is DROPPED, not queued -- the counter proves nothing was
 *     retried later. A queue would present as a stale axis command applied one
 *     tick late, which QD-5 forbids and which no crash reveals.
 *   * short writes complete INSIDE the critical section -- the writer is rigged
 *     to accept one byte at a time, and the recorded order must show one
 *     uninterrupted run per frame.
 *   * exhausting partial_send_retry reports kShortWrite, not kSent -- the
 *     caller must close the connection; reporting success would leave half a
 *     frame in the stream and corrupt every frame after it.
 *
 * Not tested here: real concurrency. Two OS threads racing would make this
 * test flaky and prove less than the deterministic re-entrancy check below,
 * which drives the exact interleaving the guard exists to prevent by calling
 * Send from inside the writer. The atomic_flag itself is tested in common.
 */

#include "quadruped/tx_owner.h"

#include <cstdio>
#include <string>
#include <vector>

using quadruped::TxCaller;
using quadruped::TxOwner;
using quadruped::TxResult;

static int g_failures = 0;

#define CHECK(cond)                                                \
  do {                                                             \
    if (!(cond)) {                                                 \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);  \
      ++g_failures;                                                \
    }                                                              \
  } while (0)

namespace {

// Records every write as "<tag>:<n>" so the assertions can read the ORDER,
// which is the property under test. A byte counter would pass happily on an
// interleaved stream.
struct Recorder {
  std::vector<std::string> writes;
  std::string tag = "a";
  long chunk = -1;  // -1 = accept everything; >0 = accept at most this many
  long fail_after = -1;  // >=0 = return an error on that call index
  long calls = 0;

  long Write(const std::uint8_t*, std::size_t len) {
    const long idx = calls++;
    if (fail_after >= 0 && idx >= fail_after) return -1;
    const long n =
        (chunk > 0 && static_cast<long>(len) > chunk) ? chunk
                                                      : static_cast<long>(len);
    writes.push_back(tag + ":" + std::to_string(n));
    return n;
  }
};

const std::uint8_t kFrame[8] = {0xEB, 0x91, 0xEB, 0x90, 0x00, 0x00, 0x01, 0x00};

}  // namespace

int main() {
  // ---- a whole frame goes out in one write -------------------------------
  {
    Recorder rec;
    TxOwner tx([&rec](const std::uint8_t* d, std::size_t n) { return rec.Write(d, n); },
               3);
    CHECK(tx.Send(TxCaller::kRealtime, kFrame, sizeof(kFrame)) == TxResult::kSent);
    CHECK(rec.writes.size() == 1);
    CHECK(rec.writes[0] == "a:8");
    CHECK(tx.sent_count() == 1);
    CHECK(tx.tx_skip_count() == 0);
  }

  // ---- short writes are completed inside the same critical section -------
  {
    Recorder rec;
    rec.chunk = 3;  // 8 bytes => 3 + 3 + 2
    TxOwner tx([&rec](const std::uint8_t* d, std::size_t n) { return rec.Write(d, n); },
               3);
    CHECK(tx.Send(TxCaller::kNonRealtime, kFrame, sizeof(kFrame)) ==
          TxResult::kSent);
    CHECK(rec.writes.size() == 3);
    CHECK(rec.writes[0] == "a:3" && rec.writes[1] == "a:3" &&
          rec.writes[2] == "a:2");
    CHECK(tx.sent_count() == 1);
  }

  // ---- exhausting the retry budget reports a SHORT WRITE, never success ---
  {
    Recorder rec;
    rec.chunk = 1;  // 8 bytes with a budget of 2 completions cannot finish
    TxOwner tx([&rec](const std::uint8_t* d, std::size_t n) { return rec.Write(d, n); },
               2);
    const TxResult r = tx.Send(TxCaller::kNonRealtime, kFrame, sizeof(kFrame));
    CHECK(r == TxResult::kShortWrite);
    // Reporting kSent here would leave half a frame in the stream and the peer
    // would parse the remainder as a header -- the exact corruption FR-4 names.
    CHECK(tx.sent_count() == 0);
  }

  // ---- a writer error is not a short write -------------------------------
  {
    Recorder rec;
    rec.fail_after = 0;
    TxOwner tx([&rec](const std::uint8_t* d, std::size_t n) { return rec.Write(d, n); },
               3);
    CHECK(tx.Send(TxCaller::kRealtime, kFrame, sizeof(kFrame)) ==
          TxResult::kWriterFailed);
    CHECK(tx.sent_count() == 0);
  }

  // ---- THE case: realtime skips while the guard is held, and drops -------
  // Re-entrancy stands in for a second thread: the writer calls Send again from
  // inside the critical section, which is exactly the interleaving the guard
  // exists to prevent, and it is deterministic where a thread race would not be.
  {
    Recorder rec;
    TxOwner* self = nullptr;
    TxResult inner = TxResult::kSent;
    long inner_calls = 0;
    TxOwner tx([&](const std::uint8_t* d, std::size_t n) -> long {
      if (inner_calls++ == 0) {
        // Realtime caller arrives mid-frame: it must NOT write, must NOT wait.
        inner = self->Send(TxCaller::kRealtime, kFrame, sizeof(kFrame));
      }
      return rec.Write(d, n);
    }, 3);
    self = &tx;
    const TxResult outer = tx.Send(TxCaller::kNonRealtime, kFrame, sizeof(kFrame));
    CHECK(outer == TxResult::kSent);
    CHECK(inner == TxResult::kSkipped);
    // One frame on the wire, not two interleaved.
    CHECK(rec.writes.size() == 1);
    CHECK(tx.sent_count() == 1);
    // The skip is counted and the frame is gone -- no queue, no later retry.
    CHECK(tx.tx_skip_count() == 1);
  }

  // ---- degenerate input is loud, not silently "sent" ---------------------
  {
    Recorder rec;
    TxOwner tx([&rec](const std::uint8_t* d, std::size_t n) { return rec.Write(d, n); },
               3);
    CHECK(tx.Send(TxCaller::kRealtime, nullptr, 8) == TxResult::kShortWrite);
    CHECK(tx.Send(TxCaller::kRealtime, kFrame, 0) == TxResult::kShortWrite);
    CHECK(rec.writes.empty());
    CHECK(tx.sent_count() == 0);
  }

  if (g_failures == 0) {
    std::printf("ALL TX_OWNER TESTS PASSED\n");
    return 0;
  }
  std::printf("%d TX_OWNER TEST(S) FAILED\n", g_failures);
  return 1;
}
