/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rt_thread_probe.cc
 * Brief: Probes mlockall flags and the SCHED_FIFO wrapper, reports errno as-is
 *
 * Description:
 * What this establishes. 13 S9.2 RTC-7 requires mlockall(MCL_CURRENT |
 * MCL_FUTURE), and "the call returned zero" is not evidence that it did what
 * RTC-7 asks. Two different things are checked instead, both from
 * /proc/self/status:
 *   * after the call, VmLck is non-zero -- the resident image really is locked,
 *     so a wrapper that passed no flags at all would be caught
 *   * a mapping made AFTER the call is locked too, and VmLck grows by its size.
 *     This is the MCL_FUTURE half, and it is the one that matters: thread
 *     stacks, guard pages and anything the DDS or TLS layers map later are all
 *     future mappings. With MCL_CURRENT alone they stay pageable and the first
 *     touch inside the 100 Hz loop is a page fault worth milliseconds, visible
 *     only as jitter
 *
 * For SCHED_FIFO the honest position is stated rather than worked around. On a
 * box where RLIMIT_RTPRIO is zero, pthread_setschedparam returns EPERM and no
 * amount of test code changes that. So this program reports the errno and lets
 * the runner announce what went unverified. What it can check regardless of
 * privilege is the wrapper's own contract: an out-of-range priority is rejected
 * with EINVAL before any syscall, a null output pointer is rejected, and a
 * failed StartFifoThread creates no thread. Those are the parts that would
 * silently rot if nobody exercised them until the day the robot boots.
 *
 * What this does NOT establish. That the ORIN will grant realtime -- that
 * depends on the systemd unit's limits, which are not this library's business.
 * Nor anything about CPU affinity: RTC-8 is open under D-42 and the wrapper
 * deliberately has no affinity API to probe.
 *
 * The trap this file exists to keep out. A wrapper that falls back to
 * SCHED_OTHER when FIFO is refused, returning success either way. Everything
 * keeps working on the desk, the soak looks fine, and the jitter only appears
 * on a loaded robot with nothing in the logs to explain it. The check for that
 * is simply that a refusal comes back as a non-zero errno.
 *
 * Output contract. key=value lines, asserted by tests/common/test_rtcomm.py.
 * Kernel constants (SCHED_FIFO, EINVAL, EPERM) are printed rather than assumed
 * on the Python side, so the comparison is against this machine's values.
 */

#include "xbrain/common/rtcomm/rt_thread.h"

#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>

#include <cerrno>
#include <cstdio>

namespace {

// One mapping large enough that its arrival in VmLck cannot be confused with
// ordinary allocator noise, and small enough to be nothing on a machine with
// RLIMIT_MEMLOCK in the gigabytes.
const size_t kProbeMapBytes = 8u * 1024u * 1024u;

// VmLck in kilobytes, or -1 if the field could not be read. Parsed from
// /proc/self/status because there is no portable syscall that answers "how much
// of this process is locked", and the whole point is to check the kernel's
// opinion rather than the wrapper's return value.
long ReadVmLckKb() {
  std::FILE* fh = std::fopen("/proc/self/status", "r");
  if (fh == nullptr) {
    return -1;
  }
  char line[256];
  long kb = -1;
  while (std::fgets(line, sizeof(line), fh) != nullptr) {
    long value = 0;
    if (std::sscanf(line, "VmLck: %ld kB", &value) == 1) {
      kb = value;
      break;
    }
  }
  std::fclose(fh);
  return kb;
}

// Results of the FIFO thread's self-inspection, read after join.
int g_thread_policy = -1;
int g_thread_priority = -1;
int g_thread_read_rc = -1;

void* InspectSelf(void*) {
  // Read from inside the thread: pthread_getschedparam on a thread that has
  // already exited is undefined, and doing it here removes the race entirely.
  g_thread_read_rc = hachist::xbrain::rtcomm::ReadSchedule(
      pthread_self(), &g_thread_policy, &g_thread_priority);
  return nullptr;
}

}  // namespace

int main() {
  // Constants first, so the Python side compares against this kernel's values
  // rather than numbers copied from a manual page.
  std::printf("const_sched_fifo=%d\n", SCHED_FIFO);
  std::printf("const_sched_other=%d\n", SCHED_OTHER);
  std::printf("const_einval=%d\n", EINVAL);
  std::printf("const_eperm=%d\n", EPERM);

  // ---------------------------------------------------------------------
  // RTC-7. mlockall, and then the evidence that it took effect.
  // ---------------------------------------------------------------------
  std::printf("vmlck_before_kb=%ld\n", ReadVmLckKb());
  const int lock_rc = hachist::xbrain::rtcomm::LockAllMemory();
  std::printf("lockall_rc=%d\n", lock_rc);
  const long after_lock_kb = ReadVmLckKb();
  std::printf("vmlck_after_lock_kb=%ld\n", after_lock_kb);

  void* probe = mmap(nullptr, kProbeMapBytes, PROT_READ | PROT_WRITE,
                     MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
  std::printf("probe_mapped=%d\n", probe != MAP_FAILED ? 1 : 0);
  long after_map_kb = -1;
  if (probe != MAP_FAILED) {
    after_map_kb = ReadVmLckKb();
    munmap(probe, kProbeMapBytes);
  }
  std::printf("vmlck_after_map_kb=%ld\n", after_map_kb);
  std::printf("probe_map_kb=%ld\n", static_cast<long>(kProbeMapBytes / 1024u));

  // Unlocked again so the rest of the program, and anything the harness does
  // with the core file if this crashes, is not affected by a locked image.
  munlockall();

  // ---------------------------------------------------------------------
  // The wrapper's own contract, checkable without any privilege.
  // ---------------------------------------------------------------------
  const int prio_min = hachist::xbrain::rtcomm::FifoPriorityMin();
  const int prio_max = hachist::xbrain::rtcomm::FifoPriorityMax();
  std::printf("prio_min=%d\n", prio_min);
  std::printf("prio_max=%d\n", prio_max);

  std::printf("rc_priority_too_high=%d\n",
              hachist::xbrain::rtcomm::ApplyFifoPriority(pthread_self(),
                                                         prio_max + 1));
  std::printf("rc_priority_too_low=%d\n",
              hachist::xbrain::rtcomm::ApplyFifoPriority(pthread_self(),
                                                         prio_min - 1));

  int policy = -1;
  std::printf("rc_null_priority_out=%d\n",
              hachist::xbrain::rtcomm::ReadSchedule(pthread_self(), &policy,
                                                    nullptr));

  pthread_t unused_thread;
  std::printf("rc_start_bad_priority=%d\n",
              hachist::xbrain::rtcomm::StartFifoThread(
                  &unused_thread, prio_max + 1, InspectSelf, nullptr));
  std::printf("rc_start_null_entry=%d\n",
              hachist::xbrain::rtcomm::StartFifoThread(&unused_thread, prio_min,
                                                       nullptr, nullptr));

  // ---------------------------------------------------------------------
  // The part that needs privilege. Reported, not required.
  // ---------------------------------------------------------------------
  const int apply_rc =
      hachist::xbrain::rtcomm::ApplyFifoPriority(pthread_self(), prio_min);
  std::printf("rc_apply_self=%d\n", apply_rc);

  pthread_t fifo_thread;
  const int start_rc = hachist::xbrain::rtcomm::StartFifoThread(
      &fifo_thread, prio_min, InspectSelf, nullptr);
  std::printf("rc_start_fifo=%d\n", start_rc);
  if (start_rc == 0) {
    pthread_join(fifo_thread, nullptr);
    std::printf("thread_read_rc=%d\n", g_thread_read_rc);
    std::printf("thread_policy=%d\n", g_thread_policy);
    std::printf("thread_priority=%d\n", g_thread_priority);
  }

  // Self is put back to ordinary scheduling if the earlier call succeeded, so
  // the process does not exit holding a realtime priority.
  if (apply_rc == 0) {
    struct sched_param param = {};
    pthread_setschedparam(pthread_self(), SCHED_OTHER, &param);
  }
  return 0;
}
