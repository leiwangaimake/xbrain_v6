/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rt_thread.h
 * Brief: mlockall and SCHED_FIFO wrappers that report failure instead of hiding it
 *
 * Description:
 * What problem this solves. 13 S9.2 RTC-7 requires mlockall(MCL_CURRENT |
 * MCL_FUTURE) at startup and QD-7 lists it alongside "no dynamic allocation, no
 * blocking log" as what makes a 100 Hz / 200 Hz loop hold its cadence. Both are
 * a handful of POSIX calls, which is exactly why every process ends up writing
 * its own copy, and why those copies end up differing in the one place that
 * matters: what they do when the call fails.
 *
 * Which design sections this implements. 13 S9.2 RTC-7 (mlockall flags), 13
 * S9.1 (the thread table assigns SCHED_FIFO to ctrl and chs_b and ordinary
 * scheduling to everything else), QD-7, and 11 S9.12.6 whose implementation
 * constraints line reads "no dynamic allocation, no blocking log, no mutex a
 * non-realtime thread could hold, mlockall at startup".
 *
 * *** No priority value appears in this file, and none may be added. 13 S9.1
 * assigns a priority to each quadruped thread and 13 S9.2 RTC-8 records that
 * the affinity and priority VALUES are still open under D-42. A default here
 * would be a safety-shaped parameter with a made-up value baked into a shared
 * library, which CLAUDE.md 3.1 forbids outright; worse, it would be a value
 * nobody can find later, since the search would start in configs/. The priority
 * is a required argument. A caller with nothing to pass is a caller whose
 * configuration is not calibrated yet, and it must fail to start rather than
 * run at a priority somebody guessed.
 *
 * What these functions do NOT do:
 *   * they do not fall back. If SCHED_FIFO cannot be applied the call returns
 *     the errno and leaves the thread as it was; it never quietly continues
 *     under SCHED_OTHER. A process that believes it is realtime and is not is
 *     the fail-silent shape CLAUDE.md 3.1 is about -- the loop still runs, the
 *     jitter only appears under load, and nothing in the logs says why
 *   * they do not set CPU affinity. RTC-8 is open under D-42 and CLAUDE.md 9.3
 *     forbids building an interface for a decision that has not been made
 *   * they do not create or name threads beyond what SCHED_FIFO needs
 *   * they do not log. These run before the logging base is up, and one of them
 *     runs on the realtime path
 *
 * Why errno-style int returns rather than bool or an exception. The three
 * outcomes have three different fixes and the caller has to tell them apart:
 * EPERM means the unit is missing LimitRTPRIO / CAP_SYS_NICE, EINVAL means the
 * configured priority is outside the kernel's range for the policy, and ENOMEM
 * from mlockall means RLIMIT_MEMLOCK is too small. Collapsing them into false
 * sends the next person to the wrong file for an afternoon. Exceptions are out
 * because CPP-2 requires the realtime thread entry points to be noexcept.
 *
 * The trap that looks right and is not. Calling mlockall with MCL_CURRENT only,
 * because "everything is allocated by then". Thread stacks, the guard pages
 * behind them, and anything the DDS or TLS layers map later are future
 * mappings; without MCL_FUTURE they stay pageable and the first touch inside
 * the 100 Hz loop is a page fault worth milliseconds. It presents as jitter,
 * never as an error, and it is invisible on a lightly loaded desk machine.
 */

#ifndef HACHIST_XBRAIN_V6_COMMON_RTCOMM_RT_THREAD_H_
#define HACHIST_XBRAIN_V6_COMMON_RTCOMM_RT_THREAD_H_

// C system headers before C++ standard ones, per the Google order CLAUDE.md 5.1
// adopts. Nothing else is included: every consumer of this header is a process
// that must build without ROS (CLAUDE.md 5.3), and one of them sits on the
// emergency-stop path.
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>

#include <cerrno>

namespace hachist {
namespace xbrain {
namespace rtcomm {

// RTC-7. Returns 0 on success, otherwise the errno the kernel reported
// (ENOMEM when RLIMIT_MEMLOCK is too small, EPERM without CAP_IPC_LOCK on
// kernels that still require it).
//
// Call this before starting any thread. MCL_FUTURE covers mappings made after
// the call, so the stacks of threads created later are locked as they appear;
// the reverse order leaves those stacks pageable and the flag only looks right.
inline int LockAllMemory() noexcept {
  // errno is only meaningful after a failing call, so it is read only then --
  // reading it unconditionally would report a stale value from some earlier
  // library call and send the caller chasing a failure that did not happen.
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    return errno;
  }
  return 0;
}

// The kernel's legal SCHED_FIFO priority range on this machine. Exposed so a
// caller can validate a configured value and report the bound it violated,
// instead of turning a configuration mistake into an opaque EINVAL at startup.
// These are queried, never hardcoded: POSIX only guarantees a range of at least
// 32 levels, and the numbers differ between kernels.
inline int FifoPriorityMin() noexcept {
  return sched_get_priority_min(SCHED_FIFO);
}

inline int FifoPriorityMax() noexcept {
  return sched_get_priority_max(SCHED_FIFO);
}

// Move an existing thread to SCHED_FIFO at the caller's priority. Returns 0, or
// an errno. Priority is required; see the file comment on why there is no
// default.
//
// The range check happens here rather than being left to the kernel so that the
// caller can distinguish "your configured priority is out of range" from "you
// are not allowed to ask for realtime at all". Both come back as an int, but
// only one of them is fixed by editing the systemd unit.
inline int ApplyFifoPriority(pthread_t thread, int priority) noexcept {
  if (priority < FifoPriorityMin() || priority > FifoPriorityMax()) {
    return EINVAL;
  }

  // sched_param has members beyond sched_priority on some platforms, so the
  // whole struct is value initialised before the one field we care about is
  // set. Handing uninitialised bytes to the kernel is the kind of thing that
  // works until the day it does not.
  struct sched_param param = {};
  param.sched_priority = priority;

  // pthread_setschedparam returns the error rather than setting errno, which is
  // the opposite of mlockall above. Getting this backwards yields a function
  // that reports success on every failure.
  return pthread_setschedparam(thread, SCHED_FIFO, &param);
}

// Read back what a thread is actually running under. Returns 0, or an errno.
//
// This exists because "we called the setter and it returned 0" is not the same
// claim as "the thread is running SCHED_FIFO", and a startup self-check that
// cannot tell the two apart is a check that passes on a machine where realtime
// was never granted.
inline int ReadSchedule(pthread_t thread, int* policy, int* priority) noexcept {
  if (policy == nullptr || priority == nullptr) {
    return EINVAL;
  }

  struct sched_param param = {};
  const int rc = pthread_getschedparam(thread, policy, &param);
  if (rc != 0) {
    return rc;
  }
  *priority = param.sched_priority;
  return 0;
}

// Start a thread that runs SCHED_FIFO from its first instruction. Returns 0, or
// an errno; on failure no thread was created and *thread is untouched.
//
// Why the attribute route rather than pthread_create followed by
// ApplyFifoPriority. The second form runs the thread's first instructions under
// the parent's policy, so a loop that starts by taking its first deadline gets
// scheduled as an ordinary task for that window. On a busy machine that is
// where the first missed tick comes from, and it is not reproducible.
//
// PTHREAD_EXPLICIT_SCHED is the load-bearing line: without it pthread_create
// INHERITS the creating thread's scheduling and silently ignores both the
// policy and the priority set below. The call still returns 0. That is the
// single most common way this wrapper gets written wrong, and the only symptom
// is jitter.
inline int StartFifoThread(pthread_t* thread, int priority,
                           void* (*entry)(void*), void* arg) noexcept {
  if (thread == nullptr || entry == nullptr) {
    return EINVAL;
  }
  if (priority < FifoPriorityMin() || priority > FifoPriorityMax()) {
    return EINVAL;
  }

  pthread_attr_t attr;
  int rc = pthread_attr_init(&attr);
  if (rc != 0) {
    return rc;
  }

  struct sched_param param = {};
  param.sched_priority = priority;

  // Each step is checked and the attribute object is destroyed on every path.
  // A leaked pthread_attr_t is small, but this runs at startup in every
  // realtime process and a leak here is the sort of thing that is found by a
  // tool years later and cannot be attributed to anyone.
  rc = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
  if (rc == 0) {
    rc = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
  }
  if (rc == 0) {
    rc = pthread_attr_setschedparam(&attr, &param);
  }
  if (rc == 0) {
    rc = pthread_create(thread, &attr, entry, arg);
  }

  pthread_attr_destroy(&attr);
  return rc;
}

}  // namespace rtcomm
}  // namespace xbrain
}  // namespace hachist

#endif  // HACHIST_XBRAIN_V6_COMMON_RTCOMM_RT_THREAD_H_
