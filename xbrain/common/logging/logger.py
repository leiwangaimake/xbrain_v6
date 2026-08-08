"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: logger.py
Brief: CHK-1-57 -- get_logger(proc_name) singleton, non-blocking + English-
       only + logrotate-safe

Description:
Every Python process in XBRAIN_V6 (p1_motion / p2_core / p3_task / p4_agent /
p5_gateway / payload / ai_asr / ai_llm / ai_tts / ai_perception -- 10 of
them) has ONE logger, and this file is the ONE place that constructs it.
The five properties that had to be baked in at construction, per CHK-1-57:

  (1) Uniform format so a `grep -h` across all ten log files stays
      column-aligned: monotonic relative time (10 S0.2.1 D-18), wall ts
      for cross-host alignment, proc name, level, module.
  (2) English-only assertion at RUN TIME (CLAUDE.md S2.1 -- logs / print /
      exception messages MUST BE ALL ENGLISH). A charset lint is the
      static half; this file is the runtime half. Non-ASCII in a msg
      RAISES -- it does not warn -- because a warning that lands as
      another log line would be silently swallowed if the assertion
      failed inside the logging path itself.
  (3) Non-blocking sink. Handlers run on a background thread via
      QueueHandler + QueueListener so the ctrl-loop can .info() without
      being pulled into a stat() or a WAL fsync from the log sink.
      MOT-PM-2 gives P1 a 60 ms P99 budget; a synchronous handler with a
      logrotate mv underway can easily eat 10+ ms.
  (4) logrotate-safe. WatchedFileHandler stats dev+ino before every
      emit and reopens if the file it holds an fd on has been renamed
      or unlinked -- otherwise the log continues writing to an already-
      deleted inode and disappears at next FD close.
  (5) Off-tree default. Sink is /opt/xbrain_v6/data/logs/{proc}/{proc}.log
      (CLAUDE.md S0.2 -- data/ is where log files live). Overridable via
      XBRAIN_LOG_DIR for tests; /var/log or similar is NOT allowed --
      CHK-1-57 variant (d) turns that into a runtime failure.

The module is DELIBERATELY defensive at construction: bad proc_name
(non-identifier / contains a slash) raises immediately instead of
producing a logger that later fails to write. The 10-process membership
is enforced by the caller (only that caller knows which proc it IS), not
by a whitelist here; the invariant this module owns is "given ANY name
this shape, the logger is well-formed."

Cross-references:
  - CLAUDE.md S2.1 English-only rule
  - CLAUDE.md S3.4 monotonic clock rule (D-18 / 11 S0.2.1)
  - CLAUDE.md S0.2 data/ layout
  - MOT-PM-2 P1 loop budget (why non-blocking matters)
"""

import logging
import logging.handlers
import os
import queue
import re
import sys
import time
from typing import Optional

# ---------------------------------------------------------------------------
# Module-level constants
# ---------------------------------------------------------------------------

# Monotonic zero-mark: every log record's "+t" column is measured against
# this. Captured at MODULE IMPORT so multiple processes started at the same
# systemd unit invocation share a comparable relative frame (they still
# each capture their own zero, but that zero is stable across the process's
# lifetime -- reader can subtract two lines to get elapsed time without
# needing to worry about clock steps or DST).
_MONO_ZERO = time.monotonic()

# Default sink root -- overridable for tests but NOT for deploy. Test rig
# sets XBRAIN_LOG_DIR to a tmp_path; production leaves it unset and the
# baked-in path is used. CHK-1-57 variant (d) mutates this to /var/log
# and the golden-path assertion goes red.
_DEFAULT_LOG_ROOT = "/opt/xbrain_v6/data/logs"

# Env var for level. Not-set default is INFO -- CHK-1-57's default matches
# what every downstream ops runbook expects at bring-up.
_LEVEL_ENV = "XBRAIN_LOG_LEVEL"

# Env var for override root; only for tests + local dev. Read at each
# get_logger call so a test's monkeypatched env is seen -- caching would
# force test runs to reset a module global they should not care about.
_LOG_DIR_ENV = "XBRAIN_LOG_DIR"

# Env var for the drop-in default level when nothing else is specified. This
# is deliberately separate from _LEVEL_ENV so the env var and the code
# default can be reasoned about independently.
_DEFAULT_LEVEL = "INFO"

# The proc_name shape: lowercase letters, digits, underscore. Rejects slashes
# (path traversal into another proc's dir), empty strings, and mixed-case
# tokens (a "P1_motion" vs "p1_motion" split would create two log files that
# a grep might miss).
_PROC_NAME_RE = re.compile(r"^[a-z][a-z0-9_]*$")

# Cache of already-constructed loggers, keyed by proc_name. A repeated
# get_logger for the same proc returns the SAME logger -- otherwise every
# call adds another handler and each line prints N times. Module-level dict
# instead of a class attribute so tests can inspect it directly via
# _logger_mod._LOGGERS without instantiating anything.
_LOGGERS: dict = {}

# One QueueListener per proc; kept in a dict so tests can stop them via
# _shutdown_all() without having to reach into logging internals.
_LISTENERS: dict = {}

# Legal level strings the env var / kwarg may carry. Anything else raises
# ValueError -- silence-is-not-success (CLAUDE.md S3.2).
_LEVEL_NAMES = ("DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL")


# ---------------------------------------------------------------------------
# Format + filter helpers
# ---------------------------------------------------------------------------

class _MonotonicFormatter(logging.Formatter):
    """Formatter that emits:
      [+SS.mmm] [YYYY-MM-DD HH:MM:SS.mmm] [proc] [LEVEL] [module] msg

    monotonic delta comes FIRST so a `sort -n` on the second column groups
    events in wall order without needing to parse a full timestamp -- an
    incident post-mortem stitches two proc logs together by wall ts, and by
    monotonic delta to compare within one proc.
    """

    def format(self, record: logging.LogRecord) -> str:
        # Compute delta each format call rather than at emit-time: rounding
        # is stable so a re-format of the same record gives the same string
        # (matters for tests that call the formatter twice on one record).
        # created_monotonic is stamped by _StampMonotonic BEFORE the record
        # is queued (see that filter's docstring for why the stamp cannot
        # happen here in the formatter -- queue latency would then land in
        # the delta column and make it useless for latency analysis).
        delta = record.created_monotonic - _MONO_ZERO
        # Wall ts uses record.created (Python's own time.time() at log call).
        # Split into strftime + explicit ".mmm" so a locale change never
        # rewrites the ms format -- localtime + strftime alone would give
        # locale-dependent output on some systems (day-name / month-name in
        # a non-ASCII locale would ALSO trip _EnglishOnlyFilter downstream,
        # but only for records emitted through the formatter, not those we
        # never format -- keep the ms-tail explicit either way).
        wall = time.strftime("%Y-%m-%d %H:%M:%S",
                             time.localtime(record.created)) \
            + ".%03d" % ((record.created * 1000) % 1000)
        # record.name is the proc name (we set it in _make_logger); module
        # is record.module (Python auto-populated from the .py file where
        # the log call originated). Both live inside brackets so a naive
        # awk on `[.*]` yields exactly five bracketed groups.
        # %07.3f in the delta field: 3 fractional digits (ms precision) +
        # 3 leading integer digits (up to ~999s uptime) padded with zeros
        # so a `sort -n` on that column stays lexically ordered even for
        # early lines with a delta below 100 s.
        return "[+%07.3f] [%s] [%s] [%s] [%s] %s" % (
            delta, wall, record.name, record.levelname, record.module,
            record.getMessage(),
        )


class _EnglishOnlyFilter(logging.Filter):
    """Filter that ASSERTS every record's rendered msg is pure ASCII.
    Not a warn -- CHK-1-57 (2) is explicit that non-ASCII RAISES. A raise
    in the filter chain propagates out of the log call, which is what
    forces the caller to fix the source rather than seeing a stray warn.

    Runs BEFORE queue submission so the raise reaches the caller stack
    (a raise inside the QueueListener thread would only reach stderr).
    """

    def filter(self, record: logging.LogRecord) -> bool:
        # getMessage() applies %-args, so the check covers formatted %s
        # values too. If the format itself is not ASCII (e.g. "%s 中文
        # %d"), .encode fails and we raise; if only an arg is non-ASCII,
        # it also fails after substitution -- both are what CHK-1-57 (2)
        # requires. Bytes are attempted via ascii encoding rather than
        # a regex over unicode categories because that catches every
        # non-ASCII codepoint including full-width punctuation, without a
        # separate charset table.
        rendered = record.getMessage()
        try:
            rendered.encode("ascii")
        except UnicodeEncodeError as exc:
            raise AssertionError(
                "log message must be pure ASCII (CLAUDE.md S2.1): %r -- "
                "offending char at pos %d" % (rendered, exc.start)
            ) from exc
        return True                      # let record through if ASCII


class _StampMonotonic(logging.Filter):
    """Attach record.created_monotonic BEFORE _EnglishOnlyFilter runs.
    Split into its own filter so a future formatter can swap without
    losing the monotonic stamp -- and so the ordering of filters is a
    visible declaration in _make_logger rather than an implicit dance."""

    def filter(self, record: logging.LogRecord) -> bool:
        # Snap once at emit; if we let the formatter capture it later,
        # a queued record could see wall time drift between enqueue and
        # dequeue -- monotonic delta would then measure queue latency
        # instead of caller elapsed.
        record.created_monotonic = time.monotonic()
        return True


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def _resolve_level(name: Optional[str]) -> int:
    """Turn a level NAME into the numeric constant, or raise if unknown.
    Kept separate from get_logger so the env-var reading path and the
    kwarg path share one validator (mutation (e) is 'ignore env, always
    INFO' -- a caller that removed this call and inlined INFO would be
    detected by the level test).
    """
    if name is None:
        # Distinct from an unset env: caller EXPLICITLY passed None.
        # Fall back to default so a proc without an env var still logs.
        name = _DEFAULT_LEVEL
    upper = name.upper()
    if upper not in _LEVEL_NAMES:
        raise ValueError(
            "unknown level %r; expected one of %s" % (name, _LEVEL_NAMES)
        )
    return getattr(logging, upper)


def _log_dir_for(proc_name: str) -> str:
    """Return the directory a proc's log files live in. Reads
    XBRAIN_LOG_DIR each call so a test env override is seen (see the
    _LOG_DIR_ENV comment above).

    proc_name is trusted here (get_logger already validated it with
    _PROC_NAME_RE); if this ever gets called from somewhere that DID NOT
    validate first, os.path.join will still produce a safe path but the
    caller might land the log outside its expected proc directory.
    """
    # env-read every call: a test that monkeypatch.setenv-s XBRAIN_LOG_DIR
    # after import needs the change to take effect at get_logger time, not
    # at module load. Cheap dict lookup, no reason to cache.
    root = os.environ.get(_LOG_DIR_ENV, _DEFAULT_LOG_ROOT)
    return os.path.join(root, proc_name)


def _make_logger(proc_name: str, level_name: Optional[str]) -> logging.Logger:
    """Build the logger + queue listener pair. Called once per proc_name;
    subsequent get_logger returns the cached instance.

    Wire diagram:
      caller  --  QueueHandler  --  queue.Queue  --  QueueListener thread
                                                        |
                                                        + _MonotonicFormatter
                                                        + WatchedFileHandler
                                                        + StreamHandler(stderr)

    Each caller-side .info/.debug/... only touches the queue (bounded but
    unbounded here -- log volume is bounded by 20 Hz control loops, so an
    unbounded queue trades a few KB of RAM at burst peaks against the
    complication of a dropped-record policy). Everything else runs on
    the listener thread.
    """
    # Bare Logger, NOT rootLogger. Using the root would let a third-party
    # basicConfig() clobber our handler set. Prefixed with "xbrain." so a
    # test running under pytest sees a distinct name from the pytest root.
    logger = logging.getLogger("xbrain." + proc_name)
    # setLevel on the logger + on the handlers below. Only messages >= level
    # ever reach the queue; anything below is dropped before the enqueue,
    # which is what makes DEBUG a cheap no-op when the level is INFO.
    logger.setLevel(_resolve_level(level_name))
    # propagate=False so a pytest capture rig on the root logger does not
    # receive our records (which would double-count them for output tests).
    logger.propagate = False

    # Sink directory. Idempotent mkdir so a proc that starts before the
    # tmpfiles.d fires still lands somewhere writable. exist_ok=True
    # tolerates a racing proc that started the same second.
    sink_dir = _log_dir_for(proc_name)
    os.makedirs(sink_dir, exist_ok=True)
    sink_path = os.path.join(sink_dir, proc_name + ".log")

    # Formatter -- one instance shared across handlers.
    fmt = _MonotonicFormatter()

    # Queue for the async handoff. maxsize=0 means unbounded; see comment
    # above on why that trade-off is acceptable at 20 Hz.
    log_queue: queue.Queue = queue.Queue(maxsize=0)

    # WatchedFileHandler is the CHK-1-57 (5) hardening -- it stat()s dev/ino
    # before each emit and reopens if they changed. A vanilla FileHandler
    # would keep writing to an unlinked inode after logrotate's mv.
    fh = logging.handlers.WatchedFileHandler(sink_path, encoding="utf-8")
    fh.setLevel(_resolve_level(level_name))
    fh.setFormatter(fmt)
    # StreamHandler(stderr) so systemd's journal captures every record too;
    # if the FS sink fails (disk full, permission bounce during rotation),
    # journal is the fallback that still records the incident.
    sh = logging.StreamHandler(sys.stderr)
    sh.setLevel(_resolve_level(level_name))
    sh.setFormatter(fmt)

    # QueueListener drains the queue on a background thread and dispatches
    # to real handlers. respect_handler_level so a handler with a stricter
    # level than the logger drops on its own without touching the queue.
    listener = logging.handlers.QueueListener(
        log_queue, fh, sh, respect_handler_level=True,
    )
    listener.start()
    _LISTENERS[proc_name] = listener

    # Caller-side handler is a QueueHandler; ONLY thing it does is put()
    # the record on the queue. That put() is what makes the caller path
    # sub-millisecond even if the sink is momentarily blocked.
    qh = logging.handlers.QueueHandler(log_queue)
    # StampMonotonic must run BEFORE EnglishOnlyFilter so the timestamp is
    # attached to the record either way -- otherwise a non-ASCII log that
    # raises would leave downstream inspection of the record short a field.
    qh.addFilter(_StampMonotonic())
    qh.addFilter(_EnglishOnlyFilter())
    logger.addHandler(qh)

    return logger


def get_logger(proc_name: str, level: Optional[str] = None) -> logging.Logger:
    """Return the singleton logger for `proc_name`.

    level defaults to XBRAIN_LOG_LEVEL (env), then _DEFAULT_LEVEL (INFO).
    Caller passing level= wins over the env; env wins over default.
    CHK-1-57 (e) mutation is "ignore env, always INFO" -- a caller that
    removed the env lookup would be caught by the level test which sets
    XBRAIN_LOG_LEVEL=WARNING and asserts .isEnabledFor(logging.INFO) is False.
    """
    if not _PROC_NAME_RE.match(proc_name):
        raise ValueError(
            "proc_name must match %s; got %r"
            % (_PROC_NAME_RE.pattern, proc_name)
        )
    if proc_name in _LOGGERS:
        return _LOGGERS[proc_name]
    # Resolve level: explicit kwarg -> env -> default. Explicit kwarg first
    # so a test can force a level without touching env.
    if level is None:
        level = os.environ.get(_LEVEL_ENV)
    logger = _make_logger(proc_name, level)
    _LOGGERS[proc_name] = logger
    return logger


def _shutdown_all() -> None:
    """Stop every QueueListener, remove handlers off the underlying Logger
    objects, and drop the cache. Test-only helper -- production processes
    run until systemd stops them, at which point the OS reaps the thread.
    Public-underscored so importlib does not treat it as a re-export target.

    Removing handlers is REQUIRED because logging.getLogger returns the same
    global object across calls; if we only drop our _LOGGERS cache and not
    the handlers we attached to that global, a repeat get_logger would ADD
    another QueueHandler and every future log line would print twice.
    """
    for name, listener in list(_LISTENERS.items()):
        listener.stop()
        del _LISTENERS[name]
    for name, logger in list(_LOGGERS.items()):
        # Copy the handler list so removal does not mutate under iteration
        # (logger.handlers is the live list Python's logging module reads).
        for handler in list(logger.handlers):
            logger.removeHandler(handler)
    _LOGGERS.clear()
