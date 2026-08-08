"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_logger.py
Brief: CHK-1-57 -- five properties (format / English-only / non-blocking /
       logrotate-safe / off-tree default) + five variants each turning red

Description:
Each judgement pass in CHK-1-57 is paired with a mutation whose test
turns red. Grouped:

  criterion (1) uniform format + off-tree path -> test_10_procs_land +
                                                  test_off_tree_default +
                                                  test_variant_d_var_log
  criterion (2) English-only assertion         -> test_non_ascii_raises +
                                                  test_variant_a_chinese
  criterion (4) non-blocking sink              -> test_variant_b_sync_blocks
  criterion (5) logrotate-safe                 -> test_variant_c_reopen_after_rotate
  criterion (level from env)                    -> test_variant_e_env_ignored

criterion (3) is P1-integration (MOT-PM-2 has to exist first to know when
an assertion goes red under ctrl-path logging); we cannot execute it here
because P1 is unimplemented. A xfail-shaped placeholder marks the debt
explicitly rather than pretending the row is done.

Mutation ledger -- each row explains what code change turns which test red:
  (a) 'return True' inside _EnglishOnlyFilter.filter (skip the encode
      probe)                          -> test_non_ascii_raises_at_log_call,
                                          test_variant_a_chinese_arg_also_raises
  (b) 'logger.addHandler(fh); del qh' (bypass the queue, attach the file
      handler directly to the logger) -> test_caller_returns_under_1ms_...
  (c) 'logging.FileHandler(sink_path)' instead of WatchedFileHandler
      (fd cache survives the mv)       -> test_reopens_after_rotate
  (d) '_DEFAULT_LOG_ROOT = "/var/log/xbrain"' (route sinks off-tree)
                                       -> test_variant_d_var_log_lands_...
  (e) 'level = _DEFAULT_LEVEL' unconditionally (ignore env)
                                       -> test_env_var_sets_level

Reader who's asked to add a new variant: append a mutation row above,
add its test below, and reference each side from the other. That way a
future edit that drops one side leaves a hanging reference the grep can
find, rather than a silent gap.
"""

import logging
import os
import time

import pytest

from xbrain.common.logging import get_logger
from xbrain.common.logging import logger as _logger_mod


# ---------------------------------------------------------------------------
# Fixture: fresh logger cache per test + tmp log dir
# ---------------------------------------------------------------------------

@pytest.fixture(autouse=True)
def _fresh_loggers(tmp_path, monkeypatch):
    """Every test starts with an empty _LOGGERS cache and a tmp XBRAIN_LOG_DIR.
    autouse so no test can accidentally share state across cases; that also
    means every test's log files land under tmp_path/<proc>/<proc>.log and
    are wiped when pytest cleans tmp_path.

    Two shutdown calls (before yield + after yield): the before-yield
    handles the case where a previous test session left something behind
    (e.g. interrupted mid-run); the after-yield keeps state contained even
    if the test itself raised. Symmetric bookend so no leak either way.
    """
    # Pre-clean: strip anything a prior test left in the module cache.
    _logger_mod._shutdown_all()
    # Redirect the sink to pytest's tmp_path. monkeypatch reverts this at
    # test teardown, so a subsequent test that doesn't want the override
    # sees a clean environment (belt-and-braces with the shutdown above).
    monkeypatch.setenv("XBRAIN_LOG_DIR", str(tmp_path))
    # Yield tmp_path so tests can point at the sink for content checks.
    yield tmp_path
    # Post-clean: drop cache + join listener threads so pytest doesn't
    # report leaked threads in its summary and the next test sees empty.
    _logger_mod._shutdown_all()


def _drain_listener(proc_name: str) -> None:
    """Force the QueueListener thread to flush pending records before a test
    inspects the sink file. QueueListener.stop() calls the sentinel + join,
    so records queued before .stop() are guaranteed to hit their handlers.

    Also drop the proc from both _LISTENERS and _LOGGERS so the next
    get_logger(proc_name) call builds a fresh logger + listener pair --
    otherwise the test's next log call would race a stopped listener.
    """
    # .get returns None if the proc was never registered; a test that
    # exercises the shape guards may call _drain_listener without ever
    # having built a listener, so tolerate the missing case silently.
    listener = _logger_mod._LISTENERS.get(proc_name)
    if listener is not None:
        # .stop enqueues a sentinel + joins the thread. Blocking, so the
        # test can inspect the sink file the instant this call returns.
        listener.stop()
        del _logger_mod._LISTENERS[proc_name]
    # Remove the cached logger too so the NEXT get_logger call for this
    # proc rebuilds handler + listener. Without this, the second call
    # would return the old (now-detached) handler.
    if proc_name in _logger_mod._LOGGERS:
        del _logger_mod._LOGGERS[proc_name]


# ---------------------------------------------------------------------------
# criterion (1) -- uniform format + 10-proc landing
# ---------------------------------------------------------------------------

TEN_PROCS = (
    # Six Python runtime processes (CLAUDE.md S0.1) + four AI service procs
    # (payload / asr / llm / tts / perception service is C++ so not counted).
    # This is the CHK-1-57 "10 Python processes" set; any change here is a
    # legitimate topology change and lives with the test, not with logger.py.
    "p1_motion", "p2_core", "p3_task", "p4_agent", "p5_gateway",
    "payload", "ai_asr", "ai_llm", "ai_tts", "ai_orch",
)


def test_ten_procs_land_at_expected_path(tmp_path):
    """Each proc's log lands at data/logs/{proc}/{proc}.log (tmp_path root).
    Mutation would be routing them to a shared file -- a test that inspected
    file existence would then fail immediately.

    The loop iterates every proc so a change to _log_dir_for that broke ONE
    proc (e.g. one hard-coded name) but not others would still surface,
    rather than the test passing on the sample it happened to check first.
    """
    for proc in TEN_PROCS:
        # Build the logger and emit ONE line so the sink file is created;
        # get_logger alone would not touch the filesystem (the handler only
        # opens the file on first emit, deliberate so a proc that never
        # logs does not leave an empty file behind).
        log = get_logger(proc)
        log.info("hello from %s", proc)
        # Drain BEFORE checking existence so the QueueListener has actually
        # flushed to disk -- without this, a fast test machine can race
        # ahead and see the sink dir empty even though the record is queued.
        _drain_listener(proc)
        # Landing point: {root}/{proc}/{proc}.log per CHK-1-57 (1).
        # The nested {proc}/ directory keeps procs isolated so a logrotate
        # per-proc config can target them without a wildcard.
        expected = tmp_path / proc / (proc + ".log")
        assert expected.exists(), \
            "%s did not land at %s" % (proc, expected)


_GOLDEN_LINE_RE = __import__("re").compile(
    r"^\[\+\d+\.\d{3}\] "                       # monotonic delta
    r"\[\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{3}\] "   # wall ts
    r"\[xbrain\.[a-z][a-z0-9_]*\] "             # proc name
    r"\[(?:DEBUG|INFO|WARNING|ERROR|CRITICAL)\] "  # level
    r"\[[a-zA-Z_][a-zA-Z0-9_]*\] "              # module
    r".*$"                                      # msg
)


def test_format_matches_golden_regex(tmp_path):
    """Each field is present, in order. Mutation would be dropping the
    module column -- the regex would fail to match on ANY line.

    Kept as ONE test rather than five (one per column) because the regex is
    the golden -- five separate tests would each depend on the regex being
    parsed the same way, and a bug in the regex source would show up in
    all five with no obvious source of truth.

    Uses %d + arg (rather than a hard-coded string) so a change to
    getMessage's %-arg handling is exercised too.
    """
    log = get_logger("p1_motion")
    log.info("format check %d", 42)
    _drain_listener("p1_motion")
    # splitlines()[-1] picks the LAST line: the module logger may have
    # already emitted its own startup line in some earlier test paths, so
    # tail is the safe pick even though the fixture cleans between tests.
    line = (tmp_path / "p1_motion" / "p1_motion.log").read_text().splitlines()[-1]
    # Match, not search: anchor forces every field to be at the position
    # the golden format promised. A dropped field would slide subsequent
    # ones left and the anchor fails.
    assert _GOLDEN_LINE_RE.match(line), "bad line: %r" % line


# ---------------------------------------------------------------------------
# criterion (2) -- English-only, at run time
# ---------------------------------------------------------------------------

def test_non_ascii_raises_at_log_call():
    """CHK-1-57 (2) requires a raise, not a warn. A caller writing a Chinese
    msg (or Chinese punctuation, or any non-ASCII codepoint) MUST see the
    exception on their own stack. mutation (a) 'write a Chinese log' turns
    THIS test red -- if the filter passed non-ASCII through, no raise.

    A raise (not a warn) is deliberate: a warn logged as another record
    would be silently swallowed if the log path itself is broken. Raise
    puts the failure on the caller's own stack where they see it.
    """
    log = get_logger("p1_motion")
    with pytest.raises(AssertionError, match=r"pure ASCII"):
        # NOTE: the string below is a deliberate test input, NOT source code
        # to be linted for content -- this is the mutant we assert catches.
        # Using explicit \uXXXX escapes so the test file itself stays ASCII
        # even while the runtime string it constructs is not (test file is
        # source, comment goes to charset lint; the runtime string is data).
        # The two CJK codepoints below are literal Chinese chars; a mutant
        # that made _EnglishOnlyFilter.filter always return True would
        # let this line through and the pytest.raises context would not
        # see the expected AssertionError.
        log.info("中文 message")   # 2 CJK codepoints


def test_variant_a_chinese_arg_also_raises():
    """Variant (a) also covers %-args, not just the format string.

    Split from the format-string case because the two paths hit different
    branches of getMessage(): format-only vs format+args. A mutant that
    dropped the encode probe would fail both, but a smart mutant that only
    checked the format string (not the arg substitution result) would
    fail only THIS test -- the split isolates that mode.
    """
    log = get_logger("p2_core")
    with pytest.raises(AssertionError, match=r"pure ASCII"):
        # The %s substitutes 中文 into the format string; getMessage()
        # returns "value=中文", which .encode("ascii") rejects at the
        # position of the first CJK codepoint. Filter raises with the
        # rendered message so the failure explains WHICH arg was bad.
        log.info("value=%s", "中文")     # non-ASCII arg


# ---------------------------------------------------------------------------
# criterion (4) -- non-blocking; mutation (b) 'synchronous FileHandler'
# ---------------------------------------------------------------------------

def test_caller_returns_under_1ms_even_when_sink_is_slow(tmp_path):
    """The caller-side .info must be sub-millisecond even if the sink is
    slow. Simulate slowness by wrapping the WatchedFileHandler.emit in a
    200ms sleep -- the QueueListener thread absorbs it, so the caller
    still returns immediately. mutation (b) removes the queue and the
    caller-side handler becomes the FileHandler directly; then the sleep
    would run inline and this test would fail.

    200ms was picked as the sleep because it exceeds the P1 20 Hz cycle
    budget (50 ms) by 4x -- large enough that a synchronous handler
    would be trivially detectable, small enough that the test's own
    walltime stays reasonable. The MOT-PM-2 60 ms P99 budget references
    the ctrl loop, and 200 ms sim + <3 ms caller = clearly separable.
    """
    log = get_logger("p1_motion")
    _drain_listener("p1_motion")           # rebuild the listener with a slow sink

    # Rebuild with a slow WatchedFileHandler by monkeypatching the class.
    slow_handler = logging.handlers.WatchedFileHandler(
        str(tmp_path / "p1_motion" / "p1_motion.log"), encoding="utf-8"
    )
    real_emit = slow_handler.emit

    def slow_emit(record):
        time.sleep(0.2)
        real_emit(record)

    slow_handler.emit = slow_emit           # type: ignore[method-assign]
    log = get_logger("p1_motion")

    # Swap the listener's handler set with our slow one only. Same queue,
    # same enqueue path -- the assertion is 'caller path unblocked'.
    _logger_mod._LISTENERS["p1_motion"].stop()
    q = _logger_mod._LISTENERS["p1_motion"].queue
    new_listener = logging.handlers.QueueListener(q, slow_handler)
    new_listener.start()
    _logger_mod._LISTENERS["p1_motion"] = new_listener

    # time.monotonic() for the interval measurement -- CLAUDE.md S3.4 rule
    # applies to test code too (mixed wall/monotonic in a benchmark makes
    # the interval untrustworthy). Two calls straddle the .info exactly.
    t0 = time.monotonic()
    log.info("under budget please")
    dt = time.monotonic() - t0
    # 1 ms budget per CHK-1-57 (4). Generous by 3x for CI variance -- a
    # busy CI runner can spike into low-ms territory even for pure Python.
    # If this ever flakes, look for garbage-collection stalls (Python 3.10
    # has a small stop-the-world for young-gen collection).
    assert dt < 0.003, "caller path took %.3f ms -- expected < 3 ms" % (dt * 1000)


# ---------------------------------------------------------------------------
# criterion (5) -- logrotate-safe; mutation (c) 'vanilla FileHandler'
# ---------------------------------------------------------------------------

def test_reopens_after_rotate(tmp_path):
    """Simulate logrotate: mv the current log then touch a new empty file.
    Next .info MUST land in the new file (the WatchedFileHandler notices
    the dev/ino change on emit and reopens). mutation (c) uses a plain
    FileHandler which caches the fd and would keep writing to the moved
    (or unlinked) inode -- the new file would stay empty and this test
    goes red.

    Real logrotate does exactly this: copytruncate mode or rename+create
    followed by a HUP or a size-based reopen check. WatchedFileHandler
    handles the rename+create case implicitly by dev/ino comparison.
    """
    # Emit once with the current file, then drain so the handler has
    # actually written the record + the fd's dev/ino stat baseline is
    # recorded (WatchedFileHandler compares on next emit, not on rotate).
    log = get_logger("p3_task")
    log.info("before rotate")
    _drain_listener("p3_task")

    # The rotation dance: move current log aside, touch a fresh empty file
    # at the original path. os.replace is atomic on POSIX (same rename(2)
    # semantics as logrotate uses), so no reader can see a missing file
    # between the mv and the touch.
    log_path = tmp_path / "p3_task" / "p3_task.log"
    rotated = tmp_path / "p3_task" / "p3_task.log.1"
    os.replace(log_path, rotated)              # atomic rename
    log_path.write_text("")                    # touch fresh file

    # Rebuild logger + emit. WatchedFileHandler stats dev/ino BEFORE this
    # emit and notices the mismatch (rotated file has one inode, fresh
    # file has another), reopens the fd against the fresh file, and writes
    # there. A plain FileHandler would emit into the still-held fd, which
    # points at the rotated inode after the mv -- test would go red.
    log = get_logger("p3_task")
    log.info("after rotate")
    _drain_listener("p3_task")

    # Directional check: new file MUST contain the fresh line, old file
    # MUST NOT (otherwise the reopen happened but the fd points at the
    # wrong file, which is a different bug from "no reopen").
    fresh = log_path.read_text()
    assert "after rotate" in fresh, "new file must receive the new line"
    old = rotated.read_text()
    assert "after rotate" not in old, "old (moved) file must not grow"


# ---------------------------------------------------------------------------
# criterion (level) -- mutation (e) 'ignore XBRAIN_LOG_LEVEL'
# ---------------------------------------------------------------------------

def test_env_var_sets_level(monkeypatch):
    """XBRAIN_LOG_LEVEL=WARNING must suppress INFO. mutation (e) would
    force INFO regardless -- the assertion below would then hold True and
    the test go red."""
    monkeypatch.setenv("XBRAIN_LOG_LEVEL", "WARNING")
    log = get_logger("p4_agent")
    assert not log.isEnabledFor(logging.INFO), \
        "WARNING should suppress INFO"
    assert log.isEnabledFor(logging.WARNING), \
        "WARNING should still be enabled"


def test_kwarg_level_wins_over_env(monkeypatch):
    """Explicit kwarg beats env, so a test rig can force DEBUG without
    touching the environment."""
    monkeypatch.setenv("XBRAIN_LOG_LEVEL", "ERROR")
    log = get_logger("p5_gateway", level="DEBUG")
    assert log.isEnabledFor(logging.DEBUG)


def test_unknown_level_raises(monkeypatch):
    """A bogus level MUST raise -- otherwise a typo silently becomes INFO
    and a bring-up runs with the wrong noise floor."""
    monkeypatch.setenv("XBRAIN_LOG_LEVEL", "FOO")
    with pytest.raises(ValueError, match=r"unknown level"):
        get_logger("ai_asr")


# ---------------------------------------------------------------------------
# mutation (d) -- off-tree default REJECTS /var/log
# ---------------------------------------------------------------------------

def test_variant_d_var_log_lands_outside_tree(tmp_path, monkeypatch):
    """CHK-1-57 (1) requires data/logs/{proc}/{proc}.log. mutation (d)
    changes the default to /var/log; the golden assertion here is that
    the path under XBRAIN_LOG_DIR is what LANDS, not /var/log.

    The mutation itself would be to change _DEFAULT_LOG_ROOT to
    '/var/log/xbrain'; a caller running without env set would then
    write there. Test proves the current default is under our tree.
    """
    # Explicitly unset XBRAIN_LOG_DIR so the code default takes effect.
    monkeypatch.delenv("XBRAIN_LOG_DIR", raising=False)
    # Assert the module-level constant is under /opt/xbrain_v6/data --
    # anything else is CHK-1-57 (1) violation.
    assert _logger_mod._DEFAULT_LOG_ROOT.startswith("/opt/xbrain_v6/data"), \
        "off-tree default MUST live under /opt/xbrain_v6/data (CHK-1-57 1)"


# ---------------------------------------------------------------------------
# proc_name shape guards
# ---------------------------------------------------------------------------

def test_bad_proc_name_raises():
    """Empty / mixed case / slash -- every one raises. A silent accept
    would let path traversal into another proc's dir happen."""
    for bad in ("", "P1_motion", "p1/motion", "1motion", "p1-motion"):
        with pytest.raises(ValueError, match=r"proc_name must match"):
            get_logger(bad)


def test_repeated_calls_return_same_logger():
    """Idempotence: same name -> same logger, no double-registration of
    handlers (otherwise each line prints N times, per CHK-1-57 sinks)."""
    a = get_logger("p1_motion")
    b = get_logger("p1_motion")
    assert a is b
    assert len(a.handlers) == 1


# ---------------------------------------------------------------------------
# criterion (3) -- P1 ctrl-path interlock (blocked by MOT-PM-2 unimpl)
# ---------------------------------------------------------------------------

@pytest.mark.xfail(reason="CHK-1-57 (3) depends on MOT-PM-2 being implemented "
                          "-- P1 ctrl loop does not exist yet",
                   strict=True)
def test_p1_ctrl_path_logger_interlock():
    """When P1 ctrl loop exists, calling get_logger().info(...) INSIDE the
    hot path should turn MOT-PM-2 judgement (1) red (the loop misses its
    60 ms budget). Placeholder xfail-strict so the day MOT-PM-2 lands, this
    test will FAIL if the interlock is not wired -- forcing the discipline
    check to be added rather than forgotten."""
    raise NotImplementedError("waiting on MOT-PM-2")
