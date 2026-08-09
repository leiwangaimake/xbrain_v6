"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: cls_permissive.py
Brief: MOT-CM-1 -- PRC-69/PRC-70 permissive cls parser: off-set -> unknown

Description:
Every other closed set in xbrain/common/enums raises ClosedSetViolation
on an off-set value. The `cls` set (perception target classes) has a
DELIBERATELY DIFFERENT rule per 19 PRC-70 (perception design):

  "model output class off the closed set -> map to 'unknown',
   dedup-count by name, emit info event; must NOT silently drop
   the target; must NOT pass an off-set class name through"

The reason (19 verbatim, paraphrased): dropping the target would
remove a physically-existing detection from targets[], which
g(targets) and suspicion judgment both consume, biasing the
system 'faster'.

So `cls` needs a SECOND entry point separate from parse_enum:

  parse_enum("cls", v)        -- strict, raises on off-set (PRC-2/PRC-69
                                 contract for anyone reading serialised
                                 cls, e.g. a decoder in state/targets)
  parse_cls_permissive(v)     -- lenient: returns "unknown" for off-set
                                 (perception's model output boundary
                                 only, where PRC-70's leniency applies)

The two functions are DIFFERENT because they defend different
directions:

  * `parse_enum` guards the RECEIVED side: a peer sending garbage
    means the contract is being violated somewhere -- we must not
    silently interpret it, per 11 S13.6.
  * `parse_cls_permissive` guards the DETECTION-BOUNDARY side: the
    model is our own code producing raw output; garbage classes are
    a MODEL-KNOWLEDGE gap, not a contract violation. The right
    response is coverage (surface it as unknown + event), not drop.

MOT-CM-1 mutation test verbatim: "copy cls handling to limiter -> red".
i.e. applying parse_cls_permissive semantics to gate_limiter (which
IS a contract closed set) must fail a test. That's why this is a
CLS-ONLY helper, NOT a generic 'permissive parser' knob.

Contract:
  parse_cls_permissive(v)
      -> str    the input v if v in CLS, else "unknown"
      -> emits  info event via emit_event callback when v is off-set
                (dedup by name so the same off-set class does not
                spam the log)

Callers:
  Only perception's model-output boundary. Everywhere else uses
  parse_enum("cls", v) or CLS.parse(v) (strict).
"""

# threading for the dedup set + event counter guard (perception may
# call from multiple threads).
import threading
# typing for optional callback signature.
from typing import Callable, Optional

# CLS closed set + strict entry point.
from xbrain.common.enums import CLS


# Module-level dedup set: an off-set class name we already emitted
# an event for is not re-emitted. Threading-guarded because
# perception's callback threads may race. Keyed by lowercased name
# so 'Cat' and 'cat' are the same misspelling.
_seen_off_set = set()
_lock = threading.Lock()
_off_set_count = 0


# Optional event emitter. Injected by perception at wire-up; None
# means 'log via print' fallback for tests / early bring-up.
_emit_event: Optional[Callable[[str, str, dict], None]] = None


def set_event_emitter(emit: Optional[Callable[[str, str, dict], None]]) -> None:
    """Register the event emitter that off-set cls names should be
    reported through. emit is called as
    emit("info", "perception.cls_off_set", {"name": <str>}).

    Passing None resets to print-fallback (used in tests). Not
    thread-safe on registration -- registration should happen once
    at wire-up.
    """
    # Module-level assignment; single-writer at wire-up so no lock.
    global _emit_event
    _emit_event = emit


def parse_cls_permissive(value: str) -> str:
    """PRC-70 permissive cls parse: off-set -> 'unknown' + info event.

    Returns 'unknown' for any value not in CLS. Emits an info event
    exactly ONCE per distinct off-set name (dedup by lowercased
    name). Never raises for a str input; raises TypeError on non-str
    (a non-string cls is a decoder bug, not a class value).
    """
    # Type gate: cls MUST be a string. A non-str means the decoder
    # is passing something wrong upstream -- surface it as a
    # TypeError so the caller stack points at the real problem, not
    # the enum layer.
    if not isinstance(value, str):
        raise TypeError(
            "cls must be str, got %s" % type(value).__name__)
    # In the closed set: pass through unchanged. Fast path.
    if value in CLS.values:
        return value
    # Off-set: normalize + dedup, then emit info event and return
    # 'unknown'. Lowercased for the dedup key so 'Cat' and 'cat'
    # (both off-set) count as one event, not two.
    normalized = value.lower()
    should_emit = False
    with _lock:
        # Track the count regardless (metrics may want the total,
        # not just the distinct set).
        global _off_set_count
        _off_set_count += 1
        # Emit only on the first sighting per distinct name so a
        # bad model class name does not spam.
        if normalized not in _seen_off_set:
            _seen_off_set.add(normalized)
            should_emit = True
    if should_emit:
        # emit is called OUTSIDE the lock to avoid holding it across
        # a potentially blocking downstream (Zenoh publish, etc).
        _report(value)
    return "unknown"


def _report(offending: str) -> None:
    """Emit the info event for a first-sight off-set cls name."""
    # emit signature: (severity, kind, detail) matching event bus.
    # Fallback: print in a fixed form so tests can grep for it
    # without a subscriber wire-up.
    if _emit_event is not None:
        _emit_event("info", "perception.cls_off_set", {"name": offending})
    else:
        print("[cls_permissive] off-set cls %r mapped to 'unknown'"
              % offending)


def off_set_count() -> int:
    """Return the running total of off-set cls hits (all, not
    distinct). Used by perception metrics + tests."""
    with _lock:
        return _off_set_count


def seen_off_set_names() -> frozenset:
    """Return the distinct off-set names seen so far. For tests +
    diagnostics; a snapshot, not a live view."""
    with _lock:
        return frozenset(_seen_off_set)


def _reset_for_tests() -> None:
    """Clear the module state. TEST HELPER ONLY -- do not call from
    production paths (a production reset would silently allow the
    same off-set name to re-spam events)."""
    global _off_set_count
    with _lock:
        _seen_off_set.clear()
        _off_set_count = 0
