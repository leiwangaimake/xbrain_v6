"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_total_steps_table.py
Brief: 15 S9.5A.2 total_steps per task type, and the CHECK that was restored

Description:
Until 2026-10-10 every admitted task row was built with total_steps=0 and the
comment "expanded later by the route layer", and schema_task.py had relaxed
the contract's CHECK (total_steps >= 1) to >= 0 so those rows could land.

The route layer was never going to fill it. 15 S9.5A.2 gives total_steps as a
CONSTANT PER TYPE and makes the point its own core, verbatim: a step is
"一个可独立失败, 失败后可单独重试或跳过的执行阶段", and "它不是 waypoint --
巡逻的进度在 patrol_progress 里, 不在 current_step 里". The reason is given
too: counting waypoints would make total_steps move whenever a route is
re-recorded, breaking CHECK (current_step <= total_steps) on existing rows.

These cases pin the table, the refusal on an unknown type, and the restored
CHECK. They exist because the relaxed CHECK had been in place for five weeks
with 142 of 144 rows holding 0 -- a value that satisfies nothing and that
ST-2 forbids ("0 会让'进度 = current/total'除零").

MUTANTS (run 2026-10-10, all five red):
  * _total_steps_for defaults to 1 on an unknown type -> the refusal case
  * charge changed from 3 to 1                        -> the table case
  * the CHECK relaxed back to >= 0                    -> the schema case
  * task_row writes a hard 1 instead of calling the table
                                                      -> the wiring case

*** THAT LAST MUTANT SURVIVED THE FIRST ROUND, and the gap is worth naming:
every case here exercised _total_steps_for directly, so replacing the call
site with a literal 1 changed nothing any of them could see. Six of the
seven types happen to BE 1, which is why a hard-coded 1 looks right. The
wiring case drives task_row_from_command on a `charge` command -- the type
whose value is 3 -- which is the only kind of input that can tell a lookup
from a constant.

WHAT THESE CASES ALSO DO NOT COVER: whether the per-type values are the
RIGHT design. They come from 15 S9.5A.2's table and this file holds the
implementation to it; if the volume re-tunes a count, the transcription
below has to be updated with it, and that is a deliberate two-step --
a criterion that read the contract at runtime would track an edit nobody
reviewed.

Nor do they cover the percent computed from current_step: 15 S9.5A.2 ST-3
says that quantity should come from distance, not steps, and whether the
task card's percent is the same quantity is unsettled (docs/NEXT.md TS-2).

WHAT THESE CASES DO NOT COVER: composite tasks. 15 S9.5A.2 gives them "子任务
条数", which is not known when one submitted row is built, and ST-1 says
total_steps is fixed at insert. Guessing a count there would be worse than
not handling it, so _total_steps_for does not -- registered in docs/NEXT.md.
"""

# re to read the CHECK out of the DDL text; dataclasses.replace to stamp a
# task_id onto a parsed command the way the ingest path does.
import re
from dataclasses import replace

import pytest

from xbrain.p3_task.ingest.task_row import (
    _TOTAL_STEPS_BY_TYPE,
    _total_steps_for,
    task_row_from_command,
)
from xbrain.p3_task.persistence.schema_task import (
    DDL_TASKS,
    TASK_TYPES,
)

#: Pure table lookups, a text read, and one in-process call through the
#: ingest path. No device, no database, no clock.
pytestmark = pytest.mark.no_device

#: The seven single types and their step counts.
#:
#: 15 S9.5A.2's table, transcribed here so the criterion states the contract
#: rather than echoing the implementation. A test that imported only
#: _TOTAL_STEPS_BY_TYPE and asserted things about it would agree with any
#: value the module happened to hold -- CLAUDE.md S3.2 form 7, defining the
#: conclusion into the premise.
_CONTRACT_TABLE = {
    "patrol": 1,
    "goto": 1,
    "charge": 3,
    "return_home": 1,
    "standby": 1,
    "teach": 3,
    "follow": 1,
}


def test_the_table_matches_15_s9_5a_2_exactly():
    """Every type, both directions, against the transcribed contract table.

    Equality on the whole dict, not key-by-key: a type present in one and
    absent from the other has to fail here, and a per-key loop over one
    side's keys cannot see the other side's extras.
    """
    assert _TOTAL_STEPS_BY_TYPE == _CONTRACT_TABLE, (
        "implementation table and 15 S9.5A.2 disagree -- a wrong step count "
        "lets current_step run past it or stops a real step from counting")
    # Both sides are also checked for the floor ST-2 sets, so a future edit
    # cannot put a 0 back in either table and have them still agree.
    assert all(v >= 1 for v in _CONTRACT_TABLE.values())
    assert all(v >= 1 for v in _TOTAL_STEPS_BY_TYPE.values())


def test_the_table_covers_the_whole_task_type_closed_set():
    """Both directions against TASK_TYPES.

    A type in the closed set but missing from the table would raise at
    admission; a type in the table but not in the closed set is dead weight
    that outlives whatever removed it. Neither is visible from the other
    direction alone.
    """
    # TASK_TYPES comes from schema_task.py, where the DDL's CHECK on
    # task_type is built from the same frozenset -- so this is a check
    # against the closed set the database enforces, not against a second
    # list of type names kept by hand.
    # Reported as two directed differences rather than one symmetric set:
    # the two have different fixes (add a table row vs remove a stale one)
    # and a combined set does not say which side is which.
    assert set(_TOTAL_STEPS_BY_TYPE) == set(TASK_TYPES), (
        "TASK_TYPES and the total_steps table have diverged: only-in-table=%s "
        "only-in-closed-set=%s"
        % (sorted(set(_TOTAL_STEPS_BY_TYPE) - set(TASK_TYPES)),
           sorted(set(TASK_TYPES) - set(_TOTAL_STEPS_BY_TYPE))))


@pytest.mark.parametrize("task_type,want", sorted(_CONTRACT_TABLE.items()))
def test_lookup_returns_the_contract_value(task_type, want):
    """Per type, through the function the ingest path actually calls.

    Parametrized off _CONTRACT_TABLE (the transcription) and not off
    _TOTAL_STEPS_BY_TYPE (the implementation): parametrizing off the
    implementation would generate exactly the cases that pass, however
    wrong the values were. The equality case above already compares the
    two dicts; this one exercises the FUNCTION, which can diverge from
    the dict it is supposed to read.
    """
    assert _total_steps_for(task_type) == want


def test_an_unknown_type_raises_rather_than_defaulting():
    """No default. A default of 1 would invent a step count.

    The superseded comment in schema_task.py made this point itself -- "NO
    不把 total_steps 默认成 1 来'满足'约束: 那是编一个步数, 而下游的
    current_step BETWEEN 0 AND total_steps 会据此放行一个不存在的进度" --
    and it was right. The caller validates task_type against TASK_TYPES
    before this runs, so reaching here means the closed set and the table
    diverged, which is what should be loud.
    """
    # pytest.raises(Exception) and not the concrete type: the point of this
    # case is that an unknown type does NOT return a number, and pinning the
    # exception class here would make it fail on a legitimate change of
    # error type while saying nothing about the behaviour under test. The
    # message assertions below are what make it specific.
    with pytest.raises(Exception) as ei:
        _total_steps_for("not_a_task_type")
    # The message must name both halves an operator has to reconcile.
    assert "TASK_TYPES" in str(ei.value)
    assert "_TOTAL_STEPS_BY_TYPE" in str(ei.value)


def test_the_ddl_check_is_back_to_one():
    """schema_task.py's CHECK (total_steps >= 1), matching 15 S9.5 DDL.

    Read out of the DDL text rather than exercised against sqlite: a case
    that inserted a 0 and expected a refusal would also pass while the
    column was simply absent. The text says which bound is written.

    ST-2 is the reason the bound is 1: "total_steps 的最小值是 1, 不是 0 --
    0 会让"进度 = current/total"除零".
    """
    # Whitespace-tolerant so a reformat of the DDL does not read as a
    # relaxation; the captured group is the only part that matters.
    m = re.search(r"total_steps\s+INTEGER\s+NOT NULL\s+CHECK\s*\("
                  r"\s*total_steps\s*>=\s*(\d+)\s*\)", DDL_TASKS)
    # Asserting the match itself: a renamed column or a dropped CHECK would
    # otherwise make the group lookup raise, and a TypeError does not tell
    # the reader which of the two happened.
    assert m, "could not find the total_steps CHECK in DDL_TASKS"
    # The message says what to do if this was relaxed deliberately, not
    # just that it is wrong: the previous relaxation DID carry a note, and
    # the note was the only reason the five-week-old divergence was
    # findable at all. What it lacked was a correct reason.
    assert m.group(1) == "1", (
        "CHECK is >= %s; 15 S9.5 writes >= 1 and ST-2 gives the reason. If "
        "this was relaxed again, the relaxation needs a dated note saying "
        "what it is waiting for -- the last one waited for something that "
        "did not exist" % m.group(1))


#: ---------------------------------------------------------------------
#: The wiring half: does the ingest path actually consult the table?
#: ---------------------------------------------------------------------


def _charge_command():
    """A parsed cmd/task submit for a charge task, through the real parser.

    parse_task_command rather than a hand-built DTO: the payload shape
    (11 S7.2) is validated on the way in, so if that shape changes this
    case fails loudly instead of quietly testing a struct nobody builds
    any more. The payload is the minimum the parser accepts for a submit.
    """
    # Local import: the rest of this file needs neither the parser nor its
    # error type, and a module-level import would make a table test depend
    # on the ingest pipeline.
    from xbrain.p3_task.ingest.task_command import parse_task_command
    return parse_task_command({
        "cmd_id": "c-charge-1",
        "action": "submit",
        "source": "cloud",
        "task": {"type": "charge", "params": {}},
    })


def test_the_ingest_path_fills_total_steps_from_the_table():
    """task_row_from_command on a `charge` command yields total_steps=3.

    The wiring criterion, and `charge` is the only reasonable choice of
    type for it: six of the seven types have total_steps 1, so a hard-coded
    1 at the call site would satisfy a case built on any of them. charge is
    3, so this case separates "looked it up" from "wrote a constant".
    """
    # replace(..., task_id=...) mirrors what task_apply._submit does: a
    # cloud submit may omit task_id, P3 mints it inside the transaction and
    # stamps it on before building the row. Passing a command with an empty
    # task_id would exercise a path the ingest layer never sees.
    row = task_row_from_command(
        replace(_charge_command(), task_id="t-charge"),
        submit_seq=1, now_mono_ms=1000)
    assert row.total_steps == 3, (
        "the ingest path is not consulting 15 S9.5A.2's table -- a charge "
        "task has three steps (to_handover / waiting_plug / charging)")
    # current_step starts at 0 and the DDL's CHECK is
    # current_step BETWEEN 0 AND total_steps, so this pair must be valid.
    # current_step starts at 0, and the pair is checked against the DDL's
    # own invariant rather than just the literal: if total_steps ever came
    # back as something smaller than current_step, the row would be
    # rejected at insert and the failure would surface far from here.
    assert row.current_step == 0
    assert 0 <= row.current_step <= row.total_steps
