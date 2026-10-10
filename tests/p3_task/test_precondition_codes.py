"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_precondition_codes.py
Brief: p3 admission codes are PC-n, never the V-n that 15 S5.2 owns

Description:
Until 2026-10-10 xbrain/p3_task/state/preconditions.py emitted codes
V-1..V-9, and they collided with 15 S5.2's V-1..V-9 while meaning different
things. Nine codes, two of which lined up:

  code    this module checks              15 S5.2's V-n of that number
  ------  -----------------------------   -----------------------------
  PC-1    task_type in the 7-value set    route/waypoint/dock exists
  PC-2    priority in [0, 100]            target inside the hard fence
  PC-3    energy-reachable                energy-reachable        <- same
  PC-4    no fence violation on the path   body capability
  PC-5    mission_json parses              allow_motion
  PC-6    total_steps > 0                  fix_type
  PC-7    not already terminal             ts_sync
  PC-8    route exists in the manifest     parameters legal (V-8a..g)
  PC-9    (not implemented)                refuse per source while link down

PC-4 is in fact 15's V-2 and PC-8 is 15's V-1 -- both checks present, both
filed under a number that means something else. The damage was to every
cross-reference: a reader finding "V-5" could not tell "allow_motion" from
"mission_json parses".

The rename closed the instances that existed. This file is what stops the
next one -- a new check added with a V-n code re-opens the collision, and
nothing else in the tree would notice.

*** THE FIRST VERSION OF THIS CRITERION WAS SELF-INJURING, and that is
worth more than the criterion. It grepped lines, and went red on
preconditions.py's own header -- which quotes the old codes while
EXPLAINING the collision. CLAUDE.md S3.2 form 3: the criterion itself contained
the string it greps for. It now reads STRING LITERALS through tokenize and
skips docstrings, so prose may discuss the old codes and only a live code
literal is a defect.

MUTANTS (run 2026-10-10, both red):
  * PC-1 reverted to V-1 in preconditions.py
  * PC-8 reverted to V-8

WHAT THIS DOES NOT COVER: the five 15 S5.2 checks that are not implemented
at all (V-4 body capability, V-5 allow_motion, V-6 fix_type, V-7 ts_sync,
V-9 link-down-by-source). Keeping the prefixes separate is what makes them
visibly missing rather than apparently done; closing them is EX-1 gap 2 and
is registered in docs/NEXT.md.
"""

# pytest for the marker; the imports the case needs are local to it so this
# module stays importable even if p3's layout moves.
import pytest

#: Reads committed source files. No device, no database.
pytestmark = pytest.mark.no_device


def test_p3_preconditions_use_pc_codes_not_bare_v_codes():
    """No PreconditionFailure in p3 may carry a bare V-n code.

    Until 2026-10-10 this module's nine codes were V-1..V-9 and collided
    with 15 S5.2's V-1..V-9 while meaning different things -- nine codes,
    two of which lined up. The damage was to every cross-reference: a
    reader finding "V-5" could not tell "health allows motion" (the
    contract) from "mission_json parses" (the code).

    Renaming fixed the instances that existed. This case is what stops the
    next one: a new check added with a V-n code would re-open the
    collision, and the only thing that notices is a criterion like this.

    Scope: the p3 precondition module and its direct callers. The V-
    prefix is used by OTHER number series elsewhere (p1's
    rotation/visual_override.py, common/errors/capability.py) and those
    are not this collision -- narrowing the scan surface to p3 is
    deliberate, and widening it would report two unrelated series as
    defects.

    *** That those two other series exist is worth knowing on its own: a
    bare "V-" prefix now carries at least three unrelated meanings in this
    codebase. This case does not try to settle that -- it closes the one
    collision EX-1 gap 4 names, which is the one where the code borrowed
    numbers a contract section already owned. Whether the other two should
    also be prefixed is a separate question about two modules this file
    has no business touching.
    """
    import io
    import os
    import tokenize
    root = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))))
    # The files that build or assert precondition codes. Listed rather than
    # globbed over p3: a glob would pull in every module that merely mentions
    # a code in prose, and those are exactly the ones allowed to.
    #
    # driver.py and main_wiring.py are here because they are the two call
    # sites -- a check renamed in preconditions.py but asserted by its old
    # code at a call site would leave the collision half-closed, which is
    # worse than not closing it (half the references resolve).
    watched = [
        # The module that mints the codes.
        "xbrain/p3_task/state/preconditions.py",
        # The two that consume them. A code asserted here by its old name
        # is as broken as one minted by it.
        "xbrain/p3_task/schedule/driver.py",
        "xbrain/p3_task/runtime/main_wiring.py",
    ]
    # *** The scan surface is STRING LITERALS ONLY, via tokenize -- not
    # the raw text. The first version of this case grepped lines and went
    # red on preconditions.py's own header, which quotes the old code
    # while EXPLAINING the collision. That is CLAUDE.md S3.2 form 3: the
    # criterion contained the string it greps for. Comments and
    # docstrings must be able to discuss the old codes; only a live code
    # literal is a defect.
    offenders = []
    for rel in watched:
        path = os.path.join(root, rel)
        if not os.path.exists(path):
            continue
        with io.open(path, 'rb') as handle:
            for tok in tokenize.tokenize(handle.readline):
                if tok.type != tokenize.STRING:
                    continue
                # A docstring is a STRING token too, so skip tokens whose
                # line begins the literal -- a live code is always an
                # argument or a comparand, never the whole statement.
                stripped = tok.line.strip()
                if stripped[:3] in ('\"\"\"', "'''") or stripped[:1] == '#':
                    continue
                # strip both quote styles, then measure: a code literal is
                # exactly three characters ("V-3"), so a longer string that
                # merely CONTAINS V-3 (a sentence, a URL) is not a hit.
                text = tok.string.strip('\"' + "'")
                if (len(text) == 3 and text[:2] == 'V-'
                        and text[2].isdigit()):
                    offenders.append('%s:%d' % (rel, tok.start[0]))
    # The message names the mapping's home rather than restating it: the
    # header in preconditions.py is where the nine-row table lives, and a
    # second copy in this message would be the duplicate that drifts.
    assert not offenders, (
        "bare V-n precondition codes are back: %s -- 15 S5.2 owns V-1..V-9 "
        "and this module's codes are PC-n (see preconditions.py's header "
        "for the mapping and for why a separate prefix)" % offenders)


def test_the_mapping_table_does_not_claim_a_check_that_does_not_exist():
    """Every PC-n the header's table describes must be a real function.

    *** This case exists because the table was wrong for one commit. Its
    first version listed "PC-4  no fence violation on the path" and thereby
    claimed 15 S5.2's V-2 was covered. There is no check_pc4 in the module;
    the row had been written from the renumbering notes instead of from the
    file. A mapping table that overstates coverage is worse than no table --
    it is what a reader consults INSTEAD of reading the code.

    The check runs the other way too: a function that exists but has no row
    is a check nobody can map back to the contract.

    How the table is read: lines of the form "  PC-n  <text>" inside the
    module docstring. Rows whose text begins with the em-dash marker are
    declared-absent and must NOT have a function.
    """
    # importlib rather than a module-level import: this case reads the
    # module's __doc__, and importing it at module scope would make the
    # whole file fail to collect if p3's layout moved -- hiding the OTHER
    # case in here, which does not need the module at all.
    import importlib
    import re

    module = importlib.import_module("xbrain.p3_task.state.preconditions")
    doc = module.__doc__ or ""

    # Rows look like "  PC-4    -- does not exist --   body capability..."
    # The first two columns are what matter; anything after is the contract
    # side and is prose.
    # setdefault, not assignment: if a number ever appears twice the FIRST
    # row wins, which keeps the failure about the duplicate rather than
    # about whichever copy happened to be last.
    rows = {}
    for line in doc.splitlines():
        m = re.match(r"\s+PC-(\d)\s{2,}(\S.*)$", line)
        if m:
            rows.setdefault(m.group(1), m.group(2).strip())
    # The emptiness guard is the one that keeps this case honest: if the
    # table's shape changes, the regex matches nothing, both differences
    # come out empty, and the case passes while checking nothing at all
    # (CLAUDE.md S3.2 form 1).
    assert rows, (
        "no PC-n rows found in the module docstring -- either the table was "
        "removed or its shape changed, and this case has been checking "
        "nothing since")

    # Two sets, then two directed differences. A single symmetric check
    # would report both kinds of mismatch as one finding, and they have
    # opposite fixes: over-claiming means the table lies about coverage,
    # under-claiming means a real check is unmapped.
    #
    # Which numbers the table says do not exist.
    absent_claimed = {n for n, text in rows.items()
                      if text.startswith("-- does not exist")}
    # Which numbers actually have a function.
    present_actual = {n for n in rows
                      if any(name.startswith("check_pc%s_" % n)
                             for name in dir(module))}

    # A row claiming a check that is not there.
    overclaimed = sorted((set(rows) - absent_claimed) - present_actual)
    assert not overclaimed, (
        "the table describes PC-%s as implemented, but no check_pc%s_* "
        "function exists -- a table that overstates coverage is what a "
        "reader consults instead of the code"
        % ("/".join(overclaimed), "/".join(overclaimed)))

    # A row claiming absence while the function is there.
    understated = sorted(absent_claimed & present_actual)
    assert not understated, (
        "the table says PC-%s does not exist, but a check_pc%s_* function "
        "is defined" % ("/".join(understated), "/".join(understated)))
