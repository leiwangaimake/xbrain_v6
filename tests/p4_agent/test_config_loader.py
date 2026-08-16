"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_config_loader.py
Brief: GWY-P4-20 -- P4 reads the product not the source, and refuses on holes

Description:
The criterion for this item has six clauses and they are not all the same kind
of thing. Three are static properties of the tree (no source-path literal in
runtime code, no defaulting construct on a safety key, no singular relative
configuration root), two are behaviours of the loader (min_agent_version, and
the refusal while the configuration is still a skeleton), and one -- clause 3 --
turns out not to be achievable at all. Each is handled here in the form it
actually has, and the one that is not achievable is recorded as such rather than
approximated by something adjacent.

*** On clause 3, which is the finding rather than the failure. The criterion
asks for a mutation where an uncalibrated value is changed from null to 0.0 and
startup assertion A then prints the key path. Assertion A cannot do that. Its
scope is verbatim, from the 10 S5.4.4 table row: 解析后全树扫描: 残留 ${,
未解析路径, null 未赋值 -- it inspects values for null, and 0.0 is an assigned
value. 10 S5.4.5 says so in as many words about this exact case:
0.0 会被断言 A 判为已赋值而放行. The door that does catch it is assertion G,
whose evaluation list (11 S9.6) includes SP-11 for this file's
gpu_token.throttle_speed_mps -- and SP-11's own row states it is blocked behind
SP-1 because common.spec.max_vx_mps is not calibrated. Beyond that, no executor
for either assertion exists yet: both belong to xbrain-config-freeze.service,
which is a later item. So the mutation as named is not runnable by anybody
today.

What is here instead is two things, and the distinction matters. First, an
executable record of the hole, so a later reader cannot mistake a green suite
for coverage of it. Second, a check that does catch the same DEFECT by a
different route: the meta-test over the configuration source refuses to let any
registered placeholder acquire a value at all, whether that value is 0.0 or 1.0.
The failure mode is covered; the mechanism the criterion names is not the one
covering it, and saying otherwise would be the exact substitution this document
package keeps catching.

Every other case names the mutation that turns it red, and each of those was
actually run against the real modules rather than only against a planted file.
CLAUDE.md 3.3: an assertion that has never been red has not been written.

*** Scan surfaces are declared, never implied. Three clauses are "zero hits"
claims, and a zero from a tree that was never walked is indistinguishable from a
zero from a clean one (CLAUDE.md 3.2 form 6). Each static test states its
surface, asserts the surface is non-empty, and carries a positive control -- a
planted file that MUST be reported -- so a checker that reports nothing at all
cannot pass by being broken. Two of them carry a negative control as well,
because an over-broad checker is switched off just as fast as a silent one.

Where each clause is discharged
-------------------------------
Written out because the clauses are numbered in the item and named nowhere in
the code, and a reader asked to check this work should not have to reconstruct
the mapping from test names.

  clause 1, no source read
      static half  test_no_runtime_module_names_the_configuration_source
                   plus its positive control, and the repository-wide
                   scripts/lint/no_config_source_read.py which already covers
                   xbrain/ including this package
      run-time half test_a_manifest_pointing_at_the_source_is_refused, which
                   drives the shared reader's confinement check through the P4
                   entry point rather than trusting that P4 inherited it

  clause 2, no default on a safety key
      test_no_p4_module_defaults_a_safety_value, with four planted forms and
      one negative control. Surface xbrain/p4_agent/, declared

  clause 3, null changed to 0.0 caught by assertion A
      NOT ACHIEVABLE, see above. Recorded by
      test_a_zero_in_place_of_a_null_is_NOT_caught_here_and_that_is_the_gap.
      The same defect is caught by a different route in
      test_every_undecided_key_is_still_null_in_the_source

  clause 4, min_agent_version
      seven cases from test_a_cmdset_needing_a_newer_agent_refuses_startup
      through test_the_agent_version_has_no_default_anywhere, covering the
      refusal, the boundary, the ordering, malformed input and the absence of
      any default for the implementation's own version

  clause 5, no singular relative configuration root
      test_nothing_in_this_item_uses_the_singular_configuration_root, with the
      surface narrowed and the reason for narrowing stated in the case itself

  clause 6, the skeleton refuses to start
      configuration side  test_every_undecided_key_is_still_null_in_the_source
                          and test_no_null_in_the_source_is_unregistered, a
                          two-way check so neither the file nor the registry can
                          drift alone
      loader side         test_the_skeleton_configuration_refuses_to_start

Beyond the six clauses this suite also covers the closed-set check on
prompt.history.enable_on, which 16 S14 states verbatim as a startup refusal and
which nothing else in the system would perform.
"""

import ast
import copy
import inspect
import json
import hashlib
import os
import re
import sys

import pytest
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from xbrain.common.config.resolved import ResolvedConfigError  # noqa: E402
from xbrain.p4_agent.config import (  # noqa: E402
    HISTORY_SCENARIOS,
    P4ConfigError,
    check_history_scenarios,
    check_min_agent_version,
    check_no_unassigned_keys,
    load_p4_config,
    parse_version,
    satisfies,
    startup_report,
)

#: The configuration source for this process, assembled from path components.
#:
#: Assembled rather than written as one string so this file contains no
#: contiguous copy of the literal it searches for. That is not superstition: the
#: static test below scans for exactly that literal, and a criterion whose own
#: text sits inside the surface it scans can never reach zero -- CLAUDE.md 3.2
#: form 3, which this project has caught three times, and whose usual repair is
#: to loosen the criterion until it passes.
#:
#: The surface here happens to exclude tests/, so this is belt and braces. It
#: costs one line and it survives somebody widening the surface later.
SOURCE_YAML = os.path.join(ROOT, "configs", "p4_agent.yaml")

#: A full boot id, the shape /proc/sys/kernel/random/boot_id returns.
#:
#: Written out rather than read from the machine so the suite behaves the same
#: on a build agent as on the robot, and so a failure here always means the
#: reader changed rather than that the host rebooted mid-run.
#:
#: The full 36-character UUID, never a prefix. 11 S3.0 defines a DIFFERENT
#: boot-derived field -- the message envelope's `boot`, the first eight hex
#: digits -- and confusing the two is the specific mistake the shared reader is
#: shaped to prevent. tests/common/test_resolved.py owns the cases for both
#: directions of that confusion; this suite only has to avoid reintroducing it
#: in a fixture.
BOOT_A = "9c26dc40-cfad-49b2-afd7-36dc01e1cb34"

#: *** Every key this configuration deliberately leaves null, with the decision
#: that owns it. This mapping is the executable form of clause 6.
#:
#: Where it comes from, and why it is NOT derived from the yaml file. Deriving
#: it would make the test tautological -- whatever is null would be declared
#: legitimately null, and by construction there could be no counter-example.
#: That is CLAUDE.md 3.2 form 7, a rule that defines its conclusion into its
#: premise. So the list is transcribed from the markers in 16 S14 and 16 S16,
#: and it is checked in BOTH directions against the file: a registered key that
#: got filled in is a violation, and a null that nobody registered is equally a
#: violation.
#:
#: The consequence, stated so it is not a surprise to whoever hits it: adding a
#: key whose value is still undecided means adding a row here naming the
#: decision that blocks it. That is the point of the mapping. A hole nobody
#: wrote down is exactly how a placeholder quietly becomes a number.
UNDECIDED_KEYS = {
    # 16 S14's note on this key calls 500 a placeholder awaiting measurement,
    # and in the same block forbids inventing a bound for it this round -- the
    # upper bound itself does not exist yet, so any assertion written now would
    # be a fabricated guarantee.
    "grammar.max_enum_items": "Q-P4-2 plus the T1 sampling-performance measurement",
    # 16 S16 Q-P4-3 calls the endpoint shown in S14 a placeholder and a
    # deployment-time value. The tension noted in the configuration file itself
    # -- that the same section's asr comment cites U52 as already allocating
    # 18082 to llama -- is recorded there and deliberately not resolved.
    "gateway.llm.endpoint": "Q-P4-3",
    # 16 S14 writes an empty string here pending model selection. An empty
    # string passes an "is it assigned" test exactly as 0.0 does, so the
    # configuration writes null instead and records the substitution.
    "gateway.asr.model": "D-AI-1 / D-AI-2 model selection",
    # The whole tts block. 16 S14 states that whether the section should exist
    # at all cannot be decided, and 16 S16 Q-P4-32 records that the correct
    # closure may be deleting it outright rather than filling it in. Each key is
    # listed separately rather than as a prefix, so that adding a key to the
    # block is a visible edit here too.
    "gateway.tts.base_url": "D-AI-3 plus the TTS-topology ruling (Q-P4-32)",
    "gateway.tts.path": "D-AI-3 plus the TTS-topology ruling (Q-P4-32)",
    "gateway.tts.model": "D-AI-3 plus the TTS-topology ruling (Q-P4-32)",
    "gateway.tts.voice": "D-AI-3 plus the TTS-topology ruling (Q-P4-32)",
    "gateway.tts.speed": "D-AI-3 plus the TTS-topology ruling (Q-P4-32)",
    "gateway.tts.response_format": "D-AI-3 plus the TTS-topology ruling (Q-P4-32)",
    "gateway.tts.stream": "D-AI-3 plus the TTS-topology ruling (Q-P4-32)",
    "gateway.tts.timeout_s": "D-AI-3 plus the TTS-topology ruling (Q-P4-32)",
    # 16 S14 calls 1.0 a placeholder and states outright that the value is a
    # tuning decision and is not to be invented. 11 S9.6 SP-11 bounds it from
    # above, and that bound is itself blocked until common.spec.max_vx_mps is
    # calibrated -- so there is nothing here to copy even indirectly.
    "gateway.gpu_token.throttle_speed_mps": "Q-P4-13 plus T1 calibration",
}

#: Values used to complete a snapshot in the cases that need one which loads.
#:
#: *** These are fixtures and they are not values. They are deliberately absurd
#: -- a speed nobody would ever calibrate to, a hostname reserved by RFC 2606 --
#: so that a copy of this mapping landing in the configuration would be obvious
#: on sight rather than plausible. A test fixture that looks like a real
#: calibration is how a placeholder gets promoted to a setting, and this
#: project has the same shape recorded against it more than once.
#:
#: The keys are exactly the keys of UNDECIDED_KEYS, and that is asserted
#: implicitly by set_dotted raising on a path it cannot find: a hole added there
#: and forgotten here makes every case using filled_tree() fail loudly rather
#: than quietly load a tree with a null still in it. The pairing is deliberate
#: -- one list of holes with two uses, not two lists that have to agree.
#:
#: 0.987654 recurs on purpose. It is the same non-value in three places, so a
#: reader who greps for it finds all of them at once and can see immediately
#: that no calibration is being asserted anywhere.
FIXTURE_FILL = {
    "grammar.max_enum_items": 7,
    "gateway.llm.endpoint": "http://not-a-real-host.invalid/v1/chat/completions",
    "gateway.asr.model": "fixture-not-a-model",
    "gateway.tts.base_url": "http://not-a-real-host.invalid",
    "gateway.tts.path": "/fixture",
    "gateway.tts.model": "fixture-not-a-model",
    "gateway.tts.voice": "fixture-not-a-voice",
    "gateway.tts.speed": 0.987654,
    "gateway.tts.response_format": "fixture",
    "gateway.tts.stream": False,
    "gateway.tts.timeout_s": 0.987654,
    "gateway.gpu_token.throttle_speed_mps": 0.987654,
}


# -- helpers -------------------------------------------------------------------

def source_tree():
    """The configuration source parsed fresh on every call.

    Not cached. Several cases below mutate the tree they are handed, and a
    module-level cache would let one case change what a later one sees --
    producing the class of failure where a suite passes and fails depending on
    which order the cases ran in. Parsing a fifteen-kilobyte yaml file forty
    times costs nothing measurable; a shared mutable fixture costs an afternoon
    the first time it bites.

    safe_load, never load. The file is checked into the repository and is not
    hostile, but the habit is the point: the same call in a process would be
    reading a file another process wrote, and there the full loader constructs
    arbitrary Python objects from tags in the document.

    A test may read the source; a process may not. scripts/lint/
    no_config_source_read.py states that in its scan surface, which covers
    xbrain/, ros2_ws/ and services/ and deliberately omits tests/ -- a fixture
    has to be able to name the file it is asserting about, and forcing it to go
    through the reader would mean the meta-tests could only see a snapshot
    somebody had already produced.
    """
    with open(SOURCE_YAML, encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def set_dotted(tree, dotted, value):
    """Assign one dotted path in a nested dict, raising if the path is absent.

    Raising rather than creating the intermediate levels. Every caller here is
    filling a key it believes already exists, so a helper that created the path
    would turn a typo in a test into a passing test -- which is the quiet way a
    suite stops testing the thing it is named after.
    """
    node = tree
    parts = dotted.split(".")
    # Walk to the parent of the leaf. No membership check on the way down: a
    # missing intermediate level raises KeyError from the dict itself, and that
    # message already names the level that was absent.
    for part in parts[:-1]:
        node = node[part]
    # The leaf is checked explicitly, because assigning into a dict creates the
    # key silently and that is precisely the failure described above.
    if parts[-1] not in node:
        raise KeyError(dotted)
    node[parts[-1]] = value


def flatten_leaves(tree, prefix=""):
    """Every leaf of a nested dict as (dotted path, value).

    Leaves only. A subtree is not yielded as a value of its own, which matches
    what ResolvedConfig.unassigned() reports, so the meta-tests here and the
    loader's null gate are talking about the same set of things.
    """
    for key, value in tree.items():
        dotted = prefix + "." + key if prefix else key
        if isinstance(value, dict):
            for pair in flatten_leaves(value, dotted):
                yield pair
        else:
            yield dotted, value


def build_root(tmp_path, tree, boot_id=BOOT_A):
    """Write a fake /run/xbrain/resolved tree and return (root, boot_id_path).

    Modelled on the builder in tests/common/test_resolved.py, so a reader who
    knows one knows the other.

    It writes a real MANIFEST with a real sha256 and a real boot id, which means
    every case here goes through the full gate rather than around it. A fixture
    that skipped the manifest would exercise a code path that does not exist on
    the machine, and the suite would then be green about something nobody ships.
    """
    # Named "resolved" under tmp_path rather than pointed at the real
    # /run/xbrain/resolved. The real directory is tmpfs owned by the freeze
    # service, and a test that wrote into it would both need privileges it
    # should not have and leave a snapshot behind for whatever started next.
    root = tmp_path / "resolved"
    root.mkdir(parents=True, exist_ok=True)
    # allow_unicode so the Chinese bypass keywords and history templates survive
    # the round trip as themselves rather than as escape sequences; sort_keys so
    # two runs over the same tree produce byte-identical content and therefore
    # the same digest.
    snapshot = yaml.safe_dump(tree, allow_unicode=True, sort_keys=True)
    (root / "p4_agent.yaml").write_text(snapshot, encoding="utf-8")
    manifest = {
        "schema": "xbrain.config.manifest/1",
        "boot_id": boot_id,
        "common_digest": "9f2c4a1b7e5d0836",
        "config_rev": "2026-08-06T00:00:00Z+gabcdef0",
        "processes": {
            "p4_agent": {
                "path": str(root / "p4_agent.yaml"),
                "sha256": hashlib.sha256(snapshot.encode("utf-8")).hexdigest(),
            },
        },
    }
    # The manifest is written AFTER the snapshot, so the digest recorded in it
    # is taken from bytes that already exist on disk. Writing it first and
    # hashing the string in memory would work here and would diverge the moment
    # a case wanted to modify the snapshot afterwards -- and one case does
    # exactly that, to prove the digest is checked at all.
    (root / "MANIFEST.json").write_text(json.dumps(manifest), encoding="utf-8")
    # The boot id file always holds BOOT_A. Cases that want the gate to fail
    # pass a different boot_id into the MANIFEST rather than changing this, so
    # the asymmetry is visible at the call site.
    boot_file = tmp_path / "boot_id"
    boot_file.write_text(BOOT_A + "\n", encoding="utf-8")
    return str(root), str(boot_file)


def filled_tree():
    """The real configuration with every registered hole filled by a fixture.

    Used by the cases that need a snapshot which loads. Without such cases the
    refusal tests would all be satisfied by a loader that refused
    unconditionally -- the do-nothing implementation of CLAUDE.md 3.2 form 1, in
    its purest form.

    deepcopy, so a case that mutates the result cannot reach back into the
    parsed source and change what a later case sees.
    """
    tree = copy.deepcopy(source_tree())
    for dotted, value in FIXTURE_FILL.items():
        set_dotted(tree, dotted, value)
    return tree


def python_files(*relative_dirs):
    """Every .py file under the given repository-relative directories.

    __pycache__ is pruned rather than filtered afterwards, so a stale .pyc
    directory cannot make the file count drift from the files actually read.
    That pruning is worth a line of its own here: a stale __pycache__ has
    already cost this project once, when a synced .py left an older .pyc in
    place and the process kept loading the old bytecode with no sign that
    anything was wrong.

    Only .py, which bounds what the static cases here can claim. A shell script
    or a unit file under the same directory would not be scanned, and neither
    exists in this package today -- but if one arrives, the claims in the cases
    below stop covering the whole package and the extension list has to grow
    with it.

    A generator rather than a list, so each caller decides whether to
    materialise. The callers here do materialise, precisely so they can assert
    the surface is non-empty before judging it.
    """
    for rel in relative_dirs:
        base = os.path.join(ROOT, rel)
        for dirpath, dirnames, filenames in os.walk(base):
            dirnames[:] = [d for d in dirnames if d != "__pycache__"]
            for name in filenames:
                if name.endswith(".py"):
                    yield os.path.join(dirpath, name)


# -- clause 6: the configuration itself ---------------------------------------

def test_every_undecided_key_is_still_null_in_the_source():
    """*** Clause 6. The case that stops a placeholder becoming a number.

    Each key here is one 16 marks as undecided, and each is null so that
    10 S5.4.4 assertion A refuses the stack and prints the key path. Filling one
    in to make something run is the single failure CLAUDE.md 3.1 is written
    against, and this is what notices.

    Mutation, run against the real configuration: set
    gateway.gpu_token.throttle_speed_mps to 1.0 -- the very number 16 S14 shows
    and calls a placeholder -- and this goes red naming that key. Run again with
    0.0, which is the criterion's own clause-3 mutation, and it goes red the
    same way. See the module docstring for why that is NOT assertion A.
    """
    # Flattened once and reused, so the file is parsed a single time and every
    # key below is judged against the same read. Re-reading per key would make
    # a concurrent edit produce a half-consistent verdict, which is a strange
    # thing to debug for no gain at all.
    leaves = dict(flatten_leaves(source_tree()))
    # Sorted so a run that fails on several keys always fails on the same one
    # first. Dictionary order is stable in this interpreter, but the registry
    # above is edited by hand and reordering it should not change which failure
    # a reader sees.
    for dotted, blocked_on in sorted(UNDECIDED_KEYS.items()):
        # Presence first. A key that vanished from the configuration entirely is
        # a different defect from a key that acquired a value, and it is the
        # more dangerous one: assertion A inspects values and has nothing to
        # report for a key nobody declared, which is why 10 S5.4.4 needed a
        # separate assertion M for exactly this.
        assert dotted in leaves, (
            "%s is registered as undecided but is not in the configuration at "
            "all; assertion A cannot report a key nobody declared" % dotted
        )
        # Then the value. The message names the decision that blocks it, so the
        # person who hit this test knows whether they are allowed to fill it in
        # yet -- without that, the fastest way past a red test is to type a
        # number.
        assert leaves[dotted] is None, (
            "%s must stay null until %s is decided; it now holds %r. An "
            "uncalibrated value is written null so assertion A names the key, "
            "while 0.0 or an empty string passes the is-it-assigned test and "
            "then takes effect silently (CLAUDE.md 3.1)"
            % (dotted, blocked_on, leaves[dotted])
        )


def test_no_null_in_the_source_is_unregistered():
    """The other direction, without which the registry above rots quietly.

    A one-directional test would pass while somebody added a new key with a null
    value and no record of what blocks it. An unexplained hole is precisely what
    gets filled in by the next person who wants the stack to boot, because
    nothing on the page tells them not to.

    Mutation, run: add `scratch: null` under session in the configuration and
    this goes red; register it above and it passes again.
    """
    # `v is None` and not `not v`, for the same reason the loader's gate is
    # written that way: 0, the empty string, the empty list and false are all
    # assigned values that appear in this file legitimately, and folding them in
    # here would produce a list of "unregistered holes" that is mostly correct
    # configuration -- which trains the reader to skim it.
    actual = {k for k, v in flatten_leaves(source_tree()) if v is None}
    # One direction only, because the other direction is the case above. Kept as
    # two cases rather than one combined difference so that a failure says which
    # of the two things went wrong: a placeholder was filled in, or a new hole
    # arrived unannounced. The fixes are different.
    unregistered = sorted(actual - set(UNDECIDED_KEYS))
    assert not unregistered, (
        "these keys are null but no decision is recorded as blocking them: %s. "
        "Either register the blocking question or give the key its value"
        % unregistered
    )


def test_the_source_declares_no_common_top_level_key():
    """L6 takes values, it does not define them.

    10 S5.4.2 R-6 and R-1 confine the common namespace to L1, and 10 S5.4.4
    assertion B refuses startup on a duplicate definition.

    The failure this prevents is the worst kind of configuration bug: two files
    both defining a safety limit, with whichever one the merge happens to visit
    second deciding the robot's speed. Nothing about that is visible in either
    file on its own.

    Mutation: add a `common:` block to the configuration and this goes red.
    """
    assert "common" not in source_tree(), (
        "an L6 file must not carry a `common` top-level key; shared values are "
        "referenced as ${common.x} and defined only in L1 (10 S5.4.2 R-6)"
    )


def test_the_source_covers_every_section_the_design_names():
    """The key skeleton is complete, not merely well-formed.

    16 S14 gives the target shape as twelve top-level sections. A configuration
    missing one is caught by no null check at all -- there is nothing there to
    be null -- and 10 S5.4.4 assertion M exists precisely because assertion A
    cannot see an absent key. This is the cheap local version of that assertion,
    scoped to the one file this item owns.

    Bidirectional again: an unexpected section is as much a defect as a missing
    one, because it means a key was invented outside the design.

    Mutation: delete the `confirm:` section and this goes red naming it.
    """
    # Transcribed from the yaml block in 16 S14, in the order that block writes
    # them. Written out rather than counted: a count would pass on twelve
    # sections with the wrong names, which is CLAUDE.md 3.7's point about a
    # number standing in for the thing it measured.
    # `timezone` is the 13th top-level key (16 S14 v1.2, 2026-08-16): the L6
    # reference to common.timezone that G24 query_time formats local time with.
    expected = {"asr_post", "bypass", "intent", "prompt", "grammar", "gateway",
                "auth", "session", "recording", "confirm", "chitchat", "cmdset",
                "timezone"}
    # Top level only. Going deeper would duplicate the design document key by
    # key here, and the copy would then be the thing that has to be kept in step
    # -- exactly the second-source-of-truth problem the whole configuration
    # design exists to avoid.
    actual = set(source_tree())
    assert actual == expected, (
        "top-level sections differ from the 16 S14 target shape -- missing %s, "
        "unexpected %s" % (sorted(expected - actual), sorted(actual - expected))
    )


def test_the_deleted_recording_keys_are_not_reintroduced():
    """16 S14 deletes four recording keys and states why: they belong to P3.

    The ruling is D-B4. Session duration, point thinning and geometry thresholds
    are P3's, and a second copy in P4 guarantees two things: the thresholds
    diverge, and two timers expire at different moments. Somebody restoring the
    keys for completeness would recreate exactly that, and the symptom would be
    a recording session that ends twice with different reasons.

    Mutation: add max_duration_s back under recording and this goes red.
    """
    recording = source_tree()["recording"]
    # All four names, not a sample. Each was deleted for its own reason and each
    # would come back on its own: a reader restoring "just the timeout" is the
    # realistic case, and a check covering only one of the four would let the
    # other three through while looking like it covered the ruling.
    for banned in ("max_duration_s", "point_min_dist_m", "point_min_angle_deg",
                   "fence_min_vertices"):
        assert banned not in recording, (
            "recording.%s was deleted by 16 S14 ruling D-B4 and belongs to P3" % banned
        )


# -- clause 6 continued: the loader refuses while the holes are open ----------

def test_the_skeleton_configuration_refuses_to_start(tmp_path):
    """*** Clause 6's behavioural half.

    The claim in every configuration file's header -- that the stack refuses to
    start before the values are filled in -- is worth exactly as much as the
    case that demonstrates it. This feeds the real configuration through the
    real loader and asserts the refusal.

    The message assertion is not decoration. A refusal that says only "the
    configuration is incomplete" leaves somebody grepping twelve sections by
    hand, and the practical result of that is a request for a switch to start
    anyway -- which is the switch CLAUDE.md 3.6 forbids outright.

    Note precisely what this establishes: the READER refuses. Refusing the whole
    stack is xbrain-config-freeze.service's job (10 S5.4.4, Stage 0c) and that
    executor is a separate item. This is the half that exists today, and it is
    the second door rather than the first.

    Mutation, run: delete the check_no_unassigned_keys call from load_p4_config
    and this goes red.
    """
    # The REAL configuration, not a stripped-down fixture. That is the whole
    # point of this case: a fixture written to be incomplete proves only that
    # the loader rejects incomplete fixtures, while this proves the file the
    # repository actually ships does not start a process.
    root, boot = build_root(tmp_path, source_tree())
    # P4ConfigError specifically, not any exception. A YAML error or a missing
    # manifest would also stop startup, and accepting either here would let this
    # case pass for a reason that has nothing to do with unfilled values.
    with pytest.raises(P4ConfigError) as exc:
        load_p4_config("1.0", root=root, boot_id_path=boot)
    message = str(exc.value)
    # Every hole named, not just the first. Filling configuration is done by a
    # person with the volumes open, and one key per boot cycle is how that turns
    # into a dozen restarts.
    for dotted in UNDECIDED_KEYS:
        assert dotted in message, "the refusal must name %s" % dotted
    # And it must point at assertion A, because reaching this code with a null
    # in the tree means the freeze line did not run or let it through -- the
    # reader is not the component at fault and the message should not read as
    # though it were.
    assert "assertion A" in message


def test_a_complete_configuration_loads(tmp_path):
    """The positive control for every refusal in this file.

    Without it a loader that raised unconditionally would satisfy every negative
    case here. That is not a hypothetical shape: 16's own PTZ boost assertions
    were all passed by a shell implementation with the state machine deleted,
    which is the example CLAUDE.md 3.3 is built on.
    """
    root, boot = build_root(tmp_path, filled_tree())
    # "1.0" is the version the shipped configuration asks for, so this is the
    # pairing the design document itself shows rather than an invented one.
    cfg = load_p4_config("1.0", root=root, boot_id_path=boot)
    assert cfg.proc == "p4_agent"
    # Two values from opposite ends of the file, so a loader that returned an
    # empty tree would not pass by accident. assert_intent_count in particular
    # is the intent closed-set count from 16 S0.5 CS-A2 (128 until GWY-P4-23
    # landed D17/D18/E09/E10 2026-08-07 -> 132; then +5 for 18-C G43-G47
    # 2026-08-16 -> 137), which is the kind of value a truncated read would
    # silently lose.
    assert cfg.require("cmdset.min_agent_version") == "1.0"
    assert cfg.require("auth.assert_intent_count") == 137


def test_zero_and_empty_string_are_assigned_values_not_holes(tmp_path):
    """auth.nonvoice_estop.teleop_rest_poll_s is 0 and that 0 is real.

    16 S14 states it verbatim: 0 = 关闭, and a value above zero switches on a
    low-frequency poll through P5. So this is the one place in the file where a
    zero is a decision rather than the fail-silent shape CLAUDE.md 3.1 forbids,
    and the difference is documented at the key itself.

    A null gate written as `if not value` would report it as unfilled, which
    would send somebody to invent a polling interval for a feature that is
    deliberately off -- and inventing an interval is how the P4 side ends up
    with a second timer for something P5 already owns.

    Mutation, run: change check_no_unassigned_keys to test falsiness instead of
    `is None` and this goes red.
    """
    tree = filled_tree()
    # Asserted on the fixture before it is even loaded, so that if somebody
    # later changes the configuration this case fails here -- pointing at the
    # configuration -- rather than three lines down pointing at the loader.
    assert tree["auth"]["nonvoice_estop"]["teleop_rest_poll_s"] == 0
    root, boot = build_root(tmp_path, tree)
    cfg = load_p4_config("1.0", root=root, boot_id_path=boot)
    assert cfg.require("auth.nonvoice_estop.teleop_rest_poll_s") == 0
    # And nothing at all is reported as a hole, so the falsy value did not
    # merely survive the gate by being listed and ignored.
    assert cfg.unassigned() == []


def test_an_empty_enable_on_list_is_legal(tmp_path):
    """16 S14: 空列表 = 全关. An empty list is a configured decision.

    Grouped with the zero case because it is the same mistake in a different
    type. A checker that treats falsy as missing rejects the documented way of
    turning the one-line history off, and the only way past it would be to
    enable a scenario nobody wanted.
    """
    tree = filled_tree()
    tree["prompt"]["history"]["enable_on"] = []
    root, boot = build_root(tmp_path, tree)
    cfg = load_p4_config("1.0", root=root, boot_id_path=boot)
    # require() rather than get(), so the empty list has to survive the accessor
    # that refuses null as well as the loader's checks. A require() that treated
    # falsy as absent would raise here, and that is the failure this pins.
    assert cfg.require("prompt.history.enable_on") == []


# -- clause 3: the mutation that is not achievable ----------------------------

def test_a_zero_in_place_of_a_null_is_NOT_caught_here_and_that_is_the_gap(tmp_path):
    """*** This case asserts a HOLE. Read the whole docstring before changing it.

    Clause 3 of the criterion asks for a mutation in which an uncalibrated value
    is changed from null to 0.0 and startup assertion A then reports the key
    path. Assertion A cannot do that, and neither can this loader.

    A's scope is verbatim 残留 ${, 未解析路径, null 未赋值 (10 S5.4.4) -- it
    looks at values for null. 0.0 is an assigned value, and 10 S5.4.5 spells out
    this exact case: 0.0 会被断言 A 判为已赋值而放行.

    The door that does catch it is assertion G, evaluating SP-11 from 11 S9.6:
    0 < gateway.gpu_token.throttle_speed_mps < common.spec.max_vx_mps, executed
    by xbrain-config-freeze.service. SP-11's own row records that it is blocked
    behind SP-1 until common.spec.max_vx_mps is calibrated, and the shared
    configuration file declares no such key today. Both assertions also lack an
    executor entirely at this point in the project.

    So the assertion below is that the value passes -- deliberately. It exists
    so a later reader cannot look at a green suite and conclude the case is
    covered. When the freeze-line item lands, this becomes the case that fails
    and gets rewritten, which is exactly what should happen.

    What DOES catch the same defect today is
    test_every_undecided_key_is_still_null_in_the_source, which refuses to let a
    registered placeholder acquire any value at all. Different mechanism, same
    failure mode. Recorded separately because substituting one for the other in
    a report is the misstatement this project keeps finding.
    """
    tree = filled_tree()
    # This key rather than any other, because it is the one 11 S9.6 SP-11 names
    # explicitly and the one whose fail-open history is written down: 16 S14
    # records that the previous value of 3.0 could never trigger once U54 fixed
    # the maximum speed at 2.0, so the throttle existed on paper and never once
    # in the field. 0.0 is the mirror image -- a threshold that triggers always.
    tree["gateway"]["gpu_token"]["throttle_speed_mps"] = 0.0
    root, boot = build_root(tmp_path, tree)
    cfg = load_p4_config("1.0", root=root, boot_id_path=boot)
    assert cfg.require("gateway.gpu_token.throttle_speed_mps") == 0.0, (
        "0.0 passes every check this reader has, and it would pass assertion A "
        "too; the catcher is assertion G / SP-11 at the freeze line"
    )
    # It does not even appear as a hole. That is the whole reason CLAUDE.md 3.1
    # forbids writing it rather than relying on something downstream to notice.
    assert cfg.unassigned() == []


# -- clause 4: min_agent_version ----------------------------------------------

def test_a_cmdset_needing_a_newer_agent_refuses_startup(tmp_path):
    """*** Clause 4, stated by 16 S12.4: 不满足则拒绝启动并明确报错.

    Why a refusal rather than a warning. 16 S12.4's increment rules make a major
    bump mean semantics changed or an intent was deleted. A cmdset above the
    running implementation therefore holds commands whose MEANING moved -- not
    commands P4 has never heard of, which it would simply fail to match. So
    continuing means executing the wrong action for a phrase the operator has
    used a hundred times and trusts.

    Mutation, run: delete the check_min_agent_version call from load_p4_config
    and this goes red.
    """
    tree = filled_tree()
    # A major above the agent, not a patch. 16 S12.4 makes a major bump mean
    # semantics changed or an intent was deleted, so this is the shape where
    # continuing is actively dangerous rather than merely incomplete -- and it
    # is the shape a real release produces, since 18 S16.5 concludes this round
    # is a single major increment.
    tree["cmdset"]["min_agent_version"] = "2.0"
    root, boot = build_root(tmp_path, tree)
    # The agent claims 1.4, which is deliberately not 1.0: a check comparing
    # only the major component would pass on 1.0 against 2.0 by accident of
    # string equality somewhere, and a check comparing only the first character
    # would pass on almost anything. An off-round number closes both.
    with pytest.raises(P4ConfigError) as exc:
        load_p4_config("1.4", root=root, boot_id_path=boot)
    message = str(exc.value)
    # Both versions in the message. Rolling the cmdset back and rolling the
    # implementation forward are different actions taken by different people,
    # and "version mismatch" alone does not say which is which.
    assert "2.0" in message and "1.4" in message
    # And the key path, so the reader knows which file to open.
    assert "cmdset.min_agent_version" in message


def test_an_equal_version_satisfies_the_minimum(tmp_path):
    """The boundary is the normal case here, not an edge.

    16 S14 pairs min_agent_version "1.0" with the implementation meant to run
    it, so an implementation reporting exactly the minimum must start. A check
    written with > instead of >= would refuse the documented pairing, and the
    repair somebody reaches for when the check refuses everything is to stop
    treating a mismatch as fatal.

    Mutation, run: change >= to > in version.satisfies and this goes red.
    """
    root, boot = build_root(tmp_path, filled_tree())
    assert load_p4_config("1.0", root=root, boot_id_path=boot) is not None


def test_the_two_component_minimum_accepts_a_three_component_agent():
    """*** The comparison that plain tuple ordering gets wrong.

    16 S14 writes min_agent_version as "1.0" with two components, while an
    implementation may reasonably call itself "1.0.0". Under plain tuple
    comparison (1, 0) < (1, 0, 0), so an agent at exactly 1.0.0 would FAIL a
    minimum of 1.0 -- the check would reject the one pairing the document
    actually shows.

    Both directions are asserted, because zero-extension has to work whichever
    side is shorter, and the negative case is there so the padding cannot be
    implemented as "always return True".

    Mutation, run: remove the zero-extension from version.satisfies and this
    goes red.
    """
    # Three components against two: the pairing 16 S14 actually ships.
    assert satisfies("1.0.0", "1.0")
    # Two against three: the same padding, applied to the other operand. An
    # implementation that padded only the agent side would pass the line above
    # and fail here, and the two are one line apart so that cannot be missed.
    assert satisfies("1.0", "1.0.0")
    # And a genuine shortfall still fails, so the padding cannot have been
    # implemented as "compare equal whenever the lengths differ" -- which would
    # pass both lines above while accepting every version in existence.
    assert not satisfies("0.9.9", "1.0")


def test_versions_are_ordered_numerically_not_lexically():
    """1.10 is above 1.9. String and float comparison each get this wrong.

    A float parse maps "1.10" to 1.1 and orders it below 1.9; a string compare
    orders "1.10" below "1.9" character by character. Either would let an agent
    nine minor versions behind pass a minimum it does not meet, and the symptom
    would be commands routed to a handler whose meaning has since changed.
    """
    # 1.10 is ten minor versions on from 1.0, so it clears a minimum of 1.9.
    assert satisfies("1.10", "1.9")
    # And the reverse must fail. Without this line a parser that mapped every
    # version to the same number would pass the line above.
    assert not satisfies("1.9", "1.10")
    # Asserted on the parse as well as on the comparison, so a failure says
    # which of the two is wrong: the tuple is what the ordering is built on, and
    # (1, 1) here would mean the string had been through float().
    assert parse_version("1.10") == (1, 10)


@pytest.mark.parametrize("bad", ["", " 1.0", "1.0 ", "v1.0", "1.0-rc1", "1..0",
                                 "1.0.0.1", "one.zero", "1.0\n"])
def test_a_malformed_version_raises_rather_than_ordering_somehow(bad):
    """Refused, never treated as zero and never treated as satisfied.

    Both fallbacks fail the same way. A malformed value ordered as zero passes
    any minimum of 0; a malformed value treated as satisfied passes every
    minimum. Either turns a configuration defect into an intent reaching the
    wrong handler at run time, which is the failure the version check exists to
    prevent.

    The list covers each shape this project could plausibly meet: an empty
    value, whitespace from a hand edit, a leading v from semver habit, a
    pre-release suffix whose ordering no volume defines, a doubled dot, a fourth
    component, words, and a trailing newline from a file read without stripping.
    """
    # ValueError, from the version module, and not the configuration error type.
    # The split is deliberate: version.py owns ordering and knows nothing about
    # which key a string came from, while loader.py wraps the failure with the
    # snapshot path when it is the one that asked. Testing the raw type here
    # keeps that boundary asserted rather than assumed.
    with pytest.raises(ValueError):
        parse_version(bad)


def test_a_yaml_float_version_is_refused(tmp_path):
    """*** An unquoted 1.10 in YAML parses as the float 1.1.

    So a loader that accepted numbers would silently compare 1.1 where the
    author wrote 1.10, and would then order it BELOW 1.9. Refusing the type
    makes that impossible rather than subtle.

    The message has to say the word string, because the fix is to add quotes in
    the configuration and that is not obvious from a bare type error.

    Mutation, run: make parse_version coerce with str() instead of refusing and
    this goes red.
    """
    tree = filled_tree()
    # A float, which is exactly what the YAML parser hands back for an unquoted
    # 1.0 in the file. So this is not a synthetic type error: it is what
    # happens the first time somebody removes the quotes while tidying.
    tree["cmdset"]["min_agent_version"] = 1.0
    root, boot = build_root(tmp_path, tree)
    with pytest.raises(P4ConfigError) as exc:
        load_p4_config("1.0", root=root, boot_id_path=boot)
    # The word matters, not the exception type alone. The fix is to put the
    # quotes back in the configuration, and a message that only said "invalid
    # version" would send the reader to the version module instead.
    assert "string" in str(exc.value)


def test_a_null_min_agent_version_names_the_key_and_assertion_A(tmp_path):
    """require(), not get() with a default.

    A default minimum would mean "when the configuration does not say which
    implementation it needs, assume this one will do" -- which is the entire
    question the check was asked. CLAUDE.md 3.1 forbids the shape outright, and
    here it would also be self-defeating.

    The message must name the key and must point at assertion A, for the same
    reason as the skeleton case: a null reaching the reader means the freeze
    line let it through, and the reader should not read as the component at
    fault.

    Mutation: replace require with get(key, "0.0") in check_min_agent_version
    and this goes red.
    """
    tree = filled_tree()
    tree["cmdset"]["min_agent_version"] = None
    root, boot = build_root(tmp_path, tree)
    with pytest.raises(ResolvedConfigError) as exc:
        load_p4_config("1.0", root=root, boot_id_path=boot)
    assert "cmdset.min_agent_version" in str(exc.value)
    assert "assertion A" in str(exc.value)


def test_the_agent_version_has_no_default_anywhere():
    """*** The check must not be able to pass by construction.

    No volume states what version this implementation is. A grep across docs/
    for min_agent_version and agent_version finds only 16 S12.4 and 16 S14, and
    both are the requirement side. A constant invented in the loader would have
    been chosen by the same person who wrote the check, so 16 S12.4's rule would
    then be satisfied by definition -- CLAUDE.md 3.2 form 1, an assertion a
    do-nothing implementation passes.

    So the parameter is required, and this asserts it stays required. A
    behavioural test cannot see a default somebody adds tomorrow; a signature
    test can.
    """
    # Both entry points, because a default added to either one is enough. The
    # loader is what a process calls; the check is what a future caller might
    # call directly, and a default there would spread to the loader the next
    # time somebody simplified the call.
    signature = inspect.signature(check_min_agent_version)
    assert signature.parameters["agent_version"].default is inspect.Parameter.empty
    signature = inspect.signature(load_p4_config)
    assert signature.parameters["agent_version"].default is inspect.Parameter.empty


# -- the history closed set ---------------------------------------------------

def test_an_unknown_history_scenario_refuses_startup(tmp_path):
    """16 S14 on prompt.history.enable_on: 闭集 ... 未知值 -> 启动即拒.

    Out-of-set values raise and are never read as the nearest known one, which
    11 S13.6 requires in as many words. The alternative -- interpret an
    unrecognised scenario as one of the three -- would attach a history line to
    the wrong situation, and 16 S13's GATE-2 row names that exact shape as
    forbidden for a neighbouring field.

    Mutation, run: delete the membership test in check_history_scenarios and
    this goes red.
    """
    tree = filled_tree()
    # One legal member beside one illegal one. An all-illegal list would also be
    # caught by an implementation that rejected any list it did not recognise
    # wholesale, and that implementation would then reject the legal
    # three-member list the configuration ships. The mixed list separates them.
    #
    # "teleop" specifically because it is a real concept elsewhere in this
    # system -- there is a teleop plane, a teleop priority and a teleop deadman
    # -- so it is the kind of plausible-looking value somebody would actually
    # type here, rather than an obvious nonsense string no reviewer would write.
    tree["prompt"]["history"]["enable_on"] = ["clarify", "teleop"]
    root, boot = build_root(tmp_path, tree)
    with pytest.raises(P4ConfigError) as exc:
        load_p4_config("1.0", root=root, boot_id_path=boot)
    # The offending value and the key path, so the report is actionable without
    # a second run.
    assert "teleop" in str(exc.value)
    assert "prompt.history.enable_on" in str(exc.value)


def test_a_bare_string_enable_on_is_reported_as_a_missing_bracket(tmp_path):
    """`enable_on: clarify` without brackets is a string, not a one-element list.

    Iterating a string yields characters, so a membership test alone would
    report an unknown scenario 'c' and send the reader hunting in entirely the
    wrong place. The type check exists so the message describes the actual
    mistake, which is a YAML author omitting two characters.
    """
    tree = filled_tree()
    # The string is a LEGAL scenario name, which is what makes this case worth
    # having. A membership test over its characters produces "unknown scenario
    # 'c'", and the reader then goes looking for where a single letter could
    # have come from -- with the actual mistake, two missing brackets, sitting
    # in plain sight in the file.
    tree["prompt"]["history"]["enable_on"] = "clarify"
    root, boot = build_root(tmp_path, tree)
    with pytest.raises(P4ConfigError) as exc:
        load_p4_config("1.0", root=root, boot_id_path=boot)
    assert "list" in str(exc.value)


def test_a_missing_line_template_is_caught_in_both_directions(tmp_path):
    """*** A subset test would miss the realistic edit.

    16 S6.3.5 requires one fixed line per scenario. A check that only asked
    "does every enabled scenario have a template" passes on a file where a
    template key was misspelled AND the scenario was dropped from enable_on --
    which is exactly what a careless edit produces, because the author fixes the
    symptom by disabling the scenario.

    On site the symptom of the miss is a history line that silently never
    appears, which reads as the model having forgotten the question it just
    asked. Nothing in the logs says otherwise.

    Mutation, run: replace the bidirectional difference with a subset test and
    the second half of this goes red while the first half still passes.
    """
    # First direction: a template is missing for a scenario that exists. This
    # half is caught by a subset test too, which is why it cannot be the only
    # half -- a check that passes the easy direction reads as covering both.
    tree = filled_tree()
    del tree["prompt"]["history"]["line_templates"]["recording"]
    root, boot = build_root(tmp_path, tree)
    with pytest.raises(P4ConfigError) as exc:
        load_p4_config("1.0", root=root, boot_id_path=boot)
    assert "recording" in str(exc.value)

    # Second direction: a template exists for a scenario that does not, and the
    # author has quietly removed the scenario from enable_on so nothing else
    # complains. A subset test is green here.
    tree = filled_tree()
    tree["prompt"]["history"]["line_templates"]["recordng"] = "typo"
    tree["prompt"]["history"]["enable_on"] = ["clarify", "pending_confirm"]
    root, boot = build_root(tmp_path / "second", tree)
    with pytest.raises(P4ConfigError) as exc:
        load_p4_config("1.0", root=root, boot_id_path=boot)
    assert "recordng" in str(exc.value)


def test_the_scenario_set_matches_the_configuration_the_design_ships():
    """The closed set in code and the lists in the configuration agree.

    Two hand-maintained copies of a closed set is how they diverge, and the
    divergence is invisible from either side. This is the metatest that notices,
    and it is nearly free: the shipped configuration enables all three
    scenarios, so code and file must line up exactly.
    """
    history = source_tree()["prompt"]["history"]
    # Sorted on both sides, so the case is about membership rather than about
    # the order the yaml happens to list them in. Order carries meaning for two
    # of the contract's closed sets -- gate_limiter's order is the attribution
    # priority and stop_reason's is a decision order -- but not for this one,
    # and asserting an order that means nothing is a case that fails on a
    # harmless edit.
    assert sorted(history["enable_on"]) == sorted(HISTORY_SCENARIOS)
    assert sorted(history["line_templates"]) == sorted(HISTORY_SCENARIOS)


# -- clause 1: the loader reads the product, never the source -----------------

#: The configuration source path, assembled from parts. See SOURCE_YAML for why
#: no contiguous copy of the literal may appear in this file.
_ABS_SOURCE_LITERAL = "/opt/" + "xbrain_v6/" + "configs/" + "p4_agent.yaml"

#: The relative spelling of the same thing. Checked as well as the absolute one
#: because a module that wrote the relative form would be reaching for the
#: source just as surely, and would additionally depend on the working directory
#: the unit happened to have.
#:
#: Note that the relative form is a substring of the absolute one, so the two
#: patterns overlap and an absolute hit is reported once rather than twice --
#: the scan tests membership per line, not per pattern. That is intentional:
#: one line naming the source is one finding, and a reader given two entries
#: for it would go looking for a second occurrence that does not exist.
_REL_SOURCE_LITERAL = "configs/" + "p4_agent.yaml"


def scan_for_source_literal(path):
    """Line numbers in one file naming this process's configuration source.

    Line numbers rather than a boolean, so a failure names the line to open. A
    checker that answers only yes or no on a four-hundred-line module leaves the
    reader to grep for a string this test file goes out of its way not to
    contain, which is an unkind puzzle to set.

    Comments are NOT exempt here, unlike in the repository-wide lint where an
    unmarked literal inside a comment is reported as prose and not failed. The
    surface is one small package that this item owns, the rule is easy to keep,
    and a path written in a comment is still a path somebody copies.
    """
    with open(path, encoding="utf-8") as fh:
        return [i for i, line in enumerate(fh, 1)
                if _ABS_SOURCE_LITERAL in line or _REL_SOURCE_LITERAL in line]


def test_no_runtime_module_names_the_configuration_source():
    """*** Clause 1's static half.

    Scan surface: every .py file under xbrain/p4_agent/. Declared here and
    asserted non-empty below, because a zero from a tree nobody walked is
    indistinguishable from a zero from a clean one (CLAUDE.md 3.2 form 6).

    tests/ is outside the surface on purpose -- a fixture has to be able to name
    the file it asserts about -- which is the same reasoning
    scripts/lint/no_config_source_read.py applies to its own surface.

    This is the static half only. The run-time half is that the shared reader
    confines every path it opens to the resolved root;
    tests/common/test_resolved.py covers that, and the case below re-checks it
    through the P4 entry point so P4 cannot have quietly built its own path.

    Mutation, run: add a module-level constant holding the source path to
    loader.py and this goes red.
    """
    files = list(python_files(os.path.join("xbrain", "p4_agent")))
    # The surface is asserted non-empty before anything is judged. A rename of
    # the package would otherwise leave this walking a directory that does not
    # exist, reporting zero hits, and passing forever -- which is the precise
    # shape CLAUDE.md 3.2 form 6 describes and the reason
    # no_config_source_read.py prints a per-directory file count on every run.
    assert files, "the scan surface is empty; nothing was actually checked"
    # Paths reported relative to the repository root, because an absolute path
    # containing a temporary directory tells the reader nothing about which file
    # in the tree they have to open.
    offenders = [(os.path.relpath(p, ROOT), scan_for_source_literal(p))
                 for p in files]
    offenders = [o for o in offenders if o[1]]
    assert not offenders, (
        "a runtime module names the configuration source: %s. Processes read "
        "the freeze line's product; naming the source is how per-process "
        "expansion creeps back in (10 S5.4.1)" % offenders
    )


def test_the_source_literal_scanner_reports_a_planted_hit(tmp_path):
    """The positive control for the scan above.

    A scanner reporting nothing on a clean tree and nothing on a dirty one would
    pass that test forever. This is not hypothetical in this repository:
    comment_ratio.py was silently a no-op through an entire fix because nothing
    exercised it.
    """
    planted = tmp_path / "planted.py"
    # Written as a module-level constant, which is the shape this would really
    # take: nobody writes an open() call on the configuration source by
    # accident, but a constant "so the path is in one place" is an entirely
    # reasonable-looking thing to add.
    planted.write_text('CFG = "%s"\n' % _ABS_SOURCE_LITERAL, encoding="utf-8")
    # The exact line number, not merely a non-empty result. A scanner that
    # returned every line of every file would satisfy a truthiness check and
    # then bury a real finding among four hundred false ones.
    assert scan_for_source_literal(str(planted)) == [1]


def test_a_manifest_pointing_at_the_source_is_refused(tmp_path):
    """*** Clause 1's run-time half, through the P4 entry point.

    A manifest is data. One naming a path under the configuration root would
    turn the reader into the source-reading path the whole item forbids, and it
    would do so with every static scan still clean -- because no module would
    contain any literal at all. That is the case rule 1 of the repository lint
    cannot see and says so.

    The sha256 is dropped from the entry because the file at the redirected path
    would not match it, and the point of this case is which check fires: it must
    be the confinement check, before anything is read.
    """
    root, boot = build_root(tmp_path, filled_tree())
    # The manifest is rewritten after the fact rather than built wrong from the
    # start, so the tree on disk is a complete, valid product and the ONLY thing
    # different about this case is where the manifest points. If the fixture
    # built a broken tree instead, a refusal here would prove nothing about the
    # confinement check.
    manifest_path = os.path.join(root, "MANIFEST.json")
    with open(manifest_path, encoding="utf-8") as fh:
        doc = json.load(fh)
    doc["processes"]["p4_agent"]["path"] = _ABS_SOURCE_LITERAL
    doc["processes"]["p4_agent"].pop("sha256", None)
    with open(manifest_path, "w", encoding="utf-8") as fh:
        json.dump(doc, fh)
    with pytest.raises(ResolvedConfigError) as exc:
        load_p4_config("1.0", root=root, boot_id_path=boot)
    assert "outside" in str(exc.value)


def test_load_p4_config_offers_no_switch_that_relaxes_any_of_this():
    """CLAUDE.md 3.6: no switch may exist that skips a safety check.

    Asserted against the signature rather than by behaviour, because the failure
    is somebody ADDING such a parameter later with a friendly default, most
    likely while trying to get a test environment up. A behavioural test cannot
    see a parameter that does not exist yet.

    The banned list covers the names this would plausibly be given, including
    warn_only -- which is the CLAUDE.md 3.6 form about assertion grading, the
    standard path from fail-safe to fail-silent.
    """
    # The first six names are the list tests/common/test_resolved.py applies to
    # the shared reader, kept identical on purpose: the two entry points are one
    # chain, and a relaxation added at either end has the same effect. The last
    # three are the ones this loader specifically invites, because it is the
    # layer that performs the refusals somebody might want to postpone.
    names = set(inspect.signature(load_p4_config).parameters)
    for banned in ("fallback", "allow_source", "source_root", "strict",
                   "skip_boot_check", "ignore_boot_id", "skip_checks",
                   "allow_unassigned", "warn_only"):
        assert banned not in names, (
            "load_p4_config must not offer %r; a parameter that relaxes a "
            "startup refusal is a remote switch for the guarantee it provides"
            % banned
        )


# -- clause 2: no defaulting construct on a safety key ------------------------

#: The four safety namespaces CLAUDE.md 3.1 and 10 S5.4.5 place off limits to
#: code defaults. Written as key-path prefixes because that is the form they
#: take inside a source file -- as the argument of a configuration lookup.
#:
#: common.qos.* is the fifth namespace 10 S5.4.4's hot-update row names, and it
#: is deliberately absent here. That row is about the intersection with the
#: hot-update whitelist rather than about code defaults, and a QoS profile with
#: a code default is not the fail-silent shape this rule is written for -- it
#: would be loud. Listing it anyway would widen the rule for appearance rather
#: than for a failure anyone can name.
SAFETY_NAMESPACES = ("common.spec.", "common.safety.", "common.motion.profiles",
                     "common.fence.")

#: Name suffixes that mark a physical quantity.
#:
#: A dataclass field or a function parameter carrying one of these AND a default
#: is the exact shape CLAUDE.md 3.1 lists first among the three forbidden
#: writings, and it is the one the item's own mutation names. It is also the one
#: that survives review, because it reads as a type declaration rather than as a
#: decision about how fast the robot may move.
#:
#: Suffix-based rather than a hand-written list of safety parameter names. Such
#: a list would have to be kept in step with 10 S5.4.5 by hand and would
#: silently miss whatever was added last -- and a checker that misses the newest
#: parameter is worst exactly when the parameter is newest. A quantity with its
#: unit in the name is what this project actually writes: max_vx_mps, t_lat_s,
#: d_safe_m, throttle_speed_mps, max_duration_floor_ms.
#:
#: What this misses, so nobody reads the pass as broader than it is: a limit
#: named without its unit -- `timeout`, `max_speed`, `margin` -- carries a
#: default straight through this rule. The project convention is to name the
#: unit and the configuration files follow it, so the rule matches the code that
#: exists; it is not a proof about code that could be written. Widening it to
#: every numerically-annotated field was considered and rejected: it would fire
#: on ordinary counters and retry limits, and a rule that fires on ordinary code
#: is switched off, which leaves nothing at all.
UNIT_SUFFIXES = ("_mps", "_mps2", "_m", "_s", "_ms", "_radps", "_deg", "_hz",
                 "_bytes", "_eps")


def _mentions_safety_namespace(node):
    """True when a syntax node contains a string naming a safety namespace.

    Walks the whole subtree rather than testing one constant, because the key
    path is often assembled: cfg.get(PREFIX + "max_vx_mps", 2.0) has no single
    constant carrying the namespace, and a check looking only at the first
    argument as a literal would miss every call written that way.

    The cost of walking is a false positive when a safety key path is mentioned
    somewhere unrelated inside the same expression. That direction is the right
    one to be wrong in: a false positive is one line of review, while a false
    negative is a limit nobody decided taking effect at run time.
    """
    for child in ast.walk(node):
        if isinstance(child, ast.Constant) and isinstance(child.value, str):
            if any(ns in child.value for ns in SAFETY_NAMESPACES):
                return True
    return False


def _is_dataclass(class_node):
    """True when a class carries a dataclass decorator in any spelling.

    Matched on the dumped decorator rather than on a name, so @dataclass,
    @dataclasses.dataclass and @dataclass(frozen=True) are all recognised. A
    name-only match would miss the frozen form, which is the one CLAUDE.md 4.5
    recommends for DTOs and therefore the one most likely to be written.
    """
    for decorator in class_node.decorator_list:
        if "dataclass" in ast.dump(decorator):
            return True
    return False


def scan_for_safety_defaults(path):
    """(lineno, rule, detail) for every defaulting construct this forbids.

    Three rules, applied where they are stated rather than folded into one
    predicate, so a reader can tell which fired and why. Parsed with ast rather
    than matched with a regex, because every one of these forms wraps across
    lines in real code and a line-based matcher would miss precisely the ones
    somebody took trouble to format.
    """
    with open(path, encoding="utf-8") as fh:
        tree = ast.parse(fh.read(), filename=path)
    findings = []

    for node in ast.walk(tree):
        # Rule 1 -- a lookup on a safety key path that carries a fallback. This
        # is the criterion's dict.get(k, v) form, and it is the one that reads
        # most innocently at a call site: the default looks like documentation
        # of the expected value rather than like a decision.
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            if node.func.attr == "get" and (len(node.args) > 1 or node.keywords):
                if node.args and _mentions_safety_namespace(node.args[0]):
                    findings.append((node.lineno, "get-with-default",
                                     "lookup on a safety key path carries a "
                                     "fallback value"))
        # Rule 2 -- the `or v` form, detected on assignment and return only.
        # Deliberately not on every BoolOp: `if a or b` is ordinary boolean
        # logic, and a rule that fired on it would be switched off within a
        # week, which is worse than not having the rule.
        if isinstance(node, (ast.Assign, ast.AnnAssign, ast.Return)):
            value = getattr(node, "value", None)
            if isinstance(value, ast.BoolOp) and isinstance(value.op, ast.Or):
                last = value.values[-1]
                if isinstance(last, ast.Constant) and _mentions_safety_namespace(value):
                    findings.append((node.lineno, "or-fallback",
                                     "a safety value falls back to a literal"))
        # Rule 3 -- a physical quantity with a code default, as a dataclass
        # field. Checked by field name rather than by whether the value looks
        # like a limit, because the dangerous default is the plausible one.
        if isinstance(node, ast.ClassDef) and _is_dataclass(node):
            for stmt in node.body:
                if isinstance(stmt, ast.AnnAssign) and stmt.value is not None:
                    name = getattr(stmt.target, "id", "")
                    if name.endswith(UNIT_SUFFIXES):
                        findings.append((stmt.lineno, "dataclass-default",
                                         "dataclass field %s has a default" % name))
        # Rule 3 continued -- the same quantity as a function parameter default.
        # Included because moving a limit from a dataclass into a helper's
        # signature is the obvious way around a dataclass-only rule, and it is
        # not obviously wrong to whoever does it.
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            args = node.args
            # Positional defaults bind to the LAST n positional parameters, so
            # the slice is taken from the end; keyword-only defaults are
            # positional in their own list with None marking "no default".
            defaulted = args.args[len(args.args) - len(args.defaults):]
            defaulted = defaulted + [a for a, d in zip(args.kwonlyargs,
                                                       args.kw_defaults)
                                     if d is not None]
            for arg in defaulted:
                if arg.arg.endswith(UNIT_SUFFIXES):
                    findings.append((node.lineno, "parameter-default",
                                     "parameter %s has a default" % arg.arg))
    return findings


def test_no_p4_module_defaults_a_safety_value():
    """*** Clause 2.

    Scan surface: every .py file under xbrain/p4_agent/, asserted non-empty.

    What this establishes, and what it does not. It catches the three writings
    CLAUDE.md 3.1 names -- a fallback on a safety key path, an `or` fallback,
    and a code default on a physical quantity. It cannot catch a default
    assembled at run time, or one hidden behind a name carrying no unit. The
    criterion is therefore "none of the three named forms is present", NOT "no
    default exists anywhere", and claiming the second would be CLAUDE.md 3.2
    form 7 -- a conclusion defined into its premise.

    Mutations, both run against the real loader: add a dataclass with a
    max_decel_mps2 default, and add a get on a common.safety key with a
    fallback. Each turns this red.
    """
    files = list(python_files(os.path.join("xbrain", "p4_agent")))
    # Same non-empty guard as the other static cases, same reason: a surface
    # that silently vanished would make this pass by measuring nothing.
    assert files, "the scan surface is empty; nothing was actually checked"
    # Every file is scanned and every finding collected before anything is
    # asserted, rather than failing on the first hit. Filling in defaults tends
    # to happen in batches -- somebody adds a limits dataclass with five fields
    # -- and reporting one per run turns one review into five.
    offenders = []
    for path in files:
        for lineno, rule, detail in scan_for_safety_defaults(path):
            # File, line, rule name and a sentence. The rule name is in the
            # message because the four rules have four different fixes, and
            # "there is a default somewhere on line 40" does not say which.
            offenders.append("%s:%d %s -- %s"
                             % (os.path.relpath(path, ROOT), lineno, rule, detail))
    assert not offenders, (
        "a safety value has a code default: %s. An uncalibrated value belongs "
        "in the configuration as null so assertion A names it; a default in "
        "code means the stack starts on a number nobody decided (CLAUDE.md 3.1)"
        % offenders
    )


@pytest.mark.parametrize("source,rule", [
    ("from dataclasses import dataclass\n"
     "@dataclass\n"
     "class Limits:\n"
     "    max_decel_mps2: float = 2.5\n", "dataclass-default"),
    ("a = cfg.get('common.safety.t_lat_s', 0.4)\n", "get-with-default"),
    ("a = cfg.get('common.spec.max_vx_mps') or 2.0\n", "or-fallback"),
    ("def f(max_vx_mps=2.0):\n    return max_vx_mps\n", "parameter-default"),
])
def test_the_safety_default_scanner_reports_each_planted_form(tmp_path, source,
                                                              rule):
    """*** The mutation the criterion names, run against each form.

    The item says: add a dataclass default and CI goes red. That is the first
    row, and the same mutation was also applied to the real loader module. The
    other three are the forms CLAUDE.md 3.1 lists beside it, planted for the
    same reason -- a checker that catches only the shape its author had in mind
    is the one that misses the shape somebody actually writes.
    """
    # A distinct filename per case, so a failure in one parametrisation cannot
    # be caused by a leftover file from another. tmp_path is already per-case
    # here, but the habit costs nothing and the alternative fails confusingly.
    planted = tmp_path / ("planted_%s.py" % rule.replace("-", "_"))
    planted.write_text(source, encoding="utf-8")
    # Membership rather than equality on the rule list. A planted snippet may
    # legitimately trip more than one rule -- the `or` sample also contains a
    # get on a safety key -- and demanding an exact match would make the case
    # fail for being too thorough.
    rules = [r for _lineno, r, _detail in scan_for_safety_defaults(str(planted))]
    assert rule in rules, "planted %s was not reported; got %s" % (rule, rules)


def test_the_safety_default_scanner_leaves_innocent_code_alone(tmp_path):
    """The negative control, without which the scanner could flag everything.

    A rule that fires on ordinary code is switched off within a week, and an
    over-broad checker is indistinguishable from a broken one. That is not
    theoretical here: header_lint.py in this repository once flagged 91 correct
    files, and the response was to stop running it.

    The sample holds one of each innocent shape: a dataclass default on a
    non-quantity, boolean logic with a literal on the right, a require() with no
    fallback at all, and an ordinary single-argument get on a payload.
    """
    # Each line of the sample below is a shape that appears in real code in this
    # repository, so a rule that fired on any of them would start producing
    # findings the day it was pointed at anything larger than this package.
    innocent = tmp_path / "innocent.py"
    innocent.write_text(
        "from dataclasses import dataclass\n"
        "@dataclass\n"
        "class Shape:\n"
        "    name: str = 'circle'\n"
        "def g(flag=True):\n"
        "    return flag or False\n"
        "v = cfg.require('common.safety.t_lat_s')\n"
        "w = payload.get('kind')\n",
        encoding="utf-8")
    assert scan_for_safety_defaults(str(innocent)) == []


# -- clause 5: no singular relative configuration root ------------------------

#: The singular form, assembled from parts. Written this way for the reason
#: given at SOURCE_YAML: a criterion whose own text lies inside the surface it
#: scans can never reach zero, and the usual repair is to loosen it until it
#: does -- CLAUDE.md 3.2 form 3.
_SINGULAR = "config" + "/"

#: A quoted path beginning with the singular root.
#:
#: Matching on the opening quote is what makes this checkable at all. The bare
#: substring occurs inside this item's OWN target directory name,
#: xbrain/p4_agent/config/, so a criterion phrased as "the substring must not
#: appear" is red by construction -- which is recorded as a finding for this
#: item rather than worked around silently.
_SINGULAR_QUOTED_RE = re.compile("[\"']" + _SINGULAR)

#: The absolute singular root. Unambiguous without a quote, because the plural
#: spelling has an s where this has a separator.
_SINGULAR_ABSOLUTE = "/opt/" + "xbrain_v6/" + _SINGULAR


def scan_for_singular_config_root(path):
    """Line numbers using the singular configuration root as a path.

    Two patterns, because the singular root appears in two shapes and only one
    of them can be recognised without a quote:

      * relative -- caught by the opening quote, since a relative path only ever
        appears inside a string literal. Without the quote the pattern would
        match the package directory name in every import and every docstring in
        this item, and would therefore be red forever;
      * absolute -- caught on its own, because the plural spelling has an `s`
        where this has a separator and the two cannot be confused.

    Applied to yaml as well as to Python. The configuration file is where a
    relative path would do the most damage: nothing in the file itself would
    look wrong, and the process reading it would resolve the path against a
    working directory nobody chose.
    """
    with open(path, encoding="utf-8") as fh:
        return [i for i, line in enumerate(fh, 1)
                if _SINGULAR_QUOTED_RE.search(line) or _SINGULAR_ABSOLUTE in line]


def test_nothing_in_this_item_uses_the_singular_configuration_root():
    """*** Clause 5, with its scan surface narrowed to what this item owns.

    Surface: every .py under xbrain/p4_agent/ and tests/p4_agent/, plus the
    configuration file itself.

    Stated because the clause as written says the whole repository, and the
    whole repository cannot pass it as a substring test: this item's own target
    directory is named xbrain/p4_agent/config/, so the bare substring is present
    by construction. The checkable property is the one that matters anyway --
    the singular root used as a PATH -- and that is what this asserts.

    Why the singular form is worth a rule. 00 CFG-03 and 10 S5.4.0 CFG-ROOT-1
    make the configuration root plural and absolute. A relative singular path
    resolves against whatever working directory a unit happens to have, so the
    same code reads different files depending on how it was started, and on a
    machine where nothing is there it reads nothing at all and falls back to
    whatever the code assumed.

    Mutation, run: add a module constant holding a singular relative path to
    loader.py and this goes red.
    """
    files = list(python_files(os.path.join("xbrain", "p4_agent"),
                              os.path.join("tests", "p4_agent")))
    # tests/p4_agent is INSIDE this surface, unlike the source-literal scan
    # above. The difference is not an oversight: a fixture has a legitimate
    # reason to name the configuration source, and no legitimate reason to write
    # a singular relative path. Including this file also means the checker is
    # applied to its own text, which is only safe because the pattern is built
    # from parts -- see the constants above.
    files.append(SOURCE_YAML)
    # Greater than one, not merely truthy: SOURCE_YAML is appended
    # unconditionally, so a truthiness check would pass on an otherwise empty
    # surface and report a clean scan of exactly one file.
    assert len(files) > 1, "the scan surface is empty; nothing was checked"
    offenders = [(os.path.relpath(p, ROOT), scan_for_singular_config_root(p))
                 for p in files]
    offenders = [o for o in offenders if o[1]]
    assert not offenders, (
        "the singular configuration root is used as a path: %s. The root is "
        "plural and absolute (00 CFG-03, 10 S5.4.0 CFG-ROOT-1)" % offenders
    )


def test_the_singular_root_scanner_reports_planted_hits_and_spares_the_plural(tmp_path):
    """Positive and negative control in one case.

    The negative half is the important one here. A checker written as a bare
    substring test reports the plural path too -- that is, every correct line in
    the tree -- and a checker that reports every correct line is one somebody
    switches off. The innocent sample therefore includes the plural source path,
    the resolved path, and a joined path naming this item's own package
    directory.
    """
    # The planted line is the mistake in its most natural form: a relative path
    # that reads perfectly well and works on the one machine where it was
    # tested, because that machine happened to start the process from the
    # repository root.
    planted = tmp_path / "planted.py"
    planted.write_text('p = "' + _SINGULAR + 'p4_agent.yaml"\n', encoding="utf-8")
    assert scan_for_singular_config_root(str(planted)) == [1]

    # Three innocent shapes, each of which a naive substring test reports:
    #   * the plural relative source path, which appears legitimately in tests;
    #   * the resolved path, which is what every process is supposed to use;
    #   * a joined path naming this item's own package directory, which is the
    #     construction that makes the bare-substring form of clause 5 red by
    #     construction and is the reason the criterion had to be narrowed.
    innocent = tmp_path / "innocent.py"
    innocent.write_text(
        'p = "' + _REL_SOURCE_LITERAL + '"\n'
        'q = "/run/xbrain/resolved/p4_agent.yaml"\n'
        'r = os.path.join(here, "p4_agent", "config", "loader.py")\n',
        encoding="utf-8")
    assert scan_for_singular_config_root(str(innocent)) == []


# -- the startup report --------------------------------------------------------

def test_the_startup_report_takes_every_field_from_one_load(tmp_path):
    """Manifest fields and snapshot fields, gathered together on purpose.

    A caller assembling them separately can end up reporting a common_digest
    from one load beside a cmdset version from another, and the resulting event
    looks like evidence while being none. That is worse than emitting nothing,
    because somebody will later reason from it.

    common_digest is reported and NOT compared here. 10 S3.3.3 puts that
    comparison at P2's Stage D with a different failure behaviour -- keep motion
    disallowed and emit a fault, rather than refuse to start -- so comparing it
    in this module would apply the wrong severity to a condition the design
    wants observable and reported.
    """
    root, boot = build_root(tmp_path, filled_tree())
    cfg = load_p4_config("1.0", root=root, boot_id_path=boot)
    # One load, one report. The whole reason this function exists rather than
    # leaving the caller to assemble a dict is that a caller with two loads in
    # scope will eventually take one field from each.
    report = startup_report(cfg)
    assert report["proc"] == "p4_agent"
    # From the manifest.
    assert report["common_digest"] == "9f2c4a1b7e5d0836"
    # From the snapshot, so the version in the event is the one actually in
    # force rather than a constant compiled into the process.
    assert report["cmdset_version"] == "1.0.0"
    assert report["min_agent_version"] == "1.0"


# -- the checks are reachable on their own ------------------------------------

def test_each_check_refuses_on_its_own(tmp_path):
    """The individual checks are exported and behave the same called alone.

    They are exported so a mutation test can probe one refusal without building
    a whole valid snapshot around it; the cost of not exporting them is tests
    that stop being written, which is how a refusal ends up with no case at all.

    This asserts the exported entry points are the same code the loader runs
    rather than a second implementation kept for the tests -- a duplicate would
    drift, and the copy the tests exercise is always the one that stays correct.
    """
    root, boot = build_root(tmp_path, filled_tree())
    cfg = load_p4_config("1.0", root=root, boot_id_path=boot)
    # A valid configuration passes all three when they are driven directly. No
    # assertion is needed on these three lines: each raises on failure, so
    # reaching the next line IS the assertion, and wrapping them in a truthiness
    # check would suggest they return a verdict when they do not.
    check_min_agent_version(cfg, "1.0")
    check_no_unassigned_keys(cfg)
    check_history_scenarios(cfg)
    # And the version check still refuses when driven directly, so the refusal
    # lives in the check rather than in the loader that happens to call it.
    with pytest.raises(P4ConfigError):
        check_min_agent_version(cfg, "0.9")
