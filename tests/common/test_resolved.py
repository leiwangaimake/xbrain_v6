"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_resolved.py
Brief: The boot_id gate, the absence of any fallback to source, and their mutants

Description:
CFG-CM-11, reader side. The item is not really about reading a YAML file -- it is
about one guarantee: a process runs on the snapshot the freeze line produced
during THIS boot, or it does not run.

Two properties carry that, and each is tested from both directions because each
has a failure mode that looks like the other:

  * the gate is full-value equality. 11 S3.0 defines an envelope field `boot`
    that is verbatim the first 8 hex digits of boot_id, so a reader that reuses
    that helper compares eight characters against a UUID and fails on EVERY boot.
    Whoever meets a permanently-red assertion loosens it -- usually to
    startswith -- and then it accepts any two boots sharing a prefix. Both the
    strict case and the loosened case have a test here, because a suite that had
    only one of them would bless the other.

  * there is no fallback. Every failure path is asserted to raise rather than to
    read /opt/xbrain_v6/configs/, including the paths where falling back would
    look helpful: no snapshot, no manifest entry, a manifest that names a source
    path outright.

Each case names the mutation that turns it red -- CLAUDE.md 3.3: an assertion
that has never been red has not been written.
"""

import hashlib
import json
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from xbrain.common.config import resolved as R  # noqa: E402
from xbrain.common.config.merge import MISSING  # noqa: E402
from xbrain.common.config.resolved import (  # noqa: E402
    ResolvedConfigError,
    load_manifest,
    load_resolved,
    read_boot_id,
)

#: A full boot id, the shape /proc/sys/kernel/random/boot_id actually returns.
#: Written out rather than read from the machine so the tests behave the same on
#: a build agent, and so the prefix cases below have a known first 8 characters.
BOOT_A = "9c26dc40-cfad-49b2-afd7-36dc01e1cb34"

#: Differs from BOOT_A only after the first eight hex digits. This is the value
#: that separates a correct comparison from one loosened to a prefix match: a
#: startswith implementation accepts it, an equality implementation does not.
BOOT_A_SAME_PREFIX = "9c26dc40-0000-0000-0000-000000000000"

#: Unrelated.
BOOT_B = "11112222-3333-4444-5555-666677778888"

SNAPSHOT = """\
loop_hz: 20
safety_distance:
  d_safe_m: 1.0
corridor:
  margin_base_m: 1.0
teleop:
  cloud:
    enabled: null
"""


def build_root(tmp_path, *, boot_id=BOOT_A, proc="p1_motion", snapshot=SNAPSHOT,
               manifest_extra=None, drop=(), sha256=True, path_override=None):
    """Construct a fake /run/xbrain/resolved tree and return (root, boot_id_path).

    A builder rather than a fixture with parameters, because nearly every case
    below differs in exactly one field and a builder makes that difference the
    only thing visible at the call site.
    """
    root = tmp_path / "resolved"
    root.mkdir(parents=True, exist_ok=True)

    entry = {"path": path_override or str(root / (proc + ".yaml")), "refs": 7}
    if snapshot is not None:
        (root / (proc + ".yaml")).write_text(snapshot, encoding="utf-8")
        if sha256:
            entry["sha256"] = hashlib.sha256(snapshot.encode("utf-8")).hexdigest()

    manifest = {
        "schema": "xbrain.config.manifest/1",
        "boot_id": boot_id,
        "common_digest": "9f2c4a1b7e5d0836",
        "config_rev": "2026-08-06T00:00:00Z+gabcdef0",
        "config_root": "/opt/xbrain_v6/configs",
        "config_root_overridden": False,
        "processes": {proc: entry},
    }
    if manifest_extra:
        manifest.update(manifest_extra)
    for key in drop:
        manifest.pop(key, None)
    (root / "MANIFEST.json").write_text(json.dumps(manifest), encoding="utf-8")

    boot_file = tmp_path / "boot_id"
    boot_file.write_text(BOOT_A + "\n", encoding="utf-8")
    return str(root), str(boot_file)


# -- the gate, both directions -------------------------------------------------

def test_matching_boot_id_loads(tmp_path):
    """The positive case, without which every negative case below is satisfied
    by a reader that refuses everything."""
    root, boot = build_root(tmp_path)
    cfg = load_resolved("p1_motion", root=root, boot_id_path=boot)
    assert cfg.get("loop_hz") == 20
    assert cfg.manifest.common_digest == "9f2c4a1b7e5d0836"


def test_stale_snapshot_from_a_previous_boot_is_refused(tmp_path):
    """*** The core assertion of CFG-CM-11.

    Mutation: delete the comparison => red. The scenario is /run not having been
    cleared as tmpfs should clear it, and the consequence of continuing is the
    whole stack running on last boot's configuration.
    """
    root, boot = build_root(tmp_path, boot_id=BOOT_B)
    with pytest.raises(ResolvedConfigError) as exc:
        load_resolved("p1_motion", root=root, boot_id_path=boot)
    text = str(exc.value)
    assert BOOT_B in text and BOOT_A in text, (
        "the message must print BOTH values; 'boot id mismatch' alone leaves the "
        "operator unable to tell which boot the snapshot came from"
    )


def test_a_boot_id_sharing_the_first_eight_characters_is_still_refused(tmp_path):
    """*** The mutation that a loosened comparison would pass.

    11 S3.0's envelope field `boot` is the first 8 hex digits of boot_id. A
    reader that compared on that basis -- or that was "fixed" into a startswith
    after the strict version appeared permanently red -- accepts this snapshot.
    It is from a different boot.

    Mutation: change the comparison to manifest_boot.startswith(current[:8]) or
    to any prefix form => red.
    """
    root, boot = build_root(tmp_path, boot_id=BOOT_A_SAME_PREFIX)
    assert BOOT_A_SAME_PREFIX[:8] == BOOT_A[:8], "the fixture must share 8 chars"
    with pytest.raises(ResolvedConfigError):
        load_resolved("p1_motion", root=root, boot_id_path=boot)


def test_the_eight_character_prefix_itself_is_not_accepted(tmp_path):
    """The other direction of the same confusion.

    A freeze line that wrote the envelope-style truncated value into MANIFEST
    would produce a snapshot this reader must refuse -- not silently accept by
    comparing only the prefix of the current value. Together with the case above,
    both readings of "compare boot ids" are pinned.
    """
    root, boot = build_root(tmp_path, boot_id=BOOT_A[:8])
    with pytest.raises(ResolvedConfigError):
        load_resolved("p1_motion", root=root, boot_id_path=boot)


def test_manifest_without_a_boot_id_is_refused(tmp_path):
    """Mutation: read boot_id with .get() and skip the check when it is None => red.

    The .get() form happens to refuse this case too, by comparing None against a
    string -- but only until someone adds an `if manifest_boot:` guard to quieten
    a test, at which point a manifest with no gate at all starts passing.
    """
    root, boot = build_root(tmp_path, drop=("boot_id",))
    with pytest.raises(ResolvedConfigError) as exc:
        load_manifest(root=root, boot_id_path=boot)
    assert "boot_id" in str(exc.value)


def test_unreadable_current_boot_id_is_refused_not_defaulted(tmp_path):
    """*** Fail closed.

    Mutation: return "" when /proc is unreadable => the comparison then fails for
    a second reason and the case still passes, so this asserts the EXCEPTION
    comes from read_boot_id and names the path. A reader returning "" invites the
    repair "well, if it is empty, skip the check", and that repair opens the gate
    precisely on machines where the kernel interface is missing.
    """
    root, _boot = build_root(tmp_path)
    missing = str(tmp_path / "no_such_boot_id")
    with pytest.raises(ResolvedConfigError) as exc:
        load_resolved("p1_motion", root=root, boot_id_path=missing)
    assert missing in str(exc.value)


def test_empty_boot_id_file_is_refused(tmp_path):
    """An empty /proc entry is treated exactly like an unreadable one."""
    root, _boot = build_root(tmp_path)
    empty = tmp_path / "empty_boot"
    empty.write_text("", encoding="utf-8")
    with pytest.raises(ResolvedConfigError):
        load_resolved("p1_motion", root=root, boot_id_path=str(empty))


def test_read_boot_id_strips_the_trailing_newline():
    """The real /proc entry ends in a newline.

    Not stripping it makes the comparison fail on every boot -- the permanently
    red failure -- and the repair someone reaches for is to loosen the
    comparison. Cheap to assert, and it removes the pressure.
    """
    value = read_boot_id()
    assert value == value.strip() and "\n" not in value
    assert len(value) == 36, "a boot id is a UUID in its canonical 36-char form"


# -- ordering: the gate is a gate ---------------------------------------------

def test_the_gate_runs_before_the_snapshot_is_parsed(tmp_path):
    """*** A gate placed after the door is a decoration.

    The snapshot here is unparseable AND the boot id is wrong. A reader that
    loaded content first would raise a YAML error; the assertion is that the
    error is the boot_id one, so the stale snapshot is never parsed and no caller
    ever sees a value from it.

    Mutation: move the boot_id check below the file read => red.
    """
    root, boot = build_root(tmp_path, boot_id=BOOT_B, snapshot="{{{ not yaml")
    with pytest.raises(ResolvedConfigError) as exc:
        load_resolved("p1_motion", root=root, boot_id_path=boot)
    assert "boot" in str(exc.value).lower()
    assert "yaml" not in str(exc.value).lower()


def test_a_prevalidated_manifest_can_be_reused(tmp_path):
    """A process reading two snapshots should not run the gate twice.

    Safe because a Manifest cannot be constructed without passing the gate --
    there is no code path that produces an unvalidated one -- so accepting one
    here is not a way to skip the check.
    """
    root, boot = build_root(tmp_path)
    manifest = load_manifest(root=root, boot_id_path=boot)
    cfg = load_resolved("p1_motion", root=root, boot_id_path=boot, manifest=manifest)
    assert cfg.manifest is manifest


# -- no fallback to the configuration source ----------------------------------

def test_a_missing_snapshot_raises_and_names_the_freeze_service(tmp_path):
    """*** The whole design in one case.

    Mutation: fall back to reading the source when the snapshot is absent => red.
    10 S5.4.1 calls per-process expansion the most critical structural decision
    to avoid, and a fallback reintroduces it exactly when the freeze line has
    failed -- which is when it matters most.

    *** The path assertion below is not decoration, and this test did not have it
    at first. With only "raises, and the message mentions freeze", a fallback
    mutant PASSED: it read the real configs/p1_motion.yaml, which is a
    comment-only skeleton that parses to None, so the reader raised "is empty. An
    empty snapshot is a freeze-line failure" -- a message that happens to contain
    the word. The test was satisfied by a reader that had just done the one thing
    the whole item forbids.

    So the assertion is now on WHICH FILE the failure is about. A fallback cannot
    satisfy that: whatever it reads, the path it reports is under the
    configuration source, and this insists the path is under the resolved root.
    """
    root, boot = build_root(tmp_path, snapshot=None)
    with pytest.raises(ResolvedConfigError) as exc:
        load_resolved("p1_motion", root=root, boot_id_path=boot)
    message = str(exc.value)
    assert "freeze" in message.lower()
    assert root in message, (
        "the failure must be about the snapshot under the resolved root; a "
        "message naming any other file means something else was read"
    )
    assert "configs" not in message, (
        "the reader must not have touched the configuration source at all -- "
        "the presence of that path in the message is the fallback confessing"
    )


def test_a_missing_manifest_raises_and_names_the_freeze_service(tmp_path):
    root, boot = build_root(tmp_path)
    os.remove(os.path.join(root, "MANIFEST.json"))
    with pytest.raises(ResolvedConfigError) as exc:
        load_manifest(root=root, boot_id_path=boot)
    assert "freeze" in str(exc.value).lower()


def test_a_manifest_naming_a_path_outside_the_resolved_root_is_refused(tmp_path):
    """*** The confinement check.

    A manifest is data. One naming a path under the configuration source would
    otherwise turn this reader into the source-reading path the whole item
    forbids -- and it would do so with every test still green, because the load
    would succeed and return a plausible tree.

    Mutation: drop the dirname comparison => red.
    """
    outside = tmp_path / "elsewhere"
    outside.mkdir()
    (outside / "p1_motion.yaml").write_text(SNAPSHOT, encoding="utf-8")
    root, boot = build_root(tmp_path, path_override=str(outside / "p1_motion.yaml"))
    with pytest.raises(ResolvedConfigError) as exc:
        load_resolved("p1_motion", root=root, boot_id_path=boot)
    assert "outside" in str(exc.value)


def test_a_manifest_cannot_be_constructed_around_the_gate(tmp_path):
    """*** This was a real hole, found by an adversarial review of this module.

    load_resolved takes a `manifest=` argument so a process reading two snapshots
    need not run the gate twice. Manifest was freely constructible, so
    load_resolved(manifest=Manifest({...any boot_id...}, "x")) skipped the gate
    outright -- while the class docstring asserted that no such path existed.

    *** The lesson is the general one: a claim written in a comment is not a
    mechanism. The fix is a construction token, and the test is here rather than
    only in the docstring because the docstring is what was wrong.

    Mutation: remove the token check from Manifest.__init__ => red.
    """
    root, boot = build_root(tmp_path)
    with pytest.raises(ResolvedConfigError) as exc:
        R.Manifest({"schema": R.MANIFEST_SCHEMA, "boot_id": BOOT_B,
                    "common_digest": "x", "processes": {}}, "forged")
    assert "load_manifest" in str(exc.value)

    # And the legitimate path still yields one, so the seal did not simply break
    # the feature it protects.
    assert isinstance(load_manifest(root=root, boot_id_path=boot), R.Manifest)


def test_a_malformed_sha256_is_refused_rather_than_skipped(tmp_path):
    """*** The second hole from the same review.

    An earlier draft skipped verification when the recorded value began with the
    ellipsis character, because 10 S5.4.6's EXAMPLE writes the field as an
    ellipsis placeholder. Any manifest carrying that one character therefore
    turned the check off, silently.

    A field that is present and not a well-formed digest is now a malformed
    manifest. Mutation: restore any "looks like a placeholder, skip it" branch
    => red.
    """
    for bad in ("…", "abc", "A" * 64, ""):
        root, boot = build_root(tmp_path / str(abs(hash(bad))), sha256=False)
        manifest_path = os.path.join(root, "MANIFEST.json")
        with open(manifest_path, encoding="utf-8") as fh:
            doc = json.load(fh)
        doc["processes"]["p1_motion"]["sha256"] = bad
        with open(manifest_path, "w", encoding="utf-8") as fh:
            json.dump(doc, fh)
        with pytest.raises(ResolvedConfigError) as exc:
            load_resolved("p1_motion", root=root, boot_id_path=boot)
        assert "sha256" in str(exc.value), "rejected for the right reason: %r" % bad


def test_there_is_no_parameter_that_enables_a_source_fallback():
    """CLAUDE.md 3.6: no switch may exist that skips a safety check.

    Asserted against the signature rather than by behaviour, because the failure
    is someone ADDING such a parameter later with a friendly default. A
    behavioural test cannot see a parameter that does not exist yet; this can.
    """
    import inspect

    names = set(inspect.signature(load_resolved).parameters)
    for banned in ("fallback", "allow_source", "source_root", "strict",
                   "skip_boot_check", "ignore_boot_id"):
        assert banned not in names, (
            "load_resolved must not offer %r; a parameter that relaxes the gate "
            "is a remote switch for the guarantee this module exists to provide"
            % banned
        )


# -- manifest integrity --------------------------------------------------------

def test_a_foreign_manifest_schema_is_refused(tmp_path):
    """Refused rather than read on a best-effort basis.

    A manifest from a later generation may put different meaning behind the same
    field names, and reading it partially is how a reader ends up confidently
    wrong instead of loudly stuck.
    """
    root, boot = build_root(tmp_path,
                            manifest_extra={"schema": "xbrain.config.manifest/2"})
    with pytest.raises(ResolvedConfigError) as exc:
        load_manifest(root=root, boot_id_path=boot)
    assert "schema" in str(exc.value)


def test_malformed_manifest_json_is_refused(tmp_path):
    root, boot = build_root(tmp_path)
    with open(os.path.join(root, "MANIFEST.json"), "w", encoding="utf-8") as fh:
        fh.write("{not json")
    with pytest.raises(ResolvedConfigError):
        load_manifest(root=root, boot_id_path=boot)


def test_a_process_absent_from_the_manifest_is_refused_and_lists_what_is_there(tmp_path):
    """The realistic cause is a freeze line running against a stale process list.

    That is the 进程在册 - 配置不在册 state 10 S5.4.6 repaired for quadruped, and
    from the failing process's side it is invisible unless the message says what
    the manifest DID contain.
    """
    root, boot = build_root(tmp_path, proc="p2_core")
    with pytest.raises(ResolvedConfigError) as exc:
        load_resolved("p1_motion", root=root, boot_id_path=boot)
    assert "p2_core" in str(exc.value), "the message must list what was present"


def test_quadruped_is_a_snapshot_process(tmp_path):
    """*** 10 S5.4.6 added it in v0.7.7 to repair a real gap.

    Before that the process was registered and its configuration was not, so
    freeze produced no snapshot and quadruped had to expand ${common.spec.*}
    itself -- breaking S5.4.1 silently. A reader whose process set omitted it
    would recreate the gap on the reader side.
    """
    assert "quadruped" in R.SNAPSHOT_PROCESSES
    root, boot = build_root(tmp_path, proc="quadruped")
    assert load_resolved("quadruped", root=root, boot_id_path=boot).proc == "quadruped"


def test_an_unknown_process_name_is_refused(tmp_path):
    """Refused here rather than turned into a file-not-found for a path nobody
    meant to build. The two need different fixes."""
    root, boot = build_root(tmp_path)
    with pytest.raises(ResolvedConfigError) as exc:
        load_resolved("perception", root=root, boot_id_path=boot)
    assert "perception" in str(exc.value)


def test_a_modified_snapshot_fails_its_recorded_sha256(tmp_path):
    """CFG-ROOT-5: editing a file under /run does not change the configuration.

    It rolls back silently at the next boot, so the useful behaviour is to catch
    it now rather than to let someone believe an edit took effect.
    """
    root, boot = build_root(tmp_path)
    with open(os.path.join(root, "p1_motion.yaml"), "a", encoding="utf-8") as fh:
        fh.write("loop_hz: 999\n")
    with pytest.raises(ResolvedConfigError) as exc:
        load_resolved("p1_motion", root=root, boot_id_path=boot)
    assert "sha256" in str(exc.value)


def test_an_absent_sha256_is_tolerated(tmp_path):
    """Pairs with the test above.

    10 S5.4.6 shows the field in every row but never says a consumer must verify
    it, so refusing to start when it is absent would enforce a rule the document
    does not state. Present-and-wrong is a different matter and is refused.
    """
    root, boot = build_root(tmp_path, sha256=False)
    assert load_resolved("p1_motion", root=root, boot_id_path=boot).get("loop_hz") == 20


# -- what the reader deliberately does not do ---------------------------------

def test_the_reader_does_not_compare_common_digest(tmp_path):
    """*** Different failure severity, therefore not this module's decision.

    10 S3.3.3 puts the common_digest comparison at P2's Stage D, and its failure
    behaviour is B -- keep motion disallowed and emit event/fault/bit with
    detail.kind config_digest_mismatch -- not the R this module applies. A reader
    that refused to start on a digest difference would apply the wrong severity
    to a condition the design wants observable and reported.

    So: the value is exposed, and loading succeeds whatever it says.
    """
    root, boot = build_root(tmp_path,
                            manifest_extra={"common_digest": "0000000000000000"})
    cfg = load_resolved("p1_motion", root=root, boot_id_path=boot)
    assert cfg.manifest.common_digest == "0000000000000000"


def test_the_reader_does_not_expand_references(tmp_path):
    """Expansion happened at the freeze line and must not happen again.

    A snapshot still holding ${common.x} means the freeze line did not finish its
    job; re-expanding here would hide that AND would be the per-process expansion
    10 S5.4.1 forbids. The value comes back as the literal string, so a caller
    that cares can see it.
    """
    root, boot = build_root(tmp_path, snapshot="a: ${common.spec.max_vx_mps}\n")
    cfg = load_resolved("p1_motion", root=root, boot_id_path=boot)
    assert cfg.get("a") == "${common.spec.max_vx_mps}"


# -- accessors -----------------------------------------------------------------

def test_get_returns_MISSING_not_None_for_an_absent_key(tmp_path):
    """null is a real value here -- declared but unassigned -- so collapsing the
    two would lose the distinction the whole layer design rests on."""
    root, boot = build_root(tmp_path)
    cfg = load_resolved("p1_motion", root=root, boot_id_path=boot)
    assert cfg.get("nope.not.here") is MISSING
    assert cfg.get("teleop.cloud.enabled") is None


def test_require_refuses_a_null_and_says_assertion_A_should_have_caught_it(tmp_path):
    """CLAUDE.md 3.1: safety parameters have no default fallback.

    require() exists so the shape available to a caller is "give me this or
    fail". If the only accessor took a default, someone would pass one.
    """
    root, boot = build_root(tmp_path)
    cfg = load_resolved("p1_motion", root=root, boot_id_path=boot)
    with pytest.raises(ResolvedConfigError) as exc:
        cfg.require("teleop.cloud.enabled")
    assert "assertion A" in str(exc.value)
    assert cfg.require("loop_hz") == 20


def test_unassigned_lists_nulls_and_not_falsy_values(tmp_path):
    """0, "" and [] are assigned values. Reporting them as holes would send
    someone to fill in a key that is already correct."""
    root, boot = build_root(
        tmp_path, snapshot="a: null\nb: 0\nc: ''\nd: []\ne: false\n")
    cfg = load_resolved("p1_motion", root=root, boot_id_path=boot)
    assert cfg.unassigned() == ["a"]


def test_an_empty_snapshot_is_a_freeze_failure_not_an_empty_config(tmp_path):
    root, boot = build_root(tmp_path, snapshot="")
    with pytest.raises(ResolvedConfigError) as exc:
        load_resolved("p1_motion", root=root, boot_id_path=boot)
    assert "empty" in str(exc.value)


def test_the_resolved_root_is_not_under_the_configuration_root():
    """CFG-ROOT-5: source and product are two different places.

    Asserted because the constant is the one thing that, changed, would make
    every other test here pass while the reader read the source.
    """
    # 2026-08-10: relocated from /run/xbrain/resolved to
    # /opt/xbrain_v6/data/run/resolved per user directive that all V6
    # runtime dependencies live under /opt/xbrain_v6/. The invariant
    # this test defends is still 'RESOLVED_ROOT is not under configs/'
    # (source vs product separation), just checked at the new location.
    assert R.RESOLVED_ROOT == "/opt/xbrain_v6/data/run/resolved"
    assert "configs" not in R.RESOLVED_ROOT
    assert "/data/run/" in R.RESOLVED_ROOT
