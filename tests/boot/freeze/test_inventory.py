"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_inventory.py
Brief: CFG-CM-10 -- MANIFEST.layers[] / config_rev / common_digest are computed

Description:
Guards the property the freeze line gained at CFG-CM-10: the two CFG-41
identities are COMPUTED from the tree that was actually resolved, not supplied
by whoever called run_freeze. Before this, xbrain/boot/freeze/__main__.py
passed the literal "stub-not-yet-computed" and tests/fixtures/conftest.py
passed "fixture-digest"; every comparison against MANIFEST.common_digest
therefore compared a constant with itself, which is CLAUDE.md S3.2 form 1.

The cases are chosen around the ONE distinction that is easy to get wrong and
expensive to get wrong: common_digest and config_rev have different scopes on
purpose (10 S5.4.4 verbatim "[*] 只覆盖 common.*, 不含进程私有段 -- 私有段变更
不应阻塞放行"). A test suite that only checked "the digest is sixteen hex
characters" would pass for an implementation that hashed the whole tree, and
that implementation would make every per-process edit hold the robot at
Stage C.

The meta-case (test_layer_rows_match_what_the_loader_opens) is the one that
keeps the rest honest. layer_rows() re-walks the config tree independently of
_layer_loader; this case OBSERVES which paths load_layers really opens and
compares. Wiring _read_dir to call layer_rows instead would make the same
statement unfalsifiable (CLAUDE.md S3.2 form 7).
"""

import json
import os

import pytest
import yaml

from tests.boot.freeze.test_registry import _scaffold_config_ctx
from xbrain.boot.freeze.assertions import _layer_loader
from xbrain.boot.freeze.inventory import config_rev_of, layer_rows
from xbrain.boot.freeze.pipeline import run_freeze
from xbrain.common.digest.digest import DIGEST_HEX_LEN


def _freeze(ctx):
    """Run a full freeze over a scaffolded tree; return the manifest."""
    # skip_files propagates the scaffold's S10 exemption (its common.yaml is
    # minimal on purpose). Everything else is the production call shape --
    # in particular there is no way to pass a digest in, which is the point.
    return run_freeze(
        boot_id="inventory-test",
        config_root=ctx["config_root"],
        config_root_overridden=False,
        resolved_root=ctx["resolved_root"],
        context={"skip_files": list(ctx.get("skip_files", []))},
    )


def _paths_at(rows, level):
    """The `path` values of every row at one layer level, in row order."""
    return [r["path"] for r in rows if r["level"] == level]


# --------------------------------------------------------------------------
# The meta-case: the enumerator and the loader must agree on the file set
# --------------------------------------------------------------------------

def test_layer_rows_match_what_the_loader_opens(tmp_path):
    """Every file load_layers opens has a row, and no row names a file it did
    not open.

    Mutant: drop the `name.startswith("_")` skip in _base_yaml_files, so a
    _skeleton.yaml placeholder gets a row the loader never read -> red here.
    Mutant: drop the variant-sibling lookup in _with_sibling -> the sim file
    is opened but unlisted -> red here.
    """
    ctx = _scaffold_config_ctx(tmp_path)
    root = ctx["config_root"]
    # Populate the directory layers so the walk has something to disagree
    # about: a real base file, a placeholder the loader skips, a variant
    # sibling only a --variant run reads, and a non-yaml file.
    models = os.path.join(root, "models")
    open(os.path.join(models, "m20s.yaml"), "w").write("common: {}\n")
    open(os.path.join(models, "_skeleton.yaml"), "w").write("common: {}\n")
    open(os.path.join(models, "m20s_sim.yaml"), "w").write("common: {}\n")
    open(os.path.join(models, "README.md"), "w").write("not yaml\n")

    opened = []
    real_read = _layer_loader._read_yaml

    def _recording_read(path):
        # Record every path the loader ASKS for, then filter to the ones that
        # exist: the loader probes common.yaml unconditionally and treats a
        # miss as {}, and an absent file is deliberately not a layer row.
        opened.append(os.path.abspath(path))
        return real_read(path)

    for variant in (None, "sim"):
        opened.clear()
        _layer_loader._read_yaml = _recording_read
        try:
            _layer_loader.load_layers(root, variant=variant)
        finally:
            _layer_loader._read_yaml = real_read
        really_opened = sorted({p for p in opened if os.path.isfile(p)})
        rows = layer_rows(root, variant=variant, env={})
        listed = sorted(r["path"] for r in rows if r["level"] in ("L1", "L2", "L3"))
        assert listed == really_opened, (
            "variant=%r: layer_rows and load_layers disagree on the file set"
            % variant
        )


def test_the_picked_layers_name_the_files_the_merge_consumed(tmp_path):
    """L4 / L4b rows follow site_id / robot_id, and are absent when the
    picker value is unset.

    Mutant: read the picker from the raw L1 file instead of the resolved
    tree -> an L5 override would list the wrong site file -> red here.
    """
    ctx = _scaffold_config_ctx(tmp_path)
    root = ctx["config_root"]
    calib = os.path.join(root, "calib")
    open(os.path.join(calib, "robot-7.yaml"), "w").write("common: {}\n")

    # No picker values -> no picked rows at all.
    bare = layer_rows(root, env={})
    assert _paths_at(bare, "L4") == []
    assert _paths_at(bare, "L4b") == []

    picked = layer_rows(root, site_id="site_scaffold", robot_id="robot-7", env={})
    assert _paths_at(picked, "L4") == [
        os.path.join(root, "sites", "site_scaffold.yaml")]
    assert _paths_at(picked, "L4b") == [os.path.join(calib, "robot-7.yaml")]

    # A picker naming a file that is not there yields no row rather than a
    # row with a fabricated hash -- _load_l4_tree returns {} for that case,
    # so a row would claim a file was read that was not.
    absent = layer_rows(root, site_id="no_such_site", env={})
    assert _paths_at(absent, "L4") == []


def test_locked_marks_safety_and_calib_only(tmp_path):
    """10 S5.4.4 marks L3 and L4b `locked: true`; nothing else carries it.

    Mutant: add "L1" to _LOCKED_LEVELS -> red. Mutant: drop the flag
    entirely -> red.
    """
    ctx = _scaffold_config_ctx(tmp_path)
    root = ctx["config_root"]
    open(os.path.join(root, "safety", "clock.yaml"), "w").write("common: {}\n")
    open(os.path.join(root, "calib", "r1.yaml"), "w").write("common: {}\n")
    rows = layer_rows(root, site_id="site_scaffold", robot_id="r1", env={})
    locked = {r["level"] for r in rows if r.get("locked")}
    assert locked == {"L3", "L4b"}
    # Full sha256, not the 16-char truncation common_digest uses: a layer row
    # is meant to be pasteable into `sha256sum -c`. Mutant: truncate to
    # DIGEST_HEX_LEN -> red.
    for row in rows:
        if "sha256" in row:
            assert len(row["sha256"]) == 64, row


def test_the_env_row_records_names_and_never_values(tmp_path):
    """L5 is `{level, path: "env", keys: [...]}` -- names only.

    A value here would put environment content into a file that is uploaded
    with incident reports (MED-S5, 11 S7.4.9). Mutant: emit the value beside
    the name -> red.
    """
    ctx = _scaffold_config_ctx(tmp_path)
    # Two whitelisted names, handed in reverse order, so the sorted() in
    # _env_row has something to do -- with one name a `list()` mutant would
    # survive. PATH is there to show a non-XBRAIN variable is not recorded.
    rows = layer_rows(ctx["config_root"],
                      env={"XBRAIN_SITE_ID": "site_secret_name",
                           "XBRAIN_LOG_LEVEL": "DEBUG",
                           "PATH": "/usr/bin"})
    env_rows = [r for r in rows if r["level"] == "L5"]
    assert len(env_rows) == 1
    row = env_rows[0]
    assert row["path"] == "env"
    assert row["keys"] == ["XBRAIN_LOG_LEVEL", "XBRAIN_SITE_ID"]
    assert "site_secret_name" not in json.dumps(row)
    assert "DEBUG" not in json.dumps(row)
    # sha256 is meaningless for a non-file row and must not be invented.
    assert "sha256" not in row


# --------------------------------------------------------------------------
# The two identities: same input, different scope
# --------------------------------------------------------------------------

class _RecordingEnv(dict):
    """A dict that counts how many times something iterated it."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.iterations = 0

    def __iter__(self):
        self.iterations += 1
        return super().__iter__()


def test_the_env_row_walks_the_environment_not_the_whitelist(tmp_path):
    """_env_row must iterate the ENVIRONMENT and filter by the whitelist.

    This asserts a spelling rather than an output, which is unusual and is
    done for a specific reason: the property at stake -- that config_rev is
    identical on two boots from an unchanged configuration -- is NOT
    observable inside one process. ENV_WHITELIST is a frozenset of str, and
    str hashing is seeded per process, so iterating IT before sorting yields
    an order that varies between boots. A mutant that walks the whitelist and
    drops the sort therefore passes or fails depending on PYTHONHASHSEED; it
    survived a full mutant run here on the seed of the day.

    Iterating `source` puts the pre-sort order in the caller's hands, which is
    what makes test_the_env_row_records_names_and_never_values able to kill
    the plain "drop the sorted()" mutant. This case guards the half of that
    arrangement the output cannot show.
    """
    env = _RecordingEnv({"XBRAIN_SITE_ID": "s", "PATH": "/usr/bin"})
    layer_rows(_scaffold_config_ctx(tmp_path)["config_root"], env=env)
    assert env.iterations >= 1, (
        "_env_row iterated ENV_WHITELIST instead of the environment; the "
        "pre-sort order then depends on PYTHONHASHSEED (see docstring)")


def test_a_common_edit_moves_both_identities(tmp_path):
    """Changing a value under common.* moves common_digest AND config_rev.

    Mutant: hash a constant instead of the tree -> red.
    """
    ctx = _scaffold_config_ctx(tmp_path)
    before = _freeze(ctx)
    common_path = os.path.join(ctx["config_root"], "common.yaml")
    tree = yaml.safe_load(open(common_path, encoding="utf-8"))
    # log_level is a plain string in the green scaffold tree; bumping it is a
    # real common.* change with no effect on any assertion.
    tree["common"]["log_level"] = "DEBUG"
    open(common_path, "w", encoding="utf-8").write(
        yaml.safe_dump(tree, allow_unicode=True))
    after = _freeze(ctx)
    assert after["common_digest"] != before["common_digest"]
    assert after["config_rev"] != before["config_rev"]


def test_a_comment_only_edit_moves_config_rev_but_not_common_digest(tmp_path):
    """The scope split, stated as the case that separates the two identities.

    A comment changes the file and changes nothing the system reads. config_rev
    is the audit trail and must see it; common_digest gates motion and must
    NOT, or every comment would hold the robot at Stage C.

    Mutant: make config_rev hash the PARSED tree -> it stops seeing the
    comment -> red on the first assert. Mutant: make common_digest hash the
    raw file bytes -> red on the second.
    """
    ctx = _scaffold_config_ctx(tmp_path)
    before = _freeze(ctx)
    common_path = os.path.join(ctx["config_root"], "common.yaml")
    with open(common_path, "a", encoding="utf-8") as fh:
        fh.write("# an operator left a note here\n")
    after = _freeze(ctx)
    assert after["config_rev"] != before["config_rev"]
    assert after["common_digest"] == before["common_digest"]


def test_a_per_process_edit_moves_its_snapshot_only(tmp_path):
    """10 S5.4.4 verbatim: common_digest covers common.* only, so a private
    per-process change must not move it -- "私有段变更不应阻塞放行".

    The per-process identity is MANIFEST.processes[proc].sha256 over the
    snapshot bytes, and THAT must move. Mutant: fold the per-proc trees into
    common_digest -> red on the common_digest assert.
    """
    ctx = _scaffold_config_ctx(tmp_path)
    proc_path = os.path.join(ctx["config_root"], "p2_core.yaml")
    open(proc_path, "w", encoding="utf-8").write(
        yaml.safe_dump({"p2_core": {"tick_hz": 20}}, allow_unicode=True))
    before = _freeze(ctx)
    assert "p2_core" in before["processes"], (
        "scaffold must materialise p2_core for this case to mean anything")
    open(proc_path, "w", encoding="utf-8").write(
        yaml.safe_dump({"p2_core": {"tick_hz": 25}}, allow_unicode=True))
    after = _freeze(ctx)
    assert (after["processes"]["p2_core"]["sha256"]
            != before["processes"]["p2_core"]["sha256"])
    assert after["common_digest"] == before["common_digest"]
    # config_rev covers the L1..L5 overlay axis only; an L6 edit is outside
    # it by design (see inventory.py "Scope boundary"). Asserted so the
    # boundary is pinned rather than assumed.
    assert after["config_rev"] == before["config_rev"]


def test_freeze_is_idempotent_on_an_unchanged_tree(tmp_path):
    """INF-DP-5 criterion (i): two freezes of one source tree agree byte for
    byte on the identities and on every snapshot hash.

    Mutant: put the wall clock into config_rev -> red. Mutant: drop sorted()
    from the directory walk or from the env keys -> red on a tree where the
    filesystem hands names back in a different order.
    """
    ctx = _scaffold_config_ctx(tmp_path)
    first = _freeze(ctx)
    second = _freeze(ctx)
    assert first["common_digest"] == second["common_digest"]
    assert first["config_rev"] == second["config_rev"]
    assert first["layers"] == second["layers"]
    assert first["processes"] == second["processes"]


def test_the_identities_are_not_placeholders(tmp_path):
    """The shape check that would have caught the pre-CFG-CM-10 state.

    Deliberately narrow: it asserts the VALUES have the computed shape, which
    "stub-not-yet-computed" and "fixture-digest" both fail. It is not a
    substitute for the scope cases above -- a constant of the right shape
    would pass here and fail every one of them.
    """
    ctx = _scaffold_config_ctx(tmp_path)
    manifest = _freeze(ctx)
    digest = manifest["common_digest"]
    assert len(digest) == DIGEST_HEX_LEN
    assert all(c in "0123456789abcdef" for c in digest), digest
    rev = manifest["config_rev"]
    assert rev.startswith("layers-")
    assert len(rev) == len("layers-") + DIGEST_HEX_LEN
    # layers[] is no longer the empty list the framework shipped with.
    assert manifest["layers"], "MANIFEST.layers[] must not be empty"


def test_the_manifest_names_the_picked_layer_files(tmp_path):
    """A full freeze -- not a direct layer_rows() call -- must record the L4
    and L4b files the merge consumed.

    The direct-call case above pins layer_rows; this one pins the WIRING, and
    the two are not the same statement. Mutant: have materialise pass
    site_id=None / robot_id=None (the shape a careless "the picker is not
    resolved yet" refactor produces) -> layer_rows is still correct, the
    direct case is still green, and the MANIFEST silently loses both rows.
    """
    ctx = _scaffold_config_ctx(tmp_path)
    # The scaffold's common.yaml sets robot_id gj-001; give L4b a file so the
    # row has something to name. site_scaffold.yaml is already there.
    open(os.path.join(ctx["config_root"], "calib", "gj-001.yaml"),
         "w", encoding="utf-8").write("common: {}\n")
    manifest = _freeze(ctx)
    assert [os.path.basename(p) for p in _paths_at(manifest["layers"], "L4")] \
        == ["site_scaffold.yaml"]
    assert [os.path.basename(p) for p in _paths_at(manifest["layers"], "L4b")] \
        == ["gj-001.yaml"]


def test_config_rev_is_a_pure_function_of_the_rows():
    """Same rows -> same rev; one changed field -> different rev.

    Deliberately NOT a claim about the {"layers": ...} wrapper: hashing the
    bare list would produce different values but satisfy both properties
    below, so no case here can distinguish the two and writing one that
    pretended to would be CLAUDE.md S3.2 form 1. The wrapper is a
    forward-compatibility choice recorded in the function docstring, not a
    tested behaviour."""
    rows = [{"level": "L1", "path": "/x/common.yaml", "sha256": "ab"}]
    other = [{"level": "L1", "path": "/x/common.yaml", "sha256": "cd"}]
    assert config_rev_of(rows) == config_rev_of(list(rows))
    assert config_rev_of(rows) != config_rev_of(other)


# --------------------------------------------------------------------------
# No injection path
# --------------------------------------------------------------------------

def test_run_freeze_refuses_when_the_materialiser_did_not_publish(tmp_path):
    """run_freeze must raise rather than fall back to a placeholder when ctx
    carries no identities -- the failure the whole item is about.

    Mutant: restore a `common_digest` parameter with a default -> red, because
    the call below would then succeed and write a MANIFEST whose digest says
    nothing.
    """
    resolved = tmp_path / "resolved"
    resolved.mkdir()
    # An empty registry: run_assertions walks ASSERT_REGISTRY, so patching the
    # module's tuple to () makes every runner -- including materialise --
    # not run, which is exactly the wiring bug the guard names.
    from xbrain.boot.freeze import pipeline as pipeline_mod
    real_registry = pipeline_mod.ASSERT_REGISTRY
    pipeline_mod.ASSERT_REGISTRY = ()
    try:
        with pytest.raises(AssertionError, match="materialise"):
            run_freeze(
                boot_id="no-materialiser",
                config_root=str(tmp_path),
                config_root_overridden=False,
                resolved_root=str(resolved),
            )
    finally:
        pipeline_mod.ASSERT_REGISTRY = real_registry


@pytest.mark.parametrize("dropped", ["common_digest", "config_rev", "layers",
                                     "robot_id", "site_id"])
def test_run_freeze_refuses_on_each_missing_identity(tmp_path, dropped):
    """The guard is per-key, and each key needs its own case.

    A single case that drops the whole ctx only exercises whichever key the
    loop checks first -- the other four could be removed from the guard and
    nothing would go red. That is exactly what a mutant run showed: deleting
    robot_id / site_id from the guard left the suite green, because
    materialise always publishes them and no case looked at that half.

    Mutant: shorten the guard tuple by any one key -> the matching parameter
    goes red, and only that one, which also proves the case is testing the key
    it names.
    """
    from xbrain.boot.freeze import pipeline as pipeline_mod
    from xbrain.boot.freeze.registry import AssertSpec

    published = {"common_digest": "0" * DIGEST_HEX_LEN,
                 "config_rev": "layers-" + "0" * DIGEST_HEX_LEN,
                 "layers": [], "robot_id": "gj-001", "site_id": "site_x",
                 "processes": {}}

    def _partial_materialiser(runner_ctx):
        # Publishes everything a real materialiser publishes EXCEPT one key --
        # the shape a partial implementation has. Patching the real runner
        # would not work: registry.py binds materialise.run into a frozen
        # AssertSpec at import, so rebinding the module attribute afterwards
        # changes nothing the pipeline calls.
        for key, value in published.items():
            if key != dropped:
                runner_ctx[key] = value
        return {"status": "pass", "assertion": "partial"}

    resolved = tmp_path / "resolved"
    resolved.mkdir()
    real_registry = pipeline_mod.ASSERT_REGISTRY
    pipeline_mod.ASSERT_REGISTRY = (
        AssertSpec("partial", "publishes all identities but one",
                   runner=_partial_materialiser),)
    try:
        with pytest.raises(AssertionError, match=dropped):
            run_freeze(boot_id="partial", config_root=str(tmp_path),
                       config_root_overridden=False,
                       resolved_root=str(resolved))
    finally:
        pipeline_mod.ASSERT_REGISTRY = real_registry


def test_run_freeze_has_no_digest_parameter(tmp_path):
    """A caller cannot hand in a digest. Stated as a signature case because
    the parameter is what would come back in a careless merge -- and it would
    come back silently, since every existing call site would keep working."""
    import inspect
    params = inspect.signature(run_freeze).parameters
    for name in ("common_digest", "config_rev", "layers"):
        assert name not in params, (
            "run_freeze must not accept %r: a caller-supplied identity makes "
            "10 S5.4.4's Stage C/D comparison agree with itself" % name)


# --------------------------------------------------------------------------
# The MANIFEST top-level fields 10 S5.4.4 lists (landed 2026-09-18)
# --------------------------------------------------------------------------

def test_the_manifest_names_the_robot_and_the_site(tmp_path):
    """"Whose MANIFEST is this" is where an incident starts, and it must be
    answerable without opening a snapshot.

    Both values are also inside common_digest (they are common.* leaves), so
    this is a readability lift, not a second source -- which is why they are
    taken from the RESOLVED tree, the same place the digest hashes and the same
    place the L4 row's filename comes from. Mutant: read them from the raw L1
    file -> an L5 override makes the MANIFEST name a different site than the
    L4 row it also carries, and the file contradicts itself.
    """
    ctx = _scaffold_config_ctx(tmp_path)
    manifest = _freeze(ctx)
    assert manifest["robot_id"] == "gj-001"
    assert manifest["site_id"] == "site_scaffold"
    # The L4 row was picked BY site_id, so the two must agree inside one file.
    assert [os.path.basename(p) for p in _paths_at(manifest["layers"], "L4")] \
        == ["%s.yaml" % manifest["site_id"]]


def test_gen_ts_is_present_and_is_not_compared_by_check(tmp_path):
    """10 S5.4.4 verbatim: gen_ts is 墙钟, 仅供人看.

    So it must be IN the file (an archived MANIFEST with no idea when it was
    produced is much less useful) and must NOT be part of any comparison --
    it moves on every run, and comparing it would make freeze --check report
    drift on a tree nobody touched.

    Mutant: compare gen_ts in check_freeze -> the no-drift case in
    test_check.py goes red. Mutant: drop the field -> red here.
    """
    ctx = _scaffold_config_ctx(tmp_path)
    first = _freeze(ctx)
    second = _freeze(ctx)
    assert isinstance(first["gen_ts"], float)
    assert first["gen_ts"] > 0.0
    # Two freezes of one tree: gen_ts is the ONLY top-level field allowed to
    # differ. Asserted as a set difference rather than field by field, so a
    # future field that wrongly carries a clock is caught by this case too.
    differing = {k for k in first
                 if k != "assertions" and first[k] != second.get(k)}
    assert differing <= {"gen_ts"}, differing


def test_each_process_row_records_how_many_shared_values_it_pulls(tmp_path):
    """MANIFEST.processes[*].refs (10 S5.4.4, CFG-41).

    It answers what the sha256 cannot: a snapshot whose refs count moved had
    its COUPLING to the shared layer changed -- someone replaced a
    ${common.*} with a hardcoded number, or the reverse. The bytes moving says
    only that something moved.

    Mutant: count on the EXPANDED tree -> refs becomes the leaf count (2 here,
    then 2 again after the ref is inlined) -> red. Mutant: omit the field ->
    red.
    """
    ctx = _scaffold_config_ctx(tmp_path)
    proc_path = os.path.join(ctx["config_root"], "p2_core.yaml")
    open(proc_path, "w", encoding="utf-8").write(yaml.safe_dump(
        {"p2_core": {"a_max": "${common.spec.max_decel_mps2}", "tick_hz": 20}},
        allow_unicode=True))
    before = _freeze(ctx)
    assert before["processes"]["p2_core"]["refs"] == 1
    # Inline the shared value: same key count, same kind of snapshot, but the
    # process no longer follows common.*. refs is the field that notices.
    open(proc_path, "w", encoding="utf-8").write(yaml.safe_dump(
        {"p2_core": {"a_max": 2.5, "tick_hz": 20}}, allow_unicode=True))
    after = _freeze(ctx)
    assert after["processes"]["p2_core"]["refs"] == 0
    assert after["common_digest"] == before["common_digest"], (
        "inlining a shared value into a per-proc file must not move the "
        "shared digest (10 S5.4.4: 私有段变更不应阻塞放行)")
