"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: check.py
Brief: freeze --check -- re-read the sources, recompute, compare, write nothing

Description:
Answers one operational question that had no answer before: an operator edited
configs/common.yaml after the machine booted -- is the running system using
that edit, or the snapshot from before it?

The structural guarantee (10 S5.4.4) is that it is NOT: every unit carries
Requires=xbrain-config-freeze.service, /run is tmpfs, and MANIFEST.boot_id is
checked at R level, so a snapshot can never outlive a boot. What that
guarantee does not cover is the case with no reboot in it: edit a source,
leave the machine running, and every process keeps the values it loaded. The
system is behaving exactly as designed and the operator's edit is inert. There
is nothing to detect structurally, because nothing is wrong -- the question is
purely "does what is loaded still match what is on disk", and that needs a
comparison somebody actually runs.

This module is that comparison, and it is deliberately REPORT-ONLY:
  * it writes nothing into resolved_root. A --check that quietly refreshed the
    snapshot would change the running system's configuration underneath
    processes that had already read it -- some would hold old values and some
    new, which is worse than a stale-but-consistent set and is the exact
    "各进程解析出不同结果" outcome 10 S5.4.1 is built to prevent.
  * it never edits a source. Iron rule 3 (CLAUDE.md): a null is a null.
The only correct response to drift is a restart of the stack, which re-runs the
freeze. 10 S10.3 says the same thing for the safety layer -- "common.* 无热更新
语义, 改了必须整栈重启以重算 common_digest".

How the fresh side is computed: by running a REAL freeze into a temporary
directory, not by a parallel re-implementation that "just computes the
digests". A second implementation would be the thing that drifts -- it would
agree with the freeze line right up until someone changed one of them, and the
disagreement would surface as a false alarm nobody could explain. The cost is a
temp directory and one extra assertion pass; the benefit is that --check
cannot be wrong about what a freeze would produce, because it ran one.

Reading MANIFEST.json here uses plain json rather than
xbrain.common.config.resolved.load_manifest, and that is a real difference in
contract, not a shortcut: load_manifest runs the boot_id gate and REFUSES
(R level) on a stale snapshot, which is right for a process about to run on
those values. --check has to survive that case in order to REPORT it, plus
everything else it finds. A diagnostic that aborts on the first problem tells
an operator one thing when they need the list.

Worked example, because the finding kinds are easy to confuse and the right
response differs for each. An operator edits configs/common.yaml on a running
machine and runs the check:

  * they only reformatted or added a comment
        -> layer_changed + config_rev_mismatch, nothing else.
        The parsed values did not move, so no snapshot would change and no
        process is running anything different. No restart needed. Knowing
        this is worth the check on its own: the alternative is scheduling an
        outage for an edit that changed nothing.
  * they changed a value under common.*
        -> layer_changed + config_rev_mismatch + common_digest_mismatch,
        plus snapshot_stale for every process whose snapshot embeds that
        value. Restart the stack; 10 S10.3 says common.* has no hot-update
        semantics, so there is no partial path.
  * they changed a value in configs/p2_core.yaml
        -> snapshot_stale for p2_core, and NO common_digest_mismatch. That
        asymmetry is 10 S5.4.4's "私有段变更不应阻塞放行" and it is the
        difference between restarting one unit and restarting everything.
  * they edited data/run/resolved/p2_core.yaml directly
        -> snapshot_tampered, and no layer finding at all. The edit is about
        to vanish at the next boot (the directory is regenerated), so it has
        to be moved into configs/. This is iron rule 2 becoming visible
        instead of being discovered later as an unexplained rollback.
  * they changed nothing and the machine rebooted without the freeze running
        -> boot_id_stale, which is also why every process is refusing to
        start. The refusal is correct; the value here is that it is explained.

Relationship to the P2-side check (10 S3.3.3 Stage A/C/D): that one compares
P2's OWN cached digest against MANIFEST and holds motion on a mismatch. It
catches a different fault -- the freeze line running again mid-startup, so
that P2's cached value and the MANIFEST on disk describe different passes.
Neither check subsumes the other: this one compares the snapshot against the
SOURCES, that one compares one process's memory against the SNAPSHOT.

Not run at boot, deliberately. At boot the freeze has just run, so the answer
is known and the extra assertion pass would be pure latency on the Stage 0c
critical path. This is an operator tool for the machine that has been up for
a week.

What it does NOT check: whether any process actually read the snapshot, or
when. That is not knowable from the filesystem, and a check that guessed
(by mtime, by process start time) would be confidently wrong on a process
that reloaded, or one started before the last freeze. The report says what the
FILES say; who has what in memory is a question for the running processes.
"""

import json
import os
import shutil
import tempfile
from typing import Any, Dict, List, Optional, Tuple

from xbrain.boot.freeze.inventory import file_sha256
from xbrain.boot.freeze.pipeline import run_freeze
from xbrain.common.errors.exceptions import XbrainError

#: The findings this module can emit. A closed set (CLAUDE.md 3.5) so a caller
#: can branch on kind without string-matching a message, and so a new condition
#: has to be named here rather than smuggled in as free text.
FINDING_KINDS: Tuple[str, ...] = (
    # No MANIFEST at all, or one that will not parse. Everything downstream is
    # unanswerable, so this is the only finding when it fires.
    # What to do: the freeze unit did not run or did not finish. Look at
    # `systemctl status xbrain-config-freeze`; do NOT create the file by hand,
    # which is what CFG-ROOT-5 warns about (the edit rolls back next boot).
    "manifest_missing",
    "manifest_unreadable",
    # The snapshot belongs to a previous boot. 10 S5.4.4 makes this an R-level
    # refusal for a PROCESS; here it is reported, because the operator asking
    # --check is entitled to hear it rather than to watch the tool abort.
    # What to do: /run did not get cleared, so the mount is not tmpfs or
    # something recreated the directory. Every process would refuse to start on
    # this anyway -- the value of seeing it here is that it explains WHY they
    # refuse, which "R class refusal" on its own does not.
    "boot_id_stale",
    # The sources do not freeze at all right now (a null that has to be filled,
    # a broken reference). Nothing can be compared, because there is no "what
    # a freeze would produce" to compare against.
    # What to do: read the key path in the detail and fill that value in
    # configs/. On the current tree this is the EXPECTED outcome -- 18 of the
    # 19 yaml files are still key-position skeletons (CLAUDE.md 3.1), so a
    # --check today reports this and nothing else. Iron rule 3: do not put a
    # number there to make the check pass.
    "sources_refuse_freeze",
    # The shared configuration changed. This is the one that gates motion:
    # 10 S5.4.4 has P2 hold "禁止运动" on a mismatch at Stage C/D.
    # What to do: restart the stack. 10 S10.3 is explicit that common.* has no
    # hot-update semantics and that a change requires a full restart so the
    # digest is recomputed; there is no partial path that is safe, because
    # half the processes would hold the old values.
    "common_digest_mismatch",
    # The audit identity changed. Implied by any layer_* finding below; kept
    # separate because config_rev is what a report quotes, and an operator
    # comparing two machines compares revs before comparing file lists.
    "config_rev_mismatch",
    # A source file the freeze read has different bytes now than when the
    # snapshot was taken. THIS is the direct answer to "I edited a yaml".
    # What to do: if it is accompanied by common_digest_mismatch or
    # snapshot_stale, the edit is live-relevant and needs a restart. On its
    # own it means the edit changed only comments or only formatting, and the
    # running system is already equivalent to the source -- worth knowing
    # before scheduling an outage.
    "layer_changed",
    # added / removed fire when the FILE SET changed, not its contents: a new
    # models/*.yaml, a site file picked by a different site_id, a variant file
    # that appeared. These move config_rev without any single file's bytes
    # having changed, which is why they are named apart from layer_changed.
    "layer_added",
    "layer_removed",
    # The snapshot a freeze would write now differs from the one recorded --
    # i.e. a source edit that actually reaches this process's values. Note the
    # asymmetry with layer_changed: a comment-only edit produces layer_changed
    # WITHOUT this, because the materialiser dumps parsed values and comments
    # never survive into a snapshot.
    # What to do: restart that process's unit at minimum; restart the stack if
    # common_digest also moved.
    "snapshot_stale",
    # The snapshot file on disk differs from what the freeze recorded writing.
    # Someone edited data/run/resolved/{proc}.yaml by hand (iron rule 2), or
    # the tmpfs is not what it was.
    # What to do: the edit is about to be silently discarded at the next boot,
    # so whatever it was trying to achieve must be moved into configs/. This
    # is the finding that makes "改快照不改源" visible instead of leaving it to
    # be discovered as a mysterious rollback (INF-DP-5 mutant 1).
    "snapshot_tampered",
    # Recorded in MANIFEST.processes but not on disk. Any process whose unit
    # starts now refuses at R level; the snapshot did not survive the freeze.
    "snapshot_missing",
    # The set of materialised processes changed -- an L6 file appeared or
    # disappeared between the recorded freeze and now. Reported separately
    # from snapshot_* because there is no pair of hashes to compare.
    "process_added",
    "process_removed",
)


class Finding:
    """One thing --check noticed.

    A value, not an exception, and that is the design: --check has to report
    the whole list. Raising on the first problem would tell an operator that
    common.yaml changed and leave them to discover, after the restart, that a
    snapshot had also been hand-edited. The caller decides the exit code from
    the list; this class decides nothing.

    __slots__ because a Finding is passed around and printed, never extended.
    An attribute typo on a slotted object raises at the assignment instead of
    creating a field that the print loop then silently omits.
    """

    __slots__ = ("kind", "detail")

    def __init__(self, kind: str, **detail: Any):
        # Closed-set enforcement at construction, not at print time: a typo in
        # a kind would otherwise reach an operator as a plausible-looking line
        # and reach a caller's branch as a silent no-match (CLAUDE.md 3.5).
        if kind not in FINDING_KINDS:
            raise AssertionError(
                "finding kind %r is not in FINDING_KINDS; add it there with a "
                "comment saying what it means" % kind)
        self.kind = kind
        self.detail = detail

    def line(self) -> str:
        """One line for journalctl / a terminal. Sorted detail keys so two runs
        of the same drift produce the same text and a diff of two reports shows
        only real differences."""
        parts = " ".join("%s=%s" % (k, self.detail[k])
                         for k in sorted(self.detail))
        return "DRIFT %-22s %s" % (self.kind, parts)

    def __repr__(self) -> str:                       # pragma: no cover -- debug
        return "Finding(%r, %r)" % (self.kind, self.detail)


def _read_manifest(resolved_root: str) -> Tuple[Optional[Dict[str, Any]],
                                                List[Finding]]:
    """Parse resolved_root/MANIFEST.json. (manifest, findings); manifest is
    None exactly when the findings list is non-empty."""
    path = os.path.join(resolved_root, "MANIFEST.json")
    # isfile before open, accepting the race between the two. This is a
    # read-only diagnostic: the worst outcome of losing that race is a
    # manifest_unreadable finding instead of a manifest_missing one, and both
    # send the operator to the same place. Holding a lock to close it would
    # mean this tool could block the freeze service.
    if not os.path.isfile(path):
        return None, [Finding("manifest_missing", path=path)]
    try:
        with open(path, encoding="utf-8") as fh:
            raw = json.load(fh)
    except (OSError, json.JSONDecodeError) as exc:
        return None, [Finding("manifest_unreadable", path=path, reason=str(exc))]
    if not isinstance(raw, dict):
        return None, [Finding("manifest_unreadable", path=path,
                              reason="document is not a JSON object")]
    return raw, []


def _layer_findings(recorded: List[Dict[str, Any]],
                    fresh: List[Dict[str, Any]]) -> List[Finding]:
    """Compare two layers[] lists file by file.

    Keyed by path rather than by list position: an added L2 file shifts every
    later row, and a positional comparison would then report every row as
    changed and bury the one that actually did.
    """
    out: List[Finding] = []
    # Rows without a sha256 (the L5 env row) are compared on their whole
    # content instead -- there is no file to hash, and the interesting change
    # there is the key list. The `rec != new` test below covers both cases
    # without branching on which kind of row it is.
    rec_by_path = {r.get("path"): r for r in recorded}
    fresh_by_path = {r.get("path"): r for r in fresh}
    # The sort key tolerates a None path (a malformed recorded row) instead of
    # raising TypeError on str-vs-None. A --check that crashed while reading a
    # damaged MANIFEST would hide every other finding behind a traceback, and
    # a damaged MANIFEST is precisely a case worth reporting.
    for path in sorted(set(rec_by_path) | set(fresh_by_path),
                       key=lambda p: (p is None, p)):
        rec = rec_by_path.get(path)
        new = fresh_by_path.get(path)
        if rec is None:
            out.append(Finding("layer_added", path=path,
                               level=new.get("level")))
        elif new is None:
            out.append(Finding("layer_removed", path=path,
                               level=rec.get("level")))
        elif rec != new:
            # Whole-row comparison here, unlike _process_findings below. Both
            # sides of a layer row describe the SAME file at the same absolute
            # path (the fresh freeze reads the same config_root), so every
            # field is comparable. A process row's path differs by
            # construction -- the fresh one is in a temp directory.
            out.append(Finding("layer_changed", path=path,
                               level=rec.get("level"),
                               was=rec.get("sha256", rec.get("keys")),
                               now=new.get("sha256", new.get("keys"))))
    return out


def _process_findings(recorded: Dict[str, Any], fresh: Dict[str, Any],
                      ) -> List[Finding]:
    """Compare the per-process snapshots three ways.

    The three are genuinely different questions and an operator needs to know
    which one fired:
      * recorded vs fresh  -> a SOURCE edit that reaches this process
      * recorded vs disk   -> the snapshot file was changed after the freeze
      * recorded, no disk  -> the snapshot is gone
    Collapsing them into one "mismatch" would leave the operator unable to
    tell "restart the stack" from "someone hand-edited a generated file".
    """
    out: List[Finding] = []
    for proc in sorted(set(recorded) | set(fresh)):
        rec = recorded.get(proc)
        new = fresh.get(proc)
        if rec is None:
            out.append(Finding("process_added", proc=proc))
            continue
        if new is None:
            out.append(Finding("process_removed", proc=proc))
            continue
        # sha256 only -- NOT `rec != new`. The fresh row's `path` points into
        # the temp directory this check froze into, so a whole-row comparison
        # reports drift on every clean run and the tool becomes noise within a
        # week. size_bytes is implied by the hash and adds nothing.
        if rec.get("sha256") != new.get("sha256"):
            out.append(Finding("snapshot_stale", proc=proc,
                               was=rec.get("sha256"), now=new.get("sha256")))
        # The recorded path, not the fresh one: the fresh snapshot lives in a
        # temp directory that is about to be deleted, and the file this check
        # cares about is the one the running system reads.
        path = rec.get("path")
        if not path or not os.path.isfile(path):
            # `not path` as well as the isfile test: a MANIFEST row with an
            # empty path is missing the snapshot just as surely as one whose
            # file was deleted, and isfile("") would answer False for a reason
            # that has nothing to do with the snapshot.
            out.append(Finding("snapshot_missing", proc=proc, path=path))
        else:
            # Hashed once and reused. Calling file_sha256 twice -- once for the
            # comparison and once for the message -- could read two different
            # contents if the file changed between them, and the report would
            # then quote a hash that matched nothing it had compared.
            on_disk = file_sha256(path)
            if on_disk != rec.get("sha256"):
                out.append(Finding("snapshot_tampered", proc=proc, path=path,
                                   was=rec.get("sha256"), now=on_disk))
    return out


def check_freeze(*, config_root: str, resolved_root: str, boot_id: str,
                 config_root_overridden: bool = False,
                 config_variant: Optional[str] = None,
                 context: Optional[Dict[str, Any]] = None) -> List[Finding]:
    """Compare the recorded freeze against what the sources say now.

    Returns every finding; an empty list means the snapshot on disk is exactly
    what a freeze of the current sources would produce. Writes nothing outside
    a temporary directory it creates and removes.
    """
    # findings accumulates in the order problems were found, which is also
    # roughly the order an operator should read them: what is missing, then
    # what is stale, then what differs and in which file.
    recorded, findings = _read_manifest(resolved_root)
    if recorded is None:
        # Nothing further is answerable without a baseline, and a list of
        # "everything differs" findings against an absent manifest would be
        # noise around the one fact that matters.
        return findings
    if recorded.get("boot_id") != boot_id:
        findings.append(Finding("boot_id_stale",
                                recorded=recorded.get("boot_id"),
                                current=boot_id))
    # mkdtemp, not a fixed path: two --check runs must not collide, and a
    # fixed path under /tmp would be a predictable target for a symlink swap.
    scratch = tempfile.mkdtemp(prefix="xbrain-freeze-check-")
    # The finally that removes `scratch` wraps the inner try/except, so the
    # early return on sources_refuse_freeze still cleans up. Putting the
    # rmtree after the except block instead would leak a directory on exactly
    # the path that fires most often today (the real configs refuse).
    try:
        try:
            fresh = run_freeze(
                boot_id=boot_id,
                config_root=config_root,
                config_root_overridden=config_root_overridden,
                config_variant=config_variant,
                resolved_root=scratch,
                # Copied, not passed through: run_assertions populates ctx
                # with caches (the overlay, the layer trees) and the caller's
                # dict must not come back mutated -- a caller that ran two
                # checks would otherwise have the second one silently reuse
                # the first one's cached overlay.
                context=dict(context) if context else None,
            )
        except XbrainError as exc:
            # The sources are not freezable right now. Report and stop: there
            # is no "what a freeze would produce" to compare against, and
            # emitting mismatches derived from a failed pass would be
            # fabricating a comparison.
            detail = getattr(exc, "detail", None) or {}
            findings.append(Finding("sources_refuse_freeze",
                                    reason=str(exc),
                                    kind_detail=detail.get("kind")))
            return findings
    finally:
        # ignore_errors so a cleanup failure cannot turn a clean check into a
        # crash; the temp tree is under /tmp and a leftover is harmless.
        shutil.rmtree(scratch, ignore_errors=True)
    # .get on the recorded side, [] on the fresh side. The recorded document
    # may have been written by an older freeze that predates a field, and a
    # KeyError there would turn "your snapshot is old" into a crash. The fresh
    # side is produced by the code in this repository right now, so a missing
    # key there IS a defect and should raise rather than compare as None.
    if recorded.get("common_digest") != fresh["common_digest"]:
        findings.append(Finding("common_digest_mismatch",
                                recorded=recorded.get("common_digest"),
                                current=fresh["common_digest"]))
    if recorded.get("config_rev") != fresh["config_rev"]:
        findings.append(Finding("config_rev_mismatch",
                                recorded=recorded.get("config_rev"),
                                current=fresh["config_rev"]))
    # `or []` rather than `.get("layers", [])`: a MANIFEST written before
    # CFG-CM-10 carries `layers: []` already, but one written by an even older
    # freeze could carry null, and `.get` with a default would not catch that.
    findings.extend(_layer_findings(recorded.get("layers") or [],
                                    fresh["layers"]))
    findings.extend(_process_findings(recorded.get("processes") or {},
                                      fresh["processes"]))
    return findings
