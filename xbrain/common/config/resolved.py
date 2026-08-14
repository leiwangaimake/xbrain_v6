"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: resolved.py
Brief: The only way a process may read its configuration at run time

Description:
What problem this solves. 10 S5.4.1 calls "references are not expanded by each
process individually" the most critical structural decision in the configuration
design, and gives the reason: each process would otherwise resolve them to
different results. The mechanism that makes it true is the freeze line, which
expands everything once and writes read-only snapshots to /run/xbrain/resolved/.
This module is the reader for those snapshots, and it exists so that "read the
product, never the source" is something a process gets by calling one function
rather than something it has to remember.

The gate, from 10 S5.4.4 and the startup-assertion table:
  MANIFEST.boot_id must equal /proc/sys/kernel/random/boot_id, and a mismatch
  means the snapshot is left over from the PREVIOUS boot -- /run was not cleared
  as tmpfs should have cleared it. Continuing would run the whole stack on stale
  configuration. The severity is R: refuse to start, E_CONFIG_INVALID.

*** Why the gate runs before anything else is read. A reader that loaded the
process YAML first and checked boot_id afterwards would work identically in every
passing case, and in the failing case it would have already handed a caller a
tree built from last boot's values. Callers that read fields during construction
would act on them before the exception unwound. So the order is not stylistic:
the check is a gate, and a gate placed after the door is a decoration.

*** The boot_id comparison is on the FULL value, never a prefix. 11 S3.0 defines
a DIFFERENT boot_id-derived field -- the message envelope's `boot`, verbatim
"发布主机 boot_id 的前 8 位十六进制" -- and reusing that helper here is the
mistake this module is written to make impossible. Two ways it goes wrong, and
the second is the one that actually happens:
  * comparing an 8-character prefix against a full UUID with == is FALSE on every
    boot, so the assertion is permanently red;
  * whoever meets that then "fixes" it by loosening the comparison to startswith,
    which turns a permanently-red assertion into one that accepts any two boots
    sharing eight hex digits. That is CLAUDE.md 3.2 form 2 turning into form 1,
    and it is why both directions have their own test.

What this module deliberately does NOT do:
  * it does not expand ${common.*} -- that already happened at the freeze line,
    and doing it here would be the very thing S5.4.1 forbids;
  * it does not run the startup assertions. Those belong to
    xbrain-config-freeze.service (10 S5.4.4, and 13 S8.3 states it verbatim for
    the QC-* family: 断言由 xbrain-config-freeze.service 统一执行, 不在进程内);
  * it does not compare common_digest against anything. 10 S3.3.3 puts that at
    P2's Stage D, and its failure behaviour is DIFFERENT from this one -- B, keep
    motion disallowed and emit event/fault/bit with
    detail.kind="config_digest_mismatch", not R. This module exposes the value so
    P2 can do it; deciding on it here would apply the wrong severity;
  * it never falls back to /opt/xbrain_v6/configs/. There is no flag for that and
    there must not be one -- CFG-ROOT-5 exists because "I edited the file under
    /run and thought I had changed the configuration" silently rolls back at the
    next boot, and a reader that quietly read the source when the snapshot was
    missing would produce the same class of confusion in the other direction.
"""

import hashlib
import json
import os
import re
from typing import Any, Dict, List, Optional

from ..errors import E_CONFIG_INVALID
from ..errors.exceptions import XbrainError
from .merge import MISSING, flatten

#: Where the freeze line writes its products. 10 S5.4.0 CFG-ROOT-5: this is
#: tmpfs and it is NOT under the configuration root -- source and product are two
#: different places. The constant is here rather than in layers.py on purpose:
#: layers.py is the source side, and a module that knew both paths would be the
#: natural home for exactly the fallback this design forbids.
#:
#: Why the section bothers to say the two are different places: an operator who
#: edits a file under /run has changed nothing durable, and the edit vanishes at
#: the next boot when tmpfs is recreated. The rollback is silent, so the symptom
#: arrives days later as "the setting we changed came back".
#:
#: Absolute, never relative. 10 S5.4.0 CFG-ROOT-2 gives the reason for the source
#: root and it applies identically here: a relative path resolves against
#: whatever WorkingDirectory the unit happens to have, so the same code would
#: read different files depending on how it was started.
# 2026-08-10 relocated: user-facing rule 'V6 系统一切运行时依赖资产必须在 /opt/xbrain_v6/ 下',
# so the freeze product tree moved from the 10 S5.4.6 default tmpfs mount
# (/run/xbrain/resolved) to /opt/xbrain_v6/data/run/resolved. The invariant
# preserved is 'resolved products are NOT under configs/'; the difference is
# the storage medium (data/run is disk-backed, not tmpfs). ProductRoot still
# refuses to open anything outside this root -- see path-confinement below.
RESOLVED_ROOT = "/opt/xbrain_v6/data/run/resolved"

#: The kernel's per-boot identifier. Read as a path rather than through a helper
#: so a test can point it somewhere else without monkey-patching.
#:
#: The seam is deliberately this small. A test that could replace "how a file is
#: read" could also replace the comparison; a test that can only choose WHICH
#: file is read still has to go through the real gate. The narrower the seam, the
#: less of the real path a test can accidentally bypass -- and a bypassed gate
#: with a green suite is the failure this module is most exposed to.
BOOT_ID_PATH = "/proc/sys/kernel/random/boot_id"

#: The six processes the freeze line produces a snapshot for, from the
#: MANIFEST.processes block of 10 S5.4.6. quadruped is in this list: it was added
#: in v0.7.7 to repair a state the document calls 进程在册 - 配置不在册, where the
#: process was a registered resident process but had no configuration entry, so
#: freeze produced no snapshot for it and it had to expand ${common.spec.*}
#: itself -- breaking S5.4.1 silently.
#:
#: Kept as a frozenset and checked, rather than trusted from the MANIFEST alone,
#: so that asking for a name that is not a process fails here instead of
#: producing a confusing "file not found" for a path nobody meant to build.
#:
#: *** perception is deliberately NOT in this set, and that is an open question
#: rather than a decision. 19 S9.1 PRC-62 states verbatim that perception reads
#: /run/xbrain/resolved/perception.yaml at run time and does not read the source,
#: and 19 S11 PMT-66 is the mutation for it. But 10 S5.4.6's MANIFEST.processes
#: block lists six processes and perception is not among them, so the freeze line
#: as specified produces no snapshot for it.
#:
#: That is structurally the same gap 10 v0.7.7 repaired for quadruped -- the
#: document called it 进程在册 - 配置不在册 -- and repairing it means editing 10,
#: which is a design change and not a reader's call (CLAUDE.md 9.1: on a conflict
#: between volumes, stop and ask). Adding perception here unilaterally would make
#: the reader accept a process the freeze line never writes a snapshot for, which
#: converts a startup refusal into a file-not-found at a random later moment.
SNAPSHOT_PROCESSES = frozenset({
    "p1_motion", "p2_core", "p3_task", "p4_agent", "p5_gateway", "quadruped",
    # rtk_driver: C++ RT-plane GNSS process, config-in-register like quadruped
    # (10 S5.4.0 exhaustive table + 13-style private-config; freeze writes its
    # resolved snapshot so 11 S3.3 thresholds are not defaulted in code, 3.1).
    "rtk_driver",
})

#: MANIFEST fields this reader requires to be present. Absence is refused rather
#: than defaulted: every one of them is either the gate itself or a value a
#: consumer will act on, and a default would be a fabricated answer to
#: "which configuration am I running".
#:
#: *** This list is a REQUIRED set, not a permitted set, and the difference
#: matters. A strict schema that rejected unknown fields would reject perfectly
#: legal manifests: 10 S5.4.6 shows thirteen top-level keys, GATE-6 adds
#: net_profile and net_rules_sha256 for its audit trail, and startup assertion I
#: needs a models section with a model_digest. A reader that insisted on exactly
#: the documented thirteen would refuse to start on a manifest that is more
#: complete than the example.
REQUIRED_MANIFEST_FIELDS = ("schema", "boot_id", "common_digest", "processes")

#: The schema string the freeze line stamps, from the MANIFEST block of 10 S5.4.6.
MANIFEST_SCHEMA = "xbrain.config.manifest/1"

#: A well-formed sha256, for the processes[].sha256 field. Lowercase only,
#: matching what hashlib.hexdigest() produces, so a manifest written with
#: uppercase is reported rather than quietly failing every comparison.
_SHA256_RE = re.compile(r"^[0-9a-f]{64}$")


class ResolvedConfigError(XbrainError):
    """A snapshot that cannot be trusted, for any reason.

    One exception type rather than one per cause, with the cause named in the
    message and carried in `code`. The caller's response is identical in every
    case -- refuse to start -- so splitting the type would offer a choice that
    does not exist, and inviting a caller to catch one kind and continue is
    precisely what must not be possible here.
    """

    def __init__(self, message: str, code: str = E_CONFIG_INVALID):
        super().__init__(message)
        # E_CONFIG_INVALID is the code 10 S5.4.4 assigns to this whole family:
        # configuration that fails its own self-check, startup refused, not
        # retryable and not degraded-through. It is the default rather than a
        # required argument because every raise in this module means exactly
        # that, and a required argument would invite a caller to pass something
        # more specific-looking that is not in the closed set.
        # The closed-set code, from the shared library rather than a literal
        # (CLAUDE.md 3.5). Carried so a caller reporting the failure upward does
        # not have to re-derive which code applies.
        self.code = code


def read_boot_id(path: str = BOOT_ID_PATH) -> str:
    """The current boot's identifier, or raise.

    *** Unreadable is a failure, never an empty string. A reader that returned ""
    on a missing /proc entry would compare "" against the MANIFEST value, get a
    mismatch, and refuse to start -- which looks safe until someone notices that
    every boot fails and makes the comparison tolerant of the empty case. Then
    the gate is open on exactly the machines where the kernel interface is
    absent. Failing loudly here keeps that repair from being available.
    """
    try:
        with open(path, encoding="ascii") as fh:
            value = fh.read().strip()
    except OSError as exc:
        raise ResolvedConfigError(
            "cannot read the current boot id from %s: %s; refusing to start "
            "because the freshness of the resolved snapshot cannot be "
            "established (10 S5.4.4)" % (path, exc)) from exc
    # Empty is treated exactly like unreadable, and both raise. The tempting
    # middle ground -- treat empty as "unknown, allow" -- is the whole failure
    # this function is shaped to prevent: the gate would then be open on
    # precisely the machines where the kernel interface is missing or has been
    # masked, which is where a stale /run is most likely in the first place.
    if not value:
        raise ResolvedConfigError(
            "%s is empty; refusing to start for the same reason as an "
            "unreadable one -- an empty value cannot establish freshness" % path)
    # Returned stripped. The real /proc entry ends in a newline, and comparing an
    # unstripped value against the manifest would fail on every boot -- the
    # permanently-red form, whose usual repair is to loosen the comparison until
    # it passes. Stripping here removes the pressure to do that.
    return value


#: Handed to Manifest.__init__ by load_manifest and by nothing else.
#:
#: Why a token and not a comment saying "do not construct this directly". The
#: first draft of this module asserted in its docstring that no constructor path
#: produced an unvalidated Manifest, and that was simply false: Manifest({...},
#: "x") worked, and load_resolved(manifest=...) accepts one, so the pair was a
#: complete bypass of the gate -- reachable by accident, with the docstring
#: saying it was not. A claim in a comment is not a mechanism.
#:
#: What this does and does not achieve, stated precisely so the next reader does
#: not over-trust it: importing _GATE_TOKEN and passing it defeats this in one
#: line. It is not a security boundary and cannot be -- it makes the bypass a
#: deliberate act that shows up in review, instead of the natural way to write
#: the call.
_GATE_TOKEN = object()


class Manifest:
    """The parsed MANIFEST.json, produced only after the boot_id gate passed.

    Holding one means the gate ran. See _GATE_TOKEN for exactly how strongly
    that is enforced, and for what it deliberately does not claim.
    """

    # Two fields only. Everything else a caller might want is derived from raw
    # through a property, so there is exactly one copy of each value and no way
    # for a cached field to disagree with the document it came from.
    __slots__ = ("raw", "path")

    def __init__(self, raw: Dict[str, Any], path: str, _gate: Any = None):
        if _gate is not _GATE_TOKEN:
            raise ResolvedConfigError(
                "Manifest instances come from load_manifest(), which runs the "
                "boot_id gate first. Constructing one directly would produce an "
                "object whose whole meaning is 'the gate passed' without the "
                "gate having run (10 S5.4.4)")
        self.raw = raw
        self.path = path

    @property
    def boot_id(self) -> str:
        """The value the gate already matched.

        Present so a caller can put it in a startup event or a log line without
        reaching into raw. Indexed rather than fetched with .get(): the key is in
        REQUIRED_MANIFEST_FIELDS, so a Manifest without it cannot exist, and a
        .get() here would suggest otherwise to the next reader.
        """
        return str(self.raw["boot_id"])

    @property
    def common_digest(self) -> str:
        """The value P2 compares at Stage D. See the module docstring.

        Exposed and not acted upon here, because the failure behaviour there is
        B (keep motion disallowed, emit a fault) rather than the R this module
        applies -- applying R to it would refuse to start on a condition the
        design wants observable and reported.
        """
        return str(self.raw["common_digest"])

    @property
    def config_rev(self) -> Any:
        """Optional. 10 S5.4.6 requires it for CFG-41 auditability, but a reader
        that refused to start without it would be enforcing the freeze line's
        obligation from the wrong side. Returns MISSING when absent so a caller
        can tell "not stamped" from "stamped empty"."""
        return self.raw.get("config_rev", MISSING)

    @property
    def config_root_overridden(self) -> bool:
        """ENV-3: true means XBRAIN_CONFIG_DIR was in effect at freeze time.

        Defaults to False when absent rather than raising. The field is the
        freeze line's record of its own environment, and a snapshot written by an
        older freeze that predates the field is not evidence of an override.
        Callers that must surface the "non-standard configuration root" marker
        read this; this module does not decide policy on it.
        """
        return bool(self.raw.get("config_root_overridden", False))

    def process_entry(self, proc: str) -> Dict[str, Any]:
        """The MANIFEST.processes row for one process, or raise.

        Every failure here is a raise and none of them is a default. The
        tempting alternative -- return an empty dict and let load_resolved build
        the conventional path -- would work on every well-formed manifest and,
        on a malformed one, would quietly read a file the manifest never
        blessed. The whole value of the manifest is that it says which file
        belongs to which process; inferring it defeats that.
        """
        processes = self.raw["processes"]
        # Type-checked before use. A manifest whose processes field is a list, or
        # a string, would otherwise raise TypeError from deep inside a membership
        # test, and the traceback would point at this line rather than at the
        # malformed document.
        if not isinstance(processes, dict):
            raise ResolvedConfigError(
                "MANIFEST.processes is not an object in %s" % self.path)
        if proc not in processes:
            # Naming the ones that ARE present, because the realistic cause is a
            # freeze line that ran against a stale process list -- which is
            # exactly the 进程在册-配置不在册 state 10 S5.4.6 repaired for
            # quadruped, and it is invisible from the failing process's side
            # unless the message says what the manifest did contain.
            raise ResolvedConfigError(
                "MANIFEST has no entry for process %r; it lists %s. The freeze "
                "line did not produce a snapshot for this process, so it has no "
                "configuration to run on (10 S5.4.6)"
                % (proc, sorted(processes)))
        entry = processes[proc]
        if not isinstance(entry, dict):
            raise ResolvedConfigError(
                "MANIFEST.processes.%s is not an object in %s" % (proc, self.path))
        return entry


def load_manifest(root: str = RESOLVED_ROOT,
                  boot_id_path: str = BOOT_ID_PATH) -> Manifest:
    """Read MANIFEST.json and run the boot_id gate.

    Chained with `from exc` on every re-raise below, so the original OSError or
    decoder error stays in the traceback. The wrapper explains what it means for
    the system; the cause explains what the filesystem actually said, and a
    reader debugging a permissions problem needs both.

    Split out from load_resolved so a process that needs the manifest alone --
    P2 at Stage D wanting common_digest, say -- does not have to load a snapshot
    it will not read.

    Every exit from this function is either a validated Manifest or an exception.
    There is no third outcome, and in particular no "degraded" manifest with the
    gate skipped: 10 S5.4.4's failure row for this condition is R, refuse to
    start, and a partial success would be a way to reach the running state
    without having satisfied it.

    The two parameters exist for testing and for nothing else. They are ordinary
    paths rather than an injected filesystem object because a test needs to point
    at a temporary directory, not to replace the notion of a file -- and the
    smaller the seam, the less of the real path a test can accidentally bypass.
    """
    # Built with os.path.join rather than an f-string so a root given without a
    # trailing separator, or with one, behaves the same. A reader that failed
    # only when someone passed "/run/xbrain/resolved/" would fail in the field
    # and not in any test, because tests pass what the constant holds.
    path = os.path.join(root, "MANIFEST.json")
    try:
        with open(path, encoding="utf-8") as fh:
            # json, not json5, even though 10 S5.4.6 documents the structure in
            # json5 with comments. The document is showing a structure to a
            # human; the file a freeze line writes is data. Accepting comments
            # would mean carrying a second parser for the benefit of a file
            # nobody hand-edits -- and hand-editing it is what CFG-ROOT-5 warns
            # against, since the edit silently rolls back at the next boot.
            raw = json.load(fh)
    except FileNotFoundError as exc:
        # The message names the freeze service, because "file not found" sends a
        # reader looking for a file to create, and creating this one by hand is
        # the worst available response.
        raise ResolvedConfigError(
            "%s does not exist. The resolved snapshot is produced by "
            "xbrain-config-freeze.service at Stage 0c; if it is absent that "
            "service did not run or did not succeed. This process will NOT fall "
            "back to reading the configuration source (10 S5.4.1)" % path) from exc
    except OSError as exc:
        raise ResolvedConfigError("cannot read %s: %s" % (path, exc)) from exc
    except json.JSONDecodeError as exc:
        # The decoder's own message carries the line and column, which is the
        # only useful thing to say about a malformed document, so it is passed
        # through rather than replaced with a friendlier sentence that says less.
        raise ResolvedConfigError(
            "%s is not valid JSON: %s" % (path, exc)) from exc

    # A JSON document may legally be a list or a bare scalar. Checking rather
    # than assuming, because raw["boot_id"] on a list raises TypeError with a
    # message about list indices -- which sends the reader looking for a bug in
    # this module instead of at the file it was handed.
    if not isinstance(raw, dict):
        raise ResolvedConfigError("%s does not contain a JSON object" % path)

    # Membership, not truthiness. `"boot_id" in raw` is what distinguishes an
    # absent key from one present and empty, and only the first is a manifest
    # from an older or broken freeze line -- the second is a freeze line that
    # wrote something wrong, which the gate below will catch and report
    # differently. Collapsing the two would give one message for two problems.
    missing = [f for f in REQUIRED_MANIFEST_FIELDS if f not in raw]
    if missing:
        # *** boot_id being absent is reported as a missing field and NOT as a
        # passing gate. A reader using raw.get("boot_id") would compare None
        # against the current value, get a mismatch and refuse -- which is the
        # right outcome by accident. It is written explicitly because the
        # accident is fragile: the moment someone adds an "if manifest_boot_id:"
        # guard to quieten a test, the absent case starts passing.
        raise ResolvedConfigError(
            "%s is missing required field(s) %s" % (path, missing))

    # *** The schema check is an implementation decision, NOT a documented
    # requirement. 10 S5.4.6 stamps the field but says nothing about what a
    # consumer does when it differs, and the mining pass over the fourteen
    # volumes found no rule for it. Refusing is the safer of the two readings --
    # a later generation may put different meaning behind the same field names,
    # and reading it partially is how a reader ends up confidently wrong rather
    # than loudly stuck. Recorded as our choice so nobody later cites it as
    # something the contract required.
    schema = raw["schema"]
    if schema != MANIFEST_SCHEMA:
        raise ResolvedConfigError(
            "%s declares schema %r, this reader implements %r. A snapshot from a "
            "different manifest generation is refused rather than read on a "
            "best-effort basis" % (path, schema, MANIFEST_SCHEMA))

    # Order: the manifest value first, then the current one. Reading /proc first
    # would mean a malformed manifest produced a /proc read that no longer
    # mattered -- harmless here, but the reverse order is what makes the failure
    # messages come out in the order a reader can act on them: what the file
    # said, then what the machine says.
    manifest_boot = str(raw["boot_id"])
    current_boot = read_boot_id(boot_id_path)
    # *** Full-value equality. Not startswith, not a prefix of either side, not
    # case-normalised. See the module docstring for why the prefix forms are the
    # specific mistakes guarded against here.
    if manifest_boot != current_boot:
        raise ResolvedConfigError(
            "MANIFEST.boot_id %r does not match the current boot id %r. The "
            "resolved snapshot in %s was produced during a PREVIOUS boot, which "
            "means /run was not cleared as tmpfs. Running on it would run the "
            "whole stack on stale configuration, so startup is refused "
            "(10 S5.4.4)" % (manifest_boot, current_boot, root))

    return Manifest(raw, path, _GATE_TOKEN)


class ResolvedConfig:
    """One process's resolved configuration, with the gate already passed.

    Carries the manifest alongside the tree because the two are only meaningful
    together: a caller reporting its startup event needs common_digest and
    config_rev from the manifest and its own values from the tree, and handing
    them back separately invites the two to be taken from different loads.

    10 S5.4.4 is explicit that the startup event carrying these values must reuse
    an EXISTING key rather than open a new one: the section notes that something
    like cmd/config/state would hit the S2.2 key table, the S2.4.7 QoS binding
    table and the S1.1.6 cross-plane whitelist all at once, and that none of it
    is needed because consistency is already guaranteed structurally by the
    freeze line. So this object hands a caller what it needs to fill in an
    existing event, and defines no transport of its own.

    Note for P1 in particular: it is one of the three cross-plane processes and
    may publish only on keys registered in the S1.1.6 whitelist. That constrains
    the caller rather than this module, and is written here because it is easy to
    miss from inside a process that has a perfectly serviceable event to hand.
    """

    # __slots__ so this stays four fields. An object that accumulated derived
    # state -- a cached flatten, an expanded tree -- would start to look like a
    # place where configuration is computed, and computation is what the freeze
    # line owns.
    __slots__ = ("proc", "tree", "manifest", "path")

    def __init__(self, proc: str, tree: Dict[str, Any], manifest: Manifest,
                 path: str):
        # No validation here. Everything that could be wrong was checked before
        # this object was built, and re-checking would create a second place
        # where the rules live -- which is how two checks drift apart and the
        # weaker one starts deciding.
        self.proc = proc
        # Not copied. The snapshot is a read-only product by design; writing into
        # this tree would desynchronise the process from the sha256 the manifest
        # recorded for the file it came from, and that sha256 is the audit trail.
        self.tree = tree
        self.manifest = manifest
        # The path is kept so every later error message can name the file the
        # value came from. "required key is absent" without a filename is an
        # invitation to go looking through six snapshots by hand.
        self.path = path

    def get(self, dotted: str, default: Any = MISSING) -> Any:
        """Leaf or subtree by dotted path.

        Returns MISSING rather than None for an absent key, matching
        OverlayResult.get -- null in this tree is a real value meaning "declared
        and unassigned", and a reader that collapsed the two would lose the
        distinction the whole layer design is built on.

        Walks the tree rather than indexing a flattened map, for the same reason
        OverlayResult.get does: flatten yields only leaves, and 10 S5.4.2 allows
        a whole object to be referenced, so a lookup has to be able to stop at a
        subtree. A flatten-based implementation would report a legal
        whole-section value as absent.
        """
        node: Any = self.tree
        for part in dotted.split("."):
            # isinstance before membership: a dotted path that runs past a leaf
            # -- "loop_hz.something" -- must come back absent rather than raise
            # TypeError, because the caller asked a question whose answer is
            # simply no.
            if not isinstance(node, dict) or part not in node:
                return default
            node = node[part]
        return node

    def require(self, dotted: str) -> Any:
        """Leaf that must be present AND non-null.

        The counterpart to get, for safety parameters. CLAUDE.md 3.1 forbids
        default fallbacks on common.safety.* and common.spec.*, so the shape a
        caller needs is "give me this or fail" -- and if the only accessor
        offered takes a default, someone will pass one.
        """
        value = self.get(dotted)
        # Two distinct failures with two distinct messages, because they call
        # for different work: an absent key means the snapshot does not contain
        # what this process expects, which points at the freeze line or at a
        # renamed key; a null means the key exists and nobody ever filled it in,
        # which points at configs/ and at whoever owns that site file.
        if value is MISSING:
            raise ResolvedConfigError(
                "%s: required key %r is absent from %s" % (self.proc, dotted, self.path))
        if value is None:
            # null means declared-but-unassigned, which assertion A should
            # already have rejected at the freeze line. Reaching here means the
            # freeze line let it through, so the message says so -- otherwise the
            # reader looks like the component at fault.
            raise ResolvedConfigError(
                "%s: key %r is null (declared but never assigned) in %s. "
                "Startup assertion A should have rejected this at the freeze "
                "line; that it did not is itself a defect worth reporting "
                "(10 S5.4.4)" % (self.proc, dotted, self.path))
        return value

    def unassigned(self) -> List[str]:
        """Dotted paths still null in this snapshot.

        Present for the same reason OverlayResult has one: when a process fails
        on a null it is useful to report every hole at once rather than one per
        restart.

        The test is `v is None`, never `not v`. 0, "" and [] are assigned values,
        and reporting them as holes would send someone to fill in a key that is
        already correct -- and would train the reader to skim the list.

        *** What this cannot see, for the same reason the overlay cannot: a
        safety parameter written 0.0 instead of null is indistinguishable from a
        genuine zero here, and it is the more dangerous of the two because it
        passes every check and then limits v_max to zero at run time with nothing
        logged. The defence is CLAUDE.md 3.1 -- write null, never 0.0 -- plus
        startup assertion G, which range-checks the values. Not this function,
        and it must not be made to look as though it were.
        """
        return sorted(k for k, v in flatten(self.tree).items() if v is None)


# Nothing in this module runs at import time. p5_gateway's minimal mode -- the
# observation window of 10 S3.3.7 W-1 -- comes up on L0 code defaults alone,
# reads no snapshot, and its unit deliberately does not Requires= the freeze
# service, so that a freeze failure shows the operator the failing assertion on
# the HMI instead of a machine that does nothing at all. A gate that ran on
# import would take that window down with everything else, and the window exists
# precisely for the case where everything else is down.
def load_resolved(proc: str,
                  root: str = RESOLVED_ROOT,
                  boot_id_path: str = BOOT_ID_PATH,
                  manifest: Optional[Manifest] = None,
                  verify_sha256: bool = True) -> ResolvedConfig:
    """*** The one entry point a process uses to obtain its configuration.

    Order of operations, and none of it is interchangeable:
      1. the process name is checked against the known set;
      2. the manifest is read and the boot_id gate runs -- BEFORE any snapshot
         content is opened, see the module docstring;
      3. the manifest's entry for this process is located;
      4. the snapshot file is read;
      5. its sha256 is compared with the one the manifest recorded.

    `manifest` may be passed by a caller that already loaded it, so a process
    reading two snapshots does not run the gate twice. It is typed as a Manifest
    and a Manifest can only exist post-gate, so accepting one here cannot be used
    to skip the check.

    `verify_sha256` exists because 10 S5.4.6 records a sha256 per process without
    stating who verifies it; see the note at the call site below. It is NOT a way
    to relax the boot_id gate -- that has no parameter and must not acquire one.
    CLAUDE.md 3.6 forbids any switch that skips a safety check, on the grounds
    that such a switch is a channel for disabling the constraint wholesale.

    Returns a ResolvedConfig or raises. There is no None return and no partially
    populated object: a caller that wrote `cfg = load_resolved(...)` and carried
    on regardless would otherwise be one forgotten check away from running
    unconfigured.
    """
    # The name check is first, before any I/O. A typo would otherwise produce a
    # file-not-found for a path that was never going to exist, and the two need
    # different fixes: one is a spelling mistake in the caller, the other is a
    # freeze line that did not run.
    if proc not in SNAPSHOT_PROCESSES:
        # The known set is printed. A caller that misspelled a name sees the
        # right spelling immediately, and a caller that asked for a process
        # genuinely outside the set -- perception, today -- sees that it is
        # outside rather than being told only that it is unknown.
        raise ResolvedConfigError(
            "%r is not a process the freeze line produces a snapshot for; "
            "known processes are %s" % (proc, sorted(SNAPSHOT_PROCESSES)))

    # The gate runs here, or it ran already and produced the Manifest that was
    # passed in. There is no path through this function that reaches the file
    # read without one of those two having happened -- see _GATE_TOKEN for what
    # enforces the second case.
    if manifest is None:
        manifest = load_manifest(root, boot_id_path)

    entry = manifest.process_entry(proc)

    # The path comes from the manifest when it supplies one, and is otherwise
    # constructed. Preferring the manifest's own path means a freeze line that
    # wrote elsewhere is followed rather than silently second-guessed -- but the
    # value is confined to `root` below, because a manifest is not a trusted
    # source of arbitrary filesystem paths.
    # Falling back to the conventional filename when the manifest gives no path,
    # rather than refusing. 10 S5.4.6 shows a path in every row, so an absent one
    # means a freeze line older than that block -- and the conventional name is
    # exactly what such a freeze line wrote. The confinement check below applies
    # either way, so the fallback cannot widen where this reads from.
    declared = entry.get("path")
    path = declared if isinstance(declared, str) and declared else os.path.join(
        root, proc + ".yaml")
    # *** Confinement check. A manifest naming /opt/xbrain_v6/configs/p1_motion.yaml
    # would otherwise turn this reader into precisely the source-reading path the
    # whole design forbids, and it would do it while every test still passed.
    # normpath on both sides, so "/run/xbrain/resolved" and
    # "/run/xbrain/resolved/" and "/run/xbrain/../xbrain/resolved" all compare
    # equal -- and so that a path containing ".." cannot climb out of the root
    # while still looking like a child of it in a naive string comparison.
    if os.path.normpath(os.path.dirname(path)) != os.path.normpath(root):
        raise ResolvedConfigError(
            "MANIFEST names %r for process %s, which is outside the resolved "
            "root %s. A snapshot reader will not follow a path out of the "
            "product directory -- that is how a process ends up reading the "
            "configuration source (10 S5.4.1, CFG-ROOT-5)" % (path, proc, root))

    # Read as bytes, not text. The sha256 below must be taken over exactly what
    # the freeze line hashed, and decoding first would make the digest depend on
    # the decoder -- a file with a BOM, or with CRLF line endings on a machine
    # opening in text mode, would hash differently while being byte-identical on
    # disk. Decoding happens once, later, in _parse_snapshot.
    try:
        with open(path, "rb") as fh:
            data = fh.read()
    except FileNotFoundError as exc:
        # Listed but absent means the freeze line was interrupted between
        # writing the manifest and writing this file. That is a partial product,
        # and a partial product is the one case where falling back to the source
        # would feel most reasonable and be most wrong: the manifest already
        # claims a digest for content that does not exist.
        raise ResolvedConfigError(
            "%s is listed in MANIFEST.processes but does not exist. The freeze "
            "line did not finish writing it; this process will NOT read the "
            "configuration source instead (10 S5.4.1)" % path) from exc
    except OSError as exc:
        raise ResolvedConfigError("cannot read %s: %s" % (path, exc)) from exc

    if verify_sha256:
        recorded = entry.get("sha256")
        # *** Absent is tolerated, present-and-anything-else is not.
        #
        # 10 S5.4.6 shows this field in every processes row but never says a
        # consumer must verify it -- unlike assertion I, which states outright
        # that MODEL.json.files[].sha256 is compared byte for byte. So refusing
        # to start when it is absent would enforce a rule the document does not
        # contain, and this check is an addition on our side rather than a
        # requirement being implemented. Recorded as such per CLAUDE.md 3.2:
        # the distinction between "the contract says so" and "we decided so"
        # must not blur.
        #
        # An earlier draft skipped verification when the value began with the
        # ellipsis character, because the document's EXAMPLE writes the field as
        # an ellipsis placeholder. That was a hole: any manifest carrying that
        # one character disabled the check silently, and nothing would have
        # reported it. The example is documentation shorthand, not a value a
        # freeze line ever writes -- so a field that is present and not a
        # well-formed digest is now a malformed manifest, not a licence to skip.
        if recorded is not None:
            if not isinstance(recorded, str) or not _SHA256_RE.match(recorded):
                raise ResolvedConfigError(
                    "MANIFEST.processes.%s.sha256 is %r, which is not a "
                    "64-character lowercase hex digest. A field that is present "
                    "but unusable is refused rather than skipped -- skipping is "
                    "how a check turns itself off without telling anyone"
                    % (proc, recorded))
            actual = hashlib.sha256(data).hexdigest()
            if actual != recorded:
                raise ResolvedConfigError(
                    "%s does not match the sha256 MANIFEST recorded for it "
                    "(recorded %s, actual %s). The snapshot was modified after "
                    "the freeze line wrote it; editing files under %s does not "
                    "change the configuration and silently rolls back at the "
                    "next boot (CFG-ROOT-5)" % (path, recorded, actual, root))

    # Parsed last, after every check. A snapshot that fails any check above is
    # never parsed at all, so no value from an untrusted file exists even
    # momentarily in a form a caller could reach.
    tree = _parse_snapshot(data, path)
    return ResolvedConfig(proc, tree, manifest, path)


def _parse_snapshot(data: bytes, path: str) -> Dict[str, Any]:
    """Parse the snapshot YAML into a tree.

    PyYAML is imported here rather than at module scope so that the boot_id gate,
    the manifest reader and the lint remain usable on a machine without it. The
    gate is the part that must work everywhere; a missing YAML parser should not
    be able to turn "your snapshot is from the last boot" into an ImportError
    three frames away from the real problem.
    """
    # Imported inside the function, not at module scope. The gate, the manifest
    # reader and the lint must all keep working on a machine without PyYAML --
    # the gate is the part that has to work everywhere -- and a module-level
    # import would turn "your snapshot is from the last boot" into an ImportError
    # raised three frames from the actual problem.
    try:
        import yaml  # noqa: PLC0415  (deliberate: see the docstring)
    except ImportError as exc:
        raise ResolvedConfigError(
            "PyYAML is required to read %s but is not installed" % path) from exc

    # safe_load, never load. The full loader constructs arbitrary Python objects
    # from tags in the document, and this file is written by another process --
    # so treating it as trusted input would make a compromised or corrupted
    # freeze line able to execute code in every process that starts afterwards.
    try:
        tree = yaml.safe_load(data.decode("utf-8"))
    except UnicodeDecodeError as exc:
        raise ResolvedConfigError("%s is not valid UTF-8: %s" % (path, exc)) from exc
    except yaml.YAMLError as exc:
        raise ResolvedConfigError("%s is not valid YAML: %s" % (path, exc)) from exc

    # An empty snapshot parses to None. Returning {} for it would make a process
    # with NO configuration indistinguishable from one whose configuration is
    # genuinely empty, and the first is a freeze-line failure.
    # An empty document parses to None, and returning {} for it would make a
    # process with NO configuration indistinguishable from one whose
    # configuration is genuinely empty. The first is a freeze-line failure that
    # must stop startup; the second is a process that would then run on nothing
    # but code defaults, which is the state CLAUDE.md 3.1 exists to prevent.
    if tree is None:
        raise ResolvedConfigError(
            "%s is empty. An empty snapshot is a freeze-line failure, not a "
            "process with no configuration" % path)
    # A YAML document may legally be a list or a scalar at the top level. Every
    # accessor here assumes a mapping, so the check belongs at the boundary
    # rather than as an AttributeError from inside get() much later.
    if not isinstance(tree, dict):
        raise ResolvedConfigError(
            "%s does not contain a mapping at the top level" % path)
    return tree
