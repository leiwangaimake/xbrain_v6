"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: layers.py
Brief: The L0~L5 overlay stack and what each layer is allowed to write

Description:
10 S5.4.3 gives every layer a namespace allowance, and those allowances are the
only thing stopping a layer from quietly redefining something it has no business
touching. They are enforced here, not documented and hoped for.

THE STACK, lowest first. The order of LAYERS below IS the precedence order --
build_overlay walks that list and later layers win -- so reordering the list
changes which file decides a value, with nothing failing to announce it.

Only the source of each layer is listed here. The allowances are NOT restated:
they are in the LAYERS table below, and a second hand-kept copy of them is the
seven-parallel-lists failure this project has already measured twice.

  L0   dataclass / struct defaults in the code, no file
  L1   common.yaml
  L2   models/{model}.yaml
  L3   safety/*.yaml            -- read from SAFETY_ROOT, not the resolved root
  L4   sites/{site_id}.yaml
  L4b  calib/{robot_id}.yaml
  L5   environment variables    -- no file; see ENV_WHITELIST
  --   freeze line              common.* is final, common_digest is computed
  L6   {proc}.yaml              references only; after the freeze line, NOT here

WHY L4b IS A LAYER OF ITS OWN and not folded into L2 or L4 (10 S5.4.3, the
paragraph whose heading grep-matches "L4b 为什么必须是独立一层"). L2 is the model
layer, shared by every robot of that model; L4 is the site layer, shared by every
robot at that site. Extrinsics are a per-vehicle measured fact. Folded into L2,
one robot's calibration would pollute every robot of the same model; folded into
L4, a robot's calibration would change when it is driven to another site. L4 and
L4b are given disjoint namespaces, which is the reason they can share a tier
without any order between them having to be defined.

*** L0 (code defaults) EXCLUDES four namespaces: common.safety.* .
common.spec.* . common.motion.profiles . common.fence.*

That exclusion was narrowed on 2026-08-05 and the reason is written into the
contract: it used to say "all", which meant a dataclass default could stand in
for ANY key including the safety parameters -- so a completely unfilled configs/
would still start, running hardcoded values, and assertion A could not see it.
A robot that starts on defaults nobody chose is worse than one that refuses to
start, because nothing reports it.

The mechanism of that blindness is what makes the exclusion load-bearing rather
than merely tidy, so it is worth spelling out. Assertion A reports keys whose
value is null. A key that an L0 default filled is not null and is not absent
either, so A has nothing to say about it. Absent keys in these four namespaces
are what assertion M rejects -- and M can only ever see them if L0 was never
permitted to fill them in the first place. Widening L0 does not weaken one
assertion, it removes the only input the other one has.

WHEN THE ALLOWANCES ARE APPLIED. build_overlay checks each layer's flattened
keys BEFORE merging that layer in, never on the merged result. Checking after
the merge would still detect that a forbidden key is present but would have lost
the one fact worth reporting -- which layer put it there. The whole value of the
message is that it names a file the operator can open.

A consequence that surprises people: a null placeholder counts as a write for
this purpose. 10 S5.4.3 has common.yaml declare common.geo.enu_origin as a bare
null for the site layer to fill (the null row, anchor 声明键位但未赋值), and that
null flattens into L1's key set and is checked against L1's allowance even
though merge.deep_merge will let it override nothing. That is the intended
behaviour:
declaring a key position is a claim about ownership, and a layer with no business
owning a key has no business declaring it either.

*** XBRAIN_CONFIG_DIR is NOT part of L5 (ENV-4). L5 overrides key VALUES; this
one decides where files come from and is resolved before L0~L5 run at all. And
ENV-2: the safety layer never follows it -- L3 always reads the compiled-in
/opt/xbrain_v6/configs/safety/, so pointing the config root at a test fixture
cannot swap the safety parameters underneath you.

WHAT THIS MODULE DOES NOT DO. Each of the following has been mistaken for its
job at least once, and each belongs somewhere else:

  * it never opens a file. Nothing in here reads YAML. It decides which root a
    reader should use and which keys each layer may contribute; the reader hands
    build_overlay trees that are already parsed. If you find yourself adding an
    open() here, the layer boundary has moved and the freeze line moved with it.

  * it does not expand ${common.*} and does not detect reference cycles. That is
    the reference axis (CFG-CM-8 / CFG-CM-9). 10 S5.4.3 states that the entire
    L0~L5 overlay must finish before any reference is expanded, because a
    reference resolved mid-merge reads a value the site layer has not overridden
    yet -- and that failure is invisible whenever the lab site and the field site
    happen to agree on the value.

  * it does not implement ENV-3. When XBRAIN_CONFIG_DIR is in effect the contract
    additionally requires a WARN log, the real root recorded in MANIFEST, a
    startup event with detail.kind = "config_root_override", and a permanent HMI
    marker so a vehicle shipped with that variable set is visible at a glance.
    None of that happens here. resolve_config_root returns a path and nothing
    else -- a successful return does not mean the override has been recorded.

  * it does not validate values, only namespaces and roots. An empty
    XBRAIN_ROBOT_ID, a negative brake coefficient and a string where a float
    belongs all pass every check in this file.
"""

import os
from typing import Dict, FrozenSet, List, NamedTuple, Optional

# Imported from the package, not from .exceptions, and the code name comes with
# it. Going through the package means the codes.yaml table is parsed and
# validated before this module can raise anything -- so a malformed table stops
# the process at import, on the bench, instead of at the first configuration
# failure it was written to describe. resolved.py in this same package already
# imports its code this way.
from ..errors import E_CONFIG_INVALID, XbrainError

#: The one and only configuration root. 00 CFG-03: absolute path, plural, and
#: symlinks are not accepted.
#:
#: The plural spelling is not cosmetic. The retired singular relative form of
#: this path (CFG-ROOT-4) is a delivery blocker: 10 S12.4 SEC-11 item 4 greps
#: every systemd unit and startup script for it and requires zero hits, and
#: CFG-ROOT-4 adds that the loader offers no compatibility fallback -- a
#: fallback would make both roots legal at once. That is also why the retired
#: form is not written out anywhere in this file, not even to say "do not use
#: it" -- a criterion whose own text matches the pattern it scans for can never
#: reach zero, and the usual repair is to loosen the criterion until it passes.
#:
#: CONFIG-SOURCE-OK(freeze): this constant IS the source root, and the freeze
#: service is the one component that reads it. Every other consumer must go
#: through xbrain/common/config/resolved.py and read /run/xbrain/resolved/
#: instead; scripts/lint/no_config_source_read.py enforces that, and this marker
#: is the declared exemption it requires. See 10 S5.4.1 for why the split exists:
#: if each process expanded ${common.*} itself they would resolve to different
#: results, which the section calls the most critical structural decision here.
DEFAULT_CONFIG_ROOT = "/opt/xbrain_v6/configs"

#: ENV-2: L3 reads from here regardless of XBRAIN_CONFIG_DIR.
#:
#: Composed from DEFAULT_CONFIG_ROOT, deliberately not from resolve_config_root().
#: The obvious refactor -- join the resolved root with "safety" so there is only
#: one root expression in the file -- is the exact backdoor ENV-2 exists to shut:
#: with it, setting one environment variable would replace everything L3 owns --
#: configs/safety/brake.yaml (common.safety.t_lat_s, brake.a_mps2,
#: brake.safety_factor) and configs/safety/clock.yaml (common.safety.clock.*) --
#: bypassing CFG-11 and startup assertion G in a single step. Note the asymmetry
#: is deliberate and not an oversight to be tidied up the other way: L2 models/
#: and L4 sites/ DO follow the override, because that is what makes the loader
#: testable at all. safety/ is the one layer that must not, which is why it needs
#: its own constant. The contract answers the testability objection directly:
#: safety parameters are made testable by unit tests that inject concrete
#: numbers, never by pointing the loader at a different directory.
SAFETY_ROOT = os.path.join(DEFAULT_CONFIG_ROOT, "safety")

#: L5 whitelist. 10 S5.4.3: exactly these three, and the set must not be extended.
#:
#: Extending it is not a local decision. L5 sits above the site layer and the
#: calibration layer, so any variable admitted here can overrule a value that was
#: measured on a specific vehicle at a specific site, from a shell nobody audits.
#:
#: frozenset, not set, so "must not be extended" is also true at run time. A
#: mutable module-level set can be added to by any importer, and the addition
#: would be invisible in review of this file -- which is the only place a reader
#: would think to look for the whitelist.
ENV_WHITELIST: FrozenSet[str] = frozenset({
    "XBRAIN_ROBOT_ID", "XBRAIN_SITE_ID", "XBRAIN_LOG_LEVEL",
})

#: Which config key each whitelisted variable overrides.
#:
#: This map is also the source of L5's namespace allowance in LAYERS below, which
#: derives it with tuple(ENV_KEY_MAP.values()) rather than restating the paths.
#: Restating them would create the failure where a variable passes the whitelist
#: and is then rejected by the namespace check, because one of the two lists was
#: updated and the other was not.
ENV_KEY_MAP: Dict[str, str] = {
    "XBRAIN_ROBOT_ID": "common.robot_id",
    "XBRAIN_SITE_ID": "common.site_id",
    "XBRAIN_LOG_LEVEL": "common.log_level",
}

#: *** L0 may not supply these. See the module docstring for why.
#:
#: Three entries end in a dot and one does not, and the difference is load-bearing
#: rather than an oversight. The trailing dot makes the prefix match children
#: only, so "common.spec." cannot be satisfied by an unrelated sibling that merely
#: shares the leading letters. common.motion.profiles carries no dot because the
#: contract excludes that key itself -- the whole profile table is a single leaf
#: -- and check_namespace must reject both the bare key and anything beneath it.
#:
#: The dotless entry does over-reach slightly: startswith() also catches a
#: hypothetical common.motion.profiles_extra. That is left as is because the
#: error is in the direction of refusing to start, which is recoverable by a
#: human reading the message; the other direction is a robot running on values
#: nobody chose, which reports nothing.
L0_EXCLUDED_PREFIXES = (
    "common.safety.",
    "common.spec.",
    "common.motion.profiles",
    "common.fence.",
)


class ConfigLayerError(XbrainError):
    """A layer wrote outside its allowance, or a root/env rule was violated."""

    # One type for both failure families on purpose. They carry the same
    # closed-set code and there is nothing a caller could usefully do
    # differently: E_CONFIG_INVALID is group L, retryable "no", meaning the
    # startup self-check refuses to let the stack run. Splitting the type would
    # invite somebody to catch the family they judged milder.
    #
    # The code appears once, here, instead of at each raise site. That is what
    # keeps CLAUDE.md 3.5 satisfiable in practice -- a spelling repeated at every
    # raise site diverges in case or prefix, and the divergence only shows up
    # during integration, when the consumer branches on the code.
    #
    # And it is the imported constant, not the string. Written as a literal it
    # was a second source of truth for the spelling: renaming the code in
    # codes.yaml would leave this line intact and still importable, and the
    # mismatch would surface only when a consumer failed to match. Now the rename
    # breaks the import instead. scripts/lint/no_literal_ecode.py enforces this
    # tree-wide.

    def __init__(self, message: str):
        super().__init__(E_CONFIG_INVALID, message)


class Layer(NamedTuple):
    """One layer of the overlay axis."""

    # `allowed` empty and `excluded` empty are NOT the same thing, and the
    # asymmetry is why this is two fields rather than one:
    #   allowed  == ()  means unrestricted -- used by L0 alone
    #   excluded == ()  means nothing is forbidden -- used by every layer but L0
    #
    # L0 is expressed as an exclusion list rather than an allowance list because
    # it legitimately has to be able to default anything that is not
    # safety-relevant. An allowance list for L0 would have to enumerate the rest
    # of the tree, and would fall out of date the first time a new namespace was
    # added -- silently widening or narrowing L0 depending on which way it rotted.

    name: str          # L0 / L1 / ...
    what: str          # human label, English -- CLAUDE.md 2.1 requires
                       # exception messages to be entirely English, and this
                       # field is interpolated straight into them.
                       # It is prose for a human, NOT a path: the labels below
                       # mention directory names only to orient the reader, and
                       # nothing may parse a path out of them. The authoritative
                       # roots are DEFAULT_CONFIG_ROOT and SAFETY_ROOT
    allowed: tuple     # dotted prefixes this layer may write; () means unrestricted
    excluded: tuple    # dotted prefixes this layer may NOT write


# The precedence order. Read it as "each line may overwrite the lines above it".
#
# L4 and L4b are adjacent on purpose and their allowances do not intersect, so
# no order between the two has to be defined; see the module docstring.
#
# L5's allowance is derived from ENV_KEY_MAP rather than written out, so the
# whitelist and the namespace check cannot disagree.
#
# Note what L2's allowance permits and what it therefore cannot police. It ends
# in a dot and covers all of common.motion., which includes common.motion.profiles
# in full -- so L2 may legally restate the entire profile table. This check has no
# way to distinguish a model-specific override of a few entries from a wholesale
# second copy of the table, because both are the same set of keys. Keeping L2 from
# becoming a second source of truth for the profiles is a freeze-line assertion
# about duplicate definitions, and looking for it in this function is how it ends
# up implemented nowhere.
LAYERS: List[Layer] = [
    Layer("L0", "code defaults", (), L0_EXCLUDED_PREFIXES),
    Layer("L1", "shared common.yaml", ("common.",), ()),
    Layer("L2", "model models/", ("common.spec.", "common.motion."), ()),
    Layer("L3", "safety safety/", ("common.safety.",), ()),
    Layer("L4", "site sites/", ("common.geo.", "common.site.", "common.retention."), ()),
    Layer("L4b", "calibration calib/", ("common.calib.",), ()),
    Layer("L5", "environment variables", tuple(ENV_KEY_MAP.values()), ()),
]


def check_namespace(layer: Layer, flat_keys) -> None:
    """Raise if this layer writes a key it is not allowed to write.

    Both directions matter. `allowed` catches L2 trying to set common.safety.*;
    `excluded` catches L0 supplying a default for a safety parameter, which is
    the fail-silent case and the reason L0 is the only layer with an exclusion
    list rather than an allowance list.
    """
    # `flat_keys` is dotted LEAF paths from merge.flatten, not a nested tree.
    # Checking the tree instead would miss the case this whole function exists
    # for: a layer that writes a legal top-level key and an illegal one below it.
    #
    # Scope note: this validates key placement only. Types, ranges and "is this
    # value plausible" are somebody else's problem, and a key that passes here is
    # not thereby known to be usable.
    #
    # Matching is on the FULL dotted path, not on the top-level key. The cheaper
    # implementation -- assert the top-level key is "common" -- looks equivalent
    # and is not. CFG-FZ-16 mutation (1), a stray top-level speed_profiles block,
    # goes red under BOTH implementations, so passing it proves nothing; mutation
    # (2), common.safety.brake placed in an L2 model file, is the one that tells
    # them apart, because its top-level key is "common" like everybody else's.
    for key in flat_keys:
        for bad in layer.excluded:
            # Equality as well as prefix, because common.motion.profiles is
            # itself an excluded key and not only a namespace above one.
            if key == bad or key.startswith(bad):
                raise ConfigLayerError(
                    # In LAYERS as it stands only L0 carries an exclusion list,
                    # so this branch fires with layer.what == "code defaults".
                    # There is no file to open. That is why the message spells
                    # out the consequence instead of stopping at the rule: the
                    # person reading it is looking at a robot that refused to
                    # start and has to be told the fix is a dataclass default in
                    # the source, not a typo in configs/.
                    f"{layer.name} ({layer.what}) must not supply {key!r}: "
                    f"prefix {bad!r} is excluded from this layer. "
                    "A code default standing in for a safety parameter starts the "
                    "robot on a value nobody chose, and assertion A cannot see it."
                )
        # `if layer.allowed` is the unrestricted case, not an emptiness guard
        # bolted on to avoid an exception: L0 has no allowance list and must fall
        # through to the exclusion check alone.
        #
        # rstrip(".") admits the key that IS the namespace root. A layer may
        # write the root itself rather than something under it -- a site file
        # with `common: {geo: null}`, or with an empty `common: {geo: {}}`, which
        # merge.flatten deliberately treats as a leaf so the branch cannot vanish
        # from the key set. Either flattens to "common.geo", which does not
        # start with "common.geo." Without the rstrip the layer would be rejected
        # for writing inside its own namespace and the message would name a key
        # the reader can see is allowed, which reads as a loader bug.
        if layer.allowed and not any(key == a.rstrip(".") or key.startswith(a)
                                     for a in layer.allowed):
            raise ConfigLayerError(
                f"{layer.name} ({layer.what}) may only write "
                f"{', '.join(layer.allowed)} but writes {key!r}"
            )


def resolve_config_root(env: Optional[Dict[str, str]] = None) -> str:
    """The config root, honouring XBRAIN_CONFIG_DIR -- ENV-1 and ENV-4.

    ENV-4: this is resolved BEFORE L0~L5 run. It is not the fourth entry of the
    L5 whitelist, because L5 overrides key values while this decides which files
    are read at all.

    ENV-1: the variable must be an absolute path that exists and is readable.
    *** On any failure this raises rather than falling back to the default root.
    The contract calls silent fallback the worst outcome, and it is: a test
    pointing at a fixture that has a typo would quietly run against production
    configuration and pass.
    """
    # `env` is injectable so tests never have to mutate os.environ. Mutating the
    # real environment leaks across test cases in whatever order pytest happens
    # to run them, which turns an ENV-1 regression into an intermittent failure
    # somebody eventually marks flaky.
    env = os.environ if env is None else env
    raw = env.get("XBRAIN_CONFIG_DIR")
    if raw is None:
        # Unset is the normal, supported, shipped configuration. Only an explicit
        # value is subject to the checks below; there is no "empty means default"
        # rule, and an empty string falls into the not-absolute branch on purpose.
        return DEFAULT_CONFIG_ROOT
    if not os.path.isabs(raw):
        raise ConfigLayerError(
            f"ENV-1: XBRAIN_CONFIG_DIR={raw!r} is not an absolute path; "
            "refusing to start (no fallback to the default root)"
        )
    if not os.path.isdir(raw):
        raise ConfigLayerError(
            f"ENV-1: XBRAIN_CONFIG_DIR={raw!r} does not exist or is not a "
            "directory; refusing to start (no fallback)"
        )
    # os.access answers for the calling user at this instant, which makes it a
    # diagnostic and not a guarantee. Permissions can change between here and the
    # first read, and the directory can be swapped underneath. It earns its place
    # by turning the two failures the field actually sees -- directory not created
    # and permissions wrong -- into two distinguishable messages instead of one
    # traceback. NEVER treat a pass here as authorisation to skip error handling
    # around the reads themselves.
    if not os.access(raw, os.R_OK):
        raise ConfigLayerError(
            f"ENV-1: XBRAIN_CONFIG_DIR={raw!r} is not readable; refusing to start"
        )
    # NOT checked here: whether the root is a symbolic link. os.path.isdir follows
    # links, so a symlinked root passes. This is not an oversight to be fixed by
    # adding an islink() test -- 10 S5.4.0 item 4 rejects symlinks for the ROOT
    # only, while links inside the tree (the model "current" pointer, assertion I)
    # are legitimate and a blanket rejection would break them. The root-is-not-a-
    # symlink check belongs with the startup path assertions and the SEC-11
    # delivery checklist, which run against the final root once, not here where
    # the same test would have to be repeated at every call site.
    return raw


def safety_root(env: Optional[Dict[str, str]] = None) -> str:
    """Where L3 reads from -- ENV-2, always the compiled-in path.

    The `env` argument is accepted and deliberately ignored so a caller cannot
    thread an override through by mistake; the signature matching
    resolve_config_root makes the asymmetry visible at every call site.
    """
    # `del env` rather than renaming the parameter to `_env`. The underscore
    # spelling would silence the same lint while also hiding the asymmetry from
    # anyone reading the call site, and the asymmetry is the design: this
    # function takes the same argument as resolve_config_root and answers
    # differently, which is what a reader needs to notice.
    #
    # NEVER make this os.path.join(resolve_config_root(env), "safety"). It is the
    # refactor this function was written to prevent; see SAFETY_ROOT above for
    # what an environment variable would then be able to replace.
    del env  # ENV-2: safety never follows XBRAIN_CONFIG_DIR.
    return SAFETY_ROOT


def env_overlay(env: Optional[Dict[str, str]] = None) -> Dict[str, str]:
    """L5 as a flat {dotted_key: value} map, whitelist enforced.

    * An XBRAIN_* variable outside the whitelist raises. Ignoring it would let a
    typo (XBRAIN_ROBOTID) look like it worked, and letting an arbitrary one
    through would make the whitelist decorative.
    """
    # Returns flat dotted keys; build_overlay unflattens. Values stay exactly as
    # the environment gave them -- str, always, with no coercion and no trimming
    # -- so a consumer of common.log_level receives a string even if the same key
    # would have been typed in YAML.
    env = os.environ if env is None else env
    out: Dict[str, str] = {}
    for name, value in env.items():
        # The XBRAIN_ prefix is the ownership boundary. Everything else in the
        # environment (PATH, HOME, the ROS variables) belongs to somebody else
        # and is skipped in silence; only names claiming to be ours are policed.
        if not name.startswith("XBRAIN_"):
            continue
        if name == "XBRAIN_CONFIG_DIR":
            continue  # ENV-4: handled before the overlay axis, not part of L5.
        # ORDER IS LOAD-BEARING: the XBRAIN_CONFIG_DIR skip above must come
        # before this test. Swap the two and merely having the documented,
        # supported root override set would raise here and the stack would
        # refuse to start -- the whitelist would reject the one variable the
        # contract deliberately placed outside it.
        if name not in ENV_WHITELIST:
            raise ConfigLayerError(
                f"L5 whitelist is {sorted(ENV_WHITELIST)} and must not be "
                f"extended; got {name!r}"
            )
        # An empty value is still a value and still overrides. XBRAIN_ROBOT_ID=""
        # yields common.robot_id = "" from L5, above whatever the site file said,
        # because "set to empty" and "not set" are different states of the
        # environment and only the second one is absence. Treating empty as unset
        # here would be a default-fallback in disguise, which CLAUDE.md 3.1 rules
        # out; rejecting it would be value validation, which this file does not do
        # (see the module docstring).
        out[ENV_KEY_MAP[name]] = value
    return out
