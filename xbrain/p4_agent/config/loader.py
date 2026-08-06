"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: loader.py
Brief: P4's only way to obtain configuration, plus the startup refusals 16 owns

Description:
What problem this solves. Two of them, and they are separate on purpose.

The first is structural and belongs to every process alike: P4 must read the
freeze line's product under /run/xbrain/resolved/ and must never read the
configuration source. 10 S5.4.1 calls "references are not expanded by each
process individually" the most critical structural decision in the design, and
gives the reason -- each process would otherwise resolve them to different
results. That half is already solved by xbrain/common/config/resolved.py, which
runs the boot_id gate, confines every path it opens to the resolved root, and
offers no parameter that would relax either. This module does NOT reimplement
any of it; it calls it. A second reader would be a second place for the rule to
live, and the copy nobody remembers is the one that stops matching.

The second is P4's own and has no home anywhere else: 16 S12.4's version table
states verbatim (grep 启动校验) that P4 validates min_agent_version at startup
and refuses to start with a clear error when it is not satisfied, and 16 S14
states verbatim of prompt.history.enable_on that it is a closed set, that an
empty list means all off, and that an unknown value means 启动即拒. Those are
refusals the freeze line does not perform -- they are not in the 10 S5.4.4
assertion table -- so if this module does not perform them nothing does.

*** Why the null gate here is a SECOND door and not a duplicated check. 10
S5.4.4 assertion A is verbatim 解析后全树扫描: 残留 ${, 未解析路径, null 未赋值,
and its executor is xbrain-config-freeze.service. Reaching this module with a
null in the tree therefore means the freeze line did not run, or ran and let it
through. Continuing would hand P4 a value it will treat as configured. 10 S5.4.5
argues the two-door shape explicitly for the 0.0 case -- 两道门抓的不是同一件事
-- and the same reasoning applies: the freeze line refuses the whole stack, this
refuses this process, and losing either one degrades a startup refusal into a
run-time surprise.

*** What this module cannot see, said plainly so the pass is not read as more
than it is. A safety value written 0.0 instead of null is indistinguishable from
a genuine zero here, and it is the more dangerous of the two: it passes every
check in this file and then limits something to zero at run time with nothing
logged. The door for that is 10 S5.4.4 assertion G, whose evaluation list
(11 S9.6, verbatim SP-1 ~ SP-7 plus SP-9 / SP-10 plus SP-11 plus AS-7) is
range-checking, and its executor is the freeze service. Not this module, and it
must not be made to look as though it were.

What this module deliberately does NOT do:
  * it does not read the configuration source, and offers no switch that would.
    CLAUDE.md 3.6 forbids any switch that skips a safety check on the grounds
    that the switch itself is the channel for disabling the constraint;
  * it does not expand ${common.*}. That happened at the freeze line;
  * it does not evaluate SP-11 (0 < gpu_token.throttle_speed_mps <
    common.spec.max_vx_mps). 11 S9.6's SP-11 row registers the executor as
    xbrain-config-freeze.service and gives the reason -- putting it in the
    process means the process is already up before the configuration is found
    to be wrong -- and its other operand lives in the common namespace, which
    this L6 snapshot does not carry;
  * it does not consume gateway.tts.*. 16 S14 states of that block that whether
    the section should exist at all cannot be decided (grep
    是否存在都定不下来), and 16 S16 Q-P4-32 records that the correct closure may
    be deleting it outright. Code that read it would be deciding that question;
  * it does not validate grammar.fallback_on_overflow against a closed set.
    16 S16 Q-P4-2 records verbatim that no legal-value set for it is registered
    anywhere, and that registering one is a choice belonging to volume 11.
    Inventing a set here would create a second source of truth for it, in the
    one place that has no authority to define it at all.
"""

from typing import Any, Dict, List, Optional

from ...common.config.resolved import (BOOT_ID_PATH, RESOLVED_ROOT, Manifest,
                                       ResolvedConfig, load_resolved)
from ...common.errors import E_CONFIG_INVALID
from ...common.errors.exceptions import XbrainError
from .version import satisfies

#: The process name the freeze line writes a snapshot under. It is already a
#: member of resolved.SNAPSHOT_PROCESSES, and passing it as a constant rather
#: than spelling the string at each call site means a rename shows up as one
#: edit instead of as a file-not-found somewhere unrelated.
P4_PROCESS = "p4_agent"

#: The three scenarios the one-line history may be enabled for, from the title
#: of 16 S6.3.5 (grep 三场景) and from the enable_on line in 16 S14.
#:
#: Why this closed set lives here and not in xbrain/common/enums/. That package
#: mirrors the contract's closed sets from 11 S13 and nothing else -- plane,
#: domain, event_category, gate_limiter, stop_reason, task_state, cls, device --
#: and its whole value is that a metatest can compare it back against 11. This
#: set is defined by 16, so putting it there would add a member that 11 does not
#: own and the metatest would then be comparing 11 against something 11 never
#: said. Kept local, with the anchor written down instead.
#:
#: A tuple and not a set, because check_history_scenarios reports the permitted
#: values in a message and an unordered set would print them differently from
#: run to run -- which makes two identical failures look like two failures.
HISTORY_SCENARIOS = ("clarify", "recording", "pending_confirm")


class P4ConfigError(XbrainError):
    """A P4 configuration that must not be started on.

    One type for all of the refusals in this file rather than one per cause.
    The caller's response is identical in every case -- do not start -- so
    splitting the type would offer a choice that does not exist, and would
    invite somebody to catch one kind and carry on, which is exactly what must
    not be possible here.

    Distinct from ResolvedConfigError even though both mean "refuse", because
    the two point at different owners: ResolvedConfigError means the snapshot
    or the freeze line is wrong, while this means the CONTENT is wrong for this
    implementation. Both derive from XbrainError, so a caller that genuinely
    wants to treat them alike still can with one except clause.
    """

    def __init__(self, message: str, code: str = E_CONFIG_INVALID):
        super().__init__(message)
        # The closed-set code from the shared library, never a literal
        # (CLAUDE.md 3.5). E_CONFIG_INVALID is what 10 S5.4.4 assigns to this
        # whole family: configuration that fails its own self-check, startup
        # refused, not retryable and not degraded through.
        self.code = code


def check_min_agent_version(cfg: ResolvedConfig, agent_version: str) -> None:
    """16 S12.4: refuse to start when the cmdset needs a newer P4 than this one.

    *** agent_version is a required argument with no default, and that is the
    load-bearing part of this function. No volume states what version this
    implementation is -- a grep for agent_version across docs/ finds only the
    requirement side, in 16 S12.4 and 16 S14 -- so a constant here would have
    been invented, and an invented constant chosen by the same person who wrote
    the check makes the check pass by construction. Requiring the caller to
    state its own version makes the missing fact visible at the call site
    instead of hidden in this file.

    Why the failure is a refusal rather than a warning. 16 S12.4 gives the
    increment rules: minor means an intent was added and stays backward
    compatible, major means semantics changed or an intent was deleted. So a
    cmdset above this implementation is not one carrying commands P4 does not
    know -- P4 would simply not match those -- it is one where a command P4
    DOES know now means something else. Continuing means executing the wrong
    action for a phrase the operator has used a hundred times.
    """
    # require(), never get() with a default. CLAUDE.md 3.1 forbids the default
    # fallback shape outright, and here it would also be nonsense: a default
    # minimum would mean "when the configuration does not say which P4 it
    # needs, assume this one will do", which is the entire question being
    # asked.
    minimum = cfg.require("cmdset.min_agent_version")
    # satisfies() raises ValueError on a malformed version on either side. Not
    # caught and converted to False here: False would refuse to start, which
    # looks safe, but it merges "the value is wrong" with "the value is too
    # low" into one message and the first needs a different fix. Converted to
    # P4ConfigError so the caller still sees one exception family, with the
    # original kept as the cause.
    try:
        ok = satisfies(agent_version, minimum)
    except ValueError as exc:
        raise P4ConfigError(
            "%s: cannot compare versions from %s: %s. Startup is refused "
            "rather than continuing on an unparseable version, because a "
            "version that cannot be ordered cannot be shown to be satisfied "
            "(16 S12.4)" % (P4_PROCESS, cfg.path, exc)) from exc
    if not ok:
        # Both values in the message, and the key path too. "version mismatch"
        # alone leaves an operator unable to tell whether to roll the cmdset
        # back or the implementation forward, and those are different actions
        # taken by different people.
        raise P4ConfigError(
            "%s: cmdset requires min_agent_version %r but this implementation "
            "is %r. Startup is refused (16 S12.4): a cmdset above the running "
            "implementation contains commands whose meaning has changed, so "
            "running it would execute the wrong action for a known phrase. "
            "Key path cmdset.min_agent_version in %s"
            % (P4_PROCESS, minimum, agent_version, cfg.path))


def check_no_unassigned_keys(cfg: ResolvedConfig) -> None:
    """The second door for 10 S5.4.4 assertion A, evaluated on this snapshot.

    Every hole is reported at once rather than one per restart. Filling
    configuration is done by a person with the volumes open, and a checker that
    surfaces one key at a time turns that into a dozen boot cycles -- after
    which the natural request is for a switch that lets it start anyway, and
    that switch is what CLAUDE.md 3.6 forbids.

    *** The test is `is None`, never `not v`. 0, "" , [] and false are assigned
    values here, and prompt.history.enable_on is legitimately an empty list --
    16 S14 states verbatim 空列表 = 全关. Reporting those as holes would send
    somebody to fill in keys that are already correct, and would train the
    reader to skim the list, which is how the real hole gets missed. The
    ResolvedConfig.unassigned() this delegates to is written that way.
    """
    holes: List[str] = cfg.unassigned()
    if not holes:
        return
    raise P4ConfigError(
        "%s: %d key(s) in %s are declared but never assigned: %s. These are "
        "null in the configuration source because the value has not been "
        "decided or measured yet, and 10 S5.4.4 assertion A should already "
        "have refused the whole stack at the freeze line -- that this process "
        "got as far as reading them is itself worth reporting. Startup is "
        "refused; do NOT fill a placeholder number in to make it run "
        "(CLAUDE.md 3.1)"
        % (P4_PROCESS, len(holes), cfg.path, ", ".join(holes)))


def check_history_scenarios(cfg: ResolvedConfig) -> None:
    """16 S14: prompt.history.enable_on is a closed set, unknown value refuses.

    Two checks, because the section states two things and only one of them is
    about enable_on:

      * every member of enable_on is one of HISTORY_SCENARIOS. 16 S14's own
        comment on that line is verbatim 闭集, 空列表 = 全关; 未知值 -> 启动即拒;
      * line_templates carries exactly one entry per scenario, from 16 S6.3.5's
        固定 1 行的模板, 三场景各一条.

    *** The second is a bidirectional difference and not a subset test, and
    the direction that is easy to forget is the one that matters. A subset test
    -- every enabled scenario has a template -- passes on a file where a
    template was misspelled AND the corresponding scenario was left out of
    enable_on, which is precisely what a careless edit produces. The symptom
    would be a history line that silently never appears, which reads on site as
    the model having forgotten the clarification it just asked.

    Out-of-set values raise; they are never passed through and never
    interpreted as the nearest known value. 11 S13.6 requires that in so many
    words, and 16 S13's GATE-2 row shows the shape it forbids -- do not treat
    an unrecognised value as the permissive one.
    """
    enable_on: Any = cfg.require("prompt.history.enable_on")
    # Type-checked before iteration. A YAML author who writes
    # `enable_on: clarify` without brackets produces a string, and iterating a
    # string yields characters -- so the membership test below would report
    # "unknown scenario 'c'" and send the reader looking in entirely the wrong
    # place.
    if not isinstance(enable_on, list):
        raise P4ConfigError(
            "%s: prompt.history.enable_on is %r in %s; it must be a list of "
            "scenario names (16 S6.3.5). A bare string here is a YAML author "
            "omitting the brackets, not a one-element list"
            % (P4_PROCESS, enable_on, cfg.path))
    unknown = [v for v in enable_on if v not in HISTORY_SCENARIOS]
    if unknown:
        raise P4ConfigError(
            "%s: prompt.history.enable_on carries value(s) %s that are outside "
            "the closed set %s (16 S6.3.5). 16 S14 states an unknown value "
            "refuses startup; an out-of-set value is never passed through and "
            "never read as the nearest known one (11 S13.6). Key path "
            "prompt.history.enable_on in %s"
            % (P4_PROCESS, unknown, list(HISTORY_SCENARIOS), cfg.path))

    templates: Any = cfg.require("prompt.history.line_templates")
    if not isinstance(templates, dict):
        raise P4ConfigError(
            "%s: prompt.history.line_templates is %r in %s; it must be a "
            "mapping of scenario name to template"
            % (P4_PROCESS, templates, cfg.path))
    # Sorted on both sides so two runs over the same broken file produce the
    # same message. An unordered difference printed straight out makes one
    # defect look like several.
    missing = sorted(set(HISTORY_SCENARIOS) - set(templates))
    extra = sorted(set(templates) - set(HISTORY_SCENARIOS))
    if missing or extra:
        raise P4ConfigError(
            "%s: prompt.history.line_templates does not match the scenario "
            "closed set in %s -- missing %s, unexpected %s. 16 S6.3.5 requires "
            "one fixed line per scenario, and a missing template shows up on "
            "site as a history line that silently never appears rather than as "
            "an error" % (P4_PROCESS, cfg.path, missing, extra))


def load_p4_config(agent_version: str,
                   root: str = RESOLVED_ROOT,
                   boot_id_path: str = BOOT_ID_PATH,
                   manifest: Optional[Manifest] = None) -> ResolvedConfig:
    """*** The one entry point P4 uses to obtain its configuration.

    Order of operations, and none of it is interchangeable:
      1. the shared reader loads the snapshot. That runs the boot_id gate before
         any content is opened and confines the path it reads to the resolved
         root, so a stale or misdirected snapshot never reaches step 2;
      2. min_agent_version. Asked first among this module's own checks, because
         it decides whether this configuration is meant for this implementation
         at all -- listing the holes in a configuration that cannot be run is
         answering a question nobody asked;
      3. the null gate, reporting every hole at once;
      4. the closed-set checks, which can only be meaningful once step 3 has
         established that the values exist.

    Returns the shared ResolvedConfig rather than a P4-specific wrapper. A
    wrapper with typed accessors would need a field per key, and every one of
    those fields would be a place where a default could later be added -- which
    is the one thing CLAUDE.md 3.1 forbids. Callers use require() for values
    they cannot run without and get() for genuinely optional ones, and the
    difference stays visible at the call site.

    `manifest` is forwarded for a caller that already loaded it. A Manifest
    cannot be constructed without passing the boot_id gate, so accepting one is
    not a way around it.

    There is no parameter that relaxes any of this, and none may be added.
    """
    cfg = load_resolved(P4_PROCESS, root=root, boot_id_path=boot_id_path,
                        manifest=manifest)
    check_min_agent_version(cfg, agent_version)
    check_no_unassigned_keys(cfg)
    check_history_scenarios(cfg)
    return cfg


def startup_report(cfg: ResolvedConfig) -> Dict[str, Any]:
    """The identifying fields a P4 startup event should carry.

    Gathered here so the manifest values and the snapshot values are taken from
    ONE load. A caller assembling them separately can end up reporting a
    common_digest from one load beside a cmdset version from another, and the
    resulting event is worse than none: it looks like evidence.

    10 S5.4.4 is explicit that such an event must reuse an EXISTING key rather
    than open a new one -- a new key would hit the 11 S2.2 key table, the
    S2.4.7 QoS binding table and the S1.1.6 cross-plane whitelist at once, for
    no gain, since consistency is already structural via the freeze line. So
    this returns fields and defines no transport of its own.

    common_digest is reported and NOT compared. 10 S3.3.3 puts that comparison
    at P2's Stage D with failure behaviour B -- keep motion disallowed, emit a
    fault -- not the refusal this module applies. Comparing it here would apply
    the wrong severity to a condition the design wants observable.
    """
    return {
        "proc": cfg.proc,
        "config_rev": cfg.manifest.config_rev,
        "common_digest": cfg.manifest.common_digest,
        # From the snapshot rather than from any constant, so the number in the
        # event is the one the process is actually running on.
        "cmdset_version": cfg.require("cmdset.version"),
        "min_agent_version": cfg.require("cmdset.min_agent_version"),
    }
