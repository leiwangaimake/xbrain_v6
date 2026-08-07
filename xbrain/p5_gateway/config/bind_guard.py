"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: bind_guard.py
Brief: GWY-P5-17 static guards -- refuse the bind shapes 17 S10.2 forbids, the
       smuggled pending key, and the dict-form retention_days

Description:
What this solves. HMI's 8080 carries the five unauthenticated writable ops
(estop, geo, task among them). Bound to 0.0.0.0 it sits on the chassis segment
and both device segments at once, so a taken-over camera can press estop
(17 S10.2 -- the hole U23's type whitelist cannot cover: that guards WHAT can be
sent, not WHO can connect). These checks make the forbidden shapes a startup
refusal naming the key, instead of a listening socket nobody audits.

What is covered HERE (static, on the parsed yaml) vs NOT covered:
  * covered -- 0.0.0.0 in hmi.bind or delivery.ftp.listen_address; a bind on a
    KNOWN forbidden segment (LAN1 10.21.33/31.x chassis, LAN3 192.168.144.x
    GZH-2, LAN4 192.168.66.x PTZ -- 11 S1.1.9.2); an unassigned (null) entry,
    refused with the key path (the U-15 LAN2 value is pending, and refusing is
    the designed behaviour for unfilled config); a pending_keys entry outside
    the three CR-EVT-1 keys (the escape hatch must not smuggle a fourth); a
    retention_days that is a mapping (the v0.1 {info: 7, ...} shape 17 S10.1
    struck -- criterion 5's mutation).
  * NOT covered -- what the process actually LISTENS on. That is the runtime
    `ss -lntup` + per-segment connect probe (17 S11 HMI 暴露面, CR-NET-1's
    script, shared with ND-2's 7447/7446 check) and lands with the Phase 2
    process. A static check cannot see a socket; claiming it could would be the
    scan-surface pretence CLAUDE.md 3.2 form 6 names.

Trap -- the asymmetry that decides every borderline here (17 S10.2): binding
too narrowly costs "one segment cannot reach HMI" (found immediately on site);
binding too widely costs "a taken-over camera can press estop" (never found).
So every unknown shape refuses; nothing is waved through as probably-fine.

Worked examples (each BAD line is a mutation test_bind_guard.py injects):
  OK   hmi.bind: ["<LAN2>:8080", "192.168.1.50:8080", "127.0.0.1:8080"]
  OK   hmi.bind: [null, null, "127.0.0.1:8080"]
       -- refused, but with "unassigned ... hmi.bind[0], hmi.bind[1]": the
       LAN2 address is U-15-pending, and refusing WITH the key path is the
       designed unfilled-config behaviour, distinct from a forbidden shape.
  BAD  hmi.bind: ["0.0.0.0:8080", ...]      -- the struck shape itself.
  BAD  hmi.bind: ["192.168.66.13:8080"]     -- no 0.0.0.0 in sight, but that
       is the LAN4 camera address: listing a device segment explicitly is the
       same exposure spelled politely, so the segment check exists SEPARATELY
       from the 0.0.0.0 check.
  BAD  hmi.bind missing / []                -- refused, never defaulted: a
       gateway falling back to a web framework's listen default gets exactly
       the 0.0.0.0 this file bans, via a path no yaml audit would show.
  BAD  startup.pending_keys: [..., "event/side_channel"]
       -- a fourth key riding the escape hatch; the hatch is for the three
       CR-EVT-1 keys only, or it becomes a generic registration bypass.
  BAD  event.retention_days: {info: 7, warn: 30}
       -- the struck v0.1 per-severity shape; one integer, or refusal.

The runtime half this file does NOT do, spelled out for the Phase 2
implementer (17 S11 "HMI 暴露面" row + CR-NET-1):
  * after the gateway binds, run `ss -lntup` and assert 8080 appears ONLY on
    LAN2 / wifi / lo interface addresses;
  * from a LAN1, LAN3 and LAN4 address each, attempt a TCP connect to 8080
    and assert refusal -- the probe, not the config, is the proof;
  * the same script carries ND-2's 7447/7446 checks (one script, CR-NET-1),
    so the two port sets cannot drift into separate half-run tools;
  * vsftpd's listen_address gets the identical treatment (COM-43).
"""

from typing import List, Tuple

from xbrain.common.errors import E_CONFIG_INVALID, XbrainError

__all__ = ["P5ConfigError", "check_p5_config", "FORBIDDEN_SEGMENTS",
           "PENDING_KEYS_ALLOWED"]

#: The segments an unauthenticated control plane must NEVER appear on
#: (11 S1.1.9.2 registration table): LAN1 chassis (both candidate ranges,
#: V-15 pending), LAN3 GZH-2 payload, LAN4 PTZ observation. wifi and lo are
#: allowed; LAN2's range is U-15-pending so it cannot be allowlisted yet --
#: which is fine, because the guard DENIES known-bad rather than allowing
#: known-good (an allowlist with an unknown member would refuse everything).
FORBIDDEN_SEGMENTS: Tuple[str, ...] = (
    "10.21.33.", "10.21.31.",       # LAN1 chassis (candidates, V-15)
    "192.168.144.",                  # LAN3 GZH-2
    "192.168.66.",                   # LAN4 PTZ
)

#: The only keys the startup self-check may exempt (17 S3.5.6): the three
#: CR-EVT-1 keys awaiting registration in 11 S2.2.5. Anything else in
#: startup.pending_keys is an unregistered key smuggled through the escape
#: hatch. Once CR-EVT-1 lands, the list in the yaml must become empty; this
#: constant then guards that no one re-adds to it.
PENDING_KEYS_ALLOWED: Tuple[str, ...] = (
    "event/replay", "event/recon/req", "event/recon/rsp",
)


class P5ConfigError(XbrainError):
    """A p5_gateway.yaml shape the gateway must not start on. The message names
    the key path, so the fix is a config edit, not a code dig."""

    def __init__(self, message: str):
        # E_CONFIG_INVALID from the shared export, never a literal (CLAUDE.md 3.5).
        super().__init__(E_CONFIG_INVALID, message)


def _check_bind_list(entries: object, keypath: str) -> List[str]:
    """Validate one bind/listen list; return the unassigned (null) key paths.

    Refuses (raises) on the actively-dangerous shapes; RETURNS the null entries
    so the caller can report every unfilled key in one refusal instead of one
    per run. An empty or missing list is itself a refusal: a gateway with no
    declared binds would fall back to some library default, and library
    defaults for "listen" are exactly the 0.0.0.0 this file exists to ban.
    """
    if not isinstance(entries, list) or not entries:
        raise P5ConfigError("%s must be a non-empty list" % keypath)
    unassigned: List[str] = []
    for i, entry in enumerate(entries):
        path = "%s[%d]" % (keypath, i)
        if entry is None:
            # Unassigned (LAN2 pending U-15 / wifi pending deployment). Not
            # dangerous, but not startable either -- collected, reported below.
            unassigned.append(path)
            continue
        if not isinstance(entry, str):
            # A number or mapping here is a yaml typo; refusing beats letting a
            # later int(entry.split(...)) crash without the key path.
            raise P5ConfigError("%s is not a string or null" % path)
        if "0.0.0.0" in entry:
            # The exact shape 17 S10.2 struck. Named with its consequence so
            # the person editing the yaml sees WHY, not just "invalid".
            raise P5ConfigError(
                "%s binds 0.0.0.0 -- that puts the unauthenticated control "
                "plane on the chassis and device segments (17 S10.2)" % path)
        if any(entry.startswith(seg) for seg in FORBIDDEN_SEGMENTS):
            raise P5ConfigError(
                "%s binds a forbidden segment (LAN1/LAN3/LAN4, 11 S1.1.9.2): %s"
                % (path, entry))
    return unassigned


def check_p5_config(mapping: dict) -> None:
    """Run every GWY-P5-17 static guard over a parsed p5_gateway.yaml.

    Raises P5ConfigError on the first violation; raises listing ALL unassigned
    keys if any bind entry is null (refusing to start on unfilled config is the
    designed behaviour -- 10 S5.4.2 R-3). Returns None when the shape is clean
    AND fully assigned.

    The check order, and why it is this order:

      1. mapping shape        -- the comment-only skeleton parses to None; every
         later check would TypeError on it with no filename in sight.
      2. retention_days shape -- both copies (event., delivery.); checked before
         the binds because it is the cheapest to fix and the refusal message
         should surface config-wide problems in a stable order across runs.
      3. pending_keys subset  -- BEFORE the binds on purpose: a smuggled key is
         an active security hole (an unregistered key exempted from the startup
         self-check), while a null bind is merely not-yet-filled; the more
         dangerous finding must not be shadowed by the more common one.
      4. both bind lists      -- dangerous shapes raise immediately inside
         _check_bind_list; null entries are only COLLECTED there and raised
         here as one combined message, so a deployment engineer filling in
         U-15 addresses sees every blank at once instead of one per restart.

    What a return (no raise) certifies -- and what it does not: the yaml SHAPE
    is safe and fully assigned. It does not certify what the process will bind
    at runtime; that proof is the CR-NET-1 probe (module docstring).
    """
    if not isinstance(mapping, dict):
        # The comment-only skeleton parses to None: same refusal as a missing file.
        raise P5ConfigError("p5_gateway.yaml is empty or not a mapping")

    # -- retention_days: single int, never the v0.1 mapping (criterion 5) ----
    # Both copies checked with one loop so they cannot drift: 17 S10.1 pins
    # delivery.retention_days to "与 events 一致", and two hand-written checks
    # is how one of them quietly stops being run.
    for keypath in ("event.retention_days", "delivery.retention_days"):
        section, leaf = keypath.split(".")
        # .get chains, not [] -- a MISSING section must fall through to the
        # not-an-integer refusal below (naming the key), not KeyError here.
        value = mapping.get(section, {}).get(leaf)
        if isinstance(value, dict):
            # The struck {info: 7, warn: 30, ...} shape: per-severity retention
            # re-introduces the split 17 S10.1 removed; assertion-C territory.
            raise P5ConfigError(
                "%s is the struck per-severity mapping; it must be one integer"
                % keypath)
        if not isinstance(value, int):
            raise P5ConfigError("%s must be an integer, got %r" % (keypath, value))

    # -- pending_keys: subset of the three CR-EVT-1 keys, nothing else -------
    # Subset, not equality: after CR-EVT-1 registers the three keys the yaml
    # list must shrink to [], and equality here would then refuse the correct
    # (empty) state -- the regression 17 S3.5.6 explicitly asks tests to keep.
    pending = mapping.get("startup", {}).get("pending_keys", [])
    if pending is None:
        pending = []                     # an explicit empty list post-CR-EVT-1
    for key in pending:
        if key not in PENDING_KEYS_ALLOWED:
            raise P5ConfigError(
                "startup.pending_keys carries %r -- the escape hatch is for "
                "the three CR-EVT-1 keys only (17 S3.5.6)" % key)

    # -- the two bind lists ---------------------------------------------------
    unassigned = _check_bind_list(
        mapping.get("hmi", {}).get("bind"), "hmi.bind")
    unassigned += _check_bind_list(
        mapping.get("delivery", {}).get("ftp", {}).get("listen_address"),
        "delivery.ftp.listen_address")
    if unassigned:
        # Every unfilled key in one message (a person fixes them in one edit).
        raise P5ConfigError(
            "unassigned bind entries (fill when U-15 / deployment assigns the "
            "addresses): %s" % ", ".join(unassigned))
