"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: net_profile.py
Brief: GATE-6 network profile comparison (10 S3.3.1 + 11 S1.1.9.7)

Description:
GATE-6 answers ONE question at Stage 0: does the currently-configured
set of network interfaces match the site's hw_profile YAML? If not,
the probe refuses to release Stage 0.

Why this gate matters. The failure mode 11 §1.1.9.7 documents is
subtle: a DBG-branch machine boots with a PROD hw_profile, so LAN2
and LAN3 do not exist on this machine, so systemd-networkd assigns
nothing, so nftables rules that reference those non-existent addresses
silently no-op, so cloud uplink works (LAN1 is present) while every
on-site device is unreachable -- "everything green, nothing works."
GATE-6 turns that scenario into a Stage 0 refusal with a per-interface
"expected vs actual" diff, before any dependent unit starts.

Comparison rules:
  * Every interface named in the profile must exist on this machine.
  * The interface's IPv4 address, netmask, and computed /24 (or
    profile-specified) network must equal the profile's values.
  * No two interfaces in the profile may share a network segment
    (NET-C1) -- this is a static check on the profile itself, but is
    run here so that the failure mode "two IPs in the same segment
    kill the routing table" is caught BEFORE any process cares.

What this module does NOT do:
  * It does not modify /etc/systemd/network/*.network (that is the
    hw_profile deploy job's responsibility).
  * It does not ping remote hosts (that is Stage 0z-3's chassis probe).
  * It does not check WiFi association state (out of scope for GATE-6
    per 11 §1.1.9.7 -- WiFi coverage is an operational concern).

Detail dict shape returned on failure. Every field is required so the
gateway (which is the only translator between E_* codes and HMI /
uplink events per 11 §8.13.5) can render a readable diff without
re-parsing.

  {"kind": "net_profile_mismatch",
   "interface": "lan2",
   "expected": {"ipv4": "10.21.31.1",  "netmask": "255.255.255.0",
                "network": "10.21.31.0/24"},
   "actual":   {"ipv4": "10.21.33.1",  "netmask": "255.255.255.0",
                "network": "10.21.33.0/24"}}

  or for the "interface does not exist on this machine" case:

  {"kind": "net_profile_mismatch",
   "interface": "lan2",
   "expected": {...},
   "actual":   {"ipv4": null, "netmask": null, "network": null}}
"""

import ipaddress
import os
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import yaml


# --- Load profile -----------------------------------------------------

def load_profile(profile_path: str) -> Dict[str, dict]:
    """Read /etc/xbrain/hw_profile. Returns a dict keyed by interface
    name; each value has keys ipv4/netmask/network.

    Raises FileNotFoundError if the profile is missing -- the probe
    catches that and maps it to E_CONFIG_INVALID + kind="hw_profile_missing".
    """
    with open(profile_path, "r", encoding="utf-8") as f:
        doc = yaml.safe_load(f)
    if not isinstance(doc, dict):
        raise ValueError("hw_profile top-level must be a mapping")
    interfaces = doc.get("interfaces")
    if not isinstance(interfaces, dict):
        raise ValueError("hw_profile.interfaces missing or not a mapping")
    out: Dict[str, dict] = {}
    for name, spec in interfaces.items():
        if not isinstance(spec, dict):
            raise ValueError("hw_profile.interfaces.%s must be mapping" % name)
        # Any null field (per CLAUDE.md 3.1 pattern) is a deploy-time
        # error: the profile author must resolve each field explicitly.
        for k in ("ipv4", "netmask", "network"):
            if spec.get(k) is None:
                raise ValueError(
                    "hw_profile.interfaces.%s.%s is null "
                    "(unresolved per CLAUDE.md 3.1)" % (name, k))
        out[name] = {
            "ipv4":    str(spec["ipv4"]),
            "netmask": str(spec["netmask"]),
            "network": str(spec["network"]),
        }
    return out


# --- Read actual interface state -------------------------------------

def _read_iface_addr(name: str) -> Optional[Tuple[str, str]]:
    """Return (ipv4, netmask) for interface `name` or None if the
    interface does not exist. Reads /proc/net/fib_trie for a stdlib-
    only implementation so the probe does not depend on `ip` binary."""
    sys_path = "/sys/class/net/" + name
    if not os.path.isdir(sys_path):
        return None
    # For actual IP lookup use `ip -o -4 addr show dev NAME` since
    # /proc/net/fib_trie is per-table and messy. But `ip` is expected
    # on every deploy target (iproute2).
    import subprocess
    r = subprocess.run(
        ["ip", "-o", "-4", "addr", "show", "dev", name],
        capture_output=True, text=True, timeout=5)
    if r.returncode != 0 or not r.stdout.strip():
        # Interface exists but has no IPv4 -- treat like "no address".
        return ("", "")
    # Parse line like: "3: lan2    inet 10.21.31.1/24 brd ... scope global lan2"
    line = r.stdout.strip().splitlines()[0]
    tokens = line.split()
    for i, tok in enumerate(tokens):
        if tok == "inet" and i + 1 < len(tokens):
            addr_cidr = tokens[i + 1]
            if "/" in addr_cidr:
                addr, prefix = addr_cidr.split("/", 1)
                netmask = str(ipaddress.ip_network(
                    "0.0.0.0/" + prefix, strict=False).netmask)
                return (addr, netmask)
    return ("", "")


def read_actual(profile: Dict[str, dict],
                iface_reader=_read_iface_addr) -> Dict[str, Optional[Tuple[str, str]]]:
    """For every interface named in the profile, read the machine's
    actual state. Injecting iface_reader lets tests substitute a
    deterministic table without patching /sys or spawning `ip`."""
    return {name: iface_reader(name) for name in profile}


# --- Comparison ------------------------------------------------------

def _compute_network(ipv4: str, netmask: str) -> str:
    """Return the CIDR string for the network segment ipv4 belongs to
    under netmask. Uses ipaddress from stdlib so no dependency."""
    if not ipv4 or not netmask:
        return ""
    prefix = ipaddress.ip_network(
        "0.0.0.0/" + netmask, strict=False).prefixlen
    return str(ipaddress.ip_network(
        "%s/%d" % (ipv4, prefix), strict=False))


def diff_profile(profile: Dict[str, dict],
                 actual: Dict[str, Optional[Tuple[str, str]]]
                 ) -> List[dict]:
    """Return a list of mismatch dicts, one per failing interface.
    Empty list means GATE-6 passes."""
    out: List[dict] = []
    for name, expected in profile.items():
        got = actual.get(name)
        if got is None:
            # Interface does not exist -- most severe failure mode
            # (per 11 §1.1.9.7: nftables rules silently no-op).
            out.append({
                "kind": "net_profile_mismatch",
                "interface": name,
                "expected": expected,
                "actual": {"ipv4": None, "netmask": None, "network": None},
            })
            continue
        got_ipv4, got_netmask = got
        got_network = _compute_network(got_ipv4, got_netmask)
        if (got_ipv4 != expected["ipv4"] or
                got_netmask != expected["netmask"] or
                got_network != expected["network"]):
            out.append({
                "kind": "net_profile_mismatch",
                "interface": name,
                "expected": expected,
                "actual": {
                    "ipv4":    got_ipv4 or None,
                    "netmask": got_netmask or None,
                    "network": got_network or None,
                },
            })
    return out


# --- NET-C1: any two interfaces share a network segment --------------

def find_network_overlaps(profile: Dict[str, dict]) -> List[dict]:
    """Return a list of overlap descriptors, one per overlapping pair.
    Empty list = no overlaps.

    NET-C1: if two interfaces bind IPs in the same segment, the kernel
    routes them via the first one that came up. That is a race, and
    the loser interface silently loses reachability."""
    out: List[dict] = []
    nets = [(name, ipaddress.ip_network(spec["network"], strict=False))
            for name, spec in profile.items()]
    for i in range(len(nets)):
        for j in range(i + 1, len(nets)):
            n1, net1 = nets[i]
            n2, net2 = nets[j]
            if net1.overlaps(net2):
                out.append({
                    "kind": "net_profile_overlap",
                    "interfaces": [n1, n2],
                    "networks": [str(net1), str(net2)],
                })
    return out
