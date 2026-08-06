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

*** L0 (code defaults) EXCLUDES four namespaces: common.safety.* .
common.spec.* . common.motion.profiles . common.fence.*

That exclusion was narrowed on 2026-08-05 and the reason is written into the
contract: it used to say "all", which meant a dataclass default could stand in
for ANY key including the safety parameters -- so a completely unfilled configs/
would still start, running hardcoded values, and assertion A could not see it.
A robot that starts on defaults nobody chose is worse than one that refuses to
start, because nothing reports it.

*** XBRAIN_CONFIG_DIR is NOT part of L5 (ENV-4). L5 overrides key VALUES; this
one decides where files come from and is resolved before L0~L5 run at all. And
ENV-2: the safety layer never follows it -- L3 always reads the compiled-in
/opt/xbrain_v6/configs/safety/, so pointing the config root at a test fixture
cannot swap the safety parameters underneath you.
"""

import os
from typing import Dict, FrozenSet, List, NamedTuple, Optional

from ..errors.exceptions import XbrainError

#: The one and only configuration root. 00 CFG-03: absolute path, plural, and
#: symlinks are not accepted.
DEFAULT_CONFIG_ROOT = "/opt/xbrain_v6/configs"

#: ENV-2: L3 reads from here regardless of XBRAIN_CONFIG_DIR.
SAFETY_ROOT = os.path.join(DEFAULT_CONFIG_ROOT, "safety")

#: L5 whitelist. 10 S5.4.3: three items, * 不得扩展.
ENV_WHITELIST: FrozenSet[str] = frozenset({
    "XBRAIN_ROBOT_ID", "XBRAIN_SITE_ID", "XBRAIN_LOG_LEVEL",
})

#: Which config key each whitelisted variable overrides.
ENV_KEY_MAP: Dict[str, str] = {
    "XBRAIN_ROBOT_ID": "common.robot_id",
    "XBRAIN_SITE_ID": "common.site_id",
    "XBRAIN_LOG_LEVEL": "common.log_level",
}

#: *** L0 may not supply these. See the module docstring for why.
L0_EXCLUDED_PREFIXES = (
    "common.safety.",
    "common.spec.",
    "common.motion.profiles",
    "common.fence.",
)


class ConfigLayerError(XbrainError):
    """A layer wrote outside its allowance, or a root/env rule was violated."""

    def __init__(self, message: str):
        super().__init__("E_CONFIG_INVALID", message)


class Layer(NamedTuple):
    """One layer of the overlay axis."""

    name: str          # L0 / L1 / ...
    what: str          # human label, English -- CLAUDE.md 2.1 requires
                       # exception messages to be entirely English
    allowed: tuple     # dotted prefixes this layer may write; () means unrestricted
    excluded: tuple    # dotted prefixes this layer may NOT write


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
    for key in flat_keys:
        for bad in layer.excluded:
            if key == bad or key.startswith(bad):
                raise ConfigLayerError(
                    f"{layer.name} ({layer.what}) must not supply {key!r}: "
                    f"prefix {bad!r} is excluded from this layer. "
                    "A code default standing in for a safety parameter starts the "
                    "robot on a value nobody chose, and assertion A cannot see it."
                )
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
    env = os.environ if env is None else env
    raw = env.get("XBRAIN_CONFIG_DIR")
    if raw is None:
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
    if not os.access(raw, os.R_OK):
        raise ConfigLayerError(
            f"ENV-1: XBRAIN_CONFIG_DIR={raw!r} is not readable; refusing to start"
        )
    return raw


def safety_root(env: Optional[Dict[str, str]] = None) -> str:
    """Where L3 reads from -- ENV-2, always the compiled-in path.

    The `env` argument is accepted and deliberately ignored so a caller cannot
    thread an override through by mistake; the signature matching
    resolve_config_root makes the asymmetry visible at every call site.
    """
    del env  # ENV-2: safety never follows XBRAIN_CONFIG_DIR.
    return SAFETY_ROOT


def env_overlay(env: Optional[Dict[str, str]] = None) -> Dict[str, str]:
    """L5 as a flat {dotted_key: value} map, whitelist enforced.

    * An XBRAIN_* variable outside the whitelist raises. Ignoring it would let a
    typo (XBRAIN_ROBOTID) look like it worked, and letting an arbitrary one
    through would make the whitelist decorative.
    """
    env = os.environ if env is None else env
    out: Dict[str, str] = {}
    for name, value in env.items():
        if not name.startswith("XBRAIN_"):
            continue
        if name == "XBRAIN_CONFIG_DIR":
            continue  # ENV-4: handled before the overlay axis, not part of L5.
        if name not in ENV_WHITELIST:
            raise ConfigLayerError(
                f"L5 whitelist is {sorted(ENV_WHITELIST)} and must not be "
                f"extended; got {name!r}"
            )
        out[ENV_KEY_MAP[name]] = value
    return out
