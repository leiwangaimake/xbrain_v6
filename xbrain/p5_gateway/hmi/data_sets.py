"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: data_sets.py
Brief: GWY-P5-19/20/21 HMI data v1.0 A-F sets + gen_hmi_dataset + UI constraints

Description:
17 S18 HMI data v1.0 has SIX groups (A-F):

  A  robot pose / velocity
  B  active tasks + progress
  C  fences / geometry
  D  events (last N)
  E  telemetry (last N per class)
  F  approvals (pending only)

gen_hmi_dataset (GWY-P5-20) is a generator that reads a single
source table (17 S18.T) and emits the dataset for each group; there
is intentionally ONE truth-source so front and back agree.

UI constraints (GWY-P5-21):
  * rotate buttons: DISABLED whenever rotation_permit is denied
    (both categories -- geometric and legal -- surfaced separately)
  * PTZ angle DISPLAY suppressed for security compliance (angles
    reveal camera aim to onlookers viewing the HMI screen);
    payload STATE still carries angles for control loops
"""

from __future__ import annotations

from dataclasses import dataclass


HMI_DATA_GROUPS = frozenset({"A", "B", "C", "D", "E", "F"})


@dataclass(frozen=True)
class UiRotationState:
    """Two separate flags: the operator sees WHICH kind of denial
    it was so they know whether to move the robot or wait for
    permit."""
    geometric_denied: bool
    legal_denied: bool

    def rotate_button_enabled(self) -> bool:
        return not (self.geometric_denied or self.legal_denied)


def redact_ptz_angle_for_ui(payload: dict) -> dict:
    """Return a copy with pan/tilt DISPLAY values scrubbed. State
    downstream (state topic) is unchanged; only what the HMI shows
    is redacted."""
    scrubbed = dict(payload)
    for k in ("pan_deg", "tilt_deg"):
        if k in scrubbed:
            scrubbed[k] = None
    return scrubbed


class UnknownDataGroup(Exception):
    pass


def validate_group(name: str) -> None:
    if name not in HMI_DATA_GROUPS:
        raise UnknownDataGroup(name)
