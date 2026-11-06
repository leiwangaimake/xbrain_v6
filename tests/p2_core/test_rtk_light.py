"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_rtk_light.py
Brief: 99 U142 -- red/blue flashes on a positioning loss, off on recovery, never against B / D mode

Description:
The light half of 11 S3.3.4 RL-3 (xbrain/p2_core/runtime/rtk_light.py).
Each case names the mutant it kills:
  * a loss switches red/blue on at the D / B pattern, a recovery off;
  * while B / D mode owns the lamp nothing is sent, but the loss is
    remembered so leaving the mode keeps the warning on;
  * heading_degraded and malformed details change nothing.
"""
from __future__ import annotations  # postponed annotations

import pytest  # marker

from xbrain.p2_core.runtime.rtk_light import RtkLight  # under test

pytestmark = pytest.mark.no_device  # pure

MODE = 7  # a deter_redblue_mode pattern number


def test_loss_on_recovery_off():
    """MUTATION: send strobe instead of redblue (00 S5.1.1: warning flashes
    are red/blue) -> red; forget to switch off -> red."""
    light = RtkLight(MODE)  # fresh
    assert light.on_event({"type": "rtk_lost"}, False) == {"redblue": MODE}  # on
    assert light.on_event({"type": "heading_recovered"}, False) == {"redblue": 0}  # off


def test_b_d_mode_owns_the_lamp_but_the_loss_is_remembered():
    """MUTATION: send while the mode owns the lamp -> flicker -> red; forget
    the loss -> leaving D switches the warning off mid-loss -> red."""
    light = RtkLight(MODE)  # fresh
    assert light.on_event({"type": "rtk_lost"}, True) is None  # D flashes already
    assert light.after_mode_exit() == {"redblue": MODE}  # still lost: keep flashing
    assert light.on_event({"type": "heading_recovered"}, True) is None  # mode still owns it
    assert light.after_mode_exit() == {"redblue": 0}  # recovered: off on exit


@pytest.mark.parametrize("detail", [{"type": "heading_degraded"}, None, "junk", {}])
def test_other_events_change_nothing(detail):
    """MUTATION: treat any event as a loss -> red."""
    light = RtkLight(MODE)  # fresh
    assert light.on_event(detail, False) is None and not light.on  # untouched
