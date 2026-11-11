"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_link_banner_view_js.py
Brief: HMI amber link / ESTOP banner (17 S6.3, NAV-64) -- hmi.js linkBannerView under node

Description:
Since 2026-11-11 (user) the HMI has ONE place for link and ESTOP sentences:
the amber banner under the toolbar. The blinking line beside the ESTOP button
is gone (it pushed the chassis-lock button and battery gauges sideways), so
everything it used to say must now come out of linkBannerView, one sentence by
priority:

  stop not sent > never connected > nothing for 2 s > socket down (REST still
  answering) > ESTOP path unconfirmed > path slow > path down > all well.

The load-bearing property: in every case where the web ESTOP cannot work the
banner says "请使用硬件急停" (NAV-64 / 11 S12.3 require the sentence to be
shown, not implied by a grey button).

How it runs: the block between the `// <link-banner-view>` markers is cut out
of hmi.js and evaluated under node. The robot has no node, so there the view
tests SKIP; the marker check runs everywhere.
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess

import pytest

_HMI_JS = os.path.join(os.path.dirname(__file__), "..", "..", "..", "xbrain",
                       "p5_gateway", "hmi", "static", "hmi.js")
_BEGIN = "// <link-banner-view>"
_END = "// </link-banner-view>"
HW = "请使用硬件急停"


def _block() -> str:
    with open(_HMI_JS, encoding="utf-8") as f:
        src = f.read()
    start = src.index(_BEGIN)
    return src[start:src.index(_END, start)]


def _views(cases):
    node = shutil.which("node")
    if node is None:
        pytest.skip("node not installed (the robot has none); markers still checked")
    script = (_block() + "\nconst cases = " + json.dumps(cases) + ";\n"
              "process.stdout.write(JSON.stringify(cases.map(linkBannerView)));\n")
    out = subprocess.run([node, "-e", script], capture_output=True, text=True,
                         timeout=30, check=True)
    return json.loads(out.stdout)


def _state(**kw):
    s = {"sendFailed": False, "everConnected": True, "fresh": True, "wsUp": True,
         "estopPath": "ok", "estopEnabled": True}
    s.update(kw)
    return s


def test_markers_present():
    assert "function linkBannerView" in _block()


def test_markup_has_no_notice_beside_the_estop_button():
    # RED MUTANT: put the old span back -> the toolbar jumps again.
    html = open(os.path.join(os.path.dirname(_HMI_JS), "index.html"),
                encoding="utf-8").read()
    assert 'id="estopNotice"' not in html
    assert "estopNotice" not in open(_HMI_JS, encoding="utf-8").read()


def test_all_well_hides_the_banner():
    (v,) = _views([_state()])
    assert v["show"] is False


@pytest.mark.parametrize("state, must_contain", [
    (_state(sendFailed=True), "急停指令未发出"),
    (_state(everConnected=False, fresh=False, wsUp=False, estopPath=None,
            estopEnabled=False), "正在连接机器人"),
    (_state(fresh=False, wsUp=False, estopEnabled=False), "链路中断"),
    (_state(wsUp=False, estopEnabled=False), "网页急停不可用"),
    (_state(estopPath=None, estopEnabled=False), "正在确认急停链路"),
    (_state(estopPath="down", estopEnabled=False), "急停链路不通"),
])
def test_every_unusable_estop_case_says_use_the_hardware_stop(state, must_contain):
    # RED MUTANT: drop the hardware-stop clause from any branch -> red here.
    (v,) = _views([state])
    assert v["show"] is True
    assert must_contain in v["text"]
    assert HW in v["text"]


def test_socket_down_with_estop_still_usable_says_so():
    # REST fallback still answers and the path is ok: the web ESTOP works.
    # RED MUTANT: always say "不可用" -> operators reach for the hardware stop
    # for nothing; always say "可用" -> red in the case above.
    (v,) = _views([_state(wsUp=False, estopEnabled=True)])
    assert "网页急停仍可用" in v["text"] and HW not in v["text"]


def test_degraded_warns_without_sending_to_the_hardware_stop():
    (v,) = _views([_state(estopPath="degraded")])
    assert "延迟高" in v["text"] and HW not in v["text"]


def test_priority_send_failure_beats_link_loss():
    # RED MUTANT: check link loss first -> "the stop did not go out" is hidden
    # behind a link message exactly when it matters most.
    (v,) = _views([_state(sendFailed=True, fresh=False, wsUp=False)])
    assert "急停指令未发出" in v["text"]


def test_priority_frozen_picture_beats_socket_down():
    (v,) = _views([_state(fresh=False, wsUp=False, estopEnabled=False)])
    assert "画面停在最后一帧" in v["text"]
