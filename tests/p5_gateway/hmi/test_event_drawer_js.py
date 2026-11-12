"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_event_drawer_js.py
Brief: HMI event drawer -- Chinese wording, "持续中" families, pinned alarms, chips (17 S6.6)

Description:
Pins the pure part of the bottom event drawer (user design 2026-11-12 with the
eight recommendations "按推荐来"): the block between `// <event-text>` markers in
hmi.js is cut out and run under node, like test_map_marks_js.py. The robot has
no node, so there the node cases SKIP; the markup checks run everywhere.

What a careless drawer gets wrong, one test each:
  * an unmapped event vanishing or worded by guess (it must show its title);
  * the low-battery alarm, repeating every 2 s, shown as a closed record while
    it is still running, or still "持续中" after low_battery_cleared;
  * an alarm of a running family pinned twice (once as 持续中, once as alarm);
  * the 告警/故障 chip leaving faults out.
"""

from __future__ import annotations  # postponed annotations

import json  # node input / output
import os  # paths
import shutil  # find node
import subprocess  # run node

import pytest  # skip / parametrize

_ROOT = os.path.join(os.path.dirname(__file__), "..", "..", "..")  # repo root
_STATIC = os.path.join(_ROOT, "xbrain", "p5_gateway", "hmi", "static")  # HMI frontend
_BEGIN, _END = "// <event-text>", "// </event-text>"  # the pure block's markers


def _src(name: str) -> str:
    """One frontend file's text (the markup checks read it directly)."""
    with open(os.path.join(_STATIC, name), encoding="utf-8") as f:  # utf-8: Chinese text
        return f.read()  # whole file


def _node(expr: str):
    """Evaluate expr after the <event-text> block under node, return its JSON."""
    node = shutil.which("node")  # the robot has none
    if node is None:  # skip rather than pass: nothing was checked
        pytest.skip("node not installed (the robot has none)")  # visible skip
    src = _src("hmi.js")  # the real frontend, not a copy
    start = src.index(_BEGIN)  # block start
    block = src[start:src.index(_END, start)]  # only the pure functions
    out = subprocess.run([node, "-e", block + "\nprocess.stdout.write(JSON.stringify("  # print
                          + expr + "));\n"], capture_output=True, text=True,  # the result
                         timeout=30, check=True)  # a JS error fails the test
    return json.loads(out.stdout)  # back to Python


def _text(ev: dict):
    """eventText(ev) -> {cat, text}."""
    return _node("eventText(" + json.dumps(ev, ensure_ascii=False) + ")")  # one call


@pytest.mark.parametrize("ev,cat,text", [  # one row per wording branch worth pinning
    ({"cat": "charging", "detail": {"type": "low_battery", "soc_pct": 9,  # soc + dock name
                                    "target_name": "1号充电站"}},  # from p3's detail
     "充电", "电量低(9%), 正在返回1号充电站"),  # both details shown
    ({"cat": "charging", "detail": {"type": "low_battery_cleared", "cause": "battery_swap"}},  # end
     "充电", "已换电, 低电告警解除"),  # the cause is worded
    ({"cat": "rtk", "detail": {"type": "rtk_lost"}}, "定位", "定位丢失, 机器人已停车"),  # family start
    ({"cat": "comm", "detail": {"kind": "cloud_lost"}}, "通信", "云端连接丢失"),  # kind, not type
    ({"cat": "health", "detail": {"type": "health.item_changed", "device": "cam_rgbd",  # p2's new event
                                  "from": "ok", "to": "fail"}},  # a transition
     "健康", "深度相机 正常 -> 故障"),  # device and states in Chinese
    ({"cat": "task", "title": "task t-20261112-003 failed", "detail": {"kind": "failed"}},  # id in title
     "任务", "任务 t-20261112-003 已失败"),  # id taken from the title
    ({"cat": "payload", "title": "payload_siren device_offline",  # device in title
      "detail": {"type": "device_offline"}}, "载荷", "警笛 离线"),  # mapped device name
    ({"cat": "arbitration", "title": "arbitration grant on speaker"}, "仲裁", "扬声器 占用"),  # no detail
    ({"cat": "motion", "title": "estop"}, "运动", "急停"),  # bare title
    ({"cat": "motion", "title": "rotation_blocked", "detail": {"kind": "rotation_blocked"}},  # kind
     "运动", "原地旋转被拦(周围空间不足)"),  # via the title table
    ({"cat": "geo", "title": "geo w-2 geo.renamed", "detail": {"type": "geo.renamed"}},  # geo op
     "地图", "地图对象 w-2 已改名"),  # id from the title
    ({"cat": "newcat", "title": "something new happened", "detail": {"type": "x.y"}},  # unmapped
     "newcat", "something new happened"),  # shown as sent, never dropped
    ({"cat": "system"}, "系统", "(无标题)"),  # no title at all
])
def test_event_wording(ev, cat, text):
    """MUTATION: drop a case -> it falls through to the English title -> red;
    unmapped -> "" instead of the title -> red (recommendation 6)."""
    assert _text(ev) == {"cat": cat, "text": text}  # exact wording


def test_a_running_family_is_one_live_record_since_its_first_occurrence():
    """MUTATION: since = the newest entry's first_ts (not the episode's) -> red;
    an end not closing the family -> rtk still live -> red."""
    items = [  # two charging entries of one episode, a closed rtk pair, an open comm
        {"cat": "charging", "sev": "alarm", "ts": 130.0, "first_ts": 100.0,  # merged entry 1
         "detail": {"type": "low_battery"}},  # started at 100
        {"cat": "charging", "sev": "alarm", "ts": 170.0, "first_ts": 150.0,  # merged entry 2
         "detail": {"type": "low_battery_unreachable"}},  # same family
        {"cat": "rtk", "sev": "alarm", "ts": 50.0, "detail": {"type": "rtk_lost"}},  # start
        {"cat": "rtk", "sev": "info", "ts": 60.0, "detail": {"type": "heading_recovered"}},  # end
        {"cat": "comm", "sev": "alarm", "ts": 80.0, "detail": {"kind": "cloud_lost"}},  # still open
    ]
    got = _node("activeFamilies(" + json.dumps(items) + ").map(a => [a.fam, a.since, a.latest.ts])")  # no nowS
    assert got == [["charging", 100.0, 170.0], ["comm", 80.0, 80.0]]  # episode start 100


def test_cleared_ends_the_charging_family_and_a_new_alarm_starts_another():
    """MUTATION: since kept across the end -> the new episode claims 100 -> red."""
    items = [  # alarm, cleared, alarm again
        {"cat": "charging", "ts": 100.0, "detail": {"type": "low_battery"}},  # episode 1
        {"cat": "charging", "ts": 200.0, "detail": {"type": "low_battery_cleared"}},  # its end
        {"cat": "charging", "ts": 300.0, "detail": {"type": "low_battery"}},  # episode 2
    ]
    assert _node("activeFamilies(" + json.dumps(items) + ").map(a => a.since)") == [300.0]  # new start
    assert _node("activeFamilies(" + json.dumps(items[:2]) + ")") == []  # cleared -> not live


def test_a_low_battery_episode_that_stopped_repeating_is_not_live():
    """Found live 2026-11-12: a 12:36 episode stayed "持续中" after the 12:52
    reboot because no low_battery_cleared ever came. The alarm repeats every
    2 s, so silence past repeatS ends it; rtk / comm (one-shot) do not age.
    MUTATION: drop the repeatS filter -> red; apply it to rtk too -> red."""
    items = [  # one stale-able charging entry, one old one-shot rtk start
        {"cat": "charging", "ts": 1000.0, "first_ts": 900.0, "detail": {"type": "low_battery"}},  # last at 1000
        {"cat": "rtk", "ts": 500.0, "detail": {"type": "rtk_lost"}},  # never recovered
    ]
    fams = "activeFamilies(%s, %%s).map(a => a.fam)" % json.dumps(items)  # nowS filled below
    assert _node(fams % "1029") == ["charging", "rtk"]  # 29 s silent: still live
    assert _node(fams % "1031") == ["rtk"]  # 31 s silent: stopped; rtk does not age


def test_pinned_alarms_skip_live_families_old_ones_and_infos():
    """MUTATION: no family exclusion -> the low-battery alarm pinned twice -> red;
    no window -> yesterday's fault pinned forever -> red."""
    items = [  # live family alarm, recent fault, old fault, info
        {"cat": "charging", "sev": "alarm", "ts": 990.0, "detail": {"type": "low_battery"}},  # in 持续中
        {"cat": "motion", "sev": "fault", "ts": 995.0, "title": "rotation_clearance_unconfigured"},  # pin
        {"cat": "bit", "sev": "fault", "ts": 100.0, "title": "old"},  # outside 600 s
        {"cat": "task", "sev": "info", "ts": 999.0, "title": "x"},  # not an alarm
    ]
    expr = ("pinnedAlarms(%s, 1000, activeFamilies(%s, 1000), 600, 5).map(e => e.ts)"  # as drawn
            % (json.dumps(items), json.dumps(items)))  # same list both times
    assert _node(expr) == [995.0]  # only the recent fault


def test_the_alarm_chip_takes_alarms_and_faults():
    """MUTATION: alarm chip = sev "alarm" only -> faults hidden -> red."""
    got = _node('["fault","alarm","warn","info"].map(s => eventPasses({sev: s}, "alarm"))')  # chip 1
    assert got == [True, True, False, False]  # fault and alarm both
    assert _node('eventPasses({sev: "warn"}, "warn") && eventPasses({sev: "info"}, "all")')  # others


def test_the_drawer_markup_and_no_more_standing_dots():
    """MUTATION: drop a chip / the drawer section -> red; the old alarm-dot
    renderer back -> red (recommendation 8)."""
    html, js, css = _src("index.html"), _src("hmi.js"), _src("hmi.css")  # no node needed
    assert 'id="eventDrawer"' in html and 'id="eventBar"' in html  # drawer + bar
    for f in ("alarm", "warn", "info", "all"):  # the four chips
        assert 'data-filter="%s"' % f in html  # each present
    assert "alarm-dot" not in js and "alarm-dot" not in css  # standing dots gone
    assert ".teach-badge[hidden] { display: none; }" in css  # hidden must win over grid
    assert "wireEventDrawer();" in js  # wired at init
