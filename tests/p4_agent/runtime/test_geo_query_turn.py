"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_geo_query_turn.py
Brief: 99 U136 -- G50 nearest waypoint, G51 how far to X, G52 how long is route X

Description:
The three candidate queries the user picked on 2026-11-05. Geometry is built
in a local metric plane so every expected number reads in metres: a 200 m
route along x (y = 0), waypoints w-a at (150, 3) on it and w-b at (0, 40) off
it, the robot at (20, 8). Each case names the mutant it kills:
  * G51 answers the length of the path a goto WOULD take (99 U127 plan via
    xbrain.common.geo.goto_plan): to w-a 8 + 130 + 3 = 141 m along 1号路径,
    to w-b straight 45 m; with a move speed the time is added;
  * G50 is the nearest live waypoint by straight line;
  * G52 is the polyline length (a closed route includes its closing edge);
  * no objects / no pose / unknown names are answered plainly;
  * the rule layer routes the sentences to G50 / G51 / G52 before the goto
    rule would take "去X多远" as a goto to "X多远".
"""
from __future__ import annotations  # postponed annotations

import os  # this tree's configs

import pytest  # parametrize, marker
import yaml  # content files

from xbrain.common.geo.polyline import Plane  # metres <-> WGS84
from xbrain.p4_agent.registry.intents import load_intent_registry  # registry
from xbrain.p4_agent.runtime.orchestrator_turn import build_orchestrator, decision_to_publishes  # as wired
from xbrain.p4_agent.runtime.turn_orchestrator import OrchestratorSession  # a fresh dialog
from xbrain.p4_agent.session.chitchat import ChitchatResponder  # presets
from xbrain.p4_agent.slots.ptp_slots import parse_ptp  # the rule layer
from xbrain.p4_agent.state.geo_queries import make_geo_query_fn  # under test

pytestmark = pytest.mark.no_device  # pure logic

_P = Plane((31.0, 121.0))  # local plane, metres
_CONFIGS = os.path.join(os.path.dirname(__file__), "..", "..", "..", "configs")  # this tree


def _ll(x, y):  # metres -> {lat, lon}
    lat, lon = _P.ll((x, y))  # WGS84
    return {"lat": lat, "lon": lon}  # as the broadcast carries points


def _objects(closed=False):  # the state/geo/objects body
    return {"routes": [{"geo_id": "r-1", "name": "1号路径", "state": "active",
                        "loop_mode": "closed" if closed else "oneway",
                        "points": [_ll(0, 0), _ll(200, 0)] + ([_ll(200, 50)] if closed else [])}],
            "waypoints": [{"geo_id": "w-a", "name": "1号测试点", "state": "active", **_ll(150, 3)},
                          {"geo_id": "w-b", "name": "东门岗亭", "state": "active", **_ll(0, 40)},
                          {"geo_id": "w-x", "name": "旧点", "state": "deleted", **_ll(20, 9)}]}


_POSE = _ll(20, 8)  # the robot
_MODE = {"move_speed": {"mps": 0.5}}  # voice-set move speed
_Q = make_geo_query_fn(5.0)  # common.geo.route_membership_m


# -- the answers ----------------------------------------------------------------
def test_distance_follows_the_route_the_robot_would_take():  # G51 on a route
    """MUTATION: answer the straight line (130 m) -> red."""
    key, f = _Q("distance_to", "一号测试点", _objects(), _POSE, None)  # canonical name
    assert key == "distance_route" and f["dist"] == 141 and f["route"] == "1号路径"  # 8 + 130 + 3
    assert f["time"] == ""  # no speed known, no time


def test_distance_off_route_is_straight_and_timed():  # G51 straight
    """MUTATION: drop the speed tail -> no time -> red."""
    key, f = _Q("distance_to", "东门岗亭", _objects(), _POSE, _MODE)  # off any route
    assert key == "distance_straight" and f["dist"] == 38  # (20,8) -> (0,40): hypot(20, 32) = 37.7
    assert f["time"] == ", 按当前速度约2分钟"  # 37.7 m / 0.5 m/s = 75 s -> 2 min


def test_nearest_waypoint_skips_deleted_ones():  # G50
    """MUTATION: include deleted points -> 旧点 (1 m) -> red."""
    key, f = _Q("nearest_point", "", _objects(), _POSE, None)  # nearest
    assert key == "nearest_point" and f["name"] == "东门岗亭" and f["dist"] == 38  # 37.7 m vs 130.2 m


@pytest.mark.parametrize("closed,length", [(False, 200), (True, 456)])
def test_route_length(closed, length):  # G52
    """MUTATION: ignore the closing edge -> 250 for the ring -> red."""
    key, f = _Q("route_length", "1号路径", _objects(closed), None, None)  # no pose needed
    assert key == "route_length" and f["length"] == length  # ring: 200 + 50 + hypot(200, 50)


@pytest.mark.parametrize("kind,name,objects,pose,want", [
    ("distance_to", "1号测试点", None, _POSE, "geo_unloaded"),  # no broadcast
    ("distance_to", "1号测试点", "objs", None, "no_pose"),  # no position
    ("nearest_point", "", "objs", None, "no_pose"),
    ("distance_to", "5号点", "objs", _POSE, "not_found"),  # unknown
    ("route_length", "9号路径", "objs", None, "route_not_found"),
])
def test_plain_refusals(kind, name, objects, pose, want):  # honest answers
    """MUTATION: measure from the last known pose -> no no_pose -> red."""
    obj = _objects() if objects == "objs" else objects  # the body or None
    assert _Q(kind, name, obj, pose, None)[0] == want  # said plainly


# -- the rule layer ---------------------------------------------------------------
@pytest.mark.parametrize("text,kind,name", [
    ("离我最近的航点是哪个", "nearest_point", ""), ("最近的点是哪个", "nearest_point", ""),
    ("去1号测试点多远", "distance_to", "1号测试点"), ("到东门岗亭有多远", "distance_to", "东门岗亭"),
    ("离1号测试点还有多远", "distance_to", "1号测试点"), ("去东门岗亭要多久", "distance_to", "东门岗亭"),
    ("1号路径多长", "route_length", "1号路径"), ("东环线有多长", "route_length", "东环线"),
])
def test_rule_layer(text, kind, name):  # parse before the goto rule
    """MUTATION: try the goto rule first -> 去1号测试点多远 is a goto -> red."""
    got = parse_ptp(text)  # one parse
    assert got is not None and (got.kind, got.name) == (kind, name)  # a query


# -- through the orchestrator ------------------------------------------------------
def _turn(text):  # one mic turn with the three state bodies
    with open(os.path.join(_CONFIGS, "intents.yaml"), encoding="utf-8") as fh:  # registry
        reg = load_intent_registry(yaml.safe_load(fh))  # this tree's registry
    with open(os.path.join(_CONFIGS, "chitchat.yaml"), encoding="utf-8") as fh:  # presets
        cc = ChitchatResponder(yaml.safe_load(fh))  # presets
    state = {"state/geo/objects": _objects(), "state/pose": _POSE, "state/mode": _MODE,
             "state/geo/manifest": {"items": []}}  # what P4 caches
    orch = build_orchestrator(reg, cc, l2_timeout_ms=8000, tier2_fn=None,
                              state_fn=lambda: state, geo_query_fn=_Q)  # as wired
    d = orch.handle_turn(text, OrchestratorSession(), 1000, channel="mic_local")  # a mic turn
    return d, [k for k, _ in decision_to_publishes(d) if k != "cmd/audio/speak"]  # minus speech


@pytest.mark.parametrize("text,iid,said", [
    ("去1号测试点多远", "G51", "去1号测试点约141米, 沿1号路径走, 按当前速度约5分钟"),
    ("去东门岗亭多远", "G51", "去东门岗亭直线约38米, 按当前速度约2分钟"),
    ("离我最近的航点是哪个", "G50", "离你最近的航点是东门岗亭, 直线约38米"),
    ("1号路径多长", "G52", "1号路径全长约200米"),
])
def test_the_answers_are_spoken_and_nothing_is_sent(text, iid, said):  # end to end
    """MUTATION: route G51 as a goto -> a cmd/task frame -> red."""
    d, frames = _turn(text)  # one turn
    assert d.intent_id == iid and d.tts_text == said and not frames  # answered only
