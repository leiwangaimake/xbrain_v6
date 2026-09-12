"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_sil_reset.py
Brief: sil_server startup loads the FULL frozen map; reset empties the map (user ruling 2026-09-12)

Description:
Two user-visible facts about the browser SIL that must not drift again:
opening the page shows the complete default scene (every obstacle and the
patrol path of scripts/sil/field_map2.json), and "重置场景" leaves an EMPTY map
(no obstacles, no path, no waypoints, robot at the start, no mission) so
obstacles can be placed by hand on a clean sheet. Driven through the FastAPI
app with TestClient (no lifespan: the 20 Hz tick task is not started).
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts" / "sil"))

pytestmark = pytest.mark.no_device


@pytest.fixture(scope="module")
def srv():
    pytest.importorskip("fastapi")
    from fastapi.testclient import TestClient
    import sil_server
    return sil_server, TestClient(sil_server.app)


def test_startup_loads_the_complete_default_map(srv):
    """mutant: skip load_field_map at import -> obstacles empty -> red."""
    sil_server, _c = srv
    m = json.loads((ROOT / "scripts" / "sil" / "field_map2.json").read_text(encoding="utf-8"))
    expected = sum(1 for o in m["obstacles"] if abs(o["x"]) <= 100 and abs(o["y"]) <= 100)
    assert len(sil_server.world.obstacles) == expected and expected > 0
    assert [tuple(p) for p in m["path"]] == list(sil_server.world.path)


def test_reset_empties_the_map(srv):
    """mutant: keep the path across reset -> red."""
    sil_server, c = srv
    sil_server.world.add_obstacle("rock", 1.0, 1.0)
    sil_server.world.waypoints.append((2.0, 2.0))
    sil_server.world.rx, sil_server.world.ry = 5.0, 5.0
    r = c.post("/api/reset")
    assert r.status_code == 200 and r.json() == {"ok": True}
    w = sil_server.world
    assert w.obstacles == {} and w.path == [] and w.waypoints == []
    assert (w.rx, w.ry) == (0.0, 0.0)
    assert sil_server.nav["state"] == "idle" and sil_server.rns._mission is None
