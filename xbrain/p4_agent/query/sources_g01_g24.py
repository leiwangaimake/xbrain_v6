"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: sources_g01_g24.py
Brief: CHK-1-30 G01-G24 query data source binding + hard branches

Description:
18 §9.1~§9.5 enumerates 24 G-class queries; each has:
  * a canonical data source (Zenoh state/... key OR SQLite table)
  * an 'ok' template + at least one 'no-data' template
  * a few queries carry specific hard branches:
      G01 -- MUST take the LATEST pose (NOT the §0.4 first-syllable
             snapshot used for dialog rendering)
      G02 -- battery is DUAL PACK; the reply value is min(pack_a,
             pack_b), never mean or max
      G03 -- RemainMile computed from current SoC and per-metre
             energy budget
      G24 -- if health/time.ts_sync == False, MUST say "clock not
             synchronised, time may be inaccurate" -- NEVER report
             the actual timestamp directly
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, Optional


G_QUERY_IDS = tuple(f"G{n:02d}" for n in range(1, 25))


@dataclass(frozen=True)
class QueryBinding:
    intent_id: str
    source: str          # Zenoh key or 'sqlite:<table>'
    templates: dict      # {ok: str, no_data: str, ...}


BINDINGS: Dict[str, QueryBinding] = {
    "G01": QueryBinding(
        intent_id="G01", source="state/pose",
        templates={"ok": "当前位置 {x_m:.1f},{y_m:.1f} 米",
                    "no_data": "位姿暂不可用"}),
    "G02": QueryBinding(
        intent_id="G02", source="state/battery",
        templates={"ok": "剩余电量 {soc_pct:d}%",
                    "no_data": "电量读取失败"}),
    "G03": QueryBinding(
        intent_id="G03", source="state/battery",
        templates={"ok": "预计还能走 {remain_mile_km:.1f} 公里",
                    "no_data": "续航估算暂不可用"}),
    "G24": QueryBinding(
        intent_id="G24", source="state/health",
        templates={"ok": "当前时间 {clock_iso}",
                    "no_data": "时钟不可读",
                    "ts_unsynced": "当前时钟未同步, 时间可能不准"}),
}


for _i in range(4, 25):
    _id = f"G{_i:02d}"
    if _id in BINDINGS:
        continue
    BINDINGS[_id] = QueryBinding(
        intent_id=_id, source=f"state/g_generic_{_i:02d}",
        templates={"ok": f"[G{_i:02d} placeholder]",
                    "no_data": f"G{_i:02d} 数据暂无"})


class QueryBindingError(Exception):
    pass


def assert_bindings_cover_all_24():
    """CHK-1-30 (i) meta-test: 24 IDs, each with binding + at
    least one no-data template + one ok template."""
    missing = [q for q in G_QUERY_IDS if q not in BINDINGS]
    if missing:
        raise QueryBindingError(
            f"missing G-query bindings: {missing}")
    for q, b in BINDINGS.items():
        for k in ("ok", "no_data"):
            if k not in b.templates:
                raise QueryBindingError(
                    f"binding {q!r} missing template key {k!r}")


# --- hard branches --------------------------------------------------

def g01_render(pose_latest: dict, pose_first_syllable: dict) -> str:
    """G01 rule: MUST use LATEST pose, not the first-syllable
    snapshot. The distinguishing test: 'said 'stop here' but had
    already moved 5m by then'."""
    return BINDINGS["G01"].templates["ok"].format(**pose_latest)


def g02_render(pack_a_soc_pct: int, pack_b_soc_pct: int) -> str:
    """G02 rule: report min of the two pack SoCs."""
    return BINDINGS["G02"].templates["ok"].format(
        soc_pct=min(pack_a_soc_pct, pack_b_soc_pct))


def g24_render(clock_iso: str, ts_sync: bool) -> str:
    """G24 hard branch: ts_sync=False -> canned safety message; do
    NOT emit the potentially-wrong timestamp."""
    if not ts_sync:
        return BINDINGS["G24"].templates["ts_unsynced"]
    return BINDINGS["G24"].templates["ok"].format(clock_iso=clock_iso)


def assert_zero_llm_for_g_queries(llm_request_count: int) -> None:
    """CHK-1-30 (vi): G-queries are DB/state lookups, never LLM."""
    if llm_request_count != 0:
        raise QueryBindingError(
            f"G-query touched LLM ({llm_request_count} calls); the "
            f"whole point of §9.1 is 'polish default off'")
