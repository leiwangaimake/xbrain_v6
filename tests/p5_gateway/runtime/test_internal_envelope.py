"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_internal_envelope.py
Brief: p5 的机内 state/link 与 event/{sev}/comm 必须带 11 S3.0 信封

Description:
S3.0 逐字"所有 Zenoh JSON 载荷共用此外层结构". 这两条 key 在 2026-09-27 之前
发的是裸 dict, 与 estop ping 那条是同一个缺陷 -- 而 state/link 是[云端判在线]
的那条 key, 一个按 S3.0 解码的消费方在必填字段那一步就退出了.

本文件分两半, 各自能在对方全绿时坏掉:
  * stamp_internal 的[逐字段]判据 -- 八个键齐全 / ts 与 mono 是秒 float(不是
    毫秒整数, p1 的 stamp_envelope 正是这么错过一次) / 无 boot 时 mono 与 boot
    一并省略(CLK-C4) / ts_sync 照传不自行判定.
  * 接线上真的用了它 -- 纯函数写对而调用处还在 json.dumps(裸 dict) 是一个
    "两半都对不上"的典型形状, 所以读 p5 真实源码.
两条消费侧回归也在这里: p3 的 _on_link 与 p2 的 state sink 必须能从 data 里取,
否则 F-5 的 return_home 判据会在没有任何报错的情况下停止看见云端链路.
"""

import inspect
import json
import time

import pytest

from xbrain.common.envelope import decode
from xbrain.p5_gateway.runtime.main_wiring import (
    run_voice_loop_wiring, stamp_internal,
)


pytestmark = pytest.mark.no_device


def _decode(raw: bytes) -> dict:
    return json.loads(raw.decode("utf-8"))


# -- the stamper --------------------------------------------------------------


def test_the_stamp_carries_all_eight_envelope_fields():
    env = _decode(stamp_internal({"gateway_up": True}, rid="m20s",
                                 boot="abc12345", seq=3, ts_sync=True))
    assert set(env) == {"v", "rid", "ts", "mono", "boot", "seq", "src",
                        "ts_sync", "data"}
    assert env["v"] == 1 and env["rid"] == "m20s" and env["seq"] == 3
    assert env["src"] == "p5_gateway" and env["ts_sync"] is True
    assert env["data"] == {"gateway_up": True}
    # It must also survive the shared decoder -- the whole point is that a
    # consumer reading per S3.0 stops rejecting this key.
    assert decode(env).data["gateway_up"] is True


def test_ts_and_mono_are_seconds_not_milliseconds():
    # p1's stamp_envelope wrote millisecond integers here until 2026-09-13 and a
    # consumer ageing per S3.0 read a 5 s old message as 5000 s old. MUTATION:
    # multiply either by 1000 -> red.
    env = _decode(stamp_internal({}, rid="m20s", boot="abc12345", seq=1,
                                 ts_sync=False))
    assert isinstance(env["ts"], float) and abs(env["ts"] - time.time()) < 1.0
    assert isinstance(env["mono"], float)
    assert abs(env["mono"] - time.monotonic()) < 1.0


def test_without_a_boot_id_mono_is_omitted_with_it():
    # CLK-C4: boot is the validity domain of a mono reading, so the pair travels
    # together. A bare mono would invite the peer to interpret our reading in ITS
    # boot domain. MUTATION: emit mono unconditionally -> red.
    env = _decode(stamp_internal({}, rid="m20s", boot="", seq=1, ts_sync=False))
    assert "mono" not in env and "boot" not in env


def test_ts_sync_is_copied_never_judged():
    # CLK-A2: rtk_driver is the sole authority; p5 copies what P1-13 mirrored.
    # MUTATION: hardcode True -> red (an unsynced peer would present as synced
    # and the clock hard cap that should drop the robot to 0.5 m/s never fires).
    assert _decode(stamp_internal({}, rid="m20s", boot="b", seq=1,
                                  ts_sync=False))["ts_sync"] is False


def test_the_seq_is_supplied_not_invented_here():
    # 11 S3.0: seq is per-key and restart-relative, so the counter belongs to the
    # caller (one per key). A stamper with its own global counter would make every
    # key look like it is constantly skipping numbers, which is exactly what U18's
    # backfill cursor reads as loss.
    seqs = [_decode(stamp_internal({}, rid="m20s", boot="b", seq=n,
                                   ts_sync=False))["seq"] for n in (7, 8)]
    assert seqs == [7, 8]


# -- the wiring actually uses it ----------------------------------------------


def test_state_link_is_published_through_the_stamper():
    # MUTATION: restore link_pub.put(json.dumps(link_payload)...) -> red.
    src = inspect.getsource(run_voice_loop_wiring)
    assert "link_pub.put(_stamp(link_payload, _link_env_seq))" in src
    assert "link_pub.put(json.dumps(" not in src


def test_the_comm_event_is_published_through_the_stamper():
    # MUTATION: restore gen.put(key, json.dumps({...})) for the comm event -> red.
    src = inspect.getsource(run_voice_loop_wiring)
    body = src[src.index("_ckind, _csev, _cdetail = _ce"):]
    body = body[:body.index("_logger.info(\"p5 comm event")]
    assert "gen.put(_ckey, _stamp(" in body
    # The inner ts was hardcoded 0.0, which p2 had already been bitten by (an
    # estop event displayed as 1970 in the local HMI ring).
    assert '"ts": 0.0' not in body


def test_the_comm_seq_is_per_key_not_shared():
    # The {sev} segment varies, so one counter across warn/alarm/info would make
    # each key look like it is skipping. MUTATION: replace the setdefault bucket
    # with a single shared slot -> red.
    src = inspect.getsource(run_voice_loop_wiring)
    assert "_comm_env_seq.setdefault(_ckey, [0])" in src


# -- the consumers ------------------------------------------------------------


def test_p3_reads_the_link_body_out_of_data():
    # This is the one that would break silently: every field p3 takes off
    # state/link would come back None and F-5's return_home judgement would stop
    # seeing the cloud link, with no error and no log.
    # MUTATION: delete the unwrap in p3's _on_link -> red.
    from xbrain.p3_task.runtime import main_wiring as p3_wiring

    src = inspect.getsource(p3_wiring)
    body = src[src.index("def _on_link(sample)"):]
    body = body[:body.index("link_holder.update")]
    assert 'isinstance(p.get("data"), dict)' in body
    assert 'p = p["data"]' in body


def test_p2_state_sink_already_unwraps_data():
    # p2 has read both forms since it was written; asserted here so the pair of
    # consumers is checked in ONE place -- a later edit that "simplifies" p2 back
    # to the top level would otherwise only surface on the robot.
    from xbrain.p2_core.runtime import main_wiring as p2_wiring

    src = inspect.getsource(p2_wiring)
    body = src[src.index("def _make_state_sink(name: str)"):]
    body = body[:body.index("return _sink")]
    assert 'body.get("data")' in body
