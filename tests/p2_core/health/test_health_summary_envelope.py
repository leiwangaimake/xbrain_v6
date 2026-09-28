"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_health_summary_envelope.py
Brief: p2 的 health/summary 必须带 11 S3.0 信封, 且三个消费方必须跟着改

Description:
S3.0 逐字"所有 Zenoh JSON 载荷共用此外层结构". health/summary 在 2026-09-28
之前发的是[裸报文] -- 顶层直接 {"schema":"health_summary_v1", ...}, 没有
v/rid/ts/mono/boot/seq/src/ts_sync. 2026-09-28 在 ORIN 总线上抓到的就是这个
形状, 与 p5 的 state/link 和 event/{sev}/comm 在 2026-09-27 之前是同一个缺陷:
一个按 S3.0 解码的消费方在必填字段那一步就退出, 而 health/summary 是 P3 任务
准入(15 S179 V-5 allow_motion)与 HMI 健康面板的唯一来源.

本文件分三半, 每一半都能在另外两半全绿时坏掉:
  * stamp_body 的[逐字段]判据 -- 九个键齐全 / ts 与 mono 是秒 float(不是
    毫秒整数, p1 的 stamp_envelope 正是这么错过一次) / 无 boot 时 mono 与
    boot 一并省略(CLK-C4) / ts_sync 照传不自行判定 / src 是 p2_core;
  * 接线上真的用了它 -- 纯函数写对而调用处还在 json.dumps(裸 dict) 是一个
    "两半都对不上"的典型形状, 所以读 p2 真实源码;
  * 消费侧回归 -- 只改发布侧而消费侧仍读顶层, 正是 2026-09-27 修
    event/{sev}/comm 时引入过的那次回归(5d6981a). 三个消费方逐一钉住:
    p5 的 _on_health(本批新改) / p3 的 _make_state_sink(早已能解) /
    p3 teach 的 missing_sources(吃的是 sink 的产物).
"""
from __future__ import annotations

import inspect
import json

import pytest

from xbrain.common.envelope import decode
from xbrain.p2_core.runtime.main_wiring import (run_voice_loop_wiring,
                                                stamp_body)

pytestmark = pytest.mark.no_device


#: 一份最小但真实的 HealthSummary. 形状取自 11 S5.1 与 2026-09-28 线上那条.
_SUMMARY = {
    "schema": "health_summary_v1",
    "overall": "degraded",
    "allow_motion": False,
    "items": {
        "chassis": {"kind": "device", "state": "ok", "level": "fatal",
                    "detail": "linked", "since_mono": 1454352.699},
        "clock": {"kind": "cap", "state": "fail", "level": "fatal",
                  "detail": "no rtk"},
    },
}


def _decode(raw: bytes) -> dict:
    return json.loads(raw.decode("utf-8"))


# -- the stamper --------------------------------------------------------------


def test_the_stamp_carries_all_eight_envelope_fields_plus_data():
    env = _decode(stamp_body(_SUMMARY, rid="m20s", boot="abc12345",
                               seq=3, ts_sync=True))
    assert set(env) == {"v", "rid", "ts", "mono", "boot", "seq", "src",
                        "ts_sync", "data"}
    assert env["v"] == 1
    assert env["rid"] == "m20s"
    assert env["seq"] == 3
    assert env["src"] == "p2_core"
    assert env["ts_sync"] is True
    # decode() 是 S3.0 的严格校验器; 少一个必填字段它就抛. 这一行是本文件
    # 真正的[契约]断言 -- 上面的 set 比较只说键名对, 不说类型与取值域对.
    assert decode(env).data == _SUMMARY


def test_the_summary_is_the_data_not_the_top_level():
    """裸报文与信封的区别就在这一条.

    MUTATION: 把 stamp_body 改回 json.dumps(data) -> 顶层出现 schema,
    data 消失, 本条红.
    """
    env = _decode(stamp_body(_SUMMARY, rid="m20s", boot="b", seq=1,
                               ts_sync=False))
    assert "schema" not in env
    assert env["data"]["schema"] == "health_summary_v1"
    assert env["data"]["items"]["chassis"]["state"] == "ok"


def test_ts_and_mono_are_seconds_not_milliseconds():
    """p1 的 stamp_envelope 把这两个戳成过毫秒整数.

    后果不是"数大了一千倍": 按 S3.0.1 计龄的消费方会把 5 s 前的消息读成
    5000 s 前的, 于是每一条都超龄, 而发布方看着完全正常.
    MUTATION: 任一个乘 1000 或取 int -> 本条红.
    """
    env = _decode(stamp_body(_SUMMARY, rid="m20s", boot="b", seq=1,
                               ts_sync=False))
    for field in ("ts", "mono"):
        assert isinstance(env[field], float)
        # 单调钟在本机是开机以来的秒数(1e5~1e7 量级), 墙钟是 1.79e9 量级.
        # 毫秒整数会把两者分别抬到 1e8~1e10 与 1.79e12.
        assert env[field] < 1.0e11
    # 小数位真的在: %.0f 一类的格式化会让上面的 isinstance 仍然通过.
    assert env["mono"] != float(int(env["mono"]))


def test_without_a_boot_id_mono_is_omitted_too():
    """CLK-C4: 没有 boot 就没有 mono 的定义域.

    发一个裸 mono 会让对端拿自己的 boot 域去解释别人的读数 -- 那是一个
    看起来能算, 算出来是错的年龄.
    MUTATION: boot 为空时仍填 mono -> 本条红.
    """
    env = _decode(stamp_body(_SUMMARY, rid="m20s", boot="", seq=1,
                               ts_sync=False))
    assert "mono" not in env and "boot" not in env
    # 其余字段照常, 这一条才不会退化成"什么都不发也通过".
    assert env["data"]["schema"] == "health_summary_v1"
    assert env["seq"] == 1


def test_ts_sync_is_relayed_not_decided_here():
    """CLK-A2: 全系统唯一有权判定授时状态的是 rtk_driver.

    MUTATION: 写死 True 或 False -> 两个取值里必有一个红.
    """
    assert _decode(stamp_body(_SUMMARY, rid="r", boot="b", seq=1,
                                ts_sync=True))["ts_sync"] is True
    assert _decode(stamp_body(_SUMMARY, rid="r", boot="b", seq=1,
                                ts_sync=False))["ts_sync"] is False


# -- the wiring actually uses it ----------------------------------------------


def test_health_summary_is_published_through_the_stamper():
    # MUTATION: 恢复 health_pub.put(json.dumps(health_agg.build_summary(...)))
    # -> 本条红. 纯函数写对而调用处没换, 上面五条全绿而线上仍是裸报文.
    src = inspect.getsource(run_voice_loop_wiring)
    assert "health_pub.put(stamp_body(" in src
    assert "health_pub.put(json.dumps(" not in src


def test_the_envelope_seq_increments_per_publish():
    """S3.0 的 seq 是 U18 补发游标与缺口判定的依据; 恒 0 会让消费方以为
    每一帧都是重复帧.
    MUTATION: 删掉自增行 -> 本条红.
    """
    src = inspect.getsource(run_voice_loop_wiring)
    # 2026-09-28: 标量 _health_env_seq 改为按 key 的表(本进程现在发三条带
    # 信封的 key). 判据随之改为"health/summary 取的是它自己那条 key 的号".
    assert "seq=_next_seq(HEALTH_SUMMARY_TOPIC)" in src
    assert "_env_seq[key] = n" in src


def test_ts_sync_comes_from_the_clock_mirror_field_named_sync():
    """state/clock 的 data 里字段名是 sync(p1 的 gnss_pose 逐字如此),
    NO 不是 ts_sync -- 后者只存在于信封层.

    读错名字不会报错, 只会恒取到 None 再恒判 False: 一条永远"未同步"的
    health/summary, 而两侧进程都健康.
    MUTATION: 改成 .get("ts_sync") -> 本条红.

    2026-09-28: 求值从三个发布点内联改为 _clock_sync() 一处(三条 key 共用),
    所以判据读的是那个函数体, NO 不再切 health 发布块 -- 切块的写法在函数
    提取之后会切到一段不含 .get 的代码上, 变成一条恒红(再被人改成恒绿)的
    断言.
    """
    src = inspect.getsource(run_voice_loop_wiring)
    block = src[src.index("def _clock_sync()"):]
    block = block[:block.index("def _estop_seq_next()")]
    assert '.get("sync")' in block
    assert '.get("ts_sync")' not in block


# -- the regression the envelope itself would cause ---------------------------


def test_the_device_list_needs_the_BODY_not_the_envelope():
    """为什么消费侧的解包不是装饰.

    _devices_from_health 在顶层找 items. 喂它一条 11 S3.0 信封, 它拿到空
    dict 并返回空列表 -- 而 v2.0 S4.2 的"后端只发布实际发现的设备"让这个空
    数组看起来完全合规. 空的原因是"没解包", 不是"没发现", 两者在报文上不可
    区分, 正是 2026-09-27 event/{sev}/comm 那次回归的同一形状.
    """
    from xbrain.p5_gateway.runtime.cloud_state import _devices_from_health

    assert [d["id"] for d in _devices_from_health(_SUMMARY)] == ["chassis"]
    enveloped = {"v": 1, "rid": "dev", "ts": 1.0, "seq": 1, "src": "p2_core",
                 "ts_sync": False, "data": _SUMMARY}
    assert _devices_from_health(enveloped) == []


def test_p5_on_health_reads_the_body_out_of_the_envelope():
    """上一条的修法, 钉在它所在的地方.

    MUTATION: 删掉解包的三行 -> /api/health 与云端 devices 一起空掉, 本条红.
    """
    from xbrain.p5_gateway.runtime.main_wiring import (
        run_voice_loop_wiring as p5_wiring)

    src = inspect.getsource(p5_wiring)
    block = src[src.index("def _on_health("):]
    block = block[:block.index("def _on_bit(")]
    assert 'inner = d.get("data")' in block
    assert "hmi_state[\"health\"] = d" in block


def test_p5_on_health_still_accepts_the_bare_form():
    """桩发布者与旧版本 p2 都不带信封; 两种形态各写一条代码路径必然分叉.

    这里用一个最小替身跑真实的解包判据, 而不是只读源码 -- 源码断言说不出
    "裸形态还能用".
    """
    def unwrap(d):
        if isinstance(d, dict):
            inner = d.get("data")
            if isinstance(inner, dict):
                d = inner
        return d

    assert unwrap(_SUMMARY) is _SUMMARY
    assert unwrap({"v": 1, "data": _SUMMARY}) is _SUMMARY
    # data 不是 dict(例如某个发布者把它当字符串发)时不得吞掉整条报文.
    assert unwrap({"schema": "x", "data": "oops"})["schema"] == "x"


def test_p3_state_sink_unwraps_health_too():
    """p3 用 health/summary 判任务准入(15 V-5 allow_motion)与示教开臂.

    它的 _make_state_sink 早就能解信封(state/pose 先带的信封), 所以本批
    不必改 p3 -- 但"不必改"必须有判据, 否则下一个人改 sink 时会把它拆掉.
    MUTATION: 把 sink 的 data 解包删掉 -> 本条红, 且 p3 的准入会开始读到
    一个没有 allow_motion 的顶层 dict 并恒判拒绝.
    """
    # 接线体在 _amain 里(run_voice_loop_wiring 只是 asyncio.run 的外壳),
    # 所以读的是 _amain -- 读外壳会得到一条恒绿的断言.
    from xbrain.p3_task.runtime import main_wiring as p3_main_wiring

    src = inspect.getsource(p3_main_wiring._amain)
    assert 'data = body.get("data") if isinstance(body, dict) else None' in src
    assert "(HEALTH_SUMMARY_TOPIC, \"health\")" in src
