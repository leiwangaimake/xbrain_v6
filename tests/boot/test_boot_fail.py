"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_boot_fail.py
Brief: CFG-BT-5 / BOOT-I4 -- 启动失败落盘与下次开机补发

Description:
BOOT-I4: "任何启动失败都必须在[下一次成功启动后]被看见".

判据点名的变异体是 Stage 0z-2 失败(通用面根本不存在) -- 那种情况下上行
通道本身就是失败的那一项, 失败信息如果只写日志就永远出不去.

本文件逐条验四件事, 每件对应一种真实的丢失方式:
  1. 追加而不是覆盖 -- 覆盖会让连续两次失败只剩最后一次, 而最早那次
     往往是根因, 后面都是它的连锁反应;
  2. tmpfs 写失败不得阻止持久盘写 -- /run 可能就是不可写的那一项,
     而那时更需要留下记录;
  3. jsonl 末行截断(掉电)时前面的条目仍要能读出来 -- 一个"解析失败就
     返回空"的实现会因为一行残缺丢掉整份历史;
  4. detail 四项必填且不许填默认值 -- 一条 stage="unknown" 的记录,
     与没有这条记录相比只多了噪声.

Boundaries: 不测真正的上行(那要 p5 与云端), 只测落盘/读取/转事件.
"""
from __future__ import annotations

import json
import os
import pathlib

import pytest

pytestmark = pytest.mark.no_device


def _rec(boot_id="b-1", stage="0z-2", code="E_CONFIG_INVALID",
         message="gen plane router absent"):
    return {"stage": stage, "code": code, "boot_id": boot_id,
            "message": message}


def test_all_four_detail_fields_are_required():
    """*** 四项必填, 且 NO 不给缺失项填默认值.

    一条 stage 为 "unknown" 的失败记录, 与没有这条记录相比只多了噪声 --
    运维还是不知道卡在哪一阶段, 而现在还多了一条"看起来有信息"的事件.
    """
    from xbrain.boot.boot_fail import BootFailRecordError, validate

    validate(_rec())
    for missing in ("stage", "code", "boot_id", "message"):
        bad = _rec()
        del bad[missing]
        with pytest.raises(BootFailRecordError) as exc:
            validate(bad)
        assert missing in str(exc.value), "报错没说清缺的是哪一项"


def test_persist_file_appends_never_overwrites(tmp_path):
    """*** 判据逐字: 追加不覆盖.

    连续两次启动失败时, 覆盖写只剩最后一次 -- 而最早那次往往是根因,
    后面的都是它的连锁反应. 丢掉根因等于把排查从头做起.

    MUTATION: 把 open(path, "a") 改成 "w" -> 这里红.
    """
    from xbrain.boot.boot_fail import PERSIST_NAME, write

    d = str(tmp_path)
    write(_rec(boot_id="b-1"), runtime_path=str(tmp_path / "rt.json"),
          persist_dir=d)
    write(_rec(boot_id="b-2", message="second failure"),
          runtime_path=str(tmp_path / "rt.json"), persist_dir=d)
    lines = (tmp_path / PERSIST_NAME).read_text(encoding="utf-8").strip().split("\n")
    assert len(lines) == 2, "第二次写覆盖了第一次: %s" % lines
    ids = [json.loads(x)["boot_id"] for x in lines]
    assert ids == ["b-1", "b-2"], "顺序或内容不对: %s" % ids


def test_a_failing_tmpfs_does_not_block_the_persistent_write(tmp_path):
    """*** /run 不可写时, 持久盘那一份必须照写.

    /run 在某些故障场景下本身就是坏的那一项(tmpfs 满 / 挂载失败). 一个
    "先写 tmpfs 失败就整个放弃"的实现, 恰好在最糟的时候什么都不留.
    """
    from xbrain.boot.boot_fail import PERSIST_NAME, write

    # 用一个不可能创建的路径当 runtime_path.
    bad_runtime = "/proc/xbrain_cannot_write_here/boot_fail.json"
    written = write(_rec(), runtime_path=bad_runtime, persist_dir=str(tmp_path))
    assert (tmp_path / PERSIST_NAME).is_file(), (
        "tmpfs 写失败连带放弃了持久盘写")
    assert bad_runtime not in written


def test_read_pending_skips_a_truncated_last_line(tmp_path):
    """*** 掉电截断的末行不得让整份历史作废.

    jsonl 的最后一行可能因为掉电而只写了一半. 一个"解析失败就返回空"的
    实现会丢掉前面所有完好的条目 -- 而那些正是要补发的.
    """
    from xbrain.boot.boot_fail import PERSIST_NAME, read_pending

    path = tmp_path / PERSIST_NAME
    path.write_text(
        json.dumps(_rec(boot_id="b-1"), ensure_ascii=False) + "\n"
        + json.dumps(_rec(boot_id="b-2"), ensure_ascii=False) + "\n"
        + '{"stage": "0z-2", "code": "E_CO',      # 截断
        encoding="utf-8")
    pending = read_pending(str(tmp_path))
    assert [p["boot_id"] for p in pending] == ["b-1", "b-2"], (
        "截断的末行让前面的条目也丢了: %s" % pending)


def test_uplinked_entries_are_not_returned_again(tmp_path):
    """已上行的不再补发 -- 否则每次开机都会重发全部历史."""
    from xbrain.boot.boot_fail import mark_uplinked, read_pending, write

    d = str(tmp_path)
    write(_rec(boot_id="b-1"), runtime_path=str(tmp_path / "rt.json"),
          persist_dir=d)
    write(_rec(boot_id="b-2"), runtime_path=str(tmp_path / "rt.json"),
          persist_dir=d)
    assert len(read_pending(d)) == 2
    changed = mark_uplinked(d, ["b-1"])
    assert changed == 1
    rest = read_pending(d)
    assert [r["boot_id"] for r in rest] == ["b-2"]


def test_mark_uplinked_keeps_the_other_rows(tmp_path):
    """*** 标记时整份重写, 不得丢掉没被标记的条目.

    jsonl 没有定长记录, 原地改会破坏后续行; 而重写实现最容易犯的错是
    只写回被改动的那些.
    """
    from xbrain.boot.boot_fail import PERSIST_NAME, mark_uplinked, write

    d = str(tmp_path)
    for i in range(3):
        write(_rec(boot_id="b-%d" % i), runtime_path=str(tmp_path / "rt.json"),
              persist_dir=d)
    mark_uplinked(d, ["b-1"])
    lines = (tmp_path / PERSIST_NAME).read_text(
        encoding="utf-8").strip().split("\n")
    assert len(lines) == 3, "重写后条目数变了: %s" % lines
    rows = [json.loads(x) for x in lines]
    assert [r["boot_id"] for r in rows] == ["b-0", "b-1", "b-2"]
    assert rows[1]["uplinked"] is True
    assert "uplinked" not in rows[0] and "uplinked" not in rows[2]


def test_the_event_uses_an_existing_category_and_channel():
    """*** 判据逐字: 不新增 key, 不新增 category.

    补发走既有的 event/fault/system, channel = normal. 新通道意味着云端
    要改 -- 而这条判据的价值恰恰在于"用已经通了的路把话带出去".
    """
    from xbrain.boot.boot_fail import REQUIRED_DETAIL, to_event

    ev = to_event(_rec())
    assert ev["category"] == "system", "用了新的 category"
    assert ev["severity"] == "fault"
    assert ev["channel"] == "normal", "没有走既有的 normal 通道"
    # detail 恰好是四项, 不多不少 -- 多出的键就是"新增 key".
    assert set(ev["detail"]) == set(REQUIRED_DETAIL), (
        "detail 的键集合不是那四项: %s" % sorted(ev["detail"]))


def test_the_stage_0z2_scenario_end_to_end(tmp_path):
    """*** 判据点名的变异体场景: Stage 0z-2 失败(通用面根本不存在).

    那一刻上行通道本身就是失败的那一项. 走一遍: 落盘 -> 下次启动读出 ->
    转事件 -> 标记已上行 -> 不再重复.
    """
    from xbrain.boot.boot_fail import (mark_uplinked, read_pending, to_event,
                                       write)

    d = str(tmp_path)
    rec = _rec(stage="0z-2", code="E_CONFIG_INVALID",
               message="gen plane router absent")
    write(rec, runtime_path=str(tmp_path / "rt.json"), persist_dir=d)

    # 下一次成功启动: p5 扫到它.
    pending = read_pending(d)
    assert len(pending) == 1, "下次启动没读到上次的失败"
    ev = to_event(pending[0])
    assert ev["detail"]["stage"] == "0z-2"
    assert ev["detail"]["boot_id"] == rec["boot_id"]

    mark_uplinked(d, [pending[0]["boot_id"]])
    assert read_pending(d) == [], "标记后仍会重复补发"
