"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: boot_fail.py
Brief: CFG-BT-5 / BOOT-I4 -- 启动失败落盘, 下次开机补发

Description:
BOOT-I4 只有一句: "任何启动失败都必须在[下一次成功启动后]被看见".

这条存在的理由是时序: 启动失败发生在门控阶段, 而那时上行通道往往还没起来
(通用面 router 可能就是失败的那一项). 失败信息如果只写日志, 它就留在那台
机器上 -- 而现场看到的是"机器人没起来", 云端什么都没收到.

*** 两个落点, 各有各的失效模式, 所以都要.
  /run/xbrain/boot_fail.json    tmpfs. 掉电即失, 但[本次开机]的其它进程
                                能立刻读到, 用于 HMI 显示当前这次的失败.
  <record.db 同盘>/boot_fail.jsonl
                                持久盘, [追加不覆盖]. 这一份才是"下次开机
                                补发"的依据 -- 而覆盖写会让连续两次失败
                                只剩最后一次, 恰好丢掉最早那个根因.

*** 判据逐字: 不新增 key, 不新增 category.
补发走既有的 event/fault/system, channel = normal. NO 不为它开一条新通道 --
新通道意味着云端要改, 而这条的价值恰恰在于"用已经通了的路把话带出去".

Boundaries: 只负责写与读, 不负责发. 补发由 p5_gateway 的事件管线做 --
本模块给它一个"还没上行过的条目"列表.
"""
from __future__ import annotations

import json
import os
from typing import Dict, List, Optional

#: tmpfs 落点. 本次开机内可读, 掉电即失.
RUNTIME_PATH = "/run/xbrain/boot_fail.json"

#: 持久落点的文件名; 目录由调用方给(record.db 所在盘).
PERSIST_NAME = "boot_fail.jsonl"

#: detail 必填四项. 少任何一项, 云端收到的事件都无法定位.
REQUIRED_DETAIL = ("stage", "code", "boot_id", "message")


class BootFailRecordError(ValueError):
    """记录形状不合法."""


def validate(record: Dict) -> None:
    """detail 四项必填. 缺一即抛.

    NO 不给缺失项填默认值: 一条 stage 为 "unknown" 的失败记录, 与没有这条
    记录相比只多了噪声 -- 运维还是不知道卡在哪一阶段.
    """
    missing = [k for k in REQUIRED_DETAIL if not record.get(k)]
    if missing:
        raise BootFailRecordError(
            "boot_fail record missing %s (BOOT-I4 requires all of %s)"
            % (missing, list(REQUIRED_DETAIL)))


def write(record: Dict, *, runtime_path: str = RUNTIME_PATH,
          persist_dir: Optional[str] = None) -> List[str]:
    """两处都写. 返回真正写成功的路径列表.

    *** tmpfs 写失败不得阻止持久盘写.
    /run 在某些故障场景下本身就不可写(比如 tmpfs 满). 那时更需要把记录
    留在持久盘上 -- 一个"先写 tmpfs 失败就整个放弃"的实现, 恰好在最糟的
    时候什么都不留.
    """
    validate(record)
    written = []
    try:
        os.makedirs(os.path.dirname(runtime_path), exist_ok=True)
        with open(runtime_path, "w", encoding="utf-8") as fh:
            json.dump(record, fh, ensure_ascii=False)
        written.append(runtime_path)
    except OSError:
        # 吞掉: 见上面的说明. 持久盘那一份才是补发的依据.
        pass
    if persist_dir:
        path = os.path.join(persist_dir, PERSIST_NAME)
        try:
            os.makedirs(persist_dir, exist_ok=True)
            # *** 追加, NO 不覆盖.
            # 覆盖会让连续两次失败只剩最后一次 -- 而最早那次往往是根因,
            # 后面的都是它的连锁反应.
            with open(path, "a", encoding="utf-8") as fh:
                fh.write(json.dumps(record, ensure_ascii=False) + "\n")
            written.append(path)
        except OSError:
            pass
    return written


def read_pending(persist_dir: str) -> List[Dict]:
    """读出还没上行过的条目(uplinked 不为真的).

    坏行跳过而不是整份放弃: jsonl 最后一行可能因为掉电而截断, 而前面那些
    完好的条目仍然要被补发. 一个"解析失败就返回空"的实现会因为一行残缺
    丢掉整份历史.
    """
    path = os.path.join(persist_dir, PERSIST_NAME)
    if not os.path.isfile(path):
        return []
    out = []
    with open(path, encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                row = json.loads(line)
            except ValueError:
                continue                     # 截断行, 跳过
            if isinstance(row, dict) and not row.get("uplinked"):
                out.append(row)
    return out


def mark_uplinked(persist_dir: str, boot_ids: List[str]) -> int:
    """把这些 boot_id 的条目标成已上行. 返回改动条数.

    整份重写而不是原地改: jsonl 没有定长记录, 原地改会破坏后续行.
    重写前先读全量, 所以这一步不丢没被标记的条目.
    """
    path = os.path.join(persist_dir, PERSIST_NAME)
    if not os.path.isfile(path):
        return 0
    rows, changed = [], 0
    with open(path, encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                row = json.loads(line)
            except ValueError:
                continue
            if (isinstance(row, dict) and row.get("boot_id") in boot_ids
                    and not row.get("uplinked")):
                row["uplinked"] = True
                changed += 1
            rows.append(row)
    with open(path, "w", encoding="utf-8") as fh:
        for row in rows:
            fh.write(json.dumps(row, ensure_ascii=False) + "\n")
    return changed


def to_event(record: Dict) -> Dict:
    """把一条失败记录转成既有的 event/fault/system 事件体.

    NO 不新增 key 也不新增 category(判据逐字). 走 channel = normal 的
    既有 U18a 双通道 -- 新通道意味着云端要改, 而这条的价值恰恰在于用
    已经通了的路把话带出去.
    """
    validate(record)
    return {
        "category": "system",
        "severity": "fault",
        "channel": "normal",
        "detail": {k: record[k] for k in REQUIRED_DETAIL},
    }
