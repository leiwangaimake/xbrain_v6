#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: qt_contract_collide.py
Brief: 云端 Qt 契约 v2.0 与 11 接口契约的逐条对撞

Description:
回国后的首要任务是云端联调, 而联调成败取决于两份契约是否逐条对得上. 本脚本
把那次对撞做成可复跑的, 理由很直接: 一份人读一遍得出的差异清单, 在任何一侧
改动之后就过期了 -- 而过期的清单与新鲜的清单读起来一模一样.

*** 三档差异的处置完全不同, 所以必须分开报.
  BLOCK  联调会直接失败(key 对不上 / 字段名不同 / 闭集不同)
  GAP    11 里没有这条契约, 网关照 v2.0 实现即可, 但 11 应补登记
  MAP    两侧都有, 只是命名或形态不同 -> 网关做映射(R10.1 已裁"映射由网关
         统一实现")
把 GAP 当 BLOCK 会让人以为联调不可能开始; 把 BLOCK 当 GAP 会让人在联调当天
才发现 Qt 发的 key 我方没人订阅.

*** 扫描面显式声明(CLAUDE.md 3.2 形态6).
只扫三份[契约文件]:
  docs/MISSON/任务枚举_qt端v2.0.md      甲方 v2.0 定稿(任务与 key)
  docs/MISSON/json格式文件_qt端v2.0.md  甲方 v2.0 定稿(字段与闭集)
  docs/11-接口契约.md                    我方契约唯一真源
NO 不扫评审意见书 docx(它是[历史来源])与客户答复 txt(它是[裁决记录]) --
两者解释差异成因, 但它们不是判据源. 拿一份已被 v2.0 取代的初稿当判据,
会把"客户已经改过来了"的条目重新报成差异.

*** 一处必须说明的局限.
本脚本判定的是[字面存在性]: 某个 key/字段名在 11 里出现过没有. 它判不了
语义一致 -- 比如 11 的 error_code 是充电底盘的, 与云端 Qt 的整数码毫无关系,
而脚本只会报"命中 9 次". 所以每条结论后面都带证据行, 由人复核;
MAP 那几条尤其如此.

Boundaries: 只报差异, 不改任何文件, 不裁决. 需要人拍板的进 docs/MISSON/
契约对撞_v2.0对11.md 的 E 节.

  python3 scripts/doccheck/qt_contract_collide.py          # 全量对撞
  python3 scripts/doccheck/qt_contract_collide.py --block  # 只报 BLOCK
"""

import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))

QT_TASKS = os.path.join(ROOT, "docs", "MISSON", "任务枚举_qt端v2.0.md")
QT_JSON = os.path.join(ROOT, "docs", "MISSON", "json格式文件_qt端v2.0.md")
DOC11 = os.path.join(ROOT, "docs", "11-接口契约.md")

BLOCK, GAP, MAP, OK = "BLOCK", "GAP", "MAP", "OK"


class Item:
    """一条对撞项.

    verdict 必须显式写出来, NO 不由命中数自动推 -- 命中数只说明字面上有没有,
    而 11 的 error_code 命中 9 次却是充电底盘的, 那是 MAP 不是 OK.
    判定是人做的, 脚本负责让它可复核.
    """

    def __init__(self, name, pattern, verdict, note, alt=()):
        self.name = name
        self.pattern = pattern      # 在 11 里找什么
        self.verdict = verdict      # 复核过的结论
        self.note = note
        self.alt = tuple(alt)       # 替代措辞, 用于证明"真的没有"


#: 对撞表. 每条的 verdict 是[逐条复核过的], 不是脚本推出来的.
ITEMS = (
    # -- A 组: 最要紧的三条 ------------------------------------------
    Item("云端下发 key", "cmd/task/ext", BLOCK,
         "v2.0 用 xbrain/{rid}/cmd/task; 11 的云端入站口是 cmd/task/ext, "
         "且 11 逐字写 p3_task 不得订阅 ext. Qt 发到 cmd/task 会被 p3_task "
         "直接收到, 绕过网关的信封重建与 rid 校验"),
    Item("外层信封字段", "schema_version`/`msg_id", BLOCK,
         "11 记的是客户 v1.0 七字段(schema_version/robot_id/timestamp); "
         "v2.0 已全面改为六字段 v/rid/ts/seq/src/data -- 评审意见客户采纳了, "
         "而 11 没跟着更新"),
    Item("ts 格式冻结", "float64 Unix", GAP,
         "v2.0 逐字冻结 float64 Unix 秒并禁 ISO8601; 11 只有一处提及, "
         "且未写明它只适用跨主机面(机内仍是 ts+mono 双字段, CLK-C3)"),

    # -- B 组: 11 里完全没有的契约面 ---------------------------------
    Item("data/file/index", "data/file/index", GAP,
         "文件索引 key, 11 全册零命中", alt=("file/index", "文件索引")),
    Item("cmd/file/ack", "cmd/file/ack", GAP, "文件下载回执 key"),
    Item("cmd/media/session", "cmd/media/session", GAP,
         "录像会话回写 key. ! 11 里只在[评审待办清单]出现过"
         "(R9.4 建议落点), 契约本体没有 -- 待办不是契约",
         alt=("录像会话",)),
    Item("credential_ref", "credential_ref", GAP,
         "state/media 每个 endpoint 的独立凭据引用"),
    Item("message_type", "message_type", GAP,
         "R12.4 变通: task/result 合并进 state/task, 用 snapshot|result 区分. "
         "客户答复确认我方接受, 11 必须登记"),
    Item("progress_percent", "progress_percent", GAP,
         "逐字'未知时必须为 null, 禁止填 0 冒充'", alt=("progress",)),
    Item("E_TASK_UNSUPPORTED", "E_TASK_UNSUPPORTED", GAP,
         "需进 11 S13 错误码闭集与 codes.yaml. "
         "CLAUDE.md 3.5: 不得自造码, 由 common/errors 导出"),
    Item("E_RID_MISMATCH", "E_RID_MISMATCH", GAP, "同上"),
    Item("E_VERSION_UNSUPPORTED", "E_VERSION_UNSUPPORTED", GAP, "同上"),
    Item("person_in_region", "person_in_region", GAP,
         "报警规则 type 闭集"),

    # -- C 组: 需网关映射 --------------------------------------------
    Item("error_code 整数码", "error_code", MAP,
         "! 11 的 error_code 是充电底盘的(/CHARGE_STATUS.error_code), "
         "与云端 Qt 整数码无关. 需新建 E_* -> error_code 映射表, "
         "且必须双向完备 -- 未映射的 E_* 落到兜底码会让 Qt 看到无法区分的错误"),
    Item("recorded_path_id", "route_id", MAP,
         "v2.0 下发用 recorded_path_id(计划级), 回报用 route_id/route_rev"
         "(实际加载的). 两者可能不同(路径被重录过), 那正是 R8.6 版本握手"),
    Item("state/link 的 state", "estop_path", MAP,
         "v2.0 要 state 三值 up|degraded|down; 11 与代码用 level 0..3. "
         "! 不是一一对应: 11 有 L3(返航触发), v2.0 只有三值 -- 需裁决"),
    Item("ESTOP 作为 task_type", "cmd/estop", MAP,
         "v2.0 的 ESTOP 报文里 data.task_type='ESTOP'; "
         "11 有 cmd/estop key 但 ESTOP 不是 task_type 值"),
    Item("state/media 保活频率", "state/media", MAP,
         "11 写 0.1 Hz(10 s), v2.0 要每 5 s 全量保活. "
         "! 必须一致 -- Qt 按 5 s 判超时而我方 10 s 才发, 每次都会误判掉线"),

    # -- D 组: 已确认一致 --------------------------------------------
    Item("state/geo/manifest", "state/geo/manifest", OK, ""),
    Item("audio/broadcast", "audio/broadcast", OK, ""),
    Item("stream_id", "stream_id", OK, ""),
    Item("pc_to_dog", "pc_to_dog", OK, ""),
    Item("exit_broadcast", "exit_broadcast", OK, ""),
    Item("payload_b64", "payload_b64", OK, ""),
    Item("chunk_seq", "chunk_seq", OK, ""),
    Item("base_rev", "base_rev", OK, ""),
    Item("E_GEO_CONFLICT", "E_GEO_CONFLICT", OK, ""),
    Item("clock.ts_sync", "ts_sync", OK, ""),
    Item("alarm_window_active", "alarm_window_active", OK, ""),
    Item("arrival_radius_m", "arrival_radius_m", OK, ""),
    Item("alarm_role", "alarm_role", OK, ""),
    Item("GOTO_KEYPOINT", "GOTO_KEYPOINT", OK, ""),
    Item("STOP_TASK", "STOP_TASK", OK, ""),
    Item("SET_ALARM_CONFIG", "SET_ALARM_CONFIG", OK, ""),
    Item("AUDIO_CONTROL", "AUDIO_CONTROL", OK, ""),
)


def qt_keys():
    """从 v2.0 §2 的正式 key 全量表提取 key 列表.

    现场解析而不是抄一份: 客户改了表, 这里跟着变.
    """
    text = _read(QT_TASKS)
    out = []
    for line in text.split("\n"):
        m = re.match(r"^\|\s*`(xbrain/\{rid\}/[^`]+)`\s*\|\s*([^|]+)\|", line)
        if m:
            out.append((m.group(1), m.group(2).strip()))
    if not out:
        raise SystemExit("v2.0 key 全量表解析到 0 条 -- 表结构变了, 本门已失效")
    return out


def _read(path):
    if not os.path.isfile(path):
        raise SystemExit("契约文件缺失: %s" % path)
    with open(path, encoding="utf-8") as fh:
        return fh.read()


def collide():
    """跑一遍对撞. 返回 (结论列表, 是否有 BLOCK)."""
    doc11 = _read(DOC11)
    rows = []
    for item in ITEMS:
        hits = doc11.count(item.pattern)
        alt_hits = {a: doc11.count(a) for a in item.alt}
        rows.append((item, hits, alt_hits))
    return rows


def main():
    only_block = "--block" in sys.argv
    rows = collide()
    keys = qt_keys()

    print("scan surface: %s + %s  vs  %s"
          % (os.path.basename(QT_TASKS), os.path.basename(QT_JSON),
             os.path.basename(DOC11)))
    print("v2.0 正式 key: %d 条" % len(keys))
    print()

    order = {BLOCK: 0, GAP: 1, MAP: 2, OK: 3}
    rows.sort(key=lambda r: (order[r[0].verdict], r[0].name))
    counts = {BLOCK: 0, GAP: 0, MAP: 0, OK: 0}
    for item, hits, alt_hits in rows:
        counts[item.verdict] += 1
        if only_block and item.verdict != BLOCK:
            continue
        if item.verdict == OK:
            continue
        print("[%-5s] %-24s 11 命中 %d" % (item.verdict, item.name, hits))
        if alt_hits:
            # 替代措辞的命中数是"真的没有"这个结论的证据.
            print("          替代措辞: %s"
                  % ", ".join("%s=%d" % (k, v) for k, v in alt_hits.items()))
        if item.note:
            for line in _wrap(item.note, 74):
                print("          " + line)
        print()

    print("criterion: zero BLOCK before cloud integration testing")
    print("BLOCK=%d GAP=%d MAP=%d OK=%d"
          % (counts[BLOCK], counts[GAP], counts[MAP], counts[OK]))
    # BLOCK 才让门失败. GAP 是"照 v2.0 实现即可"的活, MAP 是网关映射的活 --
    # 两者都不该阻断 CI, 阻断的话这个门会在联调前被关掉.
    return 1 if counts[BLOCK] else 0


def _wrap(text, width):
    out, line = [], ""
    for word in text.split(" "):
        if len(line) + len(word) + 1 > width:
            out.append(line)
            line = word
        else:
            line = (line + " " + word).strip()
    if line:
        out.append(line)
    return out


if __name__ == "__main__":
    sys.exit(main())
