"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_task_events.py
Brief: 11 S6.2 task-event mapping -- (to_state, reason) -> kind/sev

Description:
Pins the 11 S6.2 task row (info/warn, accept/reject/start/complete/fail): a
validate-fail reason is a reject (warn); ready/running/succeeded are info;
failed/aborted/cancelled are warn; internal states (pending/suspended) produce no
event. Mutations paired per 3.3.
"""
from __future__ import annotations

import io
import textwrap
import tokenize

import pytest

from xbrain.p3_task.state.machine import TRANSITIONS
from xbrain.p3_task.state.task_events import (
    _TRANSITION_EVENT,
    task_event_for_transition,
)

pytestmark = pytest.mark.no_device


# --------------------------------------------------- 判别依据是迁移, 不是 reason

def test_only_the_admission_failure_is_a_rejection():
    """*** 这条是本次重写的靶心.

    拒绝有唯一的迁移特征: 准入校验没过, pending -> failed. 其余六种带 reason
    的迁移都不是拒绝, 而旧实现按 "reason 非空" 判, 把它们全报成了 rejected --
    2026-09-03 甲方按一次暂停, 界面上就是一条 "task ... rejected".

    MUTATION: 把 ("pending","failed") 改成别的 kind -> 红.
    """
    assert task_event_for_transition("pending", "failed") == ("rejected", "warn")


def test_pause_emits_no_event_at_all():
    """S6.2 那一类逐字是"接受/拒绝/开始/完成/失败", 没有暂停. 操作员看暂停要
    从 state/task 快照看(state=paused, 1 Hz), 事件流不是状态镜像.

    MUTATION: 给 ("running","suspended") 填任何一个 kind -> 红.
    """
    assert task_event_for_transition("running", "suspended") is None


def test_resume_emits_no_event_either():
    """恢复同理. 顺带守住"让位结束自动恢复"那条(driver 的 phase 1b), 它走的是
    同一条边.

    MUTATION: 给 ("suspended","ready") 填 kind -> 红.
    """
    assert task_event_for_transition("suspended", "ready") is None


def test_preemption_is_not_a_rejection():
    """被高优先级抢占 = running -> suspended, 与操作员暂停同一条边. 旧实现因为
    reason="preempted" 非空而报 rejected.

    MUTATION: 恢复 `if reason:` 那条规则 -> 红(它会让本条返回 rejected).
    """
    assert task_event_for_transition("running", "suspended") is None


def test_cancel_says_cancelled_not_rejected():
    """甲方按停止 -> running -> cancelled. 旧实现报 rejected(reason 是操作员
    填的 operator_stop).

    MUTATION: 把 ("running","cancelled") 改成 rejected -> 红.
    """
    assert task_event_for_transition("running", "cancelled") == \
        ("cancelled", "warn")
    # 排队中被取消也一样是取消, 不是拒绝.
    assert task_event_for_transition("ready", "cancelled") == \
        ("cancelled", "warn")


def test_a_deferred_task_is_accepted_not_rejected():
    """定时任务与等依赖的任务都是[受理了], 只是还不能跑. 旧实现同样按 reason
    非空报成 rejected -- 甲方提交一条定时任务会收到"被拒绝".

    MUTATION: 把这两条改成 None 或 rejected -> 红.
    """
    assert task_event_for_transition("pending", "scheduled") == \
        ("accepted", "info")
    assert task_event_for_transition("pending", "blocked") == \
        ("accepted", "info")


# ------------------------------------------------------------- 正向的那几条不能丢

def test_the_five_s6_2_kinds_still_come_out():
    """重写不能把原本对的那半弄丢.

    MUTATION: 删掉 ("pending","ready") 或 ("ready","running") -> 红.
    """
    assert task_event_for_transition("pending", "ready") == ("accepted", "info")
    assert task_event_for_transition("ready", "running") == ("started", "info")
    assert task_event_for_transition("running", "done") == ("completed", "info")
    assert task_event_for_transition("running", "failed") == ("failed", "warn")
    assert task_event_for_transition("pending", "failed") == ("rejected", "warn")


def test_completion_finally_emits_something():
    """*** 旧表的键写的是 "succeeded", 而状态机里的终态叫 "done" -- 于是任务
    跑完[永远不发完成事件]. 今天没暴露只因为还没有执行器能让任务跑完.

    MUTATION: 把键改回 "succeeded" -> 红(("running","done") 查不到就抛).
    """
    assert task_event_for_transition("running", "done") == ("completed", "info")


def test_the_power_off_bookkeeping_does_not_double_report():
    """任务终结时已经发过一条事件, 关机前再发一条就是同一件事上报两次.

    MUTATION: 给 ("done","wait_for_power_off") 填 kind -> 红.
    """
    for frm in ("done", "failed", "cancelled", "needs_review"):
        assert task_event_for_transition(frm, "wait_for_power_off") is None


# ------------------------------------------------------------------ 完备性判据

def test_the_table_covers_every_edge_the_state_machine_can_take():
    """*** 表缺一条边, 那条迁移在运行期就抛.

    这条判据是那个 raise 的前提: 它保证 raise 在生产里不可达, 同时保证[新增
    一条边必须在这里表态]. 没有它的话, 加一条边就会静默不发事件 -- 正是
    running->done 那条的老毛病.

    MUTATION: 从 _TRANSITION_EVENT 删掉任意一条 -> 红.
    """
    machine_pairs = {(frm, to) for (frm, _ev), to in TRANSITIONS.items()}
    missing = machine_pairs - set(_TRANSITION_EVENT)
    assert not missing, "状态机有边而事件表没表态: %r" % sorted(missing)


def test_the_table_has_no_edge_the_state_machine_cannot_take():
    """反向差集: 表里的键必须都是真实存在的边.

    旧表的 "succeeded"/"aborted" 就是这么混进来的 -- 两个状态机里根本没有的
    状态, 写在表里没人发现, 而真正该映射的 "done" 反而漏了.

    MUTATION: 往表里加一条 ("running","succeeded") -> 红.
    """
    machine_pairs = {(frm, to) for (frm, _ev), to in TRANSITIONS.items()}
    extra = set(_TRANSITION_EVENT) - machine_pairs
    assert not extra, "事件表里有状态机走不到的迁移: %r" % sorted(extra)


def test_an_impossible_transition_raises_instead_of_returning_none():
    """静默返回 None 会让新增的边悄悄不发事件. 抛出来才有人看见.

    MUTATION: 把 raise 换成 return None -> 红.
    """
    with pytest.raises(KeyError) as exc:
        task_event_for_transition("running", "pending")   # 状态机里没有这条边
    assert "running" in str(exc.value) and "pending" in str(exc.value)


# --------------------------------------------- 接线: _make_publish 用哪一对判别

@pytest.mark.asyncio
async def test_the_publish_seam_decides_on_the_from_to_pair():
    """*** 纯函数对了不等于接线传对了.

    变异测试显示: 把 _make_publish 里的 task_event_for_transition(from, to) 改成
    (to, to), 上面所有单测照样全绿 -- 因为它们测的是函数本身. 判别既然按 (from,
    to) 做, 就要有一条断言盯着[接线实际喂进去的那一对].

    MUTATION: 改成 (to_state, to_state) -> 红(running->suspended 会变成
    suspended->suspended, 表里没有, 抛 KeyError).
    """
    import json

    from xbrain.p3_task.runtime.main_wiring import _make_publish

    class _Pub:
        def __init__(self):
            self.puts = []

        def put(self, payload):
            self.puts.append(json.loads(payload.decode("utf-8")))

    emitted = []

    def _emit(task_id, to_state, kind, sev, extra=None):
        # extra 是终态事实(v2.0 S3.3 的 result 要它们); 非终态迁移传 None.
        emitted.append((task_id, to_state, kind, sev))

    pub = _Pub()
    publish = _make_publish(pub, _emit)

    # 取消: running -> cancelled, 且 reason 非空(甲方填的). 旧实现在这里报
    # rejected, 新实现必须报 cancelled.
    await publish("t-1", "running", "cancelled", "operator_stop")
    assert emitted == [("t-1", "cancelled", "cancelled", "warn")], emitted

    # 暂停: running -> suspended, reason 同样非空 -> 一条事件都不发.
    emitted.clear()
    await publish("t-2", "running", "suspended", "operator_pause")
    assert emitted == [], emitted


@pytest.mark.asyncio
async def test_the_publish_seam_still_logs_the_reason_it_no_longer_judges_by():
    """reason 不再参与判别, 但它是给人看的, 不能顺手丢掉 -- 日志里没有它,
    "任务为什么停了"就查不出来了.

    MUTATION: 从 _publish 的日志里去掉 reason -> 红.
    """
    import inspect

    from xbrain.p3_task.runtime.main_wiring import _make_publish

    src = inspect.getsource(_make_publish)
    assert "reason" in src
    assert "from_state, to_state, reason" in src, (
        "日志没有同时带上 from/to/reason, 出问题时看不出迁移是从哪来的")


# ------------------------------------------- 11 S6.1 Event.ts 必须是发生时刻


def _nested_def_body(src: str, name: str) -> str:
    """把 src 里名为 name 的嵌套函数体切出来, 按缩进定界.

    *** 不用"从 def 起数 N 个字符"那种切法: 本判据第一版就是 1400 字符, 而给
    该函数补了几行注释后 gen.put 就落在窗口外, 断言当场变红. 那种红是[判据自己
    的长度假设过期], 不是代码错 -- 按 CLAUDE.md S3.2 形态2, 恒红的判据最后会被
    人放宽成"包含即可", 于是变成恒绿. 按缩进定界没有这个长度假设.
    """
    needle = "def %s(" % name
    assert needle in src, "%s 搬走了, 本判据要重新对靶" % name
    start = src.index(needle)
    line_start = src.rfind("\n", 0, start) + 1
    indent = len(src[line_start:start])          # def 这一行的缩进宽度
    lines = src[line_start:].splitlines()
    out = [lines[0]]
    for line in lines[1:]:
        # 空行属于函数体; 缩进回到 def 同级或更浅即函数结束.
        if line.strip() and len(line) - len(line.lstrip()) <= indent:
            break
        out.append(line)
    return textwrap.dedent("\n".join(out))


def _code_only(text: str) -> str:
    """去掉注释, 只留代码.

    *** 本判据踩过一次[判据自伤](CLAUDE.md S3.2 形态3): 下面那条断言要在函数体
    里 grep 带引号的 dedup key 字段名, 而同一个函数体里正好有一行解释性注释原样
    写着 ev.get(...) 那个字段名 -- 于是命中数[永不可能为 0], 断言恒红. 判据句必须
    在自己的扫描面之外, 所以这里把注释整类移出扫描面.

    用 tokenize 而不是按 # 切: 字符串里的 # 会被按字面切掉, 那种切法会在某天悄悄
    改变被测文本.
    """
    try:
        toks = list(tokenize.generate_tokens(io.StringIO(text).readline))
    except (tokenize.TokenError, IndentationError):      # pragma: no cover
        # 切出来的片段不成完整语法时退回原文: 宁可让断言在更大的面上求值(可能
        # 误红并被人看到), NO 也不要静默换成一个更容易通过的输入.
        return text
    return "".join(
        t.string if t.type != tokenize.COMMENT else ""
        for t in toks
        if t.type not in (tokenize.ENCODING, tokenize.ENDMARKER))


def test_任务事件的ts是本拍墙钟而不是写死的零():
    """*** 这一条只能在源文本上求值: 装配层的 gen.put 需要真 Zenoh session.

    原状: _emit_task_event 里 "ts": 0.0 写死. 它一直看不出问题, 是因为 p5 的
    _normalise_event 写的是
      d.get("ts") or data.get("ts") or now.timestamp()
    而 0.0 是[假值] => 入库的是 p5 收包时刻, 去重算式仍看到递增 ts, 所以没有
    任何测试会红.

    为何这一处代价最大: 同一段代码上面的注释自己写着 -- 11 S4.4 的 TaskState
    [只列非终态任务], 任务一终结就从广播里消失, "事件是终态那一刻唯一还带着
    任务的报文". 那份报文的时间戳是唯一一份, 写 0.0 等于把"任务什么时候完成
    的"交给 p5 的调度时机; p5 落后或重启时, 终态时间就是错的.

    连带: 这条通路靠一个[碰巧为假]的值兜底. 把那个 or 收紧成 is None(p5 侧的
    正确修法)会让每一条都变成真的 1970 年.

    MUTATION: 把 time.time() 改回 0.0 -> 红.
    """
    import inspect

    from xbrain.p3_task.runtime import main_wiring

    body = _nested_def_body(inspect.getsource(main_wiring._amain),
                            "_emit_task_event")
    assert 'gen.put("event/%s/task"' in body, "任务事件的 put 不在这一段里了"
    assert '"ts": time.time()' in body, (
        "任务事件没有带本拍墙钟; 写死的 ts 会被 p5 的 or 兜底换成收包时刻")
    assert '"ts": 0.0' not in body, "写死的 ts=0.0 回来了"


def test_任务事件不带dedup_key所以也不需要窗口():
    """会静默并掉事件的组合是[带 key 不带窗口], 不是[不带窗口].

    11 S6.2 的 task 行只规定 sev(info/warn) 与 channel(normal), 既没有
    dedup_key 也没有 dedup_window_s. 而 p5 的 record_dao._attempt_insert 只在
    ev.get("dedup_key") 为真时才调 _try_merge => 不带 key 时窗口根本读不到,
    带 key 不带窗口才会让 record_dao 把后续每一条都并进第一行(dedup_count 在
    涨, 云端只看得到一条).

    NO 不给它编一个窗口秒数: 那是 CLAUDE.md S3.7 说的"没人实测过的判定量",
    而且加 key 或窗口都是改契约, 不是实现选择.

    MUTATION: 在 _emit_task_event 的 put 里加 dedup_key 而不加窗口 -> 红.
    """
    import inspect

    from xbrain.p3_task.runtime import main_wiring

    body = _code_only(_nested_def_body(inspect.getsource(main_wiring._amain),
                                       "_emit_task_event"))
    if '"dedup_key"' in body:
        assert '"dedup_window_s"' in body, (
            "任务事件长出了 dedup_key 却没有 dedup_window_s -- p5 对这个组合"
            "是无条件合并, 审计链会塌成一行")
