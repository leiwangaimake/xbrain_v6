"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_task_ack.py
Brief: A-3 ack 八字段 + 幂等窗口 -- 两处会让 Qt 显示错误结论的地方

Description:
两组判据, 各对应一种[Qt 看到的与实际发生的不一致]:

*** 一 result 与 accepted 打架.
一条 {result:"rejected", accepted:true} 的 ack 会让 Qt 走成功分支而机器人
什么都没做. 两个字段表达同一件事时它们迟早会不一致, 所以 accepted 由
result 推导而不是让调用方单独传.

*** 二 幂等做在执行之后.
v2.0 S4.1 逐字"duplicate 表示原请求已经处理, 不能再次创建任务". 一个
"先执行再查重"的实现会让重发的 GOTO_KEYPOINT 创建第二条任务, 而 Qt 收到的
ack 是 duplicate -- 它以为什么都没发生. 现场表现是机器人跑了两遍.

Boundaries: 只测 ack 形状与判重. 发布与时限(2 秒内回)由接线负责.
"""
from __future__ import annotations

import pytest

pytestmark = pytest.mark.no_device


def _ack(**over):
    from xbrain.p5_gateway.outbound.task_ack import build_ack

    kw = {"msg_id": "ack-1", "ref_msg_id": "msg-1", "task_id": "task-1",
          "task_type": "GOTO_KEYPOINT", "result": "accepted"}
    kw.update(over)
    return build_ack(**kw)


# --- 一 ack 八字段与一致性 -------------------------------------------

def test_ack_carries_every_required_field():
    """*** v2.0 S3.1 的九个键一个不能少.

    少 ref_msg_id 的话 Qt 无法把 ack 关联回原请求 -- 而它可能同时有几条
    在途请求.
    """
    body = _ack()
    for field in ("msg_id", "ref_msg_id", "task_id", "task_type", "result",
                  "accepted", "error_code", "reason", "detail"):
        assert field in body, "ack 缺字段 %s" % field


def test_task_type_is_never_omitted_even_for_audio():
    """v2.0 S3.1 逐字: "音频 ack 也不得省略"."""
    body = _ack(task_type="AUDIO_CONTROL")
    assert body["task_type"] == "AUDIO_CONTROL"


def test_accepted_is_derived_not_supplied():
    """*** 两个字段表达同一件事时, 它们迟早会不一致.

    一条 {result:"rejected", accepted:true} 的 ack 会让 Qt 走成功分支而
    机器人什么都没做. v2.0 S3.1 给了对应关系, 这里让代码去推而不是让
    调用方传.
    """
    from xbrain.p5_gateway.outbound.task_ack import build_ack
    import inspect

    sig = inspect.signature(build_ack)
    assert "accepted" not in sig.parameters, (
        "build_ack 接受 accepted 参数 -- 那让它有机会与 result 不一致")
    assert _ack(result="accepted")["accepted"] is True
    assert _ack(result="duplicate")["accepted"] is True
    assert _ack(result="rejected", error_code=1003,
                reason="字段非法")["accepted"] is False


def test_result_is_a_closed_set():
    """三值闭集. error / failed 之类要抛(CLAUDE.md 3.5 闭集外必抛)."""
    from xbrain.p5_gateway.outbound.task_ack import AckShapeError

    for bad in ("error", "failed", "ok", "", None):
        with pytest.raises(AckShapeError):
            _ack(result=bad)


def test_a_rejection_must_carry_a_code_and_a_reason():
    """*** v2.0 S10 逐字: 所有拒绝都要非零 error_code + 人类可读 reason.

    一条 reason 为空的拒绝, 操作员看到的是"操作失败"四个字.
    """
    from xbrain.p5_gateway.outbound.task_ack import AckShapeError

    with pytest.raises(AckShapeError):
        _ack(result="rejected", error_code=0, reason="有理由但码是 0")
    with pytest.raises(AckShapeError):
        _ack(result="rejected", error_code=1003, reason="")


def test_an_acceptance_must_not_carry_an_error_code():
    """反向: 受理却带非零码, 是两处判断打架的信号.

    Qt 会按 accepted 走成功分支, 而 error_code 里的原因永远没人看.
    """
    from xbrain.p5_gateway.outbound.task_ack import AckShapeError

    with pytest.raises(AckShapeError):
        _ack(result="accepted", error_code=2001)


def test_detail_defaults_to_an_empty_object_not_null():
    """v2.0 S3.1: detail 无内容为 {}.

    null 会让 Qt 那边的 detail.code 读取抛异常 -- 而它对每条 ack 都会读.
    """
    assert _ack()["detail"] == {}


# --- 二 幂等窗口 ------------------------------------------------------

class _FakeClock:
    """可控单调钟. NO 不用真 sleep -- 60 秒的窗口没法等."""

    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


def test_first_time_is_not_a_duplicate():
    from xbrain.p5_gateway.outbound.task_ack import DedupWindow

    win = DedupWindow(clock=_FakeClock())
    assert win.seen("gj-001", "msg-1") is False


def test_a_resend_within_the_window_is_a_duplicate():
    """*** 重发必须被认出来.

    v2.0 S1.2: 重发同一业务请求必须复用原 msg_id, 不得生成新 ID 后冒充重发.
    """
    from xbrain.p5_gateway.outbound.task_ack import DedupWindow

    clock = _FakeClock()
    win = DedupWindow(clock=clock)
    win.seen("gj-001", "msg-1")
    clock.advance(30.0)
    assert win.seen("gj-001", "msg-1") is True


def test_the_window_is_at_least_sixty_seconds():
    """*** v2.0 S1.2 逐字: 统一去重窗口不少于 60 秒.

    MUTATION: 把 DEDUP_WINDOW_S 改小 -> 这里红.
    """
    from xbrain.p5_gateway.outbound.task_ack import DEDUP_WINDOW_S, DedupWindow

    assert DEDUP_WINDOW_S >= 60.0
    clock = _FakeClock()
    win = DedupWindow(clock=clock)
    win.seen("gj-001", "msg-1")
    clock.advance(59.0)
    assert win.seen("gj-001", "msg-1") is True, "59 秒内的重发没被认出来"


def test_a_narrower_window_is_refused_at_construction():
    """收窄窗口会让重复执行成为可能, 所以构造期就拒."""
    from xbrain.p5_gateway.outbound.task_ack import DedupWindow

    with pytest.raises(ValueError):
        DedupWindow(window_s=30.0)


def test_entries_expire_after_the_window():
    """窗口外的同一个 ID 是新请求 -- 否则窗口就是无限的, 内存也是."""
    from xbrain.p5_gateway.outbound.task_ack import DedupWindow

    clock = _FakeClock()
    win = DedupWindow(clock=clock)
    win.seen("gj-001", "msg-1")
    clock.advance(61.0)
    assert win.seen("gj-001", "msg-1") is False


def test_a_hit_does_not_refresh_the_timestamp():
    """*** 命中时不刷新时间戳.

    刷新会让一条被反复重发的消息永远留在窗口里 -- 窗口就不再是"最近
    60 秒"而是"最后一次重发起 60 秒", 而那与契约说的不是一回事.

    *** 本用例第一版把自己写错了, 值得记下来.
    第一版在循环里反复调 seen() 然后断言最后一次是 False. 那是错的: seen()
    在条目过期后会[重新登记]本次(它就该这样 -- 过期后的同一个 ID 确实是
    新请求), 于是循环末尾窗口里躺着的是最近一次登记, 断言自然不成立.
    实现没问题, 是测试搞错了 seen() 的语义.

    正确的问法只能问一次: 首次登记之后[不再碰它], 隔着几次别的 ID 的调用
    让时间走过窗口, 再问那一条.
    """
    from xbrain.p5_gateway.outbound.task_ack import DedupWindow

    clock = _FakeClock()
    win = DedupWindow(clock=clock)
    win.seen("gj-001", "msg-1")
    # 时间往前走, 期间只碰别的 ID -- 如果实现在别的 ID 上顺手刷新了
    # msg-1 的时间戳, 下面那条断言会红.
    for i in range(5):
        clock.advance(20.0)
        win.seen("gj-001", "other-%d" % i)
    # 从首次登记算起已过 100 秒 > 60 秒窗口 -> 必须已经过期.
    assert win.seen("gj-001", "msg-1") is False, (
        "首条在窗口过后仍被认为见过 -- 时间戳被别处刷新了")


def test_repeated_hits_do_not_extend_the_lifetime():
    """*** 直击"命中时刷新时间戳"这个变异体.

    上一条用例改写后只碰别的 ID, 于是"命中时刷新"这条实现改动它测不到 --
    变异体实测没红, 补这一条.

    做法: 在窗口内[反复命中同一条], 每次都在窗口内(不让它过期), 最后让
    时间从[首次登记]算起超过窗口. 刷新型实现会把到期时间一路往后推,
    于是最后那次仍判 True.

    MUTATION: 在 seen() 的命中分支里加 self._seen[key] = now -> 这里红.
    """
    from xbrain.p5_gateway.outbound.task_ack import DedupWindow

    clock = _FakeClock()
    win = DedupWindow(clock=clock)
    win.seen("gj-001", "msg-1")            # t=1000 登记
    for _ in range(4):
        clock.advance(15.0)                # 15/30/45/60 -- 都在窗口内附近
        win.seen("gj-001", "msg-1")        # 反复命中
    # 首次登记在 t=1000, 现在 t=1060 -> 已到窗口边界之外.
    clock.advance(1.0)                     # t=1061
    assert win.seen("gj-001", "msg-1") is False, (
        "反复命中把到期时间往后推了 -- 窗口变成滑动的, "
        "一条被持续重发的消息会永远占着窗口")


def test_no_entry_cap_evicts_inside_the_window():
    """*** v2.0 S1.2 逐字: 不得仅依赖条数上限提前淘汰窗口内的 ID.

    一个"只保留最近 100 条"的实现在高频下发时会把 30 秒前的 ID 挤掉,
    于是一条 40 秒后的重发被当成新请求 -- 而它满足"窗口不少于 60 秒"
    这句话的字面.

    MUTATION: 给 DedupWindow 加一个 maxlen 淘汰 -> 这里红.
    """
    from xbrain.p5_gateway.outbound.task_ack import DedupWindow

    clock = _FakeClock()
    win = DedupWindow(clock=clock)
    win.seen("gj-001", "msg-first")
    # 窗口内灌入大量别的 ID.
    for i in range(5000):
        win.seen("gj-001", "msg-flood-%d" % i)
    clock.advance(30.0)
    assert win.seen("gj-001", "msg-first") is True, (
        "窗口内的 ID 被条数上限挤掉了")


def test_dedup_is_partitioned_by_rid():
    """*** 多机场景: 两台机器人的同名 msg_id 互不影响.

    v2.0 S9.3 逐字"事件去重按 rid 分区". 不分区的话, A 的一条 msg-1 会让
    B 的 msg-1 被当成重复 -- B 那条指令就此消失, 而 Qt 收到的是 duplicate.
    """
    from xbrain.p5_gateway.outbound.task_ack import DedupWindow

    win = DedupWindow(clock=_FakeClock())
    win.seen("gj-001", "msg-1")
    assert win.seen("gj-002", "msg-1") is False, "两台机器人的去重串了"


def test_the_window_uses_a_monotonic_clock():
    """CLAUDE.md 3.4 / CLK-C1.

    墙钟在 NTP 阶跃时会往回跳, 窗口会忽然变长或变短 -- 变短的方向让
    重复执行成为可能.
    """
    import ast
    import inspect

    from xbrain.p5_gateway.outbound.task_ack import DedupWindow

    tree = ast.parse(inspect.getsource(DedupWindow))
    calls = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Attribute):
            base = node.value.id if isinstance(node.value, ast.Name) else ""
            calls.add("%s.%s" % (base, node.attr))
    assert "time.monotonic" in calls, "默认时钟不是单调钟"
    for banned in ("time.time", "datetime.now"):
        assert banned not in calls, "用了墙钟 %s" % banned


def test_duplicate_ack_is_accepted_with_zero_code():
    """幂等命中的 ack: accepted=true 且 error_code=0.

    原请求确实被受理过, 所以不是拒绝. Qt 据此知道"这条已经在处理了".
    """
    from xbrain.p5_gateway.outbound.task_ack import duplicate_ack

    body = duplicate_ack(msg_id="ack-2", ref_msg_id="msg-1",
                         task_id="task-1", task_type="GOTO_KEYPOINT")
    assert body["result"] == "duplicate"
    assert body["accepted"] is True
    assert body["error_code"] == 0
    assert body["ref_msg_id"] == "msg-1"
