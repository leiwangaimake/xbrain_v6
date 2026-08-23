"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_motion_intent_gates.py
Brief: P2 cmd/motion/intent -- G-1..G-11 十一道闸门与轴换算 (11 S9.3.2A)

Description:
守 A 类语音运动原语的落点. 这个接收端在 2026-08-21 之前不存在: 11 S2.2.3 把
p2_core 列为 cmd/motion/intent 的订阅者, 而真跑的 main_wiring 从未订过, 于是
A 类 14 条意图发出去无人接收, 两侧都不报错.

每条断言配一个必然违反它的变异体(CLAUDE.md 3.3). 三个最值得点名的, 都是
契约自己点了名的坑:

  * G-10 软急停缴械[不拦]-- 转发本身就是 re-arm 的钥匙. 写成一道检查会把
    U35 的现场表现("喊急停 -> 停住 -> 再喊前进两米 -> 立刻走")堵死在 P2;
  * face_heading 绝不能在 P2 折算成 dyaw_rad -- 折算要减[当前]yaw, 而 P2
    只有 10 Hz 快照, 语音链路 0.5-2 s, 转完必然偏;
  * turn_around 固定取正 -- +-pi 路程等长, 取定值是为了可重放.

Boundaries: 不测 P1 的速度门/围栏/RNS(那些在 P1), 只测"闸门 + 换算"这一跳.
"""
from __future__ import annotations

import math

import pytest

from xbrain.common.errors import (
    E_BUSY, E_CAPABILITY, E_LOCKED, E_NO_HEADING, E_SCHEMA, E_UNHEALTHY,
)
from xbrain.p2_core.runtime.motion_intent_wiring import (
    MotionLimits, evaluate, parse_intent_envelope, to_relative_move,
)

pytestmark = pytest.mark.no_device

_LIMITS = MotionLimits(max_distance_m=20.0, max_angle_deg=720.0)
_CLOCK_OK = {"ts_sync": True}


def _cmd(**kw):
    base = {"cmd_id": "mi-1", "turn_id": "vt-1", "channel": "mic_local",
            "auth_level": "L1", "intent": "move_forward",
            "slots": {"distance_m": 3.0}}
    base.update(kw)
    return base


def _ev(cmd=None, **kw):
    kw.setdefault("clock", _CLOCK_OK)
    return evaluate(cmd or _cmd(), limits=_LIMITS, **kw)


# -- G-1 信封 ------------------------------------------------------------

def test_envelope_without_data_is_refused():
    """*** G-1: 信封完整校验[不豁免].

    相对位移是放松型指令, 解析失败按"放行"兜底是危险的(同 S9.3.4 E-3).
    变异体: 拿不到 data 就退化成裸本体 -- 那是 cmd/mode 的取舍, 放在这条
    会让机器人动的 key 上就不对.
    """
    with pytest.raises(ValueError):
        parse_intent_envelope({"v": 1, "src": "p4_agent"})
    assert parse_intent_envelope({"data": {"intent": "x"}}) == {"intent": "x"}


# -- G-2 闭集与必填 ------------------------------------------------------

def test_intent_outside_the_eight_value_set_is_refused():
    v = _ev(_cmd(intent="moonwalk"))
    assert not v.passed and v.code == E_SCHEMA and v.detail["field"] == "intent"


def test_auth_level_other_than_l1_is_refused():
    """A05-A12 恒 L1(18 S13.1). 别的值即 P4 缺陷. """
    v = _ev(_cmd(auth_level="L0"))
    assert not v.passed and v.detail["field"] == "auth_level"


@pytest.mark.parametrize("dist", [0, -3.0, "3", None])
def test_non_positive_or_non_numeric_distance_is_refused(dist):
    """<= 0 也是 E_SCHEMA: 方向由 intent 决定, 不进 slots, 所以负数不是
    "反方向"而是 P4 缺陷. 字符串一并拒: float("3") 会成功, 一个先转换再
    判定的实现会放它过去. """
    v = _ev(_cmd(slots={"distance_m": dist}))
    assert not v.passed and v.code == E_SCHEMA


def test_boolean_distance_is_refused():
    """*** True 是 int, 会被当成 1.0 米走出去.

    变异体: 去掉 isinstance(v, bool) 排除 -- 这条即红.
    """
    assert not _ev(_cmd(slots={"distance_m": True})).passed


def test_heading_outside_the_eight_compass_points_is_refused():
    v = _ev(_cmd(intent="face_heading", slots={"heading": "up"}))
    assert not v.passed and v.detail["field"] == "slots.heading"


# -- G-3 量程 ------------------------------------------------------------

def test_distance_over_the_ceiling_is_refused_with_field_and_limit():
    """*** detail 必须同时带 field 与 limit(12 S4.5.5 样例逐字).

    变异体: 只回 field 不回 limit -> 操作员知道超限却不知道上限是多少,
    只能试.
    """
    v = _ev(_cmd(slots={"distance_m": 25.0}))
    assert not v.passed and v.code == E_SCHEMA
    assert v.detail["field"] == "slots.distance_m"
    assert v.detail["limit"] == 20.0


def test_the_ceiling_is_injected_not_defaulted():
    """*** CLAUDE.md 3.1: 上限没有代码默认值.

    这两个数决定"一句话最多能让机器人走多远", 带默认值就是没人看得见也改
    不动. 变异体: 给 MotionLimits 的字段加默认值 -> 这条构造即不再报错.
    """
    with pytest.raises(TypeError):
        MotionLimits()                       # type: ignore[call-arg]


def test_angle_over_the_ceiling_is_refused():
    v = _ev(_cmd(intent="turn_left", slots={"angle_deg": 900.0}))
    assert not v.passed and v.detail["limit"] == 720.0


# -- G-4 时钟 ------------------------------------------------------------

def test_clock_not_synced_refuses_with_unhealthy_clock():
    """S1.5.5 把相对位移明列在时钟未同步时禁止的动作里. """
    v = _ev(clock={"ts_sync": False})
    assert not v.passed and v.code == E_UNHEALTHY and v.detail["item"] == "clock"


def test_missing_clock_state_is_treated_as_not_synced():
    """*** 没收到 state/clock 取[保守侧].

    这一门的失效方向是"时钟没同步却放行", 后果是位移与录包时间对不上.
    变异体: clock is None 时放行 -> 冷启动那几秒任何位移都能过.
    """
    assert not _ev(clock=None).passed


# -- G-5 健康度 ----------------------------------------------------------

#: 真实的 health/summary 单项形状, 逐字取自 2026-08-23 ORIN 上抓到的报文.
#: *** 这个夹具是照抄下来的, 不是编的. 第一版夹具写的是 {"level": "unknown"},
#: 而实现里查的也是 level in ("fault", "unknown") -- 两边共享同一个错误词表,
#: 于是测试全绿而 ORIN 上 8 项全 fatal 却报 item: null. 11 S5.1 的真实闭集是
#: state in {ok, warn, degraded, fail, unknown} 与 level in {fatal, degraded,
#: warn}, "fault" 这个词全库不存在.
def _item(state, level, kind="device"):
    return {"state": state, "level": level, "kind": kind, "detail": ""}


def test_allow_motion_false_names_the_failing_item():
    """detail.item 要点名具体项, 不说"机器人忙" -- 操作员据此才知道修哪个. """
    v = _ev(health={"allow_motion": False,
                    "items": {"mic": _item("ok", "warn"),
                              "cam_rgbd": _item("unknown", "fatal")}})
    assert not v.passed and v.code == E_UNHEALTHY
    assert v.detail["item"] == "cam_rgbd"


def test_a_fatal_item_is_named_ahead_of_a_merely_degraded_one():
    """*** 停机时通常好几项同时不 ok(没接底盘就是 battery/chassis/cam_rgbd
    一起 unknown), 报哪个都"对", 但只有 fatal 那项是修了才能动的.

    变异体: 返回字典里的第一个不 ok 项 -> 这里会报 disk, 操作员清完磁盘发现
    还是走不了.
    """
    v = _ev(health={"allow_motion": False,
                    "items": {"disk": _item("degraded", "warn", "cap"),
                              "chassis": _item("unknown", "fatal")}})
    assert v.detail["item"] == "chassis"


def test_a_failing_item_is_never_fabricated():
    """*** 找不到失败项就回 None, NO 不编一个.

    报一个其实是好的项, 操作员会去修一个没坏的东西.
    变异体: 找不到时回 "unknown" 或第一个项名 -> 这条即红.
    """
    v = _ev(health={"allow_motion": False,
                    "items": {"mic": _item("ok", "warn")}})
    assert not v.passed and v.detail["item"] is None


# -- G-6 航向 ------------------------------------------------------------

@pytest.mark.parametrize("intent,slots", [
    ("turn_left", {"angle_deg": 90.0}),
    ("turn_around", {}),
    ("face_heading", {"heading": "north"}),
    ("move_left", {"distance_m": 1.0}),
])
def test_yaw_incapable_refuses_the_whole_command(intent, slots):
    """*** 整条拒绝, NO 不做"只执行平移部分"的部分降级(G-6 逐字).

    move_left 也在里面: 横移会改变对航向的依赖, 不是纯平移.
    """
    v = _ev(_cmd(intent=intent, slots=slots), pose={"yaw_capable": False})
    assert not v.passed and v.code == E_NO_HEADING


def test_forward_move_is_unaffected_by_yaw_capability():
    """直行不需要航向 -- 否则 G-6 会把恢复航向的唯一手段(直行五米)也拦掉,
    操作员就永远出不来了. """
    assert _ev(pose={"yaw_capable": False}).passed


# -- G-7 本体能力 --------------------------------------------------------

def test_sideways_move_refused_when_not_holonomic():
    v = _ev(_cmd(intent="move_left", slots={"distance_m": 1.0}),
            pose={"yaw_capable": True}, holonomic=False)
    assert not v.passed and v.code == E_CAPABILITY


def test_sideways_move_refused_when_holonomic_unknown():
    """*** None 与 False 同样拒绝.

    一台"没查到规格"的车不该被试着横着走. 变异体: `holonomic is False` 这种
    写法会让 None 放行.
    """
    v = _ev(_cmd(intent="move_left", slots={"distance_m": 1.0}),
            pose={"yaw_capable": True}, holonomic=None)
    assert not v.passed and v.code == E_CAPABILITY


# -- G-8 录制态 ----------------------------------------------------------

@pytest.mark.parametrize("state", ["recording", "pending_name",
                                   "pending_overwrite"])
def test_teach_session_blocks_motion(state):
    """*** U45: 录制态只有键盘/手柄能动, 这道门必须在 P2 拦.

    12 S4.7.2 只是把 relative_move 源[抑制], 指令仍会 ACCEPTED -> RUNNING
    然后空转到 T-12 的 20 s 超时才报 aborted -- 用户要等 20 秒才知道没动.
    """
    v = _ev(teach={"state": state})
    assert not v.passed and v.code == E_BUSY
    assert v.detail["reason"] == "teach_session_active"


# -- G-9 真锁 ------------------------------------------------------------

@pytest.mark.parametrize("field,reason", [("hes_lock", "hes_engaged"),
                                          ("timeout_lock", "cmd_timeout")])
def test_hard_locks_refuse_with_the_unlock_path(field, reason):
    """给出解锁路径(reason), 不要只说"被锁了". """
    v = _ev(robot={field: True})
    assert not v.passed and v.code == E_LOCKED and v.detail["reason"] == reason


# -- G-10 软急停缴械: 不拦 ----------------------------------------------

def test_soft_estop_disarm_does_NOT_block_forwarding():
    """*** 契约点名的最容易写错的一条.

    S7A.6.3 的 re-arm 触发表逐字写着"新的 cmd/motion/relative_move"--
    P2 转发这个动作[本身]就是解除软急停缴械的钥匙. 若看到
    suspended == "soft_estop" 就先回 E_ARB_DISARMED, U35 的现场表现
    ("喊急停 -> 停住 -> 再喊前进两米 -> 立刻走")被堵死在 P2, 而
    E_ARB_DISARMED 给客户端的指引"等一条真正的新运动指令"将永远等不到.

    MUTATION: 加一道 if suspended == "soft_estop": return E_ARB_DISARMED
    -- 这条即红. 真锁预拒(G-9), 缴械放行(G-10).
    """
    v = _ev(robot={"suspended": "soft_estop"})
    assert v.passed


def test_soft_estop_and_hard_lock_are_different_doors():
    """缴械放行, 真锁预拒 -- 同一帧里两者都在时, 真锁赢. """
    v = _ev(robot={"suspended": "soft_estop", "hes_lock": True})
    assert not v.passed and v.code == E_LOCKED


# -- G-11 定位降级: 不拦 -------------------------------------------------

def test_rtk_float_does_not_block():
    """S3.2.1 的 E_DEGRADED 限定"不接受新任务", 而本节裁决相对位移不是任务. """
    assert _ev(pose={"yaw_capable": True, "fix_type": "rtk_float"}).passed


# -- 轴换算 (S9.3.2A.4) --------------------------------------------------

@pytest.mark.parametrize("intent,axis,sign", [
    ("move_forward", "dx_m", +1), ("move_backward", "dx_m", -1),
    ("move_left", "dy_m", +1), ("move_right", "dy_m", -1),
])
def test_translation_axis_and_sign(intent, axis, sign):
    """方向由 intent 决定(slots 只填正数), 符号在 P2 加. """
    body = to_relative_move(_cmd(intent=intent, slots={"distance_m": 2.0}),
                            rm_cmd_id="rm-1", params={})
    assert body[axis] == pytest.approx(2.0 * sign)


@pytest.mark.parametrize("intent,sign", [("turn_left", +1), ("turn_right", -1)])
def test_rotation_converts_degrees_to_radians(intent, sign):
    body = to_relative_move(_cmd(intent=intent, slots={"angle_deg": 90.0}),
                            rm_cmd_id="rm-1", params={})
    assert body["dyaw_rad"] == pytest.approx(sign * math.pi / 2)


def test_turn_around_is_always_positive():
    """*** A11 固定取正(逆时针).

    +-pi 路程等长, 取定值是为了[可重放]: 同一句话两次执行必须转同一个
    方向, 否则事故复盘还原不出来. 变异体: 取负, 或按当前 yaw 选近路.
    """
    body = to_relative_move(_cmd(intent="turn_around", slots={}),
                            rm_cmd_id="rm-1", params={})
    assert body["dyaw_rad"] == pytest.approx(math.pi)


@pytest.mark.parametrize("heading,rad", [
    ("east", 0.0), ("north", math.pi / 2), ("west", math.pi),
    ("south", -math.pi / 2), ("northeast", math.pi / 4),
    ("southwest", -3 * math.pi / 4),
])
def test_face_heading_sets_target_yaw_and_leaves_dyaw_zero(heading, rad):
    """*** MO-3: 置 target_yaw_rad 并把 dyaw_rad 置 0, 二者互斥.

    绝不在 P2 折算成 dyaw_rad: 绝对航向要减[当前]yaw, 而 P2 只有 10 Hz
    快照, 语音链路 0.5-2 s, 机器人若在动折算出来的一定是过期量, 转完必然偏.
    折算权留在 P1(它在 ACCEPTED 那一拍用最新 yaw 解算).

    MUTATION: 在这里算 dyaw_rad = target - current_yaw -- 这条即红.
    """
    body = to_relative_move(
        _cmd(intent="face_heading", slots={"heading": heading}),
        rm_cmd_id="rm-1", params={})
    assert body["target_yaw_rad"] == pytest.approx(rad)
    assert body["dyaw_rad"] == 0.0


# -- MO-1 / MO-2 ---------------------------------------------------------

def test_forwarded_command_uses_a_new_cmd_id():
    """*** MO-1: 转发必须换新 id, NO 不复用 mi- 那个.

    两条 key 各自按 S2.3 去重, 共用 id 会让"P4 重发的意图"与"P2 重发的位移"
    互相误判为重复.
    """
    body = to_relative_move(_cmd(), rm_cmd_id="rm-abc", params={})
    assert body["cmd_id"] == "rm-abc" and body["cmd_id"] != "mi-1"


@pytest.mark.parametrize("channel,source", [
    ("mic_local", "voice_local"), ("cloud", "voice_cloud"),
    ("wecom", "voice_wecom"), ("hmi", "text"),
])
def test_channel_maps_to_source(channel, source):
    """MO-2 的映射表. """
    body = to_relative_move(_cmd(channel=channel), rm_cmd_id="rm-1", params={})
    assert body["source"] == source


def test_turn_id_is_threaded_through():
    """唯一的审计缝合点: 没有它, state/voice_turn 与本条指令事后对不上,
    "模型说了什么 -> 机器人做了什么"这条链断在 P4 出口. """
    assert to_relative_move(_cmd(), rm_cmd_id="rm-1",
                            params={})["turn_id"] == "vt-1"


def test_motion_params_come_from_the_caller_not_the_frame():
    """*** MO-2: 运动参数一律 P2 按配置填, NO 不接受 P4 提供.

    变异体: 从 cmd 里读 max_speed_mps -- 于是语音侧能自己抬速度上限.
    这里给帧里塞一个假的, 断言它没被采纳.
    """
    frame = _cmd()
    frame["max_speed_mps"] = 99.0                # P4 伪造的抬速
    body = to_relative_move(frame, rm_cmd_id="rm-1",
                            params={"max_speed_mps": 0.5,
                                    "abort_on_obstacle": True})
    assert body["max_speed_mps"] == 0.5
    # 语音来源恒 true(12 S4.5.6: 说前进 1 米但前方有人, 应停下并告知,
    # 而不是绕过去).
    assert body["abort_on_obstacle"] is True


def test_asr_text_never_reaches_the_forwarded_command():
    """*** MI-1 字段表逐字: asr_text 禁止参与判定.

    一旦它进了下游, 就有人会拿它做正则兜底, GBNF 的闭集约束随即被绕过.
    """
    body = to_relative_move(_cmd(asr_text="前进三米"), rm_cmd_id="rm-1",
                            params={})
    assert "asr_text" not in body


# -- 元测试 --------------------------------------------------------------

def test_every_intent_in_the_closed_set_converts():
    """反向差集: 八个意图都要能换算出一条 relative_move, 不能有"认得但
    换不出来"的成员. """
    from xbrain.p2_core.runtime.motion_intent_wiring import _INTENT_AXIS
    slots_for = {"distance_m": 1.0, "angle_deg": 45.0, "heading": "north"}
    for intent in _INTENT_AXIS:
        body = to_relative_move(_cmd(intent=intent, slots=slots_for),
                                rm_cmd_id="rm-x", params={})
        moved = (body["dx_m"] or body["dy_m"] or body["dyaw_rad"]
                 or body.get("target_yaw_rad"))
        assert moved, f"{intent} converted to a no-op"
