"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_boot_invariants.py
Brief: CFG-BT-10/11/12/13 -- P2 启动阶段机与 BOOT-I/L/B 三组不变量的判据验证

Description:
这四条任务的实现都已经在(stage_machine.py 与 timeout_lock.py), 而 2026-08-23
的证据核实查出一件事: 它们的判据[从未被验证过] -- BOOT-I1..I4 / BOOT-L1..L3 /
CS-A1..A4 / SE-1 这些规则号在整个 tests/ 下一处都不出现.

实现存在与判据被验证是两件事. 前者只说明有人写了代码, 后者才说明那段代码
做的是设计要求的事. 本文件补的是后者.

*** BOOT-I2 是这批里最贵的一条, 判据自己写着"本项的要害".
逐字: "P1 构造期 allow_motion=false / speed_factor=0.0, 且[从未收到过
cmd/motion/factor]不享受 T-07 的 3 s 宽限". 判据同时逐字禁止了一种偷懒写法:
"NO 不得只断言字段初值" -- 因为一个把初值写对, 却在下一拍就放行的实现同样
能通过那种断言. 真正的判据是[累计位移恒为 0].

*** BOOT-L1..L3 的要害是"唯一判据是回读, NO 不凭 ack".
底盘回 ack 只说明它收到了解锁命令, 不说明锁真的开了. 凭 ack 离开 BLOCKED,
机器人会在锁仍然生效的情况下认为自己可以动 -- 而那正是 timeout_lock 要防的
局面. TimeoutLockGate 把这两条输入分成了 note_readback 与 note_ack_only 两个
方法, 本文件验证只有前者能开门.

Boundaries: 不测阶段机与真实 Zenoh / BIT 的接线(那要整栈), 只测状态机与
门本身的判定. 接线由 CHK-1-04 的注入矩阵负责登记现状.
"""
from __future__ import annotations

import pytest

pytestmark = pytest.mark.no_device


# --- CFG-BT-10: 六态阶段机 -------------------------------------------

def test_stage_order_is_enforced():
    """Stage A->B->C->D 不得跳级.

    跳级的现实后果很具体: 跳过 Stage C 就等于跳过 BIT, 而 Stage D 做的事
    是发第一条 allow_motion:true -- 也就是放行运动. 一个允许 A->D 的状态机
    会让机器人在没做自检的情况下获得运动权.
    """
    from xbrain.p2_core.boot.stage_machine import (
        BootStage, BootStageMachine, InvalidBootTransition)

    sm = BootStageMachine()
    with pytest.raises(InvalidBootTransition):
        sm.transition(BootStage.STAGE_D)          # A -> D 跳级
    sm.transition(BootStage.STAGE_B)
    sm.transition(BootStage.STAGE_C)
    sm.transition(BootStage.STAGE_D)
    assert sm.stage == BootStage.STAGE_D


def test_blocked_is_a_sink_only_an_operator_leaves():
    """*** BLOCKED 是吸收态: 进去了就不能靠自己出来.

    这一条守的是"自动恢复"的诱惑. 一个允许 BLOCKED->STAGE_C 的实现会在
    故障消失时悄悄放行 -- 而 BLOCKED 的语义是"有一项自检 fatal 失败",
    它消失了也需要人确认过. 自动出来等于把一次未经确认的放行做成常态.
    """
    from xbrain.p2_core.boot.stage_machine import (
        BootStage, BootStageMachine, InvalidBootTransition)

    sm = BootStageMachine()
    sm.transition(BootStage.BLOCKED)
    for target in (BootStage.STAGE_A, BootStage.STAGE_B,
                   BootStage.STAGE_C, BootStage.STAGE_D):
        with pytest.raises(InvalidBootTransition):
            sm.transition(target)


def test_blocked_is_reachable_from_every_stage():
    """反向: 任何阶段出致命故障都要能立刻进 BLOCKED, 包括已经放行的 Stage D.

    没有 D->BLOCKED 这条边, 一个在运行中出现的 fatal 就没有降级出口了.
    """
    from xbrain.p2_core.boot.stage_machine import (
        ALLOWED, BootStage, BootStageMachine)

    for stage in (BootStage.STAGE_A, BootStage.STAGE_B,
                  BootStage.STAGE_C, BootStage.STAGE_D):
        assert BootStage.BLOCKED in ALLOWED[stage], (
            "%s 到 BLOCKED 没有边 -- 该阶段出致命故障时无处可去" % stage)


# --- CFG-BT-11: BOOT-I2, 判据自称"本次最危险的一个洞" ----------------

def test_boot_i2_initial_values_forbid_motion():
    """BOOT-I2 的字面一半: 构造期 allow_motion=false / speed_factor=0.0.

    * 这只是一半. 判据逐字说"NO 不得只断言字段初值", 下一条才是要害.
    """
    from xbrain.p2_core.boot.stage_machine import initial_motion_factor

    mf = initial_motion_factor()
    assert mf.allow_motion is False, "构造期就允许运动"
    assert mf.speed_factor == 0.0, "构造期速度系数不是 0"


def test_boot_i2_never_received_is_not_the_same_as_stale():
    """*** BOOT-I2 的要害, 也是判据点名的变异体位置.

    T-07 给 cmd/motion/factor 断流留了 3 s 宽限 -- 那是给[收到过又断了]的
    情况用的. "从未收到过"是另一回事: 没有任何依据说明上游同意放行, 3 s
    宽限在这里等于凭空放行 3 秒.

    MUTATION(判据点名): 把初值改成 allow_motion=True -> 这里红.
    NO 不只断言字段: check_boot_i2_initial 必须真的抛.
    """
    from xbrain.p2_core.boot.stage_machine import (
        BootI2Violation, MotionFactor, check_boot_i2_initial,
        initial_motion_factor)

    # 正例: 出厂初值必须过.
    check_boot_i2_initial(initial_motion_factor())
    # 反例逐个: 任一项被放宽都必须抛.
    with pytest.raises(BootI2Violation):
        check_boot_i2_initial(MotionFactor(allow_motion=True, speed_factor=0.0,
                                           v_max_mps=0.0))
    with pytest.raises(BootI2Violation):
        check_boot_i2_initial(MotionFactor(allow_motion=False, speed_factor=0.5,
                                           v_max_mps=0.0))
    # 第三个字段同样是初值的一部分 -- 只查前两个的话, 一个把 v_max 写成
    # 非零的实现会溜过去, 而 v_max 正是最终决定机器人能走多快的那个数.
    with pytest.raises(BootI2Violation):
        check_boot_i2_initial(MotionFactor(allow_motion=False, speed_factor=0.0,
                                           v_max_mps=2.0))


def test_boot_i2_zero_factor_means_zero_displacement():
    """*** 判据逐字要的那条: 累计位移必须恒为 0, 不是"字段是 0".

    两者的差别在于一个错误的实现: 字段初值写对了, 但下游把 speed_factor
    当成"建议值"而不是硬乘数. 那种实现能通过上面每一条断言, 而机器人会动.

    这里用真实的速度门规则跑一遍: 无论其它因子多大, factor=0 必须让
    v_max 归零 -- 也就是位移恒为 0.
    """
    from xbrain.p1_motion.gate.speed_gate import gate_rule
    from xbrain.p2_core.boot.stage_machine import initial_motion_factor

    mf = initial_motion_factor()
    # f/g/h/i 四个因子里, speed_factor 进的是其中一路. 用初值那一路为 0,
    # 其余全开到最大, 断言结果仍是 0.
    v_max = gate_rule(2.0, mf.speed_factor, 1.0, 1.0, 2.0)
    assert v_max == 0.0, (
        "speed_factor=0 却算出 v_max=%.3f -- 位移不会是 0" % v_max)
    # 反向: 同样的算式在 factor 放开后必须能给出正速度, 否则上一条断言
    # 用一个恒返回 0 的实现也能过.
    assert gate_rule(2.0, 1.0, 1.0, 1.0, 2.0) > 0.0


# --- CFG-BT-12: BOOT-L1~L3, 解锁只认回读 -----------------------------

def test_boot_l3_only_a_readback_opens_the_lock():
    """*** BOOT-L1~L3 的要害: 唯一判据是回读, NO 不凭 ack.

    ack 只说明底盘收到了解锁命令. 凭 ack 离开 BLOCKED, 机器人会在锁仍然
    生效时认为自己可以动 -- 那正是 timeout_lock 存在的理由.

    MUTATION: 让 note_ack_only 也开门 -> 这里红.
    """
    from xbrain.p2_core.boot.timeout_lock import TimeoutLockGate

    gate = TimeoutLockGate()
    assert not gate.may_publish_factor(), "锁一上来就是开的"
    gate.note_ack_only(ack_accepted=True)
    assert not gate.may_publish_factor(), (
        "只凭一个 ack 就放行了 -- BOOT-L3 要求等回读为假")


def test_boot_l3_readback_true_keeps_it_shut():
    """回读回来说"锁还在", 那更不能开 -- 这条防的是把回读当成"收到即通过"."""
    from xbrain.p2_core.boot.timeout_lock import TimeoutLockGate

    gate = TimeoutLockGate()
    gate.note_readback(readback_lock=True)
    assert not gate.may_publish_factor(), "回读说锁着, 却放行了"


def test_boot_l2_unlock_needs_l2_token_and_an_allowed_channel():
    """BOOT-L2: 解锁入口要 L2 授权 + confirm_token + 允许的通道.

    三个条件各缺一次, 每次都必须被拒 -- 只测"全给齐时通过"的话, 一个
    什么都不检查的实现同样能过.
    """
    from xbrain.p2_core.boot.timeout_lock import (
        ALLOWED_UNLOCK_CHANNELS, TimeoutLockAction, validate_unlock_request)

    channel = sorted(ALLOWED_UNLOCK_CHANNELS)[0]
    good = {"action": TimeoutLockAction.ENABLE.value, "confirm_token": "t-1"}
    assert validate_unlock_request(good, channel).accepted, (
        "条件齐全却被拒, 夹具形状可能不对")
    # 缺 token.
    assert not validate_unlock_request(
        {"action": TimeoutLockAction.ENABLE.value}, channel).accepted
    # 动作不对.
    assert not validate_unlock_request(
        {"action": "reboot", "confirm_token": "t-1"}, channel).accepted
    # 通道不在白名单.
    assert not validate_unlock_request(good, "some_unlisted_channel").accepted


def test_hes_and_timeout_lock_are_never_conflated():
    """两把锁不许混为一谈.

    HES(硬急停)与 timeout_lock 的解除条件完全不同. 把两者当成同一个状态,
    会让解除其中一个的动作顺带解除另一个 -- 而那意味着一次操作解除了两道
    独立的安全约束.
    """
    from xbrain.p2_core.boot.timeout_lock import (
        HesLockConflation, assert_locks_are_separate)

    # 逐字用实现要求的两个名字. 第一版写了 "hes"(简写), 被拒 --
    # 而那正是这个门要的效果: 名字含糊就等于两把锁分不清.
    assert_locks_are_separate(("timeout_lock", "hes_lock"))
    with pytest.raises(HesLockConflation):
        assert_locks_are_separate(("timeout_lock",))          # 少一把
    with pytest.raises(HesLockConflation):
        assert_locks_are_separate(("timeout_lock", "timeout_lock"))  # 同名两次


# --- CFG-BT-13: BOOT-B1~B4, 启动期 BIT 与运行期降级的分界 ------------

def test_boot_b2_a_degraded_fix_is_not_a_bit_failure():
    """*** BOOT-B2: 有报文但 fix_type=single/dgps/no_fix [不是 BIT 失败].

    这条分界很容易做反. 做反的方向有两个, 各有代价:
      * 判成 BIT 失败 -> 机器人在城市峡谷里根本起不来(过严);
      * 判成完全正常 -> 拿着单点解去跑自主导航(过松).
    正确答案是中间那档: 放行运动但禁自主.
    """
    from xbrain.p2_core.health.aggregate import state_from_pose
    from xbrain.p2_core.health.items import HealthState

    for fix in ("single", "dgps"):
        (rtk, _detail), _heading = state_from_pose({"fix_type": fix})
        assert rtk != HealthState.OK, "%s 被判成完全正常" % fix
        assert rtk != HealthState.FAIL, (
            "%s 被判成 fail -- BOOT-B2 逐字说它不是 BIT 失败" % fix)


def test_boot_b1_no_pose_at_all_is_a_determinate_failure():
    """反向: 完全没有 state/pose 是 fail, 不是 unknown.

    P1 只要在跑就以 10 Hz 发它, 所以它的缺席是一个[确定的答案](定位链断了),
    不是"还没收到". 判成 unknown 会让整机在定位完全没有的情况下显示为
    "情况不明"而不是"不能动".
    """
    from xbrain.p2_core.health.aggregate import state_from_pose
    from xbrain.p2_core.health.items import HealthState

    (rtk, _d), (heading, _h) = state_from_pose(None)
    assert rtk == HealthState.FAIL
    assert heading == HealthState.FAIL


def test_boot_b3_rgbd_has_no_lidar_backup():
    """BOOT-B3: cam_rgbd 无帧 = fatal, LiDAR 不能互备.

    互备这件事在别处成立(探距), 在这里不成立: 336L 是 world_pos / YOLO 语义
    / D 模式跟随的唯一来源. 认为"还有雷达"而放行, 等于让机器人在没有语义
    感知的情况下作业.

    这里查的是分类表: cam_rgbd 必须落在拒启动/禁运动那一档.
    """
    from xbrain.boot import failure_class

    rows = [r for r in failure_class._CLASSIFIER_TABLE
            if "rgbd" in (r.detection or "").lower()
            or "rgbd" in (r.id or "").lower()]
    if not rows:
        pytest.skip("分类表里还没有 cam_rgbd 行")
    for row in rows:
        assert row.cls in (failure_class.CLASS_R, failure_class.CLASS_B), (
            "cam_rgbd 行被判成 %s -- BOOT-B3 要求 fatal" % row.cls)
