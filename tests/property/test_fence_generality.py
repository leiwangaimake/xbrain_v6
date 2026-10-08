"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_fence_generality.py
Brief: 围栏链对[任意客户录制数据]的一般性 (11 S9A.2 / S9A.7)

Description:
本文件解决的问题: 围栏是客户在自己落地场景现场录制的数据 -- 场地在哪, 围几个,
每个几个顶点, 什么形状, 事前都无法预知, 而且随时可能换一批. 所以 fence.db ->
p3 广播 -> p1 自算比对 -> 持有 的整条链必须对[任意]合法输入成立, 而不是对本机
库里那一批恰好成立. 本机 data/run/fence.db 里的行是开发期的模拟数据, NO 它的
任何值都不得当作落地参数或判据依据(用户 2026-10-08 明令).

本文件用属性测试(hypothesis, 同 tests/property/test_arbiter_invariants.py 的
范式)把"一般性"写成断言, 而不是用一两个手挑的例子声称它:
  * 围栏数量 1..5(FV-3 上限), role 任意组合但恰好一个 allow(FS-5A);
  * 每个环 3..40 顶点, 顶点坐标覆盖南北半球 / 东西经 / 近赤道与中高纬;
  * priority 任意 int32 区间值, 限速值任意正数;
  * 场地原点与围栏顶点一起漂移(FV-7 的 20 km 内), 即"换一个落地场景".

断言的是链路不变量, 不是具体数值:
  (1) p3 构造的 FenceSet 一定过 p1 的自算比对(FV-8) -- 两侧配方同源;
  (2) p1 持有态逐多边形与输入 role/顶点数一致, 且 hard_enforce 按 role 派生;
  (3) ENU 编译只拿 allow/forbid 当边界, 且编译结果的顶点数与输入一致;
  (4) 带符号距离对每个编译多边形都给出有限值, 内外侧符号与 keep_in 相符 --
      凹多边形同样成立(射线法与边距离在凹处最容易打架).

*** 它[不]做什么: 不验具体距离数值(那是 tests/p1_motion/fence/ 的几何用例,
用手算过的固定几何), 不验限速施加(那在速度门那一批), 不碰 data/run/fence.db
(本文件一个字节都不读它 -- 读了就等于把模拟数据织进判据).

*** 看起来对但会出错的写法:
  * 用 hypothesis 生成[自交]多边形然后断言距离有意义. 自交环在 FV-2 就被拒,
    契约不保证它有内外之分; 这里用"顶点按极角排序"构造简单多边形, 并显式允许
    半径抖动产生凹形 -- 凹是必须覆盖的, 自交不是.
  * 让原点固定在某个城市而只抖动顶点. 那只测了"同一个场地的不同围栏", 测不到
    "换一个场地" -- 而 enu_origin 是整条链上唯一的场地相关量.
"""
from __future__ import annotations

import json
import math

import pytest
from hypothesis import HealthCheck, given, settings
from hypothesis import strategies as st

from xbrain.common.fence.roles import hard_enforce_for_role
from xbrain.p1_motion.fence.clip import CONSTRAINT_ROLES, boundary_hit, compile_fence
from xbrain.p1_motion.fence.fence_set import compile_fence_set
from xbrain.p1_motion.path.local_frame import LocalFrame
from xbrain.p3_task.fence.fence_set import build_fence_set

pytestmark = pytest.mark.no_device

# ---------------------------------------------------------------------------
# *** 红验记录(3.3: 每条断言必须红过一次才算写完). 2026-10-08 对本文件注入的
# 变异体, 6 个全部被杀(每次跑前清 __pycache__, 7.2.1):
#   compile_fence 的 role 过滤改成恒 False(收 warning/speed_limit 当边界) -> 杀
#   compile_fence 对任何集都返回空(边界全丢)                              -> 杀
#   keep_in 恒 True(禁入区当活动区)                                      -> 杀
#   d_nom 丢掉 keep_in 的符号                                            -> 杀
#   p1 持有态 hard_enforce 回"只认 warning"                              -> 杀
#   p3 wire 的 priority 恒 0(不带出)                                     -> 杀
#
# 这些变异体不是穷举, 是按 [这条断言如果写错会怎么错] 逐条挑的 --
# 一个覆盖不到任何断言的变异体(例如改一个日志串)不说明任何事.
# *** 其中 "keep_in 恒 True" 第一轮[存活], 原因值得记住: 当时的断言写成
#   if p.keep_in: assert d_nom > 0 else: assert d_nom < 0
# 它读[被测量本身]来决定期望, 于是 keep_in 恒 True 时两边都自洽 --
# 把结论定义进了前提(3.2 形态 7). 改成由 role 推期望后才被杀.
# ==> 教训: 属性测试里期望值必须从[输入]推, 不能从[产出]推.
# ---------------------------------------------------------------------------
# *** 这组用例[证明不了]什么 -- 写出来, 以免它的通过被读成比实际更多.
#
# 1. 它证明的是"链路不变量对任意合法输入成立", 不是"几何算对了".
#    举例: 若 boundary_hit 把所有距离都乘 2, 本文件全绿 -- 符号没变, 有限性
#    没变, 计数没变. 数值的正确性只能由[手算过的固定几何]来钉, 那在
#    tests/p1_motion/fence/test_clip_geometry.py. 两者是互补的, NO 不可互相
#    替代: 属性测试覆盖"输入空间", 固定几何覆盖"数值".
#
# 2. 它只覆盖[简单(不自交)]多边形. 自交环在 FV-2 就被拒, 契约不保证它有
#    内外之分, 所以这里刻意不生成. 这不是偷懒 -- 对一个自交环断言"心在内"
#    本身是无定义的.
#
# 3. 它不覆盖[退化]几何: 零面积环(FV-5 面积下限管它), 顶点重合, 跨 180 度
#    经线的环. 后者是真实的空白: equirectangular 投影在 lon 跨 +/-180 处会
#    给出相差 360*cos(lat)*M_PER_DEG 的 x -- 本文件把 lon0 限在 +/-179 并让
#    环半径远小于 1 度来回避它, 所以"跨日期变更线的场地"属未覆盖, 已在下方
#    strategy 注释里点名. 真要支持需要先裁决投影是否换成 TM/UTM, 不是补一条
#    断言能解决的.
#
# 4. 它不验证 enu_origin 与围栏的[相对距离]超出 FV-7 的 20 km 时会怎样 --
#    那一条是 p3 录制期校验(FV-7), 不在本链路上.
#
# 5. 用例里的一切坐标都是[生成]的, 与 data/run/fence.db 无关. 本文件
#    一个字节都不读那个库: 读了就等于把开发期模拟数据织进判据, 而客户现场的
#    数据与它没有任何关系(用户 2026-10-08 明令).
# ---------------------------------------------------------------------------

#: FV-7 keeps fence vertices within 20 km of enu_origin; a ring is generated
#: inside a tenth of that so the whole set fits however the origin lands.
_MAX_RING_R_DEG = 0.018          # ~2 km at the equator
#: 11 S9A.2 FV-3: at most 5 active fences, and exactly one of them is allow.
_MAX_FENCES = 5
#: 本机 local_frame 近极点拒绝(cos(lat0) ~ 0), 所以纬度留出余量; 经度全域.
_LAT_RANGE = (-70.0, 70.0)


# *** max_examples=60 的依据, 以免它被读成随手取的数.
# 本用例的输入空间里真正改变代码路径的维度只有四个: role 组合(2^3 的子集序列)
# 顶点数(3..40 但只分"三角形 / 多边形"两类行为), 凸凹(radii 长度是否为 1),
# 以及半球与东西经(lat0/lon0 的符号). hypothesis 的覆盖式生成在 60 个样本内
# 能把这四个维度的组合走遍一轮以上.
# NO 不是"跑得越多越好": 这条用例进全树门, 而单样本最坏几十毫秒 -- 把它调到
# 500 会让全树多花分钟级时间, 换来的是同一批路径再走几遍.
# 真要扩的话扩[维度](例如加入跨日期变更线), 不是扩样本数 -- 后者是在已覆盖
# 的空间里重复采样, 那正是 3.2 形态 1 里"看起来更严格而实际不变"的那种动作.


def _ring(cx: float, cy: float, n: int, radii):
    """以 (cx, cy) 为心, 按极角均分的 n 顶点简单环, 每个顶点半径由 radii 给.

    半径逐顶点不同 ==> 自然产生凹多边形(radii 里一个小值就是一个凹口), 而按
    极角单调排布保证它[不自交] -- FV-2 拒自交, 所以一般性只需覆盖到凹.
    返回 [[lat, lon], ...], 与 fence.db geom_json 的 points 同形.
    """
    pts = []
    for k in range(n):
        # 极角均分. 均分(而不是随机角)是不自交的来源: 顶点按角单调排布的
        # 星形多边形对任意正半径都是简单多边形.
        th = 2.0 * math.pi * k / n
        # 半径按 radii 循环取. 这就是凹口的来源 -- 相邻两点半径差得多,
        # 连线就凹进去.
        r = radii[k % len(radii)]
        # 返回 [lat, lon] 而不是 [lon, lat]: fence.db 的 geom_json points
        # 是 [lat, lon], 顺序写反在小区域里不会报错, 只会让整个场地转 90 度
        # 并偏到别处 -- 一个没有任何异常的错误.
        pts.append([cy + r * math.sin(th), cx + r * math.cos(th)])
    return pts


# 每个顶点的半径. 下界 0.0008 deg (约 90 m) 不是美观取值: 更小的环在
# equirectangular 投影下边长会掉到米级, 而 inset / soft margin 的量纲就是米,
# 于是"围栏比机器人还小"这种退化几何会把断言的失败归因搞混 -- 那是另一类用例
# (tests/p1_motion/fence/ 用手算几何测它), 不是一般性要覆盖的事.
# 上界取 _MAX_RING_R_DEG, 保证 5 个环连同各自的心偏移都落在 FV-7 的 20 km 内.
# min_size=1: 所有顶点同半径 => 正多边形(凸); max_size=6: 半径按 6 个值循环
# => 产生规则的凹口. 两端都要有, 凸形是基线, 凹形才是射线法与边距离最容易
# 打架的地方.
_radii = st.lists(st.floats(min_value=0.0008, max_value=_MAX_RING_R_DEG),
                  min_size=1, max_size=6)
# 一个围栏的规格: (顶点数, 半径表, priority).
_fence = st.builds(
    lambda n, radii, pri: (n, radii, pri),
    # 3 是 FV-1 的下界(围不出面积的环已在 _parse_polygon 被拒, 不在这里测);
    # 40 远超现场录制的实际点数, 取它是为了让 boundary_hit 的逐边扫描真的
    # 跑在多边形上而不是三角形上 -- O(edges) 的实现里, 3 条边的用例对
    # "最近边选错" 这类缺陷几乎没有分辨力.
    st.integers(min_value=3, max_value=40),
    _radii,
    # priority 是 int32, 且 S9A.2 逐字"负号保留". 取 +/-2^20 而不是 +/-2^31:
    # 2^31 的边界值本身由 crc32 配方的 _format_priority 单测覆盖(它测渲染),
    # 这里要的是"任意值都不改变链路不变量", 一个宽区间就够, 而过大的值会让
    # hypothesis 把样本全挤到边界上, 反而测不到中间的普通取值.
    st.integers(min_value=-2 ** 20, max_value=2 ** 20))


# deadline=None: 一个样本要走 p3 构造 + crc32 + p1 解析 + ENU 编译 + 逐多边形
# 距离, 40 顶点 x 5 个围栏时单样本可达几十毫秒. hypothesis 的默认 deadline 会
# 把它判成 flaky, 而慢不是缺陷 -- 这条路径一拍只跑一次编译(12 FS-1: 编译在拍外).
@settings(max_examples=60, deadline=None,
          suppress_health_check=[HealthCheck.too_slow])
@given(
    # 场地原点. 纬度留出极点余量(local_frame 在 cos(lat0) ~ 0 处拒绝, 见
    # _MIN_COS_LAT), 经度几乎全域 -- 东西经都要覆盖, 因为投影里经度要乘
    # cos(lat0), 符号搞错只在某一侧暴露.
    lat0=st.floats(*_LAT_RANGE),
    lon0=st.floats(min_value=-179.0, max_value=179.0),
    # allow 之外的 role 任意组合, 0..4 个. 不包含第二个 allow -- FS-5A 禁止,
    # 那是 validate_active_fence_set 的用例, 不是一般性.
    extra_roles=st.lists(st.sampled_from(["forbid", "speed_limit", "warning"]),
                         min_size=0, max_size=_MAX_FENCES - 1),
    # 恒取满 5 份规格再按 roles 的长度取用: 让 specs 的长度不随 extra_roles
    # 变化, hypothesis 的收缩(shrink)才不会把两个维度缠在一起.
    specs=st.lists(_fence, min_size=_MAX_FENCES, max_size=_MAX_FENCES),
    # 限速值任意正数, 上界取 U54 的整机最高速 2.0 -- 比它大的限速不是非法
    # (FV-13 只要求 > 0), 但那种值永不生效, 属 NEXT FN-1 登记的待裁边界.
    limit=st.floats(min_value=0.05, max_value=2.0),
)
def test_any_customer_fence_set_survives_the_whole_chain(
        lat0, lon0, extra_roles, specs, limit):
    """任意场地 + 任意围栏组合, 整条链的不变量都成立.

    MUTATION(任一处): p3 与 p1 的 crc32 配方分叉 -> (1) 红;
    hard_enforce 改回按 warning 单条判 -> (2) 的 speed_limit 行红;
    compile_fence 放 warning/speed_limit 进边界集 -> (3) 红;
    boundary_hit 的 d_nom 符号丢掉 keep_in -> (4) 的 forbid 行红.
    """
    roles = ["allow"] + extra_roles                  # FS-5A: 恰好一个 allow
    rows = []
    for i, role in enumerate(roles):
        n, radii, pri = specs[i]
        # 每个围栏的心略错开 0.002 deg (约 220 m), 但都在原点附近.
        # *** 刻意让它们可能重叠: FV-4 的 v0.3 重写逐字"不再要求 forbid 落在
        # allow 内部, 不再要求互不相交", 所以把它们分离开反而是在测一个契约
        # 不保证的子集. 重叠正是画家算法那条待裁项(NEXT FN-2)的输入形态.
        pts = _ring(lon0 + 0.002 * i, lat0 + 0.002 * i, n, radii)
        # 行形状即 FencesDAO.list_active() 的 9 元组. hard_enforce 一律填 1 --
        # 包括 advisory role: 那正是旧写入方会留下的脏值, 链路必须自己按 role
        # 归一化掉它, 而不是依赖库里那一位是对的.
        rows.append(("f-%d" % i, "fence-%d" % i, role, "polygon",
                     json.dumps({"points": pts}), 1, 1, pri,
                     limit if role == "speed_limit" else None))

    # on_unusable 直接 fail: 本用例生成的每一行都是可广播的(限速区都带了值),
    # 所以任何丢弃都说明链路在拒绝一个合法集 -- 把它变成失败而不是静默少一个
    # 多边形, 否则下面的计数断言会用一个"少一个"的集去比, 失败归因就跑偏了.
    fs = build_fence_set(rows, fence_set_id="fs-gen", rev=3,
                         enu_origin={"lat": lat0, "lon": lon0, "alt": 0.0},
                         on_unusable=lambda pid, why: pytest.fail(
                             "a generated set should be broadcastable: %s %s"
                             % (pid, why)))

    # (1) p3 算的 crc32 必须过 p1 的自算比对. compile_fence_set 不符即抛, 所以
    #     "它没抛" 就是 FV-8 通过 -- 这条不需要再比一次数值.
    held = compile_fence_set(fs)
    # rev 原样穿过: 它进 state/fence.active.rev, 而 D 用 > 比较判生效前进
    # (S9A.3 S-6). 若被改写或取默认, 生效确认会永远等不到或立刻通过.
    # 多边形计数相等: 解析侧不许丢多边形. 丢一个 allow 就是可行区消失(fail-open),
    # 丢一个 forbid 就是禁区消失 -- 两者都没有任何其它断言会注意到.
    assert held.rev == 3 and len(held.polygons) == len(roles)

    # (2) 持有态逐多边形: role 原样, 顶点数原样, hard_enforce 按 role 派生.
    for i, role in enumerate(roles):
        hp = next(p for p in held.polygons if p.poly_id == "f-%d" % i)
        # role 原样: 闭集外必抛已另有用例, 这里要的是"合法 role 不被改写".
        # 一个被改写的 role 会让整个多边形换一种执行语义(允许区变禁区).
        assert hp.role == role
        # 顶点数原样: 丢顶点 = 悄悄换了一个形状. 环少一个点仍是合法多边形,
        # 所以没有任何结构校验会红 -- 只有这条计数会.
        assert len(hp.vertices) == specs[i][0]
        # hard_enforce 必须等于按 role 的派生值. 输入故意填的全是 1(见上),
        # 所以这条等价于"两个 advisory role 的那一位被归一化掉了".
        assert hp.hard_enforce is hard_enforce_for_role(role)
        # priority 原样穿过. 它今天无消费方(NEXT FN-2), 但它进 crc32 --
        # 若在这里被取默认, 两个叠放次序不同的集会算出同一个 crc32.
        assert hp.priority == specs[i][2]
        if role == "speed_limit":
            # 限速值穿过. approx 而非 == : hypothesis 生成的 float 经 json
            # 往返, %.3f 的 crc32 渲染不影响这里, 但 json 的 repr 往返仍可能
            # 差一个 ULP, 而本条测的是"值被带到了", 不是浮点位.
            assert hp.speed_limit_mps == pytest.approx(limit)
        else:
            # 反向(FV-13 的另一半): 非限速 role 恒 None. 这里若漏成 0.0,
            # 速度门一旦消费它就会把 v_max 压到 0 -- 3.1 的 fail-silent.
            assert hp.speed_limit_mps is None

    # (3) ENU 编译只收 allow/forbid 当边界, 顶点数不变.
    cf = compile_fence(held, LocalFrame(lat0, lon0))
    # 期望由 CONSTRAINT_ROLES 过滤 roles 得出 -- roles 是输入.
    # 多一个(把 warning/speed_limit 当边界)就是拿提示性区域去硬裁剪;
    # 少一个(把 forbid 丢了)就是禁区不生效. 两个方向这条都看得见.
    want = [r for r in roles if r in CONSTRAINT_ROLES]
    assert sorted(p.role for p in cf.polygons) == sorted(want)
    for p in cf.polygons:
        i = int(p.poly_id.split("-")[1])
        # WGS84 -> ENU 是逐顶点映射, 不做抽稀. 顶点数变了说明投影环节在
        # 改几何, 而 S9A.0 逐字"不改几何".
        assert len(p.xy) == specs[i][0]

    # (4) 带符号距离: 对每个编译多边形都有限, 且在[环心]处的符号与 keep_in 相符.
    #     环心按构造在环内(极角均分的简单环), 所以 allow 应为正(允许侧=内),
    #     forbid 应为负(允许侧=外). 凹环同样成立 -- 凹口不会把心排除出去,
    #     因为所有半径都 > 0 且心是极角中心.
    for p in cf.polygons:
        i = int(p.poly_id.split("-")[1])
        # 用与生成时同一个偏移反算环心. 这里重建 LocalFrame 而不复用上面的
        # cf: CompiledFence 只存 ENU 顶点, 不存投影器 -- 它是编译期的输入,
        # 拍内只读几何(12 FS-1). 这正是"同一个 enu_origin 两处必须一致"的
        # 那条要求(S9A.2: 每个进程用同一个原点投影, ENU 逐位一致).
        cxy = LocalFrame(lat0, lon0).to_xy(lat0 + 0.002 * i, lon0 + 0.002 * i)
        hit = boundary_hit(cxy[0], cxy[1], p)
        # 有限性先于符号: 一个 NaN 距离会让下游每个比较都返回 False,
        # 于是裁剪层静默地什么都不裁 -- 没有报错, 围栏形同不存在.
        assert math.isfinite(hit.d_nom_m)
        # 法向同理: NaN 法向进半空间投影会把速度算成 NaN, 而 NaN 一路发到
        # 底盘的现象是"机器人不动且无故障码".
        assert math.isfinite(hit.normal[0]) and math.isfinite(hit.normal[1])
        # 单位法向(非退化): 环心离边界恒 > 0, 所以法向必有定义.
        assert math.hypot(*hit.normal) == pytest.approx(1.0, abs=1e-6)
        # keep_in 必须由 role 决定, 而且这条要[单独]断言 --
        # *** 第一版把下面的期望写成 `if p.keep_in: ...`, 于是"keep_in 恒 True"
        # 这个变异体存活: 读被测量本身来决定期望, 等于把结论定义进前提
        # (CLAUDE.md 3.2 形态 7). 期望必须由 role 推出 -- role 是输入, 是契约
        # 事实; keep_in 是 compile_fence 的产出, 是被测的那一半.
        assert p.keep_in is (p.role == "allow")
        if p.role == "allow":
            assert hit.d_nom_m > 0.0        # 活动区: 允许侧 = 内, 心在允许侧
        else:
            assert hit.d_nom_m < 0.0        # 禁入区: 允许侧 = 外, 心在禁止侧


@settings(max_examples=30, deadline=None)
@given(lat0=st.floats(*_LAT_RANGE),
       lon0=st.floats(min_value=-179.0, max_value=179.0))
def test_an_empty_fence_set_compiles_to_zero_boundaries_anywhere(lat0, lon0):
    """"客户还没录任何围栏" 在任意场地都编译成[零条边界], 而不是报错.

    *** 这条配的是 11 S9A.7 的 disabled 档与 clip.evaluate 的第一行:
    fence 为 None 或 polygons 为空 -> enforcement=disabled / no_fence, 且
    (vx, vy) [原样透传] ==> 机器人可任意行走, NO 不是"拒动".
    用户 2026-10-08 明令: 系统没有任何围栏数据时不对 RNS 运动做任何限制.
    逐拍零回归由 tests/p1_motion/nav/test_nav_tick_fence.py 的
    test_no_fence_stage_is_a_pass_through_over_a_whole_drive 钉住(同一输入序列
    喂接围栏与不接围栏两套栈, 80 拍逐拍要求输出相同); 本条钉的是它的前提 --
    编译这一步不会先抛.
    MUTATION: 让 compile_fence 对空集抛(或造一个兜底边界) -> 本条红.
    """
    from xbrain.p1_motion.fence.fence_set import HeldFenceSet, HeldPolygon

    # crc32 填一个假值: 本用例不走 compile_fence_set(那是自算比对那一层),
    # 直接构造持有态去测编译. 持有态是 frozen dataclass, 不校验 crc.
    empty = HeldFenceSet("fs-none", 0, "00000000", ())
    cf = compile_fence(empty, LocalFrame(lat0, lon0))
    # 零条边界, 而不是抛异常. 抛的话 nav_tick 的 step 7 会在每一拍打一个
    # 异常, 而 4.4 要求循环内注入 raise 只能让本拍零速 -- 于是"客户还没录
    # 围栏"就变成"机器人每拍零速", 正是用户点名不许出现的那个结果.
    assert cf.polygons == ()
    # 另一个同义的"没有围栏限制": 集里只有 warning / speed_limit. 它们不是边界,
    # 所以编译结果同样是零条 ==> evaluate 走 disabled/no_fence, 速度原样透传.
    #
    # NO 这里不能写 `all(r in CONSTRAINT_ROLES for r in ())` 之类对空序列求值的
    # 断言 -- 空序列的 all() 恒 True, 那是一条什么实现都能通过的断言(3.2 形态1).
    # 必须真的建两个非约束多边形再看编译结果为空.
    ring = ((lat0, lon0), (lat0 + 0.001, lon0), (lat0 + 0.001, lon0 + 0.001))
    advisory = HeldFenceSet("fs-adv", 1, "00000000", (
        HeldPolygon("f-w", "warning", "w", False, ring),
        HeldPolygon("f-s", "speed_limit", "s", False, ring,
                    speed_limit_mps=1.0)))
    assert compile_fence(advisory, LocalFrame(lat0, lon0)).polygons == ()
    # 反向: 同一个环挂上 allow 就必须编译出一条边界 -- 否则上面的"为空"可能只是
    # 因为 compile_fence 对什么都返回空.
    keep_in = HeldFenceSet("fs-a", 1, "00000000", (
        HeldPolygon("f-a", "allow", "a", True, ring),))
    assert len(compile_fence(keep_in, LocalFrame(lat0, lon0)).polygons) == 1
