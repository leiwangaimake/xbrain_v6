"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_candidate_gate.py
Brief: 候选级围栏门 candidate_fence_hazard (NEXT R-4b / T3)

Description:
本文件解决的问题: 12 S7 把围栏作用在[最终速度指令]上, 这对安全是充分的 --
机器人从不越栏. 但 RNS 不知道围栏, 于是它会一直提出一条穿过边界的子目标,
裁剪层把朝外分量去掉, 剩下的切向分量接近零, 机器人就贴在栏上. NEXT R-4b 在
SIL 实测到约 70 s 这样的停滞(最终 ARRIVED, 从未越栏). 本门让 RNS 不再提出
裁剪层会杀掉的候选.

断言分三组, 对应函数的三种情形:
  (1) 越界/无余量时[不否决] -- 这是最重要的一条, 见下;
  (2) P->S 与边真相交 -> hazard(两端都合法不等于弦合法);
  (3) 不相交时按线段到边界的距离与 inset 比.

*** 为什么 (1) 必须单独钉死. 一个已经贴栏(或越栏)的机器人需要出路, 而它的
每一条逃生候选都穿过它正贴着的那条边界; 把它们全否决, RNS 就一个候选都没有
了 -- 那是[硬停滞], 比 R-4b 的慢停滞更糟. 12 S7.2 对裁剪层说的是同一件事
(逐字"置零会让机器人卡在边界上无法退回"), 半空间投影正是为此保留退回分量.
本门的阈值用的就是裁剪层自己的那个: d_eff > 0 时才活, 也就是[裁剪层还在软带
里]的那些拍.

*** 它[不]做什么: 不裁速度(那是 clip.evaluate), 不判限速区(speed_cap_mps),
不碰 warning. 它只回答一个布尔: 这条候选能不能提.

*** 看起来对但会出错的写法:
  * 用 point_in_polygon 判候选终点. 射线法逐字"Boundary points may be
    reported either way"且不含 inset ==> 会出现"射线法判内, 带符号距离因内缩
    判外", 于是 RNS 提出, 裁剪层杀掉 -- 正是本门要修的那个停滞.
  * 只判终点 S 在不在界内. 一条弦可以两端都合法而中间切过凹边界, 或削掉禁区
    的一个角.
  * 采样线段代替精确相交. 采样会漏掉任何浅于采样间距的穿越, 而一个 95% 正确
    的几何门比没有门更糟 -- 它让失败看起来像 RNS 在乱走.
"""
from __future__ import annotations

import copy
from pathlib import Path

import pytest
import yaml

from tests.p1_motion.rns.scenes import (
    healthy_status,
    snapshot,
    uniform_free,
    with_block,
)
from xbrain.p1_motion.fence.clip import (
    CompiledFence,
    CompiledPolygon,
    candidate_fence_hazard,
)

pytestmark = pytest.mark.no_device

# ---------------------------------------------------------------------------
# *** 红验记录(3.3). 2026-10-08 对本门注入的变异体与结果见提交信息.
#
# *** 本组用例在写的过程中红过三次, 三次都是[用例的前提错了]而不是实现错了.
# 记在这里, 因为这三种形态会重现:
#
#  (1) "贴墙走"那条: 起点放在离边界 1 m 处, 而 inset 是 2 m ==> d_eff = -1,
#      按设计落进情形 1(无余量不否决), 于是不 hazard. 用例把"机器人已经在
#      内缩区里"写成了"候选贴着边界走" -- 两者是不同情形. 起点必须有余量.
#
#  (2) 接线判据的基线用了 uniform_free ==> 零候选, 于是"围栏门让候选消失"
#      这个断言在零候选上恒真. 一条什么实现都能通过的判据(3.2 形态 1).
#      必须先确认基线[有]候选, 差异才能归因到门.
#
#  (3) 接线判据的围栏半边长取了 3 m, 而这张 profile 的候选落在约 2.4 m --
#      它们本来就在界内. 几何用例的尺寸必须按[实测的]候选距离定, 不能随手取
#      一个"看起来小"的数.
#
#  (4) "恰好站在有效边界上"那条, 终点取了 (0, 0)(朝心走) -- 而那一拍线段到
#      边界的最近距离恰好等于 inset, 于是 abstain 分支与净空分支给出[同一个]
#      答案, `<= 0` 改 `< 0` 的变异体存活. 不是等价变异, 是用例站在了刀锋上:
#      要杀掉一个阈值变异体, 必须让两个分支给出[不同]答案的那种输入.
#
#  (5) 接线判据第一版直接给 s._fence_cf 赋值 -- 它证明了 _pick_candidate 会读
#      那个字段, 却[绕过了] ctx -> self 那两行, 于是"把 getattr 改成恒 None"
#      的变异体存活. 接线判据必须从[最外面]那一头进去.
#
# ==> 共同点: 几何判据红了, 先问"用例构造出的局面是不是我想测的那个局面",
#     再怀疑实现. 五次里有五次是前者.
#     而变异体存活时先问"我的用例有没有覆盖到那个分支的[差异]", 再判它是不是
#     等价变异 -- 五条里只有一条真的是等价变异(nav_tick 的第二道带子).
#
# *** 还有一条关于本门[不]该做什么的边界, 写在这里以免有人来"加强"它:
# 本门不是安全门. 安全由 12 S7 的裁剪层保证(作用在最终速度指令上, 每拍都做,
# 靠几何而不是靠信任规划器). 本门只防[停滞] -- 它让 RNS 不去提那些裁剪层会
# 杀掉的候选. 两者的失效方向因此完全不同:
#   * 裁剪层漏判 ==> 机器人越栏. 必须 fail-safe.
#   * 本门漏判   ==> 机器人贴栏停滞一会儿. 不安全性为零.
#   * 本门误判(过严) ==> RNS 候选被砍光 ==> 硬停滞, 比它要修的问题更糟.
# 所以本门的取舍一律偏向[少否决]: 无围栏不否决, 无 inset 不否决, 无余量不
# 否决, 软围栏不否决. 想让它"更安全"的改动, 真正的效果是让机器人更容易卡死.
# ---------------------------------------------------------------------------

#: 真的 configs/rns.yaml -- 候选构造与代价权重都从它取, NO 不在这里捏一份
#: 字典(S3.6 的变体制: 仿真/测试参数走变体文件, 不写进 Python 夹具).
_ROOT = Path(__file__).resolve().parents[3]
_REAL_CFG = yaml.safe_load(
    (_ROOT / "configs" / "rns.yaml").read_text(encoding="utf-8"))


def _rns_cfg():
    """一份可改的深拷贝 -- 同 tests/p1_motion/rns 的范式."""
    return copy.deepcopy(_REAL_CFG)


def _ctx_with_fence(cf, inset):
    """一个最小 tick ctx, 形状同 tests/p1_motion/rns 的 Ctx.

    duck-typed: RnsSource 用 getattr 读每个字段, 所以这里只需要有那几个名字.
    fence / fence_inset_m 原样带上(含 None), 这样同一个工厂既能造"带围栏"
    也能造"不带围栏"的那一拍.
    """
    from types import SimpleNamespace
    # SimpleNamespace 而不是 dataclass: 本门只需要 ctx 上有那几个[名字],
    # 而 RnsSource 对每个字段都走 getattr(..., None). 用 dataclass 会让人以为
    # ctx 有一个受约束的形状 -- 它没有, 这正是 SIL 台架能自己造一个的原因.
    return SimpleNamespace(
        pose_xy=(0.0, 0.0), yaw_rad=0.0, v_nom_mps=1.0, wz_max_rps=1.0,
        perception=_snap_for_tick(), now_mono_ms=5000, holonomic=False,
        fence=cf, fence_inset_m=inset)


# compute() 入口有一串感知门(T-50/T-51 的时效, 外参, 健康), 任一不过它就在
# 更早的分支返回, 那两行 getattr 根本执行不到. 所以这张快照必须是[健康]的,
# 时间戳也要落在 now_mono_ms 之前的容限内 -- 否则接线判据会因为感知门而绿,
# 看起来像接线成功, 实际上一行都没跑到.
def _snap_for_tick():
    """一张健康的感知快照 -- compute() 的感知门要它才会走到 follow 主干."""
    return snapshot(profile=_profile_with_an_obstacle(),
                    objects=None,
                    status=healthy_status(t_publish_mono_ms=4900))


def _profile_with_an_obstacle():
    """一张带障碍的 FreeSpaceProfile.

    *** NO 不能用 uniform_free: candidates_from_profile 是从[障碍边缘]与
    [间隙]构造候选的, 全空旷的 profile 一条边缘都没有 ==> 零候选. 第一版
    用了它, 于是基线就是 None, 而"围栏门让候选消失"这个断言在零候选上
    恒真 -- 一条什么实现都能通过的判据(3.2 形态 1).
    正中打一块障碍 -> 两侧各有一条边缘 -> 绕行候选存在.
    """
    base = uniform_free(t_capture_mono_ms=4980, t_seg_mono_ms=4970)
    # 打在正中: 两侧各留一条边缘, 于是左右都有绕行候选. 打在边上只会得到
    # 一条边缘, 候选集偏向一侧, 而"围栏把候选全砍掉"这个断言在单侧候选上
    # 说服力弱得多.
    mid = len(base.d_block) // 2
    for i in range(mid - 6, mid + 7):          # 中间一片连续障碍
        base = with_block(base, i, 2.0)
    return base

#: 一个 100 x 100 m 的方形活动区, 心在原点(ENU 米). 顺序 ccw.
#: 顺序必须是 ccw(逆时针). 顺序本身不进本门的判定 -- boundary_hit 的内外靠
#: 射线法, 与绕向无关 -- 但 11 S9A.2 给 allow 定的声明绕向就是 ccw, 用例照它
#: 写, 读者才不会以为这里有一个"顺序无关"的暗示.
_SQUARE = ((-50.0, -50.0), (50.0, -50.0), (50.0, 50.0), (-50.0, 50.0))
#: 本组用例统一用 2 m 内缩 -- 取一个[明显]大于零的值, 这样"差一点"与"差很多"
#: 在断言里分得开; 真实取值由 margin_by_fix 按 fix 质量给(12 S7.4).
_INSET = 2.0
#: 为什么本组用例自己定 inset 而不去读 configs: inset 的真实取值是
#: common.fence.margin_by_fix[fix_type](12 S7.4 查表), 而那张表在配置里仍是
#: null(未标定, 3.1). 用一个本地常量, 本组用例才能在冻结线跑通之前就有意义,
#: 而且它测的是"门如何用 inset", 不是"inset 取多少" -- 后者是 R-3 的裁决.


def _allow(xy=_SQUARE, hard=True):
    """一个 keep-in 边界(role=allow): 允许侧是内部.

    直接构造 CompiledPolygon 而不走 compile_fence: 本门的输入就是编译后的
    ENU 几何, 让用例多走一遍 WGS84 -> ENU 只会把投影的误差掺进几何断言里,
    而投影本身另有判据(test_fence_generality / local_frame 那组).
    """
    return CompiledPolygon("f-a", "allow", "camp", hard, True, xy, None)


def _forbid(xy, hard=True):
    """一个 keep-out 边界(role=forbid): 允许侧是外部.

    keep_in=False 是这两个工厂唯一的差别, 而它决定 d_nom 的符号 --
    所以禁区的用例不是活动区用例的镜像写法, 而是同一套代码在相反符号下的
    行为. 两者都要测.
    """
    return CompiledPolygon("f-f", "forbid", "oil", hard, False, xy, None)


def _fence(*polys):
    """只填 .polygons(边界), speed_zones 留空.

    本门只看边界 -- 限速区不是边界, 它走 speed_cap_mps. 把限速区塞进
    .polygons 是 T2 那批里专门配了变异体的那个错误, 这里不重复它.
    """
    return CompiledFence("fs", 1, tuple(polys))


# -- (1) 无余量时不否决 -----------------------------------------------------
# 下面每条用例的命名都说出[断言什么], 不说"测什么函数" -- 失败时 pytest 打
# 出来的那一行就是结论本身, 不需要再去读用例体才知道坏了哪条性质.
def test_a_breached_robot_is_not_denied_every_candidate():
    """越出活动区的机器人必须还能提候选 -- 否则它永远回不来.

    MUTATION: 删掉 `if boundary_hit(...) - inset <= 0: continue` 这一行 ->
    机器人在界外时每条候选都 hazard -> RNS 一个候选都没有 -> 本条红.
    """
    f = _fence(_allow())
    # 机器人在栏外 10 m, 朝栏内走 -- 这是唯一正确的逃生方向.
    assert candidate_fence_hazard(f, (60.0, 0.0), (30.0, 0.0), _INSET) is False
    # 连继续往外走也不否决: 本门不是安全门(那是裁剪层), 它只防停滞.
    # 若在这里兼做安全判定, 就会有两处各判一次, 迟早分叉.
    assert candidate_fence_hazard(f, (60.0, 0.0), (80.0, 0.0), _INSET) is False


def test_a_robot_exactly_at_the_effective_boundary_is_not_denied_either():
    """恰好站在有效边界上(d_nom == inset)同样不否决.

    *** 这一拍很容易被漏掉: 此时线段到边界的距离必然 <= inset, 所以若按"情形
    3"去判, 每条候选都会 hazard. 阈值取 <= 0 而不是 < 0 就是为了把它并入
    情形 1.
    MUTATION: 把 `<= 0.0` 改成 `< 0.0` -> 本条红.
    """
    f = _fence(_allow())
    # x = 48 时 d_nom = 2.0 = inset, d_eff = 0.
    # *** 终点必须选一个[若不 abstain 就会 hazard]的候选, 否则两个分支给出
    # 同一个答案, 这条断言就杀不掉 `<= 0` 改 `< 0` 的变异体 -- 第一版终点取
    # (0, 0)(朝心走), 线段到边界的最近距离恰好 = inset, 两边都判不 hazard,
    # 于是变异体存活. 那不是等价变异, 只是用例站在了刀锋上.
    # 终点 (49, 10): 线段末端离右边界只有 1 m < inset ==> 不 abstain 的话
    # 必然 hazard.
    assert candidate_fence_hazard(f, (48.0, 0.0), (49.0, 10.0),
                                  _INSET) is False
    # 朝心走同样放行(这一条本来就两边都对, 留着是因为它是现场最常见的那一拍).
    assert candidate_fence_hazard(f, (48.0, 0.0), (0.0, 0.0), _INSET) is False


# -- (2) 真相交 -------------------------------------------------------------
# 情形 2 与情形 3 的分界不是"远/近", 而是"穿没穿过". 一条穿过去的线段, 它到
# 边界的最近距离可以是零(在交点上), 也可以在两端都很大 -- 所以距离判定单独
# 是不够的, 必须先问相交. 反过来, 一条不穿过但贴得很近的线段距离判定能抓到,
# 相交判定抓不到. 两个判定各管一半, 缺任一半都有漏网的形状.
def test_a_segment_leaving_the_allow_fence_is_a_hazard():
    """从界内指向界外 -> hazard. 这是 R-4b 的那条候选.

    MUTATION: 把相交判定整段删掉(只留距离判定) -> 一条从心指向远处界外的
    候选, 其线段到边界的最近距离可能大于 inset(它是穿过去的, 不是贴着走的)
    -> 本条红.
    """
    f = _fence(_allow())
    assert candidate_fence_hazard(f, (0.0, 0.0), (80.0, 0.0), _INSET) is True


# 凹活动区那条是本文件里最接近现场的一个形状: 客户绕着一栋楼录活动区边界,
# 楼的凹角就是一个缺口. 机器人在缺口两侧之间走直线就会短暂出界 -- 而"两端都
# 在界内"的实现看不见它. FV-4 的 v0.3 重写明确[不再]要求围栏凸或互不相交,
# 所以凹不是异常输入, 是正常输入.
def test_a_chord_with_both_ends_legal_can_still_be_a_hazard():
    """两端都在允许侧, 而弦切过边界 -> 仍是 hazard.

    *** 只判终点的实现会放过它. 这里用一个[凹]的活动区: 把方形右边中间挖进
    来一个缺口, 于是缺口两侧的两个点都在界内, 而它们的连线穿出缺口.
    MUTATION: 把门改成"只判 S 点在不在界内" -> 本条红.
    """
    # ccw: 右边中间向内凹进 30 m.
    concave = ((-50.0, -50.0), (50.0, -50.0), (50.0, -10.0), (20.0, 0.0),
               (50.0, 10.0), (50.0, 50.0), (-50.0, 50.0))
    f = _fence(_allow(concave))
    # (40, -20) 与 (40, 20) 都在界内(缺口的上下两侧), 连线穿过缺口外侧.
    assert candidate_fence_hazard(f, (40.0, -20.0), (40.0, 20.0), _INSET) is True


# 禁区那条的第二个断言(绕开)不是装饰. 只有"穿过 -> hazard"一条时, 一个把
# 所有多边形都当 keep-in 的实现同样通过: 穿过一个 keep-in 边界也是 hazard.
# 必须配上"绕开 -> 放行", 因为那一拍在 keep-in 的读法下反而[在界外], 会被
# 误判成 hazard -- 两条合起来才钉住 keep_in 的符号.
def test_a_segment_entering_a_forbid_fence_is_a_hazard():
    """进入禁区 -> hazard. keep_in=False 的那一侧.

    MUTATION: candidate_fence_hazard 忽略 keep_in(把所有多边形当 keep-in)
    -> 禁区变成"必须待在里面" -> 本条的第二个断言(绕开禁区的候选)反而红.
    """
    # 以 (0, 0) 为心, 20 m 见方的禁区; 机器人在禁区外.
    box = ((-20.0, -20.0), (20.0, -20.0), (20.0, 20.0), (-20.0, 20.0))
    f = _fence(_forbid(box))
    # 穿过禁区 -> hazard.
    assert candidate_fence_hazard(f, (-60.0, 0.0), (60.0, 0.0), _INSET) is True
    # 从禁区上方绕开(留出远超 inset 的间距) -> 放行.
    assert candidate_fence_hazard(f, (-60.0, 60.0), (60.0, 60.0),
                                  _INSET) is False


# -- (3) 不相交时的净空 -----------------------------------------------------
# 本条与情形 1 那两条合起来界定了门的[活跃区间]: 机器人必须有余量门才活,
# 而候选必须留足 inset 才放行. 两个条件用的是同一个 inset, 这不是巧合 --
# 裁剪层在 d_eff <= 0 处置零, 所以"候选会把我带到 d_eff <= 0"与"我已经在
# d_eff <= 0"是同一条线的两侧, 门在线外侧守净空, 线内侧让路.
def test_a_segment_hugging_the_boundary_closer_than_the_inset_is_a_hazard():
    """不穿越, 但贴得比 inset 近 -> hazard.

    *** 为什么这也要否决: inset 内侧就是裁剪层[硬置零]的地方(d_eff <= 0 且
    hard_enforce), 所以这种候选一旦被选中, 下一拍就被裁成零 -> 又是停滞.
    阈值用的是裁剪层同一个 inset, 这是本门与裁剪层口径一致的那一半.
    MUTATION: 把 `d_min < inset_m` 改成 `d_min < 0.0` -> 本条红.
    """
    f = _fence(_allow())
    # *** 起点必须[有余量], 否则落进情形 1 而不是情形 3.
    # 第一版把起点放在 x = 49(离边界 1 m): 那时 d_eff = 1 - 2 = -1 <= 0,
    # 本门按设计[不否决], 于是断言红 -- 而原因不是距离判定错, 是用例把
    # "机器人已经在内缩区里"写成了"候选贴着边界走". 两者是不同的情形, 前者
    # 归情形 1(保出路), 后者才归情形 3(保净空).
    # 起点 (40, -30): d_nom = 10 > inset, 有余量; 终点 (49, 30) 使线段末端
    # 离右边界只有 1 m < inset -> 候选把机器人带进内缩区 -> hazard.
    assert candidate_fence_hazard(f, (40.0, -30.0), (49.0, 30.0),
                                  _INSET) is True
    # 同起点, 终点留足净空(x = 40, 离边界 10 m) -> 放行. 若没有这条反向,
    # 上面的 True 可能只是"任何候选都 hazard".
    assert candidate_fence_hazard(f, (40.0, -30.0), (40.0, 30.0),
                                  _INSET) is False


# 软围栏那条看起来像"顺手加的", 其实它守的是本门与裁剪层的口径一致:
# 本门否决的理由是"裁剪层会把它裁成零", 而裁剪层只对 hard_enforce 置零.
# 若本门连软围栏也否决, 两层的判定面就不再重合 -- 候选被砍掉, 而裁剪层本来
# 根本不会拦它, 于是砍掉的是一条完全可行的路.
def test_a_soft_only_polygon_never_vetoes():
    """hard_enforce=False 的边界不否决候选.

    11 S9A.2: hard_enforce false 逐字"只产生软减速与事件, 不做硬裁剪" ==>
    靠近它不会被裁成零, 就不存在停滞风险, 否决它只会无谓地砍掉候选.
    MUTATION: 删掉 `if not poly.hard_enforce: continue` -> 本条红.
    """
    f = _fence(_allow(hard=False))
    assert candidate_fence_hazard(f, (0.0, 0.0), (80.0, 0.0), _INSET) is False


# -- 不变量 I-1 -------------------------------------------------------------
# I-1 是用户 2026-10-08 的明令, 所以它的判据不写成"顺带检查一下", 而是把
# 每一条早退路径逐个点出来: 无围栏对象 / 有对象但无边界 / 无起点 / 无终点 /
# 无 inset. 五条都必须 False -- 任一条返回 True, 现象都是"客户还没录围栏,
# 机器人就一个候选都提不出来", 而那在现场看起来像机器人坏了.
def test_no_fence_and_no_inset_gate_nothing():
    """无围栏 / 无位置 / 无 inset -> 一律不否决(NEXT I-1).

    用户 2026-10-08 明令: 系统没有任何围栏数据时不对 RNS 运动做任何限制.
    MUTATION: 任一早退分支改成 return True -> 本条红, 而且现象会是"客户还没
    录围栏, 机器人就一个候选都提不出来".
    """
    f = _fence(_allow())
    assert candidate_fence_hazard(None, (0.0, 0.0), (80.0, 0.0), _INSET) is False
    assert candidate_fence_hazard(_fence(), (0.0, 0.0), (80.0, 0.0),
                                  _INSET) is False
    assert candidate_fence_hazard(f, None, (80.0, 0.0), _INSET) is False
    assert candidate_fence_hazard(f, (0.0, 0.0), None, _INSET) is False
    assert candidate_fence_hazard(f, (0.0, 0.0), (80.0, 0.0), None) is False


# -- 接线级 -----------------------------------------------------------------
# 下面两条是一对: 前者证明 fence_hazard 这一位有效力, 后者证明有人会去置它.
# 分开写是因为两者的失效完全不同 -- 前者红说明 candidate.py 的 gate 掉了,
# 后者红说明 source.py 没接线. 合成一条的话, 红了还得去读用例才知道是哪头.
def test_gate_3_rejects_a_hazard_candidate():
    """前置: gate 3 本身在位.

    下面那条接线判据断言的是"候选被拒", 而候选可能因为别的 gate 被拒 --
    所以先单独确认 fence_hazard 这一位真的能把候选拒掉, 否则接线那条的
    绿色说明不了任何事.
    MUTATION: 删掉 candidate.py:141 的 `if c.fence_hazard: return False`
    -> 本条红.
    """
    import dataclasses

    from xbrain.p1_motion.rns.candidate import Candidate, passes_hard_gates
    ok = Candidate(subgoal=(3.0, 0.0), clearance_m=5.0, dynamic_blocked=False,
                   in_unobserved=False, fence_hazard=False)
    assert passes_hard_gates(ok, 0.35, 0.2) is True
    assert passes_hard_gates(dataclasses.replace(ok, fence_hazard=True),
                             0.35, 0.2) is False


def test_the_gate_is_actually_wired_into_the_rns_candidate_loop():
    """驱动真的 RnsSource._pick_candidate, 证明门被[接上了].

    *** 上面那组只证明 candidate_fence_hazard 算得对, 不证明有人调它. 本仓
    收过三例"能力都在, 最后一段没接"(SW-28), 所以接线必须单独有判据, 而且
    必须驱动真入口 -- 直调被调方证明不了调用方接了线.

    做法: 建一个真 RnsSource, 按 _run_follow 做的那两件事把围栏放到 self 上,
    然后用[同一个] profile 跑两次 _pick_candidate: 一次不带围栏(基线, 必须
    选得出候选), 一次带一个把全部候选都圈在界外的围栏(必须选不出来). 两次只差
    围栏, 所以差异只能来自围栏门.
    MUTATION: 删掉 source.py 里置 fence_hazard 的那一段 -> 第二次仍选得出 ->
    本条红. 把 self._fence_cf 的 getattr 改成恒 None -> 同样红.
    """
    from xbrain.p1_motion.rns.source import RnsSource

    cfg = _rns_cfg()
    margin = max(cfg["rns"]["clearance"]["margin_by_class"].values())
    prof = _profile_with_an_obstacle()

    class FS:                           # minimal FollowState stand-in
        lookahead_point = (4.0, 0.0)

    # 基线: 无围栏 -> 必须选得出一个候选. 若这里就是 None, 下面的 None 说明
    # 不了围栏门起了作用(这正是 3.2 形态 1 的那种"什么实现都能通过").
    s = RnsSource(cfg=cfg, r_eff_m=0.35)
    base = s._pick_candidate(prof, (0.0, 0.0), 0.0, FS(), cfg["rns"], margin)
    assert base is not None, "baseline must produce a candidate"

    # 带围栏: 把机器人围在一个比候选距离更小的活动区里.
    # *** 尺寸必须按[实测的候选距离]来定, 不能随手取. 第一版用了半边长 3 m,
    # 而这张 profile 的候选落在约 2.4 m 处 -- 它们本来就在界内, 于是"候选存活"
    # 是对的, 红的是用例. 半边长取 1.2 m: 机器人在心上仍有余量(d_nom = 1.2 >
    # inset 0.3, 所以围栏门是活的, 走情形 2/3), 而每条候选都越界.
    tiny = ((-1.2, -1.2), (1.2, -1.2), (1.2, 1.2), (-1.2, 1.2))
    s2 = RnsSource(cfg=cfg, r_eff_m=0.35)
    s2._fence_cf = _fence(_allow(tiny))
    s2._fence_inset = 0.3
    gated = s2._pick_candidate(prof, (0.0, 0.0), 0.0, FS(), cfg["rns"], margin)
    assert gated is None, \
        "fence gate not wired: a candidate outside the allow fence survived"


def test_the_ctx_fields_reach_self_through_a_real_tick():
    """ctx.fence / ctx.fence_inset_m -> self._fence_cf / self._fence_inset.

    *** 上一条用例是直接给 s2._fence_cf 赋值的 -- 它证明了 _pick_candidate
    会读那两个字段, 但[绕过了] ctx -> self 那两行. 实测: 把
    `getattr(ctx, "fence", None)` 改成恒 None 的变异体在只有上一条时[存活].
    所以这一条跑一次真的 compute(), 从 ctx 这一头进去看 self 这一头.
    MUTATION: 把 _run_follow 里那两行 getattr 中任一改成恒 None -> 本条红.
    """
    from xbrain.p1_motion.rns.route import Mission
    from xbrain.p1_motion.rns.source import RnsSource
    from xbrain.p1_motion.rns.types import MissionKind, Origin

    cf = _fence(_allow())
    # 50 x 50 m 的活动区 + 原点出发 + 2 m inset: 机器人在心上有大把余量,
    # 所以这一拍围栏门是[活]的. 若起点无余量, 门会 abstain, 那时即使
    # self._fence_cf 被正确赋值也看不出任何行为差别 -- 而本条要测的恰是赋值
    # 本身, 所以它直接断言字段, 不绕道行为.
    s = RnsSource(cfg=_rns_cfg(), r_eff_m=0.35)
    # 跑 compute() 之前先确认初值是 None -- 否则"跑完是 cf"可能是夹具留下的.
    assert s._fence_cf is None and s._fence_inset is None
    # 一条沿 +x 的直线任务, 0..20 m. compute() 要有 mission 才会走到 follow
    # 主干(没有任务时它在更早的分支就返回了, 那两行 getattr 根本到不了).
    s.load_mission(Mission(MissionKind.PATH, Origin.ROUTE,
                           [(i * 0.5, 0.0) for i in range(41)],
                           search_window=8, arrival_radius_m=1.0,
                           max_deviation_m=10.0))
    s.compute(_ctx_with_fence(cf, _INSET))
    assert s._fence_cf is cf              # is, 不是 == : 必须是同一个对象
    assert s._fence_inset == _INSET
    # 反向: 一个不带这两个字段的 ctx(SIL 台架就是这样)必须让它们回到 None,
    # 否则上一拍的围栏会粘住 -- 围栏撤销后仍按旧几何门控.
    s.compute(_ctx_with_fence(None, None))
    assert s._fence_cf is None and s._fence_inset is None
