"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: locked_keys.py
Brief: CFG-BT-18 FSC-LOCK -- 运行期不可改的安全量, ConfigCommand 一律拒

Description:
有些配置项一旦系统起来就不能再改. `common.motion.free_space_corridor.*`
与 `common.motion.free_space.*` 是其中两组: 它们决定走廊几何与自由空间
判定, 也就是"多近算太近". 运行期改动它们, 等于在机器人正在动的时候搬走
安全边界 -- 而且改完立即生效, 没有任何一拍是按新旧两套一致地跑的.

所以对这两组的 ConfigCommand 一律 rejected + E_CONFIG_LOCKED, 没有例外.

*** 判据点名的要害: 只按"五组清单"判锁的实现会放行 margin_lat_m.
10 S5.4.4 的断言 E 有一份五组安全命名空间清单(common.safety.* /
common.spec.* / common.motion.profiles / common.qos.* / common.fence.*).
`common.motion.free_space_corridor.margin_lat_m` [不在那五组里] --
它在 common.motion 下, 但不是 common.motion.profiles.

一个直接复用断言 E 清单来判锁的实现, 会认为这个键没锁, 于是放行一条
改走廊横向余量的命令. 那是本模块存在的全部理由: FSC-LOCK 是断言 E 之外
的一层兜底, 不是它的复述.

*** 前缀匹配必须带分隔符.
"common.motion.free_space" 是 "common.motion.free_space_corridor" 的
前缀. 用裸 startswith 判断时, 后者会被前者匹配上 -- 这里恰好两组都锁,
所以结果对; 但只要将来其中一组解锁, 那个巧合就会变成一个错误的放行.
所以按[键路径的段]比, 不按字符串前缀比.

Boundaries: 只判"这个键能不能改", 不判值合不合法(那是 schema 的事),
也不发 Ack -- 调用方按返回值组装.
"""
from __future__ import annotations

from typing import Optional, Tuple

from ..errors import E_CONFIG_LOCKED

#: 运行期锁死的键前缀(按段给出). 每条都要说得出为什么锁.
#:
#: NO 不从断言 E 的五组清单推导: 那份清单管的是"冻结期必须存在且已赋值",
#: 与"运行期能不能改"是两件事. 复用它正是判据点名要抓的错误.
LOCKED_PREFIXES = (
    # 走廊几何: 横向余量 / 旋转余量 / 前视距离. 运行期改它 = 在机器人
    # 正在动的时候搬走安全边界.
    ("common", "motion", "free_space_corridor"),
    # 自由空间判定的阈值组. 同上.
    ("common", "motion", "free_space"),
)


def _segments(key_path: str) -> Tuple[str, ...]:
    return tuple(p for p in str(key_path).split(".") if p)


def is_locked(key_path: str) -> bool:
    """这个键路径是否运行期锁死.

    *** 按段比, 不按字符串前缀比.
    "common.motion.free_space" 是 "common.motion.free_space_corridor" 的
    字符串前缀 -- 裸 startswith 会让后者被前者匹配. 今天两组都锁所以结果
    碰巧对; 将来其中一组若解锁, 那个巧合就变成一次错误的放行.
    """
    segs = _segments(key_path)
    for prefix in LOCKED_PREFIXES:
        if len(segs) >= len(prefix) and segs[:len(prefix)] == prefix:
            return True
    return False


def check_config_command(key_path: str) -> Optional[dict]:
    """ConfigCommand 的锁检查. 返回 None 放行, 或 detail dict 表示拒.

    detail 里必须带[键路径]: 一条只说"被锁了"的拒绝, 让运维不知道是哪个
    键触发的 -- 而一条 ConfigCommand 可能带多个键.
    """
    if not is_locked(key_path):
        return None
    return {
        "code": E_CONFIG_LOCKED,
        "key": key_path,
        # 指回出处, 让读到 Ack 的人能查为什么锁.
        "reason": "FSC-LOCK: runtime-immutable safety geometry (CFG-BT-18)",
    }
