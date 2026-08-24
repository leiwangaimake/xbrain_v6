"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: error_map.py
Brief: 我方 E_* 闭集 -> 甲方 Qt 稳定整数 error_code (v2.0 S10)

Description:
甲方 v2.0 S10 用一组稳定整数码(0 / 1001..1006 / 2001..2007 / 3001)让 Qt 能
做界面分支, 同时逐字要求"后端原生字符串码放在 detail.code". 也就是说两侧
都保留: 整数给 Qt 判断, 字符串给人排查.

评审 R10.1 已裁[我方吸收, 映射由我方网关统一实现]-- 本模块就是那个唯一
实现点. 机内一律只用 E_*(CLAUDE.md 3.5 闭集), 只有出网关这一步翻译成整数.

*** 为什么必须[全覆盖]而不是留兜底.
一个"未映射的 E_* 一律落 3001"的实现, 会让 Qt 收到一堆无法区分的错误 --
操作员看到的是"配置或文件操作失败", 而真实原因可能是电量不足或急停未解除.
所以本表对 40 个 E_* 逐个给出落点, 并由测试双向查:
  * 每个 E_* 都要有落点(可多对一);
  * 每个整数码都要有来源(否则 Qt 侧写了一个永远收不到的分支).

*** 三个 v2.0 点名而我方闭集里"没有"的码, 实际都已有等价物.
v2.0 S10 的 detail.code 列举了 E_TASK_UNSUPPORTED / E_RID_MISMATCH /
E_VERSION_UNSUPPORTED, 而对撞时发现它们不在我方 40 值闭集里. 逐个查过:
  E_TASK_UNSUPPORTED   -> E_NOT_IMPLEMENTED(不支持的任务类型)
  E_RID_MISMATCH       -> E_SCHEMA(信封字段与 key 不一致, 是结构错)
  E_VERSION_UNSUPPORTED-> E_PROTO_VERSION(协议版本不支持)
=> NO 不新增三个码. 3.5 逐字"错误码是闭集, 不得自造码"; 而甲方要的是
detail.code 里有一个可读的字符串, 我方给出等价的既有码同样满足 --
差别只在字面. * 这一条要在联调纪要里与甲方确认, 已记 NEXT.

Boundaries: 只做码的翻译. 不构造 ack 报文, 不判断该不该拒绝 --
那是各业务模块的事.
"""

from __future__ import annotations

from typing import Dict

from ...common import errors

#: v2.0 S10 的整数码闭集. NO 不自造新整数 -- Qt 侧按这张表写了界面分支,
#: 多一个它认不出来.
CODE_OK = 0
CODE_JSON_PARSE = 1001
CODE_REQUIRED_FIELD = 1002
CODE_INVALID_FIELD = 1003
CODE_RID_MISMATCH = 1004
CODE_VERSION_UNSUPPORTED = 1005
CODE_TASK_UNSUPPORTED = 1006
CODE_NOT_READY = 2001
CODE_BUSY = 2002
CODE_STATE_DENIED = 2003
CODE_ID_CONFLICT = 2004
CODE_LOW_BATTERY = 2005
CODE_OUT_OF_FENCE = 2006
CODE_ESTOP_ACTIVE = 2007
CODE_STORAGE = 3001

QT_CODES = (
    CODE_OK, CODE_JSON_PARSE, CODE_REQUIRED_FIELD, CODE_INVALID_FIELD,
    CODE_RID_MISMATCH, CODE_VERSION_UNSUPPORTED, CODE_TASK_UNSUPPORTED,
    CODE_NOT_READY, CODE_BUSY, CODE_STATE_DENIED, CODE_ID_CONFLICT,
    CODE_LOW_BATTERY, CODE_OUT_OF_FENCE, CODE_ESTOP_ACTIVE, CODE_STORAGE,
)

#: E_* -> Qt 整数码. 40 个码逐个给出落点, 每组附归类理由.
#:
#: * 归类看的是[Qt 拿这个码要做什么], 不是我方内部的分类. 例如
#: E_TEACH_BUSY 与 E_BUSY 在我方是两个不同子系统, 但对 Qt 都是"机器人忙,
#: 稍后重试"(2002) -- 再细分对界面没有意义, 而 detail.code 保留了原码.
_MAP: Dict[str, int] = {
    # -- 1001..1006 结构与协议层: 报文本身就不对 --------------------
    errors.E_SCHEMA: CODE_INVALID_FIELD,
    errors.E_PROTO_VERSION: CODE_VERSION_UNSUPPORTED,
    errors.E_NOT_IMPLEMENTED: CODE_TASK_UNSUPPORTED,
    # 通道被拒 = 这条能力不对云端开放(MANUAL_VELOCITY / dog_to_pc),
    # v2.0 S2.6 逐字要求它回 1006.
    errors.E_CHANNEL_DENIED: CODE_TASK_UNSUPPORTED,
    errors.E_QOS_VIOLATION: CODE_INVALID_FIELD,

    # -- 2001 机器人未就绪 ------------------------------------------
    errors.E_UNHEALTHY: CODE_NOT_READY,
    errors.E_DEGRADED: CODE_NOT_READY,
    errors.E_NO_HEADING: CODE_NOT_READY,
    errors.E_SAFETY_LINK_LOST: CODE_NOT_READY,
    errors.E_TELEOP_STALE: CODE_NOT_READY,

    # -- 2002 忙 / 动作许可失败 -------------------------------------
    errors.E_BUSY: CODE_BUSY,
    errors.E_TEACH_BUSY: CODE_BUSY,
    errors.E_CAPABILITY: CODE_BUSY,
    errors.E_ARB_NO_SOURCE: CODE_BUSY,
    errors.E_ARB_NO_DOMAIN: CODE_BUSY,
    errors.E_ARB_LEASE_EXPIRED: CODE_BUSY,
    errors.E_TIMEOUT: CODE_BUSY,

    # -- 2003 当前状态禁止 ------------------------------------------
    errors.E_TASK_STATE: CODE_STATE_DENIED,
    errors.E_TEACH_STATE: CODE_STATE_DENIED,
    errors.E_STATUS: CODE_STATE_DENIED,
    errors.E_CONFIRM_REQUIRED: CODE_STATE_DENIED,
    errors.E_ARB_DISABLED: CODE_STATE_DENIED,
    errors.E_ARB_DISARMED: CODE_STATE_DENIED,
    errors.E_CONFIG_LOCKED: CODE_STATE_DENIED,

    # -- 2004 ID 冲突 -----------------------------------------------
    errors.E_NAME_CONFLICT: CODE_ID_CONFLICT,
    errors.E_GEO_CONFLICT: CODE_ID_CONFLICT,
    errors.E_DUPLICATE: CODE_ID_CONFLICT,
    # 找不到目标: v2.0 S2.2 逐字"不存在的任务返回 rejected + E_NOT_FOUND",
    # 而 S10 把 1003 对应"类型/范围/枚举或版本冲突". 客户答复 4.3 逐字给的
    # 是 "rejected + error_code=1003 + detail.code=E_NOT_FOUND" ->
    # 以客户答复为准, 落 1003 而不是 2004.
    errors.E_NOT_FOUND: CODE_INVALID_FIELD,

    # -- 2005/2006/2007 安全相关 ------------------------------------
    errors.E_LOW_BATTERY: CODE_LOW_BATTERY,
    errors.E_OUT_OF_FENCE: CODE_OUT_OF_FENCE,
    errors.E_FENCE_INVALID: CODE_OUT_OF_FENCE,
    errors.E_GEO_INVALID: CODE_OUT_OF_FENCE,
    errors.E_GEO_TOO_LARGE: CODE_OUT_OF_FENCE,
    errors.E_GEO_INCOMPLETE: CODE_OUT_OF_FENCE,
    errors.E_TEACH_GEOMETRY: CODE_OUT_OF_FENCE,
    errors.E_TEACH_QUALITY: CODE_OUT_OF_FENCE,
    errors.E_LOCKED: CODE_ESTOP_ACTIVE,

    # -- 3001 配置 / 持久化 / 文件 ----------------------------------
    errors.E_CONFIG_INVALID: CODE_STORAGE,
    errors.E_STORAGE_CORRUPT: CODE_STORAGE,
    # 内部错误也落 3001: 对 Qt 来说它与"存储失败"同样是"我方出了问题,
    # 重试或联系运维", 而 detail.code 保留了 E_INTERNAL 供排查.
    errors.E_INTERNAL: CODE_STORAGE,
}


class UnmappedErrorCode(KeyError):
    """一个 E_* 没有 Qt 落点.

    抛而不是兜底: 兜底会让新增的 E_* 静默落到某个整数码上, 而 Qt 侧
    显示的原因是错的 -- 那比报错难查得多.
    """


#: *** 三个整数码[不经 E_* 映射产生], 由入站信封校验直接指定.
#:
#: 1001 JSON 无法解析 / 1002 缺必填字段 / 1004 rid 与 key 不匹配 --
#: 这三种错发生在[还没解析出业务对象]的时刻, 那时根本没有一个 E_* 可谈:
#: 报文可能连 JSON 都不是. 硬把它们塞进 E_* 映射会造出一个假象 --
#: 好像机内某处产生了 E_JSON_PARSE, 而机内没有那种码.
#:
#: 但它们必须[有明确来源], 否则 Qt 侧那三个分支永远走不到而没人发现.
#: 所以在这里显式登记, 由 inbound 的信封校验调用 envelope_error() 产出.
ENVELOPE_ONLY_CODES = {
    CODE_JSON_PARSE: (errors.E_SCHEMA, "JSON 无法解析"),
    CODE_REQUIRED_FIELD: (errors.E_SCHEMA, "缺少必填字段"),
    CODE_RID_MISMATCH: (errors.E_SCHEMA, "rid 与 key 或 session 不匹配"),
}


def envelope_error(qt_code: int, reason: str, detail: Dict = None) -> Dict:
    """信封层的拒绝. 三个码只能从这里产出.

    *** 为什么 detail.code 统一是 E_SCHEMA.
    我方闭集里没有 E_JSON_PARSE / E_RID_MISMATCH 这样的码, 而 3.5 禁止自造.
    结构性错误在我方一律是 E_SCHEMA; 具体是哪一种由[整数码]区分, 那正是
    甲方要整数码的原因. detail 里再带上定位字段(field/expected/got).
    """
    if qt_code not in ENVELOPE_ONLY_CODES:
        raise UnmappedErrorCode(
            "%r is not an envelope-level code; use build_error_fields() "
            "with an E_* instead" % qt_code)
    e_code, _label = ENVELOPE_ONLY_CODES[qt_code]
    body = dict(detail or {})
    body["code"] = e_code
    return {"error_code": qt_code, "reason": reason, "detail": body}


def to_qt_code(e_code: str) -> int:
    """E_* -> Qt 整数码. 未登记即抛."""
    if e_code in _MAP:
        return _MAP[e_code]
    raise UnmappedErrorCode(
        "%r has no Qt error_code mapping; add it to error_map._MAP "
        "(v2.0 S10). NO fallback: a wrong code shows the operator a wrong "
        "reason." % e_code)


def build_error_fields(e_code: str, reason: str,
                       detail: Dict = None) -> Dict:
    """组装 v2.0 拒绝所需的四件: error_code / reason / detail.code / detail.

    v2.0 S10 逐字要求所有拒绝同时提供 result=rejected / accepted=false /
    非零 error_code / 人类可读 reason / detail.code. 前两项由调用方按
    ack 形状填, 本函数给后三项.
    """
    body = dict(detail or {})
    # detail.code 是[我方原生字符串码], 不被整数码取代 -- 两者都要在.
    body["code"] = e_code
    return {
        "error_code": to_qt_code(e_code),
        "reason": reason,
        "detail": body,
    }
