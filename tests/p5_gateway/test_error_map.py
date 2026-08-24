"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_error_map.py
Brief: E_* -> Qt error_code 映射必须双向完备, 且没有兜底

Description:
甲方 Qt 按整数码写界面分支. 映射漏一个的后果不是报错, 是[显示了错误的原因]:
操作员看到"配置或文件操作失败", 而真实原因是电量不足或急停未解除.

所以两个方向都要查:
  * 我方 40 个 E_* 每个都要有落点 -- 漏一个, 那条错误出网关时会抛(好), 或者
    在有兜底的实现里静默落到某个码上(坏);
  * v2.0 的每个整数码都要有来源 -- 没有来源意味着 Qt 侧写了一个永远收不到
    的分支, 那个分支里的文案永远不会被看到, 也就永远不会被发现写错了.

*** 本文件特别守"不许有兜底".
一个 `_MAP.get(code, 3001)` 的实现能让上面两条都通过(第一条尤其: 每个 E_*
都"有落点"了). 所以单独用一条未登记的码去打, 要求它抛.

Boundaries: 不判断某个 E_* 归到哪一档是否最合适(那是与甲方联调时对齐的事),
只保证[全覆盖 + 无兜底 + 双向].
"""
from __future__ import annotations

import pytest

pytestmark = pytest.mark.no_device


def _all_e_codes():
    from xbrain.common import errors

    return sorted(n for n in dir(errors) if n.startswith("E_"))


def test_every_error_code_has_a_qt_mapping():
    """*** 40 个 E_* 逐个必须有落点.

    MUTATION: 从 _MAP 里删掉任意一条 -> 这里红并点名那个码.
    """
    from xbrain.common import errors
    from xbrain.p5_gateway.outbound.error_map import to_qt_code

    missing = []
    for name in _all_e_codes():
        value = getattr(errors, name)
        try:
            to_qt_code(value)
        except KeyError:
            missing.append(name)
    assert not missing, (
        "这些 E_* 没有 Qt 整数码落点, 出网关时会抛: %s" % missing)


def test_every_qt_code_has_at_least_one_source():
    """*** 反向: v2.0 的每个整数码都要有来源.

    没有来源的整数码意味着 Qt 侧那个分支永远走不到 -- 里面的文案写错了也
    永远不会被发现.

    * 0(成功)不在此列: 它不由 E_* 映射产生, 是 accepted 时直接填的.
    """
    from xbrain.common import errors
    from xbrain.p5_gateway.outbound.error_map import QT_CODES, to_qt_code

    produced = set()
    for name in _all_e_codes():
        produced.add(to_qt_code(getattr(errors, name)))
    # 三个信封层的码不经 E_* 产生(报文可能连 JSON 都不是, 那时没有
    # 任何 E_* 可谈), 但它们必须在 ENVELOPE_ONLY_CODES 里显式登记 --
    # 见那里的说明. 这里把两个来源合起来查全覆盖.
    from xbrain.p5_gateway.outbound.error_map import ENVELOPE_ONLY_CODES

    produced |= set(ENVELOPE_ONLY_CODES)
    orphan = sorted(set(QT_CODES) - produced - {0})
    assert not orphan, (
        "这些 Qt 整数码没有任何 E_* 会产生它们 -- Qt 侧的对应分支永远走不到: "
        "%s" % orphan)


def test_an_unregistered_code_raises_instead_of_falling_back():
    """*** 不许有兜底.

    一个 `_MAP.get(code, 3001)` 的实现能让上面两条都通过, 而它会让将来
    新增的 E_* 静默落到 3001 -- Qt 显示"存储失败", 真实原因无从得知.

    MUTATION: 把 to_qt_code 改成 _MAP.get(e, CODE_STORAGE) -> 这里红.
    """
    from xbrain.p5_gateway.outbound.error_map import (UnmappedErrorCode,
                                                      to_qt_code)

    with pytest.raises(UnmappedErrorCode):
        to_qt_code("E_A_CODE_THAT_DOES_NOT_EXIST")


def test_channel_denied_maps_to_1006():
    """v2.0 S2.6 逐字: 禁止动作(MANUAL_VELOCITY / dog_to_pc / 云端 teleop)
    回 error_code=1006 + detail.code=E_CHANNEL_DENIED.

    这条是[客户会直接测的]: 联调时他们会故意发一条 MANUAL_VELOCITY 看我方
    是不是安全拒绝.
    """
    from xbrain.common import errors
    from xbrain.p5_gateway.outbound.error_map import (CODE_TASK_UNSUPPORTED,
                                                      to_qt_code)

    assert to_qt_code(errors.E_CHANNEL_DENIED) == CODE_TASK_UNSUPPORTED
    assert to_qt_code(errors.E_NOT_IMPLEMENTED) == CODE_TASK_UNSUPPORTED


def test_not_found_maps_to_1003_per_customer_reply():
    """*** 客户答复 4.3 逐字, 与 v2.0 S10 的表面读法不同.

    S10 把 1003 写成"类型, 范围, 枚举或版本冲突", 照字面读 E_NOT_FOUND
    更像 2004(ID 冲突). 但客户 2026-08-08 答复 4.3 逐字给的是:
    "rejected + error_code=1003 + detail.code=E_NOT_FOUND".

    * 以客户答复为准 -- 那是双方确认过的裁决, 而 S10 的分类只是概括.
    这条用例把那次裁决钉住, 免得下一个人按字面读法"订正"回 2004.
    """
    from xbrain.common import errors
    from xbrain.p5_gateway.outbound.error_map import (CODE_INVALID_FIELD,
                                                      to_qt_code)

    assert to_qt_code(errors.E_NOT_FOUND) == CODE_INVALID_FIELD


def test_build_error_fields_keeps_both_codes():
    """*** 整数码与原生字符串码[两者都要在].

    v2.0 S10 逐字: "后端原生字符串码放在 detail.code". 只给整数码, 排查时
    没人知道机内到底是哪一条; 只给字符串, Qt 没法做界面分支.
    """
    from xbrain.common import errors
    from xbrain.p5_gateway.outbound.error_map import (CODE_LOW_BATTERY,
                                                      build_error_fields)

    out = build_error_fields(errors.E_LOW_BATTERY, "电量不足, 无法出勤",
                             {"soc_pct": 8})
    assert out["error_code"] == CODE_LOW_BATTERY
    assert out["detail"]["code"] == errors.E_LOW_BATTERY
    # 调用方给的 detail 内容必须保留 -- 定位信息在里面.
    assert out["detail"]["soc_pct"] == 8
    assert out["reason"]


def test_build_error_fields_does_not_mutate_the_caller_dict():
    """detail 是调用方的对象, 不许被就地改.

    就地改会让同一个 detail 字典被复用时带上前一次的 code -- 而那种串味
    在日志里看起来像是"同一个错误发生了两次".
    """
    from xbrain.common import errors
    from xbrain.p5_gateway.outbound.error_map import build_error_fields

    original = {"field": "waypoints[3]"}
    build_error_fields(errors.E_OUT_OF_FENCE, "第 4 个关键点位于围栏外",
                       original)
    assert "code" not in original, "调用方的 detail 被就地加了 code"


def test_the_three_v2_named_codes_have_equivalents():
    """*** v2.0 S10 点名的三个 detail.code, 我方用等价既有码.

    E_TASK_UNSUPPORTED / E_RID_MISMATCH / E_VERSION_UNSUPPORTED 不在我方
    40 值闭集里. CLAUDE.md 3.5 逐字"错误码是闭集, 不得自造码", 所以 NO
    不新增三个码, 而是给出等价物:
      E_TASK_UNSUPPORTED    -> E_NOT_IMPLEMENTED
      E_RID_MISMATCH        -> E_SCHEMA
      E_VERSION_UNSUPPORTED -> E_PROTO_VERSION
    三者落到的整数码必须与 v2.0 S10 一致 -- 那才是 Qt 真正依赖的东西,
    detail.code 的字面差异要在联调纪要里与甲方确认.
    """
    from xbrain.common import errors
    from xbrain.p5_gateway.outbound.error_map import (
        CODE_RID_MISMATCH, CODE_TASK_UNSUPPORTED, CODE_VERSION_UNSUPPORTED,
        to_qt_code)

    assert to_qt_code(errors.E_NOT_IMPLEMENTED) == CODE_TASK_UNSUPPORTED
    assert to_qt_code(errors.E_PROTO_VERSION) == CODE_VERSION_UNSUPPORTED
    # ! E_SCHEMA 今天落 1003(字段非法)而不是 1004(rid 不匹配) --
    # 因为 rid 不匹配是 E_SCHEMA 的一个特例, 而 E_SCHEMA 还覆盖别的结构错.
    # 要让 Qt 收到 1004, 网关必须在[识别出是 rid 问题时]显式指定,
    # 见 inbound 的信封校验. 这条用例记录这个差异, 免得有人把 E_SCHEMA
    # 整体改成 1004 而让所有结构错都显示成"rid 不匹配".
    assert to_qt_code(errors.E_SCHEMA) != CODE_RID_MISMATCH


def test_envelope_codes_are_produced_only_by_the_envelope_path():
    """*** 1001/1002/1004 只能从 envelope_error() 出.

    它们发生在还没解析出业务对象的时刻 -- 报文可能连 JSON 都不是, 那时
    没有任何 E_* 可谈. 硬塞进 E_* 映射会造出一个假象: 好像机内某处产生了
    E_JSON_PARSE, 而机内没有那种码(3.5 闭集 40 值里没有它).

    MUTATION: 往 _MAP 里加一条产生 1001 的 -> 这里红.
    """
    from xbrain.common import errors
    from xbrain.p5_gateway.outbound.error_map import (
        CODE_JSON_PARSE, CODE_REQUIRED_FIELD, CODE_RID_MISMATCH,
        ENVELOPE_ONLY_CODES, to_qt_code)

    envelope = {CODE_JSON_PARSE, CODE_REQUIRED_FIELD, CODE_RID_MISMATCH}
    assert set(ENVELOPE_ONLY_CODES) == envelope
    # 没有任何 E_* 会映到这三个码上.
    for name in _all_e_codes():
        got = to_qt_code(getattr(errors, name))
        assert got not in envelope, (
            "%s 映到了信封层的码 %d -- 那三个码只能由 envelope_error() 产出"
            % (name, got))


def test_envelope_error_carries_a_real_e_code_in_detail():
    """信封层的拒绝, detail.code 仍必须是我方闭集里的真码.

    甲方要 detail.code 有内容; 而我方不许自造码. 结构错在我方一律 E_SCHEMA,
    具体是哪一种由整数码区分 -- 那正是甲方要整数码的原因.
    """
    from xbrain.common import errors
    from xbrain.p5_gateway.outbound.error_map import (CODE_RID_MISMATCH,
                                                      envelope_error)

    out = envelope_error(CODE_RID_MISMATCH, "rid 与 key 第二段不一致",
                         {"expected": "gj-001", "got": "gj-002"})
    assert out["error_code"] == CODE_RID_MISMATCH
    assert out["detail"]["code"] == errors.E_SCHEMA
    assert out["detail"]["expected"] == "gj-001"


def test_envelope_error_refuses_a_business_code():
    """反向: 用 envelope_error 产出业务码必须被拒.

    没有这条, envelope_error 会变成第二个"什么码都能出"的入口, 而它的
    detail.code 恒为 E_SCHEMA -- 于是一个电量不足的拒绝会显示成结构错.
    """
    from xbrain.p5_gateway.outbound.error_map import (CODE_LOW_BATTERY,
                                                      UnmappedErrorCode,
                                                      envelope_error)

    with pytest.raises(UnmappedErrorCode):
        envelope_error(CODE_LOW_BATTERY, "电量不足")
