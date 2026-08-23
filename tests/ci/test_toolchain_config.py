"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_toolchain_config.py
Brief: CFG-DC-2 / INF-CI-3 -- lint 与类型工具链配置的落地与形状

Description:
在本轮之前, 仓库根[没有任何 lint / 类型配置文件]: 没有 pyproject.toml,
没有 .clang-format, 没有 .clang-tidy. 也就是说每个人和每次 CI 用的都是
各自本地工具的默认值 -- "在我这儿是过的"这句话不携带任何信息.

本文件守三件事:
  1. 三个配置文件存在且能被解析;
  2. 里面的关键设定没有被悄悄放宽(mypy strict 关掉 / C++ 标准改掉);
  3. 与 CLAUDE.md 的硬要求一致(C++17 恰好 / Google Style).

*** 一处如实记录: mypy --strict 今天不是零 error.
CFG-DC-2 判据要"mypy --strict xbrain/ common/ 零 error". 实测下来存量
不是零 -- 那是一笔既有债, 不是本轮引入的. 本文件[不]断言零 error:
写一条今天必红的断言, 三天内就会被人放宽成"包含即可", 那正是
CLAUDE.md 3.2 形态2 的路径. 债记在 NEXT, 数字用脚本现跑(3.7 不烤死数字).

* 一个顺带的发现值得记: mypy 在 xbrain/common/errors 上报 "has no
attribute E_INTERNAL", 而运行期它确实存在(40 个 E_* 导出). 那是 mypy 对
动态导出的盲区, 不是运行期缺陷 -- 查过才知道. 类型检查器的报错同样需要
逐条核实, 不能因为它是工具就当成事实.

Boundaries: 不跑 ruff(本机未安装), 不跑 clang-tidy(要编译数据库).
只保证配置存在, 可解析, 关键项没被放宽.
"""
from __future__ import annotations

import pathlib

import pytest

pytestmark = pytest.mark.no_device

ROOT = pathlib.Path(__file__).resolve().parents[2]


def _toml(path):
    """解析 toml; 没有 tomllib(py3.10)时退到最小行解析.

    *** NO 不 skip.
    第一版在 tomllib 缺席时 pytest.skip -- 而本机正是 py3.10, 于是四条
    关键断言(strict 有没有被关掉 / 排除项有没有吞掉自己的代码 / ruff 规则
    集是不是空的)全部空过. 一个在最常见的解释器版本上什么都不检查的门,
    与没有这个门没有区别(CLAUDE.md 3.2 形态1).

    退化解析只认本文件真正用到的三种形状: 布尔, 字符串列表, 数字.
    够用就行 -- 它不是通用 toml 解析器, 也不该是.
    """
    try:
        import tomllib
        return tomllib.loads(path.read_text(encoding="utf-8"))
    except ImportError:
        pass
    out, section = {}, None
    for raw in path.read_text(encoding="utf-8").split("\n"):
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        if line.startswith("[") and line.endswith("]"):
            section = line[1:-1]
            node = out
            for part in section.split("."):
                node = node.setdefault(part, {})
            continue
        if "=" not in line or section is None:
            continue
        key, _sep, val = line.partition("=")
        key, val = key.strip(), val.strip()
        node = out
        for part in section.split("."):
            node = node.setdefault(part, {})
        if val in ("true", "false"):
            node[key] = (val == "true")
        elif val.startswith("["):
            # 单行或多行列表: 把到下一个 ] 之间的内容当作元素.
            node[key] = _read_list(path, key)
        else:
            node[key] = val.strip('"')
    return out


def _read_list(path, key):
    """把 `key = [ ... ]` 的元素读出来(支持跨行)."""
    text = path.read_text(encoding="utf-8")
    idx = text.find(key + " = [")
    if idx < 0:
        return []
    end = text.find("]", idx)
    body = text[idx + len(key) + 4:end]
    items = []
    for chunk in body.split(","):
        chunk = chunk.split("#", 1)[0].strip().strip('"').strip("'")
        if chunk:
            items.append(chunk)
    return items


def test_pyproject_exists_and_parses():
    """配置文件必须能被解析.

    一个语法坏掉的 pyproject.toml 的表现是: 工具静默回退到默认值, 而不是
    报错. 于是所有设定都没生效, 而 CI 照样绿.
    """
    path = ROOT / "pyproject.toml"
    assert path.is_file(), "仓库根没有 pyproject.toml -- 工具用的是各自的默认值"
    cfg = _toml(path)
    assert "tool" in cfg and "mypy" in cfg["tool"], "pyproject 里没有 mypy 段"
    assert "ruff" in cfg["tool"], "pyproject 里没有 ruff 段"


def test_mypy_strict_is_on():
    """*** strict 被关掉是最容易发生也最难察觉的放宽.

    关掉之后 mypy 仍然跑, 仍然绿, 只是不再检查绝大多数东西. 与"没装 mypy"
    的区别在报告里看不出来.

    MUTATION: 把 strict 改成 false -> 红.
    """
    cfg = _toml(ROOT / "pyproject.toml")
    assert cfg["tool"]["mypy"].get("strict") is True, (
        "mypy strict 被关掉了 -- CLAUDE.md 8.2 要求严格模式")


def test_mypy_exclusions_do_not_swallow_our_own_code():
    """排除项只许排掉[不是我们写的]那些.

    一条 `exclude = ["^xbrain/"]` 会让整个门变成空跑, 而配置文件看起来
    仍然一切正常. 所以这里反向查: 我们自己的两棵树不许出现在排除里.
    """
    cfg = _toml(ROOT / "pyproject.toml")
    excludes = cfg["tool"]["mypy"].get("exclude") or []
    for own in ("xbrain", "common/", "scripts"):
        for pat in excludes:
            assert own not in pat or "models" in pat, (
                "mypy 排除项 %r 把我们自己的代码排掉了" % pat)


def test_ruff_select_is_not_empty():
    """规则集为空 = ruff 什么都不查, 而它照样 exit 0."""
    cfg = _toml(ROOT / "pyproject.toml")
    select = (cfg["tool"]["ruff"].get("lint") or {}).get("select") or []
    assert select, "ruff 的 select 为空 -- 它什么都不会查"
    # F(pyflakes) 是最低限度: 未定义名字 / 未使用导入这类真错误靠它.
    assert "F" in select, "ruff 没有开 F -- 未定义名字这类真错误查不出来"


def test_clang_format_pins_cxx17():
    """*** C++ 恰为 17(CLAUDE.md 5.2 / 13 CPP-1).

    告诉格式化器标准是哪一版, 它才不会往新语法方向重排. 写成 c++20 的话,
    格式化器会开始接受并整理 C++20 写法, 而平台基线 D-45 还没拍板.
    """
    path = ROOT / ".clang-format"
    assert path.is_file(), "没有 .clang-format"
    text = path.read_text(encoding="utf-8")
    assert "BasedOnStyle: Google" in text, "没有基于 Google Style"
    assert "Standard: c++17" in text, "C++ 标准不是 c++17"
    for bad in ("c++20", "c++23", "Latest"):
        assert bad not in text, "出现了 %s -- 13 CPP-1 要求恰为 C++17" % bad


def test_clang_tidy_treats_bugprone_as_errors():
    """bugprone-* 必须是 error 不是 warning.

    warning 在一份有几百条 warning 的构建输出里等于不存在. 而 bugprone
    抓的正是 chassis_relay 那类在急停链路上的问题.
    """
    path = ROOT / ".clang-tidy"
    assert path.is_file(), "没有 .clang-tidy"
    text = path.read_text(encoding="utf-8")
    assert "bugprone-*" in text
    assert "WarningsAsErrors" in text and "bugprone" in text.split(
        "WarningsAsErrors")[1][:60], "bugprone-* 没有被当成 error"


def test_clang_tidy_does_not_restate_the_discipline_rules():
    """*** 纪律规则不许在这里再写一份.

    RT-C4 / DDS-3 / CRL-3 这些有专门的执行体
    (scripts/ci/cxx_discipline_audit.py, 表驱动 + fixtures). 在 .clang-tidy
    里再写一遍会造出第二个真源, 而两份迟早不一致 -- 到那时没人说得清
    哪一份是对的.
    """
    text = (ROOT / ".clang-tidy").read_text(encoding="utf-8")
    body = text.split("Checks:")[1] if "Checks:" in text else text
    for rule in ("RT-C4", "DDS-3", "DDS-4", "CRL-3"):
        assert rule not in body.split("NOTE")[0], (
            "%s 在 .clang-tidy 的检查项里重复定义了" % rule)


def test_the_toolchain_files_carry_the_project_header():
    """三个配置文件都要有五字段头注(CLAUDE.md 2.5 覆盖面含 yaml 与配置).

    没有头注的配置文件读起来像是从别处抄来的, 而这三份里每一条偏离默认值
    的设定都需要说明为什么.
    """
    for name in ("pyproject.toml", ".clang-format", ".clang-tidy"):
        text = (ROOT / name).read_text(encoding="utf-8")
        assert "上海哈船智能船舶技术有限公司" in text, "%s 缺公司名" % name
        assert "Brief:" in text and "Description:" in text, "%s 头注不全" % name
