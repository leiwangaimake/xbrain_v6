"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_timezone_probe.py
Brief: CHK-1-62 Stage 0 timezone assertion -- system zone must equal the config

Description:
CHS-A 的 "Time" 字段要求本地时区(CLAUDE.md 5.5 / 厂商 PDF), p2_core 的
time_window 是站点本地墙钟窗口. 两者共用一个前提: 机器的系统时区是对的.
这个前提错掉时[两边都不报错]-- 底盘第一帧回 0xE002 看起来像协议问题,
时段规则则安静地按错误的小时求值. 所以前提在 Stage 0 集中查一次.

*** 本文件守的重点是[空壳实现不许通过].
判据把变异体 (a) 写得很直白: "把断言实现成[存在 /etc/localtime 即通过]
必须仍红". 那是 CLAUDE.md 3.2 形态① -- 一个什么都不做的实现照样绿, 因为
/etc/localtime 在任何 Linux 上都存在. 所以本文件不测"能不能跑通", 只测
"该红的时候红不红":
  * 区名不一致 -> 必红(不是 warn, 不是降级);
  * 期望值为 null(未标定) -> 必红(CLAUDE.md 3.1);
  * /etc/localtime 是普通文件(不可解析区名) -> 必红, 且与"不一致"分开报;
  * probe 主流程里这条真的被调用(否则函数写对了也没接上).

*** 判据②③本批[没有做], 不许把这条当完成.
  (2) 正向对接 fake_chassis(本地时区格式化的 Time 被接受, UTC 的回 0xE002)
      -- scripts/dev/chassis_stub.py 目前不校验 Time 字段, 没有能回 0xE002
      的载体, 硬写一个断言就是自证.
  (3) time_window 跨午夜在两个时区产出不同结果 -- p2_core 只做了 time_window
      的[加载与 RE-3a 过滤](rules_loader.py), 还没有跨午夜判定器可测.
两条都记在 TODO, 本条任务不标完成.

Boundaries: 不改系统时区, 不动 /etc(测试用临时目录里的假 localtime).
"""
from __future__ import annotations

import os
import pathlib
import tempfile

import pytest
import yaml

from xbrain.boot.probe import checks

pytestmark = pytest.mark.no_device

ROOT = pathlib.Path(__file__).resolve().parents[2]
PROBE_CFG = ROOT / "configs" / "probe" / "thresholds.yaml"
COMMON_CFG = ROOT / "configs" / "common.yaml"

#: 一个真实存在的 zoneinfo 根. 测试要造"指向 zoneinfo 树里某个区"的软链,
#: 用真路径而不是自造目录, 免得解析逻辑对着一个现实中不存在的前缀过关.
_ZONEINFO = "/usr/share/zoneinfo"


def _link_to(tmpdir: str, zone: str) -> str:
    """造一个 /etc/localtime 的替身, 指向 zoneinfo 里的 zone."""
    target = os.path.join(_ZONEINFO, zone)
    if not os.path.exists(target):
        pytest.skip("zoneinfo lacks %s on this host" % zone)
    link = os.path.join(tmpdir, "localtime")
    os.symlink(target, link)
    return link


def test_resolves_the_zone_name_from_a_symlink():
    """先钉住前提: 解析本身要能用, 否则下面每条都会因为解析恒 None 而
    "红得很好看" -- 一个永远红的断言最终会被改成永远绿(CLAUDE.md 3.2 形态②).
    """
    with tempfile.TemporaryDirectory() as d:
        link = _link_to(d, "Asia/Shanghai")
        assert checks.resolve_system_zone(link) == "Asia/Shanghai"


def test_relative_symlink_resolves_the_same():
    """发行版镜像里 /etc/localtime 常是相对链(../usr/share/zoneinfo/...).
    按字符串前缀比较会漏掉这种写法, 表现是好机器被判成坏机器."""
    with tempfile.TemporaryDirectory() as d:
        sub = os.path.join(d, "etc")
        os.makedirs(sub)
        os.symlink(os.path.relpath(os.path.join(_ZONEINFO, "Asia/Tokyo"), sub),
                   os.path.join(sub, "localtime"))
        assert checks.resolve_system_zone(
            os.path.join(sub, "localtime")) == "Asia/Tokyo"


def test_mismatched_zone_fails():
    """*** 判据①的主断言, 也是变异体 (a) 打在的位置.

    MUTATION: 把 check_timezone 改成 "os.path.exists(localtime) -> None"
    (即"存在即通过"), 这条立刻红 -- 而在真机上那个空壳实现是[永远绿]的,
    因为 /etc/localtime 处处都在.
    """
    with tempfile.TemporaryDirectory() as d:
        link = _link_to(d, "Asia/Tokyo")
        fail = checks.check_timezone("Asia/Shanghai", link)
        assert fail is not None, "区名不一致却放行了"
        assert fail["kind"] == "timezone_mismatch"
        # 两个值都要打出来: 只说"时区错了"的话, 操作员不知道该改机器还是改配置.
        assert fail["expected"] == "Asia/Shanghai"
        assert fail["actual"] == "Asia/Tokyo"


def test_matching_zone_passes():
    """反向: 一致时必须放行. 没有这条, 一个 "return 失败" 的实现也能让上面
    那条绿."""
    with tempfile.TemporaryDirectory() as d:
        link = _link_to(d, "Asia/Shanghai")
        assert checks.check_timezone("Asia/Shanghai", link) is None


def test_uncalibrated_expected_value_is_refused():
    """CLAUDE.md 3.1: 未标定写 null, 缺失即拒绝启动.

    NO 不许兜底成 UTC 或"机器上是什么就算什么"-- 后者会让这条断言退化成
    恒真, 也就是 3.2 形态① 的另一种长相.
    """
    with tempfile.TemporaryDirectory() as d:
        link = _link_to(d, "Asia/Shanghai")
        fail = checks.check_timezone(None, link)
        assert fail is not None, "期望值为 null 却放行了"
        assert fail["kind"] == "timezone_not_calibrated"
        # 报出键路径, 让人知道去哪儿填.
        assert fail["key"] == "common.timezone"


def test_unresolvable_localtime_fails_and_is_reported_separately():
    """有的镜像把 /etc/localtime 做成区文件的拷贝而不是软链. 那时区名
    在文件系统上根本读不出来.

    必须红(不能因为"文件在那儿"就放行), 而且要与"不一致"分成两种 kind:
    这两件事的处置完全不同 -- 一个是重建软链, 一个是改时区或改配置.
    """
    with tempfile.TemporaryDirectory() as d:
        plain = os.path.join(d, "localtime")
        with open(plain, "wb") as fh:
            fh.write(b"TZif2\x00")            # 像模像样但不是软链
        assert checks.resolve_system_zone(plain) is None
        fail = checks.check_timezone("Asia/Shanghai", plain)
        assert fail is not None
        assert fail["kind"] == "timezone_unresolvable"


def test_missing_localtime_fails():
    """路径根本不存在时也必须红. realpath 对不存在的路径不抛异常, 它照样
    返回一个字符串 -- 只看 realpath 的实现会在这里静默通过."""
    with tempfile.TemporaryDirectory() as d:
        gone = os.path.join(d, "nope")
        assert checks.resolve_system_zone(gone) is None
        assert checks.check_timezone("Asia/Shanghai", gone) is not None


def test_probe_config_carries_the_expected_zone():
    """判据①的另一半: 期望值要真的落在 configs 里, 不是写死在代码里."""
    cfg = yaml.safe_load(PROBE_CFG.read_text(encoding="utf-8"))
    assert "timezone" in cfg, "probe 配置里没有 timezone 键"
    assert "expected" in cfg["timezone"]


def test_probe_expected_zone_equals_common_timezone():
    """*** 这条守的是[两处副本漂移].

    probe 在 Stage 0 跑, 早于冻结线, 展不开 ${common.*}(见 10 S5.4.1), 所以
    期望时区在 configs 里存了两份. 两份副本迟早会分叉, 而分叉的表现是:
    整栈按 common.timezone 干活, Stage 0 按另一个值放行/拦截 -- 两边各自
    "正确", 合起来错.

    MUTATION: 改动任一处的值, 这条红.
    """
    probe = yaml.safe_load(PROBE_CFG.read_text(encoding="utf-8"))
    common = yaml.safe_load(COMMON_CFG.read_text(encoding="utf-8"))
    want = common.get("common", common).get("timezone")
    assert probe["timezone"]["expected"] == want, (
        "probe 期望时区 %r 与 common.timezone %r 不一致"
        % (probe["timezone"]["expected"], want))


def test_the_probe_actually_calls_the_check():
    """*** 接线断言: 函数写对了但没接进主流程, 上面每条都照样绿.

    这是本仓反复遇到的一类 -- 只测构建器/纯函数看不见总线. 这里用最直接的
    办法: 把 checks.check_timezone 换成一个记账的替身, 跑一遍 run(), 看它
    有没有被调到.

    MUTATION: 删掉 __main__.py 里那段调用, 这条红.
    """
    from xbrain.boot.probe import __main__ as probe_main

    called = []

    def _spy(expected, localtime_path="/etc/localtime"):
        called.append(expected)
        return None

    real = checks.check_timezone
    checks.check_timezone = _spy
    try:
        # 配置/硬件都不齐, run() 大概率非零退出 -- 不关心返回值, 只关心
        # 这条检查在流程里被走到了.
        try:
            probe_main.run(str(PROBE_CFG), "/nonexistent/hw_profile")
        except Exception:
            pass
    finally:
        checks.check_timezone = real
    assert called, "probe 主流程没有调用 check_timezone"
    # 传进去的必须是配置里的值, 不是代码里另写的常量.
    probe = yaml.safe_load(PROBE_CFG.read_text(encoding="utf-8"))
    assert called[0] == probe["timezone"]["expected"]

def test_missing_timezone_key_reports_the_key_path_not_a_traceback():
    """*** 这条是本轮自己踩出来的.

    接线时写的是 cfg["timezone"]["expected"], 于是任何没有这个键的配置都会
    让 KeyError 带着 traceback 逃出 run() -- 现场看到的是"探针崩了", 而不是
    "去 timezone.expected 填个值". 8 个既存 probe 用例同时红, 根因全是这个.

    NO 不能改成 .get 兜底(CLAUDE.md 3.1 禁止安全参数默认值); 正确做法是缺键
    也走 E_CONFIG_INVALID 并把[键路径]放进 detail -- 与启动断言 A 同一口径.

    MUTATION: 把接线改回裸索引, 这里红(退出码仍是 1, 但 stderr 里没有那行
    JSON, 只有 traceback).
    """
    import json
    import subprocess
    import sys as _sys
    with tempfile.TemporaryDirectory() as d:
        cfg = os.path.join(d, "cfg.yaml")
        with open(cfg, "w", encoding="utf-8") as fh:
            # 有意不写 timezone 键.
            fh.write("disk: []\nmemory: {min_free_kb: 1}\n"
                     "temperature: {sensors: [], max_temp_c: 100.0}\n"
                     "databases: []\n")
        env = dict(os.environ, XBRAIN_PROBE_CONFIG=cfg,
                   XBRAIN_HW_PROFILE=os.path.join(d, "nope"))
        proc = subprocess.run([_sys.executable, "-m", "xbrain.boot.probe"],
                              capture_output=True, text=True, env=env,
                              cwd=str(ROOT))
        assert proc.returncode != 0, "缺键却放行了"
        # 关键: stderr 里要有一行能解析的 JSON, 且带键路径.
        emitted = [json.loads(l) for l in proc.stderr.splitlines()
                   if l.strip().startswith("{")]
        assert emitted, ("缺键时只吐了 traceback, 没有结构化输出:\n%s"
                         % proc.stderr)
        keys = [e.get("detail", {}).get("key") for e in emitted]
        assert "timezone.expected" in keys, (
            "报了错但没说是哪个键: %s" % emitted)
