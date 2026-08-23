"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_migration.py
Brief: CFG-BT-9 -- 四库 schema_version 迁移框架的三条判据与变异体

Description:
三条判据的方向都是不对称的, 每条各配变异体:
  (1) 库 < 代码: 按序迁移, 事务内完成, 失败即回滚且退出非零;
  (2) 库 > 代码: 直接拒绝, NO 不做向下兼容猜测(判据点名的变异体);
  (3) disk image malformed: E_STORAGE_CORRUPT + detail.db_name.

*** 判据点名的变异体: 把某库 schema_version 改成代码版本 +1, (2) 必须红.
这条守的是一个很有诱惑力的错误 -- "新库大概兼容旧代码". 它在这里特别
危险: 新版本可能加了一列 NOT NULL(旧代码 INSERT 会失败, 至少还报错),
也可能改了某列的语义(旧代码照读不误, 而值的含义已经变了 -- 不报错).

*** 事务那条要用[真的会失败的一步]来验.
一个分步提交的实现在中途失败时会留下"半新半旧"的库: 既跑不了新代码也
回不去旧代码, 而版本号看起来是合法的. 所以下面注入一步会抛的迁移, 再去
读库确认前面几步的改动[都没有留下].

Boundaries: 不定义任何一个库的真实迁移步骤(那属于各库), 只测框架.
"""
from __future__ import annotations

import pathlib
import sqlite3

import pytest

pytestmark = pytest.mark.no_device


def _make_db(tmp_path, version, name="t.db"):
    path = str(pathlib.Path(tmp_path) / name)
    conn = sqlite3.connect(path)
    conn.execute("CREATE TABLE t (a INTEGER)")
    conn.execute("PRAGMA user_version = %d" % version)
    conn.commit()
    conn.close()
    return path


def _step_add_column(col):
    def _fn(conn):
        conn.execute("ALTER TABLE t ADD COLUMN %s INTEGER" % col)
    return _fn


def _step_that_fails(conn):
    raise RuntimeError("boom")


def test_migrates_forward_in_order(tmp_path):
    """判据(1) 的正例: 库版本低时按序迁到代码版本."""
    from xbrain.persistence.migration import migrate, read_schema_version

    path = _make_db(tmp_path, 1)
    got = migrate(path, "t.db", 3,
                  [(2, _step_add_column("b")), (3, _step_add_column("c"))])
    assert got == 3
    conn = sqlite3.connect(path)
    try:
        assert read_schema_version(conn) == 3
        cols = {r[1] for r in conn.execute("PRAGMA table_info(t)")}
        assert {"b", "c"} <= cols, "迁移步骤没有真的执行: %s" % cols
    finally:
        conn.close()


def test_a_newer_database_is_refused(tmp_path):
    """*** 判据(2) / 判据点名的变异体: 库版本 = 代码版本 + 1 必须拒.

    NO 不做向下兼容猜测. 新库可能加了 NOT NULL 列(旧代码 INSERT 失败,
    至少报错), 也可能改了某列语义(旧代码照读不误而含义已变 -- 不报错).
    后者是真正要防的.
    """
    from xbrain.persistence.migration import MigrationError, migrate
    from xbrain.common.errors import E_CONFIG_INVALID

    path = _make_db(tmp_path, 4)
    with pytest.raises(MigrationError) as exc:
        migrate(path, "task.db", 3, [])
    assert exc.value.code == E_CONFIG_INVALID
    # detail 要带库名与两个版本 -- 只说"版本不对"让运维不知道是哪个库.
    assert exc.value.detail["db_name"] == "task.db"
    assert exc.value.detail["db_version"] == 4
    assert exc.value.detail["code_version"] == 3


def test_equal_version_is_a_no_op(tmp_path):
    """版本相同时不跑任何步骤.

    没有这条, 一个"每次启动都重跑全部迁移"的实现也能通过前两条 --
    而它会在每次开机时对生产库做 DDL.
    """
    from xbrain.persistence.migration import migrate

    path = _make_db(tmp_path, 2)
    ran = []
    got = migrate(path, "t.db", 2, [(2, lambda c: ran.append(1))])
    assert got == 2
    assert ran == [], "版本相同却仍执行了迁移步骤"


def test_a_failing_step_rolls_the_whole_batch_back(tmp_path):
    """*** 判据(1) 的要害: 事务内完成, 失败即回滚.

    分步提交的实现会留下半新半旧的库 -- 既跑不了新代码也回不去旧代码,
    而版本号看起来合法.

    MUTATION: 把 `with conn:` 换成逐步 commit -> 这里红(b 列会留下来).
    """
    from xbrain.persistence.migration import MigrationError, migrate

    path = _make_db(tmp_path, 1)
    with pytest.raises(MigrationError):
        migrate(path, "t.db", 3,
                [(2, _step_add_column("b")), (3, _step_that_fails)])
    conn = sqlite3.connect(path)
    try:
        cols = {r[1] for r in conn.execute("PRAGMA table_info(t)")}
        assert "b" not in cols, (
            "第一步的改动留下来了 -- 迁移不是整批原子的, 库现在半新半旧")
        assert conn.execute("PRAGMA user_version").fetchone()[0] == 1, (
            "版本号被推进了, 而迁移其实失败了 -- 下次启动会跳过这批")
    finally:
        conn.close()


def test_a_hole_in_the_migration_chain_is_refused(tmp_path):
    """迁移链缺一步就整批不跑.

    跳过一步会把库带到一个没人测过的中间状态, 而版本号会显示成"已迁到
    最新"-- 之后任何一次排查都会从"版本是对的"开始, 走错方向.
    """
    from xbrain.persistence.migration import MigrationError, migrate

    path = _make_db(tmp_path, 1)
    with pytest.raises(MigrationError) as exc:
        migrate(path, "t.db", 3, [(3, _step_add_column("c"))])   # 缺 2
    assert exc.value.detail["missing_steps"] == [2]


def test_a_corrupt_database_reports_the_db_name(tmp_path):
    """*** 判据(3): disk image malformed -> E_STORAGE_CORRUPT + db_name.

    不带库名的报错让运维要挨个打开四个库找是哪个坏了.
    """
    from xbrain.persistence.migration import MigrationError, migrate
    from xbrain.common.errors import E_STORAGE_CORRUPT

    path = str(pathlib.Path(tmp_path) / "bad.db")
    with open(path, "wb") as fh:
        fh.write(b"SQLite format 3\x00" + b"\x00" * 200)   # 头对, 内容坏
    with pytest.raises(MigrationError) as exc:
        migrate(path, "record.db", 1, [])
    assert exc.value.code == E_STORAGE_CORRUPT
    assert exc.value.detail["db_name"] == "record.db"


def test_check_all_does_not_swallow_failures(tmp_path):
    """*** 多库入口不得把失败转成 warning.

    一个吞异常的框架会让整栈带着半迁移的库跑起来 -- 而 Stage 0 的全部
    意义就是在那之前拦住.
    """
    from xbrain.persistence.migration import MigrationError, check_all

    good = _make_db(tmp_path, 1, "good.db")
    newer = _make_db(tmp_path, 9, "newer.db")
    with pytest.raises(MigrationError):
        check_all({"good.db": (good, 1, []), "newer.db": (newer, 1, [])})


def test_error_codes_come_from_the_shared_module():
    """CLAUDE.md 3.5: E_* 由 common/errors 导出, 不字符串硬编码."""
    import ast

    from xbrain.persistence import migration

    src = pathlib.Path(migration.__file__).read_text(encoding="utf-8")
    tree = ast.parse(src)
    literals = [n.value for n in ast.walk(tree)
                if isinstance(n, ast.Constant) and isinstance(n.value, str)
                and n.value.startswith("E_")]
    assert not literals, "源码里有 E_* 字符串字面量: %s" % literals
