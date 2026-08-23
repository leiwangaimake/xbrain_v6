"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: migration.py
Brief: CFG-BT-9 -- 四库 schema_version 的单向迁移框架(S-7 / S-8)

Description:
四个库(task.db / fence.db / geo.db / record.db)各有自己的 schema_version.
代码升级后库还是旧版, 需要按序迁移; 而库比代码新时[必须直接拒绝].

*** 三条判据的方向都是不对称的, 这不是巧合.
  (1) 库 < 代码: 按序迁移, [在一个事务内]完成, 失败即回滚并退出非零.
      分步提交的迁移在中途失败时会留下一个"半新半旧"的库 -- 那种库既跑
      不了新代码也回不去旧代码, 而且下次启动时版本号看起来是合法的.
  (2) 库 > 代码: 直接拒绝, NO 不做向下兼容猜测.
      "新库大概兼容旧代码"这个猜测在这里特别危险: 新版本可能加了一列
      NOT NULL, 旧代码的 INSERT 会失败; 也可能改了某列的语义, 旧代码
      照读不误而值的含义已经变了 -- 后者不报错.
  (3) disk image malformed: E_STORAGE_CORRUPT + detail.db_name.
      不带库名的报错让运维要挨个打开四个库去找是哪个坏了.

*** 为什么这里直接用 sqlite3 而不走 DAO.
CLAUDE.md 4.1 禁止 persistence/ 之外 import sqlite3 -- 本模块就在
persistence/ 内. 而且迁移发生在 Stage 0, 早于事件循环: 走 aiosqlite 需要
在那个时刻有一个 loop, 而 Stage 0 明确不该有.

Boundaries: 不定义任何一个库的具体迁移步骤(那属于各库自己), 只提供
"按序跑 + 事务内 + 失败回滚"的框架与三条判定.
"""
from __future__ import annotations

import sqlite3  # BUSINESS-IMPORT-OK(persistence-layer): this module IS the persistence layer; migration runs at Stage 0 before any event loop exists
from typing import Callable, Dict, List, Optional, Sequence, Tuple

from ..common.errors import E_CONFIG_INVALID, E_STORAGE_CORRUPT


class MigrationError(RuntimeError):
    """迁移失败. detail 里带库名与方向."""

    def __init__(self, code: str, detail: dict):
        super().__init__("%s %s" % (code, detail))
        self.code = code
        self.detail = detail


#: 一步迁移: (目标版本, 在已打开的连接上执行的函数).
#: 函数只做 DDL/DML, NO 不提交也不回滚 -- 事务由框架管, 步骤自己提交会
#: 让"整批原子"这条失效.
Step = Tuple[int, Callable[[sqlite3.Connection], None]]


def read_schema_version(conn: sqlite3.Connection) -> Optional[int]:
    """读 user_version; 读不到返回 None.

    用 sqlite 内建的 user_version 而不是自建一张表: 后者本身也需要一次
    迁移才能存在, 是个先有鸡还是先有蛋的问题.
    """
    row = conn.execute("PRAGMA user_version").fetchone()
    if row is None:
        return None
    return int(row[0])


def migrate(db_path: str, db_name: str, code_version: int,
            steps: Sequence[Step]) -> int:
    """把一个库迁到 code_version. 返回迁移后的版本.

    steps 按目标版本升序给出; 框架只跑 (当前版本, code_version] 区间内的.
    """
    try:
        # *** isolation_level=None 关掉 sqlite3 模块的自动事务管理.
        # 默认模式下, DDL(ALTER TABLE / CREATE TABLE)会被隐式提交 --
        # `with conn:` 在那种模式下[挡不住]已经落盘的 DDL. 实测: 第一步
        # ALTER 成功, 第二步抛异常, 回滚之后 b 列仍在, 库半新半旧.
        # 关掉自动管理后由本模块显式 BEGIN/COMMIT/ROLLBACK, DDL 才真正
        # 进事务(sqlite 本身支持事务性 DDL).
        conn = sqlite3.connect(db_path, isolation_level=None)
    except sqlite3.Error as exc:
        raise MigrationError(E_STORAGE_CORRUPT,
                             {"db_name": db_name, "reason": str(exc)})
    try:
        # 先做完整性检查. 一个 malformed 的库在后面的 PRAGMA 上也会抛,
        # 但那时的报错指向的是 PRAGMA 而不是"库坏了".
        try:
            ok = conn.execute("PRAGMA integrity_check").fetchone()
        except sqlite3.DatabaseError as exc:
            raise MigrationError(E_STORAGE_CORRUPT,
                                 {"db_name": db_name, "reason": str(exc)})
        if not ok or str(ok[0]).lower() != "ok":
            raise MigrationError(
                E_STORAGE_CORRUPT,
                {"db_name": db_name, "reason": str(ok[0] if ok else "unknown")})

        current = read_schema_version(conn)
        if current is None:
            raise MigrationError(E_STORAGE_CORRUPT,
                                 {"db_name": db_name,
                                  "reason": "no user_version"})
        if current > code_version:
            # *** 判据(2): 直接拒绝, NO 不做向下兼容猜测.
            raise MigrationError(
                E_CONFIG_INVALID,
                {"db_name": db_name, "db_version": current,
                 "code_version": code_version,
                 "reason": "database is newer than code; refusing to guess "
                           "backward compatibility"})
        if current == code_version:
            return current

        pending = sorted((v, fn) for v, fn in steps if current < v <= code_version)
        missing = _missing_versions(current, code_version, [v for v, _f in pending])
        if missing:
            # 缺一步就整批不跑: 跳过一步的迁移会把库带到一个没人测过的
            # 中间状态, 而版本号会显示成"已经迁到最新".
            raise MigrationError(
                E_CONFIG_INVALID,
                {"db_name": db_name, "missing_steps": missing,
                 "reason": "migration chain has holes"})

        # *** 判据(1): 整批在一个事务里. 任一步抛就整体回滚.
        try:
            conn.execute("BEGIN")
            for version, fn in pending:
                fn(conn)
                # user_version 不能用参数绑定(PRAGMA 的限制), 所以这里
                # 拼字符串 -- version 来自 steps 表里的 int, 不是外部输入.
                conn.execute("PRAGMA user_version = %d" % int(version))
            conn.execute("COMMIT")
        except MigrationError:
            conn.execute("ROLLBACK")
            raise
        except Exception as exc:
            conn.execute("ROLLBACK")
            raise MigrationError(
                E_CONFIG_INVALID,
                {"db_name": db_name, "from": current, "to": code_version,
                 "reason": "migration step failed and was rolled back: %s" % exc})
        return code_version
    finally:
        conn.close()


def _missing_versions(current: int, target: int,
                      have: Sequence[int]) -> List[int]:
    """(current, target] 区间里缺的版本号."""
    want = list(range(current + 1, target + 1))
    return [v for v in want if v not in set(have)]


def check_all(dbs: Dict[str, Tuple[str, int, Sequence[Step]]]) -> Dict[str, int]:
    """一次迁移多个库. 任一库失败即抛, 已迁的不回退.

    NO 不吞异常: 调用方(Stage 0 探针)要按 MigrationError.code 决定退出码,
    而一个把失败转成 warning 的框架会让整栈带着半迁移的库跑起来.
    """
    out = {}
    for name, (path, version, steps) in sorted(dbs.items()):
        out[name] = migrate(path, name, version, steps)
    return out
