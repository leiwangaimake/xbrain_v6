# data/

INF-DP-11: 编译数据 · 依赖数据 · 系统日志的存放根。

**四库归属**（`15` §9）：

| 文件 | 属主进程 | 用途 |
|---|---|---|
| `task.db` | `p3_task` | 任务队列 + 步骤重试计数 |
| `fence.db` | `p3_task` | 围栏定义 + 有效期 |
| `geo.db` | `p3_task` | 坐标 · 路径点 · 地图片段 |
| `record.db` | `p5_gateway` | `event/fault/system` 事件流 |

**日志**：`data/logs/{proc}.log`（由 `deploy/logrotate/xbrain` 轮转）

**初始化**：

```bash
bash scripts/init_data.sh
```

★ 目录本身通过 `.gitkeep` 入库；DB 文件与日志由运行时/初始化脚本产生，🚫 不入库。

★★ 路径口径归属：详见 `docs/17-P5-网关与HMI.md` §5.0 与 CLAUDE.md §0.2。`disk.data_root` 目前定为 `/opt/xbrain_v6/data`（用户 2026-08-05）。
