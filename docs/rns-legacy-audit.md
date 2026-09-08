# RNS 存量审计与处置表（Phase −1 交付物）

| 项 | 内容 |
|---|---|
| 任务 | `RNS_TODO` PM1.1 ~ PM1.5 |
| 日期 | 2026-09-08 |
| 对象 | `xbrain/p1_motion/`（66 个 `.py`，实测）；重点 `rns/`（v0.2 遗产，11 文件 ＋ `tests/p1_motion/test_batch_d_rns.py` 30 条） |
| ★★★ 关键实测 | **旧 `rns/` 完全未接线**：宿主对它零 import ⇒ 处置无手术风险；30 条测试是它唯一的消费者 |
| 反哺 | ★★★ 三处已回写 `20` **v1.13**（围栏滞留洞 · 录制态 RRA 引用 · 两项对偶取代论证），见 §7 |

## 1. 目录级差量表（PM1.1）

| 目录 | 定性 | 处置 |
|---|---|---|
| `rns/` | ★★★ v0.2 遗产（`rt/lidar/grid` 时代） | 见 §2 逐文件 |
| `path/path_follow.py` `target_oriented.py` `nav2_proxy.py` `relative_move.py` | 待删改行为源（#20-1/#20-9 代码半边） | 见 §4 |
| `perception_src/` | 订**旧 key** `rt/perception/targets` | 见 §5 改造蓝本 |
| `sources/`（arbiter_p1）· `gate/`（speed_gate/creep）· `rotation/` · `route/` · `arb/` · `teleop/` · `fence/` · `freshness/` · `runtime/` · `failure/` · `handshake/` · `config/` · `path/` 其余 | **宿主件**（`12` 域），RNS 只消费/对接 | 保留；接口点在 TODO P0.2/P1.8/P4.5/P7.2 |
| `profile/`（switch_sm） | ★ 宿主件：`12` §9.6.4 **档位切换状态机** —— 🚫 与 RNS 的 `ProfileMsg` 消费无关，纯重名 | 保留；RNS 侧感知消费落 `rns/inputs.py`，无冲突 |
| `ctrl_loop.py` `__main__.py` | 宿主主体 | 保留 |

## 2. 旧 `rns/` 逐文件判决（PM1.2 · PM1.5）

| 文件 | 功能（头注 Brief） | 判决 | 理由 |
|---|---|---|---|
| `audit_ring.py` | 单调钟环形审计 | ✅ **复用** | 与 `20` §9.3 完全同构，改造为新 `audit.py` |
| `side_select.py` | 选边 cost ＋ tie-break ＋ 迟滞 | ★ **改造复用** | 判据换 `20` §7.2 三级，tie-break/迟滞骨架可用 |
| `u54_semantic.py` | U54 单向安全距离（停不退 · B 类线性降速） | ★ **部分复用** | 纯函数并入 `dynamic.py`/`classify.py`；语义与 §5.3 一致 |
| `inflate.py` | 随速膨胀 `v_infl = max(\|v\|, v_req)` | 🚫 **作废** | ★ 被 §8.1A **紧度降速对偶取代**（旧：快则余量大；新：余量小则慢 —— 同一安全不等式的两个解向），反哺论证入 `20` v1.13 |
| `corridor.py` | 走廊搜索 ＋ L_min 硬门 ＋ unknown=blocked | 🚫 作废 | 被三态融合 ＋ §6.2 硬门取代；unknown 语义由 A-FUS-1/3 承接 |
| `candidate_gen.py` | 候选 ＋ wz 几何钳（RNS-ROT-1..4） | 🚫 作废 | wz 出口归 `12` §6A（P4.5），候选按 §6 重写 |
| `grid_motion.py` | 栅格运动补偿（odom Δ） | 🚫 作废 | ★ RTK **世界系锚定**（§4.2）使补偿不需要 —— 前提整个消失 |
| `targets_veto.py` | targets 距离折回几何 veto | 🚫 作废 | ★ 被 `11` §3.4A **bit2 语义注入**（感知侧）对偶取代，反哺论证入 `20` v1.13 |
| `module.py` | 骨架 ＋ `RnsSnapshot` 旧契约 | 🚫 作废 | 新契约 = `source.py` ＋ `inputs.py`（`20` §1.2 v1.10） |
| `shutdown.py` | 抑制五分支 ＋ RNS-A3 | 🚫 作废 | ★★★ 五分支分辖：grid_stale→T-50/51 · rtk→N-6 · **fence_edge→v1.13 反哺**（§7） · recording→RRA 引用（§7） · dead→`12` §5.3；RNS-A3（候选非命令）已被 RNS-M-3 覆盖 |
| `__init__.py` | — | 改造 | 随新骨架 |

★ **执行时序（PM1.5 审计修订）**：全部未接线 ⇒ 无需过渡期；**并入 P0.4 首提交**——复用件直接就位新名、作废件迁 `rns/_legacy/` 带墓碑头注、测试同批改址。🚫 先单独迁 `_legacy/` —— 那会让 9 条迁移测试先造出对过渡目录的依赖。

## 3. 30 条旧测试判决（PM1.2）

| 组 | 条数 | 判决 |
|---|---|---|
| `ring`（evicts/copy） | 2 | ✅ 迁移（随 `audit.py`） |
| `u54_*` | 5 | ✅ 迁移（随 `dynamic.py`/`classify.py`） |
| `choose_side`（min_cost/迟滞） | 2 | ★ 改造迁移（判据换 §7.2） |
| `inflate` | 3 | 🚫 作废（对偶取代，墓碑指 §8.1A） |
| `corridor`/`unknown_blocked` | 3 | 🚫 作废（墓碑指 A-FUS-1/3） |
| `veto` | 3 | 🚫 作废（墓碑指 `11` §3.4A） |
| `odom`/`grid_motion` | 3 | 🚫 作废（墓碑指 §4.2 锚定） |
| `wz_clamp` | 3 | 🚫 作废（墓碑指 `12` §6A / P4.5） |
| ★★★ `r_eff_uses_max_of_robot_and_fallback` | 1 | 🚫 **作废并高亮**：它钉的是 `12` v0.7 已**明文作废**的 `max()` 形态（唯一合法 = §6A.4.1 分支式，两式在 `0 < r_robot < 0.60` **不等价**）——留着它 = 一条强迫实现错形态的绿测试 |
| `shutdown` | 3 | 🚫 作废（墓碑指 `20` §9.0） |
| `snapshot`/`reason_string` | 2 | 🚫 作废（墓碑指新 `types.py` 闭集） |

合计：迁移/改造 **9** · 作废 **21**（作废一律墓碑注释注明取代物，🚫 静默删 —— G-1 纪律）。

## 4. 四个行为源的代码侧处置（PM1.3 —— 与 #20-1 文档批同一提交序列）

| 文件 | 处置 | 依据 |
|---|---|---|
| `path/path_follow.py` | 🚫 删（折线跟随归 RNS `route.py`，P1.3/P1.8 接管其路径指针消费） | #20-1 · `RNS-M-6` |
| `path/target_oriented.py` | 🚫 删（并入 RNS `follow_target`，本期预留 #20-13 —— 删源不删预留位） | #20-9 已裁 |
| `path/relative_move.py` | ★ 拆：**导航职责**归 RNS（goto 入口 schema 在 #20-1 批定）；指令 ack/状态机壳保留至 schema 落 | #20-9 |
| `path/nav2_proxy.py` | 保留（spin/backup/wait 仍走 Nav2，用户 2026-09-07 裁定） | #20-9 |

## 5. `perception_src/` 改造蓝本（PM1.4 → P0.2）

现状：`PerceptionSource` 抽象 ＋ 生产 Zenoh 订 `rt/perception/targets` ＋ 测试 `ReplayPerceptionSource` 注入。
改造：① 订阅面换三新 key（`profile`/`objects`/`status`，`11` §2.2.1）；② 快照结构换 `rns/inputs.py` 视图（含 T-50~53 计龄字段）；③ ★★★ **Replay 注入点形态原样保留** —— 它是全部金标场景（M-3）的进料口；④ 旧 `targets` 路径按 `debug.legacy_keys_enable` 思路留 dev 开关或直接删（联调期定，默认删）。

## 6. 命名冲突面（PM1.5）

新旧同目录冲突仅 `__init__.py`（其余 10 旧名与新 13 名零重合，实测）；`p1_motion/profile/` 与 RNS 感知消费重名已在 §1 澄清。⇒ 冲突面极小，支持"并入 P0.4 一次就位"。

## 7. ★★★ 反哺结论（本审计对 20/TODO 的回写，防功能遗漏）

| # | 发现 | 处置 |
|---|---|---|
| 1 | ★★★ **围栏滞留洞**：旧 `shutdown.py` 有 `fence_edge` 分支；现设计里围栏截断前向 ⇒ 无候选无墙，§7.2 会对不存在的墙执行贴墙，且 §7.3A 曾把 fence 排除在计时外 ⇒ **永久滞留无上报** | ✅ `20` v1.13：§7.2 加「存在可贴 BLOCKED 边界」守卫；§7.3A 排除集收窄 {航向, RTK}；M-3 加「围栏截断前向」景 |
| 2 | ★★ **录制态零引用**：`12` §4.7.2a RRA-1~6（部分抑制：仅纵向生效）已裁，`20` 全册未引用 | ✅ `20` §11 补引用行（落 `source.py` 输出裁剪） |
| 3 | ★ 旧随速膨胀、旧 targets_veto 两个机制**看似遗漏实为对偶取代** | ✅ 论证落 `20` v1.13 变更行与本表 §2，防下轮再被当缺口捡回 |
| 4 | ★ RNS-A3 / 五分支其余 / wz 钳 | ✅ 逐条确认已被现设计覆盖（§2 表），无遗漏 |
| 5 | ★ 发现一条**钉着已作废形态的绿测试**（`r_eff max()` 式） | ✅ §3 高亮作废；这是「绿测试 ≠ 对」的又一实例 |

上海哈船智能船舶技术有限公司 · XBRAIN_V6
