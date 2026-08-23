# XBRAIN_V6 · NEXT（未完成清单与推进对账）

> 用途：PB1-8（P3 任务链路接通）之后，把**还缺什么**逐条列清，供 phase 任务开发对账。
> 本文件是**推进对账表**，不是正式设计册（正式真源仍是 00-21 十一册）。
> 建立于 2026-08-12（P3 任务链路 PB1-8 收尾当日）；最近更新 2026-08-20（geo/teach 全链 + HMI 上行 W4/W2/W7 + P4 改发 §7.2 TaskCommand + 录制阻塞链定位）。
> ★ 标注约定：`[GATED-HW]` 卡真硬件/云深处底盘 · `[GATED-DESIGN]` 卡设计未写 · `[SW-NOW]` 纯软件、现在可推 · `[DONE]` 已完成。

---

## 0. 现状快照（已建成的）

- **语音闭环**：MIC→ASR(paraformer)→P4 编排器→TTS，ORIN 实测通。`[DONE]`
- **PTZ / payload 设备控制**：布控球 ONVIF、灯光/爆闪/音量，圆润理解(大类语义路由+前缀消解+后置精修)，ORIN 实测。`[DONE]`
- **P3 任务链路 PB1-8**：语音→cmd/task→task.db(SQLite)落库→调度器 pending→ready→running，状态机对齐 11 S4.4，id 生成、幂等、事务、F 类录制写路径、running→done 完成逻辑。ORIN 实测真落库+真流转。`[DONE]`
- **P2 仲裁 / 模式机 / BIT**、**P5 网关批次 A-E**、**配置冻结线(CFG-FZ)**、**甲方云端翻译/去重/Q0 急停**：主体已建。`[DONE]`
- **LLM 服务**：llama-server + qwen2.5-3b-instruct 在 ORIN 上跑(端口 18082)。`[DONE]`
- **地理要素 CRUD 全链**（2026-08-20）：`cmd/geo` 八 action（P3 单写者）+ `cmd/teach` 录制会话 + P4 的 F01–F15 发起方 + HMI 上行 W4 + HMI 录制显示。ORIN 实测。`[DONE]`（余下卡点见 §4.1 与 §7.0）
- **`health/summary`（P2）与 `state/teleop`（P1）**（2026-08-20）：契约要求的发布者补齐；`state/robot`/`state/power` 归 `chassis_relay`，做不了。`[DONE]` / `[GATED-HW]`

---

## 1. 执行环剩余（PB8 未做完的部分）

> P1 voice-loop 现为 ad-hoc 运动 MVP（单帧 cmd_vel，不执行路径、不报进度/完成）；geo.db 无航点；底盘等云深处。故下列多为 `[GATED-HW]`。

| # | 缺什么 | 状态 | 依赖 |
|---|---|---|---|
| EX-1 | 路径展开：mission_json + 航点名 → geo.db 航点 → total_steps + V-3/V-6 校验 | 未做 | geo.db 有航点(录制或播种) + B 类槽位填充 |
| EX-2 | 派发时发 `cmd/motion/route`+`cmd/motion/behavior` 给 P1 | 未做 `[GATED-HW]` | P1 真路径执行 |
| EX-3 | live 订阅 `state/motion/path_progress` → patrol_progress 全列表 + 进度落盘 | 未做 `[GATED-HW]` | P1 发进度 |
| EX-4 | live 订阅运动状态 → 调 `apply_motion_result`(逻辑已建) | 未做 `[GATED-HW]` | P1 发完成状态 |
| EX-5 | live 录制会话：位姿流累积点 → stop 时调 PB7 的 commit 写路径(写路径已建) | 未做 `[GATED-HW]` | 定位/位姿流 |
| EX-6 | patrol_progress 全列重建(~20 列+唯一 active 索引, 15 S9.5) | 未做 | 随 EX-3 一起(有驱动才建，避免 §9.3 空表) |

---

## 2. C++ / ROS2 机器人层（最大未建块）

| 进程 | 语言 | 状态 | 依赖 |
|---|---|---|---|
| **quadruped**（底盘控制 CHS-A 三通道，13 册） | C++17 | 未建 `[GATED-HW]` | 云深处 M20S 底盘 + 厂商 PDF 实测 |
| **perception**（定位/位姿/障碍/目标，33ms/30fps） | C++ | ✅ **详设已写** / 实现未建 `[GATED-HW]` | ★★ **2026-08-21 订正**：本行原写「详细设计未写 `[GATED-DESIGN]`」是**过时记录**，`docs/19-perception详细设计.md` 已 996 行且章节完整（§2 33ms 预算 · §3 走廊几何 · §9 启动自检 · §11 变异体表 · §13 未定项 · §14 配置键）。⚠️ 这条过时记录**已真实误导过一次判断**（2026-08-20 我据它建议「先写 perception 设计」，当天被 `d2e9ab1` 订正）。<br>★ 接续项是 **SW-2b 实现**，卡相机硬件与 `19` §13 的 PD-3/4/5/10 标定值（按设计**拒绝启动**，🚫 不得为让它起来而实现绕过路径） |
| **chassis_relay**（急停链路 CRL-1..5） | C++ | 未建 | common 地基库 + 底盘 |
| **rtk_driver**（GPS/授时，唯一判 ClockStatus.sync） | **C++** | ★★ **已建**（`ros2_ws/sensor/`，19 个 `.cc`：NMEA 解析 / gnss_heading / clock_status / serial_reopen 全套测试） | ★★ **2026-08-21 订正**：本行原写「未建 `[GATED-HW]`」是**过时记录** —— RTK 链路 2026-08-14 已端到端跑通（`rtk_driver → p1 → state/pose` ORIN 实测）。语言事实上已定 C++ |
| **teleop_input**（遥控 deadman） | 待定 | 未建 `[GATED-HW]` | 遥控器 |
| **behavior_proxy** + **Nav2 behavior_server** + **zenoh-bridge-ros2dds** | C++/Rust | 未建 | ROS2 环境 + Nav2 |

---

## 3. P1 完整运动执行 + RNS

| # | 缺什么 | 状态 |
|---|---|---|
| P1-1 | 真 20Hz 控制环（现仅 voice-loop MVP） | 部分件(gate/failsafe)已建，未接真感知/底盘 `[GATED-HW]` |
| P1-2 | 速度门四段 f(d_free) + 迟滞、路径跟随、旋转门 RCG 全接 | 零散件已建，未串成环 |
| P1-3 | **RNS 避障**（进程内模块，行为源 rns_avoid 900） | ✅ **详设已写** / 实现未建 —— `docs/20-RNS反应式避障详细设计.md` 1018 行（§10 单调钟 · §11 与 `12` 的接口清单 · §13 断言与变异体总表 · §14 实现要点与陷阱 · §15 待确认清单）。★★ **2026-08-21 订正**，原写「详细设计未写」过时。接续项 SW-3b 实现，输入是 `19` 产出的 `bands` |
| ★ P1-4 | **航向丢失恢复：odom 桥接 + 视觉导向重捕 COG**（用户 2026-08-16 定方向） | ★ **设计意图已记，待 quadruped odom + perception + RNS 落地** `[GATED-DESIGN+HW]` |

> ★★★ **P1-4 设计意图（用户 2026-08-16 · 航向恢复,不逼停）** —— 背景:双天线航向(L1,绝对,静止可用)突然丢时,现设计运动态已无缝切 COG(L2,`11` §3.3,不停车);但**静止 / 原地转向态**下 COG 物理上无解(无运动=无航迹),现状进 L2-blind(保持旧航向但 `heading_valid=false`)。用户方案分两级补:
> - **① odom 桥接(治静止 + 转向)**:quadruped 的里程计 yaw **静止和原地转向都有效**(正是 COG 做不到的两工况),把丢失瞬间的 GNSS 绝对航向当锚点 + 叠加 odom yaw 增量 → 静止也有效、且**知道机器人转没转**(解掉"保持旧航向但机器人转了就错"的风险)。航向源链变为 `L1 双天线 → odom 桥接(相对锚定,慢漂,有界时间)→ L2 COG → L3`。
> - ★★★ **这不是新设计 —— 契约 `11` §3.3.2a 已把它定为 `L1.5 odom_aligned` 级**(2026-07-30 裁决,夹在 L1↔L2,`yaw_capable=true`):`heading_enu = wrap(odom_yaw + north_offset)`,`north_offset = wrap(abs_heading − odom_yaw)` 由 L1/COG **持续 EMA 标定**(NO-1 `alpha=0.08`)、**静止时保持**、**转弯时冻结**(NO-3 用 1.5s 窗**最小二乘斜率** > 0.06 rad/s 判转弯,🚫 不用瞬时角速度 —— 四足每步左右晃 ±7° 会被误判);准入 `unc ≤ 0.30 rad`(≈17°)且 odom 新鲜。★ `11`:3823 明写它正好**接住"减速停稳 → COG 失效"那一下** —— 即前面聊的 L2-blind 停车态(也就是 HMI 现在会闪红 LOSS 的那个边界,L1.5 落地后自然消失)。
> - ★★ **为什么 resolver 现在没实现它(我特意留空)**:两个硬缺口 —— (a) **G-15**(`11` §14.4):L1.5 的 `i_heading` 与**硬速度上限**两格**至今空白**,须与 `odom.yaw_drift_dps`(NAV-96/97)一起标定,§3.1 禁止代码编默认;(b) **Q-U34-1**(`11`:3843):`rtk_driver` 需订阅 odom(`rt/chassis/state` 的 `odom_yaw` + 机体 `vx`),而 quadruped 未建。★ resolver 已留干净缝:`heading_resolver.h` 头注逐字写明"不实现 L1.5,消费方只见 L1/L2/L3",两缺口一补即可接上,不推翻现状态机。
> - **② 视觉导向重捕 COG**:odom 漂太多 / 需刷新绝对航向时,perception 找**无障碍方向**,P1/P3 控制机器人**朝该方向挪一小段** → 拿到 COG → 立刻回高精度航向、恢复任务("立刻开始任务")。
> - **分工**:odom 数据归 `quadruped`(GATED-HW);无障碍方向归 `perception`(GATED-DESIGN);"要不要挪 + 挪哪"的决策与执行归 P1/P3;航向 resolver(rtk_driver)只**报状态**(L2-blind/L3),需给它**加一路 odom 输入** + 实现契约**已定义的** `11` §3.3.2a `L1.5`(🚫 不是"给契约加级",是填 G-15 两格 + 接 odom 输入)。
> - **落地必带的护栏**:(a) odom 桥接**最长时长 / 最大累积 yaw 漂移上界**,超了强制去拿 COG 或退 L3,不可无限信 odom;(b) 桥接精度受**丢失瞬间锚点新鲜度**约束(锚点 cov 大则桥接也差);(c) odom 自身异常(打滑 / 腿部估计坏)要能识别并退 L3。
> - **依赖顺序**:必须在 **quadruped(odom)+ perception(无障碍方向)+ RNS** 三者落地之后做;三者任一 GATED 时本项不可动。

---

## 4. 真硬件集成

- M20S 底盘（**等云深处**）、RTK、LiDAR、可见光/热成像相机、遥控器。`[GATED-HW]`
- 集成测试三档（不需真设备 / 需 ORIN / 需真底盘）框架**未建立**。

### 4.1 ★★★ 录制（示教）端到端的硬件阻塞链（2026-08-20 · ORIN 实测逐门定位）

> ★★★ **软件侧已全部建完并实测**：`cmd/teach` 会话（§12A.3 状态机 + §12A.6 采点 + §12A.7 几何校验 + §12A.8 单点录制）、
> P4 的 F01–F10 发起方、P5 的只读显示与 HMI 轨迹渲染，全部上线且有变异体守护。
> ★ **卡的不是代码，是四样硬件/数据源**。本节把 ORIN 上实测出的**逐门拦截顺序**记下来，
> 供硬件到位后逐条销账 —— 每解决一项，录制就往下推进一道门。

**★ ORIN 实测（2026-08-20，发 `cmd/teach{action:"start"}` 逐次观察 ack）**

| 序 | 拦在哪道门 | 实测 ack | 解锁需要 | 状态 |
|:--:|---|---|---|---|
| ① | §12A.3 **状态源缺失** | `E_TEACH_QUALITY` `state_unavailable` `missing:[state/robot, state/power]` | **`chassis_relay`（C++，CR-4/CR-5）+ 真底盘** | `[GATED-HW]` |
| ② | §12A.3 **检查 3 定位质量** | `E_TEACH_QUALITY` `{fix_type:"single"}` | **RTK 基站 / NTRIP 改正**（现为单点 GPS，米级） | `[GATED-HW]` |
| ③ | §12A.3 **检查 4 `allow_motion`** | `E_UNHEALTHY` `health forbids motion` | ★★★ **RGBD 相机 + `perception` 发 `cam_rgbd` 健康** ← **当前终点** | `[GATED-HW]` + `[GATED-DESIGN]` |
| ④ | §12A.3 **检查 7 非语音急停通道** | 未到（③ 先拦） | `teleop_input`（手柄/键盘）—— 缺它判据①永假，只能靠判据②的 `cmd/estop` 链路 | `[GATED-HW]` |

**★★★ ③ 为什么是硬拦，且【不能】用开关绕过**

`14` §8.3：`cam_rgbd` 不正常时连 `obstacle_avoid` 档都不准入 ⇒ `allow_motion = false`。
而 `cam_rgbd` 在本 build 里**没有生产者**（`perception` 详设未写、进程未建），按 §3.2 只能报 `unknown`，
🚫 不得报 `ok`。⇒ 录制被拒是**正确行为**：录制期横向避障与语音急停都被抑制（§12A.3 / U45），
**没有避障感知就不该被遥控着跑**。
★ 🚫 **绝不为此加任何「跳过安全断言」的开关**（§3.6：那等于一条远程解除全部安全约束的通道）。
★ 也 🚫 **不得让 `p2_core` 把 `cam_rgbd` 默认成 `ok`** —— 那是 §3.2 形态①「一条永远绿的断言」。

**★ 已可验证到什么程度（不必等硬件）**

| 层 | 手段 | 结论 |
|---|---|---|
| 会话逻辑 / 采点 / 几何校验 | 单元 + 变异体（`tests/p3_task/test_teach_core.py`、`test_teach_runtime.py`） | ✅ 含 start→采点→mark/undo→finish→save 端到端落库 |
| P3→P5→浏览器 数据链 | `/tmp/teach_state_stub.py`（**只替代 P3 的 §12A.5 广播，🚫 不碰任何安全门**） | ✅ ORIN 实测每秒新 `seq` 到浏览器 |
| HMI 渲染 | in-app 浏览器 `javascript_tool` 读 DOM | ✅ 徽标「录制中 / 96 点」+ `teachLayer` 46 图元；停桩后归 0 |
| **真实录制** | — | ❌ **卡 ③**，桩只能证明显示层通 |

> ⚠️★★★ **销账顺序 —— 2026-08-20 当日订正（原写法基于过时信息，已作废）**
>
> ★ **原写什么** ＝ 「③ 前置 SW-2 perception **详细设计**（`[GATED-DESIGN]`），最短路径是先写详设」。
> ★★★ **为何不成立** ＝ 实测 `docs/19-perception详细设计.md`（996 行）与 `docs/20-RNS反应式避障详细设计.md`（1018 行）
> **都已写完**：章节完整，含 §11 变异体表、§9 启动自检、§14 配置键一览、§13「本册确实未定的」分类登记。
> ⇒ SW-2 / SW-3 的「设计未写」状态是**过时的**（见下方订正），CLAUDE.md 里「perception ⚠️ 详细设计尚未编写」同样过时。
>
> ★★ **现行结论**：③ 的前置是 perception 的 **C++ 实现**，不是设计。而该实现**自身还卡两层**：
> · `19` §13 **PD-3 / PD-4 / PD-5 / PD-10** 的配置值未标定 ⇒ 按其设计 **perception 拒绝启动**（fail-loud，🚫 设计明写不实现绕过路径）；
> · 这些值是**实测量**（T7 / 云深处），没有实机就没有数。
> ⇒ ★★★ **写完 perception 代码，在没有相机与标定值的机器上它也起不来** —— 这不是缺陷，是 `19` 自己选的失效方向。
> ★ 因此 ③ **确实是硬件闸**，🚫 没有「先写点什么就能解锁」的捷径。

---

## 5. 纯软件、现在可推（不卡硬件）`[SW-NOW]`

| # | 项 | 价值 | 备注 |
|---|---|---|---|
| ~~SW-1~~ | ~~LLM tier-2 接线~~ | **`[DONE]` 2026-08-12**（44c752b/cee733c）：mission_select 候选选择 + build_tier2_fn 组装 + 槽位回传契约 + grammar 收紧(slots 闭合对象)。ORIN 实证:`往前挪三米`->move_forward{distance_m:3}、`把当前位置记为集合点`->record_waypoint{name:集合点}、播报->speak_custom{text} | — |
| ~~SW-2~~ | ~~**perception 详细设计编写**~~ | — | ✅★★★ **2026-08-20 订正：详设【已写完】** —— `docs/19-perception详细设计.md` 996 行，章节完整（§2 33ms 预算分解 · §3 走廊几何 · §11 变异体表 · §9 启动自检 · §14 配置键 · §13 未定项分类）。★ 本行原状态「未做」是**过时记录**；CLAUDE.md 的「perception ⚠️ 详细设计尚未编写」同样过时，⚠️ **归用户裁定是否同批订正**（本册不代改 CLAUDE.md）。<br>★ **接续项改为 `SW-2b` perception 的 C++ 实现**（见下行） |
| **SW-2b** | **perception C++ 实现**（按 `19` 落码） | 高：`cam_rgbd` 的唯一生产者，是 §4.1 ③ 的总闸 | ★★ `[GATED-HW]` + ★ 卡标定：`19` §13 **PD-3/PD-4/PD-5/PD-10** 未标定 ⇒ 按设计**拒绝启动**（fail-loud），且都是实测量（T7/云深处）。⇒ **代码可以先写，但没有相机与标定值就起不来**，🚫 不得为「让它起来」而实现绕过路径 |
| ~~SW-3~~ | ~~**RNS 详细设计编写**~~ | — | ✅ **2026-08-20 订正：详设【已写完】** —— `docs/20-RNS反应式避障详细设计.md` 1018 行（§10 单调钟 · §11 与 `12` 的接口清单 · §13 断言与变异体总表 · §14 实现要点与陷阱 · §15 待确认清单）。接续项为 **`SW-3b` RNS 实现**（在 `p1_motion` 进程内，`19` 产出的 `bands` 是其输入） |
| SW-4 | 云端/HMI 完整上行（P5 端到端云协议） | 中 | 需甲方云端联调 |
| SW-5 | 集成测试三档框架建立（docs + harness） | 中 | 现无落点 |
| SW-6 | 配置落值：common.db.* 四库路径 + 其余 null 安全参数标定 | 低-中 | 按 §3.1，标定即启动断言放行 |
| SW-7 | 字符集存量债清理（task #46，24 漏网符号 + charset_lint 完善） | 低 | housekeeping |
| SW-8 | 充电/对接**执行**串联（状态机+仲裁器已建） | 中 `[部分 GATED-HW]` | 三级选桩逻辑可测，真对接需底盘 |
| SW-9 | **全系统圆润扩展**（非设备类的同义说法加宽） | 中 | ★ 用户早先定「聚焦 PTZ/payload，后面慢慢扩展到整个系统」的那一批。2026-08-12 已收口薄意图（1词无兜底）：C08/G09/H07/H08/F13 加同义说法 + 18 对齐 + 变异守护。★ **剩 47 个恰 2 词且无 tier-2 的意图**（多为 G 类只读查询 L0），是下一批候选；现状不是缺陷（各有 2 说法 + §2.2 边界节 C 已载「贴 keywords 说」），扩不扩看优先级 |
| SW-10 | **comment_ratio 注释债**（xbrain/ 约半数文件 < 70%） | 低-中 | ★★★ **2026-08-16 用户裁决：作为负债留到最后做【全系统语音集成测试】时一并完成，在此之前门禁 `test_lints::test_comment_ratio_holds` 保持红，是【已知已记的债】不是回归**。★ 规模（§3.7 不烤死数字，跑命令得准数）：门禁只算 `xbrain/`（`test_comment_ratio_holds` 断言 `LOW ... xbrain/` 零命中）；`python3 scripts/lint/comment_ratio.py \| grep -c 'LOW.*xbrain/'` → 2026-08-16 实测 **239** 个不达标（分布 p4_agent 57 · p2_core 50 · p1_motion 43 · p5_gateway 37 · p3_task 37 · boot 8 · common 7；旧记「498」是含 scripts/tests 的全仓数，已作废）。★ 成因：§2.4 阈值 08-06 从 25% 上调 70%（= 注释行 ≥ 2.33× 代码行）后，P1-P5 批量业务代码普遍在其下累积。★ 修法：按 §2.4「每块解释 why、不刷百分比」分批补真注释——★★ 这不是进度审计，是注释密度尺，别当「补完才算开发完成」。已开 chip（task_3e48348d）。★ 关联教训：yaml 头/字符集门禁盲区（configs/.yaml 从没被扫）已于 2026-08-12 关闭（ca08aaa/1bc9702） |
| SW-11 | **`hmi.bind[0]` LAN2 地址落值**(现 null) | 低 | ★ full 启动 `check_p5_config` 因 LAN2 bind 为 null 而**拒启**(§3.1 设计行为, 报 `hmi.bind[0] unassigned`);voice-loop MVP 走宽松 `make_bound_sockets` 只绑非空口(wifi `192.168.1.7` + `127.0.0.1`)故能跑。等 U-15 部署分配 LAN2 网段地址即落值解除。★ `bind[1]` wifi 已填(2026-08-12 用户明令),`bind_guard` 测试已对齐(f7803c9) |
| ~~SW-12~~ | **事件存证链路** | — | ✅★★★ **2026-08-17 落地上线(7 批, 287 测试, ORIN 实证)** —— commits `0881fc1`(批1 record.db DAO 按 17 §3.4 权威 schema, 两写一读三连接, ch_seq/dedup/need_ack/JSONL 降级)· `7f129b9`(批2 7 阶段 pipeline + §6.2 channel 推导, 替换错模型占位)· `03b2796`(批3 backfill: 令牌桶限速 20eps + 4:1 加权 + EventReplay 消息)· `887ac3a`(批4 uplink: DeliveryMarker + AckTracker + BackfillRunner)· `2e7f08e`(批5 EventSubsystem 同步/async 桥接进运行 p5, degrade-safe)· `8fcd255`(批6 device 掉线事件 build + debounce 监视器)· `9b46af9`(批7 ORIN 端到端: 真 p5 重启带 XBRAIN_RECORD_DB, 注入 live 事件正确落库 channel/ch_seq/delivered, e2e_check.py PASS). ★ **剩余(非本子系统, 各有卡因)**:① **3 个 device 产生方** —— ✅ **2026-08-17 接线** `DeviceHealthBridge`(p2, 复用 device_events)+ p2 事件发布器(gen.put event/{sev}/{cat}): **MIC 真+已端到端实证**(杀 arecord→cap_alive=False→debounce→`device mic offline`→p5 record.db 落 voice/device_offline); **payload 真**(轮询 payload-service `GET /status` 的 `device.{audio_connected,lights_connected}`=8519/8529 socket, 连通已验证无误报; 真掉线要 GZH-2 socket 断 `[GATED-HW]`); **ptz 真**(2026-08-17 `PtzLivenessProbe` 非阻塞 ONVIF 探测线程 commit `d41efbd`; 三态 up/down/auth, auth 首次即停防锁账户 per docs/PTZ 报告 §8; 真机 192.168.66.13 可达实测不误报);② **实时上云** `[Q-P5-8 ✅ 2026-08-17 决定 A]` —— 云端放宽实时订阅至 `event/{warn,fault}/**`(含设备掉线), 产生侧本就直发无改动, exact 通配甲方 SW-4 落定(commit `1e4abca`)。★★ **断连兜底已补**(批A `d635a70` + 批B `663b3c2`): 批A 重连触发补发(初版 LinkReconnectDetector, 已被批C 收编删除); 批B recon 对账协议(P5 周期发 `event/recon/req{my_max,my_min}`→云端 `rsp{their_max,missing_ranges}`→差集经 `event/replay/{channel}` 重发, `rc-` 批前缀 RC-2, 共享限速器 RC-4, my_min 钳制防无休止对账); **批C `e6e7934` 11 §4.6 LinkState 状态机**(P5 唯一权威 LNK-6): 单调钟 disconnected_s + L0/up→L1/degraded(≥5s)→L2/down(≥20s)→L3(≥rtb_s, rtb_s=None 停用返航 fail-safe)+ LNK-3 滞后(flap 不重置计时)+ LNK-5 冷启动 never_connected 不视为 up + link_epoch(返航幂等)。p5 发 state/link 全字段; snapshot.reconnected 边沿驱动 backfill(收编批A); 修 DeliveryMarker connected 读真 cloud_link。ORIN zenoh_echo 实测 cloud_link:down/level:2/disconnected_s 单调累加/reason:never_connected。**批D `d9ae9f3` P3 断链返航闭环(F-5 / 11 §4.6.4)**: P5 侧 `rtb_s` None→**1800s(30min, 契约建议值; 用户 2026-08-17 拍板临时值, 仍属 U-05 待甲方终确认)**解锁 L3; P3 订 state/link, level==3 按 (gw_start_mono,link_epoch) 幂等入队 return_home(source=charge/prio95, 15 §4.2.1), 实现 failure.py 早设计但未接线的 F-5 inject_return_home。ORIN live e2e: 发合成 state/link{level:3}→p3 注入 return_home t-...004 落 task.db(state=ready), 幂等只一条。★ **L2 按来源拒新任务**(TSK-22)未做(下游, 无真云时空转); reason gateway_restart(需重启持久标记)/transport_error/router_down(需底层 zenoh 信号)+ last_rx_ts 显示字段 deferred; 待真云 SW-4 端到端联调。★ **2026-08-17 架构一致性审计闭环**: U18b 落案(need_ack 并集 `8d0224d`)· F9 修 EventAck result 闭集 ok/duplicate(`2edf261`, 原误用命令 Ack 的 accepted 会让 need_ack 事件永远重发)· F3 device detail 补 reason/socket + F8 eid 防跨重启碰撞(`ad89df4`)· 死代码清理 chip(错模型 backpressure/recon orphan);③ **甲方真云端 endpoint** `[SW-4]` —— uplink/backfill P5 侧全建全测(对 loopback stub), 只剩指向甲方;④ **`common.db.record_db` 落值** `[SW-6]` —— 现走 XBRAIN_RECORD_DB dev 覆盖, 配置落值即转正 |
| **SW-13** | **事件产生方补全（23 类 + 媒体 + 游标审计）** | 中 | ★★★ **2026-08-17 审计**：事件"管道"(SW-12)建完, 但 23 类事件产生方大多未接线.<br>✅ **已接并 live 实测(4)**：`voice`/`payload`/`ptz` 的 device_offline/online(SW-12)· **`comm`**(批E `7ef0612`: p5 LinkState level 转换→event/{sev}/comm §4.6.8, live cloud_up 实证)· **`task`**(批G `2818307`: p3 scheduler on_transition→event/{sev}/task §6.2, live return_home rh-1-1→ready→accepted 实证)· 批F `a806708` 修 return_home task_id 为 15 §4.2.1 `rh-{gw}-{epoch}`(持久幂等).<br>🔒 **卡"子系统没在 MVP 跑"**：<br>　★ **`rtk`**(2026-08-17 投查纠正: 原判"能做"是错的)—— §3.3.4 rtk 事件 `action_taken` 必填, 闭集只有 `stop_and_suspend`/`stop_and_hold`/`teleop_only` **全是"已停车"值无"未动作"**; 这些停车是 p1 RL-1..8 行为(停自主运动+任务 suspended+声光), 而 p1 MVP **只跑 gnss→pose 桥不跑 20Hz 控制环**(ctrl_loop/speed_gate 在模块里但 MVP 不跑). 现发 rtk 事件只能假填机器人没做的 action_taken -> 违反 §3.2. **需先接 p1 rtk-loss 行为(RL-1..8)才能诚实发 rtk_lost**; heading_degraded/recovered 共用"全停车"action_taken 闭集属契约歧义(§9.1 待澄清).<br>　`mode_change`/`arbitration`(p2 不跑 mode/arbiter, common/arbiter/audit.py 零调用)· `health`/`bit`(p2 health 不在 MVP)· `charging`/`geo`(p3 不跑)· `fence`/`speed_limit`(p1 不跑)· `system`(approval 需 p3 审批, negative-age common/envelope/age.py 零调用)· `teach`/`teleop`/`data`.<br>🚫 **卡硬件/未建**：`intrusion`+`perception`(perception ⚠️未写, cls_permissive.py 零调用)· `chassis`(真底盘等云深处).<br>📦 **基础设施缺口**：① 媒体事件 §3.6/EVT-15(media_json 列 + reference.py helper 在, 但零产生方设 ev["media"], §5.0.2 delivery 表 DDL 都没建)· ② confirmed_upto 游标推进(DAO 有 advance_confirmed_upto 但 runtime 零调用, ack 只翻 delivered 标志, 游标停在种子 0)· ③ deferred comm: link_timer_reset(需重启标记)/ rtb_triggered(需 P3 能量 action/reason_detail + task_id 协同).<br>★ **接线范式**: 纯 helper(cat→sev/detail)+ runtime gen.put event/{sev}/{cat}(eid boot-unique)+ p5 event/** 自动持久化. |
| ~~**SW-14**~~ ✅ **2026-08-20 完成（批 14/15/16）** | ★★★ **P3 的 `cmd/task` 接收端对齐 `11` §7.2 `TaskCommand`** | 高（解锁 HMI W2/W7 + 云端转发任务） | ★★ **2026-08-20 查证的第五处「实现与契约分叉」**：P3 只认 P4 私有形状 `payload['task_request']`，不认契约 §7.2 的 `{action, task}`；control 类四动作（cancel/pause/resume/clear_queue）**零实现**；且 **P3 不发 `cmd/task/ack`**。<br>★ 三件一起做：① 认 §7.2 信封（五 action 闭集）② control 动作驱动 `machine.py` 已有的转换 ③ 发 `cmd/task/ack`。<br>⚠️ **连带待裁决**：P4 是否同步改发 `TaskCommand`（不改=两个真源；改=动已跑通的语音链路）。见 §7.0 |
| **SW-16** | ★★★ **P2 的两个接收端接线 + P4 的 C/H 类路由订正** | 高（28% 的语音指令集靠它） | ★★ **2026-08-21 审计发现：128 条意图里 36 条是哑的**，根因是 P2 的接收面从未接线（`p2_subscriber.py` 里那句 `cmd/motion/intent` 订阅是**示例模块**，真跑的 `main_wiring.py` 只订 5 个 state 话题 + speak/payload/ptz）。<br>✅ **C 类模式已完成（批 17，ORIN 实测）**：P2 接 `cmd/mode`（`ModeFace` → 已有的 `dispatch()` 六动作闭集 → `ModeStateMachine` 真换态 → 发 `cmd/mode/ack` + `state/mode`），P4 逐 id 覆盖 C01/C02/C03/C04/C05/C07 → `cmd/mode` 并新建 `mode_request.py` 构建 ModeCommand。<br>⚠️ **本批【故意不接】两条，需你裁决**：<br>　· **C06 `standby`** —— `18` 效果列是「**P3 挂起任务 + P1 hold**」，效果在 P3/P1 不在 P2。它落在 C 类里但不是模式命令，走哪条 key 要定。<br>　· **C08 `query_mode_switch_ok`** —— `18` 标「**查询类(预检)**」L0，操作员问的是「现在能切到喊话吗」。**把提问翻成 ModeCommand 就是替他切了**，🚫 不能顺手接。需要的是一个「模式切换预检」查询通道（P2 侧答，不换态）。<br>✅ **A 类已完成（批 18，ORIN 实测）**：P2 接 `cmd/motion/intent`（G-1~G-11 十一道闸门 + S9.3.2A.4 轴符号换算 + MO-1 换新 `rm-` id + MO-2 参数一律 P2 填 → 发 `cmd/motion/relative_move`），P4 新建 `motion_intent_request.py` 构建 S9.3.2A.3 报文（含 S3.0 信封）。★ **A13 `set_speed_profile` 改走 `cmd/mode`** —— 它在 A 类里，但 §7.3 把它定为 ModeCommand action，§7.3.1（D-04）明确拒绝为它新开 MotionCommand。<br>⚠️ **A 类里另两条待裁决**：**A04 `hold`**（`18` 效果列「P1 `hold` 行为源」，不在 §9.3.2A.4 八值闭集内）· **A14 `set_gait`**（「P2 → quadruped 模式三元组」，也不在 §7.3 六动作闭集内）。两条现仍按前缀落 `cmd/motion/intent`，会被 G-2 拒 —— 是**如实拒绝**不是静默丢弃。<br>✅ **B 类与 H 类已完成（批 19）**：<br>　· **B05/B06/B07** → `cmd/task` 五动作，**task_id 在发起方解析**（读 `state/task.active_task`）。§7.2 禁的是【接收方猜】不是【发起方解析】——帧里写死具体 id，P3 若发现那条任务已变会回 `E_TASK_STATE` 而不是默默暂停另一条；拿不到活动任务就口头说「现在没有正在执行的任务」，🚫 不发 task_id 为空的帧。<br>　· **B12 `stop_follow`** → `cmd/mode` `set_behavior:normal`（`18` 效果列「退出目标导向行为」是运动**行为**不是任务动作；用 cancel 代替会结束操作员还在跑的整条巡逻）。<br>　· **H01/H02/H03/H05/H06/H07/H08** → **`cmd/system`**（`11` §7.15），原 `"H": CMD_TASK` 与 C 类同种错。<br>⚠️ **两条如实不接**：**B10 `skip_waypoint`**（§7.2 五动作里**没有 skip**，映射成 cancel 会结束整条任务）· **H04 `reload_config`**（`18` 逐字「🚫 不进 `cmd/system`」，走 `cmd/config` §7.6 的 ConfigCommand，是另一个消息体）。<br>★★★ **H 类接收端三处全缺**：`11` §2.2.3 按 action 把 `cmd/system` 拆给 `p5_gateway`(reboot/shutdown/time_sync/generate_report) · `p2_core`(sleep/wake) · `p2_core:bit`(run_bit)，全仓**零订阅者**。⇒ H 类改完路由后**仍不生效**，但已是「发在正确 key、形状正确、等订阅者」而非「发在错 key 被 P3 主动丢弃」——这两者对操作员一样，联调时完全不同。<br>★ **A 类端到端的下一道墙是硬件**：G-1~G-4 已在 ORIN 真总线上逐门实测（25 m 在 G-3 带 `limit` 拒、`L1b` 在 G-2 拒、无 `state/clock` 在 G-4 保守拒），G-5 正确点名 `battery`（fatal，因 `chassis_relay` 未接线无 `state/power`）。放行侧要 `allow_motion=true`，而 battery/chassis/cam_rgbd 三项都卡硬件 —— 🚫 不得为看到转发而伪造健康度。 |
| **SW-15** | ~~W2 `goto`~~ ✅ / ~~W7 `task`~~ ✅ / **W3 `exit_broadcast` 仍未接** | 低（本身很小） | ★ W2/W7 已于 2026-08-20 接完并 ORIN 实测（见 §7.0）。**剩 W3**，仍等 P2 的 `cmd/mode{exit_broadcast}` 接收端 |

---

## 6. 建议推进顺序（软件侧）

1. ⚠️★★★ **~~SW-2 / SW-3 详细设计~~ 已于 2026-08-20 查明【早已写完】**（19 / 20 各约 1000 行，含变异体表与启动自检）。本条原判「最高杠杆、纯设计活」**作废**。★ 接续项是 **SW-2b / SW-3b 的实现**，而 perception 实现卡 `19` §13 的 PD 标定值与相机硬件 ⇒ **不再是「不卡硬件」的那一类**。
2. **SW-1 LLM tier-2** —— 让语音理解在非设备类也圆润，软件闭环、LLM 已就绪，是语音 UX 的自然增量。
3. **SW-6 配置落值 / SW-5 测试框架 / SW-7 字符集债** —— housekeeping，随时可插。

> 硬件/云深处到位后，再推第 1-4 节（执行环 + C++ 层 + P1 运动 + 硬件集成）。

---

## 7. HMI web server 接线剩余（2026-08-12 · 17 §6.10）

> ★ 已建成:HMI web server 骨架 + 数据读取方法(17 §6.8 A-F 投影)+ 客户样式前端(格栅 1m/字体/尺寸/滚轮/连线样式/标记形状全配置化)+ ESTOP 按钮,已接进 p5_gateway voice-loop 路径,绑 `192.168.1.7:18083` + `127.0.0.1:18083`(逐口, NET-C9)。浏览器 `192.168.1.7:18083` 可看外壳。
> ★★ **已接**:state/task -> 计划、state/link -> 状态/ESTOP、ESTOP -> `cmd/estop`;**W1/W2/W3/W8(2026-08-12 · commit 见下)**:cmd/fence -> 围栏、event/** -> 事件流、state/mode -> 模式、`/api/fences/active` + `/api/events` 端点。**其余按下列待补**,现状源缺前端置灰(不造假, §3.1/3.2)。

| # | 缺什么 | 状态 | 依赖 |
|---|---|---|---|
| HMI-W1 | 围栏/报警区几何:订阅 `cmd/fence` + `FenceCache` 喂 provider; `/api/fences[/active]` 接缓存 | ✅ **已接** | ★ 数据管路通(实测发 cmd/fence -> /api/fences/active 200); 上**地图落点**仍需 enu_origin(W4) |
| HMI-W2 | 事件:订阅 `event/**` -> 近期环(EVENT_RING=50)喂 events_group + `/api/events` | ✅ **已接** | ★ 实测发 event -> /api/events 返真事件; 地图**红点落位**需 pose 打坐标(W4) |
| HMI-W3 | 当前模式:订阅 P2 `state/mode` 喂 status.mode | ✅ **已接** | ★ 实测发 state/mode=patrol -> snapshot.status.mode=patrol |
| HMI-W4 | **位姿/GPS/ENU/航向/速度/实时轨迹/RTK/精度; 且 W1 围栏/W2 事件的【地图坐标落点】依赖 enu_origin** | 未接 `[GATED-HW]` | perception(设计未写)+ rtk_driver(未建)+ quadruped(云深处) |
| HMI-W5 | 真端到端 ESTOP:§6.3 estop 探活喂 estop_path;§6.4 专用 <=10ms 快路(P-1) | ✅ **探活已接** / 快路 `[GATED-HW]` | ★ **探活机制已落**:`EstopProbe` 状态机(estop_probe.py)+ 每拍 `probe/estop/ping` -> 收 `probe/estop/pong` -> RTT/连续无 pong -> ok/degraded/down(11 CR-2/CR-3 · T-23/T-24)。★★ **无底盘时诚实报 "down"**(按钮置灰),🚫 不再恒 "ok" 造假。★ 剩 §6.4 <=10ms 专用快路(P-1)与 pong 权威源都要 `quadruped`/`chassis_relay`(云深处) |
| HMI-W6 | WS 推送 state_snapshot + state_delta 增量 | ✅ **完整** | ★ /ws 端点(push_hz 可配), 前端 WS 主、REST 轮询兜底。★ 依赖 **wsproto**(uvicorn 0.52 与 websockets 16.x 服务端不兼容, 用 wsproto 后端; 缺则回退 auto+REST)。★ 修了 `from __future__ annotations` 致 FastAPI 把 ws 参数误判查询参数的 403 坑。★★ **state_delta 已接**(2026-08-14):连接发全量 keyframe, 之后每拍只发变化的顶层组(geo/pose/plan/status/events), 静默拍发空 delta 作 keepalive, 每 30 拍周期 keyframe 自愈; 前端 group-level 合并。ORIN 实测:连接 keyframe -> 静默空 delta -> 注入 state/task 后单拍 `keys=['plan']` 只带变化组 |
| HMI-W7 | 计划目标点有序表 + 逐点勾选 + 进度 2/N | ✅ **映射已接** / 数据 `[GATED-HW]` | ★ **映射已修**:`_extract_active_tasks` 从 P3 `{schema,active_task:{task_id,state}}` 抽取扁平 task 喂 `_plan`(旧 MVP 把整信封当计划 -> state/targets 全落 None 空白卡)。前端 `state==running` 驱动黄色实时轨迹显隐。兼容未来 1Hz 心跳列表(current_step/total_steps)。★ 剩目标点有序表 + 进度分数需路径展开(EX-1):geo.db 航点 + P1 真执行上报(云深处) |
| HMI-W8 | 端点集对齐冻结契约(17 §6.5 == 11 §12.2) | ✅ **端点面已齐** | ★ **全部 §6.5 只读端点已上**:`/api/routes` `/api/docks` `/api/health` `/api/bit` `/api/metrics` `/api/approval/pending`(加上已有 fences/fences_active/events)。★★ **诚实可用性**(ORIN 实测六端点全 200):`/api/health`+`/api/bit` **订阅 P2 `health/factor`/`health/bit` 已接、中继链路已证**(注入 health/factor -> available:true 原样直透, G-2 同源),但 **P2 voice-loop MVP 尚不发 health/**(那是 P2 全设计行为), 故当前 available:false;routes/docks(geo.db 卡)/metrics(遥测聚合器未实例化)/approval(L3 队列无喂入)一律 available:false 不造假。★ 剩 `/api/geo/manifest`(§12.2 新规范式, routes/fences/docks 作兼容别名)与上述源真正产数 |

> ★ **纯软件部分全部完成**:W1/W2/W3/W5(探活)/W6(含 state_delta)/W7(映射)/W8。剩余全部卡硬件(云深处底盘/rtk_driver/perception):W4 位姿全片(总闸,也解锁 W1/W2 地图落点)· W5 §6.4 <=10ms 快路 · W7 目标点表+进度(EX-1)。

#### 7.0 ★ HMI 上行（`11` §12.1.1 的 W 表 —— 与上面的 `HMI-W*` 是【两套编号】）

> ⚠️★★★ **先分清两套 W**：上表的 `HMI-W1..W8` 是**本册自造的接线进度编号**（W4=位姿、W6=WS 推送…）；
> `11` §12.1.1 的 `W1..W8` 是**契约的上行可写类**（W1=estop、W4=geo、W7=task…）。**两者毫无对应关系**。
> 本小节只讲契约那一套，逐条写 `11 W*` 以免再混。

★ **背景**：HMI 的 WebSocket 此前**只下行**。2026-08-20 接上行半边（commit `26fdcb0` / `6db7041` / `adbd290`）。

| `11` W# | 类 | 状态 | 说明 |
|:--:|---|---|---|
| **W1** `estop` | 急停 | ✅ **已接**（REST `POST /api/estop`） | ★ §12.1.1 明定它是全表唯一例外：**旁路 schema 校验、旁路限流、旁路降级**；WS 侧不再重复实现，避免两条 estop 路径 |
| **W2** `goto` | 点击导航 | ✅★★ **P5 侧已接并 ORIN 实测** / 前端点图待接 | ★ `waypoint_id` 与 `lat`+`lon` 二选一（**两者同时给以 `waypoint_id` 为准**，§12.1.1 W2 明写的优先级，🚫 不是"拒绝歧义"）；落成 `cmd/task{submit, task.type:"goto"}`（🚫 **不发 `BehaviorCommand`** —— 发布者闭集只有 p2/p3，且会绕过 P3 围栏前置校验与 U07a 断点账本）。<br>★★ **退役的 `speed_profile` 一律拒不降级**：`cruise`/`transit` 已被 U33 删除，回 `E_SCHEMA`（§13.6 ③ 禁"就近解释"；降级成 `patrol` 会让停在旧词表的前端两侧都看不出错）。<br>★ **ORIN 实测**：WS → P5 → `cmd/task` → P3 `accepted`，`task.db` 落 `t-20260820-001/002`，`source=local`（§4.2 hmi→local）、`priority=40`（§4.2 起源表，🚫 不再是写死的 50）、`trace_id=h-<req_id>`。<br>✅ **前端地图点选已做（批20）**：点图落钉 → 横幅显示经纬度 → **再点「确认前往」才发**（两步，防触屏误触）。★ 反投影 `fromXY` 是 `toXY` 的**代数逆**（同 R、同 `cos(origin.lat)`、同北向取负）——两者若各写一套近似，操作员看到的钉与机器人去的点会不一致而屏幕上看不出来。★ 无 `enu_origin` 时**拒绝点选并说明**，🚫 不落到 0,0。<br>⚠️ 机器人仍不动（见下表） |
| **W3** `exit_broadcast` | 退出喊话 | ✅★★ **已接并 ORIN 实测（批20）** | ★ 前置（P2 的 `cmd/mode` 接收端）已于批 17 建好。<br>★★ **无前置约束、无 L2 确认**，§12.1.1 逐字：不受任务状态/模式状态/L2 的任何约束，只做一件事——退出 B。**加确认反而有害**：它存在的场景正是本地麦被半双工门控关闭、云端对麦说「停止喊话」会触发自触发回路，此时它是**唯一不经语音的出口**。<br>★ 但**不旁路 `restricted`**（W1 是唯一旁路项）。<br>★ ORIN 实测：WS → `cmd/mode` → P2 ModeFace → `accepted`（`changed:false`，因当时已在 idle——诚实回答不是假成功） |
| **W4** `geo` | 地理要素 CRUD | ✅★★★ **已接并 ORIN 实测** | ★ `rename`/`set_state`/`upsert`/`delete`/`refs` 五 op；`origin` 恒打 `hmi`（CH-2）；`cmd_id = "h-" + req_id`；限流 10 msg/s（超限回 **`E_BUSY`** + `detail.reason=rate_limited`）。<br>★★★ **W4-F 围栏一律不可写**（按 `geo.type` 判**不按 op 判**）：`upsert`/`delete`/`rename`/`set_state` 四写 op 全拒 `E_CHANNEL_DENIED{reason:"fence_not_writable_from_hmi"}`，`refs` 只读放行。依据 `00` HMI-03a + §12.1.1（**停用一个 `allow` 围栏与删除它等价**，故只拒 `delete` 不够）。<br>★ **ORIN 实测**：WS 帧 → `cmd/geo` → P3 `accepted` → `geo.db` 里 `updated_by="hmi"`、rev 1→2→3；围栏 `set_state` 被拒且 `fence.db` 四条 state/rev **全未变** |
| **W5 / W6** | 墓碑 | — | ★ `W6` = `teleop` **整类移除**（`00` HMI-03a：持续驱动永不进 HMI 可写面）。号位保留不复用 |
| **W7** `task` | 任务 pause/resume/cancel/clear_queue | ✅★★★ **已接并 ORIN 实测（含浏览器点击）** | ★ `task_id` **必填**（`clear_queue` 除外）—— §12.1.1 W7 与 §7.2 用同样的话禁止"省略=当前任务"：队列是活的，操作员看到"A 在跑"到帧到达之间 A 可能已结束而 B 开始。<br>★ `pause`/`resume` **L0**，`cancel`/`clear_queue` **L2**（`18` B07）。<br>★★ **前端按钮按 P3 转换图（§4.4）画**：`running`→暂停+取消 · `suspended`→继续+取消 · **排队态（pending/scheduled/ready/blocked）→取消**（★ 最初漏了排队态 —— 误提交的任务正停在那里，是最可能要撤的一条）· 终态无按钮。<br>★★★ **确认用"同一按钮两次点击"不用 `confirm()`/`alert()`** —— 原生模态会阻塞单线程，**地图在机器人移动时停止重绘**、WS 帧堆积；armed 态就地显示任务号与已完成进度（§12.1.1 要求弹窗显示这两项）。<br>★ **ORIN 实测**：浏览器点"暂停" → `rh-1-1` `running→suspended`（`suspend_kind=passive` / `operator_pause`，CR-8：人工暂停是 passive 不是 yielding）→ 点"继续" → `ready`；`task_cmd_log` 落 `h-<req_id>`。第一次点"取消"只 armed **不发帧**（实测状态与 cmd log 均未变） |
| **W8** | PTZ 直控 | ⬜ **保留未开放** | 契约本身未开放，🚫 不得自行接线（冻结项 F-8） |
| — | `teach`（录制） | ✅ **只读已接** | ★★★ **teach 不在白名单**（5 类闭集里没有它）。用户 2026-08-20 裁决：**HMI 只读**。P5 订 `state/teach`（§12A.5）→ 快照 `teach` 组 → 前端徽标 + `teachLayer` 轨迹；🚫 不发 `cmd/teach`。<br>★ 已就地订正 `11` §2.2 的 `cmd/teach` 发布者列（划去 `p5_gateway`）。<br>⚠️ **轨迹是【近似】**：§12A.5 故意不下发点序列（2000 点会撑爆 1 Hz 话题），前端按 `last_point.seq` 逐帧累积，丢一帧就少一个点；图例标「近似」，会话一结束即清空交给 `geo.db` 权威几何 |

**★★★ W2 / W7 的真正前置 —— 缺口在 P3，不在 P5**（2026-08-20 查证 · ✅ **当日已全部做完，见批 14/15/16**）

| # | 实测事实 | 后果 |
|:--:|---|---|
| ① ✅ | ★★ **P3 的 `cmd/task` 接收端只认 P4 的私有形状 `payload['task_request']`**（`task_recorder.py` 逐字：没有 `task_request` 的帧是 control 或 device 命令 → **skipped**） | ★★★ HMI 按契约 §7.2 发 `{action:"submit", task:{…}}` 会被 **静默丢掉**。★ 同样影响**云端经 P5 转发**的任务（§2.2 v0.7.8 起 P5 是云端任务唯一转发者） |
| ② ✅ | ★★ **§7.2 的 control 类 action 零实现**：`cancel` / `pause` / `resume` / `clear_queue` 在 P3 侧没有任何接收与分派（状态机 `machine.py` 的转换图**有** `cancel`/`suspend`/`resume`，但没有从 `cmd/task` 驱动它的入口） | W7 的四个动作全部落空 |
| ③ ✅ | ★ **P3 不发 `cmd/task/ack`**（全仓仅 `p5_gateway/outbound` 的 key 清单提及），而 §12.1.1 的 W2/W7 都要求 `ack ≤ 2s` | 浏览器点了没有任何回执 |

⇒ **W2/W7 = 一个 P3 批次（§7.2 TaskCommand 接收端：认契约形状 + 五个 action + 发 ack）+ 一个很小的 P5 批次（`uplink.py` 加两个 builder）**。

| 项 | 现在做的终局效果 | 卡硬件？ |
|---|---|---|
| **W7 `task`** | ✅★★ **已兑现** —— 浏览器点击真改任务状态机，卡片跟随刷新 | ★ **不卡**，纯状态操作 |
| **W2 `goto`** | ★ **仍只有半个** —— 落库 → 进队列 → 卡片出现（已实测）；但 `allow_motion=false`（§4.1 ③）+ P1 不执行路径（EX-2）⇒ **机器人不动** | 后半段卡 |

**★ 本批【未做】的三项（🚫 不要当成已完成）**

| 项 | 事实 | 归属 |
|---|---|---|
| **语音 `pause`/`cancel` 仍然无效** | §7.2 要求 `task_id` 且禁止"省略=当前任务"，而语音说不出 `t-YYYYMMDD-NNN`。缺的是"我指哪条"→`task_id` 的解析，且要先让操作员知道那是哪条。⇒ P4 的这些 control 意图仍发 `p4_intent_v1`，P3 按**无顶层 `action`** 走旧 recorder 分支并 skip | 新工作，非本批遗漏 |
| **W2 前端地图点选未接** | 后端已可用（实测），但浏览器没有"点图发 goto"的交互 | 小前端批次 |
| **P3 遗留 ingest（`voice_task.py` / `task_recorder.py`）已成创建路径死代码** | 全仓已无发送方发 `task_request`；它现在只承担"无 `action` 帧 → skip"。删它是独立清理，🚫 本批不动（会连带动到 skip 分支） | 债，记此处 |

> ✅★★★ **已裁决并做完（用户 2026-08-20：「P4 同步改发 TaskCommand」）**：P4 现发契约 §7.2 `TaskCommand`，私有 `task_request` 形状及其过渡垫片 `looks_like_p4_shape` **已删除**（§9.3：留着就是第二个可接受形状）。旧形状现被拒并回 schema 错误，有断言守着。
>
> ★★★ **同批查出并修掉的两处更严重分叉（都不是原计划内的）**：
> ① **上行帧嵌套 —— `cmd/task` / `cmd/geo` / `cmd/teach` 三个 key 全中**：P4 的 `build_payload` 把编排器构出的命令当作**槽位**并进 `p4_intent_v1` 信封，线上跑的是 `{schema, intent_id, text, geo_command:{…}}`，而 P3 三个解析器都读**顶层** `cmd_id`/`action`。⇒ **F 类语音（录制/保存/删除）在线上从未成立过**；两侧单测各自全绿也看不见——一侧断言构建器返回值，另一侧喂手写帧。
> ② **L2 确认路径根本不构命令**：`_resolve_pending_confirm` 直接 `dispatch(entry.id, held_text)`，跳过槽位填充与命令构建。而 **L2 恰恰全是破坏性意图**（F11 删路径 / F13 删围栏 / F15 换启用围栏）—— 操作员被问"确认删除吗"、答"是"，P4 发出的是**不含目标也不含 action 的空信封**。
> ⇒ 新增 `tests/integration/test_p4_p3_command_frames.py`：全仓**唯一**断言【发布帧】与【解析器】配对的地方，且刻意调 `decision_to_publishes` 而非构建器（断言构建器正是本次盲区的成因）。

---

#### 7.1 REST 端点词表:死代码已删,全面对齐留给真实现 GWY-P5-13(2026-08-14 裁决)

**背景**:`xbrain/p5_gateway/rest/endpoints.py` 原有 `READONLY_ENDPOINTS`(自称 GWY-P5-13 / 引"17 S12")是早期残留,与现行冻结契约不一致,且它的 `check_readonly()` 是**死代码**(全仓无 live 调用,只有 `test_batch_c.py` 测它;实际 HMI server 只用 `fences_endpoint`)。

**已做(选项 A · 删死代码,不重写)**:删除 `READONLY_ENDPOINTS` + `EndpointNotAllowed` + `check_readonly` 及其 4 条元测试(`test_rest_get_ok`/`post_rejected`/`unknown_endpoint`/`len==8`),保留已接线且正确的 `fences_endpoint`(P5F-2 的 E_DEGRADED 前置)+ 其 2 条测试。理由:重写一个没人读的死常量只会造成"已对齐"假信号(§3.2);删掉即让全仓**只剩一份端点真源**(build_app 的 §6.5 live 面)。

**契约调查结论(供真实现时用)**:
- ★ REST 只读端点集**无 HW-1 式硬约束**(「单一常量生成+不一致拒启动」只管 WS 上行写白名单, 11 §12.1.4),只受 F-8 冻结评审门管;所以运行期"拒非白名单 REST 路径"守卫**并非契约要求**。
- ★★ **11 §12.2 与 17 §6.5 两张权威表本身不一致**(实测逐字):§12.2 独有 `teach/session[/points]` `teleop/state` `arbitration[/{domain}]` `geo/manifest` `geo/{type}/{id}[/refs]` `geo/conflicts`;§6.5 独有 `metrics` `approval/pending`(后者带 G-2「与 state/approval 同队列」硬语义)。W8 现按 §6.5 落。
- ★ 那 4 个旧端点(telemetry/tasks/dock/link)在 11/17/99 **零命中**——从未进过契约,非被删。归宿:telemetry 被 G-2 砍(留单个 metrics)、tasks 走 WS `state/task`、link 走 WS 下行 `link` 投影、dock 仅存 geo `/api/docks`。

**留给真实现 GWY-P5-13 一并处理**(部分卡):
1. ★★★ **先裁 REST 面以 11 §12.2 还是 17 §6.5 为准**(11 是契约唯一真源→§12.2,但 §6.5 的 metrics/approval 要有归宿);据此重建 build_app 端点集。
2. GWY-P5-13 真验收:只读拒写守卫(按定案词表重新生成)、`/api/fences*` 的 E_DEGRADED 带 `(fence_set_id,rev,crc32)` 三元组、`/api/approval/pending` 与 `state/approval` 同队列、`/api/events` 排序键 `(channel,ch_seq)`、`test_rest.py` harness。
3. ⚠️ `/api/events` 返回体 schema / since 语义 / 排序键 / 分页游标在契约里**本身仍"未定"**(17 §6.8.5 第 8 项),须 11 侧先落笔。

#### 7.1A ⚠️★★★ 待裁决 · Zenoh gossip：RT-C2 平面隔离 vs 2026-08-10 实测（全库回归里长期红）

> ★ 登记于 2026-08-23。`tests/deploy/test_zenoh_router_config.py` 两条用例
> （`test_rt_config_disables_gossip` / `test_gen_config_disables_gossip`）**持续失败**，
> 且**不是硬件阻塞** —— 是契约与实测正面冲突，必须人拍板。

| 侧 | 逐字依据 | 后果 |
|---|---|---|
| **契约** | `11` §1.1.2 **RT-C2**：两个平面都必须显式 `scouting.gossip.enabled = false`。理由写得很硬：「gossip 会通过已建立的链路扩散节点信息，形成间接串接。★ V5 只有一个 router，gossip 未关无害；**V6 有两个 router，跨面进程的两条链路正好是 gossip 的扩散通道 —— V6 必须关**」 | 这是**平面隔离**约束，不是性能取舍 |
| **实测** | `configs/zenoh/router_gen.json5` 现为 `gossip.enabled: true`（2026-08-10，标记 `V-ORIN-ZN-GOSSIP`），带 `multihop:false`。实测：peer 客户端经路由发布时 **`false` → 0 收包 / `true` → 160** | 关掉 gossip，peer 之间发现不了对方的订阅，总线不通 |

**三条路，各自的代价**

| # | 做法 | 代价 |
|---|---|---|
| ① | 客户端改 `mode=client` | 偏离 `11` §1.1.2 钉死的 `mode: peer`（`session_factory._MODE` 就是照它写的）；client 模式要求 router 先起，启动序变严 |
| ② | 接受 `gossip=true` 并订正 RT-C2 | **削弱平面隔离约束本身**。⚠️ 需论证 `multihop:false` 是否足以堵住 RT-C2 点名的那条扩散通道（跨面进程同时直连两个 router） |
| ③ | 另找发现机制（显式配置对端 / 静态订阅表） | 工作量最大，但两侧约束都不动 |

★★ **在裁定前这两条用例会一直红。**🚫 不要为了让它绿而改测试 —— 它守的是 RT-C2，而 RT-C2 守的是隔离。

---

#### 7.2 ⚠️ 待裁决 · 围栏 role 枚举 vs P3 zone_label(W1 附带发现)

**现象**:契约围栏 role 是闭集 `allow/forbid/zone`(17 §6.8 / 11 §9A.2),映射到显示类型 `active/forbid/alarm`(活动/禁入/报警,决定连线样式着色 17 §6.10.2A)。但 **P3 的 `fences` 表(15 四库模型)只存自由文本 `zone_label`,没有 role 枚举列** —— 所以 `cmd/fence` 几何 P5 收到时可能不带 role。

**现处理**(hmi.js `fenceType`):优先读 `role`;缺则按 `name`/`zone_label` 关键字回退(含"禁入"->forbid、"报警"->alarm、"活动"->active);**再缺默认 `active`(keep-in)**。⚠️ 即**未按这三个关键字命名的围栏会被误判为活动区**(亮蓝粗实线),而它可能实为禁入/报警。

**待裁决**(归 P3 `15` / 契约 `11` §9A.2,本册不代改):P3 `fences` 表是否补 role 枚举列并在 `cmd/fence` 携带 role,还是契约正式承认"按名判型 + 默认 active"这套启发式?在裁定前,W1 围栏样式对**非常规命名**的围栏不可靠。

---

## 8. 部署 / systemd 收尾（2026-08-16 · 批2-4 正规部署）

> 批2-4 把 `deploy/systemd/` 的单元从「草稿 + 若干失效」硬化成「可一键安装、ORIN 实测通」。
> 提交：`38ee531`(批2 硬化)· `6dba157`(批3 install 机制)· 本批(批4 验证 + uninstall 通配修复)。
> ★ 安装**机制已成 + ORIN 实测 install / uninstall / gated-skip / mount 全绿**。**DEC-15 已于 2026-08-17 收口**(U83：命名 `xbrain-` + install root `/opt/xbrain_v6/data/install`)；剩下是**构建系统实现(DEP-5)+ 标定/回填**，enable 不再卡决策。

### 8.1 已完成（DEC-15 无关的正确性 + 机制）`[DONE]`

- **单元硬化**(批2 · `38ee531`)：删致命 `WatchdogSec` 重启环(★ 实测确认 systemd 会杀不发 `WATCHDOG=1` 的单元，6s 后 `Result=watchdog`)· 7 个未编译 C++/bridge 单元加 `ConditionPathExists`(缺二进制干净跳过)· `StartLimit*` 从 `[Service]` 移 `[Unit]`(v229 起放错被静默忽略，Restart 单元丢重启限流)· zenohd 路径 `services/zenoh/zenohd`→`/usr/local/bin/zenohd` · 新建 `run-xbrain.mount` · 修 `llm` 悬空 `After=perception.service`→`xbrain-perception.service` · 修 zenohd-gen 恒绿失效的占位符检查(`"\${"` 是 unknown escape → bash 报错 → `!` 吞错 → 检查永不 fail，§3.2)· `Documentation=` ASCII 化。`systemd-analyze verify` 仅剩 7 个 gated 二进制的预期告警。
- **install 机制**(批3 · `6dba157`)：`scripts/install_units.sh`(install / dry-run / enable / uninstall；排除 3 个 AI 草稿；**默认不 enable**)· `deploy/etc-xbrain/{robot.env,network.env}` fail-safe 模板(rid 留空 → rtk 明确拒启；IP 用 127.0.0.1 → 绑回环不暴露)· `p1-motion` 补 `EnvironmentFile=-/etc/xbrain/robot.env` 接 `XBRAIN_ROBOT_ID`(缺则 gnss 桥 OFF、state/pose 断流)。
- **ORIN 实测**(批4)：install 18 单元 dormant + 模板创建 → 单元状态 `disabled`(不 enable)· `xbrain-maxfan` 完好 · p1 依赖链带正确 `xbrain-` 前缀解析 · **gated 跳过实证**(perception 无二进制 → `ConditionResult=no` / `Result=success` / `inactive`，**不 fail 启动**)· **run-xbrain.mount 实测挂载** `/run/xbrain` tmpfs(size=64M / mode=755)· uninstall 只删已知集(★ 修了会误删 `xbrain-maxfan` 的宽通配 bug)· 验证后**完全还原基线**(dev 栈 5 进程未扰)。

### 8.2 剩余卡点

| # | 缺什么 | 卡因 |
|---|---|---|
| DEP-1 | **enable 到 boot + 3 个 AI 单元(ai-asr/llm/payload)安装** | ~~`[GATED-DECISION]` DEC-15~~ **已解**(U83, 2026-08-17)。enable 现只等：标定安全参数(§3.1，否则 freeze 拒 null)+ `/etc/xbrain` 真值；AI 三单元另等两处 `11` 回填(§11A.2.3 ai_asr 模型账按 AIR-M1、payload 的 §11A.6.3 OOM 行)。就绪即 `sudo install_units.sh --enable`。 |
| DEP-2 | **chassis_relay 的 `10` §3.3.8 watchdog 重新加回**(Type=notify + WatchdogSec + sd_notify 三者同时) | `[GATED]` 卡 chassis_relay C++ 实现 `sd_notify(WATCHDOG=1)`；批2 已在单元内就地注明「延期非删除」。p1-p5 / 路由同理:实现心跳后可加。★ **单加 WatchdogSec 会重启环**(实测)，必须与 Type=notify + sd_notify 一起。 |
| DEP-3 | **zenohd-gen 空变量守卫**：`LAN2_IP`/`WIFI_IP` 为空 → envsubst 写空串 → endpoint 变 `tcp/:7447` → zenohd 可能当 bind-all(触 NET-C9) | `[SW-NOW]` 现仅靠模板 127.0.0.1 占位兜底，**无单元级守卫**。需 `ExecStartPre` 校验两 env 非空再启。批2 已在 zenohd-gen 单元注释标记该 gap；现有 `grep '${'` 检查抓不到(envsubst 对未设变量写空、不留占位符)。 |
| DEP-4 | **p2-p5 的 `common.robot_id` 来源确认**：config-freeze 必须**无** `XBRAIN_ROBOT_ID` 才能跑(否则 materialize abort，dev 实证)，那快照里 `common.robot_id` 从哪来? | `[SW-NOW 待核]` p1 / rtk 运行期直读 env 已解;p2-p5 走 freeze 快照的 `common.robot_id`(layers.py 在 freeze 期映射)。需确认**生产 freeze** 的 robot_id 流(configs/ 直填? 还是 freeze 另有取法)，避免快照 `common.robot_id` 为 null。 |
| DEP-5 | **构建系统实现:C++ 装到 install root `/opt/xbrain_v6/data/install`**(DEC-15/U83 定的 root) | `[进行中]` ★ **rtk_driver 已完成(2026-08-17)**:CMakeLists 加 `install(TARGETS rtk_driver RUNTIME DESTINATION lib/${PROJECT_NAME})` + `CMAKE_INSTALL_RPATH=/usr/local/lib`(自解析 libzenohc,不靠 LD_LIBRARY_PATH);ORIN 实测 `cmake --install --prefix .../data/install/rtk_driver` 落 `data/install/rtk_driver/lib/rtk_driver/rtk_driver`,`ldd` 通,单元 ExecStart 已指向它,verify 无告警。**剩余**:① `chassis_relay`/`teleop_input` **尚未建**(未建/GATED),建时按同一 install 约定加规则;② ROS2 包(perception/quadruped)走 colcon,但 **ORIN 现无 colcon/ROS2**;③ 编译树(build/)按 §0.2 宜移出 `ros2_ws/`(dev 栈仍用 `ros2_ws/sensor/build/`,是 dev-vs-deploy 正常分叉)。感知/底盘本体仍 `[GATED-HW/DESIGN]`。 |

---

## 9. 汇总 · 剩余全部卡点(2026-08-14 核对)

> 纯软件能推的都已推完(HMI W 系列 + SW-1)。以下是**现在推不动**的,按卡因归三类。已在上文各节详列,此处只作索引确认"全部有落点"。

| 卡因 | 项(章节索引) |
|---|---|
| **[GATED-HW] 云深处底盘/RTK/相机/遥控** | EX-2..6(§1)· quadruped/chassis_relay/rtk_driver/teleop_input(§2)· P1-1(§3)· 硬件集成(§4)· ★★★ **录制端到端四道门(§4.1，当前终点=`cam_rgbd` 无生产者)** · HMI-W4 位姿全片 / W5 §6.4 快路 / W7 EX-1 数据(§7)· chassis_relay watchdog 待 sd_notify(§8/DEP-2) |
| **[GATED-DESIGN] 设计未写** | ⚠️★★★ **2026-08-20 订正：perception / RNS 两份详设【已写完】**(19 · 20)，本类**不再含它们**；余 P1-4 航向丢失恢复 odom 桥接+视觉重捕(§3，依赖 quadruped odom + perception + RNS 三者的**实现**) |
| **[GATED-DECISION] 待用户/契约裁决** | REST §12.2 vs §6.5 谁权威 + GWY-P5-13 真实现(§7.1)· 围栏 role 枚举 vs zone_label(§7.2)· ★ **P4 是否同步改发 §7.2 `TaskCommand`**(§7.0/SW-14)· rtk_driver 语言待定(§2；平台基线 D-45 本身已 U74 定 Humble/22.04)· ~~DEC-15~~ **已 U83 收口(§8)** |
| **[SW-NOW] 纯软件可推(非卡,待排期)** | ★★ **SW-14 P3 `cmd/task` 对齐 §7.2(解锁 HMI W2/W7 + 云端转发)** · SW-15 W2/W3/W7 的 P5 builder · SW-2/3 设计 · SW-4 云上行 · SW-5 测试框架 · SW-6 配置落值 · SW-7 字符集债 · SW-8 充电执行 · SW-9 全系统圆润 · SW-10 comment_ratio · SW-11 LAN2 bind 落值 · ~~SW-12~~ **已上线** · DEP-3 zenohd-gen 空变量守卫 · DEP-4 robot_id 快照源核实 · DEP-5 构建系统装 data/install(§8) |
