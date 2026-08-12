# XBRAIN_V6 · NEXT（未完成清单与推进对账）

> 用途：PB1-8（P3 任务链路接通）之后，把**还缺什么**逐条列清，供 phase 任务开发对账。
> 本文件是**推进对账表**，不是正式设计册（正式真源仍是 00-21 十一册）。
> 建立于 2026-08-12（P3 任务链路 PB1-8 收尾当日）。
> ★ 标注约定：`[GATED-HW]` 卡真硬件/云深处底盘 · `[GATED-DESIGN]` 卡设计未写 · `[SW-NOW]` 纯软件、现在可推 · `[DONE]` 已完成。

---

## 0. 现状快照（已建成的）

- **语音闭环**：MIC→ASR(paraformer)→P4 编排器→TTS，ORIN 实测通。`[DONE]`
- **PTZ / payload 设备控制**：布控球 ONVIF、灯光/爆闪/音量，圆润理解(大类语义路由+前缀消解+后置精修)，ORIN 实测。`[DONE]`
- **P3 任务链路 PB1-8**：语音→cmd/task→task.db(SQLite)落库→调度器 pending→ready→running，状态机对齐 11 S4.4，id 生成、幂等、事务、F 类录制写路径、running→done 完成逻辑。ORIN 实测真落库+真流转。`[DONE]`
- **P2 仲裁 / 模式机 / BIT**、**P5 网关批次 A-E**、**配置冻结线(CFG-FZ)**、**甲方云端翻译/去重/Q0 急停**：主体已建。`[DONE]`
- **LLM 服务**：llama-server + qwen2.5-3b-instruct 在 ORIN 上跑(端口 18082)。`[DONE]`

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
| **perception**（定位/位姿/障碍/目标，33ms/30fps） | C++ | ★ **详细设计未写** `[GATED-DESIGN]` | 先写设计，再写码；需相机/LiDAR |
| **chassis_relay**（急停链路 CRL-1..5） | C++ | 未建 | common 地基库 + 底盘 |
| **rtk_driver**（GPS/授时，唯一判 ClockStatus.sync） | 待定(建议 C++) | 未建 `[GATED-HW]` | RTK 硬件；语言待 D-45 拍板 |
| **teleop_input**（遥控 deadman） | 待定 | 未建 `[GATED-HW]` | 遥控器 |
| **behavior_proxy** + **Nav2 behavior_server** + **zenoh-bridge-ros2dds** | C++/Rust | 未建 | ROS2 环境 + Nav2 |

---

## 3. P1 完整运动执行 + RNS

| # | 缺什么 | 状态 |
|---|---|---|
| P1-1 | 真 20Hz 控制环（现仅 voice-loop MVP） | 部分件(gate/failsafe)已建，未接真感知/底盘 `[GATED-HW]` |
| P1-2 | 速度门四段 f(d_free) + 迟滞、路径跟随、旋转门 RCG 全接 | 零散件已建，未串成环 |
| P1-3 | **RNS 避障**（进程内模块，行为源 rns_avoid 900） | ★ **详细设计未写** `[GATED-DESIGN]` |

---

## 4. 真硬件集成

- M20S 底盘（**等云深处**）、RTK、LiDAR、可见光/热成像相机、遥控器。`[GATED-HW]`
- 集成测试三档（不需真设备 / 需 ORIN / 需真底盘）框架**未建立**。

---

## 5. 纯软件、现在可推（不卡硬件）`[SW-NOW]`

| # | 项 | 价值 | 备注 |
|---|---|---|---|
| ~~SW-1~~ | ~~LLM tier-2 接线~~ | **`[DONE]` 2026-08-12**（44c752b/cee733c）：mission_select 候选选择 + build_tier2_fn 组装 + 槽位回传契约 + grammar 收紧(slots 闭合对象)。ORIN 实证:`往前挪三米`->move_forward{distance_m:3}、`把当前位置记为集合点`->record_waypoint{name:集合点}、播报->speak_custom{text} | — |
| SW-2 | **perception 详细设计编写** | 高：解锁整个 C++ 感知层 + P1 RNS | 纯设计活，不卡硬件；写进正式册 |
| SW-3 | **RNS 详细设计编写** | 高：解锁 P1 避障 | 纯设计活 |
| SW-4 | 云端/HMI 完整上行（P5 端到端云协议） | 中 | 需甲方云端联调 |
| SW-5 | 集成测试三档框架建立（docs + harness） | 中 | 现无落点 |
| SW-6 | 配置落值：common.db.* 四库路径 + 其余 null 安全参数标定 | 低-中 | 按 §3.1，标定即启动断言放行 |
| SW-7 | 字符集存量债清理（task #46，24 漏网符号 + charset_lint 完善） | 低 | housekeeping |
| SW-8 | 充电/对接**执行**串联（状态机+仲裁器已建） | 中 `[部分 GATED-HW]` | 三级选桩逻辑可测，真对接需底盘 |
| SW-9 | **全系统圆润扩展**（非设备类的同义说法加宽） | 中 | ★ 用户早先定「聚焦 PTZ/payload，后面慢慢扩展到整个系统」的那一批。2026-08-12 已收口薄意图（1词无兜底）：C08/G09/H07/H08/F13 加同义说法 + 18 对齐 + 变异守护。★ **剩 47 个恰 2 词且无 tier-2 的意图**（多为 G 类只读查询 L0），是下一批候选；现状不是缺陷（各有 2 说法 + §2.2 边界节 C 已载「贴 keywords 说」），扩不扩看优先级 |
| SW-10 | **comment_ratio 注释债**（498 文件 < 70%） | 低-中 | ★ 2026-08-12 跑全量 tests/common/ 照出：`test_lints::test_comment_ratio_holds` 是唯一未修的元测试失败（另 6 个 08-11 静态规则违规已 commit 07573a4 修）。§2.4 阈值 08-06 从 25% 上调 70% 后累积。按 §2.4「每块解释 why」分批补，不刷百分比。已开 chip（task_3e48348d）。★ 教训：yaml 头/字符集门禁盲区（configs/.yaml 从没被扫）已于 2026-08-12 关闭（ca08aaa/1bc9702），今后新增 yaml 自动拦 |

---

## 6. 建议推进顺序（软件侧）

1. **SW-2 / SW-3 perception + RNS 详细设计** —— 最高杠杆：这两份设计不写，整个 C++ 感知/执行层和 P1 都推不动；且是纯设计活、不卡硬件。项目记忆早已标注"押后到最后"，但**它是后续一切的前置**。
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
| HMI-W5 | 真端到端 ESTOP:§6.4 专用 <=10ms 快路(P-1)+ §6.3 estop 探活喂 estop_path(现 MVP 恒 "ok" + 走 cmd/estop) | 部分 `[SW-NOW]` | estop 快路 + 探活接线 |
| HMI-W6 | WS 推送 state_snapshot/delta(现 1Hz REST 轮询) | 未做 `[SW-NOW]` | ws_protocol 已建, 需接 push |
| HMI-W7 | 计划目标点有序表 + 逐点勾选 + 进度 2/N:路径展开(EX-1) | 未做 `[部分 GATED-HW]` | geo.db 航点 + P1 真执行上报 |
| HMI-W8 | 端点集对齐冻结契约(17 §6.5):`/api/fences/active` + `/api/events` 已加 | ✅ **部分** | ★ 剩 `/api/routes` `/api/geo/*` `/api/docks` `/api/bit` `/api/metrics` `/api/approval/pending` 需各自数据源 |

> ★ W1/W2/W3/W8 已接(纯软件); W6 纯软件可续; W4 是硬件总闸(位姿那一整片, 也卡 W1/W2 地图落点); W5/W7 部分卡硬件。
