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
| SW-10 | **comment_ratio 注释债**（xbrain/ 约半数文件 < 70%） | 低-中 | ★★★ **2026-08-16 用户裁决：作为负债留到最后做【全系统语音集成测试】时一并完成，在此之前门禁 `test_lints::test_comment_ratio_holds` 保持红，是【已知已记的债】不是回归**。★ 规模（§3.7 不烤死数字，跑命令得准数）：门禁只算 `xbrain/`（`test_comment_ratio_holds` 断言 `LOW ... xbrain/` 零命中）；`python3 scripts/lint/comment_ratio.py \| grep -c 'LOW.*xbrain/'` → 2026-08-16 实测 **239** 个不达标（分布 p4_agent 57 · p2_core 50 · p1_motion 43 · p5_gateway 37 · p3_task 37 · boot 8 · common 7；旧记「498」是含 scripts/tests 的全仓数，已作废）。★ 成因：§2.4 阈值 08-06 从 25% 上调 70%（= 注释行 ≥ 2.33× 代码行）后，P1-P5 批量业务代码普遍在其下累积。★ 修法：按 §2.4「每块解释 why、不刷百分比」分批补真注释——★★ 这不是进度审计，是注释密度尺，别当「补完才算开发完成」。已开 chip（task_3e48348d）。★ 关联教训：yaml 头/字符集门禁盲区（configs/.yaml 从没被扫）已于 2026-08-12 关闭（ca08aaa/1bc9702） |
| SW-11 | **`hmi.bind[0]` LAN2 地址落值**(现 null) | 低 | ★ full 启动 `check_p5_config` 因 LAN2 bind 为 null 而**拒启**(§3.1 设计行为, 报 `hmi.bind[0] unassigned`);voice-loop MVP 走宽松 `make_bound_sockets` 只绑非空口(wifi `192.168.1.7` + `127.0.0.1`)故能跑。等 U-15 部署分配 LAN2 网段地址即落值解除。★ `bind[1]` wifi 已填(2026-08-12 用户明令),`bind_guard` 测试已对齐(f7803c9) |
| ~~SW-12~~ | **事件存证链路** | — | ✅★★★ **2026-08-17 落地上线(7 批, 287 测试, ORIN 实证)** —— commits `0881fc1`(批1 record.db DAO 按 17 §3.4 权威 schema, 两写一读三连接, ch_seq/dedup/need_ack/JSONL 降级)· `7f129b9`(批2 7 阶段 pipeline + §6.2 channel 推导, 替换错模型占位)· `03b2796`(批3 backfill: 令牌桶限速 20eps + 4:1 加权 + EventReplay 消息)· `887ac3a`(批4 uplink: DeliveryMarker + AckTracker + BackfillRunner)· `2e7f08e`(批5 EventSubsystem 同步/async 桥接进运行 p5, degrade-safe)· `8fcd255`(批6 device 掉线事件 build + debounce 监视器)· `9b46af9`(批7 ORIN 端到端: 真 p5 重启带 XBRAIN_RECORD_DB, 注入 live 事件正确落库 channel/ch_seq/delivered, e2e_check.py PASS). ★ **剩余(非本子系统, 各有卡因)**:① **3 个 device 产生方** —— ✅ **2026-08-17 接线** `DeviceHealthBridge`(p2, 复用 device_events)+ p2 事件发布器(gen.put event/{sev}/{cat}): **MIC 真+已端到端实证**(杀 arecord→cap_alive=False→debounce→`device mic offline`→p5 record.db 落 voice/device_offline); **payload 真**(轮询 payload-service `GET /status` 的 `device.{audio_connected,lights_connected}`=8519/8529 socket, 连通已验证无误报; 真掉线要 GZH-2 socket 断 `[GATED-HW]`); **ptz 真**(2026-08-17 `PtzLivenessProbe` 非阻塞 ONVIF 探测线程 commit `d41efbd`; 三态 up/down/auth, auth 首次即停防锁账户 per docs/PTZ 报告 §8; 真机 192.168.66.13 可达实测不误报);② **实时上云** `[Q-P5-8 ✅ 2026-08-17 决定 A]` —— 云端放宽实时订阅至 `event/{warn,fault}/**`(含设备掉线), 产生侧本就直发无改动, exact 通配甲方 SW-4 落定(commit `1e4abca`)。★★ **断连兜底仍缺(新跟进)**: `trigger_backfill` 活循环无调用点(仅测试调)+ recon(17 §3Y.3)未实现 —— 真断网时非 alarm 事件重连后无自动补发, 需接「重连→触发补发」+ 实现 recon。★ **2026-08-17 架构一致性审计闭环**: U18b 落案(need_ack 并集 `8d0224d`)· F9 修 EventAck result 闭集 ok/duplicate(`2edf261`, 原误用命令 Ack 的 accepted 会让 need_ack 事件永远重发)· F3 device detail 补 reason/socket + F8 eid 防跨重启碰撞(`ad89df4`)· 死代码清理 chip(错模型 backpressure/recon orphan);③ **甲方真云端 endpoint** `[SW-4]` —— uplink/backfill P5 侧全建全测(对 loopback stub), 只剩指向甲方;④ **`common.db.record_db` 落值** `[SW-6]` —— 现走 XBRAIN_RECORD_DB dev 覆盖, 配置落值即转正 |

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
| HMI-W5 | 真端到端 ESTOP:§6.3 estop 探活喂 estop_path;§6.4 专用 <=10ms 快路(P-1) | ✅ **探活已接** / 快路 `[GATED-HW]` | ★ **探活机制已落**:`EstopProbe` 状态机(estop_probe.py)+ 每拍 `probe/estop/ping` -> 收 `probe/estop/pong` -> RTT/连续无 pong -> ok/degraded/down(11 CR-2/CR-3 · T-23/T-24)。★★ **无底盘时诚实报 "down"**(按钮置灰),🚫 不再恒 "ok" 造假。★ 剩 §6.4 <=10ms 专用快路(P-1)与 pong 权威源都要 `quadruped`/`chassis_relay`(云深处) |
| HMI-W6 | WS 推送 state_snapshot + state_delta 增量 | ✅ **完整** | ★ /ws 端点(push_hz 可配), 前端 WS 主、REST 轮询兜底。★ 依赖 **wsproto**(uvicorn 0.52 与 websockets 16.x 服务端不兼容, 用 wsproto 后端; 缺则回退 auto+REST)。★ 修了 `from __future__ annotations` 致 FastAPI 把 ws 参数误判查询参数的 403 坑。★★ **state_delta 已接**(2026-08-14):连接发全量 keyframe, 之后每拍只发变化的顶层组(geo/pose/plan/status/events), 静默拍发空 delta 作 keepalive, 每 30 拍周期 keyframe 自愈; 前端 group-level 合并。ORIN 实测:连接 keyframe -> 静默空 delta -> 注入 state/task 后单拍 `keys=['plan']` 只带变化组 |
| HMI-W7 | 计划目标点有序表 + 逐点勾选 + 进度 2/N | ✅ **映射已接** / 数据 `[GATED-HW]` | ★ **映射已修**:`_extract_active_tasks` 从 P3 `{schema,active_task:{task_id,state}}` 抽取扁平 task 喂 `_plan`(旧 MVP 把整信封当计划 -> state/targets 全落 None 空白卡)。前端 `state==running` 驱动黄色实时轨迹显隐。兼容未来 1Hz 心跳列表(current_step/total_steps)。★ 剩目标点有序表 + 进度分数需路径展开(EX-1):geo.db 航点 + P1 真执行上报(云深处) |
| HMI-W8 | 端点集对齐冻结契约(17 §6.5 == 11 §12.2) | ✅ **端点面已齐** | ★ **全部 §6.5 只读端点已上**:`/api/routes` `/api/docks` `/api/health` `/api/bit` `/api/metrics` `/api/approval/pending`(加上已有 fences/fences_active/events)。★★ **诚实可用性**(ORIN 实测六端点全 200):`/api/health`+`/api/bit` **订阅 P2 `health/factor`/`health/bit` 已接、中继链路已证**(注入 health/factor -> available:true 原样直透, G-2 同源),但 **P2 voice-loop MVP 尚不发 health/**(那是 P2 全设计行为), 故当前 available:false;routes/docks(geo.db 卡)/metrics(遥测聚合器未实例化)/approval(L3 队列无喂入)一律 available:false 不造假。★ 剩 `/api/geo/manifest`(§12.2 新规范式, routes/fences/docks 作兼容别名)与上述源真正产数 |

> ★ **纯软件部分全部完成**:W1/W2/W3/W5(探活)/W6(含 state_delta)/W7(映射)/W8。剩余全部卡硬件(云深处底盘/rtk_driver/perception):W4 位姿全片(总闸,也解锁 W1/W2 地图落点)· W5 §6.4 <=10ms 快路 · W7 目标点表+进度(EX-1)。

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
| **[GATED-HW] 云深处底盘/RTK/相机/遥控** | EX-2..6(§1)· quadruped/chassis_relay/rtk_driver/teleop_input(§2)· P1-1(§3)· 硬件集成(§4)· HMI-W4 位姿全片 / W5 §6.4 快路 / W7 EX-1 数据(§7)· chassis_relay watchdog 待 sd_notify(§8/DEP-2) |
| **[GATED-DESIGN] 设计未写** | perception 详设(§2/SW-2)· RNS 详设(§3/SW-3)· P1-4 航向丢失恢复 odom 桥接+视觉重捕(§3，依赖 quadruped odom + perception + RNS) |
| **[GATED-DECISION] 待用户/契约裁决** | REST §12.2 vs §6.5 谁权威 + GWY-P5-13 真实现(§7.1)· 围栏 role 枚举 vs zone_label(§7.2)· rtk_driver 语言待定(§2；平台基线 D-45 本身已 U74 定 Humble/22.04)· ~~DEC-15~~ **已 U83 收口(§8)** |
| **[SW-NOW] 纯软件可推(非卡,待排期)** | SW-2/3 设计 · SW-4 云上行 · SW-5 测试框架 · SW-6 配置落值 · SW-7 字符集债 · SW-8 充电执行 · SW-9 全系统圆润 · SW-10 comment_ratio · SW-11 LAN2 bind 落值 · **SW-12 事件存证链路(record.db DAO + pipeline 接线 + backfill)** · DEP-3 zenohd-gen 空变量守卫 · DEP-4 robot_id 快照源核实 · DEP-5 构建系统装 data/install(§8) |
