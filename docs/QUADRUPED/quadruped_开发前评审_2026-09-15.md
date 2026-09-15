# quadruped 开发前评审：按 13 册开发底盘 ROS 2 控制节点，是否完全符合 V6 架构与需求（2026-09-15）

> 阅读范围：`10`～`20` 全部编号册的 quadruped 触点（`10` §3.1/§3.2/§3.3/§5.4/§6.6，`11` §1.1/§2.2.1/§3.0/§3.4/§4.1/§4.2/§7.1/§7.7/§8.5/§9 全节，`12` §2.2.1/§4.8/§8.3/§9，`14`/`15`/`17`/`19`/`20` 各触点），`13` 全册逐节，`99` U74/U83，`21` 台账，加上 09-14～15 两天的底盘实测（`M20S_底盘接入与探测实录_2026-09-15.md`）。本文只登记结论与待裁决项，🚫 不改动 `10`/`11`/`13` 正文。

## 0. 结论

| 维度 | 判断 |
|---|---|
| **架构** | ✅ **符合**。`13` v1.2 已与 `10`/`11` 对齐：三通道单进程、RT-C4/RT-C5、Tier 1 逐字继承 `11` §9.12、配置 L6 + 断言 K、Stage 1 先于 P1、核 5、`rt/chassis/*` 与 `rt/safety/*` 十七条 key、故障码前缀 CF-1～5、odom 契约 §4.8。按 `13` 写代码不会违反 V6 架构。 |
| **底盘事实** | ⚠️ **13 处假设需先回填**（§3）。`13` 编写时底盘未到货，多处按「待实测」写了保守分支；两天实测已能定案，其中 5 处若不回填会**照设计写出必错的代码**（端点候选、站立稳态、休眠、GPS 组、服务查询）。 |
| **工具链** | ⚠️★★★ **硬前置**：ORIN 现无 ROS 2 Humble、无 colcon、无 CycloneDDS（`99` U83 DEP-5 未做）。`13` 的通道三（rclcpp odom/TF）与通道二（裸 CycloneDDS）**今天编不出来**。zenoh-c 已装，通道一与 RT 面可以先做。 |
| **契约未决** | ⚠️ `13` §14 登记的 10 条与 `11` §9 的差异**仍未裁决**（`11` §9.2 枚举仍是旧手册值、`/IMU_YESENSE` 仍有 7 处、`00` NAV-100/102 仍写 Jazzy）。其中 4 条影响编码（§5）。 |
| **需求** | ✅ 要求 a～g（全量上报、双域隔离、100 Hz odom、功能不遗漏、C++17、配置化、Tier 1）在 `13` 中均有可验收的落地；实测未发现需求层面的缺口，但发现两条**运行约束**（§3 第 11、12 项：底盘自带导航服务与手柄链路）。 |

⇒ **建议**：不等裁决先做与之无关的批次 B0～B3（骨架/构建/CHS-A 编解码/链路/Tier 1，§6），同时请用户裁决 §7 的 6 项；B7/B8 等 ROS 2 Humble 装好后再做。

## 1. 架构一致性核对（13 ↔ 10/11/12）

| 项 | 10/11/12 要求 | 13 的落地 | 核对 |
|---|---|---|---|
| 进程与平面 | `10` §3.1：C++，② RT + ③ CHS-A，禁通用面（RT-C4） | §1.1/§2.1/§7.1.1：事件与 ack 一律经 `chassis_relay` | ✅ |
| 三通道与双域 | `10` §6.6 / `11` §1.1.5 | §2.4 DDS-1～9：域 0 裸 participant，域 42 rclcpp，只读不写 | ✅ |
| Tier 1 | `11` §9.12（F-13） | §3.2 逐字继承，常量 200/100/50 ms，QD-1/QD-2 单函数单出口 | ✅ |
| 速度唯一出口 | `11` §2.2.1 `rt/motion/cmd_vel` 20 Hz + `estop_epoch` | §3.5 SV-1～4，只用真实轴指令 | ✅ |
| 握手 | `11` §9.1.4 / `12` §4.8（P1 已实现 `hello`） | Q-1 `hello_ack{runtime,spec}` | ✅（`12` 登记的 S14-a 超时策略仍空） |
| 状态上行 | `11` §9.8 六条 key + `RobotState`/`PowerState` | §7.1 全量不裁剪，开放集枚举 | ✅ |
| 配置 | `10` §5.4.0 第 15 个固定文件、断言 K、读 resolved 快照 | §8.1～8.3，QC-1～17 | ✅（`configs/quadruped.yaml` 现为空骨架） |
| 启动编排 | `10` §3.3 Stage 1，GATE-3 先于 P1；单元已存在（gated on 二进制） | §2.5/§10 | ✅ |
| 实时化 | `10` §3.2 核 5；`12` RTC-1～9 对齐 | §9.1 七线程；`tx_guard`；`common/rtcomm` 已有实现 | ✅（D-42：单元 `CPUAffinity=5` 与 §8.2 的 io/rt_safety 用核 6 不一致） |
| 平台基线 | `99` U74：Humble + 22.04 | §2.0 按 humble 建模，PB-1～6 | ✅（U74 改用 `ROS_LOCALHOST_ONLY`，`13` DDS-7 仍写 `ROS_AUTOMATIC_DISCOVERY_RANGE`，小订正） |

## 2. 与 `13` 无关但影响开工的现状

| 项 | 现状 | 影响 |
|---|---|---|
| `chassis_relay` | 未实现（只有 systemd 单元） | quadruped 的 ack/事件/状态没有转发者；台架测试需要一个 relay + P1 桩（放 `tests/`） |
| `p1_motion` | 已实现 `rt/chassis/hello` 握手与 20 Hz `cmd_vel`（含 gate 块） | 可直接联调通道三上游 |
| `common/` | 已有 envelope_writer、message_age、tx_guard、lockfree_slot、rt_thread、mono_clock、qos_profiles、session_config、yaml_lite、errors、units、closed_sets | 直接复用；**缺 JSON 解析器**（rtk_driver 只手写序列化，quadruped 要解析底盘上报） |
| 构建 | `ros2_ws/sensor` 是纯 CMake 范式，安装到 `data/install/<pkg>/lib/<pkg>/`；`perception` 是 ament 包但 ORIN 无 ROS 编不了 | quadruped 沿用 sensor 的 CMake/安装/单测范式，rclcpp 与 CycloneDDS 做成可选目标 |

## 3. 实测事实 vs `13` 假设（须先回填 `13`，再写对应代码）

| # | `13` 处 | `13` 假设 | 实测 | 处置 |
|---|---|---|---|---|
| 1 | §2.2 V-40 / §8.2 `endpoint_candidates` | 明文候选 `udp 10.21.31.103:30000`，加密候选 30003/30004 默认关 | 明文端口 30000/30001 编译期不存在；`enableTls=false` 后 **同端口 30003/30004 走明文**；TLS 需云深处 Project CA 签发的客户端证书 | ★★★ 照 `13` 默认配置三候选全败。默认候选改为 `tcp 10.21.33.103:30003 tls:false` / `udp 30004 tls:false`；`tls:true` 候选保留待证书。V-40 关闭 |
| 2 | §2.2 V-41 CB-1～4 | 码表待实测 | `hex32` 直接生效，`legacy_decimal` 无需 | CB-1 成立，V-41 关闭 |
| 3 | §2.2 V-42 | 头字段/版本字节是否校验 | 16 B 头按指南填、版本 0x01 全零预留，底盘接受 | V-42 关闭 |
| 4 | §6.2/§6.3 MS-5 | 站立后需**我方**下发 `MotionParam=17` | 固件收 `1` 后**自动**切 RL 模式，稳态 `17 / Gait 0x1001`，约 3 s；趴下 `4` 后约 4 s 自动回 `0` | ★★★ `stand` 的读回期望三元组是 `{…,17,0x1001}`，`prone` 终态 `0`；否则 MS-2 必判超时。§6.2 状态链与 §6.5 读回表要改 |
| 5 | §7.2 V-52 | 现行指南设备状态**无 GPS 组** | 0x00100002 上报含 `GPS` 项（AOS gnss_node 接 NMEA GPS，室内 sats 0） | V-52 关闭为「存在」，字段原样上行 |
| 6 | §12.1 V-57 SVC-1～4 | 服务状态查询方式未知，`services_ok` 只能 `false` | 底盘 `nodectl` 提供 DDS 服务 `/NODECTL_QUERY_{103,104,106}`（`drdds/srv/NodeCtlQuery`，返回 state/pid），已实测可用 | `services_probe.method` 新增 `nodectl_dds`，`services_ok` 可为真值；V-57 关闭 |
| 7 | §7.4 drdds | 等官方 msg 包 | `/opt/ros/jazzy/share/drdds/msg` 60 余型（.msg/.idl）已拷回 `data/run/chassis/drdds/`；`/MOTION_INFO` 实测 20 Hz | `XBRAIN_HAVE_DRDDS` 可在编译期打开；T-DRDDS-1 偏移对比可做 |
| 8 | §7.3 | 44 码 + 开放集 | `fault_rules_CA9C.toml` 387 条已拿到 | 码表作为数据文件，开放集设计不变；V-08 可关 |
| 9 | §7.5 / §2.5 | 每条请求都有 `ErrorCode` 应答 | **轴指令无应答、无日志**；心跳/状态/步态指令有应答 | `cmd_fail_threshold` 对轴指令只能靠发送错误 + 状态读回，不能靠 ack |
| 10 | §5.5 S-06 / TF-1 | `/IMU` 无 `frame_id` | 确认为空 | 不变 |
| 11 | `11` §9.10 / §9.7 CHG-37 | `planner`/`charge_manager` 必须 `stopped` | 本机 NOS `planner`/`global_planner`/`localization`、AOS `charge_manager` **全部自启并运行**；`/NAV_CMD` 有 2 个发布者 | ⚠️ 运行约束：要么部署期经 nodectl `stop` 并写入 `deploy_verified_stopped`（OTA 后复位），要么 `11` 放宽。需裁决 |
| 12 | `11` §9.12.5 通道 B | 原厂遥控器急停独立于软件 | 手柄经 `10.21.33.11` 接收器走 **CHS-A DTLS** 进 robot_server；明文模式下**手柄连不上** | ⚠️ 通道 B 依赖 robot_server 且与我方明文互斥；正式部署前必须拿证书恢复 TLS |
| 13 | `11` §9.8.1 `sleep` | 只是一个只读字段 | 无指令 5 min **自动趴下 + 断电机**（`Sleep 1`），所有运动指令 0xE008，**无唤醒指令**；已按 C-8 关闭自动休眠 | ★★ 新失效模式：`Sleep==1` 应进 `RobotState` 并使 quadruped 拒发轴指令；`stop_reason` 闭集无 `sleep`（是否新增归 `11`，见 §7） |
| 14 | §9.7 `runtime.version` | 本项目仅 STD，PRO 分支已移除 | 硬件表 `type=STD`，但 robot_server 上报 `Version "PRO"`、`型号 CA9C_PRO`，且导航类 ASDU 可用 | 校验改为 warn，不拒绝 |
| 15 | §2.1 图、`11` §9.5 RTSP | AOS=103/NOS=104/GOS=106；RTSP `10.21.31.103` | 103 AOS / **104 GOS / 106 NOS**；RTSP 在 `10.21.33.103:8554` | 文档订正（`13` §2.1、`11` §9.5/§2.2.10） |
| 16 | §5.4 V-51 | 「特殊步态」未定义 | 未测 | 保持置零 |
| 17 | §3.3 T-DECEL、§4.4 M-28/29 | 待实测 | 未测（首次轴指令只做了原地转：比例 0.3 ≈ 0.25 rad/s，0.2 只扭身不踏步，停后回弹 3～4°） | 常规模式比例值有死区；导航模式 m/s 的最小有效值待 T-DECEL 一并测 |

## 4. 工具链与环境（ORIN 实测 2026-09-15）

| 项 | 现状 | `13` 需要 | 缺口 |
|---|---|---|---|
| OS | Ubuntu 22.04.5，JetPack R36.4.3，aarch64，8 核 | 同 | — |
| ROS 2 | **无** `/opt/ros`，无 colcon，无 ROS apt 源 | Humble rclcpp/tf2_ros/nav_msgs（通道三） | ★★★ DEP-5：装 `ros-humble-ros-base` + `rmw-cyclonedds-cpp` + `cyclonedds`（含 `idlc`）+ colcon，需用户批准（系统级安装，联网） |
| DDS | 无 CycloneDDS，无 FastDDS | 裸 CycloneDDS C API（通道二） | 同上；退路 `fastdds_vendored` 也未 vendored |
| Zenoh | `/usr/local` 有 zenoh-c（.so/.a + 头） | RT 面 | ✅ |
| JSON | 无 nlohmann；`common/` 无解析器 | CHS-A 上报解析（非实时线程） | 需引入一个解析器（建议 vendored 单头，放 `common/`，只在非 RT 线程用） |
| TLS | 无 mbedTLS 开发包 | TLS-1 本期不启用 | 不阻塞 |
| 网络 | ORIN `m20s-chassis` = 10.21.33.200，三主机可达 | 底盘面 LAN1 | ✅ |

## 5. 契约级未决（`13` §14 十条 + 本轮新增），影响编码的标出 ★

| # | 内容 | 影响编码 | 建议裁决 |
|---|---|---|---|
| ★1 | `11` §9.2 枚举仍是旧手册（软急停 2、标准 6、敏捷 8、步态 1/2/12/13/14） | 语义字符串→数值映射表 | 按现行指南 + 实测改 `11` §9.2/§9.2.4，`agile` 改为派生只读 |
| ★2 | `11` §9.3.3 `idle` action → `MotionParam=0` | 指南注「空闲不支持下发」，实测趴下后自动回 0 | 删 `idle` 或改为「趴下后自动到达的只读态」 |
| ★3 | 楼梯步态拒绝 `prone`（13 PR-1 / D-40） | ctrl 前置校验 | 进 `11` §9.3.3 |
| ★4 | `11` §9.9 写 quadruped 发 `state/pose` | 发布者唯一性 | 已由 `11` v0.7 订正，`13` §4.8 一致，无需再裁 |
| 5 | `/IMU_YESENSE` → `/IMU`（`11` 7 处） | 话题名常量 | 直接改 |
| 6 | `load_power` 无数据源（V-56） | 健康度项 | 标无数据源 |
| 7 | `illumination` 指令不存在（V-47） | 回 `E_CAPABILITY` | 已按 13 实现即可 |
| 8 | `00` NAV-100/102 仍写 Jazzy | 无 | 回填 U74 |
| ★9（新） | `Sleep==1` 的 `stop_reason` 归属 | Tier 1 分支与 `RobotState` | 二选一：新增闭集值 `sleep`（改 `11` §4.1）或并入 `mode_mismatch{actual:"sleep"}` |
| ★10（新） | 底盘自带 planner/localization/charge_manager 自启（§3 第 11 项） | 握手自检 E_BUSY 路径 | 部署期经 nodectl 停用并记录，或 `11` §9.10 放宽为「planner 停用即可」 |

## 6. 建议的开发批次（手工顺序 batch，每批 lint + 测试 + 变异体 + 提交）

| 批 | 内容 | 依赖 | 门 |
|---|---|---|---|
| B0 | 包骨架 `ros2_ws/quadruped/`：纯 CMake 核心库 + 可选 rclcpp/CycloneDDS 目标（沿用 `sensor` 范式）、`configs/quadruped.yaml` 按 §8.2 落值（端点按 §3 第 1 项）、构建/安装脚本到 `data/install/quadruped/lib/quadruped/quadruped_m20` | §3 第 1 项回填 `13` | 无 |
| B1 | CHS-A 编解码：APDU 16 B 头、ASDU JSON、hex32 码表、TCP 分帧 FR-1～5、金标 hexdump（取自实录抓包） | JSON 解析器 | 无 |
| B2 | CHS-A 链路：`chs_a_send` 单出口 + `tx_guard`、心跳搭车、端点探测、会话状态机、四路上报解析成内部结构（开放集枚举、`chs:` 前缀、Sleep 字段） | B1 | §3 第 4/13 项回填 |
| B3 | Tier 1 单函数（`11` §9.12.2 逐字）、锁、代际、`clamp` 用 `units.h`、六种 stop_reason 注入测试 + 变异体 | B2 | §5 第 9 项 |
| B4 | RT 面（zenoh-c）：信封、hello/hello_ack、estop/ack、ping/pong、ctrl/ack、六条状态 key、`RobotState`/`PowerState` 装配；`tests/` 内 relay + P1 桩 | B3 | 无 |
| B5 | 模式三元组状态机：MS-1～6、TR-1～4、PR-1、GS-1（按实测稳态 17/0x1001） | B2 | §5 第 1/2/3 项 |
| B6 | 里程计核心（ROS 无关）：10 Hz 速度 + IMU yaw 积分、§4.4 协方差闭式、四段上界，T-ODOM-2/3 离线测试 | B2 | 无 |
| B7 | 通道二：裸 CycloneDDS `/IMU` + drdds `/MOTION_INFO`（IDL→idlc），T-CHS-1a/b | ROS 2 Humble + cyclonedds 装好 | DEP-5 |
| B8 | 通道三：rclcpp Humble `/odom_quadruped` + TF，T-ODOM-1 | 同上 | DEP-5 |
| B9 | 实时化（FIFO/亲和/mlockall）、systemd 单元、台架实测：站立/趴下/模式三元组/导航模式低速轴指令、T-DECEL、T-TIER1-2 | B3～B8 | 现场 |

## 7. 需用户裁决（开工前）

> ✅ **2026-09-15 用户已逐条裁决（`99` U85）**：① ORIN 装 Humble（U74 不变；Humble/Jazzy 跨版本结论见 U85）；② `13` 已回填为 **v1.3**（提交 90f7465）；③ 契约五条已一并改 `11`（5de80e0），`stop_reason` 闭集加 `sleep`（da6be1e），`00`/`21`/`99` 回填（26da564）；④ 完全不用底盘导航栈，回充段 2 用底盘对接能力；⑤ 维持明文；⑥ JSON 解析器 vendored 单头。★ 尚未做：ORIN 实际安装 ROS 2（网络待通）；§3 第 11 项的部署期停用脚本；D-47（nodectl 查询 writer 例外）待评审。

1. **是否现在在 ORIN 安装 ROS 2 Humble + CycloneDDS + colcon**（DEP-5）。不装则 B7/B8 与 perception 都无法构建；装是系统级改动，需联网。
2. §3 第 1、4、5、6、13、14 项**回填 `13`**（我按实测改 `13` 对应节与 §12.1 状态列，提交 docs）。
3. §5 第 1、2、3、9、10 项的契约裁决（归 `11`/`99`，不在本轮代码内）。
4. 底盘自带导航服务的处置（§3 第 11 项）：部署期停用并记录，还是先不动。
5. 手柄链路：开发期保持明文（手柄不可用），正式部署前向云深处要客户端证书。
6. JSON 解析器选型：vendored 单头（nlohmann，MIT）放 `common/`，仅非实时线程使用。
