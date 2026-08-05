# 山猫 M20 S / M20 Pro 集成方案设计 — 云深处评审版

| 项 | 内容 |
|---|---|
| 文档版本 | V1.0 |
| 日期 | 2026-07-16 |
| 编制方 | 上海哈船开发团队（上装计算机集成方） |
| 评审方 | 杭州云深处科技股份有限公司 |
| 适用底盘 | 山猫 M20 S（STD）、山猫 M20 Pro（PRO） |
| 参考资料 | 《山猫 M20 S 硬件开发手册 V1.0.0》、《山猫 M20 软件接口手册(beta) V0.1.0》、《软件开发指南 V1.0.0(2026-06-15)》|

> **本文目的**：提交我方对 M20 底盘接口的**理解**与 quadruped 集成节点的**设计**，请贵司评审确认。文中凡标注【请确认】处为我方对接口的理解，凡标注【待提供】处为需贵司补充的资料或说明，汇总见第 9 节。

---

## 1. 背景与集成路线

### 1.1 项目背景
我方为机器人「大脑」（感知 + 决策 + 导航，代号 xbrain）的开发方。M20 底盘作为新一代移动 + 传感平台，采用**背载计算机**方案：

- 上装计算机：**NVIDIA AGX Orin 32GB**（定制底板、加固航插直连）。
- 分工：**xbrain 全栈运行于 Orin**（含 CUDA 感知模型与导航），M20 底盘负责本体运动、原始传感器输出与整机遥测。

### 1.2 集成路线（路线 B）
底盘板载三主机（AOS/NOS/GOS），不承载我方感知/推理负载。因此：

- 我方**不使用**底盘内建 SLAM 导航（§1.4 巡检类，Pro 专属）。
- 由 Orin 上的 xbrain 自主导航，经速度接口驱动底盘；底盘提供原始传感器与遥测。
- 底盘对我方而言 = 「移动 + 传感 + 遥测」执行体。

### 1.3 交付约束
- M20 Pro 约 2026-07-22 到货，M20 S 约 2026-08-15 到货，项目交期 2026-09-30。
- 要求 **quadruped 集成节点一套代码同时兼容 M20 S 与 M20 Pro**（见第 6 节）。

---

## 2. 运行环境与通信约定（据贵司 2026-07-16 邮件，请复核）

| 项 | 约定 | 来源 |
|---|---|---|
| 底盘 ROS2 版本 | **jazzy** | 贵司答复 ③ |
| 底盘 DDS 实现 | **FastDDS** | 贵司答复 ④ |
| ROS_DOMAIN_ID | **0** | 贵司答复 ⑤ |
| ROS_LOCALHOST_ONLY | 跨设备订阅**无需额外配置** | 贵司答复 ⑥ |
| 激光点云外部获取 | **AGX 直连航插口可直接取 `/LIDAR/POINTS`**（非直连时需 106/NOS 主机 enable `multicast-relay.service`） | 贵司答复 ⑦ |
| 监控协议服务端 | UDP+DTLS `10.21.33.103:30004`、TCP+TLS `10.21.33.103:30003`（默认加密） | 软件开发指南 §1.1.2 |
| GPS | 我方使用自带 G90 RTK，**忽略底盘 GPS** | 双方确认 |
| URDF | `github.com/DeepRoboticsLab/deep_robotics_model` 的 `M20S/urdf` | 贵司答复 ② |

> 上装 Orin 侧亦统一采用 **ROS2 jazzy + FastDDS**，与底盘一致，规避跨版本/跨 RMW 问题。

---

## 3. 整体软件架构

xbrain 内部采用 **zenoh**（JSON key-value）通信；底盘控制侧采用 **ROS2 / CycloneDDS domain 42**（xbrain 系统内部约定）；两者之间由一组 **bridge 节点**做双向转换；`quadruped_m20` 节点承接 ROS2 与 M20 底盘（监控协议 + FastDDS）之间的适配。

```
┌─ xBrain ────────────────────────────────────────────────────────────────┐
│   RNS 反应式导航  ·  VisionGuard 接管  ·  HMI                            │
└──────────────────────────────────────────────────────────────────────────┘
   下行 · zenoh.put(JSON)              上行 · zenoh.sub(JSON)
     cmd_vel         {vx,vy,vth}         robot/odom_pose      {x,y,yaw}
     cmd_vel_estop   {vx,vy,vth}         robot/odom_velocity  {vx,vy,vth}
     chassis/command {action}            robot/battery_state  {pct,volt,chg}
                                         robot/chassis_state  {型号/模式/步态/
                                         robot/faults          续航/负载/温度}
 ═══════════════ zenoh · peer · tcp 127.0.0.1:7447 · 无多播 ═══════════════
        │ 下行                                        ▲ 上行
        ▼                                             │
┌─ bridge 层  (zenoh ⇄ ROS2 / CycloneDDS domain 42) ──────────────────────┐
│  下行 zenoh → ROS2:                                                       │
│    cmd_vel_bridge         cmd_vel        → /cmd_vel_smoothed (Twist)      │
│    cmd_vel_estop_bridge   cmd_vel_estop  → /cmd_vel_estop    (Twist)      │
│    chassis_bridge         chassis/command→ /quadruped/chassis_cmd (String)│
│  上行 ROS2 → zenoh:                                                       │
│    location_bridge        /odom_quadruped /battery_state → robot/odom_*   │
│    chassis_state_bridge   /robot/chassis_state /robot_faults → robot/*    │
└──────────────────────────────────────────────────────────────────────────┘
        │ ROS2 · CycloneDDS · domain 42                     ▲
        ▼ /cmd_vel_smoothed  /cmd_vel_estop                 │ /odom_quadruped
   [velocity_smoother] → [twist_mux(estop pri400)] → /cmd_vel│ /battery_state
        │ /cmd_vel   /quadruped/chassis_cmd                 │ /joint_states /imu
        ▼                                                    │ /robot_faults
╔═ quadruped_m20  节点   ★核心·替换厂商 SDK 封装层★   jazzy·rclcpp ═══════╗
║  订阅(CycloneDDS): /cmd_vel · /cmd_vel_estop · /quadruped/chassis_cmd     ║
║  发布(CycloneDDS): /odom_quadruped · /imu/quadruped · /joint_states ·     ║
║                    /battery_state · /robot/chassis_state · /robot_faults ·║
║                    /LIDAR/POINTS · tf(odom→base_link)                     ║
║ ─────────────────────────────────────────────────────────────────────── ║
║ ①能力探测   连监控协议读 Version/Model → is_pro   (STD/PRO 分支)          ║
║ ②控制路由   /cmd_vel {vx,vy,vth} ──→ 真实轴指令 X/Y/Yaw (导航模式·m/s)    ║
║             /cmd_vel_estop        ──→ 软急停 (运动状态转换)               ║
║             /chassis_cmd  stand ─→站立 · lie ─→趴下 ·                     ║
║                           gait_flat/gait_stair ─→步态切换 · light ─→灯语  ║
║ ③里程计     运控上报10Hz(LinearX/Y,OmegaZ) + /IMU 200Hz → 100Hz 积分      ║
║             ──→ /odom_quadruped + odom→base_link TF                       ║
║ ④状态发布   基础2Hz/运控10Hz/设备2Hz → /battery_state · /joint_states ·   ║
║             /imu/quadruped · /robot/chassis_state                         ║
║ ⑤故障管理   监控协议异常 (+ /fault_aggregator 可选) ──→ /robot_faults     ║
║ ⑥传感器桥   底盘 FastDDS ──→ CycloneDDS 转发 /IMU · /LIDAR/POINTS         ║
║ ⑦协议+守护  APDU/ASDU JSON · DTLS/TLS · 心跳1Hz · 开机状态机              ║
║   型号分支: PRO→关 planner/localization · STD→无此服务 · GOS 遥测容错     ║
╚═══════════════════════════════════════════════════════════════════════════╝
   监控协议 UDP/DTLS:30004 · TCP/TLS:30003          FastDDS · AGX 直连航插
   ↓真实轴指令/状态转换/步态/急停/灯语               ↑ /IMU 200Hz
   ↑基础2Hz · 运控10Hz · 设备2Hz · 异常故障          ↑ /LIDAR/POINTS 10Hz
        ▼                                             ▲
┌─ M20 底盘  (DEEP Robotics · jazzy · FastDDS) ───────────────────────────┐
│   STD (M20 S) = AOS + NOS   ┃   PRO = + GOS + 内建SLAM导航(§1.4·我方不用) │
└──────────────────────────────────────────────────────────────────────────┘
```

---

## 4. quadruped_m20 节点设计

`quadruped_m20` 是我方需新增/重写的**唯一**底盘适配节点，运行于 Orin，jazzy + rclcpp。对上（xbrain 侧）保持既有话题名以复用现有 bridge 与运动链，对下承接 M20 的两条通道。

### 4.1 七个功能模块

| # | 模块 | 职责 | 依赖底盘接口 |
|---|---|---|---|
| ① | 能力探测 | 读基础状态 `Version`/`Model` → `is_pro`，配置服务/字段分支 | 监控协议 |
| ② | 控制路由 | ROS2 控制话题 → 底盘控制指令（见 5.1） | 监控协议 |
| ③ | 里程计积分 | 运控速度 + IMU 积分为 100Hz odom + TF | 监控协议 + `/IMU` |
| ④ | 状态发布 | 底盘遥测 → ROS2 标准/汇总话题（见 5.4） | 监控协议 |
| ⑤ | 故障管理 | 异常事件 → `/robot_faults`（DiagnosticArray） | 监控协议异常（+ `/fault_aggregator` 可选） |
| ⑥ | 传感器桥 | 底盘 FastDDS 标准消息转发到 CycloneDDS | FastDDS `/IMU`、`/LIDAR/POINTS` |
| ⑦ | 协议 + 守护 | APDU/ASDU 编解码、DTLS/TLS、心跳、开机状态机 | 监控协议 |

### 4.2 对上接口（xbrain 侧 · ROS2 CycloneDDS domain 42）

**订阅（控制入）**

| 话题 | 类型 | 来源 bridge |
|---|---|---|
| `/cmd_vel` | geometry_msgs/Twist | cmd_vel_bridge → smoother → twist_mux |
| `/cmd_vel_estop` | geometry_msgs/Twist | cmd_vel_estop_bridge（twist_mux pri 400） |
| `/quadruped/chassis_cmd` | std_msgs/String（JSON） | chassis_bridge |

**发布（状态出）**

| 话题 | 类型 | 频率 | 内容 |
|---|---|---|---|
| `/odom_quadruped` | nav_msgs/Odometry | 100Hz | 里程计位姿 + 速度 |
| tf `odom→base_link` | tf2 | 100Hz | 与里程计同步 |
| `/imu/quadruped` | sensor_msgs/Imu | ≤200Hz | 机体 IMU |
| `/joint_states` | sensor_msgs/JointState | 10Hz | 16 关节角 + 轮速 |
| `/battery_state` | sensor_msgs/BatteryState | 2Hz | 电量/电压/充电 |
| `/robot/chassis_state` | std_msgs/String（JSON） | 2Hz | 型号/模式/步态/续航/负载/温度/CPU |
| `/robot_faults` | diagnostic_msgs/DiagnosticArray | 事件 + 1Hz 心跳 | 故障事件 |
| `/LIDAR/POINTS` | sensor_msgs/PointCloud2 | 10Hz | 转发底盘雷达 |

### 4.3 对下接口（底盘侧）

- **通道一 · 本体监控协议**（UDP/DTLS `:30004` / TCP/TLS `:30003`）：承载全部控制下发与整机状态上报。
- **通道二 · FastDDS 订阅**：仅订阅底盘**标准消息** `/IMU`(sensor_msgs/Imu)、`/LIDAR/POINTS`(sensor_msgs/PointCloud2)，转发至 CycloneDDS。

---

## 5. 控制 / 状态链路

### 5.1 下行控制链（xbrain → zenoh → bridge → quadruped_m20 → 底盘）

| # | zenoh key | ROS2 话题 | quadruped_m20 → 监控协议指令（我方理解，【请确认】） |
|---|---|---|---|
| 1 | `cmd_vel` | `/cmd_vel`（经 smoother/mux） | **真实轴指令**（§1.2.6，导航模式，物理 m/s、rad/s） |
| 2 | `cmd_vel_estop` | `/cmd_vel_estop` | **运动状态转换**（§1.2.3，软急停/关节阻尼=2） |
| 3 | `chassis/command` | `/quadruped/chassis_cmd` | `stand`→站立(1) / `lie`→趴下(4) / `gait_flat`·`gait_stair`→步态切换(§1.2.4) / `light`→自定义灯语(§1.2.7) |

> 说明：我方以 `chassis/command` 承载**底盘离散控制**（站立/趴下/步态/灯语等 M20 明确支持项）。原有面向表演特技的 `gesture`（jump/shake_hand/backflip 等）通道**在 M20 版中移除**，待贵司提供 `ActionParam` 取值表后再评估是否恢复（见第 9 节）。

### 5.2 上行状态链（底盘 → quadruped_m20 → bridge → zenoh → xbrain）

| ROS2 话题 | bridge | zenoh key |
|---|---|---|
| `/odom_quadruped`、`/battery_state` | location_bridge（复用） | `robot/odom_pose`、`robot/odom_velocity`、`robot/battery_state` |
| `/robot/chassis_state`、`/robot_faults` | chassis_state_bridge | `robot/chassis_state`、`robot/faults` |

### 5.3 里程计生成（我方理解，【请确认】）
M20 不提供原生 `nav_msgs/Odometry` 与 tf。我方在 quadruped_m20 内以 **运控状态上报**（§1.3.1.2，`LinearX/LinearY` + `Body.OmegaZ`，10Hz）叠加 `/IMU`（200Hz）做 100Hz 积分，输出 `/odom_quadruped` 与 `odom→base_link` TF；关节角/轮速取自 `MotorStatus.Joint[16]`。

### 5.4 状态字段 → ROS 话题映射（节选，我方理解，【请确认】）

| 底盘字段（监控协议） | 目标 ROS 话题 |
|---|---|
| 设备状态 `BatteryList[]`（电压/电量%/温度/充电） | `/battery_state` |
| 运控状态 `RemainMile`(km)、`Payload`(kg)、`Body/Leg` 姿态、`Height` | `/robot/chassis_state` |
| 运控状态 `MotorStatus.Joint[16]` | `/joint_states` |
| 设备状态 `DeviceTemperature`、`CPU`（AOS/NOS/GOS） | `/robot/chassis_state` |
| 基础状态 `MotionState`/`Gait`/`ControlUsageMode`/`Charge`/`Version`/`Model` | `/robot/chassis_state` |
| 异常/聚合故障 `ErrorList` / `FaultEventArray` | `/robot_faults` |

---

## 6. 型号兼容：M20 S / M20 Pro 自动识别

quadruped_m20 启动即读基础状态 `Version`(STD/PRO) 与 `Model`，置 `is_pro` 标志，全代码仅 3 处分支：

1. **PRO**：启动关闭底盘 `planner` / `localization` / `charge_manager` 服务，避免与外部速度下发抢占；
2. **STD**：无上述服务，跳过；
3. **设备状态解析**：`CPU` 中 `GOS` 字段 PRO 有、STD 缺，容错处理。

控制/状态/里程计/传感器/协议五模块两型号完全一致。Pro 独有的内建 SLAM 导航（§1.4）我方不接入。

> 【请确认】我方对「M20 S 支持切换到导航模式（`Mode=1`）并经 `/NAV_CMD` 或真实轴指令（§1.2.6）接收外部速度运动，`0xE00A『导航功能仅 Pro』` 仅指 §1.4 内建 SLAM 导航任务」的理解，请贵司实机侧背书确认。

---

## 7. drdds 依赖分析

`drdds` 为底盘自定义 ROS2 消息包。本方案对底盘的**控制与状态走监控协议，传感器走 ROS2 标准消息**，故对 `drdds` 依赖极低：

| 底盘自定义话题（drdds） | 本方案替代 | 缺 drdds 影响 |
|---|---|---|
| `/NAV_CMD` | 真实轴指令（监控协议） | 无 |
| `/MOTION_INFO`(20Hz) | 运控状态上报（监控协议 10Hz） | 频率 20→10Hz，有 200Hz IMU 补 |
| `/MOTION_STATE`·`/GAIT` | 运动状态/步态切换（监控协议） | 无 |
| `/fault_aggregator` | 监控协议异常上报 | 故障结构略简 |
| `/CHARGE_STATUS` | 基础状态 `Charge` | 无 |

- **`/IMU`、`/LIDAR/POINTS` 为 ROS2 标准 `sensor_msgs`，不属于 drdds，可直接订阅。**
- **唯一功能性缺口：底盘自主充电下发**（我方理解其经 `/CHARGE`（drdds），监控协议控制类无对应指令，【请确认】）。路线 B 下我方可自实现充电对接，或待 drdds 到位后接入。

结论：**缺 drdds 不阻塞本方案开发与交付**；drdds 到位后作为可选增强（提频、结构化故障、原生 `/NAV_CMD`、底盘自主充电）。

---

## 8. URDF 使用说明与缺口

已核实 `M20S/urdf/M20S.urdf`（21.9KB + `meshes/*.STL`）：

- ✅ 本体运动学完整：`base_link` + 4 腿 ×（`hipx`/`hipy`/`knee` revolute + `wheel` continuous）= 17 link / 16 joint；关节命名 `fl/fr/hl/hr_*` 与运控状态 `Joint[16]` 顺序一致；可直接用于 `robot_state_publisher` 出 tf 及可视化。
- ⚠️ **URDF 内不含传感器 frame**（IMU、前后广角相机、前后雷达）。我方将依据《硬件手册 §1.10 本体传感器坐标》自行补 static tf。【待提供】若贵司有含传感器 link 的官方 URDF，请一并提供。

---

## 9. 待云深处确认 / 提供清单 ★评审核心★

| 编号 | 事项 | 我方诉求 |
|---|---|---|
| Q1 | `drdds` 自定义消息 ROS2 接口包 | 请提供或告知开源上传**时间表**（用于原生话题通道，非阻塞项） |
| Q2 | `ActionParam`（§1.2.4 步态切换请求中出现 `"ActionParam":12288`，但参数表未给取值） | 请提供**取值表**，并说明 M20 S/Pro **支持哪些动作/特技**（跳跃/握手/作揖/匍匐低姿态等），以及是否有独立「匍匐」步态 |
| Q3 | 导航模式与外部速度 | 请**实机背书**：M20 S（非 Pro）可进 `Mode=1` 导航模式，并经 `/NAV_CMD` / 真实轴指令(§1.2.6) 接收外部速度运动 |
| Q4 | 自主充电下发 | 监控协议是否有「下发自主充电」控制指令？若仅 `/CHARGE`（drdds），请确认；并说明是否依赖 Pro 内建导航 |
| Q5 | 官方 URDF 传感器 frame | 是否可提供含 IMU/相机/雷达 link 的完整 URDF |
| Q6 | 完整故障码表 | 请提供 `fault_rules.toml` / `common.hpp` 中 `FaultCode` 全量枚举 |
| Q7 | 直连航插取雷达 | 请说明 AGX 直连航插取 `/LIDAR/POINTS` 的具体配置（需 enable 的服务、话题清单、QoS） |
| Q8 | 监控协议加密 | DTLS/TLS 默认开启；请提供关闭方式（`robotserve` 配置）细则，并说明加密对下发时延/吞吐影响 |
| Q9 | 运控状态提频 | 运控状态上报是否可从 10Hz 提高（用于里程计积分精度）；或推荐里程计数据源 |
| Q10 | 真实轴指令等价性 | 真实轴指令(§1.2.6) 与 `/NAV_CMD` 是否完全等价（坐标系、速度上限、时序/客户端约束如 `0xE006`） |

---

## 附录 A：监控协议指令码速查（我方理解，【请确认】）

| 功能 | Type | Command | 备注 |
|---|---|---|---|
| 心跳 | 0x00100064 | 0x00000005 | ≥1Hz |
| 使用模式切换 | 0x00100002 | 0x00500002 | Mode 0常规/1导航/2辅助 |
| 运动状态转换 | 0x00100001 | 0x00200002 | 站立1/趴下4/急停2/标零5/RL控制17 |
| 运动步态切换 | 0x00100001 | 0x00300002 | GaitParam + ActionParam |
| 运动控制（归一化轴指令） | 0x00100001 | 0x00100002 | 常规/辅助模式 |
| 真实轴指令 | 0x00100001 | 0x00110002 | 导航模式，物理单位 |
| 自定义灯语 | 0x00100005 | 0x00200002 | 头/尾灯 |
| 运动 SDK 模式 | 0x00100005 | 0x00300002 | 关节话题频率 |
| 基础状态上报 | 0x00100064 | 0x00f00000 | 2Hz |
| 运控状态上报 | 0x00100001 | 0x00f00000 | 10Hz |
| 设备状态上报 | 0x00100002 | 0x00f00000 | 2Hz |
| 异常状态上报 | 0x0010007f | 0x00f00000 | 2Hz + 变更即报 |

## 附录 B：ROS2 话题清单

**底盘发布（标准消息，我方直接订阅）**
- `/IMU`（sensor_msgs/Imu，200Hz）
- `/LIDAR/POINTS`（sensor_msgs/PointCloud2，10Hz）
- `/DEPTH_IMAGE`（sensor_msgs/Image）

**底盘自定义（drdds，本方案暂不依赖）**
- `/NAV_CMD`、`/MOTION_INFO`、`/MOTION_STATE`、`/GAIT`、`/fault_aggregator`、`/CHARGE_STATUS`

---

*（完）*
