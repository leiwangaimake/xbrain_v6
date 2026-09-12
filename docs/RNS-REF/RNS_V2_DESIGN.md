# xBrain RNS V2 反应式导航算法栈设计方案

> **平台**: 云深处 山猫 M20 S (轮足四足) + Orbbec Gemini 338Le (前向固定) + 双天线 RTK + 底盘 IMU/odom
> **决策频率**: 20 Hz
> **核心范式**: **正空间导航 (Positive-Space Navigation)** — 以"已确认可通行区域"为主输入,
> 语义目标为行为调制, 几何点云为验证与补盲
> **日期**: 2026-08-07
> **状态**: 设计方案 (待评审)

---

## 0. 本文档的写作纪律

V1 文档 (6100 行, V1.0→V1.45, R31~R98 近 70 个编号修复) 暴露了一个模式:
**绝大多数修复是文档内伪代码与 spec 不自洽导致的 NameError / AttributeError /
TypeError / UnboundLocalError**。根因是把设计文档当源码用, 但文档没有编译器。

本文档改用**契约优先**写法:

| 写 | 不写 |
|---|---|
| 数据 schema (字段名/类型/单位/坐标系) | 完整函数体伪代码 |
| 数学公式与推导 | 逐行 Python |
| 不变量 (invariant) 与验收判据 | 变量初始化细节 |
| 参数表 + 物理含义 + 标定方法 | 调用栈顺序 |
| 模块职责边界 | 异常处理分支 |

算法草图只写**最小可判读片段** (≤15 行), 且标注"仅示意, 不可直接抄"。
实现细节归代码仓库, 由 `mypy --strict` + 单测保证一致性。

---

## 1. 前提条件与几何测算

### 1.1 已确认前提

| # | 项 | 内容 |
|---|---|---|
| 1 | 深度相机 | Orbbec Gemini 338Le, **固定安装, 朝正前方**, 无云台 |
| 2 | 底盘 | 云深处 山猫 M20 S; 相机安装面高于站立机身约 50 mm |
| 3 | 定位 | 双天线 RTK, 提供**绝对位置 + 绝对航向** |
| 4 | 姿态/里程 | 底盘节点提供原始 IMU + **高速高精度积分 odom 与 TF** |
| 5 | 感知模型 | 自训练 **YOLOv11m-seg** (COCO 扩展); **含可通行区域分割** |
| 6 | 地形 | 草地 / 树林 / 山地 / 矿区 / 工厂 / 园区 / 营地 / 街道 |
| 7 | 速度 | 底盘最高 **2.0 m/s** |

### 1.2 机器人几何 (M20 S 手册)

| 参数 | 值 | 来源 |
|---|---|---|
| 机身总长 Lbody | 0.820 m | §1.7 |
| 机身总宽 Wbody | 0.506 m | §1.7 |
| 站立高度 | 0.570 m | §1.1 |
| 足轮直径 | 0.180 m | §1.8 |
| 前后髋间距 Lhip | 0.625 m | §1.7 |
| **外接圆半径 R_circ** | **0.482 m** | = hypot(0.41, 0.253) |
| 背载限重 | ≤ 35 kg | §1.4 |
| 背载限高 | ≤ 300 mm | §1.4 |
| 24V 对外供电 | 4×(24V 3A) + 网口, 总 250 W | §2.3 |

> ### ⚠️ 关键修正 1: `robot_radius_m` 必须是 0.50, 不是 0.25
>
> V1 文档 V1.41 把 `robot_radius_m` 从 0.35 下调到 0.25, 理由记为
> "真机 ~0.5m 宽"。**这个推导只考虑了宽度, 忽略了 0.82 m 的机身长度。**
>
> 反应式导航中机器人会原地转身 (V1.40 leg-start orient)、横移、斜行。
> 一旦航向与运动方向解耦, **扫掠包络就是外接圆**, 半径 0.482 m。
> 用 0.25 m 做安全半径, 意味着机器人前后各 0.23 m 的机身**在安全模型里不存在**。
>
> 这几乎肯定是 V1.42/V1.44 反复出现的"贴边几厘米""拐角轻擦 7.6 cm"的直接成因 ——
> 不是 veto 力度不够, 是 veto 用的机器人模型比真机小了一圈。
>
> **本方案采用双模型**:
> - 安全层 (硬 veto): **矩形足迹** 0.82 × 0.506 m, 按当前航向旋转, 精确计算
> - 找路层 (FTG/评分): **外接圆 R = 0.50 m** (含 18 mm 余量), 保守且便宜

### 1.3 相机安装几何 (本方案核心约束)

光心高度 h_cam = 0.570 (站立) + 0.050 (安装面) + 0.025 (光心到底面) ≈ **0.645 m**

**垂直 FOV 65° (±32.5°) 决定地面盲区**:

| 下倾角 α | 地面最近可见距离 | 5 m 处可见最高点 |
|---|---|---|
| 0° | **1.01 m** | 3.83 m |
| 5° | 0.84 m | 3.25 m |
| 10° | 0.70 m | 2.72 m |
| **12°** | **0.66 m** | 2.51 m |
| **15°** | **0.59 m** | 2.22 m |
| 20° | 0.49 m | 1.75 m |

**推荐 α = 12°~15°**。理由:
- α ≤ 32.5° 时上视线仍在水平线之上, **远景地面不被截断**, 不损失长距感知
- α = 15° 时 5 m 处仍可见 2.2 m 高, 足够覆盖人、门框、树干下部
- 再大 (20°+) 收益递减且明显损失中远距场景理解, 影响 seg 模型表现

**行走俯仰扰动的影响** (α = 15° 基准):

| 机身 pitch | 有效地面盲区 |
|---|---|
| −10° (抬头) | 0.84 m |
| −5° | 0.70 m |
| 0° | 0.59 m |
| +5° | 0.49 m |
| +10° (低头) | 0.41 m |

> ### ⚠️ 关键修正 2: 盲区是**时变**的, 不能当常数
>
> 行走时盲区在 0.41~0.84 m 之间摆动。任何"假设固定盲区"的设计都会周期性失效。
> **必须逐帧按真实视锥更新栅格的"已观测/未观测"标记** (§5.2)。

**水平 FOV 90° (±45°) 决定侧向丢失**:

侧向偏移 y 的障碍, 在纵向距离 x < y 时滑出视野 (tan 45° = 1)。

| 侧向距离 | 出视野的纵向距离 |
|---|---|
| 0.5 m | 0.50 m |
| 0.8 m | 0.80 m |
| 1.5 m | 1.50 m |
| 2.0 m | 2.00 m |

**推论 (对 L2 边界绕至关重要)**: 纯边界跟随时障碍位于正侧方 (相对航向 90°),
而半视角只有 45°。**绕行过程中障碍必然不可见, 与绕行半径无关。**
这不是可靠性问题, 是几何上的不可能。解法见 §6.4 (全向 yaw 解耦) 和 §5.2 (栅格记忆)。

### 1.4 速度上限的物理推导

反应链 @ 20 Hz:

| 环节 | 延迟 |
|---|---|
| 相机曝光 → 深度出图 | ≤ 40 ms (338Le 规格) |
| perception 处理 (YOLO + 深度 + 几何) | ~35 ms |
| Zenoh 传输 + RNS tick 相位 | ~50 ms |
| 底盘指令响应 | ~100 ms |
| **合计 t_react** | **≈ 0.225 s, 取 0.25 s** |

安全停车距离约束:

```
d_free ≥ v · t_react + v² / (2a) + margin
```

解出 v 上限:

```
v_max = −a·t_react + sqrt( (a·t_react)² + 2a·(d_free − margin) )
```

取 margin = 0.3 m, t_react = 0.25 s, 不同减速度下:

| 可用减速度 a | d_free=1.0 | 1.5 | 2.0 | 2.5 | 3.0 | 4.0 |
|---|---|---|---|---|---|---|
| 1.5 m/s² | 1.12 | 1.56 | 1.91 | 2.00 | 2.00 | 2.00 |
| 2.0 m/s² | 1.25 | 1.75 | 2.00 | 2.00 | 2.00 | 2.00 |
| **2.5 m/s²** | **1.35** | **1.90** | **2.00** | 2.00 | 2.00 | 2.00 |
| 3.0 m/s² | 1.43 | 2.00 | 2.00 | 2.00 | 2.00 | 2.00 |

**结论**:
1. 跑满 2.0 m/s 需要 **d_free ≥ 2.0 m** 的确认可通行距离 (a = 2.5 假设下)
2. 地面盲区 0.59 m 意味着**静止起步瞬间 d_free 至多 0.59 m** → 起步限速 ~0.8 m/s
3. **速度必须是 d_free 的连续函数, 不能是 V1 的三档 (clear/sparse/dense)**

`a` 必须实测标定 (§9.1)。轮足平台高速急停有翻覆风险, 建议初期取 1.5, 实测后上调。

---

## 2. 架构总览

### 2.1 范式转变: 从"枚举障碍"到"确认可通行"

V1 架构的致命缺陷在 §8.3:

```
if not obstacles_2d: return None    # → 上层走 Direct Mode 满速直冲
```

**"我没看见障碍"和"那里没有障碍"被编码成了同一个信号。** 这是 nav#2 (−0.53 m 撞穿)、
V1.43 (1944 tick / −0.57 m) 的共同根因, 而 ObstacleMemory 三轮迭代
(V1.34 建记忆 → V1.38 前向豁免 TTL → V1.43 `_near_hold` 解耦) 都是在用状态补偿这个编码错误。

**V2 的核心不变量**:

> **三态强制**: 空间中每一处必须被明确标记为 `FREE` / `BLOCKED` / `UNKNOWN` 之一。
> **`UNKNOWN` 永远不等于 `FREE`。** 机器人只允许进入 `FREE`, 进入 `UNKNOWN` 必须限速,
> `BLOCKED` 硬禁止。

这一条是整个 V2 的地基。它同时解决了:
- FOV 盲区 (盲区 = UNKNOWN, 天然限速)
- 漏检 (未确认 = UNKNOWN, 不是 FREE)
- 未标注物体 (几何 BLOCKED)
- 非结构化地形 (可通行 seg 直接给 FREE, 不需要枚举草叶石块)

### 2.2 三通道感知

| 通道 | 数据源 | 语义 | 作用 |
|---|---|---|---|
| **T — 可通行 (正空间)** | YOLO-seg 可通行 mask ∩ 深度 ∩ 几何平坦性 | "这里确认能走" | **主输入**, 产生 FREE |
| **S — 语义目标** | YOLO-seg 目标检测/分割 + 深度 + 跟踪 | "这是什么, 该怎么处理" | 行为调制 (让行/绕行/穿越) |
| **G — 几何** | 深度点云 (非地面 / 非语义) | "这里有东西 / 这里是坑" | 补盲 + **对 T 的否决** |

**融合逻辑 (保守取交)**:

```
cell = FREE      当且仅当   T说可通行 AND G说几何平坦 AND 无负障碍 AND 无S类阻挡
cell = BLOCKED   当         G说有立体障碍 OR S说是阻挡类 OR 检出负障碍 OR T说不可通行
cell = UNKNOWN   其余 (无深度 / 视锥外 / 遮挡 / 超量程 / 观测过期)
```

**为什么必须是 AND**: 草地 seg 会把埋在草里的石头、树桩、井盖标成可通行;
矿区的浮土 seg 看着平整但下面是坑。**语义给"可以走"的假设, 几何给"确实平"的证据, 缺一不可。**

反之, seg 说不可通行的地方直接 BLOCKED, **不需要几何确认** (保守方向不做二次判断)。

### 2.3 双层表征

```
┌──────────────────────────────────────────────────────────────┐
│ perception_service (C++, AGX Orin, 20 Hz)                    │
│   深度 → 地面/非地面分离 → 三通道                             │
│   输出: 极坐标观测剖面 + 语义目标 + 负障碍扇区                │
└───────────────────────┬──────────────────────────────────────┘
                        │ Zenoh, ~30 KB/帧
                        ▼
┌──────────────────────────────────────────────────────────────┐
│ RNS (Python, 20 Hz)                                          │
│                                                              │
│  ┌────────────────────────────────────────────────┐         │
│  │ L-A: 局部三态栅格 (map 系锚定, 整数格滚动)      │         │
│  │      12.8 × 12.8 m @ 0.1 m = 128 × 128         │         │
│  │      每格: state / z / trav / class / t_seen    │         │
│  │      **RTK 锚定 ⇒ 零漂移 ⇒ 记忆可达 30 s**      │         │
│  └───────────────────┬────────────────────────────┘         │
│                      │ 每 tick 导出                          │
│  ┌───────────────────▼────────────────────────────┐         │
│  │ L-B: 全向极坐标可通行剖面                       │         │
│  │      360 bin × 1°, 每 bin: d_free / d_block     │         │
│  │      + 间隙场 (distance transform)              │         │
│  └───────────────────┬────────────────────────────┘         │
│                      ▼                                       │
│              五层决策 (§6) → 全向 cmd_vel                    │
└──────────────────────────────────────────────────────────────┘
```

**为什么栅格是 map 系锚定的**: 这是 RTK 给的最大红利。绝大多数机器人的局部地图靠
里程计补偿, 误差累积让记忆只能维持 1~2 秒 (V1 的 `ttl_s = 1.2` 正是这个物理限制)。
有厘米级绝对位姿 + <0.5° 绝对航向后, 栅格锚在 map 系, **记忆时长由环境变化速率决定,
不由传感器精度决定**。静态障碍可以记 30 秒。原地转身完全不破坏地图。

**为什么导出极坐标剖面而不是直接用栅格**: FTG / gap 评分 / 滞回 / pure pursuit 全部是
角度域算法。极坐标剖面是它们的自然输入, 且 360 个 float 的定长结构消除了 V1 "障碍数量
爆炸 → tick 超时"的风险 (V1 的 gap 计算是 O(N_obstacle), 几何通道接入后 N 会从 5 涨到 50)。

---

## 3. 坐标系与时间同步

### 3.1 坐标系定义 (遵循 REP-105)

| 系 | 定义 | 性质 |
|---|---|---|
| `map` | RTK 经纬度 Mercator 投影, ENU | 绝对, 跨重启, **不连续** (RTK 跳变) |
| `odom` | 底盘积分里程 | 局部, **连续光滑**, 缓慢漂移 |
| `base_link` | M20S {Body} 系: 前 +X, 左 +Y, 上 +Z | 手册 §1.6 |
| `cam_link` | 338Le 光心, 光轴 +Z, 右 +X, 下 +Y (光学系) | |
| `cam_base` | 相机在 base_link 下的位姿 | **需标定, 见 §9.2** |

**变换链**: `map → odom → base_link → cam_base → cam_link`

- `T_odom_base`: 底盘节点提供, 高频 (≥200 Hz), 光滑 —— **用于 ego-motion 补偿**
- `T_map_odom`: 由 RTK 估计, 低频 (~5 Hz) 平滑更新 —— **用于绝对锚定**
- `T_base_cam`: 静态标定 (含下倾角 α)

> ### ⚠️ 关键修正 3: `T_map_odom` 必须是完整 SE(2) 且低通
>
> V1 的 `GeoTransform` 注释写着 "V1 假设 odom 旋转可忽略" (§8.2 R56)。
> odom 的航向漂移在四足上是**每分钟数度**量级 (足底打滑), 忽略它会让 5 m 外的
> 障碍产生米级横向误差。
>
> 正确做法: 用 RTK (位置 + 双天线航向) 与 odom 做**低通融合**估计 `T_map_odom`:
> ```
> T_map_odom_est = T_map_base(RTK) · T_odom_base⁻¹
> T_map_odom ← slerp/lerp(T_map_odom, T_map_odom_est, β)     β ≈ 0.02 @ 20Hz
> ```
> 低通是必须的: RTK 有厘米级抖动和偶发跳变, 直接用会让栅格每帧抖动、边缘糊掉。
> **RTK 从 FIX 降级 (FLOAT/SINGLE) 时冻结 β = 0**, 靠 odom 惯性维持, 并触发限速。

### 3.2 6-DOF 姿态的来源分工

RTK 给 `x, y, z, yaw`, **不给 roll / pitch**。而深度图投影到地面系需要完整 6-DOF。

| 自由度 | 主源 | 备源 | 说明 |
|---|---|---|---|
| x, y | RTK | odom 积分 | 绝对, 厘米级 |
| z | RTK | 高程栅格 | 精度较差, 主要用相对高度 |
| yaw | RTK 双天线 | odom | 绝对, <0.5° |
| **roll, pitch** | **底盘 IMU (重力向量)** | 338Le IMU | **只用重力方向, 不做姿态积分 ⇒ 无陀螺漂移** |

**误差敏感度**: pitch 偏 10° 时, 5 m 外地面点高度算错
`5 × tan(10°) = 0.88 m` —— 直接把平地判成陡坡, 或把 0.8 m 的障碍判成地面。
**这比 V1 文档里任何一个 R 修复都致命。**

底盘 IMU 位于 (60.8, −32.8, −40.65) mm, 与 base_link 有固定偏置, 需在 URDF 中正确建模。

### 3.3 时间同步 (16 cm 级误差源)

三个时钟: 深度 30 fps / RTK 50 Hz / odom 200 Hz / 决策 20 Hz。

若用"最新位姿"投影"刚到的深度帧", 误差为:
- 平移: 2.0 m/s × 40 ms = **8 cm**
- 旋转: 1.5 rad/s × 40 ms = 3.4°, 在 5 m 处 = **30 cm 横向**

**强制要求**:

1. perception 必须使用**曝光中点时间戳** `t_capture` (不是接收时间)
2. 维护 500 ms 的 `T_odom_base` 位姿环形队列, 按 `t_capture` **插值**取位姿
3. 深度与 RGB 必须同帧 (338Le 硬件同步), `|t_depth − t_rgb| ≤ 5 ms`
4. 338Le 与主机时钟通过 PTP 或 SDK 时间戳对齐; **首次上电必须做一次时钟偏置标定**
5. perception 在输出中**回传它使用的位姿** (`pose_used`), RNS 校验偏差, 超阈值丢帧

**不变量 I-1**: 任何投影到 map/odom 系的几何量, 其位姿必须来自 `t_capture` 插值,
不得使用 `now()`。CI 检查: 禁止在投影路径上出现 `time.time()` / `now()`。

---

## 4. perception_service 接口要求

这是本方案对上游的完整需求。**RNS 的一切能力上限由这一节决定。**

### 4.1 模型输出要求

#### 4.1.1 可通行区域分割 (最高优先级)

自训练 YOLOv11m-seg 必须输出**至少一个**可通行类, 建议按地形细分:

| 类名 | 典型场景 | 期望 M20S 行为 |
|---|---|---|
| `traversable_hard` | 沥青/水泥/铺装/夯实土路 | 全速 |
| `traversable_soft` | 草地/土路/砂石/矿区浮土 | 限速 (打滑风险) |
| `traversable_rough` | 碎石/树林地表/山地 | 显著限速 |
| `traversable_slope` | 明显坡面 | 限速 + 坡度校验 |

**训练要求**:
- **边界质量优先于面积精度**。可通行区与障碍的交界线位置误差直接转成机器人的
  碰撞裕度误差。建议在损失函数中对边界像素加权。
- **必须包含负样本**: 水面/深色阴影/反光地面容易被误判为可通行铺装, 这是最危险的
  假阳性。水坑误判会直接导致机器人涉水。
- **远场标注要保守**: 图像上部 (对应远距离) 的可通行标注不确定性大, 宁可标 unknown。
- 每种目标地形至少 2000 张标注, 覆盖不同光照/季节/湿度。

> **注意 338Le 是红外带通版本**: IR 双目图像与 RGB 图像的纹理表现差异很大。
> 若模型训练用的是 RGB, 而深度来自 IR 通道, 两者的**边界不重合** (尤其在低纹理区)。
> 建议: seg 跑在 RGB 上, 但 mask 投影到深度前先做 **RGB↔Depth 对齐 (D2C)**,
> 并对 mask 做 3~5 px 腐蚀吸收对齐误差。

#### 4.1.2 目标检测/分割

在 COCO 基础上, 建议确保覆盖以下对 M20S 有特殊行为需求的类:

| 行为类 | 类别举例 | RNS 行为 |
|---|---|---|
| `yield` | person, dog, cat, 各类车辆 | 停车礼让, 不绕 |
| `block` | 固定设备, 集装箱, 建筑, 围栏 | 绕行 |
| `wall_follow` | 墙面, 长围挡, 边坡 | L2 边界绕 |
| `traverse` | 低矮草丛, 落叶堆, 浅水洼 | 可穿越, 限速 |
| `hazard` | 明火, 深水, 陡崖边缘, 高压设备 | **硬禁止 + 额外膨胀** |
| `unknown` | 未匹配类 | 保守绕行 (等同 block) |

`hazard` 类是 V1 完全缺失的。矿区/营地场景必须有。

### 4.2 深度处理要求

| 项 | 要求 | 理由 |
|---|---|---|
| **采集分辨率** | **1280×800 @ 30 fps** (退让下限 848×530) | 见下方"分辨率不可逆"专题 |
| **宽高比** | **必须 1.600 (16:10)** | 16:9 模式是垂直裁切, 会损失 5° 垂直 FOV |
| **保持 organized** | 不得提前转无序点云 | 地面分割/聚类可从 O(n log n) 降到 O(n) |
| 相机侧滤波 | 开 decimation + spatial + temporal | 省 Orin 算力 |
| 距离门限 | 0.25 ~ 8 m, 超出标 invalid | 规格推荐 0.25~6 m, 8 m 外精度不足但可用于粗略 FREE |
| **无效像素必须显式标记** | invalid ≠ 无穷远 | **这是 UNKNOWN 语义的来源, 绝不能填 0 或 max** |
| 置信度 | 若 SDK 提供, 逐像素透传 | 用于加权 |

#### 专题: 分辨率是不可逆的, 但 FOV 与分辨率无关

**常见误解**: 降分辨率会缩小视野。**不会。** FOV 由镜头与传感器物理尺寸决定,
只要**宽高比不变**, 降采样只是用更少像素表达同一画面, 视野一点不少。
338Le 的 1280×800 / 848×530 / 424×265 宽高比全部严格等于 1.600 ——
厂商特意用 848×**530** 而非常见的 848×480, 正是为了保持不裁切。

> **⚠️ 但 16:9 模式要警惕**: 若 SDK 中出现 848×480 一类 1.767 宽高比模式,
> 多半是**垂直裁切**, 垂直 FOV 从 65° 降到 ~60°, 下倾 15° 时地面盲区从
> 0.59 m 恶化到 0.65 m。垂直 FOV 是本平台最金贵的资源 (直接决定地面盲区
> 与负障碍发现距离), **任何非 1.600 模式必须先确认是缩放还是裁切**。

**分辨率真正影响的是深度精度与细障碍存活率**, 且**不可逆** ——
深度由相机内 MX6800 在所选分辨率上做立体匹配, 先降采样等于永久损失。

双目误差 `δz = z²·δd / (f·b)`, 而像素焦距 f 与分辨率成正比:

| 采集分辨率 | f (px) | 深度误差 @2m / @4m / @6m |
|---|---|---|
| **1280×800** | 640 | 16 mm / 66 mm / **148 mm** |
| 848×530 | 424 | 25 mm / 99 mm / 223 mm |
| 424×265 | 212 | 50 mm / 199 mm / **447 mm** |

细障碍存活 (10 cm 直径杆件 @ 6 m):

| 采集分辨率 | 像素宽 |
|---|---|
| 1280 | 13.6 px |
| 848 | 9.0 px |
| 424 | **4.5 px** — 双目匹配边缘状态, 可能整根无有效深度 |

矿区钢筋、树林细枝、营地拉索正是"最容易漏 + 撞上后果最严重"的一类。
且 §1.4 的速度策略要求 d_free ≥ 2 m 才能跑满速, 6 m 处 447 mm 的测距误差
会让整个速度决策的输入不可信。

**因此按用途分路, 而不是统一降采样**:

| 路径 | 输入 | 处理 | 理由 |
|---|---|---|---|
| **安全 / 几何剖面** | **全分辨率** | **角度域 MIN 归约到 181 bin** | 归约本身即降维; MIN 保守且**细障碍只要 1 个有效像素就存活** |
| 地面拟合 / 聚类 | 降到 424×265 | 曲面拟合 | 本质是求平均, 降采样无损, 省算力 |
| YOLO-seg | RGB 1280×800 | letterbox → 640 | 模型输入限制; 喂 1280 不会更准, 除非 imgsz 也提到 1280 (算力 ×4, 会超时) |

> **不变量 I-4**: `d_block[θ]` 必须由**全分辨率深度的 MIN 归约**得到,
> 不得先做空间降采样再归约。先降采样会在第一步就把细障碍平均掉,
> 之后任何算法都救不回来。MIN 归约是 O(n) 单遍扫描, 全分辨率下仅数毫秒。

**带宽核算**: 1280×800 × 16 bit × 30 fps = 492 Mbps, 千兆口可承载 (占约 50%)。
若实测延迟不达标, 退到 848×530 (216 Mbps), 精度损失约 1.5 倍仍可用;
**但不得退到 424×265 作为采集分辨率**。

**可选增强**: 可通行 seg 用 640 全图 + 对图像上半部 (远场) 裁 ROI 再跑一次 640,
远场角分辨率翻倍。远场恰是速度决策最需要的区域。代价是推理 ×2, 需评估延迟。

**不变量 I-2**: 深度图中的 invalid 像素在整条流水线中必须保持 "unknown" 语义,
> 不得在任何环节被隐式转成 "far" 或 "free"。
> 玻璃、水面、强反光、超量程、投射器阴影都会产生成片 invalid ——
> **在 V1 架构下这些区域会被当成空旷全速冲过去。**

### 4.3 地面分割 (地形自适应)

按地形复杂度分三级, 运行时可切换:

**Level 1 — IMU 重力先验 + 高度阈值** (工厂/园区/街道)
- 用重力向量把点云旋转到重力对齐系, 地面判定退化为 1-DOF 高度问题
- 阈值 `|z − z_ground| < 0.08 m`
- 成本: O(n), <2 ms

**Level 2 — Himmelsbach 径向线拟合** (草地/营地/缓坡)
- 极坐标扇区 (建议 72 扇区 × 5°) 内拟合折线, 逐段判定
- 能吃斜坡和缓起伏, 比单平面 RANSAC 稳得多
- 成本: O(n), ~5 ms

**Level 3 — Patchwork++ 同心区域拟合** (山地/矿区/树林)
- 同心圆分块 + 区域地面平面拟合 + 地面似然估计
- 成本: ~10 ms, 有成熟 C++ 开源实现

> **不要用单平面 RANSAC**。若非要用, **必须加法向约束** (只接受与重力夹角 < 20° 的平面),
> 否则在树林/矿区会把边坡或大面积墙面拟合成"地面"并全部切掉。

**关键联动**: 可通行 seg mask 应作为地面分割的**先验**——mask 内的点优先作为地面模型的
内点候选。这让两个通道互相加强, 而不是各算各的。

### 4.4 输出 Schema 2.0

三个 Zenoh key。**总带宽 ≈ 600 KB/s @ 20 Hz**, 建议用 CDR/protobuf 而非 JSON。

#### Key 1: `perception/profile` — 极坐标观测剖面 (核心, 定长)

这是本方案与 V1 最大的接口差异。它是**射线观测的压缩表达**,
既是 FREE 的来源, 也是栅格 ray-casting 的输入。

```
ProfileMsg {
  schema_version : "2.0"
  t_capture      : float64        # 曝光中点, epoch s
  t_publish      : float64
  frame          : "base_link"    # 已用 t_capture 位姿从光学系转到 base_link
  pose_used      : Pose6D         # perception 实际使用的 T_map_base (供 RNS 校验)

  # 前向 ±90°, 1° 分辨率, 共 181 bin, bin[0] = −90° (右), bin[180] = +90° (左)
  # 角度定义: base_link 系, 前 = 0°, 逆时针为正
  n_bins         : 181
  d_free         : float16[181]   # 该方位"确认可通行"的最远距离 (m); 0 = 立即不可通行
  d_block        : float16[181]   # 该方位最近的立体障碍距离 (m); inf = 该方位无障碍
  h_block        : float16[181]   # 该障碍相对地面的高度 (m), 用于可跨越性判断
  src            : uint8[181]     # 位掩码: bit0=seg, bit1=geom, bit2=semantic, bit3=negative
  conf           : uint8[181]     # 0~255, 该 bin 的观测置信度
  terrain        : uint8[181]     # 地形类: 0=unknown 1=hard 2=soft 3=rough 4=slope
  slope_deg      : int8[181]      # 该方位可通行段的最大坡度 (deg), 无数据 = −128
}
```

**`d_free` 的定义 (perception 侧的核心计算)**:

沿该方位角射线, 从 0.25 m 起逐段推进, 直到遇到以下任一条件即停止, 返回停止处距离:

| 停止条件 | 说明 |
|---|---|
| 可通行 seg mask 边界 | 语义否决 |
| 局部坡度 > `slope_max` | 几何否决 |
| 相邻采样高差 > `step_max` | 阶跃 (台阶/路沿) |
| 局部粗糙度 > `rough_max` | 高度方差 |
| 立体障碍点 | 有非地面点 |
| **深度 invalid 连续段 > 0.3 m** | **未知, 不可外推** |
| **负障碍** | 射线未按预期命中地面 |
| 超过 8 m | 量程 |

**必须保证的不变量 I-3**: `d_free ≤ d_block` 恒成立。
`d_free` 语义是"确认可走到这里", `d_block` 是"这里有东西"。
两者之间的区域是 UNKNOWN (可能是遮挡/无深度/未确认地形)。

**这个 UNKNOWN 带是整个安全体系的核心**, 绝不能通过令 `d_free = d_block` 消除。

#### Key 2: `perception/objects` — 语义目标 (变长)

```
ObjectsMsg {
  schema_version : "2.0"
  t_capture      : float64
  frame          : "base_link"
  objects[] {
    track_id       : uint32
    class_name     : string
    behavior       : enum{yield, block, wall_follow, traverse, hazard, unknown}
    confidence     : float32
    footprint_xy   : float32[2][N]   # base_link 地面凸包 (CCW), 3 ≤ N ≤ 12
    z_min, z_max   : float32         # **高度区间, 不是质心** (见修正 4)
    r_near         : float32         # 凸包到机体最短距离 (ESTOP 用)
    velocity_xy    : float32[2]      # base_link 系, m/s
    stable_frames  : uint16
    depth_quality  : enum{good, fair, poor}
  }
}
```

> ### ⚠️ 关键修正 4: 高度过滤必须用 z 区间, 不能用质心
>
> V1 §8.2 用 `d.centroid_odom[2]` 判断 `min_height_m=0.05 ~ max_height_m=1.5`。
> 后果:
> - 质心 1.8 m 的横杆 / 叉车货叉 / 低垂树枝 → **整个被 `continue` 丢弃**
> - 一段 0~2.5 m 的墙, 质心 1.25 m → 通过; 但换成 0~3 m 的墙, 质心 1.5 m → **临界丢弃**
>
> 正确判据: **[z_min, z_max] 与机器人高度带 [0.03, 0.75] 求交非空**
> (0.75 = 站立 0.57 + 背载 0.30 的一半余量, 按实际上装高度调整)。
>
> 这一条对几何通道尤其致命——几何 cluster 的质心高度分布比 YOLO 检出的物体宽得多,
> 按质心过滤会被大批误杀。

#### Key 3: `perception/status` — 健康心跳 (1 Hz)

```
StatusMsg {
  fps_depth, fps_infer   : float32
  latency_ms             : float32      # t_publish − t_capture
  invalid_pixel_ratio    : float32      # ★ 深度失效比例, >0.4 时 RNS 强制限速
  ground_seg_level       : uint8        # 当前用的地面分割等级
  ground_inlier_ratio    : float32      # 地面模型内点率, <0.5 表示地形模型不可信
  clock_offset_ms        : float32      # 相机与主机时钟偏置
  intrinsics             : {fx, fy, cx, cy, w, h}
  extrinsics_base_cam    : Pose6D
}
```

`invalid_pixel_ratio` 是 V1 完全没有的关键健康指标。逆光、雨雾、强反光地面都会
让它飙升, 此时**整套感知的可信度整体下降**, 必须触发全局限速而不是等到撞上。

### 4.5 负障碍检测 (V1 完全缺失)

**这是 YOLO 永远无法提供的一类, 也是四足最危险的一类。**

原理: 按当前地面模型, 预测每条射线"本应"在多远命中地面; 实际返回明显更远、
或成片 invalid, 则该方位存在下沉。

```
预期命中距离 (平地):  d_expect(θ_v) = h_cam / tan(α + θ_v)
实际返回:            d_actual
若 d_actual > d_expect · (1 + ε)  或  成片 invalid
   → 该射线落入下沉区, 深度 = (d_actual − d_expect) · tan(α + θ_v)
```

在起伏地形上, `d_expect` 改用局部地面模型 (Level 2/3 输出的地面高度场) 预测, 而非平地假设。

**输出**: 写入 `d_free` (在负障碍边缘截断) 并置 `src` 的 bit3。

**物理限制必须写进设计**: 垂直 FOV 决定负障碍最早只能在 **0.6~1.0 m** 外发现。
2.0 m/s 下反应距离需要 1.3 m —— **来不及**。
因此:
- 高速档 (>1.2 m/s) 只允许在 `terrain = hard` 且**栅格中该区域为历史 FREE** 时使用
- 陌生区域 (栅格 UNKNOWN) 硬上限 1.0 m/s
- 这是相机安装角度 α 的另一个权衡点: α 越大负障碍看得越近, 但远景越少

### 4.6 perception 性能预算 (AGX Orin, 20 Hz = 50 ms)

| 阶段 | 预算 | 备注 |
|---|---|---|
| 深度获取 (1280×800) + 相机侧滤波 | 6 ms | 大部分在 338Le 的 MX6800 内完成 |
| D2C 对齐 | 3 ms | CUDA |
| **全分辨率角度 MIN 归约 → 181 bin** | **3 ms** | O(n) 单遍, 保细障碍 (不变量 I-4) |
| 降采样到 424×265 (仅供地面拟合/聚类) | 1 ms | CUDA |
| **YOLOv11m-seg TensorRT** | **20 ms** | FP16 @640; INT8 可降到 ~12 ms |
| 地面分割 (Level 2) | 5 ms | organized, O(n) |
| 非地面聚类 + 凸包 | 5 ms | Depth Clustering, range image 角度判据 |
| 可通行剖面合成 (seg ∩ 几何 ∩ 负障碍) | 3 ms | 与 MIN 归约结果合并 |
| 负障碍检测 | 2 ms | 与剖面生成合并 |
| 序列化 + 发布 | 3 ms | |
| **合计** | **46 ms** | 余量 4 ms |

**风险点**: YOLOv11m-seg 是本预算最大的单项。若实测超 25 ms, 三个选项:
1. INT8 量化 (精度损失需评估, 可通行 seg 对量化较敏感)
2. 输入降到 512, 保留 640 用于远场
3. **seg 跑 10 Hz, 几何通道跑 20 Hz** —— 推荐。几何变化快 (近距安全), 语义变化慢。
   RNS 侧的栅格天然处理不同频率的输入。

---

## 5. RNS 算法实现

### 5.1 模块划分

```
xbrain/navigation/
  grid/
    local_grid.py        # L-A 三态栅格 (numpy, 全向量化)
    raycast.py           # 极坐标剖面 → 栅格更新
    traversability.py    # 坡度/阶跃/粗糙度 → 可通行评分
  polar/
    profile.py           # L-B 栅格 → 360° 极坐标剖面导出
    clearance.py         # 间隙场 (distance transform)
  decision/
    ftg.py               # L0 Follow-The-Gap
    commit_thread.py     # L1 commit-穿
    boundary_follow.py   # L2 Tangent-Bug 边界绕
    backtrack.py         # L3 兜底
    arbiter.py           # 五层仲裁 + 滞回
  motion/
    speed_policy.py      # 速度 = f(d_free, terrain, health)
    yaw_policy.py        # 全向 yaw 解耦策略
    pure_pursuit.py      # 全向纯跟踪
    safety_filter.py     # 矩形足迹硬 veto
  io/
    perception_sub.py    # 三 key 订阅
    pose_buffer.py       # 位姿环形队列 + 插值
```

**硬性纪律**: `grid/` 和 `polar/` 中**不允许出现逐格 Python 循环**。
全部 numpy 向量化 / scipy.ndimage。这是 20 Hz 可行性的前提。

### 5.2 L-A: 局部三态栅格

#### 数据结构

```
尺寸: 128 × 128 @ 0.1 m  →  12.8 × 12.8 m, 机器人居中 (±6.4 m)
锚定: map 系, 原点对齐到 0.1 m 整数倍
```

| 层 | dtype | 含义 |
|---|---|---|
| `state` | uint8 | 0=UNKNOWN, 1=FREE, 2=BLOCKED |
| `z` | float32 | 地面高度 (map 系), UNKNOWN 时为 nan |
| `z_var` | float32 | 高度方差 (粗糙度) |
| `trav` | uint8 | 可通行评分 0~255 |
| `terrain` | uint8 | 地形类 |
| `cls` | uint8 | 语义类 id (BLOCKED 时有效) |
| `t_seen` | float32 | 最后观测时刻 (monotonic) |

内存: 128×128×22 B ≈ **360 KB**。

#### 整数格滚动 (关键实现细节)

机器人移动时, **绝不做插值重采样** —— 那会让栅格每帧糊一次。做法:

```python
# 仅示意
shift = np.floor((robot_xy_map - grid_origin_map) / cell) .astype(int) - center
if shift.any():
    for layer in all_layers:
        layer[:] = np.roll(layer, -shift, axis=(0,1))
    # 滚入的新边缘置 UNKNOWN
    grid_origin_map += shift * cell
```

因为栅格锚在 map 系的 0.1 m 整数网格上, 滚动永远是整数格, 零插值损失。
**这是 RTK 带来的第二个红利** (第一个是无漂移)。

#### Ray-casting 更新 (每帧)

输入是 perception 的 181-bin 剖面。对每个 bin θ:

| 距离区间 | 写入 |
|---|---|
| `[0, d_free]` | `state = FREE`, 写 `z`/`terrain`/`trav`, 刷新 `t_seen` |
| `(d_free, d_block)` | **不写** (保持原值 —— 这是 UNKNOWN 带, 可能是遮挡) |
| `d_block` 处 ±1 格 | `state = BLOCKED`, 写 `cls`/`h_block`, 刷新 `t_seen` |
| `> d_block` | **不写** (被遮挡) |

**动态障碍的清除靠 ray-casting 天然完成**: 人走开后, 该方位的 `d_free` 变大,
射线扫过原来的 BLOCKED 格并改写为 FREE。**不需要 V1 那套 CV 外推 + TTL。**

**向量化**: 181 bin × 平均 40 格 = ~7000 次格写入。用预计算的极坐标→索引查找表
(LUT, 启动时建一次), numpy fancy indexing 一次完成, 实测应 < 2 ms。

#### 非对称衰减

```
FREE    → UNKNOWN   当 (now − t_seen) > T_free    (默认 15 s)
BLOCKED → UNKNOWN   当 (now − t_seen) > T_block   (默认 45 s)
```

**必须非对称**: 忘记 FREE 只会让机器人变保守 (限速); 忘记 BLOCKED 会撞。
因为有 ray-casting 主动清除, `T_block` 可以设得很长而不产生幽灵障碍。

#### 视锥标记 (处理时变盲区)

每帧根据当前 6-DOF 位姿计算真实视锥 (含 pitch 扰动), 只有落在视锥内的格
才参与本帧更新。视锥外的格保持原状并继续衰减。

这样 §1.3 的"盲区在 0.41~0.84 m 间摆动"被自然处理: 抬头帧近处不更新, 低头帧补上。

### 5.3 可通行性评分

从栅格的 `z` 层每 tick 计算三个几何指标 (scipy.ndimage 卷积, 全向量化):

| 指标 | 计算 | M20S 阈值 | 依据 |
|---|---|---|---|
| **坡度** | 3×3 Sobel 梯度 → `atan(|∇z|)` | 警戒 15°, 上限 **25°** | 轮足爬坡能力, 反应式导航保守取值 |
| **阶跃** | 3×3 邻域 `max(z) − min(z)` | 警戒 0.06 m, 上限 **0.12 m** | 足轮直径 0.18 m, 轮式模式可越约 0.1 m |
| **粗糙度** | 5×5 窗口 `std(z)` | 警戒 0.03 m, 上限 **0.06 m** | 影响机身稳定与 RTK 天线姿态 |

综合评分 (取最保守):

```
trav_geom = min( f_slope, f_step, f_rough )        每项 ∈ [0,1], 线性映射 警戒→上限 = 1→0
trav      = trav_geom × w_terrain[terrain_class]
```

`w_terrain`: hard 1.0 / slope 0.8 / soft 0.7 / rough 0.5 / unknown 0.3

**`trav = 0` 的格降级为 BLOCKED**, 参与 FTG 与安全层。

> 这一步是 V1 完全没有的。V1 只有"障碍/非障碍"二元, 无法表达"能走但要慢"。
> 而草地/山地/矿区的绝大部分决策恰恰是这一类。

### 5.4 L-B: 极坐标剖面导出 + 间隙场

#### 全向剖面 (360 bin × 1°)

从栅格沿每个方位角射线扫描, 输出:

```
PolarState {
  d_free_360    : float32[360]   # 连续 FREE 的最远距离
  d_block_360   : float32[360]   # 最近 BLOCKED 距离
  d_unknown_360 : float32[360]   # 首次遇到 UNKNOWN 的距离
  trav_360      : float32[360]   # 沿该射线到 d_free 的最低 trav 值
}
```

同样用启动时预建的 LUT + numpy, ~2 ms。

**关键: 这是全向的**, 而相机只有前向 90°。后向和侧向的数据来自栅格记忆。
**这就是 L2 边界绕在窄 FOV 下能工作的唯一途径** (§1.3 推论)。

#### 间隙场 (Clearance Field)

```python
# 仅示意
blocked = (state == BLOCKED)
clearance = scipy.ndimage.distance_transform_edt(~blocked) * cell_size
```

128×128 的 EDT 约 1.5 ms。产出一张"每格到最近障碍的距离"的图。用途:

1. **实现"2 米绕行"要求** (§6.3 评分项)
2. 安全层快速查询 (O(1) 查表, 替代 V1 遍历障碍列表)
3. L2 边界绕的等距线跟随

> **UNKNOWN 格是否算 blocked?** 计算两张场:
> `clearance_hard` (只对 BLOCKED) 用于安全 veto;
> `clearance_soft` (对 BLOCKED ∪ UNKNOWN) 用于路径评分。
> 前者防撞, 后者鼓励走在已知区域内。

### 5.5 与 V1 的接口迁移

| V1 概念 | V2 对应 | 说明 |
|---|---|---|
| `FilteredDetection` | `ObjectsMsg.objects[]` | 字段精简, z 改区间 |
| `ProjectedObstacle.polygon_2d_map` | 栅格 BLOCKED 层 | 不再传多边形列表给 FTG |
| `ObstacleMemory` (整个模块) | **删除**, 由栅格替代 | 连同 V1.38/V1.43 全部补丁 |
| `recall` / `recall_near` | 栅格的两种查询 | 不再是两套状态 |
| `compute_obstacle_angular_span` | 剖面导出 | O(N_obs) → O(1) 定长 |
| `filter_corridor` | 沿目标方向的剖面切片 | |
| `apply_clearance_filter` | 矩形足迹 + `clearance_hard` 查表 | 见 §6.5 |
| `GeoTransform` | `pose_buffer` + REP-105 TF | 完整 SE(2), 低通 |

---

## 6. 决策层

沿用 V1.37 的五层骨架 (这部分设计是对的), 修正触发判据与输入源。

### 6.1 层次总览

| 层 | 触发 | 动作 | 输入 |
|---|---|---|---|
| **L0 FTG** (主线) | 存在朝向目标的可行 gap | 选最优 gap 穿越 | `d_free_360` + 间隙场 |
| **L1 commit-穿** | M-line 被挡 + 卡死 + 无可穿 gap | 承诺穿一侧直到障碍滑到身侧 | 同上 + 栅格记忆 |
| **L2 边界绕** | 无缝可穿, 障碍呈连续边界 | 沿等间隙线绕 | **栅格全向剖面** (关键) |
| **L3 backtrack** | L2 超时/失败 | 沿轨迹后退 + 停 | 轨迹日志 |
| **YIELD** (并行) | `behavior=yield` 目标进入礼让距离 | 停车等待 | `objects` |
| **ESTOP** (并行) | `min(r_near)` 或 `d_block` < 阈值 | 立即停 | 剖面 + objects |

**降级顺序**: L0 → L1 → L2 → L3, 每级有独立超时。
**L2 → L0 的离开判据**: Tangent-Bug 的 `d_followed < d_reached`。

> V1.38 的 "orbit-before-reverse" 原则保留且强化: 无 gap 时**一律先 L2 绕**,
> 只有 L2 也失败才 L3 后退。因为现在栅格给了 L2 真正的全向输入, L2 的成功率会大幅提高。

### 6.2 L0 FTG 在极坐标剖面上的实现

```
1. 取前向扇区 [−90°, +90°] 的 d_free_360
2. 障碍膨胀: 对每个 d_block < d_horizon 的 bin, 向两侧膨胀
   Δθ = asin( (R_circ + min_clearance) / d_block )
3. 在膨胀后的剖面上找连续的可行 bin 段 = gap 候选
4. 过滤: gap 的最小可穿宽度 = 2 · d_gap · sin(Δθ_gap/2) ≥ min_threadable_gap
5. 对每个 gap 计算评分 (§6.3), 取最高
6. 滞回判定 (§6.6)
```

**`min_threadable_gap`**: 机身宽 0.506 m + 两侧各 0.15 m 操作余量 = **0.81 m**。
V1 设的 1.0 m 偏保守但可接受; 建议 **0.85 m**, 并在 `terrain=hard` 时可降到 0.75 m。

### 6.3 Gap 评分 (含"2 米绕行"要求)

```
score = w_a · s_angle + w_w · s_width + w_c · s_clearance + w_t · s_terrain + w_k · s_kinematic
```

| 项 | 定义 | 建议权重 |
|---|---|---|
| `s_angle` | `1 − |θ_gap − θ_target| / π` | **0.35** |
| `s_width` | `clamp(gap_width / (2·min_threadable), 0, 1)` | 0.10 |
| **`s_clearance`** | **`clamp(min_clearance_along_gap / desired_clearance, 0, 1)`** | **0.20** |
| `s_terrain` | 沿 gap 路径的最低 `trav` 值 | 0.20 |
| `s_kinematic` | `1 − |θ_gap − θ_current| / π` (换向代价) | 0.15 |

> ### 关于"2 米就开始绕"的正确落位
>
> **`desired_clearance_m = 2.0` 必须放在评分项里, 绝不能放进安全层做硬约束。**
>
> 硬约束的后果: 一个 1.0 m 宽的门, 走中线时离两侧各 0.5 m, **永远违反 2 m 约束**
> → 所有门、走廊、树林间隙全部判定为不可通过。这会直接把 V1.41 刚解决的
> twogate 窄门问题以更严重的形式带回来。
>
> 放在评分项的行为是: **空旷处自然走出 2 m 间距** (clearance 分高的 gap 胜出),
> **窄处该过还是过** (那是唯一通路时角度项压过来)。这才是"2 米就开始绕"的正确语义。
>
> `min_clearance_along_gap` 直接查 §5.4 的间隙场, 零额外成本。

### 6.4 L2 边界绕: 用全向底盘解 FOV 死结

§1.3 已证明: 纯边界跟随时障碍在正侧方 90°, 超出半视角 45°, **必然不可见**。

M20 S 是 3-DOF 全向底盘 (vx + vy + wz 独立)。**这是解开死结的唯一硬件手段。**

L2 期间切换 yaw 策略:

| 模式 | yaw 目标 | 适用 |
|---|---|---|
| `motion_aligned` (默认) | 运动方向 | L0 直行/穿越 |
| **`obstacle_aligned`** | 障碍质心方向 | L2 绕行, 障碍全程在视野中心 |
| **`bisector`** | 运动方向与障碍方向的角平分线 | L2 绕行的折中, 两者各落在 FOV 边缘 |
| `target_aligned` | 目标方向 | 接近终点精对位 |

**推荐 L2 用 `bisector`**: 障碍与行进方向各偏 ~45°, 都在 FOV 边缘可见。
纯 `obstacle_aligned` 会让行进方向完全变盲, 虽有栅格兜底但风险更高。

**L2 期间硬限速 0.6 m/s** —— 横移 + 侧视的组合本身就是高风险机动。

> V1.39 的 "face-before-move" 被废弃 (实测"很恶心"鬼畜来回), V1.40 退回
> "每段起步一次性转正"。V2 不需要这些补丁: 有了栅格, **盲转不再危险**;
> yaw 可以完全按感知需求调度, 而不是被安全约束绑架。

### 6.5 安全层 (硬 veto)

> ### ⚠️ 关键修正 5: 圆模型必须换成矩形足迹
>
> V1 §8.11.1: `clearance = dist(robot, obs_centroid) − obstacle_radius − robot_radius`,
> `default_obstacle_radius_m = 0.30`。
>
> 对人、桩、锥筒尚可。对墙、路沿、连片障碍**完全失效**:
> 一段 5 m 的墙, 质心在墙中点, 按 r = 0.3 算,
> **机器人贴到离墙面 5 cm 时, clearance 仍显示有 2 m 富余**。
>
> 这是最后一道保命 veto。它必须是精确的。

**V2 做法** (O(1), 无遍历):

```
1. 按当前航向 ψ 生成矩形足迹 0.82 × 0.506 m 的采样点 (16 个边界点, 预计算模板)
2. 预测下一 tick 的位姿 (v · Δt), 生成预测足迹
3. 查 clearance_hard 场, 取所有采样点的最小值 = c_min
4. if c_min < min_clearance:
     - 去掉 cmd 中朝向最近障碍的法向分量 (投影)
     - 沿 −法向加 recover 速度 (按缺口比例, 上限 recover_speed)
   if c_min < hard_stop_clearance:
     - 保留切向分量, 总平移限到 recover_speed (慢速受控逃逸)
```

V1.37 修的"hard-stop 死锁"教训保留: **hard-stop 区内不得 `return (0,0,0)`**,
否则贴脸时永久冻死。保留切向逃逸。

**新增**: `clearance_hard` 场是从栅格算的, 所以它天然包含**视野外的记忆障碍**。
V1 需要 `recall_near` 单独喂安全层 (V1.43), V2 不需要——栅格本身就是全向的。

### 6.6 滞回

沿用 V1 §8.5 思路, 但参数按 20 Hz 调整:

```
切换新 gap 需同时满足:
  |θ_new − θ_current| > angle_threshold (10°)  或  score_new > score_current × (1 + 20%)
  且  now − last_switch > min_hold_ms (400 ms @ 20Hz = 8 tick)
```

### 6.7 速度策略 (替代 V1 三档)

```
v_geom     = −a·t_react + sqrt( (a·t_react)² + 2a·max(d_free_ahead − margin, 0) )
v_terrain  = v_max × w_terrain[terrain] × trav_along_path
v_health   = v_max × health_factor        # 见下表
v_layer    = 层限速 (L0: 2.0, L1: 1.0, L2: 0.6, L3: 0.3, YIELD: 0)

v_cmd = min(v_geom, v_terrain, v_health, v_layer, v_max)
```

**`d_free_ahead`**: 不是单条射线, 而是沿规划方向 ±(机身半宽对应角) 扇区的**最小** `d_free`。

**健康因子**:

| 条件 | health_factor |
|---|---|
| 正常 | 1.0 |
| `invalid_pixel_ratio > 0.4` | 0.5 |
| `ground_inlier_ratio < 0.5` | 0.5 |
| RTK 非 FIX | 0.4 |
| perception 帧龄 > 150 ms | 0.3 |
| 前方栅格 UNKNOWN 占比 > 60% | 0.5 |
| perception 帧龄 > 400 ms | **0 (软停)** |

**底盘死区** (V1.45 的发现, 保留): 非零小速度必须抬到底盘可执行下限
(vx/vy 0.05 m/s, wz 0.01 rad/s), 否则指令被底盘吃掉 → 真机冻死。

**加速度限制** (V1 缺失, 2.0 m/s 下必须有):
```
|Δv| ≤ a_max · Δt = 2.5 × 0.05 = 0.125 m/s per tick
|Δω| ≤ 2.0 rad/s² × 0.05 = 0.1 rad/s per tick
```
EMA 平滑保留但系数按 20 Hz 调整 (`alpha = 0.4`, V1 的 0.3 是给 10 Hz 的)。

---

## 7. 参数表

```yaml
# configs/v6/rns_v2.yaml

robot:
  length_m: 0.820          # M20S 手册 §1.7
  width_m: 0.506
  circumscribed_radius_m: 0.50    # ★ 不是 0.25 (见 §1.2 修正 1)
  height_standing_m: 0.570
  payload_height_m: 0.30
  wheel_diameter_m: 0.18

camera:
  mount_height_m: 0.645
  mount_pitch_deg: 15.0    # ★ 下倾, 必须与实际安装一致 (见 §9.2)
  mount_x_m: 0.35          # base_link 前向偏移
  hfov_deg: 90.0
  vfov_deg: 65.0
  min_range_m: 0.25
  max_range_m: 8.0
  capture_width: 1280       # ★ 采集全分辨率, 不可先降采样 (不变量 I-4)
  capture_height: 800       # ★ 宽高比必须 1.600, 禁用 16:9 模式
  capture_fps: 30
  reduce_width: 424         # 仅供地面拟合/聚类路径
  reduce_height: 265

grid:
  size_cells: 128
  cell_size_m: 0.10
  ttl_free_s: 15.0
  ttl_blocked_s: 45.0      # ★ 非对称
  robot_height_band_m: [0.03, 0.75]   # ★ z 区间过滤 (见 §4.4 修正 4)

traversability:
  slope_warn_deg: 15.0
  slope_max_deg: 25.0
  step_warn_m: 0.06
  step_max_m: 0.12
  rough_warn_m: 0.03
  rough_max_m: 0.06
  terrain_weight:
    hard: 1.0
    slope: 0.8
    soft: 0.7
    rough: 0.5
    unknown: 0.3

ftg:
  min_threadable_gap_m: 0.85        # 机身宽 0.506 + 2×0.15 余量
  desired_clearance_m: 2.0          # ★ 评分项, 非硬约束 (见 §6.3)
  horizon_m: 6.0
  weights: {angle: 0.35, width: 0.10, clearance: 0.20, terrain: 0.20, kinematic: 0.15}

hysteresis:
  angle_threshold_deg: 10.0
  score_improve_pct: 20.0
  min_hold_ms: 400

safety:
  min_clearance_m: 0.30
  hard_stop_clearance_m: 0.10
  recover_speed_m_s: 0.30
  estop_distance_m: 0.60            # ★ 用 r_near / d_block, 不用 depth_m
  yield_distance_m: 2.0

motion:
  tick_hz: 20
  max_vx: 2.0
  max_vy: 0.8
  max_wz: 1.2
  accel_max: 2.5                    # ★ 必须实测标定 (§9.1)
  t_react_s: 0.25
  brake_margin_m: 0.30
  smooth_alpha: 0.4
  deadband: {vx: 0.05, vy: 0.05, wz: 0.01}
  layer_speed_cap: {L0: 2.0, L1: 1.0, L2: 0.6, L3: 0.3}
  unknown_region_cap: 1.0           # 栅格 UNKNOWN 区域硬上限

pose:
  buffer_ms: 500
  map_odom_lpf_beta: 0.02
  rtk_fix_required_for_full_speed: true

perception:
  max_age_ms: 150
  hard_fail_age_ms: 400
  invalid_pixel_ratio_warn: 0.4
  ground_inlier_ratio_warn: 0.5
```

---

## 8. 失败模式

| 码 | 触发 | 处理 |
|---|---|---|
| `FAIL_PERCEPTION_STALE` | 帧龄 > 400 ms | 软停 (斜坡减速), 不硬刹 |
| `FAIL_PERCEPTION_DEGRADED` | invalid_ratio > 0.7 持续 3 s | 软停 + 上报 |
| `FAIL_RTK_LOST` | 非 FIX 持续 > grace | 降级到 odom 惯性 5 s, 之后软停 |
| `FAIL_NO_GAP` | L0/L1/L2/L3 全失败 | 停车 + 上报 (不撞远比撞穿安全) |
| `FAIL_ARRIVAL_TIMEOUT` | 单 waypoint 超时 | 停车 + 上报 |
| `FAIL_TERRAIN_IMPASSABLE` | 目标方向 trav 全 0 | 停车 + 上报 |
| `FAIL_NEGATIVE_HAZARD` | 负障碍在硬停距内 | 立即停 + 后退 0.5 m |
| `FAIL_TILT_LIMIT` | 机身 roll/pitch > 30° | 立即停 (防翻) |
| `FAIL_CLOCK_DESYNC` | clock_offset > 100 ms | 软停 + 上报 (投影不可信) |

**统一退出纪律** (V1.11 的教训保留): 任何路径退出 `navigate_to` 前必须发一次软停车指令,
不得依赖 watchdog 硬刹车。

---

## 9. 标定与验证

### 9.1 必做的实测标定

| 项 | 方法 | 影响 |
|---|---|---|
| **`accel_max`** | 平地 2.0 m/s 全速下发 0, 记录减速曲线, 取 90 分位 | 整个速度策略 |
| `t_react` | 下发阶跃指令, 用外部相机测实际起动延迟 | 速度策略 |
| 相机下倾角 α | 标定板 / 已知平面拟合 | 地面分割全局偏差 |
| `T_base_cam` | 标定板 + 机器人多姿态 | 投影精度 |
| 时钟偏置 | 闪烁 LED 同步 或 PTP | 时间同步 |
| `step_max` | 逐级抬高障碍物实测越障 | 可通行判据 |
| `slope_max` | 逐级斜坡实测 (含湿滑) | 可通行判据 |

### 9.2 分场景验收判据

每个地形至少 20 次任务, 记录:

| 指标 | 目标 |
|---|---|
| 最小间隙 (min clearance) | **≥ 0 恒成立** (零撞击) |
| tick 超时率 (>50 ms) | < 0.1% |
| UNKNOWN 区域进入速度 | ≤ 1.0 m/s 恒成立 |
| 冻结 tick 数 | 0 |
| 到达成功率 | 结构化 ≥95%, 非结构化 ≥85% |
| 负障碍检出率 | 静态测试 100% (0.3 m 深, 0.5 m 宽) |

**必测的对抗场景**:

1. **窄门** 0.85 m / 1.0 m / 1.2 m 宽 (验证 `desired_clearance` 不阻塞窄缝)
2. **目标正后方单障碍** (V1 的 #3 局部最小死循环)
3. **凹形障碍** (验证 L2)
4. **障碍贴近后滑出 FOV** (V1 的 −0.53 m 撞穿场景)
5. **路径导航拐角转身** (V1.43 的 −0.57 m 场景)
6. **玻璃/水面/强反光地面** (验证 UNKNOWN 不被当 FREE)
7. **下沉台阶 / 装卸台边缘** (负障碍)
8. **草丛藏石** (验证 seg ∩ 几何的 AND 逻辑)
9. **逆光 / 雨雾** (验证 health_factor 降速)
10. **动态行人横穿** (验证 ray-casting 清除, 无幽灵)

### 9.3 仿真优先

V1 的 web sim + rns_sim 是很有价值的资产, 应扩展:
- 加入**深度相机视锥模型** (含 90°×65° 与 pitch 扰动) —— V1 sim 只有 65° 半锥的粗略模型
- 加入**深度失效模型** (随机 invalid 斑块, 模拟反光/超量程)
- 加入**地形高程** (坡度/阶跃/粗糙度), 而不只是 2D 圆柱障碍
- 加入**底盘死区模型** (V1.45 已做, 保留)

---

## 10. 落地路线

| 阶段 | 内容 | 周期 | 验证 |
|---|---|---|---|
| **P0** | V1 三处纯 bug 修 (矩形足迹 / ESTOP 用 r_near / z 区间过滤) | 3 d | 现有 sim, 预期直接吃掉 7.6 cm 轻擦 |
| **P1** | 时间同步 + 6-DOF 位姿链 + `pose_buffer` | 1 w | 静态标定板, 投影误差 < 5 cm @ 5 m |
| **P2** | perception Schema 2.0 剖面通道 (先只用几何, 不接 seg) | 2 w | 离线 rosbag 回放 |
| **P3** | L-A 栅格 + L-B 剖面 + 间隙场, 替换 ObstacleMemory | 3 w | sim 全场景 + 场景 4/5 |
| **P4** | 可通行 seg 接入 + 三通道 AND 融合 + 可通行性评分 | 3 w | 场景 6/8, 草地实测 |
| **P5** | 速度策略 + 负障碍 + L2 全向 yaw | 2 w | 场景 7/9, 2.0 m/s 实测 |
| **P6** | 全地形调参 | 4 w+ | §9.2 全部 |

**P0 可以立刻做, 不依赖 338Le 到货。** 它是后面所有阶段的地基, 且能立即改善现有系统。

---

## 附录 A: V1 → V2 关键修正清单

| # | V1 问题 | 位置 | V2 修正 | 严重度 |
|---|---|---|---|---|
| 1 | `robot_radius_m = 0.25` 只算宽度, 忽略 0.82 m 机身长 | V1.41 | 矩形足迹 + R_circ 0.50 | **高** |
| 2 | 安全层用圆模型 + 默认半径 0.30, 对墙类失效 | §8.11.1 | 矩形足迹查间隙场 | **高** |
| 3 | 空 detections → Direct Mode 满速 | §8.3 | 三态强制, UNKNOWN ≠ FREE | **高** |
| 4 | 高度过滤用质心, 丢悬空障碍 | §8.2 | z 区间求交 | **高** |
| 5 | ESTOP 用 `depth_m` 而非 `r_near` | §8.3 | 改用 `r_near` / `d_block` | 中 |
| 6 | `T_map_odom` 忽略 odom 旋转 | §8.2 R56 | 完整 SE(2) + 低通 | 中 |
| 7 | 无时间同步, 用最新位姿投影 | 全文 | `t_capture` 插值 | 中 |
| 8 | 无负障碍概念 | 全文 | §4.5 独立通道 | 中 |
| 9 | 无 roll/pitch, 只有 RTK 的 yaw | 全文 | IMU 重力向量 | **高** |
| 10 | 三档速度, 与感知距离无关 | §8.9 | `v = f(d_free)` 连续 | 中 |
| 11 | 无加速度限制 | 全文 | ±0.125 m/s per tick | 中 (2 m/s 下变高) |
| 12 | 无地形可通行性概念 (只有障碍/非障碍) | 全文 | 坡度/阶跃/粗糙度评分 | **高** (非结构化地形) |
| 13 | 无感知健康降速 | 全文 | `health_factor` | 中 |
| 14 | ObstacleMemory 三轮补丁 | V1.34/38/43 | 栅格替代, 模块删除 | — |
| 15 | L2 边界绕在窄 FOV 下几何不可能 | §8.3.1 原则5 | 栅格记忆 + `bisector` yaw | **高** |
| 16 | (本方案初稿自身错误) 统一降采样到 424×265 | §4.2 | 全分辨率采集 + 角度 MIN 归约, 仅拟合路降采样 | **高** |

---

## 附录 B: 未解决问题 / 需要决策

1. **`accel_max` 未知** —— 整个速度策略挂在这个数上。P0 阶段就应该实测。
2. **YOLOv11m-seg 在 AGX Orin 上的实测延迟** —— 若 > 25 ms, 需走 §4.6 的降级方案。
3. **338Le 红外带通与 RGB seg 的对齐误差** —— 需实测 D2C 后的边界偏差, 决定 mask 腐蚀量。
4. **矿区/山地的 RTK 可用性** —— 山谷、边坡、树冠下可能长期 FLOAT。若 RTK 不可靠,
   栅格的"无漂移"红利会打折, 需要评估是否补 VIO 或降低 `ttl_blocked_s`。
5. **是否值得加第二颗相机 (后向)** —— 338Le 的 M8 A-code 支持多机同步, 24V 供电口还剩 3 个,
   总功耗预算 250 W 充裕。后向相机能让原地转身和 L2 绕行从"靠记忆"变成"靠实时观测"。
   建议在 P3 完成后重新评估: 如果栅格方案的实测效果达标就不加, 达不到就加。
