# Perception → RNS 交付条件与验收口径的答复（第三轮）

**XBRAIN_V6 · 2026-09-11**

| 项 | 内容 |
|---|---|
| 回应对象 | 贵方《Perception → RNS：交付条件与验收口径确认》（2026-09-11，第 2 版，16:49 收件） |
| 本文性质 | ① 按 **Q1～Q5** 逐项答复，每项给「已有定义 / 决定 · 附件或地址 · 负责人 · 时间 · 未闭合影响哪项验收」② 两项 09-11 负责人裁定（发布节拍 · 输出分辨率）的**正式契约文本** ③ 我方**未兑现承诺**（W-11 对手件 · 首版 class_map · `11`/`20` 正文）的交付日期 ④ 我方侧如实登记的接线缺口 |
| 来件身份核对 | 贵方持有的 `19`（`adece765…`）· 09-08 reply（`cce0f724…`）· 冻结 interface（`69116411…`）三份 SHA256 与我方仓库**当前文件逐字一致**（提交 `9b5b4ad`，tag `rns-v2.0`）⇒ 贵方手上就是最新版；**缺的只是 `11` / `20` 正文与可执行对手件**，Q1 给地址 |
| 真源 | `11` v1.9 §3.1B · `19` v1.2 · `20` v1.34。★ 本文 §Q2 / §Q4 的裁决已落 **`11` v2.1 · `19` v1.3 · `20` v1.35 · `12` v0.9**，与本文同一提交序列，tag **`perception-r3-20260911`**，提交号与 SHA 见附录 C |
| 一句话 | **五项都能先给最小可用部分**；真正阻塞贵方的只有两件：`11`/`20` 正文（本文给公开地址，立即可读）与 W-11 对手件（**2026-09-16** 交付） |

---

## 〇、三件贵方没问、但会改变交付判断的事实（先读）

| # | 事实 | 对贵方的后果 |
|---|---|---|
| **F-A** | ★★★ **生产机不是 AGX Orin，是 Jetson Orin NX 16 GB**（Super 开发套件形态，`nvidia-l4t-core 36.4.3` = JetPack 6.2，CUDA 12.6.11，功耗档 `MAXN_SUPER`，8× Cortex-A78AE @ 1.98 GHz，实存 15.3 GB，`nvdla0`/`nvdla1` 两个 DLA 节点在，Ubuntu 22.04.5）。★ **当前未安装 TensorRT 与 ROS 2**（`/opt/ros` 不存在，无 `libnvinfer`）；zenoh router `zenohd v1.5.0`。★ 同机常驻 **llama-server（qwen2.5-3b gguf，链接 `libggml-cuda`）与 sherpa-onnx ASR** 两个 AI 服务 ⇒ 「GPU 基本空闲」在生产机**不成立**，显存/内存与贵方共享 | 贵方 09-10 的 25 Hz / 间隔 P99 59–68 ms 是在开发机 `.23` 上测的；**请回告 `.23` 的型号**，否则无法换算到 NX 的 DLA 时钟与 CPU 数量。engine 必须在生产机构建（贵方已持此立场，一致） |
| **F-B** | ★★ **RNS 侧接线现状（如实）**：`profile` 的 T-50 / T-51 计龄、`invalid_pixel_ratio` 限速、`raw` 速度拒用、bit0 无分割限速 —— **已接**；`objects` 的 **T-52 计龄、`status` 的 T-53 计龄、`extrinsic_calibrated == false` 拒航门、三条 key 的生产 Zenoh 订阅器 —— 未接**（现只有 DTO 与回放注入面 `xbrain/p1_motion/rns/inputs.py`）。W-11 模拟样例集与消费 stub **尚未交付** | 这些是我方的欠账，🚫 不是贵方的等待项；已登记 `20` #20-22 ～ #20-25（附录 B），**2026-09-16** 与 W-11 同批交付 |
| **F-C** | ★★ 贵方词表 `class_schema_94.json` 里有两条**不是物体**的类：`traversable_area`(92) 是 T 通道本体，`pit`(93) 是负障碍语义类 | 前者 **🚫 不得作为 `ObjectsMsg` 目标发出**（落 `11` v2.1），RNS 侧同时加防御（收到即丢弃并审计）；后者映射 `hazard`（附录 A） |

---

## Q1．真源与最小对手件、阶段签收

### Q1.1 真源地址与版本（立即可读，不必等离线包）

| 项 | 内容 |
|---|---|
| 仓库 | **`https://github.com/leiwangaimake/xbrain_v6`（公开可读）**。贵方当前核对基线 = 提交 **`9b5b4ad`**（tag **`rns-v2.0`**，2026-09-11）；`11`/`19`/`20` 三册在 `docs/` 下同名文件 |
| 该提交下的 SHA256 | `docs/11-接口契约.md` = `07b1b8f3c504de656c6731689c659133acd57b3dd8f7e2b308a556e99feb2fe2` · `docs/20-RNS反应式导航软件系统详细设计.md` = `9a1b203d5ec2b6b54ba01cfbe6da80bad1ce57cfb48400f99b634e1e9ce9324b` · `19` = 贵方已持有的 `adece765…` |
| 下一版 | 本文裁决已落 `11` v2.1 / `19` v1.3 / `20` v1.35 / `12` v0.9，**与本文同一提交序列**，tag **`perception-r3-20260911`**；提交号与逐文件 SHA 回填在附录 C，离线包 `perception_pack_20260911.tar.gz` 随本文交付。★ 贵方按 tag 或离线包读即可，🚫 不需要我方逐条转述 |
| `rid` / schema 版本 / null 样例 | 沿用 09-08 reply §3.1（`rid` 生产 `gj-001`、dev `dev`；`schema` 字段进位规则；缺值一律 `null`，空目标帧 `"objects": []` 合法）—— 不重述 |

### Q1.2 三条 key 的 schema · QoS · 超时 · 样例 · class_map

| 项 | 定义处（已有，直接读） | 本次补交 |
|---|---|---|
| schema / 字段 / 单位 / 闭集 | `11` §3.1B.1（`ProfileMsg`）· §3.1B.2（`ObjectsMsg`，含 `velocity_status` 五值、`r_near`、退化凸包）· §3.1B.3（`StatusMsg`）—— 三段各内嵌一条 json5 样例 | src 逐位定义与样例（§Q4.1）；`traversable_area` 禁入 objects（F-C） |
| QoS | `11` §2.4.2 档位表：**Q1** = `congestion=drop · priority=real_time · reliability=best_effort · express=true · 订阅端 Ring(1)`；**Q2** = `drop · data_high · reliable · express=false · Ring(4)`。绑定：`profile`/`objects` → Q1，`status` → Q2（§2.2.1）。★ **QOS-C1：`rt/` 前缀一律 `drop`，🚫 `block`**（§2.4.3）。C++ 侧同一张表在 `common/include/xbrain/zenoh/qos_profiles.h`，两面会话配置在 `session_config.h`（RT-C1～C3 五字段） | 无 |
| 超时 | `11` §1.6.1：**T-50** `profile` 年龄 > 300 ms ⇒ 限速档；**T-51** > 1 s ⇒ 零速（不锁定）；**T-52** `objects` > 500 ms ⇒ 全部 BLOCKED 按禁止类；**T-53** `status` > 3 s ⇒ 按最坏 `invalid_pixel_ratio` 限速 | 消费端接受规则与年龄起算（§Q2.3）落 `11` §3.1B.5 v2.1 |
| ★ 模拟报文 + 消费 stub（W-11） | ⚠️ **未交付（我方欠账）** | **2026-09-16 交付**：`scripts/dev/perception_sim.py`（七场景样例生成 ＋ 可选按 Q1/Q2 QoS 真发 Zenoh）· `tests/perception/samples/*.json`（正常 / 全 UNKNOWN / 无分割 / 外参未标 / TF 过期 / 时钟重置 / 断供）· `tests/perception/test_consumer_contract.py`（**每个样例 → RNS 判决对表**：限速档 / 零速 / 拒航 / 全 BLOCKED 禁止类 / bit0 限速）· `scripts/dev/perception_rx_audit.py`（接收端按 `t_capture` 帧身份记账，§Q3 对表用） |
| ★ 首版 class_map | ⚠️ 未交付 | **本文附录 A 即草案**（按贵方 94 类词表逐类给出，含 proxy / 保留类 / 未知类 / T 类策略）；贵方对「真实能力」逐行打叉即可 |
| 1280×800 输出决定的同步 | ★ **RNS 接口面对图像分辨率零依赖**：`profile` 走原生 640×400 深度（PROF-3，不变），`objects` 是 `base_link` 几何，`status` 无图像字段。★ 受影响的**只有两处文档注记**：`19` §3.2 「mask 在彩色像面（实测 1920×1080）」→ 改「mask 所在像面（现 1280×800）」；`19` §3.2A 现路径注记同改。★ **XBRAIN 侧不存在任何消费 1920×1080 规范化流的接口**（视频出口只有 MED-2，见 §Q3.3）⇒ **无迁移安排，无 1080p 硬编码依赖** | `19` v1.3 两处注记 |

### Q1.3 正式适配所需的最小构建入口 · 依赖 · 配置读取

| 项 | 内容 |
|---|---|
| 部署形态 | systemd 单元 `deploy/systemd/xbrain-perception.service`：`ExecStart=/opt/xbrain_v6/data/install/perception/lib/perception/perception_node`（colcon install 布局），`Requires=xbrain-config-freeze.service`，`After=` 两个 zenohd 单元。★ 参照包：`ros2_ws/sensor`（rtk_driver，同为 C++17、`-Wall -Wextra -Werror -Wpedantic`、纯 CMake 可由 colcon 编） |
| `common` 地基库 | header-only INTERFACE 目标 `xbrain::common`（`add_subdirectory(common)` 即可链），**零 ROS 依赖**（`CLAUDE.md` §5.3）。贵方要用的四个头：`config/yaml_lite.h`（**解析产物读取器**：`require_*` 访问器对缺键 / `null` / 类型不符**一律抛出并带点分键路径** —— 这就是 PSC-2 的实现载体，🚫 不提供任何回退）· `clock/mono_clock.h`（CLK-C1）· `zenoh/session_config.h` ＋ `zenoh/qos_profiles.h`（两面会话与 QoS 冻结表）· `envelope/message_age.h`（年龄计算） |
| 依赖 / 锁定版本 | 生产机现状：`zenohd 1.5.0`（贵方 zenoh-c 须为 1.x 同协议大版本）· CUDA 12.6.11 · JetPack 6.2（L4T 36.4.3）。★ **TensorRT 与 ROS 2 Humble 由我方装机**（JetPack 6.2 配套 TensorRT 10.3，与贵方开发环境同版本），**2026-09-18 前完成**，实机功能交付前可用 |
| 解析产物样例 | 冻结线把 `configs/<proc>.yaml` 展开为 `/run/xbrain/resolved/<proc>.yaml` ＋ `MANIFEST.json`（sha256 登记，`10` §5.4）。⚠️ 两条如实：**`configs/perception.yaml` 骨架尚未建**（`19` §15 W-8 归我方）；**冻结线的进程清单尚未纳入 `perception.yaml`（也未纳入 `rns.yaml`）**，登记 `20` #20-26 —— 所以 **2026-09-16 与 W-11 同批交付**的是：骨架按 `19` §8.2 键表全 `null`（已裁定值填入并注出处）＋ 一份**手工产物形态**的 dev 解析样例 ＋ `yaml_lite` 读取示例；纳入冻结线另排 |
| lint | `scripts/lint/charset_lint.py`（仓库根执行）；`ros2_ws/perception` 现列于 `THIRD_PARTY_SNAPSHOTS` 排除项，**随 W-1 收编合入时删除**，🚫 不作为内部优化前置（与贵方一致） |

### Q1.4 分阶段签收（采纳贵方三段，补验收用例与接收人）

| 阶段 | 交付物 | 验收用例（编号见 `19` §14） | 接收人 | 通过判据 |
|---|---|---|---|---|
| ① 接口联调交付 | 三条 key ＋ 序列化 ＋ 正常/退化模拟消息 ＋ 消费端行为对表 | A19-ENC-1 · A19-TIME-1 · **A19-TIME-2**（快线不等慢线）· A19-BOOT-4（key 集合逐字）· A19-VEL-1 · A19-CAL-1 ＋ **W-11 七场景消费对表全绿** | 我方 RNS（wanglei） | 七场景 RNS 判决与预期表逐行一致；🚫 不授导航权威 |
| ② 实机功能交付 | 原生快线（W-3）· 真实几何与 footprint · 时间/坐标绑定 · 标定（W-2）· 反例测试 | A19-PROF-1～5 · **A19-PROF-1b**（贵方反例）· A19-NEG-1 · A19-SEM-1 · A19-CFG-1 · A19-BOOT-1～7 · A19-SLOT-1 ＋ **PD-16 实机矩阵**（§Q5.3 提案） | 我方 RNS ＋ 整机侧 | 金标全绿 ＋ 实机矩阵「承诺」行全过 |
| ③ 生产联合验收 | 精度合格模型 · 指定整机共载与场景下的节拍 / 时效 / 稳定性 · 运动验收 | A19-PERF-1（共载实测）· §Q2.1 节拍门 · §Q2.2 时效门 · T-50～T-53 接收端对表（`perception_rx_audit.py`）· RNS 实机场景（`20` §13） | 项目负责人 | §Q3 工况下逐窗全过 |

★ **接口版本与兼容期**：`schema` 字段 `perception_*_v1`；字段增删 ⇒ `_v2`，**兼容期 ≥ 一个联调周期内两版并发**；旧名 `perception/detections|status` 走 `debug.legacy_keys_enable`（默认关）—— 与 09-08 reply §六一致，🚫 直接改名破坏现有消费者。

---

## Q2．发布节拍 ≤ 50 ms 的定义、数据延迟预算

### Q2.1 节拍要求（★★★ 正式契约文本，落 `11` §3.1B.3 v2.1，取代 v1.9 的「10 s 均值 ≥ 20」口径）

| 项 | 定义 |
|---|---|
| ★ 适用通道 | **仅 `objects`**。`profile` 按自身契约（随深度帧 30 fps 目标、丢旧保新、T-50/T-51 兜底，🚫 不设逐次上限）；`status` 1 Hz。三条 🚫 混合计数 |
| ★ 计数单元 | **唯一、完整的新推理结果**：身份 = 该推理帧的 `t_capture_mono_ms`（源帧身份去重）；重发、跟踪插值、心跳、跨通道合计一律不计；空结果合法**但必须是本轮推理的真实产物** |
| ★★★ 测点 | **报文字段 `t_publish_mono_ms`** = 正式发布 API（`put`）调用**前一刻**的 `CLOCK_MONOTONIC`；`put` 非阻塞（congestion=drop，Q1 档）。★ 选它而不选「返回时刻」的理由：**测点必须是报文里可见的量** —— 接收端拿同一字段就能独立复核，🚫 不依赖发布端日志。★ `put` 调用起止**另记日志**供审计（采纳贵方建议），其耗时受 `19` §2.2 「≤ 1 ms」预算约束，单独断言 |
| ★ 指标 | `Δpub[i] = t_publish[i] − t_publish[i−1]`（相邻两条唯一新结果）；**未结束间隙** `g_open(t) = t − t_publish[last]`，在每次 `status` 发布时刻与验收窗结束时刻各记一次 —— 没有下一帧也不漏记断供 |
| ★★★ 目标与验收门（分级） | **目标：每个 `Δpub ≤ 50 ms`**（贵方已接受的优化方向）。**验收门分两级，按 §Q3.2 工况逐窗判**：**一级硬门** —— 窗内 `max Δpub ≤ 100 ms` 且 `g_open ≤ 100 ms`（= 两个控制拍；任一违反 ⇒ 该窗不通过）；**二级比例门** —— `Δpub > 50 ms` 的次数占比 ≤ 1%（30 min 窗约 36 000 帧 ⇒ 最多 360 次落在 50～100 ms；⇔ `infer_gap_ms_p99 ≤ 50`）。均值、事后补发均不抵消。报告项：`max Δpub` · 超限次数与比例 · 间隔分布 · 未结束间隙 · 重复帧数。★ 为什么不是零容忍：36 000 帧无一超 50 ms 在非实时 Linux ＋ 15 进程共载的 Orin NX 上不可证，会成为永远红的验收项；RNS 真正在意的线是 100 ms（连续两拍复用同一快照），50 ms 作为尾部要求。贵方 09-10 数据（P99 59～68 · max 72～82）：一级门已达，二级门差约 15～25% 尾部量 |
| ★★ 「不能抖动」的定义 | **= 相邻间隔上界约束（贵方表中的形式 A）**：`Δpub ≤ 50 ms` 就是全部抖动限制，🚫 不另要求围绕固定周期 T 均匀输出、🚫 不约束相邻间隔的变化量。★ 理由：RNS 每拍重跑、与感知**无相位关系**（`20` §4.1），均匀性对 RNS 没有信息量；强加 `abs(Δpub − T) ≤ J` 会逼出发布端整形队列 —— **只增加年龄，不增加信息** |
| ★ 运行期健康 | 任一 `Δpub > 50 ms` ⇒ `degraded_reasons` 置 **`infer_gap`**（边沿事件 `warn`，同因 10 s 去重）；**`infer_rate_low` 判据改写**为「10 s 窗内 `>50 ms` 占比 > 1% **或** 出现 `>100 ms`」，置位即时、清除需连续 10 s 达标。★ 迟滞**只作用于标志位**，🚫 不豁免验收计数（与贵方一致）。★ **新增字段 `infer_gap_ms_max`**（10 s 窗最大间隔，含未结束间隙）= 一级门运行期读数；`infer_gap_ms_p99` = 二级门读数；`fps_infer` 保留为统计字段 |
| ★ 不计区间 | 上电预热 30 s 不计（沿用）。**重连 / 重配置 / 故障**：以 `status.degraded_reasons` 含 `reconnecting` / `reconfiguring` 的**声明区间**为准 —— 区间内间隔**不计入验收但必须记账并报告**；未声明的暂停一律计超限。★ 退出与恢复验收计窗的条件：声明区间清除后**首个唯一新结果**起重新计窗 |
| ★ 可证性边界 | 认同贵方：有限窗内未超限只证明该窗；契约 🚫 写「任何负载下永不超限」，只写 §Q3.2 工况下逐窗判定 |

### Q2.2 分通道延迟预算（按贵方三行表逐行答，150 ms / 216.667 ms 均不进契约）

| 对象 | 门 | 数值与起止点 | 可验证时机 |
|---|---|---|---|
| ★★ 几何快线 `profile` | **G-P1 处理时间**（= A19-PERF-1 的锚） | **SDK 深度帧回调返回 → `put` 返回，P99 ≤ 14 ms**（`19` §2.2 四段：解包 2 ＋ 遍历 8 ＋ 后处理/序列化 3 ＋ put 1）。共载条件按 A19-PERF-1 v1.2 口径 | 开发机现在可测；生产机第 ③ 阶段复测 |
| | **G-P2 发布时年龄**（**独立门**，与 G-P1 不混） | **`t_publish − t_capture` P99 ≤ 60 ms**（`19` §2.2 目标行，含曝光半周期与 GigE 传输） | ★ **只有 PD-11（`t_capture` 语义）闭合后才能签**；此前按 `t_capture_estimated` 口径报数、不签 |
| | 消费端（不变） | T-50 300 ms 限速档 · T-51 1 s 零速，以 `p1_motion` 收包时刻计龄 | ③ |
| ★★ 语义慢线 `objects` | 节拍 | §Q2.1 | ③ |
| | **RNS 允许的数据年龄**（★ 本行是贵方要的「消费用途 → 预算 → 依据」） | **目标：曝光中点 → `p1_motion` 收包 P99 ≤ 200 ms；硬门：T-52 500 ms**。★ 推导（无运动补偿、双方各按最坏）：`objects` 只决定「该让谁 / 停在多远」，🚫 不参与接触安全（那是 `profile` 的事，§3.1B.2A）；礼让规则 `stop_dist_m = 3.0 m` / `resume_dist_m = 4.0 m`（迟滞带 1 m）；机器人 2.0 m/s 与行人 1.5 m/s 相向 ⇒ 接近速度 3.5 m/s ⇒ 年龄 200 ms 的位置误差 0.7 m **＜ 1 m 迟滞带**（不振荡），实际停距 ≥ 2.3 m **＞ 制动需求 2.13 m**（`v²/2a` 取 a = 1.5 m/s² 的保守值 1.33 m ＋ `t_lat` 0.4 s × 2.0 m/s 反应 0.8 m）。★ 250 ms 时停距 2.125 m 与制动需求**相等**，无余量 ⇒ 取 200。★ `a` 未标定（#20-5），实测后按公式 `stop_dist_m ≥ v²/2a ＋ v·t_lat ＋ v_close·age_P99 ＋ margin` **共同重算** —— 年龄预算与 `stop_dist_m` 是一对可交换量，这就是「双方共同定值」的数学形式 | ③（贵方按 §Q3 工况测 P99，我方按上式定 `stop_dist_m`） |
| | 超预算消费动作 | ★ RNS 三档（落 `20` §3.1.8 v1.35，键 `perception.objects_age_ok_ms`）：`age ≤ 200 ms` 全功能；`200 ms < age ≤ 500 ms` **速度判据拒用**（运动状态视为未知 ⇒ 按动态处置：减速/停等，🚫 进入「静止可绕」分支）；`age > 500 ms`（T-52）全部 BLOCKED 按禁止类。★ 稳定发布旧数据在这里**自动失效**：年龄按 `t_capture` 算，重发不刷新（§Q2.3） | 我方 #20-22 接线，09-16 |
| ★ 当前结果年龄与断供 | `11` §1.6 / `infer_gap_ms_p99` | 见 §Q2.3 消费端接受规则 | — |

### Q2.3 消费端接受规则（落 `11` §3.1B.5 v2.1 —— 贵方问的「旧帧/重复帧是否刷新健康、超时从何时起算、未结束间隙」）

| 规则 | 内容 |
|---|---|
| ★★★ 接受条件 | 一条报文被 RNS 接受 ⇔ 其 `t_capture_mono_ms` **严格大于**同 key 上一条被接受报文的 `t_capture_mono_ms`。重复（相等）与乱序（更小）**一律丢弃、🚫 刷新年龄**，计入 RNS 审计 `perception_dup` / `perception_out_of_order` |
| ★★ 年龄起算 | `age = now_tick − t_capture(最近一条被接受)`，**每个控制拍重算**（同机 `CLOCK_MONOTONIC`，无钟差）⇒ **未结束间隙天然被计**：没有新帧时年龄逐拍增长，到 T-5x 即触发，🚫 需要「下一帧到达」才能判断 |
| ★ 断供 ≠ 每帧允许延迟 | T-5x 是**年龄**门，§Q2.1 是**间隔**门，两者独立：一条年龄 400 ms 的帧仍被接受（≤ T-52），但同时被 §Q2.1 记为间隔超限、被 §Q2.2 记为年龄超预算 |
| ★ 三个测点的命名（今后统一用） | **L1** 曝光 → 发布（`t_publish − t_capture`，报文内自带，发布端门 G-P2）· **L2** 曝光 → `p1_motion` 收包（`t_rx − t_capture`，RNS 审计，T-5x 与 §Q2.2 年龄预算的量）· **L3** 收包 → 决策输出（RNS 拍内 ≤ 50 ms ＋ 拍相位 ≤ 50 ms，RNS 审计 `tick_ms`）。★ 贵方现行「SDK 返回 → 本机订阅回调」≈ L1 去掉曝光与传输，**只作开发测量**，接受 |
| 150 ms / 216.667 ms | 确认：两者都是贵方内部阶段目标，**🚫 进契约、🚫 替代 T-50～T-53**；历史实验按原判定保留 |
| 职责 | 采纳贵方分工：我方给消费用途 · 年龄预算 · 运行包络（本节 ＋ §Q3.2）；贵方在约定工况下测出可承诺的 P99 与尾部；共同定值 |

---

## Q3．生产共载与场景验收条件

### Q3.1 生产机（可复现环境，实测 2026-09-11）

| 项 | 值 |
|---|---|
| 平台 | **NVIDIA Jetson Orin NX 16 GB**（开发套件 Super 形态；最终外壳与散热随实机整机，⚠️ 待定 —— 现为套件风扇主动散热，空载 `tj` ≈ 51 °C） |
| 系统 | Ubuntu 22.04.5 · L4T **36.4.3**（JetPack 6.2）· CUDA **12.6.11** · 功耗档 **`MAXN_SUPER`** |
| 计算 | 8× Cortex-A78AE @ 1.98 GHz · Ampere GPU（Orin NX 16 GB 规格）· **DLA ×2**（`nvdla0`/`nvdla1` 节点在）· NVENC/NVDEC 在 |
| 待装 | **TensorRT 10.3（JetPack 6.2 配套）与 ROS 2 Humble —— 我方 2026-09-18 前装机**；engine 在此机构建并按 PSC-4 金标验收，🚫 拿开发机 engine 交付（与贵方一致） |
| 中间件 | `zenohd 1.5.0`（两 router：`lo:7449` RT 面 · `7447` 通用面）· RNS 侧 `eclipse-zenoh` 1.9.0 |

### Q3.2 共载清单 · 场景 · 时长 · 记录（验收工况，正式）

| 项 | 内容 |
|---|---|
| ★★★ 必须并行的进程 | `10` §3.1 **全部 15 个常驻进程**：`zenohd-gen` · `zenohd-rt` · `p1_motion`（20 Hz 控制 ＋ RNS）· `p2_core` · `p3_task` · `p4_agent` · `p5_gateway` · **`perception`** · `chassis_relay` · `quadruped` · `rtk_driver` · `teleop_input` · `behavior_proxy` · Nav2 `behavior_server` · `zenoh-bridge-ros2dds`；**加三个 AI 服务**：`xbrain-llm.service`（llama-server，qwen2.5-3b gguf，**占 GPU 与约 2 GB 内存**）· `xbrain-ai-asr.service`（sherpa-onnx，CPU）· `xbrain-payload.service`；**加 MED-2 NVENC 推流**（§Q3.3） |
| 启动清单 / 等效负载 | 单元文件：`deploy/systemd/*.service`（22 个，含 `xbrain-config-freeze` / `xbrain-probe` 两个 oneshot）；分级启动序 `scripts/start_all.sh`（`10` §3.3.6）；开发直启 `scripts/dev/run_stack_dev.sh`。★ 底盘 / RTK 硬件不在时：`scripts/dev/chassis_stub.py` ＋ `estop_pong_stub.py` 顶替，**其余进程真跑** |
| 资源预算 | ★ 当前**无逐进程配额**（`10` 未定 cgroup/affinity）；验收**记录 `tegrastats` 曲线**（CPU 逐核 · GR3D · 内存 · 温度 · 功耗）作为事实，🚫 把空闲当长期保证（与贵方一致）。首版联合验收后若需配额，由 `10` 册主裁定 |
| ★ 场景与时长（提案） | 消防场区巡检标准环线（我方场地）：**(a)** 白天顺光 · **(b)** 黄昏/逆光 · **(c)** 夜间补光 —— 每窗 **连续 30 min**，机器人按巡逻档运动（≤ 2.0 m/s），场内 **0～5 名行人 · 0～2 台车辆 · 场区固定障碍（锥筒/箱体/立柱/路沿）**；另 **1 × 10 min 静态台架窗**（原地、密集目标 ≥ 8）作为下限对照。🚫 用椅子/行李箱短窗替代（与贵方一致） |
| 谁提供 · 谁记录 | 场地与整机：我方；发布端记录：贵方（`max Δpub` · 超限次数/比例 · 分布 · 未结束间隙 · 重复帧 · 实际检测数 / mask · ROI 量 · 跟踪量 · 资源曲线）；接收端记录：我方 `perception_rx_audit.py`（**按 `t_capture` 帧身份**逐帧记 `t_rx`、L2、去重/乱序、T-5x 触发）；**双方按同一帧身份对表**，正式消费端 = `p1_motion`，开发订阅者只作诊断（与贵方一致） |
| 主裁决 | §Q2.1 分级门（一级 `max ≤ 100 ms` · 二级 `>50 ms` 占比 ≤ 1%）＋ §Q2.2 年龄门逐窗判；均值只记录 |

### Q3.3 媒体：MED-2 是独立预览支路

| 项 | 内容 |
|---|---|
| MED-2 是什么 | `11` §8 MED-2 行：`perception` **NVENC 编码 ＋ 环回 RTSP `127.0.0.1:18083`**，经网关 DNAT 出机（COM-31/32/33）供 **HMI / 云端预览拉流**；参数 `19` §13.1：H.264 zerolatency、**1280×720 @ 15**、2000 kbps CBR、GOP 30（码率/分辨率 **PD-8 现场实测可调**） |
| 与 1280×800 主输出的关系 | ★★ **独立编码支路**，🚫 不是对分析管线帧输出的旧描述。分析/检测/mask 走 1280×800 主输出（09-11 决定）；RTSP 支路从同一彩色流**另行缩放编码**。★ XBRAIN 侧**没有** 1920×1080 的任何消费者 ⇒ 🚫 为旧表述额外生成 1080p 流 |
| 验收负载 | **两路同时运行**计入 §Q3.2 共载：主输出 1280×800@30 ＋ RTSP 720p@15。若 PD-8 实测后 RTSP 改 800p，同样两路 |

---

## Q4．改变消费者行为的出口语义（用真源闭合）

### Q4.1 `src` 逐位定义与样例（落 `11` §3.1B.1 v2.1；`19` §3.4 同步修正）

★ 总则：**`profile` 承载最近阻挡边界**（v1.9 已定）；`src` 四位各自回答一个可测的问题，**距离 · 来源 · 高度关联同一证据**：

| 位 | 名 | = 1 的充要条件 | = 0 的含义 |
|---|---|---|---|
| bit0 | T | `t_seg_mono_ms` 非 `null` **且** `[blind_near_m, d_free)` 全部格 `T_ok`（PROF-5 腐蚀后）**且** `d_free > blind_near_m` | 仅几何可通行（RNS 按 `20` §3.1.11 限速）或已验证区间为空 |
| bit1 | G | **几何通道本帧对该 bin 有有效样本**（地面证据或立体障碍点任一） | 该 bin 无任何有效深度 ⇒ 必有 `d_free = null` 且 `d_block = null` |
| bit2 | S | `d_block` **由语义注入产生或收紧**（`19` §3.4A：`d_sem < d_geom`），对应目标可在 `ObjectsMsg` 中按 footprint 恢复 | `d_block` 与语义无关 |
| bit3 | NEG | `d_block` 是**负障碍边界**（穿地判据或实测坑内点），此时 `h_block ≤ 0` 或 `null` | `d_block` 是正障碍或无障碍 |

| 推论（消费方可断言） | |
|---|---|
| `d_block = null` | ⇒ bit2 = bit3 = 0；bit1 = 1 表示「几何看到通行到 `d_free`、量程内无立体障碍」 |
| `d_block ≠ null` 且 bit2 = bit3 = 0 | ⇒ **边界来自几何正障碍**（隐含，🚫 另设位） |
| bit1 = 0 | ⇒ `src = 0`，`d_free = d_block = h_block = null`（整 bin 未观测，RNS-I-1 UNKNOWN） |
| `d_free = blind_near_m` | ⇒ bit0 = 0（已验证区间为空，`11` v1.9 下界语义） |

★ **样例**（`blind_near 0.59 · range_max 6.0 · dr 0.25`）：

| 场景 | `d_free` | `d_block` | `h_block` | `src` |
|---|---|---|---|---|
| 平地，有分割，量程内无障碍 | 6.0 | `null` | `null` | `0b0011` |
| 平地，**无分割**（慢线停） | 6.0 | `null` | `null` | `0b0010` |
| 3.0 m 处 0.4 m 高箱体 | 2.75（格远端判停） | 3.0 | 0.4 | `0b0011` |
| 玻璃门：深度成片 invalid，语义 footprint 在 2.5 m | 0.59 | 2.5 | `null` | `0b0110`（bit1 = 1：门框处有立体点） |
| 1.5 m 处坑沿（穿地 ＋ 坑内点 −0.3 m） | 1.25 | 1.5 | −0.3 | `0b1011` |
| 整 bin invalid（强反光） | `null` | `null` | `null` | `0b0000` |

★ **`19` §3.4 的两处同步修正**：BIT_G 在**逐像素遍历中**对「有有效样本」的 bin 置位（初版只在 `d_free` 推进分支置位，会把「贴脸障碍、地面被挡」的 bin 标成 0，与上表冲突）；BIT_SEM 只在 `d_sem < d_geom` 时置位（初版无条件 OR）。

★ **`slope_deg`（v1 定义，落 `11` §3.1B.1 注记）**：= FREE 段内相邻格 `zbar` 最大高差角 **相对本帧拟合平面**，是「局部坡变 / 台阶」量，**🚫 不是相对重力的绝对坡度**（拟合平面顺着坡面时它读 0）；无 FREE 段填 `null`。★ **RNS 本期不消费 `slope_deg` 与 `terrain`**（DTO 无该字段）—— 绝对坡度随 PD-13 与相机 IMU 重力向量二期；贵方 v1 填 `null` 也合法。

### Q4.2 局部能力不可用时的退化（原则：**只收缩该能力支持的 FREE，🚫 停发、🚫 无条件停掉全部输出**）

| 触发 | 生产侧动作（`19` v1.3） | 消费侧动作（RNS） |
|---|---|---|
| 地面拟合失败 → `ground_fit_fallback` | 退回外参先验平面 ＋ **`d_free` 封顶 `dfree_cap_fallback_m`**（`19` §8.2 新键，建议 2.0 m）＋ reason。★ 理由：平面误差 ∝ `r·tan(Δpitch)`，近场小远场大，封顶近场即保守 | **不加新逻辑**：`d_free ≤ 2.0` 落入速度门 `f(d_free)` 的 `[1.8, 3.0) → 0.5 m/s` 段（`11` §9.6.2），限速由既有机制自动产生。「写了 reason」不等于 FREE 可信 —— 这里 FREE 的可信由封顶保证，reason 只是可见性 |
| mask 投影不可确认（无彩色帧 / D2C 内外参缺 / `t_seg` 超龄） | 该帧 `t_seg = null`、全 bin bit0 = 0（`19` §3.3 既有） | `20` §3.1.11：仅几何 FREE ⇒ `no_seg_speed_cap` 0.5 m/s ＋ `warn` |
| 旧 footprint 无法变换到当前参考系（无 TF） | ★ 注入条件收严：`t_capture − t_obj ≤ objects_stale_ms` **且**按 PROF-5 同式 `m(r) = (v_max ＋ ω_max·r)·Δt ＋ m_jitter` **膨胀 footprint 后注入**（方向与腐蚀相反：BLOCKED 只许变大）。★ `objects_stale_ms` 由「与 T-52 同量级」**改为约两个推理周期（建议 100 ms ＋ 抖动）**—— 注入是帧级证据，下一条 `objects` 就会替换它，🚫 用消费端超时值 | 无（语义注入只让 `d_block` 变近） |

### Q4.3 估计时戳误差 · 时钟 / odom 纪元

| 项 | 规则 |
|---|---|
| `t_capture_estimated` 期间 | `latency_fallback_ms`（`19` §2.4 / PD-11）必须取**上界估计**（`t_capture` 估得偏早 ⇒ 年龄偏大 ⇒ 保守方向）；**估计误差界 E 由贵方实测记录**并随 PD-11 闭合前每包报数；RNS 不加修正 —— T-50 的 300 ms 对快线 ~35 ms 实际年龄留有 ~8 倍裕量，可吸收 E ≤ 100 ms。★ E 实测 > 100 ms ⇒ 回到本表重议，🚫 静默 |
| 时钟纪元 | 单机 `CLOCK_MONOTONIC`：进程重启**不换纪元**（同一内核钟），只有整机重启才重置，而那时 15 进程全部重启。★ RNS 防御（落 `11` §3.1B.5）：只接受严格递增的 `t_capture`（§Q2.3）；`t_capture` **倒退 > 1 s** 视为纪元重置 ⇒ 清空感知基线与记忆栅格（同 A-MEM-3 的保守理由：错位记忆是假墙/假通路）＋ `warn`；未来时戳（> now ＋ 50 ms）丢弃并审计 |
| odom 纪元 | 只影响 `pose_used` 与 `velocity_frame`（`ego_removed` 四条件之④）；**RNS 本期不消费 `pose_used`**，`velocity_frame == raw` 即拒用速度（`raw_velocity_policy: reject`）—— 纪元变化对 RNS 的可见形式就是 `raw` |
| 已接受、不重问 | 外参未标 / 无 TF / 媒体未配置的既有行为（与贵方一致） |

### Q4.4 分割「10 s 曾更新」与帧级新鲜度分开

| 层 | 规则 | 期望结果（A19-TIME-2 场景：慢线人工卡死 5 s） |
|---|---|---|
| ★ 帧级（生产侧，`19` §3.3 补键 `seg_max_age_ms`，建议 300 ms） | 快线取 mask 槽时判 `t_capture − t_seg ≤ seg_max_age_ms`：否 ⇒ 本帧 `t_seg = null`、bit0 全 0；是 ⇒ 按 PROF-5 腐蚀（年龄越大 FREE 越小，300 ms 内平滑过渡到 0） | 最后一张 mask 之后 **≤ 300 ms** 起，每帧 `t_seg = null` 且 bit0 = 0；快线发布率不降 |
| ★ 帧级（消费侧，`rns.yaml perception.seg_stale_ms` = 300 ms） | RNS 再判一次 `t_capture − t_seg ≤ seg_stale_ms`，否即按无 T（双保险，`20` §3.1.11 表三条触发之一） | 同上 |
| ★ 能力级（`status.traversable_seg_available`，10 s 窗） | **只是健康 / 事件标志**：卡死 10 s 后翻 `false` ＋ `no_traversable_seg` ＋ `warn` 事件；🚫 参与任何帧级 FREE 判定 | 第 10 s 翻 `false`；恢复后需连续 `seg_recover_frames` 帧有效才撤限速（RNS 侧，#20-21 待接） |

★ 贵方来函 §Q4 末段（bin 归属 / 盲区过滤 / 负障碍早停 / 零分母 / 旋转项 / 健康窗停更）由贵方**按契约自主修复并做反例测试** —— 同意，🚫 逐条报批；反例请按 A19-* 编号登记，收编时并入 `tests/perception/`。

---

## Q5．实机输入的负责人 · 产物 · 窗口

> ⚠️ **前提如实**：实机整机**尚未到位**，以下「时间」均以「实机到位日 D」为基准，D 由项目负责人另行通知。**接口与离线工作不受此表阻塞**（与贵方一致）。

| # | 项 | 负责人 | 可用产物 | 时间 / 窗口 | 未闭合影响 |
|---|---|---|---|---|---|
| 1a | 相机最终安装位置与支架 | 整机侧（项目负责人） | 安装图纸 ＋ 实测安装位 (x, y, z, pitch) | **D ＋ 3 天** | 阻塞 W-2 标定 |
| 1b | 标定窗口（W-2，平面法 ＋ 直边参照，`19` §7.1）与物理测量配合人 | 贵方执行 · 整机侧配合 | `extrinsic_base_cam` 块 ＋ 标定记录（日期 · 方法 · 残差）落 `docs/` | 安装后 **2 天内**，一个半天窗口 | 阻塞第 ② 阶段 |
| 1c | 正式 `base_link` 定义 | 我方（`13` 册主） | 现代用 = 机体几何中心地面投影（`19` §7.1 注记）；正式定义随 `quadruped` 开工定案 | 与 1a 同批书面化 | 不阻塞（代用定义可标定，改定义只平移 t_bc） |
| 1d | odom / TF 接口 | 我方（`quadruped`，**尚未实现**） | `13` §4.3 已定：`odom ← base_link`，**100 Hz 发布 / 信息更新率 10 Hz**（⚠️ 插值只能按 10 Hz 真实更新算）；单调钟时戳；缺失/过期 ⇒ 贵方按 `raw` / `null` | 实现日期待 `13` 开工排期，**不早于 D** | 阻塞 `ego_removed`；不阻塞其他 |
| 2a | 最大扫掠高度（PD-17，`z_pass_m` 输入） | 整机侧 | 机体 ＋ 载荷 ＋ 云台 ＋ 步态起伏 ＋ 姿态余量的**实测**上界（静态 ＋ 行走各测） | **D ＋ 1 天** | 阻塞 PROF-4 落值 |
| 2b | `wz_max` / 加速度（#20-5） | 整机侧 | 平地 2.0 m/s 阶跃减速曲线 90 分位 · 角速度上界 | **D ＋ 3 天** | 阻塞 §Q2.2 年龄预算复算与 PROF-5 `ω_max` |
| 2c | 2.0 m/s 适用工况的降速/停车**预算** | 我方（已定） | `11` §9.6.2 速度门四段 `f(d_free)`：`[3.0,∞)→2.0 · [1.8,3.0)→0.5 · [1.25,1.8)→0.2 · [0,1.25)→0`；`t_lat = 0.4 s`；`D_slow = 3.0 m` 提前降速（🚫 靠急停） | 已有 | 实测停车行为随 2b 一起验收（同意贵方：策略存在 ≠ 已实测） |
| 3 | 首版障碍能力矩阵（PD-16） | 我方提案 · 贵方验证传感器证据 · 整机侧定承诺 | **§Q5.3 提案表** | 本文 | 阻塞第 ② 阶段「承诺行」验收，不阻塞开发 |

★ 有效 FOV · 盲区 · 可用量程 · 深度误差 · 地面阈值 · 拟合残差：同意归贵方在上述安装条件下测量/整定，我方只按 `11` 消费（`range_max_m ≤ 6.0` 拒启 PSC-6 不变）。

### Q5.3 首版障碍能力矩阵（提案，消防场区硬化路面 · 供三方定承诺）

| 目标 | 尺寸 | 材质 | 距离 | 光照 | 承诺等级 |
|---|---|---|---|---|---|
| 直立杆 / 立柱 / 锥筒 | 直径 ≥ 10 cm，高 ≥ 0.5 m | 哑光金属 / 混凝土 / 塑料 | ≤ 4 m | 室外阴天 · 晴天顺光 | **承诺**（A19-PROF-3 的实机对应） |
| 箱体 / 设备 / 行人 / 车辆 | 最小边 ≥ 0.3 m | 非镜面 | ≤ 6 m | 同上 | **承诺** |
| 台阶 / 路沿 | 高差 ≥ 0.12 m | — | ≤ 3 m | 同上 | **承诺**（slope/step 停止条件） |
| 坑 / 装卸台边缘 | 深 ≥ 0.3 m × 宽 ≥ 0.5 m | — | ≤ 2 m（垂直 FOV 限制） | 同上 | **承诺**（A19-NEG-1 实机） |
| 逆光 / 夜间补光 | 上述目标 | — | — | 逆光 · 补光灯 | **待实测后定**（首版不承诺） |
| 细线 / 拉索 / 钢筋 | < 3 cm | 任意 | — | — | **🚫 不承诺**（PD-16 保证边界） |
| 玻璃 / 水面 / 镜面 | — | — | — | — | **🚫 不承诺检出**；只承诺保持 UNKNOWN ＋ `invalid_pixel_ratio` 如实上报 |
| 误检约束 | 空旷硬化路面 ≤ 6 m 内假 BLOCKED bin 占比 | — | — | 三档光照 | **阈值待整定**（首窗实测后落值，🚫 先写数） |

---

## 附录 A · 首版 `class_map` 草案（按 `class_schema_94.json` 逐类；落 `configs/rns.yaml` 与 `20` §5.1.1 v1.35）

★ 行为类闭集（`20` §5.1.1）：`person_stop` · `vehicle_dynamic`（动态距离规则：减速 / 停 / 恢复；静止驻留 2 s 后转静止堆可绕）· `block`（默认）· `traverse` · `hazard`；★ **`20` v1.35 增第六类 `ignore`**（不参与行为分流、不做语义注入；物理占用仍由 `profile` 几何管）—— 仅限**空中 / 非地面**目标，逐类显式列出，**默认仍是 `block`**。

| 行为类 | 类（id） | 说明 |
|---|---|---|
| `person_stop`（代码锁死） | person(0) | 恒停不绕；`proxy` 状态**同样生效**（安全方向） |
| `vehicle_dynamic` | car(2) · truck(7) · bus(5) · motorcycle(3) · bicycle(1) · heavy_machinery(89) | 车辆族 |
| `vehicle_dynamic`（同规则，名称沿用） | dog(16) · cat(15) · horse(17) · sheep(18) · cow(19) · bird 以外的动物 | 动物按动态距离规则；`proxy` ⇒ 🚫 进入静止可绕分支 |
| `hazard` | **pit(93)** | 负障碍语义类：硬禁止 ＋ 额外膨胀（`margin hazard 0.6`）。★ 明火 / 深水仍无对应类 ⇒ 该映射集允许为空（09-08 已裁） |
| `ignore`（v1.35 已立） | bird(14) · kite(33) · frisbee(29) · sports ball(32) · airplane(4) | 空中或滚动小物，地面 footprint 无意义；🚫 扩到任何地面物 |
| `traverse` | （空） | 词表无低草 / 落叶类 |
| `block`（默认，🚫 需逐条写） | 其余全部：fire hydrant · traffic light · stop sign · parking meter · bench · pole(84) · pillar(85) · barrier(91) · traffic_cone(90) · pallet(86) · carton(87)* · storage_rack(88)* · tree_trunk(80) · rock(81) · dirt_pile(82) · bush(83) · 家具 / 器物类 …… | `*` 87/88 为 `inactive_reserved_ids`，现不输出；出现即按未映射 ⇒ `block` ＋ 限速 |
| **🚫 禁入 objects** | **traversable_area(92)** | T 通道本体，🚫 作为目标发出（落 `11` §3.1B.2 v2.1）；RNS 侧收到即丢弃 ＋ 审计 `objects_t_class_dropped` |

★ 其他策略：`semantic_status == proxy` ⇒ 取「该类映射」与 `block` 中**更保守者**（person → 仍停；车辆/动物 → 动态规则但不可绕；其余 → block）；`confidence < perception.min_confidence`（首版 0.3 起谈）⇒ 该目标按未知类处置 = `block`（🚫 `ignore` / `traverse` / `hazard` 对它生效；`person` 不受影响，任何置信度都停）；未知 / 未来新增类 ⇒ `block` ＋ 限速（`20` §5.1.1 既有）。★ 本表**只是 RNS 行为映射，不是检测承诺**（09-08 §3.4 已裁）—— 贵方对不具备检测能力的类直接标注即可。

## 附录 B · 我方本轮登记的待办（`20` §15 v1.35）

| # | 事项 | 时间 |
|---|---|---|
| **#20-22** | `objects` 三档年龄（`objects_age_ok_ms` / T-52）与 `status` T-53 计龄接线 ＋ 去重/乱序/纪元规则（§Q2.3 · §Q4.3）＋ 审计计数 | 2026-09-16 |
| **#20-23** | `extrinsic_calibrated == false` ⇒ 拒绝自主任务，reason `extrinsic_uncalibrated`（`11` §3.1B.4）—— 现未接 | 2026-09-16 |
| **#20-24** | 三条 key 生产 Zenoh 订阅器（强引用，Q1/Q2 QoS）＋ **W-11**（`perception_sim.py` · 七场景样例 · 消费对表测试）＋ `perception_rx_audit.py` ＋ `configs/perception.yaml` 骨架与 resolved 样例 | 2026-09-16 |
| **#20-25** | `class_map` 扩展（附录 A）＋ `ignore` 类 ＋ T 类防御 ＋ `min_confidence` 键 | 随附录 A 定稿 |
| **#20-26** | 冻结线 `SNAPSHOT_PROCESSES` 尚未纳入 `rns.yaml` / `perception.yaml`（`12` §12.0A「同列解析」未落地）⇒ resolved 样例先以手工产物代替；交办冻结线 | 待排 |
| 文档 | `11` v2.1（§Q2.1 节拍 · §Q2.3 接受规则 · §Q4.1 src 逐位 · `traversable_area` 禁入 · `infer_gap` / `reconnecting` / `reconfiguring` reason）· `19` v1.3（§3.4 BIT_G/BIT_SEM · §3.2/§3.2A 1280×800 注记 · §3.2A `dfree_cap_fallback_m` · §3.3 `seg_max_age_ms` · §3.4A `objects_stale_ms` 与膨胀 · §5.1/§6.3 节拍口径与 A19-RATE-1 改写 · §2.2 G-P1/G-P2 命名 · §8.2 新键 · §13.1 MED-2 独立支路 · §17 PD-11 误差界 / PD-16 矩阵）· `20` v1.35（§3.1.8 年龄三档 · §5.1.1 `ignore` 与 proxy · §12.2 新键 · §15 #20-22～26）· `12` v0.9（§12.0A 两键 ＋ class_map 扩行） | 与本文同一提交，tag `perception-r3-20260911` |

## 附录 C · 离线包清单与真源提交号（`perception_pack_20260911.tar.gz`）

| 项 | 内容 |
|---|---|
| 真源提交 | 提交号 **`7f67c72`**（四册 · `rns.yaml` · 代码批 #20-22～#20-25 之后的真源状态；附录 C 回填为其后继提交，tag 指向回填提交），tag **`perception-r3-20260911`**；仓库 `https://github.com/leiwangaimake/xbrain_v6` |
| 离线包 | `perception_pack_20260911.tar.gz`：下表全部文件 ＋ `SHA256SUMS`（逐文件）＋ `COMMIT.txt`（提交号与 tag）。★ 每次真源变更重出一包，🚫 增量 |

| 文件 | SHA256（真源提交下） |
|---|---|
| `docs/11-接口契约.md`（§1.6 · §2.4 · §3.1B） | `834f9b287fa66f9ed89c17a7267daa5ade7d227c1c8afcf61f6770625165b8ef` |
| `docs/19-perception详细设计.md` | `4cdc52aa008519facb17e3b6c3aef81e32b238d583f493c9d21180ac95db717b` |
| `docs/20-RNS反应式导航软件系统详细设计.md`（§3.1 · §5.1.1 · §12.2 · §15） | `eea1aa49a12eb69b01f26806c2df8d21a84f21aab9c682012f705e6f7b0d293b` |
| `docs/12-P1运动域详细设计.md`（§12.0A `rns.yaml` 定义处） | `1a98cad57ae09820813ef4cc972796216eeb5d5ec6b145fc480e7bfb85d29b1e` |
| `docs/perception-rns-interface-20260907.md`（冻结） | `69116411375f18048a7c1c13e5d7668781d4d51321ec6e1b89b0003c9853ef94` |
| `docs/perception-rns-reply-20260908.md` | `cce0f724f89b5ca9f3cd2f1bc12609f03895bbe944858b27c925313e950c2d61` |
| `configs/rns.yaml`（消费侧配置，含 class_map） | `56d699e970a1ff4b8bdeb2143910cd1788bf90c1af556a123664de3736a57523` |
| `xbrain/p1_motion/rns/inputs.py` ＋ `types.py`（三条 DTO 与 `SrcBit` = 消费端 schema 的代码形态） | `272edf59546e7e72cc23e9744c9405ff9315009095f4e8e30b328398bd71a4ee` ＋ `1bf9ddc744dd8332190e8e3055022bf19c2139aa06af6e534fba25b2633c9bbd` |
| `common/include/xbrain/{config,clock,zenoh,envelope}/*.h` · `common/CMakeLists.txt` | 见包内 `SHA256SUMS` |
| `deploy/systemd/xbrain-perception.service` · `scripts/lint/charset_lint.py` | 见包内 `SHA256SUMS` |
| 本文 | 见包内 `SHA256SUMS`（本文 🚫 自引） |

---

**本文新增契约三处（§Q2.1 节拍 · §Q2.3 接受规则 · §Q4.1 src 逐位）与新键四个（`objects_age_ok_ms` · `dfree_cap_fallback_m` · `seg_max_age_ms` · `min_confidence`），全部随同一提交落四册（`11` / `19` / `20` / `12`），本文自身不是真源。** 有异议之处请继续按本轮格式对账。

上海哈船智能船舶技术有限公司 · XBRAIN_V6

**依据**：`11` v1.9 §1.6 / §2.4 / §3.1B · `19` v1.2 · `20` v1.34 §3.1 / §5 / §9.0 / §15 · 09-08 reply · 贵方 09-07《接口答复》§3 Q-1 · 贵方 09-11 来函（第 2 版）· 生产机实测（2026-09-11，`ssh xbrain`）
