# XBRAIN_V6 · perception 详细设计

| 项 | 内容 |
|---|---|
| 文档 | **perception 详细设计（第 19 册）** |
| 版本 | **v1.6**（v1.0 从 0 重写 · 落地版 → v1.1 三遍核查 → v1.2 感知方对账修正 → v1.3 第三轮对账 → v1.4 W-8 / W-11 落地 → v1.5 第四轮：拟合回退有效条件 · 无深度 bin 语义注入 → **v1.6 第五轮：回退能力收窄为依据门控**；封面与 §18 变更记录同步） |
| 日期 | **2026-09-12** |
| 状态 | ★★★ **可开工** —— 本版以「照此写代码」为验收标准 |
| 进程 | `perception`（**C++17**，单进程；rclcpp 仅用于 TF / 命令订阅，感知数据链不经 ROS topic） |
| 上游 | [11-接口契约](11-接口契约.md) **§3.1B（v2.1）—— 唯一接口真源** · [20-RNS反应式导航软件系统详细设计](20-RNS反应式导航软件系统详细设计.md) **v1.35 §3.1（消费侧语义）** · [perception-rns-reply-20260911](perception-rns-reply-20260911.md)（第三轮对账答复 · 本版差量的依据） · [perception-rns-interface-20260907](perception-rns-interface-20260907.md)（交接快照 · 差量 G-1~G-4 · 约束 F-1~F-3 · Q 清单） · `00` NAV/VOI 相关条目 · `CLAUDE.md` §3 |
| 下游 | 无（本册是**契约的实现侧**，🚫 不产生新契约） |
| 基底代码 | `ros2_ws/perception`（2026-09-07 交接快照，Orbbec Gemini 338Le 工程） |

---

## 0. 文档说明

### 0.1 ★★★ 本册是什么 · 不是什么

> ★★★ **全部对外接口面（key · schema · 字段 · 单位 · 不变量 · 超时号）的唯一真源是 `11` §3.1B。**
> 本册只回答：**这个进程内部怎么把 `11` 定死的字段算出来 · 在什么预算内算完 · 算不出来时怎么表达。**
> ★ 判据一句话：**本册删掉，`11` 仍然完整；`11` 删掉，本册一行都不成立。**

| # | 本册**不做**的事 | 理由 |
|---|---|---|
| **B-1** | 🚫 不新增 / 不改动任何 Zenoh key 与报文字段 | 定义处在 `11` §3.1B / §2.2.1；发现缺口 ⇒ 登记 §16 交回 `11` |
| **B-2** | 🚫 不新增 / 不改动任何 `E_*` 错误码 | 闭集在 `11` §13.4~§13.15 |
| **B-3** | 🚫 不改动任何超时号（**T-50 ~ T-53** 等） | 总表在 `11` §1.6，各章不再各自定义 |
| **B-4** | 🚫 不代 `12` / `14` / `20` 落地任何连带 | 登记 §16 交对方册主 |
| **B-5** | 🚫 不给「跳过安全断言」留任何开关 | `CLAUDE.md` §3.6 |

### 0.2 ★★★ 为什么从 0 重写（v0.1 墓碑）

★★★ **v0.1（2026-08-05）整册建立在「整机有 LiDAR ＋ 底盘前后雷达可用」上。两个前提 2026-09-07 均被用户裁定不成立**（无 LiDAR，几何由 RGBD 产生；M20S 前后雷达不用）。
★ v0.1 的内容按三区处置，🚫 不含糊：

| 区 | v0.1 的节 | 处置 |
|---|---|---|
| ★★★ **已死**（前提消失） | §3 走廊 `bands` · §4 `dir_free` · §5.1~5.3 `sectors` · §2.3 点云线程 · §1.4 CHS-B 名字映射 · §7.2「LiDAR 缺失 ⇒ 出勤中止」 | ★ 不迁移。其中「LiDAR 缺失即中止」按新前提**恒真**，留着就是每天误报 |
| ★★ **停车场**（契约未随 RGBD 收口，🚫 本册不代裁） | `state/targets`（`PerceptionFrame` 含 `bands`/`sectors`/`wpos`/`cls` 闭集/`ambient`）· `rt/perception/targets` · PER-11 RTK 订阅 | ★ 登记 §16 **PCC-9**，本期不实现（见 §12） |
| ✅ **存活**（与 LiDAR 无关，原裁定沿用） | MED-2 推流参数 · TRT engine 纪律 · 单调钟 · deep BIT `gpu` 委托 · `U-15`/码率两条未定项 | ★ 收进 §13 / §17，逐条注明「v0.1 裁定沿用」 |

### 0.3 ★★★ 本期范围（与交接文档头表同源）

| 做 | 不做 |
|---|---|
| ① `11` §3.1B 三条 key（`profile` / `objects` / `status`）② 推理 **20 Hz**（#20-11）③ 落地顺序四步（标定 → 延迟 → ProfileMsg → 坐标正名） | 🚫 跟随 / re-ID（`20` #20-13 预留）· 🚫 停车场区（PCC-9）· 🚫 任何 LiDAR 相关 |

### 0.4 ★ 上游依据清单（逐字锚点，NUM-4 · 🚫 不写行号）

| 册 · 节 | 可 grep 的逐字锚点 | 本册用它做什么 |
|---|---|---|
| `11` §3.1B.0 | 「时序契约（v1.7 修订 —— 取代初版「三条同帧」）」 | §2 快慢线拆分的规范依据 |
| `11` §3.1B.1 | 「缺值一律 `null`，🚫 不用哨兵」·「PROF-1」~「PROF-5」 | §3 管线的五条生产侧不变量 |
| `11` §3.1B.2 | 「逐字段收严（v1.7」 | §4 字段映射（`velocity_status` 五值 · `r_near` · 退化凸包） |
| `11` §3.1B.3 | 「`fps_infer` 验收线」·「地面相关 ROI」 | §5 / §6 |
| `11` §1.6.1 | 「T-50」~「T-53」 | 消费侧超时（本册只需保证发布节律，🚫 不实现消费） |
| `20` §3.1.11 | 「T 通道不可用 / 超龄时的降级」 | §3.3：无分割时**继续发布**、bit0 不置 |
| `20` §15 #20-11 | 「推理链路必须优化到 ≥ 20 Hz」 | §6 |
| `20` §15 #20-13 | 「本期不实现，仅预留位置」 | §0.3 范围 |
| 交接文档 §四 | 「一遍循环，含可通行 mask 投地」 | §3 算法基线（本册细化为可编码规格） |
| `CLAUDE.md` §3.1 / §3.3 | 「不写默认兜底」·「必须红过一次」 | §8 / §14 |

---

## 1. 进程形态

### 1.1 ★★★ 单进程 · 五执行体 · 两条 Zenoh 会话

```text
  perception (C++17, 单进程)
  ├─ ① 采集线程        Orbbec SDK 回调: 深度帧 + 彩色帧 --> 各自 latest-wins 槽
  ├─ ② 快线 (几何)     每个深度帧: ProfileMsg 生产管线 (S3) --> RT put
  ├─ ③ 慢线 (推理)     检测 + 分割 + 跟踪 (基底既有) --> ObjectsMsg --> RT put
  │                     ├─ 分割 mask + t_seg --> latest-wins 槽 (快线消费, S3.3)
  │                     └─ 语义 footprint + t_obj --> latest-wins 槽 (快线消费, S3.4A)
  ├─ ④ status 线程     1 Hz StatusMsg --> RT put; 事件边沿 --> GEN put
  └─ ⑤ ROS executor    TF 监听 (odom<-base_link, quadruped 到位后) + capture_cmd 订阅
  会话: RT session --> tcp/127.0.0.1:7449 (rt/perception/*)
        GEN session --> tcp/127.0.0.1:7447 (event/{sev}/perception; 停车场区不发)
```

| 铁律 | 内容 |
|---|---|
| ★★★ **P19-1** | ★★★ **快线 🚫 不等慢线**（`11` TIME-2）。快线对慢线的唯一依赖是**读一个 latest-wins 槽**（最新 mask ＋ `t_seg`），槽空就按无 T 证据发（§3.3） |
| ★★ **P19-2** | ★★ 线程间交接**全部走 latest-wins 双缓冲**（写侧换指针 ＋ 序号，读侧读序号一致性重试）；🚫 无锁竞争热路径、🚫 无队列堆积（旧帧就地覆盖 —— 对导航，**新鲜 > 完整**） |
| ★★ **P19-3** | ★★ 快线稳态 🚫 **零动态分配**：bin 数组 · 地面网格 · JSON 缓冲全部预分配复用 |
| ★ **P19-4** | ★ rclcpp 只做 TF 与命令；感知数据 🚫 不经 ROS topic（基底已如此，维持） |
| ★ **P19-5** | ★ 两条会话 endpoint 来自解析产物配置（§8），🚫 不硬编码、🚫 不绑 `0.0.0.0`（NET-C9） |

### 1.2 ★★ 基底代码与收编

★ 基底 = `ros2_ws/perception` 快照（他方工程师交付，**尚未纳入版本控制**）。收编步骤（工作项见 §15 W-1）：
纳入版控 → 清 8 处全角标点（`scripts/lint/charset_lint.py` 的 `THIRD_PARTY_SNAPSHOTS` 排除项**同批删除**）→ 配置迁 `configs/`（§8.1）→ 头注补五字段（`CLAUDE.md` §2.5）。
★★ 收编前 🚫 不在快照上叠新功能 —— 否则差量与他方原始交付无法区分。

---

## 2. 时序架构（`11` TIME-1 ~ TIME-3 的实现）

### 2.1 ★ 为什么拆快慢线

★ 规范与理由的定义处是 `11` §3.1B.0（吞吐 ≠ 延迟 · `d_block` 不得等推理），🚫 本节不复述。本节只给**实现侧预算**。

### 2.2 ★★★ 快线预算（两道门 · v1.3 命名：**G-P1** 处理时间 · **G-P2** 发布时年龄 ≤ 60 ms）

| 段 | 预算 | 依据 |
|---|---|---|
| 曝光中点 → 帧完成 | ~17 ms | 30 fps 半周期（全局快门，无 rolling 校正） |
| GigE 传输 640×400×16bit | ~4 ms | ≈ 123 Mbps 瞬时 / 千兆链路 |
| SDK 解包 → 槽 | ≤ 2 ms | 零拷贝或单次 memcpy |
| ★ 逐像素遍历（§3.2） | ≤ 8 ms | 256 k px × ~30 flops，单核 NEON（§3.6） |
| 逐 bin 后处理 ＋ 序列化 | ≤ 3 ms | 181×24 cell ＋ ~10 KB JSON |
| RT put | ≤ 1 ms | 环回，非阻塞（§3.5） |
| **合计** | **≤ 35 ms** | ★ 对 T-50（300 ms）留 ~8 倍裕量 |

★★ **预算是 CI 断言不是愿望**（A19-PERF-1，§14）。★ **v1.3 把两道门分开命名**（感知方 09-11 Q2 追问「处理 P99 硬门的准确数值与起止点」）：**G-P1 处理时间** = SDK 深度帧回调返回 → `put` 返回，**P99 ≤ 14 ms**（解包 2 ＋ 遍历 8 ＋ 后处理/序列化 3 ＋ put 1；A19-PERF-1 的锚；开发机现在可测、生产机共载复测）；**G-P2 发布时年龄** = `t_publish − t_capture`，**P99 ≤ 60 ms**（独立门，含曝光半周期与传输；⚠️ 只有 PD-11 闭合后才能签，此前按 `t_capture_estimated` 报数不签）。🚫 两者相加或混用。
★ 1280×800 升档：算力仍够（~1 Gflops/s），但受 **F-1 延迟 ＋ 带宽 492 Mbps** 制约 ⇒ 升档条件登记 §17 PD-12，🚫 本期不升。

### 2.3 ★ 慢线预算

★ 20 Hz 硬指标的达标设计整节见 §6；慢线产物两个：`ObjectsMsg`（发布）与分割 mask 槽（快线消费）。

### 2.4 ★★ `t_capture_mono_ms` 的取得

| 规则 | 内容 |
|---|---|
| ★★ 首选 | Orbbec SDK 的帧时戳映射到 `CLOCK_MONOTONIC`；曝光中点 = 帧时戳 − 曝光时长/2（全局快门，单值） |
| ★ 兜底 | ⚠️ SDK 时戳语义（设备钟 or 主机钟 · 是否含传输）**待实测**（§17 **PD-11**）。实测前用 **host 收包时刻 − 固定链路估计（§2.2 前三段合计）**，并把该估计写进配置（可审计），🚫 不装作精确 |
| ★★★ 禁 | 🚫 `CLOCK_REALTIME` / 墙钟参与任何时戳（§10）；🚫 用发布时刻冒充采集时刻 |

---

## 3. ★★★ `ProfileMsg` 生产管线（本册核心）

> ★ 报文与不变量的定义处：`11` §3.1B.1（PROF-1 ~ PROF-5 · 缺值一律 `null`）。
> ★ 本节把交接文档 §四的算法基线细化到**可编码规格**：数据结构 · 更新时机 · 边界处置。

### 3.1 ★★ 预计算（外参变更时重算一次，🚫 不逐帧）

| 表 | 定义 | 用途 |
|---|---|---|
| `ray[v][u]` (float3) | `R_bc * K_inv * [u, v, 1]^T`（`base_link` 系，未归一） | 反投影：`p_b = z * ray + t_bc` |
| ~~`z_exp` 静态表~~ → **逐帧闭式**（v1.2 改，见 §3.2A） | `z_exp(v,u) = −(n·t_bc ＋ d) / (n·ray[v][u])`，`(n, d)` 为**本帧拟合地面平面** —— 🚫 不再假设地面恒为 `base_link` z=0（感知方指正：升降/俯仰/侧倾/坡面都会破坏该假设）。每像素 1 点积 ＋ 1 除，~8 flops，进 §2.2 预算 | 负障碍穿地判据 |
| `roi[v][u]` 的基准 | ★ ROI 掩膜仍按**外参先验平面**预计算（它只是统计分母，不参与安全判定，允许静态） | `invalid_pixel_ratio` 分母 |
| `r_exp` / `bin_exp`（懒求值） | 穿地候选像素（少数）上按本帧平面求 `p_exp = z_exp·ray ＋ t_bc` ⇒ `r_exp = hypot(p_exp.x, p_exp.y)`、`bin_exp = bin(atan2(p_exp.y, p_exp.x))`。★★★ **穿地证据必须记在【期望交点】的 bin（`bin_exp`），🚫 不是实际远处回波的 bin** —— 相机相对机体有横向平移时两者不同（感知方给出反例：`t=(0,0.3,1)` 时期望点 16.7°、回波点 8.5°，差一个扇区） | 负障碍归属 |
| `roi[v][u]` (bit) | `z_exp` 有效 且 交点地面距离 ≤ `range_max_m` | `invalid_pixel_ratio` 分母（`11` §3.1B.3） |
| `bin_of[v][u]` (int16) | 由 `ray` 方位角预算出 bin 序号（扇区外 = −1，★ 仅内部用，🚫 上线） | 免逐帧 `atan2` |
| `expect_px[i]` (int) | ROI 内落入 bin i 的像素配额 | `conf[i]` 分母 |

★ 内存：五表合计 ≈ 640×400 × 18 B ≈ **4.6 MB**，常驻。★ 外参未标定（占位）时表**照建**（几何仍自洽），只是 `extrinsic_calibrated = false` 随帧声明（`11` §3.1B.4 由消费方拒绝自主导航）。

### 3.2A ★★ 每帧前置：取原生深度 ＋ 拟合地面平面（v1.2 新增）

| 步 | 内容 |
|---|---|
| ★★★ 取流 | 快线消费 **D2C 之前的原生 640×400 深度帧**（PROF-3 v1.9 收窄定义）。⚠️ 交接快照在 HW-D2C 之后取 1280×720 对齐帧再放大 1080p —— 那两级都不是原生样本，改造点见 §15 W-3。★★ **2026-09-11 负责人裁定（用户书面确认）：图像 / 视频主输出维持 1280×800，🚫 再归一化 / 升采样到 1920×1080**（模型输入尺寸、原生深度 profile、帧率与精度门不因此改变；本册受影响的只有 mask 像面注记与 §13.1 MED-2 支路澄清） |
| ★★ 地面拟合 | 以外参先验平面为种子，对近区候选地面点做稳健拟合（RANSAC / IRLS）得本帧 `(n, d)`；★ 拟合失败（内点不足 / 残差超限）⇒ **退回先验平面 ＋ `degraded_reasons: ground_fit_fallback`**（`11` §3.1B.3 已登记该值），🚫 静默。★★ **v1.3 退化动作**：回退帧的 `d_free` **封顶 `dfree_cap_fallback_m`**（§8.2 新键，建议 2.0 m）—— 平面误差 ∝ `r·tan(Δpitch)`，近场小远场大，封顶近场即保守；消费侧不加新逻辑，`d_free ≤ 2.0` 落入速度门 `[1.8, 3.0) → 0.5 m/s` 段自动限速（`11` §9.6.2）。「写了 reason」不等于 FREE 可信 —— 可信由封顶保证，reason 只是可见性（A19-FIT-1）。★★★ **v1.5 补全（感知方 Q4：封顶只限制误差随距离增长，先验平面自身误差界必须可测）—— 先验平面继续支持 FREE 的有效条件，两条同时成立**：① 本帧失败原因是**内点不足**（`inlier_frac < plane_fit_min_inlier_frac`：地面被遮挡 / 成片 invalid），🚫 是**残差超限**（`resid > plane_fit_resid_max_m`：地面非平面 / 坡变，先验必错）；② IMU 重力向量相对**标定时姿态**的偏差 `‖Δtilt‖ ≤ fallback_tilt_tol_deg`（来源 PD-18 定；无 IMU ⇒ 视为不成立）。配对断言（启动）：`dfree_cap_fallback_m × tan(fallback_tilt_tol_deg) ≤ h_tol_m`（封顶处平面高度误差 🚫 超地面容差；例 2.0 m × tan 2° ≈ 7 cm ≤ 8 cm）。★★ **条件未知或不满足 ⇒ 只撤回依赖平面的 FREE，保留独立阻挡，照常发布**：全 bin `d_free = null`（本帧无法验证，🚫 写 `blind_near`），bit0 = 0；立体点高度阈提到 `h_tol_eff(r) = h_tol_m ＋ r·tan(fallback_tilt_err_max_deg)`（条件②不可测时的上界，建议 5°），低于它的点不判障；负障碍 🚫 判（穿地判据依赖平面）；`degraded_reasons` 同时含 `ground_fit_fallback` ＋ `ground_free_withdrawn`（`11` v2.2）。★ 例（拟合失败，1.5 m 处 0.12 m 路沿，同一 bin）：条件满足 ⇒ `d_free 1.25 · d_block 1.5 · h_block 0.12 · src 0b0011`；条件不满足 ⇒ `h_tol_eff(1.5) = 0.08 ＋ 1.5·tan 5° ≈ 0.21 > 0.12` 路沿不可判 ⇒ `null · null · null · 0b0010`（有样本、不可判）；RNS：前向 UNKNOWN ⇒ `20` §8.1A 压速，路沿由记忆栅格或减速后的新帧接管。失效方向：**宁可看不见路沿也不给假 FREE**（A19-FIT-2）。★★★ **v1.6 收窄（感知方 Q4 补充确认函，`perception-rns-reply-20260912-r5.md`）—— 回退分支改为依据门控**：先验平面误差界 `e(r) = |Δh| + r·tan(Δtilt) + e_align`，其中 `Δh = state/chassis_motion.height_m − calib.body_height_m`（`11` §9.8.2 字段已在线；perception 订阅须进 `11` §1.1.6 白名单，PD-19 待裁）、`Δtilt` 按 PD-18、`e_align` 为安装/对齐残余上界（§8.2 `fallback_align_err_m`）。**FREE 回退放行 ⇔ 三项齐全 ∧ 各自新鲜 ∧ `e(dfree_cap_fallback_m) ≤ h_tol_m`**；**几何阻挡保留 ⇔ 依据齐全 ∧ `h_est > h_tol_m + e(r)`**（`h_est` 是算法估计，🚫 用测试真值）；★★★ **任一项缺失 / 过期 / 键为 `null` ⇒ 整支收窄**：全 bin `d_free = null`、bit0 = 0，负障碍与 `z_exp` 穿地判据关闭，依赖该平面的几何阻挡**不承诺**（无其他有效证据时 `d_block / h_block = null`，🚫 解释为无障碍），独立有效的 S 阻挡照证据保留（其依据若受同一失效影响则同样不保留），`src` 按实际证据填（bit1 = 有样本），照常发布并同时带 `ground_fit_fallback` ＋ `ground_free_withdrawn`。v1.5 的 `h_tol_eff(r) = h_tol + r·tan(5°)` **作废**（5° 不是已建立的上界）；`e(r)` 进的是分类阈值，🚫 只做「误差 < 容差」旁路检查。消费侧后果见 `20` §4.1 v1.42 注（UNKNOWN 压速探行，记忆 / objects 兜底；p1 宿主门 🚫 把新鲜全 null 当失效） |
| ★ 高度定义 | 逐点高度 `h = n·p ＋ d`（对本帧平面），后续分类全部用它；`slope` 停止条件负责平面之外的坡变 |

### 3.2 ★★★ 逐像素遍历（原生全样本单遍，PROF-3 / PROF-4）

```text
for (v, u) 全分辨率:                       # 640x400, 不降采样 (PROF-3)
  z = depth(u, v)
  if invalid(z):
      if roi[v][u]: n_invalid += 1         # 只计地面相关 ROI
      continue                             # PROF-2: 不产生任何几何证据
  if roi[v][u] and z > z_exp(v, u) * (1 + eps_neg):     # z_exp 按本帧平面闭式求 (S3.2A)
      pierce[bin_exp(v,u)][cell_of(r_exp(v,u))] = true  # 记在期望交点的 bin (S3.1 懒求值)
  p = z * ray[v][u] + t_bc                 # (x, y, h)
  r = hypot(p.x, p.y)
  i = bin_of[v][u]
  if i < 0 or r < blind_lo or r > range_max_m: continue
  src[i] |= BIT_G                          # 11 v2.1 逐位定义: 几何本帧对该 bin 有有效样本 (S3.4 不再在推进分支置位)
  if p.h > z_pass_m:            continue   # PROF-4: 过顶, 可从下方通过
  elif p.h > h_tol_m:                      # 立体障碍
      d_blk[i] = min(d_blk[i], r)          # 角度域 MIN, 1 个有效像素即存活
      if r <= d_blk[i] + w_h_m: h_max[i] = max(h_max[i], p.h)
  elif p.h >= -h_tol_m:                    # 地面证据
      k = cell_of(r)
      G[i][k] += 1; Zs[i][k] += p.h; Zq[i][k] += p.h * p.h
      if seg_slot.valid and mask_lookup(u, v): T[i][k] += 1
  else:                                    # 实测低于地面
      neg_r[i] = min(neg_r[i], r); neg_h[i] = min(neg_h[i], p.h)
```

| 边界 | 处置 |
|---|---|
| ★ `h_max` 的滞后 | `d_blk[i]` 在遍历中还会变小 ⇒ 高度在**逐 bin 后处理重算**（`obs` 紧凑表二次窗过滤），且**负障碍优先于正障碍高度**（§3.4 v1.2），🚫 不信遍历中的首值 |
| ★ `blind_lo` | 逐帧取 `max(blind_near 配置下限, 本帧实测最近有效地面距离)`；发布字段 `blind_near_m` 用**本帧实测值**（时变，`20` RNS-I-5） |
| ★★ `mask_lookup(u, v)` | ★★ **v1.2 改为投影查表**：快线在原生深度系，mask 在彩色像面（交接快照实测二值 mask 1920×1080；★ **2026-09-11 起主输出 1280×800，mask 随之在 1280×800 像面**，投影查表只换该像面内参）⇒ 用出厂 depth↔color 内外参把该像素 3D 点投到彩色面取 mask 值（~15 flops，无遮挡 z-buffer，v1 接受 —— 误查风险被 `FREE = T ∧ G` 的 G 侧兜住并在此声明）。🚫 纯宽高比例缩放只在同一像面成立（感知方指正）。mask 取**原始可通行类分割**（`11` v1.9 T 通道取材行），槽带 `t_seg` |

### 3.3 ★★ T 证据与 PROF-5 腐蚀（在地面域做，🚫 不在像素域）

```text
m  = v_ego_max * (t_capture - t_seg) + m_jitter          # 收缩余量, 米
T_ok[i][k] = 对 [i][k] 及其邻域(径向 m/dr, 角向 m/(r_k*angle_step)) 全部成立:
             G > 0 且 T/G >= tau_T                        # min 池化实现
```

★★ **帧级新鲜度（v1.3 —— 感知方 Q4 追问「10 s 曾更新」与「本帧可用」分开）**：快线取 mask 槽时先判 `t_capture − t_seg ≤ seg_max_age_ms`（§8.2 新键，建议 300 ms，与 RNS 侧 `seg_stale_ms` 同值）—— 否 ⇒ 本帧 `t_seg = null`、bit0 全 0；是 ⇒ 按上式腐蚀（年龄越大 FREE 越小，300 ms 内平滑过渡到 0）。`status.traversable_seg_available`（10 s 窗）**只是能力 / 事件标志，🚫 参与帧级判定**。期望结果（A19-TIME-2 场景，慢线卡死 5 s）：最后一张 mask 之后 ≤ 300 ms 起每帧 `t_seg = null` 且 bit0 = 0，快线发布率不降；第 10 s `traversable_seg_available` 翻 `false` ＋ `no_traversable_seg`（A19-SEG-2）。

★★★ **无分割（槽空 / `traversable_seg_available == false`）时**：`T_ok` 整体视为「无 T 证据」——
**照常发布**，`t_seg_mono_ms = null`、全部 bin `src` bit0 = 0，`d_free` 的 T 停止条件跳过（`11` §3.1B.1 v1.7 逐字）。
🚫 **不停发、不置零、不装作有**。降级动作在 RNS 侧（`20` §3.1.11），🚫 不在本进程。

### 3.4 ★★★ 逐 bin 后处理（`d_free` 六停止 ＋ 负障碍）

```text
for i in bins:
  d_free = blind_lo(i)
  for k in cells:                          # 格覆盖 [near_k, far_k), r_k = 中心
    if far_k > d_blk[i]:             break # 障碍落在本格内或更近 => 本格不许支持 FREE,
                                           # d_free 停在 near_k (= 上一格 far)
                                           # 感知方反例已证 "r_k < d_blk 即放行" 会推出
                                           # d_free = r_k + dr/2 > d_blk, PROF-1 破 -- 按格远端判
    if pierce[i][k] or neg_r[i] <= far_k:  # 负障碍确认 (穿地判据 或 实测坑内点)
        d_blk[i] = min(d_blk[i], near_k); src[i] |= BIT_NEG; neg_flag[i] = true
        break
    if G[i][k] / expect_cell[i][k] < cover_min:   break
        # 整格覆盖率判据 (v1.2, 感知方指正): 仅点数 g_min 不能排除
        # "证据集中在格局部, 其余是 invalid/遮挡" -- 分母是该格 ROI 内期望样本数(预计算)
    zbar = Zs/G; var = Zq/G - zbar^2
    if seg_slot.valid and not T_ok[i][k]:      break   # T 停止 (仅有分割时生效, S3.3)
    if k > 0 and |zbar - zbar_prev|/dr > tan(slope_max): break  # 台阶/陡坡
    if var > sigma_max^2:                      break   # 粗糙度
    zbar_prev = zbar; d_free = far_k           # 整格通过才推进到格远端
    src[i] |= (T_ok ? BIT_T : 0)         # BIT_G 已在 S3.2 遍历中按「有有效样本」置位 (11 v2.1)
  # h_block 的选择优先级 (v1.2, 感知方指正: 初版会被正障碍最大高度无条件覆盖):
  h_out[i] = neg_flag[i] ? (实测到坑内点 ? neg_h[i] : null)          # 负障碍优先
                         : max(h for (r,h) in obs[i] if r <= d_blk[i] + w_h)
  conf[i]  = clamp255(255 * valid_px[i] / expect_px[i])   # expect_px == 0 => conf = 0
  slope[i] = atan(free 段内最大相邻 |zbar 差| / dr)      # 无 free 段 => null
  terrain[i] = 0                                        # 首版恒 unknown; 分档是二期 (S17 PD-13)
```

| 不变量 → 实现落点 | 说明 |
|---|---|
| **PROF-1** | ★★ v1.2 按感知方反例修正：以**格远端** `far_k > d_blk` 判停 ⇒ `d_free ≤ near_k < d_blk` 恒成立（原「格中心判停」可推出 `d_free > d_blk` 的越界 FREE）。`d_blk` 无障碍时内部用 +inf，**上线转 `null`**（§3.5） |
| **PROF-2** | invalid 像素在 §3.2 第一分支就被排除，任何数组不落哨兵 |
| **PROF-3** | 输入就是全分辨率流；🚫 管线内任何一处出现下采样即断言红（A19-PROF-3） |
| **PROF-4** | `z_pass_m` 随帧发布（值出自配置，§8） |
| **PROF-5** | §3.3 的 min 池化；腐蚀只会让 `T_ok` 变少 ⇒ 只会让 FREE 变小 |

### 3.4A ★★ 语义证据注入（`src` bit2 的产生规则 —— v1.1 补，初版漏）

> ★ `11` §3.1B.1 的 `src` bit2 = 语义目标；`20` §3.1.1 载体表明写 S 通道载体 = 「`ObjectsMsg` ＋ `src` 位」。
> ⚠️ 初版管线只产 bit0/bit1/bit3，bit2 恒 0 —— 语义看见、几何看不见的场景（**玻璃门被检出而深度成片 invalid**）在 profile 里完全不可见。

```text
慢线每个推理帧: 把目标 footprint(base_link) + class + t_obj 写入 latest-wins 槽
快线逐 bin 后处理末尾:
  if now - t_obj > objects_stale_ms: 跳过 (与 T-52 同量级, 过期语义不注入)
  for 每个目标:
    fp = footprint 按 m(r) = (v_ego_max + omega_max*r)*(t_capture - t_obj) + m_jitter 膨胀
                                             # v1.3: 无同步位姿时的保守变换 (PROF-5 同式, 方向相反: BLOCKED 只许变大)
    对 fp 覆盖到的 bin i: d_sem = 该 bin 方向到 fp 的最近距离
    d_geom = d_blk[i] if 本 bin 有几何阻挡 else +inf   # v1.5: G=0 (无深度) 的 bin 同样注入, 缺席按 +inf (11 v2.2)
    if d_sem < d_geom: d_blk[i] = d_sem; src[i] |= BIT_SEM
                       if d_free[i] is not null: 重算 d_free 截断 (维持 PROF-1)   # 无 FREE 可截时保持 null
                       # bit1 保持原值: G=0 的 bin 注入后为 null / d_sem / null / 0b0100 (A19-SEM-2)
                                             # v1.3: BIT_SEM 只在语义边界更近时置位 (11 v2.1 逐位定义)
```

| 规则 | 内容 |
|---|---|
| ★ 方向保守性 | 注入只会让 `d_block` 变**近**（min 并集），符合 `RNS-N-5` BLOCKED 取并；🚫 不做反向（语义没看见 🚫 不能抹几何） |
| ★ 陈旧上限 | `objects_stale_ms`（§8.2；★ **v1.3 改为约两个推理周期，建议 100 ms ＋ 抖动，🚫 与 T-52 同量级** —— 注入是帧级证据，下一条 `objects` 就替换它，消费端超时值不是它的尺度）：BLOCKED 方向留旧值是保守的，但无限留会把已离开的目标钉死 ⇒ 超龄不注入，靠几何与 RNS 侧 `ObjectsMsg` 自己的 T-52 定价 |
| ★ 🚫 不是融合 | 只做证据注入并保留归属位，「让行/绕行/穿越」的策略判断仍全在 RNS（`20` §3.1.4） |

### 3.5 ★ 编码与发布

- ★★★ 缺值一律 `null`（`11` §3.1B.1 编码约定）：`d_blk = +inf → null`，`h_out` 不可测 → `null`，`slope` 无数据 → `null`。🚫 序列器出现 `inf` / `nan` 字面量（A19-ENC-1）。
- ★ JSON 组包进预分配缓冲；`put` 非阻塞 congestion=drop —— **宁丢旧帧不堵采集**（与 `11` §2.4.6 对 lidar points 的裁决同理）。
- ★ 每帧一条，随深度帧节律 30 fps；🚫 攒批。
- ★ `conf[i]`：`expect_px[i] == 0`（扇区边缘 ROI 外）⇒ `conf = 0`，🚫 除零。

**报文头字段的产生（逐个，防「样例里有、没人产」）**

| 字段 | 产生 |
|---|---|
| `schema` | 常量 `perception_profile_v1`；字段增删 ⇒ 版本号进位（定义处在 `11`，本进程只跟随） |
| `t_capture_mono_ms` | §2.4 |
| `t_publish_mono_ms` | `put` 前一刻取 `CLOCK_MONOTONIC` |
| `frame` | 常量 `base_link`（§7.1 origin 代用注记） |
| `extrinsic_calibrated` | §7.3 判定，三条报文同源同值 |
| `pose_used` | TF 可得 ⇒ `lookupOdomPose(t_capture)` 的位姿（基底已有该函数）；不可得 ⇒ `null` |
| `t_seg_mono_ms` / `z_pass_m` / `blind_near_m` | §3.3 槽 / §8.2 配置 / §3.2 本帧实测 |

### 3.6 ★ 复杂度与内存

640×400×30 fps ≈ 7.7 M px/s × ~30 flops ≈ **0.23 Gflops/s**（单核 NEON 裕量大，与 DLA 推理零竞争）；
帧态数组 `G/Zs/Zq/T/pierce` ≈ 181×24×(4×4＋1) B ≈ **74 KB**；预计算表 4.6 MB（§3.1）。

---

## 4. `ObjectsMsg` 生产

### 4.1 ★★ 字段映射表（基底字段 → 契约字段，逐行）

| `11` §3.1B.2 字段 | 基底来源（2026-09-07 实测） | 差量 |
|---|---|---|
| `track_id` / `stable_frames` / `first_seen_mono_ms` / `last_seen_mono_ms` | `track_id` · `stable_frames` · `first_seen_ms` / `last_seen_ms` | ★ 时基统一到 `CLOCK_MONOTONIC` 毫秒（改名不改语义） |
| `class_name` / `class_id` / `confidence` | 同名字段 | 无（开放词表，`11` v1.7） |
| `semantic_status` | `semantic_status: confirmed/proxy` | 无 |
| `footprint_xy` | `ground_footprint_odom.polygon_xy` | ★★ 改名去 `_odom`（G-4 正名）＋ **顶点数 ≤ 12**：超限按「保留极点 ＋ 面积贡献最大优先」抽稀；共线 / N<3 按最小外接矩形 ＋ ε 膨胀（`11` v1.7 退化处置） |
| `z_min` / `z_max` | 由 `size_3d` ＋ 质心换算 或 逐点统计 | ★ 必须是**区间实测**，🚫 质心 ± 半高冒充（横杆问题） |
| `r_near` | 无（现只有中心距 `depth_m`） | ★★ **新增**：原点到凸包逐边最小距离，点在多边形内 = 0 |
| `velocity_xy` ＋ `velocity_frame` | `velocity_3d` ＋ **未序列化的** `last_velocity_frame_has_odom_` | ★★★ 序列化该标志（G-4 第一步，一行级）；odom 到位后 ego 消除见 §4.2 |
| `velocity_valid` / `velocity_status` | 同名（五值闭集，`11` v1.7 采纳） | 无 |
| `depth_quality` | `depth_confidence` ＋ `depth_stat` 归档为 good/fair/poor | ★ 三档映射阈值进配置 |

### 4.2 ★ ego 消除（odom 到位后启用，之前恒 `raw`）

```text
T_wb(t): OdomBuffer 取 t 前后两帧插值 (平移 lerp, 旋转 slerp), t = 各帧曝光中点
q_i = T_wb(t_i) * p_i;  v_w = (q2 - q1)/(t2 - t1);  v_b = R_wb(t2)^T * v_w
velocity_xy = (v_b.x, v_b.y);  velocity_frame = "ego_removed"
```
★ 现有窗口化 ＋ 两窗同向确认抗抖直接套在 `q` 序列上，逻辑不动。
★★★ **`ego_removed` 的置位条件（v1.2 收严 —— 感知方指正「不是一行 has_odom 就能补齐」）**，四条同时成立才许标：
① TF 命中为**精确时刻**查询（latest 回退 ⇒ 不算）且 `|pose 时刻 − t_capture| ≤ 阈值`；② 已完成**旋回当前 `base_link`**（现实现停在 odom 轴上）；③ 外参非占位（`extrinsic_calibrated == true`）；④ 两帧位姿来自同一 odom 纪元。
★★ 任一不满足 ⇒ `velocity_frame = "raw"` 如实上线 —— 🚫 不因「显得没做完」而伪装。TF 不可得（`quadruped` 未实现）时天然落 raw。

### 4.3 ★ TIME-1（报文内单帧）

一条 `ObjectsMsg` 的全部目标出自**同一推理帧**；`t_capture_mono_ms` = 该帧曝光中点。🚫 混入上一帧未更新的目标快照（跟踪器内插值属于算法内部，出口以本帧关联结果为准）。

---

## 5. `StatusMsg` 生产（1 Hz）与事件

### 5.1 逐字段算法

| 字段 | 算法 |
|---|---|
| `fps_depth` / `fps_infer` | 各自 10 s 滑窗帧计数 / 10（★ v1.3：`fps_infer` 退为**统计字段**，验收口径改为逐次间隔分级门，见下行） |
| ★★ `infer_gap_ms_p99` / `infer_gap_ms_max`<br>**（v1.3）** | `objects` 相邻**唯一新结果**的 `t_publish_mono_ms` 间隔（含未结束间隙 `now − t_publish[last]`）入 10 s 环形缓冲：P99（**二级门读数 ≤ 50 ms**）与最大值（**一级门读数 ≤ 100 ms**）。★ 唯一性按推理帧 `t_capture` 去重，重发 / 插值 / 心跳不计（`11` §3.1B.3 v2.1） |
| `latency_ms_p50` / `p99` | 每帧样本 `t_publish − t_capture` 入 10 s 环形缓冲，nearest-rank 分位；★ 分别对快线（profile）与慢线（objects）各算一组，上线取**慢线组**（保守，且与 F-1 实测口径一致） |
| `invalid_pixel_ratio` | `n_invalid / roi 像素总数`（§3.1 / §3.2）。★ 外参未标定 ⇒ ROI 退化为下半幅 ＋ `degraded_reasons` 记 `roi_fallback`（`11` §3.1B.3） |
| `ground_seg_level` | 沿用基底定义 |
| `extrinsic_calibrated` | §7.3 的判定结果（🚫 不可手置） |
| `traversable_seg_available` | 分割槽 10 s 内有有效更新 |
| `degraded_reasons` | ★ 行为相关的固定值由 `11` 定：`infer_rate_low`（§6.3，v1.3 判据改写）· `no_traversable_seg` · `roi_fallback` · `ground_fit_fallback`（§3.2A）· `t_capture_estimated`（§2.4）· ★ v1.3 增 `infer_gap`（任一 `Δpub > 50 ms`，边沿 warn，10 s 去重）· `reconnecting` / `reconfiguring`（节拍要求的声明不计区间 —— **不声明就暂停 = 计超限**）；其余为自由诊断串。★★ 🚫 用自由串顶替固定值 —— 测试按字面 grep |

### 5.2 ★ 事件（跨面白名单 PC-2）

`event/{severity}/perception`，**边沿触发**：`degraded_reasons` 集合发生增删的那一拍发一条（`warn`），恢复发 `info`；★ 同因 10 s 去重。🚫 周期性重发（那是 `status` 的活）。

---

## 6. ★★★ 推理节拍达标设计（`20` #20-11 · v1.3 改逐次间隔口径）

### 6.1 现状分解（基底 README 实测）

输出 ≈ 18.73 Hz · E2E P99 166.5~225.9 ms。双模型：COCO80 640×640 → DLA0；Active12 896×896 **FP16** → DLA1；GPU 基本空闲。★ **v1.3 事实（2026-09-11 实测）**：生产机是 **Jetson Orin NX 16 GB**（JetPack 6.2 · DLA ×2 · 同机 llama-server 占 GPU、sherpa-onnx 占 CPU），🚫 按 AGX Orin 或开发机 `.23` 的成绩外推；engine 在生产机构建（PSC-4）。感知方 09-10 开发机成绩：均值 25 Hz，**发布间隔 P99 59～68 ms · 最大 72～82 ms** —— 距 §6.3 二级门约 15～25% 尾部量，一级门已达。

### 6.2 ★ 三条路线（按性价比排序，具体取舍归实现方 —— §17 PD-14）

| # | 路线 | 说明 |
|---|---|---|
| 1 | ★★ **Active12 FP16 → INT8** | 896×896 是最大单笔开销；DLA INT8 吞吐通常显著高于 FP16，需 TAO/TRT 校准集（复用金标语料场景） |
| 2 | ★★ **双 DLA 流水重叠** | 确认两引擎 enqueue 异步并行而非串行等待；主机侧前后处理（NMS · mask 解码 · JSON 组包）与 DLA 段重叠 |
| 3 | ★ **GPU 分担一模型** | GPU 空闲是现成第二通道；代价是与 NVENC（MED-2）错峰需实测 |

★★★ 无论哪条：**吞吐达标不解延迟**（`11` §3.1B.0「吞吐 ≠ 延迟」）——E2E 压缩另行按 F-1 追（交接文档 Q-1）。

### 6.3 ★ 验收接线（长在报文里，🚫 不靠口头）

★ **v1.3 口径（`11` §3.1B.3 v2.1 · 2026-09-11 负责人裁定，分级门由用户同日选定）**：目标每个 `Δpub ≤ 50 ms`；验收**一级硬门** `max Δpub ≤ 100 ms`（含未结束间隙）、**二级比例门** `>50 ms` 占比 ≤ 1%。运行期：任一 `Δpub > 50 ms` ⇒ `infer_gap`（边沿 warn）；`infer_rate_low` := 10 s 窗内 `>50 ms` 占比 > 1% **或** 出现 `>100 ms`（即时置位，连续 10 s 达标才清）。`fps_infer` 10 s 均值只作统计。断言 A19-RATE-1（§14，v1.3 改写为间隙注入三件套）。

---

## 7. ★★★ 整机外参标定（落地顺序第 1 步，其余全部以它为前提）

### 7.1 标定量与方法

| 项 | 内容 |
|---|---|
| 标定量 | `T_base←cam`（6DoF）。`base_link` 原点按 `13` 底盘定义；`quadruped` 未实现期间以**机体几何中心投影**代用并注明 |
| ★ 方法（平面法 ＋ 直边参照） | ① 机器人置于**实测水平**的地面，采深度帧拟合地面平面 ⇒ **roll · pitch · 相机高 H**；② 摆放与机体纵轴平行的直边参照物（墙根 / 标定杆），由其在地面投影的方向 ⇒ **yaw**；③ x/y 平移用卷尺实测机械安装位，进不了拟合就直接量 —— 🚫 不为省事标 0 |
| 产物 | `configs/perception.yaml` 的 `extrinsic_base_cam:` 块（§8.2）＋ 标定记录（日期 · 方法 · 残差）落 `docs/` |
| ★ 残差判据 | 平面拟合内点 RMS ≤ 阈值（配置，`null` 待首标定给出）；不达标 ⇒ 标定不生效 |

### 7.2 依赖链

外参 ⇒ §3.1 五张预计算表 ⇒ 地面判定 · 负障碍 · ROI · footprint 全部。⇒ **标定前一切几何输出照发但 `extrinsic_calibrated = false`**，由消费方拒绝自主导航（`11` §3.1B.4）。

### 7.3 ★★★ `extrinsic_calibrated` 的判定（🚫 不可手置）

```text
true  <=>  extrinsic_base_cam 全部键非 null
        且 calib_record 存在且其 sha256 与配置内登记值一致
        且 残差字段 <= 阈值
```
★ 三条件任一不满足 ⇒ `false`。🚫 配置里不存在「强制置 true」的键（B-5 / `CLAUDE.md` §3.6）。

---

## 8. 配置

### 8.1 ★★ 落点与读取

| 规则 | 内容 |
|---|---|
| 源 | `/opt/xbrain_v6/configs/perception.yaml`（L6 per-process；`common.*` 顶层键 🚫 出现） |
| ★★★ 运行期 | 读**解析产物** `/run/xbrain/resolved/perception.yaml`（`10` §5.4.1），🚫 读源。C++ 侧用 `common/` 配置加载器（零 ROS 依赖） |
| ★ 迁移 | 基底自带 `config/perception.yaml` **退役**为部署产物模板；快照内 `lidar:` 块（`/livox/lidar`，已核实无订阅点）删除（交接文档 Q-6） |
| ★★★ null 纪律 | 未标定键一律 `null`；启动读到 `null` ⇒ **拒绝启动并报出键路径**（PSC-2）。🚫 dataclass/结构体默认值、🚫 `or` 兜底、🚫 0.0 冒充 |

### 8.2 键表（全 null ＝ 待标定；有值 ＝ 已裁定并注明出处）

```yaml
perception:
  profile:
    angle_min_rad:    null   # 待 Q-2 (实测 FOV 覆盖)
    angle_step_rad:   null   # 待 Q-2
    n_bins:           null   # 待 Q-2
    range_max_m:      null   # 待标定; 上限 6.0 (338Le 推荐量程, 11 3.1B.1); 户外强光实测后落值 (PD-12)
    dr_m:             null   # 径向格宽, 建议 0.25 起谈
    blind_near_lo_m:  null   # 盲区下限 (发布值逐帧实测, S3.2)
    h_tol_m:          null   # 地面判定容差
    z_pass_m:         null   # 过顶滤除 (PROF-4); 待 Q-7 (载荷最高点 + 余量)
    cover_min:        null   # 整格覆盖率下限 G/expect_cell (v1.2 取代单纯点数 g_min)
    omega_max_rps:    null   # PROF-5 腐蚀的角速度上界 (11 v1.9 加旋转项)
    seg_max_age_ms:   null   # v1.3 帧级 mask 新鲜度上限 (S3.3); 建议与 RNS seg_stale_ms 同值 300
    dfree_cap_fallback_m: null # v1.3 地面拟合回退帧的 d_free 封顶 (S3.2A); 建议 2.0
    plane_fit_min_inlier_frac: null # v1.5 拟合失败分类: 内点比例低于此 = 内点不足 (S3.2A 条件 1)
    plane_fit_resid_max_m:  null # v1.5 拟合失败分类: 残差高于此 = 残差超限, 先验不可用
    fallback_tilt_tol_deg:  null # v1.5 先验平面可用的 IMU 姿态偏差容差 (S3.2A 条件 2); 断言 cap*tan(tol) <= h_tol
    fallback_tilt_err_max_deg: null # v1.5 条件不可测时的姿态误差上界; v1.6 起不再用于放行 (5 度非已建立上界), 仅留作调试可见性
    fallback_align_err_m:   null # v1.6 e(r) 的安装/对齐残余上界 (S3.2A 依据门控第三项); 安装实测填, null = 回退整支收窄
    tau_T:            null   # T 证据比例阈值
    slope_max_deg:    null
    sigma_max_m:      null   # 粗糙度 (格内高度标准差)
    eps_neg:          null   # 穿地判据超差比例
    w_h_m:            null   # h_block 径向窗
    v_ego_max_mps:    null   # PROF-5 腐蚀用 (取整机 v_max)
    m_jitter_m:       null   # PROF-5 固定余量
  objects:
    depth_quality_fair: null # good/fair 分界 (depth_confidence)
    depth_quality_poor: null
  status:
    window_s:         10.0   # 本册裁定: fps 与分位数共用窗口, 与 #20-11 验收口径同源
  capture:
    latency_fallback_ms: null  # PD-11 兜底的固定链路估计 (S2.4)
  extrinsic_base_cam:
    translation:      [null, null, null]
    rotation_xyzw:    [null, null, null, null]
    calib_record:     null   # 标定记录路径
    calib_sha256:     null
    residual_max_m:   null
  med2:                      # v0.1 裁定沿用 (S13); 独立预览支路, 主输出 1280x800 (2026-09-11 裁定) 不经此块
    rgbd_rtsp_port:   18083
    rgbd_pub_port:    null   # 甲方 U-15 (PD-2); null => 不推流 + warn
    width: 1280
    height: 720
    fps: 15
    bitrate_kbps: 2000
    gop: 30
  objects_inject:
    objects_stale_ms: null   # S3.4A 语义注入的陈旧上限 (v1.3: 约两个推理周期, 建议 100 + 抖动; 不是 T-52 量级)
  debug:
    pointcloud_enable: false # 调试点云 (11 2.2.1 登记为 debug 默认关; PSC-5 按登记集比对)
    legacy_keys_enable: false # 过渡期旧名 perception/detections|status (感知方 2.6 提议采纳):
                             #   仅 dev 联调可开, 生产恒关; 新三 key 独立验收后旧名共同退场
  zenoh:
    rt_endpoint:      "tcp/127.0.0.1:7449"
    gen_endpoint:     "tcp/127.0.0.1:7447"
```

★ 慢线自身的模型 / 跟踪 / 速度抗抖参数沿用基底既有键（`velocity.window_s: 0.30` 等**实测整定值原样保留**），随收编迁入本文件同名段，🚫 本册重新发明。

---

## 9. 降级与失效表

> ★ 总则：**能发就发、如实标注，降级动作归消费方**。本进程的失效表达 = 报文字段 ＋ `degraded_reasons` ＋ 事件，🚫 私自停发（停发对 RNS 是 T-50/T-51 的超时路径，语义更糟）。

| 失效 | 本进程行为 | 消费方看到什么 |
|---|---|---|
| 相机断连 / 无深度帧 | 重连退避；期间 `profile`/`objects` 自然停更 | RNS 走 T-50 → T-51（限速 → 零速） |
| 深度成片 invalid（强光 / 雨雾 / 玻璃） | 照发；invalid 不产证据（PROF-2） | `invalid_pixel_ratio` 高 ⇒ RNS 强制限速 |
| 分割不可用 / 超龄 | 照发，bit0 全 0 · `t_seg = null`（§3.3） | `20` §3.1.11 限速通行 |
| 推理慢于 20 Hz | `objects` 照发（能多快发多快） | `infer_rate_low` ＋ warn（§6.3） |
| TF / odom 缺失 | 速度走相机系分支 | `velocity_frame = "raw"` ⇒ RNS 拒用作运动判据 |
| 外参未标定 | 全链照算照发 | `extrinsic_calibrated = false` ⇒ RNS 拒绝自主导航 |
| RT 会话断 | 重连退避 ＋ GEN 面 `fault` 事件（若 GEN 仍在） | 超时路径 |
| 快线超预算 | 丢**旧**帧保新帧（P19-2），事件计数 | `latency` 上升可见 |

---

## 10. ★★★ 单调钟（v0.1 裁定沿用）

一切时戳 · 超时 · 年龄 = `CLOCK_MONOTONIC`（`std::chrono::steady_clock`）。🚫 `system_clock` / `CLOCK_REALTIME` / 裸 `rclcpp::Clock()`（默认墙钟）。墙钟只用于日志展示与录包对齐。CI 静态扫描本进程（`scripts/lint/clock_scan.py` 扫描面含 `ros2_ws/perception`，收编时生效）。

---

## 11. 启动自检（PSC，任一不过 ⇒ 拒绝启动并报出原因）

| # | 检查 | 失效方向 |
|---|---|---|
| **PSC-1** | 解析产物存在且非源文件（路径前缀 `/run/xbrain/resolved/`） | 读源 ⇒ 拒 |
| **PSC-2** | §8.2 全部键非 `null`（`med2.rgbd_pub_port` 与 `extrinsic_base_cam` 除外 —— 前者缺省不推流，后者缺省 `calibrated=false`，两者**不阻塞启动**） | 缺键 / null ⇒ 拒并报键路径 |
| **PSC-3** | 相机连接且 SN 与配置一致；深度流 640×400 Y16 @30 起流成功 | 拒 |
| **PSC-4** | TRT engine 加载 ＋ **真跑一帧金标**比对（§13.2） | 拒 |
| **PSC-5** | 两条 Zenoh 会话建立；声明的 key 集合与 `11` §2.2.1 登记逐字一致，🚫 通配 | 拒 |
| **PSC-6** | `range_max_m ≤ 6.0`（338Le 推荐量程上限，`11` §3.1B.1） | 拒（填大了在危险方向出错） |
| **PSC-7** | 预计算表自检：`z_exp` 有效像素数 > 0 且 ROI 非空 | 拒（外参写反的最快暴露点） |

---

## 12. 跨面纪律

| 面 | key | 状态 |
|---|---|---|
| RT（7449） | `rt/perception/profile` · `objects` · `status` | ★★★ 本期实现（`11` §2.2.1 已登记） |
| RT | `rt/perception/pointcloud` | ★ **调试专用 · 默认关闭**（`11` §2.2.1 登记为 debug 行；G-2 裁定不为导航加密）。开启仅限 dev 配置 |
| RT | `rt/perception/targets` · `rt/lidar/*` | 🚫 **不实现**（`20` #20-10 历史条目） |
| GEN（7447） | `event/{severity}/perception`（PC-2） | 本期实现（§5.2） |
| GEN | `state/targets`（PC-1，`PerceptionFrame` 族） | ⚠️ **停车场**（§16 PCC-9），本期不实现 |

★ 禁任何通配发布 / 订阅（`11` §1.1.6 规则 b）。★ 本进程**零订阅**契约 key（TF 与 `capture_cmd` 走 ROS，不占 Zenoh 白名单）；PER-11 的 `rt/gnss` 订阅随 `wpos` 进停车场。

---

## 13. 保留面（v0.1 裁定沿用，逐条注明）

### 13.1 MED-2 · NVENC 推流

v0.1 裁定**原样沿用**：环回 RTSP **18083**（`127.0.0.1` only，NET-C9）；`rgbd_pub_port` 属甲方 `U-15`（PD-2），未配置 ⇒ 不推流 ＋ 一条 `warn`（`med2_not_configured`），其余功能照常；H.264 NVENC `zerolatency` B 帧 0，1280×720@15，2000 kbps CBR，GOP 30（码率/分辨率待现场实测，PD-8）。★ 与 §6 路线 3（GPU 分担模型）有资源交叠，选该路线时 NVENC 占用须一并实测。★★ **v1.3 澄清（感知方 Q3 追问）**：MED-2 是**独立的预览 / 上行编码支路**（HMI / 云端拉流），🚫 不是对分析管线帧输出的旧描述；分析 · 检测 · mask 走 **1280×800 主输出**（2026-09-11 负责人裁定，🚫 再生成 1080p 规范化流 —— XBRAIN 侧无任何 1080p 消费者）；联合验收时**两路同时运行**计入共载。若 PD-8 实测后 RTSP 改 800p，仍是两路。

### 13.2 TRT engine 纪律

★★★ 必须**整机构建**（🚫 跨机拷 engine），engine 与 `build_env` 摘要同落盘；启动真跑一帧金标比对（PSC-4）；`dla.enabled == false` 时 BIT `dla` 项报 `ok + detail:"not_used"`，🚫 `unknown`。deep BIT `gpu` 项委托通道未定 —— 维持 v0.1 裁决**不增 PC-3 订阅**，deep BIT 复用 PSC-4 的金标结果缓存。

---

## 14. ★★★ 测试与断言总表（每条配一个必然让它变红的变异体）

> ★ 金标向量 = **合成深度场景**（测试内程序化生成 organized 深度阵 ＋ 期望 profile JSON）：
> 平地 · 玻璃洞（成片 invalid）· 4 m 处单像素细杆 · 1.8 m 高横杆 · 台阶 · 坑（穿地）· 陡坡 · 粗糙带 · 旧 mask 平移边。
> ★ 场景生成器是测试资产（`tests/perception/golden/`），🚫 依赖实机。
> ⚠️★★ **合成金标的保证边界（v1.2）**：它验证的是**算法不丢/不错分已有样本**；「细杆/拉索是否产生有效回波」是传感器物理，🚫 合成场景无法代答 —— 实机细障碍验收（尺寸/材质/距离矩阵）登记 §17 **PD-16**。

| # | 断言 | ★ 变异体（注入什么 ⇒ 必须红） | 类型 |
|---|---|---|---|
| **A19-PROF-1** | 全场景 `d_free[i] < d_block[i]`（同为 null 除外） | ★ 后处理去掉 `r_k >= d_blk` 早停 ⇒ 属性测试红 | 属性 |
| **A19-PROF-2** | 玻璃洞场景：洞区 bin `d_free` 停在洞前 · `d_block = null` | ★★★ invalid 填 `range_max` ⇒ 洞区被判可通行 ⇒ 红 | 金标 |
| **A19-PROF-3** | 细杆场景：`d_block ≈ 4.0` | ★★★ 遍历前加 2×2 均值下采样 ⇒ 杆被平均掉 ⇒ 红 | 金标 |
| **A19-PROF-4** | 横杆（1.8 m，高于 `z_pass`）：不入 `d_block`；★ **反向**：横杆降到 0.5 m ⇒ 必入 | ★ 去掉过顶滤除 ⇒ 正向红；★ 全高度都滤 ⇒ 反向红 —— **成对，缺一个则空壳全绿** | 金标 ×2 |
| **A19-PROF-5** | 旧 mask 平移边场景：腐蚀后边界 FREE 收缩 ≥ `m` | ★ 去掉 min 池化 ⇒ 未收缩 ⇒ 红 | 金标 |
| **A19-NEG-1** | 坑场景：`d_block` 停坑沿 · `src` 含 BIT_NEG | ★ 关掉穿地判据 ⇒ 坑被判 UNKNOWN 缺口而 `d_free` 停得更远 ⇒ 红 | 金标 |
| **A19-TIME-1** | 一条 profile 全数组同帧（生成器给每帧异色标记，混帧可检出） | ★ bin 后处理读上一帧 `G` ⇒ 红 | 单元 |
| **A19-TIME-2** | 慢线人工卡死 5 s ⇒ 快线发布率不降 · bit0 转 0（守 **P19-1**） | ★★★ 快线等 mask 槽更新才发 ⇒ 发布率跌 ⇒ 红 —— **这是 TIME-2 的直接护栏** | 注入 |
| **A19-ENC-1** | 序列化输出经严格 JSON 解析零失败 · 全文无 `inf`/`nan` 字面量 | ★ `d_block` 直接写 +inf ⇒ 解析失败 ⇒ 红 | 单元 |
| **A19-RATE-1**<br>**（v1.3 改写）** | mock 推理注入单次 120 ms 间隙 ⇒ `infer_gap` 当拍出现、`infer_rate_low` 10 s 内置位、`infer_gap_ms_max ≥ 120`；随后连续 10 s 间隔 ≤ 50 ⇒ 两者清除。★ 反向：重发同一 `t_capture` 结果填补间隙 ⇒ **不得**缩短 `infer_gap_ms_max`（唯一性去重） | ★ 恒不置 ⇒ 正向红；★ 恒置 ⇒ 恢复段红；★ 去掉去重 ⇒ 重发把 max 压到 50 以下 ⇒ 反向红 —— 三件套 | 注入 ×3 |
| **A19-SRC-1**<br>**（v1.3 新增）** | `11` §3.1B.1 v2.1 六个 `src` 样例场景逐位相等；bit1 在「贴脸障碍、地面被挡」bin 亦为 1 | ★ 把 BIT_G 挪回推进分支 ⇒ 贴脸场景 bit1 = 0 ⇒ 红；★ BIT_SEM 无条件 OR ⇒ 语义更远场景 bit2 = 1 ⇒ 红 | 金标 ×2 |
| **A19-SEG-2**<br>**（v1.3 新增）** | mask 槽年龄 > `seg_max_age_ms` ⇒ 本帧 `t_seg = null`、bit0 全 0；≤ 上限 ⇒ 按 PROF-5 腐蚀 | ★ 去掉年龄判 ⇒ 5 s 旧 mask 仍产生 bit0 ⇒ 红 | 注入 |
| **A19-FIT-1**<br>**（v1.3 新增）** | 地面拟合回退帧：`d_free ≤ dfree_cap_fallback_m` 且 reason 含 `ground_fit_fallback` | ★ 去掉封顶 ⇒ 回退帧 `d_free` 到量程 ⇒ 红；★ 只封顶不报 reason ⇒ 反向红 | 注入 ×2 |
| **A19-FIT-2**<br>**（v1.6 改）** | 拟合回退 ＋ **依据齐全**（`Δh`、`Δtilt`、`e_align` 有效且 `e(cap) ≤ h_tol`）＋ 观测条件（箱体在 `dfree_cap` 内、在 FOV 内、有效像素 ≥ 阈）＋ 过顶条件（`h_est < z_pass_m`）⇒ 1.5 m 处 0.6 m 箱体 `d_block = 1.5`、`h_block ≈ 0.6`（真高只作真值比对）、`src` bit1 = 1；`d_free ≤ dfree_cap` | ★ 去掉 `e(r)` 进阈值 ⇒ 依据齐全时仍全撤 ⇒ 正向红；★ 依据齐全却把箱体撤掉 ⇒ 反向红 —— 成对 | 注入 ×2 |
| **A19-FIT-3**<br>**（v1.6 新增）** | 拟合回退 ＋ **依据缺失**（任一项 null / 过期）⇒ 全 bin `d_free = null`、bit0 = 0；箱体 `d_block / h_block = null`（🚫 伪造）；同帧一条独立有效的 S 阻挡（`0b0100`）仍在；reason 含 `ground_fit_fallback` ＋ `ground_free_withdrawn`；`src` bit1 按样本 | ★ 无依据仍给封顶 FREE ⇒ 红；★ 无依据仍给几何阻挡 ⇒ 红；★ 把 S 阻挡一并删掉 ⇒ 红 | 注入 ×3 |
| **A19-SEM-2**<br>**（v1.5 新增）** | 整 bin invalid ＋ 满足条件的语义 footprint 2.5 m ⇒ `null / 2.5 / null / 0b0100`（`11` v2.2 样例行） | ★ 注入前要求 G = 1 ⇒ 无框玻璃门场景无阻挡 ⇒ 红；★ 注入后给 `d_free = blind_near`（当作已验证空区间）⇒ 反向红 | 金标 ×2 |
| **A19-CFG-1** | 任一 §8.2 必填键置 `null` ⇒ 拒绝启动且报出该键路径（守 **PSC-2**） | ★★★ 给 `h_tol_m` 加代码默认值 ⇒ null 时照常启动 ⇒ 红 | 启动 |
| **A19-VEL-1** | 无 TF 场景 `velocity_frame == "raw"` | ★ 硬编码 `"ego_removed"` ⇒ 红 | 单元 |
| **A19-CAL-1** | 占位外参（全零/记录缺失）⇒ `extrinsic_calibrated == false` | ★ 判定改「配置存在即 true」⇒ 红 | 单元 |
| **A19-PERF-1** | 快线单帧处理 P99 ≤ §2.2 预算 —— ★★ **在目标机 · 双模型＋编码＋跟踪＋发布共载条件下实测**（v1.2 采纳感知方口径：🚫 由 FLOPs 推导代替） | ★ 遍历内插入 1 ms sleep ⇒ 红。⚠️ 实现前恒红：形制同 `20` #20-8，`xfail(strict=True)` 进 CI 🚫 摘除 | 性能 |
| **A19-PROF-1b**<br>**（v1.2 新增）** | 感知方反例场景（`blind 0.60 / dr 0.25 / d_blk 0.80`，障碍落格中部）⇒ `d_free ≤ 0.60`，🚫 不得 0.85 | ★ 回退成「格中心判停」⇒ 复算出 0.85 ⇒ 红 —— **这条金标就是那个反例本身** | 金标 |
| **A19-SEM-1**<br>**（v1.1 新增）** | 玻璃门场景（深度成片 invalid ＋ 语义 footprint 在洞区）⇒ 洞区 bin `src` 含 BIT_SEM 且 `d_block` 收到注入值；★ 目标超 `objects_stale_ms` ⇒ 不再注入（§3.4A） | ★ 关掉注入 ⇒ 正向红；★ 去掉陈旧上限 ⇒ 目标离开后 `d_block` 钉死 ⇒ 反向红 —— 成对 | 金标 ×2 |
| **A19-BOOT-1** | 配置路径非 `/run/xbrain/resolved/` 前缀 ⇒ 拒启（**PSC-1**） | ★ 指向 `configs/` 源 ⇒ 照常启动 ⇒ 红 | 启动 |
| **A19-BOOT-2** | mock SDK 返回不符 SN ⇒ 拒启（**PSC-3**） | ★ 跳过 SN 比对 ⇒ 红 | 启动 |
| **A19-BOOT-3** | 金标输出被篡改 ⇒ 拒启（**PSC-4**） | ★ 比对结果不检查返回值 ⇒ 红 | 启动 |
| **A19-BOOT-4** | 声明一条 `11` §2.2.1 未登记的 key ⇒ 拒启（**PSC-5**） | ★ 白名单比对改「前缀匹配」⇒ 未登记 key 混过 ⇒ 红 | 启动 |
| **A19-BOOT-5** | `range_max_m = 6.5` ⇒ 拒启（**PSC-6**） | ★ 上限判断去掉 ⇒ 红 | 启动 |
| **A19-BOOT-6** | 朝天外参（全像素 `ray.z ≥ 0`）⇒ 拒启（**PSC-7**） | ★ 预计算表空也放行 ⇒ 红 | 启动 |
| **A19-SLOT-1** | 写侧高频覆盖下读侧永不读到撕裂帧（seq 一致性，守 **P19-2**） | ★ 去掉 seq 重读 ⇒ 并发注入下读出混帧 ⇒ 红 | 属性 |
| **A19-ALLOC-1** | 快线稳态帧处理零动态分配（allocator hook 计数，守 **P19-3**） | ★ 遍历内加一次 `std::vector` 扩容 ⇒ 红。⚠️ 实现前恒红，同 A19-PERF-1 形制 | 性能 |
| **A19-LINT-1** | 静态扫描：感知数据零 ROS topic 发布（守 **P19-4**）· 代码内零硬编码 endpoint 串（守 **P19-5**） | ★ 加一条 image publisher / 写死 `tcp/...` 字面量 ⇒ 红 | 静态 |

★ 七形态自检（`CLAUDE.md` §3.2）：上表每条先问过「有没有一个什么都不做的实现能通过它」——
A19-PROF-4 / A19-RATE-1 因此成对；A19-TIME-2 专杀「快线偷偷等慢线」这个最顺手的错误实现。

---

## 15. ★★ 现有代码差量与工作分解（对照交接文档 G/F 编号）

| # | 工作项 | 对应 | 落点（基底实测文件） |
|---|---|---|---|
| **W-1** | 收编：纳版控 ＋ 清 8 处全角标点 ＋ 删 `THIRD_PARTY_SNAPSHOTS` 排除 ＋ 头注五字段 ＋ 删 `lidar:` 配置块 | §1.2 | `ros2_ws/perception` 全树 |
| **W-2** | ★★★ 外参标定工装 ＋ `extrinsic_calibrated` 判定 | §7 / G-4 前提 | 新增 `tools/calib_extrinsic` ＋ `src/common/config.cpp` |
| **W-3** | ★★★ 快线 `profile_builder`（§3 全部）＋ latest-wins mask 槽；★★ **含取流改造**：快线接 **D2C 前原生深度**（现路径只有对齐/放大帧，§3.2A） | **G-1 ＋ G-3** ＋ PROF-3 v1.9 | 新增 `src/profile/`；`src/modules/orbbec_capture.cpp` 加原生分流；`src/supervisor/supervisor.cpp` 挂线程 |
| **W-4** | key 迁移：`perception/detections|status` → `rt/perception/objects|status`（＋新增 `profile`）；`perception/pointcloud` → `rt/perception/pointcloud`（debug 默认关）；RT/GEN 双会话 | 键零重合问题 | `include/output/zenoh_publisher.hpp` · `src/output/zenoh_publisher.cpp` · `src/common/config.cpp` |
| **W-5** | `velocity_frame` 序列化（一行级）＋ `footprint` 改名/抽稀 ＋ `r_near` ＋ `z_min/max` 区间化 | **G-4** ＋ §4.1 | `src/output/zenoh_publisher.cpp` · `src/supervisor/supervisor.cpp` |
| **W-6** | `StatusMsg` 扩展（分位数 · ROI 比例 · 三个固定 reason）＋ PC-2 事件 | §5 | `src/output/zenoh_publisher.cpp` ＋ 新增 `src/status/` |
| **W-7** | 推理 20 Hz（路线三选，§6） | **#20-11** | 引擎构建脚本 ＋ `dual_model_runtime_config.json` |
| **W-8** | 配置迁 `configs/` ＋ resolved 读取 ＋ PSC-1~7。★ v1.4：**`configs/perception.yaml` 骨架已建**（§8.2 全键 · null 纪律 · 头注五字段）；⚠️ 冻结线 `SNAPSHOT_PROCESSES` 尚未纳入（`20` #20-26），纳入前解析产物形态见 `tests/perception/samples/resolved_perception.dev.yaml` | §8 / §11 | `src/common/config.cpp` 重写读取层（感知方） |
| **W-9** | 合成场景金标 ＋ A19-* 全表 | §14 | 新增 `tests/perception/` |
| ✅ **W-11**<br>**（v1.2 新增 · v1.4 落地）** | ✅ 联调资产已交付（2026-09-11）：`scripts/dev/perception_sim.py`（七场景 normal / all_unknown / no_seg / extrinsic_uncal / tf_stale / clock_reset / dropout，`--write` 生成、`--publish` 在 RT 面真发并把 `*_mono_ms` 重定到本机单调钟）＋ `tests/perception/samples/<场景>/sequence.json`（`11` §3.0 信封 ＋ §3.1B 体的逐字线上形态）＋ `tests/perception/test_consumer_contract.py`（消费端 stub 的机器形式：每个场景 → RNS 判决，样例 = 生成器）＋ `scripts/dev/perception_rx_audit.py`（接收端对表：帧身份 · L2 · Δpub 分级门读数 · 去重/乱序/纪元 · 拒收）—— 感知方 §7「先闭合接口」阶段的对手件，**先于实机标定可做** | 双方接口骨架 | `scripts/dev/perception_sim.py` · `scripts/dev/perception_rx_audit.py` · `tests/perception/` |
| **W-10** | systemd 单元（`Requires=xbrain-config-freeze`，入 15 进程栈启动序） | `10` §3.3 | `deploy/` |

★ 顺序约束：**W-2 先于 W-3/W-6 验收**（投影基准）；W-1 先于一切合入；其余可并行。🚫 点云 `pointcloud` 通道不动（保留调试用途，G-2 裁定「不加密」）。

---

## 16. ★★ 停车场与连带（🚫 本册不代改别册）

| # | 内容 | 交办 |
|---|---|---|
| ★★★ **PCC-9** | ★★★ **`state/targets` / `PerceptionFrame` 族按 RGBD 现实收口**：`11` §3.1 的 `bands`（FS-1~5）· `sectors` · `wpos`（PER-11 含 `rt/gnss` 订阅）· `cls` 闭集 · `ambient` 块（`14` §4.3.2 补光灯依赖）全部建立在 LiDAR 走廊上。★ 连带：**T-27 的 `cam_rgbd` 健康接替判据**（p2 现无可用输入）。★ 本册本期不实现该族（§12），🚫 也不按旧 schema 硬填 | `11` 册主（可与 `20` #20-10 同批） |
| **PCC-10** | `11` §2.2.1 ① 注中「实现方以 `rt/perception/*` 为准」已写；`19` v1.0 落地后建议将 v0.1 引用指针（`PCC-2` 那批「见 perception 设计（待写）」）复核改指本册新节号 | `11` 册主 |
| **PCC-11** | `README.md` 文档表的 `19` 行更新为 v1.0 | 仓库 |

## 17. ★ 本册确实未定（🚫 不假装已关）

| # | 事项 | 为什么裁不了 | 缺省行为（失效方向） |
|---|---|---|---|
| **PD-2**（沿用） | `U-15` 对外媒体端口 | 甲方网络规划 | 不推流 ＋ warn（不阻塞出勤 ✓） |
| **PD-8**（沿用） | MED-2 码率/分辨率/帧率 | 现场带宽实测 | 工程缺省值，不在安全链路 |
| **PD-11** | Orbbec SDK 帧时戳语义（设备钟/主机钟/含否传输）。★ v1.3：`latency_fallback_ms` 须取**上界估计**（估得偏早 ⇒ 年龄偏大 ⇒ 保守）；**估计误差界 E 由感知方实测记录并随包报数**，E > 100 ms ⇒ 回到 `11` §3.1B.5 重议 | 第三方 SDK 事实，须实测 | host 收包 − 配置化链路估计（保守可审计 ✓）；G-P2 门在闭合前不签 |
| **PD-12** | `range_max_m` 户外强光实测值 ＋ 1280×800 升档 | 实测量（前者同 v0.1 PD-10 的教训：🚫 硬编码上限） | `null` 拒启；升档不做（✓） |
| **PD-13** | `terrain` 分档（hard/soft/rough）算法 | 二期能力，需实地数据 | 恒 0 = unknown（诚实 ✓） |
| **PD-14** | 20 Hz 三路线的最终取舍 | 归感知实现方（交接文档 §一A「路线你方定」） | 验收只认 `fps_infer`（✓） |
| **PD-15** | 标定残差阈值 `residual_max_m` | 首次标定实测给出 | `null` ⇒ `calibrated=false`（✓） |
| **PD-16**<br>**（v1.2）** | 实机细障碍验收矩阵（尺寸 × 材质 × 距离）与误检约束。★ v1.3 首版矩阵**提案**见 `perception-rns-reply-20260911.md` §Q5.3（承诺行：≥ 10 cm 杆 ≤ 4 m · ≥ 0.3 m 箱体/人/车 ≤ 6 m · ≥ 0.12 m 路沿 ≤ 3 m · ≥ 0.3 × 0.5 m 坑 ≤ 2 m，阴天/顺光；细线 · 玻璃 · 水面 🚫 承诺；逆光 / 夜间待实测） | 传感器物理，合成金标代答不了（§14 注） | 验收前细障碍能力**不写进任何承诺**（✓） |
| **PD-17**<br>**（v1.2）** | `z_pass_m` 所需的**整机最大扫掠高度**（机体＋载荷＋云台＋步态起伏＋姿态余量，相对地面基准） | 整机侧实测/提供（感知方 Q-7 答复点名），🚫 单次静态站立高度冒充 | `null` ⇒ 拒启（✓） |
| **PD-18**<br>**（v1.5）** | 回退有效条件②的 IMU 重力向量**来源**（338Le 内置 IMU vs 底盘 IMU 经 `quadruped`）与「标定时姿态」的基准记录方式 | 取决于哪路 IMU 先可用、时戳能否与深度帧对齐（PD-11 同型） | 无可用 IMU ⇒ 条件②视为不成立 ⇒ 回退帧只保留独立阻挡（保守 ✓） |
| **PD-19**<br>**（v1.6）** | 回退依据 `Δh` 的来源：perception 订阅通用面 `state/chassis_motion`（取 `height_m`，`11` §9.8.2）须新增 `11` §1.1.6 白名单条目（跨面进程逐条登记，规则 b）；标定层新键 `calib.body_height_m` 记录外参标定时的机身高度 | 负责人裁定白名单；标定随 §15 W-1 同批记录 | 未裁 / 未标 ⇒ 依据缺失 ⇒ 回退整支收窄（保守 ✓，`perception-rns-reply-20260912-r5.md` §五 D1/D2） |

## 18. 变更记录

| 版本 | 日期 | 内容 |
|---|---|---|
| ★ **v1.6**<br>**（第五轮对账）** | 2026-09-12 | ★ 答复感知方《Q4 地面回退安全边界补充确认函》（`perception-rns-reply-20260912-r5.md`）：① §3.2A 回退分支改为**依据门控**（`e(r) = \|Δh\| + r·tan(Δtilt) + e_align`；放行 / 保留判据；任一依据缺失即整支收窄 = 感知方表逐行采纳；v1.5 的 5° 上界作废）；② §8.2 新键 `fallback_align_err_m`；③ §14 A19-FIT-2 改成对 ＋ A19-FIT-3；④ §17 PD-19（`state/chassis_motion.height_m` 白名单 ＋ `calib.body_height_m`）。同批 `20` v1.42 ＋ p1 宿主门改正（新鲜全 null 🚫 否决）。 |
| ★ **v1.5**<br>**（第四轮对账）** | 2026-09-11 | ★ 答复感知方《第三轮技术口径确认》（`perception-rns-reply-20260911-r4.md`）：① §3.2A 先验平面支持 FREE 的两条有效条件（失败原因 = 内点不足 🚫 残差超限；IMU 姿态在标定包络内）＋ 配对断言 `cap × tan(tol) ≤ h_tol` ＋ 不满足时只撤回依赖平面的 FREE（`d_free` 全 null · `h_tol_eff(r)` 独立阻挡 · `ground_free_withdrawn`）＋ 路沿例；② §3.4A G = 0 bin 同样注入（`d_geom` 缺席按 ＋∞，无 FREE 不截断）；③ §8.2 四新键；④ §14 A19-FIT-2 / A19-SEM-2；⑤ §17 PD-18。同批 `11` v2.2 / `20` v1.40。 |
| ★ **v1.4**<br>**（W-8 / W-11 落地）** | 2026-09-11 | ★ W-11 对手件交付（`scripts/dev/perception_sim.py` 七场景样例集 ＋ `tests/perception/test_consumer_contract.py` 消费判决对表 ＋ `scripts/dev/perception_rx_audit.py` 接收端记账）；W-8 的 `configs/perception.yaml` 骨架（§8.2 全键）＋ 产物形态 dev 样例；冻结线纳入登记 `20` #20-26。 |
| ★★ **v1.3**<br>**（第三轮对账）** | 2026-09-11 | ★ 答复感知方 09-11 来函（`perception-rns-reply-20260911.md`）：① §2.2 两道门命名 G-P1（处理 ≤ 14 ms，A19-PERF-1 锚）/ G-P2（发布时年龄 ≤ 60 ms，PD-11 后签）；② §3.2 遍历中按「有有效样本」置 BIT_G、§3.4 推进分支不再置、§3.4A BIT_SEM 只在更近时置 ＋ 无位姿时 footprint 按 PROF-5 同式膨胀 ＋ `objects_stale_ms` 改约两个推理周期（`11` §3.1B.1 v2.1 `src` 逐位定义）；③ §3.2A 拟合回退帧 `d_free` 封顶 `dfree_cap_fallback_m`；④ §3.3 帧级 `seg_max_age_ms` 与 10 s 能力标志分开；⑤ §5.1 / §6 节拍口径改逐次间隔**分级门**（用户 2026-09-11 选定：一级 `max ≤ 100 ms` · 二级 `>50 ms` 占比 ≤ 1%；`infer_gap` / `infer_gap_ms_max` / `infer_rate_low` 改写），A19-RATE-1 改写为间隙注入三件套；⑥ 1280×800 主输出（2026-09-11 负责人裁定）：§3.2 mask 像面、§3.2A 注记、§13.1 MED-2 独立支路；⑦ §6.1 生产机事实（Orin NX 16 GB）；⑧ §8.2 新键 `seg_max_age_ms` / `dfree_cap_fallback_m`；⑨ §14 增 A19-SRC-1 / A19-SEG-2 / A19-FIT-1；⑩ §17 PD-11 误差界、PD-16 矩阵提案。同批 `11` v2.1 / `20` v1.35。 |
| ★★★ **v1.2**<br>**（对账修正轮）** | 2026-09-08 | ★★★ **按感知方接口答复逐条修正**（该答复抓到本册/交接文档多处实错，全部采纳）：① §3.4 `d_free` 推进改**格远端判停 ＋ 整格覆盖率 `G/expect_cell ≥ cover_min`**（其反例可复算出 `d_free 0.85 > d_blk 0.80`，PROF-1「构造性成立」原不成立）＋ 金标 A19-PROF-1b 就用该反例；② 负障碍两处修正：穿地证据记**期望交点** `bin_exp`（横向平移反例：期望 16.7° vs 回波 8.5°）；`h_block` 负障碍**优先**，🚫 被正障碍最大高度覆盖；③ 新增 §3.2A **逐帧地面平面拟合**（🚫 恒 z=0；失败退先验 ＋ `ground_fit_fallback`），`z_exp` 由静态表改逐帧闭式；④ 快线取 **D2C 前原生深度**（PROF-3 v1.9），mask 改**投影查表**（跨像面比例缩放不成立）；⑤ §4.2 `ego_removed` 置位四条件（精确时刻 TF · 旋回 base_link · 外参非占位 · 同纪元）；⑥ A19-PERF-1 改共载实测口径；合成金标声明保证边界，实机细障碍验收登记 **PD-16**；`z_pass` 扫掠高度登记 **PD-17**；⑦ 过渡期旧 key 开关 `legacy_keys_enable`（默认关）；W-11 模拟消息样例集。★ 同批 `11` v1.9 八处（PROF-3 收窄 · PROF-5 加旋转项 · 角度/量纲/遮挡/分源约定 · 20 Hz 口径冻结 · `infer_gap_ms_p99` · 两个新 reason · 探针数据出处订正）。 |
| ★ **v1.1** | 2026-09-08 | ★★ **三遍核查轮（同日）**：① 补 **§3.4A** `src` bit2 语义注入产生规则（初版恒 0 ⇒「语义看见、几何瞎」的玻璃门场景在 profile 里不可见）＋ A19-SEM-1 正反对；② §3.5 补**报文头字段逐个产生表**（`pose_used`/`t_publish`/`schema` 初版无人产）；③ `rt/perception/pointcloud` 以 **debug 默认关**登记（同批 `11` §2.2.1）—— 否则基底既有通道撞 PSC-5 白名单精确比对；④ §14 补 **PSC-1~7 / P19-1~5 全员变异体覆盖**（A19-BOOT/SLOT/ALLOC/LINT 族）—— MUT-COVER 门禁同批改锚新结构；⑤ §3.1 预计算补 `r_exp`；`conf` 除零边界。 |
| ★★★ **v1.0**<br>**从 0 重写 · 落地版** | 2026-09-08 | ★★★ **v0.1 整册作废**（LiDAR 前提崩塌，§0.2 三区处置表）。★ 接口真源移交 `11` §3.1B（v1.7），本册转纯实现侧：五执行体快慢线（§1/§2）· ProfileMsg 管线可编码规格（§3，PROF-1~5 逐条落点）· ObjectsMsg 字段映射（§4）· StatusMsg 与事件（§5）· 20 Hz 达标设计（§6）· 外参标定（§7）· 配置/自检/降级/单调钟（§8~§11）· 断言总表 A19-*（§14，含三对正反向）· 工作分解 W-1~W-10（§15）。★ v0.1 存活裁定收入 §13/§17 逐条注明沿用；停车场 PCC-9 一揽子登记（§16）。★ 跟随/re-ID 按 `20` #20-13 预留，不入本册范围（§0.3） |
| ~~v0.1~~ | ~~2026-08-05~~ | ★ **已作废**，见 §0.2。其对抗验收方法论（变异体表 · PCC/PD 双清单 · 扫描面声明）本版保留并沿用 |
