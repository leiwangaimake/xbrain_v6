# perception → RNS 接口交接说明

**XBRAIN_V6 · 2026-09-07**

> 本文件是**发给 perception 实现方的交接快照**。
> ★★★ **它不是真源**：字段 · 类型 · 单位 · 闭集 · 超时号的唯一定义处是 **`11` §3.1B**（接口契约）。
> 本文只回答三件事：**为什么现在要改** · **要改成什么** · **现有实现差在哪**。
> 两处如有出入，**以 `11` §3.1B 为准**。

| 项 | 内容 |
|---|---|
| 机器人 | 云深处 山猫 M20S · `gj-001` |
| 相机 | Orbbec Gemini 338Le（Ethernet，SN `CHL5663000DS`） |
| 被审代码 | `ros2_ws/perception`（2026-09-07 工作区快照，⚠️ **尚未纳入版本控制**） |
| 结论 | 需**新增三条 key**；现有输出**四个缺口**；另有**三条实测约束**须纳入设计 |

---

## 一、前提变了（先读这一节，再读下面的要求）

2026-09-07 两条裁定，直接决定 perception 的输出形态：

| # | 裁定 | 对 perception 的后果 |
|---|---|---|
| **1** | **整机无 LiDAR**，几何全部由 RGBD 产生 | ★★★ 原契约里 `rt/lidar/scan` / `points` / `grid` 三条 key **没有生产者了**。导航要的几何必须从深度图直接降维产出 |
| **2** | **RNS 是系统唯一的导航软件栈**（`20` §1.1 `RNS-M-6`），`path_follow` 删除 | ★★★ **RNS 的能力上限完全由本接口决定**。它没有第二个几何来源可以兜底 |

⚠️ 顺带说明一个**已核实不成立**的担心：`config/perception.yaml` 里有 `lidar: topic: "/livox/lidar"` 配置块，
代码里也有 `lidar_velocity_estimator_` 这样的名字。**这两处都是历史命名** ——
全代码只有一个 `create_subscription`（`capture_cmd_sub_`，String 命令），**没有任何地方订阅 `/livox/lidar`**，
该估计器实际跑的是相机数据（配置注释逐字「Camera-only, category-independent Z evidence gate」）。
⇒ **无需为"没有 LiDAR"做任何拆除工作**，但那个配置块建议删掉，它会误导下一个读代码的人。

---

## 一A、硬性指标（2026-09-07 项目裁定，🚫 不是建议）

> ### **推理链路必须优化到 ≥ 20 Hz**（`fps_infer ≥ 20`，稳定值，非峰值）

| 项 | 内容 |
|---|---|
| **要求** | 双模型推理（检测 ＋ 分割）合成的 `objects` 输出稳定 ≥ **20 Hz**。现状 18.73 Hz 不达标 |
| **验收** | `status.fps_infer` 的 **10 s 均值 ≥ 20**；持续低于必须在 `degraded_reasons` 报 `infer_rate_low`，🚫 不得静默 |
| **登记** | 我方设计册 `20` §15 **#20-11**；契约 `11` §2.2.1 / §3.1B.0 频率行已改写为硬性要求 |

**可行性提示**（供参考，路线你方定）：

- Active12 现跑 896×896 **FP16** —— INT8 校准通常是这类 DLA 负载最大的一口余量；
- 双模型分居 DLA0/DLA1 —— 确认两条引擎**流水重叠**而非串行等待；
- **GPU 现在基本空闲**（模型全在 DLA）—— 把一个模型挪到 GPU 是现成的第二条通道；
- 检测输入 640×640 与发布路径上的前后处理（NMS / mask 解码 / JSON 组包）也在这条链路的预算里，别只看 engine 时延。

⚠️ **两条不要误读**：

1. **20 Hz 是吞吐要求，不抵延迟要求** —— 流水线化的 20 Hz 照样可以背着 200 ms 的结果年龄跑。F-1 的延迟问题（Q-1）**照样要答**，TIME-2 的快慢线拆分**不回退**。
2. **名义 20 Hz ≠ 与控制拍相位对齐** —— RNS 侧"每拍必须重跑"的纪律不因达标而撤。

## 一B、新增需求：目标重识别（re-ID，跟随任务的前置）

> ⚠️ **2026-09-08 更新：跟随功能本期不实现（预留）。本节降为背景需求 —— 你方无需现在排期，调研结论与验收口径保留，立项时以此为起点；Q-10 同降为背景问题。**

系统新增 `follow_target` 跟随任务（我方设计册 `20` §2.10）。跟随的实用上限由一件事决定：
**目标被遮挡消失、再出现后，`track_id` 能不能绑回原来那个**。项目要求（2026-09-07）：
用外观特征嵌入实现"目标消失再出现，经特征计算 `track_id` 固定"。

### 调研结论（2026-09-07）

**路线 A（推荐）：嵌入库 re-ID，挂在现有跟踪器上**

| 候选 | 说明 |
|---|---|
| **NVIDIA TAO ReIdentificationNet** | NGC 现成模型（ResNet50 基线，另有 Transformer 变体），官方支持 TensorRT / DeepStream 部署。**DeepStream 的 NvDCF / NvDeepSORT 跟踪器已内置我们要的整套机制**：目标丢失前按 `reidExtractionInterval` 抽取嵌入存入 gallery，跨遮挡/跨帧间隙做重关联 —— 即使不整体换 DeepStream，这套 gallery 匹配逻辑也可以照搬进现有管线 |
| **OSNet（osnet_x0_25）** | 轻量级全尺度 re-ID 网络（x0.25 版约 0.17M 参数），FastReID / torchreid 生态成熟，有 TensorRT 落地先例（YOLO + TensorRT-OSNet 管线的公开实测） |

**做法**：对（至少）`person` 与车辆类的每个活跃 track，按间隔抽取裁剪图嵌入（512 维级别），
维护每 track 的特征模板（EMA）；track 丢失后保留 gallery；重捕获窗口内的新 track 与 gallery 做
余弦匹配，超阈值 ⇒ **重绑定原 `track_id`**。跟随场景同时活跃目标数很小，嵌入抽取的算力可忽略。

**路线 B（二期备选）：SAM2 系单目标跟踪（SAMURAI 等）**

零样本、任意类别、靠记忆机制天然处理遮挡（SAMURAI 加了运动感知记忆选择，训练无关）。
但计算量对 Orin 上已有的双模型 DLA 管线是额外大头，公开实测多在桌面 GPU —— 列为技术储备，不做首版依赖。

### 要求与验收

| 项 | 内容 |
|---|---|
| 要求 | 目标消失 ≤ T 秒重现 ⇒ 绑回原 `track_id`（T 实测定，建议从 5 s 起谈） |
| 验收 | 同装束 · 同场景下重绑定成功率 ≥ 阈值（与 T 一起实测定）。**换装 / 隔天不承诺** —— 那是 re-ID 的开放学术问题，不进验收 |
| 接口 | **`ObjectsMsg` 不加字段** —— 重绑定发生在你方内部，对外表现就是 `track_id` 稳定。做不做 `reid_confidence` 可选字段由你方提，我方不强制 |
| 时序 | RNS 首版**不依赖** re-ID（丢失按超时报失败）；到位后我方只放宽丢失超时。⇒ **不阻塞你方其他项，但决定跟随好不好用** |

## 二、要做什么：三条新 key

**定义处 `11` §3.1B。** 这里只列骨架，字段表请直接看契约。

| key | 内容 | 频率 | QoS | RNS 拿它做什么 |
|---|---|---|---|---|
| `xbrain/{rid}/rt/perception/profile` | **几何三态剖面**（定长数组） | **与深度帧同频（30 fps 目标），🚫 不等推理** | Q1 | ★★★ **缝宽 · 走廊 · 贴墙绕行的唯一来源** |
| `xbrain/{rid}/rt/perception/objects` | 语义目标（变长） | 与**推理**同频 —— **20 Hz 硬性要求（§一A）**，现状 ~18.7 | Q1 | ★★ **行为调制**：谁该让 · 谁能绕 · 谁必须硬禁 |
| `xbrain/{rid}/rt/perception/status` | 健康心跳 | 1 Hz | Q2 | ★ 降级与限速判据 |

### 2.0　时序契约（TIME-1 ~ TIME-3，最容易做错的一条先说）

| # | 规则 | 为什么 |
|---|---|---|
| **TIME-1** | 单条报文内部**单帧一致**：一条 `profile` 全部数组出自同一深度帧；一条 `objects` 全部目标出自同一推理帧 | 混帧 ⇒ 物体边缘系统性假矛盾 |
| **TIME-2** | ★★★ `profile` 与 `objects` **不要求同帧** —— 各带各的 `t_capture_mono_ms` | ★★★ **`profile` 若等推理，纯几何的 `d_block` 年龄 ≈ 200 ms（推理延迟）＋ 一个推理周期 ≈ 250 ms，贴死 RNS 侧 300 ms 的超时上限**。拆开后 `d_block` 端到端可压到 < 100 ms（采集 33 ＋ 传输 ~5 ＋ 几何 ~10 ＋ 发布）。★ **推理提到 20 Hz（§一A）后本条照旧** —— 吞吐 ≠ 延迟 |
| **TIME-3** | `profile` 里的 T 证据允许用**更早的分割帧**，但必须带 `t_seg_mono_ms`，且发布前按 PROF-5 腐蚀 | 机器人在动而 mask 是旧的 ⇒ 边界错位方向不保守，必须收缩补偿 |

⇒ **实现形态**：几何路径（深度 → 剖面）是**独立的快线**，随每个深度帧走；推理路径（检测/分割）是**慢线**；
慢线的最新分割结果以"最近可用 ＋ 腐蚀"的方式并入快线的 T 判定。🚫 不要让快线等慢线。

⚠️ **当前实现发的是 `perception/detections` / `perception/status` / `perception/pointcloud`（无 `rt/` 前缀）**，
与契约注册的 key **零重合**。这不是本次新增的分歧 —— 它一直存在，只是此前没有消费方所以没暴露。

### 2.1 `profile` 是新增的大头

定长逐方位数组，`base_link` 系，前向为 0，逆时针为正：

- `angle_min_rad` / `angle_step_rad` / `n_bins` 描述扇区（本册示例 ±45°、0.5°、181 bin）
- 七个等长数组：`d_free` · `d_block` · `h_block` · `src` · `conf` · `terrain` · `slope_deg`
- 报文头带 `t_capture_mono_ms`（**曝光中点**）· `t_seg_mono_ms`（T 证据的分割帧时戳，无则 `null`）· `z_pass_m`（过顶滤除高度，PROF-4）· `blind_near_m`（**逐帧给**）· `extrinsic_calibrated`

**三条不变量**（`11` §3.1B.1，做错了 RNS 会在危险方向出错）：

| # | 内容 | ⚠️ 违反的后果 |
|---|---|---|
| **PROF-1** | `d_free[i] ≤ d_block[i]` 恒成立，中间那条带**就是 UNKNOWN** | 令二者相等把 UNKNOWN 消掉 = 宣称「没看见障碍就是没障碍」 |
| **PROF-2** | 深度 invalid **保持 UNKNOWN**，🚫 不填 `0`、🚫 不填 `range_max` | 玻璃 / 水面 / 强反光 / 超量程处成片 invalid，填默认值会让机器人**全速冲进去** |
| **PROF-3** | `d_block` 由**全分辨率深度的角度域 MIN 归约**得到，🚫 不得先空间降采样再归约 | 先降采样第一步就把**细障碍平均掉**（矿区钢筋 / 树林细枝 / 营地拉索）；MIN 是保守的 —— 细障碍只要 1 个有效像素就存活 |
| **PROF-4** | **过顶滤除**：`h > z_pass_m` 的点不入 `d_block`，且 `z_pass_m` 随帧发布 | 不滤 ⇒ 1.8 m 的横杆/门楣把通道判死，机器人钻得过却永远不走；滤了不发 ⇒ RNS 不知道这层假设 |
| **PROF-5** | 跨帧 T 证据按 `m = v_ego_max × (t_capture − t_seg) ＋ m_jitter` 在地面域**腐蚀**后才可支持 FREE | 旧 mask 的边界已随自车运动错位，不腐蚀则误差方向不保守 |

### 2.2 缺值一律 `null`，🚫 不用哨兵

`+inf` / `NaN` / `-1` / `-128` / `0` / `range_max` 一概不许当"没有数据"用。两条理由：

1. **JSON 没有 `+inf` / `NaN` 字面量** —— 写了就不是合法 JSON，各家解析器行为不一致
2. ★★★ **哨兵会被下游当真值参与算术** —— `d_block = -1` 进 `min()` 就是「零距离有障碍」，`d_block = 0` 就是「贴脸障碍」，两种都在**错误方向**上出错

带宽不是问题：7 × 181 个数约 10 KB/帧，30 fps ⇒ **约 300 KB/s**，RT 面不出机。🚫 **不要为省这点带宽提前上 CBOR。**

### 2.3 融合请**留给 RNS**

各通道分别降维，用 `src` 位掩码保留"**谁说的**"（可通行分割 / 几何 / 语义 / 负障碍），
🚫 **不要在 perception 内融合成单一的"堵/通"结论**。

理由是语义通道的价值不在冗余检测，在**行为差异**：`person` 停车礼让不绕 · 设备箱体绕行 ·
浅草落叶可穿越限速 · 明火深水崖边硬禁止并额外膨胀。**融合成"堵"后这四种全部塌缩成一种，
花了语义的算力只拿到几何的信息量。**

---

## 三、现有实现：先说做对的

审下来工程质量是高的，以下几处**超出**契约要求，请保留：

| 项 | 现有实现 |
|---|---|
| **地面凸包** | `ground_footprint_odom.polygon_xy` —— ★ 已经是地面多边形而不是图像框。这正是 RNS 要的（图像框 ＋ 深度是个**视锥**，算不出缝宽） |
| **深度诊断** | `depth_stat{min,median,max,stddev}` ＋ `depth_diagnostics{valid_fraction, selection_status, ...}`，细致程度超出契约要求 |
| **速度抗抖** | `window_s: 0.30`（窗口而非相邻帧）＋ `motion_confirmation_windows: 2`（连续两窗同向才确认）＋ 分类 EMA。★★ 这正是 RNS 判静止所需的「阈值 ＋ 驻留」范式 |
| **诚实自标** | `semantic_status: confirmed/proxy` · `velocity_valid` · `navigation_authoritative: false` —— ★★★ **明确标注自己不可信之处，这在感知模块里很少见，本次设计按它的字面意思采信** |
| **跟踪 ID 生命周期** | `track_id` ＋ `stable_frames` ＋ `first_seen_ms`/`last_seen_ms` 齐全 |

---

## 四、四个缺口与实现方案（算法级）

> 每个缺口给到**公式与伪代码**级别。符号约定：
> 内参 `K = [[fx,0,cx],[0,fy,cy],[0,0,1]]`；外参 `T_bc = (R_bc, t_bc)`（标定产物，`base_link ← camera`）；
> 地面 = `base_link` 的 `z = 0` 平面；扇区参数 `angle_min / angle_step / n_bins`；径向格宽 `dr`（建议 0.25 m）。

### 4.1　G-1 ＋ G-3：几何三态剖面（一遍循环，含可通行 mask 投地）

G-1（三态剖面）和 G-3（mask 投地面）**是同一个逐像素循环**，分开做会白遍历一次深度图。

**第一遍：逐像素分类与归约**（全分辨率，PROF-3）

```text
预计算 (外参标定后一次):
  ray[v][u]   = R_bc * K_inv * [u, v, 1]^T          # 每像素射线方向 (base_link 系, 未归一)
  z_exp[v][u] = -t_bc.z / ray[v][u].z               # 该像素射线与地面相交的期望深度参数
                                                    # (ray.z >= 0 即不指向地面 => 标记为无期望)
  roi[v][u]   = (z_exp 有效 且 交点距离 <= range_max) # invalid_pixel_ratio 的分母掩膜

每帧:
  for (u, v) 全分辨率:
    z = depth(u, v)
    if invalid(z):
        if roi[v][u]: n_invalid += 1                # 只统计地面相关 ROI
        continue                                    # PROF-2: invalid 不产生任何几何证据
    p = z * ray[v][u] + t_bc                        # base_link 坐标 (x, y, h)
    r = hypot(p.x, p.y)
    if r < blind_near or r > range_max: continue
    th = atan2(p.y, p.x)
    i  = floor((th - angle_min) / angle_step)
    if i < 0 or i >= n_bins: continue

    if p.h > z_pass:            continue            # PROF-4: 过顶, 机器人可从下方通过
    elif p.h > h_tol:                               # 立体障碍点
        d_blk[i] = min(d_blk[i], r)                 # PROF-3: 全分辨率角度域 MIN
        obs[i].push(r, p.h)                         # 供 h_block 窗口统计
    elif p.h >= -h_tol:                             # 地面证据点
        k = floor((r - blind_near) / dr)
        G[i][k] += 1;  Zs[i][k] += p.h;  Zq[i][k] += p.h * p.h
        if seg_mask(u, v): T[i][k] += 1             # G-3: mask 最近邻查表, 同一遍完成
    else:                                           # p.h < -h_tol: 实测低于地面
        d_neg[i] = min(d_neg[i], r);  h_neg[i] = min(h_neg[i], p.h)

    if roi[v][u] and z > z_exp[v][u] * (1 + eps_neg):
        pierce[i][floor((r_exp - blind_near)/dr)] = true
        # 射线"穿过了地面本该在的位置" => 那里没有地面 => 负障碍的第二判据
        # r_exp = 期望交点的地面距离(可与 z_exp 一并预计算)
```

`seg_mask(u, v)`：分割输出分辨率与深度不同 ⇒ 最近邻缩放查表 `mask[v * sy][u * sx]`，O(1)。

**第二遍：逐 bin 后处理**（181 × ~24 cell，微不足道）

```text
T 腐蚀 (PROF-5, 分割帧比深度帧旧时):
  m = v_ego_max * (t_capture - t_seg) + m_jitter    # 地面域收缩余量, 单位 m
  T_ok[i][k] = T[i][k]/G[i][k] >= tau_T  对 [i][k] 及其 m 半径内全部邻 cell 成立
               (角向邻域半径 = m / (r_k * angle_step), 径向 = m / dr)

for i in bins:
  d_free = blind_near;  gap = 0;  zbar_prev = null
  for k in cells:                                   # r_k = blind_near + (k + 0.5) * dr
    if r_k >= d_blk[i]: break                       # PROF-1 由构造成立: d_free <= d_block
    if pierce[i][k]:                                # 负障碍确认
        d_blk[i] = min(d_blk[i], r_k); src[i] |= BIT_NEG
        h_block[i] = h_neg[i] (若实测到坑内点) else null
        break
    if G[i][k] < g_min:                             # 无地面证据 (遮挡 / invalid / 未观测)
        break                                       # 首版: 即停, d_free 停在缺口前 (保守)
    zbar = Zs[i][k]/G[i][k];  var = Zq[i][k]/G[i][k] - zbar^2
    if has_seg and not T_ok[i][k]:           break  # T 停止: 仅本帧有分割(t_seg 非 null)时生效
                                                    # 无分割 => 不 break, 继续推进, 但该段 src 不置 bit0
                                                    # (bit0 未置 = 仅几何可通行, RNS 侧按 20 3.1.11 限速消费)
    if zbar_prev != null and
       |zbar - zbar_prev| / dr > tan(slope_max):    break  # 台阶 / 陡坡
    if var > sigma_max^2:                           break  # 粗糙度
    zbar_prev = zbar;  d_free = r_k + dr/2
  d_free_out[i] = d_free
  h_block[i] = max(h for (r, h) in obs[i] if r <= d_blk[i] + w_h)   # 障碍簇最大高度
  conf[i] = clamp255(255 * valid_px[i] / expect_px[i])              # expect_px 由 roi 预计算
  slope_deg[i] = atan(最大相邻 |zbar 差| / dr) over 已通过的 free 段
  terrain[i] = 0                                    # 首版恒 unknown, 二期再分 hard/soft/rough
```

**阈值全部进感知侧配置**（`h_tol` / `z_pass` / `g_min` / `tau_T` / `slope_max` / `sigma_max` / `eps_neg` / `w_h` / `dr`），
未标定写 `null` 拒绝启动 —— 与我方 `CLAUDE.md` §3.1 同一条纪律。

**复杂度**：逐像素 O(1)，无排序无邻域搜索。640×400@30 ≈ 7.7M px/s；1280×800@30 ≈ 30.7M px/s ×
约 30 flops ≈ **0.9 Gflops/s**，Orin CPU 单核 NEON 可承受，与 DLA 上的推理零竞争。
第二遍 181×24 cell 可忽略。**帧内存**：`G/Zs/Zq/T` 各 181×24×4 B ≈ 70 KB。

### 4.2　G-2：点云**不加密** —— profile 就是导航几何

原判"点云太稀（0.1 m 体素 / 5000 点）"的修复**不是加密点云**，是**让 profile 取代点云成为导航几何**：

- bin 的横向分辨率 = `r × angle_step`。0.5° 时 6 m 处 ≈ **5.2 cm**；
- 1.2 m 的缝在 6 m 处横跨 ≈ 0.2 rad ≈ **23 个 bin** —— 缝宽判定裕量充足；
- 直径 10 cm 的杆在 4 m 处 ≈ 1.4° ≈ **2.9 个 bin**，且 MIN 归约保证**只要 1 个有效像素就存活**（PROF-3）。

⇒ `perception/pointcloud` 保持现状（调试/可视化），🚫 不为导航加密 —— 加密走网络的代价（§六）买不到 profile 之外的信息。

### 4.3　G-4：`velocity_frame` 序列化 ＋ 去自车运动

**第一步（本周可做，一行级改动）**：把已有的 `last_velocity_frame_has_odom_` 序列化：

```text
velocity_frame = has_odom ? "ego_removed" : "raw"
```

**第二步（odom 到位后）**：去自车运动的正式算法 ——

```text
T_wb(t): 从 OdomBuffer 取 t 前后两帧位姿插值 (平移 lerp, 旋转 slerp)
         t 用目标质心所在帧的曝光中点 t_capture, 不是消息到达时刻
q1 = T_wb(t1) * p1;  q2 = T_wb(t2) * p2            # 两帧质心变换到世界系
v_w = (q2 - q1) / (t2 - t1)                        # 世界系速度 (自车运动已消)
v_b = R_wb(t2)^T * v_w                             # 旋回当前 base_link (速度只旋转不平移)
velocity_xy = (v_b.x, v_b.y);  velocity_frame = "ego_removed"
```

现有的窗口化 + 两窗确认抗抖直接套在 `q` 序列上，逻辑不变。

**`r_near` 精确定义**（`11` §3.1B.2 v1.7 已收严）：`base_link` 原点到凸包的最小距离 =
逐边取点到线段距离的最小值（🚫 不是只看顶点 —— 长边中段可能最近）；原点在多边形内 ⇒ `0`。
机体半径由 RNS 侧扣，perception 🚫 不代扣。

## 五、三条实测约束（不是缺口，是必须纳入设计的事实）

| # | 事实 | 对 RNS 的后果 |
|---|---|---|
| ★★★ **F-1** | **E2E P99 约 166–226 ms，结果年龄 P99 219.5 ms**（你们 README 亦自标"仍高于门限"） | ★★★ RNS 是 20 Hz（50 ms 一拍），而感知数据**平均已经 200 ms 旧**。2 m/s 下是 **0.44 m 位移，大于机体半径** ⇒ **运动补偿是必需项，不是优化项**；而补偿要 odom ⇒ 卡在 `quadruped` |
| ★★ **F-2** | 深度采集 **640×400**（宽高比 1.6，**没有裁 FOV**，这点做对了） | 深度误差按 `dz = z^2 * dd / (f * b)` 约翻倍 ⇒ **远处 `d_free` 不可信**，速度门的输入精度受限 |
| ★★ **F-3** | 输出约 **18.73 Hz** —— **该数是推理频率**，时序拆分（TIME-2）后只约束 `objects`；`profile` 随深度帧 30 fps。**§一A 已把 20 Hz 定为硬性要求，本行是待关闭的现状** | 达标后抖动下每拍仍不保证新帧（名义同频 ≠ 相位对齐）⇒ RNS 侧"每拍必须重跑"照旧，这一侧我们担了 |

★ 另：`range_max_m` **不得超过传感器推荐量程**。338Le 厂商推荐 0.25–6 m（精度 ≤0.8%@2 m / ≤1.6%@4 m），
超出部分误差随 `z^2` 增长，写进 `d_free` 等于**把不可信的远处当成确认可通行**。有疑问就填小 —— 失效方向不对称。

---

## 六、关于提高深度分辨率（1280×800）的评估

**结论：算量不是瓶颈，延迟和带宽才是。请按用途分路，🚫 不要统一降采样。**

- **算量**：深度图是 organized 二维数组，反投影 ＋ 变换 ＋ 地面判定 ＋ 角度归约合计约 20–30 flops/点，
  1280×800@30 fps 约 **1 Gflops/s**。且双模型跑在 **DLA0/DLA1**，GPU 基本空闲 ⇒ **不构成障碍**。
- **带宽**：1280×800×16 bit×30 fps ≈ **492 Mbps**，加 Color 流后千兆链路占用过半。
- ★★★ **延迟**：F-1 已经超标，提分辨率的第一风险是把它推得更高。

| 路径 | 分辨率 | 理由 |
|---|---|---|
| ★★★ **安全 / 几何剖面** | ★★ **全分辨率** | 角度域 **MIN 归约本身就是降维**；MIN 保守 ⇒ 细障碍只要 1 个有效像素就存活 |
| 地面拟合 / 聚类 | 可降采样 | 本质是求平均，降采样无损，省算力 |
| 分割 / 检测 | 模型输入尺寸 | 受模型限制 |

⚠️★★★ **顺序不可换**：先降采样再归约，**第一步就把细障碍平均掉了**。
矿区钢筋 · 树林细枝 · 营地拉索正是"最容易漏 ＋ 撞上后果最严重"的一类。

---

## 七、建议的落地顺序

| 序 | 事项 | 为什么在这个位置 |
|---|---|---|
| **1** | ★★★ **整机外参标定**（现 `extrinsic.yaml` 为占位、`extrinsic_base_lidar` 全零、注释逐字「待本体标定后替换为实测外参」） | ★★★ 没有它，**地面投影 · 近场盲区 · footprint 全部不准** —— 先改接口是白改 |
| **2** | ★★ **`velocity_frame` 序列化**（G-4） | ★ 改动量最小、风险最低，且**在 odom 到位之前就能让问题变可见** |
| **3** | ★★ **确认延迟能压到多少**（F-1） | 决定运动补偿的形态 |
| **4** | ★ **补 `ProfileMsg`**（G-1）＋ 可通行区投地面（G-3） | 前几条有着落后再动，否则投影基准不可信 |

★ `extrinsic_calibrated` 字段请从第 1 步起就随帧发。契约规定其为 `false` 时 **RNS 拒绝自主导航**
（`11` §3.1B.4，上报 `extrinsic_uncalibrated`）—— **让"未标定"成为报文里可见的事实，而不是靠人记得。**

---

## 八、数据闭环对账单（perception → p1_motion → RNS）

> RNS 是 `p1_motion` 的**进程内模块**（对外呈现为行为源 `rns_avoid`），订阅方就是 `p1_motion`。
> 下表把 RNS 的**每一个消费点**对到字段 · key · 超时 · 降级 —— 一行对不上就是通路断点。
> 双方联调时按此表逐行打勾。

| RNS 消费点 | 用的字段 | key | 超时（`11` §1.6） | 断供时的行为 |
|---|---|---|---|---|
| 三态融合（FREE/BLOCKED/UNKNOWN） | `d_free` / `d_block` / `src` | `profile` | T-50 300 ms / T-51 1 s | 限速 → 零速（不锁定） |
| 速度门 `f(d_free)` | 前向扇区 `min d_free[i]` | `profile` | 同上 | 同上 |
| 缝宽 / 绕行候选 / 贴墙 | `d_free` / `d_block` 逐 bin | `profile` | 同上 | 同上 |
| 可跨越判定 | `h_block` ＋ `z_pass_m` | `profile` | 同上 | 同上 |
| 负障碍（坑/崖） | `src` bit3 ＋ `h_block < 0` | `profile` | 同上 | 同上 |
| 记忆栅格（360° 补全） | 三态 ＋ `t_capture_mono_ms` | `profile` | 记忆 TTL 另计 | 栅格过期格转 UNKNOWN |
| 行为分流（让/绕/穿/禁） | `class_name` → RNS 侧 `class_map` | `objects` | T-52 500 ms | **全部 BLOCKED 按禁止类**（不绕不穿） |
| 运动/静止判据 | `velocity_xy` ＋ `velocity_frame` ＋ `velocity_status` ＋ `stable_frames` | `objects` | T-52 | `raw` ⇒ 拒用；断供 ⇒ 保守 |
| 停车距离 | `r_near` | `objects` | T-52 | 保守（按 profile 几何距离） |
| 健康限速 | `invalid_pixel_ratio` | `status` | T-53 3 s | 按最坏情况限速 |
| T 通道降级 | `traversable_seg_available` ＋ `t_seg_mono_ms` | `status` ＋ `profile` | — | 限速通行 ＋ warn（`20` §3.1.11） |
| 标定门 | `extrinsic_calibrated` | 三条皆有 | — | **拒绝自主导航**（`11` §3.1B.4） |

**perception 之外的闭环项**（列全，防"只对了感知半边"）：

| RNS 消费/产出点 | 通路 | 状态 |
|---|---|---|
| 定位 / 航向 | `rt/gnss/fix` / `heading`（`rtk_driver`） | 契约既有（T-09 / T-10） |
| 急停 | `cmd/estop`（p1 白名单 P1-21，RNS 无条件停） | 契约既有 |
| 路径下发 | `cmd/motion/route`（P1-11）→ RNS 折线 | 契约既有 |
| 执行进度 | `state/motion/path_progress`（P1-12） | 契约既有 |
| **RNS 输出 → 仲裁** | 行为源 `rns_avoid` 进 `12` 的优先级阶梯 | ⚠️ **已登记待办 #20-1**（`12` 需删 `path_follow` 重排；不阻塞感知侧） |
| 旧几何链退场 | `rt/lidar/*` ＋ `rt/perception/targets` | ⚠️ **已登记待办 #20-10**（契约标注"以 `rt/perception/*` 为准"） |

⇒ **结论**：感知侧 12 个消费点全部有字段、有 key、有超时、有降级；系统侧两个已知开口（#20-1 / #20-10）
都已登记且不阻塞感知开发。**没有第三个开口。**

## 九、需要你方答复

| # | 问题 |
|---|---|
| **Q-1** | 延迟（F-1）能压到多少？给一个可承诺的 P99。这个数直接决定 RNS 要不要做运动补偿、以及补偿到什么程度。★ **§一A 的 20 Hz 不抵这条** —— 吞吐达标后延迟仍须单独承诺 |
| **Q-2** | `ProfileMsg` 的扇区参数：实际水平 FOV 与建议的 `n_bins` / `angle_step_rad` 取多少？契约里的 ±45°/0.5°/181 是**示例值**，请按实测覆盖给 |
| **Q-3** | `range_max_m` 按实测标定填多少？ |
| **Q-4** | 可通行区域投影到地面（G-3）的工作量与排期 |
| **Q-5** | 整机外参标定的时间点 —— 它是所有其他项的前置 |
| **Q-6** | `config/perception.yaml` 里的 `lidar:` 块（`/livox/lidar`）是否可以删除？据核实无任何订阅点 |
| **Q-7** | `z_pass_m` 的取值：M20S 站立高度 ＋ 载荷最高点 ＋ 余量 = ？（过顶滤除的安全前提，装云台后要复核） |
| **Q-8** | 分割 mask 的输出分辨率与相对深度帧的典型年龄（决定 PROF-5 腐蚀参数 `v_ego_max × Δt` 的量级） |
| **Q-9** | 深度 30 fps 下 `profile` 快线能否稳定 30 Hz 发布（TIME-2 的实现形态确认：快线不等慢线） |
| **Q-10**（预留） | re-ID 路线（§一B，本期不排期）：选 TAO ReIdentificationNet 还是 OSNet？嵌入抽取间隔与 gallery 上限？可承诺的 T 秒与重绑定成功率？ |

---

## 十、顺带一条：提交门禁

`config/` 下三个 yaml 有 **8 处全角标点与特殊符号**（`perception.yaml` 与 `perception.gemini338le_m2.yaml`
各 3 处中文逗号句号，`extrinsic.yaml` 2 处 `✓`）。本项目 CI 门禁 `scripts/lint/charset_lint.py`
要求非 `.md` 文件**标点一律 ASCII**（中文**文字**允许，禁的是标点 / 符号 / emoji）。
⚠️ 纳入版本控制后它会挡住所有人的提交，建议一并清理。

★ 另：`config/perception.yaml` 与 `config/perception.gemini338le_m2.yaml` **内容完全相同**（`diff` 为空）。
如果后者本意是 338Le 的差异化配置，那么它现在没起到作用。

---

上海哈船智能船舶技术有限公司 · XBRAIN_V6

**依据**：`11` §3.1B（接口契约 · 唯一真源）· `20` §3.1（RNS 消费侧语义与不变量）· `19` §0.0（perception 详细设计的失效告知）
