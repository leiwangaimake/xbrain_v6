# Perception → RNS 第三轮技术口径确认的答复（第四轮）

**XBRAIN_V6 · 2026-09-11**

| 项 | 内容 |
|---|---|
| 回应对象 | 贵方《Perception → RNS：第三轮技术口径确认》（2026-09-11，依据离线包 `80d31299…` / `COMMIT.txt 05173e9`） |
| 本文性质 | ① 承认我方四处实错并当场修正 ② Q1～Q4 逐项定案，每项给合法字段值 ＋ RNS 消费动作 ＋ 落点 ③ §三勘误采纳 ④ 同批同步四册 · 消费端 DTO/解析器 · 样例集 |
| 真源 | 本文裁决落 **`11` v2.2 · `19` v1.5 · `20` v1.40**（`12` 不变），代码同批（`ObjectsMsg.t_publish_mono_ms` · `semantic_only` 第八场景）；tag **`perception-r4-20260911`**，提交号与 SHA 见附录 |
| 一句话 | 四项都能用现有机制定案，🚫 新立计时系统、🚫 新增 abort 值、🚫 改分级门 |

---

## 〇、先认错：贵方抓到的四处实错

| # | 我方错处 | 修正 |
|---|---|---|
| 1 | 答复 Q2.1 要接收端按 `objects.t_publish_mono_ms` 算 `Δpub`，而 `11` §3.1B.2 样例与 `inputs.py` 的 `ObjectsMsg` **都没有该字段** | `11` §3.1B.2 v2.2 补字段（样例 ＋ 字段表行），DTO / 解析器 / SIL / 样例集同批补齐；**schema 仍为 `perception_objects_v1`**（实现前修订不进位 —— v1 至今无生产者；自 tag `perception-r4-20260911` 起 v1 的定义含本字段） |
| 2 | `11` §3.1B.5 例子「年龄 400 ms 的 objects 同时被记为间隔超限」**不成立** —— 流水线可每 40 ms 发一条年龄 400 ms 的新结果 | 改写：年龄与间隔**独立判断**，各自有门（Q1 表） |
| 3 | 答复 Q3.2 称「当前无逐进程配额」—— 错。`10` §3.2 早有 **CPU 亲和表**（Orin NX 8 核），`perception` = 核 2–3，且 CI `scripts/ci/check_affinity.py` 逐单元比对该表 | Q2 定案：以 `10` §3.2 为真源，unit 已一致 |
| 4 | `20` §3.1.8 的 200 ms 推导把 `v·t_lat` 整段（0.4 s）与 `v_close·age` 叠加，而 `12` 定义 `t_latency 0.4 = 检测 0.2 ＋ 底盘链路 0.2` —— 检测段被计了两次 | §三勘误采纳：年龄项**取代**检测段，公式改 `v²/2a ＋ v·t_link(0.2) ＋ v_close·age ＋ margin`（数值见 §三） |

---

## Q1．时间字段、心跳计龄、两端账本

★ 贵方建议表**全部采纳**，逐行定案如下（落 `11` §3.1B.2 / §3.1B.3 / §3.1B.5 v2.2）：

| 对象 | 定案 |
|---|---|
| ★★ `objects` 字段 | 正式 schema · 序列化 · DTO · 接收审计**均含 `t_publish_mono_ms`**（`put` 调用前一刻的单调钟，与 profile 同义）；`t_capture_mono_ms` 仍是推理源帧身份。schema 版本 **`perception_objects_v1`**（理由见 〇-1） |
| ★★ 发布 / 接收账本 | 发布端记：源帧身份 `t_capture` · `put` 调用前 `t_publish` · `put` 返回时刻 · 接口返回状态；接收端记：同一身份 · `t_rx`。**发布尝试 · API 返回 · 实际接收三者分开对账**；接收端缺帧 🚫 自动等同发布端断供（RT 面 Q1 档 `congestion=drop` 允许在路上丢，`11` §2.4.2）；🚫 新增零丢包硬门。我方 `scripts/dev/perception_rx_audit.py` 即接收端账本 |
| ★★ `status` 与 T-53 | 按 `status` 自身 `t_publish_mono_ms` 去重 / 判乱序；每拍 `age_status = now_tick − 最近已接受心跳的 t_publish`；profile / objects 仍按各自 `t_capture` 计龄，**心跳更新 🚫 刷新感知数据年龄**。★ RNS 代码现状即如此（`source._accept` 对 status 用 `t_publish` 做身份） |
| ★★ 时间指标命名 | `Δpub`（发布间隔，发布端 `t_publish` 差）· `age_rx = t_rx − t_capture`（到达时年龄）· `age_tick = now_tick − t_capture`（控制拍当前年龄）**三者分别记录，🚫 互相替代**。T-5x 与三档用 `age_tick`；接收端账本记 `age_rx`；分级门用 `Δpub` |
| ★ 年龄 vs 间隔 | 独立判断：每 40 ms 发一条年龄 400 ms 的新结果 ⇒ **间隔达标、年龄进三档②**；断供 400 ms 无新帧 ⇒ **间隔超限、年龄同增**。🚫 从一个推另一个（〇-2 改写） |

★ 样例勘误采纳：`11` §3.1B.3 status 样例改为 `infer_gap_ms_p99 = 47.0` · `infer_gap_ms_max = 62.0`。

**一致的样例与 DTO（v2.2）**：`11` §3.1B.2 json5 增一行 `"t_publish_mono_ms": 128374690`；`xbrain/p1_motion/rns/inputs.py::ObjectsMsg` 增字段 `t_publish_mono_ms: int`；解析器 `perception_src/three_keys.py::parse_objects` 将其列为必填（缺 ⇒ 整条拒收并计数）；七场景样例全部重生成。

---

## Q2．生产性能验收固定在 CPU 核 2–3

| 项 | 定案 |
|---|---|
| ★★★ 真源 | **`10` §3.2「CPU 与内存分配（Orin NX 16GB / 8 核）」亲和表**：核 0 OS · 核 1 `zenohd-gen`/`bridge`/P5 · **核 2–3 `perception`** · 核 4 P1 独占（`isolcpus`）· 核 5 `quadruped` 独占 · 核 6 `zenohd-rt`/`chassis_relay`/`rtk_driver`/`teleop_input` · 核 7 P2/P3/P4/`behavior_proxy`/Nav2/**AI Runtime（`llama-server` ＋ ASR）**/payload。`deploy/systemd/xbrain-perception.service` 的 `CPUAffinity=2-3` 就是它的落地，CI `scripts/ci/check_affinity.py` 逐单元比对 —— **改 unit 不改表会被 CI 拦** |
| ★★ 验收基线 | **正式对齐与生产验收以核 2–3 为基线**（两核 A78AE @ 1.98 GHz）。亲和只约束**主机侧 CPU 线程**（含默认继承的工作线程）；DLA ×2 · GPU · NVENC/NVDEC · PVA 不在亲和之下。★ 贵方「不擅自解除、不以更宽 CPU 条件冒充达标」—— 一致 |
| ★ 若两核不够 | 那是 **`10` §3.2 的变更请求**（册主我方），需附第 ③ 阶段共载工况下的 `tegrastats` 逐核曲线与快线/慢线线程占用；候选只有从核 7 的 Python 共核里挤（那里同时住着 `llama-server`，是最重的共租户），或与 Nav2 换核 —— 🚫 动 4/5/6（实时线程独占是 v0.2 以来的结论） |
| ★ 我方订正 | 答复 Q3.2「无逐进程配额」一句撤回（〇-3）；`10` §3.2 另注：**内存**仍无逐进程配额，`OOMScoreAdjust=-900` 是唯一内存侧保护 |

---

## Q3．本 bin 无有效深度、但有有效语义阻挡

★★★ **定案：允许贵方给出的表达，S 只给阻挡、🚫 给 FREE**（落 `11` §3.1B.1 v2.2 推论表与样例表；`19` §3.4A v1.5 注入伪代码）：

```text
d_free  = null      # 无几何样本 -> 无已验证 FREE (RNS-I-1: 前方一律 UNKNOWN)
d_block = 2.5       # 语义 footprint (满足时效 / 变换 / 膨胀条件) 注入的最近阻挡
h_block = null      # 语义不给高度 (h_block 是可见表面几何量, 11 v1.9)
src     = 0b0100    # S = 1, G = 0, T = 0
```

| 项 | 定案 |
|---|---|
| ★ 推论表改写 | 原「bit1 = 0 ⇒ `src = 0` 且全 null」改为：**bit1 = 0 ⇒ `d_free = null`（🚫 FREE）；`d_block` 只能来自 S（bit2）注入 —— 无注入 ⇒ `null`；`h_block = null`**。★ bit3（负障碍）依赖穿地证据，无样本时不可能置位 |
| ★ bit2 定义 | `d_block` 由语义注入**产生或收紧**：`d_sem < d_geom`，**`d_geom` 缺席按 ＋∞** —— 无深度 bin 的注入即「产生」 |
| ★ 生产侧 | `19` §3.4A 注入循环对 G = 0 的 bin 同样执行；`d_free` 为 null 时**不做截断**（无 FREE 可截）；bit1 保持 0 |
| ★★ RNS 消费 | `grid.fuse_bin`：`d_block ≤ r ⇒ BLOCKED`（任一通道，并集）；`d_free = null ⇒ 其前一律 UNKNOWN`（🚫 推 FREE）⇒ 接近时 UNKNOWN 占比压速（`20` §8.1A），到 2.5 m 处为墙；行为分流走 `ObjectsMsg`（该目标的 class）。★ 代码现状已如此（无需改 RNS） |
| ★ 玻璃门样例 | 保留（门框有立体点，G = 1，`0b0110`）；**本场景另立一行**（G = 0，`0b0100`），🚫 互相代答 |
| ★ 样例 | `11` §3.1B.1 样例表增行；样例集增第八场景 **`semantic_only`**（中央 bin 全 null ＋ 2.5 m 语义阻挡），消费对表断言：栅格 1.5 m 处 🚫 FREE、2.5 m 处 BLOCKED |

---

## Q4．地面拟合失败时先验平面支持 FREE 的条件

认同贵方：**「封顶 ＋ reason」不够** —— 封顶只限制误差随距离增长，先验平面本身的误差界必须可测。定案（落 `19` §3.2A v1.5、§8.2 新键、§14 A19-FIT-2、§17 PD-18；`11` §3.1B.3 新 reason）：

### Q4.1 先验平面继续支持 FREE 的有效条件（两条同时成立）

| # | 判定要素 | 失效方向 |
|---|---|---|
| ① 失败原因 | 本帧拟合失败的原因是**内点不足**（`inlier_frac < plane_fit_min_inlier_frac`：地面被遮挡 / 成片 invalid），🚫 是**残差超限**（`resid > plane_fit_resid_max_m`：地面非平面 / 坡变 —— 先验平面此时必错） | 残差超限 ⇒ 条件不成立 |
| ② 姿态在标定包络内 | IMU 重力向量相对**标定时姿态**的偏差 `‖Δtilt‖ ≤ fallback_tilt_tol_deg`（来源：相机 IMU 或底盘 IMU，PD-18 定源；无 IMU ⇒ 条件视为不成立） | 无 IMU / 超差 ⇒ 条件不成立 |
| 配对约束 | `dfree_cap_fallback_m × tan(fallback_tilt_tol_deg) ≤ h_tol_m`（启动断言）—— 封顶距离处的平面高度误差 🚫 超过地面判定容差；例 2.0 m × tan 2° ≈ 7 cm ≤ 8 cm | 违反 ⇒ 拒启 |

★ 具体阈值由贵方结合安装与实测定（`null` 拒启纪律不变）；上表定的是**要素与方向**。

### Q4.2 条件未知或不满足：只撤回依赖平面的 FREE，保留独立阻挡，照常发布

| 项 | 定案 |
|---|---|
| ★★ FREE | **全 bin `d_free = null`**（本帧无法验证，🚫 写 `blind_near`）；`t_seg` 照发但 bit0 = 0（无 FREE 可背书） |
| ★ 独立阻挡 | 只保留**不依赖平面精度**的立体点：高度阈提到 `h_tol_eff(r) = h_tol_m ＋ r·tan(fallback_tilt_err_max_deg)`（条件②不可测时取该上界，建议 5°）；负障碍 🚫 判（穿地判据依赖平面） |
| ★ 可见性 | `status.degraded_reasons` 同时含 `ground_fit_fallback` ＋ **`ground_free_withdrawn`**（v2.2 新值）；照常 30 fps 发布，🚫 停发 |
| ★ RNS 动作 | 全 bin `d_free = null` ⇒ 前向 UNKNOWN 占比 1 ⇒ `unk_g_min × v_nom` 压速（`20` §8.1A）＋ 记忆栅格只用旧 FREE；`d_block` 照常成墙；无新 RNS 逻辑 |

### Q4.3 例：拟合失败，1.5 m 处 0.12 m 路沿（同一 bin）

| 情形 | `d_free` | `d_block` | `h_block` | `src` | RNS |
|---|---|---|---|---|---|
| 条件 ①② 满足 | 1.25（封顶 2.0 内，格远端判停） | 1.5 | 0.12 | `0b0011`（有 mask 时） | 路沿成墙，正常绕 / 停 |
| 条件不满足（tilt 上界 5°） | `null` | `null`（`h_tol_eff(1.5) = 0.08 ＋ 1.5·tan 5° ≈ 0.21 > 0.12` ⇒ 路沿不可判） | `null` | `0b0010`（有样本、不可判）| 前向 UNKNOWN ⇒ 压速；路沿由记忆栅格（若先前见过）或减速后的新帧接管 |

★ 失效方向：条件不满足时**宁可看不见路沿也不给假 FREE** —— 压速让后续帧（姿态恢复 / 拟合成功）有时间接管；🚫 反过来。

---

## 三、勘误采纳：年龄预算的时间段（`20` §3.1.8 v1.40 改写）

`12` §... `t_latency = 0.4 s = 我方检测 0.2 ＋ 底盘链路上界 0.2`（U54）。年龄项**取代**检测段，🚫 双计：

```text
stop_dist_m >= v^2 / (2a) + v * t_link + v_close * age_P99 + margin
             t_link = 0.2 s (t_lat 的底盘链路段);  age 取代 t_lat 的检测段
```

| age_P99 | 需要（a = 1.5 m/s² · v = 2.0 · v_close = 3.5） | 对比 `stop_dist_m` 3.0 |
|---|---|---|
| 200 ms | 1.33 ＋ 0.40 ＋ 0.70 = **2.43 m** | 余量 0.57 m |
| 250 ms | 1.33 ＋ 0.40 ＋ 0.875 = 2.61 m | 余量 0.39 m |

★ 门值不变（200 ms 按文档执行，贵方已接受）；原文「250 ms 无余量」的结论撤回（那是双计的产物）。

---

## 附录 A · 本轮同批变更

| 真源 / 代码 | 变更 |
|---|---|
| `11` v2.2 | §3.1B.2 `objects` 增 `t_publish_mono_ms`（样例 ＋ 字段表）· §3.1B.3 样例数字勘误 ＋ 两端账本行 ＋ `ground_free_withdrawn` · §3.1B.5 接受规则改按**身份时戳**（profile/objects `t_capture`，status `t_publish`）＋ 年龄/间隔独立 ＋ 三个时间指标命名 · §3.1B.1 推论表 bit1 = 0 改写 ＋ 无深度语义注入样例行 ＋ bit2 定义补「`d_geom` 缺席按 ＋∞」＋ `d_free = null` 含「本帧无法验证」 |
| `19` v1.5 | §3.2A 回退有效条件 ①② ＋ 配对断言 ＋ 不满足时的撤回规则 ＋ 路沿例 · §3.4A G = 0 bin 注入 · §8.2 新键 `plane_fit_min_inlier_frac` / `plane_fit_resid_max_m` / `fallback_tilt_tol_deg` / `fallback_tilt_err_max_deg` · §14 A19-FIT-2 / A19-SEM-2 · §17 PD-18（IMU 重力向量来源） |
| `20` v1.40 | §3.1.8 推导改 `t_link` 口径 · §3.1.6 S-only 边界消费注 |
| 代码 | `inputs.ObjectsMsg.t_publish_mono_ms` · `three_keys.parse_objects` 必填 · SIL / 场景助手 / 样例生成器同步 · 第八场景 `semantic_only` ＋ 消费对表断言 · 样例集重生成 |
| 离线包 | `perception_pack_20260911r4.tar.gz`（`SHA256SUMS` ＋ `COMMIT.txt`） |

## 附录 B · 真源提交号与 SHA

| 项 | 内容 |
|---|---|
| 真源提交 | **〈回填〉**（四册 ＋ 代码同批），tag **`perception-r4-20260911`** |
| `docs/11-接口契约.md` | 〈回填〉 |
| `docs/19-perception详细设计.md` | 〈回填〉 |
| `docs/20-RNS反应式导航软件系统详细设计.md` | 〈回填〉 |
| `xbrain/p1_motion/rns/inputs.py` ＋ `xbrain/p1_motion/perception_src/three_keys.py` | 〈回填〉 ＋ 〈回填〉 |
| 其余文件 | 见包内 `SHA256SUMS` |

---

**本文四处修正全部随同批提交落真源，本文自身不是真源；无排期项，无待批项。** 有异议请继续按本轮格式对账。

上海哈船智能船舶技术有限公司 · XBRAIN_V6

**依据**：`11` v2.1 §3.1B · `19` v1.4 · `20` v1.39 · `10` §3.2 · `12` `t_latency` 定义（U54）· 贵方《第三轮技术口径确认》（2026-09-11）
