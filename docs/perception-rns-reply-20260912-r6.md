# RNS → Perception：r5 交付边界集中确认 · 答复（第六轮）

日期：2026-09-12
回应：《Perception_RNS_r5_交付边界集中确认_20260912.md》（贵方审查基线 r5 包 `4692ced9…`，归档提交 `8d17099`）。只回复本函；未外发的《r5 退化帧盲区字段确认》不另处理。

---

## 〇、结论先行

| # | 结论 |
|---|---|
| ★★★ 1 | **四行全部"接受建议"**，并在同一批闭合三层：`11` v2.3 定字段可空条件与伪造判据；`19` v1.7 定生产规则；DTO / 解析器 / 消费者同步落码，随包 W-11 三个新场景 ＋ 每行"正常 / 合法未知 / 非法伪造"三组断言。 |
| ★★ 2 | 总则不变：缺值一律 `null`，🚫 哨兵。本轮只把四个**此前解析器仍强制数值**的字段改为**有条件可空** —— 条件写死在契约里，接收端把"条件不满足的 null"当伪造整条拒收，把"条件满足的 null"当合法未知消费。 |
| ★★ 3 | 合法未知 ≠ 有效数据。RNS 侧的降级按既有职责闭合：速度未知 ⇒ 按动态处置（🚫 当静止）；深度质量未知 ⇒ `RNS-I-2` 限速；检出但不可定位 ⇒ 本拍限速 ＋ 审计（🚫 当空场景）；盲区未知 ⇒ 不当已观测范围。**🚫 新增运动策略议题、🚫 停车规则。** |
| ★ 4 | 我方承认一处既有消费漏洞随本轮修正：RNS 此前**不读 `velocity_valid`**，`[0,0]` ＋ `velocity_valid=false` 会在驻留后进静态堆 —— 正是贵方行 2 担心的"当静止"。现已改：`velocity_valid=false`（无论 null 或数值）⇒ 运动状态未知。 |
| ★ 5 | 同类遗漏一并指出（§三）：`latency_ms_p50/p99` · `infer_gap_ms_p99/max` 无样本时可空（已同批定）；`depth_quality` 出现在 `11` §3.1B.2 用途表却不在样例 / DTO / 解析器 —— 本轮不改语义，登记待下一轮定去留。 |

---

## 一、逐行裁定

| 行 | 裁定 | 合法 `null` 的条件（生产侧 `19` v1.7） | 伪造判据（接收端整条拒收） | RNS 消费（`20` v1.43） |
|---|---|---|---|---|
| **1 Profile 无可验证地面** | ✅ 接受建议 | `blind_near_m = null` ⇔ 本帧无任何有效地面样本（无依据撤回帧 / 成片 invalid）⇒ 全 bin `d_free = null`、bit0 全 0；🚫 0.59 / 配置下限 / 上一帧值 | null 与任一 `d_free` 非 null 同现；null 与任一 bit0 同现 | RNS 不消费 `blind_near_m`（它只进可见性 / 审计）；未知盲区不当已观测范围；独立 S 阻挡（`0b0100`）照常 BLOCKED |
| **2 Objects 可定位、速度不可估** | ✅ 接受建议（取 `null`，🚫 `[0,0]` 占位） | `velocity_xy = null` ⇔ `velocity_valid = false` ∧ `velocity_status ∈ {warming_up, timestamp_gap}`；有 raw 估计但不可用 ⇒ 数值 ＋ `velocity_valid = false`（贵方"不等同于没有估计"的区分保留在线上） | null ∧ `velocity_valid = true`；null 配 `moving / static / motion_unconfirmed` | `velocity_valid = false` ⇒ 运动状态未知 ⇒ 🚫 静止判据、🚫 静态堆，按动态处置（走廊内 §5.3 停 / 减速）—— 与 `raw` 拒用同款 |
| **3 Status 无深度失效比例观测** | ✅ 接受建议；**🚫 沿用旧值** | `invalid_pixel_ratio = null` ⇔ 心跳窗（1 s）内无已处理深度帧；心跳照发。它是逐窗统计量，没有"时效"可言，故不设沿用规则；🚫 0（像正常）、🚫 1（像统计了全 invalid）；实收全 invalid 帧 ⇒ `1.0` 真值 | 非数值 / 越界 | null ⇒ 深度质量未知 ⇒ `RNS-I-2` 强制限速（与超阈值同处置）；🚫 因心跳新鲜就当有效观测 |
| **4 Objects 检出但不可定位** | ✅ 取贵方第一种方式：**保留目标，四几何字段同时 null** | `footprint_xy / z_min / z_max / r_near` 同时 `null`、`velocity_xy = null`、`velocity_valid = false`，class / confidence / status 照发 ⇒ 当帧列表（含可定位与不可定位）仍是完整新结果；🚫 旧位置 / 失效平面图像框投影 / 假定尺寸补造距离 | 部分字段 null；不可定位却带 `velocity_xy` | 不进几何 / 走廊 / 静态堆 / 停等（没有距离可等）；非 `ignore` 类 ⇒ 本拍限速 `rns.perception.unlocalized_speed_cap_mps`（与 `obstacle_avoid` 档同值）＋ 数量变化审计 `objects_unlocalized`；随包断言证明它**不会被静默当作空场景** |

★ 为什么行 4 不取"只发可定位目标 ＋ 计数"：计数丢掉 class，RNS 无法区分"不可定位的人"与"不可定位的风筝"；保留目标则 §5.1.1 类别分流照常工作，且"完整新结果"的定义不需要为过滤后的列表另立口径。

---

## 二、闭合产物（同批）

| 层 | 落点 |
|---|---|
| 契约 | `11` v2.3：§3.1B.1 `blind_near_m` 行 · §3.1B.2 `velocity_xy` 行 ＋ 新行"检出但不可定位" · §3.1B.3 样例注 ＋ "可空统计"行 · §3.1B.5 新表"合法缺值的消费"；文首"本轮合入 v2.3" |
| 生产规则 | `19` v1.7：§3.2 `blind_lo` 行 · §4.1 两行 · §5.1 两行 · §14 A19-NULL-1～4 |
| 消费规则 | `20` v1.43：RNS-I-2 加 null · §5.5 速度未知 · 新 §5.1A 不可定位目标 |
| DTO | `inputs.py`：`blind_near_m` / `invalid_pixel_ratio` / 四几何字段 / `velocity_xy` 可空，`TrackedObject.localized` |
| 解析器 | `three_keys.py`：四条条件与伪造拒收（表 §一第四列逐条落码） |
| 消费者 | `classify.health_speed_capped(None) == True`；`source.py` 速度有效位 ＋ 不可定位分支（限速 ＋ 边沿审计）；`config.py` 启动必填 `unlocalized_speed_cap_mps`；`rns.yaml` 新键 |
| 样例与断言 | W-11 新场景 `velocity_unknown` / `status_unknown` / `unlocalized_person`，`ground_withdrawn` 改为 `blind_near_m = null`；`tests/perception/test_consumer_contract.py` 每行三组（正常 = 既有 `normal`；合法未知 = 新场景消费断言；伪造 = 解析器拒收断言）；`tests/p1_motion/perception_src/test_three_keys.py` 解析级；`tests/p1_motion/rns/test_null_values.py` 源级 |

---

## 三、同类遗漏（本轮指出）

| 项 | 处置 |
|---|---|
| `latency_ms_p50/p99` · `infer_gap_ms_p99/max` 无样本 | ✅ 同批定为可空（`11` v2.3 行 Ⅲ；`19` §5.1）；有历史发布点时开放间隙照算 |
| `h_block` 有阻挡但高度不可估 | 既有：`h_block = null` 已合法（解析器 `_num_or_null_array`），无需改 |
| `depth_quality` | ⚠️ 出现在 `11` §3.1B.2 用途表（"远处深度不可信时降权"），但样例 / DTO / 解析器均无此字段，RNS 也不消费。本轮不动语义；登记 NEXT，下一轮定"进 schema 还是删表行" |
| `conf[i]` | 既有 uint8，0 即无置信度，🚫 需要 null |

---

## 四、贵方 §二 / §三 已明确事项

照单接受，不重复：Q4 本期范围（依据链是解除前提非验收前提）· 发布与时效口径 · 分辨率与资源 · 其他缺值路径 · 实机输入既定分工（第三轮 §Q5）。★ 一处补充：旧配置模板 / PSC 总则按 r5 收窄整理时，`19` §8.2 的回退专用键（`fallback_*`）为 null **只关闭回退分支**，🚫 整进程拒启 —— 与贵方口径一致，已在 `19` v1.6 §8.2 注明。

---

## 附录 A · 本轮同批变更

| 真源 / 代码 | 变更 |
|---|---|
| `11` v2.3 | 见 §二 |
| `19` v1.7 | 见 §二 |
| `20` v1.43 | 见 §二 |
| 代码 | `xbrain/p1_motion/rns/inputs.py` · `perception_src/three_keys.py` · `rns/classify.py` · `rns/source.py` · `rns/config.py` · `configs/rns.yaml` · `scripts/dev/perception_sim.py` · 测试三处 · 样例集重生成 |
| 离线包 | `perception_pack_20260912r6.tar.gz`（`SHA256SUMS` ＋ `COMMIT.txt`，由 tag 提交树 `git archive` 生成） |

## 附录 B · 真源提交号与 SHA

| 项 | 内容 |
|---|---|
| 真源提交 | **`3839665`**（11 v2.3 ＋ 19 v1.7 ＋ 20 v1.43 ＋ 三层代码 ＋ W-11 三场景同批），tag **`perception-r6-20260912`**（指向其后继的附录回填提交） |
| `docs/11-接口契约.md` | `9ae5d1d7a19902364fb1bab56de79b72e40c494e7013a5483f65a465ab2710b4` |
| `docs/19-perception详细设计.md` | `fc640e7976327c88d1aaf309857ebf64405534f92965d7594a41f5f23dfaab1a` |
| `docs/20-RNS反应式导航软件系统详细设计.md` | `6d830b37b75450d02b5680e57d0392b95ff16cf93ef337dbc28a202097c05090` |
| `xbrain/p1_motion/rns/inputs.py` ＋ `xbrain/p1_motion/perception_src/three_keys.py` | `bbefac6929ea657e91e8683ca3852220bd767a1d52958ea59337b071cf5a69bd` ＋ `7241d8a975b56156289ae541d2a4e13b52715b89ea43c85b2eba662e4b740a20` |
| 其余文件 | 见包内 `SHA256SUMS` |
