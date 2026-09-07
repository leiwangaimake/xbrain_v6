# Qt v2.0 协议待办清单

**XBRAIN_V6 首轮终测 · 2026-09-04**

> 本文件是**发给甲方的原文留档**。修改前请注意：已发出的版本以此为准，
> 后续变更应新建带日期的文件，不要就地改写历史交付物。
> 内部视角的完整记录见 [cloud-debug](cloud-debug.md)。

2026-09-04 完成首轮全类型覆盖联调（T01–T06）后，仍需贵方确认或处理的事项。
每条均注明 v2.0 原文、我方当前行为，以及需要什么答复。

> 本清单**不含**受硬件缺失阻塞的事项（云深处底盘／RTK、GZH-2 三合一载荷）。

| 项目 | 内容 |
|---|---|
| 机器人 | `gj-001` |
| 协议版本 | Qt v2.0（贵方冻结版） |
| 覆盖类型 | T01–T06 全部 |
| 待办合计 | 需裁决 4 项 · 需修改 1 项 · 已确认 2 项 |

---

## 一、需贵方裁决（4 项）

v2.0 未定义或定义不足之处。我方已按最保守的读法实现并说明理由，请确认或另行指定。

### D-1　任务取消时的 `result_code`

**v2.0 原文**（§3.3 终态字段表）

> `result_code` | integer | 是 | 成功 `0`；失败见 §10

而 §10 的十四个错误码全部表示拒绝或故障，**没有一个表示"操作员取消"**。

**我方现状**

`state = cancelled` 时报 `result_code: 0`，取消的语义由 `state` 与必填的 `reason` 承载。

此前恒报 `2001`（＝机器人未就绪／授时未同步）。那是一个兜底常量，与真实原因无关——终测中两条取消的终态全部报了它。已于 2026-09-04 改正。

**需要答复**

> 确认 `0` 可被贵方 Qt 正确解释；或指定一个专用码。

---

### D-2　能力已开放、子通路未建时用什么错误码

**场景**

`SET_ALARM_CONFIG` 在 §3 中是 T04「本期开放」。但一条只修改 `rules[]` 与声光标量（`regions` 为空数组）的配置，其落点通路在我方尚未建成——带 `regions[]` 的区域几何增量是可用的。

**我方现状**

```json
{
  "result": "rejected",
  "error_code": 1006,
  "detail": { "code": "E_NOT_IMPLEMENTED" },
  "reason": "regions[] is empty; this phase implements only the incremental region geometry path. Changing rules[] or the siren/light scalars needs the cmd/config channel, which is not built yet"
}
```

**问题**

§10 的 `1006` 字面是「不支持的 task_type/key」，典型 `detail.code` 为 `E_TASK_UNSUPPORTED` / `E_CHANNEL_DENIED`。贵方可能据此判定**整个 `SET_ALARM_CONFIG` 不受支持**，从而放弃这条能力的联调。

**需要答复**

> §10 是否新增一档「能力已开放、子通路未建」；或确认沿用 `1006` 且接受 `E_NOT_IMPLEMENTED` 作为 `detail.code`。

---

### D-3　急停回执 `hes` 的闭集

**v2.0 原文**

§2.3 的 `cmd/estop/ack` 样例给出 `"hes": "ok"`，但**未给出闭集**。

**我方现状**

发 `"unknown"`。依据内部契约 `11` §538 / §756：

> 硬件急停 HES 完全不经软件，软件不可解除。

其状态须由 `chassis_relay` 上报，该进程尚未编译，我方结构上读不到。报 `"ok"` 等于断言一个我们看不见的硬件信号。

**需要答复**

> `hes` 的完整闭集；以及 `"unknown"` 是否为贵方 Qt 可接受的取值。

---

### D-4　`state/audio.speaker_holder_type` 的闭集

**v2.0 原文**

§4.4 样例给出 `"speaker_holder_type": "cloud"`，同样**未给出闭集**。

**我方现状**

发 `null`。我方域②的持有者标识是 `broadcast_b` / `alarm_d` / `tts_cloud` / `tts_wecom` / `tts_local`；在网关自建一张到 cloud/local/wecom 的映射表属于臆测——猜错会让贵方 Qt 按错误值切换界面，比报 `null` 更坏。

**需要答复**

> 该字段的闭集取值，以及与上述五个持有者的对应关系。

---

## 二、需贵方修改（1 项）

### F-1　`base_rev` 的位置

**v2.0 原文**（§3.4 区域增量语义 op 表）

| op | 必带字段 |
|---|---|
| `upsert` | `id` / `base_rev` / `name` / `type` / `enabled` / `applies_to` / `vertices` |
| `delete` | `id` / `base_rev` |
| `set_state` | `id` / `base_rev` / `enabled` |

即 `base_rev` 属于**每一个 `regions[]` 条目**。

**实测**

2026-09-04 收到的 `SET_ALARM_CONFIG` 把 `base_rev` 放在 `payload` 顶层，值为 `null`。

**影响**

本次因 `regions` 为空数组未引爆。一旦带非空 `regions[]`，每个区域都会取到 `base_rev = null` 并触发 rev 冲突——现象是**「区域更新莫名失败」**，而报文其余部分完全合规，很难定位。

**需要处理**

> 将 `base_rev` 移入每个 `regions[]` 条目：新建填 `0`，更新／删除填 manifest 中该区域的当前 `rev`。

---

## 三、已确认，无需改动（2 项）

后端按 v2.0 严格校验的必填字段。**贵方现有报文均已包含**，此处仅作留档，供简化用例时参考。

### C-1　`waypoints[].name` 为必填

**v2.0 原文**（§2.1 GOTO_KEYPOINT 字段表）

> `waypoints[].name` | string | **是** | 显示名，不用于身份匹配

**后端行为**

缺失时回 `rejected` + `error_code: 1002` + `detail.field: "name"`。虽标注「不用于身份匹配」，但仍是必填项，后端不作放宽。

### C-2　`alarm_window.start/end` 为必填

**v2.0 原文**（§3.4 SET_ALARM_CONFIG 字段表）

> `alarm_window.start/end` | string | **是** | 每日 `HH:mm`；允许跨午夜

**后端行为**

缺失时回 `rejected` + `error_code: 1002` + `detail.field: "alarm_window"`。

---

## 四、本轮已确认正常（供参考，无需再测）

**音频会话 ID**
贵方已按约定采用后端分配的 `stream_id`。实测一次 `pc_to_dog` 广播共 **2280 帧**，`chunk_seq` 1→2280 连续无缺口，首帧在受理后 273 ms 到达，判帧层零丢弃。

**急停回执**
`cmd/estop/ack` 的 `detail` 现已带齐 v2.0 §3.3 要求的七项（`result` / `estop_epoch` / `applied` / `recv_mono_ms` / `latency_ms` / `hes` / `timeout_lock`）。`latency_ms` 实测 **6 ms**（判据为 ≤100 ms），贵方可据 `recv_mono_ms` / `latency_ms` 自行验证时延。

> 注：`applied` 当前恒为空数组。它要求列出**实际生效**的措施，而确认通道（`chassis_relay` 转发的底盘回执）尚未编译，我方在回执时限内拿不到任何"已生效"确认。空数组配上非空的 `latency_ms`，表示"命令收到并转发了，但没有任何一项被确认生效"。

**禁用与旧名任务**
`MANUAL_VELOCITY` 及 `INSPECTION_ROUTE` / `RETURN_HOME` / `PAUSE_TASK` 等旧名称均如实拒绝，回 `1006` 与人类可读原因，不作静默映射。

---

上海哈船智能船舶技术有限公司 · XBRAIN_V6

依据：`json格式文件_qt端v2.0.md`、`任务枚举_qt端v2.0.md`（贵方冻结版）
