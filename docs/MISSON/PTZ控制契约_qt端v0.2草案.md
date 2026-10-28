# Qt 前端 PTZ 控制契约 —— 云台转动 · 变焦 · 重新对焦（XBRAIN V6，草案 v0.2）

| 项 | 内容 |
|---|---|
| 状态 | ★ **草案 v0.2（2026-10-28）**，待甲方 Qt 侧确认后冻结。冻结前双方都 🚫 按本稿上线 |
| 范围 | 布控球的 **转动（8 个方向）· 变焦（可见光）· 重新对焦（可见光）· 停止**，即「键盘控制云台」所需的全部动作 |
| 上位文档 | 《任务枚举_qt端v2.0》《json格式文件_qt端v2.0》：外层信封、ID、时间、错误码全部沿用，本文只写**新增**部分 |
| 机内落点 | `11` §7.4 `PtzCommand`（`jog` / `nudge` / `stop` / `refocus`）· §7.4.5 dead-man（PTZ-D1 ~ D6）· §7.4.7 对焦（F1 ~ F4）· §7A 域⑤ 仲裁 |
| 参考实现 | `tests/ptz/ptzkey2.py`（键盘控制：方向键按住转动、`+/-` 变焦、`f` 重新对焦、空格停止）。★ 它直连相机，只用于机上调试，见前提 4 |
| 裁决依据 | `99` **U98**（2026-10-28 用户拍板：开放 8 方向转动 · `refocus` 进 `11` §7.4 · 本稿由 v0.1 升级） |

**版本记录**

| 版本 | 日期 | 变化 |
|---|---|---|
| v0.1 | 2026-10-26 | 只定义可见光变焦：`zoom` / `zoom_step` / `stop` |
| ★ **v0.2** | 2026-10-28 | ★ 新增 `move`（8 方向转动，§3.2）与 `refocus`（§3.5）；`stop` 的 `axes` 扩到 `pan` / `tilt` / `zoom`；新增前提 5 ~ 7；§4 时序推广到转动并附键盘映射；关闭 v0.1 的 U-4；新增 U-6。★ `zoom` / `zoom_step` 的字段与规则**一字未改**。★ 对已按 v0.1 写的 Qt 代码有两处可见变化：① `stop` 停在空任务上时 `detail.note` 由 `no_active_zoom` 改为 `no_active_motion`；② 原 §3.5「明确拒绝的写法」顺延为 §3.7，并新增角度量、`move` 带 `camera`、改相机配置三行 |

---

## 1. 先说清楚的七个前提（都来自真机实测，不是规格书推导）

1. **变焦与对焦只对可见光有效。** 热成像 `caps.Zoom=false`，视场固定。`zoom` / `zoom_step` / `refocus` **只作用于 `cam_ptz_vis`**；Qt 🚫 对红外画面提供变焦、对焦按钮。
2. **没有绝对变焦，也读不回当前倍率。** 设备刻度 `MaxZoom=3300` 到「×倍」的映射未标定（`11` T-PTZ-3），位置 / 倍率回读恒为假值（`11` PTZ-C2）。⇒ 本契约**只有「以某速度变焦」与「变焦一段时间」两种表达**，🚫 「变到 10 倍」这类目标量，收到即拒。
3. **转动与变焦都是「开始运动 → 直到停止」的连续动作。** 底层是 ONVIF `ContinuousMove`，发出后会**一直运动到收到 Stop 为止**。⇒ 链路一断，若无保护镜头会一路推到头、云台会一直转。本契约用 **dead-man（到期自停）** 兜底，见 §4。
4. **Qt 🚫 直连相机控制。** 相机的 ONVIF / LAPI / Web 端口**不对外映射**（`11` MED-C1、PTZ-C10），NAT 只开放 RTSP 媒体端口。控制**只能**走本契约的 Zenoh key，由机器人仲裁后下发。`ptzkey2.py` 那种直接调 ONVIF / LAPI 的做法只用于机上调试，🚫 用于云端。
5. ★ **转动没有角度：既不能「转到某个角度」，也读不回当前朝向。** 外置 PELCO-D 云台对绝对定位**返回成功但不动**（`11` PTZ-C1），朝向回读恒为假值 `(180, 0)`（PTZ-C2）。⇒ 转动**只有「朝某方向以某速度转」**一种表达；🚫 「转到正北」「向左转 30°」这类角度量，收到即拒。★ **水平轴没有机械限位**（0 ~ 360° 连续，`11` PTZ-C6）—— 不加保护时断链 = 云台**永远转下去**，所以 §4 的 dead-man 对转动**同样强制**。
6. ★ **一台球机、两路画面、同一个云台**（`11` U49）。转动时可见光与红外**一起转**；`move` 不区分相机，🚫 带 `camera` 字段。
7. ★ **摇摄时画面会糊，这是物理限制，不是故障。** 实测摇摄清晰度约为静止时的 **31%**，缺掉的部分是 1/100 s 快门的运动模糊（`11` §7.4.7「物理天花板」）。★★ **码率不可调**：系统已取消云台码率控制，上装任何进程都不写码率 / I 帧间隔（`11` §7.4.7 B0）—— `ptzkey2.py` 运行期临时提码率到 16384 的做法**不进本契约**，Qt 🚫 请求它。停下后相机会自动重新合焦，画面随之恢复；若停在距离差别很大的景物上仍虚，用 §3.5 `refocus`。

---

## 2. 新增 key

| Key | 方向 | QoS / 频率 | 用途 |
|---|---|---|---|
| `xbrain/{rid}/cmd/ptz` | Qt → 后端 | 可靠；按住期间续租 ≤ 4 Hz（§4） | 转动 / 变焦的开始与续租、单步变焦、重新对焦、停止 |
| `xbrain/{rid}/cmd/ptz/ack` | 后端 → Qt | 每条 `cmd/ptz` 一条：`move` / `zoom` / `stop` 收到后 **500 ms 内**；`zoom_step` 在**开始执行时**即回，🚫 等单步走完；`refocus` 在**执行完成后**回，**2 s 内**（v2.0 §1.4 普通 ack 时限） | 受理、拒绝、重复回执 |

- ★ 两条 key 加入后，v2.0 正式 key 全量表由 17 条变为 **19 条**；冻结时需同步写进《任务枚举》§2。
- ★ 云台的动作**不进 `cmd/task`**：`cmd/task` 是带 `state/task` 终态的任务生命周期，按住期间每秒几条的续租会挤占任务队列；而且续租要求**每条换新 `msg_id`**（§4 规则 R-3），与 `cmd/task` 「重发必须复用原 `msg_id`」的语义相反。与 `cmd/estop` 同理，单独成 key。
- ★ 不新增 `state/ptz`：朝向与倍率都读不回来（前提 2、5），发一个「当前朝向 / 倍率」等于发假值。是否在动、谁在控制，以 `cmd/ptz/ack` 与事件为准。★ 机内虽有「云台在动」事实位 `state/mode.ptz.moving`（`11` §7.4.7），但 v2.0 的 `state/mode`（§4.3）字段表里没有它，本期**不对 Qt 投影**。

---

## 3. JSON 格式

外层与 v2.0 §1.1 完全一致：`v=1`、`rid` 与 key 第二段一致、`ts` 为 float64 Unix 秒、`seq` 按完整 key 递增、`src="qt_hmi"`。

### 3.1 `data` 公共字段

| 字段 | 类型 | 必填 | 约束 |
|---|---|---:|---|
| `msg_id` | string | 是 | **每条消息一个新值**，续租也要新值（R-3）。格式按 v2.0 §1.2 统一 ID 正则 |
| `task_id` | string | 是 | **一次操作一个值**：从按下到松开（或一次单步 / 一次对焦）共用同一个 `task_id`，用于把开始 / 续租 / 停止关联起来。格式按 v2.0 §1.2：`task-<uuid>` |
| `task_type` | string | 是 | 固定 `PTZ_CONTROL` |
| `payload` | object | 是 | `action` 取值闭集：`move` \| `zoom` \| `zoom_step` \| `refocus` \| `stop`，各自字段见 §3.2 ~ §3.6 |

### 3.2 ★ `move` —— 按住转动（开始 / 续租）（v0.2 新增）

```json
{
  "v": 1, "rid": "gj-001", "ts": 1785733100.050, "seq": 52, "src": "qt_hmi",
  "data": {
    "msg_id": "msg-2c71e0b4",
    "task_id": "task-91d0a7f3",
    "task_type": "PTZ_CONTROL",
    "payload": {
      "action": "move",
      "direction": "up_left",
      "speed": 1.0,
      "hold_ms": 800
    }
  }
}
```

| payload 字段 | 类型 | 必填 | 约束 |
|---|---|---:|---|
| `action` | string | 是 | 固定 `move` |
| `direction` | string | 是 | 闭集 8 值：`up` \| `down` \| `left` \| `right` \| `up_left` \| `up_right` \| `down_left` \| `down_right`。`up`=抬头，`left`=从上往下看逆时针（画面向左）。斜向 = 水平与俯仰**同时**以 `speed` 转动，仍是一条命令（`11` PTZ-C5）。闭集外的值 ⇒ 拒绝（`1003`），🚫 降级成最接近的方向 |
| `speed` | number | 是 | `0.1 .. 1.0`，归一化转速；`ptzkey2.py` 固定用 `1.0`，Qt 建议默认 `1.0`。钳位与拒绝规则同 §3.3 `zoom.speed`；🚫 用负速度表示反方向（方向只由 `direction` 表达） |
| `hold_ms` | integer | 是 | dead-man 时长，`600 .. 2000` ms，建议 **800**。规则与 §3.3 `zoom.hold_ms` 完全相同 |

★ **续租、换向、被拒后的处理与 `zoom` 完全一致**：同一 `task_id`、同 `direction` 与 `speed` 的后续 `move` = 续租（只推后到期时刻，不向相机重发命令，`11` PTZ-D2）；同 `task_id` 内改 `direction` / `speed` ⇒ `rejected`（`1003`，`detail.reason="params_changed_within_task"`）并**停止**该 `task_id`；要换方向按 §4 R-5 先 `stop` 再用**新 `task_id`** 开始。
★ **方向键组合**：键盘同时按住两个方向键（如 `↑` ＋ `←`）时，Qt 发 `up_left`；从单键变成双键、或双键松开一个，都按 R-5 换向处理（`stop` ＋ 新 `task_id`）。相反方向同时按住（`←` ＋ `→`）视为无效组合，Qt 发 `stop`。
★ **俯仰没有软限位**：`00` PAY-11b 的俯仰软限位（−90° ~ +30°）以角度为依据，而角度读不回来（前提 5），**后端无法在转动中执行它**。俯仰到机械限位后继续下发是否堵转待实测（`11` T-PTZ-4）。⇒ 操作员看到画面不再变化就应松手；dead-man 保证松手或断链后在 `hold_ms` 内停下。见 §8 U-6。

### 3.3 `zoom` —— 按住变焦（开始 / 续租）

```json
{
  "v": 1, "rid": "gj-001", "ts": 1785733000.120, "seq": 41, "src": "qt_hmi",
  "data": {
    "msg_id": "msg-7f1c0a2e",
    "task_id": "task-3b9d61c4",
    "task_type": "PTZ_CONTROL",
    "payload": {
      "action": "zoom",
      "direction": "in",
      "speed": 1.0,
      "hold_ms": 800
    }
  }
}
```

| payload 字段 | 类型 | 必填 | 约束 |
|---|---|---:|---|
| `action` | string | 是 | 固定 `zoom` |
| `direction` | string | 是 | `in`（拉近，视场变窄）\| `out`（拉远） |
| `speed` | number | 是 | `0.1 .. 1.0`，归一化变焦速度；`ptzkey2.py` 用的是 `1.0`。`(0, 0.1)` 与 `(1.0, +∞)` **钳位**并在 ack `detail.clamped` 回报；`≤ 0`、NaN / Infinity、非数字 ⇒ 拒绝（`1003`，v2.0 §1.3）——方向只由 `direction` 表达，🚫 用负速度表示拉远 |
| `hold_ms` | integer | 是 | dead-man 时长，`600 .. 2000` ms，建议 **800**。★ 下限 600 = 续租周期 250 ms（§4 R-1）× 2 ＋ 余量，满足 `11` PTZ-D5「`hold_ms` 必须 > 续租周期的 2 倍」；低于它会在两次续租之间自停，表现为按住却一顿一顿。越界钳位并回报 |
| `camera` | string | 否 | 省略或 `cam_ptz_vis`；其他值见 §3.7 |

★ 同一 `task_id`、同 `direction` 与 `speed` 的后续 `zoom` = **续租**：后端只把到期时刻推后，**不向相机重发命令**（`11` PTZ-D2，相机侧零开销）。
★ 同一 `task_id` 内 **🚫 改 `direction` / `speed`**：要换方向或速度，按 §4 R-5 先 `stop` 再用**新 `task_id`** 开始。同 `task_id` 而参数不同 ⇒ `rejected`（`1003`，`detail.reason="params_changed_within_task"`），后端同时**停止**该 `task_id` 的变焦（停止永远安全）。
★ 续租被拒（例如中途被更高优先级源接管）⇒ Qt **停止对该 `task_id` 续租**，按 ack 的 `holder` 提示操作员。

### 3.4 `zoom_step` —— 单步变焦（点一下变一段）

```json
"payload": { "action": "zoom_step", "direction": "out", "speed": 0.6, "pulse_ms": 1000 }
```

| payload 字段 | 类型 | 必填 | 约束 |
|---|---|---:|---|
| `action` | string | 是 | 固定 `zoom_step` |
| `direction` | string | 是 | `in` \| `out` |
| `speed` | number | 是 | `0.1 .. 1.0` |
| `pulse_ms` | integer | 是 | `100 .. 2800` ms。后端「开始 → 等 `pulse_ms` → 停止」，自行收尾，Qt **不需要**再发 `stop` |

★ 与语音「拉近一点 / 拉远一点」（`18` E06）走同一条机内路径（`11` §7.4 `nudge`）。机内现用档位 small 350 ms / normal 1000 ms / large 2800 ms，Qt 可按这三档做按钮。
★ 单步执行中收到 `stop` 或新的 `move` / `zoom` / `zoom_step` ⇒ **立即打断**当前单步（先停再执行新指令），🚫 排队等它走完。

### 3.5 ★ `refocus` —— 重新对焦一次（v0.2 新增）

```json
"payload": { "action": "refocus" }
```

| payload 字段 | 类型 | 必填 | 约束 |
|---|---|---:|---|
| `action` | string | 是 | 固定 `refocus` |
| `camera` | string | 否 | 省略或 `cam_ptz_vis`；`cam_ptz_ir` ⇒ 拒绝（见 §3.7） |

★ **做什么**：后端让可见光相机**立刻重新合焦一次**（机内 `11` §7.4.4 `refocus`：对焦模式自动 → 手动 → 等 300 ms → 自动）。★★ **结束时相机一定回到自动对焦**（`11` §7.4.7 F2 / F4），中途出错也会先恢复自动再回 ack。
★ **什么时候用**：相机本身是自动对焦，转动或变焦停下后通常会自己合焦。只有停在与之前距离相差很大的景物上、画面仍然发虚时，才需要按一下。对应 `ptzkey2.py` 的 `f` 键。
★ **不影响转动**：`refocus` 不动云台、不建立也不续租 dead-man，转动或变焦进行中可以发，**不会**打断它们。
★ **执行中又收到 `refocus`**：不再重复执行，**立即**回 `accepted`、`detail.coalesced=true`（合并到正在进行的那一次；那一次的结果以它自己的 ack 为准）。
★ ack 在**执行完成后**回（约 0.3 ~ 1 s），🚫 收到就回 —— 否则 Qt 无法知道对焦是否失败。

### 3.6 `stop` —— 停止

```json
"payload": { "action": "stop", "axes": ["pan", "tilt", "zoom"] }
```

| payload 字段 | 类型 | 必填 | 约束 |
|---|---|---:|---|
| `action` | string | 是 | 固定 `stop` |
| `axes` | array[string] | 否 | ★ v0.2 起取值 `pan` / `tilt` / `zoom` 的非空子集；省略 = 停全部轴。**停在哪就是哪**，不回到任何位置。闭集外的轴名 ⇒ 拒绝（`1003`），**但仍先执行全轴停止**（停止永远安全） |

★ `stop` 的 `task_id` 用被停那次 `move` / `zoom` 的 `task_id`。未知或已结束的 `task_id` **仍然执行停止**（停止永远安全），ack 回 `accepted`，`detail.note="no_active_motion"`。🚫 回 `duplicate`：v2.0 §1.2 里 `duplicate` 专指「同一 `msg_id` 已处理过」，同一条 `stop` 重发（同 `msg_id`）才回 `duplicate`。
★ 键盘空格（`ptzkey2.py` 的「立即停止」）对应**省略 `axes`** 的 `stop`。
★ `stop` 不影响正在进行的 `refocus`（它不是运动）。

### 3.7 明确拒绝的写法（收到即拒，🚫 静默映射）

| 写法 | 处置 |
|---|---|
| 绝对倍率：`zoom_level` / `zoom_x` / `target_zoom` 等任何「变到某倍」的字段 | `rejected`，`error_code=1003`，`detail.code="E_INVALID_FIELD"`，`detail.reason="no_absolute_zoom"` |
| ★ 绝对或增量角度：`pan_deg` / `tilt_deg` / `angle` / `d_pan_deg` / `target_heading` 等任何「转到某角度 / 转多少度」的字段 | `rejected`，`1003`，`E_INVALID_FIELD`，`detail.reason="no_absolute_positioning"`（`11` §7.4.4 已删字段表同口径） |
| 对热成像变焦 / 对焦（`zoom` / `zoom_step` / `refocus` 带 `camera:"cam_ptz_ir"`） | `rejected`，`1006`，`detail.code="E_TASK_UNSUPPORTED"`，`detail.reason="ir_fixed_focal"` |
| `camera` 为 `cam_ptz_vis` / `cam_ptz_ir` 以外的值 | `rejected`，`1003`，`E_INVALID_FIELD`（v2.0 §1.3：🚫 把未知枚举降级解释） |
| ★ `move` / `stop` 带 `camera` 字段 | `rejected`，`1003`，`E_INVALID_FIELD`，`detail.reason="shared_head"`（前提 6：两路画面共用一个云台） |
| 本期未开放的动作（`preset` / `goto_preset` / `patrol` / `track` 等） | `rejected`，`1006`，`detail.code="E_TASK_UNSUPPORTED"` |
| ★ 修改码率、I 帧间隔、对焦模式、屏蔽触发等相机配置（含 `ptzkey2.py` 的码率提升） | `rejected`，`1006`，`E_TASK_UNSUPPORTED`，`detail.reason="camera_config_robot_managed"`。相机配置由机器人侧管理（`11` §7.4.7 F1 / F2 / B0），Qt 🚫 改 |

---

## 4. ★★★ 时序：按住转动 / 变焦、松手即停（Qt 侧必须照做）

```text
 Qt 按键/按钮        机器人(后端)                       相机
 ────────────        ────────────                       ────
 按下 ───────── move/zoom(task A, msg 1) ► 开始运动 ─────► ContinuousMove(pan/tilt 或 zoom)
                                         到期 = now + hold_ms
 按住 ── 每 250 ms ─ move/zoom(task A, msg 2..n) ► 只推后到期时刻   (不发命令)
 松开 ───────── stop(task A) ─────────►  停止 ─────────────► Stop
 ╳ 链路断/Qt 崩溃                          hold_ms 到期仍无续租
                                         ─► 自动停止 ──────► Stop  + 事件 PTZ_DEADMAN_STOP
```

| # | 规则 | 理由 |
|---|---|---|
| **R-1** | 按下立即发一条 `move` / `zoom`，按住期间**每 250 ms** 续租一次（同 `task_id`），**松开立即**发 `stop` | 续租周期必须 < `hold_ms / 2`（`11` PTZ-D5），800 / 250 留有两次丢包余量 |
| **R-2** | 🚫 依赖 `stop` 一定到达。`stop` 丢了，后端在 `hold_ms` 内自停 | `ContinuousMove` 会一直运动；★ 水平轴无限位，dead-man 是唯一防线（`11` PTZ-C6） |
| **R-3** | ★★ **续租每条换新 `msg_id`**，`task_id` 不变 | 重发同一个 `msg_id` 会被 v2.0 §1.2 的去重窗口吞掉 ⇒ 续租不生效 ⇒ 按住时云台一顿一顿（`11` PTZ-D3，必然有人踩的坑） |
| **R-4** | 键盘按住时，操作系统的**自动重复**（首延迟约 500 ms，之后约 30 次/秒）由 Qt 自己去抖：按键事件只用来判断「是否仍按着」，续租按 R-1 的固定 250 ms 发，🚫 每个重复键事件都发一条 | `ptzkey2.py` 的 `HOLD_FIRST=0.85 s / HOLD_REPEAT=0.18 s` 正是为这个去抖（属输入设备层，`11` PTZ-D5）；相机每条命令约 58 ms，上限约 17 Hz（`11` PTZ-C4） |
| **R-5** | 同一时刻只做一件事：转动中改方向、转动中开始变焦、变焦中开始转动，都先 `stop` 再用新 `task_id` 开始 | 与 `ptzkey2.py` 一致（换键先 Stop 再发新命令）；避免两条 `ContinuousMove` 在相机侧叠加。★ `refocus` 不是运动，不受本条约束 |
| **R-6** | 发送频率上限：同一机器人 `cmd/ptz` **≤ 8 条/秒**（含续租、停止、换向时的 stop＋新指令、`refocus`）。超出部分后端合并，ack `detail.coalesced=true`；★ **`stop` 永不被合并丢弃** | R-1 续租只占 4 条/秒；相机侧真正的命令只在开始 / 停止时产生（`11` §7.4.5 17 Hz 预算表），续租为零开销 |
| **R-7** | 只允许**一个**云端控制端操作同一台机器人的云台。多个 Qt 同时发，后端都按来源 `cloud` 处理，互相覆盖 | 域⑤ 按来源仲裁，不区分同一来源内的多个客户端（`11` §7A） |
| **R-8** | `state/link` 判为离线（v2.0 §4.1，3 秒未到）时 Qt 停止续租并禁用按钮 | 离线期间指令到不了；后端 dead-man 已在 `hold_ms` 内自停 |

**★ 键盘映射（参考 `ptzkey2.py`，Qt 可按自己的界面调整，但语义须一致）**

| 按键 | 动作 | 说明 |
|---|---|---|
| `↑` `↓` `←` `→` | `move`，`direction` = `up` / `down` / `left` / `right` | 按住持续转，松开发 `stop`；`speed` 默认 `1.0` |
| 两个方向键同时按住 | `move`，`direction` = `up_left` / `up_right` / `down_left` / `down_right` | 相反方向同时按住 ⇒ `stop` |
| `+`（或 `=`） | `zoom`，`direction=in` | 按住持续拉近 |
| `-`（或 `_`） | `zoom`，`direction=out` | 按住持续拉远 |
| `f` | `refocus` | 单次，不需要按住 |
| 空格 | `stop`（省略 `axes`） | 立即停止全部轴 |

★ `ptzkey2.py` 里与「键盘」无关、只属于机上调试的部分**不进本映射**：启动时改对焦设置（机器人侧负责，`11` §7.4.7 F1 / F2）、提码率（已取消，前提 7）、`q` 退出（Qt 自己的界面行为；★ 但 Qt 关闭或切走前**必须**先发一次 `stop`）。

---

## 5. 回执 `cmd/ptz/ack`

形状与 v2.0 §3.1 普通任务 ack 相同，`task_type` 固定 `PTZ_CONTROL`：

```json
{
  "v": 1, "rid": "gj-001", "ts": 1785733000.180, "seq": 77, "src": "p5_gateway",
  "data": {
    "msg_id": "ack-ptz-0193",
    "ref_msg_id": "msg-7f1c0a2e",
    "task_id": "task-3b9d61c4",
    "task_type": "PTZ_CONTROL",
    "result": "accepted",
    "accepted": true,
    "error_code": 0,
    "reason": "",
    "detail": { "action": "zoom", "renewed": false, "clamped": [], "hold_ms": 800 }
  }
}
```

| `detail` 字段 | 含义 |
|---|---|
| `action` | 回显请求的动作 |
| `renewed` | `true` = 这是一次续租（未向相机发命令）；仅 `move` / `zoom` |
| `clamped` | 被钳位的字段名，如 `["speed"]`；无则空数组 |
| `hold_ms` | 实际生效的 dead-man 时长（钳位后）；仅 `move` / `zoom` |
| `coalesced` | 可选；`true` = 本条因超频被合并，或 `refocus` 合并到正在进行的一次 |
| `note` | 可选；如 `stop` 的 `no_active_motion` |

**错误码**（沿用 v2.0 §10，不新造码）：

| 情形 | `error_code` | `detail.code` | `detail` 补充 |
|---|---:|---|---|
| 字段缺失 / 类型错 / 绝对倍率 / 角度量 / 闭集外取值 | 1002 / 1003 | `E_REQUIRED_FIELD` / `E_INVALID_FIELD` | `field`；绝对倍率带 `reason="no_absolute_zoom"`，角度量带 `reason="no_absolute_positioning"` |
| 未开放动作 / 红外变焦或对焦 / 改相机配置 | 1006 | `E_TASK_UNSUPPORTED` | `reason` |
| 云台被更高优先级占用（本地语音、自动跟踪、BIT 自检等） | 2002 | `E_BUSY` | `holder`、`holder_priority`（`11` §7A.0 域⑤） |
| 云台不可用（相机离线 / 认证锁定 / 健康度 fail） | 2001 | `E_NOT_READY` | `item="ptz"` |
| ★ `refocus` 执行失败（相机配置接口写入失败） | 2001 | `E_NOT_READY` | `item="ptz"`、`reason="refocus_failed"`、`focus_mode_restored`（`true` = 已确认回到自动对焦） |
| 急停或安全互锁期间被禁止（若用户裁定急停也停云台，见 §8 U-2） | 2007 | `E_ESTOP_ACTIVE` | —— |

★ 普通 ack 的 `accepted` 只代表**已下发给相机**（续租则代表到期时刻已推后），不代表已经转到 / 变到位（朝向与倍率都读不回来）。是否真的转了、拉近了，以 `state/media` 端点的**画面**为准。
★ **拒绝事件**：v2.0 §10「每次任务拒绝还必须产生一条可靠 `event/{sev}/task`」对本 key 同样适用，但**按 `task_id` 去重**——同一次按住里连续被拒的续租只产生**一条**事件，否则 4 Hz 续租会变成 4 Hz 事件风暴。

---

## 6. 事件

经现有 `xbrain/{rid}/event/{severity}/{category}` 发出（v2.0 §5），`category="payload"`：

| `code` | `sev` | 触发 | `detail` |
|---|---|---|---|
| `PTZ_DEADMAN_STOP` | warn | `move` / `zoom` 的 `hold_ms` 到期未续租，后端自动停止 | `task_id`、`action`、`hold_ms`、`source="cloud"` |
| `PTZ_PREEMPTED` | warn | 转动 / 变焦中被更高优先级源接管，后端已先停止再交接（`11` PTZ-D4） | `task_id`、`new_holder` |

★ 机内 PTZ 事件是 `event/warn/ptz`，网关按类别映射表把 `ptz` 归入 v2.0 的 `payload`（《契约对撞》H-4）。`code` 必须由 p2 填成上表的值；p2 不填时网关会按 `title` 派生（H-10），那就不一定是上表的码。`PTZ_PREEMPTED` 是本文新增，机内尚无对应事件。
★ `refocus` 失败只走 ack（§5），不另发事件 —— 它是操作员当场按下、当场看结果的单次动作。

---

## 7. 后端映射（写给机器人侧实现者）

| Qt `payload` | 机内 `cmd/ptz`（`11` §7.4 `PtzCommand`） |
|---|---|
| ★ `move{direction, speed, hold_ms}` | `action:"jog"`，`vel:{pan: P, tilt: T, zoom: 0}`，`hold_ms`，`source:"cloud"`。★ `P` / `T` 按方向取：`left` → `P=-speed`；`right` → `P=+speed`；`up` → `T=+speed`；`down` → `T=-speed`；斜向两轴同时取值（如 `up_left` → `P=-speed, T=+speed`），未涉及的轴为 `0`。符号约定见 `11` §7.4.4（pan + = 向右，tilt + = 抬头） |
| `zoom{direction, speed, hold_ms}` | `action:"jog"`，`vel:{pan:0, tilt:0, zoom: ±speed}`（`in`=+，`out`=−），`hold_ms`，`source:"cloud"` |
| 同 `task_id` 续租（`move` / `zoom`） | 再发一条 `vel` 完全相同的 `jog`，**新 `cmd_id`**（PTZ-D2 / D3） |
| `zoom_step{direction, speed, pulse_ms}` | `action:"nudge"`，`vel.zoom = ±speed`，`pulse_ms` |
| ★ `refocus` | `action:"refocus"`，`source:"cloud"`（`11` §7.4.4 / §7.4.7 F4） |
| `stop{axes}` | `action:"stop"`，`axes` 原样（省略则省略） |
| 机内 `cmd/ptz/ack` | 由 p5 译成 §5 的 v2.0 ack（与 `cmd/task/ack` 同一套译码，`E_*` → 整数码） |

★ 网关只做**校验 + 信封重建 + 字段映射**，仲裁（域⑤）、dead-man、相机调用都在 `p2_core`，与本地语音、HMI 共用一条路，🚫 另开通道。

**机内错误码 → Qt 整数码**（按 `xbrain/p5_gateway/outbound/error_map.py` 现表，🚫 另造）：`E_BUSY` → 2002 · `E_CAPABILITY` → 2002 · `E_UNHEALTHY` → 2001 · `E_NOT_IMPLEMENTED` → 1006 · `E_SCHEMA` → 1003。★ 本文 §3.7 的「绝对倍率 / 角度量 → 1003」「红外 / 改相机配置 → 1006」「`move` 带 `camera` → 1003」由**网关校验时直接判出**，不经 p2；若让它们走到 p2 再以 `E_CAPABILITY` 回来，会被映射成 2002「忙」，Qt 会误以为稍后重试能成功。

★★ **key 隔离（与《契约对撞》A-1 / E-1 同一个坑）**：云端 key 是 `xbrain/{rid}/cmd/ptz`，p2 订阅的是机内**相对** key `cmd/ptz`（`xbrain/p2_core/runtime/ptz_wiring.py` `CMD_PTZ_TOPIC`），两者是不同的 Zenoh key。🚫 让 p2 订阅云端那条：Qt 的原始报文会绕过网关的校验、信封重建与错误码映射直达仲裁域。⚠️ 而 `11` §2.2 现把 `xbrain/{rid}/cmd/ptz` 登记为机内 key、发布者含「云端」——这与本文的分层冲突，冻结时须在 `11` 订正（机内 key 改为相对形，云端 key 归网关独订）并登记进《契约对撞》。

---

## 8. 待定事项（冻结前需裁定）

| # | 问题 | 建议 |
|---|---|---|
| U-1 | `hold_ms` 范围上限 2000 ms 是否合适 | 2000（超过 2 s 无续租就该停）。机内 `ptz.hold_max_ms` 现为配置项，冻结时对齐 |
| U-2 | 急停是否同时停止云台（`11` PTZ-D6 / U-PTZ-4） | 建议**不停**：急停针对底盘，急停那一刻最需要画面 |
| U-3 | 云端与本地语音同时操作云台时谁优先（域⑤ 优先级） | 按 `11` §7A 现行域⑤ 表（`manual_cloud` 700）；云端被拒时 Qt 显示 ack 里的 `holder` |
| ~~U-4~~ | ~~v0.2 是否开放 pan / tilt / 预置位~~ | ✅ **已关闭（2026-10-28，`99` U98）**：开放 8 方向转动（`move`，§3.2）与重新对焦（`refocus`，§3.5）；★ **预置位、巡视仍不开放**（§3.7）。原建议的动作名 `move` 沿用，`goto_preset` 未采纳 |
| U-5 | `11` §2.2 把 `xbrain/{rid}/cmd/ptz` 登记为机内 key（见 §7 key 隔离） | 机内改相对 key，云端 key 归网关；与 A-1 同一处置 |
| ★ U-6 | 俯仰软限位（`00` PAY-11b，−90° ~ +30°）在转动中无法执行（前提 5 / §3.2）；俯仰到机械限位后持续下发是否堵转待实测（`11` T-PTZ-4） | 先以 dead-man ＋ 操作员看画面为准；T-PTZ-4 实测若会堵转，再议是否对 `up` / `down` 单次按住时长设上限 |

---

## 9. 实现现状（2026-10-28 读代码核实，冻结后按此排工）

| 环节 | 现状 | 缺口 |
|---|---|---|
| Qt → `xbrain/{rid}/cmd/ptz` | p5 **未订阅**，云端 PTZ 指令今天无落点 | p5 新增入口：校验 §3、映射 §7、ack 译码 §5、拒绝事件按 `task_id` 去重 |
| `p2_core` `cmd/ptz` 处理 | `runtime/ptz_wiring.py` 只认**语音意图**格式（`intent_id` E01/E05/E06/E07/E08/E09）；E01 转动与 E06 变焦都是「开始 → 睡 `pulse_ms` → 停止」的阻塞脉冲 | ① 接 `11` §7.4 `PtzCommand`（`jog` / `nudge` / `stop` / `refocus`）；② `jog` 的 dead-man 与续租（PTZ-D1 ~ D3，`ptz/hold_ms_renewal.py` 已有判据实现可接）；③ 发 `cmd/ptz/ack`；④ 域⑤ 仲裁与 PTZ-D4 交接先停 |
| ★ 转动（`move`） | 机内只有语音的定时脉冲；无开放式 `jog` | 同上 ①②；★ 斜向须合成**一条** `ContinuousMove`（PTZ-C5），🚫 拆成两条 |
| `zoom_step` | 可复用现有 E06 脉冲路径 | ⚠️ 现有脉冲是**持锁阻塞**（`PtzDomain` 一把锁 + `time.sleep(pulse_ms)`），执行中的 `stop` 要等它走完才进得去 ⇒ 违反 §3.4「立即打断」，2800 ms 的大档尤其明显。先上线也须改成可打断的等待；ack 须在开始时回（§2） |
| ★ `refocus` | **未实现**（p2 没有任何 LAPI 对焦调用） | 新增 LAPI `Channels/0/Image/Focus` 读改写：`FocusMode` 2 → 0 → 等 300 ms → 2；任何一步失败都补写 `2` 再回 ack（`11` §7.4.7 F4）。★ 每次 LAPI 调用的超时须收在约 0.5 s 内，才能满足 §2 的 2 s ack 时限（`ptzkey2.py` 用的 10 s 不能照搬） |
| ★ 对焦会话设置（F1 / F2） | **未实现**：p2 不写 `ShieldTrigger.MovePTZ=1`、不确保 `FocusMode=2` | p2 启动 PTZ 域时按 `11` §7.4.7 F1 / F2 设置、退出恢复；否则云台一动相机就重新对焦，画面在转动中反复变虚（`ptzkey2.py` 注释 3 的实测） |
| 事件 `code` | p2 的 PTZ 事件未填 `code` | 按 §6 填 `PTZ_DEADMAN_STOP` / `PTZ_PREEMPTED` |
| 媒体 | `state/media` 已发可见光 `main/sub` 端点（2026-10-24 上线），Qt 可看到转动与变焦效果 | 无 |

★ 在 p2 的 `jog` + dead-man 落地之前，**后端对 `move` / `zoom` 一律回 `1006 E_TASK_UNSUPPORTED`**，🚫 用阻塞脉冲去模拟「按住」——那会在松手后继续转一段 / 推一段，正是 R-2 要防的失控。`refocus` 同理，未实现前回 `1006`。
