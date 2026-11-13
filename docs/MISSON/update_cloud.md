# 云端对接契约更新说明（update_cloud）

| 项 | 内容 |
|---|---|
| 版本 | 2026-11-13（~~2026-11-11~~：围栏 U183～U188 一批加入，见 §0 第 7～9 项） |
| 机器人代码基线 | `main` = `79e030e`（2026-11-13，含围栏 `99` U183～U188）；~~`73af3ef`（2026-11-11）＋ U164~~。整机 `gj-001` |
| 读者 | 甲方云端软件（Qt 端、云端语音文本、云端键盘遥控）开发同学 |
| 基线文档 | ①《任务枚举_qt端v2.0》②《json格式文件_qt端v2.0》③《新建+Qt→后端心跳+key》④《PTZ控制契约_qt端v0.2草案》⑤《云端语音文本交互契约_v1.0》（2026-10-26）。本文只写**这些文档之后发生的变化**，以及原文没写清、联调必然会撞上的事实；没提到的条目按原文执行 |
| 依据 | 机器人当前代码逐条核对（文件与函数名见各条），以及我方决策记录 `99` U96～U188 |

**标签说明**

| 标签 | 含义 | 云端怎么做 |
|---|---|---|
| 【新增】 | 原文没有的接口或规则，机器人已经按此运行 | 照本文实现 |
| 【变更】 | 原文写了，但机器人行为已按用户裁决改变 | 以本文为准，原文对应条目作废 |
| 【澄清】 | 原文没写清、或机器人侧取值原文未列出 | 按本文理解 |
| 【偏差·待修】 | 机器人当前**不符合**原文，属我方缺陷 | ⚠️ **不要按缺陷去适配**；我方修正后另行通知。在修正前请了解其现象，避免误判 |

---

## 0. 先看这一页：联调前必须处理的九件事

| # | 事项 | 不处理的后果 | 详见 |
|---|---|---|---|
| 1 | Qt 必须**每秒发一次** `xbrain/{rid}/heartbeat/qt` | 机器人 20 s 判云端断开，**30 min 后自动返航并抢占当前任务** | §1 |
| 2 | 路径 / 围栏**录制**改为：文本指令开始和结束 ＋ **云端键盘遥控**开车（新通道，裸 key `cmd/teleop`） | 录制期间机器人只认键盘，其它方式都动不了它 | §2、§3 |
| 3 | 新增**低电告警、充电中告警、定位失效、录制提醒 / 自动保存、断链**等事件 | 云端看不到告警，或按 `code` 分流时分错 | §4 |
| 4 | 语音文本的**二次确认（L2）已全部取消**，`pending_confirm` 实际不再可用；一轮可能产生**两条** `state/voice_turn` | 云端确认弹窗等不到结果；同一轮重复展示 | §8 |
| 5 | `GOTO_KEYPOINT` 的执行语义已明确：航点**按顺序串成一条线**、只沿 `recorded_path_id` 走、按末点半径判到达；路径或航点不存在、在围栏外，提交时即拒（2026-11-11 修正） | 照旧理解成「巡整条路径」或「逐点停靠」会与机器人实际行为不符 | §6.1 |
| 6 | 急停后，云端发 `STOP_TASK resume` 或新的 `GOTO_KEYPOINT` 会**解除急停锁存，机器人随即开走** | 操作员以为只是恢复任务，机器人实际已经开走 | §6.3 |
| 7 | ★ 2026-11-13：对象清单里的围栏**按实际类型**上报（`keep_in` / `forbid_region` / `speed_limit_region` / `alarm_region`，带 `enabled`，未启用的也报）。**请先让 Qt 接受这几个类型，再升级机器人** | Qt 若按旧闭集拒收，整份对象清单被丢弃，地图上围栏全部消失 | §5.4 |
| 8 | ★ 2026-11-13：新任务 **`SET_FENCE_CONFIG`**（禁止区 / 限速区增删改与启停、活动区域只能启停、统一限速值）；**`SET_ALARM_CONFIG` 改为部分更新**，时间段 / 目标 / 声光参数存到机器人上，新增目标类型 `animal_in_region`；**关闭围栏只能由云端** | 旧版 Qt 发不了禁止区和限速区；按整包覆盖理解会误删设置 | §6.4、§6.4A |
| 9 | ★ 2026-11-13：**云端键盘只在录制时能开动机器人**；新增围栏事件（进出、撞线每 3 s 一条、对象到不了）和**报警区自主报警**事件（巡逻 / 导航途中自动声光驱离，不停车） | 不录制时按键无反应，被误判为故障；新事件无处展示 | §2、§4.3.x、§4.3.y、§4.3.z |

---

## 1. 云端在线判定与断链自动返航【新增 · 必须实现】

### 1.1 机器人怎么判定「云端在线」

- 判定方是机器人的网关 `p5_gateway`（`xbrain/p5_gateway/uplink/link_state.py`）。**收到云端的任何一条报文**，都会刷新「最后联系时刻」。这里的报文包括格式错误的报文。
- 刷新来源**只有**下面这些 key（`runtime/cloud_wiring.py` 的 `CloudBridge._rx` 包装，以及 `main_wiring` 的事件确认回调）：
  `xbrain/{rid}/cmd/task` · `cmd/estop` · `cmd/media/session` · `cmd/file/ack` · `heartbeat/qt` · `cmd/ptz`。
- ⚠️ **语音文本通道不算**：`cmd/voice_text` 和 `audio/voice_in` 由另一个进程（`p4_agent`）处理，**不刷新**链路计时；`audio/broadcast` 与云端键盘 `cmd/teleop` 也不刷新。
  ⇒ 云端只用语音文本、键盘遥控、喊话流时，机器人会认为云端已经断开。
- 机器人开机后，**从未收到过云端报文也照样计时**。云端开机后不连，同样会触发下面的返航。

### 1.2 阈值与 `state/link` 的取值

计时从最后一次收到云端报文开始，用机器人本机单调钟。

| 断开时长 | 机内等级 | `state/link.state` | `cloud_link` | 机器人动作 |
|---|---|---|---|---|
| < 5 s | L0 | `up` | `true` | — |
| ≥ 5 s | L1 | `degraded` | `true` | — |
| ≥ 20 s | L2 | `down` | `false` | — |
| ≥ 1800 s（30 min） | L3 | ⚠️ `degraded`（用户 2026-08-24 裁决：机器人仍在线、正在返航，不报 down） | `true` | **自动注入返航任务**（§1.3） |

- `disconnected_s` 是当前连续断开秒数。
- 链路状态变化时，机器人另发事件，见 §4.3.6。
- 阈值目前写在代码常量里（`LinkThresholds(degraded_s=5.0, down_s=20.0, rtb_s=1800.0, stable_s=10.0)`），我方后续会改成配置项，取值不变。

### 1.3 断链返航（用户裁决：必须执行）

- 进入 L3 时，机器人插入一条返航任务：`task_id = rh-<数字>-<数字>`，`task_type = return_home`，优先级 95。
  - 它会抢占正在执行的任务，被抢占的任务在 `state/task` 里进 `suspended`（v2.0 的 `paused`）。
- 目的地是名为「返航终点站」的航点。
- 同一次断开只插入一次。链路恢复后，返航任务**不会自动取消**，要取消请发 `STOP_TASK` `{action: "cancel", target_task_id: "rh-..."}`。
- 低电告警期间，返航任务暂缓派发，低电结束后再执行（§4.3.1）。

### 1.4 心跳 `heartbeat/qt` 的实现要求（格式见基线文档③）

- 连接成功后立即发，之后**每秒 1 条**，`data = {session_id, state: "up"}`。
- 正常退出时发一次 `state: "down"`，机器人立即判断开，不等 5 s。
- `session_id` 变化（包括第一次出现），机器人当即重发一次全量的会话级状态，例如 `state/geo/manifest`。
- 心跳不回 ack。

---

## 2. 云端键盘遥控【新增 · 替代 v2.0 §2.6 / §3.6 的「云端遥控禁止」】

> 用户裁决（`99` U134 / U135 / U150，2026-11-05～06）：云端键盘遥控~~**完全开放**~~，主要用于路径和围栏录制。
> ★★ **`99` U186（2026-11-13）起只在录制进行中生效**（用户：「键盘开车当前仅仅在路径、围栏录制的时候才生效，其他情况下键盘开车是被系统禁止的」）。没有录制时发来的键盘帧机器人**丢弃不执行**，并播报「现在没有在录制, 键盘不能控制机器人」、发一条事件（机内 `event/warn/teleop`，云端收到的是 **`event/warn/system`**（§4.1 类别映射），`title = "keyboard_refused"`、`code = "KEYBOARD_REFUSED"`、`detail = {kind: "keyboard_refused", reason: "not_recording", device: "keyboard_cloud"}`；同一次连续按键只发一条）。请先发「现在开始录制……」，收到 `teach_result accepted` 后再开放键盘（§3.1）。「录制中」指 `state/teach` 的会话状态为 `recording` 或 `paused`；机器人超过 3 s 没收到 `state/teach` 即按「不在录制」处理。
> ★ **录制时的键盘移动速度由机器人决定**（`99` U179，2026-11-13）：录制中（含暂停中）键盘帧的**移动速度**不再用语音设定的移动速度，而按定位质量取：RTK 固定解且双天线航向有效 **2.0 m/s**；RTK 浮点 / DGPS，或航向来自 COG / 惯导桥接 / 没有航向 **1.0 m/s**；单点定位 / 无定位时**按键不动**。**转向速度**仍是语音设定的值。没有避障门（用户裁决：录制是人工驾驶，看云台画面，空格即停），急停与底盘限幅照常生效。云端接口不变，键盘界面请据此提示操作员。
> v2.0 的 `MANUAL_VELOCITY` 仍然禁止，经 `cmd/task` 发照旧回 1006。本节是另一条独立通道。

### 2.1 连接与 key

| 项 | 约定 |
|---|---|
| 连接 | Zenoh 1.9.0，`client` 模式，关闭组播 scouting，直连机器人通用面 router `tcp/<机器人 IP>:7447`（当前 `tcp/192.168.188.200:7447`）。🚫 不连 7449 |
| key | ⚠️ **裸 key `cmd/teleop`**：**没有** `xbrain/{rid}/` 前缀，与其它所有云端 key 不同。机器人订阅的就是这个裸 key（`xbrain/p1_motion/runtime/main_wiring.py` `CMD_TELEOP_TOPIC = "cmd/teleop"`）；`xbrain/{rid}/cmd/teleop` **没有任何订阅者**，发了等于没发 |
| 多机器人 | 裸 key 没有 rid，按「连的是哪台机器人的 router 就控制哪台」区分。一个 session 只连一台机器人 |
| 外层 | **裸 JSON 业务体**，没有 `v/rid/ts/seq/src/data` 外层 |
| 拥塞策略 | 建议 DROP：迟到的帧比丢掉的帧更危险，下一帧 100 ms 后就到 |
| 回执 | 无 ack。实际是否在动，请看画面或 `state/robot.motion` |

### 2.2 帧格式

```json
{"device":"keyboard_cloud","client_id":"cloud-kbd-01","client_seq":118,"deadman":true,
 "axes":{"vx":1.0,"vy":0.0,"wz":0.0},"buttons":{"mark":false,"release":false}}
```

| 字段 | 约定 |
|---|---|
| `device` | 固定 `keyboard_cloud`；闭集外的值整帧丢弃 |
| `client_id` / `client_seq` | 建议填（客户端标识 / 每帧递增）。⚠️ 机器人**当前不校验**，也不按序号丢弃乱序帧，所以请保证**同一时刻只有一个云端键盘客户端** |
| `deadman` | 持有遥控权期间恒为 `true`；退出时发一帧 `false` 交还 |
| `axes` | `vx` 前后、`vy` 左右横移（左为正）、`wz` 原地转（左转为正），每轴是 `[-1, 1]` 的归一化量，机器人乘以语音设定速度（§2.3），越界值被夹到 ±1。机器人**接受连续值与多轴组合**；「只取 -1 / 0 / +1、同一时刻只有一个轴非零」是参考客户端的键位约定，不是机器人的规则。三轴全 0 表示原地停住，仍持有遥控权 |
| `buttons` | 固定 `{"mark": false, "release": false}`（本通道不打点） |

参考键位（我方交付的参考客户端 `/opt/xbrain_teleop_kbd`）：`w` 前进 · `s` 后退 · `a` 左转 · `d` 右转 · `y` 左移 · `u` 右移 · 空格停止 · `q` 退出（先发 `deadman=false`）。
按键是**锁存**的：按一下持续动作，直到按空格；按别的方向键直接切换，不必先停。

### 2.3 时序与速度

- 第一次按键后，以 **10 Hz** 持续发送当前动作，按键之前不发。
- 机器人 **400 ms** 收不到新帧即自动停车。网络中断或客户端被杀，都按这一条停。
- 速度**不由云端给**。机器人用 `axes × 语音设定值` 换算，设定值的规则见下表。

  | 设定项 | 默认 | 范围 | 语音调节 |
  |---|---|---|---|
  | 前后、横移速度 | 0.5 m/s | 0.5 ~ 1.0 | 「移动速度快一点 / 慢一点 / 最快…」，每次 ±0.1 |
  | 原地转速 | 0.6 rad/s | 0.4 ~ 0.8 | 「机器人转速快一点…」，每次 ±0.1 |

  语音设定过的值会保存，机器人重启后仍然生效。
- 优先级：机器人本地手柄、本地键盘高于云端键盘。云端键盘持有遥控权期间，自主任务暂停。

### 2.4 ⚠️ 安全约束（必须让操作员知道）

- 用户裁决 U150：云端键盘**无条件执行**，只受急停和底盘最大速度限幅约束。
  - **速度门、障碍减速与停车、旋转许可、定位、航向检查一律不作用于它。**
  - ⇒ 机器人不会自己避障。全程靠操作员看画面驾驶。
  - ★ **围栏（U186，2026-11-13）**：**录路径**时开到活动区域边界或禁止区边界会被**直接拦停**（没有提前减速），持续播报「路径到了活动区域边界, 已停止前进, 请往回开」/「路径碰到禁止区域X, 已停止前进, 请往回开」并上报 `contact_start` / 每 3 s 一条 `contact_hold` / `contact_end`（§4.3.y），往回开不受影响。**录围栏**时不拦停（要沿边界开），越界时只提示一次「注意, 现在进入了禁止区域X」/「注意, 现在已在活动区域外」。~~也会开出围栏~~（U150 原文，录路径时不再成立）。
- 录制期间语音急停**不生效**（`99` U45），停车请发三轴全 0（空格）或用 `cmd/estop`。
- 参考客户端 `README.md` 的「安全提示」还写着「机器人照常经过速度门、障碍减速、围栏」。这是 U150 之前的旧说法，以本节为准，我方会同步修正 README。

---

## 3. 路径 / 围栏录制全流程【新增】

录制**没有专用的云端 key**。开始和结束都用文本指令（`cmd/voice_text`，§8），开车用云端键盘（§2），结果从 `state/voice_turn` 和事件里看。

### 3.1 开始

- 文本示例：「现在开始录制3号路径」「现在开始录制营区活动区域围栏」（~~「现在开始录制东门围栏」~~：U176 起围栏名字必须带类型）。
- ★ **围栏类型取自名字**（`99` U176，2026-11-12）：名字里恰好一个关键字 —— 「活动区域」只许在内、「报警区域」报警区、「限速区域」限速区、「禁止区域」禁止进入。★ U184（2026-11-13）起**录制不分先后**（~~必须先录活动区域~~），活动区域只许一个：
  - 名字里没有类型：该轮直接 `rejected`，回复「围栏名字里要说明类型: 活动区域, 报警区域, 限速区域或禁止区域, 比如: 现在开始录制营区活动区域围栏」；
  - 已有活动区域又录一个：`teach_result` 为 `rejected`（`E_GEO_INVALID`）。~~限速值未配置时录限速区域被拒（`E_TEACH_STATE`）~~（U185 起限速区随时可录，取统一限速值，§6.4A）。~~还没有活动区域就录其它类型被拒~~（U184 已取消）；
  - ★ 禁止区与报警区 / 限速区 / 另一禁止区**重叠**：不保存，会话关闭，回复「录制已退出, X没有保存: 和Y重叠了, 禁止区域不能和报警区域, 限速区域或其它禁止区域重叠」（U184）；
  - ★ 不在活动区域内（或新活动区域没围住已有围栏）：**照常保存**，回复后加一句「注意: 它不在活动区域X内, 机器人到不了, 不会生效」（U184；~~不保存~~）；
  - ★ 保存活动区域 / 禁止区时，若有航点或路径会因此到不了，回复后加「注意: A和B在禁止区域X里面, 机器人到不了」或「注意: A和B在活动区域外面, 机器人到不了」，同时发事件（§4.3 `objects_unreachable`）。
- 名称要求：
  - 取「录制 / 记录」之后的整段，必须以 `路径 / 路线 / 线路 / 环线`（路径录制）或 `围栏`（围栏录制）结尾，2～12 个字。
  - 不能与已有同类对象重名。
  - 缺名或重名都直接拒绝，不追问。
- 机器人在开始前依次检查下表，第一条不满足就拒绝。★ 2026-11-11（U165）起开始的结果会**再发一条** `state/voice_turn`（§8.5）：
  - 该轮自己的记录只表示指令已发出（`layer="record_rule"`，`reply_text`「好的,开始录制X…」），**不能**据此开放键盘；
  - 随后的 `layer="teach_result"` 记录：开始成功 `verdict=accepted`、`reply_text`「已开始录制X」；被拒 `verdict=rejected`、`reject_code` 为下表的码、`reply_text` 为原因句（例如「现在不能开始录制: 定位不是RTK固定解」）。
  - 缺名、名字长度不对、重名、已在录制这几种在机器人发出命令之前就拒绝，该轮记录直接是 `verdict=rejected`，**不会再有** `teach_result`。
  - ~~【偏差·待修】拒绝不进 `state/voice_turn`，该轮记录照常是 `verdict=accepted`~~ —— U165 前的行为（2026-11-11 gj-001 室内实测），已修。

| 检查项 | 不满足时的错误码 |
|---|---|
| 已有录制会话 | `E_TEACH_BUSY` |
| 正在执行巡逻、去航点或充电任务 | `E_BUSY` |
| 定位不是 RTK 固定解（`rtk_fixed`） | `E_TEACH_QUALITY` |
| 健康度禁止运动（例如感知相机没启动） | `E_UNHEALTHY` |
| 硬件急停按下 | `E_LOCKED` |
| 电量 < 25 % | `E_LOW_BATTERY` |
| 急停链路（`state/link.estop_path`）不可用，且没有本地手柄或键盘 | `E_UNHEALTHY` |

### 3.2 录制中

- 采点规则：每 1 s 试采一次；与上一点相距不足 0.5 m 不记；定位低于 `rtk_fixed` 的点丢弃。
- **没有点数、长度限制**（用户裁决 U163，2026-11-11）。唯一的限制是时长，见 §3.4。
- 录制中只接受：查询类、会话应答、闲聊，以及「退出 / 取消 / 打点 / 查询录制」。
  - 其它指令（运动、导航任务、云台、声光等）回复「正在录制X, 录制期间只能用键盘控制机器人. 要结束请说退出录制」。
  - 这种拒绝的 `verdict` 是 **`rejected`**，`reject_code` 为 null，原因在 `reply_text`，见 §8.4（U165；~~此前为 `accepted`~~）。
  - 录制中再说开始录制，回复「正在录制X, 不能开始新的录制…」。
- 录制中说「停 / 急停 / 趴下 / 站起来」**不执行**：`verdict=rejected`、`bypass="estop"`（或对应值），回复「录制中语音急停不生效, 请用键盘空格键停车」。
- 录制中云端下发 `GOTO_KEYPOINT`，回 `rejected`、`error_code=2002`、`detail.code=E_BUSY`、`detail.reason=teach_session_active`。
- ⚠️ 录制的实时状态（点数、长度、剩余时间）**没有投影到云端 key**。云端可以发文本「录了几个点」查询，或看下面的事件。

### 3.3 结束

| 文本 | 结果 |
|---|---|
| 「退出录制」「录制完成」「结束录制」「录完了」「3号路径录制完成」 | 以开始时的名字**直接保存**并退出。带名字时，名字必须是正在录的那个，否则回复「正在录制的是A, 不是B…」，不执行 |
| 「取消录制」「放弃录制」「取消3号路径录制」 | **直接丢弃**并退出，不再二次确认（U137） |

- 结果通过**第二条** `state/voice_turn` 回报，`layer="teach_result"`（§8.5）。
  - 成功示例：「已保存3号路径, 共64个点, 长320米」。
  - 几何不合格（路径不足 2 点、围栏不足 3 顶点、自交、面积过小）时不保存，会话关闭，原因写在回复里。
  - 写入失败（例如名字在录制期间被别处占用）时**数据保留**：`verdict=rejected`，`reject_code` 为真实原因（如 `E_NAME_CONFLICT`），回复「X没有保存: 名字已被占用. 录制数据还在, 请说保存为新的名字, 或者放弃录制」（U165；~~此前码一律 `E_TEACH_GEOMETRY`~~）。

### 3.4 录制时长上限与自动保存（U162 / U163，2026-11-11）

- 一次录制最长 **180 min**（机器人配置项）。录制中和**暂停中**都计时。
- 到时前 **10 min、1 min** 各提醒一次，提醒时刻也是机器人配置项：语音播报，同时发事件（§4.3.3）和一条 `voice_turn`。
- 到时自动结束并**以开始时的名字保存**，然后播报结果、发事件、发 `voice_turn`。
- 保存失败（例如名字在录制期间被别人占用）时，**录制数据保留**。请再发「保存为新名字」，或发「放弃录制」。

### 3.5 保存之后

- 路径保存后出现在 `state/geo/manifest`：`type=recorded_path`，`geo_id` 形如 `r-1`、`r-004`。
- 围栏保存后是**未启用**状态，★ U183 起**会出现在 manifest 里**，`enabled=false`，类型按实际（见 §5.4）。~~**不出现在 manifest 里**~~
  - 要启用，请发「启用营区活动区域围栏」。该指令已不需要二次确认，发出即生效（§8.2）。
  - ~~★ U176：**先启用活动区域，再启用其它围栏**。活动区域未启用时启用其它围栏被拒……~~ ★ U184：启用**从不因位置被拒**。启用后视情况再说一句：不在活动区域内「注意: 它不在活动区域Y内, 机器人到不了, 不会生效」；有航点 / 路径因此到不了「注意: …机器人到不了」（并发事件）；机器人本身在区外 / 禁止区内「机器人现在在活动区域外, 只能往区域里面开」/「机器人现在在禁止区域X里面, 只能往外开」。
  - ~~启用后才会出现在 manifest 里，并且当前被标成 `alarm_region`，这是偏差，见 §5.4。~~ 已修（U183）：启用后 `enabled` 变为 `true`。

---

## 4. 事件 `event/{severity}/{category}`【变更 / 新增】

### 4.1 类别与级别的映射【澄清】

机器人内部有 23 个事件类别。网关按下表映射到 v2.0 的 9 个类别，key 段与 `data.category` 使用同一个映射值，表外的类别不发。实现见 `xbrain/p5_gateway/outbound/state_projection.py` 的 `INTERNAL_TO_V2_EVENT_CATEGORY`。

| v2.0 类别 | 机内来源 |
|---|---|
| `task` | `task` 任务、**`charging` 低电 / 充电** |
| `alarm` | `intrusion` 入侵、**`rtk` 定位失效** |
| `fault` | `chassis` 底盘、`health` 健康、`bit` 自检、`perception` 感知 |
| `fence` / `geo` / `teach` / `payload` / `arbitration` / `system` | 同名；另外 `ptz` → `payload`，`speed_limit` → `arbitration`，`comm`、`estop`、`data`、`mode_change`、`teleop`、`voice`、`motion` → `system` |

级别映射：机内 `info` → `info`、`warn` → `warn`、`alarm` → **`error`**、`fault` → **`fatal`**。

### 4.2 `code`、`state`、`task_id` 的实际取值【澄清 · 部分偏差·待修】

- `code`：
  - 有专门码的只有四类：`PTZ_DEADMAN_STOP`、`PTZ_PREEMPTED`（云台），`CLOUD_LINK_UP / DEGRADED / DOWN / LOST`（链路），以及网关拒绝事件的 `E_*`。
  - **其余事件的 `code` 由英文 `title` 自动派生**：取大写，非字母数字的字符替换成下划线。例如 `low battery, returning to charge` 变成 `LOW_BATTERY_RETURNING_TO_CHARGE`。【偏差·待修】这些码会随文案变化。
  - ⇒ **请按 `detail.type`（或 `detail.kind`）分流，不要按 `code`**。下文每类事件都给出了分流字段。
- `state`：机器人目前只发 `occurred`，没有 `active / cleared` 成对事件。
  - 「结束」用独立事件表达，例如低电结束是 `detail.type=low_battery_cleared`。
- `media` / `file_refs`：目前恒为空数组。

### 4.3 新增的业务事件

#### 4.3.1 低电告警与充电中告警（`event/error/task`、`event/info/task`，`99` U146 / U154 / U157）

规则（用户裁决）：

- 电量低于 **20 %** 开始告警。机器人去最近的、在某条路径 5 m 以内的充电站，没有就去「返航终点站」。续航不够也照走，并报 `low_battery_unreachable`。
  - 告警期间被打断的任务、以及告警开始时已在排队或定时的任务，统一暂缓。
- 电量 **≤ 5 %**：停车并趴下，之后不接受新任务（`E_LOW_BATTERY`，2005）。
- 底盘报告**开始充电**后，告警不结束，改报「充电中」，直到电量达到 **100 %** 才结束。拔电不足 100 % 时改报「充电中断」。
- **换电**：换上的电池电量 ≥ **80 %** 时立即结束告警，被打断的任务自动恢复；介于 20 % 和 80 % 之间时告警结束，但被打断的任务要人工「继续」。
- 告警结束后，暂缓的任务自动恢复或派发。人工「继续」不受暂缓限制。
- 告警期间**每 2 s 发一条**（配置项），持续发送，供云端持续显示。

| `detail.type` | key | 含义 |
|---|---|---|
| `low_battery` | `event/error/task` | 低电，正在去充电点 |
| `low_battery_unreachable` | `event/error/task` | 低电，续航可能不够也照走（`detail.unreachable=true`） |
| `low_battery_no_target` | `event/error/task` | 低电，没有定义充电站和返航终点站，无法自动返回 |
| `low_battery_stop` | `event/error/task` | 电量 ≤ 5 %，已停车趴下 |
| `battery_charging` | `event/error/task` | 充电中（低电告警尚未结束） |
| `charge_interrupted` | `event/error/task` | 未充满就停止充电（仍在告警中） |
| `low_battery_cleared` | `event/info/task` | 告警结束。`detail.cause` = `charge_full`（充满）或 `battery_swap`（换电）；`auto_resume` = 是否自动恢复；`cold_swap` = 关机期间换电；`held` / `parked` 分别是暂缓与转人工的任务 ID 列表 |
| `return_target_missing` | `event/error/task` | 开机检查发现缺「返航终点站」，或充电站不在任何路径 5 m 以内。`detail.missing`、`detail.docks_off_route` |

- 公共字段：`soc_pct`、`remain_mile_km`、`target_kind`、`target_name`、`task_id`（回充任务 ID，形如 `lb-<数字>-<数字>`）、`first`。
  - `first=true` 表示本次告警的第一条，机器人只对这一条出声。
- `title` 示例：`low battery, returning to charge`。`code` 按 §4.2 派生，不稳定。

#### 4.3.2 定位失效与恢复（`event/error/alarm`、`event/info/alarm`，`99` U142）

- 规则：`single` 或 `no_fix`（以及无定位）持续 1 s，判定定位失效。
  - 机器人挂起当前任务并停车，暂停派发，拒收新任务（`E_DEGRADED`，2001）。
  - `rtk_fixed / rtk_float / dgps` 照常作业，其中后两者限速 0.5 m/s。
  - 可用定位持续 2 s 判定恢复，被挂起的任务**不自动恢复**，要人工「继续」。
- 失效事件：`detail.type = rtk_lost`。
  - 定位字段：`fix_type`、`heading_source`、`blind_s`、`last_valid_pos`。
  - 任务字段：`task_id`、`action_taken`（`stop_and_suspend` 或 `stop_and_hold`）、`task_progress`。
  - 失效事件有 60 s 去重窗口。
- 恢复事件：`detail.type = heading_recovered`，带 `fix_type`、`lost_s`、`task_id`。
- ⚠️【偏差·待修】`detail.alarm_type` 当前恒为 `system_alarm`。按 v2.0 §5.2 应分别为 `rtk_lost` 和 `heading_recovered`。修正前请先按 `detail.type` 识别。

#### 4.3.3 录制提醒与自动收尾（`event/info/teach`、`event/warn/teach`，`99` U163）

| `detail.kind` | key | 含义 |
|---|---|---|
| `teach.time_warning` | `event/info/teach` | 到时前提醒。`warn_before_s`（600 或 60）、`remaining_s` |
| `teach.expired` | `event/warn/teach` | 到时自动收尾。`detail.result` = `saved`（已保存，带 `geo_id`）/ `save_failed`（保存失败、数据保留，带 `code` 和 `reason`）/ `discarded`（轨迹无效未保存，带 `issues`）/ `awaiting_name`（无名会话，等待命名） |

- 公共字段：`session_id`、`name`、`session_kind`（`route` 或 `fence`）、`channel`（发起录制的通道）、`point_count`、`length_m`、`duration_s`、`saved`、`retained`（数据是否仍保留）、`first`。
- 注意：录制对象是路径还是围栏，看 **`session_kind`**；`detail.kind` 是事件名。

#### 4.3.4 任务事件（`event/{info|warn}/task`）

- `detail.kind` 闭集与发事件的迁移：

  | `detail.kind` | 级别 | 迁移 |
  |---|---|---|
  | `accepted` | info | 受理（入队、定时、等依赖） |
  | `rejected` | warn | 受理前校验失败 |
  | `started` | info | 开始执行 |
  | `completed` | info | 完成 |
  | `failed` | warn | 执行失败、挂起后放弃或需人工处置 |
  | `cancelled` | warn | 取消 |

- 其它字段：
  - `detail.state`：迁移后的机内状态；`detail.task_id`。
  - 终态另带 `task_type`（机内值，§5.2）、`route_id`、`started_ts`、`ended_ts`、`duration_sec`、`total_count`；操作员给过原因时带 `reason`。
  - 失败时另有 `detail.error`：几何下发失败为 `{code, detail, cause}`，导航失败为 `{fail_reason}`。
- ⚠️【偏差·待修】
  - **暂停、被抢占、恢复都不发任务事件**。v2.0 §5.4 要求「暂停、恢复」也发。挂起只能从 `state/task` 快照里看到。
  - 顶层 `task_id` 当前为 null，任务 ID 只在 `detail.task_id` 里。
  - `code` 由标题 `task <task_id> <kind>` 派生，所以夹带了任务 ID，例如 `TASK_T_20261111_003_COMPLETED`。
  - v2.0 §5.4 的写法是 `code=TASK_DONE` 并带顶层 `task_id`。

#### 4.3.5 开机恢复（`event/warn/system`）

非正常断电后重启时，原先运行中的任务改为挂起：`detail.type = unclean_shutdown`，带 `detail.task_id`。该任务不自动恢复。

#### 4.3.6 云端链路（`event/{info|warn|error}/system`）

| `code` | key | 时机 |
|---|---|---|
| `CLOUD_LINK_DEGRADED` | `event/warn/system` | 进入 L1 |
| `CLOUD_LINK_DOWN` | `event/warn/system` | 进入 L2 |
| `CLOUD_LINK_LOST` | `event/error/system` | 进入 L3（同时触发返航，§1.3） |
| `CLOUD_LINK_UP` | `event/info/system` | 恢复到 L0 |

#### 4.3.7 其它类别（原文已有，补充取值）

- 围栏 `event/*/fence`：`detail.kind` 是 `11` §9A.9 的事件名（如 `zone_enter`、`zone_exit`），`code` 是其大写形式。
- 地理对象 `event/*/geo`：`detail.type` ∈ `geo.created / geo.updated / geo.deleted / geo.renamed`（info），或 `geo.conflict / geo.force_overwrite / geo.route_changed / geo.route_deleted / geo.route_remap_failed`（warn）。
- 感知通知：映射成 `fault`，且可能是 `event/info/fault`，例如 `detail.type=perception.reason_changed`。这类事件是状态通知，不是故障，请按 `sev` 区分。
- 网关拒绝事件：`event/warn/task`（任务被拒），或 `event/warn/system`（信封层协议错误）。`code` 为 `detail.code`。

#### 4.3.y 围栏进出与撞线【新增 · `99` U186，2026-11-13】

都在 `event/{sev}/fence`（`channel=alarm`），`detail.kind`：

| `detail.kind`（= `title`） | 何时 | 云端 key 的级别 | 持续 | `detail` 主要字段 |
|---|---|---|---|---|
| `zone_enter` | 进入**报警区域**（已有）或 ★**限速区域**（新增） | 报警区 `error`（机内 `alarm`）· 限速区 `info` | 否 | `role`（`warning` / `speed_limit`）、`poly_id`、`poly_name`、`episode_id`、`speed_limit_mps`（限速区为生效值，报警区为 `null`） |
| `zone_exit` | 离开上述区域 | `info` | 否 | 同上 |
| `contact_start` | 撞上**活动区域**或**禁止区域**的边界（被拦停） | `warn` | — | `role`（`allow` / `forbid`）、`poly_id`、`poly_name`、`episode_id`、`recording`（`null` 自主运行 / `route` 录路径 / `fence` 录围栏）、`d_eff_m`、`rev`、`fix_type` |
| `contact_hold` | 撞线持续，每 3 s 一条（机器人配置项） | `warn` | ★ **是**，直到撞线消失；录围栏时不发 | 同上 |
| `contact_end` | 撞线消失（离开边界 0.5 m，或 2 s 不再撞线） | `info` | — | 同上 |

- 同一次撞线的三类事件 `episode_id` 相同，可据此拼成一次完整过程并算持续时间。
- 机器人同时语音播报（话术见 `99` U186）。

#### 4.3.x 围栏「到不了」上报【新增 · `99` U184，2026-11-13】

- key：`event/warn/fence`（`channel=alarm`，与其它围栏事件同通道），一次检查一条，**不持续**。
- 何时发：启用活动区域 / 禁止区时（`trigger: "enable"`）、录制保存活动区域 / 禁止区或在围栏生效期间存航点 / 路径时（`trigger: "save"`），发现有航点或路径会因此到不了。
- `detail`：

  ```json
  {"kind": "objects_unreachable", "trigger": "enable",
   "poly_name": "北门禁止区域围栏", "role": "forbid",
   "reason": "inside_forbid", "objects": ["1号测试点", "1号路径"]}
  ```
  - `reason`：`inside_forbid`（被禁止区盖住）/ `outside_activity_area`（在活动区域外）。`objects` 为航点与路径名（含充电站、返航终点站）；路径只要有一个点在内就列出。
- 机器人**不拒绝**这些操作，只上报；云端请提示操作员调整围栏或对象。

#### 4.3.z 报警区自主报警【新增 · `99` U188，2026-11-13】

- **何时报**：机器人在执行**路径巡逻或点到点导航**，且**自身在某个已启用的报警区域内**；该区域里出现 `SET_ALARM_CONFIG` 规则所列的目标（缺省只报人），并在报警时间段内（§6.4）。机器人不在报警区内时不判断。目标持续约 1 s 才报，同一区域两次报警至少间隔 `cooldown_sec`。
- **报警事件** `event/error/alarm`（机内 `event/alarm/intrusion`，级别 `alarm` 按 §4.1 映射为 `error`），`detail` 按 v2.0 §5.2 八项**由机器人填好**：

  ```json
  {"alarm_type": "person_in_region", "matched_rule": "person_in_region",
   "zone_hits": ["f-yard"], "track_id": 12, "dwell_s": null,
   "wpos": {"latitude": 34.69712, "longitude": 135.50531, "altitude_m": null, "valid": true, "sigma_m": 0.02},
   "fence_set_id": "fs-site", "rev": 7,
   "zone_id": "f-yard", "zone_name": "北门报警区域",
   "targets": [{"track_id": 12, "cls": "person", "group": "person", "conf": 0.91,
                "d_inside_m": 3.4, "lat": 34.69712, "lon": 135.50531}],
   "rules_source": "cloud", "window": {"start": "22:00", "end": "05:00"},
   "alarm_level": 1, "siren_level": 70, "duration_sec": 5.0, "cooldown_sec": 30.0,
   "heading_level": 1, "fix_type": "rtk_fixed", "deterrent": "started"}
  ```
  - `alarm_type`：`person_in_region` / `vehicle_in_region` / ★ **新增 `animal_in_region`**（取编号最小的那个目标所属的类）。区里同时有多个目标时只报一条，`targets` 全部列出；`track_id` / `wpos` 指编号最小的那个。
  - `wpos` 由机器人位置与航向推算，误差约 0.3 m（1 m/s 时）；`dwell_s` 不提供，恒为 `null`。
  - `deterrent`（声光驱离的执行结果）：`started` 已响警笛 / `busy` 上一次驱离还在进行 / `not_idle` 机器人正处于对讲、喊话或驱离模式，未自动驱离 / `refused` 三合一设备拒绝或离线（★ 设备送修期间恒为 `refused`）。
- **驱离动作**：警笛 + 红蓝灯持续 `duration_sec` 秒，然后喊一次「您已进入管制区域，请立即离开」；**机器人不停车**，继续巡逻 / 导航（用户裁决）。
- **区域判定暂停 / 恢复**：巡逻或导航中、在报警区内，定位（非 RTK 固定 / 浮点）或航向失效时 `event/warn/alarm` `title = "zone_rules_suspended"`，恢复（或任务结束）时 `event/info/alarm` `title = "zone_rules_resumed"`；`detail.reason` 为 `position_invalid` / `heading_invalid`。各一条，不持续。
- **时钟未同步**：报警时间段按全天判，发一次 `event/warn/alarm` `title = "alarm_clock_unsynced"`（同步恢复后再失步会再发一次）。
- 以上几类的机内类别都是 `intrusion`，上云统一映射为 v2.0 `alarm` 类（§4.1）；`code` 由 `title` 派生：`ZONE_INTRUSION` / `ZONE_RULES_SUSPENDED` / `ZONE_RULES_RESUMED` / `ALARM_CLOCK_UNSYNCED`。后三类不带八项字段，网关补 `null`，`alarm_type` 补为 `system_alarm`。

### 4.4 断线补发【偏差·待修】

v2.0 §9.5 要求可靠事件断线后在原 key 上补发、复用 `eid`。**目前机器人没有在 v2.0 事件 key 上补发**：

- 内部的补发通道是裸 key `event/replay/{normal|alarm}`，配套确认是 `event/ack`、`event/recon/*`，**不属于本契约**，请不要接入。
- Qt 离线期间产生的事件，在 v2.0 key 上收不到。
- 网关拒绝事件和 2 s 超时拒绝，本身就不进入事件库。

在修正之前，Qt 不要以「断线期间零事件」作为「机器人一切正常」的依据。

---

## 5. 状态 key【澄清 / 偏差】

### 5.1 `state/robot`

| 字段 | 当前实际 |
|---|---|
| `robot_state` | 只会出现 `emergency_stop`（硬件急停或急停锁存）、`fault`（底盘 fatal 故障）、`running`（有任务在跑）、`idle`。⚠️【偏差·待修】**不会出现 `charging`**，充电状态请看 §4.3.1 的事件；`offline` 也不会出现，离线以 `state/link` 为准 |
| `task_state` | 只会出现 `running` 或 `idle` |
| `gps.heading_deg` | ⚠️【澄清】**ENU 约定：正东为 0°，逆时针为正**（0～360）。不是罗盘方位（正北 0°、顺时针）。航向无效时为 null |
| `gps.fix` | 机内 `single` → `gps`，`no_fix` → `none`，其余同名 |
| `gps.speed_mps` | 无定位时为 null |
| `motion.linear_speed_mps` | 有定位时取 GNSS 速度，否则取底盘里程计，都没有则为 null |
| `battery` | `soc` = 在位电池里较小的电量；`voltage_v` = 在位电池最低电压；`temperature_c` = 在位电池最高温度；`current_a` 恒为 null（底盘不报）。底盘离线 5 s 后整段为 null |
| `devices[]` | 只列机器人实际在报的设备。`id` 当前取值：`cam_rgbd`、`chassis`、`mic`、`payload_light`、`payload_siren`、`payload_speaker`、`payload_strobe`、`ptz`、`rtk`。⚠️ `name` 目前与 `id` 相同，不是中文显示名 |
| `storage` | 机器人数据盘，十进制 GB，保留一位小数 |
| `clock` | `ts_sync` 取自机器人授时判定；`source` 为授时来源（当前实发 `ntp`）；`age_ms` 当前为 null；源断 5 s 后整段为 null |
| `alarm_window_active` | ⚠️【偏差·待修】恒为 `false`（报警时间窗未实现，见 §6.4） |

### 5.2 `state/task`

- `task_type`【澄清】：快照与终态里都是**机内类型**，不是 v2.0 的 wire 名。取值为 `goto`、`patrol`、`return_home`、`charge`、`standby`、`teach`、`follow`。云端自己下发的 `GOTO_KEYPOINT` 在这里也显示为 `goto`。
- `task_id` 格式【澄清】：

  | 来源 | 格式 |
  |---|---|
  | 云端下发 | 原样保留 |
  | 语音 / 文本 / HMI | `t-YYYYMMDD-NNN` |
  | 低电回充 | `lb-<数字>-<数字>` |
  | 断链返航 | `rh-<数字>-<数字>` |

  所有任务都会出现在快照里，云端都能用 `STOP_TASK` 控制。
- 快照字段的当前取值【偏差·待修】：
  - `completed_count` 恒为 0；
  - `progress_percent`、`route_rev`、`current_waypoint_id` 恒为 null；
  - `total_count` 是 waypoint 个数；
  - 排队任务的 `route_id` 为 null。
- `suspended` 列表不带挂起原因。挂起可能来自：
  - 操作员暂停；
  - 急停；
  - 定位失效；
  - 低电抢占；
  - 断链返航抢占；
  - 喊话让行（§6.5）；
  - 开机恢复。

  挂起本身不发任务事件（§4.3.4）。原因只能结合同时刻的其它事件推断：
  - 低电事件（§4.3.1）；
  - 定位失效事件（§4.3.2）；
  - 开机恢复事件（§4.3.5）；
  - 断链返航任务的出现（§1.3）；
  - `state/mode` 进入喊话。
- 终态 result【偏差·待修】：
  - `failed` 的 `result_code` 一律为 2001（机内没记失败码），具体原因看 `reason` 和对应任务事件的 `detail.error`；
  - `cancelled` 为 0；
  - `distance_m` 为 null；
  - `completed_count` / `total_count` 可能为 0 / 0；
  - `duration_sec`、`started_ts`、`ended_ts` 在跨重启等情况下可能为 null。

### 5.3 `state/link`

见 §1.2。`estop_path` 是机器人的急停链路自检结果，与云端是否在线无关。

### 5.4 `state/geo/manifest`

- waypoint：
  - `geo_id` 形如 `w-1`、`w-1_2`；`altitude` 目前为 null。
  - 名字含「返航终点站」的航点就是断链返航和低电返航的目的地；名字含「充电站 / 充电桩」的航点是充电点（`99` U119 / U128）。
- recorded_path：录制保存的路径（§3.5）。
- ★★【变更 · `99` U183，2026-11-13】**围栏按实际类型上报，未启用的也上报**：

  | `type` | 围栏 | `geo_id` | 附加字段 |
  |---|---|---|---|
  | `keep_in` | 活动区域（只许在内，最多 1 个，可以没有） | `f-…` | `enabled` |
  | `forbid_region` | 禁止区域（禁止进入） | `f-…` | `enabled` |
  | `speed_limit_region` | 限速区域（区内限速） | `f-…` | `enabled`、`speed_limit_mps`（区内限速，米/秒） |
  | `alarm_region` | 报警区域（不限制运动） | `f-…` | `enabled` |

  - `enabled`：`true` = 已启用、机器人正在执行；`false` = 已录好但未启用（或已被关闭）。未启用的也会出现，云端据此才能找到要去启用的围栏。
  - ⚠️★★★ **兼容性（必须先改云端再升级机器人）**：v2.0 把 `type` 越界列为整条拒收的条件。云端解析器认得上表四种 `type` 之前，机器人一旦发出 `keep_in` / `forbid_region` / `speed_limit_region`，**整份清单（含航点、路径）都会被拒收**。请先让云端支持新类型，再升级机器人软件。
  - 围栏数量不再有上限（U183）。
  - ~~`alarm_region` 当前包含**所有已启用的围栏**，不论它是报警区还是安全围栏（只许在内、禁止进入、限速），而且 `enabled` 恒为 `true`。未启用的围栏不出现。~~ —— 被替代的表述（~ 2026-11-13），偏差 D-6 的后半句已修。请不要用 `SET_ALARM_CONFIG` 去修改或删除非报警用途的围栏：禁止区、限速区、活动区域用 `SET_FENCE_CONFIG`（§6.4A）。~~（安全围栏的云端接口见后续版本）~~
- ⚠️【偏差·待修】
  - `manifest_rev` **恒为 0**，不递增。在修正之前，请按 `objects` 内容判断清单是否变化。

### 5.5 `state/media`【澄清】

当前实发 4 个端点，都在甲方 188 自组网上（`99` U92）。

| `id` | `stream` | `url` | `max_bitrate_mbps` |
|---|---|---|---|
| `cam_ptz_vis` | `main` | `rtsp://192.168.188.200:554/media/video1` | 6.2 |
| `cam_ptz_vis` | `sub` | `rtsp://192.168.188.200:554/media/video2` | 2.0 |
| `cam_ptz_ir` | `main` | `rtsp://192.168.188.200:8554/cam/realmonitor?channel=1&subtype=0` | 2.5 |
| `cam_ptz_ir` | `sub` | `rtsp://192.168.188.200:8554/cam/realmonitor?channel=1&subtype=1` | 0.5 |

- `credential_ref` 分别为 `robot_gj001_ptz_vis`、`robot_gj001_ptz_ir`。
- `cam_rgbd` 当前不发。
- v2.0 §4.6 里的 `192.168.123.x` 地址与 Qt 出口 `192.168.123.60` 是旧网段，以本表为准。

### 5.6 其它

- `data/file/index`：`files` 恒为空数组（文件交付面未建）。
- `state/audio.recording=true`：表示机器人**本机语音识别麦克风**打开，不是 `dog_to_pc` 上行。

---

## 6. Qt 下发任务 `cmd/task` 的当前行为

### 6.1 `GOTO_KEYPOINT`【变更 · 2026-11-11 已修正（`99` U164）】

- 网关校验按原文实现：`coordinate_system`、`r-` / `w-` 正则、`arrival_radius_m` 0.5～10、拒绝航点级 `recorded_path_id`。
- **执行语义**（用户裁决）：
  - `waypoints[]` **按顺序串成一条线，一次走完**：机器人当前位置 → 第 1 点 → 第 2 点 … → 末点。
  - 每一段**只沿 `recorded_path_id` 这一条路径**规划：就近进入路径，沿路径走到离该航点最近处，再直走过去。航点离该路径超过 5 m 时，这一段直走。
  - 中间航点经过、不停留；**到达按末点的 `arrival_radius_m` 判定**。
  - 航点坐标以机器人 manifest 里的坐标为准，计划里回显的 `latitude / longitude` 只作记录。
  - 暂停后恢复，会从当时位置重新走整条线：已经经过的航点会再经过一次。
- **提交时即检查**，不通过直接拒绝，不入库：

  | 情形 | `error_code` | `detail.code` | 定位字段 |
  |---|---|---|---|
  | 路径不存在、已删或未启用 | 1003 | `E_NOT_FOUND` | `field="recorded_path_id"`、`recorded_path_id` |
  | 第 i 个航点不存在 | 1003 | `E_NOT_FOUND` | `field="waypoints[i]"`、`index`、`waypoint_id` |
  | 航点在只许在内的围栏外，或在禁止进入的围栏内 | 2006 | `E_OUT_OF_FENCE` | `field`、`index`、`waypoint_id`、`subject="target"` |

- `task_id` 原样作为机器人内部任务 ID。同一 `task_id` 换 `msg_id` 重发，回 `duplicate`，并且是持久的。
- `state/task` 里的 `route_id` 是 `recorded_path_id`。
- 原文没写、但会出现的拒绝（对所有通道生效）：

  | 情形 | `error_code` | `detail.code` | `detail.reason` |
  |---|---|---|---|
  | 电量 ≤ 5 % | 2005 | `E_LOW_BATTERY` | `battery_critical` |
  | 定位失效 | 2001 | `E_DEGRADED` | — |
  | 录制中 | 2002 | `E_BUSY` | `teach_session_active` |

### 6.2 `STOP_TASK`【变更】

- **会停车**（用户裁决 U141）。running 的任务被 `pause` 或 `cancel` 后，机器人立即撤销路线并停下。原文「STOP_TASK 不发布零速度」作废，但安全停车仍请用 `ESTOP`。
- `pause` 之后整个调度冻结（U111），排队任务不会自动开始，直到有人 `resume` 或下发新任务。
- 状态合法性：
  - `pause` 只对 running 有效，`resume` 只对 suspended 有效，否则回 `rejected`、2003、`E_TASK_STATE`，**不是 `duplicate`**。
  - 只有对已终结的任务发 `cancel` 才回 `duplicate`。
- 目标不存在：回 `rejected`、**1003**、`detail.code=E_NOT_FOUND`。
- `target_task_id` 可以是任何来源的任务（§5.2）。

### 6.3 `ESTOP`【澄清 / 变更】

- ack 的 `detail` 七项来自底盘的真实回执；`hes` 取 `ok` 或 `engaged`。
  - 100 ms 内没收到真实回执时，回兜底 ack：`applied: []`、`hes: "unknown"`，`estop_epoch` 为网关自己的计数。
- 机器人不对急停去重：每次重发都回 `accepted`，`estop_epoch` 递增，**不会回 `duplicate`**。
- `payload.action` 写错也按 `stop` 执行，停车优先。
- ⚠️ **解除急停**（U149）：
  - 没有专用 key。
  - 云端发 `STOP_TASK resume`，或发新的 `GOTO_KEYPOINT`，都视为「人工恢复」：解冻调度，并在下一次路线下发时**解除机器人的软急停锁存**，机器人随即开动。
  - 请在 Qt 上对这两个操作做明确提示。
- 急停**不停云台**，急停期间云台照常可控（PTZ 契约 U-2）。

### 6.4 `SET_ALARM_CONFIG`【变更 · `99` U187，2026-11-13】

- 每个 `regions[]` 区域写成机器人的报警区围栏；`enabled=true` 时同时启用（同前）。
- ~~`alarm_level`、`siren_level`、`duration_sec`、`cooldown_sec`、`alarm_window`、`rules[]`、区域的 `applies_to` **只做格式校验，不生效**。~~ —— 被替代（~ 2026-11-13）。★ U187 现行：除 `applies_to` 外，这些字段**存到机器人上**（重启后仍在），供巡逻 / 点到点导航途中的自主报警使用。
- ★ **部分更新**：报文里**出现的字段**才改；**没出现的字段保持机器人上原值不变**；字段值为 `null` 表示**删除**，机器人回到默认。`regions[]` 可以不带（只改时间段或规则时）。
  - 例：只改报警时间段，发 `{"alarm_window": {"start": "22:00", "end": "05:00"}}`；删除时间段（回到全天），发 `{"alarm_window": null}`。

| 字段 | 机器人怎么用 | 没设置 / 删除后的默认 |
|---|---|---|
| `alarm_window` `{start, end}`（`HH:mm`） | 每天的报警有效时间段。★ 允许跨零点（`start` 晚于 `end`，如 22:00–05:00）。全机**只有一个**时间段，对所有报警区域生效。机器人时钟未同步时按**全天**处理 | 全天 |
| `rules[]` | 要报警的可疑目标。按**整表**下发：增删目标就发新的完整列表，新表**替换**旧表。`type` 取值见下；`region_ids` 为空表示**所有报警区域**；`enabled=false` 的规则不报 | 只报**人**，所有报警区域 |
| `alarm_level` | 1 / 2，存储并随报警事件上报 | 机器人侧默认 |
| `siren_level` | 0–100，警报音量 | 机器人侧默认 |
| `duration_sec` | 1–20，一次警报持续秒数 | 机器人侧默认 |
| `cooldown_sec` | 0.5–600，同一区域两次报警的最短间隔 | 机器人侧默认 |

- `rules[].type` 取值：`person_in_region`（人；★ 行人、骑自行车、骑摩托车都算人）、`vehicle_in_region`（车）、★ **新增 `animal_in_region`**（动物：家畜、大型野生动物，**鸟类不报**；能否报出取决于感知模型是否识别出该目标）。其它取值回 1003。
- `rules[].alarm_role` 只支持 `include`；`exclude`（排除规则）机器人**未实现**，回 3001、`E_CONFIG_INVALID`，不会被当成 `include` 存下。
- 区域的 `applies_to`：**只做格式校验，不生效** —— 报什么目标以 `rules[]` 为准。
- 报文里既没有 `regions[]`、也没有任何上表字段时整条拒绝：1003，`E_SCHEMA`。~~`regions[]` 为空时整条拒绝：1006，`E_NOT_IMPLEMENTED`。~~（被替代，~ 2026-11-13）
- 只改上表字段、不改区域时，机器人执行的围栏不变，受理后**立即**回 `done`。
- ★ U188：这些设置用于报警区自主报警，报警事件与驱离行为见 §4.3.z。`alarm_level` 只随事件上报，不改变机器人动作；`siren_level`（0–100）按比例设警笛幅度，未设置时用机器人调好的值。
- 终态 result：
  - 机器人生效围栏版本前进即回 `done`，`detail` 为空（没有 `active_rev`、`played_times`、`stop_reason`）。
  - 6 s 内未前进回 `failed`、2002。
  - ~~推断（未实测）：所有区域都 `enabled=false` 时版本不会前进，结果会是 `failed`。~~ ★ U185（2026-11-13）已修：若这条配置**没有改变机器人正在执行的围栏**（例如区域都存为未启用、或重发相同内容），受理后**立即**回 `done`，不再等 6 s 后回 `failed`；改变了才等生效版本前进。
- 版本冲突回 **2004**、`E_GEO_CONFLICT`，与原文示例的 1003 不同。
  - `detail` 为 `{geo_id, rev, content_hash, updated_by, updated_ts}`，没有原文的 `region_id / base_rev / current_rev`。
- `keep_in` 区域回 3001、`E_CONFIG_INVALID`。
- ~~★ U176（2026-11-12）：`enabled=true` 的报警区域，只有机器人的活动区域围栏**已启用**且该区域整圈在活动区域内时才会启用；否则该区域启用被拒……~~ —— 被替代（2026-11-12 ~ 2026-11-13）。★ U183 / U184 现行：报警区域启用**不因位置被拒**；没有活动区域时照常生效。
- ★ U184：新建 / 修改的报警区域若与机器人上的**禁止区**有共同面积，该区域被拒（2006，`E_GEO_INVALID`，`detail = {reason: "forbid_overlap", fences: [...]}`），整条按配置事务回失败。

### 6.4A `SET_FENCE_CONFIG` 安全围栏配置【新增 · `99` U185，2026-11-13】

v2.0 §3.4 写明「营区 keep-in 必须走独立安全围栏接口」，本任务就是这个接口。报警区仍用 `SET_ALARM_CONFIG`（§6.4）。

**用途**：
- 下发、修改、删除、启用、关闭**禁止区域**与**限速区域**（含机器人现场录制的）；
- **活动区域**（`keep_in`）只能**启用 / 关闭**，不能下发、修改、删除（它只能在现场录制）；
- 设置 / 删除限速区域的**统一限速值**。

**payload**（与 `SET_ALARM_CONFIG` 的 `regions[]` 同一套增量操作）：

```json
{
  "regions": [
    {"op": "upsert", "type": "forbid_region", "id": "f-north_pit", "base_rev": 0,
     "name": "北门禁止区域", "enabled": true,
     "vertices": [{"latitude": 31.2301, "longitude": 121.4732},
                  {"latitude": 31.2305, "longitude": 121.4732},
                  {"latitude": 31.2305, "longitude": 121.4738}]},
    {"op": "upsert", "type": "speed_limit_region", "id": "f-yard", "base_rev": 0,
     "name": "车场限速区域", "enabled": true, "vertices": [ ... ],
     "speed_limit_mps": 0.5},
    {"op": "set_state", "type": "keep_in", "id": "f-camp", "enabled": true},
    {"op": "delete", "type": "forbid_region", "id": "f-old", "base_rev": 3}
  ],
  "speed_limit_default_mps": 0.8
}
```

| 字段 | 说明 |
|---|---|
| `regions[].type` | `forbid_region`（禁止区域）· `speed_limit_region`（限速区域）· `keep_in`（活动区域，只允许 `op=set_state`）。`alarm_region` 不在这里（走 `SET_ALARM_CONFIG`） |
| `regions[].op` | `upsert` 新建或整体替换（`enabled=true` 时同时启用）· `delete` 删除 · `set_state` 启用（`enabled=true`）/ 关闭（`enabled=false`）。★ **关闭只有云端能做**（语音只能启用） |
| `regions[].id` | `f-[a-z0-9_]{1,40}`；现场录制的围栏 id 从 `state/geo/manifest` 取 |
| `regions[].base_rev` | `upsert` / `delete` 必填，与 manifest 里的 `rev` 一致，否则 2004 `E_GEO_CONFLICT` |
| `regions[].enabled` | `upsert` / `set_state` 必填，布尔 |
| `regions[].vertices` | `upsert` 必填，至少 3 个顶点 |
| `regions[].speed_limit_mps` | 可选，只能出现在 `speed_limit_region` 上：**这个区自己的限速**（米/秒）。不带 ⇒ 这个区随统一限速值 |
| `speed_limit_default_mps` | 可选。**统一限速值**：带数值 ⇒ 设置（所有没有自带限速的限速区随之改变，含现场录制的）；带 `null` ⇒ 删除，回到机器人缺省 **0.6 m/s**；不带 ⇒ 不变。机器人持久化，重启后仍有效 |

**限速值取值顺序**（高者优先）：区自带的 `speed_limit_mps` → 云端统一值 `speed_limit_default_mps` → 机器人缺省 0.6 m/s。上报的 manifest（§5.4）里，限速区的 `speed_limit_mps` 是**当前生效的值**。

**拒绝**：
- 结构错误（缺字段、类型不对、`regions` 和 `speed_limit_default_mps` 都没带）：1002 / 1003，`E_SCHEMA`。
- 对 `keep_in` 做 `upsert` / `delete`：3001，`E_CONFIG_INVALID`。
- 限速值超出机器人允许范围（当前 0.1 ~ 2.0 m/s）：2006，`E_GEO_INVALID`，`detail.reason = "speed_limit_out_of_range"`，带 `min` / `max`。
- 禁止区与报警区 / 限速区 / 另一禁止区重叠（U184）：2006，`E_GEO_INVALID`，`detail = {reason: "forbid_overlap", fences: [...]}`。
- 几何不合格（自相交、面积过小等）：2006。
- 一条命令里任何一项被拒，整条回失败（配置事务，全成才成）；各项幂等，整条重发是安全的。

**终态 result**：受理后，若这条命令改变了机器人正在执行的围栏，等机器人生效版本（`state/fence.active.rev`）前进再回 `done`，6 s 内未前进回 `failed`；没有改变（例如新建一个未启用的区域、或只改了没有被启用的限速区的统一值）则立即回 `done`。

**机器人的提示**：启用活动区域 / 禁止区时，若有航点或路径因此到不了，机器人另发 §4.3.x 的 `objects_unreachable` 事件。

### 6.5 `AUDIO_CONTROL`【澄清】

- `stream_id` 格式为 `audio-<rid 去掉短横>-<4 位序号>`。⚠️ 机器人音频进程重启后序号从 0001 重新开始。
- `exit_broadcast` 的 `stream_id` 不符或会话不存在：回 1003、`E_NOT_FOUND`，`detail` 带 `stream_id` 和 `current_stream_id`。
  - 已退出后再发：回 `accepted`、`changed: false`。
- 空闲超时 300 s：从最后一帧放行的音频算起，**不设最长时长**（U97）。
- `exit_reason` 取值：
  - 带 `stream_id` 退出：`requested`；
  - 不带 `stream_id` 的云端退出：`manual_cloud`；
  - 另有 `timeout`、`fault`。
- ⚠️ **进入喊话会让正在执行的巡逻、去航点、返航任务挂起**（快照中为 `paused`），退出后自动恢复（U99）。
- 【偏差·待修】同一 `msg_id` 重发 `exit_broadcast` 时，`duplicate` ack 不回显 `stream_id`。
- 2 s 内没有结果：回 2002、`E_TIMEOUT`，不带 `stream_id`。

### 6.6 禁止项

| 请求 | 回执 |
|---|---|
| `MANUAL_VELOCITY` | 1006、`E_CHANNEL_DENIED` |
| 旧任务名、未知或缺失的 `task_type` | 1006、`E_TASK_UNSUPPORTED` |
| `dog_to_pc` | 1006、`E_CHANNEL_DENIED` |

- `confirm_token`、`auto_off` 等多余字段被忽略，不拒绝。
- `xbrain/{rid}/cmd/teleop` 没有订阅者，也**不产生**原文所说的 `event/warn/system`。云端键盘请用 §2 的通道。

### 6.7 ack 与错误码的实际映射【澄清】

`detail.code` 放的是机器人原生码，与原文 §10 的「典型 detail.code」不完全相同。实现见 `xbrain/p5_gateway/outbound/error_map.py`。

| `error_code` | 实际的 `detail.code` |
|---:|---|
| 1001 / 1002 / 1004 | `E_SCHEMA`（信封层错误；原文写的是 `E_JSON_PARSE` / `E_REQUIRED_FIELD` / `E_RID_MISMATCH`） |
| 1003 | `E_SCHEMA`、`E_NOT_FOUND`、`E_QOS_VIOLATION` |
| 1005 | `E_PROTO_VERSION` |
| 1006 | `E_NOT_IMPLEMENTED`、`E_CHANNEL_DENIED`、`E_TASK_UNSUPPORTED` |
| 2001 | `E_UNHEALTHY`、`E_DEGRADED`、`E_NO_HEADING`、`E_SAFETY_LINK_LOST`、`E_TELEOP_STALE` |
| 2002 | `E_BUSY`、`E_TEACH_BUSY`、`E_CAPABILITY`、`E_ARB_*`、**`E_TIMEOUT`** |
| 2003 | `E_TASK_STATE`、`E_TEACH_STATE`、`E_STATUS`、`E_CONFIRM_REQUIRED`、`E_CONFIG_LOCKED` |
| 2004 | `E_NAME_CONFLICT`、**`E_GEO_CONFLICT`**、`E_DUPLICATE` |
| 2005 | `E_LOW_BATTERY` |
| 2006 | `E_OUT_OF_FENCE`、`E_FENCE_INVALID`、`E_GEO_INVALID`、`E_GEO_INCOMPLETE`、`E_TEACH_GEOMETRY`、`E_TEACH_QUALITY` |
| 2007 | `E_LOCKED`（原文的 `E_ESTOP_ACTIVE` 不存在） |
| 3001 | `E_CONFIG_INVALID`、`E_STORAGE_CORRUPT`、`E_INTERNAL` |

- 机器人 2 s 内没有给出结果时，回 `rejected`、2002、`detail.code=E_TIMEOUT`、`reason="backend did not answer within 2s"`。
  - ⚠️ 此时命令可能**已经**送到机器人内部；迟到的结果会被丢弃。推断：可能出现 Qt 收到拒绝、任务实际已创建的情况。超时后请先看 `state/task`，再决定是否重发。
- ⚠️【偏差·待修】`msg_id` 在执行**之前**就登记进去重窗口。被拒（包括超时）的请求在 60 s 内用同一 `msg_id` 重发，会得到 `duplicate` 而不会重新执行。在修正之前，被拒后要重新执行时，请换新的 `msg_id`（`task_id` 可以不变）。
- 网关产生的拒绝会另发 `event/warn/task`，但不进入事件库，断线不补发；2 s 超时拒绝不发事件。

---

## 7. 云台 PTZ（草案 v0.2）

原稿 §9 的实现现状仍然成立，key、动作、`hold_ms` 600～2000、8 条/秒、错误码、事件码都没变。两处补充：

- 【新增】云端操作云台后，机器人持有云台域 **30 s**，期间自动跟踪等低优先级来源拿不到云台。
- 【澄清】`E_BUSY`（2002）的 `detail` 为 `{domain, holder, holder_priority, gen}`。`holder` 是仲裁源 ID，例如 `manual_cloud`。
- 拒绝事件的类别是 `task`；`PTZ_DEADMAN_STOP` 和 `PTZ_PREEMPTED` 的类别是 `payload`。

---

## 8. 云端语音文本交互契约 v1.0 的变更

以下对照基线文档⑤的章节号。§1～§4（连接、外层、报文、音频格式、组句上限与拒收码）**没有变化**。

### 8.1 链路计时【新增】

见 §1.1：语音文本流量**不算**云端在线。只用语音文本时，也必须同时发 `heartbeat/qt`。

### 8.2 二次确认（§6.3）已取消【变更 · `99` U147，2026-11-06】

- 用户裁决「语音、文本指令一律一问一答」。原 L2 指令现在**当轮执行**，例如：
  - 取消任务；
  - 删除航点、删除路径、删除围栏；
  - 启用围栏。

  `auth_level` 报 `L1a`；语音文本上不再出现 `L2`。
- 单独说「是 / 确认 / 算了」，回复「现在没有需要确认的操作」，`verdict=accepted`，什么都不执行。
- `pending_confirm` 只剩 L3 指令会触发（「跟着他」、带内容的「喊一句…」），回复「该操作需要云端审批,已上报等待批准」。
  - ⚠️ 但**实际没有上报任何审批请求**，之后云端回复什么都不会执行。请把它当作「未执行」展示，**不要做确认弹窗**。
- 原「反问」也取消了。参数不全时直接说明并不执行，例如「没有说走多远, 没有执行. 请带上米数说, 例如: 前进3米」。
  - 此时 `verdict=accepted`、`intent` 有值、`slots={}`，不下发动作。

### 8.3 `verdict` 与字段的实际取值【澄清 · v1.0 §5.2 / §5.3 有误】

- 大模型也判不出的句子：`verdict=rejected`（v1.0 写的是 `unknown`），`layer=unknown`。机器人出声「没太听清,请再说一遍」，但 `reply_text` 为 **null**。
- `verdict=unknown` 现在只表示旁听（`layer=overheard`）。
- 多数拒绝的 `reject_code` 为 null，原因只在 `reply_text` 里。目前只有 `E_INTERNAL` 和 `E_CHANNEL_DENIED` 带码。
  - 后者对 `channel=cloud` 实际不会出现。
- `layer` 实际取值：`long_phrase`、`large_class`、`overheard`、`unknown`、`session_context`、`payload_limits`、`ptp_rule`、`record_rule`、`teach_result`、null。`session_state` 已不会出现。
- `slots` 只在运动类指令里填写；去航点、巡逻等的参数不在 `voice_turn` 里。
- `auth_level` 只在下发动作的轮次里有值；查询、闲聊、录制拦截、L3 都是 null。
- `latency_ms`：云端语音仍然只有 `{"asr"}`；本机麦克风的记录带 `{"post","route"}`。
- `reply_kind="stream"` 表示机器人**发出了**播报请求。播报可能被更高优先级的声音（喊话、报警）抢占，不保证真的出声。
- `cmd/voice_text` 另外接受一个可选字段 `slots`（对象），只用于通道准入，一般不用填。

### 8.4 录制期间的回复【新增 · 见 §3.2】

被拦截的指令回 `verdict=rejected`，`reject_code` 为 null，`reply_text` 说明原因（U165；~~此前为 `accepted`~~）。录制相关的其它拒绝（缺名、重名、已在录制、名字不是正在录的、没有在录制）同样是 `rejected`；只有录制状态问答（「录了几个点」）是 `accepted`。录制中的语音急停回 `verdict=rejected`、`bypass` 非空。

### 8.5 一轮两条 / 无请求的 `voice_turn`【新增 · v1.0 §5.1「每轮一条」不再成立】

- **录制开始的结果**（U165）：说「开始录制X」且通过了规则检查的那一轮照常发一条（`verdict` 只表示指令已发出），机器人跑完开始检查后**再发一条** `layer="teach_result"`：成功 `accepted`「已开始录制X」，被拒 `rejected` 加原因码与原因句。**以这条为准开放键盘。**
- **录制结束或取消的结果**（U162）：
  - 说「退出录制 / 取消录制」的那一轮照常发一条，`verdict` 只表示指令已发出。
  - 机器人处理完后**再发一条**：新 `turn_id`，`layer="teach_result"`，`verdict` 为 `accepted`（已保存、已丢弃）或 `rejected`（未保存，`reject_code` 为原因码），`reply_text` 为结果句。
  - 第二条照抄原轮的 `channel`、`session`、`asr_raw`、`asr_text`、`intent`，没有请求 ID，请按 `asr_text` 和时间顺序对应。
- **打点的结果**（U167，2026-11-12）：说「保存当前位置为X」的那一轮照常发一条（`verdict` 只表示指令已发出，`reply_text`「好的,记录这个点」）；机器人处理完后**再发一条** `layer="teach_result"`、`intent="record_waypoint"`：存下 `accepted`「已记录X」，被拒 `rejected` 加原因码与原因句（如 `E_GEO_INVALID`：★ U169 起为「X没有保存: 这里离最近的路径有13.5米, 充电站和充电桩必须在路径5米以内. 请走到路径旁边再说一次」，没有任何路径时「X没有保存: 还没有任何路径, 充电站和充电桩必须建在路径旁边. 请先录制路径」；~~「X没有记录, 充电桩和充电站必须在某条路径5米以内」~~）。没说名字、名字长度不对时该轮直接 `rejected`，不再有第二条。新航点 ID 不在记录里，以 `state/geo/manifest` 同名 waypoint 的 `geo_id` 为准。**以第二条为准显示「创建成功」。** 录制期间说这句仍被拦截（§8.4）。
- ★ **其余指令的结果**（U169，2026-11-12）：机器人对指令结果说的每一句话都**再发一条** `layer="command_result"`：执行端拒绝（如「继续」被 `E_TASK_STATE` 拒、动作被 `E_BUSY` 拒、删除被拒）为 `rejected` 加原因码与原因句；「取消所有任务」的结果为 `accepted`「已取消N个排队中的任务」；任务受理后执行失败为 `rejected`「X没有完成: 原因」（原因码取事件 `detail.error.code`）。字段照抄原轮，新 `turn_id`。云端文本指令（不要求播报）也会收到这条。**以这条为准显示成败。** 受理后无话可说的指令不发第二条。
- **录制到时提醒与自动收尾**（U163）：没有对应请求，机器人主动发。
  - `asr_raw = asr_text = ""`、`intent = null`、`layer = "teach_result"`；
  - `channel` 是发起录制的通道（云端发起为 `cloud`）；
  - `verdict`：提醒与已保存为 `accepted`，其余为 `rejected`。
- 云端请按 `turn_id` 去重展示。

### 8.6 §8.1 运动上限与 §10 测试期说明的现状【变更】

- 单条相对运动上限：网关层仍是距离 ≤ 20 m、角度 ≤ 720°。超限时机器人拒绝，原因只出声。
- 「执行端只执行 ≤ 3 m / ≤ 180°」**只在基本动作测试模式**（`scripts/start_basic_motion.sh`）下成立；正式启动方式下没有这一条。
- §10.1 的「前后 0.2、横移 0.15、转向 0.3」**作废**。现为语音可调的设定值，见 §2.3 的表。
- §10.2 解锁：云端仍**没有**解锁入口。正式的解锁只在本地 HMI，带二次确认（U159）。
- §10.3「站起来后自动切到导航模式」仍然成立，而且不限于测试模式。

### 8.7 只出声、不进 `voice_turn` 的内容【澄清】

以下内容机器人只播报，**不产生 `voice_turn`**：

- ~~动作被执行端拒绝或中止（超范围、底盘锁定、正在录制等）；~~
- ~~云端下的任务执行失败（U140，只在发起那一轮出声时播报）；~~
- 低电告警、定位失效 / 恢复的首次播报（不是指令的回执，看 §4 的事件）。

~~需要这些信息时，请看 `state/task` 与 §4 的事件。v1.0 §9.1 末行「后续拒绝没有回报给云端」的情况仍然存在。~~ —— **被替代的表述**（~ 2026-11-12）：U169 起前两项都另发 `layer="command_result"` 的记录（§8.5），v1.0 §9.1「后续拒绝没有回报给云端」已解决。

### 8.8 其它

- 「正在查询」过渡句**没有实现**，查询类一轮只有一条答复。
- 急停「包含即触发」有一个例外，v1.0 没写：「停止跟踪 / 停止扫描 / 停止环视」走云台指令，**不急停**。
- 机器人的处理队列上限 16，由对话轮次和内部播报作业共用。积压时新的一轮会被丢弃，不产生 `voice_turn`。

---

## 9. 我方已知偏差清单（待修，修完逐条通知）

| # | 偏差 | 影响 | 见 |
|---|---|---|---|
| ~~D-1~~ | ~~`GOTO_KEYPOINT` 按路径整条巡逻，不去 `waypoints[]`，`arrival_radius_m` 不生效；提交时不查路径和航点是否存在~~ | ✅ **2026-11-11 已修正**（`99` U164） | §6.1 |
| D-2 | 定位失效 / 恢复事件的 `alarm_type` 恒为 `system_alarm` | 中 | §4.3.2 |
| D-3 | 任务事件的顶层 `task_id` 为 null，`code` 夹带任务 ID；暂停、抢占、恢复不发任务事件；多数事件的 `code` 由英文标题派生 | 中 | §4.2、§4.3.4 |
| D-4 | 事件不在 v2.0 key 上断线补发 | 中 | §4.4 |
| D-5 | `state/robot.robot_state` 不出现 `charging`；`alarm_window_active` 恒为 `false` | 中 | §5.1 |
| D-6 | `state/geo/manifest` 的 `manifest_rev` 恒为 0；~~所有启用的围栏都标成 `alarm_region`~~（U183 已修：按实际类型上报，未启用的也报） | 中 | §5.4 |
| D-7 | `state/task` 的进度字段恒为 0 / null；终态 `failed` 一律 2001，距离为 null | 中 | §5.2 |
| D-8 | `SET_ALARM_CONFIG` ~~只有区域生效，规则、声光、时间窗未接；~~（U187 已修：存储并用于自主报警）result 的 `detail` 为空（~~全部 `enabled=false` 时终态回 failed~~ U185 已修）；区域 `applies_to` 不生效 | 低 | §6.4 |
| D-9 | `msg_id` 在执行前登记去重，被拒后同一 `msg_id` 重发得到 `duplicate`；超时后命令可能已生效 | 中 | §6.7 |
| D-10 | `exit_broadcast` 的 `duplicate` ack 不回显 `stream_id`；`stream_id` 在音频进程重启后会重号 | 低 | §6.5 |
| D-11 | `STOP_TASK` 对已暂停的任务再 `pause`、对运行中的任务 `resume`，回 2003 而不是 `duplicate` | 低 | §6.2 |
| D-12 | 信封层错误的 `detail.code` 一律为 `E_SCHEMA`；云台与任务两个 key 的 `detail.code` 口径不一致 | 低 | §6.7 |
| D-13 | `voice_turn`：判不出时为 `rejected` 且 `reply_text` 为空；多数拒绝无 `reject_code`；L3 的 `pending_confirm` 没有真正上报 | 低 | §8.2、§8.3 |
| D-14 | 云端键盘遥控不校验 `client_id` / `client_seq`；参考客户端 README 的安全说明已过时 | 低 | §2 |
| ~~D-16~~ | ~~语音文本开始录制被拒（定位、电量、健康度等）时，`voice_turn` 仍为 `accepted`、只出声报原因，云端看不到拒绝~~ ✅ **已修（U165，2026-11-11，0aceb41）**：开始的结果另发一条 `layer="teach_result"` 的 voice_turn，被拒为 `rejected` 加原因码；规则层的录制拒绝也改为 `rejected`。gj-001 实测通过 | ~~中~~ | §3.1 |
| D-15 | 机器人通用面 router（7447）没有访问控制，连上的客户端可以发布机器人内部 key | 部署安全 | 请只发本文与基线文档列出的 key |

---

## 10. 没有变化的部分

以下内容按原文执行：

- v2.0 的外层信封、ID 规则、时间格式、`state/mode` 与 `state/audio` 字段、`cmd/media/session` 与 `cmd/file/ack` 的形状；
- 喊话流 `audio/broadcast` 的帧格式；
- PTZ 草案的其余条款；
- 语音文本契约 v1.0 的 §1～§4、§6.1～§6.2、§6.4、§7、§11～§13；
- 本文 §8 已列出的变更除外。
