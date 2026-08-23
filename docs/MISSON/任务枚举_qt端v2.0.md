问题：

1、关于R2 MANUAL_VELOCITY，暂时禁止了没用，之后具体以什么方式，gamepad / keyboard_local / keyboard_hmi / virtual_stick。
2、关于R3.4，评审文档里写了贵方将在下一版文档中统一为唯一值后书面告知我方，暂缓固化重发参数。
3、关于R3.5 本期只采纳剥离 L2 动作，下一版补齐凭据字段（下一版涉及到的没做）
4、关于R3。3：终止动作的执行端确认闭集未确定，cancel 是否需要执行端确认凭据（Qt 请求终止任务后，后端是否要求机器人执行端再确认一次，后端先返回“需要确认”，其中包含机器人执行端生成的 confirm_token，Qt 必须携带这个 token 再发一次新的终止请求。）；后端最终返回什么确认结果（ACCEPTED：显示“终止请求已接收”；CANCELLED：显示“任务已终止”；FAILED：显示“终止失败”；超时没有结果：显示“等待机器人确认”。）；不存在任务、已结束任务分别返回什么；停止原因如何写入审计事件（是否需要保留"reason": "operator_stop",）；
5、关于R5.2 ：车辆子类闭集未确认
6、关于R6.5：给出这`clock.ts_sync`、`alarm_window_active` 两个字段的完整状态 JSON 样例和承载 key
7、R7.4 我方将随本次评审附上 AudioChunk 的完整字段，这个没有给。


# Qt 前端任务与接口枚举（XBRAIN V6，后端对接版）


## 1. 不可变协议约束

1. Qt 是 Zenoh `client`，每台机器人建立独立 session，连接该机器人通用面 `tcp/<robot-ip>:7447`。Qt 不运行 router，不连接实时面 `7449`。
2. 正式 key 第一段固定为 `xbrain`。`robots/...`、裸 `task/result` 和其他历史路径全部废弃。
3. `rid` 必须匹配 `[a-z0-9_-]{1,32}`，并与 key 第二段逐字一致、大小写敏感。显示名称不得代替 `rid`。
4. 所有跨主机 JSON 使用 `v/rid/ts/seq/src/data` 外层。`v=1`。
5. **`ts` 明确冻结为 JSON number 表示的 float64 Unix 秒（UTC，可带小数）**，例如 `1785732000.123456`。禁止 ISO 8601 字符串、毫秒整数和本地时区字符串。
6. Qt 不发布 `mono`、`boot`、`ts_sync`。跨主机超时、安全判定和 TTL 不使用 `ts`，由判定端使用本机单调钟。
7. `seq` 为 `uint64` 语义，按“发布进程 + `rid` + 完整 key”分别递增；发布进程启动时从 `1` 开始，同一进程内的短线重连不重置。只有发布进程重启才允许回到 `1`；接收方在新连接周期重建水位，不得跨机器人或跨 key 比较 `seq`。
8. 普通命令用 `msg_id` 幂等，任务生命周期用 `task_id` 关联；重发同一业务请求必须复用原 `msg_id/task_id`，不得生成新 ID 后冒充重发。后端的 `rid+msg_id` 统一去重窗口不少于 60 秒；导航任务、文件、录像会话等持久对象还必须按各自 `task_id/file_id/session_id` 防止跨窗口重复执行或入库。
9. 多机器人操作拆成 N 条独立消息：每台机器人使用自己的 session、key、`rid`、`msg_id`、`task_id` 和 `seq`。本期没有跨机器人原子事务；部分成功必须逐机器人显示。
10. 加密通信模块、NAT、MTU 和防火墙属于部署面，不在 JSON 内增加临时兼容字段。

## 2. 正式 key 全量表

| Key | 方向 | QoS/频率要求 | 用途 |
|---|---|---|---|
| `xbrain/{rid}/cmd/task` | Qt → 后端 | 普通命令，可靠 | `GOTO_KEYPOINT`、`STOP_TASK`、`SET_ALARM_CONFIG`、`AUDIO_CONTROL` |
| `xbrain/{rid}/cmd/task/ack` | 后端 → Qt | 每个普通命令一条；接收后 2 s 内 | 受理、拒绝、幂等重复回执 |
| `xbrain/{rid}/state/task` | 后端 → Qt | 变化即发＋至少 1 Hz | 任务全机快照和权威终态结果；不另设 `task/progress`、`task/result` |
| `xbrain/{rid}/cmd/estop` | Qt → 后端 | Q0/最高优先级、独立队列 | 独立急停，不进入普通任务队列 |
| `xbrain/{rid}/cmd/estop/ack` | 后端 → Qt | Q0/变化即发；机器人收到后 100 ms 内转发 | 急停执行回执；Qt 点击到收到目标不超过 300 ms |
| `xbrain/{rid}/state/link` | 后端 → Qt | 1 Hz＋变化即发 | 普通可控在线和急停链路健康的唯一判据 |
| `xbrain/{rid}/state/robot` | 后端 → Qt | 10 Hz | 机器人、GPS、电池、运动、设备、存储和授时状态 |
| `xbrain/{rid}/state/mode` | 后端 → Qt | 1 Hz＋变化即发 | 当前模式、喊话模式及退出原因 |
| `xbrain/{rid}/state/audio` | 后端 → Qt | 1 Hz＋变化即发 | 放音、收音、喇叭占用和 `stream_id` |
| `xbrain/{rid}/state/geo/manifest` | 后端 → Qt | 变化即发；session 建立后 2 s 内发一份全量 | waypoint、recorded path、alarm region 的权威 ID/rev 清单 |
| `xbrain/{rid}/state/media` | 后端 → Qt | 变化即发＋每 5 s 全量保活 | 可见光、红外、RGBD 的动态 RTSP 端点 |
| `xbrain/{rid}/event/{severity}/{category}` | 后端 → Qt | 可靠、可补发 | 任务、报警、故障、地理、载荷、系统和审计事件 |
| `xbrain/{rid}/data/file/index` | 后端 → Qt | 可靠；连接/变化时发完整索引 | 可下载文件索引和校验信息 |
| `xbrain/{rid}/cmd/file/ack` | Qt → 后端 | 可靠 | 文件下载和 SHA-256 校验结果回执 |
| `xbrain/{rid}/cmd/media/session` | Qt → 后端 | 可靠 | Qt 本地录像会话证据索引回写 |
| `xbrain/{rid}/cmd/media/session/ack` | 后端 → Qt | 可靠 | 录像会话记录接收/拒绝回执 |
| `xbrain/{rid}/audio/broadcast` | Qt → 后端 | Q4/Ring，20 ms/帧 | `pc_to_dog` 实时音频帧；不进入任务 JSON |

`{severity}` 闭集为 `info | warn | error | fatal`。

`{category}` 闭集为 `task | alarm | fault | fence | geo | teach | arbitration | payload | system`。key 中的两段必须与事件 `data.sev/data.category` 逐字一致。

## 3. Qt 任务类型完整枚举

| 编号 | `task_type` | 状态 | 正式 key | 说明 |
|---|---|---|---|---|
| T01 | `GOTO_KEYPOINT` | 本期开放 | `cmd/task` | WGS84 有序多路点导航 |
| T02 | `STOP_TASK` | 本期开放 | `cmd/task` | 对明确 `target_task_id` 执行 `pause/resume/cancel` |
| T03 | `ESTOP` | 本期开放 | `cmd/estop` | 独立急停，不能排在普通任务后面 |
| T04 | `SET_ALARM_CONFIG` | 本期开放 | `cmd/task` | 报警标量、规则和 `alarm_region` 增量更新 |
| T05 | `AUDIO_CONTROL` | 控制面开放 | `cmd/task` | 进入/退出 `pc_to_dog` 喊话；媒体走 `audio/broadcast` |
| T06 | `MANUAL_VELOCITY` | 本期禁止 | 无 | 云端连续遥控未立项；Qt 和后端都不得开放或推断格式 |

以下旧名称不是本期协议能力：`INSPECTION_ROUTE`、`FOLLOW_RECORDED_PATH`、`PAUSE_TASK`、`RESUME_TASK`、`RETURN_HOME`、`SET_GEOFENCE`、`SET_KEYPOINTS`、`SET_RECORDED_PATHS`、`START_RECORDING`、`STOP_RECORDING`。后端收到这些 `task_type` 必须以不支持的任务类型拒绝，不能静默映射。

### 3.1 `GOTO_KEYPOINT`

- 每条消息只控制 key 中 `{rid}` 对应的机器人。
- `coordinate_system` 固定为 `WGS84`。
- `recorded_path_id` 是计划级权威路径 ID，格式 `r-[a-z0-9_]{1,40}`。
- `waypoints[]` 有序；每项使用后端 manifest 中的 `w-[a-z0-9_]{1,40}` ID。
- 经纬度必须使用 float64 序列化，建议至少保留 7 位小数；`altitude` 是 WGS84 椭球高，仅作记录，不参与到达判定。
- `arrival_radius_m` 必填，范围 `0.5..10.0` 米。后端不得在字段缺失时静默使用全局默认值。
- 后端执行时解析的实际 `route_id/route_rev` 必须在 `state/task` 进度和终态中回报。
- 未收到 manifest、ID 不存在、版本冲突、坐标非法或路径不可用时，后端必须拒绝并提供结构化定位字段。

### 3.2 `STOP_TASK`

- `target_task_id` 必填，不能使用“省略即当前任务”。
- `action` 只允许 `pause | resume | cancel`。
- `reason` 为可选自由文本，进入后端事件和审计。
- `pause/resume` 可恢复；`cancel` 不可恢复。`cancel` 需要 Qt 本地确认，但本地确认不是未来 L2/L3 的授权凭据。
- 不存在的目标返回 `rejected/E_NOT_FOUND`；已经完成相同动作返回 `duplicate`。
- STOP_TASK 只改变任务状态机，不代表底盘已经安全停车。安全停车必须使用 ESTOP。

### 3.3 `ESTOP`

- 点击后立即发布，不弹二次确认。
- `payload.action` 固定为 `stop`，`reason` 为自由文本。
- 后端必须单独订阅 `cmd/estop`，不得经普通任务 FIFO、限流或降级处理。
- 后端通过 `cmd/estop/ack` 回报 `result/estop_epoch/applied/recv_mono_ms/latency_ms/hes/timeout_lock`。
- 机器人收到急停到回执转发不超过 100 ms，Qt 点击到回执返回的端到端目标不超过 300 ms；该判定使用机器人单调钟字段，不用两端 `ts` 相减。
- `state/link.estop_path` 独立表示 `ok/degraded/down`。它不能由普通在线状态或一次 ack 替代。
- `state/link` 连续 3 秒未到达时，Qt 判定普通链路离线并将急停链路显示为 `down`。
- 急停自动重发周期和次数不在本文件中猜测；后端冻结唯一策略前，Qt 只执行一次发布并显示回执/链路状态。

### 3.4 `SET_ALARM_CONFIG`

- `alarm_level` 只允许 `1 | 2`：
  - `1`：警笛 ON、红蓝双闪 ON、探照灯常亮最亮（`MSG_BRIGHT=30`）；
  - `2`：警笛 ON、红蓝双闪 ON、探照灯自动档。
- `siren_level` 为报警警笛音轨电平，范围 `0..100`，不代表设备总音量。
- `duration_sec` 范围 `1..20`；终态回报 `played_times` 与 `stop_reason=completed|max_duration|preempted`。
- `cooldown_sec` 范围 `0.5..600.0`。
- `alarm_window` 是每日循环 `HH:mm`，允许 `start > end` 表示跨午夜。授时未同步时带时间窗规则不命中，后端在 `state/robot.clock.ts_sync` 和 `alarm_window_active` 回显。
- 本期只支持区域内人员/车辆判定，不支持陌生人、陌生车、人脸或车牌身份。
- `rules[]` 使用 `person_in_region | vehicle_in_region`，并以 `alarm_role=include|exclude` 和 `region_ids[]` 显式绑定区域；排除规则优先于包含规则。
- `regions[]` 只允许 `alarm_region`，禁止 `keep_in`。营区 keep-in 必须走独立安全围栏接口，本期 Qt 不发布。
- 区域采用增量语义：`op=upsert|delete|set_state`；未出现的区域不受影响，不允许整集 prune。
- `base_rev` 必填：新建为 `0`，更新/删除为 manifest 中当前 rev。冲突返回 `E_GEO_CONFLICT` 和当前 rev，Qt 刷新 manifest 后由操作员重新提交。
- 普通 ack 的 `accepted` 只表示格式和受理成功，不代表区域已经生效。后端完成 stage/commit 并确认 `state/fence.active.rev` 后，必须通过关联 `task_id` 的 `state/task` 终态回报 `done`；abort/超时回报 `failed`。
- 从后端受理到发布上述终态的时限为 6 秒。超时时 Qt 显示“生效未确认”，后端仍必须在得到权威结果后发布 result，不得将 accepted 补解释为成功。

### 3.5 `AUDIO_CONTROL` 和实时音频

- 只开放 `mode=pc_to_dog`。
- 进入喊话：`action=start`；退出喊话：`action=exit_broadcast`。
- `dog_to_pc` 本期禁止；后端收到必须拒绝，不得静默启用机上麦克风上行。
- `action=start` 的 ack 必须带 `task_type=AUDIO_CONTROL` 和后端新分配的 `stream_id`。
- `action=exit_broadcast` 的请求必须携带要退出的原 `stream_id`，其 ack 必须回显同一个 `stream_id`；后端不得为退出请求分配新 ID。
- 按钮最终状态以 `state/mode`、`state/audio` 为准，不能以一次 ack 代替实际放音状态。
- start 被受理后 1 秒内，后端必须在 `state/mode` 和 `state/audio` 回读实际模式；进入失败必须给出状态原因和可靠故障事件。
- 同一次喊话的所有 `audio/broadcast` 帧复用同一 `stream_id`。音频固定为 PCM S16LE、16 kHz、单声道、20 ms/帧。
- 后端以本机单调钟判定：最后一帧有效音频后 300 秒没有新音频，自动退出广播并在状态中回报 `exit_reason=timeout`。

### 3.6 `MANUAL_VELOCITY`

本期没有合法 key 和 JSON。方向按钮、速度、键盘、deadman、TTL、`client_seq` 和 takeover 全部不开放。后端收到历史 `MANUAL_VELOCITY` 或 `cmd/teleop` 云端来源时应拒绝并回 `E_CHANNEL_DENIED`，不能执行速度。

## 4. 上行状态、事件和结果职责

### 4.1 任务 ack、快照和终态

- `cmd/task/ack` 必须关联 `ref_msg_id/task_id/task_type`，结果只允许 `accepted/rejected/duplicate`。
- `duplicate` 表示原请求已经处理，不能再次创建任务、重置进度或重复执行。
- `state/task` 使用 `message_type=snapshot|result` 区分全机快照和权威终态。
- snapshot 包含 `current/queue/suspended`；`progress_percent` 允许为 `null`，表示路径/总里程仍在计算，禁止填 0 冒充。
- result 终态只允许 `done|failed|cancelled`；必须由任务权威模块产生，不能由网关观察状态跳变后猜测。
- `duration_sec` 是机上任务实际执行时长，不含排队等待；`distance_m` 是实际执行距离；`ended_ts` 是 float64 Unix 秒。

### 4.2 在线和机器人状态

- 只有 `state/link` 是“云端可控在线”判据。任意其他 `state/**` 到达都不能刷新在线计时。
- `state/robot` 提供 `robot_state/task_state/gps/battery/motion/devices/storage/clock/alarm_window_active`。
- 设备列表必须由后端实际发现结果生成；不得要求 Qt 用固定设备名称伪造在线行。
- 所有状态按 `rid` 分区；切换机器人必须切换到对应缓存。

### 4.3 事件

- 正式事件 ID 字段为 `eid`，后端在断线补发时复用原 `eid`。
- key 的 severity/category 必须与 `data.sev/data.category` 一致；不一致视为协议错误。
- 后端必须可靠保存并补发任务拒绝、任务终态、报警和故障事件。
- 报警 detail 必须提供 `fence_set_id/rev/matched_rule/zone_hits/dwell_s/track_id/wpos`，并通过 `media[]` 或 `file_refs[]` 关联机上判定帧。
- 取证口径：`onboard_decision` 是机器人报警判定证据；`cloud_observed` 是 Qt 从 RTSP 保存的观察记录，不能替代机上判定帧。

### 4.4 地理 manifest

- 后端在 session 建立后发布一份全量 manifest，以后变化即发。
- waypoint、recorded path、alarm region 必须包含稳定 `geo_id`、类型和 `rev`。
- ID 创建后不得因显示名称变化而改变。
- Qt 下发引用前必须使用当前机器人 manifest；后端仍执行权威校验。

### 4.5 动态媒体端点

- `state/media.endpoints[]` 每项独立描述 `id/stream/url/state/credential_ref`。
- `id` 闭集为 `cam_ptz_vis | cam_ptz_ir | cam_rgbd`；`stream` 为 `main | sub`。
- URL 必须是完整 `rtsp://host:port/path`，不得含 `user:password@`，也不得要求 Qt 根据主码流猜子码流。
- 每个 endpoint 可使用不同 `credential_ref`。凭据只存在 Qt 运行时密钥存储，禁止进入报文、项目配置和日志。
- 预览使用 `main`；连续录像使用 `cam_ptz_vis/sub`，码率不超过 2 Mbps。
- 媒体断流只降低观察/录像能力，不得作为机器人停车或在线判据。
- 当前 Qt 主控 PC 的部署出口地址为 `192.168.123.60`；若变更必须同步更新部署配置和白名单。后端/甲方的 DNAT 与防火墙只允许该源地址访问 RTSP，禁止向 `0.0.0.0/0` 开放。

## 5. 文件和录像证据接口

### 5.1 后端文件交付

- 后端通过 `data/file/index` 发布已就绪文件的 `file_id/kind/relative_path/size_bytes/sha256/created_ts` 以及事件、任务、会话关联 ID。
- `relative_path` 必须是配置根目录下的相对路径，禁止绝对路径、`..` 和凭据。
- Qt 按部署配置的只读 SFTP 连接下载，校验 `size_bytes` 和 SHA-256 后发布 `cmd/file/ack`。
- ack 只允许 `downloaded|checksum_failed|download_failed`；失败必须附 reason，不能用“未回 ack”代替失败原因。

### 5.2 Qt 本地录像会话回写

- Qt 预览使用主码流，巡检录像使用子码流，60 秒分段。
- 本地快照名使用 `<eid>_<seq>.png`，缺少 `eid` 时不生成可审计快照。
- 录像结束后 Qt 通过 `cmd/media/session` 回写 `session_id/task_id/event_ids/started_at/ended_at/robot_ts_at_start/segments[]`。
- `started_at/ended_at/robot_ts_at_start` 都是 float64 Unix 秒；每段包含 `name/started_at/ended_at/size_bytes/sha256`。
- 后端必须通过 `cmd/media/session/ack` 明确接收或拒绝；ack 必须回显原请求的 `task_id`，并用 `session_id` 将本地录像与机器人事件建立索引。

## 6. Qt 页面与后端依赖矩阵

| Qt 功能 | 后端依赖 | Qt 是否发布 |
|---|---|---|
| 机器人选择/多机看板 | 每机器人独立 session；全部状态按 rid 分区 | 否 |
| 计划编辑 | `state/geo/manifest` | 否 |
| 单机/多机计划下发 | 每个目标机器人一条 `GOTO_KEYPOINT` | 是 |
| 暂停/继续/终止 | `STOP_TASK`＋`state/task` | 是 |
| 急停 | `cmd/estop`＋`cmd/estop/ack`＋`state/link.estop_path` | 是 |
| 机器人/设备状态 | `state/robot` | 否 |
| 在线显示 | `state/link`，3 秒超时 | 否 |
| 报警配置 | `SET_ALARM_CONFIG`＋最终 result | 是 |
| 报警/故障日志 | `event/**` | 否 |
| 报警快照 | alarm event 的 `eid`＋RTSP 当前帧 | 否 |
| 观察画面 | `state/media` main 端点 | 否 |
| 巡检录像 | `state/media` sub 端点；结束后回写 session | 回写时是 |
| 喊话 | `AUDIO_CONTROL`＋`audio/broadcast`＋audio/mode 状态 | 是 |
| 文件下载 | `data/file/index`＋SFTP；完成后 file ack | 回执时是 |
| 手动移动 | 本期禁止 | 否 |
| 本地计划、巡检 JSON、PDF、视频导出、用户管理 | 无 | 否 |

## 7. 后端实现和联调验收底线

1. 所有正式 key、方向、字段名、类型和闭集与本文及 JSON 文档一致。
2. 全部出站 `ts` 是 float64 Unix 秒；不存在 ISO 字符串兼容分支。
3. 非法版本、rid/key 不一致、字段缺失、类型错误、枚举越界均返回结构化拒绝，不能静默丢弃。
4. 每个 ack/result/event 都能通过 `msg_id/task_id/eid` 关联原业务。
5. 多机器人同时在线时，不串状态、不串任务、不串媒体、不串事件。
6. 断网重连后，可靠事件和文件索引可补发；重复消息不重复执行。
7. `state/link` 中断 3 秒必然导致 Qt 判离线；其他状态流不能维持在线假象。
8. 报警配置 accepted 与最终生效结果严格分离。
9. 终态耗时和距离由机器人权威提供，Qt 不按本地收包时间推算。
10. 云端 teleop、keep-in 修改、`dog_to_pc` 和未知旧任务类型必须安全拒绝。

## 8. 评审意见覆盖索引

| 评审主题 | 本文档的完整落点 |
|---|---|
| R1 接入、key、信封、`seq`、`rid`、部署链路 | §1、§2 |
| R2 云端手动速度安全边界 | §3.6：本期整体禁止，不留半开放字段 |
| R3 STOP/急停/急停回执/L2、L3 | §3.2、§3.3、§3.4；本期需凭据的 L2/L3 动作不进入 Qt 可写面 |
| R4 报警区、增量语义、版本、异步生效 | §3.4 |
| R5 人员/车辆与身份识别边界 | §3.4 |
| R6 报警等级、音量、时长、冷却、日时间窗 | §3.4、§4.2 |
| R7 喊话模式、AudioChunk、状态回读 | §3.5、§2 的 audio/mode key |
| R8 路径、路点、到达半径、WGS84、版本 | §3.1、§4.4 |
| R9 RTSP、鉴权、两路流、录像证据、带宽 | §4.5、§5 |
| R10 错误、幂等、去重、ID 分域、拒绝审计 | §1、§4.1、§4.3、§7 |
| R11 `ts`、单调钟、授时状态、权威耗时 | §1、§3.3、§4.1、§4.2 |
| R12 状态、报警、故障、文件、进度、终态、在线判据 | §2、§4、§5、§6 |

字段类型、必填性、枚举、示例和错误结构对应见《json格式文件_qt端.md》。
