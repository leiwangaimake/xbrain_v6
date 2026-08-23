# Qt 前端与后端 JSON 接口格式（XBRAIN V6）

## 1. 全局约定

### 1.1 通用外层

所有跨主机 JSON 消息使用同一个外层：

```json
{
  "v": 1,
  "rid": "gj-001",
  "ts": 1785732000.123456,
  "seq": 1,
  "src": "qt_hmi",
  "data": {}
}
```

| 字段 | JSON 类型 |  | 约束 |
|---|---|---:|---|
| `v` | integer | | 固定 `1` |
| `rid` | string |  | `[a-z0-9_-]{1,32}`，与 key 第二段逐字一致 |
| `ts` | number | | **float64 Unix 秒（UTC，可带小数）** |
| `seq` | integer | | `uint64` 语义；按“发布进程+`rid`+完整 key”独立递增；进程启动从 `1` 开始，同进程短线重连不重置 |
| `src` | string |  | Qt → 后端固定 `qt_hmi`；后端 → Qt 固定 `p5_gateway`，内部真实来源放在业务字段 `data.source` |
| `data` | object |  | 当前 key 对应的业务对象；无参数时为 `{}` |

跨主机消息不得携带 `mono`、`boot`。Qt 请求不得携带 `ts_sync`；机器人授时状态在 `state/robot.data.clock.ts_sync` 中权威回报。本协议中的全部 `*_ts`、`started_at`、`ended_at`、`created_ts`、`generated_ts` 均使用与 `ts` 相同的 float64 Unix 秒格式，除每日时间窗明确使用 `HH:mm` 外，不存在第二种时间格式。

### 1.2 ID、幂等和顺序

业务字段，不是外层信封字段；统一放在外层 data 对象中。
| 字段 | 用途 | 规则 |
|---|---|---|
| `msg_id` | 单条业务消息幂等 ID | 建议使用 `msg-<uuid>`；强制格式以本节统一 ID 正则为准；同一请求重发必须复用 |
| `task_id` | 一次任务/控制生命周期 |  `task-<uuid>`；ack、progress、result、event 必须复用 |
| `ref_msg_id` | 回执关联的请求 | 必须等于原请求 `msg_id` |
| `eid` | 可靠事件 ID | 建议 `evt-<uuid>`；补发必须复用 |
| `file_id` | 后端文件实体 ID | 创建后稳定，下载重试不改变 |
| `session_id` | 一次音频或录像会话 | 同一会话内稳定 |

除已单独规定正则的地理 ID 外，上表 ID 统一匹配 `[A-Za-z0-9][A-Za-z0-9._:-]{0,127}`，区分大小写。后端不得截断、自动改写或按显示名重建 ID。

后端以 `rid+msg_id` 做幂等判断，统一去重窗口不少于 60 秒，且不得仅依赖条数上限提前淘汰窗口内的 ID。导航任务、文件和录像会话还要分别以 `task_id/file_id/session_id` 做持久幂等，防止 60 秒后的迟到重放重复执行或入库。`duplicate` 表示原请求已处理，不能再次执行。`seq` 用于乱序/缺口诊断，不能代替业务幂等 ID。

### 1.3 数值、字符串和未知字段

- 经纬度为 JSON number/float64；纬度 `[-90,90]`，经度 `[-180,180]`，建议至少保留 7 位小数。
- 字符串必须为 UTF-8；协议 ID、枚举和 key 只使用 ASCII。
- 接收方可以保留未知扩展字段，但不得把未知枚举降级解释为某个已知值。
- 必填字段缺失、字段类型错误、NaN/Infinity、枚举越界或 rid/key 不一致均为拒绝条件。

### 1.4 响应时限和时钟口径

- 普通 `cmd/task` 在后端收到后 2 秒内必须返回一条 ack，包括结构拒绝、业务拒绝和 duplicate。
- 急停在机器人收到后 100 ms 内转发 ack；Qt 点击到收到 ack 的端到端目标为不超过 300 ms。
- `SET_ALARM_CONFIG` 的 accepted ack 不是生效证明；权威 result 应在受理后 6 秒内发布。超时后仍必须发布最终 result。
- `AUDIO_CONTROL start` 被受理后 1 秒内，`state/mode` 和 `state/audio` 必须回读实际模式或失败原因。
- 上述超时、幂等窗口、任务耗时和音频空闲超时都用各判定端本机单调钟，不使用消息 `ts` 或两主机墙钟作差。

## 2. Qt 下发的任务和控制

普通任务统一发布到 `xbrain/{rid}/cmd/task`，其 `data` 必须包含：

| 字段 | 类型 | 必填 | 说明 |
|---|---|---:|---|
| `msg_id` | string | 是 | 消息幂等 ID |
| `task_id` | string | 是 | 当前任务或控制命令 ID |
| `task_type` | string | 是 | 本章闭集 |
| `payload` | object | 是 | 当前任务参数 |

### 2.1 `GOTO_KEYPOINT`

Key：`xbrain/{rid}/cmd/task`

```json
{
  "v": 1,
  "rid": "gj-001",
  "ts": 1785732000.123456,
  "seq": 1,
  "src": "qt_hmi",
  "data": {
    "msg_id": "msg-goto-001",
    "task_id": "task-goto-001",
    "task_type": "GOTO_KEYPOINT",
    "payload": {
      "coordinate_system": "WGS84",
      "recorded_path_id": "r-route_north",
      "waypoints": [
        {
          "id": "w-north_gate",
          "name": "北门",
          "latitude": 31.2301971,
          "longitude": 121.4732683,
          "altitude": 8.4,
          "arrival_radius_m": 3.0
        }
      ]
    }
  }
}
```

| payload 字段 | 类型 | 必填 | 约束 |
|---|---|---:|---|
| `coordinate_system` | string | 是 | 只允许 `WGS84` |
| `recorded_path_id` | string | 是 | `r-[a-z0-9_]{1,40}`，必须存在于当前 manifest |
| `waypoints` | array | 是 | 非空有序数组 |
| `waypoints[].id` | string | 是 | `w-[a-z0-9_]{1,40}`，必须存在于当前 manifest |
| `waypoints[].name` | string | 是 | 显示名，不用于身份匹配 |
| `latitude/longitude` | number | 是 | WGS84 float64 |
| `altitude` | number | 是 | WGS84 椭球高，米，仅记录 |
| `arrival_radius_m` | number | 是 | `0.5..10.0` 米 |

后端不得接受 waypoint 级 `recorded_path_id`，也不得在 `arrival_radius_m` 缺失时静默补默认值。实际加载的 `route_id/route_rev` 由后端在任务状态中回报。

### 2.2 `STOP_TASK`

Key：`xbrain/{rid}/cmd/task`

```json
{
  "v": 1,
  "rid": "gj-001",
  "ts": 1785732010.321,
  "seq": 2,
  "src": "qt_hmi",
  "data": {
    "msg_id": "msg-control-001",
    "task_id": "task-control-001",
    "task_type": "STOP_TASK",
    "payload": {
      "target_task_id": "task-goto-001",
      "action": "cancel",
      "reason": "操作员终止巡检"
    }
  }
}
```

| payload 字段 | 类型 | 必填 | 约束 |
|---|---|---:|---|
| `target_task_id` | string | 是 | 被控制的导航任务 ID，不等于本控制消息 `task_id` |
| `action` | string | 是 | `pause | resume | cancel` |
| `reason` | string | 否 | UTF-8 自由文本；原样进入事件/审计 |

不存在的任务返回 `rejected`＋`E_NOT_FOUND`；已完成的相同动作返回 `duplicate`。STOP_TASK 不发布零速度，不承担急停职责。

### 2.3 `ESTOP`

Key：`xbrain/{rid}/cmd/estop`

```json
{
  "v": 1,
  "rid": "gj-001",
  "ts": 1785732012.004,
  "seq": 1,
  "src": "qt_hmi",
  "data": {
    "msg_id": "msg-estop-001",
    "task_id": "task-estop-001",
    "task_type": "ESTOP",
    "payload": {
      "action": "stop",
      "reason": "operator_estop"
    }
  }
}
```

`action` 固定为 `stop`。后端必须为该 key 使用独立最高优先级接收路径，不得排入 `cmd/task` 队列。

### 2.4 `SET_ALARM_CONFIG`

Key：`xbrain/{rid}/cmd/task`

```json
{
  "v": 1,
  "rid": "gj-001",
  "ts": 1785732020.8,
  "seq": 3,
  "src": "qt_hmi",
  "data": {
    "msg_id": "msg-alarm-001",
    "task_id": "task-alarm-001",
    "task_type": "SET_ALARM_CONFIG",
    "payload": {
      "alarm_level": 1,
      "siren_level": 70,
      "duration_sec": 5,
      "cooldown_sec": 2.0,
      "alarm_window": {
        "start": "22:00",
        "end": "05:00"
      },
      "rules": [
        {
          "type": "person_in_region",
          "enabled": true,
          "alarm_role": "include",
          "applies_to": ["person"],
          "region_ids": ["f-alarm_equipment"]
        },
        {
          "type": "vehicle_in_region",
          "enabled": true,
          "alarm_role": "include",
          "applies_to": ["vehicle", "bicycle", "motorcycle"],
          "region_ids": ["f-alarm_equipment"]
        },
        {
          "type": "person_in_region",
          "enabled": true,
          "alarm_role": "exclude",
          "applies_to": ["person"],
          "region_ids": ["f-duty_room"]
        }
      ],
      "regions": [
        {
          "id": "f-alarm_equipment",
          "op": "upsert",
          "base_rev": 7,
          "name": "设备区",
          "type": "alarm_region",
          "enabled": true,
          "applies_to": ["person", "vehicle", "bicycle", "motorcycle"],
          "vertices": [
            {"latitude": 31.2301971, "longitude": 121.4732683},
            {"latitude": 31.2301971, "longitude": 121.473864},
            {"latitude": 31.2305962, "longitude": 121.473864},
            {"latitude": 31.2305962, "longitude": 121.4732683}
          ]
        },
        {
          "id": "f-old_zone",
          "op": "delete",
          "base_rev": 3
        }
      ]
    }
  }
}
```

报警标量：

| 字段 | 类型 | 必填 | 约束 |
|---|---|---:|---|
| `alarm_level` | integer | 是 | `1 | 2`；完整硬件语义见任务枚举文档 |
| `siren_level` | integer | 是 | `0..100`，只表示警笛音轨电平 |
| `duration_sec` | integer | 是 | `1..20` |
| `cooldown_sec` | number | 是 | `0.5..600.0` |
| `alarm_window.start/end` | string | 是 | 每日 `HH:mm`；允许跨午夜 |

规则对象：

| 字段 | 类型 | 必填 | 约束 |
|---|---|---:|---|
| `type` | string | 是 | `person_in_region | vehicle_in_region` |
| `enabled` | boolean | 是 | 是否启用该规则 |
| `alarm_role` | string | 是 | `include | exclude`；exclude 优先 |
| `applies_to` | array[string] | 是 | 人员固定 `person`；车辆可含 `vehicle/bicycle/motorcycle` |
| `region_ids` | array[string] | 是 | 启用规则时非空；必须引用本消息或 manifest 中的 alarm region |

区域操作是增量更新，未出现在数组中的区域不受影响：

| `op` | 必填字段 | 语义 |
|---|---|---|
| `upsert` | `id/base_rev/name/type/enabled/applies_to/vertices` | 新建或替换完整区域；新建 `base_rev=0` |
| `delete` | `id/base_rev` | 显式删除 |
| `set_state` | `id/base_rev/enabled` | 只改变启用状态 |

`id` 必须符合 `f-[a-z0-9_]{1,40}`，`type` 固定 `alarm_region`。禁止 `keep_in`、`auto_off` 和整集 prune。顶点至少 3 个、不自交、面积非零；机器人端执行权威几何校验。

版本冲突通过普通 ack 返回：

```json
{
  "v": 1,
  "rid": "gj-001",
  "ts": 1785732021.1,
  "seq": 40,
  "src": "p5_gateway",
  "data": {
    "msg_id": "ack-alarm-conflict-001",
    "ref_msg_id": "msg-alarm-001",
    "task_id": "task-alarm-001",
    "task_type": "SET_ALARM_CONFIG",
    "result": "rejected",
    "accepted": false,
    "error_code": 1003,
    "reason": "区域版本冲突",
    "detail": {
      "code": "E_GEO_CONFLICT",
      "region_id": "f-alarm_equipment",
      "base_rev": 7,
      "current_rev": 8
    }
  }
}
```

普通 ack 的 `accepted=true` 只表示受理，不表示最终生效。最终生效或失败必须通过 §3.3 的 `state/task` result 回报。

### 2.5 `AUDIO_CONTROL`

进入喊话：

```json
{
  "v": 1,
  "rid": "gj-001",
  "ts": 1785732030.1,
  "seq": 4,
  "src": "qt_hmi",
  "data": {
    "msg_id": "msg-audio-001",
    "task_id": "task-audio-001",
    "task_type": "AUDIO_CONTROL",
    "payload": {
      "mode": "pc_to_dog",
      "action": "start"
    }
  }
}
```

退出喊话的 payload：

```json
{
  "mode": "pc_to_dog",
  "action": "exit_broadcast",
  "stream_id": "audio-gj001-0001"
}
```

| 字段 | 类型 | 必填 | 约束 |
|---|---|---:|---|
| `mode` | string | 是 | 只允许 `pc_to_dog`；`dog_to_pc` 必须拒绝 |
| `action` | string | 是 | `start | exit_broadcast` |
| `stream_id` | string | 条件必填 | `start` 时不得携带；`exit_broadcast` 时必须等于要退出的原会话 `stream_id` |

### 2.6 明确禁止的请求

本期不定义、不得发送：`MANUAL_VELOCITY`、云端 `cmd/teleop`、`dog_to_pc`、`SET_GEOFENCE`、keep-in 修改、L2/L3 `confirm_token` 动作及第 2 章未列出的旧任务类型。

通过 `cmd/task` 收到上述能力时，后端必须返回普通拒绝 ack：`result="rejected"`、`accepted=false`、`error_code=1006`；未支持任务使用 `detail.code="E_TASK_UNSUPPORTED"`，禁止动作使用 `detail.code="E_CHANNEL_DENIED"`。对未登记的云端 key（如 `cmd/teleop`）不创造临时 ack key：必须不执行，并产生一条可靠 `event/warn/system` 协议事件。任何情况都不得忽略字段后部分执行，也不得静默映射到其他能力。

## 3. 命令回执和任务状态

### 3.1 普通任务 ack

Key：`xbrain/{rid}/cmd/task/ack`

```json
{
  "v": 1,
  "rid": "gj-001",
  "ts": 1785732001.2,
  "seq": 10,
  "src": "p5_gateway",
  "data": {
    "msg_id": "ack-001",
    "ref_msg_id": "msg-goto-001",
    "task_id": "task-goto-001",
    "task_type": "GOTO_KEYPOINT",
    "result": "accepted",
    "accepted": true,
    "error_code": 0,
    "reason": "",
    "detail": {}
  }
}
```

| 字段 | 类型 | 必填 | 约束 |
|---|---|---:|---|
| `msg_id` | string | 是 | ack 自身 ID |
| `ref_msg_id` | string | 是 | 原请求 `msg_id` |
| `task_id` | string | 是 | 原请求 `task_id` |
| `task_type` | string | 是 | 原请求任务类型；音频 ack 也不得省略 |
| `result` | string | 是 | `accepted | rejected | duplicate` |
| `accepted` | boolean | 是 | accepted/duplicate 为 true，rejected 为 false |
| `error_code` | integer | 是 | 成功为 `0`，失败见 §10 |
| `reason` | string | 是 | 成功可为空；失败必须可读 |
| `detail` | object | 是 | 结构化补充信息，无内容为 `{}` |

导航拒绝的结构化定位：

```json
{
  "msg_id": "ack-goto-rejected-001",
  "ref_msg_id": "msg-goto-001",
  "task_id": "task-goto-001",
  "task_type": "GOTO_KEYPOINT",
  "result": "rejected",
  "accepted": false,
  "error_code": 2006,
  "reason": "第 4 个关键点位于围栏外",
  "detail": {
    "code": "E_OUT_OF_FENCE",
    "subject": "target",
    "waypoint_id": "w-north_gate",
    "index": 3,
    "field": "waypoints[3]"
  }
}
```

音频 `start` 的 ack 必须在 `detail.stream_id` 返回后端新分配的 `stream_id`：

```json
{
  "msg_id": "ack-audio-001",
  "ref_msg_id": "msg-audio-001",
  "task_id": "task-audio-001",
  "task_type": "AUDIO_CONTROL",
  "result": "accepted",
  "accepted": true,
  "error_code": 0,
  "reason": "",
  "detail": {
    "stream_id": "audio-gj001-0001"
  }
}
```

音频 `exit_broadcast` 的 ack 必须在 `detail.stream_id` 回显请求中的原 `stream_id`，后端不得为退出请求分配新 ID。该规则对 `accepted`、`duplicate` 和 `rejected` 均适用。

### 3.2 `state/task` 全机快照

Key：`xbrain/{rid}/state/task`

```json
{
  "v": 1,
  "rid": "gj-001",
  "ts": 1785732050.0,
  "seq": 20,
  "src": "p5_gateway",
  "data": {
    "msg_id": "state-task-001",
    "message_type": "snapshot",
    "current": {
      "task_id": "task-goto-001",
      "task_type": "GOTO_KEYPOINT",
      "state": "running",
      "current_waypoint_id": "w-north_gate",
      "completed_count": 0,
      "total_count": 1,
      "progress_percent": null,
      "route_id": "r-route_north",
      "route_rev": 3,
      "started_ts": 1785732040.2,
      "message": "正在计算路径总里程"
    },
    "queue": [],
    "suspended": []
  }
}
```

| 字段 | 类型 | 必填 | 约束 |
|---|---|---:|---|
| `msg_id` | string | 是 | 状态消息 ID |
| `message_type` | string | 是 | 固定 `snapshot` |
| `current` | object/null | 是 | 当前执行任务；无任务为 null |
| `queue` | array | 是 | 排队任务，顺序即执行顺序 |
| `suspended` | array | 是 | 暂停任务 |

任务项字段：

| 字段 | 类型 | 必填 | 约束 |
|---|---|---:|---|
| `task_id/task_type/state` | string | 是 | state 为 `queued|running|paused` |
| `current_waypoint_id` | string/null | 是 | 尚未选择路点时为 null |
| `completed_count/total_count` | integer | 是 | 只作数量显示，不反算百分比 |
| `progress_percent` | number/null | 是 | `0..100`；未知时必须为 null，禁止填 0 |
| `route_id` | string/null | 是 | 实际加载路径 ID |
| `route_rev` | integer/null | 是 | 实际加载路径版本 |
| `started_ts` | number/null | 是 | 实际开始执行时间，不含排队 |
| `message` | string | 是 | 人类可读状态，可为空 |

### 3.3 `state/task` 权威终态

同一个 key 使用 `message_type=result`，不另设 `task/result`：

```json
{
  "v": 1,
  "rid": "gj-001",
  "ts": 1785732125.55,
  "seq": 21,
  "src": "p5_gateway",
  "data": {
    "msg_id": "result-task-001",
    "message_type": "result",
    "task_id": "task-goto-001",
    "task_type": "GOTO_KEYPOINT",
    "state": "done",
    "result_code": 0,
    "reason": "",
    "summary": {
      "completed_count": 1,
      "total_count": 1,
      "distance_m": 86.4,
      "duration_sec": 85.35,
      "started_ts": 1785732040.2,
      "ended_ts": 1785732125.55,
      "route_id": "r-route_north",
      "route_rev": 3
    },
    "detail": {}
  }
}
```

| 字段 | 类型 | 必填 | 约束 |
|---|---|---:|---|
| `msg_id` | string | 是 | result 消息自身 ID；格式遵循 §1.2 的统一 ID 规则 |
| `message_type` | string | 是 | 固定 `result` |
| `task_id/task_type` | string | 是 | 原任务关联 |
| `state` | string | 是 | `done | failed | cancelled` |
| `result_code` | integer | 是 | 成功 `0`；失败见 §10 |
| `reason` | string | 是 | 成功可为空；失败/取消必填 |
| `summary.completed_count/total_count` | integer | 是 | 权威计数 |
| `summary.distance_m` | number/null | 是 | 权威距离；不适用为 null |
| `summary.duration_sec` | number | 是 | 机上实际执行时长，不含排队 |
| `summary.started_ts/ended_ts` | number | 是 | float64 Unix 秒 |
| `summary.route_id/route_rev` | string/integer/null | 是 | 导航任务必填；其他任务可 null |
| `detail` | object | 是 | 类型相关结果 |

报警配置结果的 `detail`：

```json
{
  "msg_id": "result-alarm-001",
  "message_type": "result",
  "task_id": "task-alarm-001",
  "task_type": "SET_ALARM_CONFIG",
  "state": "done",
  "result_code": 0,
  "reason": "",
  "summary": {
    "completed_count": 1,
    "total_count": 1,
    "distance_m": null,
    "duration_sec": 1.2,
    "started_ts": 1785732020.9,
    "ended_ts": 1785732022.1,
    "route_id": null,
    "route_rev": null
  },
  "detail": {
    "active_rev": 8,
    "played_times": 2,
    "stop_reason": "completed"
  }
}
```

### 3.4 急停回执

Key：`xbrain/{rid}/cmd/estop/ack`

```json
{
  "v": 1,
  "rid": "gj-001",
  "ts": 1785732012.12,
  "seq": 6,
  "src": "p5_gateway",
  "data": {
    "msg_id": "estop-ack-001",
    "ref_msg_id": "msg-estop-001",
    "task_id": "task-estop-001",
    "task_type": "ESTOP",
    "result": "accepted",
    "accepted": true,
    "error_code": 0,
    "reason": "",
    "detail": {
      "result": "accepted",
      "estop_epoch": 42,
      "applied": ["zero_vel", "charge_abort"],
      "recv_mono_ms": 123456789,
      "latency_ms": 18,
      "hes": "ok",
      "timeout_lock": false
    }
  }
}
```

`result` 为 `accepted|duplicate|rejected`。`duplicate` 时 `accepted=true` 且 `detail.result=duplicate`；`applied` 必须为字符串数组。`recv_mono_ms/latency_ms` 由机器人端单调钟计算，Qt 不用两端 `ts` 相减推断安全时延。

## 4. 机器人状态

### 4.1 链路状态

Key：`xbrain/{rid}/state/link`，1 Hz＋变化即发。

```json
{
  "v": 1,
  "rid": "gj-001",
  "ts": 1785732060.0,
  "seq": 30,
  "src": "p5_gateway",
  "data": {
    "state": "up",
    "cloud_link": true,
    "disconnected_s": 0.0,
    "estop_path": "ok"
  }
}
```

| 字段 | 类型 | 必填 | 闭集/范围 |
|---|---|---:|---|
| `state` | string | 是 | `up | degraded | down` |
| `cloud_link` | boolean | 是 | 网关是否持有有效云端链路 |
| `disconnected_s` | number | 是 | 当前连续断开秒数；连接时为 0 |
| `estop_path` | string | 是 | `ok | degraded | down` |

Qt 只以该 key 判普通在线；连续 3 秒未收到即离线。其他状态不能刷新在线计时。

### 4.2 机器人综合状态

Key：`xbrain/{rid}/state/robot`，固定 10 Hz。

```json
{
  "v": 1,
  "rid": "gj-001",
  "ts": 1785732060.1,
  "seq": 101,
  "src": "p5_gateway",
  "data": {
    "robot_state": "running",
    "task_state": "running",
    "gps": {
      "fix": "rtk_fixed",
      "latitude": 31.2301971,
      "longitude": 121.4732683,
      "altitude_m": 8.4,
      "heading_deg": 92.5,
      "speed_mps": 0.6,
      "accuracy_m": 0.03
    },
    "battery": {
      "soc": 76,
      "voltage_v": 52.1,
      "current_a": -3.2,
      "temperature_c": 31.4
    },
    "motion": {
      "gait": "walk",
      "linear_speed_mps": 0.6,
      "angular_speed_radps": 0.0
    },
    "devices": [
      {
        "id": "cam_ptz_vis",
        "name": "布控球可见光",
        "status": "online",
        "last_update_ms": 120
      }
    ],
    "storage": {
      "free_gb": 83.5,
      "total_gb": 128.0
    },
    "clock": {
      "ts_sync": true,
      "source": "rtk",
      "age_ms": 80
    },
    "alarm_window_active": true
  }
}
```

闭集：

- `robot_state`: `offline|idle|running|charging|fault|emergency_stop`
- `task_state`: `idle|queued|running|paused|completed|failed|cancelled`
- `gps.fix`: `none|gps|dgps|rtk_float|rtk_fixed`
- `devices[].status`: `online|degraded|offline|fault|unknown`

`devices[]` 每项必须有 `id/name/status/last_update_ms`；后端只发布实际发现的设备。`clock.ts_sync=false` 时带时间窗的报警规则不命中，`alarm_window_active` 必须同步反映实际生效状态。

### 4.3 模式状态

Key：`xbrain/{rid}/state/mode`

```json
{
  "v": 1,
  "rid": "gj-001",
  "ts": 1785732061.0,
  "seq": 8,
  "src": "p5_gateway",
  "data": {
    "voice_mode": "broadcast",
    "source": "cloud",
    "stream_id": "audio-gj001-0001",
    "entered_ts": 1785732030.2,
    "exit_reason": null
  }
}
```

| 字段 | 类型 | 必填 | 闭集 |
|---|---|---:|---|
| `voice_mode` | string | 是 | `normal | broadcast | alarm` |
| `source` | string | 是 | `cloud | local | autonomy | system` |
| `stream_id` | string/null | 是 | broadcast 时必填 |
| `entered_ts` | number/null | 是 | 当前模式进入时间 |
| `exit_reason` | string/null | 是 | `requested|timeout|preempted|fault|target_lost|target_left_fence|manual_cloud|manual_wecom` |

### 4.4 音频状态

Key：`xbrain/{rid}/state/audio`

```json
{
  "v": 1,
  "rid": "gj-001",
  "ts": 1785732061.05,
  "seq": 9,
  "src": "p5_gateway",
  "data": {
    "stream_id": "audio-gj001-0001",
    "playing": true,
    "recording": false,
    "speaker_state": "playing",
    "microphone_state": "disabled",
    "speaker_holder": "cloud_broadcast",
    "speaker_holder_type": "cloud",
    "last_frame_age_ms": 20,
    "exit_reason": null
  }
}
```

`speaker_state` 闭集为 `idle|buffering|playing|fault`；`microphone_state` 为 `disabled|idle|recording|fault`。按钮选中态必须同时参考 `state/mode` 和本消息，不能只看命令 ack。

### 4.5 地理对象 manifest

Key：`xbrain/{rid}/state/geo/manifest`

session 建立后 2 秒内必须发布一份 `full=true` 的全量清单；清单变化时立即重发全量。

```json
{
  "v": 1,
  "rid": "gj-001",
  "ts": 1785732062.0,
  "seq": 4,
  "src": "p5_gateway",
  "data": {
    "manifest_rev": 12,
    "full": true,
    "objects": [
      {
        "geo_id": "w-north_gate",
        "type": "waypoint",
        "name": "北门",
        "rev": 3,
        "latitude": 31.2301971,
        "longitude": 121.4732683,
        "altitude": 8.4
      },
      {
        "geo_id": "r-route_north",
        "type": "recorded_path",
        "name": "北侧巡检路径",
        "rev": 5
      },
      {
        "geo_id": "f-alarm_equipment",
        "type": "alarm_region",
        "name": "设备区",
        "rev": 8,
        "enabled": true
      }
    ]
  }
}
```

| 字段 | 类型 | 必填 | 约束 |
|---|---|---:|---|
| `manifest_rev` | integer | 是 | 全局清单版本，单调递增 |
| `full` | boolean | 是 | 本期固定 true；消息为全量快照 |
| `objects` | array | 是 | 权威对象集合 |
| `geo_id` | string | 是 | waypoint=`w-...`、path=`r-...`、region=`f-...` |
| `type` | string | 是 | `waypoint|recorded_path|alarm_region` |
| `name` | string | 是 | 显示名 |
| `rev` | integer | 是 | 对象版本，新版本严格递增 |

waypoint 必须带 WGS84 坐标；alarm region 必须带 `enabled`。ID 创建后不得因改名而改变。

### 4.6 动态媒体端点

Key：`xbrain/{rid}/state/media`

端点变化时立即发布，正常时每 5 秒发布一份全量 `endpoints[]` 作为保活。

```json
{
  "v": 1,
  "rid": "gj-001",
  "ts": 1785732063.0,
  "seq": 5,
  "src": "p5_gateway",
  "data": {
    "endpoints": [
      {
        "id": "cam_ptz_vis",
        "stream": "main",
        "url": "rtsp://192.168.123.110:8554/ptz/visible/main",
        "state": "up",
        "credential_ref": "robot_gj001_ptz_vis",
        "max_bitrate_mbps": 6.0
      },
      {
        "id": "cam_ptz_vis",
        "stream": "sub",
        "url": "rtsp://192.168.123.110:8554/ptz/visible/sub",
        "state": "up",
        "credential_ref": "robot_gj001_ptz_vis",
        "max_bitrate_mbps": 2.0
      },
      {
        "id": "cam_ptz_ir",
        "stream": "main",
        "url": "rtsp://192.168.123.111:8554/ptz/ir/main",
        "state": "up",
        "credential_ref": "robot_gj001_ptz_ir",
        "max_bitrate_mbps": 6.0
      }
    ]
  }
}
```

| 字段 | 类型 | 必填 | 约束 |
|---|---|---:|---|
| `id` | string | 是 | `cam_ptz_vis|cam_ptz_ir|cam_rgbd` |
| `stream` | string | 是 | `main|sub` |
| `url` | string | 是 | 完整 RTSP URL；禁止明文用户名密码 |
| `state` | string | 是 | `up|down`；down 时 URL 可保留用于诊断但不得拉流 |
| `credential_ref` | string | 是 | 当前 endpoint 的运行时凭据引用 |
| `max_bitrate_mbps` | number | 是 | 端点配置上限；可见光 sub 不大于 2.0 |

每个 endpoint 独立使用自己的 `credential_ref`。后端不得要求 Qt 根据 main URL 拼接 sub URL。

当前 Qt 主控 PC 的部署出口地址为 `192.168.123.60`。后端/甲方 DNAT 与防火墙必须只允许该源地址访问 RTSP，禁止向 `0.0.0.0/0` 开放；地址变更时必须同步更新部署配置和白名单。

Qt 凭据文件不是协议报文，但其部署格式固定如下；文件权限应限制为当前运行用户可读，禁止进入版本库：

```json
{
  "credentials": {
    "robot_gj001_ptz_vis": {
      "username": "runtime-user",
      "password": "runtime-secret"
    },
    "robot_gj001_ptz_ir": {
      "username": "runtime-user",
      "password": "runtime-secret"
    }
  }
}
```

## 5. 可靠事件

Key：`xbrain/{rid}/event/{severity}/{category}`。正式 payload 字段为 `eid`、`sev`、`category`；不发布 `event_id`、`severity` 别名。

### 5.1 通用事件结构

```json
{
  "v": 1,
  "rid": "gj-001",
  "ts": 1785732070.25,
  "seq": 1001,
  "src": "p5_gateway",
  "data": {
    "eid": "evt-001",
    "sev": "info",
    "category": "system",
    "state": "active",
    "source": "p5_gateway",
    "code": "SYSTEM_READY",
    "title": "系统就绪",
    "message": "网关和任务域已就绪",
    "task_id": null,
    "operator": null,
    "result": null,
    "detail": {},
    "media": [],
    "file_refs": []
  }
}
```

| 字段 | 类型 | 必填 | 约束 |
|---|---|---:|---|
| `eid` | string | 是 | 可靠事件幂等 ID，补发复用 |
| `sev` | string | 是 | `info|warn|error|fatal`，与 key severity 一致 |
| `category` | string | 是 | 与 key category 一致 |
| `state` | string | 是 | `active|cleared|acknowledged|occurred` |
| `source/code/title/message` | string | 是 | 来源、机器可读码和人类可读信息 |
| `task_id/operator/result` | string/null | 是 | 不适用为 null |
| `detail` | object | 是 | 类别专用结构 |
| `media/file_refs` | array | 是 | 无引用为空数组 |

事件必须可靠保存并支持断线补发。Qt 以 `rid+eid` 去重；同一 eid 不得改写语义。

### 5.2 报警事件

示例 key：`xbrain/gj-001/event/error/alarm`

```json
{
  "v": 1,
  "rid": "gj-001",
  "ts": 1785732071.5,
  "seq": 1002,
  "src": "p5_gateway",
  "data": {
    "eid": "evt-alarm-001",
    "sev": "error",
    "category": "alarm",
    "state": "active",
    "source": "perception",
    "code": "PERSON_IN_REGION",
    "title": "人员进入设备区",
    "message": "目标在设备区持续停留 3.2 秒",
    "task_id": "task-goto-001",
    "operator": null,
    "result": "triggered",
    "detail": {
      "alarm_type": "person_in_region",
      "level": 1,
      "confidence": 0.94,
      "fence_set_id": "fence-set-main",
      "rev": 8,
      "matched_rule": "person_in_region:include",
      "zone_hits": ["f-alarm_equipment"],
      "dwell_s": 3.2,
      "track_id": "track-4711",
      "wpos": {
        "latitude": 31.23025,
        "longitude": 121.47331,
        "altitude_m": 8.2,
        "valid": true,
        "sigma_m": 0.18
      }
    },
    "media": [
      {
        "file_id": "file-snapshot-001",
        "kind": "snapshot",
        "source": "onboard_decision"
      }
    ],
    "file_refs": ["file-snapshot-001"]
  }
}
```

报警 `detail` 中 `alarm_type/fence_set_id/rev/matched_rule/zone_hits/dwell_s/track_id/wpos` 全部必填。`alarm_type` 闭集：

- `person_in_region`
- `vehicle_in_region`
- `rtk_lost`
- `heading_degraded`
- `heading_recovered`
- `system_alarm`

人员/车辆事件还必须带 `level/confidence`。`wpos.valid=false` 时仍保留对象，坐标字段可以为 null。

### 5.3 故障事件

示例 key：`xbrain/gj-001/event/fatal/fault`

```json
{
  "v": 1,
  "rid": "gj-001",
  "ts": 1785732072.0,
  "seq": 1003,
  "src": "p5_gateway",
  "data": {
    "eid": "evt-fault-001",
    "sev": "fatal",
    "category": "fault",
    "state": "active",
    "source": "payload",
    "code": "SPEAKER_FAULT",
    "title": "喇叭故障",
    "message": "载荷喇叭无响应",
    "task_id": "task-audio-001",
    "operator": null,
    "result": "failed",
    "detail": {
      "device_id": "speaker",
      "suggested_action": "检查载荷电源和通信链路",
      "location": {
        "latitude": 31.2301971,
        "longitude": 121.4732683
      }
    },
    "media": [],
    "file_refs": []
  }
}
```

故障清除事件必须生成新的 `eid`，保持相同 `code/source`，取 `state=cleared`，并在 `detail.ref_eid` 引用原 active 事件，同时给出 `cleared_ts` 和处理信息。这样既可关联原故障，又不会被 `rid+eid` 幂等去重误删。`source` 闭集为 `battery|motor|navigation|gps|camera|network|storage|alarm|payload|system`。

### 5.4 任务可靠事件

示例 key：`xbrain/gj-001/event/info/task`

```json
{
  "v": 1,
  "rid": "gj-001",
  "ts": 1785732125.55,
  "seq": 1004,
  "src": "p5_gateway",
  "data": {
    "eid": "evt-task-done-001",
    "sev": "info",
    "category": "task",
    "state": "occurred",
    "source": "p3_task",
    "code": "TASK_DONE",
    "title": "任务完成",
    "message": "导航任务正常完成",
    "task_id": "task-goto-001",
    "operator": null,
    "result": "done",
    "detail": {
      "task_type": "GOTO_KEYPOINT",
      "duration_sec": 85.35,
      "distance_m": 86.4,
      "ended_ts": 1785732125.55,
      "route_id": "r-route_north",
      "route_rev": 3
    },
    "media": [],
    "file_refs": []
  }
}
```

任务拒绝、开始、暂停、恢复、完成、失败、取消都必须生成可靠 task event。`state/task` result 用于界面终态；task event 用于审计和断线补发，两者使用相同 `task_id` 和结果值。

## 6. 文件交付

### 6.1 文件索引

Key：`xbrain/{rid}/data/file/index`

```json
{
  "v": 1,
  "rid": "gj-001",
  "ts": 1785732080.0,
  "seq": 12,
  "src": "p5_gateway",
  "data": {
    "msg_id": "file-index-msg-001",
    "index_id": "file-index-001",
    "generated_ts": 1785732080.0,
    "complete": true,
    "files": [
      {
        "file_id": "file-snapshot-001",
        "kind": "snapshot",
        "name": "evt-alarm-001.jpg",
        "relative_path": "snapshots/2026/08/evt-alarm-001.jpg",
        "size_bytes": 245760,
        "sha256": "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef",
        "created_ts": 1785732071.55,
        "ready": true,
        "priority": "high",
        "eid": "evt-alarm-001",
        "task_id": "task-goto-001",
        "session_id": null
      }
    ]
  }
}
```

| 字段 | 类型 | 必填 | 约束 |
|---|---|---:|---|
| `msg_id/index_id` | string | 是 | 索引消息和索引版本 ID |
| `generated_ts` | number | 是 | float64 Unix 秒 |
| `complete` | boolean | 是 | true 表示当前索引快照完整 |
| `files` | array | 是 | 可为空 |
| `file_id` | string | 是 | 稳定实体 ID |
| `kind` | string | 是 | `snapshot|video|log|report` |
| `name` | string | 是 | 仅文件名，不含目录分隔符 |
| `relative_path` | string | 是 | SFTP 配置根目录下相对路径；禁止绝对路径和 `..` |
| `size_bytes` | integer | 是 | uint64 语义 |
| `sha256` | string | 是 | 64 位小写十六进制 |
| `created_ts` | number | 是 | float64 Unix 秒 |
| `ready` | boolean | 是 | true 后才允许下载 |
| `priority` | string | 是 | `normal|high` |
| `eid/task_id/session_id` | string/null | 是 | 证据关联，不适用为 null |

文件实体通过部署配置中的只读 SFTP 连接拉取。报文和 URL 中不得出现用户名、密码或任意绝对文件系统路径。

### 6.2 文件下载回执

Key：`xbrain/{rid}/cmd/file/ack`

```json
{
  "v": 1,
  "rid": "gj-001",
  "ts": 1785732082.2,
  "seq": 1,
  "src": "qt_hmi",
  "data": {
    "msg_id": "file-ack-msg-001",
    "index_id": "file-index-001",
    "file_id": "file-snapshot-001",
    "result": "downloaded",
    "size_bytes": 245760,
    "sha256": "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef",
    "reason": ""
  }
}
```

`result` 闭集为 `downloaded|checksum_failed|download_failed`。失败时 `reason` 必填；成功时 size 和 sha256 必须等于索引值。

## 7. Qt 录像会话回写

### 7.1 会话记录

Key：`xbrain/{rid}/cmd/media/session`

```json
{
  "v": 1,
  "rid": "gj-001",
  "ts": 1785732200.0,
  "seq": 1,
  "src": "qt_hmi",
  "data": {
    "msg_id": "media-session-msg-001",
    "session_id": "video-session-001",
    "task_id": "task-goto-001",
    "event_ids": ["evt-alarm-001"],
    "source": "cloud_observed",
    "started_at": 1785732040.2,
    "ended_at": 1785732200.0,
    "robot_ts_at_start": 1785732039.95,
    "segments": [
      {
        "name": "seg_20260806_100000.mkv",
        "started_at": 1785732040.2,
        "ended_at": 1785732100.2,
        "size_bytes": 14800000,
        "sha256": "abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789"
      }
    ]
  }
}
```

| 字段 | 类型 | 必填 | 约束 |
|---|---|---:|---|
| `msg_id/session_id/task_id` | string | 是 | 会话和任务关联 |
| `event_ids` | array[string] | 是 | 无报警可为空 |
| `source` | string | 是 | 固定 `cloud_observed` |
| `started_at/ended_at` | number | 是 | Qt 墙钟的 float64 Unix 秒 |
| `robot_ts_at_start` | number/null | 是 | 开始录像前最近机器人事件 ts；没有则 null |
| `segments` | array | 是 | 至少一段；每段字段均必填 |

`segments[].name` 只允许文件名，不向机器人暴露 Qt 本地绝对目录。

### 7.2 会话回执

Key：`xbrain/{rid}/cmd/media/session/ack`

```json
{
  "v": 1,
  "rid": "gj-001",
  "ts": 1785732200.1,
  "seq": 1,
  "src": "p5_gateway",
  "data": {
    "msg_id": "media-session-ack-001",
    "ref_msg_id": "media-session-msg-001",
    "session_id": "video-session-001",
    "task_id": "task-goto-001",
    "result": "accepted",
    "accepted": true,
    "error_code": 0,
    "reason": ""
  }
}
```

`task_id` 必须等于原会话记录请求中的 `task_id`。`result` 为 `accepted|duplicate|rejected`。后端必须以 `rid+session_id` 幂等，duplicate 不重复写入证据库。

## 8. 实时喊话 AudioChunk

Key：`xbrain/{rid}/audio/broadcast`。每帧使用 JSON AudioChunk；音频固定 PCM S16LE、16 kHz、单声道、20 ms，即每帧解码后应为 640 字节。

```json
{
  "v": 1,
  "rid": "gj-001",
  "ts": 1785732030.22,
  "seq": 1,
  "src": "qt_hmi",
  "data": {
    "stream_id": "audio-gj001-0001",
    "chunk_seq": 1,
    "codec": "pcm_s16le",
    "sample_rate_hz": 16000,
    "channels": 1,
    "frame_duration_ms": 20,
    "payload_b64": "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=="
  }
}
```

| 字段 | 类型 | 必填 | 约束 |
|---|---|---:|---|
| `stream_id` | string | 是 | 等于 AUDIO_CONTROL ack/state 分配值 |
| `chunk_seq` | integer | 是 | uint32，会话内从 1 单调递增 |
| `codec` | string | 是 | 固定 `pcm_s16le` |
| `sample_rate_hz` | integer | 是 | 固定 `16000` |
| `channels` | integer | 是 | 固定 `1` |
| `frame_duration_ms` | integer | 是 | 固定 `20` |
| `payload_b64` | string | 是 | Base64；解码后恰为 640 字节 |

同一 `stream_id` 内回退或重复的 `chunk_seq` 丢弃；允许检测缺口但不得重排后无限等待。该 key 使用 Q4/Ring，拥塞时允许丢旧音频帧，不得阻塞急停和命令面。最后一帧有效音频 300 秒后后端自动退出 broadcast。

## 9. 多机器人、重连和补发规则

1. 每个机器人独立 session。任何消息的 `rid` 与收到它的 session/key 不一致时直接拒绝并记录协议错误。
2. 多机器人计划下发为 N 个独立 `GOTO_KEYPOINT`；后端分别 ack/result，不存在一个“总任务 ID”。
3. 状态缓存、任务关联、音频 `stream_id`、manifest、媒体 endpoint、文件索引和事件去重都按 rid 分区。
4. `seq` 只在发布进程重启后可从 1 重置；同一发布进程内的短线重连应继续原序号。接收方在 session 重建后允许建立新水位，不得因对端进程重启而永久拒绝低序号。
5. `event/**`、`data/file/index` 为可靠补发面，补发复用原 `eid/file_id/msg_id`；Qt 以业务 ID 去重，不因 seq 回退删除可靠消息。
6. 普通状态允许丢中间包但不得让旧包覆盖新状态；终态 result 和可靠 task event 不得丢失。

## 10. 错误码和拒绝要求

`error_code` 是 Qt 稳定整数码；后端原生字符串码放在 `detail.code`，不得用未知整数扩展替代原生码。

| code | 含义 | 典型 detail.code |
|---:|---|---|
| `0` | 已受理/成功 | 无 |
| `1001` | JSON 无法解析 | `E_JSON_PARSE` |
| `1002` | 缺少必填字段 | `E_REQUIRED_FIELD` |
| `1003` | 类型、范围、枚举或版本冲突 | `E_INVALID_FIELD`、`E_GEO_CONFLICT` |
| `1004` | rid 与 key/session 不匹配 | `E_RID_MISMATCH` |
| `1005` | 不支持的 `v` | `E_VERSION_UNSUPPORTED` |
| `1006` | 不支持的 task_type/key | `E_TASK_UNSUPPORTED`、`E_CHANNEL_DENIED` |
| `2001` | 机器人未就绪/授时未同步 | `E_NOT_READY`、`E_TS_UNSYNCED` |
| `2002` | 机器人忙或动作许可失败 | `E_BUSY`、`E_ROTATION_BLOCKED` |
| `2003` | 当前状态禁止该操作 | `E_STATE_DENIED` |
| `2004` | ID 冲突 | `E_ID_CONFLICT` |
| `2005` | 电量不足 | `E_LOW_BATTERY` |
| `2006` | 围栏/目标位置拒绝 | `E_OUT_OF_FENCE` |
| `2007` | 急停、锁定或安全互锁未解除 | `E_ESTOP_ACTIVE`、`E_LOCKED` |
| `3001` | 配置、持久化或文件操作失败 | `E_CONFIG`、`E_STORAGE` |

所有拒绝必须同时提供：

- `result=rejected`
- `accepted=false`
- 非零 `error_code`
- 人类可读 `reason`
- `detail.code`
- 能定位时提供 `field/index/waypoint_id/region_id/current_rev`

未知错误保留原始 `detail`，Qt 不自行扩展解释。每次任务拒绝还必须产生一条可靠 `event/{sev}/task`，保证断网后可审计。

## 11. 后端发送前检查清单

- [ ] `v=1`，`rid` 与 key 一致。
- [ ] `ts` 以及全部协议时间字段均为 float64 Unix 秒，不含 ISO 字符串。
- [ ] `seq` 按 rid＋完整 key 递增。
- [ ] ack 带齐 `ref_msg_id/task_id/task_type/result/accepted/error_code/reason/detail`。
- [ ] `state/task` 明确使用 `message_type=snapshot|result`。
- [ ] 未知进度为 `null`，不是 0。
- [ ] 终态只使用 `done|failed|cancelled`，并带权威 duration/distance/ended_ts。
- [ ] `state/link` 1 Hz，`state/robot`、audio、mode、manifest、media 均按各自 schema 发布。
- [ ] event key 的 severity/category 与 `data.sev/category` 逐字一致。
- [ ] 报警事件包含完整规则、区域、轨迹和世界坐标证据字段。
- [ ] 媒体 URL 不含凭据，每个 endpoint 有独立 credential_ref。
- [ ] 文件索引只有相对路径、size 和 SHA-256，不含口令和绝对路径。
- [ ] duplicate 不重复执行，可靠事件和文件支持补发。
- [ ] MANUAL_VELOCITY、云端 teleop、dog_to_pc、keep-in 修改和未知旧任务安全拒绝。
