# Qt 前端任务 JSON 格式

包含：
- Qt方发布的任务请求；
- 机器狗向 Qt 发布的任务 ack、progress 和 result；
- 全部任务类型的请求格式。
状态、故障、报警上报和文件传输不在本文档范围内。


## 1. 通用任务外层结构

所有任务请求使用以下公共结构，只有 `task_type` 和 `payload` 随任务类型变化：

```json
{
  "schema_version": 1,
  "msg_id": "MSG-20260803-000001",
  "task_id": "TASK-20260803-000001",
  "robot_id": "GJ-001",
  "timestamp": "2026-08-03T10:00:00.123+08:00",
  "task_type": "MANUAL_VELOCITY",
  "payload": {}
```

### 1.1 外层字段

| 字段 | JSON 类型 | 必填 | 说明 |
|---|---|---:|---|
| `schema_version` | integer | 是 | 协议版本 |
| `msg_id` | string | 是 | 单条消息编号 |
| `task_id` | string | 是 | 一次任务生命周期编号 |
| `robot_id` | string | 是 | 目标机器人 ID |
| `timestamp` | string | 是 | 消息生成时间 |
| `task_type` | string | 是 | 任务类型 |
| `payload` | object | 是 | 任务参数；无参数时为 `{}` |

### 1.2 字段来源与 ID 使用规则

这些字段由任务生成流程自动组成：

| 字段 | 数据来源 | 生成时机 |
|---|---|---|
| `schema_version` | Qt 与后端约定的协议版本常量 | 组装任务 JSON 时自动写入 |
| `msg_id` | Qt 消息编号生成器 | 每生成一条请求消息时自动生成 |
| `task_id` | 任务创建流程；巡检任务可关联计划编号 | 创建任务时自动生成，并在同一任务的 ack/progress/result 中复用 |
| `robot_id` | Qt 当前选中的机器狗ID，来自机器狗集群配置 | 一开始添加机器狗的时候就设定好 |
| `timestamp` | Qt 主机系统时间 | 组装消息时自动写入 |
| `task_type` | Qt 页面操作或任务调度器映射关系 | 根据具体任务类型写入 |
| `payload` | 页面控件、本地计划文件、关键点/路径/围栏配置和任务参数 | 根据任务类型组装 |

ID 规则：

- `msg_id` 标识一条消息；消息重发时应保持不变；
- `task_id` 标识一次任务生命周期；同一任务的 ack、progress、result 使用同一个值；
- 后端应以 `robot_id + msg_id` 做去重键；
- 重复消息不得重复启动导航、录像或其他有副作用的任务。
## 2. 任务 JSON 格式


### 2.1 `MANUAL_VELOCITY`——手动速度控制

对应Qt中鼠标点击控制机器狗移动的按钮

| Qt 按钮 |  | 发送的速度方向 |
|---|---|---|
| 前进 |  | `linear_x_mps` 为正 |
| 后退 |  | `linear_x_mps` 为负 |
| 左移 |  | `linear_y_mps` 为正 |
| 右移 |  | `linear_y_mps` 为负 |
| 左转 |  | `angular_z_radps` 为正 |
| 右转 |  | `angular_z_radps` 为负 |
| 驻停 |  | 三个速度分量均为 0 |

“加速”和“减速”按钮位于同一区域，只调整前端的速度档位，不单独发送 `MANUAL_VELOCITY`：

| Qt 按钮 |  | 作用 |
|---|---|---|
| 加速 |  | 直线速度档位增加 `0.5` |
| 减速 |  | 直线速度档位减少 `0.5` |

完整消息示例（此处示例是 Unitree Go2 机器狗的速度控制指令格式。）：

```json
{
  "schema_version": 1,
  "msg_id": "MSG-20260803-000001",
  "task_id": "TASK-20260803-000001",
  "robot_id": "GJ-001",
  "timestamp": "2026-08-03T10:00:00.123+08:00",
  "task_type": "MANUAL_VELOCITY",
  "payload": {
    "linear_x_mps": 0.5,
    "linear_y_mps": 0.0,
    "angular_z_radps": 0.0,
    "duration_ms": 300,
    "control_mode": "jog"
  }
}
```

payload 字段：

| 字段 | 类型 | 必填 | 值/范围 | 说明 |
|---|---|---:|---|---|
| `linear_x_mps` | number | 是 | 建议限制在 `-1.5～1.5` | 前后速度；正值前进，负值后退 |
| `linear_y_mps` | number | 是 | 建议限制在 `-1.5～1.5` | 横移速度；正值左移，负值右移 |
| `angular_z_radps` | number | 是 | 按机器人能力限制 | 偏航角速度；正值左转，负值右转 |
| `duration_ms` | integer | 是 | 示例为 `300` | 命令有效期，单位毫秒 |
| `control_mode` | string | 是 | `jog` | 点动控制模式 |

字段说明：

- `linear_x_mps`：机身前后方向的线速度；`mps` 是 `meters per second`，即“米/秒”；
- `linear_y_mps`：机身左右方向的线速度，正值表示左移，负值表示右移；
- `angular_z_radps`：绕机身垂直轴的角速度；
- `duration_ms`：本条速度命令的有效时间；`ms` 是毫秒。超过该时间没有新的速度命令时，后端必须自动输出零速度；
- `control_mode`：控制方式。当前正式协议只定义 `jog`（点动），Qt 没有控件可以切换其他模式；驻停使用零速度或 `STOP_TASK`，不另设 `stop` 模式。

方向与速度分量约定：

| Qt 按钮 | `linear_x_mps` | `linear_y_mps` | `angular_z_radps` |
|---|---:|---:|---:|
| 前进 | `+max_linear_speed` | 0 | 0 |
| 后退 | `-max_linear_speed` | 0 | 0 |
| 左移 | 0 | `+max_linear_speed` | 0 |
| 右移 | 0 | `-max_linear_speed` | 0 |
| 左转 | 0 | 0 | `+1.0` |
| 右转 | 0 | 0 | `-1.0` |
| 驻停 | 0 | 0 | 0 |

连续点动（鼠标点击）时，前端可以重复发送相同方向的速度消息；每条消息应使用新的 `msg_id`，同一连续控制过程可以共用一个 `task_id`。后端以最新速度命令刷新计时器。

可以增加键盘控制，不新增 JSON 类型，仍然发送MANUAL_VELOCITY，W：前进
S：后退
A：左移
D：右移
Q：左转
E：右转
Space：驻停


### 2.2 `STOP_TASK`——停止任务

完整消息示例：

```json
{
  "schema_version": 1,
  "msg_id": "MSG-20260803-000010",
  "task_id": "TASK-20260803-000010",
  "robot_id": "GJ-001",
  "timestamp": "2026-08-03T10:05:00.000+08:00",
  "task_type": "STOP_TASK",
  "payload": {}
}
```
取消当前导航、巡检任务；向运动控制发送零速度；回复停止命令的 ack，空 payload 表示停止该机器人当前活动任务。若后端需要指定目标任务、停止原因或停止模式，应在正式协议中增加并固定相应字段。

数据来源：任务管理页面的“停止任务”按钮、任务取消操作或安全策略触发。由于停止命令本身不需要目标点和速度参数，payload 可以为空。

### 2.3 `SET_ALARM_CONFIG`——报警参数与区域配置


完整消息示例：

```json
{
  "schema_version": 1,
  "msg_id": "MSG-20260803-000030",
  "task_id": "TASK-20260803-000030",
  "robot_id": "GJ-001",
  "timestamp": "2026-08-03T10:15:00.000+08:00",
  "task_type": "SET_ALARM_CONFIG",
  "payload": {
    "alarm_level": 1,
    "alarm_start": "2026-08-03T10:15:00+08:00",
    "alarm_end": "2026-08-04T10:15:00+08:00",
    "volume": 7,
    "times": 5,
    "cooldown_sec": 2.0,
    "auto_off": true,
    "alarm_events": {
      "person": {"enabled": true, "type": 1},
      "vehicle": {"enabled": false, "type": 1}
    },
    "regions": [
      {
        "id": "alarm_equipment",
        "name": "设备区",
        "type": "alarm_region",
        "enabled": true,
        "vertices": [
          {"latitude": 31.2301971, "longitude": 121.4732683},
          {"latitude": 31.2301971, "longitude": 121.473864},
          {"latitude": 31.2305962, "longitude": 121.473864},
          {"latitude": 31.2305962, "longitude": 121.4732683}
        ]
      }
    ]
  }
}
```

字段来源与含义：

| 字段 | 类型 | 数据来源 | 含义 |
|---|---|---|---|
| `alarm_level` | integer | 报警等级下拉框（主要区别在报警器报警触发的反应：声+光） | `1（爆闪灯+声音报警）、2（常亮灯+声音报警）` |
| `alarm_start` | string | 是否能触发报警的时间窗口控件 | ISO 8601 时间，在这个时间段内可以触发报警start-end，格式：2026-08-03-20：18 |
| `alarm_end` | string | 报警时间窗口结束控件 | ISO 8601 时间，设定停止触发报警的时间，格式 |
| `volume` | integer | 音量 SpinBox | `报警器报警时声音的音量1～10` |
| `times` | integer | 播放次数 SpinBox | `报警器报警时声音的循环次数1～999` |
| `cooldown_sec` | number | 报警冷却 SpinBox | `0.5～60.0` 秒 （同一类报警触发后，在该时间内不重复触发）|
| `auto_off` | boolean | 自动关闭报警复选框 | 是否自动关闭，报警触发后是否自动停止声音/报警输出 |
| `alarm_events.person.enabled` | boolean | 人员识别复选框 | 是否启用人员识别报警 |
| `alarm_events.person.type` | integer | 人员报警类型下拉框 | `1` 陌生人，`2` 人员禁入区（看到人就报警） |
| `alarm_events.vehicle.enabled` | boolean | 车辆识别复选框 | 是否启用车辆识别 |
| `alarm_events.vehicle.type` | integer | 车辆报警类型下拉框 | `1` 陌生车辆，`2` 车辆禁行区 |
| `regions` | array | GPS 地图编辑器和 `config/geo_fences.json` | 报警区域配置（可配置多个regions） |
| `regions[].id` | string | 围栏配置 | 区域 ID（新建时自动生成） |
| `regions[].name` | string | 围栏配置 | 区域名称（设置报警区域的时候手动输入的） |
| `regions[].type` | string | 围栏配置 | `keep_in（营区边界）` 或 `alarm_region（报警区域）` |
| `regions[].enabled` | boolean | 围栏配置 | 是否启用（保存报警区域后自动写入） |
| `regions[].vertices` | array | GPS 地图编辑器（用户在地图上拖拽绘制矩形报警区域，根据矩形起点、终点自动计算四个顶点），latitude，longitude	地图坐标转换结果，自动生成 | WGS84 顶点数组 |

后端应校验时间范围、数值范围、区域 ID 和经纬度。

### 2.4 `AUDIO_CONTROL`——音频通道控制

PC 向机器狗发送声音（只发声音，不收音）：

```json
{
  "schema_version": 1,
  "msg_id": "MSG-20260803-000040",
  "task_id": "TASK-20260803-000040",
  "robot_id": "GJ-001",
  "timestamp": "2026-08-03T10:20:00.000+08:00",
  "task_type": "AUDIO_CONTROL",
  "payload": {
    "mode": "pc_to_dog",
    "action": "start"
  }
}
```

机器狗向 PC 发送声音（只收音，不发音）：

```json
{
  "schema_version": 1,
  "msg_id": "MSG-20260803-000041",
  "task_id": "TASK-20260803-000041",
  "robot_id": "GJ-001",
  "timestamp": "2026-08-03T10:20:01.000+08:00",
  "task_type": "AUDIO_CONTROL",
  "payload": {
    "mode": "dog_to_pc",
    "action": "start"
  }
}
```

payload 字段：

| 字段 | 类型 | 必填 | 枚举 |
|---|---|---:|---|
| `mode` | string | 是 | `pc_to_dog`、`dog_to_pc` |
| `action` | string | 是 | `start`、`stop` |

两个 Qt 音频按钮互斥。切换方向时，Qt 会先对原方向发送 `stop`，再对新方向发送 `start`。Zenoh 任务只负责启停控制；实时音频流的编码、端口和传输通道由后端媒体协议另行实现。

数据来源：`mode` 来自“发送声音”或“接收声音”按钮，`action` 来自按钮的选中/取消状态。Qt 没有独立的音频会话 ID 输入框，后端可以在首次 `start` 时自行生成会话标识。

### 2.5 `GOTO_KEYPOINT`——导航

新建好计划后，下发计划的对应指令。计划中可以包含一个或多个导航点，严格按 `payload.waypoints` 数组顺序执行；导航点数量不会改变 `task_type` 或 JSON 结构。

```json
{
  "schema_version": 1,
  "msg_id": "MSG-20260803-000050",
  "task_id": "TASK-20260803-000050",
  "robot_id": "GJ-001",
  "timestamp": "2026-08-03T10:30:00.000+08:00",
  "task_type": "GOTO_KEYPOINT",
  "payload": {
    "coordinate_system": "WGS84",
    "waypoints": [
      {
        "id": "kp_north_warehouse",
        "name": "北侧仓库",
        "latitude": 31.23076,
        "longitude": 121.4737,
        "altitude": 8.4,
        "arrival_radius_m": 3.0,
        "recorded_path_id": "route_north"
      }
    ]
  }
}
```

字段来源与含义：

| 字段 | 类型 | 数据来源 | 含义 |
|---|---|---|---|
| `coordinate_system` | string | Qt 导航发送代码固定填写 `WGS84` | 导航点经纬度所用坐标系，跟GPS有关 |
| `waypoints` | array | 计划编辑页面添加、排序后的目标点列表 | 一个或多个按顺序执行的目标点 |

`waypoints[]` 字段来源与含义：

| 字段 | 数据来源 | 含义 |
|---|---|---|
| `id` | 关键点配置或计划编辑器(地图上已设置的关键点ID直接读取；为空时使用 name 作为备用 ID) | 关键点 ID |
| `name` | 关键点配置或计划编辑器（创建关键点时填写） | 关键点名称 |
| `latitude` / `longitude` | 地图选点结果或关键点配置 | WGS84 经纬度 |
| `altitude` | 地图选点/关键点配置 | 高度，单位米 |
| `arrival_radius_m` | 关键点配置或计划参数 | 到达目标点的判定半径，单位米 |
| `recorded_path_id` | 关键点配置中绑定的录制路径的ID | 去该点时使用哪一条录制好的路径 |

导航后端处理顺序：读取第一个 waypoint → 根据 `recorded_path_id` 选择路径 → 到达判定 → 读取下一个 waypoint。如果没有绑定录制路径，"recorded_path_id": ""，字段可以为空：自主探索导航规划前往该点。

## 3. 后端向 Qt 返回的任务消息

Qt 已订阅 `task/ack`、`task/progress` 和 `task/result`。正式后端应按以下字段发布。

### 3.1 Ack

Key：

```text
robots/{robot_id}/task/ack
```

示例：

```json
{
  "schema_version": 1,
  "msg_id": "ACK-20260803-000001",
  "ref_msg_id": "MSG-20260803-000020",
  "task_id": "TASK-20260803-000020",
  "robot_id": "GJ-001",
  "timestamp": "2026-08-03T10:10:00.100+08:00",
  "accepted": true,
  "state": "ACCEPTED",
  "reason": ""
}
```

| 字段 | 类型 | 必填 | 数据来源 | 后端用途 | Qt 用途 |
|---|---|---:|---|---|---|
| `schema_version` | integer | 建议 | 后端协议常量 | 说明反馈消息使用的协议版本 | 判断是否兼容该反馈格式 |
| `msg_id` | string | 建议 | 后端反馈消息 ID 生成器 | 标识这一条 ack 消息，便于日志和去重 | 记录反馈消息 ID |
| `ref_msg_id` | string | 是 | 复制 Qt 请求中的 `msg_id` | 将 ack 与具体请求一一对应 | 找到对应的请求消息 |
| `task_id` | string | 是 | 复制 Qt 请求中的 `task_id` | 关联同一任务生命周期 | 关联任务状态 |
| `robot_id` | string | 是 | Zenoh key 和请求消息 | 标识产生反馈的机器狗 | 匹配当前机器狗 |
| `timestamp` | string | 是 | 后端系统时钟 | 记录后端接收/处理 ack 的时间 | 显示或记录反馈时间 |
| `accepted` | boolean | 是 | 后端校验结果 | 表示请求是否被接收 | `true` 继续等待执行，`false` 显示拒绝 |
| `state` | string | 是 | 后端 ack 状态机 | `ACCEPTED` 或 `REJECTED` | 判断接收结果 |
| `reason` | string | 拒绝时必填 | 后端校验器或任务处理器 | 说明拒绝原因，供日志和排障使用 | 显示拒绝提示 |

正式后端应使用 `reason` 作为拒绝原因；`error_message` 不作为本协议字段。

### 3.2 Progress

Key：

```text
robots/{robot_id}/task/progress
```

示例：

```json
{
  "schema_version": 1,
  "msg_id": "PROG-20260803-000001",
  "task_id": "TASK-20260803-000020",
  "robot_id": "GJ-001",
  "timestamp": "2026-08-03T10:12:00.000+08:00",
  "state": "RUNNING",
  "current_waypoint_id": "kp_north_warehouse",
  "current_path_id": "route_north",
  "completed_count": 1,
  "total_count": 2,
  "progress_percent": 50,
  "message": "已到达北侧仓库"
}
```

| 字段 | 类型 | 必填 | 数据来源 | 后端用途 | Qt 用途 |
|---|---|---:|---|---|---|
| `schema_version` | integer | 建议 | 后端协议常量 | 标识反馈格式版本 | 判断兼容性 |
| `msg_id` | string | 建议 | 后端进度消息 ID 生成器 | 标识一次进度发布 | 记录进度消息 |
| `task_id` | string | 是 | 复制请求中的任务 ID | 将进度归属于一个任务 | 匹配当前任务 |
| `robot_id` | string | 是 | Zenoh key 和请求消息 | 标识上报机器人 | 匹配当前机器狗 |
| `timestamp` | string | 是 | 后端系统时钟 | 记录进度产生时间 | 显示或记录更新时间 |
| `state` | string | 是 | 后端任务状态机 | `QUEUED`、`RUNNING` 或后端约定的暂停状态 | 更新任务状态文本 |
| `current_waypoint_id` | string | 否 | 导航执行器当前目标点 | 指示当前执行到哪个路点 | 更新目标点高亮/状态 |
| `current_path_id` | string | 否 | 当前路点绑定的 `recorded_path_id` | 指示当前使用的录制路径 | 显示当前路径信息 |
| `completed_count` | integer | 否 | 后端任务计数器 | 统计已完成路点数量 | 更新完成数量 |
| `total_count` | integer | 否 | 请求中的 `waypoints` 数量 | 提供任务总路点数量 | 更新总数量 |
| `progress_percent` | integer | 是 | 后端根据计数和执行阶段计算 | 提供统一的 `0～100` 进度值 | 直接用于进度条 |
| `message` | string | 否 | 后端任务处理器 | 提供人可读的阶段说明 | 显示进度提示 |

### 3.3 Result

Key：

```text
robots/{robot_id}/task/result
```

示例：

```json
{
  "schema_version": 1,
  "msg_id": "RES-20260803-000001",
  "task_id": "TASK-20260803-000020",
  "robot_id": "GJ-001",
  "timestamp": "2026-08-03T10:20:00.000+08:00",
  "state": "SUCCEEDED",
  "result_code": 0,
  "message": "导航完成",
  "summary": {
    "completed_count": 2,
    "total_count": 2,
    "distance_m": 86.4,
    "duration_sec": 600.0
  }
}
```

| 字段 | 类型 | 必填 | 数据来源 | 后端用途 | Qt 用途 |
|---|---|---:|---|---|---|
| `schema_version` | integer | 建议 | 后端协议常量 | 标识反馈格式版本 | 判断兼容性 |
| `msg_id` | string | 建议 | 后端结果消息 ID 生成器 | 标识这一条最终结果 | 记录结果消息 |
| `task_id` | string | 是 | 复制请求中的任务 ID | 关联完整任务生命周期 | 找到对应任务 |
| `robot_id` | string | 是 | Zenoh key 和请求消息 | 标识执行机器人 | 匹配当前机器狗 |
| `timestamp` | string | 是 | 后端系统时钟 | 记录任务结束或失败时间 | 显示完成时间 |
| `state` | string | 是 | 后端任务状态机 | `SUCCEEDED`、`FAILED`、`CANCELLED` 或 `TIMEOUT` | 判断任务最终状态 |
| `result_code` | integer | 是 | 后端结果码定义 | `0` 表示成功，其他值说明失败类别 | 判断成功并显示错误码 |
| `message` | string | 否 | 后端任务处理器 | 提供人可读的结果说明 | 显示结果提示 |
| `summary` | object | 否 | 后端任务统计结果 | 汇总路点、距离和耗时 | 更新巡检记录和报告 |
| `summary.completed_count` | integer | 否 | 后端任务计数器 | 统计完成路点数量 | 更新巡检记录 |
| `summary.total_count` | integer | 否 | 请求中的 `waypoints` 数量 | 记录任务总路点数量 | 更新巡检记录 |
| `summary.distance_m` | number | 否 | 后端定位/里程统计 | 记录行驶距离，单位米 | 显示或写入报告 |
| `summary.duration_sec` | number | 否 | 后端任务计时器 | 记录执行时长，单位秒 | 显示或写入报告 |

Qt 将 `state == "SUCCEEDED"` 判定为成功，其他状态表示未成功完成。

## 4. 后端校验

### 4.1 消息校验

后端应拒绝以下消息并返回 `accepted=false` 的 ack：

- JSON 无法解析；
- 缺少 `msg_id`、`task_id`、`robot_id`、`task_type` 或 `payload`；
- key 中机器人 ID 与 JSON 的 `robot_id` 不一致；
- `schema_version` 不支持；
- `task_type` 未实现；
- payload 字段类型或范围错误；
- 机器人状态不允许执行该任务。

### 4.2 建议错误码

| 错误码 | 含义 |
|---:|---|
| 0 | 已接收 |
| 1001 | JSON 解析失败 |
| 1002 | 缺少必填字段 |
| 1003 | 字段类型或范围错误 |
| 1004 | `robot_id` 不匹配 |
| 1005 | 不支持的 `schema_version` |
| 1006 | 不支持的 `task_type` |
| 2001 | 机器人未就绪 |
| 2002 | 机器人忙 |
| 2003 | 当前状态禁止该操作 |
| 2004 | 消息重复或任务 ID 冲突 |
| 3001 | 配置解析或保存失败 |

若使用错误码，可在 ack 中增加：

```json
{
  "error_code": 1003,
  "reason": "waypoints[0].latitude 超出范围"
}
```

Qt 会忽略未使用的附加字段，但 `reason` 应保留为可读说明。