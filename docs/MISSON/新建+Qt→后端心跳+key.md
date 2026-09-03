## 正式 key 定义

新增加：

```markdown
| `xbrain/{rid}/heartbeat/qt` | Qt → 后端 | 1 Hz；连接成功后立即发；非持久、禁止补发 | Qt HMI 在线心跳，供后端判断当前控制端是否在线 |
```



示例：

```json
{
  "v": 1,
  "rid": "gj-001",
  "ts": 1785732060.0,
  "seq": 30,
  "src": "qt_hmi",
  "data": {
    "session_id": "hmi-session-550e8400-e29b-41d4-a716-446655440000",
    "state": "up"
  }
}
```

字段定义：

| 字段 | 类型 | 必填 | 约束 |
|---|---|---:|---|
| `session_id` | string | 是 | 每次 Qt 进程启动或重新创建 Zenoh session 时生成新值；同一连接周期保持不变 |
| `state` | string | 是 | `up\|down`；周期心跳固定为 `up`，正常退出时可发送一次 `down` |

外层字段继续遵守现有冻结规则：

- `v=1`
- `rid` 与 key 第二段一致
- `src="qt_hmi"`
- `seq` 按完整 key 独立递增
- `ts` 为 float64 Unix 秒，但不用于心跳超时计算
