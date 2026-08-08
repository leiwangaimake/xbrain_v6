<!--
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: data/README.md
Brief: Runtime data root -- what deployment writes here, why it does not enter git
-->

# `data/` — 运行时数据根

## 定位

`/opt/xbrain_v6/data/` 是系统**运行期唯一可写数据根**（CLAUDE.md §0.2 目录铁律）。三类内容:

- **编译数据**: TRT engine / ONNX 加速产物 / 模型量化输出
- **依赖数据**: 部署时下载的模型权重（ASR / LLM）、语音素材、地图瓦片
- **日志**: 各常驻进程运行日志、四库 SQLite 文件 (`task.db` / `fence.db` / `geo.db` / `record.db`)

## 为什么不入库

`.gitignore` 忽略 `data/*`，只保留本 README 与 `.gitkeep` 让空目录本身入库。理由:

1. **模型权重**约 2.8 GB（ASR paraformer / LLM Qwen2.5-3B / TRT engines），入库把仓库炸到 3.0 GB。
2. **数据库文件**运行期高频写入，入库会让每次 pull 造成合并冲突。
3. **日志**逐秒增长且含运行期敏感信息（RTK 坐标、云端 token 等）。
4. **部署时可再取**: 模型走部署脚本（`deploy/systemd/xbrain-model-fetch.service` 或手动 scp），数据库首次运行自建 schema。

## 目录布局（约定，不强制）

```
data/
├── build/          # 编译产物 (TRT engine 等)
├── logs/           # 各进程日志; 由 logrotate 按 §11 保留策略回收
├── models/         # 部署脚本从内网 mirror 下载的模型权重
├── sounds/         # payload-service 用的音效素材
├── snap/           # 事件抓拍存档 (event/{sev}/*, U18 补发通道用)
├── outbox/         # 交付台账 outbox 目录 (17 §5.0.4 DP-6)
├── speech_presets/ # 预设语句 WAV (11 §8.8.2 离线预合成)
├── task.db         # P3 任务库
├── fence.db        # P3 围栏库
├── geo.db          # P3 地理对象库
└── record.db       # P5 事件与交付记录
```

## 引用点

- **持续化定位**: `configs/p5_gateway.yaml` `disk.data_root: /data/xbrain` (17 §10.1)
- **磁盘水位守卫**: `disk.watermark` (17 §7.1A DP-7)
- **保留策略**: `event.retention_days: 90` / `delivery.retention_days: 90` (17 §5.0.5)
