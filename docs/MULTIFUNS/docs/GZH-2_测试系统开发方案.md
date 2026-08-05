# GZH-2 测试系统开发方案（架构 · 接口 · 实现规范）

> 目的：把 xbrain 集成测试系统的架构、接口、模块划分、时序和验收细化成**开发级规范**，作为后续所有测试系统代码实现的唯一指导文档。写代码前先读本文，写代码时对照本文。
>
> 定稿日期：2026-07-25。上游依据：协议见 `GZH-2_控制协议开发文档.md`，真机验证与踩坑见 `GZH-2_测试调试与xbrain_v6开发笔记.md`。
>
> 运行环境：NVIDIA Jetson Orin（aarch64，hostname `xbrain`）；设备 `192.168.144.38`，端口 **8519 音频 / 8529 灯光**。

---

## 0. 最高开发准则（HARD RULES，不可违反）

**R0 — 全部从0原创实现。** 本项目后续所有功能代码必须完整地、从零编写。**不允许 `import` 现有 `tools/` 下的 probe 脚本（probe_8519 / probe_8529 / patrol_deter / siren_gen），不允许从任何外部项目、网页、样例复制粘贴功能代码。** 协议知识以 `docs/` 里的规格文档为唯一来源，据此重新实现帧格式、CRC、编解码封装、音频编排、状态机等全部逻辑。

- **prototype vs product 的界线**：`tools/` 下的 probe 是**原型/调试工具**，只作为"已验证的协议知识"参考，产品代码一行都不得引用它们。产品代码放在 `services/` 下，独立成树。
- **允许的第三方依赖 = 基础设施，而非"借用的功能实现"**（见 §11 明确白名单）。判定标准：它是"运行时/框架/国际标准编解码绑定/数值基础库"，还是"别人写好的本项目功能逻辑"。前者允许，后者禁止。若对某依赖是否越界有疑问，先在本文 §11 登记并说明，宁可自己写。

**R1 — 单一持锁者。** 设备的两条裸 TCP（8519/8529）**只允许 payload-service 一个进程持有**。其它任何进程/模块都不得直接连设备端口，一律走 payload-service 的本地 API。

**R2 — 单一音频客户端。** payload-service 的音频流（`/mic`、`/play`）在**任一时刻只有一个客户端 = AI_runtime**。这条不变量是功能1/功能2 互斥的物理保证。

**R3 — 单一模式活跃。** 同一时刻只有一个工作模式（IDLE / 功能1 / 功能2 / 功能3）活跃，由 AI_runtime 的状态机决定、由 payload-service 在 socket 层兜底强制。

---

## 1. 术语与角色

| 名称 | 含义 |
|---|---|
| **三合一 / 设备** | GZH-2 机器狗版多合一（二代），本单元含 喊话(喇叭)/照明(探照灯)/红蓝警示/收音(MIC) |
| **功能1 近场AI** | 人在机器狗附近与其语音对话：MIC→ASR→LLM→设备TTS。半双工回合制 |
| **功能2 远场对讲** | 办公室 ↔ 机器狗 原样语音互通，**不经 ASR/LLM**。半双工 PTT（MVP） |
| **功能3 驱离** | 红蓝双闪 + 警笛 + 男声TTS 循环，威慑闯入者。灯+喇叭并发 |
| **payload-service** | 新写的设备封装服务，持有 8519/8529，对上暴露干净 API |
| **asr-service** | 独立 ASR 推理服务，OpenAI 兼容接口，纯推理、不碰设备 |
| **llama-server** | 独立 LLM 服务，OpenAI `/v1/chat/completions` |
| **AI_runtime** | 机器人控制系统（版本号 v5/v6 待定），编排大脑：模式状态机 + 音频路由 + VAD + 回合调度 |
| **office-client** | 功能2 办公端小客户端（麦克风/喇叭 + PTT），连 AI_runtime |

> ⚠️ 命名：本文用中性的 **AI_runtime** 指机器人控制系统；此前笔记写作 xbrain_v6，用户口述为 xbrain_v5。版本号定稿前统一用 AI_runtime，落地时再替换。

---

## 2. 总体架构框图

```
     ┌──────────────────────── AI_runtime (机器人控制 / 编排大脑) ─────────────────────────┐
     │  模式状态机:  IDLE  ·  功能1 近场AI  ·  功能2 远场对讲(半双工)  ·  功能3 驱离           │
     │  职责: 唯一音频路由器 + VAD + 回合调度 + 模式互斥                                      │
     └───┬───────────────┬────────────────────┬────────────────────────────┬─────────────┘
         │ OpenAI        │ OpenAI             │ 控制面 REST                 │ 数据面 WS
         │ /v1/chat      │ /v1/audio/         │ /mode /tts /volume         │ /mic  (设备→)
         │ (SSE stream)  │ transcriptions     │ /lights /deter /status     │ /play (→设备)
         ▼               ▼                    ▼                            ▼
   ┌────────────┐  ┌────────────┐     ┌────────────────────────────────────────────────┐
   │ llama-server│  │ asr-service│     │            payload-service (三合一封装)           │
   │  LLM  独立   │  │  ASR  独立  │     │  唯一持有 8519(音频)/8529(灯光) 两条 TCP 长连接      │
   │  OpenAI API │  │  OpenAI API │     │  封装: Opus编解码 · 8k↔16k重采样 · CRC · 帧格式     │
   └────────────┘  └────────────┘     │  音频流全局只有一个客户端 = AI_runtime            │
        ▲ office-client(功能2)         └───────────────┬────────────────┬───────────────┘
        └────────── WS PCM ── AI_runtime         8519 │ Opus       8529 │ 8D帧
                                                       ▼                ▼
                                              ┌──────────────────────────────────┐
                                              │   GZH-2 三合一 (192.168.144.38)     │
                                              │ 收音[40]16k · 喇叭[42]8k / TTS[31]  │
                                              │ 探照灯 · 红蓝警示灯                  │
                                              └──────────────────────────────────┘
```

传输层统一 HTTP/WS（与 llama-server、ASR 天然一致，便于 curl/wscat 单测；不引 gRPC 的 codegen 负担）。四个进程同机运行在 Orin 上。

---

## 3. 设计原则（决定接口形态的三条根据）

1. **设备封装成"唯一持锁者"**：Opus、8k↔16k 重采样、CRC-8/MAXIM、0x25 状态、close 前 flush 防 RST 丢帧等脏细节全部锁进 payload-service，对上只暴露 PCM + REST。
2. **AI_runtime 是唯一音频路由器**：麦克风的消费者、喇叭的来源都随模式变（见下表），只有 AI_runtime 知道当前模式，故读 MIC / 喂喇叭必须由它做。
3. **ASR/LLM 是纯推理服务**：只接收数据、返回文本，完全不知设备存在。ASR 不去 pull 麦克风，而是由 AI_runtime 喂音频进来。

| | 麦克风[40] 去向 | 喇叭[42] 来源 |
|---|---|---|
| 功能1 | → ASR | ← 设备TTS(LLM文本) |
| 功能2 | → 办公室 | ← 办公室MIC |
| 功能3 | （不用） | ← 警笛 + TTS |

> 对上游数据流的**一处修正**：用户初稿是"ASR 申请三合一的 MIC"。本方案改为 **AI_runtime 读 MIC → 喂 ASR**。逻辑数据流不变（语音→ASR→文本→AI_runtime→LLM→文本→设备），物理接线上让 ASR 不直连设备——否则 ASR 会绕过模式互斥、与功能2 抢麦，且破坏 R2。

---

## 4. 进程清单与职责边界

| 进程 | 是否本项目新写 | 职责 | 明确不做 |
|---|---|---|---|
| **payload-service** | 是（核心交付） | 持有设备两条 socket；协议帧封装/解析；Opus 编解码 + 重采样；模式闸门；灯光/驱离；MIC/PLAY 流 | 不做 VAD、不做 ASR/LLM 调用、不做回合调度 |
| **AI_runtime** | 是（编排逻辑，可能并入既有机器人系统） | 模式状态机；VAD；功能1 回合调度；功能2 PTT 路由；调 ASR/LLM；驱动 payload-service | 不直接碰设备 socket |
| **asr-service** | 否（既有推理服务） | 音频→文本 | 不碰设备 |
| **llama-server** | 否（既有） | 文本对话 | — |
| **office-client** | 是（功能2 用，轻量） | 办公端 MIC/喇叭 + PTT，连 AI_runtime | 不直连设备 |

---

## 5. 接口规范（开发级，含具体 schema）

所有 payload-service 接口监听本机 `127.0.0.1:<PORT_PAYLOAD>`（默认 `18080`，见 §10）。请求/响应 JSON 用 UTF-8。

### 5.1 payload-service — 控制面（REST）

**`GET /status`** — 探测设备与当前状态
```
200 → {
  "mode": "idle|func1|func2|deter",
  "device": { "audio_connected": bool, "lights_connected": bool },
  "volume": int|null,          // 本单元无 [99] 自报，可能为 null（需容错）
  "temperature": number|null,
  "lights": { "searchlight": bool, "bright": 0..30, "redblue": 0..16 }  // 由 0x25 解析
}
```

**`POST /mode`** — **模式互斥执行点**
```
body → { "mode": "idle|func1|func2|deter" }
200  → { "ok": true, "mode": "<new>", "previous": "<old>" }
```
语义：切换时**先拆掉上一模式占用的音频流**（关闭 `/mic`/`/play` 会话、停 [40]/[42]/TTS、停驱离循环），再进入新模式。非法转换返回 `409`。

**`POST /tts`** — 功能1 输出（设备端 TTS，走 [31]）
```
body → { "voice": 0|1, "text": "...", "loop": false }   // 0=男 1=女
200  → { "ok": true, "est_ms": int }                      // 估算播放时长，供 AI_runtime 门控计时
```
`est_ms` 估算公式见 §7；因设备**无 TTS-done 事件**，此值是 AI_runtime 关闭麦克风门的依据。

**`POST /volume`** → `{ "level": 0..100 }` → `{ "ok": true }`（走 [14]，hex 编码在服务内部处理）

**`POST /lights`** — 灯光（8529；功能3 或独立调灯）
```
body → { "searchlight": bool?, "bright": 0..30?, "strobe": bool?, "redblue": 0..16? }  // 缺省项不改
200  → { "ok": true }
```

**`POST /deter`** — 功能3 驱离（payload 内部自跑灯+警笛+TTS 循环）
```
body → { "on": bool, "mode": 1..16?, "siren_level": 0..1?, "tts_reps": int? }
200  → { "ok": true }
```

### 5.2 payload-service — 数据面（WebSocket，二进制 PCM）

**约定**：连接建立后，服务端先发一条 JSON 文本帧声明格式，其后全部为二进制帧，每帧一块 PCM。PCM 一律 **单声道 s16le（小端 16bit）**。

**`WS /mic`**（服务端 → 客户端；仅 func1/func2 有效）
- 格式：**16000 Hz，20 ms/帧 = 320 采样 = 640 字节/帧**。
- 服务内部把设备 [40] 的 Opus 解码后重切成 20ms 块推送。
- 若当前模式不允许（idle/deter），连接被拒（`403`）或立即关闭。

**`WS /play`**（客户端 → 服务端；func2 或功能1 流式音频用）
- 接受 **16000 Hz 单声道 s16le**，帧大小不限（服务端缓冲后自行切 8k/480 采样 Opus 帧走 [42]）。
- 客户端可在首帧 JSON 里声明采样率（8000/16000），服务端负责下采样到设备要的 8k。

> 功能1 的设备播放优先用 **`POST /tts`（设备端 TTS）**，简单且省带宽；`/play` 主要给功能2 的对讲音频，以及将来"用外部 TTS 合成音频流式播放"的场景。

### 5.3 AI_runtime ↔ asr-service（OpenAI 兼容）

- **MVP 批式**：AI_runtime 用 VAD 切出一句话 → `POST /v1/audio/transcriptions`（multipart：`file`=wav/pcm, `model`, `language`）→ `{ "text": "..." }`。
- **可选升级流式**：`WS /v1/audio/stream`，边说边出 partial/final。MVP 不做。
- ASR 不知设备存在，只吃 AI_runtime 喂进来的音频。

### 5.4 AI_runtime ↔ llama-server（OpenAI）

- `POST /v1/chat/completions`，`{ "model", "messages", "stream": true }`，SSE 逐 token 返回。
- 流式的意义：攒到一个短句就可以先发 TTS，缩短"用户说完 → 设备开口"的空档。

### 5.5 office-client ↔ AI_runtime（功能2）

- WS PCM（16k 单声道 s16le）+ PTT 控制消息（JSON：`{"ptt":"talk|listen|idle"}`）。
- 连 AI_runtime，**不直连设备**（守住 R1/R2）。AI_runtime 据 PTT 方向把 office↔`/play`、`/mic`↔office 单向接通。

---

## 6. 模式状态机（AI_runtime 内，互斥核心）

```
                 POST /mode func1                POST /mode func2
        ┌──────────────────────────┐   ┌──────────────────────────────┐
        ▼                          │   │                              ▼
  ┌───────────┐   VAD一句话      ┌──┴───┴──┐   PTT方向翻转        ┌───────────┐
  │  功能1     │  ─────────────► │  IDLE   │ ◄─────────────────  │  功能2     │
  │ 近场AI回合 │  TTS估时到点回听  │(灯可用)  │                     │ 远场半双工  │
  └─────┬─────┘                  └────┬────┘                     └─────┬─────┘
        │ Listen→Think→Speak         │ POST /mode deter               │ Talk / Listen
        │  Speak时麦克风门控(丢帧)     ▼                                 │  同一时刻只放一个方向
        │  防止无AEC自录TTS      ┌──────────┐                          │
        └───────────────────────│  功能3    │──────────────────────────┘
                                 │  驱离     │
                                 └──────────┘
   进入新模式前必须 POST /mode（payload 拆旧流），再接管音频；任一时刻仅一个模式活跃。
```

**功能1 回合（半双工天然成立）**：
1. AI_runtime 从 `WS /mic` 读 16k PCM，跑 **VAD** 切句；
2. `POST /v1/audio/transcriptions` → 文本；
3. `POST /v1/chat/completions`(stream) → 回复文本；
4. **进入 Speak：门控麦克风**——AI_runtime 停止把 MIC 帧转发给 ASR（**丢帧，但不停 [40]**，避免每回合重启录音吃 ~98ms 首帧延迟）。设备无 AEC，不门控会把自己 TTS 录回去误触发；
5. `POST /tts{voice,text}` → 设备播放，拿到 `est_ms`；
6. 等 `est_ms + 余量`（无 done 事件）→ 取消门控 → 回 Listen。

**功能2 远场对讲（半双工 PTT，无 ASR/LLM）**：
- office-client 连 AI_runtime；说：办公MIC → `/play` → 机器狗喇叭；听：机器狗 `/mic` → 办公喇叭；
- 每次只放行一个方向 → 天然避开 AEC。全双工（需两端 AEC）留作后续，MVP 不做。

---

## 7. 关键常量与时序（实现必须遵守）

| 项 | 值 | 来源/说明 |
|---|---|---|
| MIC 采样/帧 | 16000 Hz，20ms=320 采样=640 B | 设备 [40] 录音格式 |
| PLAY 设备帧 | 8000 Hz，60ms=480 采样，Opus | 设备 [42] 喊话格式 |
| 录音首帧延迟 | ~98 ms（一次性），其后 ~10 ms/块 | 真机实测；故 Speak 门控用丢帧不停录 |
| 0x25 状态周期 | 500 ms（连上即推） | 灯光状态 |
| close 前 flush | **≥200 ms** | 防未读 rx 触发 RST 丢掉最后一帧 |
| TTS 估时 `est_ms` | `max(base, 字数×per_char) + tail`，默认 base=800, per_char=180, tail=500（ms，可配） | 无 TTS-done 事件，只能估 |
| LAN 往返 | ~0.7 ms | 实时语音足够 |

---

## 8. payload-service 内部模块划分（全部从0写）

```
/opt/xbrain_v6/services/payload/           # 部署位置（Orin 挂载到本地同路径；见 §16）
  app.py                 # 入口：装配 config + device_link + api，启动 uvicorn
  config.py              # 主机/端口/超时/估时参数等（见 §10）
  protocol/
    crc.py               # CRC-8/MAXIM（poly 0x31, reflect, init 0x00）——从0写
    lights_8529.py       # 8D|len|id|payload|crc 封装 + 0x25 解析（CRC 覆盖不对称！）
    audio_8519.py        # [id]payload bracket 帧封装/解析
  codec/
    opus_stream.py       # 用 opuslib(基础设施) 做流式分帧编解码；分帧/补零逻辑自写
    resample.py          # 8k↔16k 线性重采样——自写
  core/
    device_link.py       # 持有 8519/8529 两 socket；重连；close 前 flush；0x25 读取线程
    session.py           # 模式管理 + MIC/PLAY 单客户端路由 + 闸门（R2/R3 强制点）
    deter.py             # 驱离循环（灯+警笛+TTS）——重写，不 import patrol_deter
    siren.py             # numpy 警笛合成——重写，不 import siren_gen
  api/
    rest.py              # §5.1 路由
    ws.py                # §5.2 /mic /play
```

**从0实现清单（重点强调）**：`crc.py`、`lights_8529.py`、`audio_8519.py`、`opus_stream.py` 的分帧逻辑、`resample.py`、`device_link.py`、`session.py`、`deter.py`、`siren.py` —— 全部据 `docs/` 规格新写，**禁止** `import probe_*` / `import patrol_deter` / `import siren_gen`，禁止复制其代码。probe 仅供你阅读理解协议与真机对拍验证。

AI_runtime 侧（= xbrain_v6 编排逻辑，测试进程部署到 `/opt/xbrain_v6/tests/ai_runtime/`）：含 `state_machine.py`、`audio_router.py`、`vad.py`、`asr_client.py`、`llm_client.py`、`payload_client.py`，同样从0写。office-client 部署到 `/opt/xbrain_v6/tests/office_client/`。

---

## 9. 设备约束 → 代码硬性要求（坑点对照）

| 设备事实（已真机验证） | 代码必须满足 |
|---|---|
| 无片上 AEC | 功能1 Speak 期间门控 MIC；功能2 半双工 PTT |
| 无 TTS-done 事件 | `/tts` 返回 `est_ms`；AI_runtime 用估时+余量门控；余量可配 |
| close 前有未读 rx → RST 丢帧 | 关 8529/8519 前 `sleep ≥0.2s` flush（或先读空 rx）再 close |
| 0x25 状态帧 CRC **只覆盖 payload**（控制帧覆盖 len+id+payload） | 解析/构造分两套 CRC 范围，勿混用 |
| 关灯后亮度仍保留在状态字节，仅 b7 清零 | 判"关"只看 b7，别拿整字节判断 |
| [40]+[42] 单 socket 并发 OK | 功能2 设备层可行（全双工的 AEC 另算） |
| 本单元无 [99] 自报、[89]/[90] 无回 | `/status` 的 volume/temp 可能 null，需容错；别用 [89] 探测编码 |
| 8519 编码 = bracket（字面 `[40]`） | 帧封装用字面 ASCII 中括号，无长度/终止符 |

---

## 10. 配置项（config.py，集中管理）

| 键 | 默认 | 说明 |
|---|---|---|
| `DEVICE_HOST` | `192.168.144.38` | 设备 IP |
| `PORT_AUDIO` / `PORT_LIGHTS` | `8519` / `8529` | 设备端口 |
| `PORT_PAYLOAD` | `18080` | payload-service 本地监听 |
| `ASR_BASE_URL` | `http://127.0.0.1:<asr>` | ASR OpenAI 端点 |
| `LLM_BASE_URL` | `http://127.0.0.1:<llm>` | llama-server 端点 |
| `TTS_EST_BASE_MS` / `PER_CHAR_MS` / `TAIL_MS` | 800 / 180 / 500 | 估时参数 |
| `CLOSE_FLUSH_MS` | 200 | 关 socket 前 flush |
| `SOCKET_TIMEOUT_S` | 5.0 | 连接超时 |

> 部署路径（Orin，本地挂载同路径，见 §16）：payload-service = `/opt/xbrain_v6/services/payload/`。默认值放 `config.py`，环境相关值走环境变量覆盖。运行用 Orin 的 `python3.10`（本地 dev box 无 3.10，仅编辑）。

---

## 11. 依赖白名单（"从0实现"的边界）

**允许（基础设施 / 运行时 / 标准编解码绑定，非借用功能逻辑）**：
- Python 标准库：`asyncio` `socket` `struct` `wave` `threading` `time` `json` 等。
- Web 运行时：**FastAPI + uvicorn**（或纯 `asyncio` + `websockets`）——只提供 HTTP/WS 运行时，路由/业务逻辑自写。
- **opuslib**（libopus 的 Python 绑定）：Opus 是国际标准编解码，不可能从0重写编解码器；属基础设施。**分帧/补零/流式封装逻辑仍自写。**
- **numpy**：数值/重采样/警笛合成的基础库。
- httpx / requests（调 ASR、LLM 的 HTTP 客户端）。
- ASR、LLM 服务本身通过网络 API 调用，是独立进程，不算"引用代码"。
- **sherpa-onnx**（ASR zipformer transducer 推理引擎）与 **llama-server**（LLM 推理二进制）：与 opuslib 同性质的**预置推理基础设施**，用户已选定/部署（ASR 模型见 `services/asr/model-<导出名>/`；★ 2026-08-03 起默认 `model-paraformer-zh-2023-09`，族与目录的对应在 `services/asr/config.py`），不受 R0 约束；我们只从0写它们的**服务封装/调用胶水**（OpenAI 端点整形、热词加载、client 封装）。

**禁止**：
- `import` 本项目 `tools/` 下任何 probe/工具脚本。
- 从外部项目、博客、样例直接复制粘贴功能实现（协议、音频管线、状态机、驱离、警笛等）。

> 若后续想引入某个"帮你实现功能"的库（如现成的 VAD 库、现成的 AEC 库），先在此登记并说明它属于"基础设施"还是"功能实现"，与用户确认后再用。默认从0写。

---

## 12. 实现顺序与里程碑

1. **M1 payload-service 骨架**：`config` + `device_link`（连 8519/8529、0x25 读取、flush-close）+ `/status`。真机 `GET /status` 能读到灯光状态。
2. **M2 灯光/驱离**：`lights_8529` + `/lights` + `deter`/`siren` + `/deter`。真机复现驱离模式（对拍此前 patrol_deter 的效果，但代码全新）。
3. **M3 音频封装**：`audio_8519` + `opus_stream` + `resample` + `/tts` + `WS /mic`/`/play`。真机：`/tts` 出声；`/mic` 拉到干净 16k PCM；`/play` 送 PCM 能放。
4. **M4 功能1 近场AI**：AI_runtime 状态机 + VAD + ASR/LLM 客户端 + Speak 门控。跑通"说话→设备应答"闭环。
5. **M5 功能2 远场对讲**：office-client + PTT 半双工路由。
6. （远期）功能2 全双工 + 双端 AEC。

**先做 M1→M4（功能1），再做 M5（功能2）。** 每个里程碑都要在真机上验证，并把新发现的坑回填 `GZH-2_测试调试与xbrain_v6开发笔记.md`。

---

## 13. 验收标准（每模块自测点）

- **CRC**：对协议文档 5 条示例帧逐一比对通过；0x25 状态帧用"仅 payload"口径校验通过。
- **8519 帧**：`/tts` 让设备发声；`/mic` 录一段人声，落 wav 听感干净、16k、无爆音。
- **模式互斥**：func1 activ 时开 `/mode func2`，旧 `/mic` 会话被关闭、无双客户端并存（R2）。
- **门控**：功能1 Speak 期间 ASR 不应收到设备自身 TTS（无自触发回合）。
- **驱离**：灯全灭收尾（`/status` 里 searchlight b7=0），红蓝复位，警笛/TTS 停；close 无丢帧。
- **收尾**：任何模式退出后 `/status` 回到 idle 且设备无残留亮灯/发声。

---

## 14. 与既有资料的关系

- 协议细节以 `GZH-2_控制协议开发文档.md` 为准；真机验证事实与坑点以 `GZH-2_测试调试与xbrain_v6开发笔记.md` 为准。
- 本文是**实现规范**：架构、接口 schema、模块划分、里程碑、验收。三份文档不重复，交叉引用。
- 本文如与真机行为冲突，以真机为准，并同步修订本文与开发笔记。

---

## 15. 编码规范（house style，与 xbrain_v6 对齐）

本测试系统为 **xbrain_v6** 的系统级测试而建，代码与机器人系统同栈共处，**一律遵循 house 编码规范**。规范源文件目前是 `/opt/xbrain_v5/CLAUDE.md`（2026-05-25 版；v6 尚无自己的 CLAUDE.md，直接沿用 v5 房规）。该文件不在本地自动加载目录——**真正动手写代码前先把它读一遍**。以下是适用于本项目（payload-service / AI_runtime / office-client）的可移植条目：

**语言 / 字符**
- 面向用户的**对话回复全中文**；**日志 / print / 异常 message 全英文**；代码注释单文件统一语言（全中或全英，不混用）。
- **代码内严禁 emoji 与中文标点**，只用 ASCII 英文标点。

**命名 / 结构**
- 标识符 snake_case；类名 PascalCase；常量 UPPER_CASE；（C++ 字段尾下划线）。
- **每个源文件头部 5 字段注释**：`Copyright (c) 2025 Hachist Robotics` / `Author: wanglei@hachist.com` / `上海哈船智能船舶技术有限公司` / `File:` / `Brief:` + `Description:`。
- 注释解释 **why 非 what**；复杂算法配伪代码/公式/状态机；修 bug 的代码注明原 bug + **本规范节号**（如 `见 测试系统开发方案 §6 模式状态机`）。

**Python**
- 强制 type hints；DTO 用 `pydantic.BaseModel` 或 `dataclass(frozen=True)`；自定义异常（禁裸 `Exception` / 裸 `except:`）。
- **严禁 `datetime.now()` 无参**（走时间工具函数）。
- **业务模块严禁 `import requests` 直连 AI 服务**——统一走 client 封装（本项目已按 §8 设计 `asr_client.py` / `llm_client.py` / `payload_client.py`）。
- 关停路径的 `await` 必须带 timeout 包装。

**Shell / Git / 纪律**
- Shell：`#!/usr/bin/env bash` + `set -euo pipefail`；派生 `SCRIPT_DIR` 不硬编码 `cd`；`rm -rf` 必须显式列路径。
- Commit：`<scope>: <=50字简述` + 空行 + 详述 + 空行 + `Refs: <本文§>`；一个 commit 只做一件事。
- **V1/V2 纪律**：严禁为将来扩展预留接口/配置开关（与 R0 的极简取向一致，不做投机式设计）。
- 测试：pytest + pytest-asyncio；状态机（session/模式机）要全转换覆盖；每个 bug 修复配测试。

**边界说明**：xbrain 的**机器人核心专属**规则——zenoh SubscriberRegistry 强引用、`import rclpy`/`sqlite3`/`requests` 目录级禁令、Fast Path R1–R4、motion publish 白名单、LITE_DESIGN/PERCEPTION_DESIGN 反链、blackbox/优雅关停预算——**只约束机器人核心代码**（`/opt/xbrain_v6/xbrain/`、`ros2_ws/`、`perception/` 等）。我们的测试系统目录（`services/payload/`、`tests/ai_runtime/`、`tests/office_client/`）遵循上面的**可移植房规**，但不套这些核心专属规则（它们本就不用 zenoh/rclpy/Fast Path）。

**已确认（2026-07-25）**：① 所有源文件照挂 **Hachist 公司头**（`Copyright (c) 2025 Hachist Robotics` / `Author: wanglei@hachist.com` / `上海哈船智能船舶技术有限公司` / File / Brief / Description 五字段）。② **注释覆盖率 ≥ 70%**（写真正解释 why 的注释自然达标，不写填充注释凑数）。

---

## 16. 部署布局与技术栈（2026-07-25 定）

**挂载**：Orin 的 `/opt/xbrain_v6` 已通过 sshfs 挂载到本地同路径（`jack@192.168.0.112:/opt/xbrain_v6`）。**在本地编辑 = 直接改 Orin 上的文件**；但**运行一律在 Orin**（本地 dev box 无 python3.10）。改完在 Orin 起服务/跑测试。

**目录布局（全部在 `/opt/xbrain_v6` 下）**：
| 组件 | 部署路径 | 语言/运行 |
|---|---|---|
| payload-service | `services/payload/` | Python（从0写） |
| asr-service | `services/asr/`（模型已在 `model/`） | sherpa-onnx + Orin python3.10 |
| llama-server | `services/llm/`（`bin/ lib/ prompt/ model/ llm_server.sh`） | 预置二进制 |
| ai_runtime 测试进程 | `tests/ai_runtime/` | Python（从0写） |
| office-client 测试进程 | `tests/office_client/` | Python（从0写） |

> 注：用户口述 `test/`，但实际已存在的是 `tests/`（复数，且已含空的 `ai_runtime/`、`office_client/`），故统一用 `tests/`。

**llama-server（`services/llm/`）**——**不部署 llama.cpp 源码**，只放产物：
- `bin/`：`llama-server` 二进制。 `lib/`：其依赖 `.so`。 `model/`：GGUF 模型。 `prompt/`：暂空（system prompt 后续放）。
- `llm_server.sh`：启动脚本（含 house 头 + `set -euo pipefail` + `SCRIPT_DIR` 派生；OpenAI API 形式；端口/模型/线程走脚本顶部变量或环境变量）。

**asr-service（`services/asr/`）**——**zipformer + 热词**：
- 模型：icefall `multi-zh-hans` zipformer transducer（encoder/decoder/joiner ONNX，fp32+int8；`tokens.txt` 2000 BPE；`bpe.model` 支持热词 contextual biasing）。已在 `model/`。
- 引擎：**sherpa-onnx**（推理内核 C++，预置基础设施，不受 R0）。
- 语言取舍：用户「尽量 C++、不行就 Python」。**推荐 Python 服务封装 + sherpa-onnx（推理仍是底层 C++，用 Orin 默认 python3.10）**——起活快、热词/OpenAI 端点整形好写；纯 C++ 服务留待性能确有需要时再上。**待用户拍板**。
- 对外：`POST /v1/audio/transcriptions`（OpenAI 兼容，MVP 批式）；热词从文件加载。

**其余**（payload-service / ai_runtime / office-client）全用 Python 从0实现。
