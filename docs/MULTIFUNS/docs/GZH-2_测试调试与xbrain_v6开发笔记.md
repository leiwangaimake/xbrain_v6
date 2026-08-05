# GZH-2 测试调试与 xbrain_v6 集成开发笔记

> 目的：把我们对 GZH-2 三合一负载做的**真机测试、协议验证、踩过的坑、已构建的工具**汇总成一份开发参考。后续做 xbrain_v6 集成时直接读本文即可上手，不必再逐项复现。
>
> 记录日期：2026-07-25（真机 device 192.168.144.38，经 Orin 验证）。
> 相关文件：协议原始文档见 `GZH-2_控制协议开发文档.md`、用户手册 PDF/docx；工具源码在 `/opt/speaker/tools/`。

---

## 1. 硬件与网络拓扑

- **设备**：GZH-2「机器狗版多合一（二代）」，本单元为**三合一**，四个功能里含 **喊话（喇叭 110dBA@1m/15W）、照明（补光灯/探照灯）、红蓝警示灯、收音（MIC）**。IP66 / 24V。地面站为 SIYI MK15/H16 + 智慧负载 Android APP。
- **计算单元**：NVIDIA Jetson Orin（aarch64），hostname `xbrain`。xbrain_v6 与所有对设备的代码都跑在这里。
- **网络链路**：
  - dev box `Hachist` → `ssh jack@xbrain.local`（192.168.0.112，办公室 WiFi）。
  - Orin 网卡 `enP8p1s0` = `192.168.144.100/24` → 负载 `192.168.144.38`。
  - 端口：**8519 = 音频**，**8529 = 灯光**。LAN ping ≈ 0.7ms。
- **依赖（Orin 已装）**：`opuslib`(pip) + `libopus0`(apt)；`numpy` 1.21.5。dev box numpy 2.3.3。
- **凭据**：SSH 当前免密；sudo 密码按会话临时提供，**不入库**。

---

## 2. 8529 灯光协议（真机已验证）

**帧格式**：`8D | len | MSG_ID | payload[len] | CRC8`

- **CRC = CRC-8/MAXIM**（poly 0x31，reflected 0x8C，init 0x00）。
- ⚠️ **CRC 覆盖范围不对称**（重要坑点）：
  - **控制帧**：CRC 覆盖 `len + MSG_ID + payload`。
  - **状态帧 0x25**：CRC **只覆盖 payload**。
- **控制帧一律无回复（no-reply）**。连上后设备**每 500ms** 主动推一条 `0x25` 状态帧。

**MSG_ID 表**：

| MSG_ID | 名称 | payload | 说明 |
|---|---|---|---|
| `0x01` | `MSG_LIGHT` | `0/1` | 补光灯/探照灯 开关 |
| `0x02` | `MSG_BRIGHT` | `0-30` | 探照灯亮度（`BRIGHT_MAX=30`），**只作用探照灯** |
| `0x03` | `MSG_STROBE` | `0/1` | 探照灯爆闪开关，**无频率参数**（固件固定频率） |
| `0x04` | `MSG_HOOK` | `sel(0/1/2)+on(0/1)` | 抛投钩；在规范里，本三合一单元**未测/可能无** |
| `0x07` | `MSG_REDBLUE` | `0x00-0x10` | 红蓝警示灯，0=灭，1-16=内置图案 |

**0x25 状态帧布局**（真机实测，28 字节 = `8D 18 25 <24B payload> CRC`）：

- `payload[3]` = **探照灯字节**：`b7`=开、`b6`=爆闪、`b0-5`=亮度(0-30)。
  - 由 `light_byte(on,strobe,bright)` 反推：`0x9E`=开+亮度30、`0xDE`=开+爆闪+亮度30、`0x1E`=**灭但记忆亮度30**。
- `payload[4]` = **红蓝 mode**（0=灭，1-16）。
- 常量：`payload[1]=0x05`，`payload[8]=payload[16]=0x7F`（非灯光位）。
- ⚠️ **亮度记忆**：灯灭时 `payload[3]` 只清 `b7`，低 6 位亮度保留 → 看到 `0x1E` 是「灭+记忆30」，不是「亮」。

**灯光分工（用户明确要求，务必遵守）**：

- **补光灯/探照灯**（`MSG_LIGHT` + `MSG_BRIGHT`）= 夜间照明用途，**常亮**，不要拿它做爆闪。
- **警灯「双闪」效果作用在红蓝警示灯**（`MSG_REDBLUE`），警车式「两快闪+短停」。
- **红蓝没有独立亮度寄存器**（`MSG_BRIGHT` 只管探照灯）→ 红蓝「最亮化」= 点亮即全亮，双闪时每个脉冲让它全亮即为最大。

---

## 3. 8519 音频协议（真机已验证）

**帧格式**：`<命令前缀><payload>`，命令前缀编码 = **bracket = 字面 ASCII**，如 `b"[42]"`。

- 判定依据：`[40]` 录音回了 **392/392 干净 16k Opus 帧**（ground truth）。
- ⚠️ 本单元 `[89]`/`[90]`（取 SN/UID）**无任何回复**，也**没有 `[99]` 自动上报**（三合一特性）→ **不要用 `[89]` 探测编码**，录音路径才是准。

**命令表（probe_8519 里的 `CMD_*`）**：

| CMD | 名称 | 方向/用途 | payload |
|---|---|---|---|
| `14` | `CMD_VOLUME` | 设音量 | 1 字节 `0-100`（raw，如 `0x64`=100） |
| `42` | `CMD_HAIL_PC` | PC 端喊话下行 | 一帧 Opus（8k mono，60ms=480 样本） |
| `10` | `CMD_HAIL_MOBILE` | 手机端喊话 | — |
| `11` | `CMD_HAIL_STOP_MOBILE` | 停止喊话 | 空 |
| `15` / `16` / `17` | TTS / TTS循环 / TTS停止 | — | `[15]`=文本UTF-8 |
| `31` / `32` | `CMD_TTS_V2` / 循环 | TTS（带性别） | `voice字节(0男/1女) + UTF-8文本` |
| `40` / `41` | 录音开始 / 停止 | MIC 上行 | Opus（16k mono，20ms=320 样本） |
| `89` / `90` | 取 SN / UID | 本单元静默 | — |
| `99` | 自动上报 | 本单元**无** | — |

**音频参数常量**：`HAIL_FS=8000, HAIL_FRAME=480`(60ms)；`REC_FS=16000, REC_FRAME=320`(20ms)。

**Step-1 真机结论（关键，直接影响 xbrain 设计）**：

- ✅ **并发 OK**：`[40]` 录音 + `[42]` 喊话可在**同一 TCP socket 同时跑**，上行持续（A≈B≈C 字节/s）→ **设备层支持全双工**，剩下的只有 AEC，不是固件阻塞。
- ✅ **延迟好**：录音首帧 ~98ms（一次性启动），之后 ~10ms/块，抖动 min5/max15.5ms → 实时语音够用。
- ✅ **收音 + 喇叭都在**：录到音频，且 `[42]`/`[31]` 都被接受播放；样本存 Orin `/opt/speaker/samples/mic_38.wav`（另有 `mic_38_take2.wav`）。
- ⚠️ **无「TTS 播完」事件** → 只能**按时长估算**播放结束（见 `--tts-dur`）。

---

## 4. 已构建的工具（dev box 源码 `/opt/speaker/tools/`，Orin 部署也在 `/opt/speaker/tools/`，mic 采样在 `/opt/speaker/samples/`）

| 文件 | 作用 | 关键子命令 / 接口 |
|---|---|---|
| `probe_8529.py` | 灯光 probe（纯 stdlib） | `selftest / light / bright / strobe / redblue / flash / off / status / scene / raw / mock` |
| `probe_8519.py` | 音频 probe（stdlib + 可选 opuslib） | `selftest / encoding / info / record / play / tts / duplex / raw / mock` |
| `siren_gen.py` | 警笛 WAV 生成（numpy） | `build_siren(...)` / `write_wav(...)`；CLI `--pattern --level --accent-hz` |
| `patrol_deter.py` | **驱离模式**控制器（双线程） | `lights_loop` + `audio_loop`；`--dry-run` |

**可直接复用给 xbrain_v6 的函数**（在 `probe_8519.py` 里，已验证）：

- `frame(cmd, payload, enc="bracket")` / `cmd_prefix(cmd, enc)` — 组帧。
- `encode_opus_stream(pcm, fs, frame_size)` → Opus 包列表；`decode_opus_packets(packets, fs)` → PCM。
- `read_wav_mono(path, target_fs)`（含线性重采样、多声道混单）；`write_wav(path, pcm, fs)`。
- `connect / capture_for / split_by_marker` — 网络与上行拆帧。
- 两个 probe 都有 `if __name__=="__main__"` 守卫，**可被 import 复用**（`patrol_deter.py` 即 `import probe_8519 as A / probe_8529 as L`）。

**离线开发**：两个 probe 都自带 `mock` 子命令（假设备），指向 `127.0.0.1` 即可无硬件联调。

---

## 5. 功能3「驱离模式」（已完成 + 真机验证）

**设计**：红蓝警车式双闪（8529）**并发** Wail+Yelp 警笛 + 男声 TTS 循环（8519 `[42]`/`[31]`）。两条链路走**不同端口/不同 socket/不同线程**，互不冲突。用途：模拟驱赶闯入巡逻营区的入侵者。**用户称其为 v6 重要功能之一。**

**音频状态机（`audio_loop`，无限循环）**：
1. Phase A：把一整段 **Wail+Yelp 警笛**（一个「快慢周期」）经 `[42]` Opus 8k 按 60ms 节流streaming → `[11]` 停止 → 停 0.3s 让尾音落。
2. Phase B：**男声 `[31]`（voice=0）** 播 `"你已进入管制区域，请立即离开"` **×3，每遍间隔 1s**。
3. 回到 Phase A，无限循环。

**灯光（`lights_loop`）**：开机一次性 `MSG_STROBE[0] + MSG_BRIGHT[30] + MSG_LIGHT[1]`（补光灯常亮最亮），随后持续红蓝双闪：`rb_on; wait(pulse_on); rb_off; wait(gap)` ×2，再 `wait(pause)`。

**时序 / 声光同拍**：
- flash group = `pulses*(pulse_on+gap)+pause` = `2*(0.09+0.07)+0.40` = **0.72s ≈ 1.39 组/s**。
- 警笛 `--accent-hz` 应 = `1/group` = **1.39**，让警笛的幅度重音踩在闪灯节拍上（`patrol_deter` 自动生成时已锁定）。
- 一个完整 loop ≈ `6.0(警笛) + 0.3 + 3*4.0(TTS) + 2*1.0(间隔)` ≈ **20.3s**。

**用户的 4 项强化（2026-07-25，已实现）**：
1. 红蓝爆闪最亮 —— 每个脉冲让红蓝**全亮**（协议无红蓝调光位）。
2. 补光灯最亮且常亮 —— 探照灯 `bright 30` 稳定常亮，不闪。
3. 警笛声小一点（办公室测试偏大）—— `siren_gen --level 0.45`（幅度砍半，peak 0.72→0.36）。**只调警笛 WAV，不动设备音量 `[14]`**，避免把 TTS 也压小。
4. 警笛一个快慢周期后接男声 TTS×3(间隔1s)→再回警笛，无限循环。

### 5.1 调试记录（重要坑点 + 修复）—— 开发时务必注意

- **坑① 关 socket 时 RST 丢掉最后的关灯帧** ⭐
  - 现象：42s 测试结束后**探照灯卡在常亮**（`payload[3]=0x9E`），红蓝已灭。
  - 根因：8529 每 500ms 推 `0x25`，我们从不读它；一段时间后接收缓冲积压大量未读数据，此时 `close()` 会让 Linux 发 **RST**（而非 FIN），把刚 `sendall` 出去、尚未被对端确认的 `MSG_LIGHT[0]` 关灯帧**丢弃**。
  - 修复：`lights_loop` 的 `finally` 里，发完关灯帧后 `time.sleep(0.2)` 再 `close()`，让帧在 LAN 上 flush+ACK。
  - **通用教训**：对 8529（或任何持续推流的 socket）**收尾前要留时间 flush，或先排空接收缓冲**，否则最后一帧可能丢。**收尾后务必用 `probe_8529.py status` 复核 `payload[3]` 的 b7=0。**

- **坑② `--wav` 默认路径与部署位置不符导致音频线程崩溃**
  - 现象：`patrol_deter.py --host X`（不带 `--wav`）时 `FileNotFoundError: /tmp/siren.wav`，音频线程崩，**灯还在跑**。
  - 修复：默认 `--wav` 改为**脚本同目录的 `siren.wav`**（`os.path.dirname(__file__)`；迁移到 /opt/speaker 后由 ~/siren.wav 调整为脚本相对路径，避免重新散落到 home）；且**文件缺失时自动生成**（`ensure_siren`，numpy-only，不碰设备，accent 自动锁 1/group）。现在该工具**自包含**，`--host X` 一条命令即可跑。
  - 附带新增旋钮：`--siren-level`（办公室调音量，只影响警笛）、`--regen-siren`（改 level 后强制重生成）。

### 5.2 复现 / 部署命令

```bash
# 本地语法检查
python3 -c "import ast; ast.parse(open('siren_gen.py').read())"
python3 -c "import ast; ast.parse(open('patrol_deter.py').read())"

# 部署到 Orin
scp /opt/speaker/tools/{siren_gen,patrol_deter}.py jack@xbrain.local:/opt/speaker/tools/

# 干跑（不发设备，打印时序 + 需要时自动生成警笛）
ssh jack@xbrain.local 'python3 /opt/speaker/tools/patrol_deter.py --dry-run'

# 真机：有限时长测试（一整个 loop ≈ 20.3s）
ssh jack@xbrain.local 'python3 /opt/speaker/tools/patrol_deter.py --host 192.168.144.38 --seconds 24'

# 真机：正式无限循环（Ctrl-C 停）
ssh jack@xbrain.local 'python3 /opt/speaker/tools/patrol_deter.py --host 192.168.144.38'

# 收尾复核：payload[3] b7 应为 0（灯灭）
ssh jack@xbrain.local 'python3 /opt/speaker/tools/probe_8529.py status --host 192.168.144.38'
```

**待用户耳测微调**：`--tts-dur`（默认 4.0s/句，因无 TTS-done 事件是估值）；若 3 遍重叠或留白，调它。

---

## 6. xbrain_v6 集成路线图

目标：xbrain_v6 ↔ GZH-2 组成 **2 功能系统**。**已定顺序：先做功能1，再做功能2。**

### 功能1 · 近场 AI 语音控制（先做）
人在机器旁 ↔ 机器人，ASR→LLM→TTS 闭环。
- **输入**：机器人 `[40]` 收音（Opus 16k）→ 解码 → VAD → ASR。
- **输出**：LLM → TTS 经 `[31]`/`[15]`，或流式 `[42]`（8k）。
- **半双工天然成立**（人机轮流说话）。
- **xbrain 还缺**：Opus 编解码（可复用 probe_8519 的函数）+ VAD + 8519 TCP 客户端 + 半双工调度。**因无 TTS-done 事件，要按时长估播放结束，并在 TTS 播放期间 gate 住录音**（否则自己的 TTS 会被 `[40]` 录回去）。
- **阻塞点**：需用户说明 **xbrain_v6 如何暴露 ASR / LLM / TTS 接口**（之前问过未答）。

### 功能2 · 远场对讲（后做，逐字转发，无 ASR/LLM）
- office-PC MIC → `[42]`(Opus 8k) → 机器人喇叭；机器人 `[40]`(Opus 16k) → office 喇叭。
- **半双工 PTT = 简单，推荐做 MVP。**
- **全双工 = 两端都要 AEC**：office 本地 AEC 是标准/容易；**机器狗侧回声经网络从 `[40]` 上行返回**（设备**无板载 AEC**），xbrain 必须用发出的 `[42]` 作参考消回声——但**可变 RF/网络延迟 + 8k↔16k 重采样 + 编解码非线性**使其确实较难。用户已自认「全双工要搞定回音消除」。
- 好消息：**设备层并发已验证可行**（见 §3），AEC 是剩余工作而非固件阻塞。

---

## 7. 关键坑点速查（cheat sheet）

- 8529 **状态帧 0x25 的 CRC 只覆盖 payload**；控制帧 CRC 覆盖 len+id+payload。
- 8529 探照灯**亮度记忆**：灭灯只清 b7，`0x1E`=灭+记忆30。
- 8529 关 socket 前 **sleep/flush**，否则 RST 丢最后一帧（探照灯会卡亮）。收尾用 `status` 复核。
- **红蓝无调光位**；`MSG_BRIGHT`/`MSG_STROBE` 只作用探照灯。警灯双闪走 `MSG_REDBLUE` mode。
- 8519 编码 = **bracket**；本单元 `[89]/[90]/[99]` 静默，别用 `[89]` 探编码，用 `[40]` 录音验证。
- `[31]` TTS：**首字节 = 性别（0男/1女）**，其后 UTF-8 文本。`[14]` 音量 = 单字节 0-100。
- **无 TTS-done 事件** → 计时估播放；TTS 期间要 gate 录音。
- 设备层 **[40]+[42] 并发可行**；全双工的难点是 **AEC（机器狗侧网络回声）**。

---

## 8. 环境与常用命令速查

```bash
# 登录 Orin
ssh jack@xbrain.local

# 自检（无需硬件）
python3 /opt/speaker/tools/probe_8529.py selftest          # 灯光 CRC 对齐 spec
python3 /opt/speaker/tools/probe_8519.py selftest          # 编码 + opus 往返

# 灯光单点
python3 /opt/speaker/tools/probe_8529.py light on   --host 192.168.144.38
python3 /opt/speaker/tools/probe_8529.py bright --level 30 --host 192.168.144.38
python3 /opt/speaker/tools/probe_8529.py redblue --mode 1  --host 192.168.144.38
python3 /opt/speaker/tools/probe_8529.py status     --host 192.168.144.38   # 看 0x25
python3 /opt/speaker/tools/probe_8529.py off         --host 192.168.144.38

# 音频单点
python3 /opt/speaker/tools/probe_8519.py record --seconds 5 --wav /tmp/mic.wav --host 192.168.144.38
python3 /opt/speaker/tools/probe_8519.py tts --v2 --voice 0 --text "测试" --host 192.168.144.38
python3 /opt/speaker/tools/probe_8519.py play --wav /tmp/x.wav --host 192.168.144.38
python3 /opt/speaker/tools/probe_8519.py duplex --seconds 8 --host 192.168.144.38   # 并发验证
```
