# XBRAIN_V6 · PTZ 布控球实测报告

| 项 | 内容 |
|---|---|
| 文档类型 | 实测报告（🚫 **不是** `00`~`21` 正式编号册，不作为设计契约引用） |
| 测试对象 | 快速布控球（内置路由器 ＋ 可见光相机 ＋ 热成像相机） |
| 测试平台 | ORIN（`xbrain`，JetPack 6.2 / R36.4.3 / Ubuntu 22.04 arm64） |
| 测试日期 | **2026-07-29** |
| 权威来源 | `docs/PTZ/布控球通用说明书.pdf`（30 页）· `docs/PTZ/可见光用户手册.pdf`（240 页）· `docs/PTZ/热成像使用说明书.pdf`（204 页）· 设备 HTTP API 实测 |
| 数值口径 | ★★★ 本文所有实测值均为 **2026-07-29** 当日测得。按 CLAUDE.md §3.7 精神，每节标注**可复现脚本路径**；数值与脚本结论不一致时**以重跑脚本为准**，🚫 不以本文数字为准 |

---

## 0. 文档说明

### 0.1 本文回答什么

本报告覆盖两条主线：**拉流**（能取到什么画质、实际帧率多少）与**控制**（云台、聚焦、变焦怎么控）。
每条结论都标注了判据与验证方式；**推断**与**实测**分别标注，🚫 不混。

### 0.2 阅读约定

- ✅ **实测确认** —— 有客观判据、可复现
- ⚠️ **推断** —— 有间接证据但未直接验证，后续需补测
- 🚫 **已证伪 / 死路** —— 试过且不通，避免重复投入

---

## 1. 设备构成 —— ★★★ 一壳三设备

★★★ **布控球不是一个网络设备，是三个。** 这是理解后续全部结论的前提。

| 部件 | IP（迁移后） | 账号 | 厂商系 | Web 配置路径 |
|---|---|---|---|---|
| 内置路由器 | `192.168.66.1` | admin / admin | — | `网络 > 本地网络` |
| 可见光相机 | `192.168.66.13` | admin / `Admin123.` | 宇视系（LAPI） | `配置 > 网络 > 基础配置` |
| 热成像相机 | `192.168.66.108` | admin / `Admin123.` | 大华系（CGI，白标） | `设置 > 网络设置 > TCP/IP` |
| AP 热点 | SSID `PTZ-CAMERA-5.8G-XXXXXX` | `12345678` | — | 路由器 `热点配置` |

★★ **两台相机来自不同 OEM**，API 完全不同（宇视 LAPI vs 大华 CGI），🚫 不要假设对称。

### 1.1 ⚠️ 手册自相矛盾（会坑人）

- `布控球通用说明书.pdf` 第 5 页「出厂默认参数」写 **摄像机 IP = `192.168.1.68`**
- `可见光用户手册.pdf` §1.2「登录 Web 界面」写 **默认静态 IP = `192.168.1.13`**，且**出厂默认开启 DHCP**

✅ **以 `.13` 为准** —— 实测该地址响应，`.68` 无响应。
热成像默认 `192.168.1.108`（`热成像使用说明书.pdf` §3.2「修改 IP 地址」）。

### 1.2 型号与固件（实测自设备 API）

| 部件 | 型号 | 固件 |
|---|---|---|
| 可见光 | `HCM641NX33L` | `CIPC-B2202.3.8.L68.260128`（LAPI V1.55，MAC `e4f14ca78366`） |
| 热成像 | `deviceType=TPC`，`Vendor=General`（白标，无具体型号） | `2.634.0000000.11.R build 2026-01-09`（SN `CE071EFPCU86840`） |

★ 热成像是**白标机**，`magicBox.cgi?action=getProductDefinition` 与 `devVideoInput.cgi?action=getCaps` 均返回 `Bad Request`，从型号查不出原生像元数。

---

## 2. 网络配置 —— IP 网段迁移

### 2.1 冲突根因

★★★ 冲突不只是网段重叠，是**网关 IP 完全同一个地址**：

```
公司 WiFi:        192.168.1.0/24，网关 192.168.1.1（MAC 54:9b:49:96:c1:88）
布控球内置路由器:  192.168.1.0/24，LAN 口 192.168.1.1   <- 撞车
```

ORIN 的默认路由指向 `192.168.1.1`。一旦再接上布控球，该地址就有两个含义，路由表必然歧义。

### 2.2 ✅ 迁移方案（已实施）

整体搬到 `192.168.66.0/24`（避开已占用的 `192.168.1.x` 与 ORIN 以太网侧的 `192.168.144.x`）：

| 部件 | 新 IP | 网关 |
|---|---|---|
| 路由器 LAN | `192.168.66.1` + DHCP 池 `.2`~`.250` | — |
| 可见光 | `192.168.66.13` | `192.168.66.1` |
| 热成像 | `192.168.66.108` | `192.168.66.1` |
| ORIN（`enP7p1s0`） | `192.168.66.231` | — |

### 2.3 ★★★ 操作顺序 —— 先改相机，路由器放最后

**顺序反了会把自己锁在外面。** 你是*透过*路由器访问相机的：若先把路由器 LAN 改到新网段，两台静态相机会留在旧网段成为孤岛，得在电脑上临时加一个旧网段辅助 IP 才能捞回来（同一个二层域，技术上可行但很折腾）。

路由器 `本地网络` 页必须**同一次保存**里改三个字段：`IP地址` / `起始IP地址` / `结束IP地址`
（`布控球通用说明书.pdf` §2.2 逐字提示「注意同步修改 DHCP 起始及结束地址」）。

### 2.4 锁死后的恢复路径

- **路由器**：开机状态下长按后面板**复位按键 10 秒**恢复默认（说明书第 11 页）
- **热成像**：大华 **ConfigTool** 可跨网段发现并改 IP（安装包需向供应商索取）
- **可见光**：同一二层域内，电脑加一个旧网段辅助 IP 即可访问

---

## 3. 拉流（RTSP）

> 复现脚本：`/tmp/probe2.py`（取流枚举 ＋ 帧率实测）· `/tmp/spectral.py`（原生分辨率判定）
> ★ 位于 ORIN `/tmp`，重启会清。需长期保留应移到 `/opt/xbrain_v6/scripts/` 下。

### 3.1 ★★★ 可见光是 4MP 机，**不是 4K**

宇视 LAPI `Channels/1/Media/Video/Capabilities` 是权威数据：

| 码流 | 可选分辨率 | 上限 |
|---|---|---|
| ID=0（主） | `2560x1440` / `2304x1296` / `1920x1080` | **2560x1440（3.7MP）** |
| ID=1（辅1） | `1920x1080` / `1280x720` / `720x576` | 1920x1080 |
| ID=2（辅2） | `720x576` / `640x360` / `704x288` / `352x288` | 720x576 |

✅ **能力集里根本没有 `3840x2160` 档位**，且主码流当前已配到上限。
★ 码率可调范围 `128`~`16384` kbps（三个码流一致）。

**当前编码配置**（实测）：

| 码流 | 分辨率 | 帧率 | 码率 | 编码 | I 帧间隔 |
|---|---|---|---|---|---|
| 0 | 2560x1440 | 25 | 6144 kbps | H.264 | 50 |
| 1 | 720x576 | 25 | 1024 kbps | H.264 | 50 |
| 2 | 352x288 | 25 | 128 kbps | H.264 | 50 |

### 3.2 ✅ 可见光帧率实测 —— 25fps 达标

主码流 URL：

```
rtsp://admin:Admin123.@192.168.66.13:554/media/video1
```

| 模式 | 帧数 | 时间跨度 | 帧率 | 码率 |
|---|---|---|---|---|
| copy（只收包不解码） | 500 | 19.98 s | **25.02 fps** | 5.96 Mbps |
| decode（软解） | 499 | 20.00 s | 24.95 fps | — |

★★★ **测帧率必须区分这两种模式**，否则容易得出假结论：`copy` 模式测的是**相机真实输出速率**（回答「能否到 25fps」）；`decode` 模式会把 CPU 解码能力混进来。本机两者一致，说明 Orin 软解 1440p 毫无压力 —— 但若换成真 4K，只用默认的 `ffmpeg -f null -` 去测，测出来的会是 CPU 瓶颈而不是相机能力。

### 3.3 ⚠️ 可见光 RTSP 的三个坑

**坑一：路径别名不区分码流。**
以下路径**全部返回同一路 2560x1440 主码流**：

```
/unicast/c1/s0/live    /unicast/c1/s1/live    /unicast/c1/s2/live
/video1                /cam/realmonitor?channel=1&subtype=0
```

✅ **只有 `/media/videoN` 真正区分码流**（`video1`=主 2560x1440，`video2`=辅 720x576）。
🚫 别用 `/unicast/c1/s1/live` 去取辅码流，取不到。

★ `可见光用户手册.pdf` §5.2.2.4「RTSP」只写了开关，**全册未给 URL 字符串**，只能枚举得出。

**坑二：★★★ 流里带一路 `pcm_mulaw` 音频，mp4 装不下。**

```
[mp4] Could not find tag for codec pcm_mulaw in stream #1
Could not write header for output file #0 (incorrect codec parameters ?)
```

⇒ `ffmpeg -c copy` 输出 **mp4 会直接写头失败，产出 0 字节文件**。
✅ 解法：用 `mpegts`，或加 `-map 0:v` 只取视频。xbrain 侧录像要注意。

**坑三：像素格式是 `yuvj420p`（full-range YUV）。**
消费端若按 limited range 处理，色阶会错。

### 3.4 ✅ 热成像上限 1280x1024 @ 25fps，已配到顶

大华 `encode.cgi?action=getConfigCaps` 是权威数据：

```
主码流  ResolutionTypes = SXGA(1280x1024), 720P, 640x512    FPSMax = 25
辅码流  ResolutionTypes = 640x512, 320x256                  FPSMax = 25
编码    H.264 / H.265（辅码流另支持 MJPG）
码率    768~4096 kbps（建议 1024）
```

**主码流 URL**（格式见 `热成像使用说明书.pdf` §6.2.1 区域，逐字给出 `/cam/realmonitor?channel=N&subtype=M`）：

```
rtsp://admin:Admin123.@192.168.66.108:554/cam/realmonitor?channel=1&subtype=0
```

| 码流 | 分辨率 | 编码 | 实测帧率 | 码率 | 说明 |
|---|---|---|---|---|---|
| `channel=1&subtype=0`（主） | 1280x1024 | HEVC | **25.1 fps** | 2.21 Mbps | 已达上限 |
| `channel=1&subtype=1`（辅） | 640x512 | HEVC | — | 192 kbps | 配置为 **15** fps |

✅ **只有 `channel=1`**，`channel=2` 返回错误。
★ `subtype` 语义：0=主码流，1=辅码流1，2=辅码流2。

### 3.5 ✅ 1280x1024 是真实分辨率，不是 640x512 放大

**初始怀疑**：`SXGA(1280x1024)` 正好是 `640x512` 的 2 倍，且 640x512 是标准非制冷探测器规格 ⇒ 怀疑主码流是数字放大。

**实验设计**（关键是构造「已知的 2 倍放大」参照组，以排除「热成像画面本身平滑」这个混淆因素）：

| 样本 | 频谱高频/中频 | 半分辨率往返 PSNR |
|---|---|---|
| C 已知纯 2x 放大（640x512 辅码流升采样） | −12.56 dB | **39.16 dB** |
| **A 热成像主码流 1280x1024** | **−6.32 dB** | **30.65 dB** |
| B 可见光 2560x1440（已知原生，方法对照组） | −6.31 dB | 24.53 dB |

**结论**：A 距「纯放大基准」8.5 dB，且频谱表现与已知原生采样几乎完全一致 ⇒ **1280x1024 携带真实细节，拉它比拉 640x512 有信息增益。**

⚠️ **诚实的限定**：抓帧时镜头对着一面近乎无纹理的墙，起判别作用的主要是**像素级噪声**而非景物细节。噪声无法在插值中存活，因此「排除简单放大」成立；但要断定**探测器原生像元数**，datasheet 才是定论。

### 3.6 ⚠️ 热成像时钟曾停在 2000 年

OSD 显示 `2000-01-01 08:23:11`，根因 `table.NTP.Enable=false`（NTP 服务器地址与时区都已填好，就是没启用）。已修，见 §5。

★ 读热成像时间要用 `global.cgi?action=getCurrentTime`；`magicBox.cgi?action=getCurrentTime` 在此固件上返回 `Bad Request`。

---

## 4. 控制

> 复现脚本：ORIN `/usr/local/lib/ptz/onvif.py`（ONVIF 客户端）· `/usr/local/bin/ptzkey2`（方向键控制，含 `--selftest`）

### 4.1 ★★★ 驱动云台的是可见光相机，热成像完全不驱动

**热成像不驱动的证据**：

```
ptz.cgi?action=getCurrentProtocolCaps  ->  caps.Pan=false   caps.Tile=false
                                          caps.Zoom=false  caps.Focus=false
                                          caps.AutoFocus=false   （协议 DH-SD1）
ptz.cgi?action=start&code=Left|Right|Up|Down  ->  全部 Bad Request!
```

✅ 并用可见光抓帧前后对比验证：帧差 5.9~9.3，**未超过静止噪声地板 8.5** ⇒ 确实没动。

**可见光驱动的证据**：

```
Channels/0/PTZ/Capabilities  ->  IsSupportPTZ=1   MaxZoom=3300（33 倍）
                                 SupportFocus=1   SupportZoom=1   SupportIris=1
                                 MaxPTRange: Up=-9000 Down=9000 Left=0 Right=36000
                                 PTZDirectionList=[1..8]（8 方向，含 4 斜向）
Channels/0/PTZ/PTZCfg        ->  {"Name":"PELCO-D","PTZCtrlMode":0,"AddrCode":1}
Channels/0/PTZ/ExternalPTZRange  ->  与 MaxPTRange 同值
Channel/0/IO/SerialPort      ->  ChannelID=1，9600 8N1
```

★★★ **云台是 RS-485 上的外置机芯，可见光相机通过 PELCO-D 驱动它。** 这一条解释了后面几乎所有现象。

### 4.2 🚫 LAPI 的死路清单（试过，不通，🚫 别重复投入）

**① 绝对定位：接受但不执行。**

```
PUT Channel/0/System/DeviceStatus/PTZAbsPosition
{"StatusID":8,"StatusParam":{"PTZAbsPostion":{"Latitude":Y,"Longitude":X,"MoveSpeed":s}}}
```

✅ 返回 `code=0 Succeed`，但云台**一动不动**（多次不同目标角度，位置回读始终 `(180, 0)`）。
根因：**PELCO-D 不支持绝对定位，也没有位置反馈**。
★ 注意 API 有拼写错误 **`PTZAbsPostion`**（少个 i），必须照抄。
★★★ **位置回读 `(180, 0)` 是占位默认值，不是真实反馈** —— 🚫 任何逻辑不得依赖它。

**② `PTZCtrl` 端点：body 结构未攻破。**
UI 的 URL 表里它叫 `PTZAbsoluteMove`。试过 10 种 body（PUT/POST × 各种嵌套与字段命名），**全部 `code=1 Common Error`**。

**③ 整个 LAPI 没有连续点动（jog）接口。**
把 UI 的 JS 全下载后枚举出 **944 个 LAPI 路径、70 个 PTZ 端点**，运动原语只有：
预置位 goto · `HomePosition/Goto` · `Orientation/Goto` · 巡航启停 · `AreaZoomIn/Out` · `AbsoluteZoom`。
✅ Web UI 的方向按钮走 `webPlugin.min.js` **原生二进制协议，不走 HTTP**。
✅ 聚焦同理：`Channels/0/Image/Focus` 只有配置项，**没有 near/far 动作接口**。

⚠️★★ **未探索的第三条通道**：`docs/PTZ/SDK_Linux64-可见光.rar` 是厂商提供的 Linux64 SDK，
很可能正是 `webPlugin` 走的那条原生协议的官方封装。若如此，它可能同时绕开本节的三条死路
（真正的 jog 接口 · 更低的命令延迟 · 可能暴露 §4.6 ① 的俯仰行程参数）。**本轮未验证**，
优先级见 §9。★ 但注意：引入厂商 SDK 会给 xbrain 增加二进制依赖，需先评估其许可与
对 `common/` 地基库「🚫 不引 ROS/第三方重依赖」纪律的影响（CLAUDE.md §5.3）。

**④ 创建 ONVIF 用户：不通（且最终证明不需要）。**
`Channels/0/System/Security/OnvifUsers` 的 PUT 返回 `Not Supported`；POST 用 `Level` 字段返回 `Common Error`。
`Channels/0/System/OnvifAuthMode` **只接受 0 和 1**（2/3 → `Invalid Arguments`），改成 1 也不放行。

### 4.3 ★★★ ONVIF 是唯一可用通道，认证方式极其挑剔

ONVIF **本来就是开着的**（手册说要去 `配置 > 网络 > ONVIF` 手动开，实际已开）。

```
Device   http://192.168.66.13/onvif/device_service   （免认证）
Media    http://192.168.66.13/onvif/media
PTZ      http://192.168.66.13/onvif/ptz
Imaging  http://192.168.66.13/onvif/imaging

ProfileToken = media_profile1        VideoSourceToken = video_source
```

★★★ **唯一可用的认证组合（穷举 8 种得出）**：

| 维度 | 必须取值 |
|---|---|
| 方式 | WS-Security UsernameToken **PasswordDigest**（`SHA1(nonce + created + pwd)`） |
| `Created` | **不带毫秒**（`%Y-%m-%dT%H:%M:%SZ`） |
| HTTP 层认证 | ★★★ **绝对不能叠加** HTTP digest/basic —— 叠加了就回 `ter:NotAuthorized` |
| 其它 | `Security` 带 `s:mustUnderstand="1"`；`Created` 带 wsu 命名空间 |

✅ **设备账号 admin/`Admin123.` 直接可用，🚫 不需要创建 ONVIF 用户**
（`OnvifUsers` 为 `{"Number":0,"Users":[]}` 也照样能用）。

⚠️ **这一条是最容易卡死的地方**：我最初的实现只差「不叠加 HTTP 认证」这一点，报的错是 `ter:NotAuthorized`，极易误判成「凭据不对」或「缺 ONVIF 用户」而走进 §4.2 ④ 那条死路。

### 4.4 ✅ 可用的控制能力

| 能力 | 接口 | 实测 |
|---|---|---|
| X/Y 轴点动 | PTZ `ContinuousMove` / `Stop` | ✅ 四方向均确认位移 |
| 变焦 | `ContinuousMove` 的 `<tt:Zoom>` | ✅ 33 倍光学 |
| 聚焦近/远 | Imaging `Move` → `Continuous`（Speed −1..1） | ✅ 见下 |
| 聚焦绝对位置 | Imaging `Move` → `Absolute`（Position 0..1） | ✅ 能力集支持 |
| 自动对焦 | `SetImagingSettings` → `AutoFocusMode` AUTO/MANUAL | ✅ 见下 |

**X/Y 轴位移验证**（判据：抓帧前后平均像素差需超过静止噪声地板 3 倍）：

| 方向 | 帧差 | 判据 | 结果 |
|---|---|---|---|
| ← X 轴左转 | 66.88 | > 31.62 | ✅ 通过 |
| → X 轴右转 | 65.68 | | ✅ 通过 |
| ↑ Y 轴上仰 | 55.88 | | ✅ 通过 |
| ↓ Y 轴下俯 | 56.45 | | ✅ 通过 |

**聚焦验证**（判据：拉普拉斯方差＝清晰度，虚焦会掉一个数量级）：

| 动作 | 清晰度 | 结论 |
|---|---|---|
| 基线（AUTO 已合焦） | 2288.8 | — |
| 近焦 Continuous −0.8 / 1.0s | → 169.7（**−92.6%**） | ✅ 生效，明显虚焦 |
| 远焦 +0.8 / 1.0s | → 2184.1（95%） | ✅ 生效，回到合焦点 |
| 远焦再 +1.5s | → 179.1（8%） | ✅ **焦点行程会穿过合焦位置** |
| 自动对焦 AUTO | 8s→178.9，16s→180.7，**25s→2294.7（100%）** | ✅ 生效 |

★★★ **两个必须知道的坑**：

1. **自动对焦需要约 20~25 秒，而且是阶跃式恢复不是渐进。** 早测会误判成「AUTO 失效」——
   本次就先踩过一次（只等 6s，得出「−92.2%，自动对焦无效」的错误结论）。
2. ★★★ **ONVIF 的 `AutoFocusMode=AUTO` 不会映射回 LAPI 的 `Image/Focus.FocusMode`。**
   用 ONVIF 切过 MANUAL 后，即使再用 ONVIF 切回 AUTO 且画面已合焦，LAPI 仍读到
   `FocusMode=0`（手动），场景/倍率变化时不会自动重新对焦。
   ⇒ 收尾**必须**显式 `PUT Channels/0/Image/Focus` 把 `FocusMode` 写回 **2**。

### 4.5 手感与画质调优（三个独立成因）

**① 点动手感迟钝 —— 与网络无关，是保持时间设计错误。**

★★★ 保持时间必须**大于终端键盘自动重复的首次延迟（约 500ms）**。
原实现用 0.30s，于是「动 0.3s → 超时 Stop → 重复键才到 → 再启动」，表现为**转-停-转顿挫**。
✅ 正解是自适应：首次按下给 **0.85s** 跨过首延迟；检测到连续重复键后切到 **0.18s**，松手快停。

⚠️ **一个被证伪的假设**：我以为 58ms 的命令耗时是 TCP 握手，改成 keep-alive 持久连接后**只降到 55ms** ⇒ 这个开销在**相机侧不在网络侧**，🚫 别在连接复用上花时间。

**② 「一转图像就花」—— 是编码器带宽不足，不是对焦问题。**

A/B 实测：

| 编码配置 | 摇摄时清晰度 / 静止 | 实测码率 |
|---|---|---|
| 6144 kbps，I 间隔 50 | **31%** | 封顶 6.4 Mbps |
| 16384 kbps，I 间隔 25 | **51%** | 16.9 Mbps |

2560x1440@25fps 快速摇摄产生大量帧间残差，CBR 一封顶就宏块化；I 间隔 50（=2 秒）意味着要等两秒才恢复。
✅ 提码率＋缩 I 间隔可把摇摄画质提升约 65%（相对）。
⚠️ 剩余差距是 **1/100s 快门在最高速下的运动模糊**，属物理限制（`ShutterInfo.Shutter=100`），除非降速或提高快门（画面会更暗）。

**③ 转动时对焦发虚 —— `ShieldTrigger.MovePTZ` 默认没屏蔽。**

`Channels/0/Image/Focus` 的 `ShieldTrigger.MovePTZ` 出厂为 **0 = 不屏蔽**，云台一动就触发重新对焦。
✅ 置 **1** 后运动期间不再重新对焦。

**④ ⚠️ 俯仰比水平慢。**
同一速度值下，6s 俯仰的画面变化小于 2.5s 水平摇摄。写测试时要给俯仰更长时程，否则会误判成「没动」。
★ `PTZSpeedCoefficient` 两轴都是 50，`SpeedCfg.ManualSpeedLevel=5`。

### 4.6 ⚠️ 未解决问题（明日待办）

**① 俯仰实际只到 90°，API 声明 180°。**
`MaxPTRange` 与 `ExternalPTZRange` 都是 `Up=-9000, Down=9000`（即 ±90°，**180° 跨度**）。
软限位**已排除**：`PTZAngleLimitSwitch.Enable=0`，`PTZAngleLimit` 四值均为 `65536`（0x10000，未设置哨兵）。
待查（按可能性排序）：

- ⚠️ **`PTZ/PTDrvCfg.MaxVerAng = 0`** —— 字段名即「最大垂直角」，值为 0 很可疑；同对象内
  `MaxSpeed=1`、`AccelMode=-2147483648`（未初始化的 INT_MIN），整个 `PTDrvCfg` 疑似从未正确配置
- ⚠️ **`ExternalPTZRange` 是可写的** —— 它是相机侧对外置机芯的**声明值**，不是查询值，与机芯实际行程未必一致
- ⚠️ **PELCO-D 自动翻转（auto-flip）** —— 这类云台做 180° 俯仰通常靠「下俯到底后镜头翻转继续」，
  若机芯支持而未启用，正好卡在 90°

**② 左右与上下能否同时（斜向）—— 设备支持，工具不支持。**

- ✅ ONVIF `ContinuousMove` 的速度参数本就是二维向量 `<tt:PanTilt x=".." y="..">`，一条命令给两轴
- ✅ `PTZDirectionList=[1..8]` 含 4 个斜向
- ✅ PELCO-D 命令字节里上/下与左/右是**独立的位**，pan / tilt 速度是两个独立字节

⇒ 全链路支持斜向。当前 `ptzkey2` 的 `cur` 只存单一方向名，按下不同方向键会先 `stop()` 再发新命令，
**是工具自己排除了斜向**。改法：把「按住」状态**按轴拆开**维护（pan、tilt 各一个截止时间），
任一轴变化就发一次合并后的 `(x, y)` 向量。

**③ 红外相机的聚焦如何控制。**

已知：热成像 `caps.Focus=false` / `caps.AutoFocus=false`，`ptz.cgi` 各方向全 `Bad Request!`。
⚠️ **最可能是定焦无马达** —— 该级别非制冷热成像普遍使用定焦无热化镜头，这正是 caps 全 false 的原因。
待验证（按优先级）：

1. **测热成像自己的 ONVIF** —— 本轮只在可见光 `.13` 打通 ONVIF，`.108` 完全未试。
   若其 Imaging 服务 `GetMoveOptions` 返回 focus 能力则有对焦；返回空/fault 则基本定论。**一次请求即可决定**
2. 查大华镜头配置项 `configManager.cgi?action=getConfig&name=Lens` / `name=VideoInFocus`
3. ⚠️ 若确认定焦，真正需要的是 **NUC（快门/挡片校正）** 而非对焦 —— 热图发糊、出现固定花纹时靠 NUC 解决

**④ ★★★ 双光谱视场不同步（对 xbrain 融合有直接影响）。**
`+/-` 控制的是**可见光的 33 倍光学变焦**；热成像 `caps.Zoom=false`，视场固定。
⇒ **一旦放大可见光，两路视场不再一致。** 云台指向共享（同一机芯），但焦段不共享。
融合时要么把可见光锁在固定倍率标定，要么按当前 zoom 动态换算映射关系
（当前倍率可从 `DeviceStatus/PTZAbsZoom` 的 `PTZZoomNum` 读，现值 1）。

**⑤ 毫秒级帧对齐（RTP/RTCP）尚未验证。** 见 §5.3。

---

## 5. 时间同步

> 复现脚本：ORIN `/usr/local/bin/ptz-timesync`（含 systemd timer）

### 5.1 ✅ 已实施

| 部件 | 状态 |
|---|---|
| ORIN | 装 **chrony** 作服务端（`/etc/chrony/conf.d/ptz-ball.conf`：`allow 192.168.66.0/24` ＋ `local stratum 10` 兜底）。实测 Stratum 3，RMS offset ~447 µs，root dispersion ~491 µs |
| 热成像 | `NTP.Enable=true` 指向 `192.168.66.231`，`UpdatePeriod=1`，时区 Beijing。`chronyc clients` 确认每 64s 轮询 |
| 可见光 | NTP 已开启指向 ORIN，但其客户端不可靠（见下），改由 ORIN 定时推送 |

★★★ **ORIN 原用 systemd-timesyncd，它只是 SNTP 客户端、无法授时**，所以必须装 chrony。
★ 相机网关是布控球内置 4G 路由器，外网通路不确定，故由 ORIN 本机授时更可靠。

### 5.2 ★★★ 可见光 NTP 客户端的缺陷

`System/Time/NTP.SynchronizeInterval` **单位是分钟，且最小只能设 60**（设 1 直接 `Invalid Arguments`）。
✅ 实测启用后 3 分钟内对 chrony **零轮询**，`chronyc clients` 里只有热成像。
⇒ 它**一小时才同步一次**，期间实测偏到 **+24.9 秒长期不自愈**。

✅ 补偿方案：`/usr/local/bin/ptz-timesync` ＋ systemd timer（每 5 分钟），用 LAPI
`PUT System/Time`（`{"TimeZone":"GMT+08:00","DeviceTime":<epoch>}`）直接推时间。这条接口有效。

### 5.3 ★★★ 毫秒级做不到 —— 原因是硬的

**两台相机的时间接口都只接受和返回整秒**：

- 可见光 `System/TimePrivate/LocalTime` 虽有 `Milliseconds` 字段，但**恒为 0**（未填充）
- 热成像 `global.cgi?action=getCurrentTime` 只到秒

**后果**：读数会因截断在 1s 窗口内抖动 —— 实测 27 秒内读到 −0.250s 和 −0.806s，
那**不是真实漂移**（换算 20000 ppm，物理上不可能）。
⇒ `ptz-timesync` 的阈值必须设 **1.5s**，否则每次都被抖动骗去重写时间（初版设 0.5s 就踩了这个坑）。

★★★ **RTC 路径的极限约 ±0.5s。** 要真正**毫秒级帧对齐**（xbrain 双光谱融合需要的正是这个），
机制是 **RTP/RTCP Sender Report** —— RTCP SR 把 RTP 时间戳映射到 NTP 墙钟，
精度不受 RTC 整秒限制，只要求两相机 NTP 大致对齐（现状已满足）。
ORIN 上 GStreamer 1.20.3 具备：`rtspsrc ntp-sync=true ntp-time-source=clock-time buffer-mode=synced`。
⚠️ **本轮未验证**，列入待办。

---

## 6. ★★★ 测量方法论的坑（本轮踩过，🚫 别重复）

该机**位置回读是假值**，只能靠图像判位移。但两种度量都有失效条件：

| 度量 | 失效条件 | 表现 |
|---|---|---|
| **平均帧差** | 暗光下 AGC 增益震荡造成全帧亮度波动 | 静止基线从 2 涨到 31，与真实位移信号（37）同量级 ⇒ 判据失效。★ 8x8 池化**压不掉**（那不是 i.i.d. 噪声） |
| **相位相关** | ① 不加窗 ② 位移超过一个视场 | ① 边界不连续产生的 (0,0) 峰盖过真实峰 ⇒ 输出假的 **0 px**；② 前后帧无重叠，本就无解 |

✅ **可用口径**：

- 光线好时用**帧差**（实测 49~68 vs 地板 5~10，区分明确），判据取 **地板 × 3**
- **判据必须相对实测噪声地板**，🚫 不能用绝对阈值 —— 地板随场景/光照从 2.1 变到 31
- 暗光下自动判定不可靠，**以肉眼为准**，🚫 不要硬套阈值出结论
- 俯仰要给更长时程（比水平慢）
- 相位相关**必须加汉宁窗**，且只适合小的帧间位移

★★★ 更一般的教训：**本轮有三次「测量工具失效」被误读成「硬件故障」**
（AUTO 对焦只等 6s ⇒ 误判失效 · 帧差在暗光下失效 ⇒ 误判没动 · 相位相关未加窗 ⇒ 误判没动）。
⇒ 任何「没检出」的结论，**先质疑判据再质疑硬件**。

---

## 7. 交付物清单（ORIN 上）

| 路径 | 用途 |
|---|---|
| `/usr/local/lib/ptz/onvif.py` | ONVIF 客户端（含唯一可用的认证实现 ＋ 持久连接 `Session`） |
| `/usr/local/bin/ptzkey2` → `/usr/local/lib/ptz/ptzkey2.py` | ★ **方向键控制（推荐）**：速度固定 1.0、自适应保持时间、运行期间提码率并在退出时恢复、启动即置 `FocusMode=2` ＋ `MovePTZ=1` |
| `/usr/local/bin/ptzkey` → `/usr/local/lib/ptz/ptzkey.py` | 初版（保留对比用；⚠️ 存在 §4.4 坑 2 的对焦遗留问题） |
| `/usr/local/bin/ptz-timesync` ＋ `ptz-timesync.timer` | 向两台相机推送 ORIN 时间 |
| `/etc/chrony/conf.d/ptz-ball.conf` | chrony 对 66 网段授时 |

**`ptzkey2` 按键**：

```
← → ↑ ↓   X/Y 轴转动（速度固定 1.0），按住持续转，松开即停
+ -       变焦 拉近 / 拉远
f         立刻重新对焦一次
空格      停止           q  退出
--no-boost   不临时提高码率      --selftest   非交互自检
```

⚠️ **临时脚本**：`/tmp/probe2.py`（拉流实测）· `/tmp/spectral.py`（原生分辨率判定）·
`/tmp/blur_ab.py`（画质 A/B）等位于 `/tmp`，**重启即失**。需长期保留应移入
`/opt/xbrain_v6/scripts/`（按 CLAUDE.md §0.2 目录铁律）。

---

## 8. 关键凭据

| 部件 | 地址 | 账号 |
|---|---|---|
| 路由器 | `192.168.66.1` | admin / admin |
| 可见光 | `192.168.66.13` | admin / `Admin123.` |
| 热成像 | `192.168.66.108` | admin / `Admin123.` |
| AP 热点 | SSID `PTZ-CAMERA-5.8G-XXXXXX` | `12345678` |

★ 热成像另开 **37777**（大华私有 SDK 端口），除 RTSP/ONVIF 外还有这条私有协议可走。
⚠️ 两台相机均有**连续认证失败锁定账户**的策略，🚫 严禁撞密码；探针内已硬编码「认证一失败立即中止」。

---

## 9. 待办

| # | 事项 | 优先级 | 备注 |
|---|---|---|---|
| 1 | 热成像 ONVIF Imaging 探测，定论红外有无对焦 | ★★★ | 一次请求即可决定，见 §4.6 ③ |
| 2 | 俯仰 180° 行程排查（`PTDrvCfg.MaxVerAng` / `ExternalPTZRange` / auto-flip） | ★★★ | 见 §4.6 ① |
| 2b | 评估 `SDK_Linux64-可见光.rar` —— 是否提供真正的 jog 接口与俯仰行程参数 | ★★★ | 可能一并解决待办 2、3 与命令延迟；见 §4.2 末段 |
| 3 | `ptzkey2` 支持斜向（按轴拆分「按住」状态） | ★★ | 见 §4.6 ② |
| 4 | RTP/RTCP Sender Report 毫秒级帧对齐验证 | ★★ | 见 §5.3 |
| 5 | 若确认红外定焦，找出 NUC（快门校正）触发接口 | ★★ | 见 §4.6 ③ |
| 6 | 双光谱视场随 zoom 变化的映射标定 | ★★ | 见 §4.6 ④ |
| 7 | 探针脚本从 `/tmp` 迁入 `/opt/xbrain_v6/scripts/` | ★ | `/tmp` 重启即失 |
| 8 | 决定编码码率是否永久提高（当前仅 `ptzkey2` 运行期间提） | ★ | 权衡带宽/存储，见 §4.5 ② |
