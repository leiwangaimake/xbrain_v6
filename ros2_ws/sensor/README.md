# sensor —— 机器人端 RTK 软件设计（rtk_driver ＋ rtcm_gateway）

> **本文件是机器人端 RTK 软件的设计正本**：`docs/` 下没有 RTK 专册，代码实现与代码审查以本文为准。
> 跨进程 / 跨设备的**契约**不在这里定义，只在 `docs/11-接口契约.md`：
> §3.2 `GnssFix` · **§3.2A RTCM 差分改正数输入（RTB / RTG 条款）** · §3.3 `GnssHeading` · §3.11 `ClockStatus` · §1.5 授时。
> 本文与 `11` 冲突时以 `11` 为准，并按 `CLAUDE.md` 铁律 1 回头修本文。

状态标记：✅ 已实现并实测 · 🚧 已定设计、待实现 · ⚠️ 待裁定 / 待实测（编号指向 `11` §3.2A.3）

---

## 0. 包边界

- **不是 ROS 2 节点**。`rtk_driver` 是非 ROS 的 C++ 进程，发布到 **RT 面 Zenoh**，用纯 CMake 构建（不链接 rclcpp / ament / colcon）。
  目录放在 `ros2_ws/` 下只是因为机器人侧 C++ 资产都在这里；「非 ROS」由 CMakeLists 不链接任何 ROS 目标保证。
- 只在 **ORIN（aarch64）** 上构建和运行，不要在 x86 主机上构建产物。
- 本包现有一个进程 `rtk_driver`（✅），计划新增一个进程 `rtcm_gateway`（🚧）。树莓派上的 `rtcm_bridge` 不在本包，见 §3.2。

---

## 1. 总体框架

```
  FDI base ──UART──> RPi rtcm_bridge (TCP server :PORT)            Qianxun caster
                              │ raw bytes 1:1, customer LAN              │ NTRIP v1/v2, 5G/WiFi
                              v                                          v
 ═══════════════════════════════════ ROBOT (ORIN) ══════════════════════════════════════
 ┌──────────────────────────────────────────────────────────────────────────────────┐
 │ rtcm_gateway   (NEW, C++, xbrain-rtcm-gateway.service)                    🚧     │
 │                                                                                  │
 │  ┌────────────────────┐              ┌────────────────────┐                      │
 │  │ SrcTcp  (primary)  │              │ SrcNtrip (backup)  │<──── GgaBuilder <──┐ │
 │  │ TCP client         │              │ HTTP/ICY + auth    │                    │ │
 │  │ reconnect+backoff  │              │ LoadCredential     │                    │ │
 │  └─────────┬──────────┘              └─────────┬──────────┘                    │ │
 │            v                                   v                               │ │
 │  ┌────────────────────┐              ┌────────────────────┐                    │ │
 │  │ Rtcm3Framer        │              │ Rtcm3Framer        │  0xD3 sync, CRC24Q │ │
 │  │ SourceHealth       │              │ SourceHealth       │  msg ids, frame age│ │
 │  └─────────┬──────────┘              └─────────┬──────────┘                    │ │
 │            └──────────────┬────────────────────┘                               │ │
 │                           v                                                    │ │
 │                 ┌──────────────────────┐  primary first; down fast, up slow    │ │
 │                 │ SourceSelector       │  one source at a time; flush on switch│ │
 │                 └──────────┬───────────┘                                       │ │
 │                            │ whole valid frames only        status: src/link/age │
 └────────────────────────────┼───────────────────────────────────────────────────┼─┘
                              v RTCM channel  (rt/gnss/rtcm | UDS -- U-RTCM-1)     │
 ┌──────────────────────────────────────────────────────────────────────────────┐ │
 │ rtk_driver   (EXISTING, C++, RT plane, 20 Hz)                          ✅/🚧 │ │
 │                                                                              │ │
 │  RtcmSink: bounded queue, drop oldest WHOLE frame ──┐                   🚧   │ │
 │                                                     v                        │ │
 │  serial fd  O_RDONLY -> O_RDWR   non-blocking write, partial-write safe 🚧   │ │
 │      │ NMEA read                                                        ✅   │ │
 │      v                                                                       │ │
 │  nmea_parser (+ GGA field 13 diff age 🚧) -> heading_resolver -> publish:    │ │
 │      rt/gnss/fix     (age_s := diff age per 11 S3.2) ─────────────────────────┼─┘
 │      rt/gnss/heading, rt/clock/status   (unchanged)                          │
 └──────────────────────────────────────┬───────────────────────────────────────┘
                                        │ UART 115200 (CH343, /dev/ttyACM0)
                                        │   RTCM ↓   NMEA ↑
                                 ┌──────┴──────┐
                                 │    UM982    │
                                 └─────────────┘
```

**为什么分成两个进程**（`11` RTG-1）：`rtk_driver` 是全系统唯一的授时同步判定者（CLK-A1）。
NTRIP 要做 DNS、公网 TCP、凭据处理，任何一处阻塞或崩溃如果发生在 `rtk_driver` 里，`rt/clock/status` 就会停发，
5 s 后全系统 `ts_sync = false`（CLK-A5）。放在独立进程里，最坏情况只是改正数中断，接收机自己从固定解退到浮点 / 单点，
`11` §3.2.1 的降级链照常生效。

**为什么串口只归 `rtk_driver`**（`11` RTG-2）：UM982 只有一个 COM 口经 CH343 接到 ORIN，NMEA 读和 RTCM 写必须在同一个 fd 上；
两个进程同时打开同一个 tty 会抢读字节。

---

## 2. rtk_driver 现状（✅ 已实现）

### 2.1 数据流

```
 /dev/ttyACM0 ──read()──> RtkDriver::feed()   按行切分 NMEA，按到达时刻(单调钟)给每条事实打时戳
                              │  GGA → 位置 / 解质量 / 卫星数 / HDOP
                              │  TRA → 双天线航向 + QF
                              │  RMC → 地速 + 航迹向 (COG)
                              v
                          RtkDriver::tick()  20 Hz
                              │  HeadingResolver: L1 / L2 / L3 降级链 (11 S3.3)
                              ├──> rt/gnss/heading   (GnssHeading, 11 S3.3)
                              └──> rt/gnss/fix       (GnssFix, 11 S3.2)
                          RtkDriver::tickClock()  1 Hz
                              │  chronyc -c tracking → ClockStatus 判定 (CLK-A1 / A2)
                              └──> rt/clock/status   (ClockStatus, 11 S3.11)
```

全部键形如 `xbrain/{rid}/rt/...`，外层是 `11` §3.0 信封。

### 2.2 启动顺序（fail-stop）

`main.cc` 按下列顺序启动，任一步失败都打印英文原因并以非零码退出，进程绝不在配置不全时运行：

1. 读身份：`XBRAIN_ROBOT_ID`（必填，无默认）；`boot` = `/proc/sys/kernel/random/boot_id` 前 8 位十六进制（与 Python `read_local_boot_id` 一致）；`src` 固定为 `rtk_driver`。
2. 读配置：冻结产物 `$XBRAIN_RESOLVED_DIR/rtk_driver.yaml`（默认目录 `/run/xbrain/resolved`），不读源文件（`10` §5.4.1）。任何阈值缺失或为 null 都抛异常（`CLAUDE.md` 3.1）。
3. 打开串口（raw 8N1，非阻塞）。
4. 打开 RT 面 `ZenohSink`（RT 路由不可达即抛异常）。
5. 进入 20 Hz 主循环，周期用 `steady_clock`；墙钟只用于信封 `ts`。

### 2.3 航向等级

| 等级 | 来源 | 准入（`11` §3.3.1） | `i_heading` |
|---|---|---|---|
| L1 | `dual_antenna`（TRA） | 基线固定（QF=4）且 cov ≤ 0.02 rad 且龄期 ≤ 0.2 s | 1.0 |
| L2 | `cog`（RMC） | 有解（`!fix_is_lost`）且速度 ≥ 0.5 m/s 持续 0.5 s | 0.4 |
| L3 | `none` | 都不满足，航向冻结在最后已知值（H-2） | 0.0 |

`heading_valid` 是下游唯一判据（H-1）。航向用 ENU（东 = 0，逆时针），模块输出的真北顺时针角度按 `heading_enu = wrap(pi/2 - heading_ned)` 转换。
所有阈值来自配置，上表数值是 `configs/rtk_driver.yaml` 当前值。

### 2.4 GGA 解质量映射

| GGA quality | `fix_type` | 说明 |
|---|---|---|
| 4 | `rtk_fixed` | 厘米级，需要差分改正数 |
| 5 | `rtk_float` | 分米级，需要差分改正数 |
| 2 | `dgps` | 米级 |
| 1 / 0 / 6 | `single` / `no_fix` / `no_fix` | 视作 `fix_is_lost`（不准 L2） |

GGA 超过 `gga_timeout_s` 未更新时 `fix_type = no_fix`。`cov_h_m = nominal(fix_type) × max(hdop, 1)`，各档 nominal 值在配置 `fix_cov.*`。

### 2.5 串口热插拔

`serial_reopen.h` 的纯函数 `classify_serial_read()` 判定每次 `read()` 的结果：有数据就喂给解析，EOF 或硬错误（ENODEV / EIO）就重开，
EAGAIN 但超过 3 s 无数据也重开（有的内核拔掉设备后一直返回 EAGAIN）。断开期间约 1 Hz 重试 `open()`，USB 插回后自动恢复。

### 2.6 已知与契约不符（随 RTCM 一并修）

| # | 现状 | 契约 | 处置 |
|---|---|---|---|
| 1 | `GnssFix.age_s` 填的是「距最近一条 GGA 的秒数」（`rtk_driver.cc` 中 `now_mono_s - gga_mono_`） | `11` §3.2：`age_s` = **差分龄期**；新鲜度只用 `t_mono` 判 | 🚧 解析 GGA 第 13 字段（见 §4.6.4）。无差分时的取值待 ⚠️ **U-RTCM-3** |
| 2 | 串口 `O_RDONLY`，没有写路径 | `11` RTG-2：`rtk_driver` 负责写 RTCM | 🚧 §4.6 |

---

## 3. 外部差分源

### 3.1 两个源的角色

| | 主源：LAN 自建基站 | 备源：千寻 CORS |
|---|---|---|
| 用途 | 客户现场落地 | 调试；主源无数据时兜底 |
| 物理链路 | FDI 基站 UART → 树莓派 → 客户自组网 LAN → 机器人 | 公网（5G / WiFi）→ 机器人 |
| 协议 | 裸 TCP，RTCM3 原始字节 1:1（`11` RTB） | NTRIP v1 / v2，VRS |
| 上行 | 无（RTB-3：发什么都被丢弃） | 周期上报 GGA（VRS 必需） |
| 认证 | 无（靠网段绑定，RTB-9） | 账号口令，`LoadCredential`（RTG-9） |
| 机器人数量 | 一个基站服务 LAN 内所有机器人 | 每台机器人一个账号（千寻通常一号一机在线，待与千寻确认） |
| 连接时机 | 常连 | ⚠️ **U-RTCM-2**：冷备（建议）或热备 |
| 与地点的关系 | 每个现场一台基站，`lan.host/port` 随现场配置 | **不依赖任何与地点有关的配置**：GGA 只用机器人实测位置，走远了自动重建 VRS（`11` RTG-7 / RTG-12）。同一份配置在上海、南京等任意地点可用 |

### 3.2 树莓派 `rtcm_bridge`（契约见 `11` §3.2A.1）

机器人端依赖它的以下行为，代码不得假设更多：

- 连上就收流，**没有握手、没有帧头、没有心跳**，从任意字节开始 ⇒ `Rtcm3Framer` 必须能从流中间同步（RTB-2）。
- UART 断了时树莓派**不断开 TCP**，只是没有数据 ⇒ 「连着但没数据」必须靠帧级龄期判失效，不能只看 TCP 状态（RTB-8）。
- 客户端积压太多会被树莓派主动断开 ⇒ `SrcTcp` 读取要及时，不能在读路径上做阻塞操作（RTB-4）。
- 实现位置不在本包（树莓派不是 ORIN）；建议放 `deploy/rtcm_bridge/`，待实现时定。

### 3.3 坐标基准（交付项，代码无法兜底）

自建基站必须以**固定坐标**模式工作，坐标与千寻同一基准（`11` RTG-8 / RTB-11）。否则主备切换时机器人坐标整体跳变，
在主源下录制的围栏、路线、关键点到了备源下全部错位。部署时用千寻 RTK 或测绘把基站天线坐标测准后写入基站。

---

## 4. RTCM 差分输入设计（🚧）

### 4.1 设计约束

| # | 约束 | 出处 |
|---|---|---|
| C1 | 任一时刻只把一个源的帧写进接收机 | RTG-4 |
| C2 | 只写 CRC 校验通过的**整帧**，绝不写半帧 | RTG-3 / RTG-6 |
| C3 | RTCM 写不得影响 NMEA 读和 20 Hz 发布节拍 | RTG-6，`rtk_driver` 现有节拍 |
| C4 | 网络问题不得波及 `rtk_driver` 进程 | RTG-1 |
| C5 | 所有阈值、周期、地址来自配置，代码无默认值 | `CLAUDE.md` 3.1 |
| C6 | 所有龄期 / 超时用单调钟；墙钟只用于信封 `ts` 和 GGA 的 UTC 时间字段 | CLK-C1 |
| C7 | 口令不进 argv / environ / yaml / 日志 / 任何上报 | RTG-9，`11` §7.4.9 |
| C8 | 不新增速度门：改正数中断由接收机自身退化，交给已有 `i_fix` 链路 | RTG-10 |

### 4.2 rtcm_gateway 模块划分

纯逻辑模块不做 I/O、不读时钟（时间由调用方传入），全部可离线单测；I/O 只在 `rtcm_gateway_main.cc`。

| 模块 | 文件（计划） | 职责 | 纯 / I/O |
|---|---|---|---|
| `Rtcm3Framer` | `rtcm3_framer.{h,cc}` | 字节流 → 合法 RTCM3 帧；提取电文号与基站 ID | 纯 |
| `SourceHealth` | `rtcm_source_health.{h,cc}` | 每个源的统计与「是否有效」判定 | 纯 |
| `SourceSelector` | `rtcm_selector.{h,cc}` | 主备状态机，决定当前输出哪个源 | 纯 |
| `NtripProtocol` | `ntrip_protocol.{h,cc}` | 构造请求、解析响应头、chunked 解码 | 纯 |
| `GgaBuilder` | `gga_builder.{h,cc}` | 由位置合成带校验和的 `$GPGGA` 语句 | 纯 |
| `VrsAnchor` | `vrs_anchor.{h,cc}` | 判定位置是否新鲜可报、记录 VRS 锚点、按大圆距离判定是否需要重建 | 纯 |
| `RtcmGatewayConfig` | `rtcm_gateway_config.{h,cc}` | 读冻结配置，缺值即抛 | 纯（读文件） |
| `SrcTcp` / `SrcNtrip` | `rtcm_gateway_main.cc` 内 | 非阻塞 socket 连接、重连退避、读数据 | I/O |
| 主循环 | `rtcm_gateway_main.cc` | `poll()` 事件循环、定时器、发布 | I/O |

`rtk_driver` 侧新增 `RtcmSink`（`rtcm_sink.{h,cc}`，纯：写操作通过注入的写函数完成，便于用假 fd 单测）。

### 4.3 线程模型

- **单线程 `poll()` 事件循环**处理所有 socket（两个源连接 + 通道），超时取最近一个定时器（重连退避、GGA 上报、状态发布）。
- **DNS 解析放辅助线程**：`getaddrinfo()` 是阻塞调用，千寻主机名解析可能卡数秒。辅助线程解析完把结果交回主循环；超时视为连接失败进入退避。LAN 源用 IP 直连，不解析。
- 如果通道选 Zenoh（U-RTCM-1 方案 A），订阅 `rt/gnss/fix` 的回调在 Zenoh 线程里执行，只把最新位置拷进一个加锁的槽位，GGA 由主循环取用。

### 4.4 源接入

#### 4.4.1 SrcTcp（主源，LAN 裸流）

```
 DISCONNECTED ──退避到期──> CONNECTING ──connect 成功──> STREAMING
      ^                         │                          │
      └──── 失败 / 超时 ────────┘                          │
      └──── EOF / 错误 / 读超时 ───────────────────────────┘
```

- 非阻塞 `connect()`，连接超时取配置。
- 开 `SO_KEEPALIVE`，参数与 RTB-7 对齐，用于发现对端掉线后的半开连接。
- **读超时**：STREAMING 状态下超过配置时长没有收到任何字节就主动断开重连。注意这只是链路层兜底；「连着但只有垃圾、没有合法帧」由 `SourceHealth` 判失效，不在这里判。
- 重连退避：从 `backoff_min_s` 起每次翻倍，封顶 `backoff_max_s`；进入 STREAMING 后重置。
- 收到的字节直接喂给该源自己的 `Rtcm3Framer`。

#### 4.4.2 SrcNtrip（备源，千寻）

请求（`ntrip_version` 配置项选择；口令只在内存中拼 Basic 认证头）：

```
 v1:  GET /{mountpoint} HTTP/1.0\r\n
      User-Agent: NTRIP xbrain-rtcm-gateway/1.0\r\n
      Authorization: Basic base64(user:password)\r\n
      \r\n

 v2:  GET /{mountpoint} HTTP/1.1\r\n
      Host: {host}:{port}\r\n
      Ntrip-Version: Ntrip/2.0\r\n
      User-Agent: NTRIP xbrain-rtcm-gateway/1.0\r\n
      Authorization: Basic base64(user:password)\r\n
      Connection: close\r\n
      \r\n
```

响应处理：

| 响应首行 | 含义 | 处置 |
|---|---|---|
| `ICY 200 OK` | v1 成功 | 进入 STREAMING，后续字节全是 RTCM |
| `HTTP/1.x 200 OK` | v2（或部分 v1 caster）成功 | 读完响应头；若 `Transfer-Encoding: chunked` 则经 chunked 解码后再喂 framer |
| `SOURCETABLE 200 OK` | 挂载点不存在，caster 返回源表 | 记 `bad_mountpoint`，按长退避重试（配置错误，快速重试无意义） |
| `HTTP/1.x 401` | 认证失败 | 记 `auth_failed`，按长退避重试，避免频繁试错导致账号被封；日志不打印用户名以外的任何凭据信息 |
| 其他 / 头超长 / 超时 | 异常 | 断开，普通退避 |

GGA 上报与「任意地点可用」（`11` RTG-7 / RTG-12）：

千寻 VRS 按机器人上报的 GGA 位置就近生成虚拟基站。所以**上报的位置必须是机器人此刻真实所在的位置**，
否则在上海配好的东西拿到南京（约 270 km）就会生成错误位置的虚拟基站，基线过长，无法固定甚至给出错误解。规则如下：

- **位置只取实测**：最近一条 `has_position == true` 的 `rt/gnss/fix`（单点解即可，VRS 只需要公里级位置），且龄期不超过 `gga_max_pos_age_s`。
- **没有任何替代坐标**：🚫 配置里的站点坐标，🚫 掉电前缓存的坐标（机器人可能是关机后被运到另一个城市的）。
- **没有新鲜位置就等**：SrcNtrip 增加 `WAIT_POSITION` 状态——需要备源但还没有新鲜位置时不建连；已连接时位置过旧就暂停上报（不发旧位置），恢复后继续。开机后 UM982 单点解通常很快就有，等待代价很小。
- **上报节奏**：进入 STREAMING 后立即发一条 GGA，之后每 `gga_period_s` 发一条（不发就收不到数据）。GGA 的 UTC 时间字段取墙钟，属于对外报文时间而非龄期判断，按 `clock_scan` 规则标 `WALL-CLOCK-OK`。
- **VRS 锚点与重建**：会话建立后，把首条上报 GGA 的位置记为锚点；之后当前位置与锚点的大圆距离超过 `vrs_rebase_km` ⇒ 主动断开重连，用当前位置重建 VRS（阈值与「千寻是否会随 GGA 自动更新 VRS」待 ⚠️ **M-RTCM-4** 实测）。
- **冷备**（U-RTCM-2 建议方案）：主源失效时才发起连接；切回主源后再保持 `linger_s` 才断开，避免主源抖动时反复建连。
- **账号覆盖**：千寻账号可能限定服务区域，换城市测试前确认账号覆盖目的地（⚠️ **V-RTCM-2**）。这一点代码无法检测，只能从连不上 / 无数据的状态里看出来。

#### 4.4.3 凭据

- systemd：`LoadCredential=ntrip:/etc/credstore/xbrain-ntrip`（文件 `0400 root`）。
- 进程从 `$CREDENTIALS_DIRECTORY/ntrip` 读一行 `user:password`（正好是 Basic 认证的明文格式）。
- 文件缺失或格式错误：NTRIP 源进入 `disabled` 状态并在状态中如实报出；**不影响主源工作**，进程不退出。

### 4.5 Rtcm3Framer

RTCM 3.x 帧结构：

```
 +--------+-----------+-----------------+------------------+----------+
 | 0xD3   | 6 bit = 0 | 10 bit length L | payload (L bytes) | CRC-24Q  |
 | 1 byte |       2 bytes               | 0 <= L <= 1023    | 3 bytes  |
 +--------+-----------+-----------------+------------------+----------+
 CRC-24Q 覆盖 header 3 字节 + payload；多项式 0x1864CFB，初值 0。
 电文号 = payload 前 12 bit；1005/1006、MSM、1004/1012 的基站 ID 紧随其后 12 bit。
```

算法：

1. 在缓冲区里找 `0xD3`；之前的字节丢弃（计入 `garbage_bytes`）。
2. 不足 3 字节头就等待更多数据。保留位非 0 视为伪帧头：丢 1 字节，回到第 1 步。
3. 不足 `3 + L + 3` 字节就等待。
4. 校验 CRC。**失败时只丢弃这一个 `0xD3` 字节**，从下一字节重新找帧头（不能按 L 整段跳过，L 本身可能就是垃圾）。计入 `crc_errors`。
5. 成功：输出整帧（含头和 CRC，原样写给接收机），附电文号、基站 ID、到达单调时刻。
6. 缓冲区上限 `2 × 1029` 字节，超出时丢最旧数据并重新同步。

### 4.6 SourceHealth 与 SourceSelector

#### 4.6.1 每源统计（SourceHealth）

| 字段 | 含义 |
|---|---|
| `link` | `disconnected` / `connecting` / `streaming` / `auth_failed` / `bad_mountpoint` / `disabled` |
| `bytes_rx` · `frames_ok` · `crc_errors` · `garbage_bytes` | 累计计数 |
| `last_obs_mono` | 最近一个观测电文（MSM 107x/108x/109x/112x 或 1004/1012）的到达时刻 |
| `last_station_mono` | 最近一个基站坐标电文（1005/1006）的到达时刻 |
| `station_id` | 最近的基站 ID；变化时计 `station_id_changes` 并记日志 |
| `msg_types` | 最近窗口内见过的电文号集合（排障用） |

**有效判据**（RTG-3）：`now - last_obs_mono <= obs_window_s` 且 `now - last_station_mono <= station_window_s`。
两个窗口为配置项（⚠️ **M-RTCM-1**；基站坐标电文通常 1–10 s 一条，`station_window_s` 要大于基站实际间隔）。

#### 4.6.2 主备状态机（SourceSelector）

```
                  primary invalid (immediately), backup valid
       ┌──────────┐ ───────────────────────────────────────────> ┌──────────┐
       │ PRIMARY  │                                              │  BACKUP  │
       └──────────┘ <─────────────────────────────────────────── └──────────┘
          │     ^     primary valid continuously >= hold_up_s       │     ^
          │     │                                                   │     │
  primary │     │ primary valid                        both invalid │     │ backup valid,
  invalid,│     │                                                   │     │ primary invalid
  backup  v     │                                                   v     │
  invalid ┌────────────────────────────────────────────────────────────────┐
          │                              NONE                              │
          └────────────────────────────────────────────────────────────────┘
```

| 当前 | 条件 | 下一状态 | 动作 |
|---|---|---|---|
| NONE | 主源有效 | PRIMARY | 开始输出主源帧 |
| NONE | 主源无效，备源有效 | BACKUP | 开始输出备源帧 |
| PRIMARY | 主源无效（**立即**） | BACKUP 或 NONE | 冷备时此刻发起 NTRIP 连接；清空待写帧 |
| BACKUP | 主源**连续有效**满 `hold_up_s` | PRIMARY | 清空待写帧；冷备时启动 `ntrip_linger_s` 计时 |
| BACKUP | 备源无效且主源无效 | NONE | 停止输出 |

- 切源当拍起只转发新源的帧；旧源已排队但未写出的帧全部丢弃（RTG-4）。
- 主源「连续有效」计时在主源任何一拍无效时清零。
- NONE 状态不向 `rtk_driver` 发任何东西，接收机按自身差分超时退化。

### 4.7 RTCM 通道（⚠️ U-RTCM-1）

| | 方案 A（建议）：RT 面 `xbrain/{rid}/rt/gnss/rtcm` | 方案 B：本地 Unix socket |
|---|---|---|
| 可观测 / 可录制 | 能，和其他 RT 键一样 | 不能，需要另做工具 |
| 文档改动 | `rtcm_gateway` 进 `11` §1.1.3 RT 面参与者封闭集；§2.2.1 RT 键表加行；`10` §3.1 进程清单加行 | 新增一条 Zenoh 之外的 IPC，需在 `10` 登记 |
| `rtk_driver` 接收 | Zenoh 订阅回调 → 加锁队列 → 主循环取 | 主循环里非阻塞 `recv()` |

方案 A 的消息草案（随 U-RTCM-1 裁定后写进 `11`）：外层 `11` §3.0 信封，`data` 为

```json5
{
  "src": "lan",          // lan | ntrip —— 本批帧来自哪个源
  "seq": 1234,           // 网关侧单调递增，接收方据此统计丢失
  "frames": ["0xD3..."], // 每个元素一整帧，base64；一条消息可带多帧
  "t_mono": 12345.678    // 网关收到最后一帧的单调时刻
}
```

无论选哪种，**通道上只传整帧、只传当前选中源的帧**。

### 4.8 rtk_driver 改造

#### 4.8.1 串口改读写

- `open(port, O_RDWR | O_NOCTTY | O_NONBLOCK)`，其余 termios 设置不变。
- 热插拔重开逻辑不变；重开时**清空** `RtcmSink` 的队列和半帧余量（接收机在断开期间已丢了半帧，旧帧也已过期）。

#### 4.8.2 RtcmSink

```
 channel ──> push(frame, t_arrive) ──> deque<Frame>  (bytes <= queue_max_bytes)
                                          │  over cap: drop oldest WHOLE frame
                                          v
 each 50 ms tick ──> drain(writer):
     1) 先写完上一拍剩下的半帧余量 (partial_offset)
     2) 丢弃 now - t_arrive > frame_max_age_s 的帧
     3) 逐帧 write()，直到 EAGAIN 或队列空
        - write 返回 < 帧长: 记 partial_offset，下一拍从这里续写
        - EAGAIN: 停止，下一拍继续
        - 其他错误: 交给现有 classify 逻辑触发重开
```

- 写预算：115200 baud 8N1 约 11.5 KB/s，每 50 ms 约 576 字节；四系统 MSM4 改正数约 1–3 KB/s，余量足够。
- 一个半帧写出后必须先把同一帧剩余部分写完，才能开始下一帧——这保证接收机看到的永远是完整帧序列（C2）。
- 写操作在主循环 `read()` 之后、`driver.tick()` 之前执行，不改变发布节拍（C3）。
- 计数：`rtcm_frames_written` · `rtcm_bytes_written` · `rtcm_dropped_overflow` · `rtcm_dropped_stale`，随现有心跳日志打印。

#### 4.8.3 输入侧

- 方案 A：订阅回调只做「解 base64 → 推入加锁队列」，不碰串口；主循环取出后交 `RtcmSink`。
- 收到的消息 `src` 与上一条不同时，清空队列（双保险，网关侧已清空）。

#### 4.8.4 `age_s` 改为差分龄期

- `GgaFix` 增加差分龄期字段，解析 GGA 第 13 字段（`nmea_split_fields` 下标 13；第 14 字段为差分基站 ID，可一并解析用于排障）。
- 字段为空表示没有差分 ⇒ 按 ⚠️ **U-RTCM-3** 裁定的表达填 `age_s`。
- `fix_type` 判定、`fix_is_lost`、GGA 新鲜度判断都不变（新鲜度依旧看 `gga_mono_` / `t_mono`）。

### 4.9 状态与可观测性

`rtcm_gateway` 每秒发布一条状态（键和平面随 U-RTCM-1 定，契约待补进 `11`）：

```json5
{
  "active": "lan",                 // lan | ntrip | none
  "since_mono": 1234.5,            // 进入当前状态的单调时刻
  "switch_count": 3,
  "lan":   { "link": "streaming", "valid": true,  "obs_age_s": 0.4, "station_age_s": 2.1,
             "station_id": 1, "bytes_rx": 123456, "crc_errors": 0 },
  "ntrip": { "link": "disconnected", "valid": false, "obs_age_s": null, "station_age_s": null,
             "station_id": null, "bytes_rx": 0, "crc_errors": 0 }
}
```

- 切源时打印一条英文日志（含新旧源与原因）。是否另发事件（`event` 在通用面）随 U-RTCM-1 一起定。
- 后续可在 HMI / 语音（G43「定位状态」）里补「差分来源 / 差分龄期」，属后续需求，不在本期。

### 4.10 配置

`configs/rtcm_gateway.yaml`（新增，走冻结流程：`10` §5.4.0 登记、freeze 的 `_L6_FILES` / SNAPSHOT 三处同步加入、materialize 产出 `resolved/rtcm_gateway.yaml`）。
**数值全部待定**（⚠️ M-RTCM-1），下面只列键：

```yaml
lan:
  host:                 # rtcm_bridge 静态 IP
  port:                 # RTB-1 固定端口
  connect_timeout_s:
  read_timeout_s:       # STREAMING 下无任何字节即断开重连
  backoff_min_s:
  backoff_max_s:
ntrip:
  enabled:              # false = 不使用备源
  host:
  port:
  mountpoint:
  ntrip_version:        # 1 | 2
  connect_timeout_s:
  read_timeout_s:
  backoff_min_s:
  backoff_max_s:
  backoff_auth_s:       # 401 / SOURCETABLE 后的长退避
  standby:              # cold | hot  (U-RTCM-2)
  linger_s:             # 冷备：切回主源后保持连接的时长
  gga_period_s:
  gga_max_pos_age_s:    # rt/gnss/fix 位置超过此龄期视为无位置: 不建连 / 暂停上报
  vrs_rebase_km:        # RTG-12 当前位置离 VRS 锚点超过此距离即重连 (M-RTCM-4)
  # 没有任何站点坐标键: 备源不得依赖与地点有关的配置 (RTG-7)
health:
  obs_window_s:         # RTG-3
  station_window_s:     # RTG-3
selector:
  hold_up_s:            # RTG-5 主源连续有效多久才切回
status:
  period_s:
```

`configs/rtk_driver.yaml` 新增：

```yaml
rtcm:
  queue_max_bytes:      # RtcmSink 队列上限
  frame_max_age_s:      # 排队超过此时长的帧丢弃
```

口令不在任何 yaml 里（§4.4.3）。

### 4.11 部署

- 新增 `deploy/systemd/xbrain-rtcm-gateway.service`：`Restart=always`；`After=network-online.target` 与 `xbrain-config-freeze.service`；`LoadCredential=ntrip:...`；
  方案 A 时 `After=xbrain-zenohd-rt.service`；`CPUAffinity=0,4-6`，与其他自研进程一致。
- **不列入开机自检的一票否决项**：RTK 未固定属于运行期降级，不是启动失败（`10` BOOT-B2）。
- `xbrain-rtk-driver.service` 不变（已有 `SupplementaryGroups=dialout`，读写都够）。

### 4.12 故障模式与预期行为

| 故障 | 网关 | rtk_driver / 接收机 | 系统表现 |
|---|---|---|---|
| 树莓派掉电 / LAN 断 | SrcTcp 读超时 → 重连；主源窗口过期 → 切 BACKUP 或 NONE | NONE 时无新改正数，接收机按自身差分超时从固定退到浮点 / 单点 | `i_fix` 降级，限速或禁自主（`11` §3.2.1） |
| 基站 UART 断，树莓派仍在 | TCP 仍连着但无帧 → 主源失效 → 切源 | 同上 | 同上 |
| LAN 上有垃圾数据 | CRC 失败，丢弃；`crc_errors` 增长 | 不受影响 | 无 |
| 千寻 401 | `auth_failed`，长退避 | — | 仅备源不可用，状态如实上报 |
| 关机运到另一城市后开机 | 没有掉电前缓存坐标可用；等首个单点解后才连千寻，以新位置建 VRS | 先单点，收到改正数后收敛 | 与在原地开机一样，不需要改任何配置 |
| 开机行驶远离 VRS 锚点 | 超过 `vrs_rebase_km` 断开重连，以当前位置重建 VRS | 短时浮点，重新固定 | `i_fix` 自动限速 |
| 开机后尚无定位 / 定位丢失 | 需要备源时停在 `WAIT_POSITION`，不建连；已连则暂停 GGA | 单点或无解 | 禁自主，0.5 m/s 遥控 |
| 千寻账号不覆盖当前地点 | 连不上或连上无数据 ⇒ 备源无效 | 单点 | 同「两源都不可用」；状态里能看到 `ntrip` 的 `link` / `valid` |
| 两源都不可用 | NONE | 接收机退到单点 | 禁自主，0.5 m/s 遥控 |
| 切源 | 清空旧源帧 | 接收机基站变化，重新收敛 | 短时 `rtk_float`，`i_fix` 自动限速；若两源基准不一致坐标会跳变（§3.3） |
| `rtcm_gateway` 崩溃 | systemd 重启 | 无新改正数 | 同「两源都不可用」，`rt/clock/status` 不受影响 |
| 串口拔插 | — | 现有重开逻辑；清空 RtcmSink | 同现状 |
| 串口写堵塞 | — | 队列满丢最旧整帧；读和发布不受影响 | 改正数龄期变大 |

---

## 5. 测试计划

### 5.1 离线单测（CTest，无硬件）

| 测试 | 关键用例 |
|---|---|
| `test_rtcm3_framer` | 真实抓包回放（千寻账号到位后录制，存 `test/data/`）；帧被任意切成 1 字节 / 随机长度分块喂入；帧前后夹垃圾；CRC 错一位；伪 `0xD3`；L 超长；缓冲区溢出后重新同步 |
| `test_rtcm_source_health` | 只有观测无 1005/1006 ⇒ 无效；窗口边界（等于 / 超过）；基站 ID 变化计数 |
| `test_rtcm_selector` | 按时间脚本驱动全部转移；主源抖动时不回切；`hold_up_s` 内主源一拍无效即清零；切源时 flush 信号 |
| `test_ntrip_protocol` | v1 / v2 请求逐字节比对；`ICY 200` / `HTTP 200` / chunked / 401 / SOURCETABLE / 头超长 / 分包到达的响应头 |
| `test_gga_builder` | 校验和；南纬西经；度分格式边界（如 59.9999 分进位） |
| `test_vrs_anchor` | 无位置 / 位置过旧 ⇒ 不可报（且不存在任何替代坐标的代码路径）；上海 → 南京这类远距离位置 ⇒ 判重建；锚点附近小范围移动 ⇒ 不重建；阈值边界；跨经度 180° 与高纬度的距离计算 |
| `test_rtcm_sink` | 假写函数模拟部分写、EAGAIN、错误；半帧续写不被插队；超上限丢最旧整帧；过期帧丢弃；重开清空 |
| `test_nmea_parser`（扩充） | GGA 第 13 / 14 字段：有值、为空 |
| `test_rtcm_gateway_config` | 每个键缺失 / null 都抛并带键路径 |

每批按 `CLAUDE.md` 要求跑 lint、测试和变异体（`scripts/ci/cxx_mutants.py`），关键判定路径必须有能让变异体变红的断言。

### 5.2 ORIN 集成测试

- 用录制好的 RTCM 文件起一个假树莓派：`socat` 监听 TCP 端口回放文件，验证网关同步、选源、通道、`rtk_driver` 写串口计数。
- 拔掉假树莓派验证切到千寻、再恢复验证 `hold_up_s` 后切回。
- 断开 RT 路由 / 杀网关进程，验证 `rt/clock/status` 不受影响。

### 5.3 室外实测

- 千寻单源：从单点到浮点再到固定的收敛时间（⚠️ M-RTCM-3，与 `10` M-2 同批）。
- 自建基站单源：同上，以及树莓派转发延迟（⚠️ M-RTCM-2）。
- 主备切换：拔树莓派网线，记录切换时延、重新固定时间、切换前后静止位置差（检验 §3.3 基准一致性）。
- 跨地点：同一份配置、不改任何参数，在两个相距较远的地点（如上海、南京）分别开机，都能用千寻进入固定解；
  另测一次开机行驶超过 `vrs_rebase_km` 后的重建过程（⚠️ M-RTCM-4）。

---

## 6. 实施分批（一批一提交）

| 批 | 内容 | 前置 |
|---|---|---|
| 0 | **硬件与账号验证**：UM982 恢复上电；停掉 `rtk_driver`，用一次性脚本把千寻数据灌进 `ttyACM0`，室外看 GGA 质量能否到 4 | UM982 供电、千寻账号 |
| 1 | `rtk_driver`：GGA 第 13 / 14 字段解析，`age_s` 改差分龄期 | U-RTCM-3 |
| 2 | `rtk_driver`：串口读写 + `RtcmSink`（此时还没有输入） | — |
| 3 | 纯模块：`Rtcm3Framer` / `SourceHealth` / `SourceSelector` | — |
| 4 | 纯模块：`NtripProtocol` / `GgaBuilder` / `VrsAnchor` | — |
| 5 | `rtcm_gateway` 主程序、配置与冻结登记、RTCM 通道、`rtk_driver` 输入侧 | U-RTCM-1 / U-RTCM-2 |
| 6 | systemd 单元、凭据存放、状态发布；同批改 `10` / `11` 的登记项 | 批 5 |
| 7 | ORIN 集成测试（§5.2） | 批 6 |
| 8 | 室外实测（§5.3），实测值回填配置与 `11` 待确认项 | 树莓派、FDI 基站 |

---

## 7. 代码审查清单

- [ ] 串口只有 `rtk_driver` 打开；`rtcm_gateway` 不碰任何 tty。
- [ ] 写进串口的只有 CRC 通过的整帧；半帧余量写完前不开始下一帧。
- [ ] 任一时刻只输出一个源；切源时两端都清空旧源帧。
- [ ] RTCM 写路径不阻塞：非阻塞 fd、EAGAIN 即停、队列有上限。
- [ ] `rtk_driver` 的 20 Hz 发布节拍和 1 Hz 授时判定不受 RTCM 影响。
- [ ] 主源有效判据同时看观测电文和基站坐标电文，不只看字节或 TCP 状态。
- [ ] 回切需要主源连续有效满 `hold_up_s`；失效立即切走。
- [ ] Framer CRC 失败只丢 1 字节重新同步。
- [ ] 所有阈值、周期、地址来自配置，缺值即抛，代码里没有默认数值。
- [ ] 龄期和超时全用 `steady_clock`；墙钟出现处带 `WALL-CLOCK-OK` 标注（`clock_scan`）。
- [ ] 口令只出现在内存里的 Authorization 头；日志、状态、异常信息中都没有。
- [ ] 401 / SOURCETABLE 走长退避，不高频重试。
- [ ] GGA 上报位置只来自新鲜的 `rt/gnss/fix`；代码里不存在配置坐标或缓存坐标的替代路径；无位置时不建连 / 不上报。
- [ ] 离 VRS 锚点超过 `vrs_rebase_km` 会重连重建。
- [ ] DNS 解析不在主事件循环里执行。
- [ ] 日志、异常信息全英文；单文件注释语言统一。
- [ ] `age_s` 是差分龄期，新鲜度仍由 `t_mono` 判。

---

## 8. 待定项

全部登记在 `11` §3.2A.3，本文涉及：

| 编号 | 事项 | 影响本文 |
|---|---|---|
| U-RTCM-1 | RTCM 通道：RT 面键 / Unix socket | §4.7、§4.8.3、§4.9、§4.11 |
| U-RTCM-2 | 千寻冷备 / 热备 | §4.4.2、配置 `ntrip.standby` |
| U-RTCM-3 | 无差分时 `age_s` 取值 | §4.8.4 |
| U-RTCM-4 | 机器人哪个网口接客户 LAN | 部署 |
| V-RTCM-1 | FDI 基站型号与输出配置 | §3.2 |
| V-RTCM-2 | 千寻账号服务区域与坐标框架 | §4.4.2 |
| M-RTCM-1 | 有效窗口与回切时长 | §4.6、配置 |
| M-RTCM-2 | 树莓派转发延迟 | §5.3 |
| M-RTCM-3 | UM982 RTCM 输入、差分超时行为、收敛时间 | 批 0、§5.3 |
| M-RTCM-4 | VRS 重建距离阈值；千寻是否随 GGA 自动更新 VRS | §4.4.2、§5.3 |

---

## 9. 构建与测试（在 ORIN 上）

登录方式：从登录设备 `ssh nvidia@192.168.188.200`（主链路），不通时用 `ssh nvidia@192.168.1.26`（备用链路）。所有构建和运行都在机器人上做。

```sh
cd /opt/xbrain_v6/ros2_ws/sensor
cmake -B build && cmake --build build
ctest --test-dir build --output-on-failure
# 或用全仓 C++ 测试入口（处理个别测试需要的 cwd / argv / env）
/opt/xbrain_v6/scripts/run_cxx_tests.sh
```

- 现有 9 组离线测试：`test_nmea_parser` · `test_gnss_heading` · `test_gnss_fix` · `test_clock_status` · `test_heading_resolver` · `test_rtk_driver` · `test_yaml_lite` · `test_rtk_config` · `test_serial_reopen`。
- `rtk_driver` 可执行文件只在找到 `/usr/local/lib/libzenohc` 时构建；离线测试不依赖 zenoh。
- 安装：`cmake --install build --prefix /opt/xbrain_v6/data/install/rtk_driver`，产物在 `data/install/rtk_driver/lib/rtk_driver/rtk_driver`（DEC-15），由 `xbrain-rtk-driver.service` 启动。
- 运行需要环境变量 `XBRAIN_ROBOT_ID` 和冻结产物 `rtk_driver.yaml`（`deploy/env/robot.env`、`xbrain-config-freeze.service`）。
