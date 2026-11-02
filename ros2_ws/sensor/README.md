# sensor —— 机器人端 RTK 软件设计（rtk_driver，含 RTCM 差分输入）

> **本文件是机器人端 RTK 软件的设计正本**：`docs/` 下没有 RTK 专册，代码实现与代码审查以本文为准。
> 跨进程 / 跨设备的**契约**不在这里定义，只在 `docs/11-接口契约.md`：
> §3.2 `GnssFix` · **§3.2A RTCM 差分改正数输入（RTB / RTG 条款）** · §3.3 `GnssHeading` · §3.11 `ClockStatus` · §1.5 授时。
> 本文与 `11` 冲突时以 `11` 为准，并按 `CLAUDE.md` 铁律 1 回头修本文。

状态标记：✅ 已实现并实测 · 🚧 已定设计、待实现 · ⚠️ 待裁定 / 待实测（编号指向 `11` §3.2A.3）

> **2026-11-02 第二次裁定**（`11` §3.2A 文首 ④~⑦）：差分接入**并入 `rtk_driver` 单进程**，用独立网络线程隔离（RTG-1）；
> 千寻账号口令写进 `configs/rtk_driver.yaml`（RTG-9）；千寻冷备；无差分时 `age_s = null`。
> 本文此前版本里的独立进程 `rtcm_gateway` 及其进程间通道已全部作废。

---

## 0. 包边界

- **不是 ROS 2 节点**。`rtk_driver` 是非 ROS 的 C++ 进程，发布到 **RT 面 Zenoh**，用纯 CMake 构建（不链接 rclcpp / ament / colcon）。
  目录放在 `ros2_ws/` 下只是因为机器人侧 C++ 资产都在这里；「非 ROS」由 CMakeLists 不链接任何 ROS 目标保证。
- 只在 **ORIN（aarch64）** 上构建和运行，不要在 x86 主机上构建产物。
- 本包只有一个进程 `rtk_driver`。树莓派上的 `rtcm_bridge` 不在本包，见 §3.2。

---

## 1. 总体框架

```
  FDI base ──UART──> RPi rtcm_bridge (TCP server :PORT)            Qianxun caster
                              │ raw bytes 1:1, customer LAN              │ NTRIP v1/v2, 5G/WiFi
                              v                                          v
 ═════════════════════════════ ROBOT (ORIN) · process rtk_driver ═══════════════════════════
 ┌─ network thread (CorrectionLink) ─────────────────────────────────────────────── 🚧 ─┐
 │  ┌────────────────────┐              ┌────────────────────┐                          │
 │  │ LAN source (prim.) │              │ NTRIP source (bak.)│<── GGA <── VrsAnchor <─┐ │
 │  │ TCP client         │              │ cold standby       │                        │ │
 │  │ reconnect+backoff  │              │ DNS in helper thr. │                        │ │
 │  └─────────┬──────────┘              └─────────┬──────────┘                        │ │
 │            v                                   v                                   │ │
 │  ┌────────────────────┐              ┌────────────────────┐                        │ │
 │  │ Rtcm3Framer        │              │ Rtcm3Framer        │  0xD3 sync, CRC-24Q    │ │
 │  │ SourceHealth       │              │ SourceHealth       │  msg ids, frame age    │ │
 │  └─────────┬──────────┘              └─────────┬──────────┘                        │ │
 │            └──────────────┬────────────────────┘                                   │ │
 │                           v                                                        │ │
 │                 ┌──────────────────────┐  primary first; down fast, up slow        │ │
 │                 │ SourceSelector       │  one source at a time; flush on switch    │ │
 │                 └──────────┬───────────┘                                           │ │
 └────────────────────────────┼───────────────────────────────────────────────────────┼─┘
            out queue (mutex, bounded, whole frames + source tag)      position slot (mutex)
                              v                                                       ^
 ┌─ main thread, 20 Hz loop (existing) ───────────────────────────────────────────────┼─┐
 │  take frames (never waits) ──> RtcmSink: bounded, drop oldest WHOLE frame    🚧    │ │
 │                                   │                                                │ │
 │  serial fd /dev/ttyACM0  O_RDWR <─┘ non-blocking write, partial-write safe   🚧    │ │
 │      │ NMEA read                                                             ✅    │ │
 │      v                                                                             │ │
 │  RtkDriver: nmea_parser (+ GGA field 13 diff age 🚧) -> heading_resolver           │ │
 │      ├──> rt/gnss/fix   (age_s := diff age, null when none) ── current fix ───────┘ │
 │      ├──> rt/gnss/heading                                                           │
 │      └──> rt/clock/status (1 Hz)                                                    │
 └──────────────────────────────────────┬──────────────────────────────────────────────┘
                                        │ UART 115200 (CH343)   RTCM ↓   NMEA ↑
                                 ┌──────┴──────┐
                                 │    UM982    │
                                 └─────────────┘
```

### 1.1 为什么单进程也不会被网络拖垮（`11` RTG-1）

`rtk_driver` 是全系统唯一的授时同步判定者（CLK-A1），`rt/clock/status` 停发 5 s 全系统就 `ts_sync = false`（CLK-A5）。
合并进同一进程后，靠下面四条保证「网络出什么事最多拿不到改正数」：

| # | 规则 | 实现方式 |
|---|---|---|
| ① | 主循环里没有任何网络调用 | 所有 socket、`connect`、`send/recv`、DNS 都在 `CorrectionLink` 的网络线程；DNS 再放到一次性辅助线程 |
| ② | 主循环取帧不等待 | 进程内队列只用一把短临界区的互斥锁，主循环 `take` 一次性把帧换出来，不做条件变量等待 |
| ③ | 异常不出线程 | 网络线程循环体整体 `try/catch`，任何异常都记日志、关连接、退避后继续；`std::thread` 入口函数不让异常逃逸（逃逸会 `std::terminate` 整个进程） |
| ④ | 网络线程整体失效 = 没有改正数 | 主循环不依赖网络线程的任何状态才能发布；接收机自行退到浮点 / 单点，`11` §3.2.1 降级链照常生效 |

### 1.2 为什么串口只归主线程（`11` RTG-2）

UM982 只有一个 COM 口经 CH343 接到 ORIN，NMEA 读和 RTCM 写在同一个 fd 上。只有主线程碰这个 fd：
网络线程交出来的是内存里的整帧，写串口由主循环完成，这样也不需要给 fd 加锁。

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
5. 🚧 启动网络线程 `CorrectionLink`（两路都 `enabled: false` 时不启动线程）。
6. 进入 20 Hz 主循环，周期用 `steady_clock`；墙钟只用于信封 `ts` 和 GGA 的 UTC 字段。

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
| 1 | `single` | 单点 |
| 0 / 6 | `no_fix` | 无效 / 航位推算 |

quality 0 / 1 / 6 视作 `fix_is_lost`（不准 L2）。GGA 超过 `gga_timeout_s` 未更新时 `fix_type = no_fix`。
`cov_h_m = nominal(fix_type) × max(hdop, 1)`，各档 nominal 值在配置 `fix_cov.*`。

### 2.5 串口热插拔

`serial_reopen.h` 的纯函数 `classify_serial_read()` 判定每次 `read()` 的结果：有数据就喂给解析，EOF 或硬错误（ENODEV / EIO）就重开，
EAGAIN 但超过 3 s 无数据也重开（有的内核拔掉设备后一直返回 EAGAIN）。断开期间约 1 Hz 重试 `open()`，USB 插回后自动恢复。

### 2.6 已知与契约不符（随 RTCM 一并修）

| # | 现状 | 契约 | 处置 |
|---|---|---|---|
| 1 | `GnssFix.age_s` 填的是「距最近一条 GGA 的秒数」 | `11` §3.2：`age_s` = **差分龄期**，无差分为 `null` | 🚧 批 1 |
| 2 | 串口 `O_RDONLY`，没有写路径 | `11` RTG-2：主线程负责写 RTCM | 🚧 批 4 |

---

## 3. 外部差分源

### 3.1 两个源的角色

| | 主源：LAN 自建基站 | 备源：千寻 CORS |
|---|---|---|
| 用途 | 客户现场落地 | 调试；主源无数据时兜底 |
| 物理链路 | FDI 基站 UART → 树莓派 → 客户自组网 LAN → 机器人 | 公网（5G / WiFi）→ 机器人 |
| 协议 | 裸 TCP，RTCM3 原始字节 1:1（`11` RTB） | NTRIP v1 / v2，VRS |
| 上行 | 无（RTB-3：发什么都被丢弃） | 周期上报 GGA（VRS 必需） |
| 认证 | 无（靠网段绑定，RTB-9） | 账号口令，写在 `configs/rtk_driver.yaml`（RTG-9） |
| 机器人数量 | 一个基站服务 LAN 内所有机器人 | 每台机器人一个账号（千寻通常一号一机在线） |
| 连接时机 | 常连 | 冷备：主源无效时才连，切回主源后保持 `linger_s` 再断开 |
| 与地点的关系 | 每个现场一台基站，`lan.host/port` 随现场配置 | **不依赖任何与地点有关的配置**：GGA 只用机器人实测位置，走远了自动重建 VRS（RTG-7 / RTG-12）。同一份配置在上海、南京等任意地点可用 |

### 3.2 树莓派 `rtcm_bridge`（契约见 `11` §3.2A.1）

机器人端依赖它的以下行为，代码不得假设更多：

- 连上就收流，**没有握手、没有帧头、没有心跳**，从任意字节开始 ⇒ `Rtcm3Framer` 必须能从流中间同步（RTB-2）。
- UART 断了时树莓派**不断开 TCP**，只是没有数据 ⇒ 「连着但没数据」必须靠帧级龄期判失效，不能只看 TCP 状态（RTB-8）。
- 客户端积压太多会被树莓派主动断开 ⇒ 网络线程读取要及时，读路径上不做阻塞操作（RTB-4）。
- 实现位置不在本包（树莓派不是 ORIN）；建议放 `deploy/rtcm_bridge/`，待实现时定。

### 3.3 坐标基准（交付项，代码无法兜底）

自建基站必须以**固定坐标**模式工作，坐标与千寻同一基准（`11` RTG-8 / RTB-11）。否则主备切换时机器人坐标整体跳变，
在主源下录制的围栏、路线、关键点到了备源下全部错位。部署时用千寻 RTK 或测绘把基站天线坐标测准后写入基站。

### 3.4 千寻账号现状（2026-11-02）

- 账号已发放，App 显示**未激活**、包月、剩余 1 个月。第一次用账号口令连接很可能激活并开始计时，
  所以开发和单测阶段**不用真账号**，等 UM982 恢复上电、室外第一次实测时才连（用户 2026-11-02 裁定）。
- 源表实测（无需账号）：`rtk.ntrip.qxwz.com` 的 8001 / 8002 / 8003 都提供挂载点 `AUTO` · `RTCM32_GGB`（1074/1084/1124）· `RTCM30_GG` · `RTCM23_GPS`。
  三个端口对应不同坐标系，具体对应关系待用户从千寻 App 提供（⚠️ V-RTCM-2），在此之前配置里 `ntrip.enabled: false`。

---

## 4. RTCM 差分输入设计（🚧）

### 4.1 设计约束

| # | 约束 | 出处 |
|---|---|---|
| C1 | 任一时刻只把一个源的帧写进接收机 | RTG-4 |
| C2 | 只写 CRC 校验通过的**整帧**，绝不写半帧 | RTG-3 / RTG-6 |
| C3 | RTCM 写不得影响 NMEA 读和 20 Hz 发布节拍 | RTG-6 |
| C4 | 网络问题不得波及主循环与授时发布 | RTG-1，§1.1 |
| C5 | 所有阈值、周期、地址来自配置，代码无默认值 | `CLAUDE.md` 3.1 |
| C6 | 所有龄期 / 超时用单调钟；墙钟只用于信封 `ts` 和 GGA 的 UTC 字段 | CLK-C1 |
| C7 | 口令只在内存中拼 Authorization 头；不进 argv / environ / 日志 / 状态 / 异常信息 | RTG-9 |
| C8 | 不新增速度门：改正数中断由接收机自身退化，交给已有 `i_fix` 链路 | RTG-10 |
| C9 | 备源不依赖任何与地点有关的配置 | RTG-7 / RTG-12 |

### 4.2 模块划分

纯逻辑模块不做 I/O、不读时钟（时间由调用方传入），全部可离线单测；I/O 只在 `correction_link.cc` 和 `main.cc`。

| 模块 | 文件 | 职责 | 纯 / I/O | 运行线程 |
|---|---|---|---|---|
| `Rtcm3Framer` | `rtcm3_framer.{h,cc}` | 字节流 → 合法 RTCM3 帧；提取电文号与基站 ID | 纯 | 网络 |
| `SourceHealth` | `rtcm_source.{h,cc}` | 每个源的统计与「是否有效」判定 | 纯 | 网络 |
| `SourceSelector` | `rtcm_source.{h,cc}` | 主备状态机、冷备是否需要连备源 | 纯 | 网络 |
| `NtripProtocol` | `ntrip_protocol.{h,cc}` | 构造请求、解析响应头、chunked 解码、base64 | 纯 | 网络 |
| `GgaBuilder` / `VrsAnchor` | `gga_vrs.{h,cc}` | 合成 `$GPGGA`；位置是否可报、VRS 锚点与重建判定 | 纯 | 网络 |
| `RtcmSink` | `rtcm_sink.{h,cc}` | 主线程写队列：整帧排队、过期 / 溢出丢弃、半帧续写 | 纯（写函数注入） | 主 |
| `CorrectionLink` | `correction_link.{h,cc}` | 网络线程：两路连接、选源、与主线程的两个交换点 | I/O | 网络 |
| 配置 | `rtk_config.{h,cc}`（扩充） | 读 `rtcm.*` 段，缺值即抛 | 纯（读文件） | 启动 |

### 4.3 线程与交换点

```
 main thread                                  network thread (CorrectionLink::run)
 ───────────                                  ────────────────────────────────────
 each 50 ms tick:                             loop:
   link.setPosition(driver.currentFix())  ──>   position slot  (mutex, copy one struct)
   link.takeFrames(&batch)                <──   out queue      (mutex, deque swap)
   sink.push(batch); sink.drain(write_fd)       poll(lan_fd, ntrip_fd, 100 ms)
                                                framer -> health -> selector -> out queue
                                                DNS: detached helper thread + shared state
```

- 两个交换点各一把 `std::mutex`，临界区只做拷贝 / `swap`，主线程从不等待网络线程做任何 I/O。
- 出队列有字节上限（与 `RtcmSink` 共用 `rtcm.sink.queue_max_bytes`），满了丢最旧整帧；选源切换时清空。
- 每个帧带来源标签（`lan` / `ntrip`），`RtcmSink` 发现标签变化时清空自己的队列（双保险）。
- 网络线程的 `poll` 超时 100 ms，所以停止标志最多 100 ms 被看到；进程退出时 `stop()` 置标志并 `join`。
- DNS 用**分离的**辅助线程做 `getaddrinfo()`，结果写进 `shared_ptr` 共享状态；超时就放弃这次结果（辅助线程自己跑完退出）。
  不用 `std::async`：它返回的 future 析构时会阻塞等待，DNS 卡住就会把网络线程一起卡住。

### 4.4 源接入

#### 4.4.1 LAN 源（主源，裸 TCP）

```
 DISCONNECTED ──退避到期──> CONNECTING ──connect 成功──> STREAMING
      ^                         │                          │
      └──── 失败 / 超时 ────────┘                          │
      └──── EOF / 错误 / 读超时 ───────────────────────────┘
```

- `lan.host` 填 IP（树莓派静态 IP），用 `AI_NUMERICHOST` 解析，不走 DNS。
- 非阻塞 `connect()`，超时 `connect_timeout_s`；开 `SO_KEEPALIVE`（空闲 5 s · 间隔 2 s · 3 次，与 RTB-7 对齐）。
- 读超时：STREAMING 下超过 `read_timeout_s` 没收到任何字节就断开重连。这只是链路层兜底；「连着但没有合法帧」由 `SourceHealth` 判失效。
- 重连退避：从 `backoff_min_s` 起每次翻倍，封顶 `backoff_max_s`；进入 STREAMING 后重置。

#### 4.4.2 NTRIP 源（备源，千寻）

```
 IDLE ──需要备源──> WAIT_POSITION ──有新鲜位置──> RESOLVING ──> CONNECTING ──> REQUEST_SENT ──200──> STREAMING
   ^                                                                              │ 401 / SOURCETABLE
   └──── 不再需要备源（linger 到期）/ 错误 / 读超时 / 需要重建 VRS ───────────────┴──> 退避
```

请求（`ntrip.version` 选择；口令只在内存中拼 Basic 认证头）：

```
 v1:  GET /{mountpoint} HTTP/1.0\r\n
      User-Agent: NTRIP xbrain-rtk-driver/1.0\r\n
      Authorization: Basic base64(user:password)\r\n
      \r\n

 v2:  GET /{mountpoint} HTTP/1.1\r\n
      Host: {host}:{port}\r\n
      Ntrip-Version: Ntrip/2.0\r\n
      User-Agent: NTRIP xbrain-rtk-driver/1.0\r\n
      Authorization: Basic base64(user:password)\r\n
      Connection: close\r\n
      \r\n
```

响应处理：

| 响应首行 | 含义 | 处置 |
|---|---|---|
| `ICY 200 OK` | v1 成功 | 进入 STREAMING，后续字节全是 RTCM |
| `HTTP/1.x 200` | v2（或部分 v1 caster）成功 | 读完响应头；若 `Transfer-Encoding: chunked` 则经 chunked 解码后再喂 framer |
| `SOURCETABLE 200 OK` | 挂载点不存在，caster 返回源表 | 记 `bad_mountpoint`，按 `backoff_auth_s` 长退避 |
| `HTTP/1.x 401` | 认证失败 | 记 `auth_failed`，按 `backoff_auth_s` 长退避，避免频繁试错导致账号被封；日志只出现用户名 |
| 其他 / 头超过 4 KB / 超时 | 异常 | 断开，普通退避 |

GGA 上报与「任意地点可用」（`11` RTG-7 / RTG-12）：

千寻 VRS 按机器人上报的 GGA 位置就近生成虚拟基站。所以**上报的位置必须是机器人此刻真实所在的位置**，
否则在上海配好的东西拿到南京（约 270 km）就会生成错误位置的虚拟基站，基线过长，无法固定甚至给出错误解。规则如下：

- **位置只取实测**：主线程每拍把 `RtkDriver::currentFix()`（与发布到 `rt/gnss/fix` 的是同一份）写进位置槽；
  网络线程只认 `has_position == true` 且龄期不超过 `gga_max_pos_age_s` 的位置（单点解即可）。
- **没有任何替代坐标**：🚫 配置里的站点坐标，🚫 掉电前缓存的坐标（机器人可能是关机后被运到另一个城市的）。代码里不存在这条路径。
- **没有新鲜位置就等**：需要备源但没有新鲜位置时停在 `WAIT_POSITION` 不建连；已连接时位置过旧就暂停上报（不发旧位置），恢复后继续。
- **上报节奏**：进入 STREAMING 立即发一条 GGA，之后每 `gga_period_s` 发一条。GGA 的 UTC 时间字段取墙钟，属于对外报文时间而非龄期判断，标 `WALL-CLOCK-OK`。
- **VRS 锚点与重建**：首条上报 GGA 的位置记为锚点；当前位置与锚点的大圆距离超过 `vrs_rebase_km` ⇒ 断开重连，用当前位置重建 VRS（⚠️ M-RTCM-4）。
- **账号覆盖**：千寻账号可能限定服务区域，换城市测试前确认（⚠️ V-RTCM-2）。代码检测不了，只能从连不上 / 无数据看出来。

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
5. 成功：输出整帧（含头和 CRC，原样写给接收机），附电文号、基站 ID。
6. 缓冲区不需要单独的上限：扫描只会停在「还在等后续字节的帧头」上，一帧最多 1029 字节，所以留存的数据总小于 1029 字节。

观测电文：MSM1–7 的 1071–1077（GPS）· 1081–1087（GLONASS）· 1091–1097（Galileo）· 1101–1107（SBAS）· 1111–1117（QZSS）· 1121–1127（BDS），以及传统的 1001–1004（GPS）· 1009–1012（GLONASS）。
基站坐标电文：1005 / 1006。

### 4.6 SourceHealth 与 SourceSelector

#### 4.6.1 每源统计（SourceHealth）

| 字段 | 含义 |
|---|---|
| `bytes_rx` · `frames_ok` · `crc_errors` · `garbage_bytes` | 累计计数（前三项由 framer 给） |
| `last_obs_mono` | 最近一个观测电文的到达时刻 |
| `last_station_mono` | 最近一个基站坐标电文（1005/1006）的到达时刻 |
| `station_id` | 最近的基站 ID；变化时计 `station_id_changes` 并记日志 |

**有效判据**（RTG-3）：`now - last_obs_mono <= obs_window_s` 且 `now - last_station_mono <= station_window_s`。
连接断开时调用 `reset()` 清掉时刻，该源立即无效（不等窗口过期）。

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

| 当前 | 条件 | 下一状态 |
|---|---|---|
| NONE | 主源有效 | PRIMARY |
| NONE | 主源无效，备源有效 | BACKUP |
| PRIMARY | 主源无效（**立即**），备源有效 / 无效 | BACKUP / NONE |
| BACKUP | 主源**连续有效**满 `hold_up_s` | PRIMARY |
| BACKUP | 备源无效且主源无效 | NONE |

- 每次状态变化 `update()` 返回 `switched = true`，调用方清空出队列。
- 「连续有效」计时在主源任一拍无效时清零。从 NONE 直接到 PRIMARY 不需要保持（没有在用的源，越快越好）。
- **冷备连接判定** `wantBackup(now)`：主源已**连续无效满 `backup_after_s`**，或处于 BACKUP，或**从 BACKUP 回到** PRIMARY 不足 `linger_s` ⇒ 需要备源连接。
  `backup_after_s` 的理由：冷启动时主源在收到第一条 1005/1006（几秒一条）之前都是无效的，没有这个宽限，每次开机都会去拨千寻——而未激活账号的第一次拨号就会开始计费。
  从 NONE 直接进入 PRIMARY 不触发 `linger_s`（备源从来没用过）。
- NONE 状态不输出任何帧，接收机按自身差分超时退化。

### 4.7 RtcmSink（主线程写串口）

```
 takeFrames ──> push(bytes, tag, t_arrive) ──> deque  (total bytes <= queue_max_bytes)
                                                 │  over cap: drop oldest WHOLE frame
                                                 v
 each 50 ms tick ──> drain(write_fn, now):
     1) 先写完上一拍剩下的半帧余量
     2) 丢弃 now - t_arrive > frame_max_age_s 的帧（只丢还没开始写的整帧）
     3) 逐帧 write()，直到 EAGAIN 或队列空
        - 返回 < 剩余长度: 记偏移，下一拍从这里续写
        - EAGAIN: 停止，下一拍继续
        - 其他错误: 返回错误，由主循环交给现有重开逻辑
```

- 一个帧开始写之后必须先写完它，才能写下一帧——接收机看到的永远是完整帧序列（C2）。
- 写在主循环 `read()` 之后、`driver.tick()` 之前，不改变发布节拍（C3）。
- 写预算：115200 baud 8N1 约 11.5 KB/s，每 50 ms 约 576 字节；四系统 MSM4 改正数约 1–3 KB/s，余量足够。
- 串口重开时 `clear()`：接收机在断开期间已丢了半帧，旧帧也已过期。
- 计数 `frames_written` · `bytes_written` · `dropped_overflow` · `dropped_stale` · `dropped_switch`，随心跳日志打印。

### 4.8 `age_s` 改为差分龄期（批 1）

- `GgaFix` 增加 `has_diff_age` / `diff_age_s` / `diff_station_id`，解析 GGA 第 13 / 14 字段（`nmea_split_fields` 下标 13 / 14）。
- `GnssFix` 增加 `has_age`；`to_json_data()` 在 `has_age == false` 时输出 `"age_s":null`。
- `buildFix()`：GGA 新鲜且带差分龄期时 `has_age = true`，`age_s = diff_age_s`；否则 `null`。
- `fix_type` 判定、`fix_is_lost`、GGA 新鲜度判断都不变（新鲜度依旧看 `gga_mono_` / `t_mono`）。

### 4.9 可观测性

- 心跳日志（现有 2 s 一行）追加：当前源、两路链路状态与有效性、帧计数、CRC 错误、写串口计数与丢弃计数。
- 切源、连接建立 / 断开、401 / SOURCETABLE、VRS 重建各打印一条英文日志。口令永不出现在日志里。
- 是否把差分状态发到 RT 面 / HMI / 语音，属于后续需求（需要先在 `11` 加键和消费方），本期只有日志。

### 4.10 配置（`configs/rtk_driver.yaml` 新增 `rtcm:` 段）

数值是工程值，实测前逐条标注「待实测」（用户 2026-11-02 裁定，⚠️ M-RTCM-1 / M-RTCM-4）。

```yaml
rtcm:
  sink:
    queue_max_bytes:      # RtcmSink 与网络线程出队列的字节上限
    frame_max_age_s:      # 排队超过此时长的帧丢弃
  lan:
    enabled:              # false = 不使用主源
    host:                 # rtcm_bridge 静态 IP (只接受数字 IP)
    port:                 # RTB-1 固定端口
    connect_timeout_s:
    read_timeout_s:       # STREAMING 下无任何字节即断开重连
    backoff_min_s:
    backoff_max_s:
  ntrip:
    enabled:              # false = 不使用备源; 为 false 时下面各键允许为 null
    host:
    port:
    mountpoint:
    version:              # 1 | 2
    user:
    password:             # RTG-9: 写在这里是用户裁定的例外; 永不进日志
    connect_timeout_s:
    read_timeout_s:
    backoff_min_s:
    backoff_max_s:
    backoff_auth_s:       # 401 / SOURCETABLE 后的长退避
    linger_s:             # 冷备: 切回主源后保持连接的时长
    gga_period_s:
    gga_max_pos_age_s:    # 位置超过此龄期视为无位置: 不建连 / 暂停上报
    vrs_rebase_km:        # 当前位置离 VRS 锚点超过此距离即重连
  health:
    obs_window_s:         # RTG-3
    station_window_s:     # RTG-3, 要大于基站 1005/1006 的实际间隔
  selector:
    hold_up_s:            # RTG-5 主源连续有效多久才切回
    backup_after_s:       # 冷备: 主源连续无效多久才拨备源
```

- 两路都 `enabled: false` 时不启动网络线程，行为与改造前完全一致（只是串口变成读写打开）。
- 没有任何站点坐标键（C9）。

### 4.11 部署

- `xbrain-rtk-driver.service` 不需要改：已有 `SupplementaryGroups=dialout`（读写都够）、`Restart=always`。
  千寻走公网需要网络可用，但网络线程自己会退避重连，不需要 `After=network-online.target` 来保证启动顺序。
- RTK 未固定不列入开机自检一票否决（`10` BOOT-B2）。

### 4.12 故障模式与预期行为

| 故障 | 网络线程 | 主线程 / 接收机 | 系统表现 |
|---|---|---|---|
| 树莓派掉电 / LAN 断 | 读超时 → 重连；主源无效 → 切 BACKUP 或 NONE | NONE 时无新改正数，接收机从固定退到浮点 / 单点 | `i_fix` 降级，限速或禁自主（`11` §3.2.1） |
| 基站 UART 断，树莓派仍在 | TCP 仍连着但无帧 → 主源无效 → 切源 | 同上 | 同上 |
| LAN 上有垃圾数据 | CRC 失败，丢弃 | 不受影响 | 无 |
| 千寻 401 | `auth_failed`，长退避 | — | 仅备源不可用 |
| 两源都不可用 | NONE | 接收机退到单点 | 禁自主，0.5 m/s 遥控 |
| 切源 | 清空出队列 | `RtcmSink` 按标签清空；接收机重新收敛 | 短时 `rtk_float`；若两源基准不一致坐标会跳变（§3.3） |
| DNS 卡住 | 辅助线程卡住，网络线程超时放弃、退避 | 不受影响 | 仅备源暂不可用 |
| 网络线程抛异常 | 捕获、关连接、退避后继续 | 不受影响 | 短时无改正数 |
| 关机运到另一城市后开机 | 没有缓存坐标可用；等首个单点解后才连千寻，以新位置建 VRS | 先单点，收到改正数后收敛 | 与原地开机一样，不改配置 |
| 开机行驶远离 VRS 锚点 | 超过 `vrs_rebase_km` 断开重连重建 | 短时浮点，重新固定 | `i_fix` 自动限速 |
| 开机后尚无定位 / 定位丢失 | `WAIT_POSITION` 不建连；已连则暂停 GGA | 单点或无解 | 禁自主，0.5 m/s 遥控 |
| 千寻账号不覆盖当前地点 | 连不上或无数据 ⇒ 备源无效 | 单点 | 同「两源都不可用」 |
| 串口拔插 | — | 现有重开逻辑；`RtcmSink` 清空 | 同现状 |
| 串口写堵塞 | — | 队列满丢最旧整帧；读与发布不受影响 | 改正数龄期变大 |

---

## 5. 测试计划

### 5.1 离线单测（CTest，无硬件、无网络）

| 测试 | 关键用例 |
|---|---|
| `test_nmea_parser`（扩充） | GGA 第 13 / 14 字段：有值、为空、字段不足 |
| `test_gnss_fix`（扩充） | `has_age == false` ⇒ `"age_s":null`；有值 ⇒ 数字 |
| `test_rtk_driver`（扩充） | GGA 带 / 不带差分龄期时 `rt/gnss/fix` 的 `age_s` |
| `test_rtcm3_framer` | 用 CRC 自洽的构造帧；任意切块（含 1 字节）喂入；帧前后夹垃圾；CRC 错一位只丢 1 字节；伪 `0xD3`；保留位非 0；电文号与基站 ID 提取 |
| `test_rtcm_source` | 只有观测无 1005/1006 ⇒ 无效；窗口边界；`reset()` 立即无效；选择器全部转移；主源抖动不回切；`hold_up_s` 内一拍无效清零；冷备 `wantBackup`：`backup_after_s` 宽限、`linger_s` 只在从备源回来后生效 |
| `test_ntrip_protocol` | base64 标准向量；v1 / v2 请求逐字节比对；`ICY 200` / `HTTP 200` / chunked / 401 / SOURCETABLE / 头超长 / 分包到达；chunked 跨块拆分 |
| `test_gga_vrs` | GGA 校验和与度分格式（含 59.99995 分进位、南纬西经）；无位置 / 位置过旧 ⇒ 不可报；上海 → 南京 ⇒ 需重建；锚点附近 ⇒ 不重建；阈值边界 |
| `test_rtcm_sink` | 假写函数模拟部分写、EAGAIN、错误；半帧续写不被插队；超上限丢最旧整帧；过期整帧丢弃但写了一半的不丢；标签变化清空；`clear()` |
| `test_rtk_config`（扩充） | `rtcm.*` 每个必需键缺失 / null 都抛并带键路径；`ntrip.enabled: false` 时 ntrip 其余键可为 null |

变异体：`scripts/ci/cxx_mutants.py` 新增 `rtcm` 套件，覆盖每个关键判定（CRC 失败丢 1 字节、单源输出、快切慢回、半帧续写、无位置不上报、重建距离比较、`age_s` null）。

### 5.2 ORIN 集成测试（不需要千寻账号）

- 假树莓派：本机脚本监听 TCP，循环发送 CRC 自洽的构造帧（含 1005 + MSM）；配置 `lan.enabled: true` 指向它。
  验证：主源有效、写串口计数增长、读与发布节拍不变。
- 假千寻：本机脚本实现最小 NTRIP caster（校验 Basic 头、回 `ICY 200 OK`、收 GGA、发构造帧）；关掉假树莓派验证冷备连接与切换、
  恢复后 `hold_up_s` 切回、`linger_s` 后断开；回 401 验证长退避。
- 杀掉假源、制造 DNS 失败，验证 `rt/clock/status` 1 Hz 不断。

### 5.3 室外实测（需要 UM982 上电与真账号）

- 千寻单源：从单点到浮点再到固定的收敛时间（⚠️ M-RTCM-3，与 `10` M-2 同批）；顺便确认端口与坐标系（V-RTCM-2）。
- 自建基站单源：同上，以及树莓派转发延迟（⚠️ M-RTCM-2）。
- 主备切换：拔树莓派网线，记录切换时延、重新固定时间、切换前后静止位置差（检验 §3.3 基准一致性）。
- 跨地点：同一份配置在两个相距较远的地点分别开机都能进入固定解；再测一次开机行驶超过 `vrs_rebase_km` 的重建（⚠️ M-RTCM-4）。

---

## 6. 实施分批（一批一提交）

| 批 | 内容 | 状态 |
|---|---|---|
| 1 | GGA 第 13 / 14 字段解析，`age_s` 改差分龄期，无差分为 null | 🚧 |
| 2 | 纯模块：`Rtcm3Framer` / `SourceHealth` / `SourceSelector` | 🚧 |
| 3 | 纯模块：`NtripProtocol` / `GgaBuilder` / `VrsAnchor` | 🚧 |
| 4 | 串口读写打开 + `RtcmSink` | 🚧 |
| 5 | `CorrectionLink` 网络线程、`rtcm.*` 配置、主循环接线 | 🚧 |
| 6 | ORIN 集成测试（§5.2） | 🚧 |
| 7 | 室外实测（§5.3），实测值回填配置与 `11` 待确认项 | 需 UM982 上电、真账号、树莓派 |

---

## 7. 代码审查清单

- [ ] 串口 fd 只在主线程使用；网络线程不碰任何 tty。
- [ ] 主循环里没有任何网络调用、没有条件变量等待；两个交换点的临界区只做拷贝 / swap。
- [ ] 网络线程入口与循环体捕获全部异常，异常不会逃出线程。
- [ ] 写进串口的只有 CRC 通过的整帧；半帧余量写完前不开始下一帧。
- [ ] 任一时刻只输出一个源；切源时出队列与 `RtcmSink` 都清空旧源帧。
- [ ] 主源有效判据同时看观测电文和基站坐标电文，不只看字节或 TCP 状态。
- [ ] 回切需要主源连续有效满 `hold_up_s`；失效立即切走。
- [ ] Framer CRC 失败只丢 1 字节重新同步。
- [ ] GGA 上报位置只来自新鲜的 `currentFix()`；不存在配置坐标或缓存坐标的替代路径；无位置时不建连 / 不上报。
- [ ] 离 VRS 锚点超过 `vrs_rebase_km` 会重连重建。
- [ ] DNS 不在网络线程主循环里同步执行，也不用 `std::async`。
- [ ] 所有阈值、周期、地址来自配置，缺值即抛，代码里没有默认数值。
- [ ] 龄期和超时全用 `steady_clock`；墙钟出现处带 `WALL-CLOCK-OK` 标注（`clock_scan`）。
- [ ] 口令只出现在内存里的 Authorization 头；日志、异常信息中都没有。
- [ ] 401 / SOURCETABLE 走长退避，不高频重试。
- [ ] `age_s` 是差分龄期，无差分为 null；新鲜度仍由 `t_mono` 判。
- [ ] 日志、异常信息全英文；单文件注释语言统一。

---

## 8. 待定项

全部登记在 `11` §3.2A.3，本文还受以下几项影响：

| 编号 | 事项 | 影响本文 |
|---|---|---|
| U-RTCM-4 | 机器人哪个网口接客户 LAN | 部署 |
| V-RTCM-1 | FDI 基站型号与输出配置 | §3.2 |
| V-RTCM-2 | 千寻端口与坐标系对应、账号服务区域 | §3.4、配置 `ntrip.port` |
| M-RTCM-1 | 有效窗口与回切时长 | §4.6、配置 |
| M-RTCM-2 | 树莓派转发延迟 | §5.3 |
| M-RTCM-3 | UM982 RTCM 输入、差分超时行为、收敛时间 | §5.3 |
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
# 变异体
python3 /opt/xbrain_v6/scripts/ci/cxx_mutants.py rtcm
```

- `rtk_driver` 可执行文件只在找到 `/usr/local/lib/libzenohc` 时构建；离线测试不依赖 zenoh。
- 安装：`cmake --install build --prefix /opt/xbrain_v6/data/install/rtk_driver`，产物在 `data/install/rtk_driver/lib/rtk_driver/rtk_driver`（DEC-15），由 `xbrain-rtk-driver.service` 启动。
- 运行需要环境变量 `XBRAIN_ROBOT_ID` 和冻结产物 `rtk_driver.yaml`（`deploy/env/robot.env`、`xbrain-config-freeze.service`）。
