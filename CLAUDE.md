# XBRAIN_V6 项目编码规范

> 本文件由 Claude Code 在本项目工作时**自动加载**，作为强制约束。
> 违反任意一条 = PR 不合入。任何疑问先查 `docs/` 十一册，找不到才向用户提问。
>
> **来源**：通用编码规范继承自 `xbrain_v5/CLAUDE.md`（已验证仍适用的部分）；
> §3 与 §10 是 **V6 自己的铁律**，全部来自 2026-07/08 文档收口期实测出的缺陷，🚫 不是从 V5 抄的。
>
> **最后更新**：2026-08-03（十一册定稿当日）

---

## 🔴 铁律 0 · 对话输出语言

- Claude Code 面向用户的**所有对话输出 / 分析 / 回复 / 总结必须使用中文**，不得用英文回复。
- 注意区分：这条只管**对话输出**。代码注释按 §2.1（单文件统一，中英二选一），而**日志 / print / 异常 message 仍必须全英文**。
- 即便上下文里出现英文，回复用户时也必须切回中文。

---

## 0. 项目形态（写代码前必须知道的）

**XBRAIN_V6** = 装在**云深处 山猫 M20S 四足底盘**上的「智能大脑上装」控制系统。
部署场景：**消防场区巡检 / 值守**（非军工）。

### 0.1 进程清单

★★★ **全系统 15 个常驻进程**（`10` §3.1）。**Zenoh 有【两个】物理隔离的 router**，🚫 不是一条总线。

| 进程 | 语言 | 面 | 说明 |
|---|---|---|---|
| `zenohd-gen` | Rust | ① router | 通用面 router，`tcp 7447`；★★ **逐口显式绑定，禁 `0.0.0.0`**（NET-C9） |
| `zenohd-rt` | Rust | ② router | RT 面 router，`lo:7449`，组播与 gossip 全关；★★★ **在 20 Hz 关键路径上**（星型拓扑每跳穿两次 router） |
| `p2_core` | Python | ① | 七域仲裁 · 模式状态机 · 健康度 · PTZ boost；★ **BIT 是它的一个线程，不是进程**（U48） |
| `p3_task` | Python | ① | 任务与充电；★ `task.db` / `fence.db` / `geo.db` **三库**的唯一写者 |
| `p4_agent` | Python | ① | 意图管线 128 意图 · AI 网关 |
| `p5_gateway` | Python | ① | 网关 · 事件管线 · `state/link` **唯一发布者**；★★ **HMI 后端在它进程内，系统无独立 HMI 进程** |
| `p1_motion` | Python | ★ **跨面 ①+②** | 20 Hz · **唯一速度出口**；RNS 是它的**进程内模块**，对外只呈现为行为源 `rns_avoid`(900) |
| `perception` | C++ | ★ **跨面 ①+②** | 33 ms / 30 fps；⚠️ **详细设计尚未编写** |
| `chassis_relay` | C++ | ★ **跨面 ①+②** | ★★ **在急停链路上**；CRL-1 只搬运不判断 · CRL-4 无动态分配无阻塞日志 · CRL-5 单跳 < 200 µs |
| `quadruped` | **C++17** | ② + ③ 底盘面 | 三通道双域（见 §5.4）；★★★ **RT-C4 明令禁止持通用面 session** |
| `rtk_driver` | ⚠️ 待定（建议 C++） | ② | ★★ 全系统**唯一有权判定 `ClockStatus.sync`** 的进程（CLK-A1） |
| `teleop_input` | ⚠️ 建议 C++（未定） | ② **pub-only** | ★ **不持通用面 session ⇒ 不是跨面点**；遥控 `deadman` 的时基持有者 |
| `behavior_proxy` | C++ | ② | Nav2 Action ↔ Zenoh 翻译；不持任何通用面 session |
| `Nav2 behavior_server` | C++ | ROS 2 域 | ★ **仅 spin / backup / wait 三个行为** |
| `zenoh-bridge-ros2dds` | Rust | ① ↔ ROS 2 | ⚠️ GATE-5 必须先删 V5 遗留的 `zenoh_bridge.json5`；★ 它**不是云端桥** |
| `payload-service` | Python | — | GZH-2 三合一（音频 8519 / 灯光 8529 / 载荷 18080） |

★ 另有两个 `Type=oneshot` 单元：`xbrain-probe.service`（Stage 0 · GATE-6）· `xbrain-config-freeze.service`（Stage 0c · 断言 J~K）。

★★★ **跨面点只有三个**（`p1_motion` · `perception` · `chassis_relay`），受 `11` §1.1.6 白名单**逐条**管控。🚫 新增跨面订阅必须先进白名单。
★★ **RT 面封闭集 = 9 个进程**（`11` **R-3**，原写 6 已作废）。
★★★ **Stage 4（放行）的执行者是 `p2_core`，不是 systemd。**

### 0.2 ★★★ 目录约定（用户 2026-08-05 定死，🚫 不得乱放）

```
/opt/xbrain_v6/
├── common/           # ★★★ 公共资产 —— 全局【任何模块】依赖的库与头文件
│                     #    错误码 · 闭集常量 · 配置加载器。跨语言（Python + C++）
│                     #    ★ 消费者含 chassis_relay(急停链路) ⇒ 🚫 绝不引 rclcpp 或任何 ROS 类型
├── configs/          # ★★★ 唯一配置根（绝对路径 · 复数 · 不接受符号链接）
│                     #    V6 系统【所有模块】的配置文件都在这里
│                     #    ★ 运行期读【解析产物】/run/xbrain/resolved/，🚫 不读源
├── data/             # ★★★ 所有编译数据 · 依赖数据 · 系统全部日志文件
├── deploy/           # ★ AI 服务默认开机启动的 systemd 脚本
├── docs/             # 系统相关的所有文档
│                     # 🚫🚫🚫 【用户 2026-08-05 明令】_plan/ 与 _archive/ 等下划线目录
│                     #    【已被有意删除，就是不让参考】。所有文档一律不得引用、
│                     #    不得读取这些文件，也不得从归档包里恢复它们。
│                     # ★★★ 十四册是【唯一】权威，任何结论必须能在它们内部自证。
├── ros2_ws/          # ★ 机器人底盘 · 传感器相关的 ROS2 节点资产（quadruped 等）
├── scripts/          # ★ 除 AI 服务外的其他启动脚本与测试脚本
│                     #    ★ 系统开发完毕后的【全栈启动脚本】也部署在这里
├── services/         # ★ AI 服务相关资产（asr · llm · payload · perception）
├── tests/            # ★ 所有测试资产 —— 单元测试 ＋ 集成测试
└── xbrain/           # ★★★ XBRAIN runtime 资产（P1~P5 Python 实现）
```

★★★ **代码必须严格按本表部署，🚫 不得乱放。** 放错目录 = PR 拒绝合入。


---

## 1. 设计文档权威

- 所有代码必须严格遵守 `docs/` 下十一册：
  `00` 应用需求 · `10` 顶层设计 · `11` 接口契约（25000+ 行，**契约唯一真源**）· `12` P1 运动域 ·
  `13` quadruped 与 Tier1 · `14` P2 仲裁与模式 · `15` P3 任务与充电 · `16` P4 管线 ·
  `17` P5 网关与 HMI · `18` 语音文本指令集 · `99` 决策记录（**裁决与编号台账**）
- ⚠️ `docs/_archive/01` 与 `02` **已作废**（早期基于错误假设编写，内容与现行架构相反），🚫 不得引用。
- **遇设计冲突或缺失：停下找用户讨论，🚫 严禁擅自调整设计。**
- 代码注释必须**反向链接节号**（例 `# 见 11 §9.6.2 速度门四段定标`）。
- 任何修改设计文档的提交必须同步更新代码，反之亦然。

### 1.1 跨文件引用格式（NUM-4）

- 引用别册**只写「册号 ＋ 章节号 ＋ 可 grep 的逐字锚点」**，🚫 **不写行号**。
- 理由：文档持续增删，行号必然漂移。实测某轮里三个行号引用**全部漂到空行或无关表格**，而引用方还以为自己核对过。

---

## 2. 通用编码规范（适用所有语言）

### 2.1 注释语言

- 单文件内**所有注释必须统一语言**（全中文或全英文，二选一，🚫 不允许混用）。
- **日志 / 打印输出 / 异常 message 必须全英文**，🚫 严禁中文。

```python
# OK
# 计算走廊内最近障碍距离
log.info("compute corridor min distance, bands=%d", len(bands))

# NOT OK
log.info("计算走廊内最近障碍距离, 数量=%d", len(bands))   # X 中文日志
```

### 2.2 字符集（★ 只管源代码，不管 markdown 文档）

- **源代码内严禁 emoji 与特殊符号**（✅ ❌ ⚠️ 🔴 → ★ ☑ ☐ 等）。
- 所有标点必须是 **ASCII 英文标点**，🚫 严禁中文标点 `，。；：？！""''（）【】《》、—…`。
- ★ **例外**：`docs/` 下的 markdown 文档不受此限（本项目文档大量使用 ★ / ⚠️ / 🚫 作为分级标记，这是有意的）。

### 2.3 命名规范

- 标识符**统一 snake_case**：`compute_speed_gate`、`task_id`、`rot_clearance_check`
- 🚫 **严禁驼峰** `computeSpeedGate` / `taskId`
- **例外**：类名 / 构造 / 析构 / 第三方库约定

```cpp
class ChassisApduCodec {            // 类名 PascalCase
 public:
  ChassisApduCodec(...);
  ~ChassisApduCodec();
 private:
  int frame_seq_;                   // 字段 snake_case + 尾下划线
};
```

```python
class TasksDAO:                     # 类名 PascalCase
    def get_step_retry_count(...):  # 方法 snake_case
        ...
```

- 常量：大写 + 下划线 `MAX_RETRY_COUNT = 3` / `TIER1_TIMEOUT_MS = 200`

### 2.4 注释覆盖率

- 注释行 / (代码行 + 注释行) **≥ 25%**（sanity 门禁）。
- ★★ **注释覆盖率不是目的，是手段。** 真正门禁：任何「无注释的代码块」/「代码段不解释 why」都视为不良资产，PR 不合入。
- 注释解释**为什么**这样写，不只是**做什么**（代码说明 what，注释说明 why）。
- 复杂算法必须有伪代码 / 数学公式 / 状态机说明。
- 任何「修复某个 bug」的代码必须注释指出**原 bug 描述 + 设计文档节号**。

### 2.5 头部注释模板

每个源文件必须有头部注释，五字段齐全（Copyright / Author / File / Brief / Description）。

**Python**：
```python
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: speed_gate.py
Brief: P1 speed gate, four-band f(d_free) with hysteresis

Description:
Implements the single authoritative speed gate formula from 10 S6.2.
Four bands are left-closed right-open; hysteresis requires 3.5m sustained
for 3.0s before upgrading to patrol. All limit params are injected at
construction time, never defaulted in code (see CLAUDE.md 3.1).
"""
```

**C/C++**：
```cpp
/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: apdu_codec.cc
 * Brief: M20S channel-1 APDU/ASDU codec
 *
 * Description:
 * Encodes/decodes 16-byte APDU header plus ASDU JSON payload.
 * ASDU root object MUST be wrapped in {"PatrolDevice":{...}} and MUST
 * carry a "Time" field, otherwise chassis returns 0xE002. This is NOT
 * documented in 13 S2.2 -- see the vendor PDF directly.
 */
```

**Shell / Bash**：
```bash
#!/usr/bin/env bash
#
# Copyright (c) 2026 Hachist Robotics
# Author: wanglei@hachist.com
# 上海哈船智能船舶技术有限公司
# File: start_all.sh
# Brief: Launch XBRAIN_V6 processes in staged order
#
# Description:
# Stage 0/0z/0c/1/2/3/4/5 startup ordering per 10 S3.3.
# All units Requires=xbrain-config-freeze.service, except p5_gateway
# minimal mode (observation window W-1).
#
```

---

## 3. ★★★ V6 专属铁律（本项目实测出的失效模式，每条都有血）

> 这一节是 V6 与 V5 差别最大的地方。**下面每一条都不是理论担忧，是文档收口期实测抓到的真缺陷。**

### 3.1 ★★★ 安全参数不写默认兜底

**规则**：`common.spec.*` / `common.safety.*` 这类安全参数，**代码里必须没有默认值**。

🚫 **禁止的三种写法**：
```python
@dataclass
class Limits:
    max_decel_mps2: float = 2.5        # X dataclass 默认值

a = cfg.get("brake.a_mps2", 2.5)       # X dict.get 兜底
a = cfg.brake.a_mps2 or 2.5            # X or 兜底
```

✅ **正确**：由构造期注入，缺失即抛。未标定的值在配置里写 **`null`**（启动断言 A 会报出键路径）。

★★★ **🚫 绝不写 `0.0` 冒充已赋值** —— `0.0` 会被判为「已赋值」而放行，运行期 `v_max = min(..., 0) = 0`，
**机器人不动且无任何报错**。这是 **fail-silent**，比 fail-safe 差一个量级。

★ 参照 `configs/` 现有骨架：19 个 yaml 里 18 个是**纯注释 + TODO**，头注逐字写着
「本文件在落值前会让整栈按设计拒绝启动，**这是预期行为**」。**保持这个形态，不要为了让它跑起来而填数。**

### 3.2 ★★★ 能力不足时不假装有保证

本项目已实测出 **≥10 次**这个失效模式，它有七种形态。**写每一段代码时自问一遍**：

| 形态 | 实例 | 识别法 |
|---|---|---|
| **一条永远绿的断言** | 「不发 uplink 时码率恒 6144」—— 硬编码 6144 的空壳实现同样通过 | 有没有一个「什么都不做」的实现能通过它？ |
| **一条永远红的断言** | 判据扫描面过宽，每次都红 ⇒ 最终被人改成「包含即可」⇒ 变成永远绿 | 它今天跑是不是红的？红了有没有期限？ |
| **判据自伤** | 判据句自身含它要 grep 的字串，命中数**永不可能为 0** | 判据句在不在自己的扫描面内？ |
| **拿未兑现的逐字当证据** | 「§3.1 现行逐字为 X」而 X 只存在于未合入的补丁块里 | 引用的那句话，现在真的在正文里吗？ |
| **只跑 diff 不跑全文的 lint** | selfcheck 声称「无未转义竖线 ✅」，只跑了新增的 13 行，漏掉靶心 | lint 跑的是 diff 还是全文？ |
| **扫描面不声明** | 「在用未登记码 = 0」，而扫描范围只含 7 册不含另外 4 册 | 这个数是在什么范围上跑出来的？ |
| **定义式冒充实测结论** | 「零填充判别式全库 0 例外」—— 它把结论定义进了前提，**按构造不可能有反例** | 这个规则可证伪吗？能不能构造一个反例？ |

### 3.3 ★★★ 每条断言必须红过一次才算写完

验收一条断言的**唯一**方式是**注入一个必然违反它的变异体，看它变红**。只写正例的断言 = 没写。

★ 实例：PTZ boost 的三条负向断言（不发 `uplink` ⇒ 码率恒 6144）**全部通过**一个把码率硬编码 6144、
删掉整个状态机的空壳实现。补了一条正向断言（注入 `boost_allow=true` ⇒ 码率必须在 2000ms 内变 16384）才抓得住。

### 3.4 ★★★ 单调钟（CLK-C1 ~ CLK-C6，`11` §1.5）

**一切超时 / 周期 / 年龄判定，一律用单调钟。**

| 语言 | ✅ 用 | 🚫 禁 |
|---|---|---|
| Python | `time.monotonic()` | `time.time()` · `datetime.now()` · `datetime.utcnow()` |
| C++ | `std::chrono::steady_clock` | `std::chrono::system_clock` · `CLOCK_REALTIME` |
| ROS 2 | `rclcpp::Clock(RCL_STEADY_TIME)` | **裸 `rclcpp::Clock()`**（默认构造是墙钟） |

- 墙钟（`ts`）只做三件事：**跨机对齐 · 录包 · 延迟统计**。
- ★ 这条**可自动化检查**，CI 静态扫描 `p1_motion` / `quadruped` / `perception` / `rtk_driver`，命中即失败。

### 3.5 ★★★ 闭集与错误码

- **错误码是闭集**，定义在 `11` §13.4~§13.15（A~L 十二组）。★★ **🚫 本文不写码数** —— 那是判定量，会腐烂（§3.7）；现数由 `scripts/doccheck/sec12_scan.py` 从表体读出并打印，2026-08-05 起 `L` 组新增 `E_STORAGE_CORRUPT`，四要素齐全（码名 · 含义 · 可重试性 · `detail` 必填项）。
- 🚫 **不得自造码**。`E_*` 常量由 `common/errors/` **共享库导出**，🚫 字符串硬编码。
- **闭集常量同样由共享库导出**（`domain` 7 值 · `plane` 8 值 · `Event.category` 23 值 · `gate.limiter` 14 值 ·
  `stop_reason` 8 值 · `TaskState` 12 值 等），🚫 字符串硬编码。
- ★★ **闭集外的值必须抛，🚫 不得静默透传、🚫 不得「未知值降级解释」**。

★ **一处容易搞反的**：`11` §8.13.5 标题逐字是「**错误映射（网关唯一实现点）**」，表的左列是 **HTTP status**。
⇒ **AI 服务侧返回自由文本 `detail` 不是违约** —— 映射到闭集码是**网关**的活。🚫 不要跑去改服务代码往里塞 `E_*`。

### 3.6 ★★★ 配置

- **唯一根**：`/opt/xbrain_v6/configs/`（**绝对路径 · 复数 · 不接受符号链接**，`00` **CFG-03**，设计侧 `10` §5.4.0 `CFG-ROOT-1`）
- **解析产物**：`/run/xbrain/resolved/{proc}.yaml` + `MANIFEST.json`（**tmpfs，不在 configs 下**）
- ★★★ **各进程运行期读【产物】，🚫 不读源** —— 引用轴在冻结线**一次性展开**，
  「引用不由各进程各自展开」是 `10` §5.4.1 称为「**最关键的一条结构性决定**」的东西，理由是「各进程解析出不同结果」。
- 🚫 **绝不提供任何「跳过安全断言」的开关** —— 那等于一条远程解除全部安全约束的通道。
- 🚫 **不引入「断言分级」**（把安全断言降为 warn）—— 那是 fail-safe 退化成 fail-silent 的标准路径。

### 3.7 ★★ 判定量不写进代码注释与文档

任何「实测出来的数」（扫描命中数 / 覆盖率 / 断言通过数）**不要抄进 markdown 或注释**。
落到 `scripts/` 下的脚本，只留**判据 ＋ 脚本路径 ＋ 上次跑的时间戳**。

★ 理由（本项目实测）：文档收口期有**七处**求值清单靠人保持同源，两次「计数订正」**本身都漏计了一处**。
只要数字是人抄进去的，它就会过期。

---

## 4. Python 专项

### 4.1 硬约束

| 铁律 | 出处 |
|---|---|
| `xbrain/` 内**严禁 `import rclpy`**（AI 软件栈零 rclpy 依赖，ROS 只在 `quadruped` 一侧） | `10` §3.1 |
| **持久化层强制 `aiosqlite`，严禁同步 `sqlite3`**（防 asyncio 事件循环阻塞） | `15` §9 |
| `persistence/` 之外**严禁 `import sqlite3` / `import aiosqlite`**（业务只走 DAO） | `15` §9 |
| **严禁 `datetime.now()` 无参 / `time.time()`** 做年龄与超时（见 §3.4） | `11` CLK-C1 |
| **严禁业务模块 `import requests` 直调 AI 服务**（走 `ai_client/*_client.py`） | `16` §14 |

### 4.2 Zenoh 跨线程

- Zenoh subscriber callback 在 **Rust 线程池**跑，callback 内**严禁**：
  `asyncio.create_task(...)` · `asyncio.Queue.put_nowait(...)` · 任何 `await` · 直接 `event_bus.publish(...)`
- 必须用 `event_bus.publish_threadsafe(...)` 或 `loop.call_soon_threadsafe(...)`

### 4.3 ★★ Zenoh Subscriber 强引用（zenoh-python 头号陷阱）

`declare_subscriber(...)` 的返回值**必须接住到 long-lived 容器**，否则 Python GC 回收 ⇒ Rust 端订阅**悄悄注销** ⇒ 静默失败。

```python
# OK
self._subs = SubscriberRegistry()
self._subs.declare(session, "rt/perception/targets", self._on_targets)

# NOT OK
session.declare_subscriber(...)              # X 裸调用
_ = declare_subscriber(...)                  # X 显式丢弃
def foo(): sub = declare_subscriber(...)     # X 局部变量出作用域
```

### 4.4 实时性

- **P1 控制循环 20 Hz**，周期 **P99 ≤ 60 ms · max ≤ 100 ms**。
- 循环内注入 `raise` ⇒ **本拍零速 + 落 fault + 下一拍循环仍在跑**（🚫 不许整个循环挂掉）。
- 🚫 循环内不得有阻塞 I/O。`json.dump` / `open(..., "w")` 必须包 `asyncio.to_thread(...)` + `asyncio.wait_for(...)`。

### 4.5 类型与异常

- 必须用 type hints：`def compute(self, d_free: float) -> Mps:`
- DTO 用 `pydantic.BaseModel` 或 `dataclass(frozen=True)`
- 异常类自定义放 `common/errors/exceptions.py`，🚫 严禁滥用 `Exception` 基类
- 🚫 **严禁裸 `except:`**（必须指定异常类型）
- ★ 建议给量纲开不同类型：`Mps`（速度）与 `Factor`（无量纲系数）**必须是两个类型** ——
  把 `Factor` 传进 `min()` 应当是**类型错误**，不能靠代码评审拦。

---

## 5. C / C++ 专项

### 5.1 Google C++ Style Guide

- 严格遵守 https://google.github.io/styleguide/cppguide.html
- 类成员尾下划线 `int frame_seq_;` · 头文件 include guard `#ifndef HACHIST_XBRAIN_V6_..._H_`
- 函数 < 40 行 / 类 < 200 行（超出必须拆分）
- `nullptr` 不用 `NULL` / `0` · `auto` 慎用，优先显式类型
- 🚫 严禁 `using namespace std;` 在头文件

### 5.2 ★★★ C++ 版本恰为 C++17（`13` **CPP-1** / **PB-5**）

```cmake
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)      # 防 GNU 扩展悄悄引入 C++20 特性
```

- 🚫 **严禁 C++20 / C++23 特性**：`concepts` · `coroutines` · `<format>` · three-way comparison 等
- ★★★ **🚫 不写任何发行版判断宏**（`PB-5`）—— 平台基线 `D-45`（humble/22.04 vs Jazzy/24.04）**尚未拍板**，
  写了判断宏，裁决一下来就要全推倒。★ 与 rclcpp 的耦合面**限定在三处**：创建 context / 发 `Odometry` / 发 `tf`
  （这三处 API 在 humble 与 jazzy 一致）。

### 5.3 ★★★ 地基库一个 ROS 类型都不引

`common/` 下的共享库（错误码 / 闭集常量 / 配置加载器）**绝不能依赖 rclcpp 或任何 ROS 类型**。

★ 理由：它的消费者里有 **`chassis_relay`**（★★ **在急停链路上**，`CRL-4` 要求**无动态分配、无阻塞日志**、`CRL-5` 单跳 < 200 µs）
以及 `rtk_driver` / `teleop_input`（⚠️ 两者语言在 `11` §1.1.3 / §13 仍写「**建议 C++ / 待定**」，🚫 不要当成已定）。
引了 rclcpp，这几个进程就没法用。

### 5.4 ★★★ quadruped 双域隔离（`13` §2.4 `DDS-1` ~ `DDS-9`）

**单个 C++17 进程**同时持有三条链路，**零 Docker**：

| 通道 | 技术 | 对谁 |
|---|---|---|
| 通道一 | POSIX socket，UDP/DTLS:30004 + TCP/TLS:30003，APDU/ASDU JSON | 底盘**全部控制 + 状态 + 故障**（与 ROS2 无关） |
| 通道二 | **裸 `dds_create_participant(0, ...)`**（FastDDS 域 0） | 底盘 `/IMU` 200Hz + `/LIDAR/POINTS` 10Hz |
| 通道三 | `rclcpp`（humble / CycloneDDS / **域 42**） | 朝上装 |

★★ 关键：**`rclcpp` 的域限制是 context 级，裸 participant 不受约束** ⇒ 一个进程可以同时在两个域上。

⚠️ **名字映射是最易踩的坑**：ROS 2 topic `/IMU` 在 DDS 层的 topic 名是 `"rt/IMU"`、type 名是
`sensor_msgs::msg::dds_::Imu_`。`13` **全册未写**。写错的现象是「participant 起来了、一个包收不到」，
**与网络不通不可区分**。

### 5.5 ★★★ CHS-A 协议：必须直接读厂商 PDF

⚠️ **`13` §2.2 的逐字节偏移表【不完整】**，缺三件，缺任一**第一帧就被底盘回 `0xE002`**：

1. ASDU 根对象必须包在 `{"PatrolDevice": {...}}` 里
2. 必填 `Time` 字段（本地时区 `"YYYY-MM-DD HH:MM:SS"`）
3. hex32 码在 JSON 里序列化为**十进制整数**（`0x00100064` → `1048676`；JSON 无十六进制字面量）

⇒ **必须直接读 `docs/QUADRUPED/软件开发指南.pdf`**（`pdftotext -layout` 可提取），🚫 不能只读 `13`。

### 5.6 编译要求

- 必须启用 `-Wall -Wextra -Werror -Wpedantic`
- TensorRT / CUDA 代码的 `-Wno-deprecated-declarations` **仅限**涉及厂商头文件的转译单元
- 必须支持 `-D_GLIBCXX_ASSERTIONS` debug 构建

### 5.7 ROS2 节点纪律

- 节点构造函数内**不允许做长时间初始化**（阻塞 lifecycle 切换），长初始化放 `on_configure()`
- 所有 publisher / subscriber 必须在初始化阶段**一次性声明**，🚫 不允许运行时动态创建

---

## 6. Shell / Bash 专项

- 文件开头必须 `#!/usr/bin/env bash` + `set -euo pipefail`
- 🚫 严禁绝对路径硬编码；用 `SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"` 派生
- 所有 `rm -rf` 必须显式列路径，🚫 严禁 `rm -rf $VAR/`（变量为空时会删 `/`）

---

## 7. 测试

### 7.1 分层

| 层 | 工具 | 测什么 |
|---|---|---|
| 单测 | `pytest` + `pytest-asyncio` | 纯函数：`f()` 四段 · CRC · 编解码 · 协方差 · limiter 归因 · 状态机 |
| 属性测试 | `hypothesis` | 仲裁不变量 · 速度门单调性 · 走廊几何保守性引理 |
| 金标集 | 固定向量文件 | `common_digest` · CHS-A 报文 hexdump · 43 故障码映射 · 128 意图 |
| 注入测试 | pytest fixture | 见 §3.3 —— **本项目最有价值的一类** |
| 元测试 | pytest | 断言注册表完备性 · 闭集双向差集 |
| 集成 | 桩 / 真机 | `fake_chassis` + `perception_stub` |

### 7.2 硬要求

- DAO 层单测必须用 in-memory SQLite（`aiosqlite.connect(":memory:")`）
- 关键状态机（P2 模式机 / P3 任务机 / Tier1）必须有**完整状态转换覆盖**
- **每个 bug 修复必须配测试用例**
- ★★★ **每条断言必须有一个会让它变红的变异体测试**（§3.3）

### 7.3 ⚠️ 环境前置（2026-08-03 实测）

本机缺 `fastapi` / `sherpa_onnx` / `opuslib` ⇒ **20 个 `test_*.py` 里 7 个 collect 失败**。
跑通的 13 个：**195 passed / 6 skipped / 2.47s**。

```bash
pip install fastapi uvicorn sherpa-onnx opuslib
```

⚠️★★★ **集成测试清单目前【没有落点】** —— 原指向 `_plan/` 下的文件，
而 🚫 **该目录已按用户明令永久移除、不得引用**（见 §0.2）。
⇒ 三档划分（不需要真设备 / 需要 ORIN / 需要真底盘）**须在开工时于 `docs/` 十一册内重建**。
🚫 不要去找那个文件，也 🚫 不要试图恢复它。

---

## 8. Git 提交规范

### 8.1 Commit message

- 中文或英文，**单 commit 内统一**
- 格式：`<scope>: <50 字以内简述>` + 空行 + 详细描述 + 空行 + `Refs: 11 §9.6.2`
- scope 例：`p1_motion` · `p2_core` · `quadruped` · `payload` · `common` · `configs` · `docs`
- 🚫 **单个 commit 不允许跨多个独立功能**（不允许「修 Bug A + 顺手加 Feature B」）

### 8.2 PR 门禁（CI 自动检查）

1. **ruff / flake8** Python lint
2. **mypy** 严格模式
3. **clang-tidy + clang-format** C++
4. **pytest** 单元 + 集成全绿
5. **头部注释完整性**（每个 `.py` / `.cc` / `.h` / `.sh` 必须有五字段）
6. **自定义静态规则**：
   - `xbrain/**/*.py` 出现 `import rclpy` / `import sqlite3` / `import requests` → 拒
   - 任何 `*.py` 出现 `time.time()` / `datetime.now()` 做年龄超时 → 拒
   - 任何 `*.cc` / `*.h` 出现 `system_clock` / `CLOCK_REALTIME` / 裸 `rclcpp::Clock()` → 拒
   - 安全参数出现默认值（dataclass default / `dict.get(k, v)` / `or v`）→ 拒（§3.1）
   - `E_*` 字面量不来自 `common/errors/` 导出 → 拒（§3.5）
   - `declare_subscriber(` 调用点未落到 `self.xxx =` / `list.append` / `SubscriberRegistry.declare` → 拒（§4.3）
   - 源代码注释 / 打印含 emoji 或中文标点 → 拒（§2.2）
   - **全仓 `grep -rn "config/"`（单数相对路径）必须零命中**（配置根是复数 `configs/`）

---

## 9. 工作流约定

### 9.1 优先级

1. **任何疑问优先查 `docs/` 十一册**
2. 找不到 / 文档矛盾 → **停下问用户**，🚫 不擅自决策
3. 设计层面修改 → **先更文档再写代码**，🚫 不允许代码与文档脱节
4. 编码中发现设计遗漏 / 不清晰 / 自相矛盾 → **立即停下**并告诉用户哪一册哪一节

### 9.2 代码注释反向链接

```python
# 见 11 §9.6.2 速度门四段定标（左闭右开）:
#   [3.0, inf) -> 2.0 | [1.8, 3.0) -> 0.5 | [1.25, 1.8) -> 0.2 | [0, 1.25) -> 0
# 迟滞: 降档 d_free < 3.0 立即生效; 升回 2.0 需 >= 3.5m 且持续 T_up = 3.0s
```

```python
# 见 12 S6A RCG-1: r_robot 为 0.0 占位时旋转许可恒不通过.
# 注意 spin_like 判据用 r_eff (fallback 0.60), 不用 r_robot --
# 否则 path_follow 的正常转弯 (vx=1.5, wz=0.3) 会被误判为原地旋转.
```

### 9.3 不为将来留口子

- 🚫 严禁「为将来扩展」引入接口预留 / 配置开关
- schema 里允许预留字段，但**业务逻辑不允许写消费它的代码**
- 任何「留口子」的代码在 review 时**强制删除**

---

## 10. ★★★ V6 常见陷阱速查（编码必读）

| 陷阱 | 出处 | 防护 |
|---|---|---|
| 安全参数写 `0.0` 冒充已赋值 ⇒ `v_max = 0` 静默限死整机 | §3.1 · `10` §5.4.5 | 未标定一律 `null`，断言 A 报键路径 |
| 断言只有负向 ⇒ 空壳实现全绿通过 | §3.3 · `11` §14.6 | 每条断言配一个必然违反它的变异体 |
| 判据自伤（判据句含自己要 grep 的字串）⇒ 恒红 | §3.2 · `11` EC-2 | 判据句必须在扫描面之外 |
| 墙钟阶跃误触发 Tier 1 | `11` CLK-C1 | 一切超时/周期/年龄用单调钟 |
| **纯旋转无几何安全门**（速度门六项全是线速度、围栏靠向量投影、Nav2 `simulate_ahead_time: 0.0`）⇒ **四层都拦不住 `wz`** | `12` §6A · R-3 | 旋转许可判定器 `RCG-1~4` |
| 旋转许可 fail-safe 做过头 ⇒ `path_follow` 正常转弯被拦 | `12` §6A | `spin_like` 用 `r_eff`（fallback 0.60），不用 `r_robot` |
| 半双工使**两级确认的语音否决窗口都等于 0** ⇒ `L1b` 的价值是「预告」不是「喊停」 | `18` §13.1 · U53 | 🚫 不要实现「播报中喊停」的撤销逻辑 |
| **AEC 结构上不可能**（TTS 在 GZH-2 设备内合成，上装侧拿不到播出波形） | `00` VOI-* | 靠半双工门控，🚫 不要试图做回声消除 |
| USB MIC 只有 **48000Hz 唯一采样率** ⇒ 必须 48k→16k **3:1 抽取** | `00` VOI-10a | 20ms@48k = 960 样本 → 320 样本@16k |
| ASR/TTS 超时上界 5s（`AS-7`），但代码里是 **30.0** 且 ASR/LLM **共用一个字段** | `11` §8.13.1 | 拆成 `asr_timeout_s` / `llm_timeout_s` |
| 灯光：补光灯/探照灯**常亮做照明不做爆闪**；警示双闪走**红蓝** `MSG_REDBLUE` | `00` §5.1.1 | 红蓝**无调光位**；`0x03` 探照灯爆闪必须由 P2 显式关掉（`14` GL-8） |
| 8529 灯光 CRC：**控制帧覆盖 len+ID+payload，状态帧 `0x25` 只覆盖 payload** | payload 实测 | CRC-8/MAXIM (poly 0x31, reflected 0x8C, init 0x00) |
| GZH-2 两个真机坑 | payload 实测 | RST 丢关灯帧（close 前 `sleep 0.2s`）· 亮度记忆（`0x1E` = 灭+记忆30） |
| CHS-A 缺 `PatrolDevice` / `Time` / hex32 序列化 ⇒ **第一帧回 `0xE002`** | §5.5 | 直接读厂商 PDF，不要只读 `13` |
| DDS 名字映射写错 ⇒ **participant 起来了、一个包收不到**，与网络不通不可区分 | §5.4 | `/IMU` → topic `"rt/IMU"` / type `sensor_msgs::msg::dds_::Imu_` |
| 100 Hz odom 是**发布频率**，信息更新率受 **10 Hz 限制** | `13` §4.3 | 注释里必须写清，否则下游误以为有 100Hz 新信息 |
| 各进程各自展开 `${common.*}` ⇒ **解析出不同结果** | `10` §5.4.1 | 运行期只读 `/run/xbrain/resolved/`，🚫 不读源 |
| 「跳过安全断言」的开关 = 远程解除全部安全约束的通道 | `12` §12.1 | 🚫 绝不实现 |
| `E_*` 字符串硬编码 ⇒ 大小写/前缀不一致，联调才发现 | `11` §13.8 | 由 `common/errors/` 共享库导出 |
| 闭集外的值静默透传 / 「未知值降级解释」 | `11` §13.6 | 越界必抛 |
| 服务侧返回自由文本 `detail` **不是违约**（映射是网关的活） | `11` §8.13.5 | 🚫 不要跑去改服务代码塞 `E_*` |
| 安全距离 1m 是**单向**约束 | U54 规则③ | 机器人自身运动不得使间距 <1m；障碍主动接近 ⇒ 零速**不后退** |
| 以最高速急停也会撞进安全圈（`d(2.0)=1.60m > 1.0m`） | U54 | 靠 `D_slow = 3.0m` 提前降速化解，🚫 不靠急停 |
| 雷达持续缺失 ⇒ **出勤中止**（T-04 1s / T-06 2s 停车） | `SET-03` | ★ 「LiDAR 从必需变增强」只对**探距规格**成立，对**可用性**不成立 |
| `_pyc_` stale（同步 .py 后 `.pyc` mtime 更新）⇒ 加载**旧字节码**，改的代码静默不生效 | V5 血泪 | 同步后 `find . -name __pycache__ -type d -exec rm -rf {} +` 再重启 |

---

**违反后果**：PR 拒绝合入。反复违反：协作终止。
