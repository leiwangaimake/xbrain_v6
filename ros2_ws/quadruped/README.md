# quadruped

M20S 底盘接口与 **Tier 1 安全兜底**进程（设计真源：`docs/13-quadruped与Tier1详细设计.md`）。

> 参考：`docs/QUADRUPED/quadruped_开发前评审_2026-09-15.md`（批次计划与实测回填清单）·
> `docs/QUADRUPED/M20S_底盘接入与探测实录_2026-09-15.md`（协议实测、上报频率、变更台账）。

## 本包现在是什么（B0 + B1，2026-09-15）

**它还不能控制底盘，也不假装能。** 已落的都是能离线测试的东西：

| 组件 | 文件 | 对应设计 |
|---|---|---|
| 配置加载 | `include/quadruped/quadruped_config.h` · `src/quadruped_config.cc` | `13` §8.2（v1.4）；读**解析产物**，任何 null 即抛并打印键路径（CLAUDE.md §3.1） |
| 单一出向 seam | `include/quadruped/tx_owner.h` · `src/tx_owner.cc` | `13` CA-4 / TX-4～TX-7；实时侧 try-and-skip，非实时侧有界自旋（CPP-3 / CPP-4） |
| 启动自证 | `src/main.cc` | `13` DDS-9 / CB-4：打印两个域号、探测顺序、生效码表 |
| CHS-A 编解码（B1） | `include/quadruped/chs_a_codec.h` · `src/chs_a_codec.cc` | `13` §2.2 · CLAUDE.md §5.5；16 B APDU 头、ASDU JSON、hex32 码表按**十进制**序列化 |
| CHS-A 分帧重组（B1） | `include/quadruped/chs_a_framer.h` · `src/chs_a_framer.cc` | `13` §2.2 **FR-1 / FR-2 / FR-3 / FR-5**（FR-4 是发送侧，在 `tx_owner`；FR-5 的 `TCP_NODELAY` 是套接字选项，归 B2） |

**通道仍然一条都没有起来**：链路与四路上报（B2）、Tier 1 单函数（B3）、
RT 面（B4）、域 0 DDS（B7）、rclcpp odom/TF（B8）。批次表见评审文档 §6。
B1 交付的是**纯函数层**——它能把字节变成结构、把结构变成字节，但它不持有任何套接字。

★ **无参数运行会退出 78 并说明原因**，这是有意的：systemd 单元正是这样调用它的，
若它改成「起来、打日志、空转」，`10` §3.3 的 Stage 1 就会显示成功，而 p1_motion 会一直
等一个永远不会来的 `hello_ack`，现场表现成底盘故障。单元里的 `RestartPreventExitStatus=78`
与 `main.cc` 里的拒绝分支是**一对**，进程变成真的那天一起删。

## 构建 / 测试 / 安装

```bash
scripts/build_quadruped.sh              # 配置 + 编译 + 测试
scripts/build_quadruped.sh --install    # 再安装到 data/install/quadruped
scripts/build_quadruped.sh --clean      # 先删构建树
```

- **在 ORIN 上编**（aarch64）。脚本会在 `/opt/ros/humble/setup.bash` 存在时自动 source。
- **ROS 2 是可选的**：不 source 时 `rclcpp` / `CycloneDDS` 两个可选目标跳过并打印说明，
  核心库与两个测试照常编译通过。这条双向都验证过（不 source 时跳过，source 后两者都找到）。
- 安装布局 `data/install/quadruped/lib/quadruped/quadruped_m20`，与
  `deploy/systemd/xbrain-quadruped.service` 的 `ExecStart` 逐字一致（DEC-15 / `99` U83）。
- 顺序是**编译 → 测试 → 安装**：安装会解除单元的 `ConditionPathExists` gating，
  把未跑过测试的二进制交给 systemd 是本末倒置。

## 自检（先跑这个）

```bash
data/install/quadruped/lib/quadruped/quadruped_m20 --selfcheck \
  /opt/xbrain_v6/data/run/resolved/quadruped.yaml
```

打印 `runtime.transport` 块：两个 DDS 域号、四个端点候选及其 `tls` / `enabled`、
生效码表、Tier 1 硬限幅。**这是区分「两个域塌成一个」与「网络不通」的唯一低成本手段**
——两者的现象都是 participant 起来了却一个包收不到（`13` DDS-9）。

它也回答「我的配置到底展开了没有」：未经冻结线的源文件里 `${common.spec.*}` 仍是字面量，
自检会当场报 `config key not a number: quadruped.odom.a_max_mps2 = '${common...}'`。

## 金标向量与变异体

`test/golden/chs_a_frames.txt` 里的 7 个向量**是 2026-09-15 从真实链路上抓下来的**，
不是这个编码器生成的。这个区别是它存在的全部理由：由被测代码生成的向量与被测代码
永远不会不一致，它只能证明代码等于它自己（CLAUDE.md §3.2 第一种形态）。文件头写了
每一条的来源与重抓命令。

```bash
python3 scripts/ci/quadruped_mutants.py     # 判据: 每个变异体都必须被杀
```

这个脚本是 B1 的验收方式，也是**「哪几行是真的被守住了」的清单**。它先建并跑基线，
基线不绿就直接退出——否则每个变异体都会因为无关原因显示被杀，整轮读起来像满分。
编译不过的变异体单独报 `NOBUILD` 并让整轮失败：编译错误不是断言，把它算成"杀掉"
等于把测试没做的事记在测试头上。

★ 首轮跑出两个**存活**，两个都是真洞：编码器从未被检查过 `PatrolDevice` 包裹
（只有从抓包读回的帧被检查了），而"保留字节为零"那条跑在一个恰好为零的栈缓冲上。
现在编码前会先用 `0xAA` 填满缓冲区——**局部数组恰好为零会让这条断言永远绿**。

## 三条容易踩的

1. **不要读 `configs/quadruped.yaml`**。运行期只读 `/run/xbrain/resolved/quadruped.yaml`
   （`10` §5.4.1）。冻结线把 `${common.*}` 一次性展开后**丢弃 `common` 子树**，所以
   `13` §8.2 v1.4 才必须把 `tier1.limits.*` / `odom.a_max_mps2` 写成真的引用键——
   只在注释里说「引用 common.spec.*」，值永远到不了进程，而 Tier 1 的硬限幅正是靠它。
2. **`tier1.limits.*` 现在会让进程拒绝启动**（真机 `common.spec.max_vy_mps` 等为 null，
   卡 `11` V-01 未标定）。这是设计行为，不是缺陷：一个不限幅的底盘不能开。
   🚫 不许为「让它跑起来」填 0.0（`CLAUDE.md` §3.1 的 fail-silent）。
3. **`Framer::Next()` 交回的是指向内部缓冲的指针，只在下次调用前有效**。这不是疏忽：
   最大的一路上报 2.4 KB，每帧复制一份会在 10 Hz 上白白搬数据。缓冲的回收是
   **延迟到下一次进入时**做的，所以「拿到帧 → 立刻再调一次 Next → 再读上一帧」
   读到的是已经被搬走的字节。要留就自己拷。

## 布局

```
include/quadruped/   公开头（quadruped_config.h · tx_owner.h）
src/                 实现 + main.cc
test/                离线单测（无 ROS、无硬件、无底盘），ctest 注册
test/golden/         真实链路抓包向量（🚫 不是生成的，见上）
CMakeLists.txt       纯 CMake；rclcpp 与 CycloneDDS 为可选目标
package.xml          build_type cmake（B8 加 uplink 时再加 ROS 依赖）
```
