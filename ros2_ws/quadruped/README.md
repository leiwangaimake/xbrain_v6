# quadruped

M20S 底盘接口与 **Tier 1 安全兜底**进程（设计真源：`docs/13-quadruped与Tier1详细设计.md`）。

> 参考：`docs/QUADRUPED/quadruped_开发前评审_2026-09-15.md`（批次计划与实测回填清单）·
> `docs/QUADRUPED/M20S_底盘接入与探测实录_2026-09-15.md`（协议实测、上报频率、变更台账）。

## 本包现在是什么（B0，2026-09-15）

**它还不能控制底盘，也不假装能。** B0 只落了三样能离线测试的东西：

| 组件 | 文件 | 对应设计 |
|---|---|---|
| 配置加载 | `include/quadruped/quadruped_config.h` · `src/quadruped_config.cc` | `13` §8.2（v1.4）；读**解析产物**，任何 null 即抛并打印键路径（CLAUDE.md §3.1） |
| 单一出向 seam | `include/quadruped/tx_owner.h` · `src/tx_owner.cc` | `13` CA-4 / TX-4～TX-7；实时侧 try-and-skip，非实时侧有界自旋（CPP-3 / CPP-4） |
| 启动自证 | `src/main.cc` | `13` DDS-9 / CB-4：打印两个域号、探测顺序、生效码表 |

**三条通道一条都没有**：CHS-A 编解码（B1）、链路与四路上报（B2）、Tier 1 单函数（B3）、
RT 面（B4）、域 0 DDS（B7）、rclcpp odom/TF（B8）。批次表见评审文档 §6。

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

## 两条容易踩的

1. **不要读 `configs/quadruped.yaml`**。运行期只读 `/run/xbrain/resolved/quadruped.yaml`
   （`10` §5.4.1）。冻结线把 `${common.*}` 一次性展开后**丢弃 `common` 子树**，所以
   `13` §8.2 v1.4 才必须把 `tier1.limits.*` / `odom.a_max_mps2` 写成真的引用键——
   只在注释里说「引用 common.spec.*」，值永远到不了进程，而 Tier 1 的硬限幅正是靠它。
2. **`tier1.limits.*` 现在会让进程拒绝启动**（真机 `common.spec.max_vy_mps` 等为 null，
   卡 `11` V-01 未标定）。这是设计行为，不是缺陷：一个不限幅的底盘不能开。
   🚫 不许为「让它跑起来」填 0.0（`CLAUDE.md` §3.1 的 fail-silent）。

## 布局

```
include/quadruped/   公开头（quadruped_config.h · tx_owner.h）
src/                 实现 + main.cc
test/                离线单测（无 ROS、无硬件、无底盘），ctest 注册
CMakeLists.txt       纯 CMake；rclcpp 与 CycloneDDS 为可选目标
package.xml          build_type cmake（B8 加 uplink 时再加 ROS 依赖）
```
