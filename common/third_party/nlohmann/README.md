# nlohmann/json（vendored 单头，第三方）

| 项 | 值 |
|---|---|
| 上游 | https://github.com/nlohmann/json ，tag `v3.11.3`，文件 `single_include/nlohmann/json.hpp` |
| 许可 | MIT（`LICENSE.MIT` 随附） |
| `sha256(json.hpp)` | `9bea4c8066ef4a1c206b2be5a36302f8926f7fdc6087af5d20b417d0cf103ea6` |
| 引入依据 | `99` U85 第 ⑥ 项（2026-09-15 用户裁决）：quadruped 解析底盘 CHS-A 的 JSON ASDU 需要一个解析器，`common/` 原先只有手写序列化 |
| 修改 | 🚫 **零修改**。升级 = 整文件替换 ＋ 更新本表的 tag 与 sha256 |
| lint | 本目录在 `scripts/lint/charset_lint.py` 的 `THIRD_PARTY_SNAPSHOTS` 排除表内（header / charset / no_literal_ecode / cxx_discipline_audit / no_config_singular 共用），🚫 不给它加 Hachist 头注、不改标点 |

## 使用约束（`13` QD-7 / CPP-3）

- 只允许在**非实时线程**使用（quadruped 的 `chs_a_rx` 解析上报、`chs_a_tx` 编非周期指令）；🚫 `ctrl` / `chs_b` 线程内不得出现 `nlohmann::json` —— 它做动态分配。
- 急停零速帧与心跳帧是启动时序列化好的常量帧（`13` TX-1），不经本库。
- 解析异常一律在调用点捕获（`13` CPP-2）：`nlohmann::json::parse(..., nullptr, false)` 返回 `discarded` 而不抛。
