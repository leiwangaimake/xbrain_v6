# XBRAIN_V6 Phase 0 待裁决清单

> 生成: 2026-08-06 · 来源: phase0-sweep 工作流 triage 结果
> 用途: 供用户/册主逐条拍板。裁决后回填各项 TODO 阻塞列, 即可解锁自动实现。
>
> ⚠️ **锚点来源说明**: 下列册号/章节/引文由 triage 子 agent 提供并声称已 grep 核对。
> 本项目实测过子 agent 编造引文, 故 **动手改任一册前必须自己 `grep -F` 复核该锚点**。
> 本文档是【决策依据汇总】, 不是【已核实事实】。

## 一览

- 需人裁决(BLOCKED_DECISION): **31** 项
- 待实测/厂商/D-45(BLOCKED_EXTERNAL): **3** 项

---

## A. 需人裁决（31 项）

### CFG-CM-6

判据要求 RS-LIM「映射键集与 11 §3.4 gate.limiter 闭集双向差集为空, 不等即 E_CONFIG_INVALID 拒绝启动」, 且 🚫 不得实现成单向包含。实测: 11 §3.4 为 14 值(含 heading / clock), 而 16 §8.3B 映射表逐字「现有 12 条」, 差的正是 heading 与 clock 两条中文。16 §16 Q-P4-29 与 11 §14.4 G-20 两侧台账都还开着, 16 逐字写「🚫 实现者不得自行为这两个值编中文」「这是话术设计决策, 不是机械补格」, 并给了理由(heading 一个词要盖 L2-active 降级与 L3 失效两种互斥状态; L1.5 的硬速度上限 G-15 至今空白, 触发集未闭合; 表内去术语/留术语两种风格推不出 clock 的唯一写法)。⇒ 今天写出来的 RS-LIM 是一条恒红且让整栈起不来的断言。XBRAIN_V6_TODO.md 自身已把这条记为待裁决(与 GWY-CM-P4-1 合并 / 引入 LIMITER_CN_PENDING 占位并设期限), 裁决未下。

- [ ] 已裁决 · 结论: ______

### CFG-CM-16

取值本身今天查得到: 11 §15.6A.3 的 R2.6-e 逐字给出 task/progress 的 reason 四值(rotation_blocked / lateral_clearance_unavailable / teleop_stale / deadman_released), R2.6-f 给出 control_mode「现仅 jog」。但判据要求的左操作数 —— 11 §13 闭集章的两张对应表 —— 今天都不存在(R2.6-e / R2.6-f 是 §15.6A.3 的待办台账行, 不是闭集表体), 判据自己把双向差集元测试写成【空位】。一条结构上无法求值的断言正是 CLAUDE.md §3.2 形态① 要禁的; 且把两个集合写进 sets.yaml 会当场让现有 test_every_exported_set_has_an_extractor 变红, 抽取器该指向哪个文档面(§13 还是 §15.6A.3 的台账行)是 11 册主的裁决, R2.6-e / R2.6-f 都还开着, 🚫 不由实现者定。

- [ ] 已裁决 · 结论: ______

### CFG-CM-18

判据自相矛盾, 已实测证伪: 它同时要求「导出 Mps / Factor / Mps2 / Seconds 四个 NewType」与「min(Mps(2.0), Factor(0.3)) 在 mypy 严格模式下必须报错」, 而 NewType 的基类型是 float 时 mypy 会把两者 join 成 float 并放行。我在 scratchpad 的独立 venv 里装 mypy 2.3.0 实跑 --strict: ① NewType('Mps', float) 与 NewType('Factor', float) 下 min(Mps(2.0), Factor(0.3)) => Success: no issues; ② 改成 NewType 套 float 子类后 reveal_type(x) 仍为 float, 仍 Success; ③ 只有直接赋值 b: Mps = Factor(0.3) 才报 [assignment]。⇒ 要让 min() 成为类型错误必须放弃 float 兼容(改成不继承 float 的类), 那会改掉所有算术调用点, 是设计裁决不是实现细节; 且 CLAUDE.md §4.5 与本判据是同一句要求, 两处须一起改。附注: 本机未装 mypy(判据的求值器缺位, 有网可装, 这不是主因)。

- [ ] 已裁决 · 结论: ______

### CFG-CF-1

拆两半。【可写的一半】configs/sites · configs/calib · configs/prompts/missions 今天各有 .gitkeep（不是带头注骨架），configs/secrets 与 configs/secrets/chassis_tls 已是 0700 且当前两条 find 通配零命中（文件集为空，属空真），补头注骨架 + 一个 Shell 权限基线检查今天就能写，变异体（造一个 client.key 再 chmod 0644 ⇒ 检查必须红并打印绝对路径）也能在自己的 Shell 执行体上跑红。【卡住的一半】onvif_credentials.json：`10` §5.4.4 断言 J ③ 逐字仍点名『onvif_credentials.json 必须 0600』并把它列进②的必需文件清单，而 `11` §7.4.9 的『凭证存放』表逐字写着『原明文文件 configs/onvif_credentials.json 已删除，本节给方案』、方案 C 逐字标『v0.3 的做法，已删』，且首选 A（systemd LoadCredential）与 B 之间仍挂 U-PTZ-2 待用户拍板。两册正面相反：建骨架 = 把 `11` 判死的明文凭据文件复活；写进 J 的清单说它可缺 = 改 `10` 的断言 J。TODO 自己在 CFG-FZ-2 的风险注里也逐字要求『须在 J 的清单里明确它是必需还是存在才查，🚫 不得含糊』。按 CLAUDE.md §9.1 这属于必须停下问用户的设计冲突，🚫 不自行选边。

- [ ] 已裁决 · 结论: ______

### CFG-CF-4

三个子项里有两个要人拍板。①【可写】PROVENANCE 取值域校验：brake.yaml 现存三键 common.safety.t_lat_s: 0.4 / brake.a_mps2: 2.5 / brake.k: 1.5 各带 PROVENANCE 标记，取值域字符串本身在文件头注里逐字写着，写个校验器不缺输入 —— 但它的权威出处 SET-01 是悬空引用：扫描面 = /opt/xbrain_v6/docs/*.md 全部 19 个 md（含十四册全部），实测 SET-01 仅 README.md 1 命中、XBRAIN_V6_TODO.md 7 命中，十四册内零命中（对照 SET-03 在 00/10/12/14/18/19/21 均有命中），故『护栏 3』今天没有正本，TODO 阻塞列自己也写着『SET-01 悬空引用待用户裁定』。②【卡住】补 clock.* 七键：`11` §1.5.5 的配置块逐字是顶层 clock:（sync_timeout_s / offset_threshold_ms / ref_max_age_s / unsynced_max_speed_mps / rtc_trusted / step_notify_ms / allow_unsynced_motion 七键齐全），而 `10` §5.4.3 L3 行只许 common.safety.*、§5.4.5 该行写 common.safety.clock.*；现存 configs/safety/clock.yaml 头注已就地把这条登记为 cannot_close 并写明『归 11 册主裁，本轮不代它选边』。另 rtc_trusted 的 false 挂着『待确认板载 RTC 是否装配电池』的硬件问题。③【卡住】补 common.safety.d_safe_m：`10` §5.4.4 断言 N 行逐字『上提与否是分层决策，本轮不擅自定 —— 登记 11 §15.6A.4』，而 `11` §9.6.1 的 S22 ㈡ 逐字说『上提为 common.safety.d_safe_m 的前置条件【已经满足】』（C-6 与 MR-1 两条启动断言已是第三、第四个读者）。两册对同一个分层决策给出相反状态，且 `10` §5.4.5 对照表左列今天确实没有这个键。⇒ 本项落值前必须先有用户/册主裁决，🚫 不自行补键。

- [ ] 已裁决 · 结论: ______

### CFG-CF-7

两处是十四册没说的，必须人来定。① common.site.* 无任何键清单：扫描面 = /opt/xbrain_v6/docs/*.md 全部 19 个 md，common.site. 仅在 `10` 命中 2 处，两处都只是把它当【命名空间】列出（§5.4.0 目录树的『{site_id}.yaml  common.geo.* · common.site.* · retention.*』与 §5.4.3 L4 行），十四册没有任何一节枚举它下面有哪些键；而 `10` §5.4.5 对照表左列只有 common.site_id（且逐字『文件名即值』）。判据要求 sites 文件『含 common.site.*』，今天写不出这一段。② 文件名本身是未定的实例名：sites/{site_id}.yaml 与 calib/{robot_id}.yaml 的名字即取值，site_id 由现场/甲方给、robot_id 是每台车的身份，十四册里没有任何一个具体实例名，断言 J ② 又要按 XBRAIN_SITE_ID / robot_id 去 stat 这个名字。③ 可写的部分要说清：enu_origin 的正本在 `11` §9A.12 逐字给出（`# /opt/xbrain_v6/configs/sites/{site_id}.yaml —— ★ L4 现场层`，三分量全 null，U-01/U-02 只卡取值）；calib 的 frames.*.xyz/rpy/accuracy · gate.* · lat_err_ref_m · calib_rev 在 `11` §10.4.4『输出文件 schema』有正本，M-24 只卡门限【取值】，留 null 即可 —— 这两半属于『结构可写、值留 null』。④ ★ 阻塞列里的『15/17 保留期口径未裁』经实测已经过期，🚫 不要再拿它当阻塞：`17` §... 保留期行的 v0.7.3 订正已给出『现行结论』逐条写死 events 90（P5 唯一负责）、tasks 30 / commands 180 归 P3（`15` §9.11 同值），并把 v0.1 的 {info:7,warn:30,...} 逐字判为『是错的』；`10` §5.4.5 那句『两者不可同真、归 15/17 两方裁决』是尚未回填的旧登记。

- [ ] 已裁决 · 结论: ______

### CFG-FZ-16

卡在变异体3, 是跨册冲突不是工作量问题. 已实测的部分: 依赖 CFG-CM-7 已落地 (pytest tests/common -q => 234 passed); 我用探针脚本直接调 xbrain/common/config/layers.py 的 check_namespace, 变异体1 (顶层 speed_profiles 分别放 L1 与 L2, 两次) 与变异体2 (common.safety.brake 放进 L2) 今天【都已变红】, 抛 E_CONFIG_INVALID 且消息含层名与完整键路径, 判据这两条的材料齐全. 变异体3 (全量档位表同时写进 L1 与 L2 => 断言 B 必须变红) 实测【被接受】, provenance 记为 L2, 且无法在不做设计裁决的前提下补上, 三条互相矛盾的逐字: (a) 10 §5.4.4 断言 B 正文 (grep -F 本B本 无重复定义, 即 docs/10-顶层设计.md 行 3076 那一行) 逐字为 L6 出现 common 顶层键 / 出现 §5.4.5 别名表内的私有键 / 私有键值与某个 common 叶子同名同值, 三个子项全部在【L6 源文件】上求值, 没有一条覆盖 L1 与 L2; (b) 10 §5.4.3 L2 行逐字允许 common.spec.* 与 common.motion.*, 即 L2 写整张档位表在命名空间上合法; (c) 11 §15 S22 (grep -F 全量档位表同时写进 L1 与 L2, 命中 1) 却要求断言 B 对这一形态变红. 机型差异覆盖与第二份真源的键集合完全相同, xbrain/common/config/layers.py 的 LAYERS 表上方注释已就地记过这一点. 需要人裁决的正是: 断言 B 是否新增一个在 L1/L2 上求值的子项, 其判据如何区分整表复制与部分覆盖, 还是把变异体3 改挂到别的断言编号. 按 CLAUDE.md 9.1 不自行裁决. 裁决落地后剩余工作很小: L1/L3/L4 各补一条独立用例 (L2 与 L4b 已有), L6 一条走 refs.check_l6 的 R-6 (已实现, 实测抛 E_CONFIG_INVALID). 注意不是 ALREADY_DONE: 现有 tests/common/test_config_overlay.py 只覆盖 L0/L2/L4b, 缺 L1/L3/L4 的独立用例, L6 的用例挂在 CFG-CM-8 的引用轴测试里.

- [ ] 已裁决 · 结论: ______

### CFG-BT-14

需要人拍板的点有四处,都不是我能自己定的(CLAUDE.md 9.1)。(1) 落点三选一未裁:表格目录列写 `xbrain/common/boot/ + xbrain/`;TODO 附录第 3 行逐字『CFG-BT-14 = common/boot/ + xbrain/,INF-BT-2 = xbrain/boot/。二者不是措辞差异而是架构分歧』并建议统一落 `xbrain/p2_core/boot/`;附录第 24 行结论却逐字是『落点统一到 xbrain/boot/』;Phase 2 的 CHK-1-52 又逐字写『启动链的三类实现体归 xbrain/boot/{probe,freeze,failure_class}』。三处互斥,且 docs/99-决策记录.md 全文 grep `xbrain/boot/` / `common/boot/` / `p2_core/boot/` 零命中,即无裁决。(2) 真源策略互斥:本项判据要求『表驱动 + 与 10 §3.3.6 表体双向差集为空』,重复项 INF-BT-2 要求『从表体生成,生成物入库,CI 比对漂移即失败』,附录第 24 行自己就写着『真源策略互斥』,照哪条写都会让另一条判据失效。(3) 四类 vs 五值的实打实缺口:10 §3.3.6『逐条清单』表的『类』列,第 27 行逐字是『不属失败』,第 28/29 行逐字是『指针』(perception / RNS 内部启动失败),都不在 R/B/D/T 内;而元测试要求分类器表行集合与该表体双向差集为空 —— 这三行收不收、收进来给什么类,十四册没写。实现者写到一半必撞这个。(4) 相位与依赖倒置:依赖列的 CFG-BT-5 在 Phase 2(TODO 附录第 32 行把 `CFG-BT-14(P0)->CFG-BT-5(P2)` 逐条列为跨相位倒置),附录第 24 行的处置逐字是『CFG-BT-14 从 Phase 0 移出』。另:判据①点名的变异体『把某 R 类项实现成进程不起但不禁运动』今天跑不了 —— 它需要观察窗(CFG-BT-4,Phase 2)和 P1 的 allow_motion(CFG-BT-11,Phase 2),两者都不存在,我没有跑过这个变异体,也不声称跑过。已核实为真的部分:10 §3.3.6 四类定义表与逐条清单表体完整存在(R 逐字『带病跑下去会得到错的答案而不是没有答案』),Phase 0 依赖 CFG-CM-2 已落地(python3 -c 导入 xbrain.common.errors 成功,retryable 存在)。

- [ ] 已裁决 · 结论: ______

### CFG-BT-16

落点未裁 + 三处依赖对象今天都不存在,两类阻塞叠加,主因是前者。(1) 落点未裁:TODO 附录第 4 行逐字『CFG-BT-16 = common/qos/,INF-ZN-6 = common/zenoh/』,并指出与 CFG-CM-17(common/zenoh/,QoS 档位工厂)、CPP-CXX-4(common/rtcomm/qos_profiles.h)三处并存会让 QoS 档位出现三份实现,『直接违反 11 §2.4.5 唯一真源』;docs/99-决策记录.md 对 `common/qos/` 与 `common/zenoh/` 双双零命中,无裁决。更麻烦的是本项自己的风险行逐字写『五个 Python 进程 + 三个 C++ 进程都要用,必须放 common/』,而 CLAUDE.md §0.2 明令 common/ 只放部署产物、Python 源码不进 common/ —— 语言列又写着 `Python + C++17`,C++ 半边的部署头该落哪儿同样没定。(2) 直接依赖缺失:CFG-CM-17(Phase 0,xbrain/common/zenoh/)今天在磁盘上不存在,而 QoS 六档档位名(11 F-11 逐字『Q0 / Q1 / Q2 / Q3 / Q3-rt / Q4 六个档位名』)与 publisher 声明面都由它导出,没有它就没有可登记的对象。(3) 元测试的靶子不存在:判据逐字『断言 F(FZ-6)的覆盖清单里不含 A-1』,FZ-6 指 CFG-FZ-6(Phase 1,scripts/),没有实现就没有『覆盖清单』这个可做差集的对象;改成去 grep 10 §5.4.4 断言 F 的正文,那是另一条断言,不是判据说的那条。(4) 判据点名的变异体『在 p1_motion 里让同一线程同时发 rt/motion/cmd_vel(Q1) 与一条 Q3 事件』需要 p1_motion(Phase 2),今天不存在,该变异体跑不了,我没跑也不声称跑过。已核实为真的部分:11 §2.4.8 反模式 A-1 定义完整逐字『同一线程既发 Q1/Q0 又发 Q3』;10 §5.4.4 断言 F 行逐字『A-1(同线程混发 Q1/Q0 与 Q3)无法从配置判定,由各进程启动时自检』;E_QOS_VIOLATION 已在 xbrain/common/errors/codes.yaml 落码并可导入 —— 但 11 §13.16 把它逐字列为『僵尸码…四条一律待评审时二选一:正文补引用,或整条删除』,该码的存废本身也还挂着评审。

- [ ] 已裁决 · 结论: ______

### INF-CM-3

xbrain/common/clock/ does not exist. The now_mono() wrapper itself is trivial and writable today, but the wrapper is explicitly NOT what this item is for -- the criterion says verbatim '与 CI-1 规则 (2)(3) 的关系写死: 静态扫描抓「用了墙钟」, 本项抓「阈值抄错/漏导」, 两条互不替代'. The threshold-export half, which is the whole item, needs four human rulings that no volume supplies. (1) SYMBOL NAME. Two Phase 0 rows target the same directory with different exports: CFG-CM-12 (TODO line 107) says verbatim '导出 mono_now_s() 与 MonoClock'; INF-CM-3 (line 145) says verbatim '导出 now_mono()'. The corrections appendix row 25 PROPOSES taking mono_now_s(), but that appendix is headed '两遍检查报出的判据订正（开工时逐条改）' -- pending corrections, not a ruling -- and grep of docs/99-决策记录.md finds no decision. Whichever name is written first silently decides it for every consumer, INF-CM-2 first among them. (2) THE ASSERTION IS UNFALSIFIABLE AS WRITTEN. Criterion (1) is 'a metatest extracts every T-* and threshold from 11 S1.6's table body and the symmetric difference against the exported constants must be empty' while the constants are themselves 'generated from the S1.6 table body'. Generated artifact versus its own generation source has an empty difference BY CONSTRUCTION -- CLAUDE.md 3.2 form 7. Appendix row 25 says so in the same words and states the criterion '须改成「生成物入库副本 == 重跑输出」', i.e. the criterion has to be rewritten before it can be satisfied. (3) THERE IS NO PER-ROW TIME-BASE MARKER TO ASSERT ON. Criterion (2) demands 'A 组每一项的时基标记均为 monotonic, 标 wall 即失败' and mutant 3 is '把某 A 组项标成墙钟'. I read 11 S1.6.1 (lines 1476-1490): the columns are `# | 判定方 | 被判对象 | 阈值 | 触发行为 | 出处` -- there is no 时钟 column. The only monotonic statement is the S1.6.1 section HEADING 'A · 安全回路（★ 全部单调时钟，不可配置为墙钟）'. So no row can be marked wall, the named mutation cannot be performed on the document, and the assertion collapses into 'rows in section A are in section A' -- form 7 again. (4) THRESHOLD EXTRACTION IS AMBIGUOUS IN THE SOURCE, and per the standing rule I must not pick. T-07 reads '3 s ⚠️ 建议收紧 1.5 s' and T-08 '10 s ⚠️ 建议收紧 5 s' (two numbers each, no ruling on which is exported); T-44 is '⚠️ 100-200 ms 待定'; T-24 is '3 次 / 3 s' (a count and a duration); T-22 is '≥ 1 Hz（建议 2 Hz）'; T-43 is bare 'timeout_s'; T-ESTOP-CLOUD is '⚠️ 待实测（M-23／00 COM-56）'; T-BCAST-MAX carries no number and points at 14 S11 mode.b_cast_max_duration_s. SEPARATELY AND WORTH ESCALATING ON ITS OWN: the scan surface of the metatest is itself contested. T-09a and T-10a exist ONLY inside an UNMERGED patch block at 11 lines 2719-2727, headed '═══ 编辑 C · §1.6.1 表 · 新增 T-09a / T-10a ═══' with the instruction '（T-09 行之后插入）'. The live S1.6.1 table body runs T-01..T-12 and contains neither. Yet other parts of 11 already cite them as live (S2.2.1 rows at lines 2712-2713, and V-1/V-2 at lines 2857-2858 verbatim '年龄 ≤ 500 ms（T-09a）'). So a bidirectional difference against 'the S1.6 table body' goes red the moment those consumers are honoured, and going the other way means editing 11 -- a design change, CLAUDE.md 9.1. Deciding any of these four is a human ruling, not an implementer's call.

- [ ] 已裁决 · 结论: ______

### INF-CM-4

HALF-BUILT, and the missing half is exactly the contested half -- so do not read the green tests as progress toward this item. WHAT EXISTS AND PASSES: xbrain/common/errors/ (codes.yaml + a dependency-free hand-rolled loader in __init__.py + exceptions.py) and xbrain/common/enums/ (sets.yaml + __init__.py), covered by tests/common/test_error_codes.py and tests/common/test_closed_sets.py, both green inside the 234-passing run. Those tests already deliver criterion (1) (test_every_code_has_a_valid_retryability, plus detail_requirement kept three-valued required/implied/unspecified, plus test_group_assignment_matches which catches a code moving between S13.x sections), criterion (2) (test_out_of_set_value_raises in both packages; UnknownErrorCode and ClosedSetViolation share the XbrainError base; enums/__init__ refuses aliasing and case-folding by design), and the no-counts rule (test_no_count_is_written_into_the_yaml in both). WHAT DOES NOT EXIST: scripts/gen/ is absent entirely, so the code generator scripts/gen/closed_sets_gen.py named in the criterion has not been written; there is no C++ side at all -- `find common -type f` returns only common/include/xbrain/digest/canonical_digest.h, so there is no errors.h and no enums/*.h. Consequently criterion (3) 'Python 与 C++ 导出的集合逐元素相同' is unverified and its mutant '改 C++ 侧一个枚举值 => (3) 红' cannot be run today. THE BLOCKER IS A RULING, NOT THE MISSING CODE. The TODO's own corrections appendix row 20 marks this item three-star and says the ruling must land BEFORE anyone writes: '开工前先拍板一句话并落进 docs/: 真源 = 【手写 codes.yaml】还是【11 表体】'. The conflict is concrete and it is live on disk right now. CFG-CM-1 wants codes.yaml hand-written and diffed bidirectionally against 11, which is a real falsifiable assertion -- and that is precisely the shape the current tree implements: codes.yaml is maintained by hand (there is no generator to have produced it) and test_error_codes.py test_closed_set_symmetric_difference_is_empty compares it against 11's S13.4~S13.15 table bodies, so both named mutations (add E_GHOST / delete E_STORAGE_CORRUPT) genuinely go red. INF-CM-4 instead wants codes.{py,h} GENERATED from the 11 table bodies. Implement INF-CM-4 as written and that existing metatest degenerates into comparing a generated artifact with its own generation source -- empty difference by construction, CLAUDE.md 3.2 form 7 -- i.e. writing this item DESTROYS the strongest assertion Phase 0 currently has. MOT-CM-1 supplies a third answer again ('Python 侧与 C++ 侧的每一个闭集从【同一份数据文件】生成'). Appendix row 20 lists eight items riding on this one ruling (CFG-CM-1/2/3/4/5, INF-CM-4, BIZ-CM-0, MOT-CM-1) and notes the dependency chains are already wired for the codes.yaml answer while INF-CM-4's depends-on column is empty, so 'whoever writes first sets the tone'. I grepped docs/99-决策记录.md for codes.yaml / closed_sets_gen / 单一真源: no ruling recorded. One smaller thing to settle in the same breath, resolvable from CLAUDE.md 0.2 but worth stating so nobody guesses: the criterion writes the outputs as common/errors/codes.{py,h}, and CLAUDE.md 0.2 makes common/ deployment binaries and headers only with Python source in xbrain/ -- so codes.h belongs under common/include/xbrain/errors/ (matching the digest precedent) and nothing .py may land in common/.

- [ ] 已裁决 · 结论: ______

### INF-DB-3

需要人裁的冲突有一处, 且冲突解决后本项仍无处落脚. (1) 冲突: 判据 (1) 要求断言 A09~A12 与 C07 的原地转向一律 E_BUSY + detail 三项必填, 逐字取自 21 S1 V-33 第四列; 而 18 S3.0B 的两条拒绝路径表把今天的路径定为 RJ-1 = E_CAPABILITY + detail.item = "rotation_clearance", 同节逐字写着 '两条拒绝路径必须分开, 绝不可合并成一个码', 并给出兜底规则 '若 Ack 只给到 E_BUSY 一个码, 按 RJ-1 话术兜底'. 选哪条分支取决于 r_robot: 12 S11 失效表逐字 'r_robot 已标定 (rns.inflation.r_robot_m = 0.482, 2026-08-04)', 18 S3.0A 逐字 'r_robot 仍为 0.0 占位 (待云深处)', 而实测 configs/ 全树 grep r_robot_m 零命中 -- 键根本不存在, 15 个 yaml 全是纯注释骨架. 三册对同一个'今天的取值'给了三种状态, 实现者必须先被告知测试该断言哪个码. 我不裁 (CLAUDE.md 9.1). (2) 即使裁完也无处落脚: 判据点名的消费方今天全部不存在 -- xbrain/ 下只有 common/{config,digest,enums,errors} 四个包; perception 的 bands.left/right 生产方 (C++, 19 册) 未写; P1 的旋转许可 RCG (12 S6A) 未写; P4 的 A07~A12 / C07 / E02~E04 意图处置未写; 而 (2) 要的'preset_effective 为 null 时整栈拒绝启动'需要冻结线的启动断言 A 执行体, xbrain/common/config/resolved.py 头注逐字声明 'it does not run the startup assertions'(归 xbrain-config-freeze.service), 该单元今天不存在, ptz.caps.preset_effective 这个键 (11 S7.4.8 的 yaml 块指名落在 configs/p2_core.yaml) 也不存在. (3) 变异体: 四个里三个 (只看前半环的豁免 / 把未知格当空闲 / 用 accepted 冒充已到位) 必须注入一份已存在的实现才能变红, 今天不可能跑 -- 我没有跑, 也不声称跑过. 只有第四个 (扫描文档或代码里给三档写度每秒当量) 今天可写, 但它与 Phase 0 的 GWY-P4-23 判据 (2) 重叠 (该项逐字含 'E10 短路: T-PTZ-3 未闭环时恒 rejected 并引导改用 E01 三档'), 且做成扫 docs 的静态判据会踩 21 S0.4 DBT-2 记过的判据自伤. (4) 注意本项不是 BLOCKED_EXTERNAL: 三条欠账的实测 (V-33 需云深处 + 现场, 11 T-PTZ-1 需人眼, 18 T-PTZ-3 需外部量测) 本来就不该闭环 -- 要写的正是它们未闭环时的 fail-safe 分支, 值保持 null 是设计. 卡住它的是上面的裁决与依赖, 不是那三个测量.

- [ ] 已裁决 · 结论: ______

### INF-DB-4

拆成两半: 能力守卫那一半今天可写, 闭集校验那一半被一处跨册冲突卡住, 整项按 BLOCKED 计. (1) 可写的一半 (今天不缺任何输入, 与云深处无关): xbrain/common/errors/ 导出 capability_guard(), 配一张从 21 S3 第四列提取的不可用能力表 (V-45 特技类 E_NOT_IMPLEMENTED, V-47 E_CAPABILITY, V-48 E_CAPABILITY + warn, V-49 E_NOT_IMPLEMENTED, V-54 楼梯步态趴下 E_CAPABILITY, V-59 非交集下发 E_CAPABILITY), 加一条与该表体的双向差集元测试, 加变异体'让某不可用功能返回 accepted 必须变红'. E_CAPABILITY 与 E_NOT_IMPLEMENTED 两个码今天已在 xbrain/common/errors/codes.yaml 内 (实测命中), tests/common/ 全绿 (234 passed), 所以这半只依赖已落地的 INF-CM-4 Python 侧. 阻塞列写的 V-08 / V-45 / V-59 待云深处, 不阻塞这半. (2) 卡住的一半 -- 判据 (2) 逐字要求'底盘故障码闭集外的值必须抛 E_SCHEMA 并落 event/fault/chassis', 取自 21 S2 V-08 第四列逐字'未登记码一律抛: 遇闭集外值回 E_SCHEMA + 落 event/fault/chassis'; 而 13 S1.3 八条不变量之一 QD-6 逐字'未登记的底盘枚举值 / 故障码一律进 unknown 并上报, 不得 E_SCHEMA 丢弃 (S7.3)', 13 S7.3 标题逐字'故障码: 开放集设计', 正文画出'闭集实现的后果 (必须避免) ... E_SCHEMA 拒绝 ... 把底盘的真实故障吞掉'. 第三种读法在 11 S9.8.4: CF-1 只把前缀形态 (^(chs|chg):0x[0-9A-Fa-f]{4}$) 违规判 E_SCHEMA, 对未登记的数值码一个字没写, 同节逐字'完整故障码枚举待云深处提供'. 99 全册 grep V-08 零命中, 所以 21 S0.2 的'两侧冲突以 99 为准'仲裁不适用. ChassisFault.faults[].code 到底算闭集 (走判据 2 必抛) 还是开放集 (走判据 3 标 unknown) 必须由人裁 -- 而这正是判据 3 与判据 2 方向相反的那条线, 本项自己的风险行也承认'实现里必须显式区分这个字段是闭集还是开放集'. (3) 第二处较小的裁决: 判据 (1) 要求'detail 必填项齐全', 但 11 S13.13 J 组表没有 detail 列, 对 E_CAPABILITY / E_NOT_IMPLEMENTED 一字未提 detail; 13 S12.1 的 V-47 / V-48 / V-49 / V-54 行只写'回 E_CAPABILITY'; xbrain/common/errors/codes.yaml 两码的 detail 字段现值都是 unspecified, 其头注也把这一条如实标为待契约侧回答. 照现文写, 这个子句是一条空壳实现也能通过的断言 (CLAUDE.md 3.2 形态 1). (4) 不是 ALREADY_DONE: grep capability_guard 在 xbrain/ 与 tests/ 零命中, tests/debt/ 不存在.

- [ ] 已裁决 · 结论: ______

### CPP-CXX-2

Split: the envelope half is fully specified and would be writable; the 单调钟 half has three claimed homes and needs a human ruling. Specified half -- 11 S3.0 通用信封 gives all eight fields verbatim (v / rid / ts / mono / boot / seq / src / ts_sync plus data), CLK-A2 verbatim 填写信封 ts_sync 时直接抄最近一次 ClockStatus.sync, CLK-A3 verbatim 若进程 >= 5 s 未收到 ClockStatus, ts_sync 一律填 false (fail-safe) ... 该 5 s 用单调时钟计 (CLK-C1), and boot is 发布主机 boot_id 的前 8 位十六进制 (/proc/sys/kernel/random/boot_id). Nothing is missing there and mutation (1) and (2) are both writable. Blocking half -- this row's deliverable list includes mono_clock.h in xbrain/common/rtcomm/, but CFG-CM-12 (Phase 0, xbrain/common/clock/, language Python + C++17 + Shell) verbatim 导出 mono_now_s() 与 MonoClock, and INF-CM-3 (Phase 0, same dir) verbatim 导出 now_mono() (Python time.monotonic() / C++ steady_clock). Appendix item 25 registers the CFG-CM-12 vs INF-CM-3 clash as 同一模块, 导出符号名冲突 and rules the name to mono_now_s() in common/clock/ -- but it says nothing about this row's third C++ monotonic wrapper in rtcomm/, so the ruling does not cover it. Two C++ steady_clock wrappers in two directories is the same 架构分歧 the appendix treats as needing a per-pair 目录归属 ruling (item 3), and CLAUDE.md 3.4 exists precisely so there is one monotonic discipline. I will not pick the home myself (CLAUDE.md 9.1). Secondary: nothing C++ is buildable under xbrain/common/ yet -- no CMakeLists.txt exists anywhere in the repo -- and this row declares depends_on CPP-CXX-1, which is itself blocked. Criterion (3) (审查脚本 grep 出两个 seq++ 点) is writable but note the scan surface is empty today (ros2_ws/ contains zero files), so it must print per-directory file counts the way CFG-CM-11 note (3) already requires, or it is a green-forever assertion.

- [ ] 已裁决 · 结论: ______

### CPP-CXX-4

Three things need a human, one of them registered by the document itself. (1) Ownership of the QoS profile table is unresolved. Appendix item 4 of this same TODO (the 判据订正 section, 开工时逐条改) states verbatim that CPP-CXX-4 (common/rtcomm/qos_profiles.h) overlaps INF-ZN-3 on 'QoS 档位表由一份常量表导出' and that with CFG-BT-16 (common/qos/), INF-ZN-6 and CFG-CM-17 (common/zenoh/) the table would exist in three directories, 直接违反 11 S2.4.5 rt_override 硬编码在实现中 所预设的唯一真源. Its 改法 is to merge the rows, unify on common/zenoh/ (Python) and have common/rtcomm/qos_profiles.h be GENERATED rather than hand-written -- that rewrite has NOT been applied to the Phase 0 table, so building this row as written produces exactly the violation the appendix names. Which module owns the table, and generated vs hand-written, is a scope ruling I must not make (CLAUDE.md 9.1). (2) Criterion (2) requires 自检脚本遍历五个 C++ 进程的 pub/sub 声明 ... 逐条报出文件与行. ros2_ws/ contains zero files today and CPP-CXX-0 (the workspace skeleton) is a Phase 3 row, so that scan surface is empty; fixtures can make the four rules fire, but the criterion as written would otherwise be CLAUDE.md 3.2 forms (1) and (6) -- a scan that passes because it scanned nothing. (3) Mutation (b) (把 rt_override 做成可由部署配置关闭 ⇒ 审查必须报) has no executor: appendix item 27 names CPP-CXX-4 among the 22 rows whose 'XX 审查必须报' criteria 一次都红不了, and lists 'rt_override 不得可配置关闭' among the ~20 rules no TODO row delivers an executor for. Per CLAUDE.md 3.3 that mutation cannot be run as stated, and I am not claiming otherwise. The design inputs themselves are present and fine (11 S2.4.2 档位表(冻结) with Q0/Q1/Q2/Q3/Q3-rt/Q4, S2.4.3 QOS-C1 rt/ 前缀下的任何 key congestion_control 一律为 drop 不得为 block, S2.4.5 rt_override object row 硬编码在实现中, 配置文件里的值仅供审计比对, and the four startup self-checks at S2.2) -- the blocker is ownership, not facts. Minor env note: only python zenoh is installed; there is no zenoh-c header on this box, so a C++ profile table expressed in zenoh-c types could not be compiled here today.

- [ ] 已裁决 · 结论: ______

### CPP-CXX-5

There is an UNREGISTERED verbatim contradiction inside 11 about the exact rule this row exists to enforce, and it lands on the estop path. 11 S3.0.1 (校验的方向性) says 豁免范围仅限 xbrain/{rid}/cmd/estop 一个 key, 不得扩展到任何其他 key -- 一旦扩展, 畸形包就能触发放松型动作, and the S2.2.3 cmd/estop row repeats 解析 fail-safe (S3.0.1), 豁免 v 校验, 仅此一个 key. But 11 S10.3.7 RQ-3 says op = "cancel" 属收紧型 → 豁免 v 校验, 解析出 cancel_of 即执行; 连 cancel_of 都读不出 → 按 cancel_all 执行 -- and that is a SECOND key, rt/behavior/request. So the contract both forbids and performs the extension. This is not a wording nuance for this row: the whole deliverable is the 仅此一个 key rule and mutation (a) is 把 v 校验豁免扩到第二个 key ⇒ 仅此一个 key 审查必须报, which as written would fire on the contract's own RQ-3. Either parse_guard special-cases rt/behavior/request (contradicting S3.0.1's 不得扩展到任何其他 key) or behavior_proxy cannot implement RQ-3 (CPP-BP-1 depends on it). I verified the clash is not already logged: grep -F 'RQ-3' gives exactly one hit in 11-接口契约.md and zero hits in 99-决策记录.md. Per CLAUDE.md 9.1 I am not resolving it. Two secondary blockers: mutation (a)'s 审查必须报 has no executor -- appendix item 27 names CPP-CXX-5 among the 22 rows whose review-style mutations 一次都红不了, and lists ' 仅此一个 key 校验豁免不得扩大' among the ~20 rules no row delivers an executor for; and criterion (2) (闭集越界如 stop_reason="foo" 必须抛) needs E_SCHEMA and the stop_reason closed set as C++ constants, which do not exist yet -- xbrain/common/enums/sets.yaml has stop_reason (ordered: true, source 11 S4.1) on the Python side only, and CLAUDE.md 3.5 plus the CI gate forbid spelling "E_SCHEMA" as a literal, so this waits on CPP-CXX-1. The directionality design itself (S3.0.1's 收紧型 / 放松型 table) is complete and I read it verbatim; the block is the scope contradiction, not a missing fact.

- [ ] 已裁决 · 结论: ______

### CPP-BP-4b

判据(1)的第二个比较项在十六册内【无任何键可指】-- 这不是'未标定', 是文档没写, 需人裁。逐字核对: 判据要求 'spin.rotational_acc_lim <= P1 §8.1 ② 的角加加速度限'; 而 12 §8.1 ② 逐字只有一行伪代码 '② 加加速度限幅   |v - v_prev| <= a_max · dt, 防止阶跃指令冲击本体', a_max 在 12 全册命中数为 1(就是这一行), 不是配置键; 11 §9.6 spec 块逐条只有 max_vx_mps/max_vy_mps/max_wz_radps/max_accel_mps2/max_decel_mps2, max_accel 是【线】加速度, 无角加速度项; 对 docs/*.md 全文(扫描面 = docs/ 下全部 markdown, 未截断)跑 grep -rnoE 'max_(wz|w)_?(dot|acc|accel)[a-z_]*|angular_acc[a-z_]*|alpha_max[a-z_]*|max_ang[a-z_]*' 得零命中。⇒ 判据点名的变异体(c)'只比 max_rotational_vel 不比 rotational_acc_lim ⇒ 只改 acc_lim 的坏配置必须仍被抓到'今天【不可能实现】, 因为右操作数不存在; 只写速度那一半会得到一条自称覆盖 PX-1 而实际漏掉一半的断言(CLAUDE.md §3.2 形态①)。需人裁的具体问题: 给 P1 侧角加加速度限一个键路径并定层(spec.* 还是 p1_motion 私有), 或裁定 PX-1 的 acc_lim 半边作废。★★ 本表该行阻塞列逐字写着'断言结构可写, 求值阻塞在标定', 这句【只对 max_wz_radps 那一半成立】, 对 acc_lim 半边不成立, 建议同批订正该行。★ 可写的一半确实存在: max_rotational_vel <= common.spec.max_wz_radps 的比较结构与变异体(a)(b)可以今天写, max_wz_radps 保持 null 并按变异体(b)'null 不是满足, 应连同断言 A 一起拒绝启动'落实 -- 但左操作数所在的 configs/nav2/behavior_only.yaml 是空骨架且键名受 D-45/PN-e 阻塞(11 §10.3.2 逐字'本表给 Jazzy 形态'), 该文件又被 CFG-CF-8 认领(双主)。★ 另: 本项与 CPP-BP-5 的执行体都落 scripts/ 且两行都未点名具体脚本文件(触 CHK-1-48 规则⑥'无执行体'), 开工前须各自定名以免撞车。★ 现状: scripts/ 下 grep 'PX-1|max_rotational_vel' 零命中, 非 ALREADY_DONE。

- [ ] 已裁决 · 结论: ______

### CPP-BP-5

MR-1 的左右两个操作数今天都落不了地, 两处都是跨册分歧需人裁, 不是缺一个数。(1) 右操作数不存在且归属未裁: 判据写 common.safety.d_safe_m, 而 10 §5.4.4 断言 N 逐字只覆盖 'p1_motion.corridor.margin_base_m 必须恒等于 p1_motion.safety_distance.d_safe_m', 并在同格逐字写着'若将来 14(档位准入)或 17(HMI 画安全圈)也要读 d_safe_m, 按 §5.4.5 判据须上提为 common.safety.d_safe_m ... 上提与否是分层决策, 本轮不擅自定'; 12 §12 该键块逐字'按 10 §5.4.5 的判据「只有 1 个进程读 ⇒ 留进程私有」: 读者只有 P1 ⇒ 不进 common'(登记 12 §15 #28)。实测 grep -rn d_safe_m configs/ 零命中, grep d_safe_m docs/10-顶层设计.md 仅 2 命中且均为 p1_motion 私有路径 ⇒ common.safety.d_safe_m 这个键路径今天在 configs/ 与 10 §5.4.5 对照表里都不存在, 谁建、建哪一层未裁。(2) 左操作数位置与本行目录列相左: 本行目录列写 'configs/nav2/(nav2_proxy 段)', 而 12 §4.6 裁决 RC-D7 逐字把'rotation_clearance 置于 nav2_proxy: 之下且首行 enabled: true'判为须改的形态, 现行为'顶层独立段, 无 enabled 开关', 理由逐字'旋转许可的出口层作用于全部行为源, 不只 Nav2 委托'; 11 自己承载 MR-1 的【12-F】F-1 补丁块块首也逐字写着'本子块【整块过期】, 以 12 §12 的 rotation_clearance 段为准 ... ② 本段【不是】nav2_proxy 的子段'。实际落点是 configs/p1_motion.yaml 顶层 rotation_clearance 段(12 §12 该 yaml 块首行逐字 '# /opt/xbrain_v6/configs/p1_motion.yaml')。(3) 最要害的一条: 11 §10.3.2 MR-1 下方逐字'与既有断言的分工, 不重复实现: 10 §5.4.4 断言 N 已覆盖 margin_base_m == d_safe_m 这一对; 12 §12.1 S-4b 已覆盖 P1 进程内三键相等。MR-1 覆盖的是【契约册自己写出去的那份 nav2_proxy 配置】-- 它是三者中唯一没有被前两条罩住的落点'; 而这个落点恰恰已被 RC-D7 取消 ⇒ MR-1 今天没有独立求值面。照本行原样实现只有两种结果: 去 configs/nav2/ 里找一个根本不存在的键 ⇒ 恒绿(CLAUDE.md §3.2 形态①), 或与 12 §12.1 S-4b(rns.inflation.margin_base_m == rotation_clearance.margin_rot_m == safety_distance.d_safe_m)重复实现, 违反 11 的'不重复实现'逐字。⇒ 须人裁: MR-1 的求值面落在哪个文件、右操作数用 common.safety.d_safe_m 还是 p1_motion.safety_distance.d_safe_m、以及它与 S-4b 的分工。★ 好消息: 三个变异体 M-MR1-a(0.30 必红) / M-MR1-b(1.50 必红, 区分恒等与 C-6 的 >=) / M-MR1-c(两值同改 0.30 时 MR-1 必须判绿, 该红的是 12 §12.1 S-4 的 d_safe_m >= 1.0)在 11 §10.3.2 逐字齐全且已核对, 落点一旦裁定, 实现加三变异体是小活。★ 另: 11 与 12 关于 margin_rot_m 取值 0.30 vs 1.00 的旧冲突【已闭】, 11 §14 S8 行逐字'已裁决 · 已修', 现行两册均为 1.00, 本项不受该冲突阻塞。★ 现状: xbrain/ tests/ scripts/ 内 grep 'MR-1|margin_rot' 零命中, 非 ALREADY_DONE。

- [ ] 已裁决 · 结论: ______

### CHK-1-07

值不缺, 缺的是断言的家. 三处需要人拍板: (1) 00 §5.2.1 PAY-11b 逐字要求 ptz.tilt_limit_deg 进 CFG-11 安全参数范围断言, 但本轮自查 grep -rn tilt_limit docs/*.md configs/ 只在 00 命中三处(§0.4a③表/§5.2.1/§22 T-PTZ-3), 11/12/14/17/18/configs 全零命中; 落地 CFG-11 的表是 12 §12.1(标题逐字 '安全参数断言表(v0.3 新增 -- 落地 CFG-11)'), 该表逐字自封为 'S-1 ~ S-6 也是六条', 而 tilt 属 P2 云台域, 14 册内同样零命中 -- 这条断言在十四册里没有编号也没有归属册. (2) 判据①要求登记进 CFG-FZ-7 的 ASSERT_REGISTRY, 而 CFG-FZ-13 的判据逐字把规范面写死四行(SP-*→11 §9.6 表; S-*→【只有】12 §12.1 的 S-1~S-6; QC-*→13 §8.3; AS-*→【只有】AS-7)并要求正反差集均为空 ⇒ 多登记一条无编号断言必然让该元测试反向差集非空. 是给它开 SP-*/S-* 新号(改 11 或 12), 还是扩 CFG-FZ-13 的豁免清单, 属设计裁决, CLAUDE.md §9.1 明令实现者不得自决. (3) 变异体①要求 '外扩必须伴随实测证据字段' 才放行 max: 90.0, 但十四册无此字段定义: 唯一候选 PROVENANCE 在 docs 内仅 2 命中且无取值域定义处, 其护栏出处 SET-01 全库零命中 -- 这正是 CFG-DC-5(Phase 1, 需用户裁定)列的悬空约定. 另: 判据②的消费者是 P2 域⑤ 出口层(BIZ-P2-9, Phase 2), 今天 xbrain/p2_core/ 不存在, 而条目自己逐字写 '只有配置项/没有消费者 = 一条永远绿的断言', 因此只交配置半也不算完成. 唯一今天确定不缺的是数值本身(-90/+30 由 PAY-11b 逐字给出), 🚫 不需要任何实测就能写.

- [ ] 已裁决 · 结论: ______

### CHK-1-57

两处必须人拍板, 且都直接决定判据能不能求值. (1) 落点与轮转规范在十四册内无出处: 本轮自查 grep -rn 'data/logs' docs/*.md 只在 XBRAIN_V6_TODO.md 自身命中, 10/11 两册各 0 命中; 条目自己的出处行也逐字承认 '全库对日志的规范只有 10 §10.1 三类可观测数据表第三行'(仅写 '各进程运行日志 | 本地轮转(P1 异步落盘)'), 既无路径也无格式也无保留期与 common.retention.* 的关系. 而 CFG-DC-5(Phase 1, 需用户裁定)第③问逐字就是 'data/ 的目录结构与日志落盘规范(路径 · 轮转 · 保留期与 common.retention.* 的关系)在十一册内无落点, 须先补文档再写代码' -- 本条目要写的正是那段代码. (2) 判据①的'十个进程名各取一次 logger, 落点与格式逐字段匹配金标正则'无法求值: 10 §3.1 常驻进程表里 Python 只有五个, 且名字是 xbrain_motion(P1)/xbrain_core(P2)/xbrain_task(P3)/xbrain_agent(P4)/xbrain_gateway(P5), 与 CLAUDE.md §0.1 用的 p1_motion/p2_core/p3_task/p4_agent/p5_gateway ＋ payload-service 是两套互不相同的写法; 进程名既是 data/logs/{proc}/{proc}.log 的路径段又是金标正则字段, 十个是哪十个/用哪套命名, 十四册没有闭集. 自拟一份名单等于把①做成一条按构造不可能红的断言(CLAUDE.md §3.2 形态①). 附带一条今天也跑不了: 判据③要求'ctrl 路径注入 get_logger().info(...) ⇒ MOT-PM-2 判据①必须变红', 而 MOT-PM-2 在 Phase 2, xbrain/p1_motion/ 今天不存在. 可写而不受阻的只有(不构成 GREEN): 队列＋后台线程的非阻塞落盘、WatchedFileHandler 重开、非 ASCII 即抛这三条的实现骨架; 级别一项还需先定 XBRAIN_LOG_LEVEL 是直读 env 还是读 resolved 的 common.log_level(现有 xbrain/common/config/layers.py 的 ENV_KEY_MAP 已把它映射到 common.log_level, 直读 env 会绕开 10 §5.4.1 '运行期只读产物').

- [ ] 已裁决 · 结论: ______

### CHK-2-25

本项标题写「两处【已收口】键名」, 实测只有一半成立。(1) rot_occ_max_cells => rot_occ_max 确已收口: 12 §6A.3.2 判据行与 §12 配置块逐字 `rot_occ_max:            0`, 11 §13.8 ⑧ 与 §10.3.2 RC-1 同名。(2) fail_ticks => recheck_ticks 【未收口, 需人裁】: 11 §14.4 G-24 逐字「11 侧已统一为 fail_ticks; 仍开 · 归 12: §12 的 recheck_ticks 须改名 fail_ticks」, 而 12 §15 #43 逐字「本轮不单方面改名」「在裁决落地前, 实现一律以本册 §12 的 recheck_ticks 为准」, 并指出只改叶名会得到第三种写法 (顶层 rotation_clearance.fail_ticks, 因 11 把它挂在 nav2_proxy 下而 12 按 RC-D7 挂顶层)。把 fail_ticks 做成【拒绝启动】的黑名单键, 等于替 G-24 拍板, 属 CLAUDE.md §9.1 必须停下问人。须裁两点: G-24 最终取哪个名; rotation_clearance 段挂顶层还是 nav2_proxy 下。(3) 即便裁决落地, 载体也还不在: 判据 ① ④ 的「断言 B 判疑似复制/别名并拒绝启动」落点是 CFG-FZ-4 (TODO Phase 1, 目录 scripts/), 全仓无任何 freeze/断言实现 (xbrain/common/ 只有 config·digest·enums·errors 四包); configs/p1_motion.yaml 今天 0 个有效键 (纯注释骨架), 判据 ③ 的反向变异体「rot_occ_max: 0 + recheck_ticks: 3 必须通过」无对象可通过, 键位由 Phase 0 的 MOT-PM-33 落。(4) 判据 ② 的扫描面 xbrain/p1_motion/**/*.py 今天 0 文件, 命中数恒为 0, 单独交付即 CLAUDE.md §3.2 形态① 的永远绿断言 (与 CFG-CM-11 实测③「目录没文件不得冒充扫过且干净」同型)。

- [ ] 已裁决 · 结论: ______

### CHK-2-35

拆开看: 【可写的一半】18-A §2 的五行说法扩充表在册内逐字存在 (grep -F '打开探照灯' docs/18-A-语音指令扩展.md 命中), 判据 ③「说法清单不得手写第二份, 由表体抽取做双向差集」可实现, scripts/doccheck/intent_keyword_sync.py 已是同型脚本可参照。【不成立的一半, 须人裁】判据 ② 逐字要求「『音量恢复正常』⇒ level == 配置缺省值 (从 resolved 产物取, 不在代码里写默认数)」, 而【十四册内根本没有这个配置键】: grep set_volume docs/*.md 只命中 18 §6.4 (level 值域 0-100)、18-A §2、11 §8.8 AudioCommand{kind:set_volume} 与 TODO 的 GWY-P4-28, 无任何一处定义「音量缺省值」的键路径; grep 音量 docs/10-顶层设计.md 零命中, 即 10 §5.4.5 对照表未登记该键; 14 §12 里唯一的 volume: 80 是 BIT announce 的播报音量, 不是 D10 的持久音量缺省 (11 §7.5 volume_pct)。⇒ 实现者到这一步只能自造键名与数值, 正是 CLAUDE.md §3.1/§9.1 禁止的。另有两条须一并裁: (a) 18-A 封面状态逐字仍是「v0.1(讨论稿) · 待评审」, Q-18A-1/Q-18A-2 未决; (b) 18 §6.2 PL-4 逐字「D10 set_volume 今天不可执行」(依据 11 §7.5A.4 G-1: payload-service 无 volume 端点), 本期处置是 rejected + E_CAPABILITY ⇒ 判据 ② 的 level 断言只能落在【槽位层】, 与 GWY-P4-28 ⑤ 的拒绝分支怎么分层必须先写清, 否则会写出一条与 PL-4 冲突的「静默成功」路径。【顺带的依赖】判据 ①③ 需要 configs/intents.yaml 里真有 keywords, 今天该文件 0 个有效键, 落值是 Phase 0 的 GWY-P4-07 (configs/ + xbrain/p4_agent/registry/); 判据 ①「灌入路由器」同样需要那个 registry, 而今天 xbrain/ 下只有 common/ 一个包, services/llm/prompt/ 为空。

- [ ] 已裁决 · 结论: ______

### BIZ-CM-5

判据的五条里有四条今天就能写, 但第四条的正向半在十四册里没有落点, 需要人裁, 且判据逐字规定 '只跑封禁半不算写完'. 能写的部分: P-1~P-3 三条义务逐字在 11 S7A.3 '被抢占方的三条义务' 表 (含 '即使清理未完成也要先发 ack'); T-1~T-4 逐字在同节 '超时不退怎么办' 表; T-2 的 detail.overdue_ms 在 11 S7A.7 的 detail 示例里有位; 60s/3 次两个整定值在 14 S11 arbiter 段 (forced_preempt_window_s: 60 / forced_preempt_max: 3); T-4 的 gen 丢弃兜底在 11 S7A.2 G-3. 缺口: T-3 的正向半逐字是 '待其所属进程重启后恢复' (11 S7A.3 T-3 与 11 S13 的 E_ARB_DISABLED 行 '恢复手段是重启该源所属进程' 两处同样表述), 而十四册没有任何东西说明仲裁器凭什么判定 '该源所属进程重启了': 14 S3.2 的 SourceSpec 六个字段里没有 owner/process; 11 S7A.5.1 的 sources[] 只有 source_id/priority/policy/alive 四项; 11 S7A.1 的 ArbRequest 十一个字段里没有进程或实例标识; 11 S7A.4 只说 'p5_gateway 心跳丢失 -> 广播 event/fault/system -> 各仲裁器回收该进程全部源', 但 event/fault/system 的 detail 里没有进程名 (全册 grep '"proc"' 与 '`proc`' 零命中), 信封的 boot 字段 (11 S3.0 '发布主机 boot_id 的前 8 位') 是整机 boot_id 不是进程实例. 候选解释至少三种 -- 重新 register() 即解禁 / 收到 source_death 后再次见到该源即解禁 / 等某个尚不存在的进程实例号变化 -- 三者对跨进程源 (tts_cloud 等由 14 S11 配置注册, 远端进程从不自己 register) 的行为完全不同, 选错的失效方向正是判据自己点名要防的那一种: 一次抖动把语音源永久打死. 按 CLAUDE.md 9.1, 这一条必须先由人在 docs/ 内定死 '什么信号算该源所属进程重启', 不得由实现者自选. 另外本项无论如何也要等 BIZ-CM-1 落地 (xbrain/common/arbiter/ 今天为空目录).

- [ ] 已裁决 · 结论: ______

### BIZ-P2-24

Phase 0 表里只有这一条 BIZ-P2- 项(TODO 第 92 行), 其余 28 条 BIZ-P2-* 全在 Phase 2(第 860-888 行)。现状: configs/p2_core.yaml 与 configs/suspicion_rules.yaml 都是纯注释空骨架(逐字含「本文件当前是【空骨架】」), 一个键都没有, 所以肯定不是 ALREADY_DONE。

【为什么是 BLOCKED_DECISION 而不是 GREEN】本项要交付两份 yaml, 其中 suspicion_rules.yaml 的内容今天写不出来, 因为它卡在一条仍然敞开的跨册冲突上:
(1) SD-1 / Q-P2-21 —— ✅ **已裁决 2026-08-18: 用户拍板载体定为 cmd/fence + poly_id(f- 前缀); cmd/geo 那说作废。理由: 11 SR-2 版本一致性要 (fence_set_id,rev,crc32) 三元组, cmd/geo 的 GeoObject 给不出。落地见 99/14 的 SD-1 与 Q-P2-21 台账(均已关闭)。以下为原始待裁决描述, 存档备查:**<br>14 §6.2A 逐字锚点「SD-1」与 14 §14 的「Q-P2-21」行: 本册 RE-4 说 zone 是 11 §7.8 的 GeoObject(type="zone"), 经 cmd/geo, 规则里写 GeoObject.id(如 gate_guard_post); 11 SR-1/SR-4 说是 cmd/fence 的 FenceSet.polygons[](role="zone"), 规则里写 poly_id(f- 前缀, 如 f-gate_post)。14 §6.2A 的编辑清单第 ① 条逐字「规则 YAML 的 inside_zones/exclude_zones 由名称改为 poly_id(f- 前缀)」标注为「⚠️ 未执行 —— 属 SD-1(载体本身未定), 改标识符而不改载体只会造出半套」, 并写明「🚫 不拆开半套」。写这份 yaml 就必须二选一 = 替评审做决定, 违反 CLAUDE.md §9.1; 且 14 自评该失效形态是「最危险的静默失效」(规则永不命中, 看起来在保护实际不保护)。归属 P0, 归 11 册。
(2) 同一份文件上还有第二处需要人拍板: Phase 0 的 CFG-CF-6 判据逐字要求 suspicion_rules.yaml 带 require_all_day_rule 启动自检(「规则集里若一条不带 time_window 的规则都没有即拒」), 而 14 §6.2A 编辑清单第 ④ 条把 require_all_day_rule / dwell_hold_s / fence_stale_s 三键登记为「⚠️ 未执行」「三键同属一组, 🚫 不拆开半套落地」(分别卡在 SD-3/Q-P2-23 与 SR-3/11 F-j)。两边直接对撞。
(3) 第三处需要拍板的是本项自己的阻塞列与权威册不一致: 本行阻塞列写「M-PAY-3(t_device_ms) · V-28(A 级传感器) —— 这些键在标定前一律写 null」, 而 14 §11 的 yaml 正文逐字是 `t_device_ms: 0`(注「0 是【方向安全】的占位(只会多关麦)」)与 `photocell: enabled: false`(注「待载荷厂商确认是否内置(V-28)」), 且 Phase 2 的 BIZ-P2-2 阻塞列逐字「t_device_ms 先按 0 计入, 方向安全」。写 0 还是写 null 的失效方向完全不同(0 放行启动, null 拒绝启动), 且「给未实测量写 0」正是 CLAUDE.md §3.1 点名的形态。🚫 我不代裁。

【p2_core.yaml 这一半的可写性 —— 供拆分参考】14 §11 给出了完整 yaml 正文(loop / payload_svc / arbiter / mode / suspicion / d_mode / mode_motion / ptz / health.profile_admission / bit 共约 200 行), 未标定项已在册内写成 null(auto.image.on_lux_equiv / off_lux_equiv, 注「待实测(11 M-27 照明标定)」), 所以「照抄键位 + 未标定留 null」这一半在拿到上面三条裁决后是可做的, 但它与 Phase 0 的 CFG-CF-5(六个 L6 进程私有 yaml 落值, 含 p2_core.yaml)和 CFG-CF-6(八个 L6 内容表 yaml, 含 suspicion_rules.yaml)写同两个文件, 排期时必须与它们互斥, 否则两个实现者会互相覆盖。

【即便裁决下来, 验收面今天也不成立】本项判据的执行者是 xbrain-config-freeze.service 跑 10 §5.4.4 断言 A-J 全套, 而: ① 全盘无该 systemd 单元(deploy/systemd/ 只有 xbrain-ai-asr / xbrain-llm / xbrain-payload 三个 AI 服务单元); ② xbrain/common/config/ 下只有 layers/merge/refs/cycles/resolved 五个模块(tests/common 实跑 234 passed), 无任何断言 A-J 实现, 那批是 CFG-FZ-1~CFG-FZ-15(全在 Phase 1, TODO 第 420-434 行); ③ 判据 ② 的右操作数 keys(common.motion.profiles) 不存在 —— configs/common.yaml 至今是纯注释骨架, 该键由 Phase 0 的 CFG-CF-2 提供; ④ p2_core.yaml 里 `ptz.h_camera_m: ${common.calib.frames.ptz_base.h_camera_m}` 的被引方在 L4b configs/calib/{robot_id}.yaml, 该目录今天只有 .gitkeep, 由 Phase 0 的 CFG-CF-7 提供; ⑤ CFG-FZ-17 要求 schema 校验跑在断言 A 之前, 该 schema 资产也不存在。断言 C 的逐字锚点已核实存在于 10 §5.4.4(标题行「#### 5.4.4 启动时的一致性断言」, C 行内「keys(P2.health.profile_admission) == keys(common.motion.profiles)」)。

【我不确定的点, 按要求写明】判据里那三条反向变异体(删一档 profile_admission / behavior 写成 face_target / 在 profile_admission 写入 cruise 或 transit)我没有跑 —— 今天无 freeze 服务、无断言注册表, 结构上跑不了, 所以我不声称跑过。

- [ ] 已裁决 · 结论: ______

### BIZ-P3-23

需要人拍板的跨册冲突, 不是能写代码绕过的缺口. (1) 判据要求「断言 A-J 全套通过」, 但 docs/10-顶层设计.md 断言 C 行的现行逐字子句仍是 `P3.charge.low_batt_profile 属于 keys(common.motion.profiles)`(可 grep 锚点: P3.charge.low_batt_profile), 而 10 §5.4.5 共享参数对照表与 14 §11 profile_admission 两处都把 profiles 定死为 obstacle_avoid / patrol 两档闭集 ==> 15 §12 现行值 `low_batt_profile: disabled` 按构造必被断言 C 拒绝启动. 15 §13 TC-47a 逐字写「本格今天【必须是红的】」, 其转绿期限 = 10 侧按 §14 Q-P3-31 给断言 C 加显式守卫的那一天; 15 §8.2A CR-7 同时逐字禁止走「往 common.motion.profiles 里加一档 disabled」这条路(会同时打红断言 A 与 14 §11 的两档闭集). (2) 该键存在【三方分叉】且三册都写了值: 00 CHG-51 = null, 11 §15A 收口稿 = obstacle_avoid, 15 §12 = disabled; 11 自己已把它登记为未决严重档 S6, 逐字「已登记 · 归 15 路会签」. 三条路今天全走不通(null 撞断言 A, obstacle_avoid 与 15 §8.3 正文正面冲突, disabled 撞断言 C) ==> 没有任何一个可写的半成品形态, 无法按「代码半边先做 值留 null」拆分: 这里没有 loader 半边, 整项就是落值 + 让断言求值. 按 CLAUDE.md §9.1 不得自行裁决, 不得自行给断言 C 加守卫. (3) 现场实测: configs/p3_task.yaml 是纯注释空骨架(无任何键), configs/common.yaml 同为空骨架 ==> 15 §12 里全部 ${common.retention.*} / ${common.db.*} / ${common.recording.*} / ${common.priority.task} 引用无落点; 全仓无 xbrain-config-freeze.service(deploy/systemd 只有 payload / ai-asr / llm 三个单元), scripts/ 与 xbrain/ 内无断言 A-O 执行体 ==> 即使裁决下来, 仍需 CFG-CF-2 与 INF-CM-1 先落地(这一层是 BLOCKED_DEPENDENCY 性质, 不改变本项的裁决性阻塞). (4) 排除 ALREADY_DONE 的证据: grep -rl 'TC-31|TC-47|p3_task' 在 tests/ 下零命中; 实跑 python3 -m pytest tests/common -q = 234 passed, 无一条用例触及 p3_task. (5) ⚠️ 写入面重叠须主会话调度: Phase 0 的 CFG-CF-5(六个 L6 进程私有 yaml 落值)判据逐字含 p3_task, 与本项写同一个文件, 两者不可并发派发. 另有 CHK-2-26 明确要求把 enforce_ordering 黑名单并入本项断言 B, 亦写同一文件.

- [ ] 已裁决 · 结论: ______

### GWY-P4-07

落盘现状: configs/intents.yaml 是 15 行纯注释骨架, xbrain/p4_agent/ 整个目录不存在, 故绝非 ALREADY_DONE. 判据①要求每条同时具备 id/route/auth/slots: 其中 id/slots/auth 今天可从 18 各类指令清单表(§3 A 类表的「# / intent / 槽位 / 确认」四列, §8.1 F 类表同形)与 18 §13.1「逐条枚举(供第三方复核)」表取到; 但 route 无处可取 —— 16 §5.3 的 intents.yaml 样例块里逐字写着「route  ← _triage.json 的 route(bypass | fastpath | fastpath_then_llm | llm)」, §6.6 末又逐字写「完整的 128 行判定表见 _prompt_work/_triage.json(每条含 route + 理由 + 槽位 + latency_class)」。_prompt_work/ 是用户明令删除、不得引用也不得恢复的下划线目录, 实测全仓 find 零命中。16 自身只留下聚合数: §6.6 四格 3/92/20/13 与 §6.6.2 按类分布; 20 条 fastpath_then_llm 在 §6.6.4 被逐条列出, 但 13 条 llm 只有按类计数(B1 C1 D1 F5 G3 H1 I1), 靠散落文字最多能钉住 9 条(B03 C07 H02 I06 与 G11/G12/G14, 加 §6.6.5 提到的 F06/F10), D 类那 1 条与 F 类剩余 3 条只能靠猜 —— 猜就是编造。同时 §0.5 的 CS-A1/CS-A2 断言本身就是写在 _triage.json 与 configs/cmdset_18.json 上的, 而后者 16 §14 逐字「尚未落地 —— 缺失时 CS-A1 退化」。⇒ 需要人拍板: 被删的 triage 表由谁接手当 route 真源(或裁定 18 各类表升格为 route 真源), 属 CLAUDE.md §9.1 第 2 条「文档缺失 ⇒ 停下问用户」。补充: 判据⑤「注册 18 里不存在的意图名 ⇒ CS-A1 红」在 cmdset_18.json 缺席下也只能跑退化版, 立项时要说清扫描面。

- [ ] 已裁决 · 结论: ______

### GWY-P4-23

四条新意图的上游文档今天都还是未评审的讨论稿, 且母本未回流。逐条实测: 18-A 头部状态逐字「v0.1(讨论稿)」「待评审」; 18-B 头部逐字「v0.2(讨论稿)」「待评审」。18-A §6 的 Q-18A-2 逐字把 owner 记为「16 owner」并写明处置「若并不进去, 本文的两条新意图暂缓」—— 也就是 D17/D18 是否落地本身还没定; Q-18A-1(D18 是否保留)逐字写「甲方现场评审」。回流方向由 16 §0.5 CS-1a 逐字锁死「需要改闭集时, 先改 18, 再回流」, 而实测 grep -c set_ptz_speed docs/16-*.md = 0、grep -c ptz_move_deg docs/16-*.md = 0、grep -c ptz_move_deg docs/18-语音文本指令集.md = 0, 母本 18 §2 分类表 E 段仍是 8、D 段仍是 13。落地还会同时推翻 16 §14 的 auth.assert_intent_count: 128 / assert_row_count: 130 / assert_level_hist 三个启动断言常量, 而这三条的对账物 _triage.json 已被删、cmdset_18.json 16 §14 逐字「尚未落地」⇒ 改数没有第二份可机检的数据兜底。⚠️ 判据②那一半反而不是阻塞: 21 §1 对 18 T-PTZ-3 的「未闭环默认行为」栏逐字给了 omega 恒 null 时的行为(E10 一律 rejected + 引导改用 E01 三档; E09 只发闭集档位名, 不得给度/秒当量), 18-B Q-5 同向 ⇒ 短路分支的代码语义今天是写得出来的; 真正卡外部实测的只有 E09 三档的度/秒当量, 而按裁定那本来就不该出现在代码或配置里。⇒ 主阻塞是人拍板(18-A/18-B 评审 + Q-18A-1/Q-18A-2 + CS-1a 回流), 不是测量。

- [ ] 已裁决 · 结论: ______

### GWY-P4-29

裁定这一半是齐的, 语句库那一半没有内容可落。已定的部分: 18 §6.3 的裁定表逐字给出「voice(性别)写进预设语句库中每一条语句的定义」「不新增 D08 槽位」「预设未标 voice 时取 0(男声)」「需携带性别 ⇒ 用 [31], [15] 仅作无性别需求时的兜底」, 协议事实表也逐字写了 [31]/[32] 首字节 = 性别、[15] 无性别位 ⇒ 判据②③④的语义有据。卡住的是判据①的「落值」: 11 §8.8.2 预设语句库编译流程表的最后一行逐字「⚠️ 待确认 | 预设语句条数与内容(VOI-47 要求可配置多条), 待甲方给话术」—— 条数与文本是甲方欠账, 编一批预设句就是编造。字段形状同样未决: 18 §6.3 逐字「speech_presets.yaml 的字段定义归 11 §8.8 / 14 §7.3 …… 不代改 → 登记 Q-18-22」, 而 14 §7.3.3 现行库是 warn_01/warn_02/warn_03 的扁平 map(voice 挂在 d_mode 段, 不在每条上), 与判据①要的「每条含 preset_id / text / voice」形状不一致; 文本本身还有 Q-18-25 未决 —— 实测 11 与 14 写「您已进入管制区域」(11 命中 4 次 / 14 命中 1 次), 18 记录真机跑的是「你已进入管制区域」(18 命中 1 次), 18 §17 Q-18-25 逐字要求「必须统一」但不代决。此外判据⑤要求 preset_id 闭集与 GBNF 产生式同源, GBNF 生成器是 Phase 2 的 GWY-P4-12。⇒ 需甲方话术 + Q-18-22 / Q-18-25 两条裁决, 之后才谈得上落值。configs/speech_presets.yaml 今天是 15 行纯注释骨架, 保持原样即正确。

- [ ] 已裁决 · 结论: ______

### GWY-P5-17

分两半看. 【可写的一半】17 S10.1 的 event / backfill / recon / approval / backpressure / telemetry / delivery / disk / cloud / uplink 十段可逐键照抄, cloud.rtb_s 与 approval.expires_s 按设计保持 null (U-05 / Q-P5-19), 四个【建议】值(rate_eps_total / weight / avg_event_bytes / resume_batch)是文档给出的值不是我方编造; xbrain/p5_gateway/config/ 的加载器可直接架在 xbrain/common/config/resolved.py 上(tests/common 现场跑全绿). 【卡死的一半 = 本项标题与三条 ★★★ 判据】: (1) hmi.bind 与 delivery.ftp.listen_address 写不出合法值 —— 17 S10.1 逐字是占位符 "{LAN2_IP}:8080" / "{WIFI_IP}:8080", 而 11 S1.1.9.2 网段登记表 LAN2 行逐字「待 U-15」, 10 S4.5 逐字「仍缺的取值(唯一阻塞项, 全部落在 11 U-15): LAN2 网段」, 19 PD-2 逐字「甲方网络规划, 我方不得代拍」; 同时 10 S5.4.2 R-1 逐字「禁止字符串内插」, 全库无 common.net.* 键, 十四册未定义任何 {LAN2_IP} 的展开机制 ⇒ 写占位符字面量(运行期 bind 到非法地址) / 写 null(整栈拒启) / 拆成 ip+port 两键(无出处) 三选一是设计裁决, 不是实现细节. (2) 判据 ③ 要求与 11 ND-2 同一脚本跑, 但 11 S1.1.9 ND-2 逐字仍只写「确认 7447 / 7446 未出现在 LAN1 / LAN3 / LAN4 上」, CR-NET-1(端口集扩到 8080) 仍挂在 17 附录 C 未落笔 ⇒ 判据没有契约面. (3) 判据 ①② 今天跑不了: 本机 ip -o addr 只有 lo / eno1 / docker0, 没有 LAN1~LAN4 四口, 且网口归属需要 deploy/net/{hw_profile}.network 与 /etc/xbrain/hw_profile(Phase 1 CFG-BT-20, 尚不存在)才能判定 ⇒ 变异体「把 hmi.bind 改回 0.0.0.0:8080 必须变红」无法执行, 按 CLAUDE.md S3.3 这条断言就不算写完. (4) 额外冲突需一并裁: CR-EVT-1 已落笔(11 S2.2.5 三条 event/replay · event/recon/req · event/recon/rsp 标 v0.7.10 补登记), 而 17 S10.1「全文」的 startup.pending_keys 仍列这三条, 判据 ⑥ 又要求「落笔后清空并断言为空数组」—— 两册不一致, 且 17 S3.5.6 自陈该豁免机制本身仍待 11 S2.2.14 认可. 现状核对: xbrain/p5_gateway/ 不存在, configs/p5_gateway.yaml 仍是纯注释骨架 ⇒ 排除 ALREADY_DONE.

- [ ] 已裁决 · 结论: ______

### MOT-PM-33

Phase 0 内以 MOT-PM- 开头的只有这一条(TODO 第 159 行表内, 第 379 行明细; Phase 0 共 73 行任务, 已逐行核过 id 列)。四处必须由人拍板, 实现者写到一半必撞: (1) 判据 (1) 逐字要求解析 12 §12 的 YAML 块, 但该块今天解析不了 -- docs/12-P1运动域详细设计.md 的 §12 只有一个 yaml 围栏(3059~3459), 其中 rotation_clearance 段逐字为 margin_rot_m:           *d_safe, 而同块 rns.inflation.margin_base_m 行注已写 v0.5 去锚点并改成字面量, 即锚点 &d_safe 已删除, 别名 *d_safe 漏删, yaml.safe_load 抛 ComposerError: found undefined alias 'd_safe'(我实跑确认)。修它要改 12 册一条安全键的写法, 且 12 §15 第 42 条仍逐字声称 v0.4 锚点在位, 属册内自相矛盾。手工替成 1.00 后可解析, 得 15 个顶层段 145 条键路径, 与 TODO 明细条写的九段亦不符。(2) d_safe_m 的分层归属是明写未决: 10 §5.4.4 断言 N 逐字'上提与否是分层决策, 本轮不擅自定'; 11 §10.3.2 MR-1 逐字要求 margin_rot_m 恒等于 common.safety.d_safe_m; 12 §15 第 28 条定为 P1 私有 safety_distance.d_safe_m; grep -rn d_safe /opt/xbrain_v6/configs/ 今天 0 命中。这条裁决决定本文件写字面量还是写 ${common.safety.d_safe_m}, 直接改变判据 (1) 比对的键路径集合。(3) 两册键路径对不上: 10 §5.4.4 断言 N 左操作数是 p1_motion.corridor.margin_base_m, 12 §12 实际放在 rns.inflation.margin_base_m(rns.corridor 段只有 ray_count/max_range_m)。(4) 阻塞列点名要写 null 的键里, rns.corridor.k_head_per_rad 与 margin_max 在 12 §12 中都不存在(后者只在 fence 段注释里出现, 取值等甲方 W_road, 11 §3.2.1 U-01), 补进 configs 就让判据 (1) 变红, 不补就违反阻塞列。另: 145 条键路径里只有 teleop.cloud.enabled / teleop.cloud.max_linear_mps 两条是 null, 其余全是字面量, 块内仅 5 处带待实测/待整定标记, 无 configs/safety/brake.yaml 那样的逐键 PROVENANCE, 而本项风险注逐字说文档正文里的 2.5 / 0.35 / 1.20 / 10.0 是推演用的工作假设 -- 逐键判'已定 vs 假设'正是不许自决的事。已就绪的部分: 12 §12.2 三条修法(profiles 段整删 / fence.margin_soft_m 整删 / speed_gate 三键上移 common.safety.*)在 12 §12 里已落地, 标题后半句无活可干; 判据 (2)(4)(5) 的检查脚本骨架今天可写。configs/p1_motion.yaml 现为 14 行纯注释 0 键, 无任何现存脚本覆盖它(scripts/doccheck 下无对应项), 故 ALREADY_DONE 排除; tests/common 基线 234 passed。

- [ ] 已裁决 · 结论: ______

### MOT-PCP-15

现状核实(判据⑤已复核): configs/perception.yaml 不存在; configs/models/m20s.yaml 是 16 行纯注释、0 个有效键; scripts/ 与 tests/ 全文无 free_space_corridor / perception.yaml 命中 => 不是 ALREADY_DONE。阻塞点三条, 任一条单独成立即足以卡住:

(1) 键命名空间两册逐字相反。`11` §3.1.4 ⑩ 2026-08-05 终审 S22 订正后逐字给出「common.motion.free_space_corridor.{rev, frame, x0_m, ...}」并逐字禁止「不得再写成顶层 free_space_corridor:」; 而 `19` §14 的 YAML 块正是把它写成 configs/models/m20s.yaml 的顶层 free_space_corridor:。本条判据① 把 `19` §14 解析出的键路径集合当作双向差集的左操作数 => 照 `19` 写, 文件违反 `10` §5.4.3 L2 行逐字「本层允许写入的命名空间 = common.spec.* · common.motion.*」(顶层出现非 common 键按 L1 同规则拒); 照 `11` 写, 判据① 恒红。二选一都要改一册的正文逐字, 属 CLAUDE.md §9.1「文档矛盾 -> 停下问用户」。

(2) configs/perception.yaml 在整个文档包内未登记, 且用户已把同源缺口登记为待决。扫描面 = docs/ 下 00/10/11/12/13/14/15/16/17/18(三册)/19/20/21/99 全文, 逐册计数: 「perception.yaml」仅 `19` 命中 5 处、`99` 命中 2 处(都在 U65 缺口表内), `10` 与 `11` 零命中。`10` §5.4.0 ⑤ 表(该节自称「配置路径在整个文档包内的唯一权威定义处」)的 15 个固定文件名里没有它; §5.4.3 L6 行的六个文件里没有它; §5.4.4 MANIFEST.processes 的六项里没有它。`99` 「U65 · perception 快照登记缺口」逐字「【登记为待决】, 不改代码(2026-08-06 · 用户拍板『先只登记为待决』)」并逐字警告「不得被后来者读成『已决定维持六项』」。新建这个文件 = 给 `10` 的权威表加第 16 个文件名, 与 quadruped 当年「进程在册、配置不在册」经 甲-01/甲-02 裁决才补入 `10` v0.7.7 的设计变更完全同形, 不是实现者可自行决定的。

(3) 判据③ 点名的变异体今天不可执行(CLAUDE.md §3.3): 「在 perception.yaml 里复制一份 free_space_corridor => 断言 B『疑似复制』拒绝启动」要求冻结线加载 perception.yaml, 而它不在 L6 清单、不在 §5.4.0 ⑤ 表、不在 MANIFEST.processes => 断言 B 永远碰不到该文件, 这条变异体红不了。按规则 6 如实说明, 不假装跑过。

附一(判据④ 与文档相抵): ④ 要求 r_body_m 与 `12` §12 的 rns.inflation.r_robot_m「同源同值, 写成两个字面量即元测试变红」; 但 `11` §3.1.4 ⑩ C-4 逐字是「必须与 §9A.6 及 12 的 rns.inflation.r_robot_m 三处同值」(三处字面量 + 一条相等断言), 且 `12` §12 现文就是字面量 0.482。要满足 ④ 必须把 `12` §12 改成 ${common.motion.free_space_corridor.r_body_m} 引用 —— 那是 `12` 册的设计改动, 且落到 MOT-PM-33 的文件上。

附二(margin_lat_m 取值需 PD-3 拍板): 本条阻塞列写「margin_lat_m 待评审(PD-3) => 一律 null」, 而 `19` §14 逐字 margin_lat_m: 0.30, §9.2 逐字「C-6 现值不通过 => perception 拒绝启动。这是【有意的】fail-loud, 不得为了让它跑起来而调 margin_lat」。写 null 会让取键失败先于比较发生(`19` 已就地记过 C-5 的同一个坑), 把有意的 C-6 fail-loud 换成 missing_key 报错, 现场排查方向被带偏。两种写法指向不同报错面, 需评审(PD-3 属『他人的决策权』)裁定。

明确不是阻塞点(按拆分口径): x0_m / z_hi_m(待云深处 V-03 · PD-4)、lidar_max_range_m / k_w_s(待 T7 · PD-5)、cam_rgbd_valid_range_m(待 T7 · PD-10)、med2.rgbd_pub_port(待甲方 U-15 · PD-2) 全部按 `19` §14 写 null 即可, 这些实测/第三方欠账本身不阻塞落文件; 判据② 的占位数静态规则(0.35 / 1.20 / 10.0 逐个拒, PCC-8)今天就能写。真正卡住的是键命名空间与文件登记两个人裁项。

另: configs/models/m20s.yaml 与 Phase 0 的 CFG-CF-3(L2 spec.* + terrain + odom + gait_limits)是同一个文件, 两者不可并发落笔; L1 键位骨架 CFG-CF-2 需先在。

- [ ] 已裁决 · 结论: ______

---

## B. 待实测 / 厂商 / 平台基线（3 项）

### CFG-CF-8

卡的不是取值，是【参数名的形状】，所以不能按『代码可写、值留 null』放行。`11` §10.3.2『不跑 costmap 的配置（NAV-98 定案）』的 yaml 块头三行逐字：『参数名在 Nav2 各分支间有差异（Humble 用 costmap_topic / global_frame；Jazzy 拆为 local_* / global_* 并按插件的 CostmapInfoType 选用）。实现期必须以实机 Nav2 版本的 behavior_server 参数声明为准，本表给 Jazzy 形态』；同节风险 R-7 逐字『enable_stamped_cmd_vel（Twist vs TwistStamped）在 Nav2 各分支默认值不同』。而平台基线 D-45（humble/22.04 vs Jazzy/24.04）未拍板、PB-5 又禁写任何发行版判断宏 ⇒ 今天落盘就是在两套键名里赌一套，赌错时的现象是 behavior_server 起来了、参数被静默忽略。本机装的是 humble，但那是开发机，不是已裁定的部署基线，🚫 不得据此代裁。跨分支稳定、今天就能定的只有三件：behavior_plugins 恰为 ['spin','backup','wait']（SC-3）、simulate_ahead_time: 0.0（方案 A）、不启动任何 costmap 节点 —— 可先做成一个校验脚本的判据，但文件本体仍要等 D-45。★ 另需回报判据本身的一处错位：判据说『本文件同时是 MR-1 的求值面』，而 MR-1 的键在 p1_motion.yaml（`11` §10.3.2 变异体 M-MR1-a 逐字『把解析产物里的 rotation_clearance.margin_rot_m 改成 0.30』），不在 nav2/behavior_only.yaml；且 MR-1 的右操作数 common.safety.d_safe_m 今天不在 `10` §5.4.5 左列（见 CFG-CF-4 ③）。

- [ ] 已解决 · 结论: ______

### CPP-BP-4

三重阻塞, 且没有可先写的一半。(1) 交付面不存在: /opt/xbrain_v6/ros2_ws/ 整目录零文件(find 实跑), 判据点名的 ros2_ws/src/behavior_proxy/src/selfcheck.cc 要挂在 CPP-BP-1 建的 behavior_proxy 包上, 而 CPP-BP-1 在本表 Phase 3(1641 行起的表内), 不是 Phase 0; 本表自己的两遍检查第 32 条逐字写着 'CPP-BP-4 与 CPP-BP-4b 从 Phase 0 移到 Phase 3(与 CPP-BP-1 同相位或其后)', 该重排尚未执行。(2) SC-1/SC-2 必须向一个真在跑的 Nav2 behavior_server 发 spin{angle_rad=0.0175} / wait{0.2} 并看终态: 本机 /opt/ros/humble/share 共 286 个包, nav2*/costmap/behaviortree 命中数为 0, 本地起不了 server; 且 11 §10.3.2 风险表 PN-e 逐字 '实现期必须以实机 Nav2 版本的 behavior_server 参数声明为准, 本表给 Jazzy 形态', 平台基线 D-45(humble/22.04 vs Jazzy/24.04) 未拍板 ⇒ 连参数名都不可定, 现在落值等 D-45 一裁就要全推倒。(3) Q16: 判据要 spin.max_rotational_vel <= spec.max_wz_radps, 而 11 §9.6 spec 块该键逐字为 null, configs/ 内 grep max_wz_radps 零命中。★ 另记两条给主会话: ① configs/nav2/behavior_only.yaml 今天是纯注释空骨架, 而该文件已被 Phase 0 的 CFG-CF-8 逐字认领(同一交付文件双主, 触 CHK-1-48 规则④), 开工前须先定主; ② 变异体(d)的 zenoh-bridge allow 列表 cmd_vel 检查, 其执行体属 CPP-DP-3 的 scripts/bridge_allowlist_check.sh, 不在本项。★ 现状核查: xbrain/ tests/ scripts/ common/ deploy/ 内 grep 'nav2|behavior_only|max_rotational_vel' 零命中, 无既有实现, 非 ALREADY_DONE。

- [ ] 已解决 · 结论: ______

### CPP-DP-3

Not ALREADY_DONE: find over /opt/xbrain_v6 for *zenoh_bridge* / *ros2dds* / *allowlist* returns zero hits -- neither the json5 config nor scripts/bridge_allowlist_check.sh exists. SPLIT OF THE ITEM: criterion (1) 'allow list has no cmd_vel' and (3) 'bridge connects 7447 only, not 7449' are static checks writable today; criterion (2) 'ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST 生效可验' is NOT a missing value that can be left null -- it is a mechanism that does not exist on the modelled platform baseline. MEASURED: grep -rn AUTOMATIC_DISCOVERY_RANGE /opt/ros/humble/ = 0 hits (humble ships only ROS_LOCALHOST_ONLY, see /opt/ros/humble/share/ros_environment/environment/1.ros_localhost_only.sh); the installed /usr/bin/zenoh-bridge-ros2dds v1.7.2 carries the literal string 'ROS_AUTOMATIC_DISCOVERY_RANGE will be ignored since it's not supported before ROS 2 Iron'. So on the humble side of D-45 the documented setting is a no-op and DDS discovery stays SUBNET-wide: a script that greps the env var or the config key is permanently green while the isolation is absent (CLAUDE.md 3.2 form 1), and mutant (c) '去掉 LOCALHOST 限制 ⇒ 多 bridge 间 DDS 环路检查必须报' can never be made red because there is no restriction in effect to remove. The correct humble mechanism is ROS_LOCALHOST_ONLY / plugins/ros2dds/ros_localhost_only; picking between the two IS D-45, still open -- 13 §12.4 D-45 row reads '仍开 · P0' and 13 §2.0 models humble+22.04 while 11 §1.2/§1.4 and 00 NAV-100/102 say Jazzy+24.04. SECOND INDEPENDENT GAP (report, do not resolve): the allow list CONTENTS are unspecified in all fourteen volumes. Only the negative constraint exists -- 11 §10.3.2 R-5 verbatim 'zenoh-bridge-ros2dds 的 allow 列表不得包含任何 cmd_vel'. 11 §10.3.1 接口清单 I-1~I-7 shows the whole Nav2 chain bypasses the bridge (behavior_proxy<->Nav2 is native rclcpp Action, behavior_proxy<->p1_motion is direct RT-plane Zenoh, odom->base_link TF is direct DDS), and the only payload ever attributed to the bridge is a conditional fallback in 11 §1.1.6 PC-1 note verbatim '备选（待 T7）… 由 zenoh-bridge-ros2dds 产生 state/targets'. With no allow list at all, criterion (1) and mutant (a) are form 1 as well. STALE BLOCKER COLUMN: the row names PN-d, but PN-d is closed -- 19 §6.3 heading verbatim 'PN-d 裁决 —— 域号 × 发现范围的张力', ruling 4 keeps behavior_proxy / Nav2 / zenoh-bridge-ros2dds on 域 42 + LOCALHOST. PN-d does not block; D-45 does. DEPENDS_ON: the row lists CPP-DP-2, which is a Phase 1 row (TODO line 455), not Phase 0 -- one of the phase inversions the TODO's own appendix item 32 flags -- so there is no Phase 0 dependency to name.

- [ ] 已解决 · 结论: ______
