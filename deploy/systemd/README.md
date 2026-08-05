# systemd units — ★ 草稿，不安装

Three unit drafts for the AI peripheral services. They are **not installed** and nothing in
this repo installs them, following the convention `_plan/KICKOFF-PLAN.md:231` already set
for `deploy/systemd/xbrain-config-freeze.service (草稿，不安装)`.

They are drafts because **DEC-15** (`_plan/DECISIONS-OPEN.md:304`, 「构建系统 ＋ 仓库布局 ＋
systemd 单元命名」, 谁能拍 = **主会话**) is still open. What is undecided is naming, the
install root, and where these sit in `10` §3.3's Stage table — not the directives below,
which come from `11` §11A.6.3 and are reproduced with every deviation called out.

## Why they matter more than a packaging chore

`11` §11A.6.3 is explicit that this is a **safety** item, not deployment hygiene:

> `10` §9.2 says 「GPU OOM → 卸载按需池模型，保感知」, but **nothing guarantees the OOM
> killer picks llama-server rather than perception**.

The kernel picks by `oom_score`, which has no idea what matters. Until these units exist,
`OOMScoreAdjust` is unset on every AI service, so under memory pressure the kernel is as
likely to kill `perception` — losing obstacle avoidance — as to kill the LLM. The whole
`AIR-F1`~`F3` guardrail lives in these files and nowhere else.

## Deviations from `11` §11A.6.3's snippet — each deliberate

| # | Contract snippet | Here | Why |
|---|---|---|---|
| 1 | `StartLimitBurst` / `StartLimitIntervalSec` in `[Service]` | in **`[Unit]`** | ⚠️ **The snippet as written does not work.** systemd moved both to `[Unit]` in v229; in `[Service]` they are ignored with a warning. This host runs systemd 249. Left in `[Service]`, `AIR-F1`'s 「不得无限重启」 would silently not apply — the exact failure AIR-F1 exists to prevent. |
| 2 | AIR-F2 「OOM 后不自动重启」 has no directive | **`OOMPolicy=stop`** | The snippet pairs `Restart=on-failure` with AIR-F2, but an OOM kill is SIGKILL, which `on-failure` restarts — so the snippet contradicts its own constraint. `OOMPolicy=stop` (systemd ≥243) stops the unit instead, which is what AIR-F2 asks for. |
| 3 | `ExecStart=/opt/xbrain/services/llm/start_llama_server.sh` | `/opt/xbrain_v6/services/llm/llm_server.sh` | Both root and filename differ from what exists. Reconciling `/opt/xbrain` with `/opt/xbrain_v6` is DEC-15. All three scripts `exec` their server, so systemd supervises the real process, not a shell. |
| 4 | Unit `xbrain-ai-audio`, `MemoryMax=1.5G` | **`xbrain-ai-asr`**, `MemoryMax=1G` | `99` U52 §1 struck 「`ai_audio` = ASR + TTS 合一进程」 verbatim — TTS is in the GZH-2 device, 机上零显存零进程 — and renamed the process `ai_asr`. The 1.5 G budget covered a TTS engine that does not exist here. |
| 5 | No `xbrain-payload` row in the OOM table | drafted, values marked 契约未规定·建议 | payload-service is absent from `10` §3.1's process list too (`_plan/KICKOFF-MODULES.md:161` already flags this). Its numbers are proposals, not contract values. |

## Measured, so the budgets are not guesses

Resident sizes on the Orin, 2026-08-03, all three services loaded and idle:

| unit | measured RSS | `MemoryMax` here | contract |
|---|---:|---:|---|
| `xbrain-llm` | 468 MiB host + 2.45 GB unified | **3G** | ★ `11` §11A.6.3 |
| `xbrain-ai-asr` | 374 MiB | **1G** | ⚠️ derived (see #4) |
| `xbrain-payload` | 59 MiB | **512M** | ⚠️ 契约未规定·建议 |

⚠️ **`11` §11A.2.3's ledger records `ai_asr` at 0.20 GB (zipformer ONNX int8).** The
deployed model is now paraformer int8 and measures **374 MiB ≈ 0.36 GB**. `AIR-M1` says a
model change 「必须回填 §11A.2.3 表，否则视同未评审」 — that backfill is a `11` edit and has
not been made. `GET /healthz`'s `mem_mb` is the in-band field that exposes it.

⚠️ **T-AI-3 is unresolved and it limits what `MemoryMax` buys.** Whether Tegra nvmap/CUDA
allocations count against cgroup v2 `memory.current` is untested. If they do not, `MemoryMax`
constrains only host-side allocation and a GPU overrun still surfaces as `cudaMalloc`
failure. `11` §11A.6.3 states this plainly: 「不能因为设了 `MemoryMax` 就认为 GPU OOM 已被
兜住」. That applies to `xbrain-llm`; `xbrain-ai-asr` is CPU-only (provider pinned to `cpu`
in `services/asr/core/families.py`) so its limit does bind.

## Before installing

1. Settle DEC-15: unit names, install root, and Stage placement in `10` §3.3.
2. Add the `xbrain-payload` row to `11` §11A.6.3's OOM table, or accept the drafted values.
3. Backfill `11` §11A.2.3 line 5 per AIR-M1.
4. Calibrate `MemoryMax` against T-AI-1 / T-AI-3, which the contract already marks ⚠️.
5. Verify `perception.service` and `xbrain-config-freeze.service` exist under those names —
   the `After=` lines reference them and neither exists in this repo yet.
