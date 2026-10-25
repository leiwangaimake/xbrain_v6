# systemd units

Every `*.service` here plus `run-xbrain.mount` is installed by `scripts/install_units.sh`
(`--dry-run` lists them). The three AI-service units — `xbrain-ai-asr`, `xbrain-llm`,
`xbrain-payload` — are installed like the rest.

> ★★★ **被替代的表述**（生效区间 2026-08 – 2026-10-26）：本文件原标题「systemd units — ★ 草稿，不安装」，
> 正文写这三个 AI 单元「**not installed** … because **DEC-15** … is still open」。
> ★ **为何不再成立**：① DEC-15 早已由 `99` **U83**（2026-08-17）关闭（命名 `xbrain-` · install root `data/install`）；
> ② `11` §11A.2.3 第 5 行已按 AIR-M1 回填（0.36 GB，实测 2026-08-03）；③ `99` **U95**（2026-10-26 用户拍板
> 「systemd，AI 三件套现在立刻做掉」）采纳三个单元的现值，并在 `11` §11A.6.3 OOM 表补了 `xbrain-ai-asr` 与
> `xbrain-payload` 两行。★ **依据**：文档自身的更新表述（U83 / §11A.2.3 回填行）＋ U95 裁决。

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
| 3 | `ExecStart=/opt/xbrain/services/llm/start_llama_server.sh` | `/opt/xbrain_v6/services/llm/llm_server.sh` | Both root and filename differ from what exists. `99` U83 ③ declared the snippet's `/opt/xbrain` obsolete. All three scripts `exec` their server, so systemd supervises the real process, not a shell. |
| 4 | Unit `xbrain-ai-audio`, `MemoryMax=1.5G` | **`xbrain-ai-asr`**, `MemoryMax=1G` | `99` U52 §1 struck 「`ai_audio` = ASR + TTS 合一进程」 verbatim — TTS is in the GZH-2 device, 机上零显存零进程 — and renamed the process `ai_asr`. The 1.5 G budget covered a TTS engine that does not exist here. |
| 5 | No `xbrain-payload` row in the OOM table | values marked 契约未规定·建议 | Drafted without a contract row; `99` U95 (2026-10-26) adopted them and `11` §11A.6.3 now carries the row (+300, 512M). The marks in the unit file record where each number came from. |

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

## Still open (not blocking install)

1. Calibrate `MemoryMax` against T-AI-1 / T-AI-3, which the contract marks ⚠️ (`MemoryMax`
   may not bind GPU allocations on Tegra).
2. `xbrain-config-freeze.service` and `xbrain-perception.service` exist under those names
   now; the AI units order `After=` them but deliberately do not `Requires=` them, so an AI
   unit starts even while the freeze line refuses null calibration (the core units do not).
