<!--
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: configs/prompts/missions/README.md
Brief: Header note for the P4 mission-prompt directory (documentation only)
-->

# configs/prompts/missions/

Header-note skeleton for the P4 Agent mission-prompt directory (CFG-CF-1).
This is a **documentation file, not a mission prompt**. It is deliberately
`.md` so that no mission loader picks it up (see "Extension" below), and it is
here only to keep the directory tracked and to record what belongs in it.

## What lives here

Per `10` S5.4.0 directory tree (grep anchor `prompts/missions/`) and `16`
S12.4, this directory holds the P4 **mission prompts**. The file name is the
**mission group key**, not an intent name (`16` S3630, grep anchor
`文件名 = 【mission 组 key】`). The P4 loader reads these by key; it does not
treat this README as one.

## Who fills it

FILLED 2026-08-07 by **`GWY-P4-11`**: the 11 `.txt` files are the FIRST
```text fence of each `16` S6.7.1~S6.7.10 subsection, extracted verbatim by
script (no hand copy). The files carry NO header comment on purpose -- their
bytes go straight into the LLM context, so anything added here would be spent
tokens. Provenance lives in this README and in the loader
(`xbrain/p4_agent/registry/missions.py`), whose load-time assertions bind each
file's emitted-intent set to the `16` S6.7 group table.

Known gap, registered rather than papered over: the S6.7 group table lists
`G11 query_events_period` under `M8_events`, but the M8 prompt (rule 3) folds
period queries into `query_events_recent`'s unit/n slots and never emits G11.
The loader's KNOWN_GAPS records it; a regression test asserts the gap until
`16` resolves it one way or the other.

## Hot-update status

`prompts/missions/*` is on the hot-update surface as scope `ai_corpus`
(`16` S12.3, `11` S6994). It is **not** a safety namespace, so it does not go
through the `10` S5.4.4 assertion-E lock.

## Extension: a cross-document drift left unresolved here

`10` S5.4.0 writes `prompts/missions/*.yaml`, while `16` (and `GWY-P4-11`,
"纯文本 prompt") treat mission prompts as plain text (`.txt` / `.tpl`, e.g.
`_clarify.tpl`). These disagree. CFG-CF-1 does **not** resolve that conflict
(CLAUDE.md 9.1: a new design conflict is not settled here). This note sidesteps
it entirely by being `.md`, which neither a `*.yaml` glob nor a `*.txt` /
`*.tpl` glob will select -- so whichever way the drift is later settled, this
file cannot be mistaken for a mission.

## Not here

- The mission **routing table** (128 intents) is `16` S6.6 / `configs/intents.yaml`.
- The system prompt is `configs/prompts/system.txt`, one level up.
- Credentials are `configs/secrets/**`, checked by
  `scripts/lint/secrets_perm_baseline.sh`.
