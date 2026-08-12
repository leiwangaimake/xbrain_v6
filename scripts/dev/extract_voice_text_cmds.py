#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: extract_voice_text_cmds.py
Brief: Extract 18* command-table rows into tests/command/voice_text_cmd.txt

Description:
Reads docs/18-语音文本指令集.md + 18-A + 18-B; parses each markdown
command-table row (id + backticked intent + slash-separated Chinese
phrases + slots/level/channel/notes); writes one file grouping every
class A-J with all trigger phrases. The output is the human-testable
corpus a live tester speaks into the MIC or pastes into a text-mode
client to exercise every command in 18.

The extractor is dev-tooling only. It is NOT wired into any startup
path and is safe to re-run whenever 18 changes -- deterministic
output, no side effects outside /opt/xbrain_v6/tests/command/.

Each row's phrase cell is normalised (backticks / bold markers / HTML
break tags / decorative stars U+2605 / editorial parentheticals stripped)
and split on '/' (or full-width bar U+FF5C). Empty results and 'editorial
noise' strings are dropped so the output contains only speakable phrases.

Non-ASCII chars this extractor must MATCH in the markdown (stars, the
full-width bar, full-width parentheses) are written as \\uXXXX escapes, not
literals, so this source stays ASCII (CLAUDE.md 2.2) while the regex still
matches the same input glyphs -- the same rule golden test inputs follow.


Each command row has shape:
  | A01 | `estop` | 急停 / 停止 / 紧急停止 / ... | ... | L0 | 全 | ... |

Columns: id | intent | phrases | slots | L-level | channel | notes

Output format (grouped per class A/B/.../J):
  ## <ID> <intent>  [L-level | channel | source-doc]
  phrase1
  phrase2

Phrases with parameter placeholders like 'N 米' / 'N 度' are kept
verbatim -- the user substitutes a real number at test time.
"""
from __future__ import annotations
import re, pathlib, sys

DOCS = [
    ("18-语音文本指令集.md", "18"),
    ("18-A-语音指令扩展.md",   "18A"),
    ("18-B-云台指令扩展.md",   "18B"),
]

# Matches a command-table row. The row starts with '|' then possibly
# decorative prefix (**, stars U+2605/U+2B50, warning sign, etc.), then an
# ID like A01 / B03 / E08 / H03f (letter + digits + optional lowercase
# suffix), possibly followed by more decoration before the closing '|'.
ROW = re.compile(
    r'^\|\s*(?:[*\u2605\u2b50\u26a0\ufe0f]+\s*)?'   # leading decor
    r'(?:\*\*)?\s*'                     # optional bold open
    r'([A-Z][0-9]+[a-z]?)'              # captured id
    r'\s*(?:\*\*)?'                     # optional bold close
    r'\s*\|\s*'
    r'`([^`]+)`'                        # captured intent (backticked)
    r'\s*\|(.+?)\|(.*)$'                # phrases + rest
)

# Strip these decorative artefacts from phrase text.
DECOR = re.compile(r'\*\*|__|<br\s*/?>|<[^>]+>|`|\u26a0\ufe0f|\u26a0|\u2605|\u2705|\u274c|\U0001f6ab')


def clean_phrase(raw: str) -> str:
    """Strip decoration + normalise whitespace. Return '' if the result
    is just an editorial note (contains 'v0.', '2026-', 'U33', '删除',
    parenthetical hint, etc.) rather than a real trigger phrase."""
    s = DECOR.sub('', raw).strip()
    # Squeeze whitespace and drop full-width dashes.
    s = re.sub(r'\s+', ' ', s)
    # Remove trailing/embedded editorial pointers like '(v0.1 语料)'.
    s = re.sub(r'[(\uff08][^)\uff09]*[)\uff09]', '', s).strip()
    # Editorial noise heuristics.
    if not s:
        return ''
    if len(s) > 60:                          # phrases are short imperatives
        return ''
    lowered = s.lower()
    # NOTE: 删除/取消/关闭 are COMMAND verbs (delete_route/_waypoint/_fence,
    # light_off, cancel_task), NOT editorial noise -- they were dropping the
    # whole delete family (F11/F12/F13) and every 关闭/取消 phrasing from the
    # test corpus. Only true editorial markers stay in this filter.
    if any(t in s for t in ('订正', 'U33', 'U54', '云深处', '2026-')):
        return ''
    if any(t in lowered for t in ('v0.', 'q-18-', 'q-p4-', 'cl-')):
        return ''
    return s


def parse_row_columns(payload: str) -> list[str]:
    """Split the remainder of the row on '|' -- leaving cells 3..N."""
    return [c.strip() for c in payload.split('|')]


def parse_doc(path: pathlib.Path, source_tag: str) -> list[dict]:
    """Return one dict per command row: {id, intent, phrases, level,
    channel, source}."""
    entries = []
    for line in path.read_text(encoding='utf-8').splitlines():
        m = ROW.match(line)
        if not m:
            continue
        cid, intent, phrases_cell, rest = m.group(1), m.group(2), m.group(3), m.group(4)
        rest_cells = parse_row_columns(rest)
        # rest_cells layout after id/intent/phrases = [slots, level,
        # channel, notes, ''] (trailing '|' produces an empty last cell).
        slots  = rest_cells[0] if len(rest_cells) > 0 else ''
        level  = rest_cells[1] if len(rest_cells) > 1 else ''
        chan   = rest_cells[2] if len(rest_cells) > 2 else ''
        # Split by '/' or '\uff5c' (full-width slash). Strip decoration
        # from each fragment and drop empties.
        raw_phrases = re.split(r'[/\uff5c]', phrases_cell)
        phrases = []
        seen = set()
        for r in raw_phrases:
            c = clean_phrase(r)
            if c and c not in seen:
                phrases.append(c)
                seen.add(c)
        if not phrases:
            continue
        entries.append({
            'id': cid, 'intent': intent,
            'phrases': phrases,
            'level': DECOR.sub('', level).strip(),
            'channel': DECOR.sub('', chan).strip(),
            'source': source_tag,
        })
    return entries


def main() -> int:
    docs_root = pathlib.Path('/opt/xbrain_v6/docs')
    all_entries = []
    for name, tag in DOCS:
        p = docs_root / name
        if not p.exists():
            print(f'skip missing {p}', file=sys.stderr)
            continue
        all_entries.extend(parse_doc(p, tag))
    # Dedup by (id, intent) -- 18-A and 18-B extensions may repeat.
    seen = {}
    for e in all_entries:
        key = (e['id'], e['intent'])
        if key not in seen:
            seen[key] = e
        else:
            # Merge new phrases from the extension doc.
            for ph in e['phrases']:
                if ph not in seen[key]['phrases']:
                    seen[key]['phrases'].append(ph)
    merged = list(seen.values())
    merged.sort(key=lambda x: (x['id'][0], int(re.match(r'[A-Z]([0-9]+)', x['id']).group(1)), x['id']))

    out = pathlib.Path('/opt/xbrain_v6/tests/command/voice_text_cmd.txt')
    out.parent.mkdir(parents=True, exist_ok=True)
    lines = []
    lines.append('# XBRAIN_V6 voice/text command corpus')
    lines.append('# Auto-extracted from docs/18-* by scripts/dev/extract_voice_text_cmds.py')
    lines.append('# One phrase per line; blank lines and # comment blocks separate commands.')
    lines.append('# Format per command:')
    lines.append('#   ## <ID> <intent>  [L-level | channel | source]')
    lines.append('#   phrase1')
    lines.append('#   phrase2')
    lines.append('#   ...')
    lines.append('')
    # Class-by-class grouping (A/B/C/D/E/F/G/H/I/J...).
    last_class = None
    class_names = {
        'A': 'A run/motion',
        'B': 'B navigation task',
        'C': 'C mode control',
        'D': 'D payload',
        'E': 'E PTZ',
        'F': 'F settings + recording',
        'G': 'G queries',
        'H': 'H system',
        'I': 'I session',
        'J': 'J chitchat',
        'K': 'K (reserved)',
        'L': 'L (reserved)',
    }
    for e in merged:
        cls = e['id'][0]
        if cls != last_class:
            lines.append('')
            lines.append(f'# ============ {class_names.get(cls, cls)} ============')
            lines.append('')
            last_class = cls
        header = f'## {e["id"]} {e["intent"]}  [{e["level"]} | {e["channel"]} | {e["source"]}]'
        lines.append(header)
        for p in e['phrases']:
            lines.append(p)
        lines.append('')
    lines.append(f'# total commands: {len(merged)}')
    lines.append(f'# total phrases: {sum(len(e["phrases"]) for e in merged)}')
    out.write_text('\n'.join(lines) + '\n', encoding='utf-8')
    print(f'wrote {out}: {len(merged)} commands, '
          f'{sum(len(e["phrases"]) for e in merged)} phrases')
    return 0


if __name__ == '__main__':
    sys.exit(main())
