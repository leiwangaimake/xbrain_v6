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
    r'(?:\*\*)?\s*`?'                   # optional bold open + optional backtick
    r'([A-Z][0-9]+[a-z]?)'             # captured id
    r'`?\s*(?:\*\*)?'                   # optional backtick + optional bold close
    r'\s*\|\s*'
    r'`([^`]+)`'                        # captured intent (backticked)
    r'\s*\|(.+?)\|(.*)$'                # phrases + rest
)
# The id may be bare (18 main: `| E01 |`) or backticked (18-A / 18-B extensions:
# `| \`E01\` |`). The two extension docs number their command rows the second
# way, and matching only the bare form silently dropped every one of them (18-B
# contributed 0 rows, 18-A only 2) -- the PTZ / payload lenient-phrase expansion
# lives in those two files, so the miss hid exactly the corpus the caller wants.

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


# --- S2.2 boundary tables (18 main): tier-2 lenient examples + decline negatives.
# Neither lives in a command-table ROW, so the ROW regex never sees them. They
# are the whole point of the 圆润 expansion (the LLM-routed phrasings) and of the
# test spec (the negatives every tester must cover), so they are parsed by their
# own row shapes and written into clearly labelled sections.

# | cue words | `mission` | lenient-example -> `intent`{slots} |
# Non-ASCII glyphs are written as \uXXXX escapes so this source stays ASCII
# (CLAUDE.md 2.2) while the regex still matches the same input glyphs.
_TIER2_ROW = re.compile(r'^\|[^|]*\|\s*`(M[0-9A-Za-z_]+)`\s*\|([^|]+)\|')
_ARROW = re.compile(r'\u2192|->')             # -> or U+2192 rightwards arrow
# Corner-bracket quotes (U+300C/U+300D and their half-width forms) the decline
# table wraps every test phrase in.
_QUOTED = re.compile(r'[\u300c\uff62]([^\u300d\uff63]+)[\u300d\uff63]')


def parse_tier2_examples(text: str) -> list[dict]:
    """(mission, phrase, intent) from the S2.2 mission-cue table.

    Each row is `| cue | `mission` | 圆润例 -> `intent`{slots} |`. The lenient
    example is left of the arrow, the intent it should reach is right of it.
    These are the tier-2 (LLM) phrasings a keyword tester would never trigger."""
    out = []
    for line in text.split('\n'):
        m = _TIER2_ROW.match(line)
        if not m or not _ARROW.search(m.group(2)):
            continue
        left, right = _ARROW.split(m.group(2), 1)
        phrase = clean_phrase(left)
        im = re.search(r'`([a-z_]+\*?)`', right)
        if phrase:
            out.append({'mission': m.group(1), 'phrase': phrase,
                        'intent': im.group(1) if im else ''})
    return out


def parse_decline_examples(text: str) -> list[dict]:
    """(boundary, phrase) from the S2.2 decline table (section C).

    Rows are `| boundary | expected | 'test phrase'... |`. These are NEGATIVES:
    they MUST be declined / stay silent / be rejected, never executed. Only the
    decline table is scanned (from its C. heading to the next heading), so an
    unrelated corner-bracket string elsewhere cannot leak in."""
    out, in_c = [], False
    for line in text.split('\n'):
        if line.startswith('####') and ('拒绝边界' in line
                                        or '负例' in line):
            in_c = True                                 # C. 拒绝边界 / 负例
            continue
        if in_c and line.startswith('#'):
            break                                       # next heading ends it
        if not in_c or not line.startswith('|'):
            continue
        cells = [c.strip() for c in line.split('|')[1:-1]]
        if len(cells) < 3 or set(cells[0]) <= set('-'):
            continue                                    # separator row
        boundary = DECOR.sub('', cells[0]).strip()
        for ph in _QUOTED.findall(cells[2]):
            c = clean_phrase(ph)
            if c:
                out.append({'boundary': boundary, 'phrase': c})
    return out


# 18-A S2 说法扩充 table: | **D01** `light_on` | mother-doc phrases | expansion |
# The id and intent share ONE cell (unlike command tables), and the ASR-variant
# phrases fill the two cells after it. The middle dot U+00B7 is a phrase
# separator HERE (18-A S2 lists variants with it) -- it is NOT split in the
# command tables, where it can sit inside a phrase (e.g. E07 "向左, 向右环视").
_EXP_ROW = re.compile(
    r'^\|\s*(?:[*\u2605\u2b50\u26a0\ufe0f]+\s*)?'  # leading decor (D17 row has ** ** stars)
    r'(?:\*\*)?\s*([A-Z][0-9]+[a-z]?)\s*(?:\*\*)?\s*'
    r'`([a-z_]+)`[^|]*\|(.+?)\|(.*)$')


def parse_18a_expansions(text: str) -> list[dict]:
    """(id, intent, phrases) from the 18-A S2 说法扩充 table.

    Only that section is scanned (its `## 2` heading to the next `## `). Each row
    lists the id+intent then the added phrases across the mother-doc column and
    the star-marked expansion column -- the payload lenient variants proven on
    real hardware (打开探照灯 / 音量大一点 / 亮一点 / 大点声 / 静音 ...), which the
    ROW regex misses because id and intent are not in separate cells here."""
    out, in_s2 = [], False
    for line in text.split('\n'):
        if line.startswith('## 2') and '说法扩充' in line:
            in_s2 = True
            continue
        if in_s2 and line.startswith('## '):
            break
        if not in_s2:
            continue
        m = _EXP_ROW.match(line)
        if not m:
            continue
        # Both phrase columns joined. group(4) still carries the row's closing
        # '|', so replace every bar with a separator space before splitting --
        # otherwise the last phrase keeps a trailing " |" ("把灯打开 |").
        cell = (m.group(3) + ' / ' + m.group(4)).replace('|', ' ')
        # Strip an editorial parenthetical from the WHOLE cell before splitting:
        # the D17 expansion ends in a long "(...仍 up/down, see S1.1)" note whose
        # internal '/' would otherwise split it into two unbalanced fragments
        # that clean_phrase's per-fragment paren-strip can no longer catch.
        cell = re.sub(r'[(\uff08][^)\uff09]*[)\uff09]', '', cell)
        phrases = []
        for r in re.split(r'[/\uff5c\u00b7]|<br\s*/?>', cell):
            c = clean_phrase(r)
            if c and c not in phrases:
                phrases.append(c)
        if phrases:
            out.append({'id': m.group(1), 'intent': m.group(2),
                        'phrases': phrases, 'level': '', 'channel': '',
                        'source': '18A'})
    return out


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
        # Split by '/', full-width slash '\uff5c', OR a <br> tag. <br> MUST be a
        # separator, not stripped decoration: 18-B writes E06 as
        # "... \u5927\u4e00\u70b9<br>\u62c9\u8fdc / ..." to break a wide cell across two visual lines,
        # and stripping <br> (as DECOR does for the leftover fragments) merged
        # "\u5927\u4e00\u70b9" and "\u62c9\u8fdc" into the non-phrase "\u5927\u4e00\u70b9\u62c9\u8fdc". Splitting here first
        # keeps the two speakable phrases apart.
        raw_phrases = re.split(r'[/\uff5c]|<br\s*/?>', phrases_cell)
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
    # 18-A S2 说法扩充 rows carry id+intent in one cell, so parse_doc's ROW regex
    # skips them; parse them separately and fold their ASR-variant phrases into
    # the matching D-class command via the same (id, intent) dedup below.
    a_path = docs_root / '18-A-语音指令扩展.md'
    if a_path.exists():
        all_entries.extend(parse_18a_expansions(a_path.read_text(encoding='utf-8')))
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
    # --- Tier-2 lenient (LLM-routed) examples + decline negatives (18 main S2.2).
    # Neither is a command-table row: the mission-cue table teaches the lenient
    # phrasings a keyword tester never triggers, and the decline table is the
    # negative spec every tester must cover. Both are labelled loudly so a
    # negative is never spoken expecting execution.
    main_text = (docs_root / '18-语音文本指令集.md').read_text(encoding='utf-8')
    t2 = parse_tier2_examples(main_text)
    neg = parse_decline_examples(main_text)

    lines.append('')
    lines.append('# ============ TIER-2 圆润例 (LLM 兜底, 非关键词, 应执行) ============')
    lines.append('# 经层6 LLM 到达意图, 非层2 关键词. 说这些测圆润理解.')
    lines.append('')
    for x in t2:
        lines.append('## %s (tier-2 via %s)  [tier-2 | 18 S2.2]'
                     % (x['intent'] or 'unknown', x['mission']))
        lines.append(x['phrase'])
        lines.append('')

    lines.append('')
    lines.append('# ============ 拒识负例 (NEGATIVE, 应拒绝/静默/拒识, 不要期望执行) ============')
    lines.append('# 每条都必须被礼貌拒绝(out_of_scope) / 完全静默(overheard) /')
    lines.append('# 归底盘 A 类 / 或拒绝. 因它没执行就报缺陷, 正是本段要防的错.')
    lines.append('')
    for x in neg:
        lines.append('## %s  [decline-negative | 18 S2.2]' % x['boundary'])
        lines.append(x['phrase'])
        lines.append('')

    lines.append(f'# total commands: {len(merged)}')
    lines.append(f'# total phrases: {sum(len(e["phrases"]) for e in merged)}')
    lines.append(f'# total tier-2 examples: {len(t2)}')
    lines.append(f'# total decline negatives: {len(neg)}')
    out.write_text('\n'.join(lines) + '\n', encoding='utf-8')
    print(f'wrote {out}: {len(merged)} commands, '
          f'{sum(len(e["phrases"]) for e in merged)} phrases, '
          f'{len(t2)} tier-2, {len(neg)} negatives')
    return 0


if __name__ == '__main__':
    sys.exit(main())
