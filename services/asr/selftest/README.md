# ASR deep-BIT gold corpus

`POST /v1/selftest` decodes every WAV here and compares against `gold.json`, which is the
「跑内置离线 WAV 与固定文本，比对金标」 that `11` §11A.8.1 registers for the endpoint.

## What it is and is not

This is a **load-integrity** check: did the deploy get the model that was tested? It is
**not** an accuracy benchmark, and its number must never be quoted as one.

★ The audio is **synthesized**, deliberately, for two reasons.

1. Real recordings of a person would put personal audio in the delivered package, and camp
   audio is barred outright by `11` AS-9 / `00` NFR-34.
2. `11` AS-14 forbids the running service writing audio to disk; these files are read-only
   build artefacts, which is the 离线工具 exception the same clause names.

⚠️ **The consequence, stated plainly**: this session measured that synthetic speech badly
underestimates short-command ASR — 20–29% bypass recall on synthetic against 92.0% on real
human audio for the same model and the same phrases. So the threshold below is calibrated
by measuring **this corpus** against a known-good model. It cannot be derived from, and
does not predict, field accuracy.

The six lines were chosen longest-first from the 638-utterance synthesis set, because
longer lines are what TTS renders most faithfully — that isolates "the wrong model got
loaded" from "this phrase is hard to synthesize".

## Threshold

| | |
|---|---|
| `baseline_cer` | **0.000** — measured on paraformer-zh-2023-09 int8, all six exact |
| `max_cer` | **0.10** |

`wer` in the response is a **character** error rate; the contract's field name is kept but
Chinese has no word boundaries to tokenise on, and a segmenter's disagreements would
surface as phantom model regressions.

0.10 is set well above the 0.000 baseline and well below what a genuinely wrong model
produces (a family built on another family's weights transcribes these to garbage, CER
near 1.0). With six utterances averaged, it tolerates one utterance being 60% wrong and
still fails a model that is broken across the board.

## Changing the corpus

Re-measure `baseline_cer` when you add or replace a file. A gold entry whose baseline is
not 0.000 is a bad gold entry: it means the corpus, not the model, is being tested.
