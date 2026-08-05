"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: bench_asr_device.py
Brief: On-device RTF and hotword benchmark for the TRANSDUCER family (RTF < 0.5 gate).

Description:
  asr-service has a hard requirement of RTF < 0.5, which no unit test can verify: it
  depends on the real sherpa-onnx build, the real model and the Orin's own CPU, so a
  dev-box number would mean nothing. This script is that measurement, and it must be run
  ON the Orin.

  ★ SCOPE: this script measures the TRANSDUCER family specifically, and pins that family
  explicitly rather than taking the service default. Its second half -- the hotwords ON vs
  OFF comparison -- only means anything on a transducer, because sherpa-onnx offers
  contextual biasing on no other architecture (core/families.py). Run against the current
  default family (paraformer) both arms would be identical greedy decodes with an empty
  vocabulary, and the script would report a carefully measured zero as if it were a
  finding. For an RTF number on whatever family is actually deployed, read the per-request
  rtf the running service logs, or the rtf field it now returns.

  It drives core/recognizer directly rather than through HTTP, because the requirement is
  about inference speed: adding uvicorn, multipart parsing and a loopback round trip to
  the measurement would blur the number this gate is actually about. The REST layer's own
  overhead is separately visible in the service log, which reports the same RTF per
  request once the service is running.

  What it reports, per audio file and in aggregate:
    - decode wall time, audio duration and RTF, against the 0.5 budget;
    - the recognized text, so a human can confirm the model is producing sense and not
      just producing it quickly;
    - the same run with the vocabulary disabled, because hotword biasing forces
      modified_beam_search and beam search is the slower path -- so the number that has to
      clear 0.5 is the one WITH hotwords on, and printing both shows what biasing costs.

  This is NOT a pytest (it needs the engine, a full model and several seconds of CPU),
  so it carries no test_ prefix and pytest will not collect it. Run it on the Orin:
      cd /opt/xbrain_v6 && python3 tests/asr/bench_asr_device.py [wav ...]
  With no arguments it benchmarks every wav in the transducer export's test_wavs/.

  Exit status is 0 only when every file cleared the RTF budget, so it can gate a deploy.
"""
from __future__ import annotations

import glob
import logging
import os
import sys
import tempfile

# Make "from services.asr..." resolve whether this is launched as a plain script or via
# -m: ensure the repo root (three levels up from tests/asr/) is on sys.path.
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from services.asr.config import _FAMILY_MODEL_DIRS, AsrConfig
from services.asr.core.audio_in import decode_wav
from services.asr.core.recognizer import AsrEngineError, Recognizer

# The acceptance budget. A spoken second must cost under half a second to transcribe, so
# recognition never becomes the bottleneck in a 功能1 dialogue turn.
_RTF_BUDGET = 0.5

# Default corpus: the sample wavs shipped with the model export. They are real Chinese
# speech at 16 kHz and 8 kHz, which also exercises the engine's internal resampling.
# The family this benchmark is about. Not a configuration read: the hotwords ON/OFF
# comparison below is only expressible on a transducer, so the script names its subject
# rather than inheriting whichever family happens to be deployed.
_BENCH_FAMILY = "transducer"

_DEFAULT_WAV_GLOB = os.path.join(_REPO_ROOT, "services", "asr", "model-zipformer-multi-zh-hans",
                                   "test_wavs", "*.wav")


def _load_audio(paths):
    """Read every wav once, up front, so file I/O is outside the timed region."""
    loaded = []
    for path in paths:
        with open(path, "rb") as handle:
            loaded.append((os.path.basename(path), decode_wav(handle.read())))
    return loaded


def _run(recognizer: Recognizer, clips, label: str) -> float:
    """Decode every clip with one recognizer, print per-clip results, return the worst RTF.

    The WORST rather than the average is returned because the budget is a per-utterance
    promise: one slow turn is a stall the user hears, and an average would hide it behind
    the short clips.
    """
    print(f"\n--- {label}: model={recognizer.model_name} "
          f"decoding={recognizer.decoding_method} hotwords={recognizer.hotword_count} "
          f"rejected={recognizer.rejected_hotword_count} ---")
    worst = 0.0
    for name, audio in clips:
        result = recognizer.transcribe(audio)
        worst = max(worst, result.rtf)
        # The text is printed so a human can confirm the model produces sense, not just
        # speed -- a broken model would still post an excellent RTF.
        print(f"{name:10s} {audio.duration_s:5.1f}s  decode {result.decode_ms:6.0f} ms  "
              f"rtf {result.rtf:.3f}  -> {result.text}")
    print(f"worst rtf: {worst:.3f} (budget {_RTF_BUDGET})")
    return worst


def main() -> int:
    """Benchmark the shipped or supplied wavs and report whether the RTF gate holds."""
    # Logging is turned on here rather than left to the caller because the recognizer
    # reports its vocabulary problems through the logger: any hotword phrase this model
    # cannot bias is named at WARNING during construction, and that is exactly what a
    # hotword acceptance run needs to see. Without this the benchmark would print a
    # rejected count with no way to tell which phrases it refers to.
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")

    paths = sys.argv[1:] or sorted(glob.glob(_DEFAULT_WAV_GLOB))
    if not paths:
        print("no wav files to benchmark", file=sys.stderr)
        return 2

    # Family pinned, not inherited: see the SCOPE note in the module docstring. Reading
    # the service default here would silently turn the hotword comparison below into a
    # measurement of nothing.
    config = AsrConfig(model_family=_BENCH_FAMILY,
                       model_dir=_FAMILY_MODEL_DIRS[_BENCH_FAMILY])
    clips = _load_audio(paths)

    try:
        # The configuration that actually ships: hotwords on, hence beam search. This is
        # the run whose RTF has to clear the budget.
        biased = Recognizer(config)
        worst_biased = _run(recognizer=biased, clips=clips, label="hotwords ON")

        # Same model with an empty vocabulary, which selects greedy search. Printed for
        # comparison so the cost of biasing is a measured number rather than a guess.
        with tempfile.NamedTemporaryFile("w", suffix=".txt", delete=False) as empty:
            empty.write("# no hotwords\n")
            empty_path = empty.name
        try:
            plain_config = AsrConfig(
                port_asr=config.port_asr,
                model_family=config.model_family,
                model_dir=config.model_dir,
                hotwords_file=empty_path,
                hotwords_score=config.hotwords_score,
                num_threads=config.num_threads,
                use_int8=config.use_int8,
            )
            _run(recognizer=Recognizer(plain_config), clips=clips, label="hotwords OFF")
        finally:
            os.unlink(empty_path)
    except AsrEngineError as exc:
        # The most likely cause on a fresh box is a missing engine or model file, and the
        # exception text already names which -- so report it plainly rather than as a
        # traceback the operator has to read through.
        print(f"ASR engine unavailable: {exc}", file=sys.stderr)
        return 2

    # Only the biased (shipping) configuration gates the exit status.
    if worst_biased >= _RTF_BUDGET:
        print(f"\nFAIL: worst rtf {worst_biased:.3f} exceeds budget {_RTF_BUDGET}", file=sys.stderr)
        return 1
    print(f"\nPASS: worst rtf {worst_biased:.3f} is within budget {_RTF_BUDGET}")
    return 0


if __name__ == "__main__":
    # Only run when executed as a script, never on import.
    sys.exit(main())
