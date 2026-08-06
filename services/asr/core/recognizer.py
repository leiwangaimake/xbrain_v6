"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: recognizer.py
Brief: sherpa-onnx wrapper -- model loading, hotword biasing, admission control, one decode.

Description:
  The inference core of asr-service. It owns the single loaded sherpa-onnx
  OfflineRecognizer and exposes exactly one operation: waveform in, text out. sherpa-onnx
  is prebuilt inference infrastructure and therefore outside R0 (plan section 11); what is
  written from scratch here is the service glue -- resolving and checking the model files,
  wiring the hotword vocabulary into the decoder, admission control, serializing decodes,
  and measuring RTF and confidence.

  WHICH architecture gets loaded is a configuration choice described by core/families.py.
  This module holds no knowledge of any particular family beyond what that table says: the
  file layout, the sherpa-onnx factory and whether biasing is expressible all come from the
  Family record, so evaluating a candidate model is a row there and no change here. The one
  thing this module asserts about every family is the execution provider -- CPU, always,
  because the AI Runtime memory plan is built on asr-service holding no CUDA context of its
  own (11 AIR-M3), and an inherited engine default is exactly how that invariant would get
  lost without anyone noticing.

  Why the recognizer is built ONCE and held: loading the network files takes seconds and
  allocates hundreds of megabytes. Building it per request would dwarf the decode itself
  and make the RTF < 0.5 requirement unreachable, so app.py constructs this object during
  startup and every request reuses it.

  RTF is the acceptance metric for this service (hard requirement: RTF < 0.5), so
  Transcript carries the numbers it is computed from rather than just the text. The
  measurement is deliberately narrow -- it times the decode call only, not the HTTP
  parse or the wav decode -- because that is the number that has to hold as audio gets
  longer; the surrounding costs are per-request constants and are visible separately in
  the request log.

  * Concurrency is admission control, not queueing. One decode runs at a time -- the ONNX
  session's intra-op thread budget is sized on the assumption that it is the only one
  running, so overlapping two would oversubscribe the Orin and make BOTH slower. The
  earlier implementation expressed that by blocking on a lock, which made a second caller
  WAIT. That is the one thing the interface contract forbids (11 AS-12): the gateway
  serializes AI work behind its own GPU-admission token and assigns priorities, and a
  service that queues internally silently re-orders that priority into FIFO. So a second
  concurrent decode is REFUSED here (AsrBusyError -> HTTP 429) rather than parked, and the
  gateway retries when its own scheduling says to.

  Hotwords: contextual biasing requires modified_beam_search (greedy search has no beam to
  bias), so the decoding method follows the vocabulary rather than being configured
  independently -- a configuration that asked for greedy search AND hotwords would silently
  drop the hotwords, so it is not expressible here. Biasing is also only available on the
  transducer family at all (families.Family.supports_hotwords); on a family that cannot
  bias, the vocabulary is not loaded and the reported hotword count is zero rather than a
  number that would imply an effect the engine is not producing.

  For a transducer, the vocabulary is additionally filtered against the model's own symbol
  table before it reaches the engine. modeling_unit="cjkchar" was chosen over "bpe" after
  measuring both: cjkchar biases a phrase wherever it occurs in the utterance, while bpe
  only matches at the utterance's first token, because sentencepiece prefixes a word-start
  marker that the model emits nowhere else. bpe can encode every phrase and cjkchar cannot,
  but position-independent biasing of most of the vocabulary beats first-token-only biasing
  of all of it -- a command is as often spoken mid-sentence as at the start.

  * Confidence is reported when the engine produces it and omitted when it does not. The
  contract (11 AS-4) asks for a conf field, and a transducer supplies per-token log
  probabilities from which one can be derived; Paraformer's result carries none, and
  neither do the CTC families. Returning a fabricated number there would be worse than
  returning nothing: conf exists so a caller can distrust a transcription, and a constant
  dressed up as a measurement would make every transcription look equally trustworthy.
"""
from __future__ import annotations

import asyncio
import logging
import math
import os
import threading
import time
from dataclasses import dataclass
from typing import Optional

from ..config import AsrConfig
from . import families
from .audio_in import Audio
from .hotwords import load_hotwords, load_token_vocab, materialize, select_encodable

logger = logging.getLogger("asr.recognizer")

# Symbol table file name. Shared by every family and by both precisions -- it describes
# what the model can emit, not how its weights are stored.
_TOKENS_NAME = "tokens.txt"

# Two precisions may ship side by side in a model directory. int8 is the fast path
# (config.use_int8), fp32 the accuracy reference used to A/B a suspicious transcription.
# Not every export publishes both, which is why a missing file is reported by name rather
# than assumed absent-because-unnecessary.
_INT8_SUFFIX = ".int8.onnx"
_FP32_SUFFIX = ".onnx"


class AsrEngineError(RuntimeError):
    """Raised when the inference engine cannot be built or a decode fails.

    House rule bans bare Exception. This type is what lets the REST layer answer 503
    ("the service cannot transcribe right now") while a malformed upload answers 400 --
    the caller's reaction differs, so the two faults must not share a type. RuntimeError
    is the base because these are operational failures, not bad argument values.
    """


class AsrBusyError(RuntimeError):
    """Raised when a decode is already in flight and this one is refused rather than queued.

    Deliberately NOT a subclass of AsrEngineError, following the same rule that separates
    that type from AudioInputError: two faults share a type only when the caller reacts to
    them the same way. A busy service is healthy and the right reaction is to come back
    later (HTTP 429); an engine failure means the transcription capability itself is in
    doubt (HTTP 503). Folding them together would hand the gateway one code for two
    different situations, and 11 §8.13.5 already treats 503 as evidence that AS-3's
    concurrency limit was bypassed -- a signal that must not be produced by ordinary load.
    """


@dataclass(frozen=True)
class Transcript:
    """One decode result together with the numbers its RTF is computed from.

    Frozen because it is a result value handed back across a thread boundary; nothing
    should mutate it after the decode that produced it.

    Carrying duration and decode time alongside the text is what makes the RTF < 0.5
    requirement continuously observable in production rather than only in a benchmark:
    every request can log its own RTF, so a regression (a model swap, a thread-count
    change, thermal throttling on the Orin) is visible in the ordinary service log.
    """

    # The recognized text, exactly as the engine produced it.
    text: str
    # Length of the audio that was decoded, in seconds.
    duration_s: float
    # Wall-clock time the decode call itself took, in milliseconds.
    decode_ms: float
    # Normalized [0, 1] confidence, or None when this model family does not report the
    # per-token log probabilities it is derived from. None means "not measured" and must
    # stay distinguishable from a low score, which means "measured, and poor".
    conf: Optional[float] = None

    @property
    def rtf(self) -> float:
        """Real-time factor: decode seconds divided by audio seconds.

        Returns:
            The RTF, or 0.0 for zero-length audio (which cannot be divided by).

        Below 1.0 means faster than real time; this service's hard requirement is < 0.5,
        i.e. a spoken second must cost under half a second to transcribe, so a dialogue
        turn is not held up by recognition.
        """
        # Zero-length audio never reaches a real decode (empty uploads are rejected at the
        # boundary), but keeping the property total avoids a division trap for any caller
        # that formats a Transcript before checking it.
        if self.duration_s <= 0.0:
            return 0.0
        return (self.decode_ms / 1000.0) / self.duration_s


def _import_engine():
    """Import sherpa-onnx, turning an absent package into an actionable error.

    Returns:
        The imported sherpa_onnx module.

    Raises:
        AsrEngineError: when the package is not installed.

    The import is deferred into this function rather than done at module scope for two
    reasons: a deployment that has not installed the engine gets a message naming the
    package instead of a bare ModuleNotFoundError from an unrelated-looking import line;
    and the pure-python parts of this module (Transcript, path resolution) stay importable
    on a machine without the engine, so their unit tests need no inference runtime.
    """
    try:
        import sherpa_onnx  # noqa: WPS433 -- deliberate local import, see docstring
    except ImportError as exc:
        # Chain the original so the traceback still shows what could not be imported.
        raise AsrEngineError(
            "sherpa-onnx is not installed; install it into the Orin python3.10 "
            "(python3 -m pip install --user sherpa-onnx)"
        ) from exc
    return sherpa_onnx


def _find_network_file(model_dir: str, stem: str, suffix: str) -> Optional[str]:
    """Locate one network file by stem and precision suffix, or report it absent.

    Args:
        model_dir: directory holding the export.
        stem: the substring identifying this network within the export, e.g. "encoder".
        suffix: the precision suffix, _INT8_SUFFIX or _FP32_SUFFIX.

    Returns:
        The path, or None when no file in the directory matches.

    Matching on a substring rather than on a fixed name is what lets one Family record
    cover exports that embed a checkpoint in the file name: icefall publishes
    "encoder-epoch-20-avg-1.int8.onnx" and "encoder-epoch-12-avg-4.int8.onnx" for the same
    architecture, and pinning either one would make the family describe a single download
    rather than a family. The precision suffix is matched exactly, because ".onnx" is a
    suffix of ".int8.onnx" and a naive check would hand the fp32 request an int8 file.
    """
    for name in sorted(os.listdir(model_dir)):
        if not name.endswith(suffix):
            continue
        # An fp32 request must not match an int8 file, whose name also ends in ".onnx".
        # Compared by value: an identity check would happen to work while every caller
        # passes the module constant, and break silently the first time one does not.
        if suffix == _FP32_SUFFIX and name.endswith(_INT8_SUFFIX):
            continue
        if stem in name:
            return os.path.join(model_dir, name)
    return None


def _looks_like_version(name: str) -> bool:
    """True for a directory name that is a version rather than a model name.

    Args:
        name: one path component.

    Returns:
        Whether every dot-separated part is digits, e.g. "1.0.0" or "2" but not "1.0.0rc1"
        and not "paraformer-zh-2023-09" (which contains dashes, so it never splits into
        all-digit parts and is correctly treated as a name).

    Used so a config pointing straight at a version directory -- legal, and what someone
    debugging a specific version would type -- still reports the model's name rather than
    the number. Deliberately strict: anything it is unsure about is treated as a name,
    because a wrong name is visible in /healthz while a wrongly-skipped component would
    silently report the grandparent directory.
    """
    parts = name.split(".")
    return len(parts) > 1 and all(part.isdigit() for part in parts)


def _resolve_model_paths(family: families.Family, model_dir: str, use_int8: bool) -> dict:
    """Build and check every model file this family needs at the requested precision.

    Args:
        family: the architecture description, supplying which networks to look for and
            which sherpa-onnx keyword each one satisfies.
        model_dir: directory holding the export.
        use_int8: select the int8 files when true, the fp32 files when false.

    Returns:
        Keyword arguments naming each existing file, e.g. {"encoder": ..., "tokens": ...}.

    Raises:
        AsrEngineError: if any required file is missing, naming every missing one and the
            directory searched.

    Everything is checked BEFORE the engine is built so a partial deploy -- or a family
    pointed at another family's directory, which is the mistake this indirection makes
    possible -- fails with a list of exactly what is absent, instead of an opaque error
    from inside the C++ loader that names only the first problem. Reporting them together
    means one round trip fixes a deploy missing more than one file.
    """
    if not os.path.isdir(model_dir):
        raise AsrEngineError(f"ASR model directory does not exist: {model_dir}")

    # tokens.txt is shared by every precision -- it is the model's symbol table, not a
    # weight file -- so only the network files take the precision suffix.
    suffix = _INT8_SUFFIX if use_int8 else _FP32_SUFFIX
    resolved = {}
    missing = []
    for keyword, stem in family.files:
        path = _find_network_file(model_dir, stem, suffix)
        if path is None:
            missing.append(f"{stem}*{suffix}")
        else:
            resolved[keyword] = path

    tokens = os.path.join(model_dir, _TOKENS_NAME)
    # isfile (not exists) so a directory sitting where a model file belongs is reported as
    # missing rather than passed to the loader, which would fail far less clearly.
    if not os.path.isfile(tokens):
        missing.append(_TOKENS_NAME)
    else:
        resolved["tokens"] = tokens

    if missing:
        raise AsrEngineError(
            f"missing file(s) for the {family.name!r} family in {model_dir}: "
            f"{', '.join(missing)}"
        )
    return resolved


def _mean_log_prob_confidence(log_probs) -> Optional[float]:
    """Fold per-token log probabilities into one [0, 1] score, or None when absent.

    Args:
        log_probs: the engine's per-token log probabilities; empty on families that do not
            report them.

    Returns:
        exp(mean(log_probs)) clamped to [0, 1], or None when there is nothing to fold.

    This is the formula 11 §8.13.3 names for deriving conf when a service does not publish
    one directly, applied here rather than in the gateway because this is where the raw
    numbers exist -- the HTTP response would have to carry per-token data purely so the
    caller could run the same arithmetic.

    An empty sequence yields None, NOT 1.0: exp(mean([])) has no value, and a family that
    reports nothing must be distinguishable from one that reports certainty.
    """
    if not log_probs:
        return None
    mean = sum(log_probs) / len(log_probs)
    # Clamp rather than trust: the value is a geometric mean of probabilities and so is
    # already in [0, 1] mathematically, but a NaN or a rounding excursion from the engine
    # must not escape into a field the caller uses to gate a spoken command.
    if math.isnan(mean):
        return None
    return max(0.0, min(1.0, math.exp(mean)))


class Recognizer:
    """The loaded ASR model plus its hotword vocabulary, decoding one utterance at a time.

    Construction does all the expensive and all the failure-prone work -- resolving model
    paths, reading the vocabulary, building the engine -- so that a service which finished
    starting is a service that can transcribe. Everything a request needs afterwards is
    already in memory.
    """

    def __init__(self, config: AsrConfig) -> None:
        """Load the model and vocabulary described by config.

        Args:
            config: the service configuration; supplies the model family and directory,
                precision, thread budget, and the hotword file and its bias score.

        Raises:
            FamilyError: if the configured family name is not one this service knows.
            AsrEngineError: if the engine package is absent, a model file is missing, or
                the engine refuses to build.
            HotwordsError: if the configured vocabulary file cannot be read.

        The family is resolved first because it decides which files to look for, and the
        vocabulary before the engine because it decides the decoding method -- and because
        a mistyped hotwords path should fail before spending several seconds loading a
        model of several hundred megabytes.
        """
        family = families.get(config.model_family)
        self._family = family
        engine = _import_engine()
        paths = _resolve_model_paths(family, config.model_dir, config.use_int8)
        # Kept for GET /v1/models and AS-11: the directory identifies the deploy and the
        # resolved network files are what sha256 must cover. Sorted so the digest does not
        # depend on the family table's kwarg order, which is an implementation detail.
        self._model_dir = config.model_dir
        self._model_files = tuple(sorted(paths.values()))

        # Vocabulary next: it selects the decoding method (see module docstring), and an
        # empty vocabulary is a legitimate "bias nothing" deployment rather than an error.
        # * On a family that cannot be biased the file is not read at all -- loading it
        # would produce a hotword count that /status reports and the engine ignores, which
        # is precisely the silent-inert-capability failure core/hotwords.py exists to
        # prevent. The reason is logged once, so "hotwords: 0" is never a mystery.
        if family.supports_hotwords:
            authored = load_hotwords(config.hotwords_file)
            # Then drop what this model cannot bias. The engine would drop the same phrases
            # by itself, but only as C++ warnings on stderr that no service log captures,
            # leaving the vocabulary silently smaller than the file says. Deciding it here
            # means the count reported by /status is the count actually in force, and the
            # phrases that were lost are named once at startup instead of never.
            self._hotwords, self._rejected = select_encodable(
                authored, load_token_vocab(paths["tokens"]))
            if self._rejected:
                logger.warning(
                    "%d of %d hotword phrases are not in this model's symbol table and "
                    "cannot be biased: %s",
                    len(self._rejected),
                    len(authored),
                    "; ".join(f"{entry.replace(' ', '')} ({''.join(missing)})"
                              for entry, missing in self._rejected),
                )
        else:
            self._hotwords, self._rejected = [], []
            logger.info(
                "hotword biasing is unavailable on the %r family (sherpa-onnx exposes "
                "contextual biasing only on transducers); %s is not loaded",
                family.name,
                config.hotwords_file,
            )
        self._method = family.decoding_method(bool(self._hotwords))
        # The reported identity: which export, at which precision. A log line or a /status
        # response has to say it, because the choice is invisible otherwise yet is the
        # first thing to check when accuracy looks off.
        #
        # It is derived from the DIRECTORY NAME because that is the only place the concrete
        # export is recorded today -- an icefall or FunASR download names its own build,
        # and the service has nothing else to read. The conventional "model" / "model-"
        # prefix is stripped so the result reads as an identity rather than a path
        # fragment, and the family is prepended only when the directory does not already
        # imply it, so a well-named export does not report "paraformer-paraformer-zh-...".
        #
        # ! The VERSION half of 11 AS-11 is not derived here. It comes from MODEL.json via
        # core/model_meta.py, because a version invented from a path string would be a
        # fabricated identity -- the one thing 11 §11A.7 says this field must never be.
        #
        # * "current" is skipped. Under 11 §11A.4.1's layout the configured path ends in the
        # rollback symlink (.../paraformer-zh-2023-09/current), whose basename identifies
        # nothing -- taking it produced "paraformer-current-int8", a name that is identical
        # across every model in the tree and therefore useless as the AS-11 identity it
        # feeds. The directory that names the model is the symlink's parent. A version
        # directory is skipped for the same reason: ".../1.0.0" names a version, not a model.
        tag = os.path.basename(config.model_dir.rstrip("/"))
        if tag == "current" or _looks_like_version(tag):
            tag = os.path.basename(os.path.dirname(config.model_dir.rstrip("/")))
        if tag == "model":
            tag = ""
        elif tag.startswith("model-"):
            tag = tag[len("model-"):]
        precision = "int8" if config.use_int8 else "fp32"
        parts = [tag] if tag.startswith(family.name) else [family.name, tag]
        self._model_name = "-".join([part for part in parts if part] + [precision])

        # Arguments shared by both paths. num_threads is the ONNX intra-op budget, kept
        # deliberately below the core count so the co-hosted services keep their cores.
        # provider is pinned rather than defaulted: see the provider note in the module
        # docstring -- this is what keeps asr-service off the GPU.
        kwargs = dict(paths)
        kwargs.update({
            "num_threads": config.num_threads,
            "decoding_method": self._method,
            "provider": families.PROVIDER_CPU,
        })
        kwargs.update(dict(family.extra))

        # Load time is logged, not just measured: it is the dominant part of service
        # startup, so having the number makes an "asr-service is slow to come up" report
        # answerable from an ordinary log rather than requiring a reproduction.
        started = time.perf_counter()
        # Resolved by NAME so that importing this module still does not import the engine
        # (see _import_engine): the factory is looked up on the class we were just handed,
        # not captured when the family table was written.
        try:
            factory = getattr(engine.OfflineRecognizer, family.factory)
        except AttributeError as exc:
            raise AsrEngineError(
                f"this sherpa-onnx build has no {family.factory}(); the {family.name!r} "
                f"family needs a newer engine"
            ) from exc
        try:
            if self._hotwords:
                # The engine reads the vocabulary from a PATH during construction, so the
                # normalized copy only has to exist across this call -- materialize()
                # removes it on the way out, including if construction raises.
                with materialize(self._hotwords) as hotwords_path:
                    kwargs["hotwords_file"] = hotwords_path
                    kwargs["hotwords_score"] = config.hotwords_score
                    kwargs["modeling_unit"] = family.hotword_unit
                    self._recognizer = factory(**kwargs)
            else:
                # No vocabulary: the biasing arguments are omitted entirely rather than
                # passed as empty values, so the engine takes its plain greedy path.
                self._recognizer = factory(**kwargs)
        except Exception as exc:
            # The engine raises loader-specific types from its C++ side, so a broad catch
            # is the only way to guarantee callers see one documented failure type; the
            # original is chained so nothing about the real cause is lost. This is the
            # single deliberate exception to the "no broad except" habit, and it is
            # confined to the one call that crosses into third-party native code.
            raise AsrEngineError(f"failed to build sherpa-onnx recognizer: {exc}") from exc
        load_ms = (time.perf_counter() - started) * 1000.0

        # One decode at a time: the thread budget above is sized for a single decode, so
        # overlapping two would oversubscribe the Orin and slow both. Acquired WITHOUT
        # blocking (see transcribe) -- this lock is an admission gate, not a queue.
        self._lock = threading.Lock()

        logger.info(
            "asr model ready: model=%s family=%s threads=%d method=%s provider=%s "
            "hotwords=%d load_ms=%.0f",
            self._model_name,
            family.name,
            config.num_threads,
            self._method,
            families.PROVIDER_CPU,
            len(self._hotwords),
            load_ms,
        )

    @property
    def model_name(self) -> str:
        """Model identifier reported by GET /status and in request logs."""
        return self._model_name

    @property
    def model_dir(self) -> str:
        """Directory the model was loaded from -- where 11 §11A.4.1 puts MODEL.json."""
        return self._model_dir

    @property
    def model_files(self) -> tuple:
        """Resolved network file paths, in a stable order, for GET /v1/models' sha256."""
        return self._model_files

    @property
    def decoding_method(self) -> str:
        """The decoding method actually in force (beam search when hotwords are loaded)."""
        return self._method

    @property
    def hotword_count(self) -> int:
        """How many distinct hotword phrases are biasing the decoder."""
        return len(self._hotwords)

    @property
    def hotwords_supported(self) -> bool:
        """Whether the loaded family can be biased at all.

        Reported alongside hotword_count because zero is ambiguous on its own: it means
        "the file was empty" on a transducer and "this architecture cannot be biased, so
        the file was never read" on every other family. The two have different remedies,
        and only this flag separates them.
        """
        return self._family.supports_hotwords

    @property
    def rejected_hotword_count(self) -> int:
        """How many authored phrases this model's symbol table cannot represent.

        Reported alongside hotword_count so a caller reading /status can tell "30 phrases
        are biasing the decoder" apart from "30 are, and 15 more were asked for and could
        not be" -- the second is a vocabulary that needs rewording, and it is invisible
        from the active count alone.
        """
        return len(self._rejected)

    def transcribe(self, audio: Audio) -> Transcript:
        """Decode one utterance. Blocking: call it from a worker thread, not the loop.

        Args:
            audio: the waveform to recognize, with its own sample rate.

        Returns:
            The Transcript, carrying the text, the timings its RTF is derived from, and a
            confidence when this model family reports one.

        Raises:
            AsrBusyError: if another decode is already in flight. The caller is refused,
                not queued -- see the concurrency note in the module docstring.
            AsrEngineError: if the engine fails while accepting audio or decoding.

        The sample rate is handed to the engine as-is rather than resampled here:
        sherpa-onnx converts to the model's rate internally, so a second conversion in
        this service would only add loss. A fresh stream per call keeps utterances
        independent -- no state from the previous request can leak into this one.
        """
        started = time.perf_counter()
        # Admission, not queueing: a failed acquire means REFUSE.
        #
        # The check lives HERE, inside the work function, and therefore runs on the worker
        # thread rather than on the event loop. Hoisting it into transcribe_async would
        # save a thread-pool dispatch per refused caller -- but it would also mean the lock
        # is held by a coroutine across an await, and a cancelled request would then run
        # its finally and RELEASE the lock while the worker it spawned is still decoding,
        # admitting a second concurrent decode. Mutual exclusion is the property the thread
        # budget is sized on, so it wins over the dispatch: a refusal costs one thread-pool
        # hop of a few microseconds, and the pool cannot be exhausted by refusals because
        # each one returns immediately.
        if not self._lock.acquire(blocking=False):
            raise AsrBusyError("a decode is already in flight; concurrency limit is 1")
        try:
            try:
                # A stream is the engine's per-utterance workspace: feed the whole
                # waveform, decode it, then read the text off the result. It is discarded
                # with this call, which is what keeps consecutive requests independent.
                stream = self._recognizer.create_stream()
                stream.accept_waveform(audio.sample_rate, audio.samples)
                self._recognizer.decode_stream(stream)
                result = stream.result
                text = result.text
                # Present on transducers, empty on Paraformer and the CTC families. The
                # attribute itself is read defensively because it is engine surface, not
                # ours: an engine upgrade that renames it must degrade to "no confidence
                # reported", never to a failed transcription.
                conf = _mean_log_prob_confidence(getattr(result, "ys_log_probs", None))
            except Exception as exc:
                # Same rationale as construction: normalize whatever the native layer
                # raises into this module's one documented failure type, cause preserved.
                raise AsrEngineError(f"decode failed: {exc}") from exc
        finally:
            self._lock.release()
        # Timed from before the admission check, so the reported figure is the latency the
        # caller actually experienced rather than the engine's own share of it.
        decode_ms = (time.perf_counter() - started) * 1000.0
        return Transcript(text=text, duration_s=audio.duration_s, decode_ms=decode_ms,
                          conf=conf)

    async def transcribe_async(self, audio: Audio) -> Transcript:
        """Decode one utterance without blocking the event loop.

        Args:
            audio: the waveform to recognize.

        Returns:
            The Transcript produced by transcribe().

        Raises:
            AsrBusyError: propagated from transcribe(). Note the admission check runs on
                the worker thread, not here -- see the comment in transcribe() for why
                that placement is deliberate.
            AsrEngineError: propagated from transcribe().

        Decoding is CPU-bound native work lasting hundreds of milliseconds; running it
        inline in the request coroutine would stall every other connection for that whole
        time, including the health check AI_runtime uses to decide the service is alive.
        asyncio.to_thread is the same "hand blocking work to a worker thread" discipline
        payload-service applies to its device writes.
        """
        return await asyncio.to_thread(self.transcribe, audio)

    @property
    def reports_confidence(self) -> bool:
        """Whether this family can produce the conf field 11 AS-4 asks for.

        Exposed so GET /status answers "is conf available here" once at deploy time,
        instead of leaving the gateway to infer it from a null in a transcription -- which
        cannot distinguish "this family never reports it" from "this one utterance had
        none".
        """
        return self._family.reports_confidence
