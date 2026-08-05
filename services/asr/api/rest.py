"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: rest.py
Brief: asr-service HTTP surface -- OpenAI-compatible POST /v1/audio/transcriptions, GET /status.

Description:
  The one network face of asr-service (development plan section 5.3). AI_runtime cuts an
  utterance out of the mic stream with its VAD and posts it here as multipart form data;
  the response is the recognized text. Nothing in this module knows the GZH-2 device
  exists -- that separation is deliberate and is what keeps ASR out of the mode gate and
  off the device sockets (plan section 4, invariants R1/R2).

  Why the OpenAI shape rather than an endpoint of our own design: the same client code,
  the same curl command and the same debugging habits then work against this service, a
  hosted Whisper, or any other OpenAI-compatible ASR. The cost is accepting two fields we
  do not act on, which is a small and well-understood price for that portability.

    - model: this process serves exactly ONE loaded model (loading a second would double
      the memory and the startup time for no benefit on a single-box deploy), so the field
      is accepted and ignored. GET /status reports which model is actually in force, which
      is where a client should look rather than trusting what it asked for.
    - language: the deployed model is Chinese-only (multi-zh-hans), so there is nothing to
      switch. Accepted and ignored for the same compatibility reason.

  Deliberately NOT ignored is sample_rate: it is our own addition for the raw-PCM path,
  because raw samples carry no header to declare their rate and guessing wrong yields a
  confidently wrong transcription. A wav upload declares its own rate and this field is
  then unused.

  Error taxonomy (four codes, each telling the caller a different thing): a body that is
  empty, truncated, shorter than the model can decode, or not mono 16-bit audio is a 400 --
  the caller must fix what it sends; a decode already in flight is a 429 with a
  {"code": "E_BUSY"} body -- the caller is refused rather than queued (11 AS-12) and should
  come back under its own scheduling; an engine that is not loaded or fails mid-decode is a
  503 -- the transcription capability itself is in doubt; and anything malformed at the
  multipart layer is FastAPI's own 422 before this code runs. Collapsing these would leave
  AI_runtime unable to decide between "resend different audio", "retry shortly" and
  "something is wrong with the service" -- and 11 §8.13.5 reads a 503 as evidence that the
  concurrency limit was bypassed, so ordinary load must never be able to produce one.

  Streaming (WS /v1/audio/stream, plan section 5.3) is explicitly deferred and therefore
  absent -- house V1/V2 discipline: a route that 404s is honest, a stub that returns a
  fabricated partial transcript is not.
"""
from __future__ import annotations

import json
import logging
import os
import time

from typing import Dict, List, Optional

from fastapi import APIRouter, File, Form, HTTPException, Request, UploadFile
from fastapi.responses import JSONResponse
from pydantic import BaseModel

from ..core import model_meta
from ..core.audio_in import AudioInputError, decode_upload
from ..core.recognizer import AsrBusyError, AsrEngineError, Recognizer

# Module logger. House rule: log text is English even though the recognized content and
# the system's user-facing dialogue are Chinese.
logger = logging.getLogger("asr.rest")

router = APIRouter()

# Rate assumed for a raw-PCM upload that does not state one. 16 kHz is what the whole
# audio plane produces -- the device's [40] uplink and payload-service's /mic stream are
# both 16 kHz mono s16le -- so it is the only defensible default for the 功能1 path.
_DEFAULT_RAW_SAMPLE_RATE = 16000

# ★ The rate 11 AS-2 fixes for this interface, and the service REJECTS anything else.
#
# The whole audio plane is 16 kHz by construction -- the capture path decimates 48 k to
# 16 k inside p2_core before publishing rt/audio/mic (16 §3.0.2), and 11 §8.13.3 specifies
# the upload as 16 kHz WAV -- so a different rate arriving here means the capture chain is
# misconfigured, not that a caller has a legitimate variant.
#
# Accepting it would be the more forgiving choice and the wrong one, because sherpa-onnx
# resamples silently and the mistake then produces no error anywhere. 16 §3.0.2 records
# exactly this failure from the field: requesting 16 kHz from hardware that only does 48 k
# is accepted by ALSA with a warning nobody reads, and the symptom is "ASR gets everything
# wrong while every log line looks green". A 400 is what turns that into a five-second
# diagnosis instead of a days-long one.
#
# Set ASR_REQUIRE_SAMPLE_RATE=0 to accept any rate -- for a bench feeding archived audio
# at its original rate, where the strictness is noise rather than protection.
_CONTRACT_SAMPLE_RATE = 16000

# HTTP status codes, named so the handlers read as intent rather than as numbers.
# 11 §11A.8.1's /healthz status closed set. "loading" cannot be observed from outside this
# process -- uvicorn runs the lifespan, which loads the model, BEFORE it binds the socket,
# so a probe that connects at all is talking to a service past that point. It is defined
# here anyway because the contract defines it and because a future lazy-load would need it.
_STATUS_OK = "ok"
_STATUS_LOADING = "loading"
_STATUS_ERROR = "error"

# Gold corpus for POST /v1/selftest, beside the package rather than under a configured
# path: 11 §11A.8.1 calls it 内置离线 WAV, i.e. part of the build, not a deployment input.
_SELFTEST_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                             "selftest")
_SELFTEST_MANIFEST = "gold.json"
# ★ Only "asr" is accepted. 11 §11A.8.1 writes scope as "asr"|"tts"|"both", but that table
# describes the ai_audio process that merged ASR and TTS -- and 99 U52 §1 struck that shape
# down verbatim: TTS is in the GZH-2 device, 机上零显存零进程, and the process was renamed
# ai_asr. There is no TTS here to test, so accepting "tts" could only ever return a
# fabricated result. Asking for it is a 422, which is the honest answer.
_SELFTEST_SCOPES = ("asr",)

_HTTP_BAD_REQUEST = 400
_HTTP_TOO_MANY_REQUESTS = 429
_HTTP_UNPROCESSABLE = 422
_HTTP_SERVICE_UNAVAILABLE = 503

# Error code the gateway matches on when it is turned away. 11 §13 fixes the spelling, and
# 11 §11A.8.2 AS-12 fixes when it is emitted, so it is a constant rather than a literal.
_CODE_BUSY = "E_BUSY"


class ModelRef(BaseModel):
    """Which model produced a result (11 AS-11: name + version on every call).

    It rides on every transcription rather than being fetched once from GET /status,
    because 11 §11A.7 has the gateway file this into state/voice_turn per turn and calls it
    「换模型后金标集回归失败时唯一能对上的线索」. A turn that crosses a service restart --
    exactly when the model may have changed -- is the case an out-of-band lookup cannot
    attribute, and it is the only case this field exists for.
    """

    # Directory-and-precision identifier, always present.
    name: str
    # ★ Null until 11 §11A.4.1's MODEL.json exists beside the model. Deliberately not
    # faked; see core/model_meta.py for why a placeholder version is worse than none.
    version: Optional[str] = None


class ModelEntry(BaseModel):
    """One row of GET /v1/models (11 §11A.8.1: name / version / sha256 / loaded)."""

    name: str
    version: Optional[str] = None
    # Digest over the model files as they are on disk right now -- computed, not read from
    # MODEL.json, so it can contradict the sidecar. Null when a file could not be read.
    sha256: Optional[str] = None
    loaded: bool


class AsrSelftestResult(BaseModel):
    """The asr half of POST /v1/selftest (11 §11A.8.1: {ok, wer})."""

    ok: bool
    # ★ Character error rate. The contract's field is named wer, and the name is kept, but
    # the unit is characters: the gold corpus is Chinese, which has no word boundaries to
    # tokenise on, so a word-level rate would either need a segmenter (whose disagreements
    # would show up as model regressions) or would degenerate to whole-utterance accuracy.
    wer: float
    # How many gold utterances ran. Present so a corpus that shrank to one file cannot
    # produce a confident-looking 0.0 that means nothing.
    utterances: int


class SelftestResponse(BaseModel):
    """Body of POST /v1/selftest. Only the asr half exists -- see _SELFTEST_SCOPES."""

    asr: AsrSelftestResult


class HealthResponse(BaseModel):
    """Body of GET /healthz -- the shape 11 §11A.8.1 registers, minus the tts half.

    ⚠️ This is the route 11 §11A.5.1 probe ① polls at 1 Hz with a 2 s timeout (T-48), and
    its rule is `200 + status=="ok"` → ok, anything else → fail, three fails → 熔断. So the
    handler must not do anything that can block: no disk reads, no hashing, no lock the
    decode path also takes. Everything below is either an in-memory attribute or a single
    read of /proc/self/statm.

    models keys off "asr" only. 11 §11A.8.1 shows models:{asr:{...},tts:{...}} because it
    was written for the merged ai_audio process; 99 U52 §1 struck that merge down and
    renamed the process ai_asr, TTS having turned out to live in the GZH-2 device. An empty
    tts object here would assert that a TTS exists and is fine, which is worse than absent.
    """

    status: str
    models: Dict[str, ModelRef]
    # Resident set size of this process in MiB. Reported because the contract's row asks
    # for it and because AIR-M1's memory ledger has no other in-band source for the ASR
    # line -- an operator otherwise has to ssh in and run ps to check a budgeted number.
    mem_mb: int


class TranscriptionResponse(BaseModel):
    """Body of POST /v1/audio/transcriptions -- the OpenAI json response shape plus AS-4.

    "text" is the OpenAI-portable core. The three fields beside it are what 11 AS-4
    requires of every ASR call, and they are returned rather than merely logged because
    the gateway cannot reconstruct them: it would have to re-derive the duration from the
    bytes it uploaded and time the RTF with a wall clock that includes its own queueing --
    both of which 11 §8.13.3 lists only as FALLBACKS for a service that does not publish
    them. A client that also talks to a hosted OpenAI endpoint simply ignores the extras,
    which is the standing cost of the compatibility shape and a much smaller one than
    making the contract's fields unobtainable.
    """

    # The recognized utterance. Empty string when the audio contained no speech, which is
    # a normal outcome for a VAD false trigger, not an error.
    text: str

    # Length of the decoded audio in milliseconds (AS-4). Measured from the samples that
    # actually reached the engine, so it reflects what was transcribed rather than what
    # the caller believes it sent.
    audio_duration_ms: int

    # Decode time divided by audio duration (AS-4). Server-side and engine-only: it
    # excludes HTTP and multipart parsing, so it stays comparable as a model metric and
    # does not drift with the caller's network. RTF < 0.5 is this service's hard
    # acceptance requirement, and this is the number that requirement is about.
    rtf: float

    # ★ Normalized [0, 1] confidence (AS-4), or null when the loaded model family does not
    # report the per-token log probabilities it is derived from -- Paraformer and the CTC
    # families do not. Null means "not measured" and MUST NOT be read as low confidence;
    # GET /status answers whether this deployment can produce it at all, so a caller can
    # tell a family-wide absence from a per-utterance one. Publishing null rather than a
    # plausible constant is deliberate: conf exists so a caller can distrust a
    # transcription, and a fabricated value would make every transcription look equally
    # trustworthy, which is worse than having no field.
    conf: Optional[float] = None

    # ★ 11 §11A.8.1 registers this beside model: how many hotword phrases actually biased
    # THIS decode. Not the authored count -- phrases the symbol table cannot represent are
    # already excluded, and on a family with no beam to steer it is structurally 0. It is
    # per-response rather than only on /status because a caller comparing two transcriptions
    # needs to know whether biasing differed between them.
    hotwords_applied: int

    # ★ 11 AS-11. Present on every call, with version null until MODEL.json exists.
    model: ModelRef


class StatusResponse(BaseModel):
    """Body of GET /status -- what this process actually loaded.

    This is the route AI_runtime polls before entering 功能1 and the one an operator hits
    to confirm a deploy. It answers the questions that a wrong deploy would otherwise turn
    into a mysterious quality problem: which precision is loaded, is beam search actually
    running, and did the hotword vocabulary really get picked up.
    """

    # Identifier of the loaded model: the export directory's name plus the precision, with
    # the family prefixed only when the directory does not already imply it. So the default
    # deploy reports "paraformer-zh-2023-09-int8", while the transducer export reports
    # "transducer-zipformer-multi-zh-hans-int8". Derived in recognizer.py -- see the note
    # there on why there is a name but not yet a version.
    model: str
    # Decoding method in force: modified_beam_search when hotwords are loaded, else greedy.
    decoding: str
    # Number of distinct hotword phrases biasing the decoder. Zero means no biasing --
    # which on a non-transducer family is the only possible value, see hotwords_supported.
    hotwords: int
    # Authored phrases this model's symbol table cannot represent, so they bias nothing.
    # Non-zero is not a fault -- it is a vocabulary/model mismatch worth rewording -- but
    # it has to be visible here, because the active count alone cannot reveal it.
    hotwords_rejected: int
    # ★ Whether this family can be biased at all. Distinguishes "the vocabulary is empty"
    # from "this architecture has no beam to steer, so hotwords.txt is inert here" -- two
    # situations that both show hotwords: 0 and have completely different remedies.
    hotwords_supported: bool
    # ★ Whether transcriptions from this deployment carry a conf value (11 AS-4). Answered
    # here, once, so a caller meeting conf: null in a response can tell a family-wide
    # absence from a single utterance that produced none.
    conf_supported: bool


def _model_ref(recognizer: Recognizer) -> ModelRef:
    """Build the AS-11 model identity for the currently loaded model."""
    return ModelRef(name=recognizer.model_name,
                    version=model_meta.read_version(recognizer.model_dir))


def _resident_mib() -> int:
    """This process's resident set size in MiB, or 0 if it cannot be read.

    Reads /proc/self/statm, whose second field is resident pages. resource.getrusage was
    not used because its ru_maxrss is the PEAK, and a health probe reporting a high-water
    mark from a startup spike would misrepresent steady state for the rest of the run.

    Returns 0 rather than raising on any failure: this feeds a health response, and 11
    §11A.5.1 turns a non-200 into `fail` and then into a circuit break, so no auxiliary
    field is worth failing the probe over.
    """
    try:
        with open("/proc/self/statm", "r", encoding="ascii") as handle:
            resident_pages = int(handle.read().split()[1])
    except (OSError, IndexError, ValueError):
        return 0
    return resident_pages * os.sysconf("SC_PAGE_SIZE") // (1024 * 1024)


def _character_error_rate(reference: str, hypothesis: str) -> float:
    """Levenshtein distance over characters, divided by the reference length.

    Args:
        reference: the gold transcript.
        hypothesis: what the engine produced.

    Returns:
        distance / len(reference), not clamped -- an insertion-heavy hypothesis can exceed
        1.0, and hiding that behind a cap would make a badly broken model look merely bad.
        An empty reference returns 0.0 for an empty hypothesis and 1.0 otherwise.

    Whitespace is stripped from both sides before comparing, because the gold text is
    written for humans to read and the engine emits none for Chinese; a space would
    otherwise count as a substitution on every utterance.
    """
    reference = "".join(reference.split())
    hypothesis = "".join(hypothesis.split())
    if not reference:
        return 0.0 if not hypothesis else 1.0
    # Single-row Levenshtein: the corpus is a handful of short utterances, so the O(n*m)
    # time is irrelevant, but keeping one row makes the loop easy to check by eye.
    previous = list(range(len(hypothesis) + 1))
    for i, ref_char in enumerate(reference, start=1):
        current = [i]
        for j, hyp_char in enumerate(hypothesis, start=1):
            current.append(min(previous[j] + 1,          # deletion
                               current[j - 1] + 1,       # insertion
                               previous[j - 1] + (ref_char != hyp_char)))
        previous = current
    return previous[-1] / len(reference)


@router.get("/status", response_model=StatusResponse)
async def get_status(request: Request) -> StatusResponse:
    """Report the loaded model, decoding method and hotword count.

    Args:
        request: the incoming request, used only to reach the Recognizer on app.state.

    Returns:
        A StatusResponse describing this process's loaded engine.

    Every value comes from memory, so the coroutine neither awaits nor blocks and is safe
    to run directly on the event loop. The recognizer is guaranteed present because the
    lifespan builds it before the server accepts connections -- a failed load aborts
    startup rather than leaving a half-ready service answering requests.
    """
    # Reach the single Recognizer built during lifespan startup.
    recognizer: Recognizer = request.app.state.recognizer
    return StatusResponse(
        model=recognizer.model_name,
        decoding=recognizer.decoding_method,
        hotwords=recognizer.hotword_count,
        hotwords_rejected=recognizer.rejected_hotword_count,
        hotwords_supported=recognizer.hotwords_supported,
        conf_supported=recognizer.reports_confidence,
    )


@router.get("/healthz", response_model=HealthResponse)
async def get_healthz(request: Request) -> HealthResponse:
    """Liveness for 11 §11A.5.1 probe ①, polled at 1 Hz with a 2 s budget (T-48).

    Args:
        request: the incoming request, used only to reach the Recognizer on app.state.

    Returns:
        A HealthResponse whose status is "ok" whenever this process can serve.

    ★ Why this exists next to GET /status rather than instead of it. The probe's rule is
    `200 + status=="ok"`, and StatusResponse has no status field -- so pointing the probe at
    /status would fail it just as surely as the 404 did. The two routes also answer
    different questions: /status is a human diagnostic ("which precision, is beam search on,
    did the hotwords load"), while this one is a machine's yes/no. Merging them would drag
    diagnostic fields into a hot path polled every second, forever.

    ⚠️ The handler is deliberately trivial. It does not touch the decode lock, does not
    read the model files, and does not call the engine. A probe that blocked behind an
    in-flight decode would report `fail` precisely while the service was doing its job --
    three of those and 16 §9.3 opens the breaker on a healthy service for 60 s.
    """
    recognizer: Recognizer = request.app.state.recognizer
    # Reaching this handler at all means the lifespan finished and the model is loaded:
    # uvicorn runs the lifespan to completion before binding, and a load failure aborts
    # startup rather than leaving a degraded service answering. Hence a constant "ok".
    return HealthResponse(status=_STATUS_OK,
                          models={"asr": _model_ref(recognizer)},
                          mem_mb=_resident_mib())


@router.get("/v1/models", response_model=List[ModelEntry])
async def get_models(request: Request) -> List[ModelEntry]:
    """Publish the loaded model's identity and digest (11 §11A.8.1).

    Args:
        request: the incoming request, used only to reach the Recognizer on app.state.

    Returns:
        A one-element list: this service loads exactly one model. A list rather than an
        object because the contract calls it 模型清单 and because a second entry must be
        addable without changing the response type.

    ⚠️ Unlike /healthz this route hashes the model files, which for the int8 paraformer
    export is tens of megabytes of reading on the first call. That is why the two are
    separate routes and why 11 §11A.5.1 polls the other one: this is a deploy-time and
    audit-time question, asked rarely.
    """
    recognizer: Recognizer = request.app.state.recognizer
    return [ModelEntry(name=recognizer.model_name,
                       version=model_meta.read_version(recognizer.model_dir),
                       sha256=model_meta.hash_files(recognizer.model_files),
                       # This process aborts startup if the load fails, so serving a
                       # response is itself the proof that the model is loaded.
                       loaded=True)]


@router.post("/v1/selftest", response_model=SelftestResponse)
async def post_selftest(request: Request, body: Optional[dict] = None) -> SelftestResponse:
    """Deep BIT: decode the built-in gold WAVs and compare against their gold text.

    Args:
        request: the incoming request, used to reach the Recognizer on app.state.
        body: optional {"scope": "asr"}. Absent or empty means "asr", the only scope this
            process has anything to test.

    Returns:
        A SelftestResponse carrying the mean character error rate over the corpus and
        whether it is within the shipped threshold.

    Raises:
        HTTPException: 422 if scope is outside the closed set; 503 if the gold corpus is
            missing or unreadable; 429 if a transcription is already in flight.

    ★ A missing corpus is a 503, never a pass. This route exists to answer "is the model
    that got deployed the model that was tested", and the way that answer goes wrong is by
    returning ok:true because there was nothing to disagree with. If the corpus cannot be
    read, the honest report is that the self-test could not run.

    ⚠️ It goes through the ordinary decode path, so it takes the AS-3 concurrency slot and
    will 429 against a live transcription -- correct, because a self-test that ran beside
    real traffic would measure contention rather than the model, and because AS-12 forbids
    queueing behind the slot.
    """
    scope = (body or {}).get("scope") or _SELFTEST_SCOPES[0]
    if scope not in _SELFTEST_SCOPES:
        raise HTTPException(
            status_code=_HTTP_UNPROCESSABLE,
            detail=f"scope {scope!r} is not one of {list(_SELFTEST_SCOPES)}; "
                   "TTS is in the GZH-2 device, not this process (99 U52)")

    manifest_path = os.path.join(_SELFTEST_DIR, _SELFTEST_MANIFEST)
    try:
        with open(manifest_path, "r", encoding="utf-8") as handle:
            manifest = json.load(handle)
        threshold = float(manifest["max_cer"])
        cases = manifest["cases"]
        if not cases:
            raise ValueError("gold corpus is empty")
    except (OSError, ValueError, KeyError, TypeError) as error:
        raise HTTPException(status_code=_HTTP_SERVICE_UNAVAILABLE,
                            detail=f"self-test corpus unusable at {manifest_path}: "
                                   f"{error}") from error

    recognizer: Recognizer = request.app.state.recognizer
    rates = []
    started = time.perf_counter()
    for case in cases:
        wav_path = os.path.join(_SELFTEST_DIR, case["wav"])
        try:
            with open(wav_path, "rb") as handle:
                # raw_sample_rate is irrelevant: the gold files are RIFF WAV, so the rate
                # comes from their header, and a non-16 kHz gold file must fail loudly
                # here rather than be silently reinterpreted.
                audio = decode_upload(handle.read(), _CONTRACT_SAMPLE_RATE)
        except (OSError, AudioInputError) as error:
            raise HTTPException(status_code=_HTTP_SERVICE_UNAVAILABLE,
                                detail=f"gold wav {wav_path} unusable: {error}") from error
        try:
            transcript = await recognizer.transcribe_async(audio)
        except AsrBusyError as exc:
            return JSONResponse(status_code=_HTTP_TOO_MANY_REQUESTS,
                                content={"code": _CODE_BUSY, "detail": str(exc)})
        except AsrEngineError as exc:
            raise HTTPException(status_code=_HTTP_SERVICE_UNAVAILABLE,
                                detail=str(exc)) from exc
        rates.append(_character_error_rate(case["text"], transcript.text))

    cer = sum(rates) / len(rates)
    logger.info("selftest: utterances=%d cer=%.4f threshold=%.4f elapsed_ms=%.0f",
                len(rates), cer, threshold, (time.perf_counter() - started) * 1000.0)
    return SelftestResponse(asr=AsrSelftestResult(ok=cer <= threshold,
                                                  wer=round(cer, 4),
                                                  utterances=len(rates)))


@router.post("/v1/audio/transcriptions", response_model=TranscriptionResponse)
async def post_transcriptions(
    request: Request,
    file: UploadFile = File(...),
    model: str = Form(default=""),
    language: str = Form(default=""),
    sample_rate: int = Form(default=_DEFAULT_RAW_SAMPLE_RATE),
) -> TranscriptionResponse:
    """Transcribe one uploaded utterance (wav or raw mono s16le PCM).

    Args:
        request: the incoming request, used to reach the Recognizer on app.state.
        file: the audio body. A RIFF wav is detected by its magic bytes and read with its
            own header; anything else is treated as raw mono 16-bit little-endian PCM.
        model: accepted for OpenAI compatibility and ignored -- see the module docstring.
        language: accepted for OpenAI compatibility and ignored.
        sample_rate: rate of the raw-PCM body in Hz; unused for a wav, which declares its
            own. Defaults to the 16 kHz the rest of the audio plane uses.

    Returns:
        A TranscriptionResponse carrying the recognized text and the AS-4 measurements.
        On a refused request the return is a JSONResponse carrying 429 instead.

    Raises:
        HTTPException: 400 when the body is empty, too short for the model, or not
            decodable mono 16-bit audio; 503 when the engine fails to decode it.

    The decode itself is awaited through the recognizer's worker-thread wrapper, so this
    coroutine yields the event loop for the hundreds of milliseconds the native inference
    runs -- other connections, including the /status health check, stay responsive.

    The result is logged with its RTF because RTF < 0.5 is this service's hard acceptance
    requirement: logging it per request turns that requirement into something continuously
    observable on the real box rather than something only a benchmark ever measures.
    """
    # Reach the single Recognizer built during lifespan startup, and the config snapshot
    # beside it -- the minimum-duration floor is a property of the loaded model, so it
    # travels with the deployment rather than being a constant in this module.
    recognizer: Recognizer = request.app.state.recognizer
    config = request.app.state.config

    # Pull the whole body into memory. Utterances are VAD-cut single sentences (a few
    # seconds, well under a megabyte at 16 kHz mono), so buffering is bounded and far
    # simpler than streaming it to a temp file -- and the engine needs the complete
    # waveform anyway, since this is batch (not streaming) recognition.
    data = await file.read()

    try:
        # One call handles both accepted shapes; the RIFF sniff inside decides which.
        audio = decode_upload(data, sample_rate, config.min_audio_ms)
    except AudioInputError as exc:
        # Caller-fixable: the bytes are not audio this service can read. Log at warning
        # (not error) because a bad upload is the client's bug, not this service's fault.
        logger.warning("rejected upload: %s", exc)
        raise HTTPException(status_code=_HTTP_BAD_REQUEST, detail=str(exc)) from exc

    required = config.require_sample_rate
    if required and audio.sample_rate != required:
        # See _CONTRACT_SAMPLE_RATE for why this is a rejection rather than a resample.
        detail = (
            f"audio is {audio.sample_rate} Hz; this interface is specified for "
            f"{required} Hz (11 AS-2). A different rate means the capture path is "
            f"misconfigured -- resampling it here would hide that and quietly degrade "
            f"recognition. Set ASR_REQUIRE_SAMPLE_RATE=0 to accept any rate."
        )
        logger.warning("rejected upload: %s", detail)
        raise HTTPException(status_code=_HTTP_BAD_REQUEST, detail=detail)

    try:
        transcript = await recognizer.transcribe_async(audio)
    except AsrBusyError as exc:
        # Refused, not broken: a decode is already running and AS-12 forbids queueing this
        # one behind it. Returned as a JSONResponse rather than raised as an HTTPException
        # because the contract fixes the BODY shape as {"code": "E_BUSY"}, and
        # HTTPException would wrap whatever it is given inside FastAPI's own {"detail":…}
        # envelope, which the gateway does not match on.
        logger.info("refused (busy): %s", exc)
        return JSONResponse(status_code=_HTTP_TOO_MANY_REQUESTS,
                            content={"code": _CODE_BUSY, "message": str(exc)})
    except AsrEngineError as exc:
        # Operational: the engine could not produce a result. 503 tells the caller to
        # retry rather than to change its request.
        logger.error("decode failed: %s", exc)
        raise HTTPException(status_code=_HTTP_SERVICE_UNAVAILABLE, detail=str(exc)) from exc

    # The acceptance line. Text length rather than the text itself keeps the recognized
    # content out of routine logs while still showing that something was recognized.
    logger.info(
        "transcribed: audio_s=%.2f decode_ms=%.0f rtf=%.3f conf=%s chars=%d",
        transcript.duration_s,
        transcript.decode_ms,
        transcript.rtf,
        "n/a" if transcript.conf is None else f"{transcript.conf:.3f}",
        len(transcript.text),
    )
    return TranscriptionResponse(
        text=transcript.text,
        audio_duration_ms=round(transcript.duration_s * 1000.0),
        rtf=round(transcript.rtf, 4),
        conf=None if transcript.conf is None else round(transcript.conf, 4),
        hotwords_applied=recognizer.hotword_count,
        # AS-11. version is read from disk per call, which is one stat of a small JSON
        # beside a model that is already in the page cache; it is read here rather than
        # cached at startup so that a MODEL.json dropped in by a deploy takes effect on the
        # next request instead of at the next restart.
        model=_model_ref(recognizer),
    )
