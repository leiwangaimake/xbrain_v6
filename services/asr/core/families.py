"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: families.py
Brief: The model families asr-service can load, and what each one can and cannot do.

Description:
  asr-service originally spoke exactly one dialect: an icefall zipformer TRANSDUCER, whose
  three network files, four-path layout, hotword biasing and beam-search coupling were
  spread as constants and branches through recognizer.py. That was correct while there was
  one model, and wrong the moment a second one had to be evaluated -- the differences
  between families are not stylistic, they change which files exist, which sherpa-onnx
  factory accepts them, and whether contextual biasing is expressible at all.

  This module is where those differences live, so recognizer.py can stay a story about
  loading a model and decoding an utterance rather than a switch statement about vendors.

  Why a table rather than subclasses: every family differs only in DATA -- a factory name,
  a set of file stems, a capability flag. There is no behaviour to override, so a class
  hierarchy would add indirection without adding expressiveness, and a reviewer comparing
  two families would have to read two files instead of two rows.

  ★ supports_hotwords is the field that carries real weight. In sherpa-onnx, contextual
  biasing exists ONLY on the transducer factory: from_paraformer, from_sense_voice and the
  various *_ctc entry points take no hotwords argument at all. That is not a gap in this
  service, it is a property of the architectures -- biasing steers a beam, and a CTC or
  attention decoder that emits its whole sequence in one shot has no beam to steer. Loading
  a vocabulary against such a family would therefore be a silently inert operation, which
  is exactly the failure mode core/hotwords.py was written to make impossible; so the flag
  is consulted BEFORE the vocabulary is read, and a family that cannot bias never reports a
  hotword count that would imply it can.

  ★ provider is pinned to CPU for every family, deliberately and not as a default. The
  whole AI Runtime memory plan (11 §11A.2, AIR-M3) is built on asr-service being pure CPU
  ONNX with zero device memory and no CUDA context of its own; V5's engine defaulted the
  other way, so an inherited default is the realistic way this invariant would get broken.
  Stating it here means the guarantee is visible at the place a new family gets added.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Tuple

# The one execution provider this service may use. See the provider note in the module
# docstring: this is a load-bearing invariant of the memory plan, not a tuning knob, so it
# is a module constant rather than a configuration field.
PROVIDER_CPU = "cpu"

# Decoding methods, named once so the hotword coupling below cannot drift from the strings
# sherpa-onnx actually accepts.
_METHOD_BEAM = "modified_beam_search"
_METHOD_GREEDY = "greedy_search"

# Modeling unit for hotword matching on a character-based Chinese transducer: each CJK
# character is its own unit, which is what makes a listed phrase bias wherever it occurs
# rather than only at the start of an utterance.
_UNIT_CJKCHAR = "cjkchar"


class FamilyError(ValueError):
    """Raised when a configured family name is not one this service can load.

    House rule bans bare Exception. ValueError is the base because the offending value is a
    configured string; a dedicated type lets startup report "you asked for a model family
    that does not exist" separately from "the files for a known family are missing", which
    are different mistakes with different fixes.
    """


@dataclass(frozen=True)
class Family:
    """One loadable model architecture and the facts recognizer.py needs about it.

    Frozen because it is a description of an architecture, not state: it is read during
    construction and shared thereafter, and a mutated entry would make the reported model
    name disagree with the weights actually loaded.
    """

    # Configuration name, matching the ASR_MODEL_FAMILY value an operator sets.
    name: str

    # Name of the sherpa_onnx.OfflineRecognizer classmethod that builds this family. Held
    # as a STRING rather than as the bound method so that importing this module never
    # imports the engine -- recognizer.py defers that import so its pure-python parts stay
    # testable on a machine with no inference runtime, and resolving a factory eagerly here
    # would quietly undo that.
    factory: str

    # Maps a sherpa-onnx keyword argument to the file stem it is satisfied by. The stems
    # are matched against the model directory, so a family whose export ships one network
    # file has one entry and the transducer families have three.
    files: Tuple[Tuple[str, str], ...]

    # ★ Whether sherpa-onnx can bias this family toward a hotword vocabulary at all.
    supports_hotwords: bool

    # ★ Whether decode results carry per-token log probabilities, from which the conf
    # field of 11 AS-4 can be derived. Recorded separately from supports_hotwords even
    # though the two currently agree on every row: they agree by coincidence, not by
    # construction -- one is about steering a beam, the other about what the result object
    # publishes -- and collapsing them would silently mispredict the first family where
    # they diverge.
    reports_confidence: bool

    # Extra keyword arguments this factory always needs, beyond files, tokens, threads,
    # decoding method and provider.
    extra: Tuple[Tuple[str, object], ...] = ()

    # Modeling unit for hotword matching. Meaningful only when supports_hotwords, and
    # passed only on the biased path -- the argument is part of the biasing configuration,
    # not of the model, and sending it on an unbiased build would describe a vocabulary
    # that is not there.
    hotword_unit: str = ""

    def decoding_method(self, with_hotwords: bool) -> str:
        """The decoding method to request, given whether a vocabulary is in force.

        Args:
            with_hotwords: True when a non-empty, biasable vocabulary was loaded.

        Returns:
            The sherpa-onnx decoding method name.

        Beam search is what makes biasing possible, so the two travel together rather than
        being configured independently: a configuration asking for greedy search AND
        hotwords would silently drop the hotwords, so it is not expressible.
        """
        return _METHOD_BEAM if with_hotwords else _METHOD_GREEDY


# The families this service can load. Adding one is a row here plus its files on disk --
# recognizer.py needs no change, which is the point of the table.
_FAMILIES: Dict[str, Family] = {
    # icefall zipformer transducer (encoder/decoder/joiner). The only family that can be
    # biased, and therefore the only one for which hotwords.txt has any effect.
    "transducer": Family(
        name="transducer",
        factory="from_transducer",
        files=(("encoder", "encoder"), ("decoder", "decoder"), ("joiner", "joiner")),
        supports_hotwords=True,
        reports_confidence=True,
        hotword_unit=_UNIT_CJKCHAR,
    ),
    # Alibaba Paraformer, one network file. No biasing -- see the module docstring.
    "paraformer": Family(
        name="paraformer",
        factory="from_paraformer",
        files=(("paraformer", "model"),),
        supports_hotwords=False,
        reports_confidence=False,
    ),
    # SenseVoice. use_itn stays off because the command layer wants the literal characters,
    # not "三号" rewritten to "3号"; language is pinned since V6 is a Chinese closed set.
    "sense_voice": Family(
        name="sense_voice",
        factory="from_sense_voice",
        files=(("model", "model"),),
        supports_hotwords=False,
        reports_confidence=False,
        extra=(("use_itn", False), ("language", "zh")),
    ),
    # Offline zipformer CTC, one network file.
    "zipformer_ctc": Family(
        name="zipformer_ctc",
        factory="from_zipformer_ctc",
        files=(("model", "model"),),
        supports_hotwords=False,
        reports_confidence=False,
    ),
}

# Exposed for config validation so the accepted set is declared in exactly one place.
FAMILY_NAMES = tuple(_FAMILIES)


def get(name: str) -> Family:
    """Look up a family by its configured name.

    Args:
        name: the ASR_MODEL_FAMILY value.

    Returns:
        The matching Family.

    Raises:
        FamilyError: when the name is not one of FAMILY_NAMES, listing what is.

    Failing loudly beats falling back to a default family: a typo would otherwise load a
    different architecture than the operator asked for, against a model directory that
    happens to satisfy it, and the only symptom would be an accuracy change nobody can
    explain.
    """
    try:
        return _FAMILIES[name]
    except KeyError as exc:
        raise FamilyError(
            f"unknown model family {name!r}; expected one of {', '.join(FAMILY_NAMES)}"
        ) from exc
