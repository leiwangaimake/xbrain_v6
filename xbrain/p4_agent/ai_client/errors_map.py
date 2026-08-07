"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: errors_map.py
Brief: The 11 S8.13.5 error map -- the ONE place an AI-service HTTP status or a
       transport failure becomes a closed-set error code

Description:
11 S8.13.5 is titled "错误映射(网关唯一实现点)": the AI services (ai_asr on
18081, llama-server on 18082) return free-text detail and ordinary HTTP status
codes, and turning those into the closed-set E_ codes the rest of the system
understands is the GATEWAY's job, not the service's (CLAUDE.md 3.5, 11 S8.13.5).
This module is that single implementation point. A business module never inspects
an HTTP status; it calls the ai_client and gets an XbrainError carrying a
closed-set code.

The map, verbatim from 11 S8.13.5:
  200            -> ok
  400 / 422      -> E_SCHEMA     (field-name drift; STOP retrying)          not retryable
  404            -> E_NOT_FOUND  (model / voice absent)                     not retryable
  429 / 503      -> E_BUSY       (AS-3 concurrency=1 violated; back off x1) retry x1
  500            -> E_INTERNAL   (service internal error)                   retry x1
  connect / timeout > AS-7 5 s -> E_TIMEOUT (feeds the breaker, 16 S9.3)    retryable

Traps -- things that look right and are not:
  1. Coercing an unmapped status to E_INTERNAL. A status this table does not name
    (e.g. a 418, or a 3xx redirect) is NOT silently an internal error -- it is a
    contract the service and gateway disagree on, and it raises E_SCHEMA so the
    disagreement is loud, the same fail-loud discipline the closed sets use. See
    map_status's default branch.
  2. Making 500 retryable-forever. 500 and 429/503 retry at most ONCE (S8.13.5
    "×1"); only a timeout / connection failure is freely retryable, because it is
    the breaker (16 S9.3), not this map, that bounds those.
  3. Reading E_* as strings. Every code returned here comes from
    xbrain.common.errors, never a literal (CLAUDE.md 3.5 / the no_literal_ecode
    lint), so a typo is an import error, not a code the subscriber never matches.
"""

from dataclasses import dataclass

from xbrain.common import errors

__all__ = ["MappedError", "map_status", "map_transport_error", "AS7_TIMEOUT_S"]

# 11 S8.13.5 / AS-7: the transport-timeout ceiling. A request that takes longer
# than this is treated as a timeout regardless of what the caller passed, so a
# stale 30 s default (the 16 S14 trap) cannot re-open the window AS-7 closes.
AS7_TIMEOUT_S = 5.0


@dataclass(frozen=True)
class MappedError:
    """One row of the S8.13.5 map: the closed-set code, whether the gateway may
    retry, and how many times.

    max_retries is carried alongside `retryable` rather than inferred from it,
    because the two failures 'retry once' (429/503/500) and 'retry until the
    breaker opens' (timeout) are different and a single boolean cannot tell them
    apart -- collapsing them is trap 2.
    """

    code: str          # a closed-set E_ code from xbrain.common.errors
    retryable: bool    # may the gateway retry at all
    max_retries: int   # how many times (0 when not retryable)


def map_status(http_status: int) -> MappedError:
    """Map an AI-service HTTP status to a MappedError (11 S8.13.5).

    An unmapped status raises through the default branch as E_SCHEMA, not
    E_INTERNAL (trap 1): a status the contract does not enumerate is a
    service/gateway disagreement, and E_SCHEMA is the code for "the shapes do not
    match".
    """
    if http_status == 200:
        # Not an error; callers should not route a 200 here, but be explicit
        # rather than fall to the default and mislabel a success as E_SCHEMA.
        return MappedError(errors.OK, False, 0)
    if http_status in (400, 422):
        # Field-name drift (the service changed version). Stop retrying -- a
        # retry sends the same malformed request and fails identically.
        return MappedError(errors.E_SCHEMA, False, 0)
    if http_status == 404:
        # A model or voice that does not exist. Not retryable: it will not appear.
        return MappedError(errors.E_NOT_FOUND, False, 0)
    if http_status in (429, 503):
        # Service busy -- AS-3 says concurrency is 1, so this means someone
        # bypassed the gateway. Back off and retry ONCE (S8.13.5 "×1").
        return MappedError(errors.E_BUSY, True, 1)
    if http_status == 500:
        # Service internal error. Retry once; a persistent 500 is a real fault.
        return MappedError(errors.E_INTERNAL, True, 1)
    # Trap 1: an unenumerated status is a contract mismatch, raised loud.
    return MappedError(errors.E_SCHEMA, False, 0)


def map_transport_error() -> MappedError:
    """Map a connection failure or a timeout to E_TIMEOUT (11 S8.13.5).

    Freely retryable HERE, because the bound on retries for this class of failure
    is the circuit breaker (16 S9.3: 3 consecutive -> open 60 s), not this map.
    max_retries is left at 0: the breaker, not a per-call counter, is what limits
    the timeout path, so a caller that treated max_retries as the limit would be
    reading the wrong mechanism.
    """
    return MappedError(errors.E_TIMEOUT, True, 0)
