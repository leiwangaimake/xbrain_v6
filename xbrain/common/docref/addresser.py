"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: addresser.py
Brief: INF-QD-2 constraint ID (book, section, id_local) triple addresser

Description:
The constraint IDs scattered across the 22 books (S-1, T-PTZ-3,
SP-C1, etc.) collide across books: 12 §12.1's S-* and 15 §9.1's
S-* both exist and mean different things. Referring to a bare
S-1 in code invites the wrong book's implementation to answer.

This module accepts ONLY fully qualified triples:
  (book, section_anchor, id_local)

  * book:            '11', '12', '18-A', '99' etc (str)
  * section_anchor:  a grep-able string in that book
                     (e.g. '§14.6', 'PAY-38')
  * id_local:        the local identifier (e.g. 'S-1', 'T-PTZ-3')

Bare id_local input to any doccheck script MUST be refused; that
is the discipline the item's variants guard against.
"""

from __future__ import annotations

import re
from dataclasses import dataclass


BOOK_ID_RE = re.compile(r"^(0[0-9]|1[0-9]|2[0-9]|99|1[0-9]-[A-Z])$")


class ConstraintIdShapeError(Exception):
    """A bare id_local was passed without book + section context."""


@dataclass(frozen=True)
class ConstraintId:
    book: str            # e.g. '11', '18-A'
    section: str         # anchor grep-able in that book
    id_local: str        # local id, e.g. 'S-1' or 'T-PTZ-3'

    def __post_init__(self) -> None:
        if not isinstance(self.book, str) or not BOOK_ID_RE.match(self.book):
            raise ConstraintIdShapeError(
                f"book {self.book!r} does not match book-id regex "
                f"(examples: '11', '18-A', '99')")
        if not self.section:
            raise ConstraintIdShapeError(
                "section anchor required (grep-able string in the book)")
        if not self.id_local:
            raise ConstraintIdShapeError(
                "id_local required")

    def as_string(self) -> str:
        return f"{self.book} {self.section} {self.id_local}"


def require_triple(inp) -> ConstraintId:
    """Any doccheck script uses this at its entry point. Bare
    strings (e.g. 'S-1') raise; tuples of the wrong arity raise."""
    if isinstance(inp, ConstraintId):
        return inp
    if isinstance(inp, str):
        raise ConstraintIdShapeError(
            f"bare id {inp!r} refused; wrap it as ConstraintId("
            f"book, section, id_local) before feeding it to a "
            f"doccheck script (INF-QD-2)")
    if isinstance(inp, (tuple, list)):
        if len(inp) != 3:
            raise ConstraintIdShapeError(
                f"triple must have (book, section, id_local); got "
                f"{len(inp)} elements: {inp!r}")
        book, section, id_local = inp
        return ConstraintId(book=book, section=section, id_local=id_local)
    raise ConstraintIdShapeError(
        f"require_triple: unsupported input type "
        f"{type(inp).__name__} for {inp!r}")


def constraint_cover_diff(spec_ids, impl_ids):
    """§14.6 coverage helper: given the automation-eligible IDs from
    11 §14.6 (spec_ids) and the actually-implemented IDs
    (impl_ids), return the bidirectional diff:
      spec_only  -> spec says automatable, no one implemented
      impl_only  -> implemented a check for something not in §14.6"""
    spec = {c.as_string() for c in spec_ids}
    impl = {c.as_string() for c in impl_ids}
    return {
        "spec_only": tuple(sorted(spec - impl)),
        "impl_only": tuple(sorted(impl - spec)),
    }
