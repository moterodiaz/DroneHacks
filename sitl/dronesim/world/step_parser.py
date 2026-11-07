"""Reader for ISO 10303-21 ("STEP Part 21") exchange files.

Only the clear-text encoding is supported, which is what every CAD exporter
writes.  The file is a HEADER section followed by one or more DATA sections::

    ISO-10303-21;
    HEADER; FILE_DESCRIPTION(...); FILE_NAME(...); FILE_SCHEMA(...); ENDSEC;
    DATA;
    #10=CARTESIAN_POINT('',(0.,0.,1.E-05));          <- simple instance
    #11=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT($,.METRE.));  <- complex instance
    ENDSEC;
    END-ISO-10303-21;

Parameter encoding (Part 21 section 7):

=====================  ==========================================
``'text'``              :class:`str` (``''`` is an escaped quote;
                        ``\\X2\\...\\X0\\`` etc. are decoded)
``12`` / ``1.E-05``     :class:`int` / :class:`float` (a real always
                        has a ``.``; ``0.`` is valid)
``#12``                 :class:`Ref`
``.T.`` / ``.F.``       ``True`` / ``False``
``.NAME.``              :class:`Enum` (a ``str`` subclass, no dots)
``$``                   ``None`` (unset optional attribute)
``*``                   :data:`DERIVED` (attribute derived by the schema)
``(a,b,...)``           :class:`list` (nested to any depth)
``TYPE(args)``          :class:`Typed` (e.g. ``LENGTH_MEASURE(1.E-8)``)
``"0FF"``               :class:`Binary`
=====================  ==========================================

A complex instance lists its partial entities in the alphabetical order of
the schema's external mapping, *not* in the attribute order of the leaf type.
Use :attr:`Entity.parts` to read one partial entity's attributes by name.

Parsing is lazy: :meth:`StepFile.parse` splits the file into records and
indexes them by id and type name; a record's parameters are tokenised the
first time :meth:`StepFile.get` asks for it.  References stay :class:`Ref`
values until the caller resolves them, so cyclic graphs are not a problem.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterator, List, Optional, Tuple, Union

__all__ = [
    "Binary",
    "DERIVED",
    "Entity",
    "Enum",
    "Ref",
    "StepFile",
    "StepSyntaxError",
    "Typed",
]


class StepSyntaxError(ValueError):
    """Raised when the file is not valid Part 21 clear text."""


class Ref:
    """An entity instance reference ``#id``."""

    __slots__ = ("id",)

    def __init__(self, id: int) -> None:
        self.id = int(id)

    def __repr__(self) -> str:
        return f"#{self.id}"

    def __eq__(self, other: object) -> bool:
        return isinstance(other, Ref) and other.id == self.id

    def __hash__(self) -> int:
        return hash(("Ref", self.id))


class Enum(str):
    """An enumeration value such as ``.SURF_OF_LINEAR_EXTRUSION.`` (stored without dots)."""

    def __repr__(self) -> str:
        return f".{str(self)}."


class Binary(str):
    """A binary literal ``"0FF"`` (stored as its hex digits)."""


class _Derived:
    """The ``*`` placeholder for an attribute that a subtype redeclares as derived."""

    _instance: Optional["_Derived"] = None

    def __new__(cls) -> "_Derived":
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __repr__(self) -> str:
        return "*"


DERIVED = _Derived()


@dataclass(frozen=True)
class Typed:
    """A typed parameter such as ``COUNT_MEASURE(0.)``; ``args`` is the parameter list."""

    type: str
    args: List[Any]

    @property
    def value(self) -> Any:
        """The single wrapped value (typed parameters almost always hold exactly one)."""
        return self.args[0] if len(self.args) == 1 else self.args


@dataclass
class Entity:
    """One entity instance from the DATA section.

    ``type`` is the entity name of a simple instance.  For a complex instance it
    is the partial entity names in file order, space separated and parenthesised
    (``"(LENGTH_UNIT NAMED_UNIT SI_UNIT)"``), ``args`` is the concatenation of
    all partial parameter lists, and ``parts`` maps each partial entity name to
    its own parameter list.  For simple instances ``parts == {type: args}`` so
    that ``entity.parts.get(NAME)`` works uniformly.
    """

    id: int
    type: str
    args: List[Any]
    parts: Dict[str, List[Any]] = field(default_factory=dict)

    @property
    def is_complex(self) -> bool:
        return self.type.startswith("(")

    def isa(self, type_name: str) -> bool:
        """True when ``type_name`` is this instance's type or one of its partial entities."""
        return type_name in self.parts

    def part(self, type_name: str) -> List[Any]:
        """Parameters of one partial entity; ``KeyError`` naming the entity if absent."""
        try:
            return self.parts[type_name]
        except KeyError:
            raise KeyError(f"#{self.id} {self.type} has no {type_name} part") from None

    def __repr__(self) -> str:
        return f"#{self.id}={self.type}{tuple(self.args)!r}"


# --------------------------------------------------------------------------
# Tokeniser
# --------------------------------------------------------------------------

_TOKEN_RE = re.compile(
    r"""
      (?P<ws>\s+)
    | (?P<comment>/\*.*?\*/)
    | (?P<string>'(?:[^']|'')*')
    | (?P<ref>\#\d+)
    | (?P<enum>\.[A-Za-z_][A-Za-z0-9_]*\.)
    | (?P<real>[+-]?(?:\d+\.\d*(?:[eE][+-]?\d+)?|\d+[eE][+-]?\d+))
    | (?P<int>[+-]?\d+)
    | (?P<keyword>!?[A-Za-z_][A-Za-z0-9_]*)
    | (?P<binary>"[0-9A-Fa-f]*")
    | (?P<punct>[(),$*=])
    """,
    re.VERBOSE | re.DOTALL,
)

# Splits the file into records at ';' while skipping strings and comments.
_RECORD_SPLIT_RE = re.compile(r"'(?:[^']|'')*'|/\*.*?\*/|;", re.DOTALL)
_LEADING_JUNK_RE = re.compile(r"(?:\s+|/\*.*?\*/)*", re.DOTALL)
_INSTANCE_HEAD_RE = re.compile(r"#(\d+)\s*=\s*", re.DOTALL)
_KEYWORD_RE = re.compile(r"!?[A-Za-z_][A-Za-z0-9_]*")
_DATA_KEYWORD_RE = re.compile(r"DATA\s*(?:\(|$)")

_ESCAPE_RE = re.compile(r"\\X2\\((?:[0-9A-Fa-f]{4})*)\\X0\\|\\X4\\((?:[0-9A-Fa-f]{8})*)\\X0\\"
                        r"|\\X\\([0-9A-Fa-f]{2})|\\S\\(.)|\\P[A-I]\\|\\\\")


def _decode_string(raw: str) -> str:
    """Decode a quoted Part 21 string literal (quotes included in ``raw``)."""
    text = raw[1:-1].replace("''", "'")
    if "\\" not in text:
        return text

    def sub(m: "re.Match[str]") -> str:
        if m.group(1) is not None:
            h = m.group(1)
            return "".join(chr(int(h[i:i + 4], 16)) for i in range(0, len(h), 4))
        if m.group(2) is not None:
            h = m.group(2)
            return "".join(chr(int(h[i:i + 8], 16)) for i in range(0, len(h), 8))
        if m.group(3) is not None:
            return chr(int(m.group(3), 16))
        if m.group(4) is not None:  # \S\c : ISO 8859 upper half
            return chr(ord(m.group(4)) + 128)
        if m.group(0) == "\\\\":
            return "\\"
        return ""  # \PA\ .. \PI\ code page switches carry no text

    return _ESCAPE_RE.sub(sub, text)


def _tokenize(text: str, where: str) -> List[Tuple[str, str]]:
    tokens: List[Tuple[str, str]] = []
    pos = 0
    n = len(text)
    match = _TOKEN_RE.match
    while pos < n:
        m = match(text, pos)
        if m is None:
            raise StepSyntaxError(f"{where}: unexpected character {text[pos]!r} in {text[max(0, pos - 20):pos + 20]!r}")
        kind = m.lastgroup
        if kind not in ("ws", "comment"):
            tokens.append((kind, m.group()))  # type: ignore[arg-type]
        pos = m.end()
    return tokens


class _ParamParser:
    """Recursive-descent parser over a token list for one record's parameters."""

    __slots__ = ("tokens", "pos", "where")

    def __init__(self, tokens: List[Tuple[str, str]], where: str) -> None:
        self.tokens = tokens
        self.pos = 0
        self.where = where

    def _fail(self, msg: str) -> "StepSyntaxError":
        near = " ".join(t for _, t in self.tokens[max(0, self.pos - 3):self.pos + 3])
        return StepSyntaxError(f"{self.where}: {msg} near {near!r}")

    def peek(self) -> Tuple[str, str]:
        if self.pos >= len(self.tokens):
            raise self._fail("unexpected end of record")
        return self.tokens[self.pos]

    def expect(self, text: str) -> None:
        kind, tok = self.peek()
        if tok != text:
            raise self._fail(f"expected {text!r}, got {tok!r}")
        self.pos += 1

    def at_end(self) -> bool:
        return self.pos >= len(self.tokens)

    def param_list(self) -> List[Any]:
        """``( p, p, ... )`` -> list."""
        self.expect("(")
        out: List[Any] = []
        if self.peek()[1] == ")":
            self.pos += 1
            return out
        while True:
            out.append(self.param())
            kind, tok = self.peek()
            self.pos += 1
            if tok == ")":
                return out
            if tok != ",":
                raise self._fail(f"expected ',' or ')', got {tok!r}")

    def param(self) -> Any:
        kind, tok = self.peek()
        if kind == "ref":
            self.pos += 1
            return Ref(int(tok[1:]))
        if kind == "real":
            self.pos += 1
            return float(tok)
        if kind == "int":
            self.pos += 1
            return int(tok)
        if kind == "string":
            self.pos += 1
            return _decode_string(tok)
        if kind == "enum":
            self.pos += 1
            name = tok[1:-1]
            if name == "T":
                return True
            if name == "F":
                return False
            return Enum(name)
        if kind == "punct":
            if tok == "(":
                return self.param_list()
            if tok == "$":
                self.pos += 1
                return None
            if tok == "*":
                self.pos += 1
                return DERIVED
            raise self._fail(f"unexpected {tok!r}")
        if kind == "keyword":
            self.pos += 1
            return Typed(tok, self.param_list())
        if kind == "binary":
            self.pos += 1
            return Binary(tok[1:-1])
        raise self._fail(f"unexpected token {tok!r}")


def _parse_body(body: str, where: str) -> Tuple[str, List[Any], Dict[str, List[Any]]]:
    """Parse ``TYPE(args)`` or ``(TYPE_A(args)TYPE_B(args)...)``.

    Returns ``(type, args, parts)`` as documented on :class:`Entity`.
    """
    p = _ParamParser(_tokenize(body, where), where)
    kind, tok = p.peek()
    if kind == "keyword":
        p.pos += 1
        args = p.param_list()
        if not p.at_end():
            raise p._fail("trailing tokens after instance")
        return tok, args, {tok: args}
    if tok != "(":
        raise p._fail("expected an entity name or '('")
    p.pos += 1
    parts: Dict[str, List[Any]] = {}
    order: List[str] = []
    flat: List[Any] = []
    while p.peek()[1] != ")":
        kind, name = p.peek()
        if kind != "keyword":
            raise p._fail(f"expected a partial entity name, got {name!r}")
        p.pos += 1
        args = p.param_list()
        parts[name] = args
        order.append(name)
        flat.extend(args)
    p.pos += 1
    if not p.at_end():
        raise p._fail("trailing tokens after complex instance")
    return "(" + " ".join(order) + ")", flat, parts


# --------------------------------------------------------------------------
# File
# --------------------------------------------------------------------------

class StepFile:
    """An indexed Part 21 file.  Build with :meth:`parse` (path) or :meth:`from_text`."""

    def __init__(self) -> None:
        self.path: Optional[Path] = None
        self.header: Dict[str, List[Any]] = {}
        self._text = ""
        self._spans: Dict[int, Tuple[int, int]] = {}
        self._type_of: Dict[int, str] = {}
        self._by_type: Dict[str, List[int]] = {}
        self._cache: Dict[int, Entity] = {}

    # -- construction -----------------------------------------------------
    @classmethod
    def parse(cls, path: Union[str, Path]) -> "StepFile":
        """Read and index ``path``.  Parameters are parsed lazily by :meth:`get`."""
        raw = Path(path).read_bytes()
        try:
            text = raw.decode("utf-8")
        except UnicodeDecodeError:  # Part 21 is nominally ISO 8859-1
            text = raw.decode("latin-1")
        sf = cls.from_text(text, where=str(path))
        sf.path = Path(path)
        return sf

    @classmethod
    def from_text(cls, text: str, where: str = "<text>") -> "StepFile":
        sf = cls()
        sf._text = text
        sf._index(where)
        return sf

    def _index(self, where: str) -> None:
        text = self._text
        section = None  # None | "HEADER" | "DATA"
        start = 0
        seen_magic = False
        for m in _RECORD_SPLIT_RE.finditer(text):
            if m.group() != ";":
                continue
            rec_start = _LEADING_JUNK_RE.match(text, start).end()
            rec_end = m.start()
            start = m.end()
            head = text[rec_start:rec_end].rstrip()
            if not seen_magic:
                if head != "ISO-10303-21":
                    raise StepSyntaxError(f"{where}: not a Part 21 file (starts with {head[:40]!r})")
                seen_magic = True
                continue
            if head == "HEADER":
                section = "HEADER"
                continue
            if _DATA_KEYWORD_RE.match(head):  # "DATA" or edition-3 "DATA('name',(...))"
                section = "DATA"
                continue
            if head == "ENDSEC":
                section = None
                continue
            if head == "END-ISO-10303-21":
                break
            if section == "HEADER":
                type_name, args, _ = _parse_body(head, f"{where} header")
                self.header[type_name] = args
            elif section == "DATA":
                hm = _INSTANCE_HEAD_RE.match(text, rec_start)
                if hm is None:
                    raise StepSyntaxError(f"{where}: DATA record does not start with '#n=': {head[:60]!r}")
                eid = int(hm.group(1))
                if eid in self._spans:
                    raise StepSyntaxError(f"{where}: duplicate instance #{eid}")
                body_start = hm.end()
                self._spans[eid] = (body_start, rec_end)
                km = _KEYWORD_RE.match(text, body_start)
                if km is not None:
                    names = [km.group()]
                else:  # complex instance: parse now, there are few of them
                    ent = self._parse_entity(eid)
                    names = list(ent.parts)
                self._type_of[eid] = names[0] if km is not None else "(" + " ".join(names) + ")"
                for name in names:
                    self._by_type.setdefault(name, []).append(eid)
            elif head:
                raise StepSyntaxError(f"{where}: record outside any section: {head[:60]!r}")
        if not seen_magic:
            raise StepSyntaxError(f"{where}: empty or truncated file")
        for ids in self._by_type.values():
            ids.sort()

    def _parse_entity(self, eid: int) -> Entity:
        start, end = self._spans[eid]
        type_name, args, parts = _parse_body(self._text[start:end], f"#{eid}")
        ent = Entity(eid, type_name, args, parts)
        self._cache[eid] = ent
        return ent

    # -- access -----------------------------------------------------------
    def get(self, ref: Union[int, Ref]) -> Entity:
        """The entity with id ``ref`` (an ``int`` or :class:`Ref`); ``KeyError`` if absent."""
        eid = ref.id if isinstance(ref, Ref) else int(ref)
        ent = self._cache.get(eid)
        if ent is not None:
            return ent
        if eid not in self._spans:
            raise KeyError(f"#{eid} is not defined in the file")
        return self._parse_entity(eid)

    __getitem__ = get

    def type_of(self, ref: Union[int, Ref]) -> str:
        """Entity type name without parsing the parameters."""
        eid = ref.id if isinstance(ref, Ref) else int(ref)
        return self._type_of[eid]

    def by_type(self, type_name: str) -> List[Entity]:
        """All instances of ``type_name`` (including complex instances having that part), by id."""
        return [self.get(i) for i in self._by_type.get(type_name, ())]

    def count(self, type_name: str) -> int:
        return len(self._by_type.get(type_name, ()))

    def type_names(self) -> List[str]:
        return sorted(self._by_type)

    def ids(self) -> Iterator[int]:
        return iter(sorted(self._spans))

    def resolve(self, value: Any) -> Any:
        """Replace a :class:`Ref` by its :class:`Entity` (lists are resolved element-wise)."""
        if isinstance(value, Ref):
            return self.get(value)
        if isinstance(value, list):
            return [self.resolve(v) for v in value]
        return value

    def __contains__(self, ref: object) -> bool:
        if isinstance(ref, Ref):
            return ref.id in self._spans
        return isinstance(ref, int) and ref in self._spans

    def __len__(self) -> int:
        return len(self._spans)

    def __iter__(self) -> Iterator[Entity]:
        for eid in self.ids():
            yield self.get(eid)
