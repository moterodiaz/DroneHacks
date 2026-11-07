"""Generate ROS 2 Python message/service classes from ``.msg`` / ``.srv`` files.

The classes mimic what ``rosidl_generator_py`` emits for ROS 2 Humble closely
enough that code which works (or breaks) against real ``rclpy`` messages
behaves the same way here:

* every field is a property whose setter runs the same ``assert`` checks, with
  the same messages, as the generated code (``msg.x = 1`` on a ``float64``
  raises ``AssertionError: The 'x' field must be of type 'float'``; a
  ``numpy.float32`` is not a ``float`` either);
* fixed-size numeric arrays (``float32[3]``) are ``numpy.ndarray`` of the exact
  dtype, numeric sequences (``float64[]``) are ``array.array``, sequences of
  anything else are ``list``;
* constants live on a per-message metaclass, so ``VehicleStatus.NAVIGATION_STATE_OFFBOARD``
  works and the instance attribute does not, as in rosidl;
* services expose ``Srv.Request`` / ``Srv.Response``.

Definitions come from ``interfaces/<package>/{msg,srv}`` (vendored from the
upstream repositories listed in ``interfaces/SOURCES.txt``).  ``install()``
registers an import hook so ``from px4_msgs.msg import VehicleStatus`` and
``from std_srvs.srv import Trigger`` resolve to generated classes.
"""
from __future__ import annotations

import importlib.abc
import importlib.machinery
import os
import re
import sys
import threading
import types

INTERFACES_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'interfaces')

# name -> (kind, numpy dtype name, array typecode, lo, hi, idl name)
_PRIMITIVES = {
    'bool': ('bool', None, None, None, None, 'boolean'),
    'byte': ('byte', None, None, None, None, 'octet'),
    'char': ('char', None, None, None, None, 'char'),
    'float32': ('float', 'float32', 'f', -3.402823466e+38, 3.402823466e+38, 'float'),
    'float64': ('float', 'float64', 'd', -1.7976931348623157e+308, 1.7976931348623157e+308, 'double'),
    'int8': ('int', 'int8', 'b', -128, 127, 'int8'),
    'uint8': ('int', 'uint8', 'B', 0, 255, 'uint8'),
    'int16': ('int', 'int16', 'h', -32768, 32767, 'int16'),
    'uint16': ('int', 'uint16', 'H', 0, 65535, 'uint16'),
    'int32': ('int', 'int32', 'i', -2147483648, 2147483647, 'int32'),
    'uint32': ('int', 'uint32', 'I', 0, 4294967295, 'uint32'),
    'int64': ('int', 'int64', 'q', -9223372036854775808, 9223372036854775807, 'int64'),
    'uint64': ('int', 'uint64', 'Q', 0, 18446744073709551615, 'uint64'),
    'string': ('str', None, None, None, None, 'string'),
    'wstring': ('str', None, None, None, None, 'wstring'),
}

_TYPE_RE = re.compile(
    r'^(?P<base>[A-Za-z0-9_/]+?)(?:<=(?P<strbound>\d+))?'
    r'(?:\[(?P<arr>(?:<=)?\d*)\])?$')


class FieldType:
    """Parsed field type: base type plus array shape."""

    __slots__ = ('base', 'pkg', 'name', 'is_primitive', 'string_bound',
                 'array', 'array_size', 'bounded')

    def __init__(self, token, context_pkg):
        m = _TYPE_RE.match(token)
        if not m:
            raise ValueError(f'cannot parse field type {token!r}')
        base = m.group('base')
        self.string_bound = int(m.group('strbound')) if m.group('strbound') else None
        arr = m.group('arr')
        self.array = arr is not None
        self.bounded = False
        self.array_size = None
        if arr:
            if arr.startswith('<='):
                self.bounded = True
                self.array_size = int(arr[2:])
            else:
                self.array_size = int(arr)
        if base in _PRIMITIVES:
            self.is_primitive = True
            self.base = base
            self.pkg = None
            self.name = base
        else:
            self.is_primitive = False
            if '/' in base:
                parts = base.split('/')
                self.pkg, self.name = parts[0], parts[-1]
            elif base == 'Header':
                self.pkg, self.name = 'std_msgs', 'Header'
            else:
                self.pkg, self.name = context_pkg, base
            self.base = f'{self.pkg}/{self.name}'

    @property
    def fixed(self):
        return self.array and self.array_size is not None and not self.bounded

    @property
    def sequence(self):
        return self.array and not self.fixed

    def idl(self):
        if self.is_primitive:
            name = _PRIMITIVES[self.base][5]
            if self.string_bound is not None:
                name = f'{name}<{self.string_bound}>'
        else:
            name = self.base
        if not self.array:
            return name
        if self.fixed:
            return f'{name}[{self.array_size}]'
        if self.bounded:
            return f'sequence<{name}, {self.array_size}>'
        return f'sequence<{name}>'


class Field:
    __slots__ = ('name', 'type', 'default')

    def __init__(self, name, ftype, default):
        self.name, self.type, self.default = name, ftype, default


class Spec:
    """One parsed message (or one half of a service)."""

    def __init__(self, pkg, name, fields, constants):
        self.pkg, self.name = pkg, name
        self.fields = fields
        self.constants = constants


def _strip_comment(line):
    out, quote = [], None
    for ch in line:
        if quote:
            out.append(ch)
            if ch == quote:
                quote = None
            continue
        if ch in ('"', "'"):
            quote = ch
        elif ch == '#':
            break
        out.append(ch)
    return ''.join(out).strip()


def _parse_scalar(text, ftype):
    text = text.strip()
    kind = _PRIMITIVES[ftype.base][0] if ftype.is_primitive else None
    if kind == 'str' or kind == 'char':
        if len(text) >= 2 and text[0] == text[-1] and text[0] in ('"', "'"):
            return text[1:-1]
        return text
    if kind == 'bool':
        return text.lower() in ('true', '1')
    if kind == 'float':
        return float(text)
    if kind == 'int':
        return int(text, 0)
    if kind == 'byte':
        return bytes([int(text, 0)])
    raise ValueError(f'unsupported default for {ftype.base}: {text!r}')


def _parse_value(text, ftype):
    if ftype.array:
        inner = text.strip()
        if not (inner.startswith('[') and inner.endswith(']')):
            raise ValueError(f'array default must be [..]: {text!r}')
        inner = inner[1:-1].strip()
        if not inner:
            return []
        return [_parse_scalar(part, ftype) for part in inner.split(',')]
    return _parse_scalar(text, ftype)


def parse_spec(pkg, name, text):
    fields, constants = [], []
    for raw in text.splitlines():
        line = _strip_comment(raw)
        if not line:
            continue
        parts = line.split(None, 1)
        if len(parts) < 2:
            raise ValueError(f'{pkg}/{name}: cannot parse {raw!r}')
        type_token, rest = parts
        ftype = FieldType(type_token, pkg)
        if '=' in rest.split(None, 1)[0] or re.match(r'^[A-Z][A-Z0-9_]*\s*=', rest):
            cname, value = rest.split('=', 1)
            constants.append((cname.strip(), _parse_value(value, ftype)))
            continue
        bits = rest.split(None, 1)
        fname = bits[0]
        default = _parse_value(bits[1], ftype) if len(bits) > 1 else None
        fields.append(Field(fname, ftype, default))
    return Spec(pkg, name, fields, constants)


# --------------------------------------------------------------------------
# code generation
# --------------------------------------------------------------------------
def _float_text(v):
    return repr(float(v))


def _scalar_check(attr, ftype):
    """Assert lines for a scalar value named ``value``."""
    lines = []
    if not ftype.is_primitive:
        lines.append(
            f"assert isinstance(value, {ftype.name}__cls), "
            f"\"The '{attr}' field must be a sub message of type '{ftype.name}'\"")
        return lines
    kind, _dt, _tc, lo, hi, _idl = _PRIMITIVES[ftype.base]
    if kind == 'bool':
        lines.append(f"assert isinstance(value, bool), \"The '{attr}' field must be of type 'bool'\"")
    elif kind == 'float':
        word = 'float' if ftype.base == 'float32' else 'double'
        lines.append(f"assert isinstance(value, float), \"The '{attr}' field must be of type 'float'\"")
        lines.append(
            f"assert not (value < {_float_text(lo)} or value > {_float_text(hi)}) or math.isinf(value), "
            f"\"The '{attr}' field must be a {word} in [{lo}, {hi}]\"")
    elif kind == 'int':
        lines.append(f"assert isinstance(value, int), \"The '{attr}' field must be of type 'int'\"")
        word = 'an unsigned integer' if lo == 0 else 'an integer'
        lines.append(
            f"assert value >= {lo} and value < {hi + 1}, "
            f"\"The '{attr}' field must be {word} in [{lo}, {hi}]\"")
    elif kind == 'str':
        lines.append(f"assert isinstance(value, str), \"The '{attr}' field must be of type 'str'\"")
        if ftype.string_bound is not None:
            lines.append(
                f"assert len(value) <= {ftype.string_bound}, "
                f"\"The '{attr}' field must be string with length <= {ftype.string_bound}\"")
    elif kind == 'char':
        lines.append(
            f"assert isinstance(value, str) and len(value) == 1, "
            f"\"The '{attr}' field must be of type 'str' with length 1\"")
    elif kind == 'byte':
        lines.append(
            f"assert isinstance(value, bytes) and len(value) == 1, "
            f"\"The '{attr}' field must be of type 'bytes' with length 1\"")
    return lines


def _element_check(ftype):
    """(expression over v, description) for sequence element checks."""
    if not ftype.is_primitive:
        return f'isinstance(v, {ftype.name}__cls)', f"'{ftype.name}'"
    kind, _dt, _tc, lo, hi, _idl = _PRIMITIVES[ftype.base]
    if kind == 'float':
        return (f'isinstance(v, float) and (not (v < {_float_text(lo)} or v > {_float_text(hi)}) '
                f'or math.isinf(v))', f"'float' and each float in [{lo:f}, {hi:f}]")
    if kind == 'int':
        return (f'isinstance(v, int) and v >= {lo} and v < {hi + 1}',
                f"'int' and each {'unsigned ' if lo == 0 else ''}integer in [{lo}, {hi}]")
    if kind == 'bool':
        return 'isinstance(v, bool)', "'bool'"
    if kind == 'str':
        return 'isinstance(v, str)', "'str'"
    if kind == 'char':
        return 'isinstance(v, str) and len(v) == 1', "'str'"
    return 'isinstance(v, bytes)', "'bytes'"


def _is_numeric(ftype):
    return ftype.is_primitive and _PRIMITIVES[ftype.base][0] in ('float', 'int')


def _default_expr(field):
    ft = field.type
    if not ft.array:
        if not ft.is_primitive:
            return f'{ft.name}__cls()'
        if field.default is not None:
            return repr(field.default)
        kind = _PRIMITIVES[ft.base][0]
        return {'bool': 'False', 'float': '0.0', 'int': '0', 'str': "''",
                'char': "'\\x00'", 'byte': "b'\\x00'"}[kind]
    if ft.fixed:
        if _is_numeric(ft):
            dt = _PRIMITIVES[ft.base][1]
            if field.default is not None:
                return f'numpy.array({field.default!r}, dtype=numpy.{dt})'
            return f'numpy.zeros({ft.array_size}, dtype=numpy.{dt})'
        if field.default is not None:
            return repr(list(field.default))
        if not ft.is_primitive:
            return f'[{ft.name}__cls() for _ in range({ft.array_size})]'
        kind = _PRIMITIVES[ft.base][0]
        zero = {'bool': 'False', 'str': "''", 'char': "'\\x00'", 'byte': "b'\\x00'"}[kind]
        return f'[{zero} for _ in range({ft.array_size})]'
    # sequences
    if _is_numeric(ft):
        tc = _PRIMITIVES[ft.base][2]
        init = repr(list(field.default)) if field.default is not None else '[]'
        return f"array.array('{tc}', {init})"
    return repr(list(field.default)) if field.default is not None else '[]'


def _gen_source(spec, qualified_module):
    name = spec.name
    attrs = [_py_attr(f.name) for f in spec.fields]
    out = []
    w = out.append
    w(f'class Metaclass_{name}(type):')
    w(f'    _CONSTANTS = {{{", ".join(f"{c!r}: {v!r}" for c, v in spec.constants)}}}')
    for cname, _v in spec.constants:
        w('    @property')
        w(f'    def {cname}(cls):')
        w(f'        return Metaclass_{name}._CONSTANTS[{cname!r}]')
    w('    pass')
    w('')
    w(f'class {name}(_MessageBase, metaclass=Metaclass_{name}):')
    w(f'    __slots__ = {tuple("_" + a for a in attrs)!r}')
    # rosidl's Metaclass.__prepare__ also puts the constants in the class
    # namespace, so instances can read them too.
    for cname, value in spec.constants:
        w(f'    {cname} = {value!r}')
    w(f'    _fields_and_field_types = {{{", ".join(f"{a!r}: {f.type.idl()!r}" for a, f in zip(attrs, spec.fields))}}}')
    w(f'    _field_names = {tuple(attrs)!r}')
    w(f'    _full_type = {spec.pkg + "/" + qualified_module.split(".")[1] + "/" + name!r}')
    w('')
    w('    def __init__(self, **kwargs):')
    w("        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \\")
    w("            'Invalid arguments passed to constructor: %s' % ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))")
    for attr, field in zip(attrs, spec.fields):
        ft = field.type
        if ft.fixed and _is_numeric(ft):
            dt = _PRIMITIVES[ft.base][1]
            w(f'        if {attr!r} not in kwargs:')
            w(f'            self.{attr} = {_default_expr(field)}')
            w('        else:')
            w(f'            self.{attr} = numpy.array(kwargs.get({attr!r}), dtype=numpy.{dt})')
            w(f'            assert self.{attr}.shape == ({ft.array_size}, )')
        elif ft.sequence and _is_numeric(ft):
            tc = _PRIMITIVES[ft.base][2]
            init = repr(list(field.default)) if field.default is not None else '[]'
            w(f"        self.{attr} = array.array('{tc}', kwargs.get({attr!r}, {init}))")
        else:
            w(f'        self.{attr} = kwargs.get({attr!r}, {_default_expr(field)})')
    if not spec.fields:
        w('        pass')
    w('')
    # fast structural copy (DDS delivers an independent object to each reader)
    w('    def _sim_copy(self):')
    w(f'        new = {name}.__new__({name})')
    for attr, field in zip(attrs, spec.fields):
        ft = field.type
        if not ft.array:
            if ft.is_primitive:
                w(f'        new._{attr} = self._{attr}')
            else:
                w(f'        new._{attr} = self._{attr}._sim_copy()')
        elif ft.fixed and _is_numeric(ft):
            w(f'        new._{attr} = self._{attr}.copy()')
        elif ft.sequence and _is_numeric(ft):
            w(f'        new._{attr} = array.array(self._{attr}.typecode, self._{attr})')
        elif ft.is_primitive:
            w(f'        new._{attr} = list(self._{attr})')
        else:
            w(f'        new._{attr} = [v._sim_copy() for v in self._{attr}]')
    w('        return new')
    w('')
    for attr, field in zip(attrs, spec.fields):
        ft = field.type
        w('    @property')
        w(f'    def {attr}(self):')
        w(f'        """Message field \'{field.name}\'."""')
        w(f'        return self._{attr}')
        w('')
        w(f'    @{attr}.setter')
        w(f'    def {attr}(self, value):')
        if not ft.array:
            w('        if __debug__:')
            for line in _scalar_check(attr, ft):
                w('            ' + line)
            w(f'        self._{attr} = value')
        elif ft.fixed and _is_numeric(ft):
            dt = _PRIMITIVES[ft.base][1]
            cond, desc = _element_check(ft)
            w('        if isinstance(value, numpy.ndarray):')
            w(f"            assert value.dtype == numpy.{dt}, \"The '{attr}' numpy.ndarray() must have the dtype of 'numpy.{dt}'\"")
            w(f"            assert value.size == {ft.array_size}, \"The '{attr}' numpy.ndarray() must have a size of {ft.array_size}\"")
            w(f'            self._{attr} = value')
            w('            return')
            w('        if __debug__:')
            w('            assert ((isinstance(value, Sequence) or isinstance(value, Set) or isinstance(value, UserList)) and')
            w('                    not isinstance(value, str) and not isinstance(value, UserString) and')
            w(f'                    len(value) == {ft.array_size} and all({cond} for v in value)), \\')
            w(f"                \"The '{attr}' field must be a set or sequence with length {ft.array_size} and each value of type {desc}\"")
            w(f'        self._{attr} = numpy.array(value, dtype=numpy.{dt})')
        elif ft.sequence and _is_numeric(ft):
            tc = _PRIMITIVES[ft.base][2]
            cond, desc = _element_check(ft)
            w('        if isinstance(value, array.array):')
            w(f"            assert value.typecode == '{tc}', \"The '{attr}' array.array() must have the type code of '{tc}'\"")
            if ft.bounded:
                w(f"            assert len(value) <= {ft.array_size}, \"The '{attr}' array.array() must have a size <= {ft.array_size}\"")
            w(f'            self._{attr} = value')
            w('            return')
            w('        if __debug__:')
            w('            assert ((isinstance(value, Sequence) or isinstance(value, Set) or isinstance(value, UserList)) and')
            w('                    not isinstance(value, str) and not isinstance(value, UserString) and')
            bound = f'len(value) <= {ft.array_size} and ' if ft.bounded else ''
            w(f'                    {bound}all({cond} for v in value)), \\')
            w(f"                \"The '{attr}' field must be a set or sequence and each value of type {desc}\"")
            w(f"        self._{attr} = array.array('{tc}', value)")
        else:
            cond, desc = _element_check(ft)
            size = (f'len(value) == {ft.array_size} and ' if ft.fixed else
                    f'len(value) <= {ft.array_size} and ' if ft.bounded else '')
            w('        if __debug__:')
            w('            assert ((isinstance(value, Sequence) or isinstance(value, Set) or isinstance(value, UserList)) and')
            w('                    not isinstance(value, str) and not isinstance(value, UserString) and')
            w(f'                    {size}all({cond} for v in value)), \\')
            w(f"                \"The '{attr}' field must be a set or sequence and each value of type {desc}\"")
            w(f'        self._{attr} = value')
        w('')
    return '\n'.join(out)


_KEYWORDS = {'False', 'None', 'True', 'and', 'as', 'assert', 'async', 'await', 'break', 'class',
             'continue', 'def', 'del', 'elif', 'else', 'except', 'finally', 'for', 'from',
             'global', 'if', 'import', 'in', 'is', 'lambda', 'nonlocal', 'not', 'or', 'pass',
             'raise', 'return', 'try', 'while', 'with', 'yield'}


def _py_attr(name):
    return name + '_' if name in _KEYWORDS else name


class _MessageBase:
    """Common behaviour of generated messages (repr, eq, introspection)."""

    __slots__ = ()

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        prefix = '.'.join(typename[:2]) if len(typename) >= 2 else self.__class__.__module__
        args = []
        for attr in self._field_names:
            value = getattr(self, attr)
            if hasattr(value, 'tolist') and not isinstance(value, (int, float)):
                text = f'numpy.array({value.tolist()!r}, dtype={value.dtype})'
            else:
                text = repr(value)
            args.append(f'{attr}={text}')
        return f"{prefix}.{self.__class__.__name__}({', '.join(args)})"

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        import numpy
        for attr in self._field_names:
            a, b = getattr(self, attr), getattr(other, attr)
            if isinstance(a, numpy.ndarray) or isinstance(b, numpy.ndarray):
                if not numpy.array_equal(numpy.asarray(a), numpy.asarray(b)):
                    return False
            elif a != b:
                return False
        return True

    __hash__ = None


# --------------------------------------------------------------------------
# registry + import hook
# --------------------------------------------------------------------------
class Registry:
    """Lazily generates and caches interface classes from INTERFACES_DIR."""

    def __init__(self, root=INTERFACES_DIR):
        self.root = root
        self._lock = threading.RLock()
        self._classes = {}
        self.packages = sorted(
            d for d in os.listdir(root)
            if os.path.isdir(os.path.join(root, d)) and not d.startswith('.'))

    def available(self, pkg, kind):
        folder = os.path.join(self.root, pkg, kind)
        if not os.path.isdir(folder):
            return []
        suffix = '.' + kind
        return sorted(f[:-len(suffix)] for f in os.listdir(folder) if f.endswith(suffix))

    def get_message(self, pkg, name):
        key = ('msg', pkg, name)
        with self._lock:
            if key in self._classes:
                return self._classes[key]
            path = os.path.join(self.root, pkg, 'msg', name + '.msg')
            if not os.path.isfile(path):
                raise ImportError(f'no message {pkg}/msg/{name}')
            with open(path, 'r', encoding='utf-8') as fh:
                spec = parse_spec(pkg, name, fh.read())
            cls = self._build(spec, f'{pkg}.msg')
            self._classes[key] = cls
            return cls

    def get_service(self, pkg, name):
        key = ('srv', pkg, name)
        with self._lock:
            if key in self._classes:
                return self._classes[key]
            path = os.path.join(self.root, pkg, 'srv', name + '.srv')
            if not os.path.isfile(path):
                raise ImportError(f'no service {pkg}/srv/{name}')
            with open(path, 'r', encoding='utf-8') as fh:
                text = fh.read()
            lines = text.splitlines()
            try:
                split = next(i for i, ln in enumerate(lines) if ln.strip() == '---')
            except StopIteration as exc:
                raise ImportError(f'{pkg}/srv/{name}: no --- separator') from exc
            req_text = '\n'.join(lines[:split])
            resp_text = '\n'.join(lines[split + 1:])
            req = self._build(parse_spec(pkg, name + '_Request', req_text), f'{pkg}.srv')
            resp = self._build(parse_spec(pkg, name + '_Response', resp_text), f'{pkg}.srv')
            srv = type(name, (), {
                'Request': req, 'Response': resp, '__module__': f'{pkg}.srv._{_snake(name)}',
                '_full_type': f'{pkg}/srv/{name}', '__slots__': (),
            })
            self._classes[key] = srv
            self._classes[('srv', pkg, name + '_Request')] = req
            self._classes[('srv', pkg, name + '_Response')] = resp
            return srv

    def _build(self, spec, module_name):
        import array
        import math
        from collections import UserList, UserString
        from collections.abc import Sequence, Set

        import numpy
        namespace = {
            'array': array, 'math': math, 'numpy': numpy, 'Sequence': Sequence, 'Set': Set,
            'UserList': UserList, 'UserString': UserString, '_MessageBase': _MessageBase,
        }
        for field in spec.fields:
            if not field.type.is_primitive:
                namespace[f'{field.type.name}__cls'] = self.get_message(field.type.pkg, field.type.name)
        source = _gen_source(spec, module_name)
        code = compile(source, f'<rosidl {spec.pkg}/{spec.name}>', 'exec')
        exec(code, namespace)  # noqa: S102 - generated from vendored .msg definitions
        cls = namespace[spec.name]
        cls.__module__ = f'{module_name}._{_snake(spec.name)}'
        return cls


def _snake(name):
    return re.sub(r'(?<!^)(?=[A-Z])', '_', name).lower()


REGISTRY = None


def registry():
    global REGISTRY
    if REGISTRY is None:
        REGISTRY = Registry()
    return REGISTRY


class _InterfaceModule(types.ModuleType):
    def __init__(self, name, pkg, kind, reg):
        super().__init__(name)
        self._pkg, self._kind, self._reg = pkg, kind, reg
        self.__all__ = reg.available(pkg, kind)
        self.__path__ = []  # allow `from pkg.msg import X` resolution as a package

    def __getattr__(self, attr):
        if attr.startswith('__'):
            raise AttributeError(attr)
        reg = object.__getattribute__(self, '_reg')
        pkg = object.__getattribute__(self, '_pkg')
        kind = object.__getattribute__(self, '_kind')
        try:
            if kind == 'msg':
                value = reg.get_message(pkg, attr)
            else:
                base = attr[:-8] if attr.endswith('_Request') else attr[:-9] if attr.endswith('_Response') else attr
                srv = reg.get_service(pkg, base)
                value = srv if base == attr else getattr(srv, 'Request' if attr.endswith('_Request') else 'Response')
        except ImportError as exc:
            raise AttributeError(f"module '{pkg}.{kind}' has no attribute '{attr}'") from exc
        setattr(self, attr, value)
        return value

    def __dir__(self):
        return sorted(set(super().__dir__()) | set(self.__all__))


class _Finder(importlib.abc.MetaPathFinder, importlib.abc.Loader):
    def __init__(self, reg):
        self.reg = reg
        self.packages = set(reg.packages)

    def find_spec(self, fullname, path=None, target=None):
        parts = fullname.split('.')
        if parts[0] not in self.packages:
            return None
        if len(parts) == 1 or (len(parts) == 2 and parts[1] in ('msg', 'srv')):
            return importlib.machinery.ModuleSpec(fullname, self, is_package=True)
        return None

    def create_module(self, spec):
        parts = spec.name.split('.')
        if len(parts) == 1:
            module = types.ModuleType(spec.name)
            module.__path__ = []
            module.__file__ = os.path.join(self.reg.root, parts[0], '__init__.py')
            return module
        return _InterfaceModule(spec.name, parts[0], parts[1], self.reg)

    def exec_module(self, module):
        parts = module.__name__.split('.')
        if len(parts) == 2:
            setattr(sys.modules[parts[0]], parts[1], module)


_FINDER = None


def install():
    """Register the import hook (idempotent)."""
    global _FINDER
    if _FINDER is None:
        _FINDER = _Finder(registry())
        sys.meta_path.insert(0, _FINDER)
    return _FINDER


def copy_message(msg):
    """Independent copy of a generated message (what a DDS reader receives)."""
    copier = getattr(msg, '_sim_copy', None)
    if copier is not None:
        return copier()
    import copy
    return copy.deepcopy(msg)


def full_type_name(msg_type):
    return getattr(msg_type, '_full_type', f'{msg_type.__module__}.{msg_type.__name__}')
