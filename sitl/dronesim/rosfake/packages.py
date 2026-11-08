"""The simulated install space: what ``colcon build`` + ``source install/setup.bash`` give.

A :class:`Workspace` builds ROS 2 ``ament_python`` packages the way colcon does
for the Jetson's team overlay and the kit underlay:

* runs the package's own ``setup.py`` with a capturing ``setuptools.setup`` to
  learn its Python packages, ``data_files`` and ``console_scripts``;
* copies the Python packages to ``<prefix>/lib/python3/site-packages`` (put on
  ``sys.path``) and each ``data_files`` entry to ``<prefix>/<dest>``;
* registers the package in an ament resource index, so
  ``get_package_share_directory`` and ``ros2 launch <pkg> <file>`` resolve.

Interface packages (``px4_msgs``, ``std_msgs``...) are registered with their
vendored definition folder as the share directory, so the kit's
``topics.py`` reads ``MESSAGE_VERSION`` from real ``.msg`` files.
"""
from __future__ import annotations

import ast
import glob
import os
import shutil
import sys
import types

from . import msggen


class PackageNotFoundError(KeyError):
    pass


class InstalledPackage:
    def __init__(self, name, prefix, share, executables=None, python_path=None, source=None,
                 version='0.0.0', origin=''):
        self.name = name
        self.prefix = prefix
        self.share = share
        self.executables = dict(executables or {})   # exe -> (module, function) or callable
        self.python_path = python_path
        self.source = source
        self.version = version
        self.origin = origin                         # human-readable provenance

    def __repr__(self):
        return f'<InstalledPackage {self.name} @ {self.origin or self.prefix}>'


class Workspace:
    """Registry of installed packages (underlay + overlay) for one simulation."""

    def __init__(self, root):
        self.root = os.path.abspath(root)
        os.makedirs(self.root, exist_ok=True)
        self.packages = {}
        self._sys_path_added = []

    # -- registration ------------------------------------------------------------
    def register(self, pkg):
        self.packages[pkg.name] = pkg
        if pkg.python_path and pkg.python_path not in sys.path:
            sys.path.insert(0, pkg.python_path)
            self._sys_path_added.append(pkg.python_path)
        return pkg

    def register_interfaces(self):
        reg = msggen.registry()
        for name in reg.packages:
            share = os.path.join(reg.root, name)
            self.packages.setdefault(name, InstalledPackage(
                name, share, share, origin='vendored interface definitions'))

    def register_native(self, name, executables, origin='dronesim native'):
        share = os.path.join(self.root, 'native', name, 'share', name)
        os.makedirs(share, exist_ok=True)
        return self.register(InstalledPackage(name, os.path.dirname(os.path.dirname(share)), share,
                                              executables=executables, origin=origin))

    def get(self, name):
        if name not in self.packages:
            raise PackageNotFoundError(
                f"package '{name}' not found, searching: {[p.prefix for p in self.packages.values()][:6]}")
        return self.packages[name]

    def close(self):
        for path in self._sys_path_added:
            if path in sys.path:
                sys.path.remove(path)
        self._sys_path_added = []

    # -- building --------------------------------------------------------------------
    def build_python_package(self, src_dir, origin=''):
        """colcon build (copy mode) of one ament_python package in ``src_dir``."""
        src_dir = os.path.abspath(src_dir)
        manifest = os.path.join(src_dir, 'package.xml')
        if not os.path.isfile(manifest):
            raise FileNotFoundError(f'{src_dir} has no package.xml')
        name, version, build_type = read_package_xml(manifest)
        if build_type not in ('ament_python', None):
            raise RuntimeError(f'{name}: build type {build_type} is not supported (Python packages only)')
        info = capture_setup(src_dir)
        prefix = os.path.join(self.root, 'install', name)
        if os.path.isdir(prefix):
            shutil.rmtree(prefix)
        site = os.path.join(prefix, 'lib', 'python3', 'site-packages')
        os.makedirs(site, exist_ok=True)
        package_dir = info.get('package_dir') or {}
        root_dir = os.path.join(src_dir, package_dir.get('', '')) if package_dir.get('') else src_dir
        for pkg in sorted(info.get('packages') or []):
            rel = pkg.replace('.', os.sep)
            source = os.path.join(root_dir, package_dir.get(pkg, rel)) if pkg in package_dir else os.path.join(root_dir, rel)
            dest = os.path.join(site, rel)
            if not os.path.isdir(source):
                continue
            os.makedirs(dest, exist_ok=True)
            for entry in os.listdir(source):
                full = os.path.join(source, entry)
                if os.path.isfile(full) and entry.endswith('.py'):
                    shutil.copy2(full, os.path.join(dest, entry))
                elif os.path.isfile(full) and pkg in (info.get('package_data_all') or ()):
                    shutil.copy2(full, os.path.join(dest, entry))
            # package_data
            for pattern in (info.get('package_data') or {}).get(pkg, []):
                for match in glob.glob(os.path.join(source, pattern)):
                    if os.path.isfile(match):
                        rel_match = os.path.relpath(match, source)
                        os.makedirs(os.path.dirname(os.path.join(dest, rel_match)), exist_ok=True)
                        shutil.copy2(match, os.path.join(dest, rel_match))
        for module in info.get('py_modules') or []:
            source = os.path.join(root_dir, module.replace('.', os.sep) + '.py')
            if os.path.isfile(source):
                shutil.copy2(source, os.path.join(site, os.path.basename(source)))
        for dest, files in info.get('data_files') or []:
            target_dir = os.path.join(prefix, dest)
            os.makedirs(target_dir, exist_ok=True)
            for f in files:
                source = os.path.join(src_dir, f)
                if os.path.isdir(source):
                    shutil.copytree(source, os.path.join(target_dir, os.path.basename(f)), dirs_exist_ok=True)
                elif os.path.isfile(source):
                    shutil.copy2(source, os.path.join(target_dir, os.path.basename(f)))
                else:
                    raise FileNotFoundError(f'{name}: data_files entry {f!r} does not exist (colcon would fail)')
        share = os.path.join(prefix, 'share', name)
        os.makedirs(share, exist_ok=True)
        if not os.path.isfile(os.path.join(share, 'package.xml')):
            shutil.copy2(manifest, os.path.join(share, 'package.xml'))
        executables = {}
        for spec in (info.get('entry_points') or {}).get('console_scripts', []):
            exe, _, target = spec.partition('=')
            module, _, func = target.strip().partition(':')
            executables[exe.strip()] = (module.strip(), func.strip() or 'main')
        pkg = InstalledPackage(name, prefix, share, executables=executables, python_path=site,
                               source=src_dir, version=version, origin=origin or src_dir)
        return self.register(pkg)


def read_package_xml(path):
    import xml.etree.ElementTree as ET
    root = ET.parse(path).getroot()
    name = (root.findtext('name') or '').strip()
    version = (root.findtext('version') or '0.0.0').strip()
    build_type = None
    export = root.find('export')
    if export is not None:
        build_type = (export.findtext('build_type') or '').strip() or None
    return name, version, build_type


def _find_packages(where='.', exclude=(), include=('*',)):
    import fnmatch
    out = []
    base = os.path.abspath(where)
    for dirpath, dirnames, filenames in os.walk(base):
        dirnames[:] = [d for d in dirnames if not d.startswith('.') and d != '__pycache__']
        if '__init__.py' in filenames and dirpath != base:
            name = os.path.relpath(dirpath, base).replace(os.sep, '.')
            if any(fnmatch.fnmatch(name, pat) for pat in include) and \
                    not any(fnmatch.fnmatch(name, pat) for pat in exclude):
                out.append(name)
    return sorted(out)


def capture_setup(src_dir):
    """Run ``setup.py`` with a recording ``setup()``; return its keyword arguments."""
    path = os.path.join(src_dir, 'setup.py')
    if not os.path.isfile(path):
        raise FileNotFoundError(f'{src_dir} has no setup.py (only ament_python packages are supported)')
    captured = {}

    def setup(**kwargs):
        captured.update(kwargs)

    fake = types.ModuleType('setuptools')
    fake.setup = setup
    fake.find_packages = lambda where='.', exclude=(), include=('*',): _find_packages(
        os.path.join(src_dir, where) if not os.path.isabs(where) else where, exclude, include)
    fake.find_namespace_packages = fake.find_packages
    saved = {k: sys.modules.get(k) for k in ('setuptools',)}
    old_cwd = os.getcwd()
    old_argv = sys.argv
    sys.modules['setuptools'] = fake
    try:
        os.chdir(src_dir)
        sys.argv = ['setup.py', 'build']
        with open(path, 'r', encoding='utf-8') as fh:
            code = compile(fh.read(), path, 'exec')
        exec(code, {'__name__': '__main__', '__file__': path})  # noqa: S102 - the package's own setup.py
    finally:
        os.chdir(old_cwd)
        sys.argv = old_argv
        for k, v in saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v
    if not captured:
        raise RuntimeError(f'{path} did not call setup()')
    return captured


ACTIVE = None   # the Workspace of the running simulation (ament_index_python reads it)


def set_active(workspace):
    global ACTIVE
    ACTIVE = workspace


def active():
    if ACTIVE is None:
        raise PackageNotFoundError('no simulated workspace is active')
    return ACTIVE


_ast = ast
