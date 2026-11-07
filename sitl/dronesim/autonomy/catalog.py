"""Autonomy versions: where their code comes from and how the drone would run it.

A version is loaded from ``sitl/versions.yaml`` (see that file's header).  Sources:

* ``git: <ref>`` -- archived at the ref's commit with ``git archive`` into
  ``<cache>/src/<sha>`` (read-only use of the repository: nothing is checked out,
  no branch is touched), so the same commit always flies the same code;
* ``dir: <path>`` -- a directory, e.g. a worktree with uncommitted changes;
* ``kit: true`` -- only the organizer packages from ``drone-snapshot/``.
"""
from __future__ import annotations

import copy
import io
import os
import subprocess
import tarfile

import yaml

KIT_SRC = ('drone-snapshot', 'home', 'dronehacks', 'skybeta', 'src')
KIT_PACKAGES = ('skybeta_flight_interface', 'oakd_vslam_bringup', 'competitor_example')


class VersionError(RuntimeError):
    pass


def load_catalog(path):
    with open(path, 'r', encoding='utf-8') as fh:
        data = yaml.safe_load(fh) or {}
    versions = data.get('versions') or {}
    for vid, v in versions.items():
        v.setdefault('label', vid)
        v.setdefault('packages', [])
        v.setdefault('options', {})
        v.setdefault('followers', {'default': 'default', 'choices': {'default': {'label': 'shipped'}}})
        v['id'] = vid
    return versions


def _git(repo, *args, binary=False):
    out = subprocess.run(['git', '-C', repo, *args], capture_output=True, check=False)
    if out.returncode != 0:
        raise VersionError(f"git {' '.join(args)} failed: {out.stderr.decode(errors='replace').strip()}")
    return out.stdout if binary else out.stdout.decode().strip()


def resolve_ref(repo, ref):
    sha = _git(repo, 'rev-parse', '--verify', f'{ref}^{{commit}}')
    subject = _git(repo, 'log', '-1', '--format=%h %an %ad %s', '--date=short', sha)
    return sha, subject


def materialize(repo_root, version, cache_root):
    """Return (list of package source dirs, provenance text)."""
    src = version.get('source') or {}
    pkgs = list(version.get('packages') or [])
    if src.get('kit'):
        return [], 'organizer kit only (drone-snapshot)'
    if 'git' in src:
        sha, subject = resolve_ref(repo_root, src['git'])
        dest = os.path.join(cache_root, 'src', sha[:12])
        marker = os.path.join(dest, '.complete')
        if not os.path.isfile(marker):
            os.makedirs(dest, exist_ok=True)
            data = _git(repo_root, 'archive', '--format=tar', sha, '--', *pkgs, binary=True)
            with tarfile.open(fileobj=io.BytesIO(data)) as tf:
                try:
                    tf.extractall(dest, filter='data')
                except TypeError:          # Python < 3.12
                    tf.extractall(dest)
            with open(marker, 'w', encoding='utf-8') as fh:
                fh.write(sha + '\n')
        return [os.path.join(dest, p) for p in pkgs], f"{src['git']} @ {subject}"
    if 'dir' in src:
        base = src['dir']
        if not os.path.isabs(base):
            base = os.path.normpath(os.path.join(os.path.dirname(repo_root), base))
        if not os.path.isdir(base):
            raise VersionError(f'source directory {base} does not exist')
        return [os.path.join(base, p) for p in pkgs], f'directory {base} (working tree)'
    raise VersionError(f"version {version.get('id')}: no source")


def kit_package_dirs(repo_root):
    base = os.path.join(repo_root, *KIT_SRC)
    return [os.path.join(base, p) for p in KIT_PACKAGES]


def resolve_selection(version, option_values=None, follower=None):
    """Merge the chosen options and follower into (launch file, params by node, hook)."""
    launch = dict(version['launch'])
    params = {}
    hook = None
    notes = []

    def merge(p):
        for node, vals in (p or {}).items():
            params.setdefault(node, {}).update(vals)

    for name, opt in version.get('options', {}).items():
        choice = (option_values or {}).get(name, opt.get('default'))
        ch = opt['choices'].get(str(choice))
        if ch is None:
            raise VersionError(f'{version["id"]}: option {name} has no choice {choice!r}')
        if 'launch' in ch:
            launch['file'] = ch['launch']
        merge(ch.get('params'))
        notes.append(f"{opt.get('label', name)}: {ch.get('label', choice)}")
    fol = version['followers']
    fname = follower or fol.get('default')
    fch = fol['choices'].get(fname)
    if fch is None:
        raise VersionError(f'{version["id"]}: no follower {fname!r}')
    merge(fch.get('params'))
    if fch.get('hook'):
        hook = (fch['hook'], dict(fch.get('args') or {}))
    notes.append(f"Follower: {fch.get('label', fname)}" + (' [SIM INJECTION]' if hook else ''))
    return launch, params, hook, notes


def substitute(value, subs):
    if isinstance(value, str) and value.startswith('$'):
        key = value[1:]
        if key not in subs:
            raise VersionError(f'unknown substitution {value}')
        return subs[key]
    if isinstance(value, dict):
        return {k: substitute(v, subs) for k, v in value.items()}
    if isinstance(value, list):
        return [substitute(v, subs) for v in value]
    return value


def apply_install_config(version, share_dirs, subs, extra_params):
    """Edit installed config files the way an operator / install script would.

    Returns a list of (file, node, {param: value}) written, for the run report.
    """
    written = []
    for entry in version.get('install_config') or []:
        pkg = version['launch']['package']
        share = share_dirs[pkg]
        path = os.path.join(share, entry['file'])
        if not os.path.isfile(path):
            raise VersionError(f'{pkg}: installed config {entry["file"]} not found')
        with open(path, 'r', encoding='utf-8') as fh:
            data = yaml.safe_load(fh) or {}
        by_node = copy.deepcopy(entry.get('values_by_node') or {})
        if 'node' in entry:
            by_node.setdefault(entry['node'], {}).update(entry.get('values') or {})
        for node, vals in (extra_params or {}).items():
            by_node.setdefault(node, {}).update(vals)
        for node, vals in by_node.items():
            block = data.setdefault(node, {}).setdefault('ros__parameters', {})
            resolved = substitute(vals, subs)
            for k, v in resolved.items():
                if isinstance(v, int) and not isinstance(v, bool) and isinstance(block.get(k), float):
                    v = float(v)
                block[k] = v
            written.append((entry['file'], node, resolved))
        with open(path, 'w', encoding='utf-8') as fh:
            yaml.safe_dump(data, fh, default_flow_style=False, sort_keys=False)
    return written
