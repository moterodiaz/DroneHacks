"""ament_index_python.packages backed by the simulated install space."""
import os

from dronesim.rosfake import packages as _ws


class PackageNotFoundError(KeyError):
    pass


def _get(package_name):
    try:
        return _ws.active().get(package_name)
    except _ws.PackageNotFoundError as exc:
        raise PackageNotFoundError(f"package '{package_name}' not found") from exc


def get_package_prefix(package_name):
    return _get(package_name).prefix


def get_package_share_directory(package_name, print_warning=True):
    pkg = _get(package_name)
    if print_warning and not os.path.exists(pkg.share):
        import warnings
        warnings.warn(f"Share directory for {package_name} ({pkg.share}) does not exist.", stacklevel=2)
    return pkg.share


def get_package_share_path(package_name, print_warning=True):
    from pathlib import Path
    return Path(get_package_share_directory(package_name, print_warning))


def get_packages_with_prefixes():
    return {name: pkg.prefix for name, pkg in _ws.active().packages.items()}
