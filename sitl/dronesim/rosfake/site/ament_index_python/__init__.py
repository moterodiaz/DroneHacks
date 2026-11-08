"""ament_index_python backed by the simulated install space."""
from ament_index_python.packages import (  # noqa: F401
    PackageNotFoundError, get_package_prefix, get_package_share_directory, get_package_share_path,
    get_packages_with_prefixes)
from dronesim.rosfake import packages as _ws


def get_search_paths():
    return [pkg.prefix for pkg in _ws.active().packages.values()]


def get_resources(resource_type):
    if resource_type == 'packages':
        return {name: pkg.prefix for name, pkg in _ws.active().packages.items()}
    return {}


def get_resource(resource_type, resource_name):
    if resource_type == 'packages' and resource_name in _ws.active().packages:
        return '', _ws.active().packages[resource_name].prefix
    raise LookupError(f"Could not find the resource '{resource_name}' of type '{resource_type}'")


def has_resource(resource_type, resource_name):
    return resource_type == 'packages' and resource_name in _ws.active().packages


class InvalidResourceTypeNameError(ValueError):
    pass


class InvalidResourceNameError(ValueError):
    pass
