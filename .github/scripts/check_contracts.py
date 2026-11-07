"""Require a contracts/CHANGELOG.md entry whenever contracts/ changes.

Simplified from redledger's version: DroneHacks has no team extension
namespaces yet, so every contracts/ path counts (no carve-outs).
"""

import sys


def main() -> int:
    paths = set(sys.stdin.buffer.read().split(b"\0"))
    contracts_changed = any(path.startswith(b"contracts/") for path in paths if path)
    if contracts_changed and b"contracts/CHANGELOG.md" not in paths:
        print("::error::contracts changed without contracts/CHANGELOG.md entry")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
