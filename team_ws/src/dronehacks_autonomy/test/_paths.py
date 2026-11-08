"""Make the package importable from a source checkout, and name the test data."""

import importlib.util
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
PACKAGE_ROOT = HERE.parent
REPO = PACKAGE_ROOT.parent.parent.parent

if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

PUBLISHED_MAP = HERE / "fixtures" / "published_v3_fieldguide.yaml"
CONTRACT_MAP = PACKAGE_ROOT / "maps" / "course_v3_layout.yaml"
HAVE_RCLPY = importlib.util.find_spec("rclpy") is not None
