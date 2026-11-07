"""Make ``dronehacks_planner`` importable from a source checkout (colcon installs it anyway)."""

import importlib.util
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
PACKAGE_ROOT = HERE.parent
REPO = PACKAGE_ROOT.parent

for p in (PACKAGE_ROOT, REPO / "planner"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

PUBLISHED_MAP = REPO / "planner" / "test" / "fixtures" / "published_v3_fieldguide.yaml"
HAVE_RCLPY = importlib.util.find_spec("rclpy") is not None
