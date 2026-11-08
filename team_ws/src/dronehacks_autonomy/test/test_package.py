"""Packaging, checked without colcon: what setup.py installs exists, and the launch matches the kit's."""

import ast
import re
import unittest

import _paths
import yaml

ROOT = _paths.PACKAGE_ROOT
KIT_LAUNCH = (_paths.REPO / "drone-snapshot" / "home" / "dronehacks" / "skybeta" / "src" / "competitor_example"
              / "launch" / "competitor.launch.py")


def flight_interface_arguments(path):
    """``{name: default}`` of the flight-interface arguments in a launch file's argument table."""
    tree = ast.parse(path.read_text(encoding="utf-8"))
    constants = {n.targets[0].id: n.value.value for n in tree.body
                 if isinstance(n, ast.Assign) and isinstance(n.value, ast.Constant)}

    def value(node):
        return constants[node.id] if isinstance(node, ast.Name) else node.value

    for node in tree.body:
        if isinstance(node, ast.Assign) and getattr(node.targets[0], "id", "") in ("ARGUMENTS", "FLIGHT_INTERFACE_ARGUMENTS"):
            return {value(e.elts[0]): value(e.elts[1]) for e in node.value.elts if isinstance(e, ast.Tuple)}
    raise AssertionError(f"no argument table in {path}")


class PackageTest(unittest.TestCase):
    def test_installed_files_exist(self):
        for name in ("package.xml", "README.md", "setup.cfg", "resource/dronehacks_autonomy",
                     "launch/competitor.launch.py", "config/autonomy.example.yaml", "maps/course_v3_layout.yaml"):
            self.assertTrue((ROOT / name).is_file(), name)

    def test_setup_lists_every_python_package(self):
        text = (ROOT / "setup.py").read_text(encoding="utf-8")
        self.assertIn('PACKAGE + ".planning"', text)
        for sub in ROOT.joinpath("dronehacks_autonomy").iterdir():
            if sub.is_dir() and sub.name != "__pycache__":
                self.assertEqual(sub.name, "planning", "a new subpackage must be added to setup.py")

    def test_entry_points_resolve(self):
        text = (ROOT / "setup.py").read_text(encoding="utf-8")
        for module, func in re.findall(r"= (dronehacks_autonomy\.\w+):(\w+)", text):
            path = ROOT / (module.replace(".", "/") + ".py")
            self.assertTrue(path.is_file(), module)
            names = {n.name for n in ast.parse(path.read_text(encoding="utf-8")).body if isinstance(n, ast.FunctionDef)}
            self.assertIn(func, names, module)

    def test_config_is_float_where_ros_needs_it(self):
        params = yaml.safe_load((ROOT / "config" / "autonomy.example.yaml").read_text(encoding="utf-8"))
        for key, value in params["mission"]["ros__parameters"].items():
            self.assertNotIsInstance(value if not isinstance(value, bool) else 0.0, int, f"{key} must be a float")

    def test_flight_interface_matches_the_kit(self):
        # competitor_guide.md: keep flight_interface exactly as the kit's launch file has it.
        if not KIT_LAUNCH.is_file():
            self.skipTest("the drone snapshot is not next to this checkout")
        ours = flight_interface_arguments(ROOT / "launch" / "competitor.launch.py")
        kit = flight_interface_arguments(KIT_LAUNCH)
        for name, default in ours.items():
            self.assertEqual(default, kit[name], name)
        for name in ("min_altitude_m", "max_altitude_m", "auto_offboard_on_arm", "allow_mode_request", "odometry_topic"):
            self.assertIn(name, ours)


if __name__ == "__main__":
    unittest.main()
