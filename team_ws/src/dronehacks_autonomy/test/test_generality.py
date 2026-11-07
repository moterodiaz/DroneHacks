"""The evidence for the generality score (rule 9.5): new course, new map file, no code change.

Every course goes through the same entry point the ROS node uses (``factory.prepare``) and is
flown whole in the stand-in vehicle, from arming to disarm. These are model flights, not real
ones: they show the code holds no layout knowledge, not that the drone clears these courses.
"""

import ast
import dataclasses
import re
import tempfile
import unittest
from pathlib import Path

import _paths
import yaml
from synthetic_courses import jitter_walls, mirror_east_west, random_course, rigid

from dronehacks_autonomy.contract_map import from_contract, load_contract_map
from dronehacks_autonomy.factory import flight_checks, prepare
from dronehacks_autonomy.frames import model_odom_frame
from dronehacks_autonomy.mission import MissionConfig
from dronehacks_autonomy.planning.limits import VehicleLimits
from dronehacks_autonomy.vehicle_sim import VehicleSimConfig, fly, summary

CONFIG = MissionConfig(rest_height_m=0.07, land=True)
PACKAGE = _paths.PACKAGE_ROOT / "dronehacks_autonomy"
PUBLISHED = yaml.safe_load(_paths.CONTRACT_MAP.read_text(encoding="utf-8"))


def fly_doc(doc, use_window=False):
    """Write ``doc`` to a file and fly it through the entry point the node uses."""
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "course.yaml"
        path.write_text(yaml.safe_dump(doc), encoding="utf-8")
        cmap = load_contract_map(path, use_window)
        frame = model_odom_frame(cmap.launch.position, cmap.launch.heading)
        prepared = prepare(path, CONFIG, frame=frame, use_window=use_window)
    log = fly(prepared.mission, VehicleLimits(), VehicleSimConfig())
    result = summary(log)
    result.update(flight_checks(prepared, log.pos))
    return prepared, log, result


class FlownCase(unittest.TestCase):
    def assert_clean_finish(self, doc, use_window=False):
        prepared, log, result = fly_doc(doc, use_window)
        self.assertEqual(log.final_state, "STOPPED", result["final_reason"])
        self.assertIn("disarm", log.events)
        self.assertEqual(result["wall_touches"], 0, result)
        self.assertTrue(all(result["openings"].values()), result["openings"])
        self.assertTrue(result["on_pad"], result)
        return prepared, result


class NewLayoutsTest(FlownCase):
    def test_random_courses(self):
        for seed in range(12):
            with self.subTest(seed=seed):
                prepared, _ = self.assert_clean_finish(random_course(seed))
                self.assertEqual(prepared.plan.route.goals, ["tunnel", "pad"])

    def test_a_course_with_no_maze_at_all(self):
        # A route derived from "maze dividers" has nothing to work with here.
        self.assert_clean_finish(random_course(101, dividers=0))

    def test_a_longer_maze(self):
        self.assert_clean_finish(random_course(102, dividers=4))

    def test_different_maps_give_different_routes(self):
        lengths = {round(fly_doc(random_course(s))[0].plan.route.length_m, 1) for s in (201, 202, 203)}
        self.assertEqual(len(lengths), 3)

    def test_window_route_when_reviewed(self):
        prepared, result = self.assert_clean_finish(random_course(103, clearance_radius=0.35), use_window=True)
        self.assertEqual(prepared.plan.route.goals, ["tunnel", "window", "pad"])
        self.assertTrue(result["openings"]["window"])


class PublishedVariantsTest(FlownCase):
    def test_as_published(self):
        prepared, _ = self.assert_clean_finish(PUBLISHED)
        self.assertEqual(prepared.plan.route.goals, ["tunnel", "pad"])

    def test_rotated(self):
        for degrees in (37.0, 90.0, 180.0):
            with self.subTest(degrees=degrees):
                self.assert_clean_finish(rigid(PUBLISHED, degrees * 3.14159265358979 / 180.0))

    def test_mirrored(self):
        self.assert_clean_finish(mirror_east_west(PUBLISHED))

    def test_walls_moved_slightly(self):
        # contracts/course.md: the layout "may change slightly". The map is re-surveyed and replanned.
        for seed in range(6):
            with self.subTest(seed=seed):
                self.assert_clean_finish(jitter_walls(PUBLISHED, seed, 0.05))


class NoLayoutKnowledgeInCodeTest(unittest.TestCase):
    """Rule 9.5 / AGENTS.md section 3.9, checked on what the AI code review screens."""

    @staticmethod
    def sources():
        return sorted(PACKAGE.rglob("*.py")) + sorted((_paths.PACKAGE_ROOT / "launch").glob("*.py"))

    def test_no_number_from_the_course_appears_in_code(self):
        course = set()

        def add(v):
            if isinstance(v, bool):
                return
            if isinstance(v, (int, float)) and abs(v) > 0.05:
                course.add(round(abs(float(v)), 3))
            elif isinstance(v, (list, tuple)):
                for x in v:
                    add(x)
            elif isinstance(v, dict):
                for x in v.values():
                    add(x)

        add(PUBLISHED["boulders"])
        for g in PUBLISHED["goals"]:
            add(g["centre"])
            add(g["params"])
        for s in PUBLISHED["segments"]:
            add(s["start"])
            add(s["end"])
        course -= {1.0}
        found = []
        for path in self.sources():
            for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"))):
                if isinstance(node, ast.Constant) and isinstance(node.value, float):
                    if round(abs(node.value), 3) in course:
                        found.append(f"{path.name}:{node.lineno}: {node.value}")
        self.assertEqual(found, [], "course numbers in code")

    def test_no_course_feature_names_in_executable_code(self):
        # Docstrings may explain the course. Identifiers and string values may not depend on it.
        banned = re.compile(r"maze|chicane|barrier|divider|gate_?[ab]\b|\bc2[ab]\b|\ba1\b", re.IGNORECASE)
        found = []
        for path in self.sources():
            tree = ast.parse(path.read_text(encoding="utf-8"))
            docstrings = set()
            for node in ast.walk(tree):
                if isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef)):
                    if node.body and isinstance(node.body[0], ast.Expr) and isinstance(node.body[0].value, ast.Constant):
                        docstrings.add(id(node.body[0].value))
            for node in ast.walk(tree):
                text = None
                if isinstance(node, ast.Constant) and isinstance(node.value, str) and id(node) not in docstrings:
                    text = node.value
                elif isinstance(node, ast.Name):
                    text = node.id
                elif isinstance(node, (ast.FunctionDef, ast.ClassDef)):
                    text = node.name
                if text and banned.search(text):
                    found.append(f"{path.name}:{getattr(node, 'lineno', '?')}: {text[:60]!r}")
        self.assertEqual(found, [], "course-specific names in code")

    def test_the_kit_boundary_is_respected(self):
        for path in self.sources():
            text = path.read_text(encoding="utf-8")
            with self.subTest(path=path.name):
                self.assertNotRegex(text, r"create_publisher\([^)]*/fmu/in")
                self.assertNotIn("ACRO", text)


if __name__ == "__main__":
    unittest.main()
