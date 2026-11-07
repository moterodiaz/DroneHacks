"""Stdlib course checks discovered by the existing isolated CI runner."""
import importlib.util
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "course/step_to_map.py"
MODEL = ROOT / "dronehacks-course-v3.step"
spec = importlib.util.spec_from_file_location("course_map", SCRIPT)
course_map = importlib.util.module_from_spec(spec)
spec.loader.exec_module(course_map)


class CourseMapTests(unittest.TestCase):
    def run_generator(self, model, output, *options):
        return subprocess.run(
            [sys.executable, str(SCRIPT), "--step", str(model),
             "--output", str(output), "--launch-heading", "0", *options],
            capture_output=True, text=True, timeout=20,
        )

    def rewrite_part(self, prefix, change):
        entities = course_map.parse_entities(MODEL)
        roots = course_map.named_brep_roots(entities)
        root = roots[next(name for name in roots if name.startswith(prefix))]
        reachable = course_map.reachable_entities(root, entities)
        changed = 0

        def rewrite(match):
            nonlocal changed
            if int(match[1]) not in reachable:
                return match[0]
            values = change(match[2].split(","))
            changed += 1
            return f"#{match[1]} = CARTESIAN_POINT('',({','.join(values)}))"

        text = re.sub(
            r"#(\d+)\s*=\s*CARTESIAN_POINT\('[^']*',\(([^)]*)\)\)",
            rewrite, MODEL.read_text(),
        )
        self.assertGreater(changed, 0)
        return text

    def test_real_model_matches_contract(self):
        data = course_map.generate_map(MODEL, 0.0, None)
        self.assertEqual(course_map.structural_check(data), [])
        self.assertEqual(course_map.nominal_check(data), [])
        window = next(goal for goal in data["goals"] if goal["name"] == "window")
        self.assertIsNone(window["params"]["clearance_radius"])

    def test_valid_cli_reproduces_committed_geometry(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "map.yaml"
            result = self.run_generator(MODEL, output, "--self-check")
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn("SELF-CHECK: PASS", result.stdout)
            committed = ROOT / "contracts/course_v3_layout.yaml"
            # Only the regeneration command changes with the temporary path.
            self.assertEqual(output.read_text().splitlines()[2:],
                             committed.read_text().splitlines()[2:])

    def test_nonfinite_geometry_never_replace_output(self):
        for token in ("1.E999", "-1.E999", "nan", "1.E308"):
            with self.subTest(coordinate=token), tempfile.TemporaryDirectory() as directory:
                # The tube midpoint adds two finite centres, so 1.E308 tests
                # derived overflow rather than an already nonfinite token.
                prefix = "Tube oval" if token == "1.E308" else "Boulder "
                text = self.rewrite_part(prefix, lambda values: [token, *values[1:]])
                model = Path(directory) / "invalid.step"
                model.write_text(text)
                output = Path(directory) / "map.yaml"
                output.write_text("sentinel")
                for options in ((), ("--self-check",)):
                    for _ in range(2):
                        result = self.run_generator(model, output, *options)
                        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
                        self.assertIn("finite", result.stderr)
                        self.assertNotIn("SELF-CHECK: PASS", result.stdout)
                        self.assertEqual(output.read_text(), "sentinel")
                    absent = Path(directory) / "absent.yaml"
                    result = self.run_generator(model, absent, *options)
                    self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
                    self.assertFalse(absent.exists())

    def test_window_centre_ignores_point_multiplicity(self):
        base = course_map.generate_map(MODEL, 0.0, None)
        window = next(goal for goal in base["goals"] if goal["name"] == "window")
        entities = course_map.parse_entities(MODEL)
        roots = course_map.named_brep_roots(entities)
        name = next(name for name in roots if name.endswith(course_map.WINDOW_SUFFIX))
        original = course_map.part_geometry
        points, _, _ = original(roots[name], entities)
        _, _, opening = course_map.window_opening(name, points)
        # Separate faces/edges may use distinct point entities at one corner.
        for sample in (opening[0], opening[-1]):
            with self.subTest(sample=sample):
                def resampled(root, model_entities):
                    points, circles, ellipses = original(root, model_entities)
                    if root == roots[name]:
                        points = points + [sample] * 20
                    return points, circles, ellipses

                with patch.object(course_map, "part_geometry", side_effect=resampled):
                    data = course_map.generate_map(MODEL, 0.0, None)
                self.assertEqual(course_map.structural_check(data), [])
                self.assertEqual(course_map.nominal_check(data), [])
                after = next(goal for goal in data["goals"] if goal["name"] == "window")
                self.assertEqual(after, window)

    def test_unsupported_walls_preserve_output(self):
        changes = {
            "below_slice": lambda v: [v[0], v[1], repr(float(v[2]) * 0.2)],
            "sloping": lambda v: [repr(float(v[0]) + 0.15 * float(v[2])), *v[1:]],
        }
        for name, change in changes.items():
            with self.subTest(wall=name), tempfile.TemporaryDirectory() as directory:
                model = Path(directory) / "invalid.step"
                model.write_text(self.rewrite_part("M1 maze divider", change))
                output = Path(directory) / "map.yaml"
                output.write_text("sentinel")
                for options in ((), ("--self-check",)):
                    for _ in range(2):
                        result = self.run_generator(model, output, *options)
                        self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
                        self.assertNotIn("SELF-CHECK: PASS", result.stdout)
                        self.assertEqual(output.read_text(), "sentinel")
                    absent = Path(directory) / "absent.yaml"
                    result = self.run_generator(model, absent, *options)
                    self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
                    self.assertFalse(absent.exists())


if __name__ == "__main__":
    unittest.main()
