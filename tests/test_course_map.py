"""Stdlib course checks discovered by the existing isolated CI runner."""
import contextlib
import importlib.util
import io
import math
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

# Whitespace-tolerant: SolidWorks writes "#1 = CARTESIAN_POINT ( 'NONE',  ( x, y, z ) ) ;".
POINT = re.compile(r"#(\d+)\s*=\s*CARTESIAN_POINT\s*\(\s*'[^']*'\s*,\s*\(([^)]*)\)\s*\)\s*;")


def box_step(products, spaced=False):
    """A minimal AP214 model of axis-aligned boxes, Onshape-compact or SolidWorks-spaced.

    ``products`` is ``[(product name, [(body name, (x0, y0, z0), (x1, y1, z1)), ...]), ...]``;
    each product gets the full PRODUCT -> SHAPE_DEFINITION_REPRESENTATION chain.
    """
    entities = []

    def add(body):
        entities.append(body)
        return f"#{len(entities)}"

    unit = add("(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT($,.METRE.))")
    context = add(f"(GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNIT_ASSIGNED_CONTEXT(({unit}))REPRESENTATION_CONTEXT('',''))")
    axes = [add(f"DIRECTION('',({','.join('1.' if i == k else '0.' for i in range(3))}))") for k in range(3)]
    for product, bodies in products:
        breps = []
        for body, low, high in bodies:
            def corner(key):
                return tuple(high[i] if key[i] else low[i] for i in range(3))

            def point(key):
                return add("CARTESIAN_POINT('',(" + ",".join(repr(float(v)) for v in corner(key)) + "))")

            keys = [(i, j, k) for i in (0, 1) for j in (0, 1) for k in (0, 1)]
            vertices = {key: add(f"VERTEX_POINT('',{point(key)})") for key in keys}
            edges = {}
            for key in keys:
                for axis in range(3):
                    if key[axis] == 0:
                        other = tuple(1 if i == axis else key[i] for i in range(3))
                        vector = add(f"VECTOR('',{axes[axis]},{float(high[axis] - low[axis])!r})")
                        line = add(f"LINE('',{point(key)},{vector})")
                        edges[(key, other)] = add(f"EDGE_CURVE('',{vertices[key]},{vertices[other]},{line},.T.)")
            faces = []
            for axis in range(3):
                for side in (0, 1):
                    a, b = [i for i in range(3) if i != axis]
                    ring = []
                    for u, v in ((0, 0), (1, 0), (1, 1), (0, 1)):
                        key = [0, 0, 0]
                        key[axis], key[a], key[b] = side, u, v
                        ring.append(tuple(key))
                    oriented = []
                    for first, second in zip(ring, ring[1:] + ring[:1]):
                        forward = (first, second) in edges
                        edge = edges[(first, second)] if forward else edges[(second, first)]
                        oriented.append(add(f"ORIENTED_EDGE('',*,*,{edge},{'.T.' if forward else '.F.'})"))
                    loop = add(f"EDGE_LOOP('',({','.join(oriented)}))")
                    frame = add(f"AXIS2_PLACEMENT_3D('',{point(ring[0])},{axes[axis]},{axes[a]})")
                    plane = add(f"PLANE('',{frame})")
                    bound = add(f"FACE_OUTER_BOUND('',{loop},.T.)")
                    faces.append(add(f"ADVANCED_FACE('',({bound}),{plane},{'.T.' if side else '.F.'})"))
            shell = add(f"CLOSED_SHELL('',({','.join(faces)}))")
            breps.append(add(f"MANIFOLD_SOLID_BREP('{body}',{shell})"))
        brep_rep = add(f"ADVANCED_BREP_SHAPE_REPRESENTATION('',({','.join(breps)}),{context})")
        shape_rep = add(f"SHAPE_REPRESENTATION('{product}',(),{context})")
        add(f"SHAPE_REPRESENTATION_RELATIONSHIP('','',{shape_rep},{brep_rep})")
        product_entity = add(f"PRODUCT('{product}','{product}','',())")
        formation = add(f"PRODUCT_DEFINITION_FORMATION('','',{product_entity})")
        definition = add(f"PRODUCT_DEFINITION('design','',{formation},$)")
        shape = add(f"PRODUCT_DEFINITION_SHAPE('','',{definition})")
        add(f"SHAPE_DEFINITION_REPRESENTATION({shape},{shape_rep})")
    lines = [f"#{number}={body};" for number, body in enumerate(entities, start=1)]
    if spaced:  # SolidWorks style, outside quoted strings only, with CRLF line ends
        def space(text):
            pieces = text.split("'")
            pieces[::2] = [re.sub(r"\s*([(),=;])\s*", r" \1 ", piece).replace(" ,", ",") for piece in pieces[::2]]
            return "'".join(pieces)
        lines = [space(line) for line in lines]
    newline = "\r\n" if spaced else "\n"
    header = ["ISO-10303-21;", "HEADER;", "FILE_DESCRIPTION(('test'),'2;1');", "ENDSEC;", "DATA;"]
    return newline.join(header + lines + ["ENDSEC;", "END-ISO-10303-21;", ""])


class CourseMapTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.base = course_map.generate_map(MODEL, 0.0, None)

    def run_generator(self, model, output, *options):
        return subprocess.run(
            [sys.executable, str(SCRIPT), "--step", str(model),
             "--output", str(output), "--launch-heading", "0", *options],
            capture_output=True, text=True, timeout=60,
        )

    def rewrite_part(self, prefix, change):
        """Rewrite every Cartesian point of the first solid named ``prefix...``."""
        entities = course_map.parse_entities(MODEL)
        solids = course_map.part_solids(entities)
        brep = solids[next(name for name in solids if name.startswith(prefix))]
        reachable = course_map.reachable_entities(brep, entities)
        changed = 0

        def rewrite(match):
            nonlocal changed
            if int(match[1]) not in reachable:
                return match[0]
            values = change([value.strip() for value in match[2].split(",")])
            changed += 1
            return f"#{match[1]}=CARTESIAN_POINT('',({','.join(values)}));"

        text = POINT.sub(rewrite, MODEL.read_text(encoding="utf-8"))
        self.assertGreater(changed, 0)
        return text

    def goal(self, data, name):
        return next(goal for goal in data["goals"] if goal["name"] == name)

    def test_real_model_matches_contract(self):
        data = self.base
        self.assertEqual(course_map.structural_check(data), [])
        self.assertEqual(course_map.nominal_check(data), [])
        self.assertIsNone(self.goal(data, "window")["params"]["clearance_radius"])

    def test_valid_cli_reproduces_committed_geometry(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "map.yaml"
            result = self.run_generator(MODEL, output, "--self-check")
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn("SELF-CHECK: PASS", result.stdout)
            committed = ROOT / "contracts/course_v3_layout.yaml"
            # Only the regeneration command changes with the temporary path.
            self.assertEqual(output.read_text(encoding="utf-8").splitlines()[2:],
                             committed.read_text(encoding="utf-8").splitlines()[2:])

    def test_nonfinite_geometry_never_replace_output(self):
        # Non-finite source coordinates are rejected as such; a token that is
        # not a STEP number at all (nan) is malformed; an extreme but finite
        # coordinate (1.E308) collapses the tube's geometry. None may ever
        # replace or create the output file.
        cases = (("1.E999", "Boulder ", "finite"), ("-1.E999", "Boulder ", "finite"),
                 ("nan", "Boulder ", "STEP"), ("1.E308", "Tube oval", ""))
        for token, prefix, message in cases:
            with self.subTest(coordinate=token), tempfile.TemporaryDirectory() as directory:
                text = self.rewrite_part(prefix, lambda values: [token, *values[1:]])
                model = Path(directory) / "invalid.step"
                model.write_text(text, encoding="utf-8")
                output = Path(directory) / "map.yaml"
                output.write_text("sentinel")
                for options in ((), ("--self-check",)):
                    result = self.run_generator(model, output, *options)
                    self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
                    self.assertIn(message, result.stderr)
                    self.assertNotIn("SELF-CHECK: PASS", result.stdout)
                    self.assertEqual(output.read_text(), "sentinel")
                absent = Path(directory) / "absent.yaml"
                result = self.run_generator(model, absent, "--self-check")
                self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
                self.assertFalse(absent.exists())

    def test_derived_overflow_never_replaces_output(self):
        # Derived values can overflow even from finite coordinates; the
        # structural check is the backstop that keeps them out of the map.
        overflowed = {**self.base, "goals": [dict(goal) for goal in self.base["goals"]]}
        tunnel = next(goal for goal in overflowed["goals"] if goal["name"] == "tunnel")
        tunnel["centre"] = [float("inf"), tunnel["centre"][1]]
        self.assertEqual(course_map.structural_check(overflowed), ["map geometry is not finite"])
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "map.yaml"
            output.write_text("sentinel")
            stderr = io.StringIO()
            with patch.object(course_map, "generate_map", return_value=overflowed), \
                    contextlib.redirect_stderr(stderr), contextlib.redirect_stdout(io.StringIO()):
                code = course_map.main(["--step", str(MODEL), "--output", str(output), "--launch-heading", "0"])
            self.assertEqual(code, 1)
            self.assertIn("finite", stderr.getvalue())
            self.assertEqual(output.read_text(), "sentinel")

    def test_window_centre_ignores_point_multiplicity(self):
        window = self.goal(self.base, "window")
        entities = course_map.parse_entities(MODEL)
        solids = course_map.part_solids(entities)
        name = next(name for name in solids if name.endswith(course_map.WINDOW_SUFFIX))
        original = course_map.solid_geometry
        _, _, opening = course_map.window_opening(name, original(entities, solids[name], name).points)
        # Separate faces/edges may use distinct point entities at one corner.
        for sample in (opening[0], opening[-1]):
            with self.subTest(sample=sample):
                def resampled(model_entities, brep, solid_name):
                    solid = original(model_entities, brep, solid_name)
                    if solid_name == name:
                        solid.points = solid.points + [sample] * 20
                    return solid

                with patch.object(course_map, "solid_geometry", side_effect=resampled):
                    data = course_map.generate_map(MODEL, 0.0, None)
                self.assertEqual(course_map.structural_check(data), [])
                self.assertEqual(course_map.nominal_check(data), [])
                self.assertEqual(self.goal(data, "window"), window)

    def test_unsupported_walls_preserve_output(self):
        changes = {
            "floating": lambda v: [v[0], v[1], repr(float(v[2]) + 0.25)],  # spans the slice, off the floor
            "sloping": lambda v: [repr(float(v[0]) + 0.15 * float(v[2])), *v[1:]],
        }
        for name, change in changes.items():
            with self.subTest(wall=name), tempfile.TemporaryDirectory() as directory:
                model = Path(directory) / "invalid.step"
                model.write_text(self.rewrite_part("A1 approach wall", change), encoding="utf-8")
                output = Path(directory) / "map.yaml"
                output.write_text("sentinel")
                for options in ((), ("--self-check",)):
                    result = self.run_generator(model, output, *options)
                    self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
                    self.assertIn("A1 approach wall", result.stderr)
                    self.assertNotIn("SELF-CHECK: PASS", result.stdout)
                    self.assertEqual(output.read_text(), "sentinel")
                absent = Path(directory) / "absent.yaml"
                result = self.run_generator(model, absent)
                self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
                self.assertFalse(absent.exists())

    def test_part_naming_rule(self):
        label = course_map.part_label
        # SolidWorks decorations are removed ...
        self.assertEqual(label("Barrier 1.step"), "Barrier 1")
        self.assertEqual(label("C2a chicane wall (window).step"), "C2a chicane wall (window)")
        self.assertEqual(label("Tube oval 58 x 72 x 60.STP"), "Tube oval 58 x 72 x 60")
        self.assertEqual(label("Bracket.SLDPRT"), "Bracket")
        self.assertEqual(label("Sub assembly.sldasm"), "Sub assembly")
        self.assertEqual(label("Part67^dronehacks-course-v3.step"), "Part67")
        self.assertEqual(label("Part67^dronehacks-course-v3"), "Part67")
        # ... and Onshape names come out unchanged.
        for name in ("Barrier 1", "Barrier 3 (cut to 55 in)", "Landing pad 5 ft", "Net side east", "M1 maze divider"):
            self.assertEqual(label(name), name)
        bodies = course_map.body_labels
        self.assertEqual(bodies("Barrier 1", ["dronehacks-course-v3.step<1>"]), ["Barrier 1"])
        self.assertEqual(bodies("Part67", ["Boss-Extrude1[1]", "Boss-Extrude1[2]"]),
                         ["Part67 / Boss-Extrude1[1]", "Part67 / Boss-Extrude1[2]"])
        self.assertEqual(bodies("Part9", ["Body", "Body"]), ["Part9 / 1", "Part9 / 2"])
        self.assertEqual(bodies("Part9", ["", "Body"]), ["Part9 / 1", "Part9 / 2"])

    def test_real_model_names_every_solid(self):
        names = list(course_map.part_solids(course_map.parse_entities(MODEL)))
        self.assertEqual(len(names), len(set(names)))
        self.assertIn("Part67 / Boss-Extrude1[1]", names)
        self.assertIn("Part67 / Boss-Extrude1[2]", names)
        self.assertIn("C2a chicane wall (window)", names)
        self.assertIn("Tube oval 58 x 72 x 60", names)
        self.assertFalse([name for name in names if name.lower().endswith(".step") or "^" in name])
        for removed in ("M1 maze divider", "M2 maze divider", "M3 maze divider", "Net roof"):
            self.assertNotIn(removed, names)

    def test_new_layout_walls_are_separate_segments(self):
        data = self.base
        named = dict(zip(data["_segment_names"], zip(data["segments"], data["_segment_kinds"])))
        box, box_kind = named["Part67 / Boss-Extrude1[1]"]
        diagonal, diagonal_kind = named["Part67 / Boss-Extrude1[2]"]
        self.assertEqual((box_kind, diagonal_kind), ("wall", "wall"))
        # Measured: a 1.8 x 0.350 m box and a 0.100 m diagonal at 42.9 deg from east.
        self.assertAlmostEqual(box["thickness"], 0.350, delta=1e-6)
        self.assertAlmostEqual(math.dist(box["start"], box["end"]), 1.8, delta=1e-6)
        self.assertAlmostEqual(box["start"][1], box["end"][1], delta=1e-9)
        self.assertAlmostEqual(diagonal["thickness"], 0.100, delta=1e-5)
        heading = math.degrees(math.atan2(diagonal["end"][1] - diagonal["start"][1],
                                          diagonal["end"][0] - diagonal["start"][0])) % 180.0
        self.assertAlmostEqual(heading, 42.9, delta=0.1)
        self.assertEqual(data["_segment_kinds"].count("net"), 4)
        thicknesses = {segment["thickness"] for segment, kind in zip(data["segments"], data["_segment_kinds"]) if kind == "wall"}
        self.assertEqual(thicknesses, {0.1, 0.1016, 0.35})

    def test_bspline_boulders_and_tube_are_derived_from_geometry(self):
        entities = course_map.parse_entities(MODEL)
        solids = course_map.part_solids(entities)
        for name, brep in solids.items():
            if name.startswith(("Boulder ", "Tube oval")):
                solid = course_map.solid_geometry(entities, brep, name)
                # SolidWorks wrote these outlines as B-spline curves, not CIRCLE/ELLIPSE.
                self.assertEqual(solid.conics, [], name)
                self.assertTrue(any("bspline" in kinds for face in solid.faces for kinds in face.loop_kinds), name)
        self.assertEqual(len(self.base["boulders"]), 5)
        for boulder in self.base["boulders"]:
            self.assertAlmostEqual(boulder["radius"], 0.2286, delta=1e-6)
        tunnel = self.goal(self.base, "tunnel")["params"]
        self.assertAlmostEqual(tunnel["width"], 1.4732, delta=1e-6)
        self.assertAlmostEqual(tunnel["height"], 1.8288, delta=1e-6)

    def test_onshape_and_solidworks_formatting_parse_alike(self):
        products = [("Wall A", [("Wall A", (0.0, 0.0, 0.0), (2.0, 0.1, 1.7))])]
        texts = [box_step(products, spaced=spaced) for spaced in (False, True)]
        self.assertIn("#1=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT($,.METRE.));", texts[0])  # Onshape style
        self.assertRegex(texts[1], r"#1 = +\( LENGTH_UNIT \( +\) NAMED_UNIT \( \* \) SI_UNIT \( \$, \.METRE\. \) +\) +; *\r\n")  # SolidWorks style
        with tempfile.TemporaryDirectory() as directory:
            parsed = []
            for index, text in enumerate(texts):
                path = Path(directory) / f"model-{index}.step"
                path.write_bytes(text.encode("utf-8"))
                parsed.append(course_map.parse_entities(path))
        compact, spaced = parsed
        self.assertEqual({n: (e.type, e.args, e.parts) for n, e in compact.items()},
                         {n: (e.type, e.args, e.parts) for n, e in spaced.items()})
        unit = compact[1]
        self.assertIsNone(unit.type)  # a complex instance, read part by part
        self.assertEqual(unit.parts["SI_UNIT"], [None, "METRE"])
        solids = course_map.part_solids(compact)
        self.assertEqual(list(solids), ["Wall A"])
        solid = course_map.solid_geometry(compact, solids["Wall A"], "Wall A")
        segment = course_map.segment_from_points(solid.points, "Wall A")
        self.assertAlmostEqual(segment["thickness"], 0.1, delta=1e-12)
        ends = sorted([segment["start"], segment["end"]])
        for measured, expected in zip(ends, [(0.0, 0.05), (2.0, 0.05)]):
            self.assertAlmostEqual(measured[0], expected[0], delta=1e-12)
            self.assertAlmostEqual(measured[1], expected[1], delta=1e-12)

    def test_multi_body_part_gives_one_solid_per_body(self):
        products = [
            ("Part9^course.step", [("Boss-Extrude1[1]", (0.0, 0.0, 0.0), (2.0, 0.35, 1.7)),
                                   ("Boss-Extrude1[2]", (3.0, 0.0, 0.0), (3.1, 2.0, 1.7))]),
            ("Barrier 1.step", [("course.step<1>", (0.0, 1.0, 0.0), (1.0, 1.1, 1.7))]),
        ]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "model.step"
            path.write_bytes(box_step(products, spaced=True).encode("utf-8"))
            entities = course_map.parse_entities(path)
        solids = course_map.part_solids(entities)
        self.assertEqual(list(solids), ["Barrier 1", "Part9 / Boss-Extrude1[1]", "Part9 / Boss-Extrude1[2]"])
        thicknesses = [
            course_map.segment_from_points(course_map.solid_geometry(entities, brep, name).points, name)["thickness"]
            for name, brep in solids.items()
        ]
        # Merging the two bodies into one hull would give one 3.1 m-wide footprint.
        self.assertEqual([round(value, 9) for value in thicknesses], [0.1, 0.35, 0.1])

    def test_length_unit_must_be_metres(self):
        text = box_step([("Wall A", [("Wall A", (0.0, 0.0, 0.0), (2.0, 0.1, 1.7))])])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "model.step"
            path.write_text(text.replace("SI_UNIT($,.METRE.)", "SI_UNIT(.MILLI.,.METRE.)"), encoding="utf-8")
            with self.assertRaisesRegex(course_map.StepError, "not metres"):
                course_map.parse_entities(path)


if __name__ == "__main__":
    unittest.main()
