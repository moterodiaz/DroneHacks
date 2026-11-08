// Team 08 technical presentation (8 slides). Rules: docs/presentation/DESIGN.md.
// Numbers come from technical-corpus/TECHNICAL_CORPUS.md; each carries a FLOWN / SITL / MODEL chip.
//   NODE_PATH=<dir with pptxgenjs, react, react-dom, react-icons, sharp> node docs/presentation/build_deck.js
//   SLIDES=5 builds only the first five (batch review).
const path = require("path");
const pptxgen = require("pptxgenjs");
const React = require("react");
const ReactDOMServer = require("react-dom/server");
const sharp = require("sharp");
const fa = require("react-icons/fa");

const HERE = __dirname;
const OUT = path.join(HERE, "DroneHacks-Team08-technical.pptx");
const APPLY_THEME = process.env.APPLY_THEME; // path to the pptx skill's scripts/apply_theme.js
const LIMIT = Number(process.env.SLIDES || 8);

const THEME = {
  name: "Team 08 Night Arena",
  headFontFace: "Arial",
  bodyFontFace: "Calibri",
  colors: {
    dk1: "0E1A2B", lt1: "FFFFFF", dk2: "3D4B5C", lt2: "EEF2F6",
    accent1: "F26B1D", accent2: "3F6E9E", accent3: "2BA36B", accent4: "C9D6E3",
    accent5: "8A97A6", accent6: "D64545", hlink: "3F6E9E", folHlink: "8A97A6",
  },
};
const HEX = THEME.colors;

const pres = new pptxgen();
pres.layout = "LAYOUT_16x9"; // 10 x 5.625 in
pres.author = "Team 08";
pres.title = "Drone 11 flies the course by itself";
pres.theme = { headFontFace: THEME.headFontFace, bodyFontFace: THEME.bodyFontFace };
const C = pres.SchemeColor;
const INK = C.text1, SLATE = C.text2, PAPER = C.background1, MIST = C.background2;
const ORANGE = C.accent1, STEEL = C.accent2, GO = C.accent3, ICE = C.accent4, GRAY = C.accent5, STOP = C.accent6;

// ---------- layouts ----------
const FOOT = "Team 08  ·  skybeta-11  ·  DroneHacks 2026";
pres.defineSlideMaster({
  title: "DARK_TITLE",
  background: { color: INK },
  objects: [
    { placeholder: { options: { name: "title", type: "title", x: 0.5, y: 1.2, w: 5.2, h: 1.6, fontFace: "Arial", fontSize: 38, bold: true, color: PAPER, align: "left", valign: "top", margin: 0 }, text: "" } },
  ],
});
pres.defineSlideMaster({
  title: "DARK_CLOSE",
  background: { color: INK },
  objects: [
    { placeholder: { options: { name: "title", type: "title", x: 0.5, y: 0.45, w: 9.0, h: 0.8, fontFace: "Arial", fontSize: 36, bold: true, color: PAPER, align: "left", valign: "middle", margin: 0 }, text: "" } },
  ],
});
pres.defineSlideMaster({
  title: "CONTENT",
  background: { color: PAPER },
  margin: [0.5, 0.5, 0.5, 0.5],
  objects: [
    { placeholder: { options: { name: "title", type: "title", x: 0.5, y: 0.32, w: 9.0, h: 0.7, fontFace: "Arial", fontSize: 36, bold: true, color: INK, align: "left", valign: "middle", margin: 0 }, text: "" } },
    { text: { text: FOOT, options: { x: 0.5, y: 5.22, w: 6, h: 0.25, fontFace: "Calibri", fontSize: 10, color: GRAY, margin: 0 } } },
  ],
  slideNumber: { x: 9.0, y: 5.22, w: 0.5, h: 0.25, fontFace: "Calibri", fontSize: 10, color: GRAY, align: "right" },
});

// ---------- helpers ----------
async function icon(Comp, hex, size = 256) {
  const svg = ReactDOMServer.renderToStaticMarkup(React.createElement(Comp, { color: "#" + hex, size: String(size) }));
  const buf = await sharp(Buffer.from(svg)).png().toBuffer();
  return "image/png;base64," + buf.toString("base64");
}
const T = (o) => Object.assign({ isTextBox: true, margin: 0, fontFace: "Calibri", valign: "top" }, o);

function waypoint(slide, x, y, n, d = 0.42) {
  slide.addShape(pres.shapes.OVAL, { x, y, w: d, h: d, fill: { color: INK }, line: { color: ORANGE, width: 2.25 }, objectName: `waypoint ${n}` });
  slide.addText(String(n), T({ x, y, w: d, h: d, fontFace: "Arial", fontSize: d > 0.4 ? 15 : 12, bold: true, color: PAPER, align: "center", valign: "middle" }));
}
const CHIP = { FLOWN: ORANGE, SITL: STEEL, MODEL: GRAY };
function chip(slide, x, y, kind, w = 0.62) {
  slide.addShape(pres.shapes.ROUNDED_RECTANGLE, { x, y, w, h: 0.22, rectRadius: 0.11, fill: { color: CHIP[kind] }, line: { type: "none" }, objectName: `source ${kind}` });
  slide.addText(kind, T({ x, y, w, h: 0.22, fontFace: "Arial", fontSize: 9, bold: true, color: PAPER, align: "center", valign: "middle", charSpacing: 1 }));
}
function card(slide, x, y, w, h, fill = MIST, name = "card") {
  slide.addShape(pres.shapes.ROUNDED_RECTANGLE, { x, y, w, h, rectRadius: 0.08, fill: { color: fill }, line: { type: "none" }, objectName: name });
}
function content(title, section) {
  const s = pres.addSlide({ masterName: "CONTENT", sectionTitle: section });
  s.addText(title, { placeholder: "title" });
  return s;
}

async function build() {
  const IC = {
    cam: await icon(fa.FaCamera, HEX.lt1), eye: await icon(fa.FaCrosshairs, HEX.lt1), nav: await icon(fa.FaRoute, HEX.lt1),
    gate: await icon(fa.FaShieldAlt, HEX.lt1), px4: await icon(fa.FaCogs, HEX.lt1),
  };
  const slides = [];

  // ===== 1. Title (dark) =====
  slides.push(() => {
    pres.addSection({ title: "Opening" });
    const s = pres.addSlide({ masterName: "DARK_TITLE", sectionTitle: "Opening" });
    s.addText("TEAM 08  ·  DRONEHACKS 2026  ·  8 NOV", T({ x: 0.5, y: 0.7, w: 5.2, h: 0.3, fontFace: "Arial", fontSize: 11, bold: true, color: ORANGE, charSpacing: 2 }));
    s.addText("Drone 11 flies the course by itself", { placeholder: "title" });
    s.addText("Map in, route out: no GPS, no route typed into code. A stereo camera, a planner and PX4 on skybeta-11.",
      T({ x: 0.5, y: 3.0, w: 5.0, h: 0.75, fontSize: 16, color: ICE }));
    const stats = [["0", "course coordinates in code"], ["7", "navigation designs, measured"], ["~800", "tests across the repo"]];
    stats.forEach(([n, l], i) => {
      const x = 0.5 + i * 1.75;
      s.addText(n, T({ x, y: 3.95, w: 1.6, h: 0.55, fontFace: "Arial", fontSize: 30, bold: true, color: ORANGE }));
      s.addText(l, T({ x, y: 4.5, w: 1.55, h: 0.5, fontSize: 11, color: ICE }));
    });
    card(s, 6.05, 0.55, 3.45, 3.85, PAPER, "route card");
    s.addImage({ path: path.join(HERE, "assets", "route-taped-2026-11-08.png"), x: 6.15, y: 0.65, w: 3.25, h: 3.28, altText: "Planned stop-and-go route on the taped 2026-11-08 course map: launch, corridor, tunnel, window, pad." });
    s.addText("Our planner\u2019s route on the taped course", T({ x: 6.15, y: 4.0, w: 3.25, h: 0.45, fontSize: 11, color: SLATE }));
    s.addNotes(
`(45 s) We are Team 08. Our drone, skybeta-11, flies this course by itself: no GPS, and no route typed into the code.
The one idea to remember: map in, route out. The course lives in one map file; everything else is the same code on any course.
Three numbers: zero course coordinates in our source (a test enforces it), seven navigation designs we built and measured against each other, and about 800 tests.
The picture is the actual route our planner produced on the course as we taped it this morning: launch, the corridor under the north wall, the tunnel, the window, the pad.`);
  });

  // ===== 2. Autonomy design =====
  slides.push(() => {
    pres.addSection({ title: "Autonomy design" });
    const s = content("Four parts, one moving target", "Autonomy design");
    const boxes = [
      ["EYES", "OAK-D S2", "Stereo pairs, 30 per second", IC.cam, false],
      ["SENSE OF PLACE", "cuVSLAM", "Pose from images, relative to start", IC.eye, false],
      ["NAVIGATOR", "Our node", "Plans the route, moves the setpoint, 50 Hz", IC.nav, true],
      ["GATEKEEPER", "Flight interface", "Organisers' limits: 0.3–3 m, ≤1.5 m/s", IC.gate, false],
      ["REFLEXES", "PX4", "Chases the setpoint, drives motors", IC.px4, false],
    ];
    const w = 1.64, g = 0.2, y = 1.3, h = 1.75;
    boxes.forEach(([role, name, det, ic, ours], i) => {
      const x = 0.5 + i * (w + g);
      card(s, x, y, w, h, ours ? INK : MIST, `stage ${name}`);
      s.addShape(pres.shapes.OVAL, { x: x + 0.15, y: y + 0.15, w: 0.42, h: 0.42, fill: { color: ours ? ORANGE : SLATE }, line: { type: "none" } });
      s.addImage({ data: ic, x: x + 0.25, y: y + 0.25, w: 0.22, h: 0.22, altText: name + " icon" });
      s.addText(role, T({ x: x + 0.15, y: y + 0.66, w: w - 0.25, h: 0.22, fontFace: "Arial", fontSize: 9, bold: true, color: ORANGE, charSpacing: 1 }));
      s.addText(name, T({ x: x + 0.15, y: y + 0.88, w: w - 0.25, h: 0.3, fontSize: 15, bold: true, color: ours ? PAPER : INK }));
      s.addText(det, T({ x: x + 0.15, y: y + 1.18, w: w - 0.25, h: 0.52, fontSize: 11, color: ours ? ICE : SLATE }));
      if (i < 4) s.addShape(pres.shapes.LINE, { x: x + w + 0.02, y: y + h / 2, w: g - 0.04, h: 0, line: { color: SLATE, width: 1.5, endArrowType: "triangle" } });
    });
    // phases
    s.addText("ONE FLIGHT", T({ x: 0.5, y: 3.32, w: 2, h: 0.22, fontFace: "Arial", fontSize: 10, bold: true, color: SLATE, charSpacing: 2 }));
    const ph = [["WAIT", "rest on mark"], ["CLIMB", "to 1.23 m"], ["FLY", "the route"], ["DESCEND", "to pad"], ["RETURN", "at 2.3 m"]];
    const pw = 1.64;
    ph.forEach(([p, d], i) => {
      const x = 0.5 + i * (pw + g);
      s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x, y: 3.6, w: pw, h: 0.5, rectRadius: 0.25, fill: { color: PAPER }, line: { color: i === 2 ? ORANGE : ICE, width: 1.5 }, objectName: `phase ${p}` });
      s.addText([{ text: p, options: { bold: true, color: INK, fontFace: "Arial", fontSize: 11 } }, { text: "  " + d, options: { color: SLATE, fontSize: 11 } }],
        T({ x, y: 3.6, w: pw, h: 0.5, align: "center", valign: "middle" }));
    });
    s.addText([{ text: "We never touch the motors. ", options: { bold: true, color: INK } }, { text: "Our code only moves a target point, and PX4 chases it.", options: { color: SLATE } }],
      T({ x: 0.5, y: 4.4, w: 4.4, h: 0.6, fontSize: 14 }));
    s.addText([{ text: "LOST: ", options: { bold: true, color: STOP } }, { text: "a pose that jumps 0.3\u00A0m at over 3\u00A0m/s is not real, so the drone goes straight down.", options: { color: SLATE } }],
      T({ x: 5.1, y: 4.4, w: 4.4, h: 0.6, fontSize: 14 }));
    s.addNotes(
`(50 s) Think of a car with four people in it, each listening only to the one next to it. The camera is the eyes. NVIDIA's cuVSLAM turns images into a pose, but only relative to where it started. Our node is the navigator: it knows the map and moves a target point along the route 50 times a second. PX4 is the reflexes.
Between us and PX4 sits the organisers' flight interface; it clamps height to 0.3–3 m and speed to 1.5 m/s. Above everyone is the safety pilot.
A flight: wait on the launch mark, climb to 1.23 m (the window's measured centre), fly, descend onto the pad; the return is the last ladder rung.
Design choice: we never command motors directly (MonoRace-style direct-to-motor was assessed and rejected; it throws away PX4's tested control).
Q: Why not a learned policy? A: The PX4 cascade is already a good controller; our problem was knowing where the walls are, not stabilising.`);
  });

  // ===== 3. Planning =====
  slides.push(() => {
    pres.addSection({ title: "Planning and execution" });
    const s = content("Map in, route out", "Planning and execution");
    const steps = [
      ["The map is data", "Walls, goals and their order, from the STEP CAD, corrected with tape where the course differs."],
      ["Keep 0.32 m clear", "5 cm grid; 0.28 m prop reach + 0.04 m. Rocks grow by the rangefinder's 27° cone."],
      ["A* between goals", "Tunnel flown straight from 0.75 m outside each mouth; window crossed on its normal at 1.23 m."],
      ["Fly it, slow first", "Stop-and-go at 0.3 m/s: hold within 0.15 m, settle 1 s, turn in place, go."],
    ];
    steps.forEach(([h, d], i) => {
      const y = 1.25 + i * 0.95;
      waypoint(s, 0.5, y, i + 1);
      s.addText(h, T({ x: 1.1, y: y - 0.02, w: 4.3, h: 0.3, fontSize: 16, bold: true, color: INK }));
      s.addText(d, T({ x: 1.1, y: y + 0.29, w: 4.3, h: 0.58, fontSize: 13, color: SLATE }));
    });
    s.addImage({ path: path.join(HERE, "assets", "route-taped-2026-11-08.png"), x: 6.03, y: 1.15, w: 3.47, h: 3.5, altText: "Stop-and-go route through the window at 1.23 m on the tape-measured 8 Nov map. Pale red is the no-fly zone for the drone's centre; purple waypoints 0 to 8; red cross where run 1 hit the north wall." });
    chip(s, 6.03, 4.8, "SITL");
    s.addText("109 s, window ±7 cm, pad ±7 cm", T({ x: 6.73, y: 4.8, w: 2.9, h: 0.22, fontSize: 10, color: SLATE, valign: "middle" }));
    s.addNotes(
`(50 s) Both of our main stacks share this planning core. The map is data: walls, goals and the order to pass them, generated from the organisers' STEP CAD by our own parser, which fails closed on anything ambiguous. Where the real course differs, we tape-measure and correct the map, which the rules allow.
The drone's centre must stay 0.32 m from any wall: 0.28 m prop reach plus 4 cm. Rocks are inflated by the rangefinder's 27-degree cone, because if the beam sees a rock top, PX4 thinks the floor jumped 0.53 m and climbs.
A* searches between goals. Openings are straight legs: the tunnel from 0.75 m outside each mouth, the window on its normal. In 39 SITL flights the window was the closest pass in 21 until we added the straight lead-in.
We fly stop-and-go first. In SITL on this map: 109 s, no touches, window crossed within 7 cm of centre, landed within 7 cm of the pad.
Q: Why A* and not RRT? A: 2-D, known map, 5 cm grid, optimal and deterministic, plans in under a second in pure Python on the stock image.`);
  });

  // ===== 4. Execution and safety =====
  slides.push(() => {
    const s = content("Earn speed one rung at a time", "Planning and execution");
    const rungs = [["stop", "0.3"], ["", "0.5"], ["", "0.7"], ["", "0.85"], ["", "1.0"], ["+ return", "1.0"]];
    const bx = 0.5, base = 4.75, bw = 0.7, gap = 0.1;
    rungs.forEach(([tag, v], i) => {
      const hh = 0.62 + i * 0.43, x = bx + i * (bw + gap);
      s.addShape(pres.shapes.RECTANGLE, { x, y: base - hh, w: bw, h: hh, fill: { color: i === 0 ? ORANGE : (i === 5 ? INK : ICE) }, line: { type: "none" }, objectName: `rung ${i}` });
      s.addText(v, T({ x, y: base - hh + 0.08, w: bw, h: 0.3, fontFace: "Arial", fontSize: 14, bold: true, color: i === 0 || i === 5 ? PAPER : INK, align: "center" }));
      if (tag) s.addText(tag, T({ x, y: base - hh + 0.36, w: bw, h: 0.22, fontSize: 10, color: i === 0 || i === 5 ? PAPER : INK, align: "center" }));
    });
    s.addText("m/s, set by hand between runs", T({ x: 0.5, y: 4.82, w: 4.6, h: 0.25, fontSize: 11, color: SLATE }));
    s.addText([{ text: "A crash never costs more than one rung. ", options: { bold: true, color: INK } }, { text: "Step 0 flies today; the next rung only after a clean landing.", options: { color: SLATE } }],
      T({ x: 0.5, y: 1.2, w: 4.6, h: 0.7, fontSize: 14 }));
    const guards = [
      ["Before Offboard", "Rest 2 s within 2 cm and 1°. PX4 and camera agree within 0.3 m, or nothing is sent.", null],
      ["In flight", "Setpoint leashed 0.3 m ahead of the drone. Any stick input hands control to the pilot for good.", null],
      ["Our own PX4 mode", "DH Bailout stops in 0.37–0.39 m; stock Land needs 0.54–0.59 m.", "SITL"],
    ];
    guards.forEach(([h, d, k], i) => {
      const y = 1.2 + i * 1.27;
      card(s, 5.45, y, 4.05, 1.12, MIST, `guard ${h}`);
      s.addText(h, T({ x: 5.65, y: y + 0.13, w: 2.8, h: 0.3, fontSize: 15, bold: true, color: INK }));
      if (k) chip(s, 8.7, y + 0.16, k);
      s.addText(d, T({ x: 5.65, y: y + 0.46, w: 3.7, h: 0.6, fontSize: 13, color: SLATE }));
    });
    s.addNotes(
`(45 s) Execution rule: earn speed. The ladder is stop-and-go at 0.3 m/s, then smooth runs at 0.5, 0.7, 0.85, 1.0, then 1.0 with the return home for the 1.25 multiplier. The rung lives in a file on the drone and moves only by hand, because the stack launches our code with no arguments and the code cannot see a wall touch.
Guards: before Offboard the drone must rest on the mark, and PX4's position must agree with the camera's within 0.3 m (that check exists because PX4 and the camera disagreed by 0.72 m in height once). In flight the setpoint is leashed 0.3 m ahead, so PX4 never sees a far target; in the organisers' lab, step setpoints overshot 38–60 cm.
DH Bailout is our own flight mode compiled into PX4: it runs on the flight controller with no Jetson, and wins the setpoint race against the kit's 10 Hz stream. Flashed on skybeta-11, but not on the mode switch until organiser permission is recorded.`);
  });

  // ===== 5. Testing =====
  slides.push(() => {
    pres.addSection({ title: "Testing" });
    const s = content("Every failure became a fix", "Testing");
    chip(s, 7.0, 0.56, "FLOWN");
    s.addText("skybeta-11 logs", T({ x: 7.7, y: 0.56, w: 1.8, h: 0.22, fontSize: 11, color: SLATE, valign: "middle" }));
    const hdr = (t) => ({ text: t, options: { bold: true, color: PAPER, fill: { color: INK }, fontFace: "Arial", fontSize: 11 } });
    const cell = (t, o = {}) => ({ text: t, options: Object.assign({ color: INK, fontSize: 13 }, o) });
    const rows = [
      [hdr("WHAT HAPPENED"), hdr("WHAT THE LOGS SHOWED"), hdr("WHAT WE CHANGED")],
      [cell("Asked for 0.74 m, flew at 1.33 m", { bold: true }), cell("PX4 and camera measured height from different zeros", { color: SLATE }), cell("Measure the gap; add it to every setpoint")],
      [cell("Run 1 hit the north wall", { bold: true }), cell("The hit was 24 inches inside the net: a wall no map had", { color: SLATE }), cell("Taped it; the map moved to the real wall")],
      [cell("Every scored-session flight ended early", { bold: true }), cell("Each ended on a stick input; 3–8 cm from setpoint until then", { color: SLATE }), cell("Hands off in Offboard; route 0.61 m off the wall")],
      [cell("Tracker jumped metres after a hand landing", { bold: true }), cell("Pose logs: a jump no drone can fly", { color: SLATE }), cell("LOST phase: straight down; restart after handling")],
    ];
    s.addTable(rows, { x: 0.5, y: 1.2, w: 9.0, colW: [2.75, 3.4, 2.85], fontFace: "Calibri", border: { type: "solid", pt: 0.75, color: HEX.accent4 }, fill: { color: HEX.lt1 }, margin: [0.06, 0.1, 0.06, 0.1], rowH: [0.34, 0.66, 0.66, 0.66, 0.66], valign: "middle" });
    s.addText([{ text: "The map was wrong, not the localization. ", options: { bold: true, color: ORANGE } }, { text: "Until the pilot's input at 66.2 s, the drone held 3–8 cm from its setpoint.", options: { color: INK } }],
      T({ x: 0.5, y: 4.45, w: 9.0, h: 0.55, fontSize: 15 }));
    s.addNotes(
`(55 s) This is our evidence of testing. Each row: what happened on the real drone, what the bag and PX4 log showed, what changed.
Height: the first square test asked for 0.74 m and flew 1.33 m, because PX4 measures height from the rangefinder and cuVSLAM from where it started. We now measure that gap and add it to every setpoint, and refuse to fly above 0.3 m disagreement.
Run 1 hit a wall 24 inches inside the north net that wasn't in any drawing. The log located the hit to inches, so we taped the wall and moved the map.
In the first scored session, every flight ended because a stick moved; the organisers configured stick override. In the crash, the logs show the drone 3 to 8 cm from its setpoint, no contact, until pilot input at 66.2 s. So the localisation and controller were fine; the map and the procedure were the fixes.
Not on the slide: run 2 turned early after the tunnel because the yaw began when the setpoint arrived while the drone trailed 0.25 m. Turns now wait for the drone itself.`);
  });

  // ===== 6. Experiments and originality =====
  slides.push(() => {
    pres.addSection({ title: "Experiments" });
    const s = content("Ideas tested against each other", "Experiments");
    // Drawn as shapes, not a chart object: Keynote's import drops pptxgenjs charts (DESIGN.md).
    s.addText("Feed-forward velocity tracking: 59 s → 30 s", T({ x: 0.5, y: 1.2, w: 4.6, h: 0.3, fontSize: 15, bold: true, color: INK }));
    const bars = [["Plain", "poses", 59], ["Velocity", "0.5 m/s", 42], ["Velocity", "0.8 m/s", 32], ["Velocity", "1.0 m/s", 30]];
    const base = 4.15, scale = 2.2 / 60, bw2 = 0.85, bg = 0.3, x0 = 0.75;
    bars.forEach(([a, b, v], i) => {
      const x = x0 + i * (bw2 + bg), hh = v * scale;
      s.addShape(pres.shapes.RECTANGLE, { x, y: base - hh, w: bw2, h: hh, fill: { color: i === 0 ? ICE : (i === 3 ? ORANGE : STEEL) }, line: { type: "none" }, objectName: `bar ${a} ${b}` });
      s.addText(v + " s", T({ x: x - 0.1, y: base - hh - 0.34, w: bw2 + 0.2, h: 0.3, fontFace: "Arial", fontSize: 14, bold: true, color: INK, align: "center" }));
      s.addText(a + "\n" + b, T({ x: x - 0.12, y: base + 0.08, w: bw2 + 0.24, h: 0.42, fontSize: 11, color: SLATE, align: "center" }));
    });
    s.addShape(pres.shapes.LINE, { x: 0.6, y: base, w: 4.4, h: 0, line: { color: ICE, width: 1 } });
    chip(s, 0.5, 4.8, "SITL");
    s.addText("Window route, dark tunnel, no touches", T({ x: 1.2, y: 4.8, w: 3.6, h: 0.22, fontSize: 10, color: SLATE, valign: "middle" }));
    const items = [
      ["Our own simulator", "Runs the kit's code unmodified on ported PX4 loops. Found 12 issues before the drone did, including a 0.18 s latency cliff.", "SITL"],
      ["Through the dark tunnel", "cuVSLAM goes blind for 3.1 s. We coast 1.55 m on PX4's velocity, then re-anchor.", "SITL"],
      ["Heading: an open question", "Fixed heading starved cuVSLAM, with the wall 0.4 m from the camera. Pathfly holds east because its 8 cm margins need it.", "SITL"],
    ];
    items.forEach(([h, d, k], i) => {
      const y = 1.2 + i * 1.27;
      waypoint(s, 5.45, y + 0.02, i + 1, 0.36);
      s.addText(h, T({ x: 5.95, y, w: 2.75, h: 0.3, fontSize: 15, bold: true, color: INK }));
      chip(s, 8.88, y + 0.04, k);
      s.addText(d, T({ x: 5.95, y: y + 0.33, w: 3.55, h: 0.82, fontSize: 13, color: SLATE }));
    });
    s.addNotes(
`(50 s) We built competing designs in parallel and measured them in the same simulator: seven navigation designs in all.
Chart: plain position setpoints crawl, because PX4 trails a moving pose by v over kp, and relaxing the governor made it cut corners. Flying the trajectory by velocity, with feed-forward 0.55 s ahead (PX4's velocity-loop lag) plus a pull back onto the path, took the window route from 59 s to 30 s with no touches. Our best line is 30.9–33.7 s at 2 m/s, SITL only.
The simulator is our own: it runs the organisers' kit code unmodified (the kit's 149 unit tests pass on it) with PX4's control loops ported, EKF2 behaviour and cuVSLAM tracking raycast against the STEP mesh. It found the dark-tunnel failure and the latency cliff before the drone did.
Dark tunnel: every follower that held still lost Offboard. We coast on PX4's own velocity for up to 4 s; cuVSLAM comes back after 1.3–3.3 s.
Heading: our two packages disagree, each for a measured reason. Turning costs 10–20 cm of drift; never turning made the camera stare at a wall and lose tracking; pathfly's planner margins assume a fixed rectangular footprint. That's an A/B we haven't flown yet.`);
  });

  // ===== 7. Generality =====
  slides.push(() => {
    pres.addSection({ title: "Generality" });
    const s = content("A new course is a new map file", "Generality");
    const tiles = [["12/12", "random layouts flown clean; hand route 0/12", "MODEL"], ["20/20", "boulder layouts planned clear; hand route 6/20", "SITL"], ["8/9", "never-seen layouts finished in PX4 SITL", "SITL"], ["17/17", "planted bugs caught by the tests", "MODEL"]];
    const tw = 2.1, tg = 0.2;
    tiles.forEach(([n, l, k], i) => {
      const x = 0.5 + i * (tw + tg);
      card(s, x, 1.2, tw, 1.45, MIST, `stat ${n}`);
      s.addText(n, T({ x: x + 0.15, y: 1.3, w: tw - 0.3, h: 0.55, fontFace: "Arial", fontSize: 28, bold: true, color: i === 0 ? ORANGE : INK }));
      s.addText(l, T({ x: x + 0.15, y: 1.86, w: tw - 0.3, h: 0.5, fontSize: 12, color: SLATE }));
      chip(s, x + 0.15, 2.33, k);
    });
    const col = (x, head, colr, items) => {
      s.addShape(pres.shapes.OVAL, { x, y: 2.95, w: 0.2, h: 0.2, fill: { color: colr }, line: { type: "none" } });
      s.addText(head, T({ x: x + 0.32, y: 2.92, w: 4.0, h: 0.3, fontSize: 15, bold: true, color: INK }));
      s.addText(items.map((t, i) => ({ text: t, options: { bullet: { indent: 14 }, breakLine: i < items.length - 1, paraSpaceAfter: 4 } })),
        T({ x, y: 3.3, w: 4.3, h: 1.75, fontSize: 13, color: SLATE }));
    };
    col(0.5, "Carries over unchanged", GO, [
      "No coordinate in code: a test greps our source for map numbers and wall names",
      "Planner, tunnel and window handling, every safety gate",
      "Clearances from the drone's size; course placed from its rest on the mark",
    ]);
    col(5.2, "Course-specific, and we say so", STOP, [
      "The map file: CAD plus tape-measured corrections",
      "The corridor goal added after the north-wall crash",
      "Pathfly's precomputed route and east heading: rerun plan_path.py",
    ]);
    s.addNotes(
`(50 s) Generality is 15 points, and the AI review will flag hard-coding, so here is the evidence and the honest list.
No route, waypoint or timing is in code. Each package has a test that greps its own source for any coordinate in the map and any wall name; a planted literal makes it fail.
On 21 maps through the generic package, including 12 random layouts, the planner flew all of them clean; the hand-structured waypoint version could plan none of the random ones. Pre-event, with boulders moved at random, 20 of 20 planned clear against 6 of 20 for a hand route, and the same planner finished 8 of 9 PX4 SITL flights on layouts it had never seen. Mutation testing: all 17 planted bugs were caught.
Course-specific, said before you find it: the map file itself, including tape corrections; a corridor goal we added after the crash to keep 0.61 m off the north wall; and jsun's pathfly package flies a precomputed route at a fixed heading, which on a new course means rerunning the planner on the new STEP file. Limit: the map is trusted; nothing senses walls in flight.`);
  });

  // ===== 8. Close (dark) =====
  slides.push(() => {
    pres.addSection({ title: "Close" });
    const s = pres.addSlide({ masterName: "DARK_CLOSE", sectionTitle: "Close" });
    s.addText("What reality taught us", { placeholder: "title" });
    const lessons = [
      ["The course isn't the CAD", "Measure it, and let the log locate the wall."],
      ["Earn speed", "Stop-and-go first, then one rung per clean landing."],
      ["Reject ideas with data", "Seven designs, one simulator running the real kit code."],
    ];
    lessons.forEach(([h, d], i) => {
      const y = 1.5 + i * 1.05;
      waypoint(s, 0.5, y, i + 1);
      s.addText(h, T({ x: 1.1, y: y - 0.03, w: 4.0, h: 0.32, fontSize: 17, bold: true, color: PAPER }));
      s.addText(d, T({ x: 1.1, y: y + 0.32, w: 4.0, h: 0.55, fontSize: 14, color: ICE }));
    });
    card(s, 5.6, 1.45, 3.9, 2.75, "1B2A40", "evidence card");
    s.addText("WHERE THE EVIDENCE STANDS", T({ x: 5.85, y: 1.62, w: 3.5, h: 0.25, fontFace: "Arial", fontSize: 10, bold: true, color: ORANGE, charSpacing: 2 }));
    chip(s, 5.85, 2.0, "FLOWN");
    s.addText("Square-test ladder, two stop-and-go course runs, the first scored session", T({ x: 5.85, y: 2.28, w: 3.45, h: 0.68, fontSize: 13, color: PAPER }));
    chip(s, 5.85, 3.05, "SITL");
    s.addText("Smooth trajectory (30.9–33.7 s), the return home, pathfly", T({ x: 5.85, y: 3.33, w: 3.45, h: 0.68, fontSize: 13, color: PAPER }));
    s.addText("Questions welcome: decisions, experiments, failures.", T({ x: 0.5, y: 4.82, w: 9.0, h: 0.35, fontSize: 14, italic: true, color: ICE }));
    s.addNotes(
`(40 s) Three lessons. The course isn't the CAD: the north wall was 24 inches inside the net, the window higher than published, the pad about a metre south, and our own logs found each one. Earn speed: everything flown so far is stop-and-go, and that's deliberate. Reject ideas with data: we killed ideas we liked, such as never turning, because the simulator showed them failing.
Where the evidence stands, honestly: on the real drone, the square-test ladder, two stop-and-go course runs and the first scored session. In simulation only: the smooth trajectory stack, the return home and the pathfly package.
Rehearse: What's hard-coded? (one map file, plus named constants for drone size, rates and margins.) Another course? (new map and launch mark; nothing else changes.) What would you do with more time? (fly the smooth line and the heading A/B on the drone; sense walls in flight with the lidar localizer we already built.)`);
    s.addText("Corpus: technical-corpus/TECHNICAL_CORPUS.md", T({ x: 0.5, y: 5.2, w: 5, h: 0.25, fontSize: 10, color: GRAY }));
  });

  slides.slice(0, LIMIT).forEach((f) => f());
  await pres.writeFile({ fileName: OUT });
  if (APPLY_THEME) await require(APPLY_THEME).applyTheme(OUT, THEME);
  console.log("wrote", OUT, "slides", Math.min(LIMIT, slides.length));
}
build().catch((e) => { console.error(e); process.exit(1); });
