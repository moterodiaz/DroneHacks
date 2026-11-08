// Team 08 technical presentation: a research talk told as a story.
// Goal → anchor (CAD) → shape → system → story → issues → results → decision → limits and future.
// Slides carry short recall bullets and one visual each; the detail is in the speaker notes.
// Rules: docs/presentation/DESIGN.md. Facts: technical-corpus/TECHNICAL_CORPUS.md.
//   NODE_PATH=<dir with pptxgenjs> APPLY_THEME=<pptx skill>/scripts/apply_theme.js node docs/presentation/build_deck.js
const path = require("path");
const pptxgen = require("pptxgenjs");

const HERE = __dirname;
const OUT = path.join(HERE, "DroneHacks-Team08-technical.pptx");

const THEME = {
  name: "Team 08 Report",
  headFontFace: "Arial",
  bodyFontFace: "Arial",
  colors: {
    dk1: "111111", lt1: "FFFFFF", dk2: "4D4D4D", lt2: "F2F2F2",
    accent1: "B42318", accent2: "1F3A5F", accent3: "FBE3DF", accent4: "BFBFBF",
    accent5: "808080", accent6: "7A4A2A", hlink: "1F3A5F", folHlink: "808080",
  },
};
const HEX = THEME.colors;

const pres = new pptxgen();
pres.layout = "LAYOUT_WIDE"; // 13.333 x 7.5 in
pres.author = "Team 08";
pres.title = "skybeta-11: from a CAD file to a flown course";
pres.theme = { headFontFace: THEME.headFontFace, bodyFontFace: THEME.bodyFontFace };
const C = pres.SchemeColor;
const INK = C.text1, GREY = C.text2, TINT = C.background2, BLUE = C.accent2, MUTE = C.accent5, RED = C.accent1;
const PINK = C.accent3, ROCK = C.accent6, HAIR = C.accent4;

const L = 0.6, R = 13.333 - 0.6, W = R - L;
pres.defineSlideMaster({
  title: "REPORT",
  background: { color: C.background1 },
  objects: [
    { placeholder: { options: { name: "title", type: "title", x: L, y: 0.55, w: W, h: 0.65, fontFace: "Arial", fontSize: 28, bold: true, color: INK, align: "left", valign: "middle", margin: 0 }, text: "" } },
    { text: { text: "Team 08  ·  skybeta-11  ·  DroneHacks 2026", options: { x: L, y: 7.1, w: 4, h: 0.22, fontFace: "Arial", fontSize: 10, color: MUTE, margin: 0 } } },
    { text: { text: "[F] flown on skybeta-11   [S] our SITL   [M] model, bench or test", options: { x: 4.4, y: 7.1, w: 5.5, h: 0.22, fontFace: "Arial", fontSize: 10, color: MUTE, align: "center", margin: 0 } } },
  ],
  slideNumber: { x: R - 0.6, y: 7.1, w: 0.6, h: 0.22, fontFace: "Arial", fontSize: 10, color: MUTE, align: "right" },
});

// ---------- helpers ----------
const T = (o) => Object.assign({ isTextBox: true, margin: 0, fontFace: "Arial", valign: "top", color: INK }, o);
const MONO = "Courier New";

function slide(label, title, section) {
  const s = pres.addSlide({ masterName: "REPORT", sectionTitle: section });
  s.addText(label, T({ x: L, y: 0.3, w: 8, h: 0.24, fontSize: 11, bold: true, color: MUTE, charSpacing: 1 }));
  s.addText(title, { placeholder: "title" });
  return s;
}
function head(s, x, y, w, txt) {
  s.addText(txt, T({ x, y, w, h: 0.36, fontSize: 17, bold: true }));
}
// Recall bullets. items: string, or [bold, rest] for a run-in label.
function block(s, x, y, w, h, items, size = 18) {
  const runs = [];
  items.forEach((it, i) => {
    const last = i === items.length - 1;
    const base = { breakLine: !last, paraSpaceAfter: 10 };
    if (typeof it === "string") runs.push({ text: it, options: Object.assign({ bullet: { indent: 16 }, color: GREY }, base) });
    else {
      runs.push({ text: it[0] + " ", options: { bullet: { indent: 16 }, bold: true, color: INK, paraSpaceAfter: 10 } });
      runs.push({ text: it[1], options: Object.assign({ color: GREY }, base) });
    }
  });
  s.addText(runs, T({ x, y, w, h, fontSize: size }));
}
// Booktabs table: bold header between black rules, grey hairlines between rows, no verticals.
function table(s, x, y, colW, rows, size = 15) {
  const none = { type: "none" };
  const data = rows.map((r, ri) => r.map((c) => {
    const cell = typeof c === "string" ? { text: c } : Array.isArray(c) ? { text: c } : c;
    const o = Object.assign({ fontFace: "Arial", fontSize: size, color: ri === 0 ? HEX.dk1 : HEX.dk2, bold: ri === 0, valign: "top", margin: [0.06, 0.08, 0.06, 0.08] }, cell.options || {});
    o.border = [ri === 0 ? { type: "solid", pt: 1, color: HEX.dk1 } : none, none,
      ri === 0 ? { type: "solid", pt: 1, color: HEX.dk1 } : { type: "solid", pt: 0.5, color: HEX.accent4 }, none];
    return { text: cell.text, options: o };
  }));
  s.addTable(data, { x, y, w: colW.reduce((a, v) => a + v, 0), colW });
}
const red = (t) => ({ text: t, options: { color: HEX.accent1 } });
const blue = (t) => ({ text: t, options: { color: HEX.accent2, bold: true } });
const bold = (t) => ({ text: t, options: { bold: true, color: HEX.dk1 } });

function box(s, x, y, w, h, title, sub, hi) {
  s.addShape(pres.shapes.RECTANGLE, { x, y, w, h, fill: { color: hi ? BLUE : C.background1 }, line: { color: hi ? BLUE : INK, width: 0.75 }, objectName: "box " + title });
  const runs = [{ text: title, options: { bold: true, breakLine: !!sub, fontSize: 13, color: hi ? C.background1 : INK } }];
  if (sub) runs.push({ text: sub, options: { fontSize: 11, color: hi ? C.background1 : GREY } });
  s.addText(runs, T({ x: x + 0.05, y, w: w - 0.1, h, align: "center", valign: "middle" }));
}
function line(s, x1, y1, x2, y2, o = {}) {
  s.addShape(pres.shapes.LINE, { x: Math.min(x1, x2), y: Math.min(y1, y2), w: Math.abs(x2 - x1), h: Math.abs(y2 - y1), flipH: x2 < x1, flipV: y2 < y1,
    line: Object.assign({ color: INK, width: 0.75 }, o) });
}
function arrow(s, x1, y1, x2, y2, label, lx, ly, lw, align = "center") {
  line(s, x1, y1, x2, y2, { endArrowType: "triangle" });
  if (label) s.addText(label, T({ x: lx, y: ly, w: lw, h: 0.3, fontFace: MONO, fontSize: 10.5, color: BLUE, align }));
}
function rect(s, x, y, w, h, fill, lineCol, name) {
  s.addShape(pres.shapes.RECTANGLE, { x, y, w, h, fill: { color: fill }, line: lineCol ? { color: lineCol, width: 0.75 } : { type: "none" }, objectName: name });
}
function oval(s, x, y, w, h, fill, lineCol, name, dash) {
  s.addShape(pres.shapes.OVAL, { x, y, w, h, fill: fill ? { color: fill } : { type: "none" }, line: lineCol ? { color: lineCol, width: 1, dashType: dash || "solid" } : { type: "none" }, objectName: name });
}
const lab = (s, txt, x, y, w, o = {}) => s.addText(txt, T(Object.assign({ x, y, w, h: 0.28, fontSize: 11, color: GREY }, o)));
// Horizontal bars drawn as shapes (Keynote drops pptxgenjs charts). rows: [label, value, text, color].
function bars(s, x, y, w, labW, max, rows, rowH = 0.5, size = 14) {
  const bw = w - labW - 1.7;
  rows.forEach(([lb, v, txt, col], i) => {
    const yy = y + i * rowH;
    s.addText(lb, T({ x, y: yy, w: labW, h: 0.34, fontSize: size, color: GREY, valign: "middle" }));
    rect(s, x + labW, yy + 0.06, (v / max) * bw, 0.24, col || GREY, null, "bar " + lb);
    s.addText(txt, T({ x: x + labW + (v / max) * bw + 0.08, y: yy, w: 1.65, h: 0.34, fontSize: size, color: col === RED ? RED : INK, bold: col === BLUE, valign: "middle" }));
  });
}

// ===================== Title =====================
{
  pres.addSection({ title: "Title" });
  const s = pres.addSlide({ masterName: "REPORT", sectionTitle: "Title" });
  s.addText("DRONEHACKS 2026  ·  TECHNICAL PRESENTATION  ·  8 NOV", T({ x: L, y: 0.3, w: 8, h: 0.24, fontSize: 11, bold: true, color: MUTE, charSpacing: 1 }));
  s.addText("From a CAD file to a flown course", T({ x: L, y: 1.3, w: 6.9, h: 1.6, fontSize: 36, bold: true }));
  s.addText("Team 08  ·  autonomous flight on skybeta-11, without GPS", T({ x: L, y: 3.0, w: 6.9, h: 0.45, fontSize: 20, color: GREY }));
  block(s, L, 3.95, 6.6, 2.6, [
    ["Input:", "the course CAD file"],
    ["Output:", "a route the drone flies alone"],
    ["Flown package:", "5391e34, stop-and-go"],
    ["SITL [S]:", "102 s, window ±8 cm, pad ±11 cm"],
  ]);
  s.addImage({ path: path.join(HERE, "assets", "route-5391e34.png"), x: R - 5.1, y: 1.05, w: 5.1, h: 5.1, altText: "Planned stop-and-go route at 5391e34 on the taped course map: launch, diagonal-wall gap, straight leg to the tunnel's north hold, tunnel, window, pad. Pale red is the no-fly zone for the drone centre." });
  s.addText("Planned route at 5391e34 on the taped map. Red: no-fly zone for the drone centre.",
    T({ x: R - 5.1, y: 6.25, w: 5.1, h: 0.5, fontSize: 11, color: GREY }));
  s.addNotes(
`The story in one line: we start from the course CAD, anchor everything to it, treat the drone as a shape that must pass through the free space, learn from each flight, and decide what to fly from the evidence.
Format: 5 to 7 min of slides, then 5 to 6 min of questions. The judges probe decisions, experiments, testing and challenges.
Points: 25 for the presentation, 15 for generality. An AI review of the repository flags hard-coding.
Drone: skybeta-11. Pixhawk 6C with PX4 1.16, Jetson Orin Nano, OAK-D S2 stereo camera, downward rangefinder.
Flown package: dronehacks_course_follower, branch core/flight-iteration at 5391e34.
SITL on the taped course at 5391e34: stop-and-go 102 s, smooth 0.5 m/s 76 s, 0 contacts. Window crossed within 8 cm of centre. Landed within 11 cm of the pad centre.

Questions to rehearse:
How do you know where you are? cuVSLAM from stereo, anchored by the 2 s rest on the launch mark, checked against PX4 before take-off.
The dark tunnel? Line up outside, fly straight, no turns inside. The jump guard lands the drone if the pose leaps.
Why stop-and-go? Each in-place turn costs 10 to 20 cm of drift. The hold lets it settle. Speed comes one rung at a time.
Why not never yaw? In SITL a fixed heading made the camera stare at the north wall and lose tracking. On the real tunnel run, the heading drift was under 1 degree through four turns and about 2.5 degrees by the tunnel.
What is hard-coded? The course is the map file. Code constants are the drone size, rates and margins.
Another course? New CAD, new map file, new launch mark. The planner, the tunnel and window handling and every safety check carry over.

Do not claim: a 60/40 split (it is 65/35); that cuVSLAM uses the IMU (the kit runs it stereo-only); that the tunnel run was a collision (it was a kill); course completion on the real drone before it happens.`);
}

// ===================== 1. Goal =====================
{
  pres.addSection({ title: "Goal" });
  const s = slide("1  GOAL", "Fly the course alone, from its CAD file", "Goal");
  head(s, L, 1.5, 6.6, "Task");
  block(s, L, 1.95, 6.6, 2.4, [
    "We get a CAD file of the course",
    "Gap → dark tunnel → quadrant 4 → window → pad",
    "One start command. No GPS, no pilot",
    "3 min limit. Each contact adds 5 s",
  ]);
  head(s, L, 4.35, 6.6, "Constraints");
  block(s, L, 4.8, 6.6, 2.2, [
    "Corridor 0.93 m. Props reach 0.28 m",
    "Only write path: the kit flight_interface",
    "A stick input ends autonomy",
  ]);
  const tx = 7.9;
  head(s, tx, 1.5, R - tx, "Points");
  table(s, tx, 1.95, [3.0, 1.83], [
    ["Checkpoint", "Points"],
    ["Exit quadrant 1", "6"], ["Tunnel", "10"], ["Enter quadrant 4", "7"], ["Window", "+4"],
    ["Pad: blue / ring / centre", "3 / 5 / 7"], ["Return home", "× 1.25"],
  ]);
  s.addText([{ text: "Run 65 % · judges 35 %", options: { bold: true, color: INK, breakLine: true } },
    { text: "Full run 65 → with window 73.7 → with return 92.1", options: { color: GREY } }],
  T({ x: tx, y: 5.6, w: R - tx, h: 0.9, fontSize: 15 }));
  s.addNotes(
`The goal: one autonomous run per start command, from the course CAD file. There is no GPS and no pilot input.
Sequence: exit quadrant 1 through the gap past the diagonal wall, the dark tunnel, enter quadrant 4, the window, land on the pad. The return to the launch mark multiplies the run points by 1.25. The best run counts.
Score = 65/30 x (checkpoints + window) x 1.25 + 35 x judges/40. A checkpoint reached over a wall gives 15 percent. The window is worth more than the gap between a 3- and a 7-point landing.
Platform: 7 in quad, 1.25 kg. Jetson Orin Nano 8 GB, ROS 2 Humble. Pixhawk 6C, PX4 1.16. OAK-D S2 stereo, 75 mm baseline, 30 fps. VL53L1X rangefinder, 0 to 4 m, down. cuVSLAM stereo-only, 30 Hz, 62 to 69 ms latency.
Course as measured: net 6.10 m square, walls 1.727 m high, corridors 0.93 to 1.07 m. Error budget for the drone centre: plus or minus 0.18 m.
Interface: pose altitude 0.3 to 3.0 m, twist at most 1.5 m/s, resend 10 Hz, timeout 1 s. Auto-Offboard occurs one time per arming. A stick input ends Offboard for the remainder of the flight. The kill switch always operates.
Rules: no writes to /fmu/in, only allowlisted PX4 parameters, no Acro, no course geometry in code.`);
}

// ===================== 2. Anchor =====================
{
  pres.addSection({ title: "Approach" });
  const s = slide("2  APPROACH: ANCHOR ON THE CAD", "The CAD is the anchor. The launch mark ties it to the floor", "Approach");
  const steps = [
    ["Course CAD", "STEP file"],
    ["Map file", "walls, rocks, goals"],
    ["Tape check", "correct the map"],
    ["Launch mark", "2 s rest = frame"],
    ["Route", "A*, straight legs"],
  ];
  const g = 0.45, bw = (W - 4 * g) / 5, by = 1.6, bh = 0.95;
  steps.forEach(([t, sub], i) => {
    const x = L + i * (bw + g);
    box(s, x, by, bw, bh, t, sub, i === 0);
    if (i < 4) arrow(s, x + bw, by + bh / 2, x + bw + g, by + bh / 2);
  });
  const cw = (W - 0.6) / 3;
  const xs = [0, 1, 2].map((i) => L + i * (cw + 0.3));
  head(s, xs[0], 3.0, cw, "Why the CAD");
  block(s, xs[0], 3.45, cw, 3.4, [
    "One file holds the whole course",
    "No coordinates in code: a test checks",
    "New course = new CAD file",
  ], 17);
  head(s, xs[1], 3.0, cw, "Where the CAD was wrong");
  block(s, xs[1], 3.45, cw, 3.4, [
    "North wall 0.61 m inside the net",
    "Window opening 36–61 in",
    "Our flight log found the wall",
  ], 17);
  head(s, xs[2], 3.0, cw, "Generality test");
  table(s, xs[2], 3.5, [2.0, 1.0, 1.0], [
    ["Layouts", "Planner", "Hand"],
    ["Random [M]", [blue("12 / 12")], "0 / 12"],
    ["Walls ± 5 cm [M]", [blue("6 / 6")], "0 / 6"],
    ["Rocks moved [S]", [blue("20 / 20")], "6 / 20"],
    ["Unseen, PX4 [S]", "8 / 9", "—"],
  ], 14);
  s.addNotes(
`Everything starts from the CAD. step_to_map.py reads the STEP AP214 file and writes one map file: walls, boulders, goals and goal order. The parser stops if a part is ambiguous.
The built course differs from the CAD. Run 1 hit the north wall; our log put the hit 24 in inside the net, and the tape agreed. Localization was right to inches; the map was wrong. Tape corrections go in the map file, not in code (rule 6.4): north wall, diagonal wall, window height 36 to 61 in, rocks and pad.
The launch mark anchors the map to the floor. The drone rests 2 s on the mark; the mean pose gives the course-to-odometry transform within 2 cm and 1 degree. Nobody types a transform.
The route comes from the map: A* and straight legs through the openings.
Generality: the planner holds no layout knowledge. It flew all 12 random layouts; the hand route planned none of them. Also: published map 1 of 1, rotated and mirrored 2 of 2, 17 of 17 planted bugs caught. A test searches our source for map coordinates and wall names. A wall across the window makes A* find Gate B; no code names Gate B. Constants are the drone size, rates and margins, each with a unit.
Course-specific, say it first: the map file with the 8 Nov tape corrections; the flight height 1.23 m from the taped window opening; the launch mark position and heading, by tape. The corridor goal added after the crash was removed in 5391e34.
New course procedure: get the STEP file, run step_to_map.py, tape-measure and correct the map, tape the launch mark, run the dry run and our SITL, fly stop-and-go at 0.3 m/s.`);
}

// ===================== 3. Shape =====================
{
  const s = slide("3  APPROACH: THE DRONE AS A SHAPE", "Cut the drone's shape through the free space", "Approach");
  const pw = (W - 0.6) / 3;
  const px = [0, 1, 2].map((i) => L + i * (pw + 0.3));
  const top = 1.5, dTop = 2.0, dH = 2.45; // drawing area
  const sc = 2.1; // inches per metre for the plan views

  // Panel 1: point with a 0.32 m circle in a 0.93 m corridor.
  // Panel 2: exact 0.50 x 0.40 m rectangle at a fixed heading; the dashed circle is its sweep if it yaws.
  [0, 1].forEach((k) => {
    const x0 = px[k], cx = x0 + pw / 2, cwid = 0.93 * sc;
    const wl = cx - cwid / 2, wr = cx + cwid / 2;
    rect(s, wl - 0.16, dTop, 0.16, dH, INK, null, "wall left " + k);
    rect(s, wr, dTop, 0.16, dH, INK, null, "wall right " + k);
    if (k === 0) {
      const inf = 0.32 * sc;
      rect(s, wl, dTop, inf, dH, PINK, null, "no-fly left");
      rect(s, wr - inf, dTop, inf, dH, PINK, null, "no-fly right");
      line(s, cx, dTop, cx, dTop + dH, { color: BLUE, width: 1.5, dashType: "dash" });
      oval(s, cx - inf, dTop + dH / 2 - inf, 2 * inf, 2 * inf, null, BLUE, "circle 0.32 m");
      oval(s, cx - 0.05, dTop + dH / 2 - 0.05, 0.1, 0.1, BLUE, null, "centre");
      lab(s, "0.32 m", cx + 0.06, dTop + dH / 2 - 0.3, 0.8, { color: BLUE });
    } else {
      const rw = 0.40 * sc, rh = 0.50 * sc, r = 0.32 * sc;
      oval(s, cx - r, dTop + dH / 2 - r, 2 * r, 2 * r, null, MUTE, "yaw sweep", "dash");
      rect(s, cx - rw / 2, dTop + dH / 2 - rh / 2, rw, rh, C.background1, BLUE, "footprint");
      s.addShape(pres.shapes.RECTANGLE, { x: cx - rw / 2, y: dTop + dH / 2 - rh / 2, w: rw, h: rh, fill: { color: BLUE, transparency: 80 }, line: { color: BLUE, width: 1.5 }, objectName: "footprint fill" });
      lab(s, "0.50 × 0.40 m", cx - 0.7, dTop + dH / 2 + r + 0.04, 1.4, { color: BLUE, align: "center" });
    }
    lab(s, "0.93 m corridor", cx - 0.9, dTop + dH + 0.05, 1.8, { align: "center" });
  });
  // Panel 3: side view at the window.
  {
    const x0 = px[2], fy = dTop + dH, vs = 1.35; // inches per metre vertical
    const Y = (m) => fy - m * vs;
    line(s, x0, fy, x0 + pw, fy, { width: 1.5 });
    const wx = x0 + pw - 0.9;
    rect(s, wx, Y(0.91), 0.16, 0.91 * vs, INK, null, "wall below window");
    rect(s, wx, Y(1.727), 0.16, (1.727 - 1.55) * vs, INK, null, "wall above window");
    lab(s, "window 0.91–1.55 m", wx - 0.55, Y(1.727) - 0.32, 1.9, { align: "center" });
    const dx = x0 + 0.5, dy = Y(1.23);
    line(s, dx + 0.45, dy, wx + 0.8, dy, { color: BLUE, width: 1.5, dashType: "dash" });
    rect(s, dx, dy - 0.07, 0.45, 0.14, BLUE, null, "drone side");
    lab(s, "fly 1.23 m", dx + 0.6, dy - 0.32, 1.2, { color: BLUE });
    // rangefinder cone onto a rock
    const cx = dx + 0.225, half = (1.23 * Math.tan(13.5 * Math.PI / 180)) * vs;
    line(s, cx, dy + 0.07, cx - half, fy, { color: RED, dashType: "dash" });
    line(s, cx, dy + 0.07, cx + half, fy, { color: RED, dashType: "dash" });
    oval(s, cx + half - 0.1, fy - 0.2, 0.55, 0.4, ROCK, null, "rock");
    rect(s, cx + half - 0.15, fy, 0.7, 0.22, C.background1, null, "rock mask");
    line(s, x0, fy, x0 + pw, fy, { width: 1.5 });
    lab(s, "rangefinder cone 27°", x0, fy + 0.05, 2.0, { color: RED });
  }
  const titles = ["2D point + circle", "2D exact footprint", "3D: height"];
  const subs = ["course follower [F]", "pathfly [M]", "both"];
  const items = [
    ["Safe at any heading", "A* on a 5 cm grid", "Straight legs through openings"],
    ["Free space read from the CAD", "Margins to 8 cm", "Nose fixed east: a yaw sweeps the circle"],
    ["Window centre sets 1.23 m", "Rocks grow by the rangefinder cone", "pathfly returns over walls at 2.0 m"],
  ];
  [0, 1, 2].forEach((k) => {
    s.addText([{ text: titles[k] + "  ", options: { bold: true, color: INK } }, { text: subs[k], options: { color: MUTE, fontSize: 13 } }],
      T({ x: px[k], y: top, w: pw, h: 0.36, fontSize: 17 }));
    block(s, px[k], 4.95, pw, 2.0, items[k], 15);
  });
  s.addNotes(
`The free space of the course is an irregular shape. The drone is a shape too. Planning means cutting the drone's shape through the free space. Our journey went from a point to a 2D shape to 3D.
Panel 1, the course follower (flown): the drone is a point with a 0.32 m circle, the 0.28 m prop reach plus 4 cm. The circle is safe at any heading, so the drone can turn anywhere. A* on a 5 cm grid, 8-connected, octile heuristic; cost exp(-3 x margin) beyond the lethal zone (Nav2 inflation). Openings are straight legs. Tunnel: hold 0.6 m before the north mouth and 0.75 m past the south mouth, no turn inside. Window on its normal, plus or minus 0.7 m. In 39 SITL flights the window was the closest pass in 21 before this rule. A straight segment replaces grid steps if it adds no cost; the end check is exact segment-to-segment distance.
Panel 2, pathfly (navigation/pp-path-flight, jsun4231): plan_path.py reads the arena straight from the STEP B-rep. The floor face's outer loop is the free space; its two circular inner loops are the start and goal markers. It computes the exact signed distance from the true 0.50 x 0.40 m rectangle to the boundary. A* on a 2 cm grid, 16-connected, then an elastic band and Douglas-Peucker. Path 12.27 m. Tightest margins 0.079 to 0.082 m under the north wall. These margins hold only at one heading, so the nose stays east. Any yaw sweeps the rectangle through a 0.32 m circle. 3 degrees of placement error moves the goal end about 0.17 m.
Panel 3, height: the window opening is 36 to 61 in (0.91 to 1.55 m); we fly at its centre, 1.23 m, a map value. The rangefinder's 27 degree cone sees a rock top as the floor rising 0.53 m, and PX4 climbs. So rocks grow by the cone footprint, (h + 0.20) tan 13.5 deg + 0.10 m. pathfly climbs 0.2 to 1.25 m over the first 1.5 m of path, and its return flies straight back over the walls at 2.0 m, without yawing.
The heading tension: the course follower turns to face along the route because a fixed heading starved cuVSLAM by the north wall; pathfly holds east because its margins need it. Turning costs drift; not turning can cost tracking. That is an experiment, not a preference.`);
}

// ===================== 4. System =====================
{
  const s = slide("4  APPROACH: SYSTEM", "We send setpoints. PX4 closes the loops", "Approach");
  const lx = L, lw = 8.1;
  rect(s, lx, 1.5, lw, 2.35, TINT, null, "jetson zone");
  rect(s, lx, 4.35, lw, 1.9, TINT, null, "pixhawk zone");
  s.addText("JETSON ORIN NANO · ROS 2", T({ x: lx + 0.12, y: 1.56, w: 5, h: 0.22, fontSize: 10, bold: true, color: MUTE, charSpacing: 1 }));
  s.addText("PIXHAWK 6C · PX4 1.16", T({ x: lx + lw - 5.12, y: 4.41, w: 5, align: "right", h: 0.22, fontSize: 10, bold: true, color: MUTE, charSpacing: 1 }));
  const by = 2.15, bh = 0.85;
  box(s, 0.75, by, 1.4, bh, "OAK-D S2", "stereo camera");
  box(s, 2.7, by, 1.45, bh, "cuVSLAM", "odometry");
  box(s, 4.7, by, 1.6, bh, "Our node", "map, route, guards");
  box(s, 6.85, by, 1.7, bh, "flight_interface", "kit, 10 Hz");
  box(s, 2.7, 3.25, 1.45, 0.45, "odom bridge", null);
  arrow(s, 2.15, by + 0.42, 2.7, by + 0.42, "30 Hz", 2.0, by - 0.32, 0.85);
  arrow(s, 4.15, by + 0.42, 4.7, by + 0.42, "30 Hz", 4.0, by - 0.32, 0.85);
  arrow(s, 6.3, by + 0.42, 6.85, by + 0.42, "setpoint", 6.1, by - 0.32, 1.0);
  arrow(s, 3.42, by + bh, 3.42, 3.25);
  const py = 4.95, ph = 0.9;
  box(s, 0.75, py, 1.6, ph, "EKF2", "vision + IMU");
  box(s, 2.8, py, 1.6, ph, "Position", "P loop");
  box(s, 4.85, py, 1.6, ph, "Velocity", "PI loop");
  box(s, 6.9, py, 1.65, ph, "Attitude", "→ motors");
  arrow(s, 2.35, py + 0.45, 2.8, py + 0.45);
  arrow(s, 4.4, py + 0.45, 4.85, py + 0.45);
  arrow(s, 6.45, py + 0.45, 6.9, py + 0.45);
  arrow(s, 3.42, 3.7, 1.55, py, "vision pose", 0.75, 3.98, 1.5, "left");
  arrow(s, 7.7, by + bh, 3.6, py, "trajectory setpoint", 5.6, 3.98, 2.4, "left");

  const rx = 9.15, rw = R - rx;
  head(s, rx, 1.5, rw, "Flight states");
  block(s, rx, 1.95, rw, 2.2, [
    "Climb to 1.23 m",
    "Fly the route",
    "Descend; PX4 disarms",
    "Pose jump → descend",
  ], 17);
  head(s, rx, 4.2, rw, "Guards fail closed");
  block(s, rx, 4.65, rw, 2.3, [
    "Rest 2 s on the mark first",
    "Stale pose → zero velocity",
    "Stick or kill always wins",
  ], 17);
  s.addNotes(
`The camera sends stereo images to cuVSLAM. cuVSLAM sends odometry at 30 Hz. The bridge sends the same pose to EKF2 as vision. EKF2 also uses the rangefinder and the IMU.
Our node reads the map and the pose at 50 Hz. It sends one pose or one twist. The kit's flight_interface clamps it and resends it at 10 Hz.
PX4 flies a cascade: position P (MPC_XY_P 0.95), velocity PI (P 1.8, I 0.4), then attitude and rate. We rejected a direct-to-motor neural policy (MonoRace, SkyDreamer): the rules forbid writes to PX4 topics, the link latency is 68.7 ms, and there is no RPM feedback.
States: WAIT, CLIMB to 1.23 m, FLY, DESCEND at 0.5 m/s until PX4 detects touchdown and disarms, RETURN at 2.3 m. LOST: a pose jump over 0.3 m at over 3 m/s; descend straight down. STOPPED: Offboard ended; send no more setpoints.
Guards: before Offboard, rest 2 s within 2 cm and 1 degree, PX4 and camera agree within 0.3 m, else send nothing. Abort if Offboard starts more than 0.5 m from the mark. An invalid map stops the node before it creates a publisher. In flight the line to the setpoint must be clear. Off path by 0.35 to 0.5 m: hold. A pose setpoint never expires, so a stale state sends zero twists. Landing continues down on a stale pose.
Invariants: no Acro, no /fmu/in writes, no change to the organisers' PX4 safety settings or the flight interface. One PX4 parameter change only: COM_DISARM_LAND 5 s, organiser approval (#40).
Verification: our own SITL, 26k lines, no ROS or GPU, runs the kit code unmodified; it ports PX4's control loops and EKF2 delayed vision and models cuVSLAM on the STEP mesh. About 800 tests; 17 of 17 planted bugs caught. One bag and one report per flight. Independent AI review before each merge.`);
}

// ===================== 5. Story =====================
{
  pres.addSection({ title: "Journey" });
  const s = slide("5  JOURNEY: THE STORY", "One primitive per flight, then the course", "Journey");
  const steps = [
    ["Fri night", "Model", "65 rules, our own SITL", null],
    ["Sat", "Primitives", "Climb, box, land, relaunch, circle", "blue"],
    ["Sun 00:13", "Course run 1", "Hit a wall not in the map", "red"],
    ["Sun 00:21", "Tunnel run", "Early turn. Kill, no contact", "red"],
    ["Sun AM", "Scored session", "Sticks ended every run", "red"],
    ["Sun AM", "Second planner", "pathfly: exact footprint", null],
    ["Sun 12:01", "Straight leg", "5391e34: 102 s, 0 contacts [S]", "blue"],
  ];
  const g = 0.15, cw = (W - 6 * g) / 7, ly = 1.95;
  line(s, L, ly, R, ly, { width: 1 });
  steps.forEach(([when, name, result, tone], i) => {
    const x = L + i * (cw + g);
    oval(s, x, ly - 0.08, 0.16, 0.16, i === 6 ? BLUE : INK, null, "dot " + name);
    s.addText(when, T({ x, y: 1.55, w: cw, h: 0.28, fontSize: 11, color: MUTE }));
    s.addText(name, T({ x, y: 2.2, w: cw, h: 0.35, fontSize: 15, bold: true }));
    s.addText([tone === "red" ? red(result) : tone === "blue" ? blue(result) : { text: result, options: { color: HEX.dk2 } }],
      T({ x, y: 2.65, w: cw, h: 1.0, fontSize: 13 }));
  });
  head(s, L, 4.1, 6, "Method");
  block(s, L, 4.55, 6, 2.4, [
    "Prove one primitive per flight",
    "Find the cause in the logs first",
    "Add code only for a measured failure",
  ]);
  head(s, 7.1, 4.1, R - 7.1, "Before the event, in SITL");
  block(s, 7.1, 4.55, R - 7.1, 2.4, [
    "Corners set the time, not top speed",
    "+50 ms latency doubled the error",
    "Map 10 cm off: 5 of 5 runs touched",
  ]);
  s.addNotes(
`This is the journey. We did not fly the course first. We proved one primitive per flight: hold, move, turn, land, disarm, relaunch, curve, then the course.
Fri night: 65 rules, 4 contracts, pre-event Gazebo SITL, then our own SITL. MonoRace and SkyDreamer research set the decision to send setpoints.
Sat: the first flight flipped (motor configuration overwritten). Then 003042 climbed to 1.33 m for 0.73 m. 013018 flew a 0.5 m box with four yaws. 035510 flew square, land, relaunch, 1 m circle, land, disarm at 107 s.
Sun 00:13, flight 081345: hit the north wall. Sun 00:21, flight 082135: climbed to 0.80 m, flew the route, turned early at the tunnel's south mouth; pilot kill at 79.57 s; no contact.
Sun morning, the first scored session: every flight ended when a stick moved. A north-wall crash came after pilot input at 66.2 s.
Sun morning: a second route package, dronehacks_pathfly, with the footprint-exact planner.
Sun 12:01, 5391e34: corridor goal removed; one straight leg from the gap to the tunnel's north hold. SITL stop-and-go 109 to 102 s.
Pre-event SITL: 1.0, 1.5 and 2.0 m/s all took 27 to 31 s, so corners set the time. +50 ms latency doubled the error. A course 10 cm off its map touched in 5 of 5 runs, so we survey to 5 cm. Facing a wall at 0.3 m withheld 390 of 543 frames.`);
}

// ===================== 6. Issues =====================
{
  const s = slide("6  JOURNEY: ISSUES", "Each failure had one root cause in the logs", "Journey");
  table(s, L, 1.5, [2.9, 5.2, 4.03], [
    ["Issue", "Root cause", "What we changed"],
    [[red("Flew 1.33 m for 0.73 m")], "PX4 and camera measure height from different zeros", "Add the PX4 − camera offset"],
    [[red("10–18 cm drift per yaw")], "Body-fixed push; PX4 holds it in world axes", "Slow yaw, settle 1 s"],
    [[red("Hit a wall")], "The built course differs from the CAD", "Tape the course into the map"],
    [[red("Early tunnel turn")], "Turn began at the setpoint; drone 0.25 m behind", "Turns wait for the drone"],
    [[red("Runs ended early")], "A stick input ends Offboard", "Hands off in Offboard"],
    [[red("Pose jumped metres")], "Tracker after a hand landing", "LOST state; restart the stack"],
    [[bold("Heading: open")], "Turning drifts; a fixed heading blinds the camera", "Face along the route, for now"],
  ], 15);
  s.addNotes(
`Each row is a failure, the root cause we found in the bag and the PX4 ULog, and the change.
Height: PX4 height comes from the rangefinder, zero at power-up. cuVSLAM is zero at stack start. The interface sends odometry z to PX4 unchanged. On the floor PX4 read minus 0.72 m. Fix: z_sp = z_odom + (z_PX4 - z_odom), filtered with tau 1 s.
Yaw drift: a 0.10 m/s^2 push is fixed in the body frame. PX4's velocity integrator cancels it in world axes. After a yaw dpsi, the residual is 2 sin(dpsi/2) x push. The model predicts each corner's drift direction within 9 to 18 degrees. Estimate vs odometry median 1.3 cm.
Wall: run 1 hit the north wall 24 in inside the net. The log located it to inches; the tape agreed. Localization was right; the map was wrong.
Tunnel turn: flight 082135. Yaw began when the setpoint reached the corner while the drone trailed 0.25 m. Pilot kill at 79.57 s, motor current 0. No contact. Fix: turns wait for the drone; hold 0.75 m past the south mouth.
Scored session: every run ended on stick input. The crash came after full throttle and yaw at 66.2 s; the drone was 3 to 8 cm from its setpoint until then.
Pose jump: after a manual landing the tracker jumped metres. New LOST phase; restart the stack after any handling.
Heading: holding north in SITL made the camera stare at the north wall from 0.4 m and lose tracking, so the course follower faces along the route (1ecee21). On the real tunnel run the heading drift was under 1 degree over four turns and about 2.5 degrees by the tunnel, against PX4's gyro. pathfly holds east for its 8 cm margins. A flown A/B test decides this.
Also: PX4 disarmed 0.8 s into a relaunch; COM_DISARM_LAND 5 s with organiser approval (#40). Land detection took 4 s; it needs a commanded descent of at least 0.275 m/s.`);
}

// ===================== 7. Results =====================
{
  pres.addSection({ title: "Results" });
  const s = slide("7  RESULTS", "Tracking held. The map and the sticks failed", "Results");
  const lw = 6.2;
  head(s, L, 1.5, lw, "Flown on skybeta-11 [F]");
  table(s, L, 1.95, [1.15, 5.05], [
    ["Flight", "Result"],
    ["003042", [red("Climbed to 1.33 m, not 0.73 m")]],
    ["013018", "Box: 10–18 cm drift per yaw"],
    ["035510", [blue("All primitives passed")]],
    ["081345", [red("Hit a wall not in the map")]],
    ["082135", [red("Early tunnel turn. Kill, no contact")]],
    ["Scored", [red("Sticks ended runs. Error 3–8 cm before")]],
  ], 15);
  const rx = 7.3, rw = R - rx;
  head(s, rx, 1.5, rw, "SITL, launch to pad [S]");
  bars(s, rx, 2.05, rw, 2.25, 102, [
    ["Stop-and-go", 102, "102 s"],
    ["Smooth 0.5 m/s", 76, "76 s"],
    ["Trajectory, poses", 59, "59 s"],
    ["Trajectory, twist", 30, "30 s", BLUE],
  ], 0.5, 14);
  block(s, rx, 4.35, rw, 2.6, [
    "Window ±8 cm, pad ±11 cm",
    "Return home: 5 of 5 within 23 cm",
    "Dark tunnel: 3.1 s blind, no failsafe",
  ], 17);
  s.addNotes(
`REAL FLIGHTS TODAY: add the results here before the talk. Do not claim completion before it happens.
Only stop-and-go has flown the course. The smooth stacks have only flown in simulation.
003042: height origins differed. 013018: height 0.71 to 0.75 m for a 0.73 m target; PX4 estimate vs odometry median 1.3 cm, 95th percentile 3.3 cm, heading 0.6 deg; hold without a yaw within 7 cm. 035510: square, land, relaunch, circle, land, disarm at 107 s. 081345: contact with a wall not in the map; pilot kill. 082135: turned early at the tunnel south mouth; kill at 79.57 s; no contact. Scored session: stick input ended each run; crash after full throttle and yaw at 66.2 s.
SITL bars: stop-and-go and smooth 0.5 m/s on the taped map at 5391e34 (102 s, 76 s). Trajectory stack on the 7 Nov layout: plain poses 59 s, twist 1.0 m/s 30 s. Velocity feed-forward took 59 s to 30 s; plain poses lag by v over kp. Window line at 2 m/s: 30.9 to 33.7 s, no contact. Gate B at 0.8 to 1.0 m/s: 38 to 39 s with 1 to 2 contacts.
Window crossed within 8 cm of centre, landed within 11 cm of the pad centre (5391e34, taped map). Return home 5 of 5 within 23 cm of the mark (published course). Dark tunnel: tracking lost for 3.1 to 3.2 s, coast 1.55 m in Offboard, no failsafe.
Planner bench: waypoint steps 30.0 s, smoothed one speed 65.6 s, speed profile with lead 23.5 s; the speed profile saves 22 percent.
Pre-event SITL, 15 layouts: tunnel blinded, 12 of 14 cleared, error 2.4 to 7.6 cm rms. With +50 ms latency: 19 of 20 cleared.`);
}

// ===================== 8. Decision =====================
{
  pres.addSection({ title: "Decision" });
  const s = slide("8  FINAL DECISION", "Fly stop-and-go from the map. Then climb the ladder", "Decision");
  const chosen = { options: { color: HEX.lt1, bold: true, fill: { color: HEX.accent2 } } };
  table(s, L, 1.5, [2.6, 3.2, 3.2, 3.13], [
    ["", { text: "Stop-and-go  ✓ chosen", options: chosen.options }, "Trajectory stack", "pathfly"],
    [[bold("Flown on the drone")], [blue("2 course runs [F]")], "No", "No"],
    [[bold("SITL time")], "102 s", [blue("30 s")], "≈ 60 s mission [M]"],
    [[bold("Shape")], "0.32 m circle", "0.32 m circle", "Exact rectangle, 8 cm"],
    [[bold("Heading")], "Along the route", "Look ahead", "Fixed east"],
    [[bold("New course")], "New map file", "New map file", "Run the planner again"],
  ], 15);
  head(s, L, 4.3, 6, "Why");
  block(s, L, 4.75, 6, 1.8, [
    "Only follower with flown course evidence",
    "Tracking 3–8 cm until the sticks moved",
  ]);
  head(s, 7.1, 4.3, R - 7.1, "Declared course-specific");
  block(s, 7.1, 4.75, R - 7.1, 1.8, [
    "Map file with tape corrections",
    "Flight height 1.23 m and launch mark",
  ]);
  s.addNotes(
`The decision: fly the course follower, stop-and-go at 0.3 m/s, at 5391e34, facing along the route. It is ladder step 0.
Why: it is the only follower that has flown the course on skybeta-11. Its tracking held 3 to 8 cm until stick input. Its failures were the map and the sticks, and both are fixed.
Trajectory stack: 30 s in SITL with velocity feed-forward, but not flown. It climbs in after clean stop-and-go runs.
pathfly: footprint-exact margins down to 8 cm and a fixed east heading. About 60 s arming to disarm in its own model. Staged bring-up is ready; not flown. On a new course it needs plan_path.py again on the new STEP (one offline step, no code change). Its frame is the arena corner, anchored when Offboard begins.
Course-specific items, declared: the map file with the 8 Nov tape corrections; the flight height 1.23 m, the middle of the taped window opening, a map value; the launch mark position and heading, by tape; pathfly's precomputed route and heading.
Confirm this slide against the team's call on the day before presenting.`);
}

// ===================== 9. Limitations and future work =====================
{
  pres.addSection({ title: "Future" });
  const s = slide("9  LIMITATIONS AND FUTURE DIRECTIONS", "The map is trusted. Only stop-and-go has flown", "Future");
  const cw = (W - 0.5) / 2, x2 = L + cw + 0.5;
  head(s, L, 1.5, cw, "Limitations");
  block(s, L, 1.95, cw, 3.0, [
    ["Map trusted:", "no wall sensing in flight"],
    ["Map error:", "5 cm wall shift, 61 → 81 s [S]"],
    ["Flown:", "stop-and-go only"],
    ["Heading:", "turn or hold, still open"],
  ]);
  head(s, x2, 1.5, cw, "Future directions");
  block(s, x2, 1.95, cw, 3.0, [
    ["Lidar localization:", "see walls, 0.6 cm bench"],
    ["Trajectory stack:", "30 s vs 102 s in SITL"],
    ["3D footprint:", "shape and heading in one plan"],
    ["Heading A/B:", "flown, PX4 gyro as reference"],
  ]);
  head(s, L, 5.2, W, "Speed ladder: one step per clean run");
  const ladder = [["0.3 m/s", "stop-and-go"], ["0.5 m/s", ""], ["0.7 m/s", ""], ["0.85 m/s", ""], ["1.0 m/s", ""], ["1.0 m/s", "+ return × 1.25"]];
  const lg = 0.12, lcw = (W - 5 * lg) / 6;
  ladder.forEach(([v, sub], i) => {
    const x = L + i * (lcw + lg);
    const now = i === 0;
    s.addShape(pres.shapes.RECTANGLE, { x, y: 5.7, w: lcw, h: 0.85, fill: { color: now ? BLUE : C.background1 }, line: { color: now ? BLUE : INK, width: 0.75 }, objectName: "ladder " + i });
    s.addText([{ text: v, options: { bold: true, fontSize: 16, color: now ? C.background1 : INK, breakLine: true } },
      { text: now ? "flown [F]" : (sub || "next"), options: { fontSize: 12, color: now ? C.background1 : GREY } }],
    T({ x, y: 5.7, w: lcw, h: 0.85, align: "center", valign: "middle" }));
  });
  s.addNotes(
`Limitations.
The map is trusted. The planner believes the map file. No sensor finds walls in flight. Run 1 hit a wall that was not in the map.
A 5 cm wall shift turned a corner tight: 61 to 81 s in SITL. A course 10 cm off its map file touched in 5 of 5 SITL runs.
Only stop-and-go has flown the course. The smooth follower, trajectory stack, return home, pathfly and yaw_comp feed-forward are SITL or bench only. DH Bailout, our PX4 stop mode, is flashed but not on the mode switch (organiser permission pending); SITL stop distance 0.38 m against 0.56 m for stock Land, and 1.22 m with no disarm when the kit stream runs.
Heading: turning drifts 10 to 20 cm per turn; a fixed heading starved cuVSLAM by the north wall in SITL. On the real tunnel run the drift was under 1 deg over 4 turns and about 2.5 deg by the tunnel. CI runs about 6 percent of the tests.
Future directions.
Lidar localization: Gauss-Newton point-to-plane scan match to the map, 0.6 cm rms against PX4 on the bench; pose_fusion corrects the setpoints for drift. This also gives the drone a way to see a wall that is not in the map.
Trajectory stack: 30 s in SITL with velocity feed-forward against 102 s for stop-and-go.
3D footprint: combine the exact rectangle with heading and height in one plan, so the drone can yaw where the space allows it and hold heading where the margins are thin.
Heading A/B test on the drone, with PX4's gyro as the reference.
Speed ladder: 0.3 stop-and-go, 0.5, 0.7, 0.85, 1.0, then 1.0 with the return (x 1.25). One step per clean run, set by hand.`);
}

(async () => {
  await pres.writeFile({ fileName: OUT });
  if (process.env.APPLY_THEME) await require(process.env.APPLY_THEME).applyTheme(OUT, THEME);
  console.log("wrote", OUT);
})().catch((e) => { console.error(e); process.exit(1); });
