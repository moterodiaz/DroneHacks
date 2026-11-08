// Team 08 technical presentation: title slide + 7 slides, engineering-report style, ASD-STE100 text.
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
    accent1: "B42318", accent2: "1F3A5F", accent3: "2E7D32", accent4: "BFBFBF",
    accent5: "808080", accent6: "E6E6E6", hlink: "1F3A5F", folHlink: "808080",
  },
};
const HEX = THEME.colors;

const pres = new pptxgen();
pres.layout = "LAYOUT_WIDE"; // 13.333 x 7.5 in
pres.author = "Team 08";
pres.title = "skybeta-11 autonomy: stack, approach, results";
pres.theme = { headFontFace: THEME.headFontFace, bodyFontFace: THEME.bodyFontFace };
const C = pres.SchemeColor;
const INK = C.text1, GREY = C.text2, TINT = C.background2, BLUE = C.accent2, MUTE = C.accent5, RED = C.accent1;

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
  s.addText(label, T({ x: L, y: 0.3, w: 6, h: 0.24, fontSize: 11, bold: true, color: MUTE, charSpacing: 1 }));
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

function box(s, x, y, w, h, title, sub) {
  s.addShape(pres.shapes.RECTANGLE, { x, y, w, h, fill: { color: C.background1 }, line: { color: INK, width: 0.75 }, objectName: "box " + title });
  const runs = [{ text: title, options: { bold: true, breakLine: !!sub, fontSize: 13, color: INK } }];
  if (sub) runs.push({ text: sub, options: { fontSize: 11, color: GREY } });
  s.addText(runs, T({ x: x + 0.05, y, w: w - 0.1, h, align: "center", valign: "middle" }));
}
function arrow(s, x1, y1, x2, y2, label, lx, ly, lw, align = "center") {
  s.addShape(pres.shapes.LINE, { x: Math.min(x1, x2), y: Math.min(y1, y2), w: Math.abs(x2 - x1), h: Math.abs(y2 - y1), flipH: x2 < x1, flipV: y2 < y1, line: { color: INK, width: 0.75, endArrowType: "triangle" } });
  if (label) s.addText(label, T({ x: lx, y: ly, w: lw, h: 0.3, fontFace: MONO, fontSize: 10.5, color: BLUE, align }));
}
// Horizontal bars drawn as shapes (Keynote drops pptxgenjs charts). rows: [label, value, text, color].
function bars(s, x, y, w, labW, max, rows, rowH = 0.5, size = 14) {
  const bw = w - labW - 1.7;
  rows.forEach(([lab, v, txt, col], i) => {
    const yy = y + i * rowH;
    s.addText(lab, T({ x, y: yy, w: labW, h: 0.34, fontSize: size, color: GREY, valign: "middle" }));
    s.addShape(pres.shapes.RECTANGLE, { x: x + labW, y: yy + 0.06, w: (v / max) * bw, h: 0.24, fill: { color: col || GREY }, line: { type: "none" }, objectName: "bar " + lab });
    s.addText(txt, T({ x: x + labW + (v / max) * bw + 0.08, y: yy, w: 1.65, h: 0.34, fontSize: size, color: col === RED ? RED : INK, bold: col === BLUE, valign: "middle" }));
  });
}

// ===================== Title =====================
{
  pres.addSection({ title: "Title" });
  const s = pres.addSlide({ masterName: "REPORT", sectionTitle: "Title" });
  s.addText("DRONEHACKS 2026  ·  TECHNICAL PRESENTATION  ·  8 NOV", T({ x: L, y: 0.3, w: 8, h: 0.24, fontSize: 11, bold: true, color: MUTE, charSpacing: 1 }));
  s.addText("Team 08: autonomous course flight on skybeta-11", T({ x: L, y: 1.3, w: 6.9, h: 1.6, fontSize: 36, bold: true }));
  s.addText("Autonomy stack, approach and results", T({ x: L, y: 3.0, w: 6.9, h: 0.45, fontSize: 20, color: GREY }));
  block(s, L, 3.95, 6.6, 2.6, [
    ["No GPS.", "Stereo camera odometry, PX4 flies"],
    ["Map in, route out.", "One map file per course"],
    ["Flown package:", "5391e34, stop-and-go 0.3 m/s"],
    ["SITL [S]:", "102 s, window ±8 cm, pad ±11 cm"],
  ]);
  s.addImage({ path: path.join(HERE, "assets", "route-5391e34.png"), x: R - 5.1, y: 1.05, w: 5.1, h: 5.1, altText: "Planned stop-and-go route at 5391e34 on the taped course map: launch, diagonal-wall gap, straight leg to the tunnel's north hold, tunnel, window, pad. Pale red is the no-fly zone for the drone centre." });
  s.addText("Planned route at 5391e34 on the taped map. Red: no-fly zone for the drone centre.",
    T({ x: R - 5.1, y: 6.25, w: 5.1, h: 0.5, fontSize: 11, color: GREY }));
  s.addNotes(
`Format: 5 to 7 min of slides, then 5 to 6 min of questions. The judges probe decisions, experiments, testing and challenges.
Points: 25 for the presentation, 15 for generality. An AI review of the repository flags hard-coding.
Drone: skybeta-11. Pixhawk 6C with PX4 1.16, Jetson Orin Nano, OAK-D S2 stereo camera, downward rangefinder.
Flown package: dronehacks_course_follower, branch core/flight-iteration at 5391e34. A* on a 5 cm grid. Stop-and-go at 0.3 m/s, then a speed ladder.
Flown so far: square-test flights, 2 stop-and-go course runs, 1 scored session.
SITL on the taped course at 5391e34: stop-and-go 102 s, smooth 0.5 m/s 76 s, 0 contacts. Window crossed within 8 cm of centre. Landed within 11 cm of the pad centre.

Questions to rehearse:
How do you know where you are? cuVSLAM from stereo, anchored by the 2 s rest on the launch mark, checked against PX4 before take-off.
The dark tunnel? Line up outside, fly straight, no turns inside. The jump guard lands the drone if the pose leaps.
Why stop-and-go? Each in-place turn costs 10 to 20 cm of drift. The hold lets it settle. Speed comes one rung at a time.
Why not never yaw? In SITL a fixed heading made the camera stare at the north wall and lose tracking. On the real tunnel run, the heading drift was under 1 degree through four turns and about 2.5 degrees by the tunnel.
What is hard-coded? The course is the map file. Code constants are the drone size, rates and margins.
Another course? New map file and launch mark. The planner, the tunnel and window handling and every safety check carry over.

Do not claim: a 60/40 split (it is 65/35); that cuVSLAM uses the IMU (the kit runs it stereo-only); that the tunnel run was a collision (it was a kill); course completion on the real drone before it happens.`);
}

// ===================== 1. Problem =====================
{
  pres.addSection({ title: "Problem" });
  const s = slide("1  PROBLEM", "Fly the course alone, without GPS", "Problem");
  head(s, L, 1.5, 6.6, "Task");
  block(s, L, 1.95, 6.6, 2.4, [
    "One start command. No pilot input",
    "Gap → dark tunnel → quadrant 4 → window → pad",
    "Optional return home: run points × 1.25",
    "3 min limit. Each contact adds 5 s",
  ]);
  head(s, L, 4.35, 6.6, "Constraints");
  block(s, L, 4.8, 6.6, 2.2, [
    "Corridor 0.93 m. Props reach 0.28 m from centre",
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
`The task is one autonomous run per start command. There is no GPS and no pilot input.
Sequence: exit quadrant 1 through the gap past the diagonal wall, the dark tunnel, enter quadrant 4, the window, land on the pad. The return to the launch mark multiplies the run points by 1.25. The best run counts.
Score = 65/30 x (checkpoints + window) x 1.25 + 35 x judges/40. A checkpoint reached over a wall gives 15 percent. The window is worth more than the gap between a 3- and a 7-point landing.
Platform: 7 in quad, 1.25 kg. Jetson Orin Nano 8 GB, ROS 2 Humble. Pixhawk 6C, PX4 1.16. OAK-D S2 stereo, 75 mm baseline, 30 fps. VL53L1X rangefinder, 0 to 4 m, down. cuVSLAM stereo-only, 30 Hz, 62 to 69 ms latency. uXRCE-DDS on TELEM2.
Course as measured: net 6.10 m square, walls 1.727 m high, corridors 0.93 to 1.07 m. Error budget for the drone centre: plus or minus 0.18 m. The built course differs from the CAD: the north wall is 24 in inside the net, the window opening is 36 to 61 in above the floor, the diagonal-wall gap is about 3 ft.
Interface: pose altitude 0.3 to 3.0 m, twist at most 1.5 m/s, resend 10 Hz, timeout 1 s. Auto-Offboard occurs one time per arming. A stick input ends Offboard for the remainder of the flight. The kill switch always operates.
Rules: no writes to /fmu/in, only allowlisted PX4 parameters, no Acro, no course geometry in code.
Two origins: cuVSLAM starts at the stack start. PX4 height starts at power-up from the rangefinder. This difference caused our first failure.`);
}

// ===================== 2. Autonomy stack =====================
{
  pres.addSection({ title: "System" });
  const s = slide("2  SYSTEM", "We send setpoints. PX4 closes the loops", "System");
  const lx = L, lw = 8.1;
  s.addShape(pres.shapes.RECTANGLE, { x: lx, y: 1.5, w: lw, h: 2.35, fill: { color: TINT }, line: { type: "none" } });
  s.addShape(pres.shapes.RECTANGLE, { x: lx, y: 4.35, w: lw, h: 1.9, fill: { color: TINT }, line: { type: "none" } });
  s.addText("JETSON ORIN NANO · ROS 2", T({ x: lx + 0.12, y: 1.56, w: 5, h: 0.22, fontSize: 10, bold: true, color: MUTE, charSpacing: 1 }));
  s.addText("PIXHAWK 6C · PX4 1.16", T({ x: lx + lw - 5.12, y: 4.41, w: 5, align: "right", h: 0.22, fontSize: 10, bold: true, color: MUTE, charSpacing: 1 }));
  const by = 2.15, bh = 0.85;
  box(s, 0.75, by, 1.4, bh, "OAK-D S2", "stereo camera");
  box(s, 2.7, by, 1.45, bh, "cuVSLAM", "odometry");
  box(s, 4.7, by, 1.6, bh, "Our node", "plan, follow, guard");
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
  head(s, rx, 4.2, rw, "Design rules");
  block(s, rx, 4.65, rw, 2.2, [
    "Logic has no ROS",
    "Course frame from a 2 s rest on the mark",
    "No direct motor control",
  ], 17);
  s.addNotes(
`The camera sends stereo images to cuVSLAM. cuVSLAM sends odometry at 30 Hz.
The bridge sends the same pose to EKF2 as vision. EKF2 also uses the rangefinder and the IMU.
Our node reads the map and the pose at 50 Hz. It sends one pose or one twist. The kit's flight_interface clamps it and resends it at 10 Hz.
PX4 flies a cascade: position P (MPC_XY_P 0.95), velocity PI (P 1.8, I 0.4), then attitude and rate.
Not shown: the pilot RC goes directly to the Pixhawk. DH Bailout, our PX4 module, runs on the Pixhawk without the Jetson.
States: WAIT, CLIMB to 1.23 m (the window centre, a map value), FLY, DESCEND at 0.5 m/s until PX4 detects touchdown and disarms, RETURN at 2.3 m (last ladder step). LOST: a pose jump over 0.3 m at over 3 m/s; descend straight down. STOPPED: Offboard ended; send no more setpoints.
Code layers: 1 logic, step(t, pose) to command, no ROS. 2 ROS node: topics, parameters, recording. 3 launch. 4 package. The same logic runs in unit tests, our SITL and the drone.
Frames: course (map), odom ENU (cuVSLAM), PX4 NED, body. Course to odom comes from the mean pose over 2 s at rest on the mark, within 2 cm and 1 degree. Nobody types a transform. Every setpoint z adds the PX4 minus camera height offset, filtered with tau 1 s.
Decision: we rejected a direct-to-motor neural policy (MonoRace, SkyDreamer). The rules forbid writes to PX4 topics, the link latency is 68.7 ms, and there is no RPM feedback.`);
}

// ===================== 4. Approach: planning =====================
{
  pres.addSection({ title: "Method" });
  const s = slide("3  METHOD: PLANNING", "Map in, route out", "Method");
  const lw = 6.6;
  head(s, L, 1.5, lw, "Planner");
  block(s, L, 1.95, lw, 2.6, [
    "The map file is the only course input",
    "A* on a 5 cm grid",
    "No-fly radius 0.32 m: prop reach + 4 cm",
    "Straight through the tunnel and the window",
  ]);
  head(s, L, 4.4, lw, "Followers");
  block(s, L, 4.85, lw, 2.1, [
    ["Stop-and-go [F]:", "hold at each corner, 0.3 m/s"],
    ["Trajectory stack [S]:", "time-optimal speed, twist"],
    ["pathfly [M]:", "exact footprint, fixed heading"],
  ]);
  const fx = R - 5.2;
  s.addImage({ path: path.join(HERE, "assets", "route-5391e34.png"), x: fx, y: 1.35, w: 5.2, h: 5.2, altText: "Stop-and-go route at 5391e34 on the taped map with the no-fly zone and holds 1 to 10." });
  s.addText("Stop-and-go route at 5391e34, taped map. Red: no-fly zone for the drone centre.", T({ x: fx, y: 6.6, w: 5.2, h: 0.4, fontSize: 11, color: GREY }));
  s.addNotes(
`All followers use one planning core. The map is the only course input.
Map: STEP AP214 to the map file: walls, boulders, goals, goal order. The parser stops on ambiguity. Tape corrections go in the map (rule 6.4).
Grid: 5 cm cells. Lethal radius 0.32 m = 0.28 m prop reach + 0.04 m. Cost exp(-3 x margin) beyond it (Nav2 inflation).
Rocks grow by the rangefinder cone, (h + 0.20) tan 13.5 deg + 0.10 m. If the beam sees a rock, PX4 sees the floor rise 0.53 m and climbs.
Search: A*, 8-connected, octile heuristic. A diagonal step needs both side cells free. A straight segment replaces grid steps if it adds no cost.
Openings are straight legs. Tunnel: hold 0.6 m before the north mouth and 0.75 m past the south mouth; no turn inside. Window on its normal, plus or minus 0.7 m. With the window closed, A* finds Gate B. In 39 SITL flights the window was the closest pass in 21 before this rule.
Stop-and-go flew on the drone. Polyline, hold at corners. The setpoint is at most 0.3 m ahead of the drone. Heading along the route so the camera sees open course. Turn at the hold at 30 deg/s, settle 1 s within 0.15 m.
Trajectory stack (SITL): fillets with 0.35 m curvature smoothing. Speed caps, tightest wins: lateral acceleration v <= sqrt(a_lat/kappa) with a_lat 1.0 m/s^2; lateral jerk; braking room v^2/2a + v tau + margin <= free distance; TOPP forward-backward pass with |a| <= 3 m/s^2. Twist command v_ref(t + 0.55 s) + k(r - x); 0.55 s is PX4's velocity-loop lag. Look 0.8 m ahead. In the dark tunnel it coasts up to 4 s on PX4's velocity, then re-anchors.
pathfly: offline planner with the exact drone rectangle, A* at 2 cm, elastic band, 0.25 to 0.6 m/s from margin. Twist only. It holds east, because the margins are 8 cm.`);
}

// ===================== 5. Approach: safety and tools =====================
{
  const s = slide("4  METHOD: SAFETY AND VERIFICATION", "Every guard fails closed", "Method");
  const cw = 3.95, g = 0.3;
  const x1 = L, x2 = L + cw + g, x3 = L + 2 * (cw + g);
  head(s, x1, 1.5, cw, "Guards");
  block(s, x1, 1.95, cw, 5.0, [
    "Rest 2 s on the mark before Offboard",
    "Abort if > 0.5 m from the mark",
    "Stale pose → zero velocity",
    "Stick or kill switch always wins",
    "No /fmu/in writes. No Acro",
  ], 17);
  head(s, x2, 1.5, cw, "Verification");
  block(s, x2, 1.95, cw, 5.0, [
    "Own SITL runs the kit code unmodified",
    "~800 tests. 17 of 17 planted bugs caught",
    "One bag and one report per flight",
    "Independent AI review before each merge",
  ], 17);
  head(s, x3, 1.5, R - x3, "DH Bailout: stop distance [S]");
  bars(s, x3, 2.05, R - x3, 1.45, 1.22, [
    ["DH Bailout", 0.38, "0.38 m", BLUE],
    ["Stock Land", 0.56, "0.56 m"],
    ["Land + kit", 1.22, "1.22 m", RED],
  ], 0.55, 14);
  block(s, x3, 3.95, R - x3, 2.9, [
    "Our PX4 flight mode",
    "Flashed on skybeta-11",
    "Not on the mode switch: permission pending",
  ], 15);
  s.addNotes(
`Every guard fails closed. If a check fails, the node sends nothing, and PX4 does not enter Offboard.
Before Offboard: rest 2 s within 2 cm and 1 degree. PX4 and the camera agree within 0.3 m. Abort if Offboard starts more than 0.5 m from the mark.
Start-up: an invalid map stops the node before it creates a publisher. In flight: the line from the drone to the setpoint must be clear, else freeze the reference. Off path by 0.35 to 0.5 m: hold for the pilot.
A pose setpoint never expires. So a stale state sends zero twists, not the last pose. Landing continues down on a stale pose; else the drone climbs 0.27 m (SITL).
Invariants: no Acro, no /fmu/in writes, no change to the organisers' PX4 safety settings or the flight interface, no course literals in code. Connect only to skybeta-11. One PX4 parameter change only: COM_DISARM_LAND 5 s, organiser approval (#40). We save and compare FC parameters before and after each change. Contracts: frames, course, px4, ros-graph. If code disagrees with a contract, the code is wrong.
DH Bailout is our own PX4 flight mode, external mode 1 (nav_state 23), uORB only. Brake at 3 m/s^2, jerk 20 m/s^3, descend 0.7 m/s, 0.35 m/s below 0.8 m. SIH SITL stop distance 0.37 to 0.39 m versus 0.54 to 0.59 m for stock Land. With the kit stream on, stock Land travelled 1.22 m and did not disarm: the stream reset the land detector. Host tests: 205,132 checks, 0 failures.
Localization: cuVSLAM stereo-only, 2.3 percent return-to-start drift on a 57 s walk. EKF2 fuses the IMU. vo_guard withholds stale, failed, jumping or spinning poses. EKF2 rejects all vision at 0.18 s latency or more; we measured 62 to 69 ms. Lidar scan matching and pose_fusion are built and bench tested (0.6 cm rms) but not flown.
Our SITL: 26k lines, no ROS or GPU. It ports PX4's position, attitude, rate and allocation loops and EKF2 delayed vision, and models cuVSLAM from 35 rays on the STEP mesh. The kit's 149 tests pass on it. A referee scores crossings and contacts on ground truth. It found the dark-tunnel failure and the latency cliff before the drone did.`);
}

// ===================== 3. Approach: iteration =====================
{
  pres.addSection({ title: "Experiments" });
  const s = slide("5  EXPERIMENTS: ITERATIVE FLIGHT TESTS", "One primitive per flight. One fix per failure", "Experiments");
  const steps = [
    ["Sat 15:30", "Climb", [red("Flew 1.33 m for 0.73 m")], "Add the PX4 height offset"],
    ["Sat 17:30", "Box, 4 yaws", [red("10–18 cm drift per yaw")], "Slow yaw, settle 1 s"],
    ["Sat 19:55", "All primitives", [blue("Square, land, relaunch, circle: pass")], "Next: the course"],
    ["Sun 00:13", "Course run 1", [red("Hit a wall not in the map")], "Tape the course into the map"],
    ["Sun 00:21", "Tunnel run", [red("Turned early. Kill, no contact")], "Turns wait for the drone"],
    ["Sun AM", "Scored runs", [red("Stick input ended each run")], "Hands off in Offboard"],
    ["Sun 12:01", "Straight leg", [blue("5391e34: 102 s, 0 contacts [S]")], "Fly it at 0.3 m/s"],
  ];
  const g = 0.15, cw = (W - 6 * g) / 7, ly = 1.95;
  s.addShape(pres.shapes.LINE, { x: L, y: ly, w: W, h: 0, line: { color: INK, width: 1 } });
  steps.forEach(([when, name, result, fix], i) => {
    const x = L + i * (cw + g);
    s.addShape(pres.shapes.OVAL, { x: x, y: ly - 0.08, w: 0.16, h: 0.16, fill: { color: i === 6 ? BLUE : INK }, line: { type: "none" }, objectName: "dot " + name });
    s.addText(when, T({ x, y: 1.55, w: cw, h: 0.28, fontSize: 11, color: MUTE }));
    s.addText(name, T({ x, y: 2.2, w: cw, h: 0.35, fontSize: 15, bold: true }));
    s.addText(result, T({ x, y: 2.65, w: cw, h: 0.85, fontSize: 13 }));
    s.addText("→ " + fix, T({ x, y: 3.45, w: cw, h: 0.7, fontSize: 13, color: GREY }));
  });
  head(s, L, 4.85, 6, "Method");
  block(s, L, 5.3, 6, 1.7, [
    "Find the cause in the logs first",
    "Add code only for a measured failure",
    "Raise speed one step per clean run",
  ]);
  head(s, 7.1, 4.85, R - 7.1, "Pre-event SITL lessons");
  block(s, 7.1, 5.3, R - 7.1, 1.7, [
    "Corners set the time, not top speed",
    "+50 ms latency doubled the error",
    "Map 10 cm off: 5 of 5 runs touched",
  ]);
  s.addNotes(
`We did not fly the course first. We proved one primitive per flight: hold, move, turn, land, disarm, relaunch, curve, then the course. We found the cause in the bag and the PX4 ULog, then changed the code.
Fri night: 65 rules, 4 contracts, pre-event Gazebo SITL. Sat first flight: the drone flipped; the motor configuration was overwritten. Fix: motor check before arming, kit 0.3.3, move to skybeta-11.
003042: target 0.73 m, flew 1.33 m. PX4 z read minus 0.72 m on the floor. PX4 height comes from the rangefinder, zero at power-up. cuVSLAM is zero at stack start. Fix: z_sp = z_odom + (z_PX4 - z_odom), filtered with tau 1 s.
013018: 10 to 18 cm drift after each 90 degree yaw. A 0.10 m/s^2 push is fixed in the body frame. PX4's velocity integrator cancels it in world axes. After a yaw the residual is 2 sin(dpsi/2) x push. The model predicts each corner's drift direction within 9 to 18 degrees. Estimate vs odometry 1.3 cm. Also: PX4 disarmed 0.8 s into a relaunch; COM_DISARM_LAND 5 s with organiser approval (#40).
035510: square, land, relaunch, 1 m circle, land, disarm at 107 s. Land detection took 4 s; it needs a commanded descent of at least 0.275 m/s. Push 1.0 m/s down: 4.20 to 3.58 s in SITL.
081345: hit the north wall. The log put it 24 in inside the net; the tape agreed. Localization was right to inches; the map was wrong.
082135: the turn began when the setpoint reached the corner while the drone trailed 0.25 m. Pilot kill at 79.57 s, motor current 0. No contact. Fix: turns wait for the drone; hold 0.75 m past the south mouth.
Scored session: each run ended on stick input. The crash came after full throttle and yaw at 66.2 s; the tracking error was 3 to 8 cm before it. The pose jumped metres after a hand landing; fix: LOST state, restart the stack after handling.
5391e34: corridor goal removed; one straight leg from the gap to the tunnel's north hold. SITL stop-and-go 109 to 102 s, smooth 0.5 m/s 79 to 76 s.
Speed ladder: 0.3 stop-and-go, 0.5, 0.7, 0.85, 1.0, 1.0 plus return. One step per clean run, set by hand.
Pre-event SITL: 1.0, 1.5 and 2.0 m/s all took 27 to 31 s. Facing a wall at 0.3 m withheld 390 of 543 frames, so look 0.8 m ahead. Organiser lab: step setpoints overshot 38 to 60 cm.`);
}

// ===================== 6. Results: evidence =====================
{
  pres.addSection({ title: "Results" });
  const s = slide("6  RESULTS", "Tracking held. The map and the sticks failed", "Results");
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
Planner bench: waypoint steps 30.0 s, smoothed one speed 65.6 s, speed profile with lead 23.5 s. One safe speed must suit the slowest corner; the speed profile saves 22 percent.
Pre-event SITL, 15 layouts: tunnel blinded, 12 of 14 cleared, error 2.4 to 7.6 cm rms, launch to pad 27 to 34 s. With +50 ms latency: 19 of 20 cleared.`);
}

// ===================== 7. Results: generality =====================
{
  pres.addSection({ title: "Generality" });
  const s = slide("7  GENERALITY", "A new course is a new map file", "Generality");
  const lw = 6.2;
  head(s, L, 1.5, lw, "Planner vs hand route");
  table(s, L, 1.95, [3.2, 1.5, 1.5], [
    ["Test", "Planner", "Hand route"],
    ["Random layouts [M]", [blue("12 / 12")], "0 / 12"],
    ["Walls moved ≤ 5 cm [M]", [blue("6 / 6")], "0 / 6"],
    ["Boulders moved [S]", [blue("20 / 20")], "6 / 20"],
    ["Unseen layouts, PX4 SITL [S]", "8 / 9", "—"],
  ], 15);
  block(s, L, 4.35, lw, 2.0, [
    "No coordinates in code: a test checks",
    "Closed window → A* finds Gate B",
  ], 17);
  const rx = 7.3, rw = R - rx;
  head(s, rx, 1.5, rw, "Course-specific (declared)");
  block(s, rx, 1.95, rw, 1.9, [
    "The map file, with tape corrections",
    "Flight height 1.23 m, from the map",
    "Launch mark, by tape",
  ], 17);
  head(s, rx, 3.85, rw, "New course procedure");
  block(s, rx, 4.3, rw, 2.0, [
    "STEP file → map file",
    "Tape-measure and correct the map",
    "SITL, then stop-and-go at 0.3 m/s",
  ], 17);
  s.addText("Credits: skybeta starter stack · NVIDIA Isaac ROS cuVSLAM · Luxonis DepthAI, depthai-ros · PX4, QGroundControl · ROS 2, rosbag2, MCAP · Gazebo · Nav2 inflation cost · Pham & Pham TOPP · NumPy, PyYAML · AI coding tools (rule 9.2)",
    T({ x: L, y: 6.35, w: W, h: 0.5, fontSize: 11, color: MUTE }));
  s.addNotes(
`The planner holds no layout knowledge. It flew all 12 random layouts. The hand route planned none of them. Also: published map 1 of 1, rotated and mirrored 2 of 2, 17 of 17 planted bugs caught.
A test searches our source for map coordinates and wall names. A wall across the window makes A* find Gate B; no code names Gate B. Goals come from the CAD geometry. The course frame comes from the rest on the launch mark. Constants are the drone size, rates and margins, each with a unit.
Course-specific, say it first: the map file with the 8 Nov tape corrections; the flight height 1.23 m, the middle of the taped window opening, a map value; pathfly's precomputed route and fixed east heading (run plan_path.py again on a new course); the launch mark position and heading, by tape. The corridor goal from after the crash was removed in 5391e34.
Limits: the map is trusted; no sensor finds walls in flight. A 5 cm wall shift made a tight corner: 61 to 81 s. The contract schema has no checkpoint-gate field. CI runs about 6 percent of the tests; the other suites run locally.
Not yet flown: smooth follower, trajectory stack, return home, pathfly, yaw_comp feed-forward, DH Bailout on the mode switch. Heading A/B: a fixed heading starved cuVSLAM at the north wall in SITL; on the real tunnel run the heading drift was under 1 deg over 4 turns and about 2.5 deg by the tunnel.
Procedure for a new course: 1 get the STEP file. 2 run step_to_map.py; it stops if a part is ambiguous. 3 measure the built course with tape and correct the map file. 4 tape the launch mark and its heading. 5 run the dry run and our SITL on the new map. 6 fly ladder step 0, stop-and-go at 0.3 m/s.
Credits (rule 9.1): Pham & Pham TOPP is T-RO 2018. AI coding tools (rule 9.2): we can explain each line that flies.`);
}

// ===================== 8. Limitations and future work =====================
{
  pres.addSection({ title: "Discussion" });
  const s = slide("8  LIMITATIONS AND FUTURE WORK", "The map is trusted. Only stop-and-go has flown", "Discussion");
  const cw = (W - 0.5) / 2, x2 = L + cw + 0.5;
  head(s, L, 1.5, cw, "Limitations");
  block(s, L, 1.95, cw, 3.0, [
    ["Map trusted:", "no wall sensing in flight"],
    ["Map error:", "5 cm wall shift, 61 → 81 s [S]"],
    ["Flown:", "stop-and-go only"],
    ["Heading:", "fixed vs along the route, open"],
    ["CI:", "runs ~6 % of the tests"],
  ]);
  head(s, x2, 1.5, cw, "Future directions");
  block(s, x2, 1.95, cw, 3.0, [
    ["Lidar localization:", "see walls, 0.6 cm bench"],
    ["Trajectory stack:", "30 s vs 102 s in SITL"],
    ["DH Bailout:", "on the mode switch, if approved"],
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
The map is trusted. The planner believes the map file. No sensor finds walls in flight. Run 1 hit a wall that was not in the map; the log located it, and we taped the map by hand.
A 5 cm wall shift turned a corner tight: 61 to 81 s in SITL. A course 10 cm off its map file touched in 5 of 5 SITL runs.
Only stop-and-go has flown the course. The smooth follower, trajectory stack, return home, pathfly and yaw_comp feed-forward are SITL or bench only.
Heading: a fixed heading starved cuVSLAM at the north wall in SITL. On the real tunnel run the drift was under 1 deg over 4 turns and about 2.5 deg by the tunnel against PX4's gyro. pathfly needs a fixed east heading for 8 cm margins. This needs a flown A/B.
The contract schema has no checkpoint-gate field. CI runs about 6 percent of the tests; the other suites run locally.
Future directions.
Climb the speed ladder one step per clean run: 0.3, 0.5, 0.7, 0.85, 1.0, then 1.0 with the return (x 1.25).
Fly the trajectory stack: 30 s in SITL with velocity feed-forward against 102 s for stop-and-go.
Lidar localization: Gauss-Newton point-to-plane scan match to the map, 0.6 cm rms against PX4 on the bench. pose_fusion corrects the setpoints for cuVSLAM drift (target error 0.208 to 0.003 m in the model). This also gives the drone a way to see a wall that is not in the map.
DH Bailout on the mode switch, after organiser permission: 0.38 m stop distance against 0.56 m for stock Land in SITL.
Heading A/B test on the drone, with PX4's gyro as the reference.`);
}

(async () => {
  await pres.writeFile({ fileName: OUT });
  if (process.env.APPLY_THEME) await require(process.env.APPLY_THEME).applyTheme(OUT, THEME);
  console.log("wrote", OUT);
})().catch((e) => { console.error(e); process.exit(1); });
