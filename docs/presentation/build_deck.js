// Team 08 technical presentation: title slide + 7 slides, engineering-report style, ASD-STE100 text.
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
const INK = C.text1, GREY = C.text2, TINT = C.background2, BLUE = C.accent2, MUTE = C.accent5;

const L = 0.45, R = 13.333 - 0.45, W = R - L;
pres.defineSlideMaster({
  title: "REPORT",
  background: { color: C.background1 },
  objects: [
    { placeholder: { options: { name: "title", type: "title", x: L, y: 0.52, w: W, h: 0.5, fontFace: "Arial", fontSize: 22, bold: true, color: INK, align: "left", valign: "middle", margin: 0 }, text: "" } },
    { text: { text: "Team 08  ·  skybeta-11  ·  DroneHacks 2026", options: { x: L, y: 7.1, w: 4, h: 0.22, fontFace: "Arial", fontSize: 9, color: MUTE, margin: 0 } } },
    { text: { text: "[F] flown on skybeta-11   [S] our SITL   [M] model, bench or test", options: { x: 4.6, y: 7.1, w: 5.5, h: 0.22, fontFace: "Arial", fontSize: 9, color: MUTE, align: "center", margin: 0 } } },
  ],
  slideNumber: { x: R - 0.6, y: 7.1, w: 0.6, h: 0.22, fontFace: "Arial", fontSize: 9, color: MUTE, align: "right" },
});

// ---------- helpers ----------
const T = (o) => Object.assign({ isTextBox: true, margin: 0, fontFace: "Arial", valign: "top", color: INK }, o);
const MONO = "Courier New";

function slide(n, label, title, section) {
  const s = pres.addSlide({ masterName: "REPORT", sectionTitle: section });
  s.addText(label, T({ x: L, y: 0.28, w: 6, h: 0.22, fontSize: 10, bold: true, color: MUTE, charSpacing: 1 }));
  s.addText(title, { placeholder: "title" });
  return s;
}
function head(s, x, y, w, txt) {
  s.addText(txt, T({ x, y, w, h: 0.26, fontSize: 12, bold: true }));
}
// Text block. items: string = bullet; [bold, rest] = bullet with a run-in label.
function block(s, x, y, w, h, items, size = 11.5) {
  const runs = [];
  items.forEach((it, i) => {
    const last = i === items.length - 1;
    const base = { breakLine: !last, paraSpaceAfter: 3 };
    if (typeof it === "string") runs.push({ text: it, options: Object.assign({ bullet: { indent: 10 }, color: GREY }, base) });
    else {
      runs.push({ text: it[0] + " ", options: { bullet: { indent: 10 }, bold: true, color: INK } });
      runs.push({ text: it[1], options: Object.assign({ color: GREY }, base) });
    }
  });
  s.addText(runs, T({ x, y, w, h, fontSize: size }));
}
// Booktabs table: bold header with a black rule under it, grey hairlines between rows, no verticals.
// A cell is a string, {text, options}, or an array of runs.
function table(s, x, y, colW, rows, size = 10.5) {
  const none = { type: "none" };
  const data = rows.map((r, ri) => r.map((c) => {
    const cell = typeof c === "string" ? { text: c } : Array.isArray(c) ? { text: c } : c;
    const o = Object.assign({ fontFace: "Arial", fontSize: size, color: ri === 0 ? HEX.dk1 : HEX.dk2, bold: ri === 0, valign: "top", margin: [0.03, 0.06, 0.03, 0.06] }, cell.options || {});
    o.border = [ri === 0 ? { type: "solid", pt: 1, color: HEX.dk1 } : none, none,
      ri === 0 ? { type: "solid", pt: 1, color: HEX.dk1 } : { type: "solid", pt: 0.5, color: HEX.accent4 }, none];
    return { text: cell.text, options: o };
  }));
  s.addTable(data, { x, y, w: colW.reduce((a, v) => a + v, 0), colW });
}
const b = (t) => ({ text: t, options: { bold: true, color: HEX.dk1 } });
const t_ = (t) => ({ text: t });
const red = (t) => ({ text: t, options: { color: HEX.accent1 } });

function box(s, x, y, w, h, title, sub) {
  s.addShape(pres.shapes.RECTANGLE, { x, y, w, h, fill: { color: C.background1 }, line: { color: INK, width: 0.75 }, objectName: "box " + title });
  const runs = [{ text: title, options: { bold: true, breakLine: !!sub, fontSize: 10.5, color: INK } }];
  if (sub) runs.push({ text: sub, options: { fontSize: 9, color: GREY } });
  s.addText(runs, T({ x: x + 0.06, y, w: w - 0.12, h, align: "center", valign: "middle" }));
}
function arrow(s, x1, y1, x2, y2, label, lx, ly, lw, align = "center") {
  s.addShape(pres.shapes.LINE, { x: Math.min(x1, x2), y: Math.min(y1, y2), w: Math.abs(x2 - x1), h: Math.abs(y2 - y1), flipH: x2 < x1, flipV: y2 < y1, line: { color: INK, width: 0.75, endArrowType: "triangle" } });
  if (label) s.addText(label, T({ x: lx, y: ly, w: lw, h: 0.3, fontFace: MONO, fontSize: 8.5, color: BLUE, align }));
}

// ===================== Title =====================
{
  pres.addSection({ title: "Title" });
  const s = pres.addSlide({ masterName: "REPORT", sectionTitle: "Title" });
  s.addText("DRONEHACKS 2026  ·  TECHNICAL PRESENTATION  ·  8 NOV", T({ x: L, y: 0.28, w: 8, h: 0.22, fontSize: 10, bold: true, color: MUTE, charSpacing: 1 }));
  s.addText("Team 08: autonomous course flight on skybeta-11", T({ x: L, y: 1.1, w: 7.3, h: 1.3, fontSize: 32, bold: true }));
  s.addText("Autonomy stack, approach and results", T({ x: L, y: 2.45, w: 7.3, h: 0.4, fontSize: 16, color: GREY }));
  table(s, L, 3.2, [1.6, 5.7], [
    ["Item", "Value"],
    ["Drone", "skybeta-11: Pixhawk 6C with PX4 1.16, Jetson Orin Nano, OAK-D S2 stereo camera, downward rangefinder. No GPS"],
    ["Flown package", "dronehacks_course_follower, branch core/flight-iteration at 5391e34"],
    ["Method", "One map file in, route out. A* on a 5 cm grid. Stop-and-go at 0.3 m/s, then a speed ladder"],
    ["Flown [F]", "Square-test flights, 2 stop-and-go course runs, 1 scored session"],
    ["SITL [S]", "Taped course: stop-and-go 102 s, smooth 0.5 m/s 76 s, 0 contacts, window ±8 cm, pad ±11 cm"],
  ], 11);
  s.addImage({ path: path.join(HERE, "assets", "route-5391e34.png"), x: R - 4.6, y: 1.05, w: 4.6, h: 4.6, altText: "Planned stop-and-go route at 5391e34 on the taped course map: launch, diagonal-wall gap, straight leg to the tunnel's north hold, tunnel, window, pad. Pale red is the no-fly zone for the drone centre." });
  s.addText("Stop-and-go route at 5391e34 on the taped map, from the package's own planner. Red: no-fly zone for the drone centre (walls + 0.32 m). Holds 1–10; 10 is the pad.",
    T({ x: R - 4.6, y: 5.72, w: 4.6, h: 0.6, fontSize: 9.5, color: GREY }));
  s.addNotes(
`Format: 5 to 7 min of slides, then 5 to 6 min of questions. The judges probe decisions, experiments, testing and challenges.
Points: 25 for the presentation, 15 for generality. An AI review of the repository flags hard-coding.
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
  const s = slide(1, "PROBLEM", "Autonomous indoor course flight on skybeta-11, without GPS", "Problem");
  const cw = 3.95, g = 0.24;
  const x1 = L, x2 = L + cw + g, x3 = L + 2 * (cw + g);
  head(s, x1, 1.25, cw, "1.1  Task and scoring (8 Nov rules)");
  block(s, x1, 1.6, cw, 2.3, [
    "The drone flies the course after one start command. There is no GPS and no pilot input.",
    "Sequence: exit quadrant 1 through the gap past the diagonal wall, dark tunnel, enter quadrant 4, window, land on the pad.",
    "Optional return to the launch mark. The return multiplies the run points by 1.25.",
    "Limit 3 min per run. Each contact adds 5 s. The best run counts.",
  ]);
  table(s, x1, 3.3, [2.55, 1.4], [
    ["Item", "Points"],
    ["Exit quadrant 1", "6"], ["Tunnel", "10"], ["Enter quadrant 4", "7"], ["Window (bonus)", "+4"],
    ["Pad: blue / ring / centre", "3 / 5 / 7"], ["Valid return", "× 1.25"],
    ]);
  s.addText([{ text: "Score = 65/30 × (checkpoints + window) × 1.25 + 35 × judges/40", options: { fontFace: MONO, fontSize: 10, color: INK, breakLine: true } },
    { text: "Full run 65. With the window 73.7. With the return 92.1. A checkpoint reached over a wall gives 15 %.", options: { fontSize: 11, color: GREY } }],
  T({ x: x1, y: 5.15, w: cw, h: 0.9 }));

  head(s, x2, 1.25, cw, "1.2  Platform");
  table(s, x2, 1.6, [1.2, 2.75], [
    ["Part", "Specification"],
    ["Airframe", "7 in quad, 1.25 kg. Prop tips 0.28 m from centre"],
    ["Computer", "Jetson Orin Nano 8 GB, ROS 2 Humble"],
    ["Flight ctrl", "Pixhawk 6C, PX4 v1.16.2"],
    ["Camera", "OAK-D S2 stereo, 75 mm baseline, 30 fps, IMU 200 Hz, forward"],
    ["Height", "VL53L1X time of flight, 0–4 m, 27° cone, down"],
    ["Odometry", "cuVSLAM stereo-only, 30 Hz, 62–69 ms latency"],
    ["Link", "uXRCE-DDS on serial (TELEM2), DDS domain 11"],
    ]);
  head(s, x2, 4.45, cw, "1.3  Course as measured");
  block(s, x2, 4.78, cw, 1.75, [
    "Net 6.10 m square. Walls 1.727 m high. Corridors 0.93–1.07 m.",
    "Error budget for the drone centre: ±0.18 m in the narrowest corridor.",
    "The course differs from the CAD: north wall 24 in (0.61 m) inside the net; window opening 36–61 in above the floor; diagonal-wall gap about 3 ft.",
    ]);

  head(s, x3, 1.25, cw, "1.4  Constraints on the software");
  block(s, x3, 1.6, cw, 5.3, [
    ["Interface.", "The kit flight_interface is the only write path to PX4. Pose altitude 0.3–3.0 m. Twist ≤ 1.5 m/s. Resend 10 Hz. Timeout 1 s."],
    ["Offboard.", "Auto-Offboard occurs one time per arming: arm in Altitude, then select Position."],
    ["Pilot.", "A stick input ends Offboard for the remainder of the flight. The kill switch always operates."],
    ["PX4.", "Do not publish to /fmu/in/*. Write only allowlisted parameters (rule 8.5). Do not use Acro."],
    ["Generality.", "Do not put course geometry in code (rule 9.5). An AI review flags hard-coding to the judges."],
    ["Origins.", "cuVSLAM starts at the stack start. PX4 height starts at power-up, from the rangefinder."],
  ]);
  s.addNotes(
`The task is one autonomous run per start command. There is no GPS.
The run points come from the checkpoints. The window and the return give the largest bonuses.
The error budget is 0.18 m. The corridor is 0.93 m and the props reach 0.28 m.
The kit interface sets our limits. We cannot write to PX4 directly. A stick input ends autonomy.
Two origins exist. cuVSLAM starts at the stack start. PX4 height comes from the rangefinder. This difference caused our first failure (slide 3).`);
}

// ===================== 2. Autonomy stack =====================
{
  pres.addSection({ title: "Autonomy stack" });
  const s = slide(2, "AUTONOMY STACK", "Data flow, control loops and code layers", "Autonomy stack");
  const lx = L, lw = 8.15;
  s.addShape(pres.shapes.RECTANGLE, { x: lx, y: 1.25, w: lw, h: 2.3, fill: { color: TINT }, line: { type: "none" } });
  s.addShape(pres.shapes.RECTANGLE, { x: lx, y: 3.95, w: lw, h: 1.7, fill: { color: TINT }, line: { type: "none" } });
  s.addText("JETSON ORIN NANO · ROS 2 HUMBLE", T({ x: lx + 0.1, y: 1.3, w: 5, h: 0.2, fontSize: 9, bold: true, color: MUTE, charSpacing: 1 }));
  s.addText("PIXHAWK 6C · PX4 1.16", T({ x: lx + lw - 5.1, y: 4.0, w: 5, align: "right", h: 0.2, fontSize: 9, bold: true, color: MUTE, charSpacing: 1 }));
  const by = 1.85, bh = 0.75;
  box(s, 0.6, by, 1.3, bh, "OAK-D S2", "depthai-ros");
  box(s, 2.45, by, 1.35, bh, "cuVSLAM", "stereo VO, GPU");
  box(s, 4.4, by, 1.75, bh, "Team node", "plan, follow, guard; 50 Hz");
  box(s, 6.75, by, 1.7, bh, "flight_interface", "clamp, 10 Hz resend");
  box(s, 2.45, 3.0, 1.35, 0.42, "px4_odom_bridge", null);
  arrow(s, 1.9, by + 0.37, 2.45, by + 0.37, "stereo 30 Hz", 1.75, by - 0.3, 0.85);
  arrow(s, 3.8, by + 0.37, 4.4, by + 0.37, "odom 30 Hz", 3.65, by - 0.3, 0.9);
  arrow(s, 6.15, by + 0.37, 6.75, by + 0.37, "pose | twist", 6.0, by - 0.3, 0.9);
  arrow(s, 3.12, by + bh, 3.12, 3.0);
  const py = 4.5, ph = 0.8;
  box(s, 0.6, py, 1.6, ph, "EKF2", "vision + range + IMU");
  box(s, 2.65, py, 1.6, ph, "Position P", "MPC_XY_P 0.95");
  box(s, 4.7, py, 1.6, ph, "Velocity PI", "P 1.8, I 0.4");
  box(s, 6.75, py, 1.7, ph, "Attitude, rate", "allocation → motors");
  arrow(s, 2.2, py + 0.4, 2.65, py + 0.4);
  arrow(s, 4.25, py + 0.4, 4.7, py + 0.4);
  arrow(s, 6.3, py + 0.4, 6.75, py + 0.4);
  arrow(s, 3.12, 3.42, 1.4, py, "vision pose", 0.6, 3.62, 1.2, "left");
  arrow(s, 7.6, by + bh, 3.45, py, "trajectory_setpoint (NED)", 5.3, 3.62, 2.4, "left");
  s.addText([{ text: "Not shown: ", options: { bold: true, color: INK } },
    { text: "the pilot RC goes directly to the Pixhawk. DH Bailout (our PX4 module) runs on the Pixhawk without the Jetson.", options: { color: GREY } }],
  T({ x: lx, y: 5.8, w: lw, h: 0.4, fontSize: 10 }));
  s.addText([{ text: "Decision: ", options: { bold: true, color: INK } },
    { text: "we send setpoints and PX4 closes the loops. We rejected a direct-to-motor neural policy (MonoRace, SkyDreamer): the rules forbid writes to PX4 topics, the link latency is 68.7 ms, and there is no RPM feedback.", options: { color: GREY } }],
  T({ x: lx, y: 6.25, w: lw, h: 0.6, fontSize: 10 }));

  const rx = 8.85, rw = R - rx;
  head(s, rx, 1.25, rw, "2.1  Node states");
  block(s, rx, 1.58, rw, 1.55, [
    ["Main:", "WAIT → CLIMB (1.23 m, window centre) → FLY → DESCEND (0.5 m/s; PX4 detects touchdown and disarms) → RETURN (2.3 m, last ladder step)."],
    ["LOST:", "pose jump > 0.3 m at > 3 m/s. Descend straight down."],
    ["STOPPED:", "Offboard ended. Send no more setpoints."],
    ]);
  head(s, rx, 3.0, rw, "2.2  Code layers");
  block(s, rx, 3.33, rw, 1.6, [
    ["1 Logic:", "step(t, pose, …) → command. No ROS."],
    ["2 ROS node:", "topics, parameters, QoS, recording."],
    ["3 Launch:", "flight_interface + node, map choice."],
    ["4 Package:", "colcon, COMPETITOR_PKG. One bad package stops all launches."],
    ]);
  head(s, rx, 4.95, rw, "2.3  Frames and height");
  block(s, rx, 5.28, rw, 1.65, [
    "course (map) → odom ENU (cuVSLAM) → PX4 NED → body.",
    "course→odom: mean pose over 2 s at rest on the mark, ≤ 2 cm, ≤ 1°.",
    "Every setpoint z adds the PX4 − camera offset (τ 1 s). No stream above 0.3 m.",
    ]);
  s.addNotes(
`The camera sends stereo images to cuVSLAM. cuVSLAM sends odometry at 30 Hz.
The bridge sends the same pose to EKF2. EKF2 also uses the rangefinder and the IMU.
Our node reads the map and the pose. It sends one pose or one twist. The interface clamps it and resends it at 10 Hz.
PX4 flies a cascade: position P, then velocity PI, then attitude and rate.
We do not command the motors. The link has 68.7 ms latency and no RPM feedback.
The logic layer has no ROS. The same code runs in the unit tests, our simulator and the drone.
The course frame comes from the drone at rest on the launch mark. Nobody types a transform.`);
}

// ===================== 3. Approach: first principles and iteration =====================
{
  pres.addSection({ title: "Approach" });
  const s = slide(3, "APPROACH 1/3", "First principles and the iteration timeline", "Approach");
  const tw = 8.75;
  table(s, L, 1.25, [0.95, 1.25, 3.6, 2.95], [
    ["When", "Step", "Evidence", "Change"],
    ["Fri night", "0 Model", "65 rules, 4 contracts. Pre-event Gazebo SITL [S]", "Finish first. Corners set the time. Plan from the map"],
    ["Sat", "1 Fly at all", [b("Drone flipped:"), t_(" motor configuration overwritten. Kit 0.3.2 Offboard went to MAVLink system 1")], "Motor check before arming. Kit 0.3.3. Move to skybeta-11"],
    ["Sat 15:30", "2a Climb", [b("003042:"), t_(" target 0.73 m, flew 1.33 m. PX4 z −0.72 m on the floor [F]")], "Add the PX4 − camera offset to every setpoint"],
    ["Sat 17:30", "2b Box, 4 yaws", [b("013018:"), t_(" 10–18 cm drift after each 90° yaw [F]. Estimate vs odom 1.3 cm")], "Body-fixed push 0.10 m/s². Slow yaw, settle 1 s, fixed-heading option"],
    ["Sat 17:30", "2b Relaunch", [b("013018:"), t_(" PX4 disarmed 0.8 s into the climb [F]")], "COM_DISARM_LAND 5 s, organiser approval (#40)"],
    ["Sat 19:55", "2c All primitives", [b("035510:"), t_(" square, land, relaunch, 1 m circle, land, disarm at 107 s [F]")], "Land detection took 4 s. Push 1.0 m/s down: 4.20 → 3.58 s [S]"],
    ["Sun 00:13", "3 Course run 1", [b("081345:"), t_(" hit the north wall. The log put it 24 in inside the net; the tape agreed [F]")], "Tape corrections into the map"],
    ["Sun 00:21", "3 Tunnel run", [b("082135:"), t_(" turn began 0.25 m before the drone reached the corner. Kill at 79.57 s (motor current 0). No contact [F]")], "Turns wait for the drone. Hold 0.75 m past the south mouth"],
    ["Sun AM", "4 Scored runs", [b("Each run"), t_(" ended on stick input. Crash after full throttle and yaw at 66.2 s; error 3–8 cm before. Pose jumped metres after a hand landing [F]")], "Pilot procedure. LOST state. Restart the stack after handling"],
    ["Sun 12:01", "5 Straight leg", [b("5391e34:"), t_(" corridor goal removed; one leg from the gap to the tunnel's north hold. 0 contacts [S]")], "Stop-and-go 109 → 102 s. Smooth 0.5 m/s 79 → 76 s"],
    ], 9.5);

  const rc = [
    ["3.3  Height: two origins", "PX4 z comes from the rangefinder, zero at power-up. cuVSLAM z is zero at stack start. The interface sends odom z to PX4 unchanged. Fix: z_sp = z_odom + (z_PX4 − z_odom), filtered with τ 1 s."],
    ["3.4  Yaw: body-fixed push", "A 0.10 m/s² push is fixed in the body frame. PX4's velocity integrator cancels it in world axes. After a yaw Δψ, the residual is 2·sin(Δψ/2)·push. The model predicts each corner's drift direction within 9–18°."],
    ["3.5  Landing detection", "Ground contact needs low thrust and a commanded descent ≥ 1.1 × LNDMC_Z_VEL_MAX = 0.275 m/s. Each stage then takes about 1 s. A 1.0 m/s down command lowers the thrust at once."],
  ];
  const rcw = (tw - 0.4) / 3;
  rc.forEach(([h, d], i) => {
    const x = L + i * (rcw + 0.2);
    head(s, x, 5.45, rcw, h);
    s.addText(d, T({ x, y: 5.76, w: rcw, h: 1.25, fontSize: 10, color: GREY }));
  });

  const rx = L + tw + 0.3, rw = R - rx;
  head(s, rx, 1.25, rw, "3.1  Method");
  block(s, rx, 1.58, rw, 2.3, [
    "Prove one primitive per flight: hold, move, turn, land, disarm, relaunch, curve, then the course.",
    "Find the cause in the bag and the PX4 ULog. Then change the code.",
    "Add code only where a test measured a failure.",
    "Speed ladder: 0.3 stop-and-go, 0.5, 0.7, 0.85, 1.0, 1.0 + return. One step per clean run, set by hand.",
    ]);
  head(s, rx, 3.85, rw, "3.2  Pre-event findings that set the design");
  block(s, rx, 4.18, rw, 2.75, [
    "Latency is the largest cost: +50 ms doubled the error [S].",
    "Top speed does not set the time: 1.0, 1.5, 2.0 m/s all took 27–31 s [S].",
    "Facing a wall at 0.3 m withheld 390 of 543 frames [S]. Look 0.8 m ahead.",
    "A course 10 cm off its map file: 5 of 5 runs touched [S]. Survey to 5 cm.",
    "Organiser lab: step setpoints overshot 38–60 cm. Use leashed or smooth setpoints.",
    ]);
  s.addNotes(
`We did not fly the course first. We proved one primitive per flight.
Each row is one failure on the real drone, its evidence and its change.
003042: PX4 and cuVSLAM measured height from different zeros. The drone flew 1.33 m for a 0.73 m target.
013018: after each yaw the drone moved 10 to 18 cm. The bags show a push fixed to the airframe. PX4's integrator holds the correction in world axes, so a yaw makes it wrong.
035510: the full primitive set flew. Square, land, relaunch, circle, land, disarm.
The first course runs found a wall that no map had. The log located it to inches.
In the scored session, the pilot sticks ended every run. Our tracking error was 3 to 8 cm until then.`);
}

// ===================== 4. Approach: planning and following =====================
{
  const s = slide(4, "APPROACH 2/3", "Planning core and three followers", "Approach");
  const cw = 3.65;
  head(s, L, 1.25, cw, "4.1  Shared planning core");
  block(s, L, 1.58, cw, 5.4, [
    ["Map.", "STEP AP214 → map file: walls, boulders, goals, goal order. The parser stops on ambiguity. Tape corrections go in the map (rule 6.4)."],
    ["Grid.", "5 cm cells. Lethal radius 0.32 m = 0.28 m prop reach + 0.04 m. Cost exp(−3·margin) beyond it (Nav2 inflation)."],
    ["Rocks.", "Grow by the rangefinder cone (h + 0.20)·tan 13.5° + 0.10 m. A rock top under the beam reads as a +0.53 m floor step."],
    ["Search.", "A*, 8-connected, octile heuristic, nearest-goal targets. A diagonal step needs both side cells free."],
    ["Openings.", "Straight legs. Tunnel: hold 0.6 m before the north mouth and 0.75 m past the south mouth; no turn inside. Window on its normal, ±0.7 m. Window closed → A* finds Gate B."],
    ["Shortcut.", "A straight segment replaces grid steps if it adds no cost and no lethal cell. Exact segment-to-segment check at the end."],
    ]);

  const tx = L + cw + 0.3, tw = 5.75;
  head(s, tx, 1.25, tw, "4.2  Followers");
  table(s, tx, 1.58, [0.85, 1.55, 1.85, 1.5], [
    ["", "Stop-and-go [F]", "Trajectory stack [S]", "pathfly [M]"],
    [[b("Path")], "Polyline, hold at corners", "Fillets, 0.35 m curvature smoothing, proven clearance bound", "Footprint-exact distance, A* 2 cm, elastic band"],
    [[b("Speed")], "0.3 m/s. Setpoint ≤ 0.3 m ahead", "Time-optimal: v ≤ √(a/κ), jerk ≤ 4 m/s³, braking room", "0.25–0.6 m/s from margin"],
    [[b("Command")], "Pose", "Twist: v_ref(t + 0.55 s) + k·(r − x)", "Twist only (pose floor 0.3 m)"],
    [[b("Heading")], "Along the route: the camera sees open course. Turn at hold, 30°/s; settle 1 s, 0.15 m", "Look 0.8 m ahead, ≤ 1 rad/s", "Fixed east (8 cm margins)"],
    [[b("Tunnel")], "Straight leg, LOST guard", "Coast ≤ 4 s on PX4 velocity, re-anchor", "Not applicable"],
    [[b("Package")], "course_follower", "planner/, nav_traj", "dronehacks_pathfly"],
    ]);
  head(s, tx, 4.75, tw, "4.3  Trajectory speed caps (tightest wins)");
  table(s, tx, 5.08, [1.15, 3.05, 1.55], [
    ["Cap", "Limit", "Reason"],
    ["Lateral accel.", "v ≤ √(a_lat / κ), a_lat 1.0 m/s²", "Centripetal a = v²κ"],
    ["Lateral jerk", "v ≤ (j_max / |dκ/ds|)^(1/3)", "Curvature ramps"],
    ["Braking", "v²/2a + v·τ + margin ≤ free distance, τ 0.17 s", "Stop before a wall"],
    ["Pass", "TOPP forward-backward pass; |a| ≤ 3 m/s²", "Time-optimal"],
  ], 10);
  const fx = R - 2.55, fw = 2.55;
  s.addImage({ path: path.join(HERE, "assets", "route-5391e34.png"), x: fx, y: 1.3, w: fw, h: 2.55, altText: "Stop-and-go route at 5391e34 on the taped map with the no-fly zone and holds 1 to 10." });
  s.addText("Stop-and-go route at 5391e34, taped map. Red: no-fly zone for the centre.", T({ x: fx, y: 3.92, w: fw, h: 0.45, fontSize: 8.5, color: GREY }));
  s.addImage({ path: path.join(HERE, "assets", "trajectory-sitl-2026-11-07.png"), x: fx, y: 4.45, w: fw, h: 2.0, sizing: { type: "cover", w: fw, h: 2.0 }, altText: "Smooth trajectory through Gate B in the SITL GUI on the 7 Nov layout." });
  s.addText("Trajectory stack in our SITL GUI, 7 Nov layout.", T({ x: fx, y: 6.5, w: fw, h: 0.3, fontSize: 8.5, color: GREY }));
  s.addNotes(
`All followers use one planning core. The map is the only course input.
The lethal radius is 0.32 m: the prop reach plus 4 cm.
Rocks grow by the rangefinder cone. If the beam sees a rock, PX4 sees the floor rise 0.53 m and climbs.
Openings are straight legs. In 39 SITL flights the window was the closest pass in 21 before this rule.
Stop-and-go flew on the drone. It moves the setpoint only while the drone is within 0.3 m of it.
The trajectory stack computes a time-optimal speed profile. It flies by twist with feed-forward 0.55 s ahead. That is PX4's velocity-loop lag.
In the dark tunnel cuVSLAM stops. The follower continues on PX4's velocity for up to 4 s.
pathfly flies a route from an offline planner with the exact drone rectangle. It holds east, because the margins are 8 cm.`);
}

// ===================== 5. Approach: safety, localization, tools =====================
{
  const s = slide(5, "APPROACH 3/3", "Safety, localization and verification tools", "Approach");
  const cw = 2.95, g = 0.18;
  const xs = [0, 1, 2, 3].map((i) => L + i * (cw + g));
  head(s, xs[0], 1.25, cw, "5.1  Fail-closed guards");
  block(s, xs[0], 1.58, cw, 5.4, [
    ["Before Offboard:", "rest 2 s within 2 cm and 1°. PX4 and camera agree within 0.3 m. Else send nothing. Abort if Offboard starts > 0.5 m from the mark."],
    ["Start-up:", "an invalid map stops the node before it creates a publisher."],
    ["In flight:", "the line from drone to setpoint must be clear. Else freeze the reference."],
    ["Off path:", "> 0.35–0.5 m: hold for the pilot."],
    ["Stale pose:", "GROUNDED sends zero twists. A pose setpoint does not expire."],
    ["Landing:", "continue down on a stale pose. Else the drone climbs 0.27 m [S]."],
    ["Pilot:", "Offboard end = STOPPED. The kill switch always operates."],
    ]);

  head(s, xs[1], 1.25, cw, "5.2  DH Bailout (PX4 module)");
  block(s, xs[1], 1.58, cw, 2.6, [
    "Brake at 3 m/s², jerk 20 m/s³. Settle below 0.15 m/s. Descend 0.7 m/s; 0.35 m/s below 0.8 m.",
    "External mode 1 (nav_state 23). uORB only. No commander or MAVLink change.",
    "It re-publishes its setpoint after each kit sample. Flash: 708 bytes free.",
    "Host tests: 205,132 checks, 0 failures [M].",
    ]);
  table(s, xs[1], 3.5, [1.05, 0.95, 0.95], [
    ["SIH SITL [S]", "Stop", "Kit stream on"],
    ["DH Bailout", "0.37–0.39 m", "0.38–0.39 m"],
    ["Stock Land", "0.54–0.59 m", [red("1.22 m, no disarm")]],
    ]);
  s.addText("Flashed on skybeta-11. Not on the mode switch: organiser permission pending.", T({ x: xs[1], y: 4.5, w: cw, h: 0.7, fontSize: 11, color: GREY }));

  head(s, xs[2], 1.25, cw, "5.3  Localization");
  block(s, xs[2], 1.58, cw, 5.4, [
    ["cuVSLAM:", "2.3 % return-to-start drift on a 57 s walk [M]. Stereo-only; EKF2 fuses the IMU."],
    ["vo_guard:", "withholds stale (> 0.5 s), failed, > 10 m/s jump, > 720°/s spin, > 3° gyro disagreement in 0.5 s."],
    ["Lidar:", "Gauss-Newton point-to-plane scan match to the map. 0.6 cm rms vs PX4 [M]."],
    ["pose_fusion:", "EKF on the drift (tx, ty, yaw). It corrects setpoints, not EKF2. Target error 0.208 → 0.003 m [M]."],
    ["Latency:", "EKF2 rejects all vision at ≥ 0.18 s [S]. Measured 62–69 ms."],
    ]);

  head(s, xs[3], 1.25, cw, "5.4  Verification tools");
  block(s, xs[3], 1.58, cw, 5.4, [
    ["Own SITL:", "26k lines, no ROS or GPU. Runs our launch and the kit code unmodified. The kit's 149 tests pass on it."],
    ["Models:", "PX4 position, attitude, rate and allocation loops, ported. EKF2 delayed vision. cuVSLAM from 35 rays on the STEP mesh."],
    ["Referee:", "rules on ground truth: crossings, +5 s per contact."],
    ["Recorder:", "one bag per flight from arming. report.md per flight. ULog triage."],
    ["Tests:", "~800. Mutation tests: 17 of 17 caught. Independent AI review before each merge."],
    ]);
  head(s, L, 5.35, W, "5.5  Contracts and parameter control");
  const cc = [
    ["Contracts:", "frames.md, course.md, px4.md, ros-graph.md. If code disagrees with a contract, the code is wrong. A CI guard refuses a contract change without a changelog entry."],
    ["PX4 parameters:", "we save the FC parameters and firmware before and after each change, and compare them. One parameter change only: COM_DISARM_LAND 5 s, organiser approval (#40)."],
    ["Invariants:", "no Acro mode. No /fmu/in writes. No change to the organisers' PX4 safety settings or flight interface. No course literals in code."],
    ["Drone identity:", "connect only to skybeta-11. The lidar installer refuses any other host name."],
  ];
  const ccw = (W - 3 * g) / 4;
  cc.forEach(([h, d], i) => s.addText([{ text: h + " ", options: { bold: true, color: INK } }, { text: d, options: { color: GREY } }],
    T({ x: L + i * (ccw + g), y: 5.68, w: ccw, h: 1.25, fontSize: 10.5 })));
  s.addNotes(
`Every guard fails closed. If a check fails, the node sends nothing, and PX4 does not enter Offboard.
A pose setpoint never expires. So a stale state sends zero twists, not the last pose.
DH Bailout is our own PX4 flight mode. It runs on the flight controller. It stops in about half the distance of stock Land.
With the kit stream active, stock Land did not detect the landing. The stream reset the land detector.
The lidar and pose fusion stack is built and bench tested. It is not in the flown configuration.
Our simulator runs the organisers' code without changes. It found the dark-tunnel failure and the latency cliff before the drone did.`);
}

// ===================== 6. Results: evidence =====================
{
  pres.addSection({ title: "Results" });
  const s = slide(6, "RESULTS 1/2", "Flight, simulation and bench results", "Results");
  head(s, L, 1.25, 4.6, "6.1  Flown on skybeta-11 [F]");
  table(s, L, 1.58, [0.95, 1.45, 2.2], [
    ["Flight", "Task", "Result"],
    ["003042", "Climb to 0.73 m", [red("1.33 m: height origins differ")]],
    ["013018", "0.5 m box, 4 yaws", "Height 0.71–0.75 m. Drift 10–18 cm per yaw"],
    ["035510", "All primitives", "Square, land, relaunch, circle, land. Disarm 107 s"],
    ["081345", "Course, 0.3 m/s", [red("Contact: wall not in the map. Pilot kill")]],
    ["082135", "Course, 0.3 m/s", [red("Turned early at the tunnel south mouth. Kill at 79.57 s. No contact")]],
    ["Scored", "Course, 0.3 m/s", [red("Stick input ended each run. Crash after full throttle + yaw at 66.2 s")]],
    ]);

  const mx = L + 4.85, mw = 4.3;
  head(s, mx, 1.25, mw, "6.2  SITL, full course [S]");
  const runs6 = [["Stop-and-go, taped map", 102, "102 s"], ["Smooth 0.5 m/s, taped map", 76, "76 s"], ["Trajectory, plain poses", 59, "59 s"],
    ["Trajectory, twist 0.5 m/s", 42, "42 s"], ["Trajectory, twist 1.0 m/s", 30, "30 s"], ["Trajectory, window, 2 m/s", 32.3, "30.9–33.7 s"], ["Trajectory, Gate B, 0.8–1.0", 38.5, "38–39 s, 1–2 contacts"]];
  const lw6 = 1.95, bw6 = mw - lw6 - 0.05, sc6 = (bw6 - 0.95) / 110;
  runs6.forEach(([lab, v, txt], i) => {
    const y = 1.62 + i * 0.37;
    s.addText(lab, T({ x: mx, y, w: lw6, h: 0.3, fontSize: 10, color: GREY, valign: "middle" }));
    s.addShape(pres.shapes.RECTANGLE, { x: mx + lw6, y: y + 0.05, w: v * sc6, h: 0.2, fill: { color: i === 5 ? BLUE : (i === 6 ? C.accent1 : GREY) }, line: { type: "none" }, objectName: "bar " + lab });
    s.addText(txt, T({ x: mx + lw6 + v * sc6 + 0.06, y, w: 1.6, h: 0.3, fontSize: 10, color: i === 6 ? C.accent1 : INK, valign: "middle" }));
  });
  s.addText("Launch to pad. 0 contacts unless stated. Twist = velocity feed-forward.", T({ x: mx, y: 4.25, w: mw, h: 0.25, fontSize: 9.5, color: MUTE }));
  block(s, mx, 4.65, mw, 2.25, [
    "Taped map at 5391e34: window crossed within 8 cm of centre. Landed within 11 cm of the pad centre.",
    "Dark tunnel: tracking lost for 3.1–3.2 s. Coast 1.55 m in Offboard. No failsafe.",
    "Return home: 5 of 5 within 23 cm of the mark (published course). Follower: 0.11 m.",
    ]);

  head(s, L, 4.05, 4.6, "6.4  Tracking quality on skybeta-11 [F]");
  block(s, L, 4.38, 4.6, 2.2, [
    "013018: PX4 estimate vs odometry: median 1.3 cm, 95th percentile 3.3 cm, heading 0.6°.",
    "Hold without a yaw: within 7 cm. Height 0.71–0.75 m for a 0.73 m target.",
    "Scored-session crash: 3–8 cm from setpoint, no contact, until the pilot input at 66.2 s.",
  ]);

  const rx = mx + mw + 0.25, rw = R - rx;
  head(s, rx, 1.25, rw, "6.3  Planner bench [M]");
  table(s, rx, 1.58, [1.6, 0.75, 0.8], [
    ["Method", "Time", "Closest"],
    ["Waypoint steps", "30.0 s", "0.37 m"],
    ["Smoothed, one speed", "65.6 s", "0.41 m"],
    ["Speed profile + lead", "23.5 s", "0.35 m"],
    ["Speed profile only", "51.0 s", "0.41 m"],
    ["Lidar carrot", "54.8 s", "0.41 m"],
    ]);
  block(s, rx, 3.1, rw, 3.3, [
    "One safe speed must suit the slowest corner. The speed profile saves 22 %.",
    "PX4 trails a moving pose by v/kp. The lead removes this lag. At 10 Hz the lead touched in SITL, so it is off by default.",
    "Velocity feed-forward replaced the lead: 59 s became 30 s.",
    ]);
  head(s, rx, 4.75, rw, "6.5  Pre-event SITL, 15 layouts [S]");
  block(s, rx, 5.08, rw, 1.8, [
    "Tunnel blinded: 12 of 14 cleared. Error 2.4–7.6 cm rms. Launch to pad 27–34 s.",
    "With +50 ms latency: 19 of 20 cleared.",
  ]);
  s.addNotes(
`Add today's real-flight results here before the talk. Do not claim completion before it happens.
Only stop-and-go has flown the course. The smooth stacks have only flown in simulation.
The flown course runs show tracking within 3 to 8 cm. The failures came from the map and from stick input.
In SITL on the taped map at 5391e34, stop-and-go takes 102 s. The smooth follower takes 76 s. The SITL runs real PX4 code and models cuVSLAM on the course mesh.
The trajectory stack takes 30.9 to 33.7 s at 2 m/s on the 7 Nov map, with no contact.
Velocity feed-forward took the time from 59 s to 30 s. Plain poses lag by v over kp.`);
}

// ===================== 7. Results: generality and limits =====================
{
  const s = slide(7, "RESULTS 2/2", "Generality, declared course-specific items and open items", "Results");
  head(s, L, 1.25, 4.5, "7.1  Generality evidence");
  table(s, L, 1.58, [2.3, 1.15, 1.05], [
    ["Test", "Map planner", "Hand route"],
    ["Published map [M]", "1 / 1", "1 / 1"],
    ["Rotated, mirrored [M]", "2 / 2", "0 / 2"],
    ["Walls moved ≤ 5 cm [M]", "6 / 6", "0 / 6"],
    ["Random layouts [M]", "12 / 12", "0 / 12"],
    ["Boulders moved [S]", "20 / 20", "6 / 20"],
    ["Unseen layouts, PX4 SITL [S]", "8 / 9", "—"],
    ["Planted bugs caught [M]", "17 / 17", "—"],
    ]);
  block(s, L, 3.6, 4.5, 1.4, [
    "The map file is the only course input. A test searches our source for map coordinates and wall names.",
    "A wall across the window makes A* find Gate B. No code names Gate B.",
    "Goals come from the CAD geometry. The course frame comes from the rest on the launch mark.",
    "Constants are drone size, rates and margins, each with a unit.",
    ]);

  const mx = L + 4.8, mw = 3.9;
  head(s, mx, 1.25, mw, "7.2  Course-specific (declared)");
  block(s, mx, 1.58, mw, 1.9, [
    "The map file, with the 8 Nov tape corrections.",
    "The flight height 1.23 m: the middle of the taped window opening, a map value.",
    "pathfly: precomputed route and fixed east heading. On a new course, run plan_path.py again.",
    "The launch mark position and heading, by tape.",
    ]);
  head(s, mx, 2.95, mw, "7.3  Limits");
  block(s, mx, 3.28, mw, 1.7, [
    "The map is trusted. No sensor finds walls in flight.",
    "A 5 cm wall shift made a tight corner: 61 → 81 s.",
    "The contract schema has no checkpoint-gate field.",
    "CI runs about 6 % of the tests. The other suites run locally.",
    ]);

  const rx = mx + mw + 0.3, rw = R - rx;
  head(s, rx, 1.25, rw, "7.4  Not yet flown");
  block(s, rx, 1.58, rw, 2.2, [
    "Smooth follower, trajectory stack, return home, pathfly.",
    "yaw_comp feed-forward for the body-fixed push.",
    "Heading A/B. A fixed heading starved cuVSLAM at the north wall [S]. Real tunnel run: heading drift < 1° over 4 turns, ~2.5° by the tunnel [F]. pathfly holds east for 8 cm margins.",
    "DH Bailout on the mode switch.",
    ]);
  head(s, rx, 3.25, rw, "7.5  Credits (rule 9.1)");
  s.addText("skybeta starter stack · NVIDIA Isaac ROS cuVSLAM · Luxonis DepthAI, depthai-ros · PX4, QGroundControl · ROS 2, rosbag2, MCAP · Gazebo · Nav2 inflation cost · Pham & Pham TOPP (T-RO 2018) · NumPy, PyYAML · AI coding tools (rule 9.2): we can explain each line that flies.",
    T({ x: rx, y: 3.58, w: rw, h: 1.5, fontSize: 10.5, color: GREY }));
  head(s, L, 5.2, W, "7.6  Procedure for a new course");
  const steps = [
    "Get the course STEP file.",
    "Run step_to_map.py. It stops if a part is ambiguous.",
    "Measure the built course with tape. Correct the map file.",
    "Tape the launch mark and its heading on the floor.",
    "Run the dry run and our SITL on the new map.",
    "Fly ladder step 0 (stop-and-go, 0.3 m/s).",
  ];
  const sw = (W - 5 * 0.2) / 6;
  steps.forEach((t, i) => {
    const x = L + i * (sw + 0.2);
    s.addText(String(i + 1), T({ x, y: 5.55, w: 0.3, h: 0.3, fontSize: 16, bold: true, color: BLUE }));
    s.addText(t, T({ x: x + 0.32, y: 5.58, w: sw - 0.32, h: 1.2, fontSize: 10.5, color: GREY }));
  });
  s.addNotes(
`The planner holds no layout knowledge. It flew all 12 random layouts. The hand route planned none of them.
On a new course we change one map file and tape the launch mark. The code stays the same.
These items are course-specific. We say them first: the map file, the flight height from the taped window, pathfly's route and heading. The corridor goal from after the crash was removed in 5391e34.
The main limit: the map is trusted. Nothing senses walls in flight. The lidar localizer is the next step for this.
Open items: the smooth stacks, the return and pathfly have not flown. The heading question needs a flown A/B.`);
}

(async () => {
  await pres.writeFile({ fileName: OUT });
  if (process.env.APPLY_THEME) await require(process.env.APPLY_THEME).applyTheme(OUT, THEME);
  console.log("wrote", OUT);
})().catch((e) => { console.error(e); process.exit(1); });
