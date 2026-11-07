# OAK-D S2 on-device stereo depth for indoor drone obstacle detection (DepthAI v3), and depth-to-obstacle processing

Scope: OAK-D-S2-AF (RVC2 / Myriad X, 2x OV9282 1280x800 global-shutter mono, 7.5 cm baseline, no IR projector), DepthAI v3 (3.10.0), Jetson Orin Nano 8 GB host that already runs cuVSLAM on the raw 1280x800 / 30 fps stereo stream, Pixhawk 6C / PX4, 7-inch quad, indoor, GPS-denied. Researched 2026-10-25.

Notation used below:
- "[v2-only]" = the statement applies only to DepthAI v2 (or v2-era doc text).
- "[Pro-only]" = applies only to OAK-D Pro (IR dot projector / flood LED).
- "[measured]" = measured on the owner's unit (OAK-D-S2-AF, depthai 3.10.0) and recorded in this repo, not a Luxonis claim.
- "[snippet]" = seen only in a search-engine excerpt; I could not load the full page, so treat it as less certain.

Two conventions that trip people up, verified on this unit:
- In v3 the stereo confidence threshold is inverted relative to v2: 255 = most confident, and a HIGHER threshold keeps FEWER pixels. Some Luxonis pages still carry the v2 wording (details in section 2).
- v3 preset names: DEFAULT, ROBOTICS, HIGH_DETAIL, FACE, FAST_ACCURACY (= ACCURACY), FAST_DENSITY (= DENSITY). The v2 names HIGH_DENSITY / HIGH_ACCURACY are not in 3.10.0.

---

## 1. Depth accuracy and range for the 7.5 cm baseline (MinZ, MaxZ, error vs distance, subpixel, 800P vs 400P)

### Takeaway
With this unit's calibration, MinZ is about 63 cm at 800P (normal disparity). It drops to about 32 cm at 400P or with extended disparity, and to the 19.6 cm lens-focus floor with both. Luxonis rates median error at under 2 % below 4 m, under 4 % from 4 to 7 m and under 6 % from 7 to 10 m (800P, subpixel, textured target). For obstacle avoidance, plan on roughly 0.3 to 5 m of trustworthy range at 400P, and only "something is there" information beyond about 7 m.

### Cited Findings
- **Formula.** Depth = focal_length_px × baseline / disparity_px. Disparity search is 0 to 95 px in normal mode and 0 to 190 px with extended disparity. Halving resolution (800P to 400P) halves focal length in pixels and so halves MinZ. — [Luxonis: Configuring Stereo Depth](https://docs.luxonis.com/hardware/platform/depth/configuring-stereo-depth/); [Luxonis v2 StereoDepth node](https://docs.luxonis.com/software/depthai-components/nodes/stereo_depth/)
- **Luxonis worked example (original OAK-D, HFOV 71.9°, f = 882.5 px at 800P).** MinZ is 882.5 × 7.5 / 95 = 69.7 cm normal and 34.8 cm extended. Combining 400P with extended is "limited by the 19.6 cm focal distance" of the fixed-focus mono lenses. — [Configuring Stereo Depth](https://docs.luxonis.com/hardware/platform/depth/configuring-stereo-depth/); [v2 StereoDepth](https://docs.luxonis.com/software/depthai-components/nodes/stereo_depth/)
- **OAK-D S2 product page.** "Ideal depth range ~80 cm to 12 m"; MinZ ~80 cm (800P), ~40 cm (400P or 800P + extended), ~20 cm (400P + extended); HFOV 80°, VFOV 55° for the mono pair. — [OAK-D S2 product page](https://docs.luxonis.com/hardware/products/OAK-D%20S2). The repo's spec sheet notes that these MinZ numbers are product-family boilerplate, printed identically on the Pro, S2 PoE and Lite pages. — [camera/docs/oak-d-s2-specs.md](../../camera/docs/oak-d-s2-specs.md)
- **[measured] This unit's calibration.**
  - Focal length: CAM_B fx = 799.13 px at 1280x800 and 399.6 px at 640x400.
  - Baseline: 74.97 mm.
  - MinZ from the formula: **63.1 cm** (800P), **31.5 cm** (800P + extended, or 400P), and 15.8 cm (400P + extended), which is lens-limited to 19.6 cm.
  - Luxonis' approximate MaxZ formula gives 35.5 m; the 1 px-disparity distance is 59.9 m.
  - Source: [camera/docs/oak-d-s2-specs.md](../../camera/docs/oak-d-s2-specs.md)
- **MaxZ approximation.** Dm = (baseline/2) × tan((90 − HFOV/HPixels) × π/180), which gives 38.25 m for the original OAK-D. — [Configuring Stereo Depth](https://docs.luxonis.com/hardware/platform/depth/configuring-stereo-depth/)
- **Published accuracy, "800P, 75 mm baseline" OAKs (the group that includes the S2).** Median absolute error is below 2 % under 4 m, below 4 % at 4 to 7 m, and below 6 % at 7 to 10.5 m.
  - Method: textured random-noise board perpendicular to the camera, highest resolution, subpixel with 5 fractional bits, no post-processing filters, top and bottom 5 % outliers removed, median of 10 runs.
  - The page notes a dot projector "makes no difference" under these near-ideal texture conditions.
  - Source: [Luxonis Depth Accuracy](https://docs.luxonis.com/hardware/platform/depth/depth-accuracy/)
- **The measured device was an OAK-D Pro, not an S2.** It has the same OV9282 pair and baseline, but a 940 nm notch filter instead of IR-cut. — [Depth Accuracy](https://docs.luxonis.com/hardware/platform/depth/depth-accuracy/) as summarised in [camera/docs/oak-d-s2-specs.md](../../camera/docs/oak-d-s2-specs.md)
- **Close-range figure.** At about 70 cm with good texture, error is "below 2 %", roughly ±1.5 cm. "Accuracy falls exponentially with distance; keep objects close to MinZ." — [Configuring Stereo Depth](https://docs.luxonis.com/hardware/platform/depth/configuring-stereo-depth/)
- **Subpixel.** Adds 3, 4 or 5 fractional bits, giving 754, 1506 or 3010 depth levels instead of 96. It "improves precision… especially useful for long range" but uses more hardware. "All profile presets have subpixel ON… For high FPS, turn subpixel OFF." — [v3 StereoDepth node](https://docs.luxonis.com/software-v3/depthai/depthai-components/nodes/stereo_depth/); [Configuring Stereo Depth](https://docs.luxonis.com/hardware/platform/depth/configuring-stereo-depth/)
- **Resolution trade-off.** "Lower resolution lowers MinZ but also reduces depth accuracy in cm, since there are fewer pixels." Decimation factor 2 on an 800P input gives 640x400 output. — [Configuring Stereo Depth](https://docs.luxonis.com/hardware/platform/depth/configuring-stereo-depth/)
- **Disparity shift.** Raises MinZ by shifting the search window. Example: a 30 px shift searches 2.2 m down to 50 cm with "theoretically below 5 mm depth error", but MaxZ shrinks much faster than MinZ. It does not work with RGB alignment. — [Configuring Stereo Depth](https://docs.luxonis.com/hardware/platform/depth/configuring-stereo-depth/); [v2 StereoDepth](https://docs.luxonis.com/software/depthai-components/nodes/stereo_depth/)
- **Near-edge blind band.** A vertical band on the outer edge of each mono image can never get depth. Its width is B = W × Dv / D; for example, about 33 px at 400P for an object at 100 cm. — [v2 StereoDepth](https://docs.luxonis.com/software/depthai-components/nodes/stereo_depth/)
- **[measured] Disparity encoding.** The `depth` output is uint16 millimetres with 0 = invalid. The disparity output is RAW16 scaled to `getMaxDisparity()`, not px × 8192/95. Use the `depth` output for metric values. — [camera/docs/depthai-v3-cheatsheet.md](../../camera/docs/depthai-v3-cheatsheet.md)

### Inferences
- **Derived depth-quantisation table (unit's calibration).** f·B = 59.9 px·m at 800P and 29.96 px·m at 400P; dZ = Z² × Δd / (f·B).

  | Distance | 800P, 1 px | 400P, 1 px | 400P, 1/8 px (3-bit subpixel) |
  |---|---|---|---|
  | 1 m | 1.7 cm | 3.3 cm | 0.4 cm |
  | 2 m | 6.7 cm | 13 cm | 1.7 cm |
  | 4 m | 27 cm | 53 cm | 6.7 cm |
  | 6 m | 60 cm | 1.2 m | 15 cm |

  Real SGM subpixel accuracy is worse than the 1/8 px quantisation step, so read Luxonis' 2/4/6 % bands as the practical figure. At 400P, expect errors roughly twice the 800P bands at the same distance.
- **Range for avoidance is set by detection reliability, not by MaxZ.** Indoors at drone speeds, the useful horizon is about 4 to 5 m. Beyond that, depth steps and the 4 to 6 % error make small obstacles unreliable.
- **400P is the better operating point.** MinZ ~32 cm keeps obstacles visible inside about one airframe length; 800P blinds the first ~63 cm. 400P is also far cheaper on RVC2 (section 2). Add extended disparity only if obstacles closer than ~32 cm must be seen; it about doubles stereo time.
- **Objects closer than MinZ are not guaranteed to come back as zeros.** Their true disparity is outside the search range, so the matcher either invalidates them or returns a wrong (usually farther) match. Treat "nothing seen" at very close range as unknown, and keep the vehicle from closing below MinZ using map memory and VIO. This is an inference from SGM behaviour; I found no Luxonis statement on it.

### Gaps
- Luxonis publishes no S2-specific accuracy curve, no fill-rate figures, and no accuracy data at 400P or on low-texture targets. All published bands are for textured boards.
- No Luxonis figure covers accuracy degradation with subpixel at 3 bits versus 5 bits. The accuracy test used 5 bits, while the ROBOTICS preset uses 3 bits.

---

## 2. Which StereoDepth settings matter for obstacle avoidance, and their FPS/latency cost on RVC2

### Takeaway
Luxonis' explicit recommendation for navigation and obstacle detection is the **ROBOTICS** preset. It turns the temporal filter off to avoid motion smearing and uses LR-check, 3-bit subpixel, a 7x7 median, decimation ×2, spatial hole-filling and a 0 to 10 m threshold.

On RVC2 the stereo cost scales strongly with resolution. LR-check plus subpixel takes about 10 ms at 400P but about 30 ms at 800P, plus rectification. So on-device depth should run at 400P input, not 800P, when it has to coexist with a 30 fps raw stream.

For thin obstacles, the preset's 7x7 median and decimation ×2 are the settings to reconsider (section 4).

### Cited Findings
- **Preset purposes (RVC2).** DEFAULT = general purpose; ROBOTICS = "navigation and obstacle detection, no temporal filter"; FACE = short-range with extended disparity; HIGH_DETAIL = fine detail, more noise; FAST_DENSITY = high fill-rate, faster; FAST_ACCURACY = higher accuracy, lower fill-rate, faster. — [Configuring Stereo Depth](https://docs.luxonis.com/hardware/platform/depth/configuring-stereo-depth/)
- **v3 preset table.** Robotics: "Navigation, obstacle detection", output 1/2 input resolution, Motion Blur False. Fast presets: full input resolution, range 0 to 65 m. "By default, all profile presets have subpixel ON… leads to lower FPS. For high FPS, turn subpixel OFF." HIGH_DETAIL and FACE turn on extended disparity. — [v3 StereoDepth node](https://docs.luxonis.com/software-v3/depthai/depthai-components/nodes/stereo_depth/)
- **[measured] Presets as dumped from `initialConfig` on this unit (3.10.0).**

  | Preset | Subpixel (bits) | Extended | LR-check | Confidence | Median | Decimation | Spatial / temporal / speckle | Threshold |
  |---|---|---|---|---|---|---|---|---|
  | ROBOTICS | 3 | off | on | 15 | 7x7 | 2 | on / off / on | 0 to 10,000 mm |
  | DEFAULT | 3 | off | on | 15 | 7x7 | 2 | on / on / on | 0 to 15,000 mm |
  | HIGH_DETAIL | 5 | on | on | 55 | off | 2 | on / on / on | — |
  | FACE | 5 | on | on | — | off | 2 | — | 30 to 3,000 mm |
  | FAST_ACCURACY = ACCURACY | 5 | off | on | 55 | off | 1 | all off | 0 to 65,535 mm |
  | FAST_DENSITY = DENSITY | 5 | off | on | 15 | off | 1 | all off | 0 to 65,535 mm |

  Source: [camera/docs/depthai-v3-cheatsheet.md](../../camera/docs/depthai-v3-cheatsheet.md)
- **Luxonis' parameter values per preset.**
  - Spatial filter: DEFAULT delta 3, hole-filling radius 1; ROBOTICS delta 20, hole-filling radius 2; off in FAST presets.
  - Temporal filter: alpha 0.5 in DEFAULT, FACE and HIGH_DETAIL; off in ROBOTICS and FAST presets.
  - Speckle range: 200 in most presets.
  - LR-check threshold: 10, or 5 for HIGH_DETAIL and FAST_ACCURACY.
  - Decimation: factor 2 in DEFAULT, ROBOTICS, FACE and HIGH_DETAIL.
  - Source: [Configuring Stereo Depth](https://docs.luxonis.com/hardware/platform/depth/configuring-stereo-depth/)
- **Conflicting ROBOTICS threshold.** The v3 node page lists Robotics range 0 to 15 m and Default 0 to 10 m. The configuring page and the unit's own dump say ROBOTICS 0 to 10 m and DEFAULT 0 to 15 m. — [v3 StereoDepth node](https://docs.luxonis.com/software-v3/depthai/depthai-components/nodes/stereo_depth/); contradicted by [Configuring Stereo Depth](https://docs.luxonis.com/hardware/platform/depth/configuring-stereo-depth/) and [measured](../../camera/docs/depthai-v3-cheatsheet.md)
- **Temporal filter.** "can cause motion blur or smearing and suits static scenes best", which is why ROBOTICS disables it. — [Configuring Stereo Depth](https://docs.luxonis.com/hardware/platform/depth/configuring-stereo-depth/)
- **Confidence threshold, v3 semantics.** StereoDepthConfig says "Disparities with confidence value over this threshold are accepted", with a range of 0 to 255. — [v3 StereoDepthConfig](https://docs.luxonis.com/software-v3/depthai/depthai-components/messages/stereo_depth_config/)
  - **[measured]** On this unit, 255 = most confident; a higher threshold means fewer valid pixels; 255 always empties the map. Presets use 15 or 55.
  - Dim scene: 70 to 74 % invalid at threshold 0, 85 % at 15, 95 % at 55.
  - Bright scene: 21 to 24 % invalid at 0, 38 to 40 % at 15, 72 to 74 % at 55.
  - Source: [camera/docs/depthai-v3-cheatsheet.md](../../camera/docs/depthai-v3-cheatsheet.md)
  - **[v2-only]** The Luxonis "configuring" page and the v2 node page still say "0 means highest confidence… pixels above the threshold are invalidated", and that the HIGH_DENSITY / HIGH_ACCURACY presets set 245 / 200. — [Configuring Stereo Depth](https://docs.luxonis.com/hardware/platform/depth/configuring-stereo-depth/); [v2 StereoDepth](https://docs.luxonis.com/software/depthai-components/nodes/stereo_depth/). These pages contradict v3.
- **[measured] No node-level setters in 3.10.0.** There is no `setMedianFilter` or `setConfidenceThreshold` on the StereoDepth node; use `stereo.initialConfig.setMedianFilter(...)` and `initialConfig.setConfidenceThreshold(...)`. Runtime changes go through `inputConfig`. — [camera/docs/depthai-v3-cheatsheet.md](../../camera/docs/depthai-v3-cheatsheet.md)
- **Subpixel and median interactions.**
  - RVC2: median filtering is supported only with 3 fractional bits.
  - [measured] Subpixel + extended on a preset logs "Maximum disparity value '1520' exceeds the maximum supported '1024' by median filter. Disabling median filter!"
  - The old docstring "subpixel incompatible with extended disparity" is stale: HIGH_DETAIL and FACE enable both and run on RVC2.
  - Sources: [v3 StereoDepth node](https://docs.luxonis.com/software-v3/depthai/depthai-components/nodes/stereo_depth/); [camera/docs/depthai-v3-cheatsheet.md](../../camera/docs/depthai-v3-cheatsheet.md)
- **Other knobs in StereoDepthConfig.**
  - `numInvalidateEdgePixels` (RVC2).
  - `brightnessFilter` min/max: invalidates very dark or saturated pixels; Luxonis says it is useful for bright light sources and rectification-edge artifacts.
  - `decimationFilter`: factor 1 to 4, mode PIXEL_SKIPPING, NON_ZERO_MEDIAN or NON_ZERO_MEAN.
  - `thresholdFilter` min/max.
  - `speckleFilter` speckleRange / differenceThreshold.
  - `filteringOrder`: 5 entries from DECIMATION, SPECKLE, MEDIAN, SPATIAL, TEMPORAL.
  - `enableCompanding`: 176-disparity sparse search, RVC2, an alternative to extended disparity.
  - `censusTransform.kernelSize`: AUTO / 5x5 / 7x7 / 7x9.
  - Hole-filling and adaptive-median are **RVC4-only**.
  - Sources: [v3 StereoDepthConfig](https://docs.luxonis.com/software-v3/depthai/depthai-components/messages/stereo_depth_config/); [Configuring Stereo Depth](https://docs.luxonis.com/hardware/platform/depth/configuring-stereo-depth/)
- **Border advice.** Invalidate about 3 px around the depth-image border and enable the brightness filter against edge artifacts. — [Configuring Stereo Depth](https://docs.luxonis.com/hardware/platform/depth/configuring-stereo-depth/)
- **RVC2 stereo-matching time per mode** (v2-era doc table, same RVC2 hardware block; ms):

  | Mode | 400P | 720P | 800P |
  |---|---|---|---|
  | No LR, no extended, no subpixel | 3.4 | 7.4 | 8.1 |
  | Subpixel | 6.2 | 21.7 | 23.9 |
  | Extended | 8.5 | 21.5 | 23.5 |
  | LR-check | 7.4 | 18 | 19.8 |
  | **LR-check + subpixel** | **10.1** | 27.6 | **30.5** |
  | LR-check + extended | 16.7 | 43.7 | 48.3 |
  | LR-check + extended + subpixel | 20.4 | 56 | 62 |

  - Rectification adds 1.5 / 5 / 6 ms at 400P / 720P / 800P.
  - Median filtering adds 0.8 ms at 400P and 1.8 ms at 720P/800P; kernel size does not change it.
  - The page converts LR + extended + subpixel at 800P to "~16 FPS" (1 s / 62 ms).
  - Source: [v2 StereoDepth node](https://docs.luxonis.com/software/depthai-components/nodes/stereo_depth/)
- **Trace-log example.** "Stereo took 12.27 ms", "Stereo rectification took 2.69 ms", "Stereo post processing (total) took 0.93 ms". — [Luxonis Optimizing FPS & Latency](https://docs.luxonis.com/software/depthai/optimizing/)
- **Filter cost.** "Filters cost performance at high resolution (1MP), so use 400P and/or decimation. Filters add latency." If you hit SHAVE or memory allocation errors, try `setPostProcessingHardwareResources(3, 3)`. Extended disparity "runs the pipeline twice". — [Configuring Stereo Depth](https://docs.luxonis.com/hardware/platform/depth/configuring-stereo-depth/)
- **Node throughput limit.** The stereo node can process up to 400 FPS on 256x256 inputs; camera FPS normally limits output. — [v3 StereoDepth node](https://docs.luxonis.com/software-v3/depthai/depthai-components/nodes/stereo_depth/)
- **Power.** The S2 stereo-depth subsystem uses up to 0.5 W. — [OAK-D S2 product page](https://docs.luxonis.com/hardware/products/OAK-D%20S2)
- **[v2-only] Luxonis forum measurement.** A user measured 400P LR-check + subpixel at about 24 ms frame-to-host (80 FPS); a Luxonis engineer got about 21 ms at 60.3 FPS. [snippet] — [Luxonis forum thread 957](https://discuss.luxonis.com/d/957-impact-of-latency-on-practical-applications/27)

### Inferences
- **Recommended starting configuration for obstacle detection** (untested on this unit at these exact settings):
  - Preset ROBOTICS, with the stereo input at 640x400 and 30 fps.
  - LR-check on, 3-bit subpixel, extended disparity off. Turn extended on only if obstacles closer than ~32 cm matter; stereo time goes from ~10 to ~20 ms.
  - Decimation 1, to keep 640x400 and thin objects; the preset's ×2 would give 320x200.
  - Median 5x5 or 3x3 instead of 7x7, for thin objects.
  - Temporal off. Spatial on with small hole-fill. Speckle on.
  - Threshold filter about 200 to 8,000 mm.
  - `numInvalidateEdgePixels` about 3; brightness filter max about 250 to drop saturated light fixtures.
  - Confidence 15 (v3 semantics). Raising it toward 55 trades fill-rate for fewer false matches; the [measured] fill-rate drop is steep in dim scenes.
- **At 800P input,** LR-check + subpixel (~30.5 ms) plus rectification (~6 ms) is right at or above the 33 ms frame budget. Expect frame drops or added queueing at 30 fps. Avoid 800P on-device stereo when the same device also streams raw 800P to the host.
- **FAST_DENSITY / FAST_ACCURACY** skip all post-processing and output full resolution. They are useful baselines for measuring raw fill-rate, but give noisier maps for a planner.

### Gaps
- No Luxonis-published end-to-end FPS table per v3 preset at 800P versus 400P could be loaded. A search excerpt claimed roughly 16 FPS at 800P for DEFAULT and ROBOTICS and 30 to 34 FPS at 400P, but the fetched page did not show these figures, so they are unverified [snippet]. Measure on the unit with the latency method in section 7.
- The latency table above comes from v2-era docs. I found no v3-specific re-benchmark, although the RVC2 hardware block is the same.
- The exact semantics of `speckleRange` (blob size versus disparity difference) are not documented in the v3 reference.

---

## 3. Getting depth from the same pipeline as the cuVSLAM raw stream (one process), USB bandwidth, and the host-GPU alternative

### Takeaway
DepthAI v3 lets one pipeline both stream raw 1280x800 mono frames to the host and feed an on-device StereoDepth node.
- A Camera node can serve several `requestOutput()` calls, and one output can link to several consumers.
- StereoDepth also exposes `syncedLeft/Right` passthroughs and `rectifiedLeft/Right`.

USB3 has ample headroom: raw L+R at 800P/30 fps is about 0.49 Gb/s and 400P depth adds about 0.12 Gb/s, against about 2.5 Gb/s usable. USB2 does not.

Host-side GPU stereo on the Orin Nano is possible but competes with cuVSLAM for the same 1024-core GPU:
- ESS DNN reaches 34.8 fps (576p) or 82 fps (Light, 288p) node-only.
- VPI stereo would be CUDA-only, because Orin Nano has no PVA.

On-device RVC2 stereo is the lower-risk default.

### Cited Findings
- **v3 Camera node.**
  - "`ColorCamera` and `MonoCamera` are deprecated in favour of the new `Camera` node"; replace `.out` with `requestOutput`.
  - "Request multiple outputs simply by calling `requestOutput` again"; one output can feed two consumers (shown with `preview.link(manip1…)` and `preview.link(manip2…)`).
  - XLink nodes are created automatically; host queues come from `output.createOutputQueue()`.
  - Source: [Luxonis v2→v3 porting guide](https://docs.luxonis.com/software-v3/depthai/tutorials/v2-vs-v3/)
- **StereoDepth outputs.** depth, disparity, `rectifiedLeft/Right` (RAW8), `syncedLeft/Right` ("passthrough of the left and right inputs"), and `confidenceMap`. — [v3 StereoDepth node](https://docs.luxonis.com/software-v3/depthai/depthai-components/nodes/stereo_depth/)
- **Official v3 VIO example.** Basalt VIO builds two Camera nodes with `sensorFps=60` and links `requestOutput((640, 400))` directly into the VIO node. VSLAM host nodes "remain in early access". — [Luxonis Basalt VIO example](https://docs.luxonis.com/software-v3/depthai/examples/vslam/basalt_vio)
- **[measured] Repo pattern.** The existing depth test already uses `left = Camera.build(CAM_B)`, `left.requestOutput(size, fps=…)` and `StereoDepth.build(left=…, right=…, presetMode=…)`. — [camera/02_stereo_depth.py](../../camera/02_stereo_depth.py)
- **[measured] One process per device.** The S2 is USB-boot; every host process re-boots the camera, and only one process can hold it. — [camera/docs/oak-d-s2-specs.md](../../camera/docs/oak-d-s2-specs.md)
- **Raw bandwidth figures.**
  - 800P mono at 30 fps = 246 Mbps; 800P depth at 30 fps = 492 Mbps.
  - Measured time-to-host: 800P mono at 120 fps = 24.5 ms (442 to 482 Mbps); 400P at 120 fps = 7.5 ms; 400P mono at 90 fps = 8 ms over USB.
  - 10 Gbps needs a Gen-2 cable and `maxUsbSpeed=dai.UsbSpeed.SUPER_PLUS`.
  - Source: [Luxonis Optimizing FPS & Latency](https://docs.luxonis.com/software/depthai/optimizing/)
- **[measured] Link speed.** The unit enumerates as USB `SUPER` (5 Gbps class). Once in five back-to-back boots it came up as USB2 `HIGH` (480 Mbps). Luxonis' USB guide puts a healthy USB3 downlink at about 2.5 Gbps. — [camera/docs/oak-d-s2-specs.md](../../camera/docs/oak-d-s2-specs.md)
- **No neural or GPU stereo on RVC2.** The v3 Depth node's NEURAL, NEURAL_ASSISTED_STEREO and GPU_STEREO backends are RVC4-only; on RVC2, AUTO uses ToF if present, otherwise StereoDepth. — [v3 Depth node](https://docs.luxonis.com/software-v3/depthai/depthai-components/nodes/depth). [measured] `device.isGpuStereoSupported()` and `isNeuralDepthSupported()` return False on this unit. — [camera/docs/depthai-v3-cheatsheet.md](../../camera/docs/depthai-v3-cheatsheet.md)
- **Orin Nano vs Orin NX accelerators.** Orin Nano 8GB has a 1024-core Ampere GPU with 32 Tensor Cores, **no DLA, no PVA**, 68 GB/s memory bandwidth and a 7 to 15 W envelope. Orin NX 8GB has 1x DLA, 1x PVA v2 and 102.4 GB/s. — [Connect Tech Orin module comparison](https://connecttech.com/orin-module-comparison/)
- **VPI backends.**
  - PVA is "a processor in Jetson AGX Orin, AGX Thor and Orin NX devices"; OFA is "a specialized processor in the new Jetson AGX Orin and Thor devices". Orin Nano is not mentioned. — [NVIDIA VPI 4.0.5 architecture](https://docs.nvidia.com/vpi/4.0.5/architecture.html)
  - The CUDA backend implements SGM, and OFA runs "an advanced SGM version". — [NVIDIA VPI stereo disparity](https://docs.nvidia.com/vpi/algo_stereo_disparity.html)
  - The CUDA backend clamps maxDisparity to 64, 128 or 256; OFA requires 128 or 256. [snippet] — [VPI StereoDisparityEstimator reference](https://docs.nvidia.com/vpi/4.0.5/group__VPI__StereoDisparityEstimator.html)
- **Isaac ROS ESS (DNN stereo), node-level FPS.**

  | Model | Platform | FPS | Latency at 30 Hz |
  |---|---|---|---|
  | ESS Full, 576p | Orin Nano Super 8GB | 34.8 | 30 ms |
  | ESS Full, 576p | AGX Orin | 66.5 | 17 ms |
  | ESS Light, 288p | Orin Nano | 82.2 | 13 ms |

  - The DNN stereo nodes were migrated from NITROS to rosidl::Buffer (2026-09-21); the package is ROS 2-based.
  - A forum user on Orin Nano 8GB + HAWK reported disparity fps "much lower than on Orin", without numbers [snippet].
  - Sources: [isaac_ros_dnn_stereo_depth](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_stereo_depth); [NVIDIA forum](https://forums.developer.nvidia.com/t/nitros-negotiation-failed-in-isaac-ros-dnn-stereo-depth/290890)

### Inferences
- **Recommended pipeline shape** (sketch, verify on device):

  ```python
  left  = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=30)
  right = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=30)
  L800 = left.requestOutput((1280, 800), fps=30)   # -> host queue for cuVSLAM (as today)
  R800 = right.requestOutput((1280, 800), fps=30)
  L400 = left.requestOutput((640, 400), fps=30)    # -> on-device StereoDepth
  R400 = right.requestOutput((640, 400), fps=30)
  stereo = p.create(dai.node.StereoDepth).build(left=L400, right=R400,
                                                presetMode=dai.node.StereoDepth.PresetMode.ROBOTICS)
  qL, qR = L800.createOutputQueue(maxSize=4, blocking=False), R800.createOutputQueue(maxSize=4, blocking=False)
  qD = stereo.depth.createOutputQueue(maxSize=2, blocking=False)
  ```

  - If two sizes per mono camera turn out not to be supported on RVC2, there are two fallbacks:
    - Feed StereoDepth at 800P with decimation ×2. This costs about 30 ms of stereo time (section 2) and MinZ 63 cm.
    - Link the single 800P output to both the host queue and StereoDepth.
  - Both images come from the same exposure, so depth frames can be paired with cuVSLAM poses by sequence number or timestamp.
- **Bandwidth budget.** L+R at 800P (0.49 Gb/s) + 400P depth (0.12) + optional 400P confidence (0.06) ≈ 0.67 Gb/s. That is well within USB3, but it exceeds USB2 (0.48 Gb/s theoretical). The existing "refuse to fly unless `getUsbSpeed()==SUPER`" rule should also cover the depth stream.
- **Host-GPU stereo is a fallback, not the default.**
  - Running ESS or VPI-CUDA SGM on the Orin Nano would take GPU time from cuVSLAM on the same 1024-core GPU; there is no PVA to offload to.
  - ESS also needs a ROS 2 / Isaac ROS stack and resizing to 576p or 288p.
  - It is worth evaluating only if RVC2 depth on low-texture competition surfaces proves inadequate, since ESS-style DNN stereo usually fills textureless regions better than SGM. That last point is an inference; I found no head-to-head on these surfaces.
  - Keep the on-device stereo at 400P to leave RVC2 headroom.

### Gaps
- I found no Luxonis statement or example confirming that a single RVC2 mono Camera node can serve both a 1280x800 and a 640x400 output at the same time. The porting guide says multiple `requestOutput` calls are allowed in general. Needs a 1-minute bench test.
- There is no published VPI CUDA stereo timing for Orin Nano at 1280x800 or 640x400; the VPI performance tables list only AGX Orin and Thor. There is also no ESS figure measured while cuVSLAM runs concurrently.
- Whether OFA exists on Orin Nano is not stated in the VPI pages I could load. Absence from the OFA device list suggests it does not.

---

## 4. Thin obstacles and textureless surfaces without an IR projector: artifacts and mitigations

### Takeaway
The S2 is purely passive, with IR-cut filters and no projector or flood LED.
- Plain walls, uniform floors, foam pads and evenly painted boxes produce holes, not wrong depth. With the v3 confidence threshold and LR-check those pixels become 0.
- Thin vertical poles are detectable when wide enough in pixels. They are easily erased by the presets' decimation ×2 and 7x7 median, and fattened at their edges by window-based matching.
- Horizontal thin objects (ropes, bars, wires) parallel to the 7.5 cm horizontal baseline are fundamentally unreliable for any horizontal stereo pair.

Design the obstacle layer so that "invalid" means unknown, not free.

### Cited Findings
- **Passive stereo and texture.** "Passive stereo depth perception may not work well with featureless surfaces like walls or ceilings, as disparity matching relies on distinctive points or features." Active stereo (dot projector) "can resolve both texture and lighting issues" — available only on Pro models. — [Luxonis Depth Perception](https://docs.luxonis.com/hardware/platform/features/depth/); [Configuring Stereo Depth](https://docs.luxonis.com/hardware/platform/depth/configuring-stereo-depth/)
- **S2 hardware.** "Dot projector: No", "Infrared: No". [measured/spec] The stereo lenses have a 650 nm IR-cut filter, so external IR illuminators would not help either. — [OAK-D S2 product page](https://docs.luxonis.com/hardware/products/OAK-D%20S2); [camera/docs/oak-d-s2-specs.md](../../camera/docs/oak-d-s2-specs.md)
- **[Pro-only] OAK-D Pro.** Adds an IR laser dot projector (up to 1 W) and an IR flood LED. Its stereo cameras use 940 nm notch filters. The projector is rated only for 10 to 60 °C. — [camera/docs/oak-d-s2-specs.md](../../camera/docs/oak-d-s2-specs.md) citing the [OAK-D Pro page](https://docs.luxonis.com/hardware/products/OAK-D%20Pro) and [operating temperature page](https://docs.luxonis.com/hardware/platform/environmental-specifications/operating-temperature)
- **Low-light advice.** Use the mono cameras (better quantum efficiency), longer exposure instead of higher gain, and ISP denoise/sharpness tuning. — [Configuring Stereo Depth](https://docs.luxonis.com/hardware/platform/depth/configuring-stereo-depth/)
- **LR-check.** Removes wrongly computed disparity "due to occlusions at object borders". — [v2 StereoDepth node](https://docs.luxonis.com/software/depthai-components/nodes/stereo_depth/)
- **Median filter.** "non-edge-preserving" and hardware-accelerated. The ROBOTICS spatial filter uses delta 20 and hole-filling radius 2. — [Configuring Stereo Depth](https://docs.luxonis.com/hardware/platform/depth/configuring-stereo-depth/)
- **Point clouds.** Luxonis recommends subpixel (reduces layering), decimation with median or mean mode, and host-side voxelization plus statistical outlier removal (Open3D or PCL). — [Configuring Stereo Depth](https://docs.luxonis.com/hardware/platform/depth/configuring-stereo-depth/)
- **Host-side refinement.** Luxonis points to a WLS (weighted-least-squares) filter example for host-side refinement; it supports RVC2. — [Luxonis Depth Perception](https://docs.luxonis.com/hardware/platform/features/depth/); [oak-examples depth-measurement](https://github.com/luxonis/oak-examples/tree/main/depth-measurement)
- **Foreground fattening.** Window-based matching "assumes constant disparity within the window", which breaks at depth boundaries and makes foreground surfaces grow into the background. Large windows blur boundaries and lose thin objects. [snippet] — [TU Wien stereo paper](https://www.ims.tuwien.ac.at/publications/tuw-139882.pdf). For SGM, fattening appears as halos around thin objects. [snippet] — [Xue et al., MIT](https://people.csail.mit.edu/tfxue/project/multiview-stereo)
- **Edges parallel to the baseline.** These are "difficult to recover regardless of the motion… all stereo methods suffer from this", and depth estimates for such structures are "typically unreliable". [snippet] — [Event-based Stereo VO, arXiv 2007.15548](https://arxiv.org/pdf/2007.15548)
- **Drone thin-obstacle literature.** Thin wires are "thin, featureless, and visually ambiguous" and degrade depth estimation; systems are unable to "consistently and reliably detect thin obstacles". [snippet] — [UCorr, arXiv 2509.14989](https://arxiv.org/pdf/2509.14989). CMU's DROAN pairs a monocular CNN wire detector with a binocular stereo pair for generic obstacles, both represented in disparity space, and was tested on hardware. — [DROAN (IROS 2018)](https://publications.ri.cmu.edu/droan-disparity-space-representation-for-obstacle-avoidance-enabling-wire-mapping-avoidance/)
- **LENS neural stereo.** Luxonis' learned stereo is marketed as handling edge cases classical pipelines fail on. It runs only on RVC4 (NEURAL backend). — [Luxonis forum: LENS](https://discuss.luxonis.com/d/6553-neural-stereo-depth-estimation-with-lens) [snippet]; [v3 Depth node](https://docs.luxonis.com/software-v3/depthai/depthai-components/nodes/depth)
- **[measured] Fill-rate sensitivity.** Invalid fraction is 38 to 40 % at confidence 15 in a bright scene versus 85 % in a dim scene, with the same preset. This shows how strongly illumination and texture drive fill-rate on this unit. — [camera/docs/depthai-v3-cheatsheet.md](../../camera/docs/depthai-v3-cheatsheet.md)

### Inferences
- **Pixel width of a 2.5 cm pole (derived: w = f·W/Z, unit fx).**

  | Distance | 800P | 400P | 400P + decimation ×2 |
  |---|---|---|---|
  | 2 m | 10 px | 5 px | 2.5 px |
  | 4 m | 5 px | 2.5 px | 1.25 px |
  | 6 m | 3.3 px | 1.7 px | — |

  A 7x7 median (or a 7x7 / 7x9 census window) at the decimated resolution can remove or bias a 1 to 3 px object. So for poles:
  - Run 400P with decimation 1 and median 3x3 or 5x5.
  - Aggregate with a robust minimum, such as a low percentile with a minimum pixel count, not a median.
  - Expect reliable detection of ~2 to 3 cm poles only to about 3 to 4 m at 400P.
- **Edge fattening and flying pixels.** Expect obstacles to appear slightly wider than reality, which is the safe direction for avoidance. Expect flying pixels (interpolated depths between foreground and background) along silhouettes, which can create phantom points in free space behind a pole. Mitigations:
  - LR-check on.
  - Speckle filter on.
  - Per-sector "k-of-N" voting: require several pixels within a small depth window.
  - Statistical outlier removal on the point cloud.
  - Short temporal persistence on the host (2 to 3 frames). The on-device temporal filter is not used because it smears during motion.
- **Textureless large obstacles.** A plain wall seen face-on mostly reads as holes, with valid depth only at its edges, posters, seams, or shading. Mitigations:
  - Treat sectors with low valid-fraction as unknown or blocked, not free.
  - Keep a short-term map (VIO pose plus accumulated voxels) so a wall seen at an oblique angle earlier stays in memory.
  - Lower the confidence threshold toward 0 to 15 (v3) for more fill, at the cost of more noise.
  - The spatial filter's hole-filling (radius 2 in ROBOTICS) bridges only small gaps.
  - Foam pads and uniformly painted boxes behave like walls. Their corners and edges are what stereo will see.
- **False near returns.** Likely sources are repetitive patterns (nets, mesh, floor lines), specular reflections and saturated light fixtures: periodic texture along the epipolar line produces ambiguous matches. LR-check, confidence threshold, the brightness filter (maxBrightness) and k-of-N voting are the mitigations. If the venue has netting, test it explicitly. This is general stereo reasoning; I found no Luxonis-specific numbers.
- **Horizontal ropes, bars and wires** cannot be relied on with a horizontal-baseline camera. Rotating the camera 90° would break the cuVSLAM calibration and layout. If such obstacles exist, a monocular detector on the mono or color stream (as in DROAN) or physical margins are the realistic options.

### Gaps
- No Luxonis data quantifies fill-rate or error on specific low-texture materials (foam, painted wood, gym walls) or a minimum detectable pole diameter versus distance. These need bench measurement with the actual competition-like materials.
- No published guidance quantifies how far the RVC2 census window and SGM aggregation fatten edges, in pixels.

---

## 5. Indoor lighting: OV9282 auto-exposure, flicker (50/60 Hz anti-banding) and motion blur at drone speeds

### Takeaway
The OV9282 is global shutter, so there is no rolling-shutter skew. But auto-exposure will push exposure up to 25 to 33 ms under indoor light, which smears thin obstacles at drone speeds.
- DepthAI's default anti-banding is MAINS_50_HZ, which is wrong for 60 Hz US mains. Set MAINS_60_HZ; Luxonis then caps auto-exposure at 25 ms.
- Cap exposure further with `setAutoExposureLimit` (a few ms) and let the ISO cap rise. Accept more noise in exchange for sharper edges.
- Apply identical settings to both mono cameras.

### Cited Findings
- **OV9282 specs on RVC2.**
  - Modes: 1280x800 (full), 1280x720 (crop) and 640x400 (binning). Max FPS 129, 143 and 255 respectively.
  - 3 µm pixels, global shutter.
  - Minimum exposure: 10 µs manual, 20 µs auto.
  - Maximum exposure: manual 1/FPS (33 ms at 30 fps); auto 33 ms with anti-banding OFF, 30 ms at 50 Hz, 25 ms at 60 Hz.
  - Source: [Luxonis OV9282 sensor page](https://docs.luxonis.com/hardware/sensors/OV9282)
- **CameraControl API (v3).**
  - `setAntiBandingMode`: OFF / MAINS_50_HZ / MAINS_60_HZ / AUTO, "Default: MAINS_50_HZ".
  - `setAutoExposureLimit(maxExposureTimeUs)`: "By default the AE algorithm prioritizes increasing exposure over ISO, up to around frame-time (subject to further limits imposed by anti-banding)".
  - `setAutoExposureMaxISO`.
  - `setManualExposure(us, iso)`: ISO "usual range 100..1600".
  - `setAutoExposureRegion`, `setAutoExposureCompensation` (−9..9), `setAutoExposureLock`.
  - `setLumaDenoise` / `setSharpness` (0..4, default 1).
  - Source: [Luxonis v3 CameraControl](https://docs.luxonis.com/software-v3/depthai/depthai-components/messages/camera_control/)
- **Luxonis exposure guidance.** The example is 1/30 s at 30 FPS and 1/15 s at 15 FPS, and the advice is "longer exposure instead of higher gain" for low light. — [Configuring Stereo Depth](https://docs.luxonis.com/hardware/platform/depth/configuring-stereo-depth/)
- **[measured] The repo's scripts** use a 1.5 s auto-exposure warm-up before reading depth. — [camera/02_stereo_depth.py](../../camera/02_stereo_depth.py)

### Inferences
- **Motion blur in pixels (derived: blur ≈ fx·ω·t for rotation, fx·v·t/Z for translation).**
  - Yaw at 60 °/s (1.05 rad/s) with 25 ms exposure: about 21 px at 800P and 10 px at 400P.
  - The same yaw with 5 ms exposure: about 4 px at 800P and 2 px at 400P.
  - Lateral 2 m/s at 2 m range with 25 ms: about 20 px at 800P.

  Blur that large wipes out the 2.5 to 10 px pole widths from section 4. Cap auto-exposure to about 3 to 6 ms for flight, and verify the image does not drop into the dim-scene fill-rate regime (section 2).
- **Exposure is also a VIO concern.** cuVSLAM uses the same frames, so the limit should be tuned jointly. A short exposure is generally good for VIO too, but noise and low light hurt feature tracking.
- **Flicker on a global shutter.** Mains-powered LED or fluorescent lights flicker at 100 or 120 Hz. With a global shutter, flicker appears as frame-to-frame brightness changes, not spatial banding. That matters more for AE stability and VIO than for single-frame stereo, since both cameras are exposed together.
  - When exposure is capped well below 8.3 ms (one 120 Hz half-period), anti-banding can no longer quantise exposure. Expect some brightness pumping, and if it destabilises AE, consider locking exposure after takeoff (`setAutoExposureLock`).
  - This is an inference; I found no Luxonis test of OV9282 under 120 Hz flicker with short exposure.
- **Gym and hall lighting.** High-bay fixtures in the field of view can saturate pixels. Use the brightness filter's max to invalidate them, and an AE region that excludes the ceiling (`setAutoExposureRegion`) so the floor and obstacles are not underexposed.

### Gaps
- I found no source on whether DepthAI v3 automatically syncs AE between CAM_B and CAM_C, or whether control must be sent to each Camera node. To be safe, apply identical settings to both.
- There is no Luxonis guidance specific to motion blur at drone speeds.

---

## 6. Turning depth into obstacles: SpatialLocationCalculator, PointCloud, voxels, ground/ceiling removal, OBSTACLE_DISTANCE (72 sectors), 3D occupancy; what Luxonis provides

### Takeaway
Luxonis provides building blocks but no drone obstacle-avoidance pipeline:
- An on-device SpatialLocationCalculator (ROI statistics, including MIN).
- A PointCloud node, which runs on the host for RVC2.
- Host examples (spatial-on-host, WLS, RGBD point cloud).

The fastest implementable path is a host-side numpy step: depth → back-project → rotate to a gravity-aligned body frame → height band → 72 × 5° polar minimum with unknown handling → MAVLink OBSTACLE_DISTANCE at ≥ 10 Hz. Optionally accumulate a small local voxel map using cuVSLAM poses for memory beyond the 80° FOV.

### Cited Findings
- **SpatialLocationCalculator (v3).**
  - Runs on device by default (`setRunOnHost` switches to host), with RDF coordinates.
  - Per-ROI calculation algorithm MEDIAN (default) / AVERAGE / MIN / MAX / MODE; global lower and upper thresholds in depth units (mm by default; defaults 0 to 65535); stepSize (default AUTO).
  - Runtime reconfiguration through `inputConfig`. The example uses thresholds 100 to 10000.
  - The maximum number of ROIs is not stated.
  - Sources: [v3 SLC node](https://docs.luxonis.com/software-v3/depthai/depthai-components/nodes/spatial_location_calculator/); [v3 SLC config](https://docs.luxonis.com/software-v3/depthai/depthai-components/messages/spatial_location_calculator_config/)
- **PointCloud node (v3.6.0+).**
  - "On RVC2, only host-side processing is supported"; `setRunOnHost(False)` offloads only on RVC4.
  - Outputs `getPoints()` as an N×3 float32 array.
  - `setLengthUnit`, `setOrganized`, `setTargetCoordinateSystem` (camera socket or housing frames such as IMU), and `setTransformationMatrix` (4×4).
  - Sources: [v3 PointCloud node](https://docs.luxonis.com/software-v3/depthai/depthai-components/nodes/pointcloud/); [camera/docs/depthai-v3-cheatsheet.md](../../camera/docs/depthai-v3-cheatsheet.md)
- **Luxonis v3 example set (oak-examples/depth-measurement).** calc-spatial-on-host, dynamic-calibration, stereo-on-host, stereo-runtime-configuration, triangulation, wls-filter, and measurement-3d (box-measurement, rgbd-pointcloud, tof-pointcloud), all marked RVC2-supported. None is a collision-avoidance, occupancy or bird's-eye example. — [oak-examples depth-measurement](https://github.com/luxonis/oak-examples/tree/main/depth-measurement)
- **Older collision-avoidance examples.** A v2-era DepthAI SDK "Collision Avoidance" sample existed (depth-based proximity slowdown for supervised robots); its URL now lands on the v3 SDK page [v2-only]. — [old SDK sample](https://docs-old.luxonis.com/projects/sdk/en/latest/samples/mixed/sdk_collision_avoidance). A forum user hit errors running the depthai-experiments collision-avoidance demo on an OAK-D Lite [snippet]. — [Luxonis forum 1061](https://discuss.luxonis.com/d/1061-collision-avoidance-on-oak-d-lite)
- **PX4 ObstacleDistance message.**
  - `distances` is uint16[72] in cm. 0 = obstacle directly at the sensor; max_distance+1 = no obstacle; UINT16_MAX = unknown/not used.
  - `increment` = angular width per element in degrees; `min_distance` / `max_distance` in cm; `angle_offset` = angle of element 0 (0 = forward, positive clockwise).
  - `frame`: MAV_FRAME_GLOBAL (north-aligned, default) or MAV_FRAME_BODY_FRD (12) for body-mounted sensors.
  - Source: [PX4 ObstacleDistance](https://docs.px4.io/main/en/msg_docs/ObstacleDistance.html)
- **PX4 Collision Prevention.**
  - Applies to multicopters in **Position mode**, which requires MPC_POS_MODE = acceleration-based.
  - Parameters: CP_DIST (minimum distance, "measured to the sensors"); CP_DELAY (sensor plus tracking delay; external vision "may be up to 0.2 s", tracking delay "typically 0.1 to 0.5 s"); CP_GUIDE_ANG ("30 degrees is a good balance"); CP_GO_NO_DATA (default 0 = no motion into directions without data).
  - If range data is lost for 5 s, the vehicle switches to HOLD.
  - The companion must send OBSTACLE_DISTANCE. Initial testing ran at 4 m/s with 10 Hz messages. The companion setup is "currently untested" and the original avoidance project is archived.
  - The page is inconsistent on 36 versus 72 sectors.
  - Source: [PX4 Collision Prevention](https://docs.px4.io/main/en/computer_vision/collision_prevention)
- **ArduPilot RealSense template (d4xx_to_mavlink.py).**
  - Takes one horizontal "obstacle line" whose height (`obstacle_line_height_ratio`) and thickness (`obstacle_line_thickness_pixel`) are configurable, and uses the lowest value in the band.
  - Divides HFOV into 72 rays (5°), compensates pitch using the ATTITUDE message, and sends OBSTACLE_DISTANCE at "10Hz or higher" (about 15 Hz observed).
  - Example parameters: AVOID_MARGIN 1.5 m, AVOID_BEHAVE Stop or Slide.
  - Source: [ArduPilot RealSense depth camera](https://ardupilot.org/copter/docs/common-realsense-depth-camera.html)
- **ArduPilot OAK-D-IoT script.** Splits the depth image into a 3x3 grid and sends the closest distance per cell as OBSTACLE_DISTANCE_3D (Copter 4.1+, beta). — [ArduPilot forum, OAK-D Part 2](https://discuss.ardupilot.org/t/easy-way-to-integrate-ai-with-ardupilot-oak-d-part-2/79307); code at [rishabsingh3003/ardupilot_depthai_scripts](https://github.com/rishabsingh3003/ardupilot_depthai_scripts/tree/master/IOT/obstacle_avoidance)
- **Disparity-space representations** (inflating obstacles directly in disparity images and collision-checking there) are an established MAV alternative to 3D grids. — [DROAN (IROS 2018)](https://publications.ri.cmu.edu/droan-disparity-space-representation-for-obstacle-avoidance-enabling-wire-mapping-avoidance/). So is planning directly in depth-image space with a single forward stereo camera and onboard compute [snippet]. — [Ahmad & Fierro, arXiv 2010.09098](https://arxiv.org/abs/2010.09098)

### Inferences
- **Recommended host pipeline per depth frame** (640x400 at 30 fps is about 256k pixels; numpy handles this in a few ms on an Orin CPU, which is an estimate to measure):
  1. Mask out depth == 0 and values outside about 0.3 to 8 m.
  2. Optional stride 2 for speed, applied after masking so thin objects survive.
  3. Back-project with intrinsics from `depth.getTransformation()` or the calibration (fx 399.6 at 400P): X = (u−cx)Z/fx, Y = (v−cy)Z/fy.
  4. Transform camera → body with the camera extrinsics, then roll/pitch-level using cuVSLAM or PX4 attitude, so the height band stays horizontal while the vehicle tilts. Tilt is significant on a 7-inch quad accelerating.
  5. Remove floor and ceiling: keep points with height in [z_vehicle − h_low, z_vehicle + h_high], for example ±0.3 to 0.5 m around the vehicle's own vertical extent plus margin, using altitude from VIO or a downward rangefinder. A RANSAC plane fit is unnecessary if attitude and altitude are known.
  6. Polar binning: compute azimuth atan2 in the body FRD frame and bin into 72 × 5°. Only about 16 bins fall inside the 80° HFOV.
  7. Per bin, take a robust minimum: for example, the 5th-percentile range among points, requiring at least k points (k ≈ 5 to 20 at 400P) within 10 to 15 cm of that value.
  8. Bins with too few valid pixels inside the FOV become a conservative value or UINT16_MAX (unknown). Bins outside the FOV become UINT16_MAX, and CP_GO_NO_DATA = 0 blocks motion there.
  9. Send OBSTACLE_DISTANCE with frame BODY_FRD, increment 5, angle_offset chosen so element 0 is forward (0°), min_distance = MinZ (32 cm at 400P), and max_distance = the trusted range (for example 500 to 800 cm). Send at camera rate or at least 10 to 15 Hz.
- **Memory beyond the FOV.** PX4 Collision Prevention only sees the current message, and the camera sees only 80°. For lateral or backward moves, maintain a local ring map: rotate and translate previous bins or voxels with cuVSLAM odometry, decay them over 1 to 3 s, and fuse into the 72-bin message. The alternative is to forbid motion outside the FOV.
- **3D option.** Voxelize the leveled points (for example 10 cm voxels, numpy unique or Open3D `voxel_down_sample`) into a robot-centred rolling grid with hit/miss counters. Ray-cast free space only for valid pixels; never clear voxels from invalid pixels.
  - nvblox (GPU, ROS 2) is the off-the-shelf version, but it competes with cuVSLAM for the GPU (section 3) and its Orin Nano performance is unverified.
- **SpatialLocationCalculator as a zero-host-CPU variant.** Configure, for example, a 1 × 16 strip of ROIs (one per 5° sector) or a 3 × 16 grid of ROIs over the obstacle band with algorithm MIN or a low statistic and thresholds 300 to 8000 mm. The device then returns per-sector distances directly.
  - Limitations: ROIs are fixed in the image, so there is no attitude compensation unless ROIs are moved at runtime via `inputConfig`. MIN is sensitive to single flying pixels, and the stats per ROI do not include a percentile.
  - Use it as a cross-check or a fallback, not the primary path.
- **Planner interface.** In Offboard or autonomous modes, PX4's Collision Prevention (documented for Position mode) will not protect the vehicle. The companion planner must consume the same 72-bin / voxel output itself, and OBSTACLE_DISTANCE serves as a second safety layer during manual Position-mode flights.

### Gaps
- No Luxonis or PX4 documentation covers an OAK-D to PX4 OBSTACLE_DISTANCE bridge; this is custom code.
- Whether PX4 Collision Prevention keeps a memory of sectors from earlier messages, or works in Offboard or Mission modes on current PX4 (v1.16 or newer), is not stated on the page I loaded.
- The SpatialLocationCalculator's maximum ROI count and per-ROI latency on RVC2 are undocumented.
- There is no Orin Nano nvblox benchmark: a search excerpt showed about 4.9 fps with unlabeled columns [snippet].

---

## 7. End-to-end latency (exposure to depth on host) and what it means for safe speed and stopping distance indoors

### Takeaway
A realistic budget for 400P on-device depth is about 40 to 70 ms from mid-exposure to depth on the host:
- Exposure: up to a few ms if capped.
- Readout and rectification: ~1.5 ms.
- Stereo: ~10 to 20 ms.
- Post-processing: ~1 to 2 ms.
- USB transfer: ~8 to 25 ms.
- Queueing.

Add host processing, MAVLink and PX4 setpoint tracking (0.1 to 0.5 s per PX4), plus 1 to 2 frames for k-of-N voting. With 3 to 5 m of reliable detection range indoors, about 2 to 3 m/s is a defensible top speed. That matches the existing 3 m/s VIO guard.

### Cited Findings
- **Luxonis time-to-host for raw mono over USB 3 (5 Gbps):** 800P at 120 fps = 24.5 ms; 400P at 120 fps = 7.5 ms; 400P at 90 fps = 8 ms (std 0.47). Lowering camera FPS to match downstream consumers "typically provides the best latency performance". — [Luxonis Optimizing FPS & Latency](https://docs.luxonis.com/software/depthai/optimizing/)
- **Stereo stage times:** 400P LR + subpixel 10.1 ms, plus rectification 1.5 ms, plus median 0.8 ms. 800P LR + subpixel 30.5 ms, plus rectification 6 ms, plus median 1.8 ms. — [v2 StereoDepth node](https://docs.luxonis.com/software/depthai-components/nodes/stereo_depth/)
- **[snippet] Forum measurement:** 400P LR-check + subpixel at about 21 to 24 ms frame-timestamp-to-host. — [Luxonis forum thread 957](https://discuss.luxonis.com/d/957-impact-of-latency-on-practical-applications/27)
- **PX4 delays:** CP_DELAY covers sensor plus tracking delay; external vision "may be up to 0.2 s"; tracking delay is "typically between 0.1 and 0.5 seconds". Velocity is limited so the vehicle can stop before CP_DIST, using MPC_XY_P, MPC_JERK_MAX and MPC_ACC_HOR. The feature "may restrict vehicle maximum speed if the sensor range isn't large enough!" — [PX4 Collision Prevention](https://docs.px4.io/main/en/computer_vision/collision_prevention)
- **ArduPilot rate guidance:** OBSTACLE_DISTANCE at "10Hz or higher, depends on how fast the vehicle is moving". — [ArduPilot RealSense](https://ardupilot.org/copter/docs/common-realsense-depth-camera.html)
- **[measured] Project context:** the cuVSLAM starvation guard rejects motion faster than 3 m/s (`--max-speed` default) or 720 °/s. — [methods/cuvslam-vio/08_cuvslam_track.py](../../methods/cuvslam-vio/08_cuvslam_track.py)

### Inferences
- **How to measure latency on this unit.** For each depth message, compute `dai.Clock.now() − msg.getTimestamp()` on the host. DepthAI timestamps are host-synced. Compare with the raw-mono queue to isolate stereo cost.
  - Check whether the timestamp marks exposure start, middle or end; DepthAI exposes an exposure-offset argument on `getTimestamp` in recent versions, but this was not verified for v3.10.
  - Do this once each at 400P and 800P with the chosen preset.
- **Stopping distance (derived):** d = v·t_lat + v² / (2a). With t_lat = 0.15 s (depth ~0.06 s, plus host, MAVLink and control) and a conservative indoor braking a = 3 m/s²:

  | Speed | Stopping distance |
  |---|---|
  | 1 m/s | 0.32 m |
  | 2 m/s | 0.97 m |
  | 3 m/s | 1.95 m |
  | 4 m/s | 3.3 m |

  Add CP_DIST or planner margin (0.5 to 1 m), plus the drone's half-span (~0.2 to 0.25 m for a 7-inch), plus one frame of travel at 30 fps (3 to 10 cm). At 3 m/s the obstacle must be reliably detected by about 3 m. That is near the limit for 2 to 3 cm poles at 400P (section 4) but comfortable for boxes and walls with texture.
- **Speed recommendation.** 2 m/s near clutter and up to 3 m/s in open corridors, if the planner uses a forward-looking horizon. Lateral or backward moves outside the 80° FOV should be limited to remembered-map speeds, about 1 m/s or less.
- **Speed versus blur.** At these speeds the exposure cap from section 5 matters more than the stereo latency.

### Gaps
- There is no published Luxonis exposure-to-host latency for StereoDepth at 400P or 800P with the ROBOTICS preset in v3, and no measurement on this unit yet.
- PX4 MPC_ACC_HOR and braking performance of this specific 7-inch airframe are unknown; the 3 m/s² figure is an assumption.

---

## 8. Known drone deployments of OAK-D / OAK-D Lite / OAK-D S2 for obstacle avoidance

### Takeaway
I found no peer-reviewed paper or official ArduPilot or PX4 documentation that flight-tests an OAK-D-family camera for obstacle avoidance with quantitative results. Evidence consists of:
- ArduPilot community scripts (3x3-grid closest distance, sent as OBSTACLE_DISTANCE_3D).
- An unresolved forum report of an OAK-D Lite + Jetson Nano setup that sent data but did not make the copter stop.
- OAK-D use for VIO (not avoidance) on ArduPilot.

Treat OAK-based avoidance as experimental and budget time for bench and tethered testing.

### Cited Findings
- **ArduPilot dev "OAK-D-IoT" project (sub-100 g, ESP32 to flight controller over MAVLink).** Divides the depth view into a 3x3 grid and sends the closest distance per cell via OBSTACLE_DISTANCE_3D. It is beta, requires Copter 4.1+, has a demo video, and reports no quantitative avoidance results. — [ArduPilot forum: OAK-D Part 2](https://discuss.ardupilot.org/t/easy-way-to-integrate-ai-with-ardupilot-oak-d-part-2/79307)
- **ArduPilot forum, OAK-D Lite on Jetson Nano with pymavlink.** Distance data appeared in Mission Planner but the copter did not hold distance or back away; no resolution was posted [snippet]. — [ArduPilot forum 103145](https://discuss.ardupilot.org/t/simple-object-avoidance-using-oak-d-lite-camera-and-pymavlink-is-not-working/103145)
- **ArduPilot official OAK-D docs** cover using an OAK-D with a Raspberry Pi as a GPS substitute (VINS-Fusion VIO), not obstacle avoidance [snippet]. — [ArduPilot: Luxonis OAK-D](https://ardupilot.org/copter/docs/common-vio-oak-d.html)
- **Community opinion:** "the quality of the Depth Frames isn't as good as the RealSense cameras" [snippet]. — [ArduPilot forum 76235](https://discuss.ardupilot.org/t/opencv-ai-aka-oak-d-and-ardupilot/76235)
- **Vibration.** Luxonis recommends fixed-focus color cameras for heavy-vibration uses "such as drones" because the AF voice-coil lens vibrates. This affects only the color camera, not the fixed-focus stereo pair. — [camera/docs/oak-d-s2-specs.md](../../camera/docs/oak-d-s2-specs.md) citing the [Luxonis focus-type guide](https://docs.luxonis.com/hardware/platform/sensors/focus-type)
- **Vibration test.** The OAK-D family was tested to EN 60068-2-6 (5 to 200 Hz, 50 m/s²) with depth accuracy checked before and after. — [Luxonis vibration tolerance](https://docs.luxonis.com/hardware/platform/environmental-specifications/vibration-tolerance) (via the repo spec sheet)
- **PX4 side:** the companion-computer collision-prevention path is "currently untested" and the original PX4 avoidance project is archived. — [PX4 Collision Prevention](https://docs.px4.io/main/en/computer_vision/collision_prevention)
- **Wider context:** a systematic review found that vision-based obstacle-detection work is "mainly tested using simulation", with only 26 papers reporting physical flying-vehicle tests (through 2022) [snippet]. — [PMC10607331](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC10607331/)
- **Recent stereo-only quadrotor work (not OAK):** Stereo-Planner (arXiv 2610.15560, 11 Oct 2026) flew raw-stereo-to-trajectory planning on an Orin NX at up to about 7 m/s outdoors. The camera model is not stated. — [Stereo-Planner](https://arxiv.org/html/2610.15560)

### Inferences
- The closest proven recipe is the ArduPilot RealSense "obstacle line → 72 rays → OBSTACLE_DISTANCE" design (section 6) with OAK depth substituted. Most of the risk is in depth quality on the actual venue surfaces and in the flight-controller integration, not in the math.
- The OAK-D Lite thread's failure (data visible, no avoidance) is a typical symptom of frame, orientation or parameter mismatches: angle_offset or frame, min/max distance, or the avoidance mode not being active. Verify in a bench test that PX4's `obstacle_distance` uORB topic shows the expected sectors before flying.

### Gaps
- I found no academic paper reporting flight results for OAK-D / OAK-D Lite / OAK-D S2 obstacle avoidance, and no PX4-specific OAK integration.

---

## 9. Thermal guidance relevant to a drone mount (power modes, fan, airflow)

### Takeaway
Luxonis rates the S2 for −20 to 50 °C ambient at full VPU load. At 25 °C ambient a fully loaded RVC2 sits around 70 °C, and DepthAI shuts down at a 105 °C die temperature. There is no official fan or drone-mount guidance, but community data shows active airflow cuts the chip temperature by 15 to 20 °C.
- On a drone, mount the aluminium housing in prop-wash or free air (never inside foam).
- Cut load: no color camera unless needed, no encoder, stereo at 400P, minimal filters, no unused outputs.
- Log `getChipTemperature()` and add a software cutoff well below 105 °C.

### Cited Findings
- **S2 thermal specs.** Ambient operating range is −20 to 50 °C "at full VPU use"; "The RVC2 VPU can run continuously at 105 °C, after which depthai automatically shuts the device down". Power: base + streaming 2.5 to 3 W, AI up to 1 W, stereo depth up to 0.5 W, encoder up to 0.5 W; "Lowering pipeline FPS reduces consumption." — [OAK-D S2 product page](https://docs.luxonis.com/hardware/products/OAK-D%20S2)
- **Operating temperature page.**
  - RVC2 silicon is rated −40 to 105 °C. At 25 °C ambient and full power the chip is about 70 °C (35 °C margin); worst-case self-heating is about 45 °C, so the theoretical maximum ambient is 60 °C.
  - OAK-D-Lite (smaller heatsink) under 3 cameras + stereo + NN + encoding: VPU 87 °C at 30 °C ambient, 94 °C at 35 °C, 100 °C at 40 °C, and a crash at 125 °C VPU at 50 °C ambient. Disabling video encoding reduced temperature.
  - OV9282 module temperature rise is 13 °C over ambient.
  - No fan or airflow recommendations are given.
  - Source: [Luxonis operating temperature](https://docs.luxonis.com/hardware/platform/environmental-specifications/operating-temperature)
- **Community data, OAK-D (forum).** The pipeline shut down shortly after the chip reached about 77 °C. A 50 mm fan brought it to about 62 °C; two 30 mm fans held about 55 °C for a 1.5 h run [snippet]. — [Luxonis forum 4994](https://discuss.luxonis.com/d/4994-error-xlink-read-rel-resp)
- **Community data, OAK-D-Lite (forum).** The MyriadX reached about 73.5 °C after 90 min with the enclosure at about 56 °C [snippet]. — [Luxonis forum 780](https://discuss.luxonis.com/d/780/1)
- **[measured] Telemetry.** The repo already reads `device.getChipTemperature()` (average plus per-block css/mss/upa/dss) in its device-info tool. — [camera/00_device_info.py](../../camera/00_device_info.py)
- **[Pro-only]** The OAK-D Pro's IR projector is limited to 10 to 60 °C (absolute 0 to 80 °C); this does not apply to the S2. — [Luxonis operating temperature](https://docs.luxonis.com/hardware/platform/environmental-specifications/operating-temperature)

### Inferences
- **Why the bench unit runs hot.** Overheating after 5 to 10 min on a bench is consistent with still air plus sustained load. In flight, prop-wash over an exposed aluminium housing should behave like the forum's fan cases.
  - For bench sessions, add a small 5 V fan (the forum data suggests 15 to 20 °C reduction) and keep sessions short.
  - Have the watchdog stop the pipeline at a chip-temperature threshold, for example 85 to 90 °C, well below the 105 °C shutdown and below the ~77 °C instability one forum user saw. That forum value is anecdotal, so set the threshold from this unit's logs.
- **Load reductions that also help depth latency:**
  - 400P stereo instead of 800P (roughly 3× less stereo time).
  - Decimation and median kept modest.
  - Color camera (CAM_A) and encoder off in flight.
  - Confidence-map and disparity outputs disabled unless used.
  - Only the streams cuVSLAM and the obstacle layer need.
  - Lowering FPS (for example depth at 15 fps while raw stays at 30) would cut power further. This needs a second camera-output rate, which is unverified on RVC2.
- **Mounting.**
  - Avoid enclosing the camera in foam vibration isolation that also insulates it thermally. Use isolation at the mount points and leave the housing faces exposed.
  - Do not mount it directly above hot ESCs or the Orin Nano heatsink exhaust.
  - The VESA M4 holes on the back and the 1/4"-20 socket are the mounting points; the repo spec sheet has the dimensions and weight (97 × 29.5 × 22.9 mm, ~91 g).

### Gaps
- Luxonis publishes no S2-specific thermal curve, no fan or heatsink accessory guidance, and no "low-power mode" API for RVC2 beyond reducing the workload or FPS.
- There is no in-flight temperature data for an OAK on a multirotor.
