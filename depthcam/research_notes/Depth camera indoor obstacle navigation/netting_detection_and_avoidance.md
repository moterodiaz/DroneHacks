# Detecting and avoiding enclosure safety netting with a passive stereo camera (OAK-D S2), plus non-perception fallbacks

Research date: 2026-10-25. Older findings carry their year. "Snippet only" means the claim came from a search-result excerpt and the page itself could not be opened, so weight it lower.

Platform assumed (from the brief and the repo): OAK-D S2, 2x OV9282 global-shutter mono, 1280x800, 7.5 cm baseline, no IR projector, RVC2 on-device SGM; IMX378 color; Jetson Orin Nano 8 GB; PX4 on Pixhawk 6C; cuVSLAM stereo VIO with about 2-3 % return-to-start drift on handheld walks.

---

## 1. Why passive stereo struggles with thin mesh netting, and at what range a strand is resolvable

### Takeaway
A 1-3 mm strand covers at least one pixel on the OAK-D S2 mono cameras at 1280x800 only within about 0.8-2.4 m (0.4-1.2 m at 640x400). Standard-mode MinZ is about 0.63-0.8 m. So the window where stereo can see the net at all is roughly 0.6 m to 1.5-2 m. Beyond that the matching window is mostly background seen through the holes (open area 72-96 %), so SGM returns the background depth or nothing. Inside that window the 2-5 cm mesh period (5-80 px) is shorter than the 96-level disparity search, so the matcher can also lock onto the wrong mesh cell. A 1-3 mm strand is 4-13x thinner than the 1/2 inch (12.7 mm) floor below which Skydio says its vision system cannot see obstacles. Do not count on stereo depth to detect the net early enough to stop at speed.

### Cited Findings
**Camera geometry (the numbers the tables below are built on)**
- Official OAK-D S2 spec: OV9282 stereo pair DFOV/HFOV/VFOV 89.5 / 80 / 55 deg, global shutter, fixed focus. Baseline 7.5 cm. Ideal depth range about 80 cm to 12 m. MinZ about 80 cm at 800P, about 40 cm at 400P or 800P with extended disparity, about 20 cm at 400P with extended disparity. Median depth accuracy below 2 % under 4 m, below 4 % at 4-7 m, below 6 % at 7-10 m. IMX378 color: AF variant 78 / 66 / 54 deg, fixed-focus variant 82 / 69 / 55 deg, rolling shutter. — [Luxonis OAK-D S2 product docs](https://docs.luxonis.com/hardware/products/OAK-D%20S2)
- Luxonis' stereo configuration page works its examples with fx = 882.5 px and HFOV 71.9 deg for 1280 px. It gives MinZ of "roughly 70cm" with the standard 95 px search and "34.84cm" with extended disparity (190 px). It gives a focal-length floor of 19.6 cm from the fixed-focus mono lenses. — [Luxonis: Configuring Stereo Depth](https://docs.luxonis.com/hardware/platform/depth/configuring-stereo-depth)
- The user's own unit was measured in the repo: CAM_B fx = 799.13 px and CAM_C fx = 797.54 px at 1280x800 (HFOV from intrinsics about 77.4 deg; EEPROM spec HFOV 71.86 deg). CAM_B fx = 399.6 px at 640x400. The color camera has fx about 3066 px at 4056x3040 and 1451.3 px at 1920x1080. — [repo: camera/docs/oak-d-s2-specs.md](../../camera/docs/oak-d-s2-specs.md); [repo: camera/docs/depthai-v3-cheatsheet.md](../../camera/docs/depthai-v3-cheatsheet.md)
- Third-party listings disagree on the stereo FOV (for example 81 / 72 / 49 deg), and a Luxonis forum user found mismatched FOV values across Luxonis pages. — [Arducam OAK-D S2 listing](https://www.arducam.com/blog/?p=88661); [Luxonis forum 1892](https://discuss.luxonis.com/d/1892-whichis-the-exact-spec-for-oak-d-pro-stereo-pair-fov)

**Pixel footprint vs distance (my calculation).** Pinhole model: pixels = fx * size / Z, with the unit's calibrated fx = 799.1 px (1280x800) and 399.6 px (640x400). Disparity = fx * 0.075 / Z. Source of fx: [repo specs file](../../camera/docs/oak-d-s2-specs.md).

Mono 1280x800 (fx 799.1):

| Z (m) | 0.5 | 0.75 | 1.0 | 1.5 | 2.0 | 3.0 | 4.0 | 5.0 | 7.0 | 10 |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 mm strand (px) | 1.60 | 1.07 | 0.80 | 0.53 | 0.40 | 0.27 | 0.20 | 0.16 | 0.11 | 0.08 |
| 2 mm strand (px) | 3.20 | 2.13 | 1.60 | 1.07 | 0.80 | 0.53 | 0.40 | 0.32 | 0.23 | 0.16 |
| 3 mm strand (px) | 4.79 | 3.20 | 2.40 | 1.60 | 1.20 | 0.80 | 0.60 | 0.48 | 0.34 | 0.24 |
| 2 cm mesh period (px) | 32.0 | 21.3 | 16.0 | 10.7 | 8.0 | 5.3 | 4.0 | 3.2 | 2.3 | 1.6 |
| 3 cm mesh period (px) | 47.9 | 32.0 | 24.0 | 16.0 | 12.0 | 8.0 | 6.0 | 4.8 | 3.4 | 2.4 |
| 5 cm mesh period (px) | 79.9 | 53.3 | 40.0 | 26.6 | 20.0 | 13.3 | 10.0 | 8.0 | 5.7 | 4.0 |
| disparity (px) | 119.9 | 79.9 | 59.9 | 40.0 | 30.0 | 20.0 | 15.0 | 12.0 | 8.6 | 6.0 |

- Strand reaches 1 px at: 1 mm, 0.80 m; 2 mm, 1.60 m; 3 mm, 2.40 m. It reaches 0.5 px at 1.6 / 3.2 / 4.8 m. (calculation; fx from [repo](../../camera/docs/oak-d-s2-specs.md))
- Mesh period falls below 2 px (Nyquist, where it aliases into moire) at: 2 cm, 8.0 m; 3 cm, 12.0 m; 5 cm, 20.0 m. It falls below 5 px at 3.2 / 4.8 / 8.0 m. (calculation)
- At 640x400 (fx 399.6) every distance halves. A 2 mm strand is 1 px at 0.80 m, which is about the 800P MinZ. A 2 cm mesh aliases beyond 4.0 m. (calculation)
- MinZ with fx 799.1: 0.63 m (95 px search) or 0.32 m (extended, 190 px) at 1280x800; 0.32 m or 0.16 m at 640x400. The 0.16 m case is below Luxonis' 19.6 cm optical floor. (calculation; floor from [Luxonis](https://docs.luxonis.com/hardware/platform/depth/configuring-stereo-depth))
- Using the spec HFOV of 80 deg (fx 762.7) instead moves the 1 px distances to 0.76 / 1.53 / 2.29 m. Using 71.9 deg (fx 880.9) moves them to 0.88 / 1.76 / 2.64 m. The conclusion does not change. (calculation)
- Color IMX378 for comparison (fx 3066 at 4056x3040; 1451.3 at 1920x1080). A 2 mm strand is 1 px at 6.1 m at full resolution and 2.9 m at 1080p. A 1 mm strand is 1 px at 3.1 m and 1.45 m. A 3 cm mesh stays at or above 5 px out to 18 m at full resolution. (calculation; fx from [repo](../../camera/docs/oak-d-s2-specs.md))
- Open-area fraction of a square mesh, ((P-w)/P)^2: 20 mm / 1 mm gives 90 %; 20 mm / 3 mm gives 72 %; 30 mm / 2 mm gives 87 %; 50 mm / 1 mm gives 96 %. So 72-96 % of any matching window on the net is background seen through it. (calculation)
- Typical drone-enclosure netting: vendors list mesh openings of 20-100 mm, with 1-3/4 in (44 mm) knotted nylon common for micro-quad and research cages. Knotless polypropylene or nylon is also used. Twine is sold by number (#12, #24...; a #24 is about twice as thick as a #12), not by stated diameter. — vendor pages via search: [CNW Netting](https://www.cnwnetting.com/pages/drone-cages), [US Netting](https://www.usnetting.com/drone-netting/), [InCord](https://incord.com/wp-content/uploads/2022/07/InCord_Drone-Netting.pdf), [Batting Cages Inc FAQ](https://www.battingcagesinc.com/faq/) (snippet only; vendor sources)

**Matching behavior**
- RVC2 stereo uses a census transform (mean mode) and cost aggregation with P1 = 250 and P2 = 500 (horizontal and vertical). The standard search is 96 levels (0-95), or 191 with extended disparity. Confidence runs 0 (most confident) to 255; pixels above the threshold get disparity 0. RVC2 presets use 15 (DEFAULT, ROBOTICS) or 55 (HIGH_DETAIL). The left-right check is on in all RVC2 presets (threshold 10, or 5 in HIGH_DETAIL). The decimation filter takes 1280x800 to 640x400. Depth 0 "means that the distance is invalid/unknown." Passive stereo needs texture, and blank walls give "noisy/invalid" depth. The page does not cover repetitive patterns or thin objects. — [Luxonis: Configuring Stereo Depth](https://docs.luxonis.com/hardware/platform/depth/configuring-stereo-depth)
- Keller & Scherer (CMU, arXiv 2019): thin obstacles are hard for "sliding window based disparity methods" because they fill only a small fraction of each window. Reflections, repeating patterns and structures parallel to the baseline cause mismatches. A typical two-camera stereo algorithm detected only 5 % of wire pixels. — [arXiv 1910.04874](https://arxiv.org/abs/1910.04874)
- DepthVanish (arXiv 2025): periodic textures create multiple equally plausible matches and so raise the chance of wrong or unstable stereo depth. The paper exploits this to make stereo-invisible patches. — [arXiv 2506.16690](https://arxiv.org/pdf/2506.16690) (snippet only)
- Intel RealSense whitepaper: repetitive structures such as striped walls or window blinds can produce wrong, very close depth, so a drone reacts to obstacles the operator cannot see. Tuning parameters alone removed those outliers only by discarding nearly all valid data. — [RealSense: Depth Map Improvements for Stereo-based Depth Cameras on Drones](https://dev.realsenseai.com/?p=41319) (page updated 2026-02-04; tests used a D435i on SDK 2.23, about 2019)
- Intel also publishes a separate repetitive-pattern mitigation guide for the D400 (second-peak threshold and census registers, depth HDR, an IR pass filter, more projector power). It says the D400f with IR pass filter fixes false depth on repetitive vertical patterns such as airport poles, blinds and ceiling pipes. — [Intel: Mitigation of Repetitive Pattern Effect, D400](https://dev.intelrealsense.com/docs/mitigate-repetitive-pattern-effect-stereo-depth-cameras); [RealSense: Stereo Depth with IR Pass Filter](https://www.realsenseai.com/?p=27194) (snippet only)

### Inferences
- **Usable detection window.** For a 2 mm strand at 1280x800 the net fills at least 1 px per strand only between MinZ (0.63 m standard, 0.32 m extended) and about 1.6 m. A 3 mm strand stretches that to about 2.4 m and a 1 mm strand shrinks it to about 0.8 m. At sub-pixel widths the strand still changes pixel intensity in proportion to its coverage (about 0.5 at 3.2 m for 2 mm), but lens blur and the census noise threshold make it unlikely to win a match against a textured background. At 640x400, which many people use for speed, the window nearly disappears. Run stereo at 1280x800 with extended disparity for net work.
- **Stopping distance against that window** (my calculation: v * latency + v^2 / (2a)). With a = 5 m/s^2 and 0.15 s latency: 0.25 m at 1 m/s, 0.70 m at 2 m/s, 1.35 m at 3 m/s, 2.2 m at 4 m/s. With a = 3 m/s^2 and 0.25 s latency: 0.42 / 1.17 / 2.25 / 3.67 m. Stereo-only net detection at 1.6 m only protects you at about 2 m/s or less.
- **Three failure modes, in order of likelihood:**
  1. Background bleed-through, the most common. With 72-96 % open area, the window cost is dominated by whatever is behind the net, so the depth image shows the hall wall behind the net as if there were nothing in front.
  2. Invalid pixels, where the confidence threshold or LR check rejects the mixed window.
  3. Wrong near or far disparity from mesh-period ambiguity. For a 3 cm mesh at 2 m the period is 12 px and the true disparity 30 px, so candidate minima at 18, 42, 54... px all sit inside the 0-95 search. SGM smoothness (P1/P2) favors one consistent wrong surface, which can show up as a phantom wall nearer or farther than the net.
- Dark netting against a dark background, or white netting against white walls, cuts strand contrast further. A strand at 0.3-0.5 px coverage with 20 % luminance contrast changes a pixel by only a few gray levels, about the same as sensor noise at indoor exposure. This is a plausibility argument; I found no measured data.
- The posts, poles and frame that hold up the net are much larger (several cm). They fall below 1 px only at tens of meters, so stereo should range them reliably. The same goes for any opaque banner or skirt along the bottom of the net.

### Gaps
- I found no measured OAK-D (RVC2) depth maps of drone-cage netting, and no published resolve-distance experiment for nets at a given mesh and strand size. The tables are first-principles calculations. A 10-minute test is worth doing: walk the camera toward the actual net from 4 m to 0.5 m and log the valid-pixel fraction and median depth in an ROI on the net. Keep the session short and watchdog-guarded for camera heat.
- The RVC2 census window size and the number of SGM aggregation paths are not documented on the page I read.
- The competition's net material, color, mesh and twine size are unknown. The vendor ranges above are only a prior.

---

## 2. Evidence from practice: what commercial and dev-kit stereo systems do with nets, fences, wires and glass

### Takeaway
Every vendor that documents limits says thin wires, nets and chain-link fencing are not reliably seen. Skydio names "netting" and "chain link fencing" explicitly and sets the floor at 1/2 inch (12.7 mm). DJI's consumer manuals warn about "obstacles with small surface areas," "repeating patterns" and transparent surfaces. DJI's LiDAR add-on for the Matrice 4 is marketed as detecting wires down to 12 mm. Stereo-camera forum threads show the repetitive-pattern failure as phantom near obstacles from grilles and grates. I found no first-hand report of an OAK-D, ZED or VOXL camera reliably seeing a drone-cage net.

### Cited Findings
- Skydio 2 Safety and Operating Guide, verbatim: "Skydio 2 can't see certain visually challenging obstacles. Do not fly around thin branches, telephone or power lines, ropes, netting, wires, chain link fencing or other objects less than ½ inch in diameter." Also: "Do not fly around transparent surfaces like windows or reflective surfaces like mirrors greater than 60cm wide." And: "If your Skydio 2 does hit an obstacle it can't see, it will do its best to stabilize itself and continue the flight." It also says it "won't work at night or in other low-light situations." — [Skydio 2 Safety and Operating Guide (PDF, about 2019-2020)](https://support.skydio.com/hc/article_attachments/360052168754/Skydio_2_Safety_and_Operating_Guide.pdf)
- Skydio blog: obstacles thinner than 1/2 inch are typically not identified by the navigation system. — [Skydio blog, How to fly safely with Skydio 2](https://skydio.com/blog/how-to-fly-safely-with-skydio-2) (snippet only; page now returns 404)
- Skydio X10 / X10D manuals: use caution around small obstacles "(e.g., thin branches, utility lines, or chain link fencing)." Obstacle avoidance may be degraded around transparent or reflective surfaces over 23 in (58 cm) wide. It is disabled during launch and landing. X2D close-proximity settings go down to "Minimal (~4\", 10 cm)" for tight indoor spaces. — [Skydio support attachments](https://support.skydio.com/hc/article_attachments/28469442092571), [Skydio support attachment](https://support.skydio.com/hc/article_attachments/41897370107803) (snippet only)
- DJI Air 3 manual (p. 57, transcribed on a forum): the vision systems cannot work properly when "flying near obstacles with small surface areas (e.g., tree branches and power lines)." The same list covers monochrome surfaces, highly reflective, water or transparent surfaces, moving objects, rapidly changing light, and repeating patterns such as tiles. — [MavicPilots forum transcription](https://mavicpilots.com/threads/settings.143545/post-1616681) (secondary; official PDF not retrieved)
- A DJI forum user quotes a manual: the upward infrared system "only detects straight line distances directly above the sensor" and "tiny obstacles such as leaves or electrical wires cannot" be detected. — [DJI forum thread](https://forum.dji.com/thread-199259-1-1.html) (snippet only)
- DJI's LiDAR-based Obstacle Sensing Module for the Matrice 4 series is reported to detect wires as thin as 12 mm at up to 15 m/s. — [DroneDJ, 2025-05-07](https://dronedj.com/2025/05/07/dji-drone-obstacle-sensing-module/) (snippet only)
- UK CHIRP report DUAS 0039: a DJI Mini 5 became entangled after a single fine branch went undetected. The board notes that very thin branches or wires against a grey background catch pilots out. — [CHIRP DUAS 0039](https://chirp.co.uk/report/duas-0039/) (snippet only)
- Luxonis forum, "Wrong depth estimation caused by repetitive patterns": a user running OAK-D (Gen 1) for robot obstacle avoidance reported false close-range depth on a fridge grill and a door grill, which created ghost obstacles. They asked for an Intel-style second-peak threshold. — [Luxonis forum 1961](https://discuss.luxonis.com/d/1961-wrong-depth-estimation-cased-by-repetitive-patterns) (snippet only; the thread page failed to load, so the staff reply and any fix are unverified)
- RealSense community: a ventilation grille gave inaccurate depth. The same effect is described for a row of fence posts, a row of tree-tops, or stacked blind slats, and the High Accuracy preset is suggested to filter the moire-like patterns. — [RealSense support community](https://support.realsenseai.com/hc/en-us/community/posts/4406303633171/comments/4406303976595) (snippet only)
- Aix-Marseille / ISM, ICUAS 2023 (Warsaw, pp. 32-39, DOI 10.1109/ICUAS57906.2023.10156597), "Safety Net Detection by Optic Flow Processing": safety nets separating two contiguous spaces are hard to detect for lidar and for pattern-recognition image processing. The paper detects the net from the optic-flow pattern instead. — [ISM / HAL record](https://ism.univ-amu.fr/doc/hal-04139578/_1); [HAL CV listing](https://cv.archives-ouvertes.fr/romain-raffin/anrProjectId_i/54315) (abstract-level only; full text blocked)
- NVIDIA Isaac ROS DNN stereo page: a qualitative figure shows FoundationStereo handling a distant plant and railings with smoother estimates than ESS. Railings are a thin-structure case. — [isaac_ros_dnn_stereo_depth](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_stereo_depth)

### Inferences
- The commercial systems with the most engineering behind them (Skydio's six-camera deep-learned stereo; DJI's omnidirectional vision) explicitly disclaim nets and chain-link. A 7.5 cm baseline running classical SGM will not do better. Treat the net as invisible to the depth pipeline beyond about 1.5 m, and only intermittently visible closer.
- The grille and fence-post reports point to the other risk: phantom near obstacles near the net. That means unplanned stops or dodges, not just missed detections. The planner needs temporal persistence or multiple hits before reacting (see section 4).

### Gaps
- I found no forum or GitHub report specifically about OAK-D, ZED, VOXL stereo or RealSense D4xx pointed at a drone-cage net, and no video evidence I could verify.
- I could not retrieve an official DJI PDF wording that names "nets" or "netting." The DJI evidence is through forum transcriptions.
- I did not find how Skydio handles nets beyond the warnings, such as whether it has a dedicated thin-obstacle network.

---

## 3. Published methods for detecting nets, fences, wires and thin structures, and whether they fit a Jetson Orin Nano

### Takeaway
Nobody ships a mature, Jetson-ready "net detector." The closest pieces are:
- learned wire segmentation (CMU: more than 3 Hz on a 2017 Jetson TX2);
- thin-structure edge reconstruction (CVPRW 2017);
- fence segmentation from de-fencing work (CNN or U-Net, desktop-scale);
- lattice or periodicity detection (2008, brittle);
- learned stereo (NVIDIA ESS: 35 fps full or 82 fps light on Orin Nano Super);
- monocular foundation depth (Depth Anything; no Orin Nano V2 benchmark found);
- an ICUAS 2023 optic-flow safety-net detector.

The only dataset with an explicit "fences and nets" class (DDOS, CVPRW 2024) is synthetic aerial imagery. Some of these can be built in days on the color camera, where strands are 3-4x wider in pixels than on the mono pair: a periodicity or edge-density detector, or a small segmentation net fine-tuned on a few hundred labeled frames of the actual netting.

### Cited Findings
**Thin-structure and wire detection**
- Zhou et al., "Fast, Accurate Thin-Structure Obstacle Detection for Autonomous Mobile Robots" (CVPR 2017 workshops, arXiv 1708.04006). It targets wires, cables and branches, noting they are hard for lidar, sonar and stereo. Obstacles are modeled as image edges and reconstructed in 3D with edge-based visual odometry. There is a monocular+IMU version for scale and a stereo version. — [arXiv 1708.04006](https://arxiv.org/abs/1708.04006v1); [CVF open access](https://openaccess.thecvf.com/content_cvpr_2017_workshops/w4/papers/Zhou_Fast_Accurate_Thin-Structure_CVPR_2017_paper.pdf)
- Keller & Scherer (2019): trinocular window-based stereo raised wire-pixel detection from 5 % (binocular) to about 44 %. Fusing with a semantic wire detector reached about 53 %. A polarization camera handled reflective objects, with 9.27 % bad pixels vs 18.4 % for typical stereo. — [arXiv 1910.04874](https://arxiv.org/abs/1910.04874)
- Madaan, Maturana & Scherer, IROS 2017. Wire detection is framed as semantic segmentation with dilated CNNs, pretrained on synthetic wires rendered over 67K frames of flight video, then fine-tuned on real data. The best models ran at more than 3 Hz on a Jetson TX2 at 480x640, with AP 0.73 on the USF test split. They beat traditional CV and FCN, SegNet and E-Net baselines. — [CMU RI publication page](https://www.ri.cmu.edu/publications/wire-detection-using-synthetic-data-and-dilated-convolutional-networks-for-unmanned-aerial-vehicles); [paper PDF](https://www.ri.cmu.edu/app/uploads/2017/08/root.pdf)
- Madaan's 2018 CMU MS thesis (CMU-RI-TR-18-61) extends this to 3D wire reconstruction with a catenary model and shows avoidance on a real UAV. — [CMU RI thesis page](https://www.ri.cmu.edu/publications/wire-detection-reconstruction-andavoidance-for-unmanned-aerial-vehicles)
- LS-Net (arXiv 2019): a single-shot power-line segment detector (VGG-16 backbone, synthetic training) at about 20.4 FPS; the hardware is not stated in what I saw. — [arXiv 1912.09532](https://arxiv.org/pdf/1912.09532) (snippet only)

**Fence and net datasets and segmentation**
- DDOS, Drone Depth and Obstacle Segmentation (Kolbeinsson & Mikolajczyk, Imperial College, CVPRW 2024):
  - Data: synthetic AirSim, 340 flights x 100 frames = 34,000 frames at 1280x720, with depth 0-100 m, segmentation, flow and normals.
  - Classes: "Small Mesh pertains to fences and nets"; Ultra Thin is wires and cables.
  - Monocular depth baselines, AbsRel: Ultra Thin about 0.95-1.04 for all three models; Small Mesh 0.115 (DepthFormer), 0.129 (BinsFormer), 0.178 (SimIPU).
  - The authors say Ultra Thin is "exceptionally challenging" and that their own collision avoidance still crashes on thin structures.
  - Hosted on Hugging Face (benediktkol/DDOS).
  - [arXiv 2312.12494](https://arxiv.org/html/2312.12494v2); [CVF](https://openaccess.thecvf.com/content/CVPR2024W/VDU/html/Kolbeinsson_DDOS_The_Drone_Depth_and_Obstacle_Segmentation_Dataset_CVPRW_2024_paper.html)
- EDFNet (N. Fathi, arXiv 2026-04-06): early fusion of RGB, depth and edge channels for thin-obstacle segmentation on DDOS. The best model (pretrained RGBDE U-Net) reached Thin-Structure Evaluation Score 0.244, mIoU 0.219 and boundary IoU 0.234, at 19.62 FPS on unnamed hardware. The rarest ultra-thin classes stay low for every model. — [arXiv 2604.09694](https://arxiv.org/abs/2604.09694)
- De-fencing (fence segmentation as the first step):
  - Liu, Belkina, Hays & Lublinerman, "Image De-fencing," CVPR 2008. It finds the fence as a deformed lattice using translational symmetry (via Hays et al.'s near-regular lattice detector), separates layers by appearance regularity, and inpaints, recovering backgrounds up to 53 % mesh-occluded. A later paper notes the lattice detector "is not a robust approach and fails for many real world images." — [CMU RI page](https://www.ri.cmu.edu/publications/image-de-fencing); [PDF](https://faculty.cc.gatech.edu/~hays/papers/DF.pdf); [follow-up, arXiv 1612.03273](https://arxiv.org/pdf/1612.03273)
  - Jonna, Nakka & Sahay (arXiv 2016): pretrained CNN fence-pixel segmentation from a single image, used for video de-fencing. — [arXiv 1609.07727](https://arxiv.org/pdf/1609.07727)
  - Video de-fencing with CNNs and temporal information (ICME 2018). — [arXiv 1806.10781](https://arxiv.org/pdf/1806.10781)
  - Efficient Flow-Guided Multi-frame De-fencing (2023): a U-Net fence segmenter trained from scratch on ground-truth fence masks overlaid on DAVIS images with BCE loss. This recipe can be copied for synthetic net masks. — [arXiv 2301.10759](https://arxiv.org/pdf/2301.10759)
  - Two-stage CNN for fence segmentation and completion (2024), noting fence textures vary widely in shape, color and pattern. — [arXiv 2409.13242](https://arxiv.org/abs/2409.13242v1)
  - Stereo image de-fencing with smartphones (2016). — [arXiv 1612.01323](https://arxiv.org/pdf/1612.01323)
- Optic-flow safety-net detection (ICUAS 2023), see section 2. — [ISM / HAL record](https://ism.univ-amu.fr/doc/hal-04139578/_1)

**Monocular and learned depth**
- Depth Anything V2 (2024) targets fine detail, for example thin objects like chair legs. It notes V1 was vulnerable to transparent objects and reflections, and moves to synthetic-image training. — [arXiv 2406.09414](https://arxiv.org/html/2406.09414v2)
- LayeredDepth / "Seeing and Seeing Through the Glass" (ICCV 2025): most monocular methods fail on clean transparent surfaces. DAv2 was best but reached only 85.34 % pair-wise and 70.43 % quadruplet-wise accuracy. Fine-tuning Metric3D V2 on their synthetic transparent data raised quadruplet accuracy from 55.14 % to 75.20 %. — [arXiv 2503.11633](https://arxiv.org/pdf/2503.11633)

**Jetson runtimes**
- NVIDIA Isaac ROS ESS (learned stereo disparity), measured on Orin Nano Super 8 GB: ESS Full at 576p, 34.8 fps (30 ms); ESS Light at 288p, 82.2 fps (13 ms). AGX Orin: 66.5 and 75.5 fps. Last update listed 2026-09-21 (benchmarks reference release-5.0). — [isaac_ros_dnn_stereo_depth](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_stereo_depth)
- A user on Orin Nano 8 GB with JetPack 5.1.2 reported ESS disparity fps "much lower than that on ORIN" (no number given). — [NVIDIA forum](https://forums.developer.nvidia.com/t/nitros-negotiation-failed-in-isaac-ros-dnn-stereo-depth/290890) (snippet only)
- Depth Anything 3 (Small) ROS 2 wrapper: TensorRT 10.3 reached 43+ FPS (about 23 ms) on a Jetson Orin NX 16 GB, vs 6.35 FPS in PyTorch. Another user on an Orin Nano 8 GB reported about 0.12 s per forward pass. — [Open Robotics Discourse, GerdsenAI release](https://discourse.openrobotics.org/t/release-gerdsenais-depth-anything-3-ros2-wrapper-with-real-time-tensorrt-for-jetson/52364) (snippet only)
- NVIDIA NGC Depth Anything V2 (Large) at 518x914: about 3.7 FPS FP16 via trtexec on Jetson AGX Orin. — [NGC model card](https://catalog.ngc.nvidia.com/orgs/nvidia/teams/tao/models/nvdepthanythingv2) (snippet only)

### Inferences
Ranked by what a small team can build in days:
1. **Periodicity or edge-density detector on the color camera.** Take the IMX378 at 1080p (2 mm strand about 1 px at 2.9 m; 3 cm mesh 14.5 px period at 3 m) or crop a full-resolution ROI (1 px at 6 m). Tile the image into 64-128 px patches and compute a 2D FFT or autocorrelation. Flag patches with strong off-center peaks at a period consistent with the known mesh size over the expected range (period_px = fx * P / Z). That peak also gives a rough range estimate. Liu et al. 2008 shows lattices are a usable cue, but warns lattice fitting is brittle, so use only the spectral peak, not a full lattice fit. This approach is my design, not from a paper. It costs a few ms per frame with cuFFT or OpenCV.
2. **Color or edge segmentation, if the net color differs from the background** (for example black net vs a light hall, or a colored net). Combine an HSV threshold with high Scharr-edge density. Validate on recorded footage first.
3. **Fine-tuned small segmenter** (U-Net or BiSeNet class) on a few hundred labeled frames of the real net. Augment with synthetic net masks pasted onto indoor images, following the de-fencing U-Net recipe ([arXiv 2301.10759](https://arxiv.org/pdf/2301.10759)) and Madaan's synthetic-wire recipe. Madaan reached more than 3 Hz on a TX2 in 2017. A modern small segmenter in TensorRT FP16 on Orin Nano should run tens of fps at 640x480, but that is unbenchmarked.
4. **Learned stereo (ESS Light 288p, 82 fps; ESS Full 576p, 35 fps)** on the raw OAK rectified pair. This is the only learned stereo with measured Orin Nano numbers. It still runs at or below the native resolution, so the sub-pixel strand problem remains. Expect better handling of thin railings than SGM, not of 2 mm strands at 3 m.
5. **Monocular depth (Depth Anything V2/3)**: DDOS's small-mesh AbsRel of about 0.12 suggests mesh regions can get plausible depth when the model was trained on them, in synthetic aerial scenes. No evidence shows off-the-shelf DAv2 puts a translucent indoor net at the net's depth instead of the background's. It also uses GPU time that cuVSLAM needs. It is low priority.

Cheap hints the net is there, without detecting the net itself:
- Posts and frame members detected in stereo depth.
- A strong change in the stereo valid-pixel fraction in the ROI.
- Disagreement between stereo depth and the geofence-predicted distance to the wall.

### Gaps
- No Orin Nano TensorRT FPS figure for Depth Anything V2 Small, Metric3D or UniDepth was found.
- No paper evaluates any detector on indoor drone-cage netting specifically. The ICUAS 2023 optic-flow paper's method, test distances and success rates could not be read.
- No measured runtimes on Orin-class hardware for Zhou et al. 2017 or for fence-segmentation networks.

---

## 4. Non-perception fallbacks: geofence, unknown-as-occupied, standoff margins, landmarks, calibration, drift budget

### Takeaway
The main defense against the net should be a virtual box in the VIO frame, enforced by the companion (clamp every setpoint), with an inset margin sized from drift, stopping distance, vehicle size and net sag. A PX4 geofence (cylinder or polygon around a set EKF origin) should be the independent backstop with a Land or Hold action. PX4's own Collision Prevention treats directions with no sensor data as blocked by default (CP_GO_NO_DATA = 0), which is a precedent for "unknown = occupied." It runs only in Position mode, not Offboard. With 2-3 % drift, a 20-60 m flown path accumulates about 0.4-1.8 m of position error. Re-anchoring on known landmarks (posts, floor tags, take-off pad) shrinks the margin a lot.

### Cited Findings
- **PX4 geofence (docs):**
  - Two kinds. The failsafe geofence is a cylinder centered on home with a maximum radius and altitude; the breach action is set in the Geofence Failsafe settings (warning or Return are named). The Geofence Plan holds polygon or circular inclusion and exclusion areas planned in QGroundControl.
  - Geofences apply in all modes, including missions and manual flight.
  - A geofence that does not include home is rejected, and so is one that would be breached immediately after upload in flight.
  - [PX4 Geofence](https://docs.px4.io/main/en/flying/geofence)
- **PX4 indoors without GPS (community reports):**
  - QGC will not show or plan for the vehicle without a lat/long reference. The global origin must be set and kept in sync if the VIO origin resets. — [PX4 Discuss 22793](https://discuss.px4.io/t/how-to-use-indoor-positioning-with-global-coordinates-qgroundcontrol/22793); [PX4 Discuss 31629](https://discuss.px4.io/t/is-this-possible-to-plan-atonomous-flights-without-gps-with-positioning-system-installed-inside-with-qgroundcontrol/31629)
  - PX4 does not support missions in local coordinates, so local control means Offboard. — [PX4 Discuss 31629](https://discuss.px4.io/t/is-this-possible-to-plan-atonomous-flights-without-gps-with-positioning-system-installed-inside-with-qgroundcontrol/31629)
  - Users report Offboard rejected or exiting without GPS even with vision fusion configured. — [PX4 Discuss 34156](https://discuss.px4.io/t/offboard-mode-with-only-vision-positioning-no-gps/34156); [PX4 Discuss 48291 (Orin Nano)](https://discuss.px4.io/t/offboard-mode-exits-without-gps-velocity-control-using-vision-only-camera-data-on-a-jetson-orin-nano/48291)
- **PX4 Collision Prevention (docs):**
  - Multicopter Position mode only, with MPC_POS_MODE = Acceleration based. The page does not cover Offboard or Mission.
  - Companions feed it with OBSTACLE_DISTANCE (or ROS 2). Testing used 10 Hz at 4 m/s.
  - CP_DIST is the minimum distance (negative disables). CP_DELAY is sensor plus tracking delay (0.1-0.5 s typical; up to 0.2 s sensor delay for external vision). CP_GUIDE_ANG allows deviation (30 deg suggested).
  - CP_GO_NO_DATA defaults to 0, so velocity toward directions with no sensor data is restricted to zero.
  - If range data stops for more than 0.5 s, xy setpoints go to zero. After 5 s the vehicle switches to Hold.
  - The page warns the companion path is untested and the original companion project is archived.
  - [PX4 Collision Prevention](https://docs.px4.io/main/en/computer_vision/collision_prevention)
- **Conservative depth handling for drones (RealSense whitepaper):**
  - Prefer "fewer depth values at a high confidence."
  - Mask depth to Scharr-edge and Harris-corner pixels. This cut valid depth from 25 % to 10 % of pixels and outliers from 0.3 % to 0 %, at about 0.6 ms/frame.
  - Decimate 4x keeping the closest nonzero depth in each block, "since the closest object matters most for avoidance."
  - Accumulate into a spatial map such as Octomap that needs multiple hits across frames or viewpoints, to suppress phantom points.
  - [RealSense drone whitepaper](https://dev.realsenseai.com/?p=41319)
- **Precedents from an indoor caged competition, IMAV 2026 (rulebook V3, July 2026):**
  - Arena is a 14 m x 7 m x 4 m flight cage.
  - Textured flooring (flower wallpaper) is added under the obstacle course and side passages "to improve navigation."
  - Teams may install beacons (UWB or other) in the workspace, but doing so drops the autonomy factor to 0.7.
  - Off-board computation also gives 0.7, and manual actions give 0.4.
  - [IMAV 2026 Rulebook V3](https://2026.imavs.org/wp-content/uploads/2026/07/Rulebook_IMAV2026_V3.pdf)
- Context from the brief: cuVSLAM VIO measured about 2-3 % return-to-start drift on handheld walks, with occasional glitches on blank walls at close range. A blank wall at close range is the situation near an enclosure wall if the background is featureless. — assignment brief

**Drift and margin arithmetic (my calculation).** Error is about drift % x path length flown since the last absolute fix.

| Path flown | 2 % | 3 % | 5 % (pessimistic for flight vs handheld) |
|---|---|---|---|
| 10 m | 0.20 m | 0.30 m | 0.50 m |
| 20 m | 0.40 m | 0.60 m | 1.00 m |
| 30 m | 0.60 m | 0.90 m | 1.50 m |
| 50 m | 1.00 m | 1.50 m | 2.50 m |
| 100 m | 2.00 m | 3.00 m | 5.00 m |

### Inferences
- **Margin budget.** Inset = drift error + boundary-measurement error + stopping distance at the clamp-limited speed + vehicle half-span + net sag or bulge. Typical values:
  - Vehicle half-span: a 7-inch quad is about 0.25-0.3 m including props.
  - Net sag or bulge: unknown, but loose nets can hang inward tens of cm. Check on site.
  - Example: a 15 x 10 m course, 40 m flown, 3 % drift gives 1.2 m. Add 0.2 m measurement, 0.7 m stop at 2 m/s, 0.3 m half-span and 0.2 m sag, for about 2.6 m inset from the net. That uses half the width of a 10 m course, which shows that drift resets matter more than raw drift.
  - With a re-anchor at every lap or gate (drift path about 10 m), the inset drops to about 1.5 m.
- **Clamp speed near the virtual wall.** Velocity toward the wall should not exceed sqrt(2 * a * (d_wall - margin)), the same logic PX4 Collision Prevention uses. That way a drift error cannot turn into a fast impact.
- **Two independent layers:**
  1. Companion box. Pros: works in Offboard, uses the cuVSLAM frame directly, and is easy to test in sim and on a tether.
  2. PX4 geofence. Set an EKF global origin at power-up, which is a known workaround but must be validated on the bench. Use a cylinder radius or polygon slightly outside the companion box, with action Land or Hold, as protection against companion crashes or VIO jumps. A VIO jump moves both layers with the estimate, so neither protects against a large odometry glitch. Pair them with a VIO health monitor: tracking-lost flag, a velocity-innovation spike, or a disagreement between cuVSLAM and IMU-integrated velocity. Trigger hold or land on a fault.
- **Unknown-as-occupied, scoped carefully.** Treat invalid-depth pixels in a near-field forward ROI (for example 0.3-1.5 m band, central 30 deg) as occupied only when the vehicle is also within about 2.5 m of a virtual wall, or when the invalid fraction jumps suddenly. Blank walls and floors also produce invalid depth (Luxonis docs), so a blanket rule would freeze the drone mid-course.
- **Persistence before reacting.** Use min-pooling decimation and an occupancy grid that needs multiple hits. This handles the phantom near obstacles that grilles and mesh produce (sections 1-2).
- **Landmarks and take-off calibration:**
  - Net posts, frame tubes, floor markings and the take-off pad are large and stereo-visible.
  - At take-off, yaw slowly through 360 deg (or along the course axis). Range the posts or frame with stereo, or tag them with AprilTag or ArUco if the rules allow; the IMAV precedent is that beacons may cost autonomy points. Fit the rectangle in the VIO frame and set the virtual box from the measured rectangle rather than the nominal dimensions.
  - Re-anchor whenever a known landmark is re-observed. cuVSLAM's SLAM or loop-closure mode, or a tag pose, can reset accumulated drift.
  - Do not fly toward the net to "calibrate." Stereo will not see it in time.
- **Use the floor.** IMAV's added floor texture is a cue that organizers expect VIO to need help near walls. A downward-looking view of floor texture or floor tape lines gives both VIO features and a direct visual boundary cue that does not depend on seeing the net.

### Gaps
- I did not read the full PX4 parameter reference for GF_SOURCE, GF_ACTION, GF_PREDICT or GF_MAX_HOR_DIST. Whether the geofence can be enforced from vision-only local position (with a set origin) needs bench verification on the current PX4 release.
- No published flight (not handheld) drift figure for cuVSLAM on this camera. Drift during aggressive flight may exceed the 2-3 % handheld figure.
- The real net sag or bulge of the competition enclosure is unknown.

---

## 5. What indoor competitions with netted arenas say about touching the net, and how teams avoided it

### Takeaway
I found no public rulebook that sets an explicit penalty for touching the safety net. Indoor and caged events instead frame it as "stay within the flight zone": leaving it requires land or motor cut, and violations bring "penalties or disqualification." They also require a mandatory kill switch and pre-flight safety certification (IMAV 2016-2026). Read the competition's own rules for net-contact wording, and ask the organizers directly.

### Cited Findings
- **IMAV 2026 (rulebook V3, July 2026), general rules:**
  - "All MAVs must remain within designated flight zones; if a MAV strays beyond these zones, it is required to activate a land measure, and finally, if the drone goes too far, it must cut its engines." "Failure to comply with any of these safety measures will lead to penalties or disqualification."
  - "Failure to abide by the boundaries and frequencies will lead to a severe penalty and disqualification."
  - A kill switch on the RC is mandatory, and a per-drone "Green Card" safety certification is required on training day. Indoor take-off weight is at most 3 kg, with at most 3 drones flying at once indoors.
  - Indoor arena: a 14 x 7 x 4 m flight cage in the INSA Strasbourg hall, with textured flooring added "to improve navigation."
  - The indoor obstacle-course mission scores windows, bars and a pipe passage; I found no net-contact term in the scoring table.
  - [IMAV 2026 Rulebook V3](https://2026.imavs.org/wp-content/uploads/2026/07/Rulebook_IMAV2026_V3.pdf)
- IMAV 2017 general rules: crossing the no-fly boundary requires motors off, and failing to respect boundaries or frequencies can lead to a penalty or disqualification. IMAV 2016: teams failing to comply with safety rules will be disqualified. IMAV 2023 indoor rules defer safety to a separate safety-regulations document. — [IMAV 2017 General Rules v2.3](https://www.imavs.org/2017/content/uploads/2017/09/IMAV2017_General_Rules_v2_3.pdf); [IMAV 2016 rules](https://www.imavs.org/2016/documents/IMAV2016_Competition_rules_v2.3.pdf); [IMAV 2023 indoor rules](https://2023.imavs.org/wp-content/uploads/2023/08/IMAV2023_Indoor_CompetitionRules_v1.1.pdf) (snippet only for 2016, 2017 and 2023)
- FAI 2025 drone racing sporting rules have "Crash" and "Safety issue" sections and ban automatic positioning systems (manual racing). — [FAI CIAM 2025 drone racing rules](https://ciam.fai.org/sites/default/files/ciam/wcup_drones/twg_2025_drone_racing_sporting_rules_v3.pdf) (snippet only; section text not read)
- A university drone obstacle-course rulebook tells pilots who lose control to fly into a prescribed crash "catch" zone net. No penalty for net contact is stated. — [ULM drone obstacle course rules](https://www.ulm.edu/cbss/pa-uas/documents/drone-obstacle-course-rules.pdf) (snippet only)
- A2RL autonomous drone racing (2025-2026) limits drones to one forward monocular RGB camera plus IMU, all processing onboard. This shows top autonomous racing teams deliberately work without stereo. — [ADNEC press release](https://www.adnec.ae/en/press-and-media/press-release-listing/ai-takes-flight-in-a2rl-drone-championship-at-umex-2026-with-live-ai-vs-human-pilot-showdown) (snippet only; no net rules found)

### Inferences
- Treat any net contact as at least a failed run and possibly a safety violation. With the "land if you stray" pattern, an autonomous boundary-breach behavior (hold, then land) is likely expected and is worth demonstrating during safety certification.
- What caged-event teams publicly do: rely on odometry plus known arena geometry and floor texture, sometimes beacons or markers if allowed. IMAV 2026's textured floor and beacon clause show this is the norm, not detecting the net itself.

### Gaps
- I found no published team report or paper describing how IMAV indoor or other caged-arena teams kept clear of the net, and no explicit net-touch penalty in any rulebook I could open.
- The IMAV safety-regulations document referenced by the 2023 indoor rules was not retrieved.
- The competition's own rulebook was not part of this research and should be checked for net-contact, boundary and beacon or marker clauses.
