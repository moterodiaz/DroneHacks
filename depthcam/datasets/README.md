# Datasets: recording, format, calibration

Everything a localisation method needs from the camera without the method itself.

| File | What it is |
|---|---|
| [07_record_dataset.py](07_record_dataset.py) | records the synchronised stereo pair (host-undistorted GRAY8) and the raw IMU into `recordings/euroc_<stamp>/` in the EuRoC ASL layout; `--help` for every flag |
| [dataset-format.md](dataset-format.md) | the layout, `sensor.yaml`, `dataset.json`, the TUM trajectory format, what NVIDIA's EuRoC loader needs |
| [fit_imu_calibration.py](fit_imu_calibration.py) | six-pose accelerometer / gyroscope fit (`--check` to see whether a stored fit still holds) |
| [calibration/](calibration/) | stored IMU corrections per device id (`imu_<id>.json`) |
| [bench_png_writer.py](bench_png_writer.py) | throughput benchmark of the PNG writer (how many encoder threads a machine needs at 1280x800 @ 30 fps) |

The writer / reader code is `cuvslam_oak/dataset.py` (EuRoC writer with a threaded PNG pipeline, reader,
validator), the camera pipeline is `cuvslam_oak/oak_source.py`, and the EEPROM-to-rig maths is
`cuvslam_oak/rig.py`; the recorder uses all three, so a new method can read `recordings/euroc_*` with
`cuvslam_oak.dataset.EurocReader` or any EuRoC loader.

```
.venv\Scripts\python datasets\07_record_dataset.py --seconds 60 --preview --note "loop 1"   # 1280x800 @ 30 fps, IMU 200 Hz
.venv\Scripts\python datasets\07_record_dataset.py --seconds 60 --width 640 --height 400       # lighter
.venv\Scripts\python datasets\fit_imu_calibration.py recordings\euroc_<six-pose recording>
```

Measured on this laptop: 1280x800 at 30 fps is drop-free with the default 4 encoder threads (about
1.5 GB per minute of PNGs); the recordings folder is git-ignored. Record in a lit, textured room and
read the walking protocol in [../methods/cuvslam-vio/docs/cuvslam.md](../methods/cuvslam-vio/docs/cuvslam.md)
section 3.2 before making a benchmark recording.
