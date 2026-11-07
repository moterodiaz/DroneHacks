# Public snapshot

This `depthcam/` folder comes from a private working branch, the team's lidar version of the
flight stack. That branch carries material from a private research repo, so before it was
published here, two kinds of detail were replaced with placeholders:

| What | In the docs and code | Replaced by |
|---|---|---|
| The OAK-D S2's device ID | examples like `--device-id …`, docs, test constants | `OAKDS2EXAMPLEID000` |
| Paths on the developers' machines | setup docs and one script comment | `<repo checkout>`, `/mnt/c/<path to the repo checkout>`, `C:\Users\<user>\`, `/home/<user>/`, `<scratchpad>` |

Two files were renamed to match the placeholder:

- `datasets/calibration/imu_OAKDS2EXAMPLEID000.json`, the measured IMU correction.
- `tests/data/calibration_OAKDS2EXAMPLEID000.json`, the camera's EEPROM calibration, used as a
  test fixture.

`cuvslam_oak/imu_calib.py` finds the IMU correction by the connected camera's ID
(`imu_<device id>.json`). With a real camera, rename the file to your camera's ID, or pass the
file explicitly. The drone package (`dronehacks_lidar`, see `methods/lidar-vo-fusion/`) does not
use this file. It runs on the organisers' cuVSLAM.

Nothing else was changed: the code, maps, routes and tests are the same as the team's working
branch.
