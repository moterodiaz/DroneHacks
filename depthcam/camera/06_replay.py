#!/usr/bin/env python3
"""06_replay.py - replay a recording made by 05_record.py (DepthAI v3, depthai==3.10.0).

What this test checks
---------------------
* ``--mode video`` (default): the recorded video file(s) can be decoded on the HOST with
  ``dai.node.ReplayVideo`` in host-only pipelines (``dai.Pipeline(createImplicitDevice=False)``).
  NO CAMERA IS NEEDED - the OAK stays unbooted (verified in camera/docs/depthai-v3-cheatsheet.md 8.1).
  Frames come back as ``dai.ImgFrame`` with the recorded sequence numbers and timestamps when
  the sidecar ``<name>.mcap`` metadata file is present, so this also checks that the metadata
  written by 05_record.py is readable.  The 16-bit depth PNG sequence that 05_record.py writes
  by default (``depth/depth_NNNNNN.png`` + ``depth_index.csv``, ``--depth-stream depth``) is
  replayed on the host as well: every PNG is read with ``cv2.imread(..., IMREAD_UNCHANGED)``,
  paced by the timestamps in the index and shown through ``common.colorize_depth`` (stream
  name ``depth``).
* ``--mode holistic``: a holistic recording (``recording_*.tar`` written by
  ``pipeline.enableHolisticRecord``) is fed back into real ``Camera`` (and optionally ``IMU``)
  nodes with ``pipeline.enableHolisticReplay``.  In depthai 3.10.0 this NEEDS THE CAMERA
  CONNECTED: the pipeline must own a device (``dai.Pipeline(device)``) because ``Camera`` is a
  device node (``createImplicitDevice=False`` raises "Pipeline is host only, cannot create
  device node").  The frames still come from the tar, not from the sensor.  The device is
  opened through ``common.open_device`` (``--device-id`` picks one).

Layout written by 05_record.py (what the positional ``recording`` argument points at)
--------------------------------------------------------------------------------------
    recordings/video_<stamp>/     rgb.mp4 + rgb.mcap (H264), depth/depth_NNNNNN.png +
                                  depth_index.csv  (or disparity.mkv + disparity.mcap with
                                  --depth-stream disparity), imu.mcap, calibration.json,
                                  session.json
    recordings/holistic_<stamp>/  recording_YYYY-MM-DD_HH-MM-SS.tar (CameraCAM_X.mp4 or .avi +
                                  CameraCAM_X.mcap, IMU.mcap, camera_info.json,
                                  record_config.json) + session.json

Accepted inputs (positional ``recording``)
------------------------------------------
* a folder written by 05_record.py: every ``*.mp4`` (H264, encoded) / ``*.mkv`` / ``*.avi``
  (raw) directly inside it is replayed, each with its ``<same stem>.mcap`` sidecar when present,
  plus the ``depth/`` PNG sequence when there is one; if the folder has no video files, the
  newest ``*.tar`` inside it is used (for ``--mode video`` its videos are extracted to a
  temporary folder first); if there is still nothing, the newest sub-folder that contains
  recording files is used;
* a single video file (``.mp4`` / ``.mkv`` / ``.avi``), or a ``.mcap`` next to its video;
* a holistic ``recording_*.tar`` (both modes), or a folder that holds ``camera_info.json`` plus
  ``CameraCAM_X.mcap`` files (holistic mode).

One pipeline per file
---------------------
A ``ReplayVideo`` node stops the WHOLE pipeline it lives in when it reaches the end of its
file (depthai-core Replay.cpp: "This will stop even if there is still frames in the
pipeline"), so two files of different length in one pipeline would lose the tail of the
longer one.  Video mode therefore builds one host-only pipeline per video file (and per
``--imu`` metadata file); every stream plays to its own end and the summary prints
``N of M frames (complete)`` per stream, M being the frame count cv2 probed from the container.

How to run (from the repo root)
-------------------------------
    python camera/06_replay.py recordings/<folder>                 # host only, no camera
    python camera/06_replay.py recordings/<folder>/rgb.mp4 --loop --speed 2
    python camera/06_replay.py recordings/<folder> --headless --duration 10
    python camera/06_replay.py recordings/<folder> --imu           # also replay IMU .mcap files
    python camera/06_replay.py recordings/<folder> --stream depth  # only the depth PNG sequence
    python camera/06_replay.py recordings/recording_2026-09-18_00-29-46.tar --mode holistic --duration 15

What you should see
-------------------
One OpenCV window per stream (title ``replay: <name>``; the name is the file stem, or
``<stem>_<ext>`` when two files share a stem, and ``depth`` for the PNG sequence) showing the
recorded frames with an overlay: frame index (counted here), recorded sequence number,
recorded timestamp, time since the first frame, frame size, measured display fps and the
requested speed factor (the depth window also shows the centre-pixel distance).  In
``--headless`` mode one status line per stream is printed once per second instead.  When every
stream has reached its end of file (and ``--loop`` is off) the script prints a summary
("Replay finished: <n> frames in <t> s", then one line per stream with the delivered frame
count, the recorded span and the EFFECTIVE speed) and exits 0.  In holistic mode the built-in
replay loops forever, so stop it with ``q``, ``--duration`` or Ctrl-C.

Keys (window focused): ``q`` or ``Esc`` quit, ``s`` saves every stream's current frame as a
PNG into ``<repo>/snapshots/`` (``--snapshot-dir`` changes it; the absolute path is printed).
Ctrl-C in the terminal also quits cleanly, even while a tar is being extracted.

Exit codes: 0 normal end (files finished, ``--duration`` reached, ``q``, Ctrl-C);
2 with ONE line on stderr when the path does not exist / holds no recording files, when no
frame could be decoded, or when ``--mode holistic`` finds no camera (via common.open_device).

Speed control (``--speed``) and what "real time" means here
-----------------------------------------------------------
``ReplayVideo`` paces frames by the recorded timestamps when a metadata file is present and
``setFps`` is not called; with ``setFps(x)`` it TARGETS one frame every ``round(1000/x)`` ms
(``sleep_until`` after each frame); without metadata it targets 30 fps.  The script leaves the
node alone for ``--speed 1`` with metadata and otherwise calls ``setFps(base * speed)`` where
``base`` is ``--fps`` or the container frame rate probed with ``cv2.VideoCapture`` (30 if the
probe fails).  Two caveats, both visible in the summary line ``effective speed``:

* the container fps is not necessarily the recorded rate (an OAK H264 recording probes as
  33 fps while its timestamps span ~30.5 fps), so ``--speed 3`` may be 3.25x nominal;
* the delivered rate is limited by the host timer: on Windows ``sleep_until`` has ~16 ms
  granularity, so a 33 ms interval becomes 31-47 ms (measured here: ~24 fps for setFps(30),
  ~35 fps for setFps(60), ~60 fps for setFps(99)); on Linux (Jetson) the sleeps are accurate
  to ~1 ms.  The depth PNG sequence is paced on the host by its own timestamps (or
  ``--fps``) times ``--speed`` and has the same timer limits.

Holistic mode has no speed control.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import shutil
import sys
import tarfile
import tempfile
import time
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import depthai as dai

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import (  # noqa: E402  (import after sys.path tweak on purpose)
    FpsCounter,
    Periodic,
    RunClock,
    add_device_arg,
    add_run_args,
    colorize_depth,
    draw_label,
    ensure_dir,
    open_device,
    print_device_summary,
    timestamp_name,
    wait_key_quit,
)

#: Exit status for "nothing to replay" (missing path, no recording files, undecodable video).
EXIT_BAD_INPUT = 2

VIDEO_EXTS = (".mp4", ".mkv", ".avi")
META_EXT = ".mcap"
TAR_EXT = ".tar"
#: 05_record.py --mode video --depth-stream depth (its default): depth/depth_NNNNNN.png + depth_index.csv
DEPTH_DIR_NAME = "depth"
DEPTH_INDEX_NAME = "depth_index.csv"
DEPTH_PNG_GLOB = "depth_*.png"
DEPTH_STREAM_NAME = "depth"
DEPTH_COLOR_MAX_MM = 5000.0
#: A holistic mp4 of exactly this size is a header-only file from a failed recording
#: (camera/docs/depthai-v3-cheatsheet.md section 7); replaying it makes the Camera node throw.
HEADER_ONLY_MP4_BYTES = 436
#: Default output request for holistic replay (the official holistic_replay.py asks for a
#: small fixed size and 30 fps regardless of the recorded resolution).
HOLISTIC_DEFAULT_SIZE = (640, 400)
HOLISTIC_DEFAULT_FPS = 30.0
FALLBACK_SOURCE_FPS = 30.0
#: Default snapshot folder: <repo root>/snapshots, independent of the current directory
#: (same convention as 01_rgb_preview.py).
DEFAULT_SNAPSHOT_DIR = Path(__file__).resolve().parent.parent / "snapshots"


# --------------------------------------------------------------------------------------
# Small helpers
# --------------------------------------------------------------------------------------

def fmt_td(td: Optional[dt.timedelta]) -> str:
    """``datetime.timedelta`` -> ``H:MM:SS.mmm`` (``-`` when unknown)."""
    if td is None:
        return "-"
    total_ms = int(round(td.total_seconds() * 1000.0))
    sign = "-" if total_ms < 0 else ""
    total_ms = abs(total_ms)
    hours, rem = divmod(total_ms, 3_600_000)
    minutes, rem = divmod(rem, 60_000)
    seconds, millis = divmod(rem, 1000)
    return "{}{}:{:02d}:{:02d}.{:03d}".format(sign, hours, minutes, seconds, millis)


def parse_size(text: str) -> Tuple[int, int]:
    """``'640x400'`` -> ``(640, 400)`` for argparse."""
    try:
        w_str, h_str = text.lower().replace(",", "x").split("x")
        width, height = int(w_str), int(h_str)
    except ValueError:
        raise argparse.ArgumentTypeError("expected WxH, e.g. 640x400, got {!r}".format(text))
    if width <= 0 or height <= 0:
        raise argparse.ArgumentTypeError("size must be positive, got {!r}".format(text))
    return width, height


def probe_video(path: Path) -> Tuple[int, int, float, int]:
    """``(width, height, fps, frame_count)`` via ``cv2.VideoCapture``; zeros when it cannot open.

    Best effort only: it is used for the default pacing fps and for the ``N of M frames``
    bookkeeping (the count can be off by one for .mkv files, where FFmpeg estimates it).
    """
    try:
        cap = cv2.VideoCapture(str(path))
        try:
            if not cap.isOpened():
                return 0, 0, 0.0, 0
            width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
            height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
            fps = float(cap.get(cv2.CAP_PROP_FPS) or 0.0)
            count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
        finally:
            cap.release()
    except Exception:  # noqa: BLE001 - probing must never abort the replay
        return 0, 0, 0.0, 0
    if not np.isfinite(fps) or fps <= 0.0 or fps > 1000.0:
        fps = 0.0
    return width, height, fps, max(0, count)


def fail(message: str) -> int:
    """Print ONE line to stderr and return the bad-input exit status."""
    print(" ".join(str(message).split()), file=sys.stderr, flush=True)
    return EXIT_BAD_INPUT


def name_matches(name: str, filters: Optional[Sequence[str]]) -> bool:
    """``--stream`` filter: no filter, or any given substring occurs in *name* (case-insensitive)."""
    if not filters:
        return True
    lowered = name.lower()
    return any(f.lower() in lowered for f in filters)


# --------------------------------------------------------------------------------------
# Recording discovery
# --------------------------------------------------------------------------------------

class VideoSource:
    """One replayable video file plus its optional ``.mcap`` metadata sidecar."""

    def __init__(self, video: Path, metadata: Optional[Path]) -> None:
        self.video: Path = video
        self.metadata: Optional[Path] = metadata
        self.name: str = video.stem           # made unique by dedupe_names() when stems collide
        self.kind: str = "encoded" if video.suffix.lower() == ".mp4" else "raw"


class DepthSequence:
    """The ``depth/depth_NNNNNN.png`` + ``depth_index.csv`` layout of 05_record.py."""

    def __init__(self, folder: Path) -> None:
        self.folder: Path = folder                                   # the recording folder
        self.pngs: List[Path] = sorted((folder / DEPTH_DIR_NAME).glob(DEPTH_PNG_GLOB))
        index = folder / DEPTH_INDEX_NAME
        self.index: Optional[Path] = index if index.is_file() else None

    def __len__(self) -> int:
        return len(self.pngs)


def find_depth_sequence(folder: Path) -> Optional[DepthSequence]:
    """``DepthSequence`` when *folder* holds ``depth/depth_*.png`` files, else None."""
    if not (folder / DEPTH_DIR_NAME).is_dir():
        return None
    try:
        seq = DepthSequence(folder)
    except OSError:
        return None
    return seq if seq.pngs else None


class Recording:
    """What was found at the user-supplied path."""

    def __init__(self, path: Path) -> None:
        self.path: Path = path
        self.videos: List[VideoSource] = []
        self.meta_only: List[Path] = []          # .mcap files without a matching video (IMU etc.)
        self.depth: Optional[DepthSequence] = None   # depth/ PNG sequence of a 05_record.py folder
        self.holistic: Optional[Path] = None     # .tar or holistic folder for enableHolisticReplay
        self.origin: str = ""                    # human-readable note about where files came from
        self._tempdir: Optional[tempfile.TemporaryDirectory] = None

    def temp_dir(self) -> Path:
        if self._tempdir is None:
            self._tempdir = tempfile.TemporaryDirectory(prefix="oak_replay_")
        return Path(self._tempdir.name)

    def cleanup(self) -> None:
        if self._tempdir is not None:
            try:
                self._tempdir.cleanup()
            except Exception:  # noqa: BLE001 - a locked temp file must not turn into a traceback
                pass
            self._tempdir = None


def _videos_in_dir(folder: Path) -> List[Path]:
    return sorted(p for p in folder.iterdir()
                  if p.is_file() and p.suffix.lower() in VIDEO_EXTS and not p.name.startswith("."))


def _tars_in_dir(folder: Path) -> List[Path]:
    tars = [p for p in folder.iterdir() if p.is_file() and p.suffix.lower() == TAR_EXT]
    return sorted(tars, key=lambda p: p.stat().st_mtime, reverse=True)   # newest first


def _mcaps_in_dir(folder: Path) -> List[Path]:
    return sorted(p for p in folder.iterdir() if p.is_file() and p.suffix.lower() == META_EXT)


def _is_holistic_dir(folder: Path) -> bool:
    """Layout that ``enableHolisticReplay`` accepts as a directory: camera_info.json + Camera*.mcap."""
    if not (folder / "camera_info.json").is_file():
        return False
    return any(p.stem.startswith("Camera") for p in _mcaps_in_dir(folder))


def _sidecar(video: Path) -> Optional[Path]:
    meta = video.with_suffix(META_EXT)
    return meta if meta.is_file() else None


def dedupe_names(videos: Sequence[VideoSource]) -> None:
    """Give files that only differ by extension (synthetic.mp4 + synthetic.avi) distinct names.

    The name is the window title, the ``--stream`` match key and the snapshot prefix, so two
    streams must never share one: colliding stems become ``<stem>_<ext>``.
    """
    counts: Dict[str, int] = {}
    for source in videos:
        key = source.video.stem.lower()
        counts[key] = counts.get(key, 0) + 1
    for source in videos:
        if counts[source.video.stem.lower()] > 1:
            source.name = "{}_{}".format(source.video.stem, source.video.suffix.lower().lstrip("."))


def _add_dir_files(rec: Recording, folder: Path) -> None:
    videos = _videos_in_dir(folder)
    stems = set()
    for video in videos:
        rec.videos.append(VideoSource(video, _sidecar(video)))
        stems.add(video.stem)
    dedupe_names(rec.videos)
    rec.meta_only = [m for m in _mcaps_in_dir(folder) if m.stem not in stems]
    rec.depth = find_depth_sequence(folder)


def list_tar_files(tar_path: Path) -> List[Tuple[str, int]]:
    """``[(file name, size)]`` of the regular files inside a tar (directory prefixes stripped)."""
    entries: List[Tuple[str, int]] = []
    with tarfile.open(str(tar_path), "r:*") as tar:
        for member in tar.getmembers():
            if member.isfile():
                entries.append((Path(member.name).name, int(member.size)))
    return entries


def extract_tar_files(tar_path: Path, dest: Path, suffixes: Sequence[str]) -> List[Path]:
    """Copy the regular files with one of *suffixes* out of *tar_path* into *dest* (flat).

    Members are written one by one from ``extractfile`` so no tar path can escape *dest*
    (works on Python 3.10 without the ``filter=`` argument of ``extractall``).
    """
    out: List[Path] = []
    wanted = tuple(s.lower() for s in suffixes)
    with tarfile.open(str(tar_path), "r:*") as tar:
        for member in tar.getmembers():
            if not member.isfile():
                continue
            name = Path(member.name).name
            if not name or name.startswith(".") or not name.lower().endswith(wanted):
                continue
            src = tar.extractfile(member)
            if src is None:
                continue
            target = dest / name
            with src, open(target, "wb") as dst:
                shutil.copyfileobj(src, dst)
            out.append(target)
    return out


def _use_tar_for_video(rec: Recording, tar_path: Path) -> Optional[str]:
    """Extract the videos (+ .mcap) of a holistic tar into the temp folder; returns an error text or None."""
    try:
        files = extract_tar_files(tar_path, rec.temp_dir(), VIDEO_EXTS + (META_EXT,))
    except (tarfile.TarError, OSError) as exc:
        return "Cannot read tar {}: {}".format(tar_path, exc)
    except BaseException:               # Ctrl-C half-way through a 20 MB extraction: no temp litter
        rec.cleanup()
        raise
    videos = [f for f in files if f.suffix.lower() in VIDEO_EXTS]
    if not videos:
        return "No video files (*.mp4/*.mkv/*.avi) inside {}".format(tar_path)
    stems = set()
    for video in sorted(videos):
        rec.videos.append(VideoSource(video, _sidecar(video)))
        stems.add(video.stem)
    dedupe_names(rec.videos)
    rec.meta_only = [f for f in files if f.suffix.lower() == META_EXT and f.stem not in stems]
    rec.holistic = tar_path
    rec.origin = "extracted from {} into {}".format(tar_path.name, rec.temp_dir())
    return None


def discover_recording(path: Path, mode: str) -> Tuple[Optional[Recording], str]:
    """Work out what to replay at *path*.  Returns ``(Recording, '')`` or ``(None, one-line reason)``."""
    if not path.exists():
        return None, "Recording path does not exist: {}".format(path)
    rec = Recording(path)

    if path.is_file():
        suffix = path.suffix.lower()
        if suffix == TAR_EXT:
            if mode == "holistic":
                rec.holistic = path
                return rec, ""
            err = _use_tar_for_video(rec, path)
            return (None, err) if err else (rec, "")
        if mode == "holistic":
            return None, ("--mode holistic needs a recording_*.tar (or a holistic folder), not {}; "
                          "use --mode video for a plain video file".format(path.name))
        if suffix in VIDEO_EXTS:
            rec.videos.append(VideoSource(path, _sidecar(path)))
            return rec, ""
        if suffix == META_EXT:
            for ext in VIDEO_EXTS:
                video = path.with_suffix(ext)
                if video.is_file():
                    rec.videos.append(VideoSource(video, path))
                    return rec, ""
            return None, ("{} is a metadata file with no matching video next to it "
                          "(expected {}.mp4/.mkv/.avi)".format(path.name, path.stem))
        return None, ("Not a recording file: {} (expected .mp4/.mkv/.avi/.tar or a folder)".format(path))

    if not path.is_dir():
        return None, "Recording path is neither a file nor a folder: {}".format(path)

    # Folder: files directly inside it, then a tar inside it, then the newest sub-folder.
    candidates = [path]
    try:
        subdirs = sorted((p for p in path.iterdir() if p.is_dir() and not p.name.startswith(".")),
                         key=lambda p: p.stat().st_mtime, reverse=True)
    except OSError as exc:
        return None, "Cannot list {}: {}".format(path, exc)
    candidates.extend(subdirs)

    for folder in candidates:
        try:
            videos = _videos_in_dir(folder)
            tars = _tars_in_dir(folder)
            holistic_dir = _is_holistic_dir(folder)
            depth = find_depth_sequence(folder) if mode == "video" else None
        except OSError:
            continue
        if not videos and not tars and not holistic_dir and depth is None:
            continue
        note = "" if folder == path else " (newest sub-folder of {})".format(path)
        if mode == "holistic":
            if holistic_dir:
                rec.holistic = folder
                rec.origin = "holistic folder {}{}".format(folder, note)
                return rec, ""
            if tars:
                rec.holistic = tars[0]
                rec.origin = "newest tar in {}{}".format(folder, note)
                return rec, ""
            continue  # plain videos cannot be replayed holistically; keep looking
        if videos or depth is not None:
            _add_dir_files(rec, folder)
            rec.origin = "folder {}{}".format(folder, note)
            return rec, ""
        if tars:
            err = _use_tar_for_video(rec, tars[0])
            if err:
                return None, err
            rec.origin += note
            return rec, ""
        if holistic_dir:
            _add_dir_files(rec, folder)      # a holistic folder also holds plain CameraCAM_X.mp4 files
            if rec.videos:
                rec.origin = "holistic folder {}{}".format(folder, note)
                return rec, ""

    if mode == "holistic":
        return None, ("No holistic recording (recording_*.tar or camera_info.json + Camera*.mcap) "
                      "found in {}".format(path))
    return None, "No recording files (*.mp4/*.mkv/*.avi/*.tar or depth/depth_*.png) found in {}".format(path)


# --------------------------------------------------------------------------------------
# Per-stream bookkeeping and the shared display / stats loop
# --------------------------------------------------------------------------------------

class Stream:
    """Statistics and display bookkeeping of one replayed stream (subclasses supply ``poll``)."""

    def __init__(self, name: str, description: str, expected: int = 0,
                 has_timestamps: bool = False, target_fps: Optional[float] = None) -> None:
        self.name: str = name
        self.description: str = description
        self.expected: int = int(expected)             # frame count known up front (0 = unknown)
        self.count_source: str = "cv2 probed"           # where ``expected`` came from (summary text)
        self.has_timestamps: bool = has_timestamps      # recorded timestamps (else wall-clock ones)
        self.target_fps: Optional[float] = target_fps   # pacing target when one was requested
        self.frames: int = 0
        self.first_seq: int = -1
        self.last_seq: int = -1
        self.first_ts: Optional[dt.timedelta] = None
        self.last_ts: Optional[dt.timedelta] = None
        self.first_wall: Optional[float] = None
        self.last_wall: Optional[float] = None
        self.width: int = 0
        self.height: int = 0
        self.fps: FpsCounter = FpsCounter(window_s=2.0)
        self.image: Optional[np.ndarray] = None         # freshest frame as BGR/gray (window mode)
        self.extra: str = ""                            # optional extra overlay text
        self.last_img: Optional[np.ndarray] = None      # last displayed frame (for snapshots)
        self.finished: bool = False                     # source ended (EOF or its pipeline stopped)
        self.window: str = "replay: {}".format(name)

    def poll(self, want_image: bool) -> bool:
        """Fetch at most one new frame; True when one arrived (``self.image`` set if wanted)."""
        raise NotImplementedError

    def _note(self, seq: int, ts: dt.timedelta, width: int, height: int) -> None:
        now = time.monotonic()
        if self.first_ts is None:
            self.first_ts = ts
            self.first_wall = now
            self.first_seq = seq
        self.last_ts = ts
        self.last_wall = now
        self.last_seq = seq
        self.width, self.height = width, height
        self.frames += 1
        self.fps.tick()

    def rel_seconds(self) -> float:
        if self.first_ts is None or self.last_ts is None:
            return 0.0
        return (self.last_ts - self.first_ts).total_seconds()

    def wall_seconds(self) -> float:
        if self.first_wall is None or self.last_wall is None:
            return 0.0
        return self.last_wall - self.first_wall

    def effective_speed(self) -> Optional[float]:
        """Recorded span / wall-clock span between the first and last frame (needs recorded timestamps)."""
        if not self.has_timestamps or self.frames < 2:
            return None
        wall = self.wall_seconds()
        rel = self.rel_seconds()
        if wall <= 0.0 or rel <= 0.0:
            return None
        return rel / wall

    def count_text(self, looping: bool = False) -> str:
        if self.expected <= 0:
            return "{} frames".format(self.frames)
        if looping:
            return "{} frames ({} per pass, looping)".format(self.frames, self.expected)
        if self.frames == self.expected:
            return "{} of {} frames (complete)".format(self.frames, self.expected)
        if self.frames > self.expected:
            return "{} frames ({} {})".format(self.frames, self.count_source, self.expected)
        return "{} of {} frames".format(self.frames, self.expected)

    def status(self) -> str:
        return "{}: frames={} seq={} ts={} t+{:.2f}s {}x{} {:.1f} fps".format(
            self.name, self.frames, self.last_seq, fmt_td(self.last_ts), self.rel_seconds(),
            self.width, self.height, self.fps.fps())


class QueueStream(Stream):
    """Output queue of a ``ReplayVideo`` (video mode) or ``Camera`` (holistic mode) node."""

    def __init__(self, name: str, queue: dai.MessageQueue, pipeline: dai.Pipeline, description: str,
                 expected: int = 0, has_timestamps: bool = False, target_fps: Optional[float] = None) -> None:
        super().__init__(name, description, expected, has_timestamps, target_fps)
        self.queue: dai.MessageQueue = queue
        self.pipeline: dai.Pipeline = pipeline

    def poll(self, want_image: bool) -> bool:
        if self.finished:
            return False
        try:
            msg = self.queue.tryGet()
        except dai.MessageQueue.QueueException:      # queue closed: the node stopped its pipeline
            self.finished = True
            return False
        if msg is None:
            if not self.pipeline.isRunning():        # stopped and nothing left in the queue
                self.finished = True
            return False
        ts = msg.getTimestamp()
        if ts.total_seconds() <= 0.0:                # no metadata: the node only fills the device timestamp
            ts = msg.getTimestampDevice()
        self._note(int(msg.getSequenceNum()), ts, int(msg.getWidth()), int(msg.getHeight()))
        if want_image:
            self.image = msg.getCvFrame()
        return True


class PngStream(Stream):
    """Host-side replay of 05_record.py's ``depth/depth_NNNNNN.png`` sequence (uint16 mm)."""

    def __init__(self, name: str, seq: DepthSequence, args: argparse.Namespace) -> None:
        self.rows: List[Tuple[int, Optional[float], Path]] = load_depth_index(seq)
        timestamps = [ts for _, ts, _ in self.rows]
        has_ts = bool(self.rows) and all(ts is not None for ts in timestamps) and (
            len(self.rows) < 2 or timestamps[-1] > timestamps[0])
        self.speed: float = float(args.speed)
        self.loop: bool = bool(args.loop)
        self.size: Optional[Tuple[int, int]] = args.size
        self.read_errors: int = 0
        self.pos: int = 0
        self.start_wall: Optional[float] = None
        # Like ReplayVideo, keep sequence numbers and timestamps monotonic across --loop passes.
        self.seq_shift: int = self.rows[0][0] if self.rows else 0
        self.ts_shift: float = 0.0
        if args.fps:
            base = float(args.fps)
            origin = "--fps"
        elif has_ts and len(self.rows) >= 2:
            base = (len(self.rows) - 1) / (timestamps[-1] - timestamps[0])
            origin = "from depth_index.csv"
        else:
            base = FALLBACK_SOURCE_FPS
            origin = "assumed"
        # Pace by the recorded timestamps (scaled by --speed) unless --fps overrides the rate.
        self.by_timestamps: bool = has_ts and not args.fps
        self.interval: float = 1.0 / max(0.2, base * self.speed)
        if self.by_timestamps:
            pacing = "recorded timestamps x {:g} (~{:.1f} fps)".format(self.speed, base * self.speed)
        else:
            pacing = "{:.1f} fps ({} {:.1f} x {:g})".format(base * self.speed, origin, base, self.speed)
        description = "{} [uint16 mm PNGs] {} frames | index: {} | pacing: {}".format(
            seq.folder / DEPTH_DIR_NAME, len(self.rows),
            seq.index.name if seq.index is not None else "none (file names)", pacing)
        super().__init__(name, description, expected=len(self.rows), has_timestamps=has_ts,
                         target_fps=None if self.by_timestamps else base * self.speed)
        self.count_source = "the index lists"
        self.nominal_interval: float = 1.0 / base       # recorded seconds between frames

    def _due(self, index: int) -> float:
        assert self.start_wall is not None
        if self.by_timestamps:
            first_ts = self.rows[0][1] or 0.0
            ts = self.rows[index][1] or first_ts
            return self.start_wall + (ts - first_ts) / self.speed
        return self.start_wall + index * self.interval

    def poll(self, want_image: bool) -> bool:
        if self.finished or not self.rows:
            self.finished = True
            return False
        now = time.monotonic()
        if self.start_wall is None:
            self.start_wall = now
        if self.pos >= len(self.rows):
            if not self.loop:
                self.finished = True
                return False
            self.pos = 0
            self.start_wall = now
            self.seq_shift = self.last_seq + 1
            if self.last_ts is not None:
                self.ts_shift = self.last_ts.total_seconds() + self.nominal_interval - (self.rows[0][1] or 0.0)
        if now < self._due(self.pos):
            return False
        seq, ts, path = self.rows[self.pos]
        self.pos += 1
        seq = seq - self.rows[0][0] + self.seq_shift
        if ts is not None:
            ts = ts + self.ts_shift
        depth = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
        if depth is None:
            self.read_errors += 1
            if self.read_errors == 1:
                print("warning: cannot read {} (further unreadable PNGs are only counted)".format(path),
                      file=sys.stderr, flush=True)
            return False
        if depth.ndim == 3:
            depth = depth[:, :, 0]
        if self.size is not None:
            depth = cv2.resize(depth, self.size, interpolation=cv2.INTER_NEAREST)
        stamp = dt.timedelta(seconds=ts) if ts is not None else dt.timedelta(seconds=now - self.start_wall)
        self._note(int(seq), stamp, int(depth.shape[1]), int(depth.shape[0]))
        centre = int(depth[depth.shape[0] // 2, depth.shape[1] // 2])
        self.extra = "centre {} mm".format(centre) if centre > 0 else "centre: no depth"
        if want_image:
            self.image = colorize_depth(depth, DEPTH_COLOR_MAX_MM)
        return True


def load_depth_index(seq: DepthSequence) -> List[Tuple[int, Optional[float], Path]]:
    """``[(sequence number, timestamp seconds or None, png path)]`` in replay order.

    Prefers ``depth_index.csv`` (columns index,sequence,timestamp_s,device_timestamp_s,width,
    height,file as written by 05_record.py); falls back to the sorted file names.
    """
    rows: List[Tuple[int, Optional[float], Path]] = []
    if seq.index is not None:
        try:
            with open(seq.index, newline="", encoding="utf-8") as fh:
                for row in csv.DictReader(fh):
                    path = seq.folder / str(row.get("file", "")).strip()
                    if not path.is_file():
                        continue
                    try:
                        sequence = int(row["sequence"])
                    except (KeyError, TypeError, ValueError):
                        sequence = len(rows)
                    try:
                        stamp: Optional[float] = float(row["timestamp_s"])
                    except (KeyError, TypeError, ValueError):
                        stamp = None
                    rows.append((sequence, stamp, path))
        except (OSError, csv.Error, UnicodeDecodeError):
            rows = []
    if not rows:
        for i, path in enumerate(seq.pngs):
            digits = path.stem.rsplit("_", 1)[-1]
            rows.append((int(digits) if digits.isdigit() else i, None, path))
    return rows


class MetaStream:
    """Output queue of a ``ReplayMetadataOnly`` node (IMU or other metadata-only recording)."""

    def __init__(self, name: str, queue: dai.MessageQueue) -> None:
        self.name: str = name
        self.queue: dai.MessageQueue = queue
        self.messages: int = 0
        self.packets: int = 0
        self.kind: str = "?"
        self.closed: bool = False
        self.rate: FpsCounter = FpsCounter(window_s=2.0)

    def take_all(self) -> None:
        if self.closed:
            return
        try:
            msgs = self.queue.tryGetAll()
        except dai.MessageQueue.QueueException:      # its pipeline stopped (the node failed)
            self.closed = True
            return
        for msg in msgs:
            self.messages += 1
            self.kind = type(msg).__name__
            if isinstance(msg, dai.IMUData):
                self.packets += len(msg.packets)
            self.rate.tick()

    def status(self) -> str:
        return "{}: {} msgs={} packets={} {:.1f} msg/s".format(
            self.name, self.kind, self.messages, self.packets, self.rate.fps())


def save_snapshots(streams: Sequence[Stream], out_dir: Path) -> None:
    try:
        ensure_dir(out_dir)
    except OSError as exc:
        print("Cannot create snapshot folder {}: {}".format(out_dir, exc), file=sys.stderr, flush=True)
        return
    for stream in streams:
        if stream.last_img is None:
            continue
        target = out_dir / timestamp_name("replay_{}".format(stream.name), "png")
        if cv2.imwrite(str(target), stream.last_img):
            print("Saved {}".format(target.resolve()), flush=True)
        else:
            print("Could not write {}".format(target.resolve()), file=sys.stderr, flush=True)


def end_reason(streams: Sequence[Stream], mode: str) -> str:
    """Label for "every stream finished": honest about failures instead of always saying EOF."""
    if mode == "holistic":
        return ("pipeline stopped unexpectedly (the built-in holistic replay loops forever - "
                "see the depthai log above)")
    empty = [s.name for s in streams if s.frames == 0]
    if empty:
        return ("replay stopped without a frame from {} (the replay node could not open/decode "
                "the file - see the depthai log above)".format(", ".join(empty)))
    return "end of file on every stream"


def run_loop(streams: Sequence[Stream], metas: Sequence[MetaStream], args: argparse.Namespace,
             looping: bool, mode: str) -> Tuple[str, int]:
    """Pump frames until every stream ends, ``--duration`` expires, ``q`` is pressed or Ctrl-C.

    Returns ``(reason, total_frames)``.  Frames are polled with ``tryGet`` so several streams
    of different rates never block each other; the queues are blocking on the producer side,
    so nothing is dropped and a fast ``--speed`` simply back-pressures the replay node.
    """
    clock = RunClock(args.duration)
    stats = Periodic(1.0)
    headless = bool(args.headless)
    speed_tag = "{:.2f}x".format(args.speed) if args.speed != 1.0 else "1x"
    reason = ""
    try:
        while True:
            if clock.expired():
                reason = "duration of {:.1f} s reached".format(args.duration)
                break
            got_any = False
            for stream in streams:
                if stream.finished or not stream.poll(want_image=not headless):
                    continue
                got_any = True
                if headless or stream.image is None:
                    continue
                img = stream.image
                if img.ndim == 2:
                    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
                draw_label(img, "frame {}  seq {}\nts {}  t+{:.2f}s\n{}x{}  {:.1f} fps  {}{}{}".format(
                    stream.frames, stream.last_seq, fmt_td(stream.last_ts), stream.rel_seconds(),
                    stream.width, stream.height, stream.fps.fps(), speed_tag,
                    "  loop" if looping else "",
                    "\n{}".format(stream.extra) if stream.extra else ""))
                stream.last_img = img
                cv2.imshow(stream.window, img)
            for meta in metas:
                meta.take_all()
            if streams and all(s.finished for s in streams):
                reason = end_reason(streams, mode)
                break
            if headless:
                if stats.ready():
                    lines = [s.status() for s in streams] + [m.status() for m in metas]
                    print("[{:6.1f}s] {}".format(clock.elapsed(), " | ".join(lines)), flush=True)
                if not got_any:
                    time.sleep(0.002)
            else:
                key = wait_key_quit(1)
                if key == "q":
                    reason = "quit by user"
                    break
                if key == "s":
                    save_snapshots(streams, args.snapshot_dir)
    except KeyboardInterrupt:
        reason = "interrupted (Ctrl-C)"
    total = sum(s.frames for s in streams)
    return reason, total


def print_summary(reason: str, started: float, streams: Sequence[Stream], metas: Sequence[MetaStream],
                  args: argparse.Namespace) -> None:
    elapsed = max(1e-6, time.monotonic() - started)
    total = sum(s.frames for s in streams)
    print("Replay finished: {} frames in {:.1f} s ({:.1f} fps overall) - {}".format(
        total, elapsed, total / elapsed, reason), flush=True)
    for stream in streams:
        wall = stream.wall_seconds()
        delivered = (stream.frames - 1) / wall if stream.frames >= 2 and wall > 0 else 0.0
        line = "  {}: {}, seq {}..{}, {}x{}, recorded span {:.2f} s, delivered {:.1f} fps".format(
            stream.name, stream.count_text(bool(args.loop)), stream.first_seq, stream.last_seq,
            stream.width, stream.height, stream.rel_seconds(), delivered)
        speed = stream.effective_speed()
        if speed is not None:
            line += ", effective speed {:.2f}x (requested {:g}x)".format(speed, args.speed)
        elif stream.target_fps:
            line += " (target {:.1f} fps; no recorded timestamps)".format(stream.target_fps)
        if isinstance(stream, PngStream) and stream.read_errors:
            line += ", {} unreadable PNG(s)".format(stream.read_errors)
        print(line, flush=True)
    for meta in metas:
        print("  {}: {} {} messages, {} IMU packets".format(meta.name, meta.messages, meta.kind, meta.packets),
              flush=True)


# --------------------------------------------------------------------------------------
# --mode video: host-only ReplayVideo pipelines (no camera)
# --------------------------------------------------------------------------------------

def pacing_fps(source: VideoSource, args: argparse.Namespace, probed_fps: float) -> Tuple[Optional[float], str]:
    """Decide what to pass to ``ReplayVideo.setFps`` (None = leave the recorded-timestamp pacing).

    The base rate is ``--fps`` or the container fps probed by cv2 (30 when unknown); see the
    module docstring for why the delivered rate can be lower than the target.
    """
    base = args.fps if args.fps else (probed_fps if probed_fps > 0 else FALLBACK_SOURCE_FPS)
    if source.metadata is not None and args.speed == 1.0 and not args.fps:
        return None, "recorded timestamps (real time)"
    target = max(0.2, base * args.speed)   # the node ignores fps <= 0.1
    origin = "--fps" if args.fps else ("probed" if probed_fps > 0 else "assumed")
    return target, "{:.1f} fps ({} {:.1f} x {:g})".format(target, origin, base, args.speed)


def select_streams(videos: List[VideoSource], filters: Optional[List[str]]) -> List[VideoSource]:
    return [v for v in videos if name_matches(v.name, filters)]


def run_video_mode(rec: Recording, args: argparse.Namespace) -> int:
    videos = select_streams(rec.videos, args.stream)
    depth = rec.depth if rec.depth is not None and name_matches(DEPTH_STREAM_NAME, args.stream) else None
    if not videos and depth is None:
        available = [v.name for v in rec.videos] + ([DEPTH_STREAM_NAME] if rec.depth is not None else [])
        return fail("No stream matches --stream {}; available: {}".format(
            args.stream, ", ".join(available) or "none"))
    print("Mode: video (host-only ReplayVideo pipelines - no camera needed)", flush=True)
    if rec.origin:
        print("Source: {}".format(rec.origin), flush=True)

    pipelines: List[dai.Pipeline] = []      # one per file: a ReplayVideo node stops ITS pipeline at EOF
    streams: List[Stream] = []
    for source in videos:
        width, height, probed_fps, count = probe_video(source.video)
        pipeline = dai.Pipeline(createImplicitDevice=False)     # never touches the OAK
        node = pipeline.create(dai.node.ReplayVideo)
        node.setReplayVideoFile(source.video)
        if source.metadata is not None:
            node.setReplayMetadataFile(source.metadata)
        node.setOutFrameType(dai.ImgFrame.Type.NV12)         # getCvFrame() -> BGR, as in replay_video.py
        node.setLoop(bool(args.loop))                        # the node defaults to loop=True!
        if args.size is not None:
            node.setSize(args.size[0], args.size[1])
        target_fps, pacing_text = pacing_fps(source, args, probed_fps)
        if target_fps is not None:
            node.setFps(float(target_fps))
        queue = node.out.createOutputQueue(maxSize=8, blocking=True)
        probe_text = ("{}x{} @ {:.1f} fps, {} frames".format(width, height, probed_fps, count)
                      if width > 0 else "cv2 could not probe it")
        description = "{} [{}] {} | metadata: {} | pacing: {}".format(
            source.video, source.kind, probe_text,
            source.metadata.name if source.metadata else "none", pacing_text)
        print("  stream {}: {}".format(source.name, description), flush=True)
        streams.append(QueueStream(source.name, queue, pipeline, description, expected=count,
                                   has_timestamps=source.metadata is not None, target_fps=target_fps))
        pipelines.append(pipeline)

    if depth is not None:
        taken = {v.name for v in videos}
        name = DEPTH_STREAM_NAME if DEPTH_STREAM_NAME not in taken else DEPTH_STREAM_NAME + "_png"
        stream = PngStream(name, depth, args)
        print("  stream {}: {}".format(name, stream.description), flush=True)
        streams.append(stream)
    elif rec.depth is not None:
        print("  (depth/ PNG sequence present: {} frames; not selected by --stream)".format(len(rec.depth)),
              flush=True)

    metas: List[MetaStream] = []
    if args.imu:
        for meta_file in rec.meta_only:
            pipeline = dai.Pipeline(createImplicitDevice=False)
            node = pipeline.create(dai.node.ReplayMetadataOnly)
            node.setReplayFile(meta_file)
            node.setLoop(True)   # always loop: a short IMU file must not stop before the videos
            metas.append(MetaStream(meta_file.stem, node.out.createOutputQueue(maxSize=64, blocking=False)))
            pipelines.append(pipeline)
            print("  metadata {}: {} (looping, real-time pacing)".format(meta_file.stem, meta_file), flush=True)
        if not metas:
            print("  --imu: no metadata-only .mcap file found next to the videos", flush=True)
    elif rec.meta_only:
        print("  ({} metadata-only .mcap file(s) present; add --imu to replay them)".format(len(rec.meta_only)),
              flush=True)

    if not args.headless:
        print("Keys: q/Esc quit, s save snapshot(s) to {}".format(args.snapshot_dir), flush=True)
    started = time.monotonic()
    try:
        for pipeline in pipelines:
            try:
                pipeline.start()
            except RuntimeError as exc:
                return fail("Could not start the replay pipeline for {}: {}".format(rec.path, exc))
        reason, total = run_loop(streams, metas, args, bool(args.loop), "video")
    finally:
        for pipeline in pipelines:
            try:
                pipeline.stop()
            except Exception:  # noqa: BLE001 - already stopped by the replay node
                pass
        cv2.destroyAllWindows()
    print_summary(reason, started, streams, metas, args)
    if total == 0:
        return fail("No frames could be replayed from {} - unsupported/corrupt video file? "
                    "(see the depthai log lines above)".format(rec.path))
    return 0


# --------------------------------------------------------------------------------------
# --mode holistic: enableHolisticReplay into Camera / IMU nodes (needs the OAK connected)
# --------------------------------------------------------------------------------------

def holistic_contents(source: Path) -> Tuple[List[str], Dict[str, str], bool, List[str]]:
    """``(camera socket names, {socket: video file}, has IMU, problems)`` of a recording tar or holistic folder."""
    entries: List[Tuple[str, int]]
    if source.is_dir():
        entries = [(p.name, p.stat().st_size) for p in source.iterdir() if p.is_file()]
    else:
        entries = list_tar_files(source)
    names = {name for name, _ in entries}
    sockets: List[str] = []
    videos: Dict[str, str] = {}
    problems: List[str] = []
    for name in sorted(names):
        stem, ext = Path(name).stem, Path(name).suffix.lower()
        if not stem.startswith("Camera"):
            continue
        socket_name = stem[len("Camera"):]
        if ext == META_EXT:
            if socket_name in dai.CameraBoardSocket.__members__:
                sockets.append(socket_name)
            else:
                problems.append("unknown camera socket in {}".format(name))
        elif ext in VIDEO_EXTS:
            videos[socket_name] = name
    for name, size in entries:
        if name.lower().endswith(".mp4") and size <= HEADER_ONLY_MP4_BYTES:
            problems.append("{} is a header-only mp4 ({} bytes): the recording failed, re-record".format(name, size))
    has_imu = "IMU.mcap" in names
    if "camera_info.json" not in names:
        problems.append("camera_info.json missing (not a holistic recording)")
    return sockets, videos, has_imu, problems


def run_holistic_mode(rec: Recording, args: argparse.Namespace) -> int:
    source = rec.holistic
    if source is None:
        return fail("No holistic recording found at {}".format(rec.path))
    try:
        sockets, videos, has_imu, problems = holistic_contents(source)
    except (tarfile.TarError, OSError) as exc:
        return fail("Cannot read holistic recording {}: {}".format(source, exc))
    if problems:
        return fail("Holistic recording {} is not usable: {}".format(source, "; ".join(problems)))
    if not sockets:
        return fail("No Camera*.mcap streams inside {}".format(source))
    if args.speed != 1.0 or args.loop:
        print("Note: --speed/--loop are ignored in holistic mode (the built-in replay runs at "
              "recorded pace and loops forever)", flush=True)

    print("Mode: holistic (enableHolisticReplay - needs the OAK connected in depthai 3.10.0)", flush=True)
    print("Recording: {} | cameras: {} | IMU: {}".format(
        source, ", ".join(sockets), "yes" if has_imu else "no"), flush=True)
    size = args.size or HOLISTIC_DEFAULT_SIZE
    fps = args.fps or HOLISTIC_DEFAULT_FPS

    # An explicit --device-id never retries: one stderr line even when nothing enumerates.
    # Without an id the open is retried while a camera reboots after another script.
    device = open_device(args.device_id, retries=1 if args.device_id else 3)  # exits 2 on failure
    print_device_summary(device)
    pipeline = dai.Pipeline(device)
    streams: List[Stream] = []
    metas: List[MetaStream] = []
    started = time.monotonic()
    try:
        try:
            pipeline.enableHolisticReplay(str(source))   # must come BEFORE creating nodes
            for socket_name in sockets:
                socket = dai.CameraBoardSocket.__members__[socket_name]
                cam = pipeline.create(dai.node.Camera).build(socket)
                output = cam.requestOutput((int(size[0]), int(size[1])), fps=float(fps))
                queue = output.createOutputQueue(maxSize=8, blocking=False)
                description = "Camera {} <- {} requested {}x{} @ {:g} fps".format(
                    socket_name, videos.get(socket_name, "Camera{}.mcap".format(socket_name)),
                    size[0], size[1], fps)
                print("  stream {}: {}".format(socket_name, description), flush=True)
                streams.append(QueueStream(socket_name, queue, pipeline, description,
                                           has_timestamps=True, target_fps=float(fps)))
            if args.imu:
                if has_imu:
                    imu = pipeline.create(dai.node.IMU)
                    imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 200)
                    imu.setBatchReportThreshold(10)
                    imu.setMaxBatchReports(10)
                    metas.append(MetaStream("IMU", imu.out.createOutputQueue(maxSize=64, blocking=False)))
                    print("  metadata IMU: IMU.mcap -> IMU node (accel+gyro RAW, batches of 10)", flush=True)
                else:
                    print("  --imu: recording has no IMU.mcap", flush=True)
            if not args.headless:
                print("Keys: q/Esc quit, s save snapshot(s) to {}".format(args.snapshot_dir), flush=True)
            pipeline.start()
        except RuntimeError as exc:
            return fail("Holistic replay of {} could not be set up: {}: {}".format(
                source, type(exc).__name__, exc))
        reason, total = run_loop(streams, metas, args, True, "holistic")
    finally:
        try:
            pipeline.stop()          # also closes the device
        except Exception:  # noqa: BLE001
            pass
        try:
            device.close()
        except Exception:  # noqa: BLE001 - already closed by pipeline.stop()
            pass
        cv2.destroyAllWindows()
    print_summary(reason, started, streams, metas, args)
    if total == 0:
        return fail("No frames were replayed from {} (see the depthai log lines above; a header-only "
                    "mp4 or a socket mismatch makes the Camera node stop the pipeline)".format(source))
    return 0


# --------------------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Replay a recording made by 05_record.py: host-only video / depth-PNG replay "
                    "(no camera) or holistic replay into Camera/IMU nodes (camera required).",
        epilog="Examples:\n"
               "  python camera/06_replay.py recordings/<folder>\n"
               "  python camera/06_replay.py recordings/<folder>/rgb.mp4 --loop --speed 2\n"
               "  python camera/06_replay.py recordings/<folder> --headless --duration 10 --imu\n"
               "  python camera/06_replay.py recordings/<folder> --stream depth\n"
               "  python camera/06_replay.py recordings/recording_*.tar --mode holistic --duration 15\n",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("recording",
                        help="recording folder from 05_record.py, a single .mp4/.mkv/.avi (or its .mcap), "
                             "or a holistic recording_*.tar")
    parser.add_argument("--mode", choices=("video", "holistic"), default="video",
                        help="video = host-only ReplayVideo decode (+ depth PNG sequence), no camera needed "
                             "(default); holistic = pipeline.enableHolisticReplay into Camera/IMU nodes, "
                             "needs the OAK connected in depthai 3.10.0")
    parser.add_argument("--loop", action="store_true",
                        help="restart from the beginning at end of file instead of stopping (video mode)")
    parser.add_argument("--speed", type=float, default=1.0, metavar="FACTOR",
                        help="playback speed factor, e.g. 0.5 or 2 (video mode; default 1 = real time). "
                             "The base rate is the container fps probed by cv2 (or --fps), and the delivered "
                             "rate is capped by the host timer (~16 ms steps on Windows) - the summary prints "
                             "the effective speed actually achieved")
    parser.add_argument("--fps", type=float, default=None, metavar="FPS",
                        help="source frame rate to assume for --speed pacing (default: probe the file with "
                             "cv2, else 30; the depth PNGs use their index timestamps); in holistic mode the "
                             "fps requested from the Camera node (30)")
    parser.add_argument("--size", type=parse_size, default=None, metavar="WxH",
                        help="resize replayed frames (ReplayVideo.setSize / cv2.resize for depth PNGs) / "
                             "holistic output size (default: native / 640x400)")
    parser.add_argument("--stream", action="append", default=None, metavar="SUBSTR",
                        help="only replay streams whose name contains SUBSTR (repeatable; video mode; "
                             "the depth PNG sequence is named 'depth')")
    parser.add_argument("--imu", action="store_true",
                        help="also replay metadata-only .mcap files (IMU) and count their messages")
    parser.add_argument("--snapshot-dir", type=Path, default=DEFAULT_SNAPSHOT_DIR, metavar="DIR",
                        help="folder for 's' snapshots (default: <repo>/snapshots)")
    add_device_arg(parser)      # --device-id, only used by --mode holistic
    add_run_args(parser)        # --duration, --headless
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    if not np.isfinite(args.speed) or args.speed <= 0.0:
        return fail("--speed must be a positive number, got {}".format(args.speed))
    if args.fps is not None and (not np.isfinite(args.fps) or args.fps <= 0.0):
        return fail("--fps must be a positive number, got {}".format(args.fps))

    path = Path(args.recording).expanduser()
    rec: Optional[Recording] = None
    try:
        rec, problem = discover_recording(path, args.mode)   # may extract a tar: Ctrl-C-safe
        if rec is None:
            return fail(problem)
        if args.mode == "holistic":
            return run_holistic_mode(rec, args)
        return run_video_mode(rec, args)
    except KeyboardInterrupt:
        print("Interrupted (Ctrl-C)", flush=True)
        return 0
    finally:
        cv2.destroyAllWindows()
        if rec is not None:
            rec.cleanup()


if __name__ == "__main__":
    sys.exit(main())
