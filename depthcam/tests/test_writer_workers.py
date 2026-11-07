"""Tests for ``EurocWriter``'s threaded PNG pipeline (encoder pool + ordered committer).

No camera, no calibration file: a synthetic pinhole rig and noise images.  Every test checks the
invariants the format relies on regardless of the thread count: ``cam0/data.csv`` and
``cam1/data.csv`` hold the same strictly increasing timestamps, every row's PNG exists on both
sides and decodes to the recorded shape, ``dataset.json`` counts agree with the files, and
``EurocReader.validate()`` reports no error.

Runs on Windows with ``python -m pytest tests/test_writer_workers.py -q`` in well under 30 s.
"""

from __future__ import annotations

import json
import os
import time
from pathlib import Path
from typing import Callable, List, Tuple

import cv2
import numpy as np
import pytest

from cuvslam_oak import dataset

T0 = 1_758_000_000_000_000_000
PERIOD_NS = 33_333_333
SMALL = (320, 200)                # (width, height): fast encodes for the functional tests
LARGE = (640, 400)                # the load tests: ~10 ms per noisy image at compression 1


def make_rig(size: Tuple[int, int]) -> dataset.RigInfo:
    width, height = size
    f = 0.5 * width
    T1 = np.eye(4)
    T1[0, 3] = 0.075
    cam = dict(width=width, height=height, fu=f, fv=f, cu=width / 2.0, cv=height / 2.0)
    return dataset.RigInfo(cam0=dataset.CameraInfo(socket="CAM_B", role="left", **cam),
                           cam1=dataset.CameraInfo(T_BS=T1, socket="CAM_C", role="right", **cam),
                           imu=dataset.ImuInfo(), device_id="TEST")


def noise_pairs(count: int, size: Tuple[int, int], seed: int = 7) -> List[Tuple[int, np.ndarray, np.ndarray]]:
    rng = np.random.default_rng(seed)
    shape = (size[1], size[0])
    return [(T0 + i * PERIOD_NS,
             rng.integers(0, 256, size=shape, dtype=np.uint8),
             rng.integers(0, 256, size=shape, dtype=np.uint8)) for i in range(count)]


def read_rows(csv_path: Path) -> List[Tuple[int, str]]:
    lines = csv_path.read_text(encoding="ascii").splitlines()
    assert lines[0] == dataset.CAM_CSV_HEADER
    rows = []
    for line in lines[1:]:
        if line.strip():
            ts, name = line.split(",")
            rows.append((int(ts), name))
    return rows


def wait_until(pred: Callable[[], bool], timeout_s: float = 10.0) -> None:
    deadline = time.monotonic() + timeout_s
    while not pred():
        assert time.monotonic() < deadline, "timed out waiting for the writer"
        time.sleep(0.01)


def check_dataset(root: Path, size: Tuple[int, int], *, decode_every: int = 1) -> List[Tuple[int, str]]:
    """The invariants: equal csv files, strictly increasing, every PNG present and decodable, no dangling PNG."""
    mav0 = root / "mav0"
    rows0 = read_rows(mav0 / "cam0" / "data.csv")
    rows1 = read_rows(mav0 / "cam1" / "data.csv")
    assert rows0 == rows1
    ts = [r[0] for r in rows0]
    assert all(a < b for a, b in zip(ts, ts[1:])), "csv order is not strictly increasing"
    shape = (size[1], size[0])
    for i, (stamp, name) in enumerate(rows0):
        assert name == "{}.png".format(stamp)
        for cam in ("cam0", "cam1"):
            path = mav0 / cam / "data" / name
            assert path.is_file(), path
            if i % decode_every == 0:
                img = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
                assert img is not None and img.dtype == np.uint8 and img.shape == shape, path
    for cam in ("cam0", "cam1"):
        assert len(list((mav0 / cam / "data").glob("*.png"))) == len(rows0)
    doc = json.loads((root / "dataset.json").read_text(encoding="utf-8"))
    assert doc["counts"]["stereo_pairs"] == doc["counts"]["frames_written"] == len(rows0)
    report = dataset.EurocReader(root).validate(decode_images=2)
    assert report.errors == [], report.errors
    return rows0


# --------------------------------------------------------------------------------------

def test_resolve_png_workers(monkeypatch: pytest.MonkeyPatch) -> None:
    assert dataset.resolve_png_workers(3) == 3
    assert dataset.resolve_png_workers(1) == 1
    cpus = os.cpu_count() or 1
    assert dataset.resolve_png_workers(0) == min(4, max(1, cpus - 2))
    for fake, expected in ((22, 4), (6, 4), (5, 3), (3, 1), (2, 1), (1, 1), (None, 1)):
        monkeypatch.setattr(dataset.os, "cpu_count", lambda fake=fake: fake)
        assert dataset.resolve_png_workers(0) == expected, fake
        assert dataset.resolve_png_workers(-2) == expected, fake


def test_slow_pace_three_workers(tmp_path: Path) -> None:
    """60 pairs at a deliberately slow pace: everything written, in order, fully described in dataset.json."""
    root = tmp_path / "slow"
    pairs = noise_pairs(60, SMALL)
    with dataset.EurocWriter(root, make_rig(SMALL), png_workers=3, queue_size=64) as writer:
        assert writer.png_workers == 3
        for ts, left, right in pairs:
            assert writer.add_frame(ts, left, right)
            time.sleep(0.005)
        live = writer.stats()
        for key in ("png_workers", "png_write_ms_mean", "png_encode_ms_mean", "png_throughput_pairs_per_s",
                    "queue_depth", "queue_size"):
            assert key in live, key
        assert live["png_workers"] == 3 and live["queue_size"] == 64
    summary = writer.summary
    assert summary is not None
    rows = check_dataset(root, SMALL)
    assert [r[0] for r in rows] == [p[0] for p in pairs]
    doc = json.loads((root / "dataset.json").read_text(encoding="utf-8"))
    assert doc == json.loads(json.dumps(summary, default=str))
    counts, measured, drops = doc["counts"], doc["measured"], doc["drops"]
    assert counts["frames_accepted"] == counts["frames_written"] == counts["stereo_pairs"] == 60
    assert counts["frames_failed"] == 0
    assert all(v == 0 for v in drops.values()), drops
    assert doc["record"]["png_workers"] == 3
    assert measured["png_write_ms_mean"] > 0.0
    assert measured["png_encode_ms_mean"] > 0.0
    assert measured["png_throughput_pairs_per_s"] > 0.0
    assert measured["png_bytes_total"] == sum(
        (root / "mav0" / cam / "data" / name).stat().st_size for _, name in rows for cam in ("cam0", "cam1"))
    assert doc["warnings"] == []


def test_ordering_under_load(tmp_path: Path) -> None:
    """200 pairs pushed as fast as possible through a tiny queue: drops allowed and counted, order never broken."""
    root = tmp_path / "load"
    pairs = noise_pairs(200, LARGE)
    accepted = 0
    dropped = 0
    with dataset.EurocWriter(root, make_rig(LARGE), png_workers=4, queue_size=8) as writer:
        for ts, left, right in pairs:
            if writer.add_frame(ts, left, right):
                accepted += 1
            else:
                dropped += 1
        assert writer.frames_accepted == accepted
        assert writer.frames_dropped == dropped
    assert accepted + dropped == 200 and accepted >= 8
    rows = check_dataset(root, LARGE, decode_every=10)
    assert len(rows) == accepted, "every accepted pair is written, none twice"
    wanted = {p[0] for p in pairs}
    assert all(r[0] in wanted for r in rows)
    doc = json.loads((root / "dataset.json").read_text(encoding="utf-8"))
    assert doc["drops"]["stereo_write_queue_dropped"] == dropped
    assert doc["counts"]["frames_failed"] == 0
    assert doc["counts"]["frames_accepted"] == accepted


def test_encode_failure_is_counted_and_recording_continues(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """One pair whose encode raises: counted (not fatal), later pairs still written, csv rows consistent."""
    root = tmp_path / "fail"
    pairs = noise_pairs(20, SMALL)
    poison_index = 7
    for i, (_, left, right) in enumerate(pairs):
        left[0, 0] = 255 if i == poison_index else 0     # the marker the patched encoder looks for
        right[0, 0] = 0
    poison_ts = pairs[poison_index][0]
    real_encode = dataset._encode_png

    def flaky_encode(img: np.ndarray, compression: int) -> bytes:
        if int(img[0, 0]) == 255:
            raise RuntimeError("simulated encoder failure")
        return real_encode(img, compression)

    monkeypatch.setattr(dataset, "_encode_png", flaky_encode)
    with dataset.EurocWriter(root, make_rig(SMALL), png_workers=3) as writer:
        for ts, left, right in pairs:
            assert writer.add_frame(ts, left, right)
            time.sleep(0.002)
        wait_until(lambda: writer.frames_written + writer.stats()["frames_failed"] == 20)
        assert writer.writer_error is None            # an encode failure is not fatal
        assert writer.add_frame(pairs[-1][0] + PERIOD_NS, pairs[0][1], pairs[0][2])   # still accepting
    rows = check_dataset(root, SMALL)
    assert len(rows) == 20                            # 19 of the first batch + the extra pair
    assert poison_ts not in [r[0] for r in rows]
    assert [r[0] for r in rows] == [p[0] for p in pairs if p[0] != poison_ts] + [pairs[-1][0] + PERIOD_NS]
    doc = json.loads((root / "dataset.json").read_text(encoding="utf-8"))
    assert doc["counts"]["frames_failed"] == 1
    assert doc["counts"]["frames_accepted"] == 21 and doc["counts"]["frames_written"] == 20
    assert doc["drops"]["stereo_writer_failed"] == 0
    assert any(w.startswith("png write failed: frame {}: RuntimeError".format(poison_ts)) for w in doc["warnings"])
    assert "1 stereo pairs failed to encode/write" in doc["warnings"]
    assert not (root / "mav0" / "cam0" / "data" / "{}.png".format(poison_ts)).exists()


def test_close_drains_everything(tmp_path: Path) -> None:
    """30 pairs queued and close() called at once: all 30 committed; a second close() is a no-op."""
    root = tmp_path / "drain"
    pairs = noise_pairs(30, LARGE)
    writer = dataset.EurocWriter(root, make_rig(LARGE), png_workers=4, queue_size=64)
    for ts, left, right in pairs:
        assert writer.add_frame(ts, left, right)
    assert writer.queue_depth > 0 or writer.frames_written < 30    # close() has real work to do
    summary = writer.close(stopped_by="test")
    assert writer.closed and writer.frames_written == 30
    assert summary["counts"]["frames_written"] == summary["counts"]["stereo_pairs"] == 30
    assert summary["record"]["stopped_by"] == "test"
    again = writer.close()
    assert again is summary
    with pytest.raises(RuntimeError):
        writer.add_frame(pairs[-1][0] + PERIOD_NS, pairs[0][1], pairs[0][2])
    rows = check_dataset(root, LARGE, decode_every=5)
    assert [r[0] for r in rows] == [p[0] for p in pairs]


def test_single_worker_regression(tmp_path: Path) -> None:
    """png_workers=1 is the historical single-encoder behaviour, with the same files and counters."""
    root = tmp_path / "one"
    pairs = noise_pairs(40, SMALL)
    with dataset.EurocWriter(root, make_rig(SMALL), png_workers=1, queue_size=64) as writer:
        assert writer.png_workers == 1
        for ts, left, right in pairs:
            assert writer.add_frame(ts, left, right, right_ts_ns=ts + 12_000)
            time.sleep(0.004)                         # slower than one encoder needs for a 320x200 pair
        # the historical rejections still happen in the caller thread, before the queue
        assert not writer.add_frame(pairs[-1][0], pairs[0][1], pairs[0][2])              # not increasing
        assert not writer.add_frame(pairs[-1][0] + PERIOD_NS, pairs[0][1], pairs[0][2],
                                    right_ts_ns=pairs[-1][0] + PERIOD_NS + 5_000_000)   # 5 ms skew
        with pytest.raises(dataset.DatasetError):
            writer.add_frame(pairs[-1][0] + 2 * PERIOD_NS, pairs[0][1][:10], pairs[0][2])
    rows = check_dataset(root, SMALL)
    assert [r[0] for r in rows] == [p[0] for p in pairs]
    doc = json.loads((root / "dataset.json").read_text(encoding="utf-8"))
    assert doc["record"]["png_workers"] == 1
    assert doc["counts"]["frames_written"] == doc["counts"]["frames_accepted"] == 40
    assert doc["drops"]["frames_out_of_order"] == 1 and doc["drops"]["stereo_sync_violations"] == 1
    assert doc["drops"]["stereo_write_queue_dropped"] == 0
    assert doc["measured"]["png_write_ms_mean"] > 0.0 and doc["measured"]["png_encode_ms_mean"] > 0.0
    assert doc["measured"]["max_left_right_skew_us"] == pytest.approx(5000.0)


def test_csv_failure_is_fatal_with_workers(tmp_path: Path) -> None:
    """A data.csv write error stops the stereo stream (as before) while close() still finishes the dataset."""
    root = tmp_path / "csv"
    pairs = noise_pairs(12, SMALL)
    writer = dataset.EurocWriter(root, make_rig(SMALL), png_workers=3)
    for ts, left, right in pairs[:4]:
        assert writer.add_frame(ts, left, right)
    wait_until(lambda: writer.frames_written == 4)
    writer._cam1_csv.close()                          # simulate a lost/closed csv handle
    for ts, left, right in pairs[4:8]:
        writer.add_frame(ts, left, right)
    wait_until(lambda: writer.writer_error is not None)
    assert "ValueError" in (writer.writer_error or "")
    rejected = [writer.add_frame(ts, left, right) for ts, left, right in pairs[8:]]
    assert rejected == [False] * 4
    assert writer.drops["stereo_writer_failed"] == 4
    summary = writer.close()
    assert writer.closed
    assert (root / "dataset.json").is_file()
    assert summary["counts"]["frames_written"] == 4 and summary["counts"]["stereo_pairs"] == 4
    assert summary["counts"]["frames_failed"] == 4
    assert summary["drops"]["stereo_writer_failed"] == 4
    assert any("stopped accepting frames after a data.csv error" in w for w in summary["warnings"])
    rows0 = read_rows(root / "mav0" / "cam0" / "data.csv")
    assert [r[0] for r in rows0] == [p[0] for p in pairs[:4]]
    for cam in ("cam0", "cam1"):
        assert len(list((root / "mav0" / cam / "data").glob("*.png"))) == 4
