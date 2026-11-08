import json, os, struct, sys, tempfile
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from dronehacks_lidar_log.logger import MAGIC, RECORD, Recorder


def read(path):
    b = open(path, "rb").read()
    assert b.startswith(MAGIC)
    out, i = [], len(MAGIC)
    while i < len(b):
        t, n = RECORD.unpack_from(b, i); i += RECORD.size
        out.append((t, b[i:i + n])); i += n
    return out


def test_records_only_from_arming_to_the_tail_after_disarm():
    root = tempfile.mkdtemp()
    r = Recorder(root, tail_s=5.0)
    r.on_bytes(1.0, b"before")                       # disarmed: dropped
    assert r.on_state(False, 1.0) is None
    assert "armed" in r.on_state(True, 100.0, "/dev/x", 230400)
    r.on_bytes(100.1, b"\x54\x2cflight")
    assert "disarmed" in r.on_state(False, 130.0)
    r.on_bytes(133.0, b"tail")                       # inside the 5 s tail: kept
    assert r.on_state(False, 134.0) is None
    assert "closed" in r.on_state(False, 135.0)
    r.on_bytes(136.0, b"after")                      # closed: dropped
    folder = r.folder
    assert os.path.basename(folder).endswith("-lidar")
    assert read(os.path.join(folder, "lidar.bin")) == [(100.1, b"\x54\x2cflight"), (133.0, b"tail")]
    meta = json.load(open(os.path.join(folder, "meta.json")))
    assert meta["bytes"] == 12 and meta["device"] == "/dev/x" and meta["closed_wall_time"] == 135.0


def test_rearming_in_the_tail_keeps_one_file_and_a_new_arm_later_gets_a_new_one():
    root = tempfile.mkdtemp()
    r = Recorder(root, tail_s=5.0)
    r.on_state(True, 1000.0); first = r.folder
    r.on_state(False, 1010.0)
    assert "still recording" in r.on_state(True, 1012.0)
    r.on_state(False, 1020.0); r.on_state(False, 1026.0)
    assert not r.recording
    r.on_state(True, 2000.0)
    assert r.folder != first and r.recording
    assert "closed" in r.close(2001.0)
