"""The speed ladder: which top speed the next run flies, and how a run's outcome moves it."""
from __future__ import annotations

import json
from pathlib import Path

import pytest

from dronehacks_course_follower.ladder import parse_steps, read_step, record_run

# Four steps, as the default ladder has
_STEPS = (0.5, 0.7, 0.85, 1.0)


def test_with_no_file_the_first_run_flies_the_slowest_step(tmp_path: Path) -> None:
    assert read_step(tmp_path / "ladder.json", len(_STEPS)) == 0


def test_a_clean_landing_moves_the_next_run_up_one_step_and_stops_at_the_top(tmp_path: Path) -> None:
    path = tmp_path / "ladder.json"
    for expected in (1, 2, 3, 3):
        step = read_step(path, len(_STEPS))
        assert record_run(path, step, _STEPS, landed=True) == expected

    assert read_step(path, len(_STEPS)) == 3


def test_a_run_that_did_not_land_keeps_its_speed_and_every_run_is_kept(tmp_path: Path) -> None:
    path = tmp_path / "ladder.json"
    record_run(path, 0, _STEPS, landed=True)
    assert record_run(path, 1, _STEPS, landed=False) == 1

    runs = json.loads(path.read_text(encoding="utf-8"))["runs"]
    assert [(run["speed_mps"], run["landed"]) for run in runs] == [(0.5, True), (0.7, False)]


def test_a_hand_edited_or_broken_file_is_read_safely(tmp_path: Path) -> None:
    path = tmp_path / "ladder.json"

    # The team sets the step by hand to repeat or skip ahead; out of range is pulled back in
    path.write_text('{"step": 9}', encoding="utf-8")
    assert read_step(path, len(_STEPS)) == 3

    path.write_text("not json", encoding="utf-8")
    assert read_step(path, len(_STEPS)) == 0


def test_steps_are_read_from_a_comma_list_and_must_rise() -> None:
    assert parse_steps("0.5, 0.7,1.0") == (0.5, 0.7, 1.0)
    with pytest.raises(ValueError, match="rise"):
        parse_steps("0.7,0.5")
