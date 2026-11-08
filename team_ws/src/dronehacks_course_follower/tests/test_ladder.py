"""The speed ladder: which top speed the next run flies, and how a run's outcome moves it."""
from __future__ import annotations

import json
from pathlib import Path

import pytest

from dronehacks_course_follower.ladder import (DEFAULT_LADDER, LadderStep, chosen_run, parse_steps, read_step,
                                               record_run)

# Four smooth steps
_STEPS = (LadderStep(0.5, False), LadderStep(0.7, False), LadderStep(0.85, False), LadderStep(1.0, False))


def test_with_no_file_the_first_run_flies_the_slowest_step(tmp_path: Path) -> None:
    assert read_step(tmp_path / "ladder.json", len(_STEPS)) == 0


def test_a_run_never_moves_the_step_landed_or_not_so_the_team_escalates_by_hand(tmp_path: Path) -> None:
    path = tmp_path / "ladder.json"
    for landed in (True, True, False):
        record_run(path, read_step(path, len(_STEPS)), _STEPS, landed=landed)
        assert read_step(path, len(_STEPS)) == 0

    # A step set by hand is the one flown and kept
    path.write_text(json.dumps({"step": 2, "runs": []}), encoding="utf-8")
    record_run(path, read_step(path, len(_STEPS)), _STEPS, landed=True)
    assert read_step(path, len(_STEPS)) == 2


def test_every_run_is_kept_with_its_step_and_outcome(tmp_path: Path) -> None:
    path = tmp_path / "ladder.json"
    record_run(path, 0, _STEPS, landed=True)
    record_run(path, 1, _STEPS, landed=False)

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
    assert [step.speed_mps for step in parse_steps("0.5, 0.7,1.0")] == [0.5, 0.7, 1.0]
    with pytest.raises(ValueError, match="rise"):
        parse_steps("0.7,0.5")


def test_a_stop_entry_is_a_stop_and_go_step_and_is_recorded_as_one(tmp_path: Path) -> None:
    steps = parse_steps("stop:0.3,0.5,0.7")
    assert steps == (LadderStep(0.3, True), LadderStep(0.5, False), LadderStep(0.7, False))
    assert [str(step) for step in steps] == ["stop:0.3", "0.5", "0.7"]

    path = tmp_path / "ladder.json"
    record_run(path, read_step(path, len(steps)), steps, landed=True)
    assert json.loads(path.read_text(encoding="utf-8"))["runs"][0]["mode"] == "stop"
    with pytest.raises(ValueError, match="stop"):
        parse_steps("stop:fast,0.5")


def test_a_return_suffix_adds_the_return_and_ranks_above_the_same_speed_without() -> None:
    steps = parse_steps("stop:0.3,1.0,1.0+return")
    assert steps[-1] == LadderStep(1.0, stop=False, return_home=True) and str(steps[-1]) == "1+return"
    assert parse_steps("stop:0.3+return") == (LadderStep(0.3, stop=True, return_home=True),)
    with pytest.raises(ValueError, match="rise"):
        parse_steps("1.0+return,1.0")


def test_the_default_ladder_starts_stop_and_go_and_ends_with_the_return(tmp_path: Path) -> None:
    steps = parse_steps(DEFAULT_LADDER)
    assert steps[0] == LadderStep(0.3, stop=True) and steps[-1].return_home
    assert not any(step.return_home for step in steps[:-1])

    # Set by hand to the top, it flies the return
    path = tmp_path / "ladder.json"
    path.write_text(json.dumps({"step": len(steps) - 1}), encoding="utf-8")
    assert chosen_run(steps[read_step(path, len(steps))], float("nan"), 0.3, "", "").return_home


def test_the_launch_can_force_the_route_mode_and_the_return_over_the_ladder() -> None:
    stop_step = LadderStep(0.3, stop=True)
    assert chosen_run(stop_step, float("nan"), 0.3, "", "") == LadderStep(0.3, stop=True)
    assert chosen_run(stop_step, float("nan"), 0.3, "smooth", "") == LadderStep(0.3, stop=False)
    assert chosen_run(LadderStep(1.0, False, True), float("nan"), 0.3, "", "false").return_home is False

    # A given speed flies smooth unless stop-and-go is forced, at its own leg speed
    assert chosen_run(None, 0.8, 0.25, "", "true") == LadderStep(0.8, stop=False, return_home=True)
    assert chosen_run(None, 0.8, 0.25, "stop", "") == LadderStep(0.25, stop=True)
    with pytest.raises(ValueError, match="route_mode"):
        chosen_run(None, 0.8, 0.25, "fast", "")
