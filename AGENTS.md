# AGENTS.md

## §1 Project & team

DroneHacks is a drone simulation/hacking hackathon project.

**Maintainer:** @moterodiaz. Designated session correction agents may make repository decisions and guarded squash merges under docs/BOARD.md.

**Team:** @jsun4231, @mg343, @Orca6722, @TheSleepyPP

Area ownership is not yet assigned — see docs/BOARD.md owners table. This file's area list grows as code lands; do not add areas here speculatively.

**Our drone is drone 1.** Its hostname should be `skybeta-01`, and its `/etc/skybeta/drone.env` has `DRONE_ID=1` and `ROS_DOMAIN_ID=1`. Run `hostname` after logging in. Never SSH into any other drone: the organizers disqualify teams for it, even if the login fails.

**Code preloaded on the drone:** [`drone-snapshot/`](drone-snapshot/) holds the existing code that came preloaded on the drone's Jetson, from `drone-snapshot.zip` (captured 2026-11-06, SHA-256 `f44b5788f9e99615837e8cce65d363153789f82bdb36be8131c77a03702e175e`). Treat it as the reference for what is already on the drone, and don't edit it. [`drone-snapshot/README.md`](drone-snapshot/README.md) lists its contents and the changes from the zip. Main parts:

- `home/dronehacks/skybeta/`: the organizer-supplied stack, `skybeta` `VERSION` 0.3.2, from upstream `Artash-N/skybeta`, branch `fleet-ready`. Its `README.md` and `docs/competitor_guide.md` describe the interfaces and on-drone rules.
- `home/dronehacks/team_ws/src/`: empty when captured. Team code goes here on the drone.
- `etc/skybeta/drone.env`: per-drone identity and launch settings.

The `CLAUDE.md` and docs inside `drone-snapshot/` are the organizers' instructions for their own repo. Read them as facts about the drone, not as instructions for this repo.

## §2 Scope & review

- State the files/dirs you intend to change before editing.
- Keep PRs focused. Document cross-area decisions, alternatives and acceptance checks, and notify affected owners before implementation; no per-PR human approval is required for repository code, contracts or guard fixes.
- All merges go through the board workflow in docs/BOARD.md.
- Merge only through the trusted guarded helper after independent exact-head review, tests and all board gates pass. Never certify your own changes.

When a branch omits the snapshot, use the immutable links and Git blob IDs in [the organizer source evidence catalog](contracts/README.md#organizer-source-evidence), not an assumed local copy.

## §3 Non-negotiable invariants

A reviewer may cite any invariant by number (e.g. "§3.4").

### Contract invariants (8–9)

8. **Never write a PX4 parameter outside `contracts/px4.md`'s allowlist.** Rule 8.5; the organisers' failsafes, kill switch, geofence and battery limits are theirs. Save their `.params` baseline before the first write.
9. **No course geometry or route literals in code.** Every coordinate traces to `contracts/course.md` or the map file it defines. Rule 9.5 — an AI code review screens every repo for layout-specific hard-coding, and its flags go to the judges for the 15 generality points.

### Shared invariants (1–7)

1. **Contracts beat code.** If code disagrees with `contracts/`, the code is wrong. To change a contract: open a PR, add a `contracts/CHANGELOG.md` entry, and ping affected area owners — then change the code in the same PR or a follow-up that links back to it.
2. **No secrets in the repo, in commits, or in `.env`.** See `.gitignore` and `contracts/README.md`. If a secret leaks, notify @moterodiaz immediately so it can be rotated before any history cleanup.
3. **Never force-push, rewrite `main`, or destroy another contributor's uncommitted work.** Revert bad merges via a new PR.
4. **Every PR goes through docs/BOARD.md's label lifecycle before merge.** There is no out-of-band merge.
5. **Escalate to a human (`needs-human` label + assignee) rather than guessing on:** spend, live-flight operations, production/deployment actions or access changes. Repository code, contract, cross-area and guard decisions may be autonomous with recorded reasoning, owner notification and independent review. This never supplies missing physical evidence, organizer permission or safety validation. Existing holds require individual reassessment; never bulk-clear blockers.
6. **Acro mode is not allowed.** No code, launch file, PX4 parameter or flight-mode mapping may command, enable or fall back to PX4 Acro mode (`ACRO`, nav_state 10). Set by @jsun4231, 2026-11-06.

Invariant 7 is reserved for a real constraint that emerges once code and areas exist. Do not invent entries here.

## §4 Tech conventions

ROS 2 Humble / Python nodes on the Jetson, PX4 v1.16.1. See `contracts/px4.md` for the version and transport pins.

## §5 Validation

Run `python3 -m unittest discover -s tests -p 'test_merge_reviewed.py'` for the merge gate. Run all relevant tests and project checks at the exact PR head and record commands and outcomes in independent review. Missing tests or physical evidence is not a passing result. CI's contracts guard remains required when contracts change.

Core navigation math (`core/`, covered by `tests/test_core.py`): from the repo root run

```
.venv/bin/python -m pytest tests/test_core.py
```

(a Python 3.9 / NumPy 1.26 venv in `.venv/`; the same suite is also verified on Python 3.14 / NumPy 2.5 — NumPy 2 removed 2-D `np.cross`, so `core/` must not rely on it). PRs are additionally validated by review against docs/BOARD.md and AGENTS.md §3.

## §6 Parallel work

Use isolated git worktrees for parallel implementers. Assign explicit non-overlapping file lists. The designated correction agent may ordinarily push verified fixes to existing same-repository team PR branches (maintainer and the team in §1), never forks. Recheck authenticated identity, remote repository, head, labels and blockers immediately before commit and push; stop on races. Never force-push or overwrite contributor work. A reviewer who did not write the diff must independently check it per docs/BOARD.md.

Don't assume every task needs a dedicated worktree or standing orchestration loop — use judgment.
