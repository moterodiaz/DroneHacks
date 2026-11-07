# Board

GitHub Issues and PR labels are the agent coordination layer. No chat or relay between agents — everything needed to act is in the issue or PR body and its labels. An agent reading a message shares no conversation with the one that wrote it; the body is the entire brief.

## Owners

| Area | Branch prefix | Owner |
|---|---|---|
| localization | `localization/*` | @moterodiaz |
| navigation | `navigation/*` | @moterodiaz |
| course | `course/*` | @moterodiaz |
| infra | `infra/*` | @moterodiaz |
| docs | `docs/*` | @moterodiaz |
| contracts | `contracts/*` | @moterodiaz |

Real areas and per-area owners replace these generic rows as code lands — add a row, don't redesign the table. Team: @jsun4231, @mg343, @Orca6722, @TheSleepyPP — assign them as areas are claimed.

## Label taxonomy

### State labels (exactly one per PR at a time)

| Label | Meaning | Set by |
|---|---|---|
| `needs-review` | Open, unclaimed, head not yet reviewed | `board.yml` workflow |
| `in-review` | A reviewer has claimed it — this is the lock | Reviewer |
| `changes-requested` | Findings filed; back to the author | Reviewer |
| `review-passed` | Independently signed off at a recorded SHA; subject to merge gates | Independent reviewer |

### Blocking labels (additive, on PRs or issues)

| Label | Meaning | Note |
|---|---|---|
| `needs-human` | Stop: spend, live flight, deployment or access decision | Assignee is who must decide |
| `blocked` | Waiting on another issue/PR | Body must name the blocker |

### Kind labels (on issues)

| Label | Meaning |
|---|---|
| `review-finding` | An independently-closable defect found in review |
| `task` | A standalone piece of work |
| `decision` | Recorded repository choice or manual-action escalation |
| `contract` | Touches `contracts/` |

### Area labels

`area:localization`, `area:navigation`, `area:course`, `area:docs`, `area:infra`, `area:contracts`

## Title grammar

| Kind | Pattern | Example |
|---|---|---|
| Review finding | `PR #<n>: <problem>` | `PR #12: sim clock drifts under 4x replay; no bound on catch-up` |
| Decision | `PR #<n>: <what> — needs decision` | `PR #12: raises flight-envelope ceiling — needs decision` |
| Task | `<area>: <imperative>` | `core: bound replay catch-up to one tick per frame` |

## The handshake

1. **Implementer opens a PR** from an area branch (`localization/*`, `navigation/*`, `course/*`, `docs/*`, `infra/*`, `contracts/*`). The `board.yml` workflow automatically strips any stale `in-review`, `review-passed`, or `changes-requested` labels left over from a previous head, then applies `needs-review`.

2. **A review agent runs `make review-queue`**, takes the oldest entry, and immediately runs `make claim N=<n>` to atomically swap `needs-review` → `in-review` and self-assign. The label swap IS the lock — a second reviewer who lists the queue after this point sees nothing to claim for this PR.

3. **The reviewer reads the diff at a pinned SHA** (`gh pr view <n> --json headRefOid`) and records that SHA in every comment and issue it files from this review. All findings are anchored to the exact snapshot reviewed.

4. **Exactly one of three outcomes:**

   - **Clean:** The independent reviewer labels the PR `review-passed` and comments `reviewed <full-sha>: pass`. Automated merging additionally requires the structured trusted review described below. The fixer cannot supply this sign-off. Direct `gh pr merge` and self-approval remain denied.

   - **Findings:** The reviewer files one GitHub issue per independent finding (so each can close independently) using the `review-finding` issue template, titled `PR #<n>: <problem>`, labelled `review-finding` + the relevant `area:*` label, assigned to the PR author. All finding issues are linked from one summary comment on the PR. The reviewer labels the PR `changes-requested`.

   - **Decision:** Record repository code, contract, cross-area and guard decisions with rationale, alternatives, acceptance checks and affected-owner notification. These may be autonomous. For spend, live flight, deployment or access changes, add `needs-human` and assign @moterodiaz. Missing physical evidence or organizer permission stays blocked. Reassess old approval-only holds individually, never bulk-close technical findings.

5. **Author or designated correction agent pushes a verified fix**, using an isolated worktree and fresh identity, head, repository, label and blocker checks. Team PRs in this repository only, never forks or force pushes. Comment the resolving issue number(s). The next push (`synchronize`) invalidates old review labels via `board.yml` and requires fresh independent review.

6. **Guarded squash merge**, by the designated session correction agent:
   run `make merge-reviewed N=<n>` from a trusted policy checkout for a dry run.
   After it passes, use `make merge-reviewed N=<n> EXECUTE=1` from a clean checkout
   of current main. The sole bootstrap exception is PR #31's independently
   reviewed exact head, verified by the pinned legacy main reviewer described
   below. Never execute a helper or Makefile from unreviewed PR code.
   The helper requires same-repo team authors, independent exact-full-head/base
   review, recorded passing relevant tests, passing project/required checks,
   no unresolved threads, changes-requested reviews, linked findings/decisions,
   locks, pauses or blockers, and fresh identity/head/base/mergeability checks.
   It journals intent and verifies the result; uncertain writes stop for
   reconciliation, never blind retry. GitHub atomically guards the head SHA,
   but offers no base-SHA merge precondition; a final base read is not a lock.
   No admin bypass, direct main push, self-sign-off or branch deletion.
   Claude's automatic command permission is scoped to
   `python3 .github/scripts/merge_reviewed.py <n> [--execute]`, not arbitrary
   Make invocations. The Make alias validates its environment arguments without
   evaluating Make or shell expressions. Reviewer tools expose read views and
   intended comment/board writes, not general-purpose raw API writes.

   For a stopped intent, run `python3 .github/scripts/merge_reviewed.py <n>
   --reconcile` from the trusted checkout. This never sends a merge request.
   It records an observed merge, or clears an intent only when no HTTP attempt
   was journaled (new prepared intents), or when an open PR has a different
   head that excludes the old expected-SHA request. An attempted same-head
   write with unknown outcome remains blocked; inspect GitHub and the journal,
   never append a fabricated `not-merged` event or blindly repeat the request.
   A detected base race before HTTP now records `not-merged` automatically.

## Rules that make it work without chat

- The issue/PR body is the entire brief. Include absolute file paths, the head SHA, and the acceptance check that proves a fix. Never write "as discussed" or "the file we looked at" — the agent reading it shares no conversation history with the one that wrote it.
- One finding per issue. Each must be independently closable.
- Cross-link blockers in both directions: `Blocked by #7` on the blocked item, `Blocks #12` on the blocker.
- A stale SHA invalidates a review. Always state the SHA you reviewed; always check whether the head has moved before acting on a `review-passed`.
- Agents may only merge through step 6. Never self-approve, force-push or bypass the gates. Spending, live flight, deployment and access changes remain manual; no review fabricates physical evidence or organizer permission.

## No pinned board-index issue

There is no pinned board-index issue. Labels plus the `gh` queries below are the board. A hand-maintained index issue drifts out of date within a day; `make board` is derived from live label state and needs no upkeep.

## Queries

The following are the raw `gh` commands that `make board`, `make review-queue`, and `make claim N=<n>` expand to. An agent without `make` can run these directly.

**`make board`** — print every board lane:
```sh
echo "--- needs-review ---"
gh pr list --label needs-review --json number,title,author --jq '.[] | "#\(.number)\t\(.author.login)\t\(.title)"'
echo "--- in-review ---"
gh pr list --label in-review --json number,title,assignees --jq '.[] | "#\(.number)\t\(.assignees|map(.login)|join(","))\t\(.title)"'
echo "--- changes-requested ---"
gh pr list --label changes-requested --json number,title,author --jq '.[] | "#\(.number)\t\(.author.login)\t\(.title)"'
echo "--- review-passed ---"
gh pr list --label review-passed --json number,title,author --jq '.[] | "#\(.number)\t\(.author.login)\t\(.title)"'
echo "--- needs-human ---"
gh issue list --label needs-human --state open --json number,title,assignees --jq '.[] | "#\(.number)\t\(.assignees|map(.login)|join(","))\t\(.title)"'
echo "--- blocked ---"
gh issue list --label blocked --state open --json number,title --jq '.[] | "#\(.number)\t\(.title)"'
```

**`make review-queue`** — list claimable PRs, oldest first, with head SHA:
```sh
gh pr list --label needs-review --json number,title,headRefOid,author,createdAt \
  --jq 'sort_by(.createdAt)[] | "#\(.number)\t\(.headRefOid[0:8])\t\(.author.login)\t\(.title)"'
```

**`make claim N=<n>`** — claim a PR for review:
```sh
gh pr edit <n> --remove-label needs-review --add-label in-review --add-assignee "@me"
gh pr view <n> --json headRefOid --jq .headRefOid
```

## Automated Claude reviewer

The existing workflow reviews PRs and supports review-only `@claude` mentions.
A green check or free-text praise is not sign-off. Only the action-created
`claude[bot]` tracking comment with a `dronehacks-review-v1` JSON attestation qualifies
for automated merging. It records repository, PR, full head and base SHAs,
run ID, independent verdict, zero findings, tests (command/result/evidence),
manual actions and missing evidence. The helper verifies the tracking comment's
ID against the action's pre-model job log, rejecting missing, mismatched or
ambiguous binding. Tracking edits are allowed only within the completed run,
not afterward. The latest updated exact-head/base verdict must be pass;
comments do not dismiss an outstanding changes-requested review.
Every relevant test must pass; absent
physical validation remains a blocker, never a guessed pass.

The attestation is a visible fenced code block with language
`dronehacks-review-v1` and a JSON object inside. Do not use an HTML comment:
the review action strips HTML comments from both requests and tracking updates.

Test entries require `source: "executed"` and the actual CI `run_id`.
The helper verifies that independent exact-head/base CI execution completed the
`Isolated Python tests` step successfully under the reviewed test workflow.
The existing `changes` job runs the exact head's exported source in a read-only
container with no network, credentials or Git metadata, bounded resources and
a timeout. Zero discovered tests fail. Review must still assess coverage and
dependency gaps; author-reported results and unrelated passing tests cannot
establish relevant coverage. No new CI job is added.

The helper accepts only a successful team-triggered **main-branch mention**
run using the exact trusted workflow, with a run-bound review-context artifact.
PR-triggered review is useful feedback but not merge authority. Workflow-changing
PRs therefore cannot certify themselves. For **policy PR #31 only**, the helper
also accepts the existing main-branch mention reviewer at pinned workflow blob
`fa10d8dc5102314526de6d6c1477b659424e8ec3`. This bootstrap review must independently
provide the same structured exact-head/base verdict and tests, plus
`bootstrap: true` and `request_comment`, bound to an unedited team review-only
request containing both full SHAs. The bot, successful main run, workflow blob,
timing, action-log comment binding, independent authorship and request are all verified. Only that reviewed
policy checkout may bootstrap execution. No other PR gets this exception, and
the candidate workflow's PR-triggered run never supplies its own authority.

Operate in this session only. Add no CI job, daemon or service. Keep one
five-minute loop after rollout, paused during policy publication/review.
