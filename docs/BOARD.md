# Board

GitHub Issues and PR labels are the agent coordination layer. No chat or relay between agents — everything needed to act is in the issue or PR body and its labels. An agent reading a message shares no conversation with the one that wrote it; the body is the entire brief.

## Owners

| Area | Branch prefix | Owner |
|---|---|---|
| core | `core/*` | @moterodiaz |
| docs | `docs/*` | @moterodiaz |
| infra | `infra/*` | @moterodiaz |
| contracts | `contracts/*` | @moterodiaz |

Real areas and per-area owners replace these generic rows as code lands — add a row, don't redesign the table. Team: @jsun4231, @mg343, @Orca6722, @TheSleepyPP — assign them as areas are claimed.

## Label taxonomy

### State labels (exactly one per PR at a time)

| Label | Meaning | Set by |
|---|---|---|
| `needs-review` | Open, unclaimed, head not yet reviewed | `board.yml` workflow |
| `in-review` | A reviewer has claimed it — this is the lock | Reviewer |
| `changes-requested` | Findings filed; back to the author | Reviewer |
| `review-passed` | Signed off at a recorded SHA; awaiting merge authorization | Reviewer |

### Blocking labels (additive, on PRs or issues)

| Label | Meaning | Note |
|---|---|---|
| `needs-human` | Stop — a maintainer decision is required | Assignee is who must decide |
| `blocked` | Waiting on another issue/PR | Body must name the blocker |

### Kind labels (on issues)

| Label | Meaning |
|---|---|
| `review-finding` | An independently-closable defect found in review |
| `task` | A standalone piece of work |
| `decision` | A choice only a maintainer can make |
| `contract` | Touches `contracts/` |

### Area labels

`area:core`, `area:docs`, `area:infra`, `area:contracts`

## Title grammar

| Kind | Pattern | Example |
|---|---|---|
| Review finding | `PR #<n>: <problem>` | `PR #12: sim clock drifts under 4x replay; no bound on catch-up` |
| Decision | `PR #<n>: <what> — needs decision` | `PR #12: raises flight-envelope ceiling — needs decision` |
| Task | `<area>: <imperative>` | `core: bound replay catch-up to one tick per frame` |

## The handshake

1. **Implementer opens a PR** from an area branch (`core/*`, `docs/*`, `infra/*`, `contracts/*`). The `board.yml` workflow automatically strips any stale `in-review`, `review-passed`, or `changes-requested` labels left over from a previous head, then applies `needs-review`.

2. **A review agent runs `make review-queue`**, takes the oldest entry, and immediately runs `make claim N=<n>` to atomically swap `needs-review` → `in-review` and self-assign. The label swap IS the lock — a second reviewer who lists the queue after this point sees nothing to claim for this PR.

3. **The reviewer reads the diff at a pinned SHA** (`gh pr view <n> --json headRefOid`) and records that SHA in every comment and issue it files from this review. All findings are anchored to the exact snapshot reviewed.

4. **Exactly one of three outcomes:**

   - **Clean:** The reviewer labels the PR `review-passed` and comments `reviewed <sha>: pass`. The reviewer does NOT merge — `.claude/settings.json` denies `gh pr merge` and `gh pr review --approve` to agents entirely.

   - **Findings:** The reviewer files one GitHub issue per independent finding (so each can close independently) using the `review-finding` issue template, titled `PR #<n>: <problem>`, labelled `review-finding` + the relevant `area:*` label, assigned to the PR author. All finding issues are linked from one summary comment on the PR. The reviewer labels the PR `changes-requested`.

   - **Needs a human decision** (spend, production/deploy action, security-guard change, contract break, or cross-area invariant conflict — see AGENTS.md §3.5): open an issue with the `decision` template, label `needs-human`, assign @moterodiaz (or the relevant area owner once assigned). The issue body must state what the PR does, why it is being escalated and which AGENTS.md §3 invariant triggered it, the available options, and a recommendation.

5. **Author pushes a fix**, comments the resolving issue number(s) on the PR. The next push (a `synchronize` event) automatically flips `changes-requested` back to `needs-review` via `board.yml` — this is also how a stale `review-passed` gets invalidated: any new push strips it and re-queues the PR for a fresh review.

6. **Maintainer merges at the exact reviewed head:**
   ```
   gh pr merge <n> --squash --delete-branch --match-head-commit <reviewed-sha>
   ```
   If the head has moved since review, this command fails rather than merging unreviewed code — that is the point of recording the SHA in step 3.

## Rules that make it work without chat

- The issue/PR body is the entire brief. Include absolute file paths, the head SHA, and the acceptance check that proves a fix. Never write "as discussed" or "the file we looked at" — the agent reading it shares no conversation history with the one that wrote it.
- One finding per issue. Each must be independently closable.
- Cross-link blockers in both directions: `Blocked by #7` on the blocked item, `Blocks #12` on the blocker.
- A stale SHA invalidates a review. Always state the SHA you reviewed; always check whether the head has moved before acting on a `review-passed`.
- Agents never merge, never approve, never force-push. Only the maintainer runs the merge command in step 6.

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

`.github/workflows/claude-review.yml` already runs an automated Claude-based reviewer on every PR open, sync, reopen, and ready-for-review event. It posts inline comments and can file `[claude] ...` issues for out-of-scope defects. It participates in the same board as a human or agent reviewer would, but currently does not set board state labels itself. A human or another agent should read its output and apply the appropriate state label (`review-passed` or `changes-requested`) until or unless that workflow is extended to do so.
