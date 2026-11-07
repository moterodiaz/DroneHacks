# Contributing

## 1. Branches & commits

- Branch prefixes: `localization/`, `navigation/`, `course/`, `docs/`, `infra/`, `contracts/`
- Commit grammar: `<area>: <imperative summary>` — e.g. `core: bound replay catch-up to one tick per frame`
- Never force-push or rewrite `main`'s history. Revert bad merges via a new PR.

## 2. PR requirements

- Use the PR template (`.github/pull_request_template.md`).
- Link every relevant board item (closes/blocked-by) in the Board section.
- Keep scope focused. Record cross-area repository decisions and acceptance checks, and notify affected owners before implementation. Code, contract and guard decisions do not need per-PR human approval.

## 3. Merge rules

- Squash-merge only through `make merge-reviewed N=<n>` from a trusted policy checkout, dry-run first. See docs/BOARD.md for execution and review requirements.
- Merge only after the PR carries the `review-passed` label.
- Designated session correction agents may merge qualifying same-repository team PRs. No forks, direct main pushes, admin bypasses, force-pushes or self-sign-off.
- Spending, live flight, deployment and access changes remain manual. Physical validation and organizer restrictions cannot be waived by review.

## 4. Recovery

- Revert a bad merge by opening a new revert PR — never rewrite `main`'s history.
- If a secret is committed, notify @moterodiaz immediately so it can be rotated before any history cleanup begins.
