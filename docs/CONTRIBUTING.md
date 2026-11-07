# Contributing

## 1. Branches & commits

- Branch prefixes: `localization/`, `navigation/`, `course/`, `docs/`, `infra/`, `contracts/`
- Commit grammar: `<area>: <imperative summary>` — e.g. `core: bound replay catch-up to one tick per frame`
- Never force-push or rewrite `main`'s history. Revert bad merges via a new PR.

## 2. PR requirements

- Use the PR template (`.github/pull_request_template.md`).
- Link every relevant board item (closes/blocked-by) in the Board section.
- Keep scope to one area unless a maintainer has explicitly approved cross-area work before you start.

## 3. Merge rules

- Squash-merge only: `gh pr merge <n> --squash --delete-branch --match-head-commit <sha>`
- Merge only after the PR carries the `review-passed` label.
- Only @moterodiaz runs the merge command. Agents must not merge or approve.

## 4. Recovery

- Revert a bad merge by opening a new revert PR — never rewrite `main`'s history.
- If a secret is committed, notify @moterodiaz immediately so it can be rotated before any history cleanup begins.
