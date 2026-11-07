.PHONY: help labels board review-queue claim devlog merge-reviewed
.DEFAULT_GOAL := help

merge-reviewed: ## dry-run guarded merge; EXECUTE=1 enables the final write
	@test -n "$(N)" || (echo "usage: make merge-reviewed N=<pr-number> [EXECUTE=1]" && exit 1)
	python3 .github/scripts/merge_reviewed.py "$(N)" $(if $(filter 1,$(EXECUTE)),--execute,)

help: ## list targets
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-16s\033[0m %s\n", $$1, $$2}'

labels: ## create/update the full label taxonomy (idempotent)
	gh label create needs-review       --color "fbca04" --force --description "Open, unclaimed, head not yet reviewed"
	gh label create in-review          --color "0e8a16" --force --description "A reviewer has claimed this PR"
	gh label create changes-requested  --color "d93f0b" --force --description "Findings filed; back to the author"
	gh label create review-passed      --color "0e8a16" --force --description "Signed off at a recorded SHA; awaiting merge"
	gh label create needs-human        --color "b60205" --force --description "Manual spend, live-flight, deployment or access decision required"
	gh label create blocked            --color "e4e669" --force --description "Waiting on another issue/PR"
	gh label create review-finding     --color "5319e7" --force --description "An independently-closable defect found in review"
	gh label create task               --color "c5def5" --force --description "A standalone piece of work"
	gh label create decision           --color "b60205" --force --description "Recorded repository choice or manual-action escalation"
	gh label create contract           --color "5319e7" --force --description "Touches contracts/"
	gh label create area:localization  --color "c2e0c6" --force --description "localization/"
	gh label create area:navigation    --color "c2e0c6" --force --description "navigation/"
	gh label create area:course        --color "c2e0c6" --force --description "course/"
	gh label create area:docs          --color "c2e0c6" --force --description "docs/"
	gh label create area:infra         --color "c2e0c6" --force --description "infra/"
	gh label create area:contracts     --color "c2e0c6" --force --description "contracts/"

board: ## print every board lane
	@echo "--- needs-review ---";       gh pr list --label needs-review       --json number,title,author --jq '.[] | "#\(.number)\t\(.author.login)\t\(.title)"'
	@echo "--- in-review ---";          gh pr list --label in-review          --json number,title,assignees --jq '.[] | "#\(.number)\t\(.assignees|map(.login)|join(","))\t\(.title)"'
	@echo "--- changes-requested ---";  gh pr list --label changes-requested  --json number,title,author --jq '.[] | "#\(.number)\t\(.author.login)\t\(.title)"'
	@echo "--- review-passed ---";      gh pr list --label review-passed     --json number,title,author --jq '.[] | "#\(.number)\t\(.author.login)\t\(.title)"'
	@echo "--- needs-human ---";        gh issue list --label needs-human --state open --json number,title,assignees --jq '.[] | "#\(.number)\t\(.assignees|map(.login)|join(","))\t\(.title)"'
	@echo "--- blocked ---";            gh issue list --label blocked --state open --json number,title --jq '.[] | "#\(.number)\t\(.title)"'

review-queue: ## list claimable PRs, oldest first, with head SHA
	gh pr list --label needs-review --json number,title,headRefOid,author,createdAt --jq 'sort_by(.createdAt)[] | "#\(.number)\t\(.headRefOid[0:8])\t\(.author.login)\t\(.title)"'

claim: ## claim a PR for review: make claim N=12
	@test -n "$(N)" || (echo "usage: make claim N=<pr-number>" && exit 1)
	gh pr edit $(N) --remove-label needs-review --add-label in-review --add-assignee "@me"
	@echo "claimed #$(N) at $$(gh pr view $(N) --json headRefOid --jq .headRefOid)"

devlog: ## append a dated devlog fragment: make devlog WHO=Mateo MSG="..." SLUG=board
	@test -n "$(WHO)" || (echo "usage: make devlog WHO=<name> MSG=\"...\" [SLUG=topic]" && exit 1)
	@test -n "$(MSG)" || (echo "usage: make devlog WHO=<name> MSG=\"...\" [SLUG=topic]" && exit 1)
	@mkdir -p docs/history/devlog
	@slug="$(SLUG)"; \
	if [ -z "$$slug" ]; then slug=$$(git rev-parse --abbrev-ref HEAD | tr '/' '-' | tr -cs 'a-z0-9-' '-'); fi; \
	now=$$(date -u +%Y-%m-%dT%H%M); \
	f="docs/history/devlog/$${now}Z-$(WHO)-$${slug}.md"; \
	test ! -e "$$f" || (echo "devlog: $$f already exists" && exit 1); \
	printf '# %s\n\n%s\n' "$(WHO) — $$now" "$(MSG)" > "$$f"; \
	echo "wrote $$f"
