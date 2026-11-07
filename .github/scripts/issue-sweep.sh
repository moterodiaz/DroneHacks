#!/usr/bin/env bash
set -euo pipefail

repo=${GITHUB_REPOSITORY:-moterodiaz/DroneHacks}
dry=${DRY:-0}
override=${PR_OVERRIDE:-}
cutoff=$(date -u -v-24H +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || date -u -d '24 hours ago' +%Y-%m-%dT%H:%M:%SZ)
issues=$(gh issue list -R "$repo" --state open --limit 1000 \
  --json number,title,labels,assignees)

tmp=$(mktemp -d); trap 'rm -rf "$tmp"' EXIT
while IFS=$'\t' read -r issue title labels assignees; do
  [[ $title =~ ^PR\ \#([0-9]+):\  ]] || continue
  pr=${override:-${BASH_REMATCH[1]}}
  if jq -e 'index("needs-human") != null' <<<"$labels" >/dev/null; then
    echo "SKIP issue #$issue needs-human: $title"
    continue
  fi
  entry="#$issue ${title#*: }"
  printf '%s|' "$entry" >>"$tmp/f.$pr"
  printf '%s,' "$assignees" >>"$tmp/a.$pr"
done < <(jq -r '.[] | [.number,.title,([.labels[].name]|@json),([.assignees[].login]|join(","))] | @tsv' <<<"$issues")

for f in "$tmp"/f.*; do
  [[ -e $f ]] || break
  pr=${f##*/f.}; findings=$(<"$f"); findings=${findings%|}; assignees=$(<"$tmp/a.$pr")
  data=$(gh pr view "$pr" -R "$repo" \
    --json state,mergedAt,mergeCommit,labels,assignees,author,headRefOid)
  state=$(jq -r .state <<<"$data")
  if [[ $state == MERGED ]]; then
    sha=$(jq -r '.mergeCommit.oid // "unknown"' <<<"$data")
    IFS='|' read -ra rows <<<"$findings"
    for row in "${rows[@]}"; do
      issue=${row%% *}; issue=${issue#\#}
      msg="PR #$pr merged at $sha; closing."
      if [[ $dry == 1 ]]; then
        echo "CLOSE issue #$issue -- $msg"
      else
        gh issue close "$issue" -R "$repo" --comment "$msg"
      fi
    done
    continue
  fi
  if [[ $state == CLOSED ]]; then
    echo "JUDGE PR #$pr closed unmerged; findings: $findings"
    continue
  fi

  comments=$(gh api --paginate "repos/$repo/issues/$pr/comments")
  marker=$(jq -sr '[.[][] | select(.body | contains("<!-- issue-sweep -->")) | .created_at] | max // ""' <<<"$comments")
  latest=$(jq -sr '[.[][]] | max_by(.created_at) // {} | [.created_at // "", .user.type // ""] | @tsv' <<<"$comments")
  IFS=$'\t' read -r latest_at latest_type <<<"$latest"
  if [[ -n $marker && $marker > $cutoff ]]; then
    echo "THROTTLE PR #$pr: issue-sweep comment is less than 24h old"
    continue
  elif [[ $latest_type != Bot && -n $latest_at && ( -z $marker || $latest_at > $marker ) ]]; then
    echo "THROTTLE PR #$pr: latest comment is newer human activity"
    continue
  fi

  label=$(jq -r '[.labels[].name] | if index("changes-requested") then "changes-requested" elif index("in-review") then "in-review" elif index("needs-review") then "needs-review" elif index("review-passed") then "review-passed" else "" end' <<<"$data")
  head=$(jq -r .headRefOid <<<"$data")
  author=$(jq -r .author.login <<<"$data")
  reviewers=$(jq -r '[.assignees[].login] | join(" @")' <<<"$data")
  people=$(jq -nr --arg people "$assignees" \
    '$people | split(",") | map(select(length > 0)) | unique | map("@" + .) | join(" ")')
  case $label in
    changes-requested) who=${people:-@$author}; note="$who please address these open findings and push a fix; board.yml will re-queue the PR." ;;
    in-review) who=${reviewers:+@$reviewers}; who=${who:-@$author}; note="$who review of head \`$head\` is pending." ;;
    needs-review) note="@$author this review is unclaimed; run \`make review-queue\` then \`make claim N=$pr\`." ;;
    review-passed) note="Independent review recorded for \`$head\`; run \`make merge-reviewed N=$pr\` from a trusted policy checkout. Open findings still block merging." ;;
    *) echo "SKIP PR #$pr: no recognized state label"; continue ;;
  esac
  body=$(printf '<!-- issue-sweep -->\n%s\n\nOpen findings:\n' "$note")
  IFS='|' read -ra rows <<<"$findings"; for row in "${rows[@]}"; do body+=$'\n'"- $row"; done
  if [[ $dry == 1 ]]; then echo "PING PR #$pr [$label]: ${body//$'\n'/ | }"; else gh pr comment "$pr" -R "$repo" --body "$body"; fi
done
