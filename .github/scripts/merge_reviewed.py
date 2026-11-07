#!/usr/bin/env python3
"""Session-only, fail-closed squash merge gate. No PR code is executed here."""
import argparse
import base64
import fcntl
import hashlib
import io
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import uuid
import zipfile

REPO = "moterodiaz/DroneHacks"
TEAM = {"moterodiaz", "jsun4231", "mg343", "Orca6722", "TheSleepyPP"}
STATES = {"needs-review", "in-review", "changes-requested", "review-passed"}
BLOCKERS = {"blocked", "needs-human", "paused", "in-review"}
WORKFLOW = ".github/workflows/claude-review.yml"
BOOTSTRAP_PR = 31
# Git blob independently read from the pre-rollout main a3592e2, never PR input.
LEGACY_WORKFLOW_BLOB = "fa10d8dc5102314526de6d6c1477b659424e8ec3"
MARKER = re.compile(r"<!-- dronehacks-review-v1\s+(\{.*?\})\s*-->", re.S)
SHA = re.compile(r"[0-9a-f]{40}")


def require(condition, reason):
    if not condition:
        raise RuntimeError(reason)


def command(args):
    result = subprocess.run(args, capture_output=True, text=True, timeout=90)
    require(result.returncode == 0, f"{args[0]} failed: {result.stderr.strip()}")
    return result.stdout


def api(path, method="GET", data=None):
    args = ["gh", "api", "--method", method, path]
    if data is not None:
        args += ["--input", "-"]
        result = subprocess.run(args, input=json.dumps(data), capture_output=True,
                                text=True, timeout=90)
        require(result.returncode == 0, f"GitHub {method} failed: {result.stderr.strip()}")
        return json.loads(result.stdout)
    return json.loads(command(args))


def pages(path, field=None):
    """Follow every page, without gh's --slurp/--jq interaction."""
    output = []
    page = 1
    while True:
        chunk = api(f"{path}{'&' if '?' in path else '?'}per_page=100&page={page}")
        rows = chunk[field] if field else chunk
        require(isinstance(rows, list), "Malformed paginated response")
        output.extend(rows)
        if len(rows) < 100:
            return output
        page += 1


def threads(number):
    query = """query($cursor:String) {
      repository(owner:"moterodiaz",name:"DroneHacks") {
        pullRequest(number:NUMBER) {
          reviewThreads(first:100,after:$cursor) {
            nodes { isResolved } pageInfo { hasNextPage endCursor }
          }
        }
      }
    }""".replace("NUMBER", str(number))
    cursor = None
    result = []
    while True:
        args = ["gh", "api", "graphql", "-f", f"query={query}"]
        if cursor:
            args += ["-f", f"cursor={cursor}"]
        response = json.loads(command(args))
        require(not response.get("errors"), "Incomplete review-thread read")
        data = response["data"]["repository"]["pullRequest"]["reviewThreads"]
        result.extend(data["nodes"])
        if not data["pageInfo"]["hasNextPage"]:
            return result
        cursor = data["pageInfo"]["endCursor"]
        require(cursor, "Missing review-thread pagination cursor")


def collect(number):
    prefix = f"repos/{REPO}"
    pr = api(f"{prefix}/pulls/{number}")
    head, base = pr["head"]["sha"], pr["base"]["sha"]
    return {
        "actor": api("user")["login"], "pr": pr,
        "comments": pages(f"{prefix}/issues/{number}/comments"),
        "reviews": pages(f"{prefix}/pulls/{number}/reviews"),
        "threads": threads(number),
        "issues": pages(f"{prefix}/issues?state=open"),
        "checks": pages(f"{prefix}/commits/{head}/check-runs", "check_runs"),
        "statuses": pages(f"{prefix}/commits/{head}/statuses"),
        "files": pages(f"{prefix}/pulls/{number}/files"),
        "commits": pages(f"{prefix}/pulls/{number}/commits"),
        # Branch rules include applicable rulesets; inability to inspect fails closed.
        "rules": api(f"{prefix}/rules/branches/main"),
        "base": api(f"{prefix}/git/ref/heads/main")["object"]["sha"],
        "head": head, "base_at_start": base,
    }


def validate(snapshot, number):
    p = snapshot["pr"]
    head, base = snapshot["head"], snapshot["base"]
    require(snapshot["actor"] in TEAM, "Authenticated account is not on the team")
    require(p["user"]["login"] in TEAM, "PR author is not on the team")
    require(p["head"]["repo"] and p["head"]["repo"]["full_name"] == REPO,
            "Fork PRs are excluded")
    require(p["base"]["repo"]["full_name"] == REPO and p["base"]["ref"] == "main",
            "Unexpected base repository or branch")
    require(SHA.fullmatch(head) and SHA.fullmatch(base), "Full SHAs required")
    require(base == snapshot["base_at_start"], "Base moved during collection")
    require(p["state"] == "open" and not p["draft"] and not p["locked"],
            "PR is closed, draft or locked")
    labels = {v["name"] for v in p["labels"]}
    require(not labels & BLOCKERS, "PR has a blocking label")
    require(labels & STATES == {"review-passed"}, "Independent board sign-off required")
    require(p["mergeable"] is True and p["mergeable_state"] == "clean",
            "Mergeability is unknown or not clean")
    require(not any(not t["isResolved"] for t in snapshot["threads"]),
            "Unresolved review threads")
    # Mere historical/example mentions do not transfer another PR's blockers.
    linked = re.compile(rf"^PR #{number}:|"
                        rf"github\.com/{re.escape(REPO)}/pull/{number}(?![0-9])|"
                        rf"\bblocks\s+#{number}(?![0-9])", re.I)
    texts = [p["body"] or ""] + [c["body"] or "" for c in snapshot["comments"]]
    for issue in snapshot["issues"]:
        if issue["number"] == number:
            continue
        blocked_by = re.compile(
            rf"\bblocked[\s-]+by\s+(?:#{issue['number']}|"
            rf"https://github\.com/{re.escape(REPO)}/(?:issues|pull)/"
            rf"{issue['number']})(?![0-9])", re.I)
        require(not any(blocked_by.search(text) for text in texts),
                f"Explicit open dependency #{issue['number']}")
        issue_labels = {v["name"] for v in issue["labels"]}
        relation = bool(linked.search(issue["title"] + "\n" + (issue["body"] or "")))
        explicit = re.compile(
            rf"\b(?:blocked[\s-]+by|close[sd]?|fix(?:e[sd])?|resolve[sd]?)\s+"
            rf"(?:#{issue['number']}|https://github\.com/{re.escape(REPO)}/"
            rf"issues/{issue['number']})(?![0-9])", re.I)
        relation |= any(explicit.search(text) for text in texts)
        require(not (relation and issue_labels & (BLOCKERS | {"review-finding", "decision"})),
                f"Open linked blocker/finding/decision #{issue['number']}")
    # COMMENTED/PENDING do not clear a prior change request. DISMISSED is a
    # server-side dismissal of that review, not a new blocking decision.
    latest = {}
    for review in snapshot["reviews"]:
        if review["state"] in {"APPROVED", "CHANGES_REQUESTED"}:
            latest[review["user"]["login"]] = review
    require(not any(r["state"] == "CHANGES_REQUESTED" for r in latest.values()),
            "Outstanding changes-requested review")
    checks = {}
    for check in snapshot["checks"]:
        key = (check["name"], check["app"]["id"])
        if key not in checks or check["id"] > checks[key]["id"]:
            checks[key] = check
    require(checks, "No project checks")
    for c in checks.values():
        require(c["head_sha"] == head and c["status"] == "completed" and
                c["conclusion"] in {"success", "skipped"},
                f"Check not passing: {c['name']}")
    statuses = {}
    for status in snapshot["statuses"]:
        statuses.setdefault(status["context"], status)
    require(all(s["state"] == "success" for s in statuses.values()),
            "Pending or failed commit status")
    required = []
    for rule in snapshot["rules"]:
        if rule["type"] == "required_status_checks":
            required += rule["parameters"]["required_status_checks"]
    for context in required:
        matching = [c for c in checks.values() if c["name"] == context["context"] and
                    (not context.get("integration_id") or
                     c["app"]["id"] == context["integration_id"]) and
                    c["conclusion"] == "success"]
        require(matching or (not context.get("integration_id") and
                             context["context"] in statuses and
                             statuses[context["context"]]["state"] == "success"),
                f"Required context missing: {context['context']}")
    require(any(c["name"] == "changes" and c["conclusion"] == "success"
                for c in checks.values()), "Project changes check missing")
    if any(f["filename"].startswith("contracts/") for f in snapshot["files"]):
        require(any(c["name"] == "contracts-guard" and c["conclusion"] == "success"
                    for c in checks.values()), "Contracts guard missing")
    return head, base


def attestation(snapshot, number):
    """Only a bot comment tied to a trusted main-branch mention run qualifies."""
    candidates = []
    for comment in snapshot["comments"]:
        if comment["user"]["login"] != "claude[bot]" or comment["user"]["type"] != "Bot":
            continue
        for match in MARKER.finditer(comment["body"]):
            try:
                record = json.loads(match.group(1))
            except ValueError:
                continue
            if (record.get("repo"), record.get("pr"), record.get("head"),
                record.get("base")) == (
                    REPO, number, snapshot["head"], snapshot["base"]):
                candidates.append((comment, record))
    require(candidates, "No exact-head/base trusted review attestation")
    comment, record = max(candidates, key=lambda pair:
                          (pair[0]["updated_at"], pair[0]["created_at"],
                           pair[0].get("id", 0)))
    require(record.get("verdict") == "pass", "Latest exact-head/base review is not passing")
    record = {k: v for k, v in record.items() if k != "_verified_legacy_bootstrap"}
    require(record.get("findings") == 0 and record.get("independent") is True,
            "Review has findings or is not independent")
    tests = record.get("tests")
    require(isinstance(tests, list) and tests and
            all(isinstance(t, dict) and t.get("command") and t.get("result") == "pass"
                and t.get("evidence") and t.get("source") == "executed"
                for t in tests), "Independently executed passing tests required")
    require(record.get("manual_actions") == [] and record.get("missing_evidence") == [],
            "Manual action or missing physical/organizer evidence")
    require(all(c.get("author", {}).get("login") != "claude[bot]" and
                c.get("committer", {}).get("login") != "claude[bot]"
                for c in snapshot["commits"]), "Reviewer authored PR commits")
    run_id = record.get("run_id")
    require(type(run_id) is int and run_id > 0, "Missing workflow run identity")
    run = api(f"repos/{REPO}/actions/runs/{run_id}")
    require(run["path"] == WORKFLOW and run["event"] == "issue_comment" and
            run["head_sha"] == snapshot["base"] and run["head_branch"] == "main" and
            run["head_repository"]["full_name"] == REPO and
            run["status"] == "completed" and run["conclusion"] == "success" and
            run["actor"]["login"] in TEAM, "Untrusted review workflow provenance")
    require(run["created_at"] <= comment["created_at"] <= run["updated_at"],
            "Review comment not created during attested run")
    require(comment["created_at"] <= comment["updated_at"] <= run["updated_at"],
            "Review edited after completed run")
    jobs = pages(f"repos/{REPO}/actions/runs/{run_id}/jobs", "jobs")
    mention = [j for j in jobs if j["name"] == "mention" and j["conclusion"] == "success"]
    require(len(mention) == 1, "Missing unique successful mention job")
    bind_comment(run_id, mention[0]["id"], comment["id"])
    content = api(f"repos/{REPO}/contents/{WORKFLOW}?ref={snapshot['base']}")
    legacy = (number == BOOTSTRAP_PR and
              content["sha"] == LEGACY_WORKFLOW_BLOB and
              record.get("bootstrap") is True)
    if legacy:
        # The existing main reviewer predates context artifacts. Its independently
        # authored verdict must explicitly bind the exact review-only request.
        require(snapshot["pr"]["user"]["login"] == "moterodiaz",
                "Bootstrap PR author changed")
        context = record
    else:
        artifacts = pages(f"repos/{REPO}/actions/runs/{run_id}/artifacts", "artifacts")
        contexts = [a for a in artifacts if a["name"] == "review-context" and
                    not a["expired"] and a["workflow_run"]["id"] == run_id]
        require(len(contexts) == 1, "Missing unique trusted review-context artifact")
        downloaded = subprocess.run(
            ["gh", "api", f"repos/{REPO}/actions/artifacts/{contexts[0]['id']}/zip"],
            capture_output=True, timeout=90)
        require(downloaded.returncode == 0, "Cannot read trusted review context")
        with zipfile.ZipFile(io.BytesIO(downloaded.stdout)) as archive:
            require(archive.namelist() == ["review-context.json"],
                    "Unexpected review context archive")
            context = json.loads(archive.read("review-context.json"))
    require(all(context.get(k) == record.get(k) for k in
                ("repo", "pr", "head", "base", "run_id")), "Review context mismatch")
    request = api(f"repos/{REPO}/issues/comments/{int(context['request_comment'])}")
    require(request["user"]["login"] in TEAM and
            request["user"]["login"] == run["actor"]["login"] and
            request["issue_url"] == f"https://api.github.com/repos/{REPO}/issues/{number}" and
            "@claude" in request["body"] and
            request["created_at"] <= run["created_at"], "Untrusted review request")
    if legacy:
        require(snapshot["head"] in request["body"] and
                snapshot["base"] in request["body"] and
                request["created_at"] == request["updated_at"] and
                "review-only" in request["body"].lower(),
                "Bootstrap request must pin head/base and remain unedited")
        record = {**record, "_verified_legacy_bootstrap": True}
    # The exact workflow must equal the trusted policy checkout, not PR content.
    trusted = Path(__file__).resolve().parents[1] / "workflows" / "claude-review.yml"
    require(legacy or base64.b64decode(content["content"]) == trusted.read_bytes(),
            "Review workflow differs from trusted policy checkout")
    for test in tests:
        verify_test_run(test, snapshot, number)
    return record


def verify_test_run(test, snapshot, number):
    """A copied author's result cannot masquerade as independent CI execution."""
    run_id = test.get("run_id")
    require(type(run_id) is int and run_id > 0, "Test run identity required")
    run = api(f"repos/{REPO}/actions/runs/{run_id}")
    require(run["path"] == ".github/workflows/ci.yml" and
            run["event"] == "pull_request" and
            run["head_sha"] == snapshot["head"] and
            run["head_repository"]["full_name"] == REPO and
            run["status"] == "completed" and run["conclusion"] == "success" and
            any(p["number"] == number and p["base"]["sha"] == snapshot["base"]
                for p in run["pull_requests"]), "Stale or untrusted test run")
    jobs = pages(f"repos/{REPO}/actions/runs/{run_id}/jobs", "jobs")
    require(any(j["name"] == "changes" and j["conclusion"] == "success" and
                any(s["name"] == "Isolated Python tests" and s["conclusion"] == "success"
                    for s in j["steps"]) for j in jobs),
            "Independent sandbox test step did not pass")
    content = api(f"repos/{REPO}/contents/.github/workflows/ci.yml?ref={snapshot['head']}")
    trusted = Path(__file__).resolve().parents[1] / "workflows" / "ci.yml"
    require(base64.b64decode(content["content"]) == trusted.read_bytes(),
            "Test workflow differs from independently reviewed policy")


def bind_comment(run_id, job_id, comment_id):
    """Bind to the action-created tracking comment, not model-chosen run IDs.

    The trusted action emits this line before starting the model. A model that
    echoes another such line only creates ambiguity, which fails closed.
    """
    log = command(["gh", "run", "view", str(run_id), "--repo", REPO,
                   "--job", str(job_id), "--log"])
    ids = re.findall(
        r"^mention\tRun anthropics/claude-code-action@v1\t[^\n]*"
        r"Created initial comment with ID: ([0-9]+)\s*$", log, re.M)
    require(ids == [str(comment_id)], "Review comment is not bound to the actual run")


def journal(path, event):
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(event, sort_keys=True) + "\n")
        stream.flush()
        os.fsync(stream.fileno())


def merge(number, execute, journal_path, reconcile=False):
    journal_path.parent.mkdir(parents=True, exist_ok=True)
    with journal_path.with_suffix(".lock").open("a") as lock, \
            (journal_path.parent / "state.lock").open("a") as state_lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        fcntl.flock(state_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        prior = [json.loads(line) for line in journal_path.read_text().splitlines()] \
            if journal_path.exists() else []
        pending = [e for e in prior if e["event"] == "intent" and e["pr"] == number and
                   not any(x.get("key") == e["key"] and x["event"] in {"merged", "not-merged"}
                           for x in prior)]
        if pending:
            p = api(f"repos/{REPO}/pulls/{number}")
            for e in pending:
                if p["merged"] and p["head"]["sha"] == e["head"]:
                    journal(journal_path, {**e, "event": "merged",
                                          "merge_sha": p["merge_commit_sha"]})
                elif reconcile and p["state"] == "open" and (
                        (e.get("phase") == "prepared" and not any(
                            x["event"] == "attempt" and x.get("key") == e["key"]
                            for x in prior)) or p["head"]["sha"] != e["head"]):
                    # Exclusive local locks prove no writer remains. No HTTP
                    # attempt was made, or expected-head now excludes the old one.
                    journal(journal_path, {**e, "event": "not-merged",
                                          "observed_head": p["head"]["sha"]})
                else:
                    raise RuntimeError("Uncertain previous merge; reconcile manually, do not retry")
            return {"result": "reconciled"}
        require(not reconcile, "No pending intent to reconcile")
        snap = collect(number)
        state_path = journal_path.parent / "state.json"
        require(state_path.exists(), "Session state is required")
        state = json.loads(state_path.read_text())
        entry = state.get("prs", {}).get(str(number), {})
        require(not entry.get("paused") and entry.get("failed_rounds", 0) < 3 and
                not entry.get("active_round"), "Session pause or unfinished correction round")
        head, base = validate(snap, number)
        review = attestation(snap, number)
        fingerprint = hashlib.sha256(json.dumps(
            snap, sort_keys=True).encode()).hexdigest()
        if not execute:
            return {"result": "dry-run-pass", "head": head, "base": base,
                    "review_run": review["run_id"]}
        root = Path(__file__).resolve().parents[2]
        checkout = command(["git", "-C", str(root), "rev-parse", "HEAD"]).strip()
        require(checkout == base or
                (number == BOOTSTRAP_PR and
                 review.get("_verified_legacy_bootstrap") is True and checkout == head),
                "Execute only from trusted main or the independently reviewed bootstrap head")
        require(not command(["git", "-C", str(root), "status", "--porcelain"]).strip(),
                "Trusted policy checkout is dirty")
        # Re-read all evidence, including identity, head AND base just before intent.
        fresh = collect(number)
        validate(fresh, number)
        attestation(fresh, number)
        require(hashlib.sha256(json.dumps(fresh, sort_keys=True).encode()).hexdigest()
                == fingerprint, "Evidence changed; rerun dry-run")
        require(not any(e.get("pr") == number and e.get("head") == head and
                        e["event"] == "merged" for e in prior),
                "Merge already recorded")
        key = f"{number}:{head}:{base}:{uuid.uuid4().hex}"
        require(not any(e.get("key") == key for e in prior), "Attempt ID collision")
        intent = {"event": "intent", "key": key, "pr": number, "head": head,
                  "base": base, "phase": "prepared"}
        journal(journal_path, intent)
        # GitHub atomically guards the head. GitHub has no base-SHA merge precondition;
        # the final base read is conservative, not an atomic base lock.
        if api(f"repos/{REPO}/git/ref/heads/main")["object"]["sha"] != base:
            journal(journal_path, {**intent, "event": "not-merged",
                                  "reason": "Base moved before any merge request"})
            raise RuntimeError("Base moved before merge; no merge request sent")
        journal(journal_path, {**intent, "event": "attempt"})
        result = api(f"repos/{REPO}/pulls/{number}/merge", "PUT",
                     {"sha": head, "merge_method": "squash"})
        require(result.get("merged") is True, "Merge not confirmed; reconcile before retry")
        p = api(f"repos/{REPO}/pulls/{number}")
        require(p["merged"] and p["head"]["sha"] == head and
                p["merge_commit_sha"] == result["sha"], "Merge readback uncertain")
        journal(journal_path, {**intent, "event": "merged", "merge_sha": result["sha"]})
        return {"result": "merged", "sha": result["sha"]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pr", type=int)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--reconcile", action="store_true",
                        help="Reconcile a pending intent without sending a merge request")
    parser.add_argument("--journal", type=Path, default=Path.home() /
                        ".factory/pr-review-correction/moterodiaz--DroneHacks/merge-journal.jsonl")
    args = parser.parse_args()
    require(args.pr > 0 and args.journal.is_absolute(), "Positive PR and absolute journal required")
    require(not (args.execute and args.reconcile), "Reconcile never executes a merge")
    print(json.dumps(merge(args.pr, args.execute, args.journal, args.reconcile)))


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, ValueError, KeyError, TypeError, OSError,
            subprocess.TimeoutExpired, zipfile.BadZipFile) as error:
        print(f"STOP: {error}", file=sys.stderr)
        sys.exit(1)
