"""All GitHub writes are mocked. These tests never merge a real PR."""
import base64
import copy
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import zipfile

PATH = Path(__file__).resolve().parents[1] / ".github/scripts/merge_reviewed.py"
spec = importlib.util.spec_from_file_location("merge_reviewed", PATH)
gate = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gate)
HEAD, BASE = "a" * 40, "b" * 40


def snapshot():
    return {
        "actor": "moterodiaz", "head": HEAD, "base": BASE, "base_at_start": BASE,
        "pr": {"user": {"login": "jsun4231"}, "head": {"sha": HEAD,
               "repo": {"full_name": gate.REPO}}, "base": {"ref": "main",
               "repo": {"full_name": gate.REPO}}, "state": "open", "draft": False,
               "locked": False, "labels": [{"name": "review-passed"}],
               "mergeable": True, "mergeable_state": "clean", "body": ""},
        "comments": [], "reviews": [], "threads": [], "issues": [],
        "checks": [{"name": "changes", "id": 1, "app": {"id": 1},
                    "head_sha": HEAD, "status": "completed", "conclusion": "success"}],
        "statuses": [], "rules": [], "files": [], "commits": [],
    }


class GateTests(unittest.TestCase):
    def setUp(self):
        self.binding_log = (
            "mention\tRun anthropics/claude-code-action@v1\t"
            "2026-11-07T01:00:00Z Created initial comment with ID: 7\n")
        mock = patch.object(gate, "command", return_value=self.binding_log)
        self.command = mock.start()
        self.addCleanup(mock.stop)

    def test_valid_team_pr(self):
        self.assertEqual(gate.validate(snapshot(), 42), (HEAD, BASE))

    def test_comment_does_not_clear_change_request(self):
        s = snapshot()
        s["reviews"] = [{"user": {"login": "reviewer"}, "state": state}
                        for state in ["CHANGES_REQUESTED", "COMMENTED"]]
        with self.assertRaises(RuntimeError):
            gate.validate(s, 42)
        s["reviews"].append({"user": {"login": "reviewer"}, "state": "APPROVED"})
        self.assertEqual(gate.validate(s, 42), (HEAD, BASE))

    def test_all_disqualifying_states(self):
        mutations = [
            lambda s: s.update(actor="outsider"),
            lambda s: s["pr"]["user"].update(login="outsider"),
            lambda s: s["pr"]["head"].update(repo={"full_name": "fork/repo"}),
            lambda s: s["pr"].update(draft=True),
            lambda s: s["pr"].update(locked=True),
            lambda s: s["pr"].update(state="closed"),
            lambda s: s["pr"].update(mergeable=None),
            lambda s: s.update(base="c" * 40),
            lambda s: s.update(threads=[{"isResolved": False}]),
            lambda s: s.update(reviews=[{"user": {"login": "reviewer"},
                                         "state": "CHANGES_REQUESTED"}]),
            lambda s: s["checks"][0].update(status="in_progress"),
            lambda s: s["checks"][0].update(conclusion="failure"),
            lambda s: s["checks"][0].update(head_sha="d" * 40),
            lambda s: s.update(checks=[]),
            lambda s: s.update(statuses=[{"context": "test", "state": "pending"}]),
            lambda s: s.update(rules=[{"type": "required_status_checks",
                "parameters": {"required_status_checks": [{"context": "absent"}]}}]),
            lambda s: s.update(files=[{"filename": "contracts/course.md"}]),
        ]
        for mutate in mutations:
            with self.subTest(mutation=mutate):
                s = snapshot()
                mutate(s)
                with self.assertRaises(RuntimeError):
                    gate.validate(s, 42)
        for label in gate.BLOCKERS | {"needs-review", "changes-requested"}:
            s = snapshot()
            s["pr"]["labels"].append({"name": label})
            with self.subTest(label=label), self.assertRaises(RuntimeError):
                gate.validate(s, 42)

    def test_linked_open_findings_and_manual_holds(self):
        for label in ["decision", "review-finding", "needs-human", "blocked"]:
            s = snapshot()
            s["issues"] = [{"number": 12, "title": "PR #42: defect", "body": "",
                            "labels": [{"name": label}]}]
            with self.subTest(label=label), self.assertRaises(RuntimeError):
                gate.validate(s, 42)

    def test_historical_issue_mentions_are_not_blockers(self):
        s = snapshot()
        s["pr"]["body"] = "Existing technical holds #23 remain unchanged, unrelated to this policy."
        s["issues"] = [{"number": 23, "title": "PR #22: safety clearance",
                        "body": "", "labels": [{"name": "needs-human"}]}]
        self.assertEqual(gate.validate(s, 42), (HEAD, BASE))
        for body in ["Blocked by #23", "Closes #23"]:
            s["pr"]["body"] = body
            with self.subTest(body=body), self.assertRaises(RuntimeError):
                gate.validate(s, 42)

    def test_missing_and_spoofed_review(self):
        for login, kind in [("claude", "User"), ("attacker", "Bot")]:
            s = snapshot()
            s["comments"] = [{"user": {"login": login, "type": kind},
                              "body": "<!-- dronehacks-review-v1 {} -->"}]
            with self.assertRaises(RuntimeError):
                gate.attestation(s, 42)

    def trusted(self):
        s = snapshot()
        record = {"repo": gate.REPO, "pr": 42, "head": HEAD, "base": BASE,
                  "run_id": 10, "verdict": "pass", "independent": True, "findings": 0,
                  "tests": [{"command": "test", "result": "pass", "evidence": "1 passed",
                             "source": "executed", "run_id": 20}],
                  "manual_actions": [], "missing_evidence": []}
        s["comments"] = [{"id": 7, "user": {"login": "claude[bot]", "type": "Bot"},
            "body": "<!-- dronehacks-review-v1 " + json.dumps(record) + " -->",
            "created_at": "2026-11-07T01:00:01Z", "updated_at": "2026-11-07T01:00:01Z"}]
        run = {"path": gate.WORKFLOW, "event": "issue_comment", "head_sha": BASE,
               "head_branch": "main", "head_repository": {"full_name": gate.REPO},
               "status": "completed", "conclusion": "success",
               "actor": {"login": "moterodiaz"}, "created_at": "2026-11-07T01:00:00Z",
               "updated_at": "2026-11-07T01:00:02Z"}
        context = {**record, "request_comment": 20}
        blob = io.BytesIO()
        with zipfile.ZipFile(blob, "w") as archive:
            archive.writestr("review-context.json", json.dumps(context))
        def api(path, *args):
            if path.endswith("/runs/10"):
                return run
            if path.endswith("/runs/20"):
                return {"path": ".github/workflows/ci.yml", "event": "pull_request",
                        "head_sha": HEAD, "head_repository": {"full_name": gate.REPO},
                        "status": "completed", "conclusion": "success",
                        "pull_requests": [{"number": n, "base": {"sha": BASE}}
                                          for n in [31, 42]]}
            if "/issues/comments/" in path:
                return {"user": {"login": "moterodiaz"}, "body": "@claude review",
                        "issue_url": f"https://api.github.com/repos/{gate.REPO}/issues/42",
                        "created_at": "2026-11-07T00:59:59Z"}
            if "/contents/" in path:
                workflow = "ci.yml" if "/ci.yml?" in path else "claude-review.yml"
                return {"sha": "new-workflow", "content": base64.b64encode(
                    (PATH.parents[1] / "workflows" / workflow).read_bytes()).decode()}
            raise AssertionError(path)
        def pages(path, field):
            if field == "jobs":
                if "/runs/20/" in path:
                    return [{"name": "changes", "conclusion": "success",
                             "steps": [{"name": "Isolated Python tests",
                                        "conclusion": "success"}]}]
                return [{"id": 70, "name": "mention", "conclusion": "success"}]
            return [{"name": "review-context", "expired": False, "id": 1,
                     "workflow_run": {"id": 10}}]
        return s, record, run, api, pages, blob.getvalue()

    def test_legacy_bootstrap_is_bound_to_one_pr_and_trusted_workflow(self):
        s, record, run, api, pages, blob = self.trusted()
        s["pr"]["user"]["login"] = "moterodiaz"
        record.update(pr=31, bootstrap=True, request_comment=20)
        s["comments"][0]["body"] = "<!-- dronehacks-review-v1 " + json.dumps(record) + " -->"
        def legacy_api(path, *args):
            if "/contents/.github/workflows/claude-review.yml" in path:
                return {"sha": gate.LEGACY_WORKFLOW_BLOB}
            if "/issues/comments/" in path:
                return {"user": {"login": "moterodiaz"},
                        "issue_url": f"https://api.github.com/repos/{gate.REPO}/issues/31",
                        "body": f"@claude review-only {HEAD} {BASE}",
                        "created_at": "2026-11-07T00:59:59Z",
                        "updated_at": "2026-11-07T00:59:59Z"}
            return api(path, *args)
        with patch.object(gate, "api", side_effect=legacy_api), \
             patch.object(gate, "pages", side_effect=pages):
            self.assertTrue(gate.attestation(s, 31)["_verified_legacy_bootstrap"])
            # No matching verdict for another PR, even with bootstrap requested.
            with self.assertRaises(RuntimeError):
                gate.attestation(s, 32)
            s["pr"]["user"]["login"] = "jsun4231"
            with self.assertRaises(RuntimeError):
                gate.attestation(s, 31)

    def test_valid_trusted_review_and_untrusted_provenance(self):
        s, record, run, api, pages, blob = self.trusted()
        with patch.object(gate, "api", side_effect=api), \
             patch.object(gate, "pages", side_effect=pages), \
             patch.object(gate.subprocess, "run") as execute:
            execute.return_value.returncode = 0
            execute.return_value.stdout = blob
            self.assertEqual(gate.attestation(s, 42), record)
            for event in ["pull_request", "pull_request_review_comment"]:
                run["event"] = event
                with self.assertRaises(RuntimeError):
                    gate.attestation(s, 42)

    def test_review_rejects_stale_findings_missing_tests_and_evidence(self):
        for key, value in [("head", "c" * 40), ("base", "c" * 40),
                           ("findings", 1), ("independent", False), ("tests", []),
                           ("manual_actions", ["deploy"]), ("missing_evidence", ["hover"])]:
            s, record, *_ = self.trusted()
            record[key] = value
            s["comments"][0]["body"] = "<!-- dronehacks-review-v1 " + json.dumps(record) + " -->"
            with self.subTest(field=key), self.assertRaises(RuntimeError):
                gate.attestation(s, 42)

    def test_later_nonpass_verdict_revokes_pass(self):
        s, record, run, api, pages, blob = self.trusted()
        later = copy.deepcopy(s["comments"][0])
        record["verdict"] = "blocked"
        later["body"] = "<!-- dronehacks-review-v1 " + json.dumps(record) + " -->"
        later["created_at"] = later["updated_at"] = "2026-11-07T01:00:02Z"
        s["comments"].append(later)
        with patch.object(gate, "api", side_effect=api), \
             patch.object(gate, "pages", side_effect=pages), \
             patch.object(gate.subprocess, "run") as execute:
            execute.return_value.returncode = 0
            execute.return_value.stdout = blob
            with self.assertRaises(RuntimeError):
                gate.attestation(s, 42)

    def test_pagination(self):
        with patch.object(gate, "api", side_effect=[[{}] * 100, [{"last": True}]]) as read:
            self.assertEqual(len(gate.pages("test")), 101)
            self.assertIn("page=2", read.call_args.args[0])

    def test_run_binding_rejects_spoofed_and_ambiguous_comments(self):
        gate.bind_comment(10, 70, 7)
        self.assertIn("--job", self.command.call_args.args[0])
        with self.assertRaises(RuntimeError):
            gate.bind_comment(10, 70, 8)
        self.command.return_value = self.binding_log * 2
        with self.assertRaises(RuntimeError):
            gate.bind_comment(10, 70, 7)
        self.command.return_value = ""
        with self.assertRaises(RuntimeError):
            gate.bind_comment(10, 70, 7)

    def test_copied_author_test_results_do_not_qualify(self):
        s, record, *_ = self.trusted()
        record["tests"][0]["source"] = "author-reported"
        s["comments"][0]["body"] = "<!-- dronehacks-review-v1 " + json.dumps(record) + " -->"
        with self.assertRaises(RuntimeError):
            gate.attestation(s, 42)

    def test_test_run_is_checked_independently(self):
        s, record, run, api, pages, _ = self.trusted()
        def stale_api(path, *args):
            result = api(path, *args)
            if path.endswith("/runs/20"):
                result["head_sha"] = "c" * 40
            return result
        with patch.object(gate, "api", side_effect=stale_api), \
             patch.object(gate, "pages", side_effect=pages):
            with self.assertRaises(RuntimeError):
                gate.verify_test_run(record["tests"][0], s, 42)

    def test_merge_paths_are_mocked(self):
        for changed, execute in [(False, False), (False, True), (True, True)]:
            with self.subTest(changed=changed, execute=execute), tempfile.TemporaryDirectory() as d:
                path = Path(d) / "merge.jsonl"
                (Path(d) / "state.json").write_text('{"prs":{}}')
                first, second = snapshot(), snapshot()
                if changed:
                    second["pr"]["body"] = "changed while collecting"
                def api(endpoint, method="GET", data=None):
                    if method == "PUT":
                        self.assertEqual(data, {"sha": HEAD, "merge_method": "squash"})
                        return {"merged": True, "sha": "e" * 40}
                    if "/git/ref/" in endpoint:
                        return {"object": {"sha": BASE}}
                    return {"merged": True, "head": {"sha": HEAD},
                            "merge_commit_sha": "e" * 40}
                with patch.object(gate, "collect", side_effect=[first, second]), \
                     patch.object(gate, "attestation", return_value={"run_id": 10}), \
                     patch.object(gate, "command", side_effect=[BASE, ""]), \
                     patch.object(gate, "api", side_effect=api) as remote:
                    if changed:
                        with self.assertRaises(RuntimeError):
                            gate.merge(42, execute, path)
                        self.assertFalse(path.exists())
                    else:
                        result = gate.merge(42, execute, path)
                        self.assertEqual(result["result"], "merged" if execute else "dry-run-pass")
                        self.assertEqual(sum(c.args[1:2] == ("PUT",)
                                             for c in remote.call_args_list), int(execute))

    def test_uncertain_write_never_retried(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "merge.jsonl"
            gate.journal(path, {"event": "intent", "key": "42:head:base",
                               "pr": 42, "head": HEAD, "base": BASE})
            with patch.object(gate, "api", return_value={"merged": False}) as remote:
                with self.assertRaises(RuntimeError):
                    gate.merge(42, True, path)
                self.assertEqual(remote.call_count, 1)
                self.assertEqual(remote.call_args.args,
                                 (f"repos/{gate.REPO}/pulls/42",))

    def test_reconcile_only_provably_unsubmitted_or_stale_head(self):
        for attempted, changed_head in [(False, False), (True, True), (True, False)]:
            with self.subTest(attempted=attempted, changed=changed_head), \
                    tempfile.TemporaryDirectory() as d:
                path = Path(d) / "merge.jsonl"
                event = {"event": "intent", "key": "42:head:base", "pr": 42,
                         "head": HEAD, "base": BASE, "phase": "prepared"}
                gate.journal(path, event)
                if attempted:
                    gate.journal(path, {**event, "event": "attempt"})
                pr = {"state": "open", "merged": False,
                      "head": {"sha": "c" * 40 if changed_head else HEAD}}
                with patch.object(gate, "api", return_value=pr) as remote:
                    if attempted and not changed_head:
                        with self.assertRaises(RuntimeError):
                            gate.merge(42, False, path, reconcile=True)
                    else:
                        self.assertEqual(gate.merge(42, False, path, reconcile=True),
                                         {"result": "reconciled"})
                        self.assertEqual(json.loads(path.read_text().splitlines()[-1])
                                         ["event"], "not-merged")
                    self.assertEqual(remote.call_count, 1)


if __name__ == "__main__":
    unittest.main()
