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
    def test_valid_team_pr(self):
        self.assertEqual(gate.validate(snapshot(), 42), (HEAD, BASE))

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
                  "tests": [{"command": "test", "result": "pass", "evidence": "1 passed"}],
                  "manual_actions": [], "missing_evidence": []}
        s["comments"] = [{"user": {"login": "claude[bot]", "type": "Bot"},
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
            if "/issues/comments/" in path:
                return {"user": {"login": "moterodiaz"}, "body": "@claude review",
                        "issue_url": f"https://api.github.com/repos/{gate.REPO}/issues/42",
                        "created_at": "2026-11-07T00:59:59Z"}
            if "/contents/" in path:
                return {"sha": "new-workflow", "content": base64.b64encode(
                    (PATH.parents[1] / "workflows/claude-review.yml").read_bytes()).decode()}
            raise AssertionError(path)
        def pages(path, field):
            if field == "jobs":
                return [{"name": "mention", "conclusion": "success"}]
            return [{"name": "review-context", "expired": False, "id": 1,
                     "workflow_run": {"id": 10}}]
        return s, record, run, api, pages, blob.getvalue()

    def test_legacy_bootstrap_is_bound_to_one_pr_and_trusted_workflow(self):
        s, record, run, api, pages, blob = self.trusted()
        s["pr"]["user"]["login"] = "moterodiaz"
        record.update(pr=31, bootstrap=True, request_comment=20)
        s["comments"][0]["body"] = "<!-- dronehacks-review-v1 " + json.dumps(record) + " -->"
        def legacy_api(path, *args):
            if "/contents/" in path:
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

    def test_pagination(self):
        with patch.object(gate, "api", side_effect=[[{}] * 100, [{"last": True}]]) as read:
            self.assertEqual(len(gate.pages("test")), 101)
            self.assertIn("page=2", read.call_args.args[0])

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


if __name__ == "__main__":
    unittest.main()
