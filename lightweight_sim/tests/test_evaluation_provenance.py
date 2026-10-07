"""Keep diagnostic archives usable across checkout and installed environments."""

import hashlib
import subprocess
import zipfile

import pytest

from lightweight_sim.engine.analysis import evaluation


@pytest.mark.parametrize("worktree", [False, True])
def test_provenance_runs_git_at_checkout_root(tmp_path, monkeypatch, worktree):
    checkout = tmp_path / "checkout"
    root = checkout / "lightweight_sim"
    root.mkdir(parents=True)
    if worktree:
        (checkout / ".git").write_text("gitdir: /external/worktree")
    else:
        (checkout / ".git").mkdir()
    monkeypatch.setattr(evaluation, "ROOT", root)
    calls = []

    def git(command, **kwargs):
        calls.append(command)
        assert kwargs["cwd"] == checkout
        return "abc123\n" if command[1] == "rev-parse" else ""

    monkeypatch.setattr(evaluation.subprocess, "check_output", git)
    metadata = evaluation.provenance()
    assert metadata["git_commit"] == "abc123"
    assert metadata["working_tree_status"] == ""
    assert metadata["git_errors"] == {}
    assert len(calls) == 2


@pytest.mark.parametrize("error", [
    subprocess.CalledProcessError(128, ["git"], stderr="not a git repository"),
    FileNotFoundError("git executable missing"),
    subprocess.TimeoutExpired(["git"], 5),
])
def test_archive_preserves_sources_when_git_unavailable(tmp_path, monkeypatch, error):
    root = tmp_path / "package"
    (root / "engine").mkdir(parents=True)
    source = b"# archived source\n"
    (root / "engine" / "example.py").write_bytes(source)
    monkeypatch.setattr(evaluation, "ROOT", root)

    def unavailable(*args, **kwargs):
        raise error

    monkeypatch.setattr(evaluation.subprocess, "check_output", unavailable)
    metadata = evaluation.provenance()
    assert metadata["git_commit"] is None
    assert metadata["working_tree_status"] is None
    assert set(metadata["git_errors"]) == {"rev-parse HEAD", "status --short"}
    assert metadata["source_sha256"]["engine/example.py"] == hashlib.sha256(source).hexdigest()
    evaluation.archive_sources(metadata, tmp_path / "archive")
    with zipfile.ZipFile(tmp_path / "archive" / metadata["source_bundle"]) as bundle:
        assert bundle.read("engine/example.py") == source
