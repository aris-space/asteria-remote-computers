#!/usr/bin/env python3
"""
Generate a JSON intermediate file to stdout for a Typst config validation document.

Usage:
    python gen_metadata.py <config_path> <base_config_path> <binary_commit>
"""

import sys
import difflib
import subprocess
import json
from pathlib import Path


def object_hash(path: Path) -> str:
    result = subprocess.run(
        ["git", "hash-object", str(path)],
        capture_output=True, text=True, check=True
    )
    return "H" + result.stdout.strip()[:5]


def git_log_oneline(ref: str, cwd: Path) -> str:
    try:
        result = subprocess.run(
            ["git", "log", "--oneline", "-1", ref],
            capture_output=True, text=True, check=True, cwd=cwd
        )
        parts = result.stdout.strip().split(" ", 1)
        return parts[1] if len(parts) > 1 else ""
    except subprocess.CalledProcessError:
        return "unknown"


def git_branch(ref: str, cwd: Path) -> str:
    try:
        result = subprocess.run(
            ["git", "name-rev", "--name-only", ref],
            capture_output=True, text=True, check=True, cwd=cwd
        )
        return result.stdout.strip().replace("remotes/origin/", "")
    except subprocess.CalledProcessError:
        return "unknown"


def git_config_info(config_path: Path) -> dict:
    """Get last commit info for a tracked file."""
    try:
        result = subprocess.run(
            ["git", "log", "--oneline", "-1", "--", str(config_path.name)],
            capture_output=True, text=True, check=True, cwd=config_path.parent
        )
        line = result.stdout.strip()
        if not line:
            return {"commit": "unknown", "branch": "unknown", "last_change": "unknown"}
        commit_short, _, msg = line.partition(" ")
        branch = git_branch(commit_short, config_path.parent)
        return {"commit": commit_short, "branch": branch, "last_change": msg}
    except subprocess.CalledProcessError:
        return {"commit": "unknown", "branch": "unknown", "last_change": "unknown"}


def annotate_diff(config_lines: list[str], base_lines: list[str]) -> list[str]:
    matcher = difflib.SequenceMatcher(None, base_lines, config_lines, autojunk=False)
    result = []
    for op, i1, i2, j1, j2 in matcher.get_opcodes():
        if op == "equal":
            result.extend(line.rstrip("\n") for line in config_lines[j1:j2])
        elif op == "insert":
            for line in config_lines[j1:j2]:
                result.append(line.rstrip("\n") + " # +")
        elif op == "delete":
            for line in base_lines[i1:i2]:
                result.append(line.rstrip("\n") + " # -")
        elif op == "replace":
            for line in config_lines[j1:j2]:
                result.append(line.rstrip("\n") + " # +")
            for line in base_lines[i1:i2]:
                result.append(line.rstrip("\n") + " # -")
    return result


def main():
    if len(sys.argv) != 4:
        print(f"Usage: {sys.argv[0]} <config_path> <base_config_path> <binary_commit>")
        sys.exit(1)

    config_path = Path(sys.argv[1])
    base_config_path = Path(sys.argv[2])
    cwd = config_path.parent
    commit = subprocess.run(
        ["git", "rev-parse", sys.argv[3]],
        capture_output=True, text=True, check=True, cwd=cwd
    ).stdout.strip()

    for p in (config_path, base_config_path):
        if not p.exists():
            print(f"Error: {p} does not exist")
            sys.exit(1)

    config_lines = config_path.read_text().splitlines(keepends=True)
    base_lines = base_config_path.read_text().splitlines(keepends=True)
    annotated = annotate_diff(config_lines, base_lines)

    config_info = git_config_info(config_path)

    data = {
        "binary": {
            "commit": commit[:7],
            "branch": git_branch(commit, cwd),
            "last_change": git_log_oneline(commit, cwd),
        },
        "config": {
            "path": str(config_path),
            "base": str(base_config_path),
            "hash": object_hash(config_path),
            "branch": config_info["branch"],
            "last_change": config_info["last_change"],
        },
        "toml_annotated": "\n".join(annotated),
    }

    print(json.dumps(data, indent=2))


if __name__ == "__main__":
    main()