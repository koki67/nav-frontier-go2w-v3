from __future__ import annotations

import ast
from pathlib import Path


SRC_ROOT = Path(__file__).resolve().parents[2]
LOGGER_METHODS = {"debug", "info", "warning", "warn", "error", "fatal"}


def _package_python_files() -> list[Path]:
    package_dirs = [
        SRC_ROOT / "nav_frontier_go2w_bridge" / "nav_frontier_go2w_bridge",
        SRC_ROOT / "nav_frontier_go2w_frontier" / "nav_frontier_go2w_frontier",
        SRC_ROOT / "nav_frontier_go2w_planner" / "nav_frontier_go2w_planner",
    ]
    files: list[Path] = []
    for package_dir in package_dirs:
        files.extend(sorted(package_dir.glob("*.py")))
    return files


def test_rclpy_logger_calls_are_humble_compatible():
    offenders: list[str] = []

    for path in _package_python_files():
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            if not isinstance(node.func, ast.Attribute):
                continue
            if node.func.attr not in LOGGER_METHODS:
                continue
            if len(node.args) <= 1:
                continue

            offenders.append(f"{path.relative_to(SRC_ROOT)}:{node.lineno}")

    assert offenders == []
