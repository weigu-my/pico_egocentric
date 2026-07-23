#!/usr/bin/env python3
"""Tests for Pico session path resolution."""

import os
import sys
import tempfile
from pathlib import Path

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(SCRIPT_DIR, ".."))

from pico_egocentric_pkg.pico.session_paths import resolve_session_dir  # noqa: E402


def _make_session(root: Path, name: str) -> Path:
    session = root / name
    session.mkdir(parents=True)
    (session / "poses.jsonl").write_text("", encoding="utf-8")
    return session


def test_resolve_session_dir_accepts_existing_path():
    with tempfile.TemporaryDirectory() as tmpdir:
        expected = _make_session(Path(tmpdir), "20260703_120000")

        resolved = resolve_session_dir(expected)

    assert resolved == expected.resolve()


def test_resolve_session_dir_finds_session_name_under_data_root():
    with tempfile.TemporaryDirectory() as tmpdir:
        root = Path(tmpdir) / "egocentric_data"
        expected = _make_session(root, "20260703_120000")

        resolved = resolve_session_dir("20260703_120000", roots=[root])

    assert resolved == expected.resolve()


def test_resolve_session_dir_maps_host_anyverse_path_to_container_path():
    with tempfile.TemporaryDirectory() as tmpdir:
        container_root = Path(tmpdir) / "anyverse"
        expected = _make_session(container_root / "data" / "egocentric_data", "20260703_120000")

        resolved = resolve_session_dir(
            "/host/anyverse/data/egocentric_data/20260703_120000",
            roots=[],
            anyverse_roots=[
                (Path("/host/anyverse"), container_root),
            ],
        )

    assert resolved == expected.resolve()


def test_resolve_session_dir_maps_legacy_output_path_to_data_root():
    with tempfile.TemporaryDirectory() as tmpdir:
        root = Path(tmpdir) / "egocentric_data"
        expected = _make_session(root, "20260703_120000")

        resolved = resolve_session_dir(
            "/host/output/egocentric_data/20260703_120000",
            roots=[root],
        )

    assert resolved == expected.resolve()


if __name__ == "__main__":
    tests = [
        test_resolve_session_dir_accepts_existing_path,
        test_resolve_session_dir_finds_session_name_under_data_root,
        test_resolve_session_dir_maps_host_anyverse_path_to_container_path,
        test_resolve_session_dir_maps_legacy_output_path_to_data_root,
    ]
    for test in tests:
        test()
        print(f"PASS: {test.__name__}")
