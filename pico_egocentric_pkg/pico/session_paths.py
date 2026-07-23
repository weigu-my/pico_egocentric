#!/usr/bin/env python3
"""Pico session 目录解析工具。"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Iterable

_DEFAULT_ROOTS_TEXT = os.environ.get(
    "PICO_EGOCENTRIC_DATA_ROOTS",
    "/anyverse/data/egocentric_data",
)

DEFAULT_SESSION_ROOTS = tuple(
    Path(item).expanduser()
    for item in _DEFAULT_ROOTS_TEXT.split(":")
    if item
)

DEFAULT_ANYVERSE_ROOT_MAP = (
    (
        Path(os.environ.get("PICO_HOST_ANYVERSE_ROOT", "~/anyverse")).expanduser(),
        Path(os.environ.get("PICO_CONTAINER_ANYVERSE_ROOT", "/anyverse")).expanduser(),
    ),
    (
        Path(os.environ.get("PICO_CONTAINER_ANYVERSE_ROOT", "/anyverse")).expanduser(),
        Path(os.environ.get("PICO_HOST_ANYVERSE_ROOT", "~/anyverse")).expanduser(),
    ),
)

EGOCENTRIC_DATA_DIRNAME = "egocentric_data"


def _has_poses(path: Path) -> bool:
    return (path / "poses.jsonl").is_file()


def _map_anyverse_path(path: Path, anyverse_roots: Iterable[tuple[Path, Path]]) -> list[Path]:
    mapped: list[Path] = []
    path_str = path.as_posix()
    for source_root, target_root in anyverse_roots:
        source_str = source_root.as_posix().rstrip("/")
        if path_str == source_str:
            mapped.append(target_root)
        elif path_str.startswith(source_str + "/"):
            mapped.append(target_root / path.relative_to(source_root))
    return mapped


def candidate_session_dirs(
    session_dir: Path | str,
    roots: Iterable[Path] | None = None,
    anyverse_roots: Iterable[tuple[Path, Path]] | None = None,
) -> list[Path]:
    """返回可能的 session 目录候选。"""

    raw = Path(session_dir).expanduser()
    session_roots = tuple(DEFAULT_SESSION_ROOTS if roots is None else roots)
    root_map = tuple(DEFAULT_ANYVERSE_ROOT_MAP if anyverse_roots is None else anyverse_roots)

    candidates: list[Path] = [raw]
    candidates.extend(_map_anyverse_path(raw, root_map))

    if not raw.is_absolute():
        for root in session_roots:
            candidates.append(root / raw)
    elif EGOCENTRIC_DATA_DIRNAME in raw.parts and raw.name:
        # 兼容旧路径，例如 /host/output/egocentric_data/<session>。
        for root in session_roots:
            candidates.append(root / raw.name)

    deduped: list[Path] = []
    seen: set[str] = set()
    for candidate in candidates:
        key = candidate.as_posix()
        if key not in seen:
            seen.add(key)
            deduped.append(candidate)
    return deduped


def resolve_session_dir(
    session_dir: Path | str,
    roots: Iterable[Path] | None = None,
    anyverse_roots: Iterable[tuple[Path, Path]] | None = None,
) -> Path:
    """解析 session 目录，支持完整路径或 egocentric_data 下的 session 名。"""

    candidates = candidate_session_dirs(session_dir, roots=roots, anyverse_roots=anyverse_roots)
    for candidate in candidates:
        resolved = candidate.resolve()
        if _has_poses(resolved):
            return resolved

    checked = "\n".join(f"- {candidate}" for candidate in candidates)
    raise FileNotFoundError(f"poses.jsonl not found for session {session_dir!r}. Checked:\n{checked}")
