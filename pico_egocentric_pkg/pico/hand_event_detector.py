#!/usr/bin/env python3
"""从 Pico 手部关节数据识别 Revo2 抓握/松开事件。"""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Literal, Protocol

import numpy as np

from .revo2_retarget_optimizer import FeatureExtractor, PicoHandCanonicalizer
from .session_paths import resolve_session_dir

HandEventName = Literal["grasp", "release"]

NUM_HAND_JOINTS = 26
DEFAULT_BOTTLE_GRASP_POSITIONS = [500, 750, 850, 900, 900, 900]
DEFAULT_RELEASE_POSITIONS = [0, 0, 0, 0, 0, 0]
DEFAULT_GRIP_FINGERS = ("index", "middle", "ring", "pinky")
DEFAULT_GRASP_THRESHOLD = 0.45
DEFAULT_RELEASE_THRESHOLD = 0.35
DEFAULT_HOLD_FRAMES = 3
DEFAULT_MIN_EVENT_GAP_NS = 500_000_000


class SupportsHandFrame(Protocol):
    fid: int
    timestamp_ns: int
    positions: np.ndarray
    quaternions: np.ndarray


@dataclass(frozen=True)
class HandEventConfig:
    """抓握事件识别参数。"""

    grasp_threshold: float = DEFAULT_GRASP_THRESHOLD
    release_threshold: float = DEFAULT_RELEASE_THRESHOLD
    hold_frames: int = DEFAULT_HOLD_FRAMES
    min_event_gap_ns: int = DEFAULT_MIN_EVENT_GAP_NS
    grip_fingers: tuple[str, ...] = DEFAULT_GRIP_FINGERS
    grasp_positions: tuple[int, ...] = tuple(DEFAULT_BOTTLE_GRASP_POSITIONS)
    release_positions: tuple[int, ...] = tuple(DEFAULT_RELEASE_POSITIONS)

    def __post_init__(self) -> None:
        if not 0.0 <= self.release_threshold < self.grasp_threshold <= 1.0:
            raise ValueError(
                "release_threshold and grasp_threshold must satisfy "
                "0 <= release < grasp <= 1"
            )
        if self.hold_frames <= 0:
            raise ValueError("hold_frames must be positive")
        if self.min_event_gap_ns < 0:
            raise ValueError("min_event_gap_ns must be non-negative")
        if not self.grip_fingers:
            raise ValueError("grip_fingers must not be empty")


@dataclass(frozen=True)
class HandEvent:
    """一次稳定识别到的手部事件。"""

    event: HandEventName
    fid: int
    timestamp_ns: int
    grip_score: float
    finger_curls: dict[str, float]
    revo2_positions: list[int]


@dataclass(frozen=True)
class HandFrame:
    """从 poses.jsonl 提取出的单帧右手数据。"""

    fid: int
    timestamp_ns: int
    positions: np.ndarray
    quaternions: np.ndarray


@dataclass(frozen=True)
class _StableWindowStart:
    """连续稳定窗口的起点，用于事件时间对齐。"""

    fid: int
    timestamp_ns: int
    grip_score: float
    finger_curls: dict[str, float]


def compute_finger_curls(
    positions: np.ndarray,
    quaternions: np.ndarray,
    side: str = "right",
) -> dict[str, float]:
    """计算 Pico 五指归一化弯曲程度，范围为 0..1。"""

    positions = np.asarray(positions, dtype=np.float64)
    quaternions = np.asarray(quaternions, dtype=np.float64)
    if positions.shape != (NUM_HAND_JOINTS, 3):
        raise ValueError(f"positions must have shape (26, 3), got {positions.shape}")
    if quaternions.shape != (NUM_HAND_JOINTS, 4):
        raise ValueError(f"quaternions must have shape (26, 4), got {quaternions.shape}")
    if not np.all(np.isfinite(positions)) or not np.all(np.isfinite(quaternions)):
        raise ValueError("positions and quaternions must be finite")

    canonicalizer = PicoHandCanonicalizer()
    extractor = FeatureExtractor()
    canonical = canonicalizer.canonicalize(positions, quaternions, side)
    return extractor.compute_pico_curls(canonical)


def compute_grip_score(
    finger_curls: dict[str, float],
    fingers: Iterable[str] = DEFAULT_GRIP_FINGERS,
) -> float:
    """按指定手指平均弯曲程度得到抓握分数。"""

    values = []
    for finger in fingers:
        if finger not in finger_curls:
            raise ValueError(f"missing finger curl: {finger}")
        values.append(float(np.clip(finger_curls[finger], 0.0, 1.0)))
    if not values:
        raise ValueError("fingers must not be empty")
    return float(np.mean(values))


def command_for_event(
    event: HandEventName,
    config: HandEventConfig | None = None,
) -> list[int]:
    """返回事件对应的 Revo2 六指位置命令。"""

    config = config or HandEventConfig()
    if event == "grasp":
        return list(config.grasp_positions)
    if event == "release":
        return list(config.release_positions)
    raise ValueError(f"unsupported hand event: {event!r}")


class HandEventDetector:
    """带滞回和连续帧确认的抓握事件检测器。"""

    def __init__(self, config: HandEventConfig | None = None):
        self.config = config or HandEventConfig()
        self.state: HandEventName = "release"
        self._grasp_count = 0
        self._release_count = 0
        self._last_event_ts: int | None = None
        self._grasp_start: _StableWindowStart | None = None
        self._release_start: _StableWindowStart | None = None

    def update(
        self,
        fid: int,
        timestamp_ns: int,
        positions: np.ndarray,
        quaternions: np.ndarray,
        side: str = "right",
    ) -> HandEvent | None:
        """输入一帧 Pico 关节数据，若状态稳定切换则返回事件。"""

        curls = compute_finger_curls(positions, quaternions, side=side)
        return self.update_curls(fid, timestamp_ns, curls)

    def update_curls(
        self,
        fid: int,
        timestamp_ns: int,
        finger_curls: dict[str, float],
    ) -> HandEvent | None:
        """输入已计算好的手指弯曲程度，便于测试和离线复用。"""

        score = compute_grip_score(finger_curls, self.config.grip_fingers)
        if score >= self.config.grasp_threshold:
            if self._grasp_count == 0:
                self._grasp_start = _StableWindowStart(
                    int(fid),
                    int(timestamp_ns),
                    float(score),
                    {key: float(value) for key, value in finger_curls.items()},
                )
            self._grasp_count += 1
            self._release_count = 0
            self._release_start = None
        elif score <= self.config.release_threshold:
            if self._release_count == 0:
                self._release_start = _StableWindowStart(
                    int(fid),
                    int(timestamp_ns),
                    float(score),
                    {key: float(value) for key, value in finger_curls.items()},
                )
            self._release_count += 1
            self._grasp_count = 0
            self._grasp_start = None
        else:
            # 中间区域保留当前状态，避免阈值附近抖动触发事件。
            self._grasp_count = 0
            self._release_count = 0
            self._grasp_start = None
            self._release_start = None

        if self.state == "release" and self._grasp_count >= self.config.hold_frames:
            return self._emit_if_allowed("grasp", timestamp_ns, self._grasp_start)
        if self.state == "grasp" and self._release_count >= self.config.hold_frames:
            return self._emit_if_allowed("release", timestamp_ns, self._release_start)
        return None

    def _emit_if_allowed(
        self,
        event: HandEventName,
        trigger_timestamp_ns: int,
        window_start: _StableWindowStart | None,
    ) -> HandEvent | None:
        if window_start is None:
            return None
        if (
            self._last_event_ts is not None
            and trigger_timestamp_ns - self._last_event_ts < self.config.min_event_gap_ns
        ):
            return None

        self.state = event
        self._last_event_ts = int(trigger_timestamp_ns)
        self._grasp_count = 0
        self._release_count = 0
        self._grasp_start = None
        self._release_start = None
        return HandEvent(
            event=event,
            fid=window_start.fid,
            timestamp_ns=window_start.timestamp_ns,
            grip_score=window_start.grip_score,
            finger_curls=dict(window_start.finger_curls),
            revo2_positions=command_for_event(event, self.config),
        )


def detect_hand_events(
    frames: Iterable[SupportsHandFrame],
    config: HandEventConfig | None = None,
    side: str = "right",
) -> list[HandEvent]:
    """从已解析的 Pico 回放帧里提取抓握/松开事件。"""

    detector = HandEventDetector(config)
    events: list[HandEvent] = []
    for frame in frames:
        event = detector.update(
            frame.fid,
            frame.timestamp_ns,
            frame.positions,
            frame.quaternions,
            side=side,
        )
        if event is not None:
            events.append(event)
    return events


def _extract_right_hand_frame(row: dict, sync_threshold_ms: float | None) -> HandFrame | None:
    try:
        fid = int(row["fid"])
        timestamp_ns = int(row["ts"])
    except (KeyError, TypeError, ValueError):
        return None

    hand_timestamp = row.get("hand_snapshot_boot_ns")
    if hand_timestamp is not None and sync_threshold_ms is not None and sync_threshold_ms > 0:
        try:
            sync_delta_ms = abs(int(hand_timestamp) - timestamp_ns) / 1e6
        except (TypeError, ValueError):
            return None
        if sync_delta_ms > sync_threshold_ms:
            return None

    right_hand = row.get("rh")
    if not isinstance(right_hand, dict) or not right_hand.get("active", False):
        return None
    joints = right_hand.get("joints")
    if not isinstance(joints, list) or len(joints) != NUM_HAND_JOINTS:
        return None

    positions = np.empty((NUM_HAND_JOINTS, 3), dtype=np.float64)
    quaternions = np.empty((NUM_HAND_JOINTS, 4), dtype=np.float64)
    for index, joint in enumerate(joints):
        if not isinstance(joint, (list, tuple)) or len(joint) < 7:
            return None
        try:
            positions[index] = [float(value) for value in joint[:3]]
            quaternions[index] = [float(value) for value in joint[3:7]]
        except (TypeError, ValueError):
            return None

    if not np.all(np.isfinite(positions)) or not np.all(np.isfinite(quaternions)):
        return None
    norms = np.linalg.norm(quaternions, axis=1)
    if np.any(norms < 1e-8):
        return None
    quaternions = quaternions / norms[:, None]
    return HandFrame(
        fid=fid,
        timestamp_ns=timestamp_ns,
        positions=positions,
        quaternions=quaternions,
    )


def load_hand_frames(
    session_dir: Path | str,
    sync_threshold_ms: float | None = 50.0,
    start_fid: int | None = None,
    end_fid: int | None = None,
    max_frames: int | None = None,
) -> list[HandFrame]:
    """从 Pico session 的 poses.jsonl 中读取有效右手帧。"""

    session_path = resolve_session_dir(session_dir)
    poses_path = session_path / "poses.jsonl"
    if not poses_path.is_file():
        raise FileNotFoundError(f"poses.jsonl not found: {poses_path}")
    if max_frames is not None and max_frames <= 0:
        raise ValueError("max_frames must be positive")

    frames: list[HandFrame] = []
    with poses_path.open("r", encoding="utf-8") as stream:
        for line in stream:
            line = line.strip()
            if not line:
                continue
            try:
                row = json.loads(line)
            except (TypeError, json.JSONDecodeError):
                continue

            try:
                fid = int(row["fid"])
            except (KeyError, TypeError, ValueError):
                continue
            if (start_fid is not None and fid < start_fid) or (
                end_fid is not None and fid > end_fid
            ):
                continue

            frame = _extract_right_hand_frame(row, sync_threshold_ms)
            if frame is None:
                continue
            frames.append(frame)
            if max_frames is not None and len(frames) >= max_frames:
                break
    return frames


def load_hand_events(
    session_dir: Path | str,
    config: HandEventConfig | None = None,
    sync_threshold_ms: float | None = 50.0,
    start_fid: int | None = None,
    end_fid: int | None = None,
    max_frames: int | None = None,
) -> list[HandEvent]:
    """读取 session 的 poses.jsonl 并输出抓握/松开事件。"""

    frames = load_hand_frames(
        session_dir,
        sync_threshold_ms=sync_threshold_ms,
        start_fid=start_fid,
        end_fid=end_fid,
        max_frames=max_frames,
    )
    return detect_hand_events(frames, config=config, side="right")


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Detect Revo2 grasp/release events from Pico poses.jsonl."
    )
    parser.add_argument("session_dir", type=Path, help="包含 poses.jsonl 的 Pico session 目录")
    parser.add_argument("--grasp-threshold", type=float, default=DEFAULT_GRASP_THRESHOLD)
    parser.add_argument("--release-threshold", type=float, default=DEFAULT_RELEASE_THRESHOLD)
    parser.add_argument("--hold-frames", type=int, default=DEFAULT_HOLD_FRAMES)
    parser.add_argument("--min-gap-sec", type=float, default=DEFAULT_MIN_EVENT_GAP_NS / 1e9)
    parser.add_argument("--sync-threshold-ms", type=float, default=50.0)
    parser.add_argument("--start-fid", type=int, default=None)
    parser.add_argument("--end-fid", type=int, default=None)
    parser.add_argument("--max-frames", type=int, default=None)
    parser.add_argument("--json", action="store_true", help="以 JSON Lines 输出事件")
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = _build_arg_parser()
    args = parser.parse_args(argv)
    config = HandEventConfig(
        grasp_threshold=args.grasp_threshold,
        release_threshold=args.release_threshold,
        hold_frames=args.hold_frames,
        min_event_gap_ns=int(args.min_gap_sec * 1e9),
    )
    events = load_hand_events(
        args.session_dir,
        config=config,
        sync_threshold_ms=args.sync_threshold_ms,
        start_fid=args.start_fid,
        end_fid=args.end_fid,
        max_frames=args.max_frames,
    )

    if args.json:
        for event in events:
            print(json.dumps(event.__dict__, ensure_ascii=False, sort_keys=True))
        return 0

    if not events:
        print("未识别到 grasp/release 事件")
        return 0

    for event in events:
        print(
            f"{event.event:7s} fid={event.fid} "
            f"t={event.timestamp_ns / 1e9:.3f}s "
            f"score={event.grip_score:.3f} "
            f"cmd={event.revo2_positions}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
