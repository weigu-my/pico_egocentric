#!/usr/bin/env python3
"""Error metrics for Pico hand targets vs Revo2 retargeting results."""

import csv
import json
import math
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, Iterable, List, Optional

import numpy as np

from .revo2_retarget_optimizer import FINGER_NAMES, SPREAD_PAIRS, RetargetResult

PINCH_KEYS = ("thumb_index", "thumb_middle")


def _nan_dict(keys: Iterable[str]) -> Dict[str, float]:
    return {key: math.nan for key in keys}


def _safe_vec(value) -> np.ndarray:
    arr = np.asarray(value, dtype=float)
    if arr.shape != (3,) or not np.all(np.isfinite(arr)):
        return np.full(3, math.nan, dtype=float)
    return arr


def _safe_scalar(value, default=math.nan) -> float:
    try:
        out = float(value)
    except Exception:
        return float(default)
    return out if math.isfinite(out) else float(default)


def _finite_values(values: Iterable[float]) -> np.ndarray:
    arr = np.asarray(list(values), dtype=float)
    if arr.size == 0:
        return np.array([], dtype=float)
    return arr[np.isfinite(arr)]


def _nanmean_dict(d: Dict[str, float]) -> float:
    vals = _finite_values(d.values())
    return float(np.mean(vals)) if vals.size else math.nan


def _nanmax_dict(d: Dict[str, float]) -> float:
    vals = _finite_values(d.values())
    return float(np.max(vals)) if vals.size else math.nan


def _fmt(value: float, precision: int = 2, unit: str = "") -> str:
    if value is None or not math.isfinite(float(value)):
        return "nan"
    return f"{float(value):.{precision}f}{unit}"


@dataclass
class FrameErrorMetrics:
    side: str
    frame_idx: Optional[int] = None
    frame_valid_for_metrics: bool = False
    solver_success: bool = False
    fallback_used: bool = False
    iterations: int = 0
    tip_errors_mm: Dict[str, float] = field(default_factory=dict)
    tip_mean_mm: float = math.nan
    tip_max_mm: float = math.nan
    dir_errors_deg: Dict[str, float] = field(default_factory=dict)
    dir_mean_deg: float = math.nan
    dir_max_deg: float = math.nan
    pinch_errors_mm: Dict[str, float] = field(default_factory=dict)
    spread_errors_mm: Dict[str, float] = field(default_factory=dict)
    spread_mean_mm: float = math.nan
    spread_max_mm: float = math.nan
    curl_errors: Dict[str, float] = field(default_factory=dict)
    curl_mean: float = math.nan
    curl_max: float = math.nan
    weighted_total_loss: float = math.nan

    def to_dict(self) -> Dict[str, object]:
        return {
            "frame_idx": self.frame_idx,
            "side": self.side,
            "frame_valid_for_metrics": bool(self.frame_valid_for_metrics),
            "solver_success": bool(self.solver_success),
            "fallback_used": bool(self.fallback_used),
            "iterations": int(self.iterations),
            "tip_errors_mm": {k: _safe_scalar(v) for k, v in self.tip_errors_mm.items()},
            "tip_mean_mm": _safe_scalar(self.tip_mean_mm),
            "tip_max_mm": _safe_scalar(self.tip_max_mm),
            "dir_errors_deg": {k: _safe_scalar(v) for k, v in self.dir_errors_deg.items()},
            "dir_mean_deg": _safe_scalar(self.dir_mean_deg),
            "dir_max_deg": _safe_scalar(self.dir_max_deg),
            "pinch_errors_mm": {k: _safe_scalar(v) for k, v in self.pinch_errors_mm.items()},
            "spread_errors_mm": {k: _safe_scalar(v) for k, v in self.spread_errors_mm.items()},
            "spread_mean_mm": _safe_scalar(self.spread_mean_mm),
            "spread_max_mm": _safe_scalar(self.spread_max_mm),
            "curl_errors": {k: _safe_scalar(v) for k, v in self.curl_errors.items()},
            "curl_mean": _safe_scalar(self.curl_mean),
            "curl_max": _safe_scalar(self.curl_max),
            "weighted_total_loss": _safe_scalar(self.weighted_total_loss),
        }

    def to_csv_row(self) -> Dict[str, object]:
        row = {
            "frame_idx": -1 if self.frame_idx is None else int(self.frame_idx),
            "side": self.side,
            "frame_valid_for_metrics": int(self.frame_valid_for_metrics),
            "solver_success": int(self.solver_success),
            "fallback_used": int(self.fallback_used),
            "iterations": int(self.iterations),
            "tip_mean_mm": _safe_scalar(self.tip_mean_mm),
            "tip_max_mm": _safe_scalar(self.tip_max_mm),
            "dir_mean_deg": _safe_scalar(self.dir_mean_deg),
            "dir_max_deg": _safe_scalar(self.dir_max_deg),
            "pinch_thumb_index_error_mm": _safe_scalar(self.pinch_errors_mm.get("thumb_index", math.nan)),
            "pinch_thumb_middle_error_mm": _safe_scalar(self.pinch_errors_mm.get("thumb_middle", math.nan)),
            "spread_mean_mm": _safe_scalar(self.spread_mean_mm),
            "spread_max_mm": _safe_scalar(self.spread_max_mm),
            "curl_mean": _safe_scalar(self.curl_mean),
            "curl_max": _safe_scalar(self.curl_max),
            "weighted_total_loss": _safe_scalar(self.weighted_total_loss),
        }
        for finger in FINGER_NAMES:
            row[f"tip_{finger}_mm"] = _safe_scalar(self.tip_errors_mm.get(finger, math.nan))
            row[f"dir_{finger}_deg"] = _safe_scalar(self.dir_errors_deg.get(finger, math.nan))
            row[f"curl_{finger}"] = _safe_scalar(self.curl_errors.get(finger, math.nan))
        for a, b in SPREAD_PAIRS:
            key = f"{a}_{b}"
            row[f"spread_{key}_mm"] = _safe_scalar(self.spread_errors_mm.get(key, math.nan))
        return row


@dataclass
class RunningErrorStats:
    side: Optional[str] = None
    max_frames: Optional[int] = None
    _frames: Deque[FrameErrorMetrics] = field(default_factory=deque)

    def update(self, frame_metrics: FrameErrorMetrics) -> None:
        if self.side is None:
            self.side = frame_metrics.side
        self._frames.append(frame_metrics)
        if self.max_frames is not None:
            while len(self._frames) > self.max_frames:
                self._frames.popleft()

    def frames(self) -> List[FrameErrorMetrics]:
        return list(self._frames)

    def to_csv_rows(self) -> List[Dict[str, object]]:
        return [frame.to_csv_row() for frame in self._frames]

    def summary_dict(self) -> Dict[str, object]:
        frames = list(self._frames)
        valid = [f for f in frames if f.frame_valid_for_metrics]

        def _series(attr: str) -> np.ndarray:
            return _finite_values(getattr(frame, attr) for frame in valid)

        tip_mean = _series("tip_mean_mm")
        dir_mean = _series("dir_mean_deg")
        spread_mean = _series("spread_mean_mm")
        curl_mean = _series("curl_mean")
        loss_vals = _series("weighted_total_loss")
        pinch_ti = _finite_values(frame.pinch_errors_mm.get("thumb_index", math.nan) for frame in valid)
        pinch_tm = _finite_values(frame.pinch_errors_mm.get("thumb_middle", math.nan) for frame in valid)
        tip_max = _finite_values(frame.tip_max_mm for frame in valid)

        return {
            "side": self.side,
            "window_size": len(frames),
            "num_frames_total": len(frames),
            "num_frames_valid": sum(frame.frame_valid_for_metrics for frame in frames),
            "num_solver_fail": sum(not frame.solver_success for frame in frames),
            "num_fallback": sum(frame.fallback_used for frame in frames),
            "tip_mean_mm_avg": float(np.mean(tip_mean)) if tip_mean.size else math.nan,
            "tip_mean_mm_p95": float(np.percentile(tip_mean, 95)) if tip_mean.size else math.nan,
            "tip_max_mm_max": float(np.max(tip_max)) if tip_max.size else math.nan,
            "dir_mean_deg_avg": float(np.mean(dir_mean)) if dir_mean.size else math.nan,
            "dir_mean_deg_p95": float(np.percentile(dir_mean, 95)) if dir_mean.size else math.nan,
            "pinch_thumb_index_mm_avg": float(np.mean(pinch_ti)) if pinch_ti.size else math.nan,
            "pinch_thumb_middle_mm_avg": float(np.mean(pinch_tm)) if pinch_tm.size else math.nan,
            "spread_mean_mm_avg": float(np.mean(spread_mean)) if spread_mean.size else math.nan,
            "curl_mean_avg": float(np.mean(curl_mean)) if curl_mean.size else math.nan,
            "weighted_total_loss_avg": float(np.mean(loss_vals)) if loss_vals.size else math.nan,
        }


def compute_frame_error_metrics(result: RetargetResult, side: str,
                                frame_idx: Optional[int] = None) -> FrameErrorMetrics:
    debug = result.debug_info or {}
    scale = _safe_scalar(debug.get("scale", 1.0), default=1.0)
    valid = bool(debug.get("target_tip_positions"))

    tip_errors = _nan_dict(FINGER_NAMES)
    dir_errors = _nan_dict(FINGER_NAMES)
    pinch_errors = _nan_dict(PINCH_KEYS)
    spread_errors = _nan_dict(f"{a}_{b}" for a, b in SPREAD_PAIRS)
    curl_errors = _nan_dict(FINGER_NAMES)

    if valid:
        target_tips = debug.get("target_tip_positions", {})
        robot_tips = debug.get("robot_tip_positions", {})
        for finger in FINGER_NAMES:
            t = _safe_vec(target_tips.get(finger))
            r = _safe_vec(robot_tips.get(finger))
            if np.all(np.isfinite(t)) and np.all(np.isfinite(r)):
                tip_errors[finger] = float(np.linalg.norm(scale * t - r) * 1000.0)

        target_dirs = debug.get("target_dirs", {})
        robot_dirs = debug.get("robot_dirs", {})
        for finger in FINGER_NAMES:
            t = _safe_vec(target_dirs.get(finger))
            r = _safe_vec(robot_dirs.get(finger))
            if np.all(np.isfinite(t)) and np.all(np.isfinite(r)):
                nt = np.linalg.norm(t)
                nr = np.linalg.norm(r)
                if nt > 1e-8 and nr > 1e-8:
                    dot = float(np.clip(np.dot(t / nt, r / nr), -1.0, 1.0))
                    dir_errors[finger] = float(np.degrees(np.arccos(dot)))

        target_pinch = debug.get("target_pinch", {})
        robot_pinch = debug.get("robot_pinch", {})
        for key in PINCH_KEYS:
            t = _safe_scalar(target_pinch.get(key))
            r = _safe_scalar(robot_pinch.get(key))
            if math.isfinite(t) and math.isfinite(r):
                pinch_errors[key] = abs(scale * t - r) * 1000.0

        target_spread = debug.get("target_spread", {})
        robot_spread = debug.get("robot_spread", {})
        for a, b in SPREAD_PAIRS:
            key = f"{a}_{b}"
            t = _safe_scalar(target_spread.get(key))
            r = _safe_scalar(robot_spread.get(key))
            if math.isfinite(t) and math.isfinite(r):
                spread_errors[key] = abs(scale * t - r) * 1000.0

        target_curls = debug.get("target_curls", {})
        robot_curls = debug.get("robot_curls", {})
        for finger in FINGER_NAMES:
            t = _safe_scalar(target_curls.get(finger))
            r = _safe_scalar(robot_curls.get(finger))
            if math.isfinite(t) and math.isfinite(r):
                curl_errors[finger] = abs(t - r)

    loss_terms = debug.get("loss_terms", {}) or {}

    return FrameErrorMetrics(
        side=side,
        frame_idx=frame_idx,
        frame_valid_for_metrics=valid,
        solver_success=bool(debug.get("solver_success", False)),
        fallback_used=bool(debug.get("fallback_used", False)),
        iterations=int(debug.get("iterations", 0)),
        tip_errors_mm=tip_errors,
        tip_mean_mm=_nanmean_dict(tip_errors),
        tip_max_mm=_nanmax_dict(tip_errors),
        dir_errors_deg=dir_errors,
        dir_mean_deg=_nanmean_dict(dir_errors),
        dir_max_deg=_nanmax_dict(dir_errors),
        pinch_errors_mm=pinch_errors,
        spread_errors_mm=spread_errors,
        spread_mean_mm=_nanmean_dict(spread_errors),
        spread_max_mm=_nanmax_dict(spread_errors),
        curl_errors=curl_errors,
        curl_mean=_nanmean_dict(curl_errors),
        curl_max=_nanmax_dict(curl_errors),
        weighted_total_loss=_safe_scalar(loss_terms.get("weighted_total")),
    )


def format_frame_metrics_text(metrics: FrameErrorMetrics) -> str:
    lines = [
        f"tip mean: {_fmt(metrics.tip_mean_mm, 1, ' mm')}",
        f"tip max: {_fmt(metrics.tip_max_mm, 1, ' mm')}",
        f"dir mean: {_fmt(metrics.dir_mean_deg, 1, ' deg')}",
        f"pinch ti: {_fmt(metrics.pinch_errors_mm.get('thumb_index', math.nan), 1, ' mm')}",
        f"pinch tm: {_fmt(metrics.pinch_errors_mm.get('thumb_middle', math.nan), 1, ' mm')}",
        f"spread mean: {_fmt(metrics.spread_mean_mm, 1, ' mm')}",
        f"curl mean: {_fmt(metrics.curl_mean, 3)}",
        f"solver: {metrics.solver_success}",
        f"fallback: {metrics.fallback_used}",
    ]
    return "\n".join(lines)


def format_running_summary(summary: Dict[str, object], prefix: str = "") -> str:
    side_prefix = f"{prefix} " if prefix else ""
    return (
        f"{side_prefix}valid={summary['num_frames_valid']}/{summary['num_frames_total']} "
        f"tip_mean={_fmt(summary['tip_mean_mm_avg'], 1, 'mm')} "
        f"tip_max={_fmt(summary['tip_max_mm_max'], 1, 'mm')} "
        f"dir_mean={_fmt(summary['dir_mean_deg_avg'], 1, 'deg')} "
        f"pinch_ti={_fmt(summary['pinch_thumb_index_mm_avg'], 1, 'mm')} "
        f"spread={_fmt(summary['spread_mean_mm_avg'], 1, 'mm')} "
        f"curl={_fmt(summary['curl_mean_avg'], 3)} "
        f"fallbacks={summary['num_fallback']}"
    )


def write_metrics_csv(path: str, rows: List[Dict[str, object]]) -> None:
    if not rows:
        rows = [{"frame_idx": -1, "side": "", "frame_valid_for_metrics": 0}]
    fieldnames = list(rows[0].keys())
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_metrics_json(path: str, payload: Dict[str, object]) -> None:
    with open(path, "w") as f:
        json.dump(payload, f, indent=2)
