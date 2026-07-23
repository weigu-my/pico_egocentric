#!/usr/bin/env python3
"""Synthetic tests for Pico grasp/release event detection."""

import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(SCRIPT_DIR, ".."))

from pico_egocentric_pkg.pico.hand_event_detector import (  # noqa: E402
    DEFAULT_BOTTLE_GRASP_POSITIONS,
    DEFAULT_RELEASE_POSITIONS,
    HandEventConfig,
    HandEventDetector,
    command_for_event,
    compute_grip_score,
)


def test_compute_grip_score_averages_long_fingers():
    curls = {
        "thumb": 1.0,
        "index": 0.25,
        "middle": 0.50,
        "ring": 0.75,
        "pinky": 1.0,
    }

    assert compute_grip_score(curls) == 0.625


def test_detector_emits_window_start_for_stable_events():
    detector = HandEventDetector(
        HandEventConfig(
            grasp_threshold=0.65,
            release_threshold=0.35,
            hold_frames=2,
            min_event_gap_ns=0,
        )
    )
    open_curls = {"index": 0.1, "middle": 0.2, "ring": 0.2, "pinky": 0.1}
    closed_curls = {"index": 0.8, "middle": 0.9, "ring": 0.8, "pinky": 0.9}

    assert detector.update_curls(0, 0, open_curls) is None
    assert detector.update_curls(1, 10_000_000, closed_curls) is None
    grasp = detector.update_curls(2, 20_000_000, closed_curls)
    assert grasp is not None
    assert grasp.event == "grasp"
    assert grasp.fid == 1
    assert grasp.timestamp_ns == 10_000_000

    assert detector.update_curls(3, 30_000_000, closed_curls) is None
    assert detector.update_curls(4, 40_000_000, open_curls) is None
    release = detector.update_curls(5, 50_000_000, open_curls)
    assert release is not None
    assert release.event == "release"
    assert release.fid == 4
    assert release.timestamp_ns == 40_000_000


def test_command_for_event_returns_default_revo2_targets():
    assert command_for_event("grasp") == DEFAULT_BOTTLE_GRASP_POSITIONS
    assert command_for_event("release") == DEFAULT_RELEASE_POSITIONS


if __name__ == "__main__":
    for test in (
        test_compute_grip_score_averages_long_fingers,
        test_detector_emits_window_start_for_stable_events,
        test_command_for_event_returns_default_revo2_targets,
    ):
        test()
        print(f"PASS: {test.__name__}")
