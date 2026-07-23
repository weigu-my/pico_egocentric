#!/usr/bin/env python3
"""Tests for the Revo2 ROS hand command node helpers."""

import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(SCRIPT_DIR, ".."))

from pico_egocentric_pkg.pico.revo2_hand_node import (  # noqa: E402
    AsyncRevo2Client,
    normalize_revo2_positions,
)


def test_normalize_revo2_positions_accepts_six_numeric_values():
    assert normalize_revo2_positions([0, 100.2, 500.8, 900, 1000, 42]) == [
        0,
        100,
        501,
        900,
        1000,
        42,
    ]


def test_normalize_revo2_positions_rejects_wrong_length():
    try:
        normalize_revo2_positions([0, 1, 2])
    except ValueError as exc:
        assert "exactly 6" in str(exc)
    else:
        raise AssertionError("expected ValueError")


def test_normalize_revo2_positions_rejects_out_of_range():
    try:
        normalize_revo2_positions([0, 100, 200, 300, 400, 1001])
    except ValueError as exc:
        assert "0..1000" in str(exc)
    else:
        raise AssertionError("expected ValueError")


def test_async_client_defaults_to_plain_position_api():
    client = AsyncRevo2Client("canfd0", 127, sdk_python_dir="/tmp/unused")

    assert client.speed is None


if __name__ == "__main__":
    tests = [
        test_normalize_revo2_positions_accepts_six_numeric_values,
        test_normalize_revo2_positions_rejects_wrong_length,
        test_normalize_revo2_positions_rejects_out_of_range,
        test_async_client_defaults_to_plain_position_api,
    ]
    for test in tests:
        test()
        print(f"PASS: {test.__name__}")
