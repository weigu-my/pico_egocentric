# pico_egocentric

Public notes and reusable utilities from a PICO egocentric teleoperation/data-collection project.

This repository intentionally contains a sanitized subset only:

- PICO recording remote-control client (`recording_ctl.py`)
- USB pedal recording controller (`pedal_ctl.py`)
- PICO session path resolver (`session_paths.py`)
- grasp/release event detector from PICO hand joints (`hand_event_detector.py`)
- Revo2 hand command ROS2 node helper (`revo2_hand_node.py`)
- PICO hand to Revo2 retargeting utilities (`revo2_retarget_optimizer.py`)
- overlay and metric helpers for offline inspection
- Unity C# scripts for the PICO egocentric recorder
- public Revo2 URDF text files and license
- synthetic/unit tests that do not include real robot data
- project memory document in `docs/project_memory_for_github.md`
- sanitized project blog in `docs/project_blog.md`

The following are not included:

- real egocentric sessions, images, logs, rosbags, or robot run data
- company/internal robot launch files, controller addresses, or deployment credentials
- Manus hardware access notes and private hardware assets
- large mesh binaries and generated Unity cache/build folders

## Basic Usage

Remote-control the PICO Unity recording app:

```bash
python3 -m pico_egocentric_pkg.pico.recording_ctl --host <PICO_IP> ping
python3 -m pico_egocentric_pkg.pico.recording_ctl --host <PICO_IP> start
python3 -m pico_egocentric_pkg.pico.recording_ctl --host <PICO_IP> stop
```

Detect hand open/close events from an exported session:

```bash
python3 -m pico_egocentric_pkg.pico.hand_event_detector <SESSION_DIR>
```

Run lightweight tests:

```bash
python3 tests/test_session_paths.py
python3 tests/test_revo2_hand_node.py
python3 tests/test_hand_event_detector.py
```

## Notes

The Revo2 ROS2 node requires ROS2 plus BrainCo's Python SDK at runtime. The pure helpers and synthetic tests can be used without hardware.

## Unity PICO Recorder Scripts

The Unity-side recorder scripts are under:

```text
unity/Assets/Scripts/
  EgocentricDataCollector.cs
  EgocentricDataTransforms.cs
  RecordingControlServer.cs
```

`EgocentricDataCollector.cs` is the main PICO data collector. It captures RGB frames, head/controller poses, hand joints, and writes `metadata.json`, `poses.jsonl`, and `frames/*.jpg`.

`EgocentricDataTransforms.cs` contains image and pose conversion helpers used by the collector.

`RecordingControlServer.cs` adds the TCP command server used by the Python `recording_ctl.py` and `pedal_ctl.py` clients.
