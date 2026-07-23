#!/usr/bin/env python3
"""Draw recorded Pico hand joint 2D projections onto exported RGB frames."""

from __future__ import annotations

import argparse
import json
import math
import shutil
import subprocess
import statistics
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Iterator

from PIL import Image, ImageDraw

JOINT_NAMES = [
    "Palm", "Wrist",
    "ThumbMeta", "ThumbProx", "ThumbDist", "ThumbTip",
    "IndexMeta", "IndexProx", "IndexInter", "IndexDist", "IndexTip",
    "MiddleMeta", "MiddleProx", "MiddleInter", "MiddleDist", "MiddleTip",
    "RingMeta", "RingProx", "RingInter", "RingDist", "RingTip",
    "LittleMeta", "LittleProx", "LittleInter", "LittleDist", "LittleTip",
]

FINGER_CHAINS = {
    "Thumb": [1, 2, 3, 4, 5],
    "Index": [1, 6, 7, 8, 9, 10],
    "Middle": [1, 11, 12, 13, 14, 15],
    "Ring": [1, 16, 17, 18, 19, 20],
    "Little": [1, 21, 22, 23, 24, 25],
}

HAND_COLORS = {
    "lh": (40, 150, 255),
    "rh": (255, 90, 70),
}
WRIST_COLOR = (255, 220, 30)
SAVED_JOINTS_FIELD = "joints_2d"
REPROJECT_FIELD = "reproject_pos_converted"
DEFAULT_HEAD_ABOVE_PALM_M = 0.24
LOCAL_HEAD_POSE_GAP_M = 0.8


@dataclass(frozen=True)
class ProjectionOptions:
    head_y_override: float | None = None


def apply_image_transform(
    u: float,
    v: float,
    width: int,
    height: int,
    rotate_180: bool,
    flip_horizontal: bool,
) -> tuple[float, float]:
    """Apply the same pixel transform order used by the Unity RGB exporter."""
    if rotate_180:
        u = width - 1 - u
        v = height - 1 - v
    if flip_horizontal:
        u = width - 1 - u
    return u, v


def _is_valid_joint(joint: object, width: int, height: int) -> bool:
    if not isinstance(joint, (list, tuple)) or len(joint) < 4:
        return False
    u, v, depth, valid = joint[:4]
    if not valid or depth <= 0:
        return False
    return 0 <= u < width and 0 <= v < height


def _draw_circle(draw: ImageDraw.ImageDraw, u: float, v: float, radius: int, color: tuple[int, int, int]) -> None:
    draw.ellipse((u - radius, v - radius, u + radius, v + radius), fill=color)


def _draw_hand(
    draw: ImageDraw.ImageDraw,
    joints: list,
    color: tuple[int, int, int],
    width: int,
    height: int,
    radius: int,
    line_width: int,
) -> int:
    valid = [_is_valid_joint(joint, width, height) for joint in joints]

    for chain in FINGER_CHAINS.values():
        for a, b in zip(chain[:-1], chain[1:]):
            if a < len(joints) and b < len(joints) and valid[a] and valid[b]:
                au, av = joints[a][0], joints[a][1]
                bu, bv = joints[b][0], joints[b][1]
                draw.line((au, av, bu, bv), fill=color, width=line_width)

    drawn = 0
    for idx, joint in enumerate(joints[: len(JOINT_NAMES)]):
        if not valid[idx]:
            continue
        dot_color = WRIST_COLOR if idx == 1 else color
        _draw_circle(draw, joint[0], joint[1], radius, dot_color)
        drawn += 1
    return drawn


def draw_pose_overlay(
    image: Image.Image,
    pose: dict,
    field: str = REPROJECT_FIELD,
    radius: int = 4,
    line_width: int = 2,
    metadata: dict | None = None,
    projection_options: ProjectionOptions | None = None,
) -> Image.Image:
    """Return a copy of image with lh/rh 2D hand joints overlaid."""
    out = image.convert("RGB").copy()
    draw = ImageDraw.Draw(out)
    width, height = out.size

    for side in ("lh", "rh"):
        hand = pose.get(side)
        if not isinstance(hand, dict) or not hand.get("active", False):
            continue
        joints = _select_joints_field(hand, field, pose, metadata, width, height, projection_options)
        if not isinstance(joints, list):
            continue
        _draw_hand(draw, joints, HAND_COLORS[side], width, height, radius, line_width)

    return out


def _select_joints_field(
    hand: dict,
    field: str,
    pose: dict | None = None,
    metadata: dict | None = None,
    width: int | None = None,
    height: int | None = None,
    projection_options: ProjectionOptions | None = None,
) -> object:
    if field == REPROJECT_FIELD:
        if pose is None or metadata is None or width is None or height is None:
            raise ValueError(f"{field} requires pose metadata and image size")
        return _reproject_hand_joints(hand, pose, metadata, width, height, projection_options or ProjectionOptions())

    if field == SAVED_JOINTS_FIELD:
        return hand.get(field)

    raise ValueError(f"unsupported hand joints field: {field}")


def _reproject_hand_joints(
    hand: dict,
    pose: dict,
    metadata: dict,
    width: int,
    height: int,
    options: ProjectionOptions,
) -> list | None:
    joints = hand.get("joints")
    head_pose = pose.get("head_tob_pose")
    if not isinstance(joints, list) or not isinstance(head_pose, dict):
        return None

    intrinsics = metadata.get("camera_intrinsics") or {}
    extrinsics = _select_projection_extrinsics(metadata)
    if not extrinsics:
        return None

    fx = float(intrinsics["fx"])
    fy = float(intrinsics["fy"])
    cx = float(intrinsics["cx"])
    cy = float(intrinsics["cy"])

    camera_pos = [float(v) for v in extrinsics["position"]]
    camera_rot = [float(v) for v in extrinsics["rotation_xyzw"]]
    camera_pos = _convert_right_handed_position_to_unity(camera_pos)

    head_pos = [float(v) for v in head_pose["p"]]
    if options.head_y_override is not None:
        head_pos[1] = options.head_y_override
    head_rot = _normalize_quat([float(v) for v in head_pose["r"]])
    camera_rot = _normalize_quat(camera_rot)

    camera_world_pos = _vec_add(head_pos, _quat_rotate(head_rot, camera_pos))
    camera_world_rot = _quat_mul(head_rot, camera_rot)
    world_to_camera_rot = _quat_inverse(_normalize_quat(camera_world_rot))

    rotate_180 = int(metadata.get("image_rotation_correction_degrees", 0)) == 180
    flip_horizontal = bool(metadata.get("image_horizontal_flip_correction", False))

    projected = []
    for joint in joints:
        if not isinstance(joint, (list, tuple)) or len(joint) < 3:
            projected.append([0.0, 0.0, 0.0, 0])
            continue

        joint_world = [float(joint[0]), float(joint[1]), float(joint[2])]
        joint_camera = _quat_rotate(world_to_camera_rot, _vec_sub(joint_world, camera_world_pos))

        # PICO RGB 外参的相机前方对应本地 -Z，depth 取反后再套针孔模型。
        depth = -joint_camera[2]
        u = 0.0
        v = 0.0
        valid = depth > 0.0001
        if valid:
            u = fx * joint_camera[0] / depth + cx
            v = cy - fy * joint_camera[1] / depth
            u, v = apply_image_transform(u, v, width, height, rotate_180, flip_horizontal)
            valid = 0 <= u < width and 0 <= v < height
        projected.append([u, v, depth, 1 if valid else 0])

    return projected


def _select_projection_extrinsics(metadata: dict) -> dict | None:
    render_mode = str(metadata.get("render_mode", ""))
    key = "camera_extrinsics_right" if "RIGHT" in render_mode else "camera_extrinsics_left"
    extrinsics = metadata.get(key)
    return extrinsics if isinstance(extrinsics, dict) else None


def _convert_right_handed_position_to_unity(position: list[float]) -> list[float]:
    return [position[0], position[1], -position[2]]


def _normalize_quat(q: list[float]) -> list[float]:
    norm = math.sqrt(sum(v * v for v in q))
    if norm <= 0.0:
        return [0.0, 0.0, 0.0, 1.0]
    return [v / norm for v in q]


def _quat_inverse(q: list[float]) -> list[float]:
    return [-q[0], -q[1], -q[2], q[3]]


def _quat_mul(a: list[float], b: list[float]) -> list[float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ]


def _quat_rotate(q: list[float], v: list[float]) -> list[float]:
    rotated = _quat_mul(_quat_mul(q, [v[0], v[1], v[2], 0.0]), _quat_inverse(q))
    return rotated[:3]


def _vec_add(a: list[float], b: list[float]) -> list[float]:
    return [a[i] + b[i] for i in range(3)]


def _vec_sub(a: list[float], b: list[float]) -> list[float]:
    return [a[i] - b[i] for i in range(3)]


def iter_pose_rows(poses_path: Path) -> Iterator[dict]:
    with poses_path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                yield json.loads(line)


def estimate_head_y_override(rows: list[dict]) -> float | None:
    """Estimate global head Y when recorded head pose is in local tracking space."""
    head_ys = []
    palm_ys = []
    for pose in rows:
        head_pose = pose.get("head_tob_pose")
        if isinstance(head_pose, dict):
            p = head_pose.get("p")
            if isinstance(p, list) and len(p) >= 2:
                head_ys.append(float(p[1]))

        for side in ("lh", "rh"):
            hand = pose.get(side)
            if not isinstance(hand, dict) or not hand.get("active", False):
                continue
            joints = hand.get("joints")
            if isinstance(joints, list) and joints and isinstance(joints[0], list) and len(joints[0]) >= 2:
                palm_ys.append(float(joints[0][1]))

    if not head_ys or not palm_ys:
        return None

    median_head_y = statistics.median(head_ys)
    median_palm_y = statistics.median(palm_ys)
    if median_palm_y - median_head_y < LOCAL_HEAD_POSE_GAP_M:
        return None

    return median_palm_y + DEFAULT_HEAD_ABOVE_PALM_M


def resolve_overlay_output_dir(session_dir: Path | str, output_dir: Path | str | None = None) -> Path:
    """Return the overlay directory used for generated images and video."""
    session_path = Path(session_dir).expanduser().resolve()
    if output_dir is None:
        return session_path / "overlay"

    output_path = Path(output_dir).expanduser().resolve()
    if output_path.name == "overlay":
        return output_path
    return output_path / "overlay"


def render_session(
    session_dir: Path,
    output_dir: Path | None = None,
    field: str = REPROJECT_FIELD,
    limit: int | None = None,
    make_video: bool = False,
    fps: int = 30,
) -> tuple[int, Path]:
    """Render overlay images for one recorded session."""
    session_dir = session_dir.expanduser().resolve()
    output_dir = resolve_overlay_output_dir(session_dir, output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    poses_path = session_dir / "poses.jsonl"
    frames_dir = session_dir / "frames"
    if not poses_path.is_file():
        raise FileNotFoundError(f"poses.jsonl not found: {poses_path}")
    if not frames_dir.is_dir():
        raise FileNotFoundError(f"frames directory not found: {frames_dir}")

    metadata = None
    metadata_path = session_dir / "metadata.json"
    if metadata_path.is_file():
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))

    pose_rows = list(iter_pose_rows(poses_path))
    head_y_override = estimate_head_y_override(pose_rows) if field == REPROJECT_FIELD else None
    projection_options = ProjectionOptions(head_y_override=head_y_override)
    if head_y_override is not None:
        print(
            f"WARNING: head_tob_pose appears local; using estimated global head_y={head_y_override:.3f}m "
            "for overlay reprojection."
        )

    rendered = 0
    for pose in pose_rows:
        if limit is not None and rendered >= limit:
            break
        fid = int(pose["fid"])
        frame_path = frames_dir / f"{fid:06d}.jpg"
        if not frame_path.is_file():
            continue
        image = Image.open(frame_path)
        out = draw_pose_overlay(image, pose, field=field, metadata=metadata, projection_options=projection_options)
        out.save(output_dir / f"{fid:06d}.jpg", quality=95)
        rendered += 1

    if make_video and rendered > 0:
        _run_ffmpeg(output_dir, fps)

    return rendered, output_dir


def _run_ffmpeg(output_dir: Path, fps: int) -> None:
    ffmpeg = find_ffmpeg()
    if ffmpeg is None:
        raise RuntimeError("ffmpeg not found; omit --video or install ffmpeg/imageio-ffmpeg")

    output_path = output_dir / "overlay.mp4"
    cmd = [
        ffmpeg,
        "-y",
        "-framerate",
        str(fps),
        "-pattern_type",
        "glob",
        "-i",
        str(output_dir / "*.jpg"),
        "-c:v",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        str(output_path),
    ]
    subprocess.run(cmd, check=True)


def find_ffmpeg() -> str | None:
    """Return a usable ffmpeg executable path."""
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg:
        return ffmpeg

    try:
        import imageio_ffmpeg
    except ImportError:
        return None

    return imageio_ffmpeg.get_ffmpeg_exe()


def parse_args(argv: Iterable[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Overlay Pico 2D hand joints on recorded RGB frames.")
    parser.add_argument("session_dir", type=Path, help="Recorded egocentric session directory")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Output root directory; files are written to OUTPUT/overlay. Default: SESSION/overlay",
    )
    parser.add_argument(
        "--field",
        default=REPROJECT_FIELD,
        help=(
            "Hand field to draw: reproject_pos_converted (default) or joints_2d"
        ),
    )
    parser.add_argument("--limit", type=int, default=None, help="Render at most this many frames")
    parser.add_argument("--video", action="store_true", help="Also encode overlay.mp4 with ffmpeg")
    parser.add_argument("--fps", type=int, default=30, help="Video FPS when --video is set")
    return parser.parse_args(argv)


def main(argv: Iterable[str] | None = None) -> None:
    args = parse_args(argv)
    rendered, output_dir = render_session(
        args.session_dir,
        output_dir=args.output_dir,
        field=args.field,
        limit=args.limit,
        make_video=args.video,
        fps=args.fps,
    )
    print(f"Rendered {rendered} overlay frames to {output_dir}")


if __name__ == "__main__":
    main()
