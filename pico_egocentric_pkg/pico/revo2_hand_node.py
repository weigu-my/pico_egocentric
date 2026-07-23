#!/usr/bin/env python3
"""ROS2 节点：订阅 Revo2 手部目标并通过 BrainCo SDK 下发 CANFD 命令。"""

from __future__ import annotations

import argparse
import asyncio
import importlib
import sys
import threading
from concurrent.futures import Future
from pathlib import Path
from typing import Iterable

HAND_TOPIC = "/revo2/right_hand/target"
DEFAULT_IFACE = "canfd0"
DEFAULT_SLAVE_ID = 127
DEFAULT_SPEED = 0


def normalize_revo2_positions(values: Iterable[float]) -> list[int]:
    """校验并归一化 Revo2 六指位置命令，范围为 0..1000。"""

    positions = [int(round(float(value))) for value in values]
    if len(positions) != 6:
        raise ValueError(f"Revo2 command must contain exactly 6 values, got {len(positions)}")
    for value in positions:
        if not 0 <= value <= 1000:
            raise ValueError(f"Revo2 positions must be in 0..1000, got {positions}")
    return positions


def _default_sdk_python_dir() -> Path:
    """定位 third_party/brainco-hand-sdk/python。"""

    return Path(__file__).resolve().parents[4] / "third_party" / "brainco-hand-sdk" / "python"


def _load_common_init(sdk_python_dir: Path):
    sdk_path = str(sdk_python_dir.expanduser().resolve())
    if sdk_path not in sys.path:
        sys.path.insert(0, sdk_path)
    return importlib.import_module("common_init")


class AsyncRevo2Client:
    """在后台 asyncio loop 中持有 BrainCo SDK 连接。"""

    def __init__(
        self,
        iface: str,
        slave_id: int,
        sdk_python_dir: Path,
        speed: int | None = DEFAULT_SPEED,
    ):
        self.iface = iface
        self.slave_id = int(slave_id)
        self.sdk_python_dir = sdk_python_dir
        self.speed = None if speed is None or speed <= 0 else int(speed)
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._run_loop, name="revo2-hand-async", daemon=True)
        self._device = None
        self._common_init = None

    def _run_loop(self) -> None:
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    def start(self, timeout_sec: float = 10.0) -> None:
        self._thread.start()
        future = asyncio.run_coroutine_threadsafe(self._open(), self._loop)
        future.result(timeout=timeout_sec)

    async def _open(self) -> None:
        self._common_init = _load_common_init(self.sdk_python_dir)
        self._device = await self._common_init.init_socketcan(
            self.iface,
            self.slave_id,
            is_canfd=True,
        )
        if self._device is None:
            raise RuntimeError(f"Revo2 CANFD 初始化失败: iface={self.iface}, slave_id={self.slave_id}")

    def send_positions(self, positions: Iterable[float]) -> Future:
        normalized = normalize_revo2_positions(positions)
        return asyncio.run_coroutine_threadsafe(self._send_positions(normalized), self._loop)

    async def _send_positions(self, positions: list[int]) -> None:
        if self._device is None:
            raise RuntimeError("Revo2 device is not initialized")
        ctx = self._device.ctx
        slave_id = self._device.slave_id
        if self.speed is not None and hasattr(ctx, "set_finger_positions_and_speeds"):
            await ctx.set_finger_positions_and_speeds(slave_id, positions, [self.speed] * 6)
        else:
            await ctx.set_finger_positions(slave_id, positions)

    def close(self, timeout_sec: float = 5.0) -> None:
        if self._loop.is_closed():
            return
        try:
            if self._device is not None and self._common_init is not None:
                future = asyncio.run_coroutine_threadsafe(
                    self._common_init.cleanup_context(self._device),
                    self._loop,
                )
                future.result(timeout=timeout_sec)
        finally:
            self._loop.call_soon_threadsafe(self._loop.stop)
            self._thread.join(timeout=timeout_sec)
            self._loop.close()


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--iface", default=DEFAULT_IFACE, help=f"SocketCAN 接口(默认 {DEFAULT_IFACE})")
    parser.add_argument("--slave-id", type=int, default=DEFAULT_SLAVE_ID,
                        help=f"Revo2 slave id(默认 {DEFAULT_SLAVE_ID})")
    parser.add_argument("--topic", default=HAND_TOPIC, help=f"订阅目标话题(默认 {HAND_TOPIC})")
    parser.add_argument("--sdk-python-dir", type=Path, default=_default_sdk_python_dir(),
                        help="BrainCo SDK python 目录，默认使用 third_party/brainco-hand-sdk/python")
    parser.add_argument("--speed", type=int, default=DEFAULT_SPEED,
                        help=">0 时使用 set_finger_positions_and_speeds；默认 0 使用普通位置接口")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_arg_parser().parse_args(argv)

    import rclpy
    from std_msgs.msg import Float64MultiArray

    controller = AsyncRevo2Client(
        iface=args.iface,
        slave_id=args.slave_id,
        sdk_python_dir=args.sdk_python_dir,
        speed=args.speed,
    )
    rclpy.init()
    node = rclpy.create_node("revo2_hand_node")

    try:
        node.get_logger().info(
            f"连接 Revo2: iface={args.iface}, slave_id={args.slave_id}, sdk={args.sdk_python_dir}"
        )
        controller.start()
        node.get_logger().info(f"Revo2 手节点已就绪，订阅 {args.topic}")

        def on_target(msg: Float64MultiArray) -> None:
            try:
                positions = normalize_revo2_positions(msg.data)
            except (TypeError, ValueError) as exc:
                node.get_logger().error(f"忽略非法 Revo2 命令: {exc}")
                return

            future = controller.send_positions(positions)

            def on_done(done: Future) -> None:
                try:
                    done.result()
                except Exception as exc:  # noqa: BLE001
                    node.get_logger().error(f"Revo2 命令下发失败: {exc}")
                else:
                    node.get_logger().info(f"Revo2 命令已下发: {positions}")

            future.add_done_callback(on_done)

        node.create_subscription(Float64MultiArray, args.topic, on_target, 10)
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("用户中断 Revo2 手节点")
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(f"Revo2 手节点退出: {exc}")
        return 1
    finally:
        controller.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
