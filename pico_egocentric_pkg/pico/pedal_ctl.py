"""Foot-pedal triggered Pico recording control.

Listens for global keyboard hotkeys (from a USB foot pedal mapped to
keyboard shortcuts) and sends start/stop commands to the Pico device.

Default mapping (configurable via CLI):
  Ctrl+Space  →  start recording
  Ctrl+Right  →  stop recording

Usage:
    python3 -m pico_egocentric_pkg.pico.pedal_ctl --host <PICO_IP>
    # or
    PICO_DEVICE_HOST=<PICO_IP> python3 -m pico_egocentric_pkg.pico.pedal_ctl
"""
import argparse
import os
import sys
import threading
import time

from pynput import keyboard

from .recording_ctl import PicoRecordingClient

DEFAULT_PORT = 9877


class PedalController:
    def __init__(self, host: str, port: int, timeout: float):
        self._host = host
        self._port = port
        self._timeout = timeout
        self._recording = False
        self._lock = threading.Lock()
        self._episode = 0

    def _send_cmd(self, action: str):
        with self._lock:
            try:
                with PicoRecordingClient(self._host, self._port, self._timeout) as c:
                    if action == "start":
                        resp = c.start_recording()
                        self._recording = True
                        self._episode += 1
                        print(f"\033[92m● REC #{self._episode}  started\033[0m  {resp}")
                    else:
                        resp = c.stop_recording()
                        self._recording = False
                        print(f"\033[93m■ STOP #{self._episode}\033[0m  {resp}")
            except Exception as e:
                print(f"\033[91m✗ {action} failed: {e}\033[0m")

    def start(self):
        if self._recording:
            print("  (already recording, ignored)")
            return
        self._send_cmd("start")

    def stop(self):
        if not self._recording:
            print("  (not recording, ignored)")
            return
        self._send_cmd("stop")


def main():
    parser = argparse.ArgumentParser(description="Foot-pedal Pico recording control")
    parser.add_argument(
        "--host",
        default=os.environ.get("PICO_DEVICE_HOST", ""),
        help="Pico device IP (or set PICO_DEVICE_HOST)",
    )
    parser.add_argument(
        "--port", type=int,
        default=int(os.environ.get("PICO_RECORD_PORT", str(DEFAULT_PORT))),
    )
    parser.add_argument("--timeout", type=float, default=5.0)
    args = parser.parse_args()

    if not args.host:
        print("error: --host or PICO_DEVICE_HOST required", file=sys.stderr)
        return 1

    ctl = PedalController(args.host, args.port, args.timeout)

    # verify connection
    try:
        with PicoRecordingClient(args.host, args.port, args.timeout) as c:
            rtt = c.ping()
        print(f"Connected to Pico @ {args.host}:{args.port}  (rtt={rtt:.0f}ms)")
    except Exception as e:
        print(f"error: cannot reach Pico: {e}", file=sys.stderr)
        return 1

    pressed_keys = set()

    def on_press(key):
        pressed_keys.add(key)

        ctrl_held = (
            keyboard.Key.ctrl_l in pressed_keys
            or keyboard.Key.ctrl_r in pressed_keys
        )
        if not ctrl_held:
            return

        if key == keyboard.Key.space:
            ctl.start()
        elif key == keyboard.Key.right:
            ctl.stop()

    def on_release(key):
        pressed_keys.discard(key)

    print(
        "Pedal mapping:\n"
        "  Ctrl+Space  →  START recording\n"
        "  Ctrl+Right  →  STOP  recording\n"
        "  Ctrl+C      →  exit\n"
        "\nWaiting for pedal input..."
    )

    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        try:
            listener.join()
        except KeyboardInterrupt:
            print("\nCtrl+C received, exiting.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
