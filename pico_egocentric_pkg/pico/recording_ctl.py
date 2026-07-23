"""Remote control for Pico egocentric recording (start / stop / status).

Connects to the RecordingControlServer TCP socket running on the Pico
device and sends newline-delimited JSON commands.

Env vars
--------
PICO_DEVICE_HOST  Pico device IP (required unless --host is given)
PICO_RECORD_PORT  TCP port, default 9877
"""
import argparse
import json
import os
import socket
import sys
import time


DEFAULT_PORT = 9877


class PicoRecordingClient:
    def __init__(self, host: str, port: int = DEFAULT_PORT, timeout: float = 5.0):
        self._host = host
        self._port = port
        self._timeout = timeout
        self._sock: socket.socket | None = None

    def connect(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.settimeout(self._timeout)
        self._sock.connect((self._host, self._port))
        self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._buf = b""

    def close(self):
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *exc):
        self.close()

    def _send(self, cmd: dict) -> dict:
        if self._sock is None:
            raise ConnectionError("not connected")
        payload = json.dumps(cmd, separators=(",", ":")) + "\n"
        self._sock.sendall(payload.encode())
        return self._recv_line()

    def _recv_line(self) -> dict:
        while b"\n" not in self._buf:
            chunk = self._sock.recv(4096)
            if not chunk:
                raise ConnectionError("connection closed by Pico")
            self._buf += chunk
        line, self._buf = self._buf.split(b"\n", 1)
        return json.loads(line)

    def start_recording(self) -> dict:
        return self._send({"cmd": "start_recording"})

    def stop_recording(self) -> dict:
        return self._send({"cmd": "stop_recording"})

    def status(self) -> dict:
        return self._send({"cmd": "status"})

    def ping(self) -> float:
        t0 = time.monotonic()
        self._send({"cmd": "ping"})
        return (time.monotonic() - t0) * 1000.0


def main():
    parser = argparse.ArgumentParser(
        description="Remote control Pico egocentric recording",
    )
    parser.add_argument(
        "command",
        choices=["start", "stop", "status", "ping"],
        help="Command to send",
    )
    parser.add_argument(
        "--host",
        default=os.environ.get("PICO_DEVICE_HOST", ""),
        help="Pico device IP (or set PICO_DEVICE_HOST env var)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=int(os.environ.get("PICO_RECORD_PORT", str(DEFAULT_PORT))),
        help=f"TCP port (default {DEFAULT_PORT})",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=5.0,
        help="Socket timeout in seconds (default 5)",
    )
    args = parser.parse_args()

    if not args.host:
        print("error: --host or PICO_DEVICE_HOST is required", file=sys.stderr)
        return 1

    cmd_map = {
        "start": "start_recording",
        "stop": "stop_recording",
        "status": "status",
        "ping": "ping",
    }

    try:
        with PicoRecordingClient(args.host, args.port, args.timeout) as client:
            if args.command == "ping":
                rtt = client.ping()
                print(f"pong  rtt={rtt:.1f}ms")
            else:
                resp = getattr(client, cmd_map[args.command])()
                print(json.dumps(resp, indent=2))
    except ConnectionRefusedError:
        print(f"error: connection refused at {args.host}:{args.port}", file=sys.stderr)
        return 1
    except ConnectionError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    except socket.timeout:
        print(f"error: timeout connecting to {args.host}:{args.port}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
