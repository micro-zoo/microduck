#!/usr/bin/env python3
"""Read-only Live Twin bridge for robotd's JSON-RPC state stream.

This process never opens the Dynamixel UART and never sends a robot intent. It is
safe to run beside robotd: robotd remains the only motor-bus owner.
"""
import argparse
import json
import math
import mimetypes
import select
import socket
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote

MAX_LINE = 64 * 1024
STATIC_DIR = Path(__file__).resolve().parent / "dist"
MOTORS = (
    (20, "left_hip_yaw", "左髋 · YAW"), (21, "left_hip_roll", "左髋 · ROLL"),
    (22, "left_hip_pitch", "左髋 · PITCH"), (23, "left_knee", "左膝"),
    (24, "left_ankle", "左踝"), (30, "neck_pitch", "颈部 · PITCH"),
    (31, "head_pitch", "头部 · PITCH"), (32, "head_yaw", "头部 · YAW"),
    (33, "head_roll", "头部 · ROLL"), (34, "mouth", "嘴部"),
    (10, "right_hip_yaw", "右髋 · YAW"), (11, "right_hip_roll", "右髋 · ROLL"),
    (12, "right_hip_pitch", "右髋 · PITCH"), (13, "right_knee", "右膝"),
    (14, "right_ankle", "右踝"),
)

def finite(value):
    return isinstance(value, (int, float)) and math.isfinite(value)


class Bridge:
    def __init__(self, socket_path, hz):
        self.socket_path = str(socket_path)
        self.hz = hz
        self.lock = threading.Condition()
        self.sequence = 0
        self.connection = "connecting"
        self.last_error = None
        self.last_state_at = None
        self.received_times = deque(maxlen=100)
        self.hello = None
        self.health = None
        self.robot_state = None
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.run, name="robotd-ipc", daemon=True)
        self.thread.start()

    def publish(self, **values):
        with self.lock:
            for key, value in values.items():
                setattr(self, key, value)
            if "last_state_at" in values:
                self.received_times.append(values["last_state_at"])
            self.sequence += 1
            self.lock.notify_all()

    def send(self, sock, number, method, params):
        request = {"jsonrpc": "2.0", "id": number, "method": method, "params": params}
        sock.sendall((json.dumps(request, separators=(",", ":")) + "\n").encode())

    def messages(self, sock, buffer, timeout):
        ready = select.select([sock], [], [], timeout)[0]
        if not ready:
            return buffer, []
        chunk = sock.recv(65536)
        if not chunk:
            raise ConnectionError("robotd closed the IPC socket")
        buffer += chunk
        if len(buffer) > MAX_LINE * 2:
            raise ValueError("robotd IPC buffer is too large")
        result = []
        while b"\n" in buffer:
            line, buffer = buffer.split(b"\n", 1)
            if len(line) > MAX_LINE:
                raise ValueError("robotd IPC line is too large")
            if line:
                result.append(json.loads(line))
        return buffer, result

    def request(self, sock, buffer, number, method, params):
        self.send(sock, number, method, params)
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            buffer, values = self.messages(sock, buffer, max(0, deadline - time.monotonic()))
            for value in values:
                if value.get("method") == "robot.state":
                    self.publish(robot_state=value.get("params"), last_state_at=time.monotonic(), connection="live", last_error=None)
                if value.get("id") == number:
                    if "error" in value:
                        raise RuntimeError(value["error"].get("message", "robotd RPC error"))
                    return buffer, value.get("result")
        raise TimeoutError(f"robotd did not answer {method}")

    def run(self):
        while not self.stop.is_set():
            try:
                with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
                    sock.setblocking(True)
                    sock.settimeout(3)
                    sock.connect(self.socket_path)
                    buffer = b""
                    buffer, self.hello = self.request(sock, buffer, 1, "hello", {"api_version": 37})
                    buffer, self.health = self.request(sock, buffer, 2, "robot.health", {})
                    buffer, _ = self.request(sock, buffer, 3, "robot.subscribe", {"hz": self.hz})
                    sock.settimeout(None)
                    self.publish(connection="live", last_error=None)
                    while not self.stop.is_set():
                        buffer, values = self.messages(sock, buffer, 1.0)
                        for value in values:
                            if value.get("method") == "robot.state":
                                self.publish(robot_state=value.get("params"), last_state_at=time.monotonic(), connection="live", last_error=None)
            except Exception as error:
                self.publish(connection="stale" if self.robot_state else "offline", last_error=str(error))
                self.stop.wait(1)

    def snapshot(self):
        with self.lock:
            state = self.robot_state if isinstance(self.robot_state, dict) else None
            age = None if self.last_state_at is None else (time.monotonic() - self.last_state_at) * 1000
            if age is not None and age > 1500 and self.connection == "live":
                connection = "stale"
            else:
                connection = self.connection
            if state is None and connection == "live":
                connection = "connecting"
            times = self.received_times
            read_hz = ((len(times) - 1) / (times[-1] - times[0])
                       if connection == "live" and len(times) > 1 and times[-1] > times[0] else 0)
            measured = state.get("joints", []) if state else []
            targets = state.get("targets", []) if state else []
            motors = []
            for index, (motor_id, name, label) in enumerate(MOTORS):
                q = measured[index] if index < len(measured) and finite(measured[index]) else None
                target = targets[index] if index < len(targets) and finite(targets[index]) else None
                motors.append({"id": motor_id, "name": name, "label": label, "group": "head" if 5 <= index <= 9 else ("left" if index < 5 else "right"), "online": connection == "live" and q is not None and age is not None and age < 1500, "calibrated": q is not None, "angle_rad": q, "angle_deg": math.degrees(q) if q is not None else None, "target_rad": target, "torque": None, "torque_known": False, "current_ma": None, "temperature_c": None, "voltage_v": None, "hardware_error": None, "status_error": None})
            return {"sequence": self.sequence, "source": "robotd-ipc", "connection": connection, "age_ms": age, "motors": motors, "read_hz": read_hz, "last_error": self.last_error, "health": self.health, "hello": self.hello, "robot_state": state, "read_only": True}

    def close(self):
        self.stop.set()
        self.thread.join(2)


def handler_for(bridge, static_dir=None):
    static_root = Path(static_dir or STATIC_DIR).resolve()

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *_):
            return

        def answer(self, body, status=200, content_type="application/json; charset=utf-8"):
            data = body if isinstance(body, bytes) else body.encode()
            self.send_response(status); self.send_header("Content-Type", content_type); self.send_header("Content-Length", str(len(data))); self.send_header("Cache-Control", "no-store"); self.end_headers(); self.wfile.write(data)

        def do_GET(self):
            route = self.path.split("?", 1)[0]
            if route == "/api/state":
                return self.answer(json.dumps(bridge.snapshot(), ensure_ascii=False, separators=(",", ":")))
            if route == "/api/events":
                self.send_response(200); self.send_header("Content-Type", "text/event-stream"); self.send_header("Cache-Control", "no-cache"); self.send_header("Connection", "close"); self.end_headers()
                previous = -1
                try:
                    while not bridge.stop.is_set():
                        with bridge.lock:
                            if previous == bridge.sequence: bridge.lock.wait(timeout=.8)
                            payload = bridge.snapshot(); previous = bridge.sequence
                        self.wfile.write(("data: " + json.dumps(payload, ensure_ascii=False, separators=(",", ":")) + "\n\n").encode()); self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    pass
                return
            if route.startswith("/api/"):
                return self.answer(json.dumps({"error": "not found"}), 404)
            try:
                relative = unquote(route).lstrip("/") or "index.html"
                path = (static_root / relative).resolve()
                if not path.is_relative_to(static_root) or not path.is_file():
                    return self.answer(json.dumps({"error": "not found"}), 404)
                body = path.read_bytes()
                content_type = mimetypes.guess_type(str(path))[0] or "application/octet-stream"
                return self.answer(body, content_type=content_type)
            except (OSError, ValueError):
                pass
            return self.answer(json.dumps({"error": "not found"}), 404)

        def do_POST(self):
            self.answer(json.dumps({"error": "read-only robotd IPC viewer"}), 405)

    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-socket", default="/run/robotd.sock")
    parser.add_argument("--listen", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--hz", type=int, default=10)
    parser.add_argument("--static-dir", type=Path, default=STATIC_DIR,
                        help="Live Twin static files (default: dist beside this script)")
    args = parser.parse_args()
    bridge = Bridge(args.robot_socket, max(1, min(args.hz, 50)))
    server = ThreadingHTTPServer((args.listen, args.port), handler_for(bridge, args.static_dir))
    try:
        server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.close(); server.server_close()


if __name__ == "__main__":
    main()
