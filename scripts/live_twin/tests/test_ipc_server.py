import http.client
import json
import os
import socket
import tempfile
import threading
import time
import unittest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import ipc_server


class FakeBridge:
    stop = threading.Event()
    def __init__(self):
        self.sequence = 1
        self.lock = threading.Condition()

    def snapshot(self):
        joints = [0.0] * 15
        targets = [0.1] * 15
        return {
            "sequence": self.sequence,
            "source": "robotd-ipc",
            "connection": "live",
            "age_ms": 20,
            "motors": [
                {"id": motor_id, "name": name, "label": label, "online": True,
                 "calibrated": True, "angle_rad": joints[i], "target_rad": targets[i],
                 "torque": None, "torque_known": False}
                for i, (motor_id, name, label) in enumerate(ipc_server.MOTORS)
            ],
            "read_hz": 10,
            "last_error": None,
            "health": {"healthy": True},
            "hello": {"api_version": 28},
            "robot_state": {"joints": joints, "targets": targets},
            "read_only": True,
            "control": {"enabled": False},
        }


class IpcServerTests(unittest.TestCase):
    def setUp(self):
        self.bridge = FakeBridge()
        self.server = ipc_server.ThreadingHTTPServer(
            ("127.0.0.1", 0), ipc_server.handler_for(self.bridge, None)
        )
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.port = self.server.server_address[1]

    def tearDown(self):
        self.bridge.stop.set()
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(1)

    def request(self, method, path):
        connection = http.client.HTTPConnection("127.0.0.1", self.port, timeout=2)
        connection.request(method, path)
        response = connection.getresponse()
        body = response.read()
        connection.close()
        return response.status, body

    def test_state_is_mapped_read_only_without_torque_claims(self):
        status, body = self.request("GET", "/api/state")
        self.assertEqual(status, 200)
        state = json.loads(body)
        self.assertEqual(state["source"], "robotd-ipc")
        self.assertTrue(state["read_only"])
        self.assertEqual(len(state["motors"]), 15)
        self.assertIsNone(state["motors"][9]["torque"])
        self.assertEqual(state["motors"][9]["id"], 34)
        self.assertEqual(state["motors"][9]["target_rad"], 0.1)

    def test_page_and_post_are_safe_viewer_operations(self):
        status, body = self.request("GET", "/")
        self.assertEqual(status, 200)
        self.assertIn("只读".encode(), body)
        status, body = self.request("POST", "/api/control/home")
        self.assertEqual(status, 405)
        self.assertIn(b"read-only", body)

    def test_unknown_route_is_not_a_control_fallback(self):
        status, _ = self.request("GET", "/api/control/home")
        self.assertEqual(status, 404)

    def test_bridge_subscribes_without_opening_a_uart(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "robotd.sock")
            ready = threading.Event()
            stop = threading.Event()

            def robotd():
                listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                listener.bind(path)
                listener.listen(1)
                ready.set()
                connection, _ = listener.accept()
                stream = connection.makefile("rwb", buffering=0)
                try:
                    for line in stream:
                        request = json.loads(line)
                        method = request["method"]
                        result = {
                            "hello": {"api_version": 28, "daemon_version": "test"},
                            "robot.health": {"healthy": True},
                            "robot.subscribe": {"accepted": True},
                        }[method]
                        stream.write((json.dumps({"jsonrpc": "2.0", "id": request["id"], "result": result}) + "\n").encode())
                        if method == "robot.subscribe":
                            stream.write((json.dumps({"jsonrpc": "2.0", "method": "robot.state", "params": {"joints": [0.0] * 15, "targets": [0.1] * 15, "loop": {"hz": 50}, "policy": "held"}}) + "\n").encode())
                            while not stop.wait(.05):
                                stream.write((json.dumps({"jsonrpc": "2.0", "method": "robot.state", "params": {"joints": [0.0] * 15, "targets": [0.1] * 15, "loop": {"hz": 50}, "policy": "held"}}) + "\n").encode())
                finally:
                    stream.close(); connection.close(); listener.close()

            thread = threading.Thread(target=robotd, daemon=True)
            thread.start(); self.assertTrue(ready.wait(1))
            bridge = ipc_server.Bridge(path, 10)
            try:
                deadline = time.monotonic() + 2
                while bridge.robot_state is None and time.monotonic() < deadline:
                    time.sleep(.01)
                self.assertIsNotNone(bridge.robot_state)
                snapshot = bridge.snapshot()
                self.assertEqual(snapshot["connection"], "live")
                self.assertEqual(snapshot["read_hz"], 50)
                self.assertEqual(snapshot["motors"][5]["angle_rad"], 0.0)
                self.assertIsNone(snapshot["motors"][5]["torque"])
            finally:
                bridge.close(); stop.set(); thread.join(1)


if __name__ == "__main__":
    unittest.main()
