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
    def __init__(self):
        self.stop = threading.Event()
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
            "hello": {"api_version": 37},
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
        self.assertIn(b'data-read-only="true"', body)
        self.assertIn(b'src="/app.js"', body)
        status, body = self.request("POST", "/api/control/home")
        self.assertEqual(status, 405)
        self.assertIn(b"read-only", body)

    def test_unknown_route_is_not_a_control_fallback(self):
        status, _ = self.request("GET", "/api/control/home")
        self.assertEqual(status, 404)

    def test_three_modules_and_every_model_mesh_are_available_offline(self):
        for path in ("/app.js", "/rig.js", "/state.js",
                     "/vendor/three/three.module.js", "/vendor/three/OrbitControls.js",
                     "/vendor/three/STLLoader.js"):
            connection = http.client.HTTPConnection("127.0.0.1", self.port, timeout=2)
            connection.request("GET", path)
            response = connection.getresponse()
            self.assertEqual(response.status, 200, path)
            self.assertIn("javascript", response.getheader("Content-Type"), path)
            self.assertTrue(response.read(), path)
            connection.close()
        status, body = self.request("GET", "/assets/model.json")
        self.assertEqual(status, 200)
        def check_meshes(node):
            for geom in node["geoms"]:
                status, body = self.request("GET", "/assets/meshes/" + geom["mesh"])
                self.assertEqual(status, 200, geom["mesh"])
                self.assertGreater(len(body), 84)
            for child in node["children"]:
                check_meshes(child)
        check_meshes(json.loads(body)["root"])

    def test_static_paths_cannot_escape_the_web_root(self):
        for path in ("/../ipc_server.py", "/%2e%2e/ipc_server.py", "/assets/%2e%2e/%2e%2e/ipc_server.py", "/%00", "/assets/"):
            self.assertEqual(self.request("GET", path)[0], 404, path)

    def test_sse_messages_carry_viewer_state(self):
        connection = http.client.HTTPConnection("127.0.0.1", self.port, timeout=2)
        connection.request("GET", "/api/events")
        response = connection.getresponse()
        self.assertEqual(response.getheader("Content-Type"), "text/event-stream")
        line = response.readline()
        self.assertTrue(line.startswith(b"data: "))
        self.assertTrue(json.loads(line[6:])["read_only"])
        connection.close()

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
                            "hello": {"api_version": 37, "daemon_version": "test"},
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
                while len(bridge.received_times) < 2 and time.monotonic() < deadline:
                    time.sleep(.01)
                self.assertIsNotNone(bridge.robot_state)
                snapshot = bridge.snapshot()
                self.assertEqual(snapshot["connection"], "live")
                self.assertGreater(snapshot["read_hz"], 0)
                self.assertEqual(snapshot["robot_state"]["loop"]["hz"], 50)
                self.assertEqual(snapshot["motors"][5]["angle_rad"], 0.0)
                self.assertIsNone(snapshot["motors"][5]["torque"])
            finally:
                bridge.close(); stop.set(); thread.join(1)
            bridge.publish(last_state_at=time.monotonic() - 2)
            snapshot = bridge.snapshot()
            self.assertEqual(snapshot["connection"], "stale")
            self.assertTrue(all(not motor["online"] for motor in snapshot["motors"]))
            self.assertEqual(snapshot["read_hz"], 0)


if __name__ == "__main__":
    unittest.main()
