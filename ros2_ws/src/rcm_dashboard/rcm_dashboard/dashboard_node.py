"""dashboard: local web UI for the RCM stack. Pure stdlib HTTP + Server-Sent Events.

Open http://localhost:8080  (param host:=0.0.0.0 to allow other laptops on the LAN)

GET  /events          SSE stream (~20 Hz): servo state, bend estimate, test status, kinematics info
POST /api/<action>    JSON body -> ROS publish / service call; returns {"ok", "message", ...}
     bend {theta_deg, phi_deg, speed} | raw {id, pos, speed} | torque {id, enable}
     estop | rearm | rest | reload_cal
     test/list | test/start {protocol, overrides, notes} | test/respond {value} | test/abort
"""
from __future__ import annotations

import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger

from rcm_msgs.msg import ArmState, BendCommand, BendState, TendonGoal
from rcm_msgs.srv import Respond, SetTorque, StartTest

MIME = {".html": "text/html", ".js": "application/javascript", ".css": "text/css"}


class Dashboard(Node):
    def __init__(self):
        super().__init__("dashboard")
        self.host = self.declare_parameter("host", "127.0.0.1").value
        self.port = self.declare_parameter("port", 8080).value
        web = self.declare_parameter("web_dir", "").value
        self.web_dir = Path(web) if web else Path(get_package_share_directory("rcm_dashboard")) / "web"

        self.arm: dict = {}
        self.bend: dict = {}
        self.test: dict = {}
        self.info: dict = {}
        self.seq = 0
        self.notices: list[dict] = []          # [{"n": counter, "text": ...}], last 20
        self.notice_n = 0

        cg = ReentrantCallbackGroup()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(ArmState, "/rcm/state", self.on_state, 10, callback_group=cg)
        self.create_subscription(BendState, "/rcm/bend_estimate", self.on_bend, 10, callback_group=cg)
        self.create_subscription(String, "/rcm/test/status", self.on_test, 10, callback_group=cg)
        self.create_subscription(String, "/rcm/kinematics/info", self.on_info, latched, callback_group=cg)
        self.create_subscription(String, "/rcm/notice", self.on_notice, 20, callback_group=cg)
        self.bend_pub = self.create_publisher(BendCommand, "/rcm/cmd_bend", 10)
        self.goal_pub = self.create_publisher(TendonGoal, "/rcm/goal", 10)
        c = lambda t, n: self.create_client(t, n, callback_group=cg)  # noqa: E731
        self.cli = {
            "torque": c(SetTorque, "/rcm/torque"), "estop": c(Trigger, "/rcm/estop"),
            "rearm": c(Trigger, "/rcm/rearm"), "reload_cal": c(Trigger, "/rcm/kinematics/reload"),
            "rest": c(Trigger, "/rcm/rest"),
            "test/list": c(Trigger, "/rcm/test/list"), "test/start": c(StartTest, "/rcm/test/start"),
            "test/respond": c(Respond, "/rcm/test/respond"), "test/abort": c(Trigger, "/rcm/test/abort"),
        }

    # ---------------- ROS -> snapshot ----------------
    def on_state(self, m: ArmState):
        self.arm = {
            "t": time.time(), "tripped": m.tripped, "torque": m.torque_enabled, "sim": m.simulated,
            "servos": [dict(id=s.id, pos=s.pos, goal=s.goal, speed=s.speed, load=s.load, voltage=round(s.voltage, 1),
                            temperature=s.temperature, current=s.current, moving=s.moving, error=s.error)
                       for s in m.servos],
        }
        self.seq += 1

    def on_bend(self, m: BendState):
        self.bend = {"theta_deg": math.degrees(m.theta), "phi_deg": math.degrees(m.phi),
                     "tip": [m.tip.x, m.tip.y, m.tip.z], "common_mm": m.common_mode_mm,
                     "dl_mm": list(m.tendon_dl_mm), "cmd_theta_deg": math.degrees(m.cmd_theta),
                     "cmd_phi_deg": math.degrees(m.cmd_phi)}

    def on_test(self, m: String):
        self.test = json.loads(m.data)

    def on_notice(self, m: String):
        self.notice_n += 1
        self.notices = (self.notices + [{"n": self.notice_n, "text": m.data}])[-20:]

    def on_info(self, m: String):
        self.info = json.loads(m.data)

    def snapshot(self, with_info: bool) -> dict:
        d = {"arm": self.arm, "bend": self.bend, "test": self.test, "now": time.time(), "notices": self.notices}
        if with_info:
            d["info"] = self.info
        return d

    # ---------------- HTTP -> ROS ----------------
    def call(self, name: str, req, timeout: float = 3.0):
        cli = self.cli[name]
        if not cli.service_is_ready() and not cli.wait_for_service(timeout_sec=1.0):
            return {"ok": False, "message": f"service for '{name}' not available (node running?)"}
        fut = cli.call_async(req)
        done = threading.Event()
        fut.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout):
            return {"ok": False, "message": f"'{name}' timed out"}
        r = fut.result()
        out = {"ok": bool(r.success), "message": r.message}
        if hasattr(r, "session_dir"):
            out["session_dir"] = r.session_dir
        return out

    def api(self, action: str, b: dict) -> dict:
        if action == "bend":
            self.bend_pub.publish(BendCommand(theta=math.radians(float(b["theta_deg"])),
                                              phi=math.radians(float(b.get("phi_deg", 0))),
                                              speed=int(b.get("speed", 200))))
            return {"ok": True, "message": "sent"}
        if action == "raw":
            self.goal_pub.publish(TendonGoal(ids=[int(b["id"])], positions=[int(b["pos"])],
                                             speed=int(b.get("speed", 150))))
            return {"ok": True, "message": "sent"}
        if action == "torque":
            return self.call("torque", SetTorque.Request(id=int(b.get("id", 0)), enable=bool(b.get("enable"))))
        if action in ("estop", "rearm", "rest", "reload_cal", "test/abort"):
            return self.call(action, Trigger.Request())
        if action == "test/list":
            r = self.call("test/list", Trigger.Request())
            if r["ok"]:
                r["protocols"] = json.loads(r.pop("message"))
            return r
        if action == "test/start":
            return self.call("test/start", StartTest.Request(protocol=b["protocol"],
                                                              overrides_yaml=b.get("overrides", ""),
                                                              notes=b.get("notes", "")))
        if action == "test/respond":
            return self.call("test/respond", Respond.Request(value=str(b.get("value", ""))))
        return {"ok": False, "message": f"unknown action '{action}'"}


def make_handler(node: Dashboard):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):        # keep the terminal quiet
            pass

        def _json(self, obj, code=200):
            data = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            path = self.path.split("?")[0]
            if path == "/events":
                return self._sse()
            name = "index.html" if path in ("/", "") else path.lstrip("/")
            rel = Path(name)
            f = node.web_dir / rel          # files may be symlinks (colcon --symlink-install)
            if rel.is_absolute() or ".." in rel.parts or not f.is_file():
                self.send_error(404)
                return
            data = f.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", MIME.get(f.suffix, "application/octet-stream"))
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _sse(self):
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            k = 0
            try:
                while rclpy.ok():
                    msg = json.dumps(node.snapshot(with_info=(k % 40 == 0)), default=str)
                    self.wfile.write(f"data: {msg}\n\n".encode())
                    self.wfile.flush()
                    k += 1
                    time.sleep(0.05)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_POST(self):
            if not self.path.startswith("/api/"):
                self.send_error(404)
                return
            n = int(self.headers.get("Content-Length") or 0)
            try:
                body = json.loads(self.rfile.read(n) or b"{}")
                self._json(node.api(self.path[len("/api/"):], body))
            except (KeyError, ValueError, TypeError) as e:
                self._json({"ok": False, "message": f"bad request: {e}"}, 400)
    return H


def main(args=None):
    rclpy.init(args=args)
    node = Dashboard()
    ex = SingleThreadedExecutor()      # HTTP threads wait on futures; executor just services ROS
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    try:
        srv = ThreadingHTTPServer((node.host, node.port), make_handler(node))
    except OSError as e:
        node.get_logger().fatal(f"cannot bind {node.host}:{node.port} ({e}) — another dashboard running? "
                                f"use dashboard port param or: fuser -k {node.port}/tcp")
        node.destroy_node()
        rclpy.try_shutdown()
        return
    srv.daemon_threads = True
    node.get_logger().info(f"dashboard: http://{'localhost' if node.host == '127.0.0.1' else node.host}:{node.port}")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
