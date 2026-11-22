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

import signal

import rclpy
from rclpy.executors import ExternalShutdownException
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger

from rcm_msgs.msg import ArmState, BendCommand, BendState, FlexState, ImuState, TendonGoal
from rcm_msgs.srv import Respond, SetTorque, SetZero, StartTest

def jdump(obj) -> str:
    """JSON for the browser: NaN/inf are not valid JSON (JSON.parse throws and the UI freezes) -> null."""
    def clean(x):
        if isinstance(x, float) and (x != x or x in (float("inf"), float("-inf"))):
            return None
        if isinstance(x, dict):
            return {k: clean(v) for k, v in x.items()}
        if isinstance(x, (list, tuple)):
            return [clean(v) for v in x]
        return x
    return json.dumps(clean(obj), default=str, allow_nan=False)


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
        self.imu: dict = {}
        self.flex: dict = {}
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
        self.create_subscription(ImuState, "/rcm/imu/state", self.on_imu, 20, callback_group=cg)
        self.create_subscription(FlexState, "/rcm/flex/state", self.on_flex, 10, callback_group=cg)
        self.bend_pub = self.create_publisher(BendCommand, "/rcm/cmd_bend", 10)
        self.goal_pub = self.create_publisher(TendonGoal, "/rcm/goal", 10)
        c = lambda t, n: self.create_client(t, n, callback_group=cg)  # noqa: E731
        self.cli = {
            "torque": c(SetTorque, "/rcm/torque"), "estop": c(Trigger, "/rcm/estop"),
            "rearm": c(Trigger, "/rcm/rearm"), "reload_cal": c(Trigger, "/rcm/kinematics/reload"),
            "rest": c(Trigger, "/rcm/rest"),
            "set_home": c(Trigger, "/rcm/set_home"), "set_zero": c(SetZero, "/rcm/set_zero"),
            "imu/set_reference": c(Trigger, "/rcm/imu/set_reference"),
            "flex/set_flat": c(Trigger, "/rcm/flex/set_flat"), "flex/set_90": c(Trigger, "/rcm/flex/set_90"),
            "imu/clear_reference": c(Trigger, "/rcm/imu/clear_reference"),
            "imu/calibrate_gyro": c(Trigger, "/rcm/imu/calibrate_gyro"),
            "test/list": c(Trigger, "/rcm/test/list"), "test/start": c(StartTest, "/rcm/test/start"),
            "test/respond": c(Respond, "/rcm/test/respond"), "test/abort": c(Trigger, "/rcm/test/abort"),
        }

    # ---------------- ROS -> snapshot ----------------
    def on_state(self, m: ArmState):
        self.arm = {
            "t": time.time(), "tripped": m.tripped, "torque": m.torque_enabled, "sim": m.simulated,
            "connected": m.connected,
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

    def on_imu(self, m: ImuState):
        f = lambda x: None if x != x else round(x, 2)  # noqa: E731  NaN -> null for JSON
        o, r = m.orientation, m.relative
        self.imu = {"t": time.time(), "state": m.state, "message": m.message, "rate": round(m.rate_hz, 1),
                    "q": [o.w, o.x, o.y, o.z], "q_rel": [r.w, r.x, r.y, r.z], "has_ref": m.has_reference,
                    "roll": f(m.roll_deg), "pitch": f(m.pitch_deg), "tilt": f(m.tilt_from_ref_deg),
                    "accel": [round(m.accel.x, 3), round(m.accel.y, 3), round(m.accel.z, 3)],
                    "gyro_dps": [round(math.degrees(v), 2) for v in (m.gyro.x, m.gyro.y, m.gyro.z)],
                    "temp": f(m.temperature), "bad": m.bad_lines}

    def on_flex(self, m: FlexState):
        r = lambda x, d=1: None if x != x else round(x, d)  # noqa: E731
        fr, to = list(m.from_cm), list(m.to_cm)
        self.flex = {"t": time.time(), "state": m.state, "message": m.message, "rate": round(m.rate_hz),
                     "total": r(m.total_deg), "covered_cm": r(m.covered_cm), "length_cm": r(m.length_cm),
                     "sensors": [{"name": n, "v": r(v, 3), "r_k": r(rr / 1000.0, 2), "angle": r(a), "cal": bool(c),
                                  "from": r(fr[k]) if k < len(fr) else None, "to": r(to[k]) if k < len(to) else None}
                                 for k, (n, v, rr, a, c) in enumerate(zip(m.names, m.volts, m.resistance_ohm,
                                                                          m.angle_deg, m.calibrated))]}

    def on_notice(self, m: String):
        self.notice_n += 1
        self.notices = (self.notices + [{"n": self.notice_n, "text": m.data}])[-20:]

    def on_info(self, m: String):
        self.info = json.loads(m.data)

    def snapshot(self, with_info: bool) -> dict:
        d = {"arm": self.arm, "bend": self.bend, "test": self.test, "now": time.time(), "notices": self.notices,
             "imu": self.imu, "flex": self.flex}
        if with_info:
            d["info"] = self.info
        return d

    # ---------------- HTTP -> ROS ----------------
    def call(self, name: str, req, timeout: float = 3.0):
        cli = self.cli[name]
        if not cli.service_is_ready() and not cli.wait_for_service(timeout_sec=5.0):
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
        if action == "set_home":
            if (self.test or {}).get("state") in ("running", "prompt"):
                return {"ok": False, "message": "a test is running -- abort it first"}
            r = self.call("set_home", Trigger.Request(), timeout=10.0)
            if not r["ok"]:
                return r
            h = json.loads(r["message"])
            z = self.call("set_zero", SetZero.Request(ids=h["ids"], counts=[int(a) for a in h["after"]]))
            if not z["ok"]:
                return {"ok": False, "message": f"servos re-centred but saving home failed: {z['message']}"}
            return {"ok": True, "message": f"home saved: servos re-centred {h['before']} -> {h['after']} (holding)"}
        if action in ("estop", "rearm", "rest", "reload_cal", "test/abort",
                      "imu/set_reference", "imu/clear_reference", "imu/calibrate_gyro",
                      "flex/set_flat", "flex/set_90"):
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
            data = jdump(obj).encode()
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
                    msg = jdump(node.snapshot(with_info=(k % 40 == 0)))
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
    def spin():
        try:
            ex.spin()
        except Exception:  # noqa: BLE001 — Ctrl+C shuts the context down under the spinning thread
            pass
    spinner = threading.Thread(target=spin, daemon=True)
    spinner.start()
    try:
        srv = ThreadingHTTPServer((node.host, node.port), make_handler(node))
    except OSError as e:
        node.get_logger().fatal(f"cannot bind {node.host}:{node.port} ({e}) — another dashboard running? "
                                f"use dashboard port param or: fuser -k {node.port}/tcp")
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        rclpy.try_shutdown()
        return
    srv.daemon_threads = True
    node.get_logger().info(f"dashboard: http://{'localhost' if node.host == '127.0.0.1' else node.host}:{node.port}")
    try:
        srv.serve_forever()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)   # 2nd SIGINT (from ros2 launch) must not cut cleanup short
        srv.server_close()
        ex.shutdown(timeout_sec=1.0)   # stop spinning BEFORE destroying the node (else C++ abort, exit -6)
        spinner.join(timeout=1.0)
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
