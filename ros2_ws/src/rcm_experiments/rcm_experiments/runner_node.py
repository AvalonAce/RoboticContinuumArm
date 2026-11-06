"""test_runner: runs protocols (protocols.py) and logs every session.

Srv  /rcm/test/start    rcm_msgs/StartTest   protocol file stem + YAML overrides + notes
Srv  /rcm/test/respond  rcm_msgs/Respond     answer the current operator prompt
Srv  /rcm/test/abort    std_srvs/Trigger     stop the protocol; arm HOLDS position
Srv  /rcm/test/list     std_srvs/Trigger     JSON of available protocol files
Pub  /rcm/test/status   std_msgs/String      JSON status (5 Hz + on change)

Each run writes   <data_dir>/<YYYYmmdd_HHMMSS>_<stem>/
    telemetry.csv  one row per /rcm/state sample (all servos, bend estimate, command)
    events.csv     protocol marks (payload, hold_start, top, failure, ...) with JSON data
    meta.json      protocol, params, notes, geometry/calibration, result, timing
"""
from __future__ import annotations

import csv
import json
import math
import os
import threading
import time
import traceback
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger

from rcm_msgs.msg import ArmState, BendCommand, BendState, TendonGoal
from rcm_msgs.srv import Respond, SetZero, StartTest

from .context import Aborted, ProtocolError, TestContext, Tripped
from .protocols import PROTOCOLS, resolve


def safe(fn):
    """Never let a callback exception kill the node: log the traceback to the terminal AND to
    <data_dir>/runner_errors.log, publish a notice, and (for services) return a failure."""
    import functools

    @functools.wraps(fn)
    def wrap(self, *a):
        try:
            return fn(self, *a)
        except Exception as e:  # noqa: BLE001
            tb = traceback.format_exc()
            self.get_logger().error(f"{fn.__name__} crashed: {e!r}\n{tb}")
            try:
                self.data_dir.mkdir(parents=True, exist_ok=True)
                with open(self.data_dir / "runner_errors.log", "a") as f:
                    f.write(f"--- {time.strftime('%Y-%m-%d %H:%M:%S')} {fn.__name__}\n{tb}\n")
                self.notice_pub.publish(String(data=f"test_runner: {fn.__name__} error: {e!r}"))
            except Exception:  # noqa: BLE001
                pass
            if len(a) == 2 and hasattr(a[1], "success"):      # service (request, response)
                a[1].success, a[1].message = False, f"internal error: {e!r} (see runner_errors.log)"
                return a[1]
            return None
    return wrap


def default_data_dir() -> str:
    if os.environ.get("RCM_DATA"):
        return os.environ["RCM_DATA"]
    if os.environ.get("RCM_ROOT"):
        return str(Path(os.environ["RCM_ROOT"]) / "data")
    return str(Path.home() / "rcm_data")


class RosContext(TestContext):
    def __init__(self, node: "Runner"):
        self.n = node

    # primitives
    def state(self):
        with self.n.lock:
            return {k: dict(v) for k, v in self.n.servos.items()}

    def tripped(self):
        return self.n.trip_reason

    def aborted(self):
        return self.n.abort_flag.is_set()

    @property
    def geometry(self):
        return self.n.kin_info

    def _send_bend(self, theta, phi, speed):
        self.n.cmd = (theta, phi)
        self.n.bend_pub.publish(BendCommand(theta=theta, phi=phi, speed=speed))

    def _send_raw(self, targets, speed):
        self.n.goal_pub.publish(TendonGoal(ids=list(targets), positions=list(targets.values()), speed=speed))

    def _ask(self, text, kind, default):
        n = self.n
        n.answer_evt.clear()
        n.prompt_obj = {"text": text, "kind": kind, "default": default}
        n.set_state("prompt")
        try:
            while not n.answer_evt.wait(0.05):
                self.check()
            return n.answer
        finally:
            n.prompt_obj = None
            n.set_state("running")

    def set_zero(self, zeros):
        cli = self.n.zero_cli
        if not cli.wait_for_service(timeout_sec=2.0):
            raise ProtocolError("/rcm/set_zero unavailable (kinematics node running?)")
        fut = cli.call_async(SetZero.Request(ids=list(zeros), counts=list(zeros.values())))
        done = threading.Event()
        fut.add_done_callback(lambda _f: done.set())
        if not done.wait(5.0) or not fut.result().success:
            raise ProtocolError(f"set_zero failed: {fut.result().message if fut.done() else 'timeout'}")
        # wait for the latched info update so later bends use the new zeros
        t0 = time.monotonic()
        while time.monotonic() - t0 < 3.0:
            if [int(x) for x in self.n.kin_info.get("zero_counts", [])] == [zeros[i] for i in self.ids]:
                return
            time.sleep(0.05)
        raise ProtocolError("kinematics did not publish new calibration")

    def mark(self, event, **data):
        self.n.write_event(event, data)

    def log(self, msg):
        self.n.add_log(msg)


class Runner(Node):
    def __init__(self):
        super().__init__("test_runner")
        # Prefer the source tree (edit YAMLs without rebuilding); fall back to installed share
        src = Path(os.environ.get("RCM_ROOT", "/nonexistent")) / "ros2_ws/src/rcm_experiments/protocols"
        share = Path(get_package_share_directory("rcm_experiments")) / "protocols"
        self.protocols_dir = Path(self.declare_parameter("protocols_dir", "").value
                                  or (src if src.is_dir() else share))
        self.data_dir = Path(self.declare_parameter("data_dir", "").value or default_data_dir())

        self.lock = threading.Lock()
        self.servos: dict[int, dict] = {}
        self.bend_est: BendState | None = None
        self.kin_info: dict = {}
        self.trip_reason = ""
        self.cmd = (float("nan"), float("nan"))

        self.state = "idle"
        self.protocol = ""
        self.session_dir = ""
        self.logs: list[str] = []
        self.result: dict | None = None
        self.prompt_obj: dict | None = None
        self.answer = ""
        self.answer_evt = threading.Event()
        self.abort_flag = threading.Event()
        self.t_start = 0.0
        self.worker: threading.Thread | None = None
        self.tel_fh = self.tel = self.ev_fh = self.ev = None
        self.file_lock = threading.Lock()
        self.start_lock = threading.Lock()       # one start at a time (double-clicks)

        cg = ReentrantCallbackGroup()
        self.bend_pub = self.create_publisher(BendCommand, "/rcm/cmd_bend", 10)
        self.goal_pub = self.create_publisher(TendonGoal, "/rcm/goal", 10)
        self.status_pub = self.create_publisher(String, "/rcm/test/status", 10)
        self.notice_pub = self.create_publisher(String, "/rcm/notice", 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(ArmState, "/rcm/state", self.on_state, 20, callback_group=cg)
        self.create_subscription(BendState, "/rcm/bend_estimate", self.on_bend, 20, callback_group=cg)
        self.create_subscription(String, "/rcm/kinematics/info", self.on_info, latched, callback_group=cg)
        self.zero_cli = self.create_client(SetZero, "/rcm/set_zero", callback_group=cg)
        self.create_service(StartTest, "/rcm/test/start", self.on_start, callback_group=cg)
        self.create_service(Respond, "/rcm/test/respond", self.on_respond, callback_group=cg)
        self.create_service(Trigger, "/rcm/test/abort", self.on_abort, callback_group=cg)
        self.create_service(Trigger, "/rcm/test/list", self.on_list, callback_group=cg)
        self.create_timer(0.2, self.publish_status, callback_group=cg)
        self.get_logger().info(f"protocols: {self.protocols_dir}  data: {self.data_dir}")

    # ---------------- subscriptions ----------------
    @safe
    def on_info(self, msg: String):
        self.kin_info = json.loads(msg.data)

    def on_bend(self, msg: BendState):
        self.bend_est = msg

    @safe
    def on_state(self, msg: ArmState):
        with self.lock:
            for s in msg.servos:
                self.servos[s.id] = dict(pos=s.pos, speed=s.speed, load=s.load, voltage=s.voltage,
                                         temperature=s.temperature, current=s.current, moving=s.moving,
                                         error=s.error, goal=s.goal)
            self.trip_reason = msg.tripped
        if self.tel is not None:
            self.write_telemetry(msg)

    # ---------------- logging ----------------
    def _ids(self):
        return list(self.kin_info.get("ids") or sorted(self.servos))

    def open_session(self, stem: str) -> Path:
        base = f"{time.strftime('%Y%m%d_%H%M%S')}_{stem}"
        d, k = self.data_dir / base, 2
        while d.exists():                       # never reuse/truncate a session folder
            d, k = self.data_dir / f"{base}_{k}", k + 1
        d.mkdir(parents=True)
        ids = self._ids()
        cols = ["host_t", "stamp"]
        for i in ids:
            cols += [f"{k}_{i}" for k in ("pos", "goal", "speed", "load", "temp", "volt", "current", "moving", "err")]
        cols += ["est_theta_deg", "est_phi_deg", "tip_x", "tip_y", "tip_z", "common_mm",
                 "cmd_theta_deg", "cmd_phi_deg", "tripped"]
        with self.file_lock:
            self.tel_fh = open(d / "telemetry.csv", "w", newline="", buffering=1)
            self.tel = csv.writer(self.tel_fh)
            self.tel.writerow(cols)
            self.tel_ids = ids
            self.ev_fh = open(d / "events.csv", "w", newline="", buffering=1)
            self.ev = csv.writer(self.ev_fh)
            self.ev.writerow(["host_t", "elapsed_s", "event", "data"])
        return d

    def close_session(self):
        with self.file_lock:
            for fh in (self.tel_fh, self.ev_fh):
                if fh:
                    fh.close()
            self.tel = self.ev = self.tel_fh = self.ev_fh = None

    def write_telemetry(self, msg: ArmState):
        by = {s.id: s for s in msg.servos}
        row = [f"{time.time():.4f}", f"{msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9:.4f}"]
        for i in self.tel_ids:
            s = by.get(i)
            row += ([s.pos, s.goal, s.speed, s.load, s.temperature, f"{s.voltage:.1f}", s.current,
                     int(s.moving), int(s.error)] if s else [""] * 9)
        b = self.bend_est
        if b is not None:
            row += [f"{math.degrees(b.theta):.3f}", f"{math.degrees(b.phi):.3f}", f"{b.tip.x:.5f}",
                    f"{b.tip.y:.5f}", f"{b.tip.z:.5f}", f"{b.common_mode_mm:.3f}"]
        else:
            row += [""] * 6
        row += [f"{math.degrees(self.cmd[0]):.3f}", f"{math.degrees(self.cmd[1]):.3f}", msg.tripped]
        with self.file_lock:
            if self.tel:
                self.tel.writerow(row)

    def write_event(self, event: str, data: dict):
        with self.file_lock:
            if self.ev:
                self.ev.writerow([f"{time.time():.4f}", f"{time.monotonic() - self.t_start:.3f}", event,
                                  json.dumps(data, default=str)])

    def add_log(self, msg: str):
        line = f"[{time.monotonic() - self.t_start:7.1f}s] {msg}" if self.t_start else msg
        self.logs = (self.logs + [line])[-200:]
        self.get_logger().info(msg)

    def set_state(self, s: str):
        self.state = s
        self.publish_status()

    # ---------------- protocol files ----------------
    def load_file(self, stem: str) -> dict:
        f = self.protocols_dir / f"{stem}.yaml"
        if not f.exists():
            raise ProtocolError(f"no protocol file {f}")
        d = yaml.safe_load(f.read_text()) or {}
        if d.get("protocol") not in PROTOCOLS:
            raise ProtocolError(f"{f.name}: 'protocol' must be one of {sorted(PROTOCOLS)}")
        return d

    @safe
    def on_list(self, _req, res: Trigger.Response):
        out = {}
        for f in sorted(self.protocols_dir.glob("*.yaml")):
            try:
                d = yaml.safe_load(f.read_text()) or {}
                out[f.stem] = {"protocol": d.get("protocol"), "description": d.get("description", ""),
                               "params_yaml": yaml.safe_dump(d.get("params", {}), sort_keys=False)}
            except yaml.YAMLError as e:
                out[f.stem] = {"error": str(e)}
        res.success, res.message = True, json.dumps(out)
        return res

    # ---------------- services ----------------
    @safe
    def on_start(self, req: StartTest.Request, res: StartTest.Response):
        if not self.start_lock.acquire(blocking=False):
            res.success, res.message = False, "busy: another start is in progress"
            return res
        try:
            return self._start(req, res)
        finally:
            self.start_lock.release()

    def _start(self, req: StartTest.Request, res: StartTest.Response):
        if self.worker and self.worker.is_alive():
            res.success, res.message = False, f"busy: {self.protocol} is {self.state}"
            return res
        if self.trip_reason:
            res.success, res.message = False, f"bridge tripped ({self.trip_reason}) — rearm first"
            return res
        if not self.kin_info:
            res.success, res.message = False, "no kinematics info yet (kinematics node running?)"
            return res
        try:
            d = self.load_file(req.protocol)
            params = dict(d.get("params") or {})
            if req.overrides_yaml.strip():
                ov = yaml.safe_load(req.overrides_yaml) or {}
                if not isinstance(ov, dict):
                    raise ProtocolError("overrides must be a YAML mapping")
                params.update(ov)
            params = resolve(d["protocol"], params)
        except (ProtocolError, yaml.YAMLError) as e:
            res.success, res.message = False, str(e)
            return res

        self.abort_flag.clear()
        self.result, self.logs, self.prompt_obj = None, [], None
        self.protocol, self.t_start = req.protocol, time.monotonic()
        sdir = self.open_session(req.protocol)
        self.session_dir = str(sdir)
        meta = {"stem": req.protocol, "protocol": d["protocol"], "description": d.get("description", ""),
                "params": params, "notes": req.notes, "started": time.strftime("%Y-%m-%d %H:%M:%S"),
                "geometry": self.kin_info}
        (sdir / "meta.json").write_text(json.dumps(meta, indent=2, default=str))
        self.worker = threading.Thread(target=self._run, args=(d["protocol"], params, sdir, meta), daemon=True)
        self.state = "running"
        self.worker.start()
        res.success, res.message, res.session_dir = True, f"started {req.protocol}", str(sdir)
        return res

    def _run(self, name: str, params: dict, sdir: Path, meta: dict):
        ctx = RosContext(self)
        self.add_log(f"start {name} -> {sdir.name}")
        ctx.mark("start", protocol=name, params=params)
        final, result = "done", None
        try:
            result = PROTOCOLS[name](ctx, params)
            ctx.mark("done", result=result)
            self.add_log(f"done: {result}")
        except Aborted:
            final = "aborted"
            ctx.mark("aborted")
            self.add_log("ABORTED by operator — arm holding position")
        except Tripped as e:
            final = "tripped"
            ctx.mark("tripped", reason=str(e))
            self.add_log(f"SAFETY TRIP: {e} — torque off")
        except ProtocolError as e:
            final = "error"
            ctx.mark("error", message=str(e))
            self.add_log(f"ERROR: {e}")
        except Exception as e:  # noqa: BLE001 — never kill the node
            final = "error"
            ctx.mark("error", message=repr(e), tb=traceback.format_exc())
            self.add_log(f"CRASH: {e!r}")
            self.get_logger().error(traceback.format_exc())
        finally:
            self.result = result
            meta.update(result=result, outcome=final, ended=time.strftime("%Y-%m-%d %H:%M:%S"),
                        duration_s=round(time.monotonic() - self.t_start, 2))
            (sdir / "meta.json").write_text(json.dumps(meta, indent=2, default=str))
            self.close_session()
            self.prompt_obj = None
            self.set_state(final)

    @safe
    def on_respond(self, req: Respond.Request, res: Respond.Response):
        if self.prompt_obj is None:
            res.success, res.message = False, "no prompt pending"
            return res
        self.answer = req.value
        self.answer_evt.set()
        res.success, res.message = True, "ok"
        return res

    @safe
    def on_abort(self, _req, res: Trigger.Response):
        if not (self.worker and self.worker.is_alive()):
            res.success, res.message = False, "nothing running"
            return res
        self.abort_flag.set()
        res.success, res.message = True, "abort requested (arm holds position)"
        return res

    @safe
    def publish_status(self):
        st = {"state": self.state, "protocol": self.protocol, "session_dir": self.session_dir,
              "prompt": self.prompt_obj, "log": self.logs[-40:], "result": self.result,
              "elapsed_s": round(time.monotonic() - self.t_start, 1) if self.t_start else 0,
              "tripped": self.trip_reason}
        self.status_pub.publish(String(data=json.dumps(st, default=str)))


def main(args=None):
    rclpy.init(args=args)
    node = Runner()
    ex = MultiThreadedExecutor(num_threads=6)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.abort_flag.set()
        node.close_session()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
