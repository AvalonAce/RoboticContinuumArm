"""flex_bridge: flex (bend) sensors on their own QT Py -> bend angles on ROS.

Pub  /rcm/flex/state       rcm_msgs/FlexState   ~25 Hz: volts, resistance, angle per sensor
Pub  /rcm/notice           std_msgs/String      connect / calibration messages for the dashboard
Srv  /rcm/flex/set_flat    std_srvs/Trigger     all sensors flat now   -> 0 deg point
Srv  /rcm/flex/set_90      std_srvs/Trigger     sensors bent 90 deg now -> 90 deg point (bent ones only)

Wiring per channel: 3V -> flex -> pin -> R_fixed (10 kOhm) -> GND, so
    V = Vcc * R_fixed / (R_fixed + R_flex)   ->   R_flex = R_fixed * (Vcc / V - 1)
Angle is linear in resistance between the two calibration points:
    angle = 90 * (R - R_flat) / (R_90 - R_flat)
Until Set 90 is done, R_90 defaults to 2 x R_flat (typical for long flex sensors; an estimate).
Calibration is kept in <RCM_ROOT>/config/flex_calibration.yaml.

Board choice: the flex board is the sensor board that streams flex (F) lines and NO IMU (I) lines;
the IMU board (taken by sensor_bridge) is skipped. sim:=true derives angles from /rcm/bend_estimate.
"""
from __future__ import annotations

import glob
import math
import os
import signal
import threading
import time
from pathlib import Path

import rclpy
import serial
import yaml
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from rcm_msgs.msg import BendState, FlexState

from .sensor_serial import SensorBoard

PIN_NAMES = ("A2", "A3", "TX")


def candidate_ports() -> list[str]:
    ids = sorted(glob.glob("/dev/serial/by-id/*"))
    servo = {os.path.realpath(p) for p in ids if "CP210" in p}
    out, seen = [], set(servo)
    for p in ids:
        rp = os.path.realpath(p)
        if rp not in seen and any(k in p for k in ("Adafruit", "QT_Py", "Espressif")):
            out.append(p)
            seen.add(rp)
    out += [p for p in sorted(glob.glob("/dev/ttyACM*")) if os.path.realpath(p) not in seen]
    return out


def probe(port: str, wait_s: float = 1.5) -> str:
    """'flex' (flex lines, no IMU), 'imu', 'none' or 'busy'. Closes the port again."""
    try:
        s = serial.Serial(port, 921600, timeout=0.1, exclusive=True)
    except (serial.SerialException, OSError):
        return "busy"
    try:
        s.write(b"stream on\n")
        buf, end = b"", time.monotonic() + wait_s
        while time.monotonic() < end and len(buf) < 20000:
            buf += s.read(512)
    except (serial.SerialException, OSError):
        return "none"
    finally:
        s.close()
    if b"\nI," in buf:
        return "imu"
    return "flex" if b"\nF," in buf else "none"


class FlexBridge(Node):
    def __init__(self):
        super().__init__("flex_bridge")
        p = self.declare_parameter
        self.port_param = p("port", "").value
        self.sim = p("sim", False).value
        self.channels = [int(c) for c in p("flex_channels", [0, 1, 2]).value]  # 0=A2 1=A3 2=TX
        labels = list(p("flex_names", ["flex1 tip", "flex2 mid", "flex3 base"]).value)
        # where each sensor sits along the bendable length (cm from the base) -> whole-arm bend estimate
        self.length_cm = float(p("bend_length_cm", 27.0).value)
        fr = [float(x) for x in p("flex_from_cm", [16.0, 5.0, 0.0]).value]
        to = [float(x) for x in p("flex_to_cm", [27.0, 16.0, 10.5]).value]
        self.spans = [(fr[k], to[k]) if k < len(fr) and k < len(to) and to[k] > fr[k] else (float("nan"), float("nan"))
                      for k in range(len(self.channels))]
        self.names = [f"{labels[k] if k < len(labels) else f'flex{k + 1}'} ({PIN_NAMES[c]})"
                      for k, c in enumerate(self.channels)]
        self.r_fixed = float(p("flex_r_fixed_ohm", 10000.0).value)
        self.vcc = float(p("flex_vcc_v", 3.3).value)
        root = os.environ.get("RCM_ROOT")
        self.cal_file = Path(p("flex_cal_file", "").value or
                             (Path(root) / "config" / "flex_calibration.yaml" if root
                              else Path.home() / ".rcm_flex_calibration.yaml"))
        self.cal = self._load_cal()

        self.lock = threading.Lock()
        self.acc: list[list[float]] = [[] for _ in self.channels]   # volts since last publish
        self.recent: list[list[float]] = [[] for _ in self.channels]  # last ~0.5 s, for calibration
        self.n, self.rate, self.t_rate = 0, 0.0, time.monotonic()
        self.board: SensorBoard | None = None
        self.last_flex = 0.0
        self.t_connect = 0.0
        self.skip: dict[str, float] = {}
        self.sim_theta = 0.0

        self.pub = self.create_publisher(FlexState, "/rcm/flex/state", 10)
        self.notice_pub = self.create_publisher(String, "/rcm/notice", 10)
        self.create_service(Trigger, "/rcm/flex/set_flat", self.on_set_flat)
        self.create_service(Trigger, "/rcm/flex/set_90", self.on_set_90)
        self.create_timer(0.04, self.publish)
        if self.sim:
            self.get_logger().warn("SIMULATED flex sensors (from /rcm/bend_estimate)")
            self.create_subscription(BendState, "/rcm/bend_estimate", self.on_bend, 10)
            self.create_timer(0.02, self.sim_tick)
        else:
            self.create_timer(1.0, self.health)
            self.health()

    # ---------------- calibration ----------------
    def _load_cal(self) -> dict:
        try:
            d = yaml.safe_load(self.cal_file.read_text()) or {}
        except (OSError, yaml.YAMLError):
            d = {}
        return {int(k): dict(v) for k, v in (d.get("channels") or {}).items()}

    def _save_cal(self) -> None:
        self.cal_file.parent.mkdir(parents=True, exist_ok=True)
        data = {"updated": time.strftime("%Y-%m-%d %H:%M:%S"), "r_fixed_ohm": self.r_fixed, "vcc_v": self.vcc,
                "channels": {c: {k: round(float(v), 1) for k, v in self.cal[c].items()} for c in self.cal}}
        self.cal_file.write_text(yaml.safe_dump(data, sort_keys=False))

    def resistance(self, v: float) -> float:
        v = min(max(v, 0.01), self.vcc - 0.01)
        return self.r_fixed * (self.vcc / v - 1.0)

    def angle(self, c: int, r: float) -> tuple[float, bool]:
        cal = self.cal.get(c)
        if not cal or "r_flat" not in cal:
            return float("nan"), False
        r0 = cal["r_flat"]
        r90 = cal.get("r_90", 2.0 * r0)
        if abs(r90 - r0) < 1.0:
            return float("nan"), False
        return 90.0 * (r - r0) / (r90 - r0), "r_90" in cal

    def _recent_r(self) -> list[float | None]:
        """Mean resistance over the last ~0.5 s per channel; None if no data or no sensor (~0 V)."""
        with self.lock:
            out = []
            for v in self.recent:
                mv = sum(v) / len(v) if v else None
                out.append(self.resistance(mv) if mv is not None and mv >= 0.15 else None)
            return out

    def on_set_flat(self, _req, res: Trigger.Response):
        rs = self._recent_r()
        if all(r is None for r in rs):
            res.success, res.message = False, "no flex data -- board connected?"
            return res
        done, skipped = [], []
        for n, c, r in zip(self.names, self.channels, rs):
            if r is None:
                skipped.append(n)
                continue
            self.cal.setdefault(c, {})["r_flat"] = r
            done.append(f"{n} {r / 1000:.1f} kOhm")
        self._save_cal()
        res.success = True
        res.message = "flat (0 deg) set: " + ", ".join(done) + \
                      (f"; not connected (skipped): {', '.join(skipped)}" if skipped else "")
        self.notice(res.message)
        return res

    def on_set_90(self, _req, res: Trigger.Response):
        rs = self._recent_r()
        done, skipped = [], []
        for n, c, r in zip(self.names, self.channels, rs):
            r0 = self.cal.get(c, {}).get("r_flat")
            if r is None or r0 is None:
                skipped.append(f"{n} (set flat first)")
            elif r < r0 * 1.1:                       # not meaningfully bent: don't wreck its calibration
                skipped.append(f"{n} (not bent: {r / 1000:.1f} vs flat {r0 / 1000:.1f} kOhm)")
            else:
                self.cal[c]["r_90"] = r
                done.append(f"{n} {r / 1000:.1f} kOhm")
        if done:
            self._save_cal()
        res.success = bool(done)
        res.message = ("90 deg set: " + ", ".join(done) if done else "nothing set") + \
                      ("; skipped " + ", ".join(skipped) if skipped else "")
        self.notice(res.message)
        return res

    # ---------------- board ----------------
    def health(self) -> None:
        now = time.monotonic()
        b = self.board
        if b is not None:
            if b.link_ok and now - max(self.last_flex, self.t_connect) < 2.0:
                return
            self.notice(f"flex board lost ({b.link_error or 'no data'}) -- reconnecting")
            b.close()
            self.board = None
        self.connect()

    def connect(self) -> None:
        now = time.monotonic()
        self.skip = {p: t for p, t in self.skip.items() if now - t < 30.0}
        ports = [self.port_param] if self.port_param else candidate_ports()
        for port in ports:
            rp = os.path.realpath(port)
            if not self.port_param and rp in self.skip:
                continue
            kind = "flex" if self.port_param else probe(port)
            if kind != "flex":
                if kind == "imu":
                    self.skip[rp] = now                 # the IMU board; sensor_bridge's
                continue
            try:
                self.board = SensorBoard(port, on_imu=lambda s: None, on_text=self.on_text, on_flex=self.on_flex)
                self.t_connect = time.monotonic()
                self.notice(f"flex board connected ({port})")
                self.get_logger().info(f"flex board {port}")
                return
            except Exception as e:  # noqa: BLE001
                self.board = None
                self.get_logger().warn(f"cannot open {port}: {e}", throttle_duration_sec=10.0)
        self.get_logger().warn("no flex board found -- retrying", throttle_duration_sec=15.0)

    def on_text(self, line: str) -> None:
        if line.startswith("ERR") and "imu" not in line.lower():
            self.notice(f"flex board: {line}")

    def on_flex(self, _t: float, mv: tuple, _host_t: float) -> None:
        self.last_flex = time.monotonic()
        with self.lock:
            self.n += 1
            for k, c in enumerate(self.channels):
                v = mv[c] / 1000.0
                self.acc[k].append(v)
                self.recent[k] = (self.recent[k] + [v])[-25:]

    def notice(self, text: str) -> None:
        self.notice_pub.publish(String(data=f"flex_bridge: {text}"))

    # ---------------- publish ----------------
    def publish(self) -> None:
        now = time.monotonic()
        with self.lock:
            vals = [sum(a) / len(a) if a else None for a in self.acc]
            self.acc = [[] for _ in self.channels]
            if now - self.t_rate >= 1.0:
                self.rate, self.n, self.t_rate = self.n / (now - self.t_rate), 0, now
        m = FlexState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.names = self.names
        m.rate_hz = float(self.rate)
        live = self.sim or (self.board is not None and now - self.last_flex < 0.5)
        if not live:
            m.state, m.message = "offline", "flex board not connected"
            self.pub.publish(m)
            return
        if all(v is None for v in vals):
            return                                       # nothing new this tick
        m.state = "sim" if self.sim else "ok"
        missing = []
        for n, c, v in zip(self.names, self.channels, vals):
            v = v if v is not None else float("nan")
            if v == v and v < 0.15:                     # ~0 V: no sensor / 3V side open on this channel
                missing.append(n)
                r = a = float("nan")
                ok = False
                m.volts.append(float(v))
                m.resistance_ohm.append(r)
                m.angle_deg.append(a)
                m.calibrated.append(ok)
                continue
            r = self.resistance(v) if v == v else float("nan")
            a, ok = self.angle(c, r) if r == r else (float("nan"), False)
            m.volts.append(float(v))
            m.resistance_ohm.append(float(r))
            m.angle_deg.append(float(a))
            m.calibrated.append(ok)
        m.from_cm = [float(a) for a, _ in self.spans]
        m.to_cm = [float(b) for _, b in self.spans]
        m.length_cm = self.length_cm
        m.total_deg, m.covered_cm = self.whole_arm(list(m.angle_deg))
        if missing:
            m.message = "not connected: " + ", ".join(missing)
        elif any(c not in self.cal for c in self.channels):
            m.message = "press Set flat with the arm straight"
        elif not all(m.calibrated):
            m.message = "90 deg point estimated -- bend to 90 and press Set 90 to calibrate"
        self.pub.publish(m)

    def whole_arm(self, angles: list[float], dx: float = 0.25) -> tuple[float, float]:
        """Integrate curvature along the bendable length. Each sensor's angle is spread evenly over its span
        (deg/cm); where spans overlap the curvatures are averaged, so overlap is not double-counted.
        Uncovered length contributes nothing (reported via covered_cm)."""
        n = int(round(self.length_cm / dx))
        total, covered = 0.0, 0
        for i in range(n):
            x = (i + 0.5) * dx
            ks = [a / (b - f) for a, (f, b) in zip(angles, self.spans)
                  if a == a and f == f and f <= x < b]
            if ks:
                total += sum(ks) / len(ks) * dx
                covered += 1
        return (float(total) if covered else float("nan")), covered * dx

    # ---------------- simulation ----------------
    def on_bend(self, b: BendState) -> None:
        self.sim_theta = math.degrees(b.theta)

    def sim_tick(self) -> None:
        r0 = 10000.0
        mv = [0, 0, 0]
        for c, (f, b) in zip(self.channels, self.spans):          # constant curvature: angle ~ sensor length
            share = self.sim_theta * ((b - f) / self.length_cm if b == b else 0.0)
            r = r0 * (1 + share / 90.0) * (1 + 0.003 * math.sin(time.monotonic() * 7 + c))
            mv[c] = int(1000 * self.vcc * self.r_fixed / (self.r_fixed + r))
        self.last_flex = time.monotonic()
        self.on_flex(0.0, tuple(mv), 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = FlexBridge()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node.board:
            node.board.close()
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
