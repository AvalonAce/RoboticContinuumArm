"""sensor_bridge: tip IMU (QT Py ESP32 + ICM-20948) -> filtered orientation on ROS.

Pub  /rcm/imu/state     rcm_msgs/ImuState     ~100 Hz: orientation, relative-to-reference, roll/pitch,
                                              tilt_from_ref, bias-corrected gyro, raw accel, status
Pub  /rcm/imu/data      sensor_msgs/Imu       filtered (orientation + bias-corrected gyro + accel)
Pub  /rcm/imu/data_raw  sensor_msgs/Imu       raw accel/gyro, no orientation
Pub  /rcm/notice        std_msgs/String       connect/disconnect/calibration messages for the dashboard
Srv  /rcm/imu/set_reference    std_srvs/Trigger   current pose = reference ("straight")
Srv  /rcm/imu/clear_reference  std_srvs/Trigger
Srv  /rcm/imu/calibrate_gyro   std_srvs/Trigger   re-estimate gyro bias (hold still ~2 s)

Hot-plug tolerant: if the board is missing or unplugged, state = offline and it keeps retrying.
sim:=true synthesizes the IMU from /rcm/bend_estimate (tip rotation of the CC model) so the whole
pipeline + dashboard can be exercised without hardware.
"""
from __future__ import annotations

import math
import os
import random
import threading
import time

import signal

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Quaternion, Vector3
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from std_srvs.srv import Trigger

from rcm_msgs.msg import BendState, ImuState

from .imu_filter import G, ImuProcessor, gravity_in_sensor, q_conj, q_from_axis_angle, q_mul
from .sensor_serial import ImuSample, SensorBoard, find_sensor_port

STALE_S = 0.5


def _quat(q) -> Quaternion:
    return Quaternion(w=float(q[0]), x=float(q[1]), y=float(q[2]), z=float(q[3]))


def _vec(v) -> Vector3:
    return Vector3(x=float(v[0]), y=float(v[1]), z=float(v[2]))


class SensorBridge(Node):
    def __init__(self):
        super().__init__("sensor_bridge")
        p = self.declare_parameter
        self.port_param = p("port", "").value
        self.sim = p("sim", False).value
        self.frame_id = p("frame_id", "rcm_imu_link").value
        self.proc = ImuProcessor(beta=p("beta", 0.08).value, bias_s=p("bias_s", 2.0).value)
        self.lock = threading.Lock()
        self.board: SensorBoard | None = None
        self.last_sample = 0.0
        self.temp = float("nan")
        self.n_rate, self.rate, self.t_rate = 0, 0.0, time.monotonic()
        self.was_calibrated = False
        self.t_connect = 0.0
        self.skip: dict[str, float] = {}      # realpath -> monotonic time; ports that gave no IMU data

        self.state_pub = self.create_publisher(ImuState, "/rcm/imu/state", 20)
        self.imu_pub = self.create_publisher(Imu, "/rcm/imu/data", 20)
        self.raw_pub = self.create_publisher(Imu, "/rcm/imu/data_raw", 20)
        self.notice_pub = self.create_publisher(String, "/rcm/notice", 10)
        self.create_service(Trigger, "/rcm/imu/set_reference", self.on_set_ref)
        self.create_service(Trigger, "/rcm/imu/clear_reference", self.on_clear_ref)
        self.create_service(Trigger, "/rcm/imu/calibrate_gyro", self.on_calibrate)
        self.create_timer(0.5, self.health)

        if self.sim:
            self.get_logger().warn("SIMULATED IMU (from /rcm/bend_estimate)")
            self.sim_q = (1.0, 0.0, 0.0, 0.0)
            self.sim_q_prev = self.sim_q
            self.sim_target = (0.0, 0.0)
            self.sim_t = 0.0
            self.sim_bias = (0.012, -0.008, 0.004)
            self.create_subscription(BendState, "/rcm/bend_estimate", self.on_bend, 10)
            self.create_timer(0.01, self.sim_tick)
        else:
            self.connect()

    # ---------------- connection ----------------
    def connect(self) -> None:
        now = time.monotonic()
        self.skip = {p: t for p, t in self.skip.items() if now - t < 30.0}
        port = self.port_param or find_sensor_port(set(self.skip))
        # every board was skipped (none sent IMU data): wait for their 30 s skip to expire instead of
        # re-grabbing them now -- re-grabbing the flex board every few s knocked flex_bridge off it
        if not port:
            self.get_logger().warn("no sensor board found — retrying", throttle_duration_sec=10.0)
            return
        try:
            self.board = SensorBoard(port, self.on_sample, self.on_text)
            self.t_connect = time.monotonic()
            self.notice(f"IMU board connected ({port})")
            self.get_logger().info(f"sensor board {port}")
        except Exception as e:  # noqa: BLE001
            self.board = None
            self.get_logger().warn(f"cannot open {port}: {e}", throttle_duration_sec=10.0)

    def health(self) -> None:
        now = time.monotonic()
        if now - self.t_rate >= 1.0:
            self.rate, self.n_rate, self.t_rate = self.n_rate / (now - self.t_rate), 0, now
        if self.sim:
            return
        b = self.board
        if b is None:
            self.connect()
            self.publish_offline("sensor board not found")
        elif not b.link_ok or now - max(self.last_sample, self.t_connect) > 2.0:   # no IMU samples
            if b.link_ok and not self.port_param:   # port open but no IMU lines: probably the flex board
                self.skip[os.path.realpath(b.port)] = now
                self.notice(f"no IMU data on {b.port} (flex board, or IMU cable loose?) — trying the next board")
            else:
                self.notice(f"IMU board lost ({b.link_error or 'no data'}) — reconnecting")
            b.close()
            self.board = None
            self.publish_offline("sensor board disconnected")

    def on_text(self, line: str) -> None:
        self.get_logger().info(f"board: {line}")
        if line.startswith("ERR"):
            self.notice(f"IMU board: {line}")

    def notice(self, text: str) -> None:
        self.notice_pub.publish(String(data=f"sensor_bridge: {text}"))

    # ---------------- samples ----------------
    def on_sample(self, s: ImuSample) -> None:
        with self.lock:
            out = self.proc.update(s.t, s.accel, s.gyro)
            msg_text = self.proc.message
            calibrated = self.proc.calibrated
        self.n_rate += 1
        self.last_sample = time.monotonic()
        self.temp = s.temp
        stamp = self.get_clock().now().to_msg()

        raw = Imu()
        raw.header.stamp, raw.header.frame_id = stamp, self.frame_id
        raw.orientation_covariance[0] = -1.0
        raw.angular_velocity, raw.linear_acceleration = _vec(s.gyro), _vec(s.accel)
        self.raw_pub.publish(raw)

        if calibrated and not self.was_calibrated:
            self.notice("IMU gyro calibrated")
        self.was_calibrated = calibrated

        st = ImuState()
        st.header = raw.header
        st.rate_hz = float(self.rate)
        st.message = msg_text
        st.temperature = float(s.temp)
        st.bad_lines = self.board.bad_lines if self.board else 0
        st.accel = _vec(s.accel)
        if out is None:
            st.state = "calibrating"
            st.orientation = st.relative = _quat((1, 0, 0, 0))
            st.tilt_from_ref_deg = float("nan")
            st.gyro = _vec(s.gyro)
        else:
            st.state = "ok"
            st.orientation, st.relative = _quat(out.q), _quat(out.q_rel)
            st.has_reference = out.has_reference
            st.roll_deg, st.pitch_deg, st.tilt_from_ref_deg = out.roll_deg, out.pitch_deg, out.tilt_from_ref_deg
            st.gyro, st.gyro_bias = _vec(out.gyro), _vec(out.bias)
            imu = Imu()
            imu.header = raw.header
            imu.orientation = st.orientation
            imu.orientation_covariance = [0.0004, 0.0, 0.0, 0.0, 0.0004, 0.0, 0.0, 0.0, 1e6]  # yaw unobserved
            imu.angular_velocity, imu.linear_acceleration = st.gyro, st.accel
            self.imu_pub.publish(imu)
        self.state_pub.publish(st)

    def publish_offline(self, why: str) -> None:
        st = ImuState(state="offline", message=why)
        st.header.stamp = self.get_clock().now().to_msg()
        st.orientation = st.relative = _quat((1, 0, 0, 0))
        st.tilt_from_ref_deg = float("nan")
        self.state_pub.publish(st)

    # ---------------- services ----------------
    def on_set_ref(self, _req, res: Trigger.Response):
        with self.lock:
            ok = self.proc.set_reference()
        res.success = ok
        res.message = "reference set (current pose)" if ok else "not calibrated yet — hold the IMU still"
        return res

    def on_clear_ref(self, _req, res: Trigger.Response):
        with self.lock:
            self.proc.clear_reference()
        res.success, res.message = True, "reference cleared"
        return res

    def on_calibrate(self, _req, res: Trigger.Response):
        with self.lock:
            self.proc.recalibrate()
        res.success, res.message = True, "recalibrating gyro — hold the IMU still ~2 s"
        return res

    # ---------------- simulation ----------------
    def on_bend(self, m: BendState) -> None:
        self.sim_target = (m.theta, m.phi)

    def sim_tick(self) -> None:
        """Tip orientation of a CC section with base z up: rotation by theta about (-sin phi, cos phi, 0)."""
        th, ph = self.sim_target
        q = q_from_axis_angle((-math.sin(ph), math.cos(ph), 0.0), th)
        dq = q_mul(q_conj(self.sim_q_prev), q)                # body-frame rotation over the step
        ang = 2 * math.acos(max(-1.0, min(1.0, dq[0])))
        sn = math.sqrt(max(1e-12, 1 - dq[0] * dq[0]))
        omega = tuple((dq[k + 1] / sn) * ang / 0.01 for k in range(3)) if ang > 1e-9 else (0.0, 0.0, 0.0)
        self.sim_q_prev = q
        self.sim_t += 0.01
        acc = tuple(G * c + random.gauss(0, 0.03) for c in gravity_in_sensor(q))
        gyr = tuple(o + b + random.gauss(0, 0.002) for o, b in zip(omega, self.sim_bias))
        self.on_sample(ImuSample(self.sim_t, acc, gyr, (0.0, 0.0, 0.0), 30.0, time.monotonic()))


def main(args=None):
    rclpy.init(args=args)
    node = SensorBridge()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)   # 2nd SIGINT (from ros2 launch) must not cut cleanup short
        if node.board:
            node.board.close()
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
