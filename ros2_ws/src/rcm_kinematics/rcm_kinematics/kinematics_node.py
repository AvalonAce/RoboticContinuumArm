"""kinematics: CC configuration space <-> servo counts.

Sub  /rcm/cmd_bend       rcm_msgs/BendCommand  -> Pub /rcm/goal (rcm_msgs/TendonGoal)
Sub  /rcm/state          rcm_msgs/ArmState     -> Pub /rcm/bend_estimate (rcm_msgs/BendState)
Srv  /rcm/set_zero       rcm_msgs/SetZero      store calibration zeros (writes calibration_file)
Srv  /rcm/kinematics/reload  std_srvs/Trigger  re-read calibration_file
Srv  /rcm/rest           std_srvs/Trigger      move all servos (slowly) to the winding rest pose and hold
Pub  /rcm/kinematics/info    std_msgs/String   JSON geometry + calibration (latched)
Pub  /rcm/notice             std_msgs/String   operator-visible warnings (rejected commands)
"""
from __future__ import annotations

import json
import os
import time
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger

from rcm_msgs.msg import ArmState, BendCommand, BendState, TendonGoal
from rcm_msgs.srv import SetZero

from .cc import ArmGeometry, OutOfRange, bend_to_counts, counts_to_bend, range_report, rest_counts


def default_calibration_file() -> str:
    root = os.environ.get("RCM_ROOT")
    return str(Path(root) / "config" / "calibration.yaml") if root else str(Path.home() / ".rcm" / "calibration.yaml")


class Kinematics(Node):
    def __init__(self):
        super().__init__("kinematics")
        p = self.declare_parameter
        self.geom_params = dict(
            length_mm=p("length_mm", 200.0).value,
            tendon_radius_mm=p("tendon_radius_mm", 15.0).value,
            spool_radius_mm=p("spool_radius_mm", 10.0).value,
            ids=list(p("ids", [1, 2, 3]).value),
            angles_deg=list(p("angles_deg", [0.0, 120.0, 240.0]).value),
            reel_in_sign=list(p("reel_in_sign", [1, 1, 1]).value),
            takeup_mm=p("takeup_mm", 0.0).value,
            theta_max_deg=p("theta_max_deg", 90.0).value,
            pretension_allowance_mm=p("pretension_allowance_mm", 3.0).value,
        )
        self.rest_override = list(p("rest_counts", [-1, -1, -1]).value)   # -1 = auto
        self.rest_speed = p("rest_speed", 150).value
        self.cal_file = p("calibration_file", "").value or default_calibration_file()
        self.cal: dict = {}
        self.g: ArmGeometry = self._build()

        self.goal_pub = self.create_publisher(TendonGoal, "/rcm/goal", 10)
        self.est_pub = self.create_publisher(BendState, "/rcm/bend_estimate", 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.info_pub = self.create_publisher(String, "/rcm/kinematics/info", latched)
        self.notice_pub = self.create_publisher(String, "/rcm/notice", 10)
        self.create_subscription(BendCommand, "/rcm/cmd_bend", self.on_cmd, 10)
        self.create_subscription(ArmState, "/rcm/state", self.on_state, 10)
        self.create_service(SetZero, "/rcm/set_zero", self.on_set_zero)
        self.create_service(Trigger, "/rcm/kinematics/reload", self.on_reload)
        self.create_service(Trigger, "/rcm/rest", self.on_rest)
        self.cmd = (0.0, 0.0)
        self._publish_info()

    # ---------------- calibration ----------------
    def _load_cal(self) -> dict:
        try:
            with open(self.cal_file) as f:
                return yaml.safe_load(f) or {}
        except FileNotFoundError:
            return {}

    def _build(self) -> ArmGeometry:
        self.cal = self._load_cal()
        gp = dict(self.geom_params)
        z = self.cal.get("zero_counts")
        if z:
            gp["zero_counts"] = [int(z[i]) for i in gp["ids"]]
            self.get_logger().info(f"calibration {self.cal_file}: zeros {gp['zero_counts']}")
        else:
            gp["zero_counts"] = [2048] * len(gp["ids"])
            self.get_logger().warn(f"UNCALIBRATED (no {self.cal_file}); zeros = 2048. Run the calibrate protocol.")
        return ArmGeometry(**gp)

    def _publish_info(self) -> None:
        calibrated = bool(self.cal.get("zero_counts"))
        info = dict(self.geom_params, zero_counts=self.g.zero_counts, calibrated=calibrated,
                    calibration_file=self.cal_file, calibration=self.cal, mm_per_count=self.g.mm_per_count,
                    rest_counts=self.rest(), range=range_report(self.g) if calibrated else None)
        self.info_pub.publish(String(data=json.dumps(info)))

    # ---------------- callbacks ----------------
    def on_cmd(self, msg: BendCommand) -> None:
        try:
            counts = bend_to_counts(self.g, msg.theta, msg.phi)
        except OutOfRange as e:
            self.get_logger().error(f"bend command rejected: {e}")
            self.notice_pub.publish(String(data=f"kinematics: move rejected — {e}"))
            return
        self.cmd = (msg.theta, msg.phi)
        self.goal_pub.publish(TendonGoal(ids=self.g.ids, positions=counts, speed=msg.speed, acc=msg.acc))

    def on_state(self, msg: ArmState) -> None:
        by_id = {s.id: s for s in msg.servos}
        if any(i not in by_id or by_id[i].error for i in self.g.ids):
            return
        e = counts_to_bend(self.g, [by_id[i].pos for i in self.g.ids])
        out = BendState(theta=e.theta, phi=e.phi, kappa=e.kappa, tendon_dl_mm=e.dl_mm,
                        common_mode_mm=e.common_mode_mm, cmd_theta=self.cmd[0], cmd_phi=self.cmd[1])
        out.header = msg.header
        out.tip.x, out.tip.y, out.tip.z = e.tip_m
        self.est_pub.publish(out)

    def on_set_zero(self, req: SetZero.Request, res: SetZero.Response):
        if len(req.ids) != len(req.counts) or set(req.ids) != set(self.g.ids):
            res.success, res.message = False, f"need zeros for exactly ids {self.g.ids}"
            return res
        cal = dict(self.cal)
        cal["zero_counts"] = {int(i): int(c) for i, c in zip(req.ids, req.counts)}
        cal["updated"] = time.strftime("%Y-%m-%d %H:%M:%S")
        Path(self.cal_file).parent.mkdir(parents=True, exist_ok=True)
        with open(self.cal_file, "w") as f:
            yaml.safe_dump(cal, f, sort_keys=False)
        self.g = self._build()
        self._publish_info()
        res.success, res.message = True, f"zeros {cal['zero_counts']} -> {self.cal_file}"
        self.get_logger().info(res.message)
        return res

    def rest(self) -> list[int]:
        auto = rest_counts(self.g)
        ov = self.rest_override + [-1] * len(auto)
        return [int(ov[k]) if ov[k] >= 0 else a for k, a in enumerate(auto)]

    def on_rest(self, _req, res: Trigger.Response):
        r = self.rest()
        self.cmd = (float("nan"), float("nan"))
        self.goal_pub.publish(TendonGoal(ids=self.g.ids, positions=r, speed=self.rest_speed))
        res.success, res.message = True, f"moving to rest {dict(zip(self.g.ids, r))} and holding"
        self.get_logger().info(res.message)
        return res

    def on_reload(self, _req, res: Trigger.Response):
        self.g = self._build()
        self._publish_info()
        res.success, res.message = True, f"zeros {self.g.zero_counts}"
        return res


def main(args=None):
    rclpy.init(args=args)
    node = Kinematics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
