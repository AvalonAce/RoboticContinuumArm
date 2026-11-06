"""hw_bridge: owns the serial port. ROS <-> RCM firmware.

Pub  /rcm/state   rcm_msgs/ArmState      (telemetry at `hz`)
Sub  /rcm/goal    rcm_msgs/TendonGoal    (raw counts; clamped to soft limits; one SyncWrite)
Srv  /rcm/torque  rcm_msgs/SetTorque     (id 0 = all)
Srv  /rcm/estop   std_srvs/Trigger       (torque off + block goals)
Srv  /rcm/rearm   std_srvs/Trigger       (clear trip; servos stay limp until next goal)

Safety: load limit (|load| > load_limit -> torque off), temperature limit, soft position
limits per servo. Any trip blocks goals until /rcm/rearm. Shutdown -> torque off.
"""
from __future__ import annotations

import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from rcm_msgs.msg import ArmState, ServoState, TendonGoal
from rcm_msgs.srv import SetTorque

from .rcm_serial import RCM, RCMError

STALE_S = 0.5


class HwBridge(Node):
    def __init__(self):
        super().__init__("hw_bridge")
        p = self.declare_parameter
        self.port = p("port", "").value
        self.sim = p("sim", False).value
        self.hz = p("hz", 50).value
        self.load_limit = p("load_limit", 900).value
        self.temp_limit = p("temp_limit", 60).value
        self.default_speed = p("default_speed", 400).value
        self.default_acc = p("default_acc", 30).value
        self.ids = list(p("ids", [1, 2, 3]).value)
        mins = list(p("soft_min", [300, 300, 300]).value)
        maxs = list(p("soft_max", [3800, 3800, 3800]).value)
        self.limits = {i: (mins[k], maxs[k]) for k, i in enumerate(self.ids)}

        self._fake = None
        if self.sim:
            from .sim import FakeBoard
            self._fake = FakeBoard(ids=self.ids)
            self.port = self._fake.start()
            self.get_logger().warn(f"SIMULATED board on {self.port}")

        try:
            self.arm = RCM(port=self.port or None, hz=self.hz, load_limit=self.load_limit)
        except (RCMError, OSError) as e:
            self.get_logger().fatal(f"cannot open board: {e}")
            raise
        missing = [i for i in self.ids if i not in self.arm.ids]
        self.get_logger().info(f"board {self.arm.port}, servos {self.arm.ids}")
        if missing:
            self.get_logger().error(f"configured servos missing from bus: {missing}")
        self.torque_on = False

        cmd_group = MutuallyExclusiveCallbackGroup()   # serial commands are serialized
        self.pub = self.create_publisher(ArmState, "/rcm/state", 10)
        self.create_subscription(TendonGoal, "/rcm/goal", self.on_goal, 10, callback_group=cmd_group)
        self.create_service(SetTorque, "/rcm/torque", self.on_torque, callback_group=cmd_group)
        self.create_service(Trigger, "/rcm/estop", self.on_estop)
        self.create_service(Trigger, "/rcm/rearm", self.on_rearm, callback_group=cmd_group)
        self.create_timer(1.0 / self.hz, self.publish_state)

    # ---------------- callbacks ----------------
    def on_goal(self, msg: TendonGoal) -> None:
        if self.arm.tripped:
            self.get_logger().warn(f"goal ignored: tripped ({self.arm.tripped}) — call /rcm/rearm",
                                   throttle_duration_sec=2.0)
            return
        if len(msg.ids) != len(msg.positions) or not msg.ids:
            self.get_logger().error("bad goal: ids/positions length mismatch")
            return
        targets = {}
        for i, pos in zip(msg.ids, msg.positions):
            if i not in self.limits:
                self.get_logger().error(f"goal for unknown servo {i} ignored")
                continue
            lo, hi = self.limits[i]
            c = max(lo, min(hi, int(pos)))
            if c != pos:
                self.get_logger().warn(f"servo {i} goal {pos} clamped to {c} (soft limit)",
                                       throttle_duration_sec=1.0)
            targets[i] = c
        if not targets:
            return
        try:
            self.arm.move(targets, speed=msg.speed or self.default_speed, acc=msg.acc or self.default_acc)
            self.torque_on = True
        except RCMError as e:
            self.get_logger().error(f"move failed: {e}")

    def on_torque(self, req: SetTorque.Request, res: SetTorque.Response):
        ids = self.ids if req.id == 0 else [req.id]
        try:
            for i in ids:
                self.arm.torque(i, req.enable)
            if req.id == 0 or not req.enable:
                self.torque_on = bool(req.enable)
            res.success, res.message = True, f"torque {ids} = {int(req.enable)}"
        except RCMError as e:
            res.success, res.message = False, str(e)
        return res

    def on_estop(self, _req, res: Trigger.Response):
        self.arm.trip("estop")
        self.torque_on = False
        self.get_logger().warn("E-STOP: torque off")
        res.success, res.message = True, "torque off; goals blocked until /rcm/rearm"
        return res

    def on_rearm(self, _req, res: Trigger.Response):
        was = self.arm.tripped
        self.arm.reset_trip()
        self.arm.targets.clear()          # don't treat stale goals as active
        self.get_logger().info(f"rearmed (was: {was or 'not tripped'}); servos limp until next goal")
        res.success, res.message = True, f"rearmed (was: {was or 'not tripped'})"
        return res

    def publish_state(self) -> None:
        snap = self.arm.snapshot()
        now = time.monotonic()
        msg = ArmState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for i in self.ids:
            s = snap.get(i)
            sv = ServoState(id=i, goal=int(self.arm.targets.get(i, -1)))
            if s is None or s.err or now - s.host_t > STALE_S:
                sv.error = True
            if s is not None:
                sv.pos, sv.speed, sv.load = s.pos, s.speed, s.load
                sv.voltage, sv.temperature, sv.current, sv.moving = s.volt, s.temp_C, s.current, bool(s.moving)
                if s.temp_C > self.temp_limit and not self.arm.tripped:
                    self.arm.trip(f"servo {i} temperature {s.temp_C} C > {self.temp_limit}")
                    self.get_logger().error(self.arm.tripped)
            msg.servos.append(sv)
        if self.arm.tripped:
            self.torque_on = False
        msg.torque_enabled = self.torque_on
        msg.tripped = self.arm.tripped or ""
        self.pub.publish(msg)

    def shutdown(self) -> None:
        self.get_logger().info("shutdown: torque off")
        self.arm.close()
        if self._fake:
            self._fake.stop()


def main(args=None):
    rclpy.init(args=args)
    node = HwBridge()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
