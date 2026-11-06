"""hw_bridge: owns the serial port. ROS <-> RCM firmware.

Pub  /rcm/state   rcm_msgs/ArmState      (telemetry at `hz`)
Sub  /rcm/goal    rcm_msgs/TendonGoal    (raw counts; clamped to soft limits; one SyncWrite)
Srv  /rcm/torque  rcm_msgs/SetTorque     (id 0 = all)
Srv  /rcm/estop   std_srvs/Trigger       (torque off + block goals)
Srv  /rcm/rearm   std_srvs/Trigger       (clear trip; servos stay limp until next goal)
Pub  /rcm/notice  std_msgs/String        operator-visible warnings (ignored/clamped goals)

Safety: load limit (|load| > load_limit -> torque off), temperature limit, soft position
limits per servo. Any trip blocks goals until /rcm/rearm. Shutdown -> torque off.
"""
from __future__ import annotations

import threading
import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from rcm_msgs.msg import ArmState, ServoState, TendonGoal
from rcm_msgs.srv import SetTorque

from .rcm_serial import RCM, RCMError

STALE_S = 0.5
LINK_TIMEOUT_S = 1.5      # no bytes at all for this long -> link lost
TEMP_DEBOUNCE_S = 1.0     # temperature must stay over the limit this long to trip
RECONNECT_EVERY_S = 2.0


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
            self.arm = self._open()
        except (RCMError, OSError) as e:
            self.get_logger().fatal(f"cannot open board: {e}")
            raise
        self._temp_over: dict[int, float] = {}
        self._bad_seen = 0
        self._bad_report_t = time.monotonic()
        self._link_down_since: float | None = None
        self._reconnecting = False
        self._held_trip = ""            # trip reason that survives a reconnect (needs Rearm)
        missing = [i for i in self.ids if i not in self.arm.ids]
        self.get_logger().info(f"board {self.arm.port}, servos {self.arm.ids}")
        if missing:
            self.get_logger().error(f"configured servos missing from bus: {missing}")
        self.torque_on = False

        cmd_group = MutuallyExclusiveCallbackGroup()   # serial commands are serialized
        self.pub = self.create_publisher(ArmState, "/rcm/state", 10)
        self.notice_pub = self.create_publisher(String, "/rcm/notice", 10)
        self._last_notice = (0.0, "")
        self.create_subscription(TendonGoal, "/rcm/goal", self.on_goal, 10, callback_group=cmd_group)
        self.create_service(SetTorque, "/rcm/torque", self.on_torque, callback_group=cmd_group)
        self.create_service(Trigger, "/rcm/estop", self.on_estop)
        self.create_service(Trigger, "/rcm/rearm", self.on_rearm, callback_group=cmd_group)
        self.create_timer(1.0 / self.hz, self.publish_state)

    def _open(self) -> RCM:
        return RCM(port=self.port or None, hz=self.hz, load_limit=self.load_limit)

    def _reconnect(self) -> None:
        try:
            try:
                self.arm.close()
            except Exception:  # noqa: BLE001
                pass
            time.sleep(0.3)
            new = self._open()
            new.trip(self._held_trip or "serial link lost")   # stay tripped until operator rearms
            self.arm = new
            self._link_down_since = None
            self.notice(f"serial link restored ({self.arm.port}); board was reset — press Rearm")
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"reconnect failed: {e}", throttle_duration_sec=5.0)
        finally:
            self._reconnecting = False

    def _check_link(self, now: float) -> None:
        arm = self.arm
        down = (not arm.link_ok) or (now - arm.last_rx > LINK_TIMEOUT_S)
        if not down:
            self._link_down_since = None
            return
        if self._link_down_since is None:
            self._link_down_since = now
            self._held_trip = f"serial link lost ({arm.diag()})"
            arm.trip(self._held_trip)
            self.notice(self._held_trip + " — reconnecting")
        if not self._reconnecting and now - self._link_down_since >= 0.5:
            self._reconnecting = True
            self._link_down_since = now - 0.5 + RECONNECT_EVERY_S   # rate-limit retries
            threading.Thread(target=self._reconnect, daemon=True).start()

    # ---------------- callbacks ----------------
    def on_goal(self, msg: TendonGoal) -> None:
        if self._reconnecting:
            self.notice("move ignored: serial link reconnecting")
            return
        if self.arm.tripped:
            self.notice(f"move ignored: bridge tripped ({self.arm.tripped}) — press Rearm")
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
                self.notice(f"servo {i} goal {pos} clamped to {c} (soft limit)")
            targets[i] = c
        if not targets:
            return
        try:
            self.arm.move(targets, speed=msg.speed or self.default_speed, acc=msg.acc or self.default_acc)
            self.torque_on = True
        except RCMError as e:
            self.notice(f"move failed: {e}")

    def notice(self, text: str) -> None:
        """Log + publish for the dashboard; identical notices throttled to 1 per 2 s."""
        now = time.monotonic()
        if text == self._last_notice[1] and now - self._last_notice[0] < 2.0:
            return
        self._last_notice = (now, text)
        self.get_logger().warn(text)
        self.notice_pub.publish(String(data=f"hw_bridge: {text}"))

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
        if not self.arm.link_ok:
            res.success, res.message = False, f"cannot rearm: {self.arm.diag()}"
            return res
        self._held_trip = ""
        self.arm.reset_trip()
        self.arm.targets.clear()          # don't treat stale goals as active
        self.get_logger().info(f"rearmed (was: {was or 'not tripped'}); servos limp until next goal")
        res.success, res.message = True, f"rearmed (was: {was or 'not tripped'})"
        return res

    def publish_state(self) -> None:
        now = time.monotonic()
        if not self._reconnecting:
            self._check_link(now)
        if now - self._bad_report_t >= 5.0:
            n = self.arm.bad_lines - self._bad_seen
            if n > 0:
                self.notice(f"{n} corrupted serial lines dropped in the last 5 s (checksum/range)")
            self._bad_seen, self._bad_report_t = self.arm.bad_lines, now
        snap = self.arm.snapshot()
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
                if s.temp_C > self.temp_limit and not sv.error:
                    t0 = self._temp_over.setdefault(i, now)
                    if now - t0 >= TEMP_DEBOUNCE_S and not self.arm.tripped:
                        self.arm.trip(f"servo {i} temperature {s.temp_C} C > {self.temp_limit}")
                        self.notice(self.arm.tripped)
                else:
                    self._temp_over.pop(i, None)
            msg.servos.append(sv)
        if self.arm.tripped:
            self.torque_on = False
        msg.torque_enabled = self.torque_on
        msg.simulated = bool(self.sim)
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
    ex = MultiThreadedExecutor(num_threads=4)   # telemetry timer must not wait behind a serial command
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
