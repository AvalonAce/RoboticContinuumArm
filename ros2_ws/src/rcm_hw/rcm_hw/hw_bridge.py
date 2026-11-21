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

import faulthandler
import os
import queue
import threading
import time
from pathlib import Path

import signal

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from rcm_msgs.msg import ArmState, ServoState, TendonGoal
from rcm_msgs.srv import SetTorque

from .rcm_serial import RCM, RCMError

STALE_S = 0.5
STALL_DUMP_S = 1.5        # publish loop late by this much -> dump every thread's stack (diagnostics)
STALL_GRACE_S = 1.0       # after the bridge itself was frozen, don't judge the link for this long
LINK_TIMEOUT_S = 3.0      # no bytes at all for this long -> link lost (trip + reconnect)
GAP_NOTICE_S = 1.0        # shorter silences are only reported
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

        self.arm: RCM | None = None
        try:
            self.arm = self._open()
        except (RCMError, OSError) as e:      # not plugged in: run anyway, keep trying (sensors-only use)
            self.get_logger().warn(f"servo board not connected ({e}) — will connect when plugged in")
        self._temp_over: dict[int, float] = {}
        self._last_try = 0.0
        self._gap_noticed = False
        self._last_tick = time.monotonic()
        self._grace_until = 0.0
        # Stall diagnostics: a C-level watchdog dumps all Python stacks if publish_state stops running
        root = Path(os.environ.get("RCM_DATA") or (Path(os.environ["RCM_ROOT"]) / "data" if os.environ.get("RCM_ROOT")
                                                   else Path.home() / "rcm_data"))
        root.mkdir(parents=True, exist_ok=True)
        self._stall_path = root / "hw_bridge_stalls.log"
        self._stall_file = open(self._stall_path, "a", buffering=1)
        self._stall_file.write(f"\n=== hw_bridge started {time.strftime('%Y-%m-%d %H:%M:%S')} pid {os.getpid()}\n")
        self._bad_seen = 0
        self._bad_report_t = time.monotonic()
        self._link_down_since: float | None = None
        self._reconnecting = False
        self._held_trip = ""            # trip reason that survives a reconnect (needs Rearm)
        if self.arm:
            self._report_bus()
        self.torque_on = False

        # Threading (rclpy's MultiThreadedExecutor was seen to stop dispatching callbacks for seconds, freezing
        # moves + telemetry): the executor only RECEIVES; one worker thread owns every serial command
        # (queued, goals coalesced to the latest); telemetry is published from its own fixed-rate thread.
        self._run = True
        self._cmdq: queue.Queue = queue.Queue()
        self.pub = self.create_publisher(ArmState, "/rcm/state", 10)
        self.notice_pub = self.create_publisher(String, "/rcm/notice", 10)
        self._last_notice = (0.0, "")
        self.create_subscription(TendonGoal, "/rcm/goal", lambda m: self._cmdq.put(("goal", m)), 10)
        self.create_service(SetTorque, "/rcm/torque", self._queued(self.on_torque))
        self.create_service(Trigger, "/rcm/estop", self.on_estop)              # immediate, never queued
        self.create_service(Trigger, "/rcm/rearm", self._queued(self.on_rearm))
        self.create_service(Trigger, "/rcm/set_home", self._queued(self.on_set_home, timeout=20.0))
        threading.Thread(target=self._cmd_worker, name="serial_cmds", daemon=True).start()
        threading.Thread(target=self._pub_loop, name="telemetry", daemon=True).start()

    # ---------------- threads ----------------
    def _queued(self, fn, timeout: float = 8.0):
        """Wrap a service handler so it runs on the serial worker thread; the service call waits for it."""
        def handler(req, res):
            done = threading.Event()
            box = {}
            self._cmdq.put(("call", (fn, req, res, box, done)))
            if not done.wait(timeout):
                res.success, res.message = False, f"servo bridge busy: {fn.__name__} timed out after {timeout:.0f} s"
                return res
            return box.get("res", res)
        return handler

    def _cmd_worker(self) -> None:
        while self._run:
            try:
                kind, item = self._cmdq.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                if kind == "goal":
                    merged = {}                                     # coalesce consecutive goals: latest wins
                    speed, acc = item.speed, item.acc
                    pending = [item]
                    while True:
                        try:
                            k2, it2 = self._cmdq.get_nowait()
                        except queue.Empty:
                            break
                        if k2 != "goal":
                            self._cmdq.queue.appendleft((k2, it2))  # keep non-goal work, in order
                            break
                        pending.append(it2)
                    for g in pending:
                        merged.update(zip(g.ids, g.positions))
                        speed, acc = g.speed or speed, g.acc or acc
                    self.on_goal(TendonGoal(ids=list(merged), positions=list(merged.values()), speed=speed, acc=acc))
                else:
                    fn, req, res, box, done = item
                    try:
                        box["res"] = fn(req, res)
                    finally:
                        done.set()
            except Exception as e:  # noqa: BLE001 — the worker must never die
                self.get_logger().error(f"serial command failed: {e!r}")

    def _pub_loop(self) -> None:
        period = 1.0 / self.hz
        nxt = time.monotonic()
        while self._run and rclpy.ok():
            try:
                self.publish_state()
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f"publish_state: {e!r}", throttle_duration_sec=5.0)
            nxt += period
            d = nxt - time.monotonic()
            if d > 0:
                time.sleep(d)
            else:
                nxt = time.monotonic()                              # fell behind: don't burst

    def _open(self) -> RCM:
        return RCM(port=self.port or None, hz=self.hz, load_limit=self.load_limit)

    def _report_bus(self) -> None:
        missing = [i for i in self.ids if i not in self.arm.ids]
        self.get_logger().info(f"servo board {self.arm.port}, servos {self.arm.ids}")
        if missing:
            self.get_logger().error(f"configured servos missing from bus: {missing}")

    def _reconnect(self) -> None:
        """(Re)connect in a background thread: after a lost link, or when the board was never plugged in."""
        try:
            if self.arm is not None:
                try:
                    self.arm.close()
                except Exception:  # noqa: BLE001
                    pass
                self.arm = None
                time.sleep(0.3)
            new = self._open()
            if self._held_trip:
                new.trip(self._held_trip)        # lost link stays tripped until the operator rearms
            self.arm = new
            self._link_down_since = None
            self._report_bus()
            self.notice(f"servo board connected ({new.port})" +
                        ("; board was reset — press Rearm" if self._held_trip else "; servos limp until a command"))
        except Exception as e:  # noqa: BLE001
            self.get_logger().info(f"servo board not available: {e}", throttle_duration_sec=30.0)
        finally:
            self._reconnecting = False

    def _check_link(self, now: float) -> None:
        arm = self.arm
        gap = now - arm.last_rx
        down = (not arm.link_ok) or (gap > LINK_TIMEOUT_S)
        if GAP_NOTICE_S < gap <= LINK_TIMEOUT_S and arm.link_ok:
            if not self._gap_noticed:              # once per gap
                self._gap_noticed = True
                self.notice(f"telemetry gap > {GAP_NOTICE_S:.0f} s (no data from the servo board)")
        elif gap <= GAP_NOTICE_S:
            self._gap_noticed = False
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
        if self.arm is None:
            self.notice("move ignored: servo board not connected")
            return
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
        if self.arm is None:
            res.success, res.message = False, "servo board not connected"
            return res
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

    def on_set_home(self, _req, res: Trigger.Response):
        """Current pose = home. Re-centres every servo's encoder so its current angle reads 2048
        (STS3215 midpoint offset, stored in the servo), giving equal travel both ways, then holds there.
        Per servo: torque off -> zero -> goal 2048 -> torque on (no motion: goal == new position).
        Message = JSON {"ids": [...], "before": [...], "after": [...]}."""
        import json
        arm = self.arm
        if arm is None or self._reconnecting:
            res.success, res.message = False, "servo board not connected"
            return res
        if arm.tripped:
            res.success, res.message = False, f"bridge tripped ({arm.tripped}) -- Rearm first"
            return res
        before = {i: s.pos for i, s in arm.snapshot().items()}
        try:
            for i in self.ids:
                arm.torque(i, False)
                arm.command(f"zero {i}", reply="zeroed", timeout=1.5)
                time.sleep(0.05)                                  # offset is written to servo EEPROM
                arm.move({i: 2048}, speed=100, acc=self.default_acc)
                arm.torque(i, True)
            time.sleep(0.3)                                       # fresh telemetry in the new frame
            after = {i: s.pos for i, s in arm.snapshot().items()}
            bad = {i: after.get(i) for i in self.ids if abs(after.get(i, 0) - 2048) > 20}
            self.torque_on = True
            if bad:
                res.success, res.message = False, f"re-centre did not take on {bad} (expected ~2048)"
            else:
                res.success = True
                res.message = json.dumps({"ids": self.ids, "before": [before.get(i) for i in self.ids],
                                          "after": [after.get(i) for i in self.ids]})
            self.get_logger().info(f"set_home: before {before} after {after}")
        except RCMError as e:
            res.success, res.message = False, f"set_home failed: {e}"
        return res

    def on_estop(self, _req, res: Trigger.Response):
        if self.arm is None:
            res.success, res.message = True, "servo board not connected (nothing to stop)"
            return res
        self.arm.trip("estop")
        self.torque_on = False
        self.get_logger().warn("E-STOP: torque off")
        res.success, res.message = True, "torque off; goals blocked until /rcm/rearm"
        return res

    def on_rearm(self, _req, res: Trigger.Response):
        if self.arm is None:
            res.success, res.message = False, "servo board not connected"
            return res
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
        # re-arm the stall watchdog every tick (cheap); it only fires if this loop stops running
        faulthandler.cancel_dump_traceback_later()
        faulthandler.dump_traceback_later(STALL_DUMP_S, repeat=False, file=self._stall_file)
        late = now - self._last_tick
        self._last_tick = now
        if late > STALL_GRACE_S:                     # WE were frozen; serial data may just be queued up
            self._grace_until = now + STALL_GRACE_S
            self._stall_file.write(f"--- {time.strftime('%H:%M:%S')} publish loop resumed after {late:.2f} s\n")
            self.notice(f"servo bridge stalled {late:.1f} s (diagnostics: {self._stall_path.name})")
        arm = self.arm                               # local: the reconnect thread may swap self.arm
        if arm is not None and arm.slow_writes:
            for w in arm.slow_writes:
                self._stall_file.write(f"--- slow serial write at {time.strftime('%H:%M:%S', time.localtime(w[0]))}: "
                                       f"{w[1]!r} lock-wait {w[2]} s, write {w[3]} s\n")
            arm.slow_writes.clear()
        if arm is None:                              # not plugged in: report it, retry every 2 s
            if not self._reconnecting and now - self._last_try >= RECONNECT_EVERY_S:
                self._last_try, self._reconnecting = now, True
                threading.Thread(target=self._reconnect, daemon=True).start()
            msg = ArmState(connected=False, simulated=bool(self.sim), torque_enabled=False)
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.servos = [ServoState(id=i, goal=-1, error=True) for i in self.ids]
            self.torque_on = False
            self.pub.publish(msg)
            return
        if not self._reconnecting and now >= self._grace_until:
            self._check_link(now)
        if now - self._bad_report_t >= 5.0:
            n = arm.bad_lines - self._bad_seen
            if n > 0:
                self.notice(f"{n} corrupted serial lines dropped in the last 5 s (checksum/range)")
            self._bad_seen, self._bad_report_t = arm.bad_lines, now
        snap = arm.snapshot()
        msg = ArmState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for i in self.ids:
            s = snap.get(i)
            sv = ServoState(id=i, goal=int(arm.targets.get(i, -1)))
            if s is None or s.err or now - s.host_t > STALE_S:
                sv.error = True
            if s is not None:
                sv.pos, sv.speed, sv.load = s.pos, s.speed, s.load
                sv.voltage, sv.temperature, sv.current, sv.moving = s.volt, s.temp_C, s.current, bool(s.moving)
                if s.temp_C > self.temp_limit and not sv.error:
                    t0 = self._temp_over.setdefault(i, now)
                    if now - t0 >= TEMP_DEBOUNCE_S and not arm.tripped:
                        arm.trip(f"servo {i} temperature {s.temp_C} C > {self.temp_limit}")
                        self.notice(arm.tripped)
                else:
                    self._temp_over.pop(i, None)
            msg.servos.append(sv)
        if arm.tripped:
            self.torque_on = False
        msg.torque_enabled = self.torque_on
        msg.connected = self._link_down_since is None and not self._reconnecting
        msg.simulated = bool(self.sim)
        msg.tripped = arm.tripped or ""
        self.pub.publish(msg)

    def shutdown(self) -> None:
        self._run = False
        time.sleep(0.05)
        faulthandler.cancel_dump_traceback_later()
        self.get_logger().info("shutdown: torque off")
        if self.arm:
            self.arm.close()
        if self._fake:
            self._fake.stop()


def main(args=None):
    rclpy.init(args=args)
    node = HwBridge()
    ex = SingleThreadedExecutor()      # only receives; serial work + telemetry run on their own threads
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)   # 2nd SIGINT (from ros2 launch) must not cut cleanup short
        node.shutdown()
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
