#!/usr/bin/env python3
"""Spool tendon line: drive one servo at a time continuously with the keyboard.

    python3 tools/spool.py          (or: rcm spool)
    Close the ROS stack / serial console first — this needs the servo board's port.

Keys
  1 2 3      select servo (stops the current one)
  A / D      HOLD to spin  (A = negative direction, D = positive). Release -> stops.
  W / S      speed up / down
  space      stop
  P          stop and hold here (position mode)
  L          limp (torque off) — turn the spool by hand
  Z          zero the turn counter
  Q          quit (all servos back to position mode, then limp)

Safety: stops if |load| stays above --load-limit (line taut / snag), and on release (dead-man).
The servo spins in wheel mode only while you spool; it is put back in position mode afterwards.
After spooling: dashboard -> Go to rest -> final slack/tie-off -> Calibrate.
"""
from __future__ import annotations

import argparse
import math
import os
import select
import sys
import termios
import time
import tty
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from rcm_serial import RCM, RCMError  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
CFG = ROOT / "ros2_ws" / "src" / "rcm_bringup" / "config" / "arm.yaml"
SPEEDS = [150, 300, 600, 1000, 1600, 2400]          # counts/s (4096 counts = 1 spool turn)
FIRST_GAP_S = 0.6     # key-repeat starts ~0.5 s after the first press
REPEAT_GAP_S = 0.15   # then repeats every ~30 ms; this long without a key = released


def spool_radius_mm() -> float | None:
    try:
        return float(yaml.safe_load(open(CFG))["kinematics"]["ros__parameters"]["spool_radius_mm"])
    except Exception:  # noqa: BLE001
        return None


class Keys:
    """Raw single-key reads from the terminal (no Enter needed)."""
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, *a):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

    def get(self, timeout: float) -> str | None:
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        return os.read(self.fd, 1).decode(errors="ignore").lower() if r else None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=None)
    ap.add_argument("--speed", type=int, default=600, help="starting speed, counts/s")
    ap.add_argument("--load-limit", type=int, default=600, help="stop if |load| exceeds this (0..1000)")
    a = ap.parse_args()
    if not sys.stdin.isatty():
        print("run this in a terminal (it reads single key presses)")
        return 1

    r_mm = spool_radius_mm()
    mm_per_count = 2 * math.pi * r_mm / 4096 if r_mm else None
    spd_i = min(range(len(SPEEDS)), key=lambda k: abs(SPEEDS[k] - a.speed))

    try:
        arm = RCM(port=a.port, hz=50)
    except (RCMError, OSError) as e:
        print(f"cannot connect to the servo board: {e}\n(stack or `rcm console` running? close it first)")
        return 1
    ids = arm.ids
    if not ids:
        print("no servos found on the bus")
        arm.close()
        return 1
    print(__doc__.split("Keys")[1].split("Safety")[0])
    print(f"servos {ids}" + (f" | spool radius {r_mm} mm (arm.yaml)" if r_mm else ""))

    sel = ids[0]
    wheeled: set[int] = set()          # servos we put in wheel mode (restore on exit)
    spinning = 0                       # -1, 0, +1
    last_key_t, n_keys = 0.0, 0
    over = 0
    turns_counts = {i: 0.0 for i in ids}
    last_pos: dict[int, int] = {}
    msg = ""

    def set_spin(direction: int) -> None:
        nonlocal spinning
        if direction == spinning:
            return
        arm.wheel(sel, direction * SPEEDS[spd_i])
        wheeled.add(sel)
        spinning = direction

    def stop() -> None:
        nonlocal spinning
        if spinning:
            arm.wheel(sel, 0)
            spinning = 0

    try:
        with Keys() as kb:
            while True:
                k = kb.get(0.03)
                now = time.monotonic()
                if k in ("a", "d"):
                    d = -1 if k == "a" else 1
                    n_keys = n_keys + 1 if d == spinning else 1
                    last_key_t = now
                    set_spin(d)
                elif k in [str(i) for i in ids]:
                    stop()
                    sel, msg = int(k), ""
                elif k == "w":
                    spd_i = min(spd_i + 1, len(SPEEDS) - 1)
                    if spinning:
                        arm.wheel(sel, spinning * SPEEDS[spd_i])
                elif k == "s":
                    spd_i = max(spd_i - 1, 0)
                    if spinning:
                        arm.wheel(sel, spinning * SPEEDS[spd_i])
                elif k == " ":
                    stop()
                elif k == "p":
                    stop()
                    pos = arm.posmode(sel)
                    wheeled.discard(sel)
                    msg = f"servo {sel} holding at {pos} (position mode)"
                elif k == "l":
                    stop()
                    arm.torque(sel, False)
                    msg = f"servo {sel} limp — turn by hand"
                elif k == "z":
                    turns_counts[sel] = 0.0
                elif k == "q":
                    break

                # dead-man: released keys stop the motor
                if spinning and now - last_key_t > (FIRST_GAP_S if n_keys <= 1 else REPEAT_GAP_S):
                    stop()

                snap = arm.snapshot()
                for i in ids:                       # unwrap single-turn encoder -> total counts
                    if i in snap and not snap[i].err:
                        p = snap[i].pos
                        if i in last_pos:
                            dp = p - last_pos[i]
                            dp -= 4096 if dp > 2048 else (-4096 if dp < -2048 else 0)
                            turns_counts[i] += dp
                        last_pos[i] = p
                s = snap.get(sel)
                if s and spinning and abs(s.load) > a.load_limit:
                    over += 1
                    if over >= 5:
                        stop()
                        msg = f"STOPPED: load {s.load} > {a.load_limit} (line taut / snag)"
                else:
                    over = 0
                tc = turns_counts[sel]
                line = (f"\rservo {sel} {'<<' if spinning < 0 else '>>' if spinning > 0 else '--'} "
                        f"speed {SPEEDS[spd_i]:4d} | turns {tc / 4096:+7.2f}"
                        + (f" ({tc * mm_per_count:+7.1f} mm line)" if mm_per_count else "")
                        + (f" | pos {s.pos:4d} load {s.load:+4d} {s.temp_C}C" if s else " | no data")
                        + (f" | {msg}" if msg else "") + "      ")
                sys.stdout.write(line[:160])
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    except RCMError as e:
        print(f"\nerror: {e}")
    finally:
        print()
        try:
            stop()
        except Exception:  # noqa: BLE001
            pass
        for i in sorted(wheeled):
            try:
                print(f"servo {i} -> position mode at {arm.posmode(i)}")
            except Exception as e:  # noqa: BLE001
                print(f"servo {i}: could not restore position mode ({e}) — power-cycle the servos")
        arm.close()
        print("done (servos limp).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
