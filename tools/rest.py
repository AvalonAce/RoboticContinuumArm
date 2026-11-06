#!/usr/bin/env python3
"""Move servos to the winding REST pose and hold them there while you wind + tie off tendons.
No ROS needed (uses the same rest computation as the kinematics node, from arm.yaml).

    python3 tools/rest.py            # move slowly to rest, hold until Enter, then torque off
    rcm rest                         # same, via boot.sh

Why: the servos use single-turn position mode (4096 counts total). Winding at the rest pose
puts each tendon's calibrated zero near the middle of that range, leaving room to reel in
AND pay out for full bends in every direction.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "rcm_kinematics"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from rcm_kinematics.cc import geometry_from_dict, rest_counts, travel_needed_counts  # noqa: E402
from rcm_serial import RCM, RCMError  # noqa: E402

CFG = ROOT / "ros2_ws" / "src" / "rcm_bringup" / "config" / "arm.yaml"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=str(CFG))
    ap.add_argument("--speed", type=int, default=150)
    ap.add_argument("--port", default=None)
    a = ap.parse_args()

    kp = yaml.safe_load(open(a.config))["kinematics"]["ros__parameters"]
    g = geometry_from_dict(kp)
    rest = rest_counts(g)
    for k, o in enumerate(kp.get("rest_counts", [])):
        if o >= 0:
            rest[k] = int(o)
    target = dict(zip(g.ids, rest))
    print(f"rest pose {target}  (travel needed each way for {g.theta_max_deg:.0f} deg: "
          f"{travel_needed_counts(g)} counts)")

    try:
        with RCM(port=a.port, hz=20) as arm:
            missing = [i for i in g.ids if i not in arm.ids]
            if missing:
                print(f"servos missing from bus: {missing}")
                return 1
            arm.wait_fresh(g.ids)
            now = {i: arm.snapshot()[i].pos for i in g.ids}
            print(f"current    {now}\nmoving slowly to rest...")
            arm.move(target, speed=a.speed, acc=10)
            errs = arm.wait_until_stopped(g.ids, tol=10, timeout=40)
            print(f"at rest (err {errs}). HOLDING torque.\n")
            print("  1. Mount spools / horns without rotating the servos.")
            print("  2. Wind each tendon with SLIGHT slack and tie off.")
            print("  3. Then run calibration (dashboard: Setup -> Calibrate).\n")
            input("Press Enter to release torque and exit (Ctrl+C also releases)... ")
    except KeyboardInterrupt:
        print("\ninterrupted")
    except RCMError as e:
        print(f"error: {e}")
        return 1
    print("torque OFF")
    return 0


if __name__ == "__main__":
    sys.exit(main())
