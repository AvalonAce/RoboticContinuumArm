#!/usr/bin/env python3
"""unspool_diag: try to let line out on ONE servo in several ways and log everything, to find out why it
won't move (servo overload protection, jam, power, wrong direction).

    python3 tools/unspool_diag.py            # servo 3, release ~10 mm total
    python3 tools/unspool_diag.py --id 2 --counts 300

Run with the ROS stack and `rcm spool` closed (they hold the port). Ctrl+C stops the motor at any time.

Phases (each logged; a one-line verdict is printed after each):
  0 baseline      1.0 s   nothing commanded: position, load, current, voltage, temperature
  1 torque reset  torque off 1 s, torque on   (re-enabling torque can clear an overload shutdown)
  2 pos step      position mode, goal = here + N counts in the RELEASE direction, slow
  3 wheel slow    wheel mode, release direction, 300 counts/s for 2 s
  4 wheel fast    wheel mode, release direction, 1000 counts/s for 2 s
  end             position mode, hold where it is

Release direction = opposite of reel_in_sign in arm.yaml (reel_in_sign -1 -> release = counts increasing).
Every telemetry row goes to data/unspool_diag_<time>_id<N>.csv; a summary goes to ..._summary.txt.
Stops a phase early if the servo strains IN the release direction (line wound the other way / snag).
"""
from __future__ import annotations

import argparse
import statistics as st
import sys
import time
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from rcm_serial import RCM, RCMError  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
CFG = ROOT / "ros2_ws" / "src" / "rcm_bringup" / "config" / "arm.yaml"


def release_dir(sid: int) -> int:
    try:
        kp = yaml.safe_load(open(CFG))["kinematics"]["ros__parameters"]
        sign = int(kp["reel_in_sign"][list(kp["ids"]).index(sid)])
    except Exception:  # noqa: BLE001
        sign = -1
    return -sign


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--id", type=int, default=3)
    ap.add_argument("--counts", type=int, default=200, help="position-step size for phase 2 (59 counts ~ 1 mm)")
    ap.add_argument("--strain", type=int, default=900, help="stop a phase if load in the release direction > this")
    ap.add_argument("--port", default=None)
    a = ap.parse_args()
    sid, rd = a.id, release_dir(a.id)

    try:
        arm = RCM(port=a.port, hz=50)          # no host-side load trip: we want to see the numbers
    except (RCMError, OSError) as e:
        print(f"cannot connect: {e}\n(close the ROS stack / rcm spool first)")
        return 1
    if sid not in arm.ids:
        print(f"servo {sid} not found on the bus (found {arm.ids}) -- 12 V supply on? servo cable?")
        arm.close()
        return 1

    out = ROOT / "data"
    stem = f"unspool_diag_{time.strftime('%Y%m%d_%H%M%S')}_id{sid}"
    lines: list[str] = []

    def say(s: str) -> None:
        print(s)
        lines.append(s)

    say(f"servo {sid}: release direction = {'+' if rd > 0 else '-'}counts (from arm.yaml reel_in_sign)")

    def watch(name: str, seconds: float, stop_on_strain: bool = True) -> dict:
        """Sample for `seconds`; return stats. Stops early if straining in the release direction."""
        rows, t_end = [], time.monotonic() + seconds
        strained = False
        while time.monotonic() < t_end:
            s = arm.snapshot().get(sid)
            if s is not None and not s.err:
                rows.append(s)
                if stop_on_strain and rd * s.load > a.strain:
                    strained = True
                    break
            time.sleep(0.02)
        if not rows:
            say(f"  [{name}] NO TELEMETRY from servo {sid}")
            return {"moved": 0, "strained": strained}
        p0, p1 = rows[0].pos, rows[-1].pos
        dp = p1 - p0
        dp -= 4096 if dp > 2048 else (-4096 if dp < -2048 else 0)
        ld = [r.load for r in rows]
        cur = [r.current for r in rows]
        v = [r.volt for r in rows]
        res = {"moved": dp, "strained": strained, "load_mean": round(st.fmean(ld)), "load_max": max(ld, key=abs),
               "cur_mean": round(st.fmean(cur)), "cur_max": max(cur), "v_min": round(min(v), 2),
               "temp": rows[-1].temp_C, "pos_end": p1, "moving": any(r.moving for r in rows)}
        say(f"  [{name}] pos {p0} -> {p1} (moved {dp:+d}, {'release' if dp * rd > 0 else 'REEL-IN' if dp else 'none'}) "
            f"| load mean {res['load_mean']:+d} max {res['load_max']:+d} | current mean {res['cur_mean']} max "
            f"{res['cur_max']} | Vmin {res['v_min']} | {res['temp']} C | moving flag {res['moving']}"
            + (" | STOPPED: straining in release direction" if strained else ""))
        return res

    r: dict[str, dict] = {}
    try:
        with arm.record(out / f"{stem}.csv"):
            say("phase 0: baseline (nothing commanded)")
            r["base"] = watch("baseline", 1.0, stop_on_strain=False)

            say("phase 1: torque off 1 s, torque on")
            arm.torque(sid, False)
            r["off"] = watch("torque off", 1.0, stop_on_strain=False)
            arm.torque(sid, True)
            r["on"] = watch("torque on", 0.5, stop_on_strain=False)

            say(f"phase 2: position step {rd * a.counts:+d} counts (release), slow")
            pos = arm.posmode(sid)
            if pos is None or pos < 0:
                pos = arm.snapshot()[sid].pos
            arm.move_one(sid, (pos + rd * a.counts) % 4096, speed=200, acc=20)
            r["step"] = watch("pos step", 2.5)

            say("phase 3: wheel release 300 counts/s, 2 s")
            arm.wheel(sid, rd * 300)
            r["slow"] = watch("wheel slow", 2.0)
            arm.wheel(sid, 0)
            time.sleep(0.2)

            if not r["slow"]["strained"]:
                say("phase 4: wheel release 1000 counts/s, 2 s")
                arm.wheel(sid, rd * 1000)
                r["fast"] = watch("wheel fast", 2.0)
                arm.wheel(sid, 0)
                time.sleep(0.2)
    except KeyboardInterrupt:
        say("interrupted")
    except RCMError as e:
        say(f"serial error: {e}")
    finally:
        try:
            arm.wheel(sid, 0)
            held = arm.posmode(sid)
            say(f"end: servo {sid} holding at {held} (position mode)")
        except Exception:  # noqa: BLE001
            pass

    # ---- verdict ----
    moved = sum(abs(v.get("moved", 0)) for k, v in r.items() if k in ("step", "slow", "fast"))
    rel = sum(v.get("moved", 0) * rd for k, v in r.items() if k in ("step", "slow", "fast"))
    strained = any(v.get("strained") for v in r.values())
    base = r.get("base", {})
    say("")
    if not r or "base" not in r:
        say("VERDICT: no telemetry -- check 12 V and the servo cable.")
    elif rel > 50:
        say(f"VERDICT: servo {sid} RELEASED {rel} counts (~{rel / 59:.1f} mm). It works now -- likely the torque reset "
            "cleared an overload shutdown. Use `rcm spool` (D) to let out more.")
    elif strained:
        say("VERDICT: the servo STRAINS when turning the release way -- the line on this spool is wound the other way, "
            "or it is jammed/caught. Inspect the spool; do not force it.")
    elif moved < 10 and max(v.get("cur_max", 0) for v in r.values()) < 20:
        say("VERDICT: commanded but drew ~no current and did not move -- the servo is not driving "
            "(overload/protection latched, or no motor power). Power-cycle the 12 V supply, then run this again.")
    elif moved < 10:
        say("VERDICT: drew current but did not move -- mechanically stuck (gear jam, line wedged between spool and "
            "bracket, or spool rubbing). Release tension by hand: `rcm spool`, select the servo, L (limp), turn the "
            "spool / loosen the tendon at the tip.")
    else:
        say(f"VERDICT: moved {moved} counts but not clearly in the release direction (net {rel:+d}) -- check "
            "reel_in_sign for this servo in arm.yaml.")
    say(f"baseline: pos {base.get('pos_end')} load {base.get('load_mean')} V {base.get('v_min')} temp {base.get('temp')} C")
    (out / f"{stem}_summary.txt").write_text("\n".join(lines) + "\n")
    print(f"\nlog: {out / (stem + '.csv')}\nsummary: {out / (stem + '_summary.txt')}")
    arm.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
