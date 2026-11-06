#!/usr/bin/env python3
"""Quick multi-servo bus test. Run with TENDONS DETACHED.

    python3 tools/motor_test.py              # expects IDs 1 2 3
    python3 tools/motor_test.py --ids 1 2 3 --amp 30 --yes

Checks, per servo: responds, supply voltage, temperature, idle load,
individual move accuracy/time; then all servos together via SyncWrite
(start-time spread). Saves raw telemetry to data/motor_test_<ts>.csv.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from rcm_serial import CENTER, RCM, RCMError, counts_to_deg, deg_to_counts  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]

# Pass thresholds (tune once you know your hardware)
V_MIN, V_MAX = 6.0, 12.6           # supply window, volts
T_MAX = 55                         # °C
POS_TOL = 15                       # counts (~1.3°)
IDLE_LOAD_MAX = 100                # |load| at rest, unloaded
START_SPREAD_MAX_MS = 60           # sync move: max spread in motion onset


class Report:
    def __init__(self):
        self.rows: list[tuple[str, bool, str]] = []

    def add(self, name: str, ok: bool, detail: str = "") -> bool:
        self.rows.append((name, ok, detail))
        print(f"  [{'PASS' if ok else 'FAIL'}] {name:<34} {detail}")
        return ok

    @property
    def ok(self) -> bool:
        return all(r[1] for r in self.rows)


def onset_times(arm: RCM, ids, start_pos: dict[int, int], thresh: int = 10,
                timeout: float = 3.0) -> dict[int, float]:
    """Host time at which each servo first moved > thresh counts from start."""
    t0 = time.monotonic()
    onset: dict[int, float] = {}
    while len(onset) < len(ids) and time.monotonic() - t0 < timeout:
        snap = arm.snapshot()
        for i in ids:
            if i not in onset and i in snap and abs(snap[i].pos - start_pos[i]) > thresh:
                onset[i] = snap[i].host_t
        time.sleep(0.002)
    return onset


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ids", type=int, nargs="+", default=[1, 2, 3])
    ap.add_argument("--amp", type=float, default=30.0, help="sweep amplitude, degrees (default 30)")
    ap.add_argument("--speed", type=int, default=800)
    ap.add_argument("--port", default=None)
    ap.add_argument("--yes", action="store_true", help="skip tendon-detached confirmation")
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args()

    if not a.yes:
        ans = input(f"Servos {a.ids} will move ±{a.amp:.0f}° around CENTER (2048). "
                    "Tendons detached? [y/N] ")
        if ans.strip().lower() != "y":
            print("aborted")
            return 1

    amp = deg_to_counts(a.amp)
    rep = Report()
    out = ROOT / "data" / f"motor_test_{time.strftime('%Y%m%d_%H%M%S')}.csv"

    try:
        with RCM(port=a.port, hz=50, load_limit=900, verbose=a.verbose) as arm:
            print(f"board {arm.port}\n\n== bus ==")
            found = set(arm.ids)
            missing = [i for i in a.ids if i not in found]
            rep.add("all IDs present", not missing,
                    f"found {sorted(found)}" + (f", missing {missing}" if missing else ""))
            ids = [i for i in a.ids if i in found]
            if not ids:
                raise RCMError("no servos to test")

            with arm.record(out):
                arm.wait_fresh(ids)
                snap = arm.snapshot()

                print("\n== health (idle) ==")
                for i in ids:
                    s = snap[i]
                    rep.add(f"id {i} supply voltage", V_MIN <= s.volt <= V_MAX, f"{s.volt:.1f} V")
                    rep.add(f"id {i} temperature", s.temp_C <= T_MAX, f"{s.temp_C} °C")

                print("\n== center all (sync) ==")
                arm.move({i: CENTER for i in ids}, speed=a.speed)
                errs = arm.wait_until_stopped(ids, tol=POS_TOL)
                for i in ids:
                    rep.add(f"id {i} reaches center", abs(errs[i]) <= POS_TOL, f"err {errs[i]:+d}")
                time.sleep(0.3)
                snap = arm.snapshot()
                for i in ids:
                    rep.add(f"id {i} idle load", abs(snap[i].load) <= IDLE_LOAD_MAX, f"{snap[i].load:+d}")

                print(f"\n== individual sweeps ±{a.amp:.0f}° ==")
                for i in ids:
                    for tgt in (CENTER + amp, CENTER - amp, CENTER):
                        t = time.monotonic()
                        arm.move({i: tgt}, speed=a.speed)
                        e = arm.wait_until_stopped([i], tol=POS_TOL)[i]
                        dt = time.monotonic() - t
                        rep.add(f"id {i} -> {tgt}", abs(e) <= POS_TOL,
                                f"err {e:+d} ({counts_to_deg(e):+.1f}°), {dt*1000:.0f} ms")
                    # others must not have moved
                    snap = arm.snapshot()
                    still = all(abs(snap[j].pos - CENTER) <= POS_TOL for j in ids if j != i)
                    rep.add(f"id {i} moves alone (no cross-talk)", still)

                print("\n== synchronized move ==")
                for tgt in (CENTER + amp, CENTER - amp):
                    start = {i: arm.snapshot()[i].pos for i in ids}
                    arm.move({i: tgt for i in ids}, speed=a.speed)
                    on = onset_times(arm, ids, start)
                    errs = arm.wait_until_stopped(ids, tol=POS_TOL)
                    if len(on) == len(ids):
                        spread = (max(on.values()) - min(on.values())) * 1000
                        rep.add(f"sync -> {tgt} onset spread", spread <= START_SPREAD_MAX_MS,
                                f"{spread:.0f} ms (telemetry-limited)")
                    else:
                        rep.add(f"sync -> {tgt} all started", False, f"moved: {sorted(on)}")
                    rep.add(f"sync -> {tgt} all reach", all(abs(e) <= POS_TOL for e in errs.values()),
                            " ".join(f"{i}:{e:+d}" for i, e in errs.items()))

                arm.move({i: CENTER for i in ids}, speed=a.speed)
                arm.wait_until_stopped(ids, tol=POS_TOL)
    except KeyboardInterrupt:
        print("\ninterrupted — torque off")
        return 130
    except RCMError as e:
        rep.add("run completed", False, str(e))

    print(f"\ntelemetry -> {out}")
    n_fail = sum(not r[1] for r in rep.rows)
    print(f"RESULT: {'PASS' if rep.ok else 'FAIL'}  ({len(rep.rows) - n_fail}/{len(rep.rows)} checks)")
    print("torque is OFF (servos are free).")
    return 0 if rep.ok else 2


if __name__ == "__main__":
    sys.exit(main())
