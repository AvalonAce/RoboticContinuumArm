#!/usr/bin/env python3
"""Live view of the flex sensors (sensor-board firmware v1: F,t_ms,mv1,mv2,mv3 lines).

    python3 tools/flex_check.py        (or: rcm flex)
    python3 tools/flex_check.py --port /dev/ttyACM1 --log data/flex_test.csv

Finds the sensor board that is streaming flex data (tries each QT Py port), then shows per channel:
voltage, change since start, and a bar. Keep sensors flat for the first second (baseline), then bend.
Wiring: 3V -> flex -> pin -> 10-15 kOhm -> GND. Flat ~1.65 V (10k) / ~2.0 V (15k); bending LOWERS it.
Close the ROS stack first if it is holding the board's port.
"""
from __future__ import annotations

import argparse
import glob
import os
import statistics
import sys
import time

import serial

PINS = ("A2", "A3", "TX")


def verify(line: str) -> str | None:
    if "*" not in line:
        return None
    body, _, cs = line.rpartition("*")
    x = 0
    for ch in body.encode(errors="replace"):
        x ^= ch
    try:
        return body if int(cs, 16) == x else None
    except ValueError:
        return None


def candidates() -> list[str]:
    out, seen = [], set()
    for p in sorted(glob.glob("/dev/serial/by-id/*")):
        if any(k in p for k in ("Adafruit", "QT_Py", "Espressif")):
            out.append(p)
            seen.add(os.path.realpath(p))
    out += [p for p in sorted(glob.glob("/dev/ttyACM*")) if os.path.realpath(p) not in seen]
    return out


def probe(p: str, wait_s: float) -> tuple[serial.Serial | None, bool, bool, str]:
    """Open p, ask it to stream, report (serial, has_flex, has_imu, note)."""
    try:
        s = serial.Serial(p, 921600, timeout=0.1, exclusive=True)
    except serial.SerialException as e:
        return None, False, False, f"busy ({'in use -- ROS stack running?' if 'lock' in str(e) else e})"
    s.write(b"stream on\n")
    buf, end = b"", time.monotonic() + wait_s
    while time.monotonic() < end:
        buf += s.read(512)
    has_f, has_i = b"\nF," in buf, b"\nI," in buf
    return s, has_f, has_i, ("flex" if has_f else "no flex data") + (" + IMU" if has_i else "")


def find_flex_board(wait_s: float = 1.5) -> tuple[serial.Serial | None, list[str]]:
    """Every sensor board streams F lines (unconnected pins float), so prefer the board WITHOUT an IMU:
    that is the flex board. The IMU board is only used if it is the only one."""
    notes, flex_only, with_imu = [], [], []
    for p in candidates():
        s, has_f, has_i, note = probe(p, wait_s)
        notes.append(f"{p}: {note}")
        if s is None:
            continue
        if has_f and not has_i:
            flex_only.append(s)
        elif has_f:
            with_imu.append(s)
        else:
            s.close()
    pick = (flex_only or with_imu or [None])[0]
    for s in flex_only + with_imu:
        if s is not pick:
            s.close()
    if pick is not None and pick in with_imu:
        notes.append("WARNING: only the IMU board is streaming flex data -- is the flex board plugged in / flashed?")
    if len(flex_only) > 1:
        notes.append("several flex-only boards found; using the first (use --port to choose)")
    return pick, notes


def status(mv: float, sd: float) -> str:
    if mv > 3050:
        return "≈3.3 V: resistor to GND missing"
    if mv < 80:
        return "≈0 V: flex/3V not connected"
    if sd > 60:
        return "noisy: floating? check joints"
    return "ok"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=None)
    ap.add_argument("--log", default=None)
    a = ap.parse_args()

    if a.port:
        try:
            ser = serial.Serial(a.port, 921600, timeout=0.1, exclusive=True)
            ser.write(b"stream on\n")
        except serial.SerialException as e:
            print(f"cannot open {a.port}: {e}")
            return 1
    else:
        ser, notes = find_flex_board()
        if ser is None:
            print("no board streaming flex data found:\n  " + "\n  ".join(notes or ["no QT Py ports present"]))
            return 1
        print("boards:\n  " + "\n  ".join(notes))
    print(f"flex board: {ser.port}   (keep sensors FLAT for 1 s, then bend; Ctrl+C to quit)")
    print("Channels with NO sensor wired float and copy a neighbour's voltage -- only trust a channel that\n"
          "changes when YOU bend its sensor (watch Δ and the bar).\n")
    log = open(a.log, "w") if a.log else None
    if log:
        log.write("host_t,t_ms,mv_a2,mv_a3,mv_tx\n")

    buf, base, hist, n, t0, last = b"", None, [[], [], []], 0, time.monotonic(), 0.0
    lo, hi = [9999] * 3, [0] * 3
    try:
        while True:
            buf += ser.read(512)
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode(errors="replace").strip()
                if not line.startswith("F,"):
                    continue
                body = verify(line)
                if body is None:
                    continue
                p = body.split(",")
                mv = [int(x) for x in p[2:5]]
                n += 1
                if log:
                    log.write(f"{time.time():.4f},{p[1]},{mv[0]},{mv[1]},{mv[2]}\n")
                for k in range(3):
                    hist[k] = (hist[k] + [mv[k]])[-25:]
                    lo[k], hi[k] = min(lo[k], mv[k]), max(hi[k], mv[k])
                now = time.monotonic()
                if base is None and now - t0 > 1.0 and len(hist[0]) >= 10:
                    base = [statistics.fmean(h) for h in hist]
                if now - last > 0.1:
                    last = now
                    rows = []
                    for k in range(3):
                        v = statistics.fmean(hist[k])
                        sd = statistics.pstdev(hist[k]) if len(hist[k]) > 1 else 0
                        d = (v - base[k]) if base else 0.0
                        bar = "█" * int(max(0, min(30, -d / 20)))        # 20 mV per block, bending -> grows
                        rows.append(f"  {PINS[k]}: {v / 1000:5.2f} V  Δ{d:+6.0f} mV  [{bar:<30}]  "
                                    f"range {lo[k]/1000:.2f}–{hi[k]/1000:.2f} V  {status(v, sd)}")
                    sys.stdout.write("\x1b[3A" if n > 1 and getattr(main, "_drawn", False) else "")
                    sys.stdout.write("\n".join(r.ljust(110) for r in rows) + "\n")
                    sys.stdout.flush()
                    main._drawn = True
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        if log:
            log.close()
            print(f"logged {n} samples -> {a.log}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
