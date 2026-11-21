#!/usr/bin/env python3
"""Live view + sanity check for the sensor board (QT Py ESP32 + ICM-20948).

    python3 tools/imu_check.py            # live readout; after ~3 s at rest prints PASS/FAIL checks
    python3 tools/imu_check.py --raw      # print the raw CSV lines
    python3 tools/imu_check.py --log data/imu_test.csv
    rcm imu                               # same, via boot.sh

Leave the IMU still and flat for the first 3 s, then tilt/rotate it and watch the values move.
"""
from __future__ import annotations

import argparse
import glob
import math
import os
import sys
import time

import serial

FIELDS = ["t_ms", "ax", "ay", "az", "gx", "gy", "gz", "mx", "my", "mz", "temp"]
G = 9.80665


def find_sensor_port() -> str | None:
    if os.environ.get("RCM_SENSOR_PORT") and os.path.exists(os.environ["RCM_SENSOR_PORT"]):
        return os.environ["RCM_SENSOR_PORT"]
    ids = sorted(glob.glob("/dev/serial/by-id/*"))
    servo = [p for p in ids if "CP210" in p]                    # the Waveshare servo board
    for key in ("Adafruit", "QT_Py", "Espressif", "1a86", "CH9102", "CH340"):
        for p in ids:
            if key in p and p not in servo:
                return p
    servo_real = {os.path.realpath(p) for p in servo}
    for p in sorted(glob.glob("/dev/ttyACM*")) + sorted(glob.glob("/dev/ttyUSB*")):
        if os.path.realpath(p) not in servo_real:
            return p
    return None


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


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=None)
    ap.add_argument("--raw", action="store_true", help="print raw lines")
    ap.add_argument("--log", default=None, help="write parsed samples to CSV")
    ap.add_argument("--rest-s", type=float, default=3.0, help="at-rest window for the checks")
    a = ap.parse_args()

    port = a.port or find_sensor_port()
    if not port:
        print("no sensor board found (plugged in? flashed with firmware_sensors? set RCM_SENSOR_PORT)")
        return 1
    print(f"sensor board: {port}")
    try:
        ser = serial.Serial(port, 921600, timeout=0.2, exclusive=True)
    except serial.SerialException as e:
        print(f"cannot open {port}: {e}\n(another program — serial monitor, ROS sensor node — using it?)")
        return 1
    ser.write(b"stream on\ninfo\n")
    log = open(a.log, "w") if a.log else None
    if log:
        log.write("host_t," + ",".join(FIELDS) + "\n")

    buf, n, bad, t_rate, n_rate, rate = b"", 0, 0, time.monotonic(), 0, 0.0
    rest: list[list[float]] = []
    t0 = time.monotonic()
    checked = False
    last_print = 0.0
    try:
        while True:
            chunk = ser.read(512)
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode(errors="replace").strip()
                if not line:
                    continue
                if line.startswith("F,"):                     # flex readings (see flex_check.py)
                    continue
                if not line.startswith("I,"):
                    print(f"\n<< {line}")                      # status / errors from the board
                    continue
                body = verify(line)
                if body is None:
                    bad += 1
                    continue
                try:
                    v = [float(x) for x in body.split(",")[1:]]
                except ValueError:
                    bad += 1
                    continue
                if len(v) != len(FIELDS):
                    bad += 1
                    continue
                n += 1
                n_rate += 1
                if a.raw:
                    print(body)
                if log:
                    log.write(f"{time.time():.4f}," + ",".join(f"{x:g}" for x in v) + "\n")
                now = time.monotonic()
                if now - t_rate >= 1.0:
                    rate, n_rate, t_rate = n_rate / (now - t_rate), 0, now
                if now - t0 < a.rest_s:
                    rest.append(v)
                elif not checked and rest:
                    checked = True
                    report(rest, rate, bad)
                if not a.raw and now - last_print > 0.2:
                    last_print = now
                    _, ax, ay, az, gx, gy, gz, mx, my, mz, tc = v
                    am, mm = math.sqrt(ax*ax + ay*ay + az*az), math.sqrt(mx*mx + my*my + mz*mz)
                    roll = math.degrees(math.atan2(ay, az))
                    pitch = math.degrees(math.atan2(-ax, math.hypot(ay, az)))
                    sys.stdout.write(
                        f"\r{rate:5.0f} Hz | a [{ax:6.2f} {ay:6.2f} {az:6.2f}] |a| {am:5.2f} m/s2 | "
                        f"g [{math.degrees(gx):6.1f} {math.degrees(gy):6.1f} {math.degrees(gz):6.1f}] dps | "
                        f"|m| {mm:5.1f} uT | roll {roll:6.1f} pitch {pitch:6.1f} | {tc:4.1f} C | bad {bad}   ")
                    sys.stdout.flush()
            if not chunk and time.monotonic() - t0 > 3 and n == 0:
                print("no IMU data yet — waiting (check the board's status lines above)")
                t0 = time.monotonic()
    except KeyboardInterrupt:
        print()
    finally:
        ser.close()
        if log:
            log.close()
            print(f"logged {n} samples -> {a.log}")
    return 0


def report(rest: list[list[float]], rate: float, bad: int) -> None:
    import statistics as st
    col = list(zip(*rest))
    amag = [math.sqrt(r[1]**2 + r[2]**2 + r[3]**2) for r in rest]
    mmag = [math.sqrt(r[7]**2 + r[8]**2 + r[9]**2) for r in rest]
    gbias = [math.degrees(st.fmean(col[k])) for k in (4, 5, 6)]
    gnoise = max(math.degrees(st.pstdev(col[k])) for k in (4, 5, 6))
    anoise = max(st.pstdev(col[k]) for k in (1, 2, 3))
    checks = [
        ("sample rate >= 90 Hz", rate >= 90, f"{rate:.0f} Hz"),
        ("|accel| ~ 1 g at rest", abs(st.fmean(amag) - G) < 0.6, f"{st.fmean(amag):.2f} m/s2"),
        ("accel noise small", anoise < 0.1, f"sd {anoise:.3f} m/s2"),
        ("gyro bias small", max(map(abs, gbias)) < 3.0, "bias " + " ".join(f"{b:+.2f}" for b in gbias) + " dps"),
        ("gyro noise small", gnoise < 0.5, f"sd {gnoise:.2f} dps"),
        ("|mag| plausible (20-70 uT)", 20 <= st.fmean(mmag) <= 70, f"{st.fmean(mmag):.1f} uT"),
        ("no corrupted lines", bad == 0, f"{bad} bad"),
    ]
    print(f"\n\n== at-rest check ({len(rest)} samples; keep it STILL for this part) ==")
    for name, ok, det in checks:
        print(f"  [{'PASS' if ok else 'FAIL'}] {name:<28} {det}")
    print("  (|mag| fails near motors/magnets/steel — expected; we may not use mag on the arm)\n")


if __name__ == "__main__":
    sys.exit(main())
