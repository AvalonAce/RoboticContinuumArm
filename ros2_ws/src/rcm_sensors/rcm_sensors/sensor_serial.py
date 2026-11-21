"""Serial link to the sensor board (QT Py ESP32 + ICM-20948). No ROS dependency.

Board stream: I,t_ms,ax,ay,az,gx,gy,gz,mx,my,mz,temp*HH  (see firmware_sensors/src/main.cpp)
"""
from __future__ import annotations

import glob
import os
import threading
import time
from dataclasses import dataclass
from typing import Callable

import serial


@dataclass
class ImuSample:
    t: float                 # board time, s
    accel: tuple             # m/s^2
    gyro: tuple              # rad/s
    mag: tuple               # uT (unused by the filter)
    temp: float              # degC
    host_t: float            # time.monotonic() at receipt


def find_sensor_port(skip: set[str] | frozenset = frozenset()) -> str | None:
    """Sensor board port, never the Waveshare servo board (CP210x). `skip`: real paths of ports
    already tried that sent no IMU data (e.g. the flex-only board), so the next one is returned."""
    env = os.environ.get("RCM_SENSOR_PORT")
    if env and os.path.exists(env):
        return env
    ids = sorted(glob.glob("/dev/serial/by-id/*"))
    servo = {os.path.realpath(p) for p in ids if "CP210" in p} | set(skip)
    for key in ("Adafruit", "QT_Py", "Espressif", "1a86", "CH9102", "CH340"):
        for p in ids:
            if key in p and os.path.realpath(p) not in servo:
                return p
    for p in sorted(glob.glob("/dev/ttyACM*")) + sorted(glob.glob("/dev/ttyUSB*")):
        if os.path.realpath(p) not in servo:
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


def parse_imu(body: str, host_t: float) -> ImuSample | None:
    p = body.split(",")
    if len(p) != 12 or p[0] != "I":
        return None
    v = [float(x) for x in p[1:]]
    acc, gyr = tuple(v[1:4]), tuple(v[4:7])
    if any(abs(a) > 200 for a in acc) or any(abs(g) > 40 for g in gyr):    # outside sensor ranges
        return None
    return ImuSample(v[0] / 1000.0, acc, gyr, tuple(v[7:10]), v[10], host_t)


class SensorBoard:
    def __init__(self, port: str, on_imu: Callable[[ImuSample], None],
                 on_text: Callable[[str], None] | None = None,
                 on_flex: Callable[[float, tuple, float], None] | None = None):
        self.port = port
        self.on_imu, self.on_text = on_imu, on_text or (lambda s: None)
        self.on_flex = on_flex                 # (board_t_s, (mv1, mv2, mv3), host_t); None = ignore flex
        self.bad_lines = 0
        self.link_error: str | None = None
        self.last_rx = time.monotonic()
        self._run = True
        self.ser = serial.Serial(port, 921600, timeout=0.1, exclusive=True)
        self._rx = threading.Thread(target=self._reader, daemon=True)
        self._rx.start()
        self.send("stream on")
        self.send("info")

    def send(self, line: str) -> None:
        try:
            self.ser.write((line + "\n").encode())
        except Exception as e:  # noqa: BLE001
            self.link_error = f"write failed: {e!r}"

    @property
    def link_ok(self) -> bool:
        return self.link_error is None and self._rx.is_alive()

    def _reader(self) -> None:
        buf = b""
        while self._run:
            try:
                chunk = self.ser.read(512)
            except Exception as e:  # noqa: BLE001 — unplugged, multiple access, ...
                if self._run:
                    self.link_error = f"serial read failed: {e!r}"
                return
            if not chunk:
                continue
            self.last_rx = time.monotonic()
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode(errors="replace").strip()
                if not line:
                    continue
                try:
                    if line.startswith("F,"):
                        body = verify(line)
                        if body is None:
                            self.bad_lines += 1
                        elif self.on_flex:
                            p = body.split(",")
                            self.on_flex(int(p[1]) / 1000.0, tuple(int(x) for x in p[2:5]), self.last_rx)
                    elif line.startswith("I,"):
                        body = verify(line)
                        s = parse_imu(body, self.last_rx) if body else None
                        if s is None:
                            self.bad_lines += 1
                        else:
                            self.on_imu(s)
                    else:
                        self.on_text(line)
                except Exception:  # noqa: BLE001 — a bad line (or callback bug) never kills the reader
                    self.bad_lines += 1

    def close(self) -> None:
        self._run = False
        try:
            self._rx.join(timeout=0.5)
            self.ser.close()
        except Exception:  # noqa: BLE001
            pass
