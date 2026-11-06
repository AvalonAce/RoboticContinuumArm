"""Host-side driver for the RCM bring-up firmware (text console over USB serial).

    from rcm_serial import RCM
    with RCM() as arm:                       # opens RCM_PORT, waits for boot, starts telemetry
        arm.move({1: 2500, 2: 1800})         # simultaneous (SyncWrite)
        arm.wait_until_stopped()
        with arm.record("data/trial.csv"):
            ...
    # leaving the block (or Ctrl+C / exception) -> torque off on all servos

Telemetry comes from the firmware's `stream all <hz>` CSV:
    t_ms,id,pos,speed,load,volt_dV,temp_C,current,moving
A background thread parses it into `arm.state[id]`.
"""
from __future__ import annotations

import contextlib
import glob
import os
import queue
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import serial  # pyserial

CENTER = 2048
COUNTS_PER_REV = 4096
FIELDS = ("t_ms", "id", "pos", "speed", "load", "volt_dV", "temp_C", "current", "moving")


class RCMError(RuntimeError):
    pass


class LoadLimitTripped(RCMError):
    pass


@dataclass
class ServoState:
    id: int
    t_ms: int = 0
    pos: int = 0
    speed: int = 0
    load: int = 0          # signed output duty, ~‰ of max (tension proxy)
    volt_dV: int = 0
    temp_C: int = 0
    current: int = 0
    moving: int = 0
    host_t: float = 0.0    # time.monotonic() when received
    err: bool = False

    @property
    def volt(self) -> float:
        return self.volt_dV / 10.0


def find_port() -> str | None:
    if os.environ.get("RCM_PORT") and os.path.exists(os.environ["RCM_PORT"]):
        return os.environ["RCM_PORT"]
    for pat in ("/dev/serial/by-id/*CP210*", "/dev/ttyUSB*", "/dev/ttyACM*"):
        hits = sorted(glob.glob(pat))
        if hits:
            return hits[0]
    return None


@dataclass
class _Recorder:
    fh: object
    ids: set[int] | None = None
    rows: int = 0


class RCM:
    def __init__(self, port: str | None = None, hz: int = 50, load_limit: int | None = None,
                 verbose: bool = False, boot_timeout: float = 3.0):
        self.port = port or find_port()
        if not self.port:
            raise RCMError("no board found (set RCM_PORT or plug in USB)")
        self.hz = hz
        self.load_limit = load_limit
        self.verbose = verbose
        self.state: dict[int, ServoState] = {}
        self.ids: list[int] = []
        self.targets: dict[int, int] = {}

        self._msgs: queue.Queue[str] = queue.Queue()
        self._wlock = threading.Lock()
        self._slock = threading.Lock()
        self._rec: _Recorder | None = None
        self._tripped: str | None = None
        self._run = True
        self._rx_bytes = 0
        self._recent: list[str] = []          # last non-telemetry lines, for error messages
        self.bad_lines = 0                     # corrupted / failed-checksum telemetry lines dropped
        self.link_error: str | None = None     # set if the reader thread stops (port gone, etc.)
        self.last_rx = time.monotonic()
        self.trip_samples = 3                  # consecutive over-limit samples before a load trip
        self._over: dict[int, int] = {}

        # Opening the port pulses DTR/RTS -> the board's auto-reset circuit reboots the ESP32.
        # We embrace that: it guarantees a known state, and the boot banner tells us the IDs.
        self.ser = serial.Serial(self.port, 115200, timeout=0.05, exclusive=True)
        self._rx = threading.Thread(target=self._reader, daemon=True)
        self._rx.start()

        boot = self.expect("found", timeout=boot_timeout)
        if boot is None:                       # no reset happened -> ask explicitly
            self.ids = self.scan()
        else:
            self.ids = self._parse_found(boot)
        if hz:
            self.stream(hz)

    # ---------------- low level ----------------
    def send(self, line: str) -> None:
        if self.verbose:
            print(f">> {line}")
        with self._wlock:
            self.ser.write((line + "\n").encode())

    def expect(self, prefix: str, timeout: float = 1.0) -> str | None:
        """Wait for a non-telemetry line starting with `prefix`; discard others."""
        end = time.monotonic() + timeout
        while (left := end - time.monotonic()) > 0:
            try:
                msg = self._msgs.get(timeout=left)
            except queue.Empty:
                break
            if prefix in msg:                  # tolerate a corrupted fragment glued in front
                return msg
        return None

    def command(self, line: str, reply: str | None = None, timeout: float = 1.0) -> str | None:
        self._check_trip()
        self._drain()
        self.send(line)
        if reply is None:
            return None
        r = self.expect(reply, timeout)
        if r is None:
            raise RCMError(f"no '{reply}' reply to '{line}' — {self.diag()}")
        return r

    def _drain(self) -> None:
        while not self._msgs.empty():
            self._msgs.get_nowait()

    def _reader(self) -> None:
        buf = b""
        while self._run:
            try:
                chunk = self.ser.read(256)
            except Exception as e:  # noqa: BLE001 — port vanished, multiple access, ...
                if self._run:
                    self.link_error = f"serial read failed: {e!r}"
                return
            if not chunk:
                continue
            buf += chunk
            self._rx_bytes += len(chunk)
            self.last_rx = time.monotonic()
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                try:
                    self._handle(raw.decode(errors="replace").strip())
                except Exception:  # noqa: BLE001 — one bad line must never kill the reader
                    self.bad_lines += 1

    @staticmethod
    def _verify(line: str) -> str | None:
        """Strip and check '*HH' checksum. Lines without one (older firmware) pass through."""
        if "*" not in line:
            return line
        body, _, cs = line.rpartition("*")
        x = 0
        for ch in body.encode(errors="replace"):
            x ^= ch
        try:
            return body if int(cs, 16) == x else None
        except ValueError:
            return None

    @staticmethod
    def _plausible(v: list[int]) -> bool:
        _, sid, pos, spd, load, volt, temp, cur, mov = v
        return (0 <= sid <= 253 and 0 <= pos <= 4095 and abs(spd) <= 10000 and abs(load) <= 1000
                and 0 <= volt <= 200 and -10 <= temp <= 120 and mov in (0, 1))

    def _handle(self, line: str) -> None:
        if not line:
            return
        if line[0].isdigit() and line.count(",") >= 2:   # telemetry row
            body = self._verify(line)
            if body is None:
                self.bad_lines += 1
                return
            parts = body.split(",")
            now = time.monotonic()
            if len(parts) == 3 and parts[2] == "ERR":
                sid = int(parts[1])
                with self._slock:
                    s = self.state.setdefault(sid, ServoState(sid))
                    s.err, s.host_t = True, now
                return
            if len(parts) != len(FIELDS):
                self.bad_lines += 1
                return
            v = [int(x) for x in parts]                    # ValueError -> counted by caller
            if not self._plausible(v):
                self.bad_lines += 1
                return
            sid = v[1]
            with self._slock:
                s = self.state.setdefault(sid, ServoState(sid))
                (s.t_ms, _, s.pos, s.speed, s.load, s.volt_dV, s.temp_C, s.current, s.moving) = v
                s.err, s.host_t = False, now
                if self._rec and (self._rec.ids is None or sid in self._rec.ids):
                    self._rec.fh.write(f"{now:.4f},{body}\n")
                    self._rec.rows += 1
            # load trip only after N consecutive over-limit samples (one glitch != a stall)
            if self.load_limit is not None and abs(v[4]) > self.load_limit:
                self._over[sid] = self._over.get(sid, 0) + 1
                if self._over[sid] >= self.trip_samples and not self._tripped:
                    self._tripped = f"servo {sid} load {v[4]} > {self.load_limit}"
                    self.send("stop")
            else:
                self._over[sid] = 0
            return
        if line.startswith("t_ms,"):                     # stream header
            return
        if self.verbose:
            print(f"<< {line}")
        self._recent = (self._recent + [line])[-5:]
        self._msgs.put(line)

    def _check_trip(self) -> None:
        if self._tripped:
            raise LoadLimitTripped(self._tripped)

    @staticmethod
    def _parse_found(line: str) -> list[int]:
        # "found 3: 1 2 3"
        return [int(x) for x in line.split(":", 1)[1].split()] if ":" in line else []

    @property
    def link_ok(self) -> bool:
        return self.link_error is None and self._rx.is_alive()

    def diag(self) -> str:
        if self.link_error or not self._rx.is_alive():
            return f"serial link down: {self.link_error or 'reader stopped'}"
        if self._rx_bytes == 0:
            return (f"received 0 bytes from {self.port}: board not running our firmware, "
                    "or another program (serial console?) is reading the port")
        return f"received {self._rx_bytes} bytes; last lines: {self._recent}"

    # ---------------- commands ----------------
    def scan(self) -> list[int]:
        was = self.hz
        if was:
            self.send("stream off")
            time.sleep(0.05)
        r = self.command("scan", reply="found", timeout=5.0)
        self.ids = self._parse_found(r)
        if was:
            self.stream(was)
        return self.ids

    def stream(self, hz: int | None) -> None:
        if not hz:
            self.send("stream off")
        else:
            self.send(f"stream all {hz}")
        self.hz = hz or 0

    def torque(self, sid: int, on: bool) -> None:
        self.command(f"torque {sid} {int(on)}", reply="torque")

    def move(self, targets: dict[int, int], speed: int = 800, acc: int = 30) -> None:
        """Simultaneous move via SyncWrite. targets: {id: raw_pos 0..4095}."""
        tg = {int(k): max(0, min(4095, int(v))) for k, v in targets.items()}
        args = " ".join(f"{k} {v}" for k, v in tg.items())
        self.command(f"sync {speed} {acc} {args}", reply="sync")
        self.targets.update(tg)

    def move_one(self, sid: int, pos: int, speed: int = 800, acc: int = 30) -> None:
        self.move({sid: pos}, speed, acc)

    def stop(self) -> None:
        """Torque off on all servos (soft e-stop). Never raises."""
        try:
            self.send("stop")
        except Exception:
            pass

    @property
    def tripped(self) -> str | None:
        return self._tripped

    def trip(self, reason: str) -> None:
        """Externally triggered trip (e.g. temp limit, e-stop): torque off, block commands."""
        if not self._tripped:
            self._tripped = reason
        self.stop()

    def reset_trip(self) -> None:
        self._tripped = None

    # ---------------- state helpers ----------------
    def snapshot(self) -> dict[int, ServoState]:
        with self._slock:
            return {k: ServoState(**vars(v)) for k, v in self.state.items()}

    def wait_fresh(self, ids=None, timeout: float = 1.0) -> None:
        """Block until every id has a telemetry sample newer than now."""
        ids = list(ids or self.ids)
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            snap = self.snapshot()
            if all(i in snap and snap[i].host_t > t0 for i in ids):
                return
            time.sleep(0.005)
        raise RCMError(f"no fresh telemetry for {ids} (stream on? servos powered?)")

    def wait_until_stopped(self, ids=None, tol: int = 15, timeout: float = 5.0,
                           settle: float = 0.08) -> dict[int, int]:
        """Wait until each servo is within `tol` counts of its target and not moving.
        Returns final position error per id. Raises on timeout or load trip."""
        ids = list(ids or self.targets.keys())
        time.sleep(settle)                     # give the move time to start
        t0 = time.monotonic()
        while True:
            self._check_trip()
            snap = self.snapshot()
            errs = {i: snap[i].pos - self.targets[i] for i in ids if i in snap}
            done = len(errs) == len(ids) and all(
                abs(errs[i]) <= tol and not snap[i].moving for i in ids)
            if done:
                return errs
            if time.monotonic() - t0 > timeout:
                raise RCMError(f"timeout; pos errors {errs}")
            time.sleep(0.01)

    @contextlib.contextmanager
    def record(self, path: str | os.PathLike, ids=None):
        """Write all telemetry rows to CSV while inside the block."""
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        fh = open(p, "w", buffering=1)
        fh.write("host_t," + ",".join(FIELDS) + "\n")
        self._rec = _Recorder(fh, set(ids) if ids else None)
        try:
            yield p
        finally:
            rec, self._rec = self._rec, None
            fh.close()
            if self.verbose:
                print(f"recorded {rec.rows} rows -> {p}")

    # ---------------- lifecycle ----------------
    def close(self) -> None:
        self.stop()
        time.sleep(0.05)
        self._run = False
        self._rx.join(timeout=0.5)
        try:
            self.ser.close()
        except Exception:  # noqa: BLE001
            pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False


def counts_to_deg(c: float) -> float:
    return c * 360.0 / COUNTS_PER_REV


def deg_to_counts(d: float) -> int:
    return round(d * COUNTS_PER_REV / 360.0)
