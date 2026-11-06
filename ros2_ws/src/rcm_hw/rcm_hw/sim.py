"""Simulated Waveshare board: speaks the RCM firmware text protocol on a pseudo-terminal.

Lets the whole stack (bridge, kinematics, runner, dashboard) run with no hardware:
    ros2 launch rcm_bringup rcm.launch.py sim:=true
or standalone:
    python3 -m rcm_hw.sim          # prints the pty path; connect RCM(port=...) to it

Model (deliberately crude — for exercising software, not physics):
  * servos slew toward goal at the commanded speed when torque is on
  * each tendon goes taut at a hidden position; reeling in past it raises load linearly
    (so the calibration protocol has something real to find)
  * temperature creeps up with load; voltage sags slightly with total load
"""
from __future__ import annotations

import os
import pty
import random
import threading
import time
import tty


class FakeBoard:
    def __init__(self, ids=(1, 2, 3), reel_in_sign=None, stiffness=0.6, seed=0):
        # Fault injection for testing (env RCM_SIM_FAULTS="corrupt=0.02,dropout=20:3"):
        #   corrupt=p    each telemetry line has probability p of a byte dropped/flipped
        #   dropout=t:d  at t s after start, go silent for d s (simulates a dead USB link)
        f = dict(kv.split("=", 1) for kv in os.environ.get("RCM_SIM_FAULTS", "").split(",") if "=" in kv)
        self.corrupt = float(f.get("corrupt", 0))
        self.dropout = tuple(map(float, f["dropout"].split(":"))) if "dropout" in f else None
        self.t_start = time.monotonic()
        rnd = random.Random(seed)
        self.ids = list(ids)
        sign = reel_in_sign or {i: 1 for i in self.ids}
        self.srv = {
            i: dict(pos=float(1750 + rnd.randint(-40, 40)), goal=None, spd=0.0, torque=False,
                    taut=1950 + rnd.randint(-60, 60), sign=sign.get(i, 1), temp=24.0)
            for i in self.ids
        }
        self.k = stiffness
        self.stream_hz = 0
        self._lock = threading.Lock()
        self._run = False
        self.master, self.slave = pty.openpty()
        tty.setraw(self.slave)
        self.port = os.ttyname(self.slave)

    # -- helpers --
    def _silent(self) -> bool:
        if not self.dropout:
            return False
        t = time.monotonic() - self.t_start
        return self.dropout[0] <= t < self.dropout[0] + self.dropout[1]

    def _tel(self, body: str) -> None:
        x = 0
        for ch in body.encode():
            x ^= ch
        line = f"{body}*{x:02X}"
        if self.corrupt and random.random() < self.corrupt:
            k = random.randrange(len(line))
            line = line[:k] + line[k + 1:] if random.random() < 0.5 else line[:k] + "7" + line[k + 1:]
        self._out(line)

    def _out(self, line: str) -> None:
        if self._silent():
            return
        try:
            os.write(self.master, (line + "\n").encode())
        except OSError:
            pass

    def _reel(self, s) -> float:
        return (s["pos"] - s["taut"]) * s["sign"]          # counts reeled in past taut

    def _load(self, s) -> int:
        # Coupled tendons: differential reel-in just bends the arm (soft, kb); common-mode
        # reel-in stretches everything (stiff, k). Crude, but calibration + bending both behave.
        mean = sum(self._reel(x) for x in self.srv.values()) / len(self.srv)
        tension = max(0.0, self.k * mean + 0.12 * self.k * (self._reel(s) - mean))
        moving = s["goal"] is not None and s["torque"] and abs(s["goal"] - s["pos"]) > 0.5
        friction = 25 if moving else 0
        return int(round((tension + friction + random.uniform(-3, 3)) * s["sign"]))

    # -- threads --
    def _physics(self) -> None:
        last = time.monotonic()
        last_tx = 0.0
        while self._run:
            time.sleep(0.002)
            now = time.monotonic()
            dt, last = now - last, now
            with self._lock:
                for s in self.srv.values():
                    if s["goal"] is not None and s["torque"]:
                        d = s["goal"] - s["pos"]
                        step = s["spd"] * dt
                        s["pos"] = s["goal"] if abs(d) <= step else s["pos"] + step * (1 if d > 0 else -1)
                    s["temp"] += dt * (abs(self._load(s)) / 1000.0 * 0.05 - (s["temp"] - 24.0) * 0.002)
                if self.stream_hz and now - last_tx >= 1.0 / self.stream_hz:
                    last_tx = now
                    total = sum(abs(self._load(s)) for s in self.srv.values())
                    volt = int(round(124 - total / 300))
                    for i, s in self.srv.items():
                        mv = int(s["goal"] is not None and s["torque"] and abs(s["goal"] - s["pos"]) > 0.5)
                        ld = self._load(s)
                        self._tel(f"{int(now * 1000)},{i},{round(s['pos'])},{int(s['spd']) if mv else 0},"
                                  f"{ld},{volt},{int(s['temp'])},{abs(ld) // 2},{mv}")

    def _console(self) -> None:
        buf = b""
        self._out("RCM bring-up console v0. type `help`")
        self._out(f"found {len(self.ids)}: " + " ".join(map(str, self.ids)))
        while self._run:
            try:
                chunk = os.read(self.master, 256)
            except OSError:
                break
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                self._cmd(raw.decode(errors="replace").strip())

    def _cmd(self, line: str) -> None:
        t = line.split()
        if not t:
            return
        with self._lock:
            c = t[0]
            if c == "scan":
                self._out(f"found {len(self.ids)}: " + " ".join(map(str, self.ids)))
            elif c == "stream":
                if len(t) > 1 and t[1] == "off":
                    self.stream_hz = 0
                    self._out("stream off")
                else:
                    self.stream_hz = max(1, min(int(t[2]) if len(t) > 2 else 20, 200))
                    self._out("t_ms,id,pos,speed,load,volt_dV,temp_C,current,moving")
            elif c == "sync" and len(t) >= 5:
                spd = float(t[1]) or 1000.0
                vals = list(map(int, t[3:]))
                n = 0
                for i, p in zip(vals[::2], vals[1::2]):
                    if i in self.srv:
                        self.srv[i].update(goal=max(0, min(4095, p)), spd=spd, torque=True)
                        n += 1
                self._out(f"sync {n} servos (spd {int(spd)} acc {t[2]})")
            elif c == "pos" and len(t) >= 3:
                i, p = int(t[1]), int(t[2])
                spd = float(t[3]) if len(t) > 3 else 1000.0
                if i in self.srv:
                    self.srv[i].update(goal=p, spd=spd, torque=True)
                self._out(f"move {i} -> {p} (spd {int(spd)} acc 50)")
            elif c == "torque" and len(t) >= 3:
                i, on = int(t[1]), bool(int(t[2]))
                if i in self.srv:
                    self.srv[i]["torque"] = on
                    if not on:
                        self.srv[i]["goal"] = None
                self._out(f"torque {i} = {int(on)}")
            elif c == "stop":
                for s in self.srv.values():
                    s["torque"], s["goal"] = False, None
                self._out("all torque off")
            elif c == "help":
                self._out("scan | sync | pos | torque | stream | stop  (simulated)")
            else:
                self._out(f"? [{line}] (type help)")

    # -- lifecycle --
    def start(self) -> str:
        self._run = True
        threading.Thread(target=self._physics, daemon=True).start()
        threading.Thread(target=self._console, daemon=True).start()
        return self.port

    def stop(self) -> None:
        self._run = False


def main() -> None:
    fb = FakeBoard()
    print(fb.start(), flush=True)
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        fb.stop()


if __name__ == "__main__":
    main()
