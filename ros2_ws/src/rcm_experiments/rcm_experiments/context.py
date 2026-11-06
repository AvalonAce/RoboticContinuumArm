"""Test context: the API protocols are written against. No ROS dependency.

A concrete context (the ROS runner node) implements the primitives marked ABSTRACT.
Everything protocols need beyond those (settle, sample, hold) is built here on top.
"""
from __future__ import annotations

import math
import statistics
import time


class Aborted(Exception):
    """Operator pressed Abort. The arm HOLDS its current position."""


class Tripped(Exception):
    """Bridge safety trip (load/temp/e-stop). Torque is off."""


class ProtocolError(Exception):
    """Protocol cannot continue (bad params, hardware not behaving as expected)."""


class TestContext:
    poll_s = 0.02

    # ---------------- ABSTRACT primitives ----------------
    def state(self) -> dict[int, dict]:
        """Latest servo telemetry: {id: {pos, speed, load, voltage, temperature, moving, error, goal}}"""
        raise NotImplementedError

    def tripped(self) -> str:
        raise NotImplementedError

    def aborted(self) -> bool:
        raise NotImplementedError

    def _send_bend(self, theta: float, phi: float, speed: int) -> None:
        raise NotImplementedError

    def _send_raw(self, targets: dict[int, int], speed: int) -> None:
        raise NotImplementedError

    def _ask(self, text: str, kind: str, default) -> str:
        """Block until operator answers (must call self.check() while waiting)."""
        raise NotImplementedError

    def set_zero(self, zeros: dict[int, int]) -> None:
        raise NotImplementedError

    def mark(self, event: str, **data) -> None:
        raise NotImplementedError

    def log(self, msg: str) -> None:
        raise NotImplementedError

    geometry: dict = {}          # kinematics info (ids, reel_in_sign, theta_max_deg, ...)

    # ---------------- built on primitives ----------------
    def check(self) -> None:
        if self.aborted():
            raise Aborted("aborted by operator")
        t = self.tripped()
        if t:
            raise Tripped(t)

    def sleep(self, s: float) -> None:
        end = time.monotonic() + s
        while time.monotonic() < end:
            self.check()
            time.sleep(min(self.poll_s, max(0.0, end - time.monotonic())))

    hold = sleep                 # protocol readability: ctx.hold(5)

    @property
    def ids(self) -> list[int]:
        return list(self.geometry.get("ids", sorted(self.state())))

    def bend(self, theta_deg: float, phi_deg: float = 0.0, speed: int = 200) -> None:
        self.check()
        tmax = self.geometry.get("theta_max_deg", 90.0)
        if abs(theta_deg) > tmax:
            raise ProtocolError(f"theta {theta_deg} deg > theta_max {tmax} deg")
        self.mark("cmd_bend", theta_deg=theta_deg, phi_deg=phi_deg, speed=speed)
        self._send_bend(math.radians(theta_deg), math.radians(phi_deg), int(speed))

    def raw(self, targets: dict[int, int], speed: int = 150) -> None:
        self.check()
        self._send_raw({int(k): int(v) for k, v in targets.items()}, int(speed))

    def prompt(self, text: str, kind: str = "confirm", default=None) -> str:
        """kind: confirm | number | text. Returns the operator's answer string."""
        self.mark("prompt", text=text, kind=kind)
        ans = self._ask(text, kind, default)
        if kind == "number":
            try:
                float(ans)
            except (TypeError, ValueError):
                raise ProtocolError(f"expected a number, got {ans!r}")
        self.mark("answer", value=ans)
        return ans

    def wait_fresh(self, timeout: float = 2.0) -> None:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.check()
            st = self.state()
            if st and all(i in st and not st[i]["error"] for i in self.ids):
                return
            time.sleep(self.poll_s)
        raise ProtocolError("no telemetry from all servos (bridge running? servos powered?)")

    def settle(self, timeout: float = 10.0, still_s: float = 0.3, min_wait: float = 0.15) -> dict:
        """Wait until every servo reports not-moving for `still_s`. Under load a servo may
        stop short of its goal; that steady-state error is returned (not treated as failure).
        Returns {"t_s": settle time, "err": {id: pos-goal}, "timeout": bool}."""
        t0 = time.monotonic()
        self.sleep(min_wait)
        still_since = None
        while True:
            self.check()
            st = self.state()
            now = time.monotonic()
            quiet = all(i in st and not st[i]["moving"] and abs(st[i]["speed"]) < 5 for i in self.ids)
            if quiet:
                still_since = still_since or now
                if now - still_since >= still_s:
                    break
            else:
                still_since = None
            if now - t0 > timeout:
                self.log(f"settle timeout after {timeout:.0f}s")
                break
            time.sleep(self.poll_s)
        st = self.state()
        err = {i: st[i]["pos"] - st[i]["goal"] for i in self.ids if st[i]["goal"] >= 0}
        t_settle = (still_since if quiet else time.monotonic()) - t0
        return {"t_s": round(t_settle, 3), "err": err, "timeout": not quiet}

    def sample(self, seconds: float) -> dict[int, dict]:
        """Mean/sd of load and pos per servo over `seconds` (while holding)."""
        acc: dict[int, dict[str, list]] = {i: {"load": [], "pos": [], "temperature": []} for i in self.ids}
        end = time.monotonic() + seconds
        while time.monotonic() < end:          # poll at ~telemetry rate; mean is what matters
            self.check()
            st = self.state()
            for i in self.ids:
                if i in st and not st[i]["error"]:
                    for k in acc[i]:
                        acc[i][k].append(st[i][k])
            time.sleep(self.poll_s)
        out = {}
        for i, d in acc.items():
            if not d["load"]:
                raise ProtocolError(f"no samples from servo {i}")
            out[i] = {
                "load": round(statistics.fmean(d["load"]), 1),
                "load_sd": round(statistics.pstdev(d["load"]), 1),
                "pos": round(statistics.fmean(d["pos"]), 1),
                "temperature": max(d["temperature"]),
                "n": len(d["load"]),
            }
        return out
