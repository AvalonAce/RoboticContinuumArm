"""Constant-curvature (CC) kinematics for one tendon-driven section. No ROS dependency.

Model (Webster & Jones 2010, single section, tendons at radius d, angles theta_i):
    tendon length   l_i = L * (1 - kappa * d * cos(theta_i - phi))
    =>  dl_i = l_i - L = -theta * d * cos(theta_i - phi)          (theta = kappa * L)
Tendons on the inside of the bend shorten (dl < 0 -> winch reels in).

Servo mapping (winch spool radius r, 4096 counts/rev, calibrated zero z_i, sign s_i):
    counts_i = z_i + s_i * (-dl_i - takeup) / r * 4096 / (2*pi)
z_i is the straight, pretensioned pose found by the calibration protocol. s_i = +1 if
increasing counts reels tendon i IN (shortens it); set per servo from how it's wound.

Inverse (from encoder counts): with 3+ symmetric tendons,
    a = -(2/N) * sum(dl_i cos theta_i) = theta d cos(phi)
    b = -(2/N) * sum(dl_i sin theta_i) = theta d sin(phi)
This is the configuration IMPLIED BY TENDON LENGTHS: it ignores tendon stretch and
payload-induced deflection. The gap between it and a measured tip pose (IMU) is the
quantity the lifting-dynamics study is about.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

COUNTS_PER_REV = 4096


@dataclass
class ArmGeometry:
    length_mm: float                      # section length L
    tendon_radius_mm: float               # d: backbone axis -> tendon
    spool_radius_mm: float                # r: winch spool (tendon centerline)
    ids: list[int] = field(default_factory=lambda: [1, 2, 3])
    angles_deg: list[float] = field(default_factory=lambda: [0.0, 120.0, 240.0])
    reel_in_sign: list[int] = field(default_factory=lambda: [1, 1, 1])
    zero_counts: list[int] = field(default_factory=lambda: [2048, 2048, 2048])
    takeup_mm: float = 0.0                # extra common-mode reel-in (pretension trim)
    theta_max_deg: float = 90.0
    pretension_allowance_mm: float = 3.0  # expected reel-in from winding slack to taut (rest offset)

    def __post_init__(self):
        n = len(self.ids)
        for name in ("angles_deg", "reel_in_sign", "zero_counts"):
            if len(getattr(self, name)) != n:
                raise ValueError(f"{name} must have {n} entries (one per tendon id)")
        if min(self.length_mm, self.tendon_radius_mm, self.spool_radius_mm) <= 0:
            raise ValueError("geometry lengths must be > 0")

    @property
    def mm_per_count(self) -> float:
        return 2 * math.pi * self.spool_radius_mm / COUNTS_PER_REV


class OutOfRange(ValueError):
    pass


def tendon_dl(g: ArmGeometry, theta: float, phi: float) -> list[float]:
    """Tendon length change [mm] for bend angle theta [rad] in plane phi [rad]."""
    return [-theta * g.tendon_radius_mm * math.cos(math.radians(a) - phi) for a in g.angles_deg]


def dl_to_counts(g: ArmGeometry, dl_mm: list[float], check: bool = True) -> list[int]:
    out = []
    for i, dl, z, s in zip(g.ids, dl_mm, g.zero_counts, g.reel_in_sign):
        c = z + s * (-dl - g.takeup_mm) / g.mm_per_count
        c = int(round(c))
        if check and not 0 <= c < COUNTS_PER_REV:
            raise OutOfRange(f"tendon {i}: {c} counts outside single-turn range 0..4095 "
                             f"(needs multi-turn mode or a bigger spool)")
        out.append(c)
    return out


def counts_to_dl(g: ArmGeometry, counts: list[int]) -> list[float]:
    """Encoder-implied tendon length change [mm] relative to calibration zero.
    (A commanded takeup shows up here as negative common mode.)"""
    return [-(c - z) * s * g.mm_per_count for c, z, s in zip(counts, g.zero_counts, g.reel_in_sign)]


def bend_to_counts(g: ArmGeometry, theta: float, phi: float) -> list[int]:
    if abs(theta) > math.radians(g.theta_max_deg) + 1e-9:
        raise OutOfRange(f"theta {math.degrees(theta):.1f} deg exceeds theta_max {g.theta_max_deg} deg")
    if theta < 0:                         # canonical form: theta >= 0, flip plane
        theta, phi = -theta, phi + math.pi
    return dl_to_counts(g, tendon_dl(g, theta, phi))


@dataclass
class BendEstimate:
    theta: float          # rad
    phi: float            # rad
    kappa: float          # 1/m
    tip_m: tuple[float, float, float]
    dl_mm: list[float]
    common_mode_mm: float


def counts_to_bend(g: ArmGeometry, counts: list[int]) -> BendEstimate:
    dl = counts_to_dl(g, counts)
    n = len(dl)
    cm = sum(dl) / n
    dd = [x - cm for x in dl]             # remove common mode (stretch/slack) before fitting
    a = -(2.0 / n) * sum(x * math.cos(math.radians(t)) for x, t in zip(dd, g.angles_deg))
    b = -(2.0 / n) * sum(x * math.sin(math.radians(t)) for x, t in zip(dd, g.angles_deg))
    theta = math.hypot(a, b) / g.tendon_radius_mm
    phi = math.atan2(b, a) if theta > 1e-9 else 0.0
    return BendEstimate(theta, phi, theta / (g.length_mm / 1000.0), tip_position(g, theta, phi), dl, cm)


def tip_position(g: ArmGeometry, theta: float, phi: float) -> tuple[float, float, float]:
    """CC tip position [m] in the base frame (z along the straight backbone)."""
    L = g.length_mm / 1000.0
    if abs(theta) < 1e-6:
        return (0.0, 0.0, L)
    r = L / theta
    rho = r * (1 - math.cos(theta))
    return (rho * math.cos(phi), rho * math.sin(phi), r * math.sin(theta))


CENTER = COUNTS_PER_REV // 2


def travel_needed_counts(g: ArmGeometry) -> int:
    """Counts each tendon must move (both directions) from zero to reach theta_max."""
    dl = math.radians(g.theta_max_deg) * g.tendon_radius_mm + abs(g.takeup_mm)
    return int(math.ceil(dl / g.mm_per_count))


def rest_counts(g: ArmGeometry) -> list[int]:
    """Pose to hold while winding/tying tendons. Center, shifted toward PAY-OUT by the expected
    pretension reel-in, so the calibrated zero ends up near center (max travel both ways)."""
    off = g.pretension_allowance_mm / g.mm_per_count
    return [int(round(CENTER - s * off)) for s in g.reel_in_sign]


def range_report(g: ArmGeometry, zeros: list[int] | None = None) -> dict:
    """Per tendon: counts available each way from zero and the theta reachable (deg)."""
    zeros = zeros if zeros is not None else g.zero_counts
    need = travel_needed_counts(g)
    out = {"need_counts": need, "tendons": {}}
    for i, z in zip(g.ids, zeros):
        room = min(z, COUNTS_PER_REV - 1 - z)
        reach = math.degrees(max(0.0, room * g.mm_per_count - abs(g.takeup_mm)) / g.tendon_radius_mm)
        out["tendons"][i] = {"zero": z, "room_counts": room, "theta_reach_deg": round(reach, 1),
                             "ok": room >= need}
    out["ok"] = all(t["ok"] for t in out["tendons"].values())
    out["theta_reach_deg"] = min(t["theta_reach_deg"] for t in out["tendons"].values())
    return out


def geometry_from_dict(d: dict, calibration: dict | None = None) -> ArmGeometry:
    keys = ArmGeometry.__dataclass_fields__.keys()
    g = {k: v for k, v in d.items() if k in keys}
    if calibration and "zero_counts" in calibration:
        z = calibration["zero_counts"]
        ids = g.get("ids", [1, 2, 3])
        g["zero_counts"] = [int(z[i] if isinstance(z, dict) else z[k]) for k, i in enumerate(ids)]
    return ArmGeometry(**g)
