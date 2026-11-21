"""IMU processing, no ROS dependency: gyro-bias calibration -> 6-axis Madgwick -> derived angles.

Conventions
  * quaternion q = (w, x, y, z), Hamilton, maps SENSOR frame -> WORLD frame (same as
    sensor_msgs/Imu.orientation). World z is up. No magnetometer, so yaw (rotation about
    world z) is unobservable and drifts slowly; roll/pitch and tilt are drift-free.
  * accel in m/s^2 (reads +g on the axis pointing UP at rest), gyro in rad/s.

Derived quantities (kept model-agnostic on purpose; the arm model maps them to theta later)
  * relative orientation q_rel = q_ref^-1 * q  (rotation since the reference pose)
  * tilt_from_ref: angle between the gravity direction now and at the reference pose, in the
    sensor frame. Independent of yaw drift. For a bend in a vertical plane this equals the
    tip's rotation since the reference.
"""
from __future__ import annotations

import math
import statistics
from dataclasses import dataclass, field

G = 9.80665


# ---------------------------------------------------------------- quaternion helpers
def q_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def q_conj(q):
    return (q[0], -q[1], -q[2], -q[3])


def q_norm(q):
    n = math.sqrt(sum(c * c for c in q)) or 1.0
    return tuple(c / n for c in q)


def q_from_axis_angle(axis, angle):
    n = math.sqrt(sum(c * c for c in axis)) or 1.0
    s = math.sin(angle / 2) / n
    return (math.cos(angle / 2), axis[0] * s, axis[1] * s, axis[2] * s)


def q_rotate(q, v):
    """Rotate vector v by q (sensor -> world)."""
    p = q_mul(q_mul(q, (0.0, *v)), q_conj(q))
    return p[1:]


def gravity_in_sensor(q):
    """World 'up' expressed in the sensor frame (what a still accelerometer reads, normalized)."""
    w, x, y, z = q
    return (2 * (x * z - w * y), 2 * (w * x + y * z), w * w - x * x - y * y + z * z)


def q_from_accel(ax, ay, az):
    """Orientation with yaw = 0 whose gravity matches a still accelerometer reading."""
    roll = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.hypot(ay, az))
    cr, sr, cp, sp = math.cos(roll / 2), math.sin(roll / 2), math.cos(pitch / 2), math.sin(pitch / 2)
    return (cr * cp, sr * cp, cr * sp, -sr * sp)


def roll_pitch_deg(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    return math.degrees(roll), math.degrees(pitch)


def angle_between(u, v) -> float:
    d = sum(a * b for a, b in zip(u, v))
    nu = math.sqrt(sum(a * a for a in u)) or 1.0
    nv = math.sqrt(sum(b * b for b in v)) or 1.0
    return math.degrees(math.acos(max(-1.0, min(1.0, d / (nu * nv)))))


# ---------------------------------------------------------------- Madgwick (IMU / 6-axis)
class Madgwick:
    """Madgwick gradient-descent orientation filter, accelerometer + gyroscope only.
    beta trades gyro trust (low) vs accel correction (high). ~0.03-0.1 typical."""

    def __init__(self, beta: float = 0.08):
        self.beta = beta
        self.q = (1.0, 0.0, 0.0, 0.0)

    def reset(self, q=(1.0, 0.0, 0.0, 0.0)):
        self.q = q_norm(q)

    def update(self, gyro, accel, dt: float, beta: float | None = None):
        b = self.beta if beta is None else beta
        q0, q1, q2, q3 = self.q
        gx, gy, gz = gyro
        qd0 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
        qd1 = 0.5 * (q0 * gx + q2 * gz - q3 * gy)
        qd2 = 0.5 * (q0 * gy - q1 * gz + q3 * gx)
        qd3 = 0.5 * (q0 * gz + q1 * gy - q2 * gx)
        ax, ay, az = accel
        an = math.sqrt(ax * ax + ay * ay + az * az)
        if an > 1e-6:
            ax, ay, az = ax / an, ay / an, az / an
            _2q0, _2q1, _2q2, _2q3 = 2 * q0, 2 * q1, 2 * q2, 2 * q3
            _4q0, _4q1, _4q2 = 4 * q0, 4 * q1, 4 * q2
            _8q1, _8q2 = 8 * q1, 8 * q2
            q0q0, q1q1, q2q2, q3q3 = q0 * q0, q1 * q1, q2 * q2, q3 * q3
            s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay
            s1 = _4q1 * q3q3 - _2q3 * ax + 4 * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az
            s2 = 4 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az
            s3 = 4 * q1q1 * q3 - _2q1 * ax + 4 * q2q2 * q3 - _2q2 * ay
            sn = math.sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3)
            if sn > 1e-12:
                qd0 -= b * s0 / sn
                qd1 -= b * s1 / sn
                qd2 -= b * s2 / sn
                qd3 -= b * s3 / sn
        self.q = q_norm((q0 + qd0 * dt, q1 + qd1 * dt, q2 + qd2 * dt, q3 + qd3 * dt))
        return self.q


# ---------------------------------------------------------------- full pipeline
@dataclass
class ImuOutput:
    state: str                      # calibrating | ok
    q: tuple
    q_rel: tuple
    has_reference: bool
    roll_deg: float
    pitch_deg: float
    tilt_from_ref_deg: float
    gyro: tuple                     # bias-corrected rad/s
    accel: tuple
    bias: tuple
    message: str = ""


@dataclass
class ImuProcessor:
    beta: float = 0.08
    bias_s: float = 2.0             # still time needed for gyro-bias calibration
    still_gyro_sd: float = 0.02     # rad/s (~1.1 dps)
    still_accel_tol: float = 0.6    # m/s^2 from 1 g
    still_accel_sd: float = 0.15    # m/s^2: gravity direction must not change
    max_bias: float = 0.1           # rad/s (~5.7 dps): larger "bias" means it was actually turning
    converge_s: float = 1.0         # high-beta settling after calibration
    filt: Madgwick = field(init=False)
    bias: tuple = (0.0, 0.0, 0.0)
    calibrated: bool = False
    q_ref: tuple | None = None
    _win: list = field(default_factory=list)
    _t_last: float | None = None
    _t_cal: float = 0.0
    message: str = "hold the IMU still to calibrate the gyro"

    def __post_init__(self):
        self.filt = Madgwick(self.beta)

    def recalibrate(self):
        self.calibrated, self._win, self.message = False, [], "hold the IMU still to calibrate the gyro"

    def set_reference(self) -> bool:
        if not self.calibrated:
            return False
        self.q_ref = self.filt.q
        return True

    def clear_reference(self):
        self.q_ref = None

    def update(self, t: float, accel, gyro) -> ImuOutput | None:
        """t: board time in seconds (monotonic). Returns None while calibrating."""
        if not self.calibrated:
            self._win.append((t, accel, gyro))
            while self._win and t - self._win[0][0] > self.bias_s:
                self._win.pop(0)
            if self._win and t - self._win[0][0] >= self.bias_s * 0.95 and len(self._win) > 20:
                gs = list(zip(*[w[2] for w in self._win]))
                amag = [math.sqrt(sum(c * c for c in w[1])) for w in self._win]
                acs = list(zip(*[w[1] for w in self._win]))
                still = (max(statistics.pstdev(c) for c in gs) < self.still_gyro_sd
                         and max(abs(statistics.fmean(c)) for c in gs) < self.max_bias  # slow steady turn != bias
                         and max(statistics.pstdev(c) for c in acs) < self.still_accel_sd
                         and abs(statistics.fmean(amag) - G) < self.still_accel_tol)
                if still:
                    self.bias = tuple(statistics.fmean(c) for c in gs)
                    a = [statistics.fmean(c) for c in zip(*[w[1] for w in self._win])]
                    self.filt.reset(q_from_accel(*a))
                    self.calibrated, self._t_cal, self._t_last = True, t, t
                    self.message = "gyro bias calibrated"
                else:
                    self.message = "moving — hold the IMU still to calibrate the gyro"
            return None

        dt = t - self._t_last if self._t_last is not None else 0.01
        self._t_last = t
        if not 0 < dt < 0.5:                         # gap / reorder: don't integrate garbage
            dt = 0.01
        g = tuple(gi - bi for gi, bi in zip(gyro, self.bias))
        beta = self.beta * 10 if t - self._t_cal < self.converge_s else None
        q = self.filt.update(g, accel, dt, beta)
        roll, pitch = roll_pitch_deg(q)
        if self.q_ref is not None:
            q_rel = q_norm(q_mul(q_conj(self.q_ref), q))
            tilt = angle_between(gravity_in_sensor(self.q_ref), gravity_in_sensor(q))
        else:
            q_rel, tilt = (1.0, 0.0, 0.0, 0.0), float("nan")
        return ImuOutput("ok", q, q_rel, self.q_ref is not None, roll, pitch, tilt, g, tuple(accel),
                         self.bias, self.message)
