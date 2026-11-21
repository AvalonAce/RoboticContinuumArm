import math
import random

import pytest

from rcm_sensors.imu_filter import (G, ImuProcessor, angle_between, gravity_in_sensor, q_from_axis_angle,
                                    q_mul, q_norm, roll_pitch_deg)

DT = 0.01


def simulate(proc, q_true, omega_body, seconds, bias=(0, 0, 0), noise=0.0, t0=0.0, rng=None):
    """Feed a rigid-body motion (constant body rate) into the processor; returns (out, q_true, t)."""
    rng = rng or random.Random(1)
    t, out = t0, None
    for _ in range(int(seconds / DT)):
        w = math.sqrt(sum(c * c for c in omega_body))
        if w > 0:
            q_true = q_norm(q_mul(q_true, q_from_axis_angle(omega_body, w * DT)))
        acc = tuple(G * c + rng.gauss(0, noise) for c in gravity_in_sensor(q_true))
        gyr = tuple(o + b + rng.gauss(0, noise * 0.01) for o, b in zip(omega_body, bias))
        t += DT
        out = proc.update(t, acc, gyr) or out
    return out, q_true, t


def test_calibrates_bias_when_still():
    p = ImuProcessor()
    simulate(p, (1, 0, 0, 0), (0, 0, 0), 2.5, bias=(0.01, -0.02, 0.005), noise=0.02)
    assert p.calibrated
    assert p.bias == pytest.approx((0.01, -0.02, 0.005), abs=2e-3)


def test_no_calibration_while_moving():
    p = ImuProcessor()
    simulate(p, (1, 0, 0, 0), (0.5, 0, 0), 3.0)
    assert not p.calibrated and "moving" in p.message


@pytest.mark.parametrize("axis,deg", [((1, 0, 0), 30), ((0, 1, 0), -50), ((1, 1, 0), 70)])
def test_converges_to_static_tilt(axis, deg):
    q_true = q_from_axis_angle(axis, math.radians(deg))
    p = ImuProcessor()
    out, _, _ = simulate(p, q_true, (0, 0, 0), 6.0, noise=0.03)
    err = angle_between(gravity_in_sensor(out.q), gravity_in_sensor(q_true))
    assert err < 0.5


def test_tracks_rotation_and_tilt_from_reference():
    p = ImuProcessor()
    _, q, t = simulate(p, (1, 0, 0, 0), (0, 0, 0), 4.0)          # calibrate + settle
    assert p.set_reference()
    out, q, t = simulate(p, q, (math.radians(30), 0, 0), 2.0, t0=t)  # 60 deg about sensor x
    out, q, t = simulate(p, q, (0, 0, 0), 1.0, t0=t)
    assert out.tilt_from_ref_deg == pytest.approx(60, abs=1.5)
    roll, _ = roll_pitch_deg(out.q)
    assert roll == pytest.approx(60, abs=1.5)


def test_pure_yaw_gives_zero_tilt():
    p = ImuProcessor()
    _, q, t = simulate(p, (1, 0, 0, 0), (0, 0, 0), 4.0)
    p.set_reference()
    out, q, t = simulate(p, q, (0, 0, math.radians(45)), 2.0, t0=t)   # spin about vertical
    assert out.tilt_from_ref_deg < 1.0


def test_reference_requires_calibration():
    assert not ImuProcessor().set_reference()
