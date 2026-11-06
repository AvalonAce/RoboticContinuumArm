import math
import pytest
from rcm_kinematics.cc import (ArmGeometry, OutOfRange, bend_to_counts, counts_to_bend, range_report,
                               rest_counts, tendon_dl, tip_position, travel_needed_counts, geometry_from_dict)

G = ArmGeometry(length_mm=200, tendon_radius_mm=15, spool_radius_mm=10,
                zero_counts=[2000, 2100, 1900], reel_in_sign=[1, -1, 1])


def test_straight_is_zero():
    assert bend_to_counts(G, 0.0, 0.0) == [2000, 2100, 1900]


def test_dl_sums_to_zero_for_symmetric_tendons():
    for th in (0.2, 0.7, 1.2):
        for ph in (0, 1, 2.5, -2):
            assert abs(sum(tendon_dl(G, th, ph))) < 1e-9


def test_bending_toward_tendon1_shortens_it():
    dl = tendon_dl(G, math.radians(30), 0.0)
    assert dl[0] < 0 and dl[1] > 0 and dl[2] > 0
    assert dl[0] == pytest.approx(-math.radians(30) * 15)


@pytest.mark.parametrize("th_deg,ph_deg", [(10, 0), (30, 45), (60, 170), (85, -100), (45, 240)])
def test_round_trip(th_deg, ph_deg):
    c = bend_to_counts(G, math.radians(th_deg), math.radians(ph_deg))
    e = counts_to_bend(G, c)
    tol = math.radians(0.5)       # count quantization
    assert e.theta == pytest.approx(math.radians(th_deg), abs=tol)
    dphi = (e.phi - math.radians(ph_deg) + math.pi) % (2 * math.pi) - math.pi
    assert abs(dphi) < math.radians(1.5)
    assert abs(e.common_mode_mm) < 0.05


def test_sign_convention_counts():
    c = bend_to_counts(G, math.radians(30), 0.0)
    assert c[0] > 2000            # tendon 1 reels in, sign +1 -> counts up
    assert c[1] > 2100            # tendon 2 pays out, sign -1 -> counts up


def test_negative_theta_flips_plane():
    assert bend_to_counts(G, -0.5, 0.0) == bend_to_counts(G, 0.5, math.pi)


def test_limits():
    with pytest.raises(OutOfRange):
        bend_to_counts(G, math.radians(95), 0)
    big = ArmGeometry(length_mm=200, tendon_radius_mm=40, spool_radius_mm=2, theta_max_deg=180)
    with pytest.raises(OutOfRange):
        bend_to_counts(big, math.radians(170), 0)


def test_tip_position():
    assert tip_position(G, 0, 0) == (0, 0, 0.2)
    x, y, z = tip_position(G, math.pi / 2, 0)
    assert x == pytest.approx(0.4 / math.pi) and z == pytest.approx(0.4 / math.pi) and y == pytest.approx(0)


def test_common_mode_detects_slack():
    e = counts_to_bend(G, [2000 - 30, 2100 + 30, 1900 - 30])   # all paid out 30 counts
    assert e.common_mode_mm == pytest.approx(30 * G.mm_per_count)
    assert e.theta == pytest.approx(0, abs=1e-6)


def test_geometry_from_dict_with_calibration():
    g = geometry_from_dict({"length_mm": 200, "tendon_radius_mm": 15, "spool_radius_mm": 10, "junk": 1},
                           {"zero_counts": {1: 2010, 2: 2020, 3: 2030}})
    assert g.zero_counts == [2010, 2020, 2030]


def test_rest_offsets_toward_payout():
    r = rest_counts(G)          # signs [1, -1, 1]
    off = round(3.0 / G.mm_per_count)
    assert r == [2048 - off, 2048 + off, 2048 - off]


def test_travel_needed_90deg():
    g = ArmGeometry(length_mm=200, tendon_radius_mm=15, spool_radius_mm=10)
    assert travel_needed_counts(g) == math.ceil(math.radians(90) * 15 / g.mm_per_count)   # ~1536


def test_range_report_flags_badly_wound_tendon():
    g = ArmGeometry(length_mm=200, tendon_radius_mm=15, spool_radius_mm=10)
    rep = range_report(g, [2048, 2100, 600])
    assert rep["tendons"][1]["ok"] and rep["tendons"][2]["ok"] and not rep["tendons"][3]["ok"]
    assert not rep["ok"] and rep["theta_reach_deg"] < 40
