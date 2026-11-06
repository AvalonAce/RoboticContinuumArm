"""Test protocols. Each is `fn(ctx, p)`, with defaults in DEFAULTS[name].

Write new ones here and register them in PROTOCOLS; add a YAML in ../protocols/ to expose
them in the dashboard. Conventions:
  * angles in degrees, payloads in grams, speeds in servo counts/s
  * ctx.mark(...) every event an analysis script will need to segment the telemetry
  * leave the arm holding a safe pose at the end (usually straight, payload removed)
"""
from __future__ import annotations

from .context import ProtocolError, TestContext, Tripped

DEFAULTS: dict[str, dict] = {}
PROTOCOLS: dict[str, callable] = {}


def protocol(name: str, **defaults):
    def deco(fn):
        PROTOCOLS[name] = fn
        DEFAULTS[name] = defaults
        fn.__protocol_name__ = name
        return fn
    return deco


def resolve(name: str, overrides: dict | None) -> dict:
    if name not in PROTOCOLS:
        raise ProtocolError(f"unknown protocol '{name}' (known: {sorted(PROTOCOLS)})")
    p = dict(DEFAULTS[name])
    unknown = set(overrides or {}) - set(p)
    if unknown:
        raise ProtocolError(f"unknown params for {name}: {sorted(unknown)} (valid: {sorted(p)})")
    p.update(overrides or {})
    return p


def _grams(ctx: TestContext, text: str, nominal: float) -> float:
    return float(ctx.prompt(f"{text}\nEnter MEASURED total payload mass in grams.", "number", nominal))


# --------------------------------------------------------------------------- calibrate
@protocol("calibrate", step_counts=6, load_threshold=60, max_travel_counts=1200,
          speed=150, sample_s=0.3, verify_hold_s=2.0, rest_tolerance_counts=300)
def calibrate(ctx: TestContext, p: dict) -> dict:
    """Pretension all tendons together in small steps; each tendon stops independently when
    its |load| rises `load_threshold` above its idle baseline. That pose = calibration zero
    (straight, all tendons at equal tension). Stored via /rcm/set_zero."""
    ctx.prompt("Tendons attached with slight slack, arm hanging STRAIGHT, NO payload.\n"
               "Servos will slowly reel in. Continue?")
    ctx.wait_fresh()
    st = ctx.state()
    cur = {i: st[i]["pos"] for i in ctx.ids}

    # Tendons should have been wound with the servos at the rest pose (Setup -> Go to rest).
    rest = dict(zip(ctx.geometry.get("ids", ctx.ids), ctx.geometry.get("rest_counts", [])))
    off = {i: cur[i] - rest[i] for i in ctx.ids if i in rest}
    far = {i: d for i, d in off.items() if abs(d) > p["rest_tolerance_counts"]}
    ctx.mark("calib_rest_check", rest=rest, offset=off)
    if far:
        ctx.prompt(f"WARNING: servos {sorted(far)} are {far} counts from the rest pose.\n"
                   "If tendons were wound away from rest, the arm may not reach full bend in every\n"
                   "direction (single-turn range). Abort, Go to rest, re-wind -- or Continue anyway.")
    start = dict(cur)
    sign = dict(zip(ctx.geometry.get("ids", ctx.ids), ctx.geometry.get("reel_in_sign", [1] * len(ctx.ids))))

    ctx.raw(cur, p["speed"])              # torque on, hold where they are
    ctx.settle(timeout=3)
    base = ctx.sample(p["sample_s"] * 2)
    baseline = {i: abs(base[i]["load"]) for i in ctx.ids}
    ctx.mark("calib_baseline", baseline=baseline, start=start)
    ctx.log(f"baseline |load|: {baseline}")

    zeros: dict[int, int] = {}
    while len(zeros) < len(ctx.ids):
        active = [i for i in ctx.ids if i not in zeros]
        for i in active:
            cur[i] += sign[i] * p["step_counts"]
            if abs(cur[i] - start[i]) > p["max_travel_counts"]:
                raise ProtocolError(f"tendon {i} never got taut within {p['max_travel_counts']} counts "
                                    f"— check winding direction (reel_in_sign) / attachment")
        ctx.raw({i: cur[i] for i in active}, p["speed"])
        ctx.settle(timeout=3, still_s=0.1)
        m = ctx.sample(p["sample_s"])
        for i in active:
            rise = abs(m[i]["load"]) - baseline[i]
            if rise >= p["load_threshold"]:
                zeros[i] = int(round(m[i]["pos"]))
                ctx.mark("taut", id=i, pos=zeros[i], load=m[i]["load"], travel=zeros[i] - start[i])
                ctx.log(f"tendon {i} taut at {zeros[i]} (travel {zeros[i] - start[i]:+d}, load {m[i]['load']})")

    ctx.set_zero(zeros)
    ctx.mark("calibrated", zeros=zeros)
    rng = ctx.geometry.get("range") or {}
    if rng:
        ctx.mark("range_check", **rng)
        bad = {i: t for i, t in rng["tendons"].items() if not t["ok"]}
        ctx.log(f"reachable theta with these zeros: {rng['theta_reach_deg']} deg "
                f"(theta_max {ctx.geometry.get('theta_max_deg')})")
        if bad:
            ctx.prompt("RANGE WARNING: " + "; ".join(
                f"tendon {i} zero {t['zero']} leaves {t['room_counts']} counts (need {rng['need_counts']}), "
                f"reach {t['theta_reach_deg']} deg" for i, t in bad.items())
                + "\nBends beyond that will be rejected. Re-wind at rest to fix, or Continue.")
    ctx.bend(0, 0, p["speed"])
    ctx.settle()
    v = ctx.sample(p["verify_hold_s"])
    loads = {i: v[i]["load"] for i in ctx.ids}
    ctx.mark("calib_verify", loads=loads)
    ctx.log(f"straight-pose loads after calibration: {loads}")
    ctx.prompt("Calibration stored. Is the arm straight with all tendons taut?")
    return {"zeros": zeros, "verify_loads": loads}


# --------------------------------------------------------------------------- static load
@protocol("static_load", configs=[{"theta": 30, "phi": 0}], payloads_g=[0, 50, 100, 150, 200],
          settle_s=3.0, hold_s=5.0, speed=200)
def static_load(ctx: TestContext, p: dict) -> dict:
    """Incremental static loading: at each configuration, step the payload up; record
    steady-state tendon loads and position error (tendon-estimate vs command)."""
    results = []
    for ci, cfg in enumerate(p["configs"]):
        th, ph = float(cfg["theta"]), float(cfg.get("phi", 0))
        ctx.prompt(f"Config {ci}: theta={th} deg phi={ph} deg. Remove any payload. Move there?")
        ctx.bend(th, ph, p["speed"])
        ctx.settle()
        for g in p["payloads_g"]:
            grams = _grams(ctx, f"Config {ci} (theta {th}): attach payload {g} g.", g) if g else 0.0
            ctx.mark("payload", grams=grams, config=ci, theta=th, phi=ph)
            ctx.hold(p["settle_s"])
            ctx.mark("hold_start", grams=grams, config=ci)
            m = ctx.sample(p["hold_s"])
            st = ctx.state()
            err = {i: st[i]["pos"] - st[i]["goal"] for i in ctx.ids}
            ctx.mark("hold_end", grams=grams, config=ci, mean=m, pos_err=err)
            ctx.log(f"cfg {ci} {grams:.0f} g: loads {[m[i]['load'] for i in ctx.ids]} err {list(err.values())}")
            results.append({"config": ci, "grams": grams, "loads": {i: m[i]["load"] for i in ctx.ids}, "err": err})
    ctx.prompt("Remove payload. Return straight?")
    ctx.bend(0, 0, p["speed"])
    ctx.settle()
    return {"steps": len(results)}


# --------------------------------------------------------------------------- lift cycles
def _lift(ctx, p, k, speed, tag):
    ctx.mark("cycle_start", k=k, speed=speed, tag=tag)
    ctx.bend(p["theta_high"], p["phi"], speed)
    up = ctx.settle(timeout=p.get("settle_timeout_s", 15))
    ctx.mark("top", k=k, speed=speed, settle_s=up["t_s"], err=up["err"], tag=tag)
    ctx.hold(p["dwell_s"])
    ctx.bend(p["theta_low"], p["phi"], speed)
    dn = ctx.settle(timeout=p.get("settle_timeout_s", 15))
    ctx.mark("bottom", k=k, speed=speed, settle_s=dn["t_s"], err=dn["err"], tag=tag)
    ctx.hold(p["dwell_s"])
    return up, dn


@protocol("lift_cycles", payload_g=100, theta_low=0, theta_high=45, phi=0, cycles=10,
          speed=300, dwell_s=2.0, settle_timeout_s=15)
def lift_cycles(ctx: TestContext, p: dict) -> dict:
    """Repeated lifts at a sub-limit payload: repeatability + drift across cycles."""
    ctx.bend(p["theta_low"], p["phi"], p["speed"])
    ctx.settle()
    grams = _grams(ctx, f"Attach payload ({p['payload_g']} g nominal).", p["payload_g"])
    ctx.mark("payload", grams=grams)
    tops = []
    for k in range(int(p["cycles"])):
        up, _ = _lift(ctx, p, k, p["speed"], "cycle")
        tops.append(up["err"])
        ctx.log(f"cycle {k + 1}/{p['cycles']}: top err {list(up['err'].values())}, settle {up['t_s']} s")
    ctx.prompt("Cycles done. Remove payload?")
    return {"cycles": len(tops), "grams": grams}


# --------------------------------------------------------------------------- sustained hold
@protocol("hold", payload_g=100, theta=45, phi=0, duration_s=180, speed=200, report_every_s=15)
def hold(ctx: TestContext, p: dict) -> dict:
    """Sustained hold: creep, load drift, temperature rise."""
    ctx.bend(0, p["phi"], p["speed"])
    ctx.settle()
    grams = _grams(ctx, f"Attach payload ({p['payload_g']} g nominal).", p["payload_g"])
    ctx.mark("payload", grams=grams)
    ctx.bend(p["theta"], p["phi"], p["speed"])
    ctx.settle()
    ctx.mark("hold_start", grams=grams, duration_s=p["duration_s"])
    n = max(1, int(p["duration_s"] // p["report_every_s"]))
    for k in range(n):
        m = ctx.sample(p["duration_s"] / n)
        ctx.mark("hold_progress", k=k, mean=m)
        ctx.log(f"hold {(k + 1) * p['duration_s'] / n:.0f}/{p['duration_s']} s: loads "
                f"{[m[i]['load'] for i in ctx.ids]} temps {[m[i]['temperature'] for i in ctx.ids]}")
    ctx.mark("hold_end", grams=grams)
    ctx.bend(0, p["phi"], p["speed"])
    ctx.settle()
    ctx.prompt("Hold done, arm lowered. Remove payload?")
    return {"grams": grams, "duration_s": p["duration_s"]}


# --------------------------------------------------------------------------- speed sweep
@protocol("speed_sweep", payload_g=100, speeds=[100, 200, 400, 800], reps=3,
          theta_low=0, theta_high=45, phi=0, dwell_s=2.0, settle_timeout_s=15)
def speed_sweep(ctx: TestContext, p: dict) -> dict:
    """Lifting speed effect on overshoot / settling / tracking."""
    ctx.bend(p["theta_low"], p["phi"], min(p["speeds"]))
    ctx.settle()
    grams = _grams(ctx, f"Attach payload ({p['payload_g']} g nominal).", p["payload_g"])
    ctx.mark("payload", grams=grams)
    for v in p["speeds"]:
        for r in range(int(p["reps"])):
            up, dn = _lift(ctx, p, r, int(v), "speed")
            ctx.log(f"speed {v}: rep {r + 1}/{p['reps']} settle up {up['t_s']} s / down {dn['t_s']} s")
    ctx.prompt("Sweep done. Remove payload?")
    return {"grams": grams}


# --------------------------------------------------------------------------- limit / failure
@protocol("limit", theta=45, phi=0, payloads_g=[50, 100, 150, 200, 300, 400, 500], hold_s=10.0, speed=200)
def limit(ctx: TestContext, p: dict) -> dict:
    """Increase payload until failure (operator-declared or safety trip). Documents the
    failure mode -> upper bound of the operating envelope."""
    passed = []
    for g in p["payloads_g"]:
        ctx.bend(0, p["phi"], p["speed"])
        ctx.settle()
        grams = _grams(ctx, f"Arm straight. Attach payload {g} g.", g)
        ctx.mark("payload", grams=grams)
        try:
            ctx.bend(p["theta"], p["phi"], p["speed"])
            r = ctx.settle(timeout=15)
            ctx.mark("hold_start", grams=grams)
            m = ctx.sample(p["hold_s"])
            ctx.mark("hold_end", grams=grams, mean=m, err=r["err"])
        except Tripped as e:
            ctx.mark("failure", grams=grams, mode=f"safety trip: {e}")
            ctx.log(f"FAILURE at {grams} g: safety trip ({e}) — torque is off")
            return {"failed_at_g": grams, "mode": f"trip: {e}", "passed_g": passed}
        ans = ctx.prompt(f"{grams} g held at theta {p['theta']}. Type 'ok' to continue, or describe "
                         "the failure (tendon slip, stall, buckling, twist, ...)", "text", "ok")
        if ans.strip().lower() not in ("ok", "y", "yes", ""):
            ctx.mark("failure", grams=grams, mode=ans)
            ctx.bend(0, p["phi"], p["speed"])
            ctx.settle()
            return {"failed_at_g": grams, "mode": ans, "passed_g": passed}
        passed.append(grams)
    ctx.bend(0, p["phi"], p["speed"])
    ctx.settle()
    ctx.log("no failure up to the largest payload in the list")
    return {"failed_at_g": None, "passed_g": passed}
