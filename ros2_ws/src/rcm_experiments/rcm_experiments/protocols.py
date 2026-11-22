"""Test protocols. Each is `fn(ctx, p)`, with defaults in DEFAULTS[name].

Write new ones here and register them in PROTOCOLS; add a YAML in ../protocols/ to expose
them in the dashboard. Conventions:
  * angles in degrees, payloads in grams, speeds in servo counts/s
  * ctx.mark(...) every event an analysis script will need to segment the telemetry
  * leave the arm holding a safe pose at the end (usually straight, payload removed)
"""
from __future__ import annotations

import time

from .context import TIMEOUT, ProtocolError, TestContext, Tripped

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
@protocol("calibrate", mode="auto", step_counts=6, load_threshold=60, free_steps=5, confirm_steps=3,
          max_travel_counts=1200, speed=150, sample_s=0.3, verify_hold_s=2.0, rest_tolerance_counts=300)
def calibrate(ctx: TestContext, p: dict) -> dict:
    """Find the straight, pretensioned pose and store it as the calibration zero (/rcm/set_zero).

    mode=auto:   reel all tendons in small steps. The first `free_steps` steps (through slack) set
                 each servo's moving baseline |load|; a tendon counts as taut once its load stays
                 `load_threshold` above that baseline for `confirm_steps` consecutive steps
                 (zero = position at the first of those steps).
    mode=manual: the operator reels each tendon in by hand from the prompt until it feels taut.
    """
    ctx.prompt("Tendons attached, arm hanging STRAIGHT, NO payload.\n"
               + ("Servos will slowly reel in. Continue?" if p["mode"] != "manual" else
                  "MANUAL mode: you will reel each tendon in from this prompt. Continue?"))
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

    def travel_ok(i):
        if abs(cur[i] - start[i]) > p["max_travel_counts"]:
            raise ProtocolError(f"tendon {i} not taut within {p['max_travel_counts']} counts "
                                f"-- check winding direction (reel_in_sign) / attachment")

    zeros: dict[int, int] = {}
    if p["mode"] == "manual":
        ctx.mark("calib_manual_start", start=start)
        while True:
            ans = ctx.prompt(
                "Reel in until each tendon is just taut (no slack, arm still straight).\n"
                "Enter counts: '40' = all three, '2:40' = tendon 2 only, '-20' / '2:-20' = let out.\n"
                "(1 count = 0.017 mm of line; 60 counts = 1 mm.)  Enter 'done' when all are taut.\n"
                "Now: " + ", ".join(f"T{i} {cur[i] - start[i]:+d}" for i in ctx.ids), "text", "done").strip()
            if ans.lower() in ("done", "d", "ok", ""):
                break
            try:
                ids, n = ((ctx.ids, int(ans)) if ":" not in ans else
                          ([int(ans.split(":")[0])], int(ans.split(":")[1])))
            except ValueError:
                ctx.log(f"didn't understand {ans!r}")
                continue
            ids = [i for i in ids if i in cur]
            for i in ids:
                cur[i] += sign[i] * n
                travel_ok(i)
            ctx.raw({i: cur[i] for i in ids}, p["speed"])
            ctx.settle(timeout=5, still_s=0.1)
        st = ctx.state()
        zeros = {i: int(round(st[i]["pos"])) for i in ctx.ids}
        ctx.mark("calib_manual", zeros=zeros, travel={i: zeros[i] - start[i] for i in ctx.ids})
    else:
        free: dict[int, list[float]] = {i: [] for i in ctx.ids}
        streak: dict[int, list[int]] = {i: [] for i in ctx.ids}       # positions of consecutive high steps
        k = 0
        while len(zeros) < len(ctx.ids):
            active = [i for i in ctx.ids if i not in zeros]
            for i in active:
                cur[i] += sign[i] * p["step_counts"]
                travel_ok(i)
            ctx.raw({i: cur[i] for i in active}, p["speed"])
            ctx.settle(timeout=3, still_s=0.1)
            m = ctx.sample(p["sample_s"])
            k += 1
            if k <= p["free_steps"]:                                  # still in slack: learn the baseline
                for i in active:
                    free[i].append(abs(m[i]["load"]))
                if k == p["free_steps"]:
                    base = {i: max(v) for i, v in free.items()}
                    ctx.mark("calib_baseline", baseline=base, start=start)
                    ctx.log(f"moving baseline |load| (through slack): {base}")
                continue
            for i in active:
                rise = abs(m[i]["load"]) - base[i]
                if rise >= p["load_threshold"]:
                    streak[i].append(int(round(m[i]["pos"])))
                else:
                    streak[i] = []
                if len(streak[i]) >= p["confirm_steps"]:
                    zeros[i] = streak[i][0]
                    ctx.mark("taut", id=i, pos=zeros[i], load=m[i]["load"], travel=zeros[i] - start[i])
                    ctx.log(f"tendon {i} taut at {zeros[i]} (travel {zeros[i] - start[i]:+d}, load {m[i]['load']})")
                    cur[i] = zeros[i]
                    ctx.raw({i: cur[i]}, p["speed"])                 # back off the confirm steps

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


# --------------------------------------------------------------------------- max bend
@protocol("max_bend", tendon=3, target_deg=90.0, step_counts=15, release_ratio=0.5, max_travel_counts=1990,
          load_limit=700, speed=150, sample_s=0.3, return_steps=True, require_imu=True)
def max_bend(ctx: TestContext, p: dict) -> dict:
    """Pull ONE tendon in steps while the other two pay out (release_ratio x the pull, from the
    1 : 1/2 : 1/2 constant-curvature split), measuring tip tilt with the IMU after every step.
    Stops at target_deg (or the load / travel limit), holds, then releases back to zero in the
    same steps so the pull and release curves (hysteresis) are both recorded.
    Needs a calibration (zeros) and, by default, the tip IMU."""
    ids = ctx.ids
    T = int(p["tendon"])
    if T not in ids:
        raise ProtocolError(f"tendon {T} not in {ids}")
    zero = dict(zip(ctx.geometry.get("ids", ids), map(int, ctx.geometry.get("zero_counts", []))))
    if not ctx.geometry.get("calibrated", True) or set(zero) != set(ids):
        raise ProtocolError("no calibration -- run Calibrate first")
    sign = dict(zip(ctx.geometry.get("ids", ids), ctx.geometry.get("reel_in_sign", [1] * len(ids))))
    r = float(ctx.geometry.get("spool_radius_mm", 11.0))
    mm_per_count = 2 * 3.141592653589793 * r / 4096
    others = [i for i in ids if i != T]

    ctx.prompt(f"MAX BEND on tendon {T}: it reels in, tendons {others} pay out ({p['release_ratio']} x).\n"
               f"Arm goes to the calibrated straight pose first; the IMU reference is set there.\n"
               f"Stops at {p['target_deg']} deg IMU tilt, |load| {p['load_limit']}, or "
               f"{p['max_travel_counts']} counts. Clear the bend path. Continue?")
    ctx.raw(zero, p["speed"])
    ctx.settle(timeout=8)
    ctx.hold(1.0)
    if ctx.imu() is None:
        if p["require_imu"]:
            raise ProtocolError("tip IMU offline/calibrating -- plug in the sensor board and wait for 'ok' "
                                "(or set require_imu: false to run on counts only)")
        ctx.log("IMU offline: running on counts only")
    elif not ctx.imu_set_reference():
        raise ProtocolError("could not set IMU reference (hold the arm still and retry)")
    ctx.hold(0.3)

    def targets(k: int) -> dict[int, int]:
        t = {T: zero[T] + sign[T] * k}
        for i in others:
            t[i] = zero[i] - sign[i] * int(round(p["release_ratio"] * k))
        return t

    pull, rel, rows = [], [], []

    def measure(k: int, phase: str) -> dict:
        m = ctx.sample(p["sample_s"])
        im = ctx.imu_sample(p["sample_s"]) if ctx.imu() else None
        act = int(round((m[T]["pos"] - zero[T]) * sign[T]))          # measured reel-in of the pulled tendon
        row = {"phase": phase, "cmd_counts": k, "counts": act, "mm": round(act * mm_per_count, 2),
               "tilt": im["tilt"] if im else None, "roll": im["roll"] if im else None,
               "pitch": im["pitch"] if im else None,
               "flex": ctx.flex(),
               "load": {i: m[i]["load"] for i in ids}, "pos": {i: m[i]["pos"] for i in ids}}
        ctx.mark("step", **row)
        rows.append(row)
        (pull if phase == "pull" else rel).append([act, row["tilt"] if row["tilt"] is not None else 0.0])
        ctx.plot(title=f"Tendon {T} pull vs IMU tilt", x_label="tendon %d reel-in (counts)" % T, y_label="tilt (deg)",
                 target=p["target_deg"], series={"pull": pull, "release": rel},
                 summary=(f"{phase}: T{T} {act:+d} counts ({row['mm']:.1f} mm) -> tilt "
                          f"{row['tilt'] if row['tilt'] is not None else 'n/a'} deg | |load| "
                          + " ".join(f"{i}:{abs(row['load'][i]):.0f}" for i in ids)
                          + (f" | flex {row['flex']['total']} deg" if row["flex"] and row["flex"]["total"] is not None
                             else "")))
        return row

    measure(0, "pull")
    k, stop = 0, None
    while stop is None:
        k += p["step_counts"]
        if k > p["max_travel_counts"]:
            stop, k = "travel limit", k - p["step_counts"]
            break
        ctx.raw(targets(k), p["speed"])
        ctx.settle(timeout=4, still_s=0.15)
        row = measure(k, "pull")
        if row["tilt"] is not None and row["tilt"] >= p["target_deg"]:
            stop = "target reached"
        elif max(abs(v) for v in row["load"].values()) >= p["load_limit"]:
            stop = "load limit"
        elif len(rows) >= 3 and rows[-1]["counts"] - rows[-3]["counts"] < p["step_counts"] * 0.5:
            stop = "pulled tendon stopped advancing (servo range limit or jam)"   # commanded 2 steps, got < 1/4
    top = rows[-1]
    k = max(0, top["counts"])            # release from where the tendon really is, not the clamped command
    per_deg = round(top["counts"] / top["tilt"], 2) if top["tilt"] else None
    ctx.mark("max_bend", reason=stop, counts=top["counts"], mm=top["mm"], tilt=top["tilt"],
             counts_per_deg=per_deg, load=top["load"], flex=top["flex"], release_counts={i: int(round(p["release_ratio"] * top["counts"]))
                                                                      for i in others})
    ctx.log(f"STOP ({stop}): tendon {T} {top['counts']} counts = {top['mm']} mm, tilt {top['tilt']} deg, "
            f"{per_deg} counts/deg; others paid out {int(round(p['release_ratio'] * top['counts']))} counts")
    ctx.prompt(f"Holding at max: tilt {top['tilt']} deg, tendon {T} +{top['counts']} counts ({top['mm']} mm).\n"
               "Check the arm (slack on the released tendons? buckling?). Continue to release.")

    if p["return_steps"]:
        while k > 0:
            k = max(0, k - p["step_counts"])
            ctx.raw(targets(k), p["speed"])
            ctx.settle(timeout=4, still_s=0.15)
            measure(k, "release")
    else:
        ctx.raw(zero, p["speed"])
        ctx.settle()
        measure(0, "release")

    end = rows[-1]
    return {"stop": stop, "tendon": T, "max_counts": top["counts"], "max_mm": top["mm"], "max_tilt_deg": top["tilt"],
            "counts_per_deg": per_deg, "loads_at_max": top["load"], "flex_at_max": top["flex"], "residual_tilt_deg": end["tilt"],
            "steps": len(rows)}


# --------------------------------------------------------------------------- final bend (project data)
@protocol("final_bend", targets=[30, 60, 75, 90], trials=3, tendon=3, release_ratio=0.5, step_counts=15,
          fine_step_counts=5, fine_within_deg=6.0, tol_deg=0.5, hold_s=5.0, imu_offset_deg=0.0, imu_scale=1.0,
          max_travel_counts=1990, load_limit=700, speed=150, sample_s=0.3, label="", require_imu=True)
def final_bend(ctx: TestContext, p: dict) -> dict:
    """Project bend-accuracy test. For every trial and every target angle: go home, zero the IMU there,
    pull ONE tendon (others pay out x release_ratio) until the corrected IMU tilt reaches the target
    (coarse steps, then fine steps within fine_within_deg), HOLD hold_s and average everything, then release
    back home in steps and measure the residual tilt. One row per bend goes to results.csv; every raw stream
    (telemetry / imu / flex / events) is logged as usual.
    Corrected tilt = imu_scale * raw_tilt + imu_offset_deg (both raw and corrected are recorded)."""
    ids = ctx.ids
    T = int(p["tendon"])
    targets = [float(t) for t in (p["targets"] if isinstance(p["targets"], list) else [p["targets"]])]
    if T not in ids or not targets:
        raise ProtocolError(f"bad tendon {T} or no targets")
    zero = dict(zip(ctx.geometry.get("ids", ids), map(int, ctx.geometry.get("zero_counts", []))))
    if set(zero) != set(ids):
        raise ProtocolError("no saved home -- Save home first")
    sign = dict(zip(ctx.geometry.get("ids", ids), ctx.geometry.get("reel_in_sign", [1] * len(ids))))
    mmpc = 2 * 3.141592653589793 * float(ctx.geometry.get("spool_radius_mm", 11.0)) / 4096
    others = [i for i in ids if i != T]
    sc, off = float(p["imu_scale"]), float(p["imu_offset_deg"])
    corr = lambda raw: None if raw is None else round(sc * raw + off, 2)  # noqa: E731
    n_bends = int(p["trials"]) * len(targets)

    ctx.prompt(f"FINAL BEND TEST{' [' + p['label'] + ']' if p['label'] else ''}\n"
               f"Targets {targets} deg x {int(p['trials'])} trials = {n_bends} bends on tendon {T}.\n"
               f"Each: home -> zero IMU -> pull to target -> hold {p['hold_s']} s -> release home.\n"
               f"IMU correction: corrected = {sc} x raw + {off} deg. Clear the bend path. Continue?")
    if ctx.imu() is None and p["require_imu"]:
        raise ProtocolError("tip IMU offline/calibrating -- wait for IMU 'ok'")

    def targets_at(k: int) -> dict[int, int]:
        t = {T: zero[T] + sign[T] * k}
        for i in others:
            t[i] = zero[i] - sign[i] * int(round(p["release_ratio"] * k))
        return t

    def snapshot(seconds: float) -> dict:
        m = ctx.sample(seconds)
        im = ctx.imu_sample(seconds) if ctx.imu() else None
        fx = ctx.flex()
        act = int(round((m[T]["pos"] - zero[T]) * sign[T]))
        return {"counts": act, "mm": round(act * mmpc, 2), "raw": im["tilt"] if im else None,
                "tilt": corr(im["tilt"]) if im else None, "roll": im["roll"] if im else None,
                "pitch": im["pitch"] if im else None, "load": {i: m[i]["load"] for i in ids},
                "pos": {i: m[i]["pos"] for i in ids}, "flex": fx}

    def go_home() -> None:
        ctx.raw(zero, p["speed"])
        ctx.settle(timeout=10)
        ctx.hold(1.0)

    results, curves = [], {}
    bend_no = 0
    for trial in range(1, int(p["trials"]) + 1):
        for tgt in targets:
            bend_no += 1
            key = f"{tgt:g} deg t{trial}"
            curves[key] = []
            go_home()
            if ctx.imu() is not None and not ctx.imu_set_reference():
                raise ProtocolError("could not zero the IMU at home (hold the arm still and retry)")
            ctx.hold(0.3)
            start = snapshot(p["sample_s"])
            ctx.mark("bend_start", trial=trial, target=tgt, **start)
            t0 = time.monotonic()
            k, stop, prev = 0, None, []
            cur = start
            while True:
                near = cur["tilt"] is not None and cur["tilt"] >= tgt - p["fine_within_deg"]
                k += p["fine_step_counts"] if near else p["step_counts"]
                if k > p["max_travel_counts"]:
                    stop, k = "travel limit", k - (p["fine_step_counts"] if near else p["step_counts"])
                    break
                ctx.raw(targets_at(k), p["speed"])
                ctx.settle(timeout=4, still_s=0.15)
                cur = snapshot(p["sample_s"])
                curves[key].append([cur["counts"], cur["tilt"] if cur["tilt"] is not None else 0.0])
                ctx.mark("step", trial=trial, target=tgt, phase="pull", cmd_counts=k, **cur)
                prev.append(cur["counts"])
                if cur["tilt"] is not None and cur["tilt"] >= tgt - p["tol_deg"]:
                    stop = "target reached"
                    break
                if max(abs(v) for v in cur["load"].values()) >= p["load_limit"]:
                    stop = "load limit"
                    break
                if len(prev) >= 3 and prev[-1] - prev[-3] < p["fine_step_counts"] * 0.5:
                    stop = "pulled tendon stopped advancing (range limit or jam)"
                    break
                ctx.plot(table=results, title=f"Final bend: tendon {T} pull vs IMU tilt (corrected)",
                         x_label=f"tendon {T} reel-in (counts)", y_label="tilt (deg)", target=tgt,
                         series={kk: v for kk, v in curves.items() if kk.endswith(f"t{trial}")},
                         summary=(f"bend {bend_no}/{n_bends} (trial {trial}, target {tgt:g} deg): "
                                  f"{cur['counts']:+d} counts ({cur['mm']:.1f} mm) -> tilt {cur['tilt']} deg"
                                  + (f" | flex {cur['flex']['total']} deg" if cur["flex"] else "")))
            t_reach = round(time.monotonic() - t0, 1)
            live = dict(table=results, title=f"Final bend: tendon {T} pull vs IMU tilt (corrected)",
                        x_label=f"tendon {T} reel-in (counts)", y_label="tilt (deg)", target=tgt,
                        series={kk: v for kk, v in curves.items() if kk.endswith(f"t{trial}")})
            ctx.plot(**live, hold={"until": time.time() + p["hold_s"], "s": p["hold_s"], "target": tgt,
                                   "tilt": cur["tilt"]},
                     summary=f"HOLDING bend {bend_no}/{n_bends}: target {tgt:g} deg at {cur['tilt']} deg ({stop})")
            held = snapshot(p["hold_s"])                     # average over the hold
            ctx.plot(**live, summary=f"bend {bend_no}/{n_bends}: held {held['tilt']} deg -> returning home")
            ctx.mark("hold", trial=trial, target=tgt, reason=stop, **held)
            # release home in steps (records the return path)
            k = max(0, held["counts"])
            while k > 0:
                k = max(0, k - p["step_counts"] * 2)
                ctx.raw(targets_at(k), p["speed"])
                ctx.settle(timeout=4, still_s=0.15)
                r = snapshot(p["sample_s"])
                ctx.mark("step", trial=trial, target=tgt, phase="release", cmd_counts=k, **r)
            go_home()
            back = snapshot(1.0)
            fx = held["flex"] or {}
            row = {"bend": bend_no, "trial": trial, "target_deg": tgt, "label": p["label"], "stop": stop,
                   "tilt_deg": held["tilt"], "error_deg": None if held["tilt"] is None else round(held["tilt"] - tgt, 2),
                   "tilt_raw_deg": held["raw"], "imu_scale": sc, "imu_offset_deg": off,
                   "roll_deg": held["roll"], "pitch_deg": held["pitch"],
                   "counts": held["counts"], "mm": held["mm"],
                   "counts_per_deg": round(held["counts"] / held["tilt"], 2) if held["tilt"] else None,
                   "time_to_target_s": t_reach,
                   "residual_after_return_deg": back["tilt"], "residual_raw_deg": back["raw"]}
            for i in ids:
                row[f"load_{i}"] = round(held["load"][i], 1)
            row["flex_total_deg"] = fx.get("total")
            for n, a, v in zip(fx.get("names", []), fx.get("angles", []), fx.get("volts", [])):
                tag = n.split(" (")[0].replace(" ", "_")
                row[f"{tag}_deg"], row[f"{tag}_v"] = a, v
            results.append(row)
            ctx.save_table("results.csv", results)
            ctx.plot(table=results, title=f"Final bend: tendon {T} pull vs IMU tilt (corrected)",
                     x_label=f"tendon {T} reel-in (counts)", y_label="tilt (deg)", target=tgt,
                     series={kk: v for kk, v in curves.items() if kk.endswith(f"t{trial}")},
                     summary=f"bend {bend_no}/{n_bends} done: target {tgt:g} -> {held['tilt']} deg ({stop})")
            ctx.log(f"bend {bend_no}/{n_bends}: target {tgt:g} -> {held['tilt']} deg ({stop}), "
                    f"{held['counts']} counts, residual {back['tilt']} deg")

    # per-target summary (mean / sd over trials)
    import statistics as st
    summary = []
    for tgt in targets:
        rs = [r for r in results if r["target_deg"] == tgt and r["tilt_deg"] is not None]
        if not rs:
            continue
        def ms(key):
            v = [r[key] for r in rs if r.get(key) is not None]
            return (round(st.fmean(v), 2), round(st.stdev(v), 2) if len(v) > 1 else 0.0) if v else (None, None)
        e, c, fl, rd = ms("error_deg"), ms("counts"), ms("flex_total_deg"), ms("residual_after_return_deg")
        summary.append({"target_deg": tgt, "n": len(rs), "error_mean_deg": e[0], "error_sd_deg": e[1],
                        "counts_mean": c[0], "counts_sd": c[1], "mm_mean": None if c[0] is None else round(c[0] * mmpc, 2),
                        "flex_total_mean_deg": fl[0], "residual_mean_deg": rd[0],
                        "reached": sum(1 for r in rs if r["stop"] == "target reached")})
    ctx.save_table("summary.csv", summary)
    return {"bends": len(results), "summary": summary}


# --------------------------------------------------------------------------- grasp trials (manipulator)
@protocol("grasp_trials", object="", object_size_mm="", object_mass_g=0.0, trials=5, hold_s=10.0, label="",
          zero_imu_each_trial=True)
def grasp_trials(ctx: TestContext, p: dict) -> dict:
    """Manipulator grasp trials. The arm and gripper are driven by the team's own control code; this protocol
    is the operator's checklist + timer + recorder. Per trial: Start -> grasp (operator clicks GRASPED or
    FAILED) -> timed hold of hold_s (operator clicks DROPPED / SLIPPED if it happens; otherwise success) ->
    release -> Done. Success = grasped AND held for hold_s without a drop. Raw telemetry / IMU / flex are
    logged for the whole session; one row per trial goes to results.csv, per-object tally to summary.csv."""
    obj = str(p["object"]).strip() or "unnamed object"
    n = int(p["trials"])
    hold = float(p["hold_s"])
    ids = ctx.ids
    results = []

    def snap(seconds: float = 0.4) -> dict:
        out = {"tilt": None, "flex": ctx.flex(), "load": {}, "volt": None}
        try:
            m = ctx.sample(seconds)
            out["load"] = {i: round(m[i]["load"], 1) for i in ids}
        except Exception:  # noqa: BLE001 -- servos may be unpowered/absent during EE-only work
            pass
        im = ctx.imu_sample(seconds) if ctx.imu() else None
        out["tilt"] = im["tilt"] if im else None
        st = ctx.state()
        v = [st[i].get("voltage") for i in ids if i in st and st[i].get("voltage")]
        out["volt"] = round(sum(v) / len(v), 1) if v else None
        return out

    def live(summary: str, hold_info: dict | None = None) -> None:
        kw = dict(table=results, title=f"Grasp trials: {obj}", summary=summary)
        if hold_info:
            kw["hold"] = hold_info
        ctx.plot(**kw)

    live(f"{obj}: {n} trials, hold {hold:g} s")
    for k in range(1, n + 1):
        ans = ctx.prompt(f"{obj} -- trial {k}/{n}\nPlace the object, bring the arm/gripper to the start pose.\n"
                         f"Press START when ready (SKIP to skip this trial).", kind="choice", choices=["start", "skip"])
        row = {"trial": k, "object": obj, "object_size_mm": p["object_size_mm"], "object_mass_g": p["object_mass_g"],
               "label": p["label"]}
        if ans == "skip":
            row.update(outcome="skipped", success=0)
            results.append(row)
            ctx.save_table("results.csv", results)
            continue
        if p["zero_imu_each_trial"] and ctx.imu() is not None:
            ctx.imu_set_reference()                     # tilt = arm motion/droop relative to the start pose
        t0 = time.monotonic()
        start = snap()
        ctx.mark("trial_start", trial=k, object=obj, **{kk: start[kk] for kk in ("tilt", "load", "volt")})
        live(f"trial {k}/{n}: grasping...")
        g = ctx.prompt(f"{obj} -- trial {k}/{n}: GRASP the object with the gripper control, lift it clear.\n"
                       "Press GRASPED once it is held, or FAILED if the grasp did not work.",
                       kind="choice", choices=["grasped", "failed"])
        t_grasp = round(time.monotonic() - t0, 1)
        if g != "grasped":
            row.update(outcome="failed to grasp", success=0, time_to_grasp_s=t_grasp)
            ctx.mark("trial_end", **row)
            results.append(row)
            ctx.save_table("results.csv", results)
            ctx.prompt("Reset the object and gripper, then press DONE.", kind="choice", choices=["done"])
            continue
        at_grasp = snap()
        ctx.mark("grasped", trial=k, t=t_grasp, tilt=at_grasp["tilt"], load=at_grasp["load"])
        # timed hold: the operator only clicks if something goes wrong
        t_h0, slip, dropped_at = time.monotonic(), False, None
        while True:
            left = hold - (time.monotonic() - t_h0)
            if left <= 0:
                break
            live(f"trial {k}/{n}: HOLDING", {"until": time.time() + left, "s": hold, "target": obj,
                                              "tilt": at_grasp["tilt"]})
            a = ctx.prompt(f"{obj} -- trial {k}/{n}: HOLDING {hold:g} s. Do nothing if it stays put.\n"
                           "Click DROPPED if it falls, SLIPPED if it moves in the gripper but stays held.",
                           kind="choice", choices=["dropped", "slipped"], timeout=left)
            if a == TIMEOUT:
                break
            if a == "dropped":
                dropped_at = round(time.monotonic() - t_h0, 1)
                break
            slip = True
            ctx.mark("slip", trial=k, t=round(time.monotonic() - t_h0, 1))
        held_s = dropped_at if dropped_at is not None else hold
        end = snap()
        ok = dropped_at is None
        fx = end["flex"] or {}
        row.update(outcome="success" if ok else f"dropped at {dropped_at:g} s", success=int(ok), slipped=int(slip),
                   held_s=held_s, time_to_grasp_s=t_grasp,
                   tilt_at_grasp_deg=at_grasp["tilt"], tilt_end_hold_deg=end["tilt"],
                   droop_during_hold_deg=(None if at_grasp["tilt"] is None or end["tilt"] is None
                                          else round(end["tilt"] - at_grasp["tilt"], 2)),
                   flex_total_deg=fx.get("total"), supply_v=end["volt"])
        for i in ids:
            row[f"load_{i}_at_grasp"] = at_grasp["load"].get(i)
            row[f"load_{i}_end_hold"] = end["load"].get(i)
        ctx.mark("trial_end", **row)
        results.append(row)
        ctx.save_table("results.csv", results)
        live(f"trial {k}/{n}: {row['outcome']}")
        ctx.prompt(f"{obj} -- trial {k}/{n}: {row['outcome'].upper()}.\nRelease/place the object, then press DONE.",
                   kind="choice", choices=["done"])

    done = [r for r in results if r["outcome"] != "skipped"]
    succ = sum(r["success"] for r in done)
    summary = [{"object": obj, "object_size_mm": p["object_size_mm"], "object_mass_g": p["object_mass_g"],
                "label": p["label"], "trials": len(done), "successes": succ,
                "success_rate": round(succ / len(done), 3) if done else None,
                "grasp_failures": sum(1 for r in done if r["outcome"] == "failed to grasp"),
                "drops": sum(1 for r in done if str(r["outcome"]).startswith("dropped")),
                "slips": sum(r.get("slipped", 0) for r in done),
                "mean_time_to_grasp_s": round(sum(r["time_to_grasp_s"] for r in done) / len(done), 1) if done else None}]
    ctx.save_table("summary.csv", summary)
    live(f"{obj}: {succ}/{len(done)} successful")
    return summary[0]


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
