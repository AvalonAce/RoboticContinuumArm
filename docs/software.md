# RCM software (ROS 2 Jazzy)

```
 browser ── HTTP/SSE ── dashboard ─┐
                                   │ /rcm/cmd_bend        /rcm/goal          serial (USB)
 test_runner ──────────────────────┼──────────────► kinematics ──────► hw_bridge ◄────► ESP32 ── ST bus ── STS3215 ×3
   (protocols, logging)            │                  ▲  CC model          │ safety: load/temp/soft limits, e-stop
                                   └─────── /rcm/state, /rcm/bend_estimate ◄┘
```

| Package | Node | Role |
|---|---|---|
| `rcm_msgs` | – | `ArmState`, `ServoState`, `TendonGoal`, `BendCommand`, `BendState`; srvs `SetTorque`, `SetZero`, `StartTest`, `Respond` |
| `rcm_hw` | `hw_bridge` | owns the serial port; telemetry → `/rcm/state`; `/rcm/goal` → one SyncWrite. Also `rcm_serial.py` (driver) and `sim.py` (fake board) |
| `rcm_kinematics` | `kinematics` | `cc.py` (pure Python, unit-tested): bend (θ, φ) ↔ tendon Δl ↔ counts; publishes `/rcm/bend_estimate`; stores calibration |
| `rcm_experiments` | `test_runner` | protocols (`protocols.py`), YAML configs (`protocols/`), session logging |
| `rcm_dashboard` | `dashboard` | stdlib HTTP + Server-Sent Events, no extra installs; page in `web/` |
| `rcm_bringup` | – | `rcm.launch.py`, `config/arm.yaml` |

## Run
```bash
source boot.sh
rcm ws_build          # after pulling or editing msgs/setup.py (python edits are live: --symlink-install)
rcm sim               # whole stack, simulated board — develop/test without hardware
rcm launch            # real hardware (close any serial console first)
rcm dash              # http://localhost:8080
```
Edits to `config/arm.yaml` and `protocols/*.yaml` in the source tree take effect on the next launch / test start (no rebuild).

## Safety semantics
| Action | Effect |
|---|---|
| **E-STOP** (button / Esc / `ros2 service call /rcm/estop std_srvs/srv/Trigger`) | torque off immediately — **a lifted payload drops**. Goals blocked until **Rearm**. |
| Load limit (`load_limit`) / temperature limit | same as e-stop, automatic. Reason shown in the dashboard. |
| **Abort** (test panel) | protocol stops; **arm holds its current pose** (torque stays on). |
| Soft limits (`soft_min/max`) | every goal clamped per servo. Tighten once winch travel is known. |
| Node shutdown (Ctrl+C) | torque off. |
| Rearm | clears the trip; servos stay limp until the next command (which moves at commanded speed). |

## Tendon setup (do this before winding any spool)
1. **Go to rest**: dashboard Setup → *Go to rest*, or without ROS: `rcm rest`. Servos move slowly to the
   rest pose and **hold torque**, so the spools can't turn while you work.
2. Mount spools/horns without rotating the servos. Wind each tendon with **slight slack**, tie off.
3. **Calibrate** (dashboard Setup → *Calibrate*). It warns if the servos aren't near rest, and after
   storing zeros checks that every tendon has enough travel each way for `theta_max_deg`.

Why: position mode is single-turn (4096 counts). A 90° bend needs ±`theta·d/r` of tendon travel
(≈ ±1536 counts at d = 15 mm, r = 10 mm). Rest = center (2048) shifted toward pay-out by
`pretension_allowance_mm`, so after calibration reels in the slack the zero sits near center.
Override per servo with `rest_counts` in `arm.yaml` (-1 = auto).

## Calibration
`calibrate` protocol: all tendons reel in together in small steps; each stops when its |load| rises `load_threshold` above idle. That pose = zero (straight, equal tension). Saved to `config/calibration.yaml` (commit it — it describes the shared arm). Software zeros, not servo EEPROM: reversible, and the encoder is absolute within one turn, so zeros survive power cycles **as long as no winch passes a full revolution while unpowered**.

## Session data (`data/<YYYYmmdd_HHMMSS>_<protocol>/`)
- `telemetry.csv` — every `/rcm/state` sample (~50 Hz): per servo `pos/goal/speed/load/temp/volt/current/moving/err`, bend estimate (`est_theta_deg`, `est_phi_deg`, `tip_xyz`, `common_mm`), command, trip reason
- `events.csv` — `host_t, elapsed_s, event, data(JSON)`: `payload`, `hold_start/hold_end`, `cycle_start/top/bottom`, `taut`, `failure`, ... Segment telemetry by `host_t`.
- `meta.json` — protocol, resolved params, notes, geometry + calibration, outcome, duration

## Protocols
| File | Purpose | Key events |
|---|---|---|
| `calibrate` | pretension + store zeros | `taut`, `calibrated` |
| `static_load` | payload steps at fixed configs | `payload`, `hold_start`, `hold_end` (means) |
| `lift_cycles` | N lifts at sub-limit payload | `cycle_start`, `top`, `bottom` (settle time, error) |
| `hold` | 3-min sustained hold | `hold_progress` |
| `speed_sweep` | lifts at several speeds | `top`/`bottom` tagged with speed |
| `limit` | payload until failure | `failure` (mode) |

New protocol: write `fn(ctx, p)` with `@protocol("name", **defaults)` in `protocols.py`, add `protocols/<name>.yaml`. The `ctx` API: `bend()`, `raw()`, `settle()`, `sample()`, `hold()`, `prompt()`, `mark()`, `log()`, `state()`.

## Must measure before real tests (`config/arm.yaml`, marked PLACEHOLDER)
- `length_mm`, `tendon_radius_mm`, `spool_radius_mm`
- `reel_in_sign` per servo: jog a servo **+** in the dashboard (raw jog) and see whether its tendon shortens (+1) or lengthens (−1)
- `soft_min/max` from actual winch travel; `load_limit` once you've seen real tension levels

## Known limits
- The bend estimate comes from tendon lengths (encoders) — it does not see tendon stretch or payload sag. The tip IMU (not yet wired) is what measures true deflection.
- 50 Hz telemetry ceiling (115200-baud USB). Raise firmware USB baud if the dynamics tests need more.
- Single-turn position mode: a bend whose tendon travel exceeds one spool revolution is rejected by kinematics.
