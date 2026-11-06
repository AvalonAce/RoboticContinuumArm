# Robotic Continuum Arm

Tendon-driven continuum manipulator (ME555 Fall 2026).

## Layout
```
firmware/   ESP32 (Waveshare Servo Driver) — PlatformIO
ros2_ws/    ROS 2 packages (not started)
tools/      standalone Python scripts
docs/       bring-up, wiring, protocol
```

## Hardware (current)
- Waveshare Servo Driver with ESP32 (USB-C, CP210x)
- Feetech STS3215 serial bus servos (ST bus, 1 Mbps)
- 7.4–12 V supply to the driver's DC jack (servos are NOT powered from USB)

## Quick start (firmware)
```bash
pip install platformio
cd firmware
pio run -t upload        # flash
pio device monitor       # console, type `help`
```
See `docs/motor_bringup.md`.

## Servo ID map
| ID | Role |
|----|------|
| 1  | Tendon A (0°) |
| 2  | Tendon B (120°) |
| 3  | Tendon C (240°) |
| 4  | Wrist roll (later) |
| 5  | Gripper (later) |

## Boot script
```bash
source boot.sh      # every new terminal; or add to ~/.bashrc:
                    #   source ~/development/RoboticContinuumArm/boot.sh
rcm help            # build | flash | console | log | up | port | cd | ws_build | status | reload
```
Modules live in `scripts/boot.d/` and load in filename order (`00-paths`, `10-ros`, `20-serial`, `30-firmware`).
To add one, create `scripts/boot.d/NN-name.sh`:
```bash
rcm_section "name"
# checks: rcm_ok / rcm_warn / rcm_fail / rcm_info "msg"
rcm_cmd_foo() { echo "does foo $*"; }
rcm_register foo "one-line description"   # -> `rcm foo`
```
Then `rcm reload`. Modules are sourced, so never `exit` or `set -e` in them; use `return`.
