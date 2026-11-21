# Sensor board (QT Py ESP32 + ICM-20948)

Separate from the servo board: its own USB port, firmware in `firmware_sensors/`.

## Bring-up
```bash
source boot.sh            # shows "sensor board at /dev/..." when plugged in
rcm sflash qtpy_s3        # or qtpy_s3_psram | qtpy_s2 | qtpy_pico — match the label on the board
rcm smon                  # raw serial: expect "imu ICM-20948 at 0x69 on STEMMA QT (Wire1)" then I,... lines
rcm imu                   # live readout + at-rest checks (keep it still for the first 3 s)
```
First flash of an S2/S3 that runs other firmware (CircuitPython, factory demo): if upload can't
connect, hold **BOOT**, tap **RESET**, release BOOT, then `rcm sflash ...` again; press RESET after.

## Stream format (100 Hz)
`I,t_ms,ax,ay,az,gx,gy,gz,mx,my,mz,temp*HH` — accel m/s², gyro rad/s, mag µT, °C, IMU axes,
`*HH` = XOR checksum. Commands: `scan`, `info`, `rate <hz>`, `stream on|off`.

## Troubleshooting
| Symptom | Cause |
|---|---|
| `ERR: no ICM-20948 found` + scan shows nothing | cable not seated / wrong port; STEMMA QT is `Wire1` |
| scan shows 0x68 or 0x69 but init fails | power-cycle (unplug USB) |
| `rcm imu`: cannot open port | `rcm smon` or the ROS sensor node is holding it |
| |accel| ≠ 9.8 at rest | board moving, or accel range misconfigured |

## ROS: `sensor_bridge` (package `rcm_sensors`)
Runs with the stack: `rcm launch` works with **whatever is plugged in** — servo board, sensor board, both or
neither. Each bridge shows "not connected"/"offline" and connects when its board is plugged in.

| Topic / service | |
|---|---|
| `/rcm/imu/state` (`rcm_msgs/ImuState`, ~100 Hz) | state, orientation, relative-to-reference, roll/pitch, **tilt_from_ref**, accel, bias-corrected gyro |
| `/rcm/imu/data` (`sensor_msgs/Imu`) | filtered orientation (yaw covariance huge: no magnetometer) |
| `/rcm/imu/data_raw` (`sensor_msgs/Imu`) | raw accel/gyro |
| `/rcm/imu/set_reference`, `clear_reference`, `calibrate_gyro` (`std_srvs/Trigger`) | dashboard buttons |

Processing (`rcm_sensors/imu_filter.py`, pure Python, unit-tested):
1. **Gyro bias**: at startup (and on *Recalibrate*), waits for ~2 s of stillness, averages the gyro.
2. **6-axis Madgwick** (accel + gyro, `beta` in `arm.yaml`). No magnetometer: yaw drifts slowly;
   roll/pitch/tilt do not.
3. **Reference**: *Set reference* stores the current orientation (e.g. arm straight).
   `tilt_from_ref` = angle between gravity directions now vs reference — yaw-free. For a bend in a
   vertical plane it equals the tip rotation; how it maps to the arm model is left to the model.

Logged per test session: `imu.csv` (every sample) and `imu_*` columns in `telemetry.csv`.
