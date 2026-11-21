# Sensor board: Adafruit QT Py ESP32 + ICM-20948 (+ flex later). Separate USB port from the servo board.
rcm_section "sensors"
export RCM_SENSOR_FW="${RCM_ROOT}/firmware_sensors"

_rcm_find_sensor_port() {
  local p
  for p in /dev/serial/by-id/*; do
    [[ -e "$p" && "$p" != *CP210* ]] || continue
    case "$p" in *Adafruit*|*QT_Py*|*Espressif*|*1a86*|*CH9102*|*CH340*) echo "$p"; return ;; esac
  done
  local servo; servo="$(readlink -f "${RCM_PORT:-/nonexistent}" 2>/dev/null)"
  for p in /dev/ttyACM* /dev/ttyUSB*; do
    [[ -e "$p" && "$(readlink -f "$p")" != "$servo" ]] && { echo "$p"; return; }
  done
}

if [[ -n "${RCM_SENSOR_PORT:-}" && -e "$RCM_SENSOR_PORT" ]]; then
  rcm_ok "sensor board (preset) ${RCM_SENSOR_PORT}"
else
  RCM_SENSOR_PORT="$(_rcm_find_sensor_port)"
  if [[ -n "$RCM_SENSOR_PORT" ]]; then export RCM_SENSOR_PORT; rcm_ok "sensor board at ${RCM_SENSOR_PORT}"
  else unset RCM_SENSOR_PORT; rcm_info "no sensor board plugged in"; fi
fi

# Which PlatformIO env? From the USB name where possible; S3 flash size can't be detected, so ask.
_rcm_sensor_env() {
  case "${RCM_SENSOR_PORT:-}" in
    *S2*) echo qtpy_s2 ;;
    *1a86*|*CH9102*|*CH340*) echo qtpy_pico ;;
    *) echo "" ;;
  esac
}

rcm_cmd_sflash() {
  local env="${1:-$(_rcm_sensor_env)}"
  if [[ -z "$env" ]]; then
    echo "which board? rcm sflash qtpy_s3 | qtpy_s3_psram | qtpy_s2 | qtpy_pico"
    echo "  (QT Py ESP32-S3: 'qtpy_s3' = 8 MB no-PSRAM version, 'qtpy_s3_psram' = 4 MB flash / 2 MB PSRAM)"
    return 1
  fi
  local -a pa=(); [[ -n "${RCM_SENSOR_PORT:-}" ]] && pa=(--upload-port "$RCM_SENSOR_PORT")
  pio run -d "$RCM_SENSOR_FW" -e "$env" -t upload "${pa[@]}" && sleep 2 && rcm_cmd_sport >/dev/null
}
rcm_register sflash "flash sensor board (arg: qtpy_s3|qtpy_s3_psram|qtpy_s2|qtpy_pico)"

rcm_cmd_sport() {
  RCM_SENSOR_PORT="$(_rcm_find_sensor_port)"
  if [[ -n "$RCM_SENSOR_PORT" ]]; then export RCM_SENSOR_PORT; echo "$RCM_SENSOR_PORT"
  else unset RCM_SENSOR_PORT; echo "no sensor board found"; return 1; fi
}
rcm_register sport "re-detect sensor board port"

rcm_cmd_smon() {
  [[ -n "${RCM_SENSOR_PORT:-}" ]] || rcm_cmd_sport >/dev/null || { echo "no sensor board"; return 1; }
  pio device monitor -d "$RCM_SENSOR_FW" -p "$RCM_SENSOR_PORT" -b 921600 "$@"
}
rcm_register smon "raw serial monitor on the sensor board"

rcm_cmd_flex() { python3 "${RCM_TOOLS}/flex_check.py" "$@"; }
rcm_register flex "live flex sensor voltages (A2, A3, TX) + wiring diagnosis"

rcm_cmd_imu() { python3 "${RCM_TOOLS}/imu_check.py" "$@"; }
rcm_register imu "live IMU readout + at-rest sanity checks (--raw, --log FILE)"

rcm_cmd_sensors() {
  [[ -n "${ROS_DISTRO:-}" ]] || { echo "ROS not sourced"; return 1; }
  ros2 launch rcm_bringup rcm.launch.py arm:=false "$@"
}
rcm_register sensors "ROS stack WITHOUT the servo board: IMU + dashboard (then: rcm dash)"
