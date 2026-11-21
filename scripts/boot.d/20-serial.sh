# Serial: find the Waveshare board (CP210x), check permissions.
rcm_section "serial"

_rcm_find_port() {
  # Servo board = Waveshare (CP2102). Never return the sensor board (QT Py: ttyACM / Espressif / 1a86).
  local p
  for p in /dev/serial/by-id/*CP210*; do [[ -e "$p" ]] && { echo "$p"; return; }; done
  local sensors=""
  for p in /dev/serial/by-id/*; do
    case "$p" in *Adafruit*|*QT_Py*|*Espressif*|*1a86*|*CH9102*|*CH340*) sensors+=" $(readlink -f "$p")" ;; esac
  done
  for p in /dev/ttyUSB*; do
    [[ -e "$p" && " $sensors " != *" $(readlink -f "$p") "* ]] && { echo "$p"; return; }
  done
}

# A preset RCM_PORT is kept only if it still exists and isn't the sensor board (ttyACM / QT Py)
_rcm_is_sensor_port() {
  local rp; rp="$(readlink -f "$1")"
  [[ "$1" == *Adafruit* || "$1" == *QT_Py* || "$1" == *Espressif* || "$rp" == /dev/ttyACM* ]] && return 0
  local p
  for p in /dev/serial/by-id/*; do
    case "$p" in *Adafruit*|*QT_Py*|*Espressif*|*1a86*|*CH9102*|*CH340*)
      [[ "$(readlink -f "$p")" == "$rp" ]] && return 0 ;;
    esac
  done
  return 1
}

if [[ -n "${RCM_PORT:-}" && -e "$RCM_PORT" ]] && ! _rcm_is_sensor_port "$RCM_PORT"; then
  rcm_ok "port (preset) ${RCM_PORT}"
else
  RCM_PORT="$(_rcm_find_port)"
  if [[ -n "$RCM_PORT" ]]; then
    export RCM_PORT
    rcm_ok "board at ${RCM_PORT}"
  else
    unset RCM_PORT
    rcm_info "servo board not plugged in (motor controls disabled until it is)"
  fi
fi

# `id -nG` = groups of THIS login session; `id -nG <user>` = what /etc/group says
if id -nG | grep -qw dialout; then
  rcm_ok "user in dialout"
elif id -nG "$(id -un)" | grep -qw dialout; then
  rcm_fail "added to dialout but this session predates it — log out/in (or: newgrp dialout)"
else
  rcm_fail "user not in dialout:  sudo usermod -aG dialout \$USER  (then re-login)"
fi

if dpkg -s brltty &>/dev/null; then
  rcm_warn "brltty installed — can steal CP210x ports:  sudo apt remove brltty"
fi

rcm_cmd_port() {
  RCM_PORT="$(_rcm_find_port)"
  if [[ -n "$RCM_PORT" ]]; then export RCM_PORT; echo "$RCM_PORT"
  else unset RCM_PORT; echo "no board found"; return 1; fi
}
rcm_register port "re-detect board serial port"
