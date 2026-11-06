# Serial: find the Waveshare board (CP210x), check permissions.
rcm_section "serial"

_rcm_find_port() {
  local p
  # Prefer the stable by-id path (survives replug / enumeration order)
  for p in /dev/serial/by-id/*CP210*; do [[ -e "$p" ]] && { echo "$p"; return; }; done
  for p in /dev/ttyUSB* /dev/ttyACM*; do [[ -e "$p" ]] && { echo "$p"; return; }; done
}

if [[ -n "${RCM_PORT:-}" && -e "$RCM_PORT" ]]; then
  rcm_ok "port (preset) ${RCM_PORT}"
else
  RCM_PORT="$(_rcm_find_port)"
  if [[ -n "$RCM_PORT" ]]; then
    export RCM_PORT
    rcm_ok "board at ${RCM_PORT}"
  else
    unset RCM_PORT
    rcm_warn "no board found — plug in USB-C (data cable, not charge-only)"
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
