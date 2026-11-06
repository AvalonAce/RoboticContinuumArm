# Firmware: PlatformIO build / flash / console.
rcm_section "firmware"

# pio installed via the official installer lives here, not on PATH
[[ -d "$HOME/.platformio/penv/bin" && ":$PATH:" != *":$HOME/.platformio/penv/bin:"* ]] \
  && export PATH="$PATH:$HOME/.platformio/penv/bin"

if command -v pio &>/dev/null; then
  rcm_ok "platformio $(pio --version 2>/dev/null | awk '{print $NF}')"
else
  rcm_fail "platformio missing:  pip install platformio"
fi

if [[ ! -f /etc/udev/rules.d/99-platformio-udev.rules ]]; then
  rcm_warn "PlatformIO udev rules missing (optional):"
  rcm_info "curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules && sudo udevadm control --reload-rules"
fi

_rcm_port_args() { [[ -n "${RCM_PORT:-}" ]] && printf -- '--port\n%s\n' "$RCM_PORT"; }

rcm_cmd_build() { pio run -d "$RCM_FW" "$@"; }
rcm_register build "compile firmware"

rcm_cmd_flash() {
  local -a pa; mapfile -t pa < <(_rcm_port_args | sed 's/^--port$/--upload-port/')
  pio run -d "$RCM_FW" -t upload "${pa[@]}" "$@"
}
rcm_register flash "compile + upload firmware"

rcm_cmd_console() {
  local -a pa; mapfile -t pa < <(_rcm_port_args)
  pio device monitor -d "$RCM_FW" "${pa[@]}" "$@"
}
rcm_register console "serial console (Ctrl+C to quit)"

# Console with the whole session also written to data/logs/device-monitor-<ts>.log
# (use with `stream all 50` to capture CSV telemetry)
rcm_cmd_log() {
  local -a pa; mapfile -t pa < <(_rcm_port_args)
  ( cd "$RCM_DATA" || exit 1
    pio device monitor -d "$RCM_FW" "${pa[@]}" -f send_on_enter -f log2file "$@" )
}
rcm_register log "console + save session to data/"

rcm_cmd_up() { rcm_cmd_flash && rcm_cmd_console; }
rcm_register up "flash, then open console"
