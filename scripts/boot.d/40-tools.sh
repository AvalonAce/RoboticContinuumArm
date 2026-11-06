# Host-side Python tools (tools/rcm_serial.py library + test scripts).
rcm_section "tools"

# Make `from rcm_serial import RCM` work from anywhere
[[ ":${PYTHONPATH:-}:" != *":${RCM_TOOLS}:"* ]] && export PYTHONPATH="${RCM_TOOLS}${PYTHONPATH:+:$PYTHONPATH}"

if python3 -c "import serial" &>/dev/null; then
  rcm_ok "pyserial $(python3 -c 'import serial; print(serial.__version__)')"
else
  rcm_fail "pyserial missing:  sudo apt install python3-serial"
fi

# Free the port: the console and Python can't both hold it
_rcm_port_busy() {
  [[ -n "${RCM_PORT:-}" ]] && command -v fuser &>/dev/null && fuser "$(readlink -f "$RCM_PORT")" &>/dev/null
}

rcm_cmd_motortest() {
  if _rcm_port_busy; then echo "port busy — close the serial console first"; return 1; fi
  python3 "${RCM_TOOLS}/motor_test.py" "$@"
}
rcm_register motortest "bus/servo self-test, tendons detached (--ids 1 2 3 --amp 30)"

rcm_cmd_rest() {
  if _rcm_port_busy; then echo "port busy — use the dashboard's Go to rest, or stop the stack first"; return 1; fi
  python3 "${RCM_TOOLS}/rest.py" "$@"
}
rcm_register rest "move servos to winding rest pose and hold (no ROS needed)"

rcm_cmd_py() {
  if _rcm_port_busy; then echo "port busy — close the serial console first"; return 1; fi
  python3 -i -c "from rcm_serial import *; arm = RCM(verbose=True); print('arm ready: ids', arm.ids, '| arm.close() when done')"
}
rcm_register py "interactive Python with arm = RCM() connected"
