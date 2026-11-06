# ROS 2 application: launch the stack, open the dashboard.
rcm_section "app"

if [[ -n "${ROS_DISTRO:-}" ]] && ros2 pkg prefix rcm_bringup &>/dev/null; then
  rcm_ok "rcm packages built"
else
  rcm_info "rcm packages not built/sourced (rcm ws_build)"
fi

rcm_cmd_launch() {
  [[ -n "${ROS_DISTRO:-}" ]] || { echo "ROS not sourced"; return 1; }
  ros2 launch rcm_bringup rcm.launch.py "$@"
}
rcm_register launch "start full stack on hardware (args: port:=... dashboard:=false ...)"

rcm_cmd_sim() { rcm_cmd_launch sim:=true "$@"; }
rcm_register sim "start full stack with a SIMULATED board (no hardware)"

rcm_cmd_dash() { xdg-open "http://localhost:${1:-8080}" &>/dev/null & disown; }
rcm_register dash "open the dashboard in a browser"
