# ROS 2: source the distro, then the workspace overlay if it has been built.
rcm_section "ros2"

_rcm_ros_distro=""
if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  _rcm_ros_distro="$ROS_DISTRO"
else
  for d in jazzy humble; do
    [[ -f "/opt/ros/${d}/setup.bash" ]] && { _rcm_ros_distro="$d"; break; }
  done
fi

if [[ -n "$_rcm_ros_distro" ]]; then
  # shellcheck source=/dev/null
  source "/opt/ros/${_rcm_ros_distro}/setup.bash"
  rcm_ok "ROS 2 ${_rcm_ros_distro} sourced"
  # Keep our traffic off other teams' graphs on shared lab networks
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
  rcm_info "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
else
  rcm_warn "no ROS 2 install found in /opt/ros (fine for firmware-only work)"
fi

if [[ -f "${RCM_WS}/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${RCM_WS}/install/setup.bash"
  rcm_ok "workspace overlay sourced"
else
  rcm_info "workspace not built yet (rcm ws_build)"
fi

rcm_cmd_ws_build() {
  [[ -n "${ROS_DISTRO:-}" ]] || { echo "ROS not sourced"; return 1; }
  ( cd "$RCM_WS" && colcon build --symlink-install "$@" ) \
    && source "${RCM_WS}/install/setup.bash"
}
rcm_register ws_build "colcon build ros2_ws and re-source overlay"
unset d
