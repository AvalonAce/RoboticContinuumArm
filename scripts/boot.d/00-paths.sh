# Paths used by every other module.
rcm_section "paths"

export RCM_FW="${RCM_ROOT}/firmware"
export RCM_WS="${RCM_ROOT}/ros2_ws"
export RCM_TOOLS="${RCM_ROOT}/tools"
export RCM_DATA="${RCM_DATA:-${RCM_ROOT}/data}"   # gitignored; override to log elsewhere

mkdir -p "$RCM_DATA"
rcm_ok "RCM_ROOT=${RCM_ROOT}"
rcm_info "data -> ${RCM_DATA}"

rcm_cmd_cd() {
  local d
  case "${1:-root}" in
    root) d="$RCM_ROOT" ;;  fw) d="$RCM_FW" ;;  ws) d="$RCM_WS" ;;
    tools) d="$RCM_TOOLS" ;; data) d="$RCM_DATA" ;;
    *) echo "rcm cd [root|fw|ws|tools|data]"; return 1 ;;
  esac
  cd "$d" || return
}
rcm_register cd "jump to root|fw|ws|tools|data"
