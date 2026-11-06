#!/usr/bin/env bash
# RCM boot script — SOURCE it, don't execute it:
#
#     source boot.sh          (or:  . boot.sh)
#
# Loads every module in scripts/boot.d/ in filename order, then prints a status
# summary. Each module can export env vars, run checks, and register `rcm`
# subcommands. To extend: drop a new NN-name.sh into scripts/boot.d/.
#
# Usage after sourcing:  rcm help

# ---- guard: must be sourced so env vars / functions land in your shell -------
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "boot.sh must be sourced:  source ${0}" >&2
  exit 1
fi

RCM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export RCM_ROOT

# ---- tiny framework used by modules ------------------------------------------
_RCM_OK=0; _RCM_WARN=0; _RCM_FAIL=0
declare -gA _RCM_HELP=()          # subcommand -> description
declare -ga _RCM_ORDER=()         # subcommand registration order

rcm_ok()   { printf '  \e[32m✓\e[0m %s\n' "$*"; _RCM_OK=$((_RCM_OK+1));   }
rcm_warn() { printf '  \e[33m!\e[0m %s\n' "$*"; _RCM_WARN=$((_RCM_WARN+1)); }
rcm_fail() { printf '  \e[31m✗\e[0m %s\n' "$*"; _RCM_FAIL=$((_RCM_FAIL+1)); }
rcm_info() { printf '    %s\n' "$*"; }
rcm_section() { printf '\e[1m[%s]\e[0m\n' "$*"; }

# rcm_register <name> <description> — exposes function rcm_cmd_<name> as `rcm <name>`
rcm_register() {
  [[ -z "${_RCM_HELP[$1]+x}" ]] && _RCM_ORDER+=("$1")
  _RCM_HELP[$1]="$2"
}

# `rcm <cmd> [args]` dispatcher
rcm() {
  local cmd="${1:-help}"; shift || true
  if declare -F "rcm_cmd_${cmd}" >/dev/null; then
    "rcm_cmd_${cmd}" "$@"
  else
    echo "rcm: unknown command '${cmd}'  (rcm help)" >&2
    return 1
  fi
}

rcm_cmd_help() {
  echo "rcm <command> [args]"
  local c
  for c in "${_RCM_ORDER[@]}"; do printf '  %-14s %s\n' "$c" "${_RCM_HELP[$c]}"; done
}
rcm_register help "list commands"

rcm_cmd_status() { _rcm_boot; }
rcm_register status "re-run all boot checks"

rcm_cmd_reload() { source "${RCM_ROOT}/boot.sh"; }
rcm_register reload "re-source boot.sh (after editing modules)"

# ---- load modules ------------------------------------------------------------
_rcm_boot() {
  _RCM_OK=0; _RCM_WARN=0; _RCM_FAIL=0
  local m
  for m in "${RCM_ROOT}"/scripts/boot.d/*.sh; do
    [[ -r "$m" ]] || continue
    # shellcheck source=/dev/null
    source "$m"
  done
  echo
  printf 'RCM ready: %d ok, %d warn, %d fail.  Type: rcm help\n' \
    "$_RCM_OK" "$_RCM_WARN" "$_RCM_FAIL"
}

_rcm_boot
