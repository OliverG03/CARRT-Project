#!/usr/bin/env bash

# Source this file in each ADL terminal, then run:
#   adl_log_new_run
#   adl_log_run T1_arm_moveit_rviz ros2 launch adl_tasks arm_start.launch.py ...
# Other terminals can call adl_log_run directly; they will reuse the current run id.

adl_log_root() {
  local ws="${ADL_WS:-$HOME/workspace/ros2_kortex_ws}"
  printf '%s\n' "${ADL_LOG_ROOT:-$ws/src/error-logs}"
}

adl_log_run_file() {
  printf '%s\n' "${ADL_LOG_RUN_FILE:-/tmp/adl_tasks_current_log_run}"
}

adl_log_new_run() {
  local run_id="${1:-$(date +%Y%m%d_%H%M%S)}"
  local root
  root="$(adl_log_root)"
  mkdir -p "$root/$run_id"
  printf '%s\n' "$run_id" > "$(adl_log_run_file)"
  export ADL_LOG_RUN_ID="$run_id"
  printf 'ADL log run: %s\n' "$root/$run_id"
}

adl_log_current_run() {
  if [[ -n "${ADL_LOG_RUN_ID:-}" ]]; then
    printf '%s\n' "$ADL_LOG_RUN_ID"
    return 0
  fi

  local run_file
  run_file="$(adl_log_run_file)"
  if [[ -s "$run_file" ]]; then
    ADL_LOG_RUN_ID="$(tr -d '[:space:]' < "$run_file")"
  else
    ADL_LOG_RUN_ID="$(date +%Y%m%d_%H%M%S)"
    printf '%s\n' "$ADL_LOG_RUN_ID" > "$run_file"
  fi
  export ADL_LOG_RUN_ID
  printf '%s\n' "$ADL_LOG_RUN_ID"
}

adl_log_run() {
  if [[ $# -lt 2 ]]; then
    printf 'usage: adl_log_run LABEL COMMAND [ARG ...]\n' >&2
    return 2
  fi

  local label="$1"
  shift
  local root run_id log_dir log_path
  root="$(adl_log_root)"
  run_id="$(adl_log_current_run)"
  log_dir="$root/$run_id"
  mkdir -p "$log_dir"
  log_path="$log_dir/${label}.log"

  {
    printf '\n===== %s | %s =====\n' "$(date --iso-8601=seconds)" "$label"
    printf 'cwd=%s\n' "$PWD"
    printf 'cmd='
    printf '%q ' "$@"
    printf '\n\n'
  } | tee -a "$log_path"

  "$@" 2>&1 | tee -a "$log_path"
  local cmd_status=${PIPESTATUS[0]}
  return "$cmd_status"
}
