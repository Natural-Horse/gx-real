#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

exec "${SCRIPT_DIR}/run_leg12_real.sh" \
  --device cpu \
  --pose_estimator none \
  --standup-mode internal \
  --base-command-source external_vla \
  --external-base-topic /vla/base_cmd \
  --external-base-watchdog-sec 0.25 \
  --arm-control-owner external_spacemouse \
  --require-arm-state-for-rl \
  --gripper-cmd 0.0 \
  --arm_pose 0.0 0.3 0.5 0.0 0.0 0.0 \
  --arm-reset-pose 0.0 0.3 0.5 0.0 0.0 0.0 \
  "$@"
