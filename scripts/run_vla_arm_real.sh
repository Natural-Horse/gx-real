#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

exec "${SCRIPT_DIR}/run_spacemouse_arm.sh" \
  --model X5 \
  --can-interface can0 \
  --control-source external_vla \
  --external-target-topic /vla/arm_target \
  --external-target-watchdog-sec 0.25 \
  --safety-topic /safety/estop \
  "$@"
