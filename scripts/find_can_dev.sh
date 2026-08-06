#!/usr/bin/env bash
set -euo pipefail

# 输出本机可用的 USB-CAN by-id 设备路径（优先 Openlight CANable2，其次 ARX）。
# 用法：
#   CAN_DEV=$(bash scripts/find_can_dev.sh)

for pattern in "usb-Openlight_Labs_CANable2*" "usb-ARX*"; do
  hit=$(ls /dev/serial/by-id/${pattern} 2>/dev/null | head -1 || true)
  if [[ -n "${hit}" ]]; then
    echo "${hit}"
    exit 0
  fi
done

for dev in /dev/ttyACM*; do
  if [[ -e "${dev}" ]]; then
    echo "${dev}"
    exit 0
  fi
done

echo "ERROR: no USB-CAN device found under /dev/serial/by-id or /dev/ttyACM*" >&2
exit 1
