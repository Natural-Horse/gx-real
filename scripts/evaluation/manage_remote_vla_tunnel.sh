#!/usr/bin/env bash
set -euo pipefail

ACTION="${1:-}"
SERVER="${2:-}"
ROBOT="${3:-}"
REMOTE_PORT="${REMOTE_PORT:-10093}"
LOCAL_RELAY_PORT="${LOCAL_RELAY_PORT:-10094}"
ROBOT_PORT="${ROBOT_PORT:-10093}"
SERVER_SOCKET="${SERVER_SOCKET:-/tmp/gx-vla-server-${UID}-${LOCAL_RELAY_PORT}.sock}"
ROBOT_SOCKET="${ROBOT_SOCKET:-/tmp/gx-vla-robot-${UID}-${ROBOT_PORT}.sock}"

usage() {
  echo "用法: $0 <start|check|stop> <server-ssh-host> <robot-ssh-host>" >&2
  echo "该脚本在同时能 SSH 到推理服务器和机器狗的工作站运行。" >&2
  exit 2
}

[[ "${ACTION}" =~ ^(start|check|stop)$ ]] || usage
[[ -n "${SERVER}" && -n "${ROBOT}" ]] || usage

check_tunnel() {
  local socket="$1"
  local host="$2"
  ssh -S "${socket}" -O check "${host}" >/dev/null
}

case "${ACTION}" in
  start)
    if ! check_tunnel "${SERVER_SOCKET}" "${SERVER}" 2>/dev/null; then
      ssh -M -S "${SERVER_SOCKET}" -fNT \
        -o ExitOnForwardFailure=yes \
        -o ServerAliveInterval=30 \
        -o ServerAliveCountMax=3 \
        -L "127.0.0.1:${LOCAL_RELAY_PORT}:127.0.0.1:${REMOTE_PORT}" \
        "${SERVER}"
    fi
    if ! check_tunnel "${ROBOT_SOCKET}" "${ROBOT}" 2>/dev/null; then
      ssh -M -S "${ROBOT_SOCKET}" -fNT \
        -o ExitOnForwardFailure=yes \
        -o ServerAliveInterval=30 \
        -o ServerAliveCountMax=3 \
        -R "127.0.0.1:${ROBOT_PORT}:127.0.0.1:${LOCAL_RELAY_PORT}" \
        "${ROBOT}"
    fi
    echo "隧道已启动: ${ROBOT}:127.0.0.1:${ROBOT_PORT} -> ${SERVER}:127.0.0.1:${REMOTE_PORT}"
    ;;
  check)
    check_tunnel "${SERVER_SOCKET}" "${SERVER}"
    check_tunnel "${ROBOT_SOCKET}" "${ROBOT}"
    echo "服务器与机器狗两段 SSH 隧道均正常。"
    ;;
  stop)
    ssh -S "${ROBOT_SOCKET}" -O exit "${ROBOT}" 2>/dev/null || true
    ssh -S "${SERVER_SOCKET}" -O exit "${SERVER}" 2>/dev/null || true
    ;;
esac
