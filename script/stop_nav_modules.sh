#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PID_FILE="${WS_ROOT}/log/modular_nav/latest.pids"

if [[ ! -f "${PID_FILE}" ]]; then
  echo "[WARN] PID file not found: ${PID_FILE}"
  exit 0
fi

echo "[INFO] Stopping modular navigation processes..."

# 倒序停止，先停上层模块再停驱动
mapfile -t lines < "${PID_FILE}"
for (( idx=${#lines[@]}-1; idx>=0; idx-- )); do
  line="${lines[idx]}"
  pid="$(awk '{print $1}' <<<"${line}")"
  name="$(awk '{print $2}' <<<"${line}")"

  if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
    echo "[INFO] SIGINT ${name} (pid=${pid})"
    kill -INT "${pid}" 2>/dev/null || true
  fi
done

sleep 2

for (( idx=${#lines[@]}-1; idx>=0; idx-- )); do
  line="${lines[idx]}"
  pid="$(awk '{print $1}' <<<"${line}")"
  name="$(awk '{print $2}' <<<"${line}")"

  if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
    echo "[INFO] SIGKILL ${name} (pid=${pid})"
    kill -KILL "${pid}" 2>/dev/null || true
  fi
done

rm -f "${PID_FILE}"
echo "[INFO] Stopped."
