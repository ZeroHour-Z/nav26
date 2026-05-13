#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

mkdir -p "${HOME}/.vnc"
ln -sfn "${WS_ROOT}/script/vnc_xstartup.sh" "${HOME}/.vnc/xstartup"
chmod +x "${WS_ROOT}/script/vnc_xstartup.sh"

if [ ! -f "${HOME}/.vnc/passwd" ]; then
    if [ -n "${VNC_PASSWORD:-}" ]; then
        printf '%s\n%s\nn\n' "${VNC_PASSWORD}" "${VNC_PASSWORD}" | vncpasswd "${HOME}/.vnc/passwd" >/dev/null
        chmod 600 "${HOME}/.vnc/passwd"
    else
        echo "VNC password is not set."
        echo "Run: vncpasswd ~/.vnc/passwd"
        echo "Or set VNC_PASSWORD before starting this script."
        exit 1
    fi
fi

vncserver -kill :1 >/dev/null 2>&1 || true
vncserver :1 -localhost yes -geometry 1600x1000 -depth 24

VNC_TUNNEL_HOST="${VNC_TUNNEL_HOST:-<host-or-vm-ip>}"

cat <<MSG

VNC desktop is running on display :1 / port 5901.

If VNC is running on a remote VM or server, open an SSH tunnel:
  ssh -L 5901:localhost:5901 zerohour@${VNC_TUNNEL_HOST}

Then open macOS Screen Sharing, Remmina, or another VNC client:
  vnc://localhost:5901

Run ROS launch commands through:
  cd ~/xjtu_nav26
  ./script/run_in_vnc.sh ros2 launch nav2_client_cpp nav2_stack_with_gvc_sim.launch.py

MSG
