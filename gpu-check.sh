#!/usr/bin/env bash
# 排查 Gazebo / RViz2 报 "failed to create drawable" 的辅助脚本。
# 在宿主机执行：./gpu-check.sh
set -uo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

CONTAINER_NAME="${CONTAINER_NAME:-slam-ros2-dev}"
DISPLAY="${DISPLAY:-:1}"

ok()   { printf '  [ OK ]   %s\n' "$1"; }
warn() { printf '  [WARN]   %s\n' "$1"; }
bad()  { printf '  [FAIL]   %s\n' "$1"; }

echo "== Docker =="
if docker info >/dev/null 2>&1; then
    ok "docker daemon reachable"
    runtimes="$(docker info --format '{{range $k, $v := .Runtimes}}{{$k}} {{end}}' 2>/dev/null || true)"
    echo "  runtimes: ${runtimes:-unknown}"
    if grep -qw nvidia <<<"$runtimes" || [[ -e /var/run/cdi/nvidia.yaml ]]; then
        ok "NVIDIA GPU access available -> compose.nvidia.yaml can be used"
        [[ -e /var/run/cdi/nvidia.yaml ]] && ok "CDI spec: /var/run/cdi/nvidia.yaml"
    else
        warn "neither nvidia runtime nor CDI spec found"
        warn "keep LIBGL_ALWAYS_SOFTWARE=1 and use software rendering"
    fi
else
    bad "cannot talk to the docker daemon"
fi

echo
echo "== Host GPU =="
command -v nvidia-smi >/dev/null 2>&1 \
    && nvidia-smi --query-gpu=name,driver_version --format=csv,noheader \
    || warn "nvidia-smi not found on the host"
[[ -d /dev/dri ]] && ok "/dev/dri exists: $(ls /dev/dri | tr '\n' ' ')" \
    || warn "/dev/dri missing (no DRM device, hardware GL unavailable)"

echo
echo "== X11 =="
display_number="${DISPLAY#:}"
socket="/tmp/.X11-unix/X${display_number%%.*}"
[[ -S "$socket" ]] && ok "socket ${socket}" || bad "socket ${socket} not found"
for candidate in "${XAUTHORITY:-}" "/run/user/$(id -u)/gdm/Xauthority" "$HOME/.Xauthority"; do
    [[ -n "$candidate" && -r "$candidate" ]] && { ok "Xauthority readable: ${candidate}"; break; }
done

echo
echo "== Inside container (${CONTAINER_NAME}) =="
if ! docker compose ps --status running --services 2>/dev/null | grep -qx "$CONTAINER_NAME"; then
    bad "container not running - start it with ./start.sh first"
    exit 1
fi

docker compose exec -T "$CONTAINER_NAME" bash -lc '
set -uo pipefail
if command -v nvidia-smi >/dev/null 2>&1; then
    echo "GPU: $(nvidia-smi --query-gpu=name --format=csv,noheader)"
else
    echo "GPU: nvidia-smi not injected"
fi
if compgen -G "/dev/dri/*" >/dev/null; then
    echo "DRM nodes: $(ls -1 /dev/dri | tr "\n" " ")"
else
    echo "DRM nodes: /dev/dri missing"
fi
echo "LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE:-unset}"
echo "QT_QPA_PLATFORM=${QT_QPA_PLATFORM:-unset}"
if ldconfig -p 2>/dev/null | grep -q libGLX_nvidia; then
    echo "GLX vendor: nvidia"
elif command -v glxinfo >/dev/null 2>&1; then
    glxinfo -B 2>&1 | grep -E "OpenGL renderer|OpenGL version" || true
else
    echo "GLX vendor: nvidia libs absent (add mesa-utils for a full glxinfo probe)"
fi
'
