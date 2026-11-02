#!/usr/bin/env bash
#
# Prepares a Raspberry Pi for the INA700 station.
#
# Run this while the Pi has internet: in the lab the Ethernet port is on the
# robot subnet, which has no route to the internet.
#
# The image must be 64-bit (aarch64). The same SD card then boots on a Pi 3 and
# a Pi 4, because the installed wheels depend only on the architecture.
#
# Usage:
#   bash tools/setup_pi.sh --check          only report what is already present
#   bash tools/setup_pi.sh --all            python deps + network
#   bash tools/setup_pi.sh --python         python dependencies only
#   bash tools/setup_pi.sh --network        static IP on the robot subnet only
#
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="${PROJECT_DIR}/.venv"
WHEEL_DIR="${PROJECT_DIR}/wheels"
REQUIREMENTS_FILE="${PROJECT_DIR}/requirements-pi.txt"

# The Pi is the server on .2, the ABB controller is the client on .1.
ETHERNET_DEVICE="eth0"
PI_STATIC_IP="192.168.125.2/24"
ROBOT_IP="192.168.125.1"
SERVER_PORT="5000"
CONNECTION_NAME="robot-net"

log()  { echo ""; echo "=== $* ==="; }
info() { echo "    $*"; }

# --------------------------------------------------------------------------
check_environment() {
    log "Environment"
    info "model:        $(tr -d '\0' < /proc/device-tree/model 2>/dev/null || echo unknown)"
    info "architecture: $(uname -m)   <-- must be aarch64 for the pypylon wheels"
    info "os:           $(. /etc/os-release && echo "${PRETTY_NAME}")"
    info "python:       $(python3 --version 2>&1)"

    log "Python packages"
    # Prefer the project venv if it exists, so the report matches what runs.
    local python_binary="python3"
    [ -x "${VENV_DIR}/bin/python" ] && python_binary="${VENV_DIR}/bin/python"
    "${python_binary}" - <<'PYEOF' || true
import importlib
for module_name in ("cv2", "cv2.aruco", "numpy", "yaml", "pypylon", "onnxruntime"):
    try:
        module = importlib.import_module(module_name)
        print("    %-12s %s" % (module_name, getattr(module, "__version__", "installed")))
    except ImportError:
        print("    %-12s MISSING" % module_name)
PYEOF

    log "Network"
    ip -4 addr show "${ETHERNET_DEVICE}" 2>/dev/null | sed 's/^/    /' || info "${ETHERNET_DEVICE} not found"
    info "route to the robot: $(ip route get "${ROBOT_IP}" 2>/dev/null | head -1 || echo none)"
}

# --------------------------------------------------------------------------
install_python_dependencies() {
    log "Python dependencies"

    # Bookworm marks the system Python as externally managed, so a virtual
    # environment is the only clean way to pip install on it.
    if [ ! -d "${VENV_DIR}" ]; then
        info "creating a virtual environment with access to the system packages (Tk)"
        python3 -m venv --system-site-packages "${VENV_DIR}"
    fi

    if [ -d "${WHEEL_DIR}" ] && compgen -G "${WHEEL_DIR}/*.whl" > /dev/null; then
        info "installing from the offline wheels in ${WHEEL_DIR}"
        "${VENV_DIR}/bin/pip" install --no-index --find-links "${WHEEL_DIR}" -r "${REQUIREMENTS_FILE}"
    else
        info "no offline wheels found, downloading from PyPI"
        "${VENV_DIR}/bin/pip" install --upgrade pip
        "${VENV_DIR}/bin/pip" install -r "${REQUIREMENTS_FILE}"
    fi

    info "verifying the imports"
    "${VENV_DIR}/bin/python" -c "import cv2.aruco, numpy, yaml, onnxruntime, pypylon.pylon; print('    all imports OK')"
}

# --------------------------------------------------------------------------
configure_network() {
    log "Static IP on the robot subnet"

    # No gateway and no DNS on this connection: the robot subnet has no route to
    # the internet, and claiming a default route here would take over from Wi-Fi.
    if nmcli connection show "${CONNECTION_NAME}" > /dev/null 2>&1; then
        info "updating the existing '${CONNECTION_NAME}' connection"
        sudo nmcli connection modify "${CONNECTION_NAME}" \
            ipv4.addresses "${PI_STATIC_IP}" ipv4.method manual \
            ipv4.gateway "" ipv4.never-default yes
    else
        info "creating the '${CONNECTION_NAME}' connection on ${ETHERNET_DEVICE}"
        sudo nmcli connection add type ethernet ifname "${ETHERNET_DEVICE}" \
            con-name "${CONNECTION_NAME}" \
            ipv4.addresses "${PI_STATIC_IP}" ipv4.method manual \
            ipv4.never-default yes
    fi
    sudo nmcli connection up "${CONNECTION_NAME}"

    info "current address:"
    ip -4 addr show "${ETHERNET_DEVICE}" | sed 's/^/    /'
    info "testing the robot (fails until the controller is powered on):"
    ping -c 2 -W 2 "${ROBOT_IP}" || info "no answer from ${ROBOT_IP} yet; expected until the Pi is on the robot network"

    if command -v ufw > /dev/null && sudo ufw status | grep -q "Status: active"; then
        info "opening port ${SERVER_PORT} in ufw"
        sudo ufw allow "${SERVER_PORT}/tcp"
    else
        info "ufw is not active, no firewall rule needed"
    fi
}

# --------------------------------------------------------------------------
main() {
    local action="${1:---check}"
    case "${action}" in
        --check)   check_environment ;;
        --python)  install_python_dependencies ;;
        --network) configure_network ;;
        --all)
            check_environment
            install_python_dependencies
            configure_network
            log "Done. Re-run with --check to confirm."
            ;;
        *)
            echo "usage: bash tools/setup_pi.sh [--check|--all|--python|--network]"
            exit 1
            ;;
    esac
}

main "$@"
