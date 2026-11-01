#!/usr/bin/env bash
# One-click start of the lab station: pre-flight checks, then the operator
# screen (GUI + robot server threads, src/app/station.py).
#
# Every lab session lost minutes to the same three faults -- the camera fell
# back to USB 2.0 (pypylon refuses it), eth0 had no 192.168.125.2 (the server
# cannot bind), or an old station still held the camera and port 5000. This
# script finds each one before the GUI starts and says what to do; a hard
# fault waits for Enter and checks again, so the cable can be fixed without
# closing the window.
#
#   tools/start_station.sh                   checks, then the station (fullscreen)
#   tools/start_station.sh --check-only      checks only
#   tools/start_station.sh --install-desktop put a double-click launcher on ~/Desktop
#   tools/start_station.sh -- --no-verify    anything after -- goes to station.py
#
# The robot connects as a client, so it may still be off: a failed ping is
# only a warning. Start this first, then run the RAPID program.
set -u

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON="$PROJECT_DIR/.venv/bin/python"
LOG_DIR="$PROJECT_DIR/logs"
ROBOT_ADDRESS="192.168.125.1"
BASLER_USB_VENDOR="2676"
USB3_MIN_SPEED_MBPS=5000
PING_TIMEOUT_SEC=1
OLD_PROCESS_PATTERN="src\.app\.(station|main)"
OLD_PROCESS_STOP_WAIT_SEC=2
DESKTOP_FILE_NAME="INA700_Station.desktop"
DEFAULT_STATION_ARGS=(--camera basler --fullscreen)

ok()   { printf '  \033[32m[ OK ]\033[0m %s\n' "$*"; }
warn() { printf '  \033[33m[WARN]\033[0m %s\n' "$*"; }
fail() { printf '  \033[31m[FAIL]\033[0m %s\n' "$*"; }

# Without a terminal behind stdin, read fails at once; quitting then keeps the
# check-again loop from spinning forever.
wait_for_enter() {
    read -r -p "  $1" _ || { echo; exit 1; }
}

settings_value() {
    (cd "$PROJECT_DIR" && "$PYTHON" -c \
        "from src.core.config import Config; print(Config.load().get('$1'))")
}

check_python() {
    if [ -x "$PYTHON" ]; then
        ok "Python venv: $PYTHON"
        return 0
    fi
    fail "venv missing: $PYTHON"
    return 1
}

# Sysfs gives the negotiated link speed; lsusb alone does not.
basler_usb_speed_mbps() {
    local device
    for device in /sys/bus/usb/devices/*; do
        if [ "$(cat "$device/idVendor" 2>/dev/null)" = "$BASLER_USB_VENDOR" ]; then
            cat "$device/speed" 2>/dev/null
            return 0
        fi
    done
    return 1
}

check_camera() {
    local speed
    if ! speed="$(basler_usb_speed_mbps)"; then
        fail "Basler camera not on USB -- plug it into a blue USB 3.0 port"
        return 1
    fi
    if [ "${speed%%.*}" -lt "$USB3_MIN_SPEED_MBPS" ]; then
        fail "camera is on USB 2.0 (${speed} Mbit/s) -- re-seat the cable at BOTH ends, blue port"
        return 1
    fi
    ok "camera on USB 3.0 (${speed} Mbit/s)"
}

check_server_address() {
    local host
    host="$(settings_value comm.host)" || { fail "cannot read comm.host from settings.yaml"; return 1; }
    if ip -4 -o addr show | grep -qF "inet ${host}/"; then
        ok "server address $host present ($(ip -4 -o addr show | awk -v h="$host" '$4 ~ "^"h"/" {print $2}'))"
        return 0
    fi
    fail "no interface has $host -- is the Ethernet cable plugged in? (ip -4 -br addr)"
    return 1
}

stop_old_station() {
    if ! pgrep -f "$OLD_PROCESS_PATTERN" >/dev/null; then
        ok "no old station running"
        return 0
    fi
    warn "an old station is running (holds camera + port) -- stopping it"
    pkill -f "$OLD_PROCESS_PATTERN"
    sleep "$OLD_PROCESS_STOP_WAIT_SEC"
    pkill -9 -f "$OLD_PROCESS_PATTERN" 2>/dev/null
    ok "old station stopped"
}

check_robot() {
    if ping -c 1 -W "$PING_TIMEOUT_SEC" "$ROBOT_ADDRESS" >/dev/null 2>&1; then
        ok "robot answers at $ROBOT_ADDRESS"
    else
        warn "robot $ROBOT_ADDRESS does not answer yet -- fine if the controller is still off"
    fi
}

run_checks() {
    echo "== INA700 station: pre-flight checks =="
    check_python || return 1
    stop_old_station
    local failed=0
    check_camera || failed=1
    check_server_address || failed=1
    check_robot
    return "$failed"
}

checks_until_pass() {
    until run_checks; do
        echo
        wait_for_enter "Fix the [FAIL] lines, then press Enter to check again (Ctrl+C quits) "
        echo
    done
}

run_station() {
    local log_file
    mkdir -p "$LOG_DIR"
    log_file="$LOG_DIR/station_$(date +%Y-%m-%d_%H%M%S).log"
    echo
    echo "== Starting the station (Esc leaves fullscreen) -- log: $log_file =="
    echo "   Now start the RAPID program on the robot."
    cd "$PROJECT_DIR" || return 1
    PYTHONUNBUFFERED=1 "$PYTHON" -m src.app.station "$@" 2>&1 | tee "$log_file"
    return "${PIPESTATUS[0]}"
}

install_desktop_launcher() {
    local desktop_dir="$HOME/Desktop"
    local launcher="$desktop_dir/$DESKTOP_FILE_NAME"
    mkdir -p "$desktop_dir"
    chmod +x "$PROJECT_DIR/tools/start_station.sh"
    cat > "$launcher" <<EOF
[Desktop Entry]
Type=Application
Name=INA700 Station
Comment=Pre-flight checks, then the operator screen and robot server
Exec=$PROJECT_DIR/tools/start_station.sh
Path=$PROJECT_DIR
Terminal=true
Icon=applications-engineering
Categories=Utility;
EOF
    chmod +x "$launcher"
    echo "launcher written: $launcher"
}

main() {
    case "${1:-}" in
        --install-desktop) install_desktop_launcher; return 0 ;;
        --check-only) run_checks; return $? ;;
        --) shift ;;
        "") ;;
        *) echo "unknown option: $1" >&2; return 2 ;;
    esac
    checks_until_pass
    local station_args=("${DEFAULT_STATION_ARGS[@]}" "$@")
    run_station "${station_args[@]}"
    local status=$?
    echo
    if [ "$status" -eq 0 ]; then
        echo "== Station closed normally =="
    else
        echo "== Station exited with code $status -- the lines above show why =="
    fi
    wait_for_enter "Press Enter to close this window "
}

main "$@"
