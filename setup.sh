#!/bin/bash
# Set up the EPS Dashboard service on a stock Neoracer.
#
# On the car:
#   git clone https://github.com/Neobotics-Foundation-Inc/eps_dashboard.git
#   bash eps_dashboard/setup.sh
#
# The service runs the files where this repository already sits. setup.sh
# writes that path into the unit and copies nothing, so the checkout can live
# anywhere. Idempotent: safe to re-run.
#
# A first install leaves neoracer-eps.service installed, stopped, and
# disabled; start it with `bash setup.sh enable`. A re-run keeps whatever state
# the service is in, restarting it only when it is already running.
#
# Subcommands:
#   (none)    install or update the unit; a first install does not start it
#   enable    start now and at every boot
#   disable   stop now and keep off across boots
#   restart   restart the service, taking port 8084 back first
#   remove    stop, disable, and uninstall the unit; files here are kept
#
# Needs nothing beyond the stock image: ROS Humble at /opt/ros/humble, the
# driver workspace at /home/racecar/ros2_ws, and the racecar user.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SVC=neoracer-eps.service
UNIT_IN="$SCRIPT_DIR/$SVC.in"
UNIT="/etc/systemd/system/$SVC"
PORT=8084

# The unit ships as a template; @DIR@ becomes this checkout, so the service
# always runs the copy it was installed from.
render_unit() {
    sed "s|@DIR@|$SCRIPT_DIR|g" "$UNIT_IN"
}

# PIDs listening on $PORT. Unprivileged ss names only our own processes, so
# fall back to sudo when a listener exists but its owner stays hidden.
port_pids() {
    local out
    out="$(ss -ltnpH "sport = :$PORT" 2>/dev/null || true)"
    if [[ -n "$out" && "$out" != *pid=* ]]; then
        out="$(sudo ss -ltnpH "sport = :$PORT" 2>/dev/null || true)"
    fi
    grep -o 'pid=[0-9]*' <<<"$out" | cut -d= -f2 | sort -u || true
}

# Service unit a PID belongs to, empty for a process started by hand.
unit_of() {
    sed -n 's|.*/\([^/]*\.service\).*|\1|p' "/proc/$1/cgroup" 2>/dev/null | head -1
}

# sudo only for processes we do not own.
signal_pid() {
    if [[ "$(stat -c %u "/proc/$1" 2>/dev/null)" == "$(id -u)" ]]; then
        kill "$2" "$1" 2>/dev/null || true
    else
        sudo kill "$2" "$1" 2>/dev/null || true
    fi
}

# PIDs on $PORT that are not this service.
foreign_pids() {
    local pid
    for pid in $(port_pids); do
        [[ "$(unit_of "$pid")" == "$SVC" ]] || echo "$pid"
    done
}

# Clear anything else off $PORT: an earlier install of this dashboard under a
# different unit name or directory, a sibling dashboard, or a hand-started
# eps.py. Killing a service-owned PID would only trip its own
# Restart=on-failure, so stop the owning unit instead.
free_port() {
    local pid unit
    for pid in $(foreign_pids); do
        unit="$(unit_of "$pid")"
        if [[ -n "$unit" ]]; then
            echo "  port $PORT held by $unit: stopping it"
            sudo systemctl stop "$unit" || true
        else
            echo "  port $PORT held by pid $pid started outside systemd: stopping it"
            signal_pid "$pid" -TERM
        fi
    done
    for _ in $(seq 20); do
        [[ -z "$(foreign_pids)" ]] && return 0
        sleep 0.5
    done
    for pid in $(foreign_pids); do
        [[ -n "$(unit_of "$pid")" ]] || signal_pid "$pid" -KILL
    done
    sleep 1
    [[ -z "$(foreign_pids)" ]] || echo "  warning: port $PORT is still busy"
}

# Start and confirm it stayed up; a bind failure or a traceback shows here
# rather than as a service that quietly loops on Restart=on-failure.
start_and_check() {
    sudo systemctl "$1" "$SVC"
    sleep 3
    systemctl is-active --quiet "$SVC" || {
        echo "$SVC failed to start:"; systemctl status "$SVC" --no-pager | tail -5; exit 1; }
    echo "EPS Dashboard running: http://$(hostname -I | awk '{print $1}'):$PORT"
}

case "${1:-}" in
    disable)
        sudo systemctl disable --now "$SVC"
        echo "$SVC stopped and disabled. It will not start at boot."
        echo "Re-enable with: bash setup.sh enable"
        exit 0
        ;;
    enable)
        free_port
        sudo systemctl enable "$SVC" 2>/dev/null
        start_and_check start
        exit 0
        ;;
    restart)
        sudo systemctl stop "$SVC" 2>/dev/null || true
        free_port
        start_and_check restart
        exit 0
        ;;
    remove)
        sudo systemctl disable --now "$SVC" 2>/dev/null || true
        sudo rm -f "$UNIT"
        sudo systemctl daemon-reload
        sudo systemctl reset-failed "$SVC" 2>/dev/null || true
        echo "$SVC removed. Files in $SCRIPT_DIR are untouched."
        echo "Reinstall with: bash setup.sh"
        exit 0
        ;;
    "")
        ;;
    *)
        echo "usage: bash setup.sh [enable|disable|restart|remove]" >&2
        exit 2
        ;;
esac

rendered="$(mktemp)"
trap 'rm -f "$rendered"' EXIT
render_unit >"$rendered"

# Whether this is a first install decides the service state below, so read it
# before the unit lands.
first_install=1
[[ -f "$UNIT" ]] && first_install=0

if ! cmp -s "$rendered" "$UNIT"; then
    sudo install -m 0644 "$rendered" "$UNIT"
    sudo systemctl daemon-reload
    echo "  $SVC: installed"
fi

# A first install stays off until someone asks for it. A re-run is an update,
# so it leaves the enable state alone and only restarts a service that is
# already running, picking up the new code.
if (( first_install )); then
    echo "$SVC installed, stopped and disabled."
    echo "Start it with: bash setup.sh enable"
elif systemctl is-active --quiet "$SVC"; then
    sudo systemctl stop "$SVC"
    free_port
    start_and_check start
else
    echo "$SVC updated. It is not running; start it with: bash setup.sh enable"
fi
