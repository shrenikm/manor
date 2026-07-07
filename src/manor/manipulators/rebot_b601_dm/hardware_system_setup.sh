#!/usr/bin/env bash
#
# One-shot hardware system setup for manor robot serial devices on a Linux host (tested on Ubuntu 24.04).
# Run once per host with the robot's USB bridge plugged in; safe to re-run (idempotent).
#
# What it does:
#   1. Detects the robot's USB serial bridge (CDC ACM / USB-serial) and writes a udev rule that
#      - pins a stable device name (default /dev/rebot_b601_dm) via SYMLINK, matched on the bridge's
#        USB vendor/product IDs plus serial number when available, so the name survives replugs,
#        reboots, and enumeration-order changes (no more ttyACM0 vs ttyACM1 roulette),
#      - sets GROUP=dialout MODE=0660 on the node,
#      - tags the port with ID_MM_DEVICE_IGNORE so ModemManager never probes it with AT commands
#        (port-scoped; no need to disable ModemManager globally).
#      Rules for all manor robots share /etc/udev/rules.d/99-manor-robots.rules, one marker-delimited
#      block per device name, so other robots can be added by re-running with --name / --device.
#   2. Adds the current user to the dialout group if missing (serial port permissions).
#   3. Reloads udev rules, triggers them, and verifies the symlink appears.
#
# Usage:
#   ./hardware_system_setup.sh                       # auto-detect the bridge, name it rebot_b601_dm
#   ./hardware_system_setup.sh --device /dev/ttyACM1 # skip detection
#   ./hardware_system_setup.sh --name my_leader_arm --device /dev/ttyUSB0   # another robot / device
#
# Needs sudo for the privileged steps (udev rule install, usermod); run as your normal user, NOT via
# "sudo ./hardware_system_setup.sh" (the script must know who the real user is for the dialout add).

set -euo pipefail

DEFAULT_NAME="rebot_b601_dm"
RULES_FILE="/etc/udev/rules.d/99-manor-robots.rules"

name="${DEFAULT_NAME}"
device=""

log() { echo "[setup] $*"; }
die() {
    echo "[setup] ERROR: $*" >&2
    exit 1
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --name)
            name="$2"
            shift 2
            ;;
        --device)
            device="$2"
            shift 2
            ;;
        -h | --help)
            sed -n '2,25p' "$0" | sed 's/^# \{0,1\}//'
            exit 0
            ;;
        *)
            die "unknown argument: $1 (see --help)"
            ;;
    esac
done

[[ ${EUID} -eq 0 ]] && die "run as your normal user, not root/sudo; the script escalates only where needed"

# ---------------------------------------------------------------------------
# 1. Find the device.
# ---------------------------------------------------------------------------
shopt -s nullglob
if [[ -z "${device}" ]]; then
    candidates=(/dev/ttyACM* /dev/ttyUSB*)
    if [[ ${#candidates[@]} -eq 0 ]]; then
        die "no /dev/ttyACM* or /dev/ttyUSB* devices found -- plug the robot's USB bridge in and re-run"
    elif [[ ${#candidates[@]} -eq 1 ]]; then
        device="${candidates[0]}"
        log "found one serial device: ${device}"
    else
        log "multiple serial devices found:"
        for i in "${!candidates[@]}"; do
            desc=$(udevadm info -q property --name="${candidates[$i]}" 2>/dev/null |
                sed -n 's/^ID_MODEL=//p' | head -1)
            echo "  [$i] ${candidates[$i]}  (${desc:-unknown model})"
        done
        read -rp "[setup] index of the robot's bridge: " idx
        device="${candidates[$idx]:-}"
        [[ -n "${device}" ]] || die "invalid index"
    fi
fi
[[ -e "${device}" ]] || die "device ${device} does not exist"

# ---------------------------------------------------------------------------
# 2. Read its USB identity for a stable udev match.
# ---------------------------------------------------------------------------
props=$(udevadm info -q property --name="${device}")
vid=$(sed -n 's/^ID_VENDOR_ID=//p' <<<"${props}")
pid=$(sed -n 's/^ID_MODEL_ID=//p' <<<"${props}")
serial=$(sed -n 's/^ID_SERIAL_SHORT=//p' <<<"${props}")
model=$(sed -n 's/^ID_MODEL=//p' <<<"${props}")
[[ -n "${vid}" && -n "${pid}" ]] || die "could not read USB vendor/product IDs from ${device}"

log "device: ${device} (${model:-unknown model})"
log "  idVendor=${vid} idProduct=${pid} serial=${serial:-<none>}"

rule="SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"${vid}\", ATTRS{idProduct}==\"${pid}\""
if [[ -n "${serial}" ]]; then
    rule+=", ATTRS{serial}==\"${serial}\""
else
    log "WARNING: no USB serial number -- the rule matches any ${vid}:${pid} device. If you later plug"
    log "         in a second identical bridge (e.g. a leader arm), both will race for this name."
fi
rule+=", SYMLINK+=\"${name}\", GROUP=\"dialout\", MODE=\"0660\", ENV{ID_MM_DEVICE_IGNORE}=\"1\""

# ---------------------------------------------------------------------------
# 3. Install the rule (replace this name's block if it already exists).
# ---------------------------------------------------------------------------
tmp=$(mktemp)
trap 'rm -f "${tmp}"' EXIT
if [[ -f "${RULES_FILE}" ]]; then
    sudo cat "${RULES_FILE}" | awk -v name="${name}" '
        $0 == "# BEGIN manor " name { skip = 1; next }
        $0 == "# END manor " name { skip = 0; next }
        !skip { print }
    ' >"${tmp}"
fi
{
    echo "# BEGIN manor ${name}"
    echo "${rule}"
    echo "# END manor ${name}"
} >>"${tmp}"
sudo install -m 0644 "${tmp}" "${RULES_FILE}"
log "udev rule installed in ${RULES_FILE}:"
log "  ${rule}"

sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
sleep 1

# ---------------------------------------------------------------------------
# 4. dialout group membership.
# ---------------------------------------------------------------------------
needs_relogin=0
if id -nG "${USER}" | grep -qw dialout; then
    log "user ${USER} already in dialout"
else
    sudo usermod -aG dialout "${USER}"
    needs_relogin=1
    log "added ${USER} to dialout"
fi

# ---------------------------------------------------------------------------
# 5. Environment warnings (informational; the udev rule already scopes the fixes).
# ---------------------------------------------------------------------------
if systemctl is-active --quiet ModemManager 2>/dev/null; then
    log "ModemManager is running, but this port is tagged ID_MM_DEVICE_IGNORE -- no action needed"
fi
if dpkg -s brltty >/dev/null 2>&1; then
    log "WARNING: brltty is installed; it is known to hijack some USB-serial adapters. If the device"
    log "         node vanishes right after plugging in, remove it: sudo apt remove brltty"
fi

# ---------------------------------------------------------------------------
# 6. Verify.
# ---------------------------------------------------------------------------
if [[ -e "/dev/${name}" ]]; then
    log "OK: $(ls -la "/dev/${name}" | sed 's/^/  /')"
else
    log "symlink /dev/${name} not present yet -- unplug and replug the bridge, then check:"
    log "  ls -la /dev/${name}"
fi
if [[ ${needs_relogin} -eq 1 ]]; then
    log "IMPORTANT: log out and back in (or run 'newgrp dialout') for the group change to apply"
fi
log "done. Point the CLI at it with: rebot_b601_dm probe --channel /dev/${name}"
