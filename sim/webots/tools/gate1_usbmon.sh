#!/usr/bin/env bash
# Optional operator-approved bus capture only. Never opens the Probe UART.
set -euo pipefail
usage() { printf 'Usage: sudo bash %s USB_BUS ABSOLUTE_FRESH_EVIDENCE_DIR\n' "$0"; }
if [[ ${1:-} == --help ]]; then usage; exit 0; fi
if [[ $# != 2 ]]; then usage >&2; exit 2; fi
if [[ $EUID != 0 ]]; then
    printf 'STOP: this optional capture requires operator-approved sudo\n' >&2
    exit 2
fi
if [[ ! ${SUDO_UID:-} =~ ^[0-9]+$ || ! ${SUDO_GID:-} =~ ^[0-9]+$ ||
      -z ${SUDO_USER:-} || ${SUDO_UID:-0} == 0 ]]; then
    printf 'STOP: require the original non-root operator sudo identity\n' >&2
    exit 2
fi
bus=$1
evidence=$2
if [[ ! $bus =~ ^[1-9][0-9]{0,4}$ ]] || (( bus > 65535 )); then
    printf 'STOP: supply the resolved Probe bus, never bus0/all buses\n' >&2
    exit 2
fi
if [[ $evidence != /* || ! -d $evidence ||
      $(stat -c %u -- "$evidence") != "$SUDO_UID" ||
      $(stat -c %a -- "$evidence") != 700 ||
      $(/usr/bin/id -u "$SUDO_USER") != "$SUDO_UID" ]]; then
    printf 'STOP: require an absolute operator-owned evidence directory, mode0700\n' >&2
    exit 2
fi
pidfile=$evidence/usbmon.pid
errfile=$evidence/usbmon.stderr
for file in "$pidfile" "$errfile" "$evidence/usb.pcap"; do
    if [[ -e $file || -L $file ]]; then
        printf 'STOP: capture output already exists: %s\n' "$file" >&2
        exit 2
    fi
done
umask 077
set -C
: >"$errfile"
# Create/chown under sudo BEFORE redirecting, even if the caller used sudo sh.
chown "$SUDO_UID:$SUDO_GID" "$errfile"
exec 2>>"$errfile"
modprobe usbmon
printf '%s\n' "$$" >"$pidfile"
chown "$SUDO_UID:$SUDO_GID" "$pidfile"
# PID is unchanged by exec; tcpdump opens the bus then drops to the operator.
exec /usr/bin/tcpdump -Z "$SUDO_USER" -i "usbmon$bus" -s 0 -U -w "$evidence/usb.pcap"
