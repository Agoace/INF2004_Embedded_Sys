#!/usr/bin/env bash
#
# Restrict inbound telemetry UDP port 4242 to one Pico W.
#
# The telemetry receiver accepts unauthenticated UDP. Any device on the local
# network can send packets to it. This script adds an nftables rule set that
# accepts port 4242 only from the Pico W hardware address, and drops all other
# traffic to that port.
#
# The rule matches the hardware address, not the IP address. A hardware address
# does not change when DHCP gives the Pico a different IP address. The match
# works only on the local network segment, which is where the Pico is.
#
# This script uses its own nftables table. It does not read, change, or remove
# any other firewall rule.
#
# Usage:
#   sudo ./restrict-telemetry-port.sh apply <pico-mac>
#   sudo ./restrict-telemetry-port.sh status
#   sudo ./restrict-telemetry-port.sh remove
#
# The rules are not permanent. Run "apply" again after a reboot, or add the
# table to /etc/nftables.conf to keep it.

set -euo pipefail

TABLE_NAME="telemetry_guard"
TELEMETRY_PORT=4242

usage() {
    cat <<'EOF'
Usage:
  sudo ./restrict-telemetry-port.sh apply <pico-mac>
  sudo ./restrict-telemetry-port.sh status
  sudo ./restrict-telemetry-port.sh remove

Find the Pico hardware address with:
  ip neigh | grep <pico-ip>
EOF
    exit 1
}

require_root() {
    if [ "$(id -u)" -ne 0 ]; then
        echo "Error: run this script with sudo." >&2
        exit 1
    fi
}

validate_mac() {
    local mac="$1"
    if ! printf '%s' "$mac" | grep -qiE '^([0-9a-f]{2}:){5}[0-9a-f]{2}$'; then
        echo "Error: '$mac' is not a hardware address in aa:bb:cc:dd:ee:ff form." >&2
        exit 1
    fi
}

apply_rules() {
    local mac="$1"
    validate_mac "$mac"

    # Start from a known state so a second run does not stack rules.
    nft delete table inet "$TABLE_NAME" 2>/dev/null || true

    nft add table inet "$TABLE_NAME"
    nft add chain inet "$TABLE_NAME" input \
        '{ type filter hook input priority 0 ; policy accept ; }'

    # Accept telemetry from the Pico only.
    nft add rule inet "$TABLE_NAME" input \
        ether saddr "$mac" udp dport "$TELEMETRY_PORT" accept

    # Always allow the loopback interface, for local testing.
    nft add rule inet "$TABLE_NAME" input \
        iif lo udp dport "$TELEMETRY_PORT" accept

    # Drop every other source for this port. Counters show blocked attempts.
    nft add rule inet "$TABLE_NAME" input \
        udp dport "$TELEMETRY_PORT" counter drop

    echo "Applied: UDP port $TELEMETRY_PORT accepts $mac and loopback only."
    echo "The policy is accept, so no other port or service is affected."
    echo "These rules are lost on reboot. Run apply again, or store the table"
    echo "in /etc/nftables.conf."
}

show_status() {
    if nft list table inet "$TABLE_NAME" 2>/dev/null; then
        return 0
    fi
    echo "No $TABLE_NAME table is present. Port $TELEMETRY_PORT is unrestricted."
}

remove_rules() {
    if nft delete table inet "$TABLE_NAME" 2>/dev/null; then
        echo "Removed the $TABLE_NAME table. Port $TELEMETRY_PORT is unrestricted again."
        return 0
    fi
    echo "No $TABLE_NAME table was present. Nothing was changed."
}

main() {
    local action="${1:-}"
    case "$action" in
        apply)
            require_root
            [ "$#" -eq 2 ] || usage
            apply_rules "$2"
            ;;
        status)
            require_root
            show_status
            ;;
        remove)
            require_root
            remove_rules
            ;;
        *)
            usage
            ;;
    esac
}

main "$@"
