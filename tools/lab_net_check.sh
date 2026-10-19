#!/usr/bin/env bash
# Harden the Pi's Ethernet setup for the lab, where no DHCP server exists.
#
# The problem: at the lab "Wired connection 1" runs DHCP that never answers.
# NetworkManager only tolerates a failed address family while the *other* one
# succeeds, so with IPv6 unable to complete either, the whole profile goes down
# and takes the manual 192.168.125.2 with it -- the Pi is then only reachable
# after the fall-back profile "robot-net" takes over.
#
# The fix: ipv4.may-fail=yes plus ipv6.method=link-local, which always
# succeeds, so the profile stays up and keeps its manual IPv4 address while
# DHCP fails in the background.  Phase 1 proves that on a throwaway veth pair
# -- a real 802-3-ethernet device with carrier, just with nothing to talk to --
# before Phase 2 touches the real profile.
#
# The active eth0 connection is never reactivated here, so running this cannot
# drop an SSH session.  The new settings take effect at the next activation,
# i.e. when the cable is plugged in at the lab (or on reboot).
set -u

ETH_PROFILE="Wired connection 1"
FALLBACK_PROFILE="robot-net"
ROBOT_NET_ADDRESS="192.168.125.2/24"
TEST_DEVICE="labveth0"
TEST_PEER="labveth1"
TEST_PROFILE="labsim-dhcpfail"
TEST_ADDRESS="192.168.199.2/24"
DHCP_TIMEOUT_SEC=10
SETTLE_SEC=8

if [ "$(id -u)" -ne 0 ]; then
    echo "Run me with sudo:  sudo bash ~/lab_net_check.sh" >&2
    exit 1
fi

cleanup_test() {
    nmcli con down "$TEST_PROFILE" >/dev/null 2>&1
    nmcli con delete "$TEST_PROFILE" >/dev/null 2>&1
    ip link delete "$TEST_DEVICE" >/dev/null 2>&1
}
trap cleanup_test EXIT

echo "=== 0. Current state ==================================================="
ip -br a show eth0
nmcli -t -f NAME,DEVICE,STATE con show --active
echo

echo "=== 1. Experiment: does the manual address survive a DHCP failure? ====="
echo "    (on veth pair $TEST_DEVICE -- eth0 is not touched)"
cleanup_test
verdict="INCONCLUSIVE"
if ip link add "$TEST_DEVICE" type veth peer name "$TEST_PEER" 2>/dev/null &&
   ip link set "$TEST_PEER" up && ip link set "$TEST_DEVICE" up &&
   nmcli con add type ethernet ifname "$TEST_DEVICE" con-name "$TEST_PROFILE" \
        ipv4.method auto ipv4.addresses "$TEST_ADDRESS" \
        ipv4.may-fail yes ipv4.dhcp-timeout "$DHCP_TIMEOUT_SEC" \
        ipv6.method link-local connection.autoconnect no >/dev/null 2>&1; then
    nmcli con up "$TEST_PROFILE" >/dev/null 2>&1
    # Wait past the DHCP timeout, then see what is left on the interface.
    sleep $((DHCP_TIMEOUT_SEC + SETTLE_SEC))
    echo -n "    after $((DHCP_TIMEOUT_SEC + SETTLE_SEC))s: "
    if ! ip link show "$TEST_DEVICE" >/dev/null 2>&1; then
        echo "device gone -- the profile was torn down"
        verdict="FAIL"
    else
        ip -br a show "$TEST_DEVICE"
        if ip -4 addr show "$TEST_DEVICE" | grep -q "${TEST_ADDRESS%/*}"; then
            verdict="PASS"
        else
            verdict="FAIL"
        fi
    fi
else
    echo "    could not build the veth test profile -- experiment skipped"
fi
cleanup_test
echo "    VERDICT: $verdict"
echo

echo "=== 2. Applying the matching configuration ============================="
if [ "$verdict" = "PASS" ]; then
    echo "    manual address survives -> eth0 keeps 192.168.125.2 from second 0"
    nmcli con mod "$ETH_PROFILE" ipv4.may-fail yes \
                                 ipv4.dhcp-timeout "$DHCP_TIMEOUT_SEC" \
                                 ipv6.method link-local
else
    echo "    not proven -> keeping the fail-over chain, only shortening the wait"
    nmcli con mod "$ETH_PROFILE" ipv4.may-fail no \
                                 ipv4.dhcp-timeout "$DHCP_TIMEOUT_SEC"
fi
# The fall-back profile stays armed either way.
nmcli con mod "$ETH_PROFILE" connection.autoconnect yes connection.autoconnect-priority 10
nmcli con mod "$FALLBACK_PROFILE" connection.autoconnect yes connection.autoconnect-priority 1 \
                                  ipv4.method manual ipv4.addresses "$ROBOT_NET_ADDRESS" \
                                  ipv4.never-default yes ipv4.may-fail yes
echo

echo "=== 3. Stored settings (take effect on next plug-in / reboot) =========="
nmcli con show "$ETH_PROFILE"      | grep -E 'ipv4.(method|addresses|dhcp-timeout|may-fail):|ipv6.method:'
echo "    --"
nmcli con show "$FALLBACK_PROFILE" | grep -E 'ipv4.(method|addresses|may-fail):'
echo "    --"
nmcli -t -f NAME,AUTOCONNECT,AUTOCONNECT-PRIORITY con show | grep -E "^($ETH_PROFILE|$FALLBACK_PROFILE):"
echo
echo "Done. eth0 was not reactivated; this session is intact:"
ip -br a show eth0
