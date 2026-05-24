#!/usr/bin/env bash
# Disable services that are pure idle overhead on rover1.
#
# Re-runnable; quietly ignores units that are already disabled or absent.
# Saves roughly 0.5–1.0 W combined, which extends runtime by 30–60 minutes
# on a typical hobby pack.
#
# Run once after each fresh OS install (or whenever new defaults get pulled
# in by `apt upgrade`):
#   sudo scripts/lean-rover.sh
#
# To re-enable any of them later:
#   sudo systemctl enable --now <unit>

set -u

units=(
    snap.cups.cupsd          # CUPS printer daemon; no printer here.
    snap.cups.cups-browsed   # auto-discovers network printers; same.
    ModemManager             # cellular modem manager; no modem on the bus.
    bluetooth                # Stadia uses the 2.4 GHz dongle, not BT.
    hciuart                  # serial driver for the on-board BT module.
    unattended-upgrades      # could run apt mid-mission; not what we want.
    serial-getty@ttyAMA0     # login prompt on the GPIO UART; we don't use it.
    lttng-sessiond           # kernel tracing daemon; not used.
)

for u in "${units[@]}"; do
    if systemctl list-unit-files "$u" >/dev/null 2>&1; then
        sudo systemctl disable --now "$u" 2>/dev/null || true
        echo "disabled: $u"
    else
        echo "skipped (not installed): $u"
    fi
done

echo
echo "Done. Reboot recommended to fully release Bluetooth/UART resources."
