#!/usr/bin/env bash
#
# test_dl_with_reader.sh - swap to DisplayLink-only AND immediately start a
# reader draining /dev/udl_gadget, to test whether the first-bulk-OUT board
# hang is caused by the gadget fifo having no consumer.
#
# Run as root with the host connected on the OTG port (A733 PD port).  Watch a
# separate `sudo dmesg -wH` session.
#
set -uo pipefail
MOD_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ $EUID -ne 0 ]]; then echo "run as root"; exit 1; fi

echo "=== swapping to DisplayLink-only (force, host attached) ==="
"$MOD_DIR/load_module.sh" --dl-only --force-attached || { echo "load failed"; exit 1; }

# Start draining the gadget fifo immediately so bulk-OUT completions have a
# consumer.  Discard the data; we only care whether draining prevents the hang.
echo "=== starting /dev/udl_gadget reader (draining to /dev/null) ==="
if [[ -e /dev/udl_gadget ]]; then
    ( cat /dev/udl_gadget > /dev/null 2>&1 & echo "reader pid $!" )
else
    echo "WARN: /dev/udl_gadget missing"
fi

echo "=== reader running. Now the host should push bulk pixel data. ==="
echo "Watch dmesg for 'bulk complete entered' (good) vs a hang (bad)."
sleep 3
echo "=== bulk rx stats after 3s ==="
dmesg | grep -iE 'bulk complete|bulk rx|f_displaylink: bulk' | tail -5 || echo "(no bulk completions logged)"
