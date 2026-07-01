#!/usr/bin/env bash
# Disable system services that are not needed for the breezy-box appliance.
#
# Idempotent — safe to re-run.  Run as root.
#
# At the end, prints any enabled services that are neither in the disable list
# nor in the known-good lists below, so surprises on new Armbian images surface
# immediately rather than silently consuming resources.

set -euo pipefail

if [[ "$(id -u)" -ne 0 ]]; then
    echo "error: run this script with sudo" >&2
    exit 1
fi

section()  { echo; echo "=== $* ==="; }
done_msg() { echo "  done: $*"; }
skip_msg() { echo "  skip: $* (already disabled/masked)"; }

# ---------------------------------------------------------------------------
# Services we explicitly disable.
# ---------------------------------------------------------------------------

disable_svc() {
    local svc="$1"
    local state
    state="$(systemctl is-enabled "$svc" 2>/dev/null || true)"
    if [[ "$state" == "disabled" || "$state" == "masked" || "$state" == "not-found" ]]; then
        skip_msg "$svc"
    else
        systemctl disable --now "$svc" 2>/dev/null || true
        done_msg "disabled $svc"
    fi
}

section "Disable non-essential Armbian services"
# armbian-hardware-monitor: logs hardware state at boot; not needed for appliance
# armbian-hardware-optimize: applies generic IRQ/governor tuning; setup_system.sh
#   handles the only tuning we need (udev rules)
# armbian-led-state: restores LED state across reboots; cosmetic
# armbian-ramlog: stores /var/log in RAM via zram; journald already buffers in RAM,
#   and this adds ~30s to boot
# armbian-zram-config: creates zram swap/log devices; unexpected swap is undesirable
#   on a display appliance
disable_svc armbian-hardware-monitor.service
disable_svc armbian-hardware-optimize.service
disable_svc armbian-led-state.service
disable_svc armbian-ramlog.service
disable_svc armbian-zram-config.service

section "Disable logging / cron services"
# rsyslog: redundant — journald is the primary log sink; keeping both doubles writes
# cron: no scheduled jobs; avoids surprise apt/maintenance CPU spikes mid-session
disable_svc rsyslog.service
disable_svc cron.service

section "Disable dnsmasq (conflicts with systemd-resolved on port 53)"
# dnsmasq fails to start because resolved owns port 53.  networkd's built-in
# DHCPServer covers the direct Ethernet link (configured by setup_system.sh).
disable_svc dnsmasq.service

section "Disable NetworkManager auxiliary services"
# NetworkManager-wait-online: stalls boot 30-90s waiting for a default route that
#   may not exist until a host cable is plugged in
# NetworkManager-dispatcher: runs scripts in /etc/NetworkManager/dispatcher.d/;
#   we have none, so this is a watcher process with no work to do
disable_svc NetworkManager-wait-online.service
disable_svc NetworkManager-dispatcher.service

section "Disable console / keyboard setup services"
# console-setup: sets VT font + keymap; breezy-renderer owns tty1 via KMS/DRM,
#   there is no interactive console to configure
# keyboard-setup: loads a keymap into the kernel; no keyboard attached
disable_svc console-setup.service
disable_svc keyboard-setup.service

section "Disable fake-hwclock (NTP handles time sync)"
# fake-hwclock saves/restores time across reboots for boards with no battery RTC.
# systemd-timesyncd handles sync once the network is up; fake-hwclock is only
# useful on air-gapped boxes and adds two disk writes per boot/shutdown.
disable_svc fake-hwclock-load.service
disable_svc fake-hwclock-save.service

section "Disable e2scrub periodic filesystem check"
# e2scrub_reap cleans up stale snapshot volumes; we have no scheduled scrubs
# (cron is disabled), so this is a watcher that does nothing
disable_svc e2scrub_reap.service

section "Disable systemd-pstore (EFI/ACPI crash dump persistence)"
# systemd-pstore reads crash dumps from the platform persistent store (ACPI/NVRAM).
# The A733 has no pstore backend; the service exits immediately but stalls boot.
disable_svc systemd-pstore.service

# ---------------------------------------------------------------------------
# Audit: report enabled services that are unrecognised.
# ---------------------------------------------------------------------------
#
# KNOWN_GOOD: services we deliberately keep running.
# KNOWN_IGNORE: services that are enabled/static by default in systemd or Debian
#   and are safe to leave alone (one-shots, generators, socket-activated helpers,
#   template units, etc.) — listed here so they don't pollute the unknown report.

KNOWN_GOOD=(
    # systemd core — pulled in by dependencies, not by being "enabled"
    dbus.service
    systemd-journald.service
    systemd-udevd.service
    systemd-logind.service
    systemd-networkd.service
    systemd-resolved.service
    systemd-timesyncd.service
    # networking
    NetworkManager.service
    wpa_supplicant.service
    avahi-daemon.service
    nftables.service
    # breezy stack
    breezy-renderer.service
    breezy-wlan-mdns.service
    breezy-gadget-reset.service
    # remote access
    ssh.service
    sshd-keygen.service
)

KNOWN_IGNORE=(
    # Debian/systemd infrastructure one-shots and generators (static, not enabled)
    alsa-restore.service
    alsa-state.service
    apt-daily-upgrade.service
    apt-daily.service
    armbian-disable-autologin.service
    capsule@.service
    console-getty.service
    container-getty@.service
    dpkg-db-backup.service
    e2scrub@.service
    e2scrub_all.service
    e2scrub_fail@.service
    emergency.service
    fstrim.service
    getty-static.service
    getty@.service
    initrd-cleanup.service
    initrd-parse-etc.service
    initrd-switch-root.service
    initrd-udevadm-cleanup-db.service
    kmod-static-nodes.service
    kmsconvt@.service
    ldconfig.service
    logrotate.service
    man-db.service
    modprobe@.service
    netplan-ovs-cleanup.service
    nm-priv-helper.service
    pam_namespace.service
    quotaon-root.service
    quotaon@.service
    rc-local.service
    rescue.service
    serial-getty@.service
    system-update-cleanup.service
    systemd-ask-password-console.service
    systemd-ask-password-wall.service
    systemd-backlight@.service
    systemd-battery-check.service
    systemd-binfmt.service
    systemd-bsod.service
    systemd-creds@.service
    systemd-exit.service
    systemd-fsck@.service
    systemd-fsck-root.service
    systemd-growfs-root.service
    systemd-growfs@.service
    systemd-halt.service
    systemd-hibernate-clear.service
    systemd-hibernate-resume.service
    systemd-hibernate.service
    systemd-hostnamed.service
    systemd-hwdb-update.service
    systemd-hybrid-sleep.service
    systemd-initctl.service
    systemd-journal-catalog-update.service
    systemd-journal-flush.service
    systemd-journald-sync@.service
    systemd-journald.service
    systemd-journald@.service
    systemd-kexec.service
    systemd-machine-id-commit.service
    systemd-modules-load.service
    systemd-network-generator.service
    systemd-networkd-wait-online.service
    systemd-pcrfs@.service
    systemd-pcrlock@.service
    systemd-pcrmachine.service
    systemd-pcrphase-initrd.service
    systemd-pcrphase-sysinit.service
    systemd-pcrphase.service
    systemd-poweroff.service
    systemd-quotacheck-root.service
    systemd-quotacheck@.service
    systemd-random-seed.service
    systemd-reboot.service
    systemd-remount-fs.service
    systemd-rfkill.service
    systemd-soft-reboot.service
    systemd-storagetm.service
    systemd-suspend-then-hibernate.service
    systemd-suspend.service
    systemd-sysctl.service
    systemd-sysext@.service
    systemd-sysusers.service
    systemd-timedated.service
    systemd-tmpfiles-clean.service
    systemd-tmpfiles-setup-dev-early.service
    systemd-tmpfiles-setup-dev.service
    systemd-tmpfiles-setup.service
    systemd-tpm2-setup-early.service
    systemd-tpm2-setup.service
    systemd-udev-settle.service
    systemd-udev-trigger.service
    systemd-udevd.service
    systemd-update-done.service
    systemd-update-utmp-runlevel.service
    systemd-update-utmp.service
    systemd-user-sessions.service
    systemd-volatile-root.service
    user-runtime-dir@.service
    user@.service
)

section "Audit: enabled services not accounted for"

# Build lookup sets from KNOWN_GOOD + KNOWN_IGNORE + DISABLE lists.
declare -A KNOWN=()
for s in "${KNOWN_GOOD[@]}" "${KNOWN_IGNORE[@]}"; do KNOWN["$s"]=1; done

DISABLE=(
    armbian-hardware-monitor.service armbian-hardware-optimize.service
    armbian-led-state.service armbian-ramlog.service armbian-zram-config.service
    rsyslog.service cron.service dnsmasq.service
    NetworkManager-wait-online.service NetworkManager-dispatcher.service
    console-setup.service keyboard-setup.service
    fake-hwclock-load.service fake-hwclock-save.service
    e2scrub_reap.service systemd-pstore.service
)
for s in "${DISABLE[@]}"; do KNOWN["$s"]=1; done

UNKNOWN=()
while IFS= read -r line; do
    # list-unit-files output: "name.service  enabled  enabled"
    svc="$(awk '{print $1}' <<< "$line")"
    state="$(awk '{print $2}' <<< "$line")"
    [[ -z "$svc" ]] && continue
    [[ "${KNOWN[$svc]+_}" ]] && continue
    UNKNOWN+=("$svc ($state)")
done < <(systemctl list-unit-files --type=service --no-legend 2>/dev/null \
         | awk '$2 == "enabled" || $2 == "enabled-runtime"')

if [[ "${#UNKNOWN[@]}" -eq 0 ]]; then
    echo "  all enabled services are accounted for"
else
    echo "  REVIEW: the following enabled services are not in the known-good or disable lists."
    echo "  Add them to KNOWN_IGNORE (leave alone) or DISABLE (turn off) as appropriate."
    echo
    for s in "${UNKNOWN[@]}"; do
        echo "    $s"
    done
fi

echo
echo "=== Done. ==="
