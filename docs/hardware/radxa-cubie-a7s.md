# Hardware notes — Radxa Cubie A7S

Definitive board-specific notes for running breezy-box on the **Radxa Cubie A7S**.
Keep this current: anything here is something that bit us and will bite a future
change if forgotten. Companion file: [rock-4c-plus.md](rock-4c-plus.md).

| | |
|---|---|
| SoC | Allwinner A733 (ARM, big.LITTLE) |
| GPU | Arm Mali (Panfrost/Mesa) |
| OS image | Armbian-based, **apt/Debian** package manager |
| OTG / gadget port | USB-C OTG (libcomposite RNDIS + DisplayLink) |
| Wired-to-host path | USB RNDIS gadget `usb0` (192.168.7.2/30), host gets .1 |
| Status | Brought up 2026-06; primary "newer Cubie hardware" |

---

## 1. Network stack — NetworkManager + systemd-networkd both active

The image runs **both** NetworkManager and systemd-networkd, enabled and active:

- **NetworkManager** manages `wlan0` and `eth0`.
- **systemd-networkd** manages `usb0` (NM marks it `unmanaged`).

**Consequence:** any systemd-networkd `.network` profile for `eth0` is **inert** —
NM keeps the device and applies no static IP / DHCP. The original `setup_system.sh`
installed `05-breezy-eth0.network` and it silently did nothing here.

**Fix in setup_system.sh (NM-aware):** when `nm_active()` is true, the direct-Ethernet
section writes an NM keyfile instead:

```
/etc/NetworkManager/system-connections/breezy-eth0.nmconnection
  ipv4.method=shared
  address1=192.168.77.2/30      # NM's own dnsmasq serves the host on the far end
  chmod 0600                    # NM refuses group/world-readable keyfiles
```

…and **removes the fighting networkd profile** so the two stacks don't both grab
the interface. Falls back to networkd's built-in `[DHCPServer]` only when NM isn't
active.

> 192.168.77.x is deliberately uncommon among consumer routers so the wired link
> never collides with the Wi-Fi/LAN uplink subnet.

## 2. No firewall tooling shipped — port redirect silently failed

The image ships with **neither `nft` nor `iptables`** installed. The setup section
that redirects `:443 → :8443` and `:80 → :8081` hit its final `else` branch and only
*warned*. Result: the web UI was reachable **only on `:8443`**; `https://<box>` (port
443) got no answer — looked like "the UI isn't accessible."

**Fix in setup_system.sh:** auto-installs `nftables` via apt when neither tool is
present, then enables `nftables.service` so the rule survives reboot.

**Immediate access while debugging (no redirect needed):**
```
https://192.168.0.<wlan>:8443      # over WLAN
https://192.168.7.2:8443           # over the USB RNDIS link
```
The TLS cert SAN includes both `192.168.77.2` and `192.168.7.2`.

## 3. DRM master stolen by a console daemon (kmscon) — black screen + restart loop

The single worst gotcha. The image ships **`kmsconvt@tty1.service`** (kmscon, a
userspace KMS console) running as **root**, which takes **DRM master on
`/dev/dri/card0`** and holds it.

The breezy renderer is a system service bound to tty1 to get a seat0 session, but
because kmscon already owns master, the renderer gets:

```
kms: drmSetMaster: Permission denied (may still work)
drmModeSetCrtc: Permission denied
```

…then exits, `Restart=always` respawns it, and you see a **tight restart loop**
(two interleaved `EGL 1.5 … / DMA-buf path: enabled / compositor running` blocks in
the journal) with a black screen.

**Diagnose:**
```
sudo fuser -v /dev/dri/card*
#   /dev/dri/card0:  root  <pid>  kmscon          <-- the thief
#                    wayne <pid>  displaylink_kms
loginctl list-sessions          # seat0/tty1 leader should be the renderer
```

**Fix in setup_system.sh — generic console masking (not hardcoded):** the renderer
must be the *sole* owner of its VT. Rather than name getty/kmscon, setup discovers
and masks every console unit that could occupy `RENDERER_VT` (=`tty1`):

1. Loaded `*@tty1.service` instances (whatever the image actually spawned).
2. Template families `getty` / `kmsconvt` / `serial-getty` whose unit *file* exists →
   mask their `@tty1` instance (catches the not-yet-loaded, lazily-instantiated case).
3. Static console services `kmscon.service` / `console-getty.service` if present.

Scoped to tty1 (other VTs untouched); **services only, never `.target`s**.

> A *running* console daemon keeps card0 until stopped — re-running setup masks +
> stops it; otherwise `sudo systemctl mask --now kmsconvt@tty1.service` then restart
> breezy-renderer.

## 4. Render-thread RT boost — do NOT use AmbientCapabilities

The renderer puts its GL render thread on `SCHED_FIFO` (+ pins to CPU 0) so IMU
head-anchoring stays smooth while decode threads load the other cores.

**Trap:** `AmbientCapabilities=CAP_SYS_NICE` on `breezy-renderer.service` **breaks the
seat**. Combined with `User=` + `PAMName=login` it changes the PAM credential ordering
so logind no longer seats the session on seat0 — which *also* costs DRM master
(`drmSetMaster: Permission denied`). Symptom looks identical to the kmscon problem.

**What actually works:** `LimitRTPRIO=40` alone is **NOT** enough on this kernel — it
still refuses unprivileged `SCHED_FIFO` (`kms: SCHED_FIFO boost unavailable (Operation
not permitted)`). The fix is to grant `CAP_SYS_NICE` **on the binary** via `setcap`
(setup_system.sh runs `setcap cap_sys_nice+ep` on the renderer after the unit install):

```
sudo setcap cap_sys_nice+ep ~/.local/bin/displaylink_kms_renderer
```

A *file* capability is independent of the systemd credential path, so it grants the cap
**without** perturbing the seat (unlike `AmbientCapabilities`). Caveats:
- Must be **re-applied whenever the binary is recopied** — setup does it every run; if
  you `cp` a fresh build manually, re-run setup or re-`setcap`.
- A file-capability binary is treated like setuid by ld.so, so it ignores
  `LD_LIBRARY_PATH`/`$ORIGIN` rpath — fine here, the renderer's libs (Mesa/EGL/libdrm/
  gbm) are in standard system paths.
- The CPU pin (`sched_setaffinity`) needs no capability. `LimitRTPRIO=40` stays in the
  unit as a backstop. If the boost still can't be set the renderer logs and continues at
  normal priority (best-effort).

## 5. Async page flip unsupported — no true vsync-off / tearing path

The Mali/DRM driver does **not** advertise `DRM_CAP_ASYNC_PAGE_FLIP`:

```
kms: async page flip: disabled
```

So the "no vsync, accept tearing, lowest latency" path is **unavailable on this
board** — the render loop falls back to vsync-synced flips, capping framerate to the
display refresh. This is a hardware limit, not a bug. The latency wins on this board
come from the DMA-buf decode-decoupling (decode thread writes the scanout buffer
directly) and the RT render thread, not from async flip.

## 6. DisplayLink decode is a hardware-bound wall on ARM (shared with all SBCs)

DisplayLink decompression is **memory-stall-bound**, not compute-bound: the comp16
lookup working set (~1.37 MB) exceeds typical ARM shared L2, so every lookup misses to
DRAM. This is an architectural mismatch (DisplayLink targets x86 hosts with large L3 +
kernel-mode decode), **not a fixable software bug**.

- **Use case is productivity only** (mouse/keyboard/static content) — idle damage is
  trivially cheap. Video/gaming saturates the decompressor and is unsupported by design.
- With N displays, N× the working set competes for the same L2 → contention scales with
  display count.
- See [rock-4c-plus.md §DisplayLink decode](rock-4c-plus.md) and the
  eth-CPU-pegging notes for the full RK3399 analysis; the same wall applies here, just
  with a different cache size. **Measure on this board** before assuming numbers.

---

## Deploy / verify cheat-sheet

```sh
cd ~/breezy-box && git pull
sudo ./setup_system.sh                 # NM link, nftables, console masking, units

# Make sure no console daemon is squatting on card0:
sudo systemctl mask --now kmsconvt@tty1.service
sudo fuser -v /dev/dri/card0           # should show ONLY displaylink_kms

sudo systemctl restart breezy-renderer.service
sleep 2
journalctl -u breezy-renderer -n 25 --no-pager \
  | grep -iE 'master|crtc|SCHED_FIFO|async|rendering to|Permission'
# WANT: "rendering to ... DP-1" + "SCHED_FIFO prio 20", NO "Permission denied"

systemctl --user daemon-reload && systemctl --user restart breezy.target
sudo ss -tlnp | grep -E ':(8443|8080|8081|3240)'   # web, noVNC, http, USB/IP
```

## Quick reference — addresses & ports

| Thing | Value |
|---|---|
| WLAN | `192.168.0.x` (DHCP from your router) |
| USB RNDIS link (`usb0`) | box `192.168.7.2/30`, host `.1` |
| Direct Ethernet (`eth0`, NM shared) | box `192.168.77.2/30`, host `.1` |
| Web UI (HTTPS) | `:8443` (redirect from `:443` once nftables installed) |
| noVNC | `:8080` |
| USB/IP | `:3240` |
| Renderer VT | `tty1` (seat0) |
