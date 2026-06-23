# Hardware notes — Radxa Cubie A7S

Definitive board-specific notes for running breezy-box on the **Radxa Cubie A7S**.
Keep this current: anything here is something that bit us and will bite a future
change if forgotten. Companion file: [rock-4c-plus.md](rock-4c-plus.md).

| | |
|---|---|
| SoC | Allwinner A733 (ARM, big.LITTLE) |
| GPU | Imagination **PowerVR BXM-4-64** (B-series, BVNC `36.56.104.183`) — render via closed `pvrsrvkm`. **NOT Mali/Panfrost.** See §7 |
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
- Two unit settings can silently *mask* a working file cap (neither bit us here, but
  check them first if `getcap` shows the cap yet `sched_setscheduler` still gets `EPERM`):
  an explicit **`CapabilityBoundingSet`** that omits `cap_sys_nice` (the cap is ANDed with
  it), and **`NoNewPrivileges=yes`** (nullifies file caps entirely). `breezy-renderer.service`
  sets neither — its bounding set defaults to all-caps and `NoNewPrivileges=no` — so on this
  board the cap was fine and the blocker was the kernel (below).
- The CPU pin (`sched_setaffinity`) needs no capability. `LimitRTPRIO=40` stays in the
  unit as a backstop. If the boost still can't be set the renderer logs and continues at
  normal priority (best-effort).

**The cap is necessary but NOT sufficient on this board — the real blocker is the kernel.**
This Allwinner BSP kernel ships **`CONFIG_RT_GROUP_SCHED=y`** with **cgroup v2**. That
combination denies `SCHED_FIFO` to *every* non-root task group — and **cgroup v2 has no
interface to grant RT bandwidth to a child cgroup** (that was the v1 `cpu.rt_runtime_us`
knob). Since systemd puts every service in a sub-cgroup, *nothing* — not even root —
can get realtime. Proof: `sudo chrt -f 10 true` fails with `Operation not permitted`
even as root. (This is why §4's `setcap` recipe works on the RK3399, which ships
`RT_GROUP_SCHED=n`, but not here.)

**Fix (guarded):** disable RT *throttling* globally, which bypasses the per-group
enforcement:
```sh
sudo sysctl -w kernel.sched_rt_runtime_us=-1      # test
echo 'kernel.sched_rt_runtime_us = -1' | sudo tee /etc/sysctl.d/10-breezy-rt.conf  # persist
# → renderer log: "kms: render thread boosted to SCHED_FIFO prio 20"
```
`-1` removes RT throttling system-wide (a runaway RT thread could starve the box) — fine
for a dedicated appliance running one bounded, CPU-0-pinned render thread, but **only
apply it where RT is actually blocked.** `setup_system.sh` gates it on a live
`chrt -f 1 true` probe: it writes the sysctl only if the probe is denied, so boards where
`setcap` already suffices keep their RT throttle intact. The *correct* long-term fix is a
kernel built with `CONFIG_RT_GROUP_SCHED=n` (what Debian/Ubuntu generic kernels do for
exactly this reason) — worth raising with Radxa for the a733 BSP.

> ld.so + the cap: a file-capability binary is AT_SECURE, so ld.so **ignores
> `LD_LIBRARY_PATH` and `$ORIGIN` rpath**. The renderer links its sibling
> `ZeroKvm.NativeBridge.so` (NativeAOT) *and* — on this board — the PowerVR Mesa in
> `/usr/local/lib` (see §7), neither of which is on the default search path. So setup must
> bake an **absolute** rpath (`patchelf --force-rpath --set-rpath "$BINDIR:/usr/local/lib"`),
> applied **before** `setcap` (patchelf rewrites the file and wipes the cap), and the
> `ZeroKvm.NativeBridge.so` must sit next to the binary. See §7 for the full wiring.

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

## 7. GPU acceleration — PowerVR working via Radxa BXM userspace (verified 2026-06)

**Status: hardware GLES *and* Vulkan confirmed on Armbian/Debian 13.** This is **not**
a Panfrost SoC — the A733 has an **Imagination PowerVR**, not a Mali. Two *separate*
DRM devices:

- **Display controller** `sunxi-drm` (`/dev/dri/card0`, `/soc@3000000/sunxi-drm`) —
  Allwinner display engine: connectors/CRTCs/planes/page-flip/DMA-buf scanout.
  **GPU-independent: putting a buffer on screen never touches PowerVR.**
- **GPU render engine** `pvrsrvkm` (`gpu@1800000`, `img,gpu`) — the **closed**
  Imagination kernel module. HW reports **BVNC `36.56.104.183` = BXM-4-64 (B-series)**,
  DDK **`24.2@6603887`** (`dmesg | grep 'Read BVNC'` / `Driver Version`).

**The original gap:** Armbian ships only the *kernel* half (`img-bxm-dkms`, source
`aw-drivers-dkms`) — no userspace blob, and Armbian/Debian repos don't carry one →
`vulkaninfo` said *"Found no drivers"* and GL fell to **llvmpipe**. (Also a stale
`/usr/share/vulkan/icd.d/img_icd.json` pointing at a non-existent `libVK_IMG.so`.)

### The fix — install Radxa's matching userspace blob

The userspace lives in **Radxa's a733 test repo**, version-locked to the same DDK as
the kernel module, so it binds with **no `DDK_VERSION_MISMATCH`**:

```sh
# One package contains Vulkan + GLES + libsrv_um + firmware + a full Mesa/pvr_dri stack
BASE=https://radxa-repo.github.io/a733-trixie-test
FN=$(curl -sL $BASE/dists/a733-trixie-test/main/binary-arm64/Packages.gz \
      | zcat | awk '/^Package: xserver-xorg-img-bxm-1.21.1-2.deb$/{f=1}
                    f&&/^Filename:/{print $2; exit}')
curl -L -o /tmp/img-bxm.deb "$BASE/$FN"
sudo dpkg -i /tmp/img-bxm.deb          # pkg "xserver-xorg-img-bxm-1.21.1-2.deb" v1.0.1
sudo /usr/sbin/ldconfig                # NB: ldconfig isn't on the default user PATH
```

What it installs (DDK `24.2@6603887`, matches the kernel exactly):
- `/usr/lib/libVK_IMG.so*`, `libsrv_um.so*`, `libGLESv2_PVR_MESA.so*` — closed IMG backend
- `/usr/local/lib/{libEGL,libgbm,libGLESv2,libglapi,libvulkan}.so*` + `dri/pvr_dri.so`
  + `dri/sunxi-drm_dri.so` + `libpvr_mesa_wsi.so` — a **full Mesa build with the PVR
  Gallium driver**, under `/usr/local`
- `/lib/firmware/rgx.fw.36.56.104.183` + `rgx.sh.36.56.104.183` — the GPU firmware +
  shader binary (the *other* missing piece; without these `dmesg` shows
  `Firmware Version: 0.0 @ 0` and the GPU never inits)

### Verify (headless — no glasses needed)

```sh
sudo dmesg | grep -iE 'rgx|firmware|bvnc|mismatch'
#   WANT: "RGX Firmware image 'rgx.fw.36.56.104.183' loaded" + shader binary, NO mismatch
vulkaninfo 2>&1 | grep -iE 'deviceName|driverName|driverInfo'
#   WANT: PowerVR B-Series BXM-4-64 MC1 / PowerVR B-Series Vulkan Driver / 24.2@6603887
LD_LIBRARY_PATH=/usr/local/lib LIBGL_DRIVERS_PATH=/usr/local/lib/dri \
  eglinfo 2>&1 | grep -iE 'GBM platform|EGL driver name|profile renderer|profile version'
#   WANT: GBM platform / sunxi-drm / PowerVR B-Series BXM-4-64 / OpenGL ES 3.2 24.2@6603887
```

### Gotchas (all hit during bring-up)

- **`eglinfo`/`es2_info` need `libxcb-dri2-0`** (else "cannot open libxcb-dri2.so.0" and
  *no* output) — `sudo apt install -y libxcb-dri2-0 mesa-utils`.
- **`glmark2-drm` fails headless** ("Failed to find a suitable connector") — it does a
  real modeset and needs the glasses attached. Use `eglinfo` (surfaceless/GBM) instead.
- The `.deb` declares **no `Depends`** (Allwinnertech vendor packaging) — it assumes
  libc/libdrm/libvulkan1/glvnd are already present (they are on Armbian).
- It installs a **parallel Mesa under `/usr/local/lib`** that must shadow the system
  Mesa. Fine on a headless box, but see the wiring caveat below.
- **Android blobs are a dead end:** the `a733_android13` image's PowerVR libs
  (`libsrv_um.so`, `vulkan.jupiter.so`, …) are **Bionic** builds with Android WSI — they
  won't load on glibc. Use the *Linux* (`Rogue_DDK_Linux_WS`) build above.

### Wiring breezy to it — absolute rpath (the `cap_sys_nice` interaction, see §4)

**Verified live:** `/proc/$(pidof displaylink_kms_renderer)/maps` shows the renderer on
`renderD128` + `/usr/local/lib/{libEGL,libgbm,libGLESv2}` + `pvr_dri.so` (no `swrast`/
`llvmpipe`). Note `EGL 1.5 on Mesa Project` in the log is **not** a sign of software — the
PVR Mesa reports the same `"Mesa Project"` vendor string; only the maps disambiguate.

The renderer has `cap_sys_nice+ep`, so it's AT_SECURE: **ld.so ignores `LD_LIBRARY_PATH`/
`$ORIGIN` rpath, and Mesa ignores `LIBGL_DRIVERS_PATH`.** So the env-var trick from the
verify commands can't wire breezy — it must resolve on default paths. Two facts:

- **DRI path is baked in** — `LD_LIBRARY_PATH=/usr/local/lib eglinfo` (no
  `LIBGL_DRIVERS_PATH`) still reports PowerVR, because the Mesa is built `prefix=/usr/local`
  → defaults to `/usr/local/lib/dri`. So the secure-mode `LIBGL_DRIVERS_PATH` strip is moot.
- **The loader cache can't be reordered to help** — on this board `ldconfig -p` lists
  `/lib/aarch64-linux-gnu/lib{EGL,gbm,GLESv2}` *first* and a `00-pvr.conf` (`/usr/local/lib`)
  **does not flip it** (the multiarch dir stays ahead regardless of `.conf` sort). So the
  cache-ordering approach is a dead end here.

**The method that works: an absolute rpath on the binary** (absolute → AT_SECURE-safe,
unlike `$ORIGIN`). It must list **both** the binary's own dir (for its NativeAOT sibling
`ZeroKvm.NativeBridge.so`) **and** `/usr/local/lib` (PVR Mesa):

```sh
BIN=~/.local/bin/displaylink_kms_renderer
# ZeroKvm.NativeBridge.so must sit NEXT TO the binary — it's only built into the .NET
# publish tree by default; copy it into the install dir so the install is self-contained:
cp .../ZeroKvm.NativeBridge/bin/Release/net10.0/linux-arm64/publish/ZeroKvm.NativeBridge.so ~/.local/bin/
patchelf --force-rpath --set-rpath "$(dirname "$BIN"):/usr/local/lib" "$BIN"
sudo setcap cap_sys_nice+ep "$BIN"     # AFTER patchelf — it rewrites the file and wipes the cap
```

Gotchas that bit us here:
- **Order matters:** `patchelf` after `setcap` silently drops the cap → SCHED_FIFO fails.
- **`ZeroKvm.NativeBridge.so` not beside the binary** → `cannot open shared object file`
  at startup. Under the cap, `$ORIGIN` won't save you; the rpath's first entry must be the
  binary's absolute dir, and the `.so` must actually be there.
- Both rpath and cap are **wiped on every recopy** — `setup_system.sh` re-applies them.

> Confirm via `/proc/$PID/maps` (PVR libs + `pvr_dri.so`, no `llvmpipe`/`swrast`) before
> trusting it on the headset, so a half-wired EGL doesn't drop into the §3/§4 restart-loop.

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
