# Hardware notes — Radxa Rock Pi 4C+

Definitive board-specific notes for running breezy-box on the **Radxa Rock Pi 4C+**.
This is the original bring-up board; most of the deep DisplayLink/OTG analysis was
done here. Companion file: [radxa-cubie-a7s.md](radxa-cubie-a7s.md).

| | |
|---|---|
| SoC | Rockchip **RK3399** (2× Cortex-A72 + 4× Cortex-A53, big.LITTLE) |
| GPU | Mali-T860 (Panfrost/Mesa) |
| OS image | **Armbian** rockchip64, kernel `6.18.33-current-rockchip64` (era of these notes) |
| OTG / gadget controller | dwc3 `fe800000.usb` behind `rockchip_usb2phy` |
| Wired-to-host path | USB RNDIS gadget `usb0`; native Ethernet port also available |
| Status | Original board; superseded for daily use by the Cubie A7S |

---

## 1. OTG port: dwc3 `fe800000.usb`, **High-Speed only**

The OTG/gadget port sits behind `rockchip_usb2phy`, which means it is **USB
High-Speed only — SuperSpeed is physically unavailable** on this port. Earlier
SuperSpeed bring-up attempts broke because of this; do not retry. This constraint is
load-bearing for the CPU analysis below (no bursting / large-transfer batching to cut
transfer count).

### Forcing gadget mode — `install_usb0_peripheral_overlay.sh`

The controller must be put into gadget-capable mode via a boot-persistent DT overlay:

```sh
sudo ./install_usb0_peripheral_overlay.sh            # mode=peripheral (known-good)
# overlay name: rk3399-rock-4c-plus-usb0-peripheral
```

`peripheral` is the known-good setting for exposing `fe800000.usb` as a UDC on the
current Armbian image.

### BC1.2 / DCP misclassification recovery — `reset_usb0_otg_stack.sh`

The RK3399 `usb2phy` OTG path has a quirk where the charger-detection logic
misclassifies the host as a **DCP** (dumb charger) instead of **CDP**, leaving the
controller in device mode but **not attached** with `current_speed = UNKNOWN`. The
raw-gadget process can't recover from this on its own.

```sh
sudo ./reset_usb0_otg_stack.sh                 # auto-detects the bound UDC
sudo ./reset_usb0_otg_stack.sh --udc fe800000.usb --settle 2
sudo ./reset_usb0_otg_stack.sh --dry-run
```

It unbinds/rebinds the dwc3 platform device and its matching usb2phy OTG PHY in order.
Targets are the bad state: UDC exists, device mode but unattached, `current_speed`
stuck UNKNOWN, usb2phy extcon reporting `DCP=1` instead of `CDP=1`. On non-RK3399 SoCs
the nodes don't exist and it exits cleanly without touching anything.

## 2. Single-core CPU pegging on the OTG ethernet path — **two layers, both real**

The headline symptom: **one CPU core pegs >100% when the DisplayLink stream comes over
the OTG ethernet link, but stays ~60% over WLAN.** The eth-vs-WLAN gap is a *feed-rate
artifact* (eth feeds the consumer faster than it runs; WLAN's slower link rate-limits
the producer so the consumer keeps up).

### Layer A — userspace, copy-bound (the confirmed root cause)

`top -H` at peak shows one `displaylink_kms` thread at 99%; `perf` shows it
**copy-bound, not compute-bound**. `udl_transport_feed` called
`udl_sink_sync_output` → full-frame RGB565→XRGB8888 diff-convert **once per bulk packet
(~200/s)**, each scanning the entire ~2 MP framebuffer regardless of how little changed.

**Fixes implemented (in-repo):**
- Sync-throttle bug fixed: a top-of-feed `udl_transport_flush_sync` ran on *every*
  packet, defeating the throttle. Now `udl_runtime_flush_pending` fires only when the
  decode queue drains (stream went idle). Tunable via `BREEZY_UDL_SYNC_INTERVAL_MS`
  (default 12 ms).
- Dirty range accumulates across packets (`ResetDirtyRange` moved into
  `CopyFrameBufferTo`).
- `CopyLine16Full` no longer writes the never-read `_frameBufferDiff16` (≈33% less sync
  memory traffic per row).
- Per-command-type pixel counters + bridge stats via `BREEZY_UDL_DECODE_STATS=1`.

### Layer B — kernel per-USB-transfer churn (secondary, hardware-bounded)

dwc3 (IRQ 85, `fe800000.usb`) fires ~2,500–5,500 IRQs/s **all pinned to CPU0** (single
GIC line, no multi-queue), plus NET_RX/NET_TX softirqs also on CPU0. Cost scales with
**USB transfer count (~1–2 KB each), not IP packet size** — which is why no IP-level
knob moves it.

**Ruled out (each tested, all null):**
- **Jumbo MTU / NCM** (6.5 KB frames, ~5× fewer IP packets) → load unchanged. Not
  per-IP-packet.
- **Lower refresh (50/30 Hz)** → no effect (continuous damage; frame cap doesn't cut
  per-transfer rate). Also see §4 — 30 Hz isn't even offerable.
- **IRQ smp_affinity** → moves the hotspot, can't split a single-queue IRQ.
- **dwc3 IMOD** (`install_usb0_imod_overlay.sh`) → null. IMOD only moderates the
  *hardware IRQ* rate, but the cost is softirq + per-skb wakeups + scheduler/load-
  balancer churn (`finish_task_switch`, `sched_balance_*`), which IMOD can't reduce.

**Remaining kernel levers:** NAPI/GRO batching in `u_ether` (the only thing that cuts
per-skb churn — but it's an excluded kernel-module change); reduce data volume; or
different hardware (a SuperSpeed-capable gadget port, unavailable here).

> `install_usb0_imod_overlay.sh` is kept for completeness. Default `--interval 2000`
> (= 500 µs, caps dwc3 IRQs ~2000/s). Whether IMOD engages in gadget mode depends on
> the dwc3 IP revision — verify via `/proc/interrupts` before/after. On this board it
> did **not** reduce the pegging (consistent with Layer B not being hardirq-bound).

## 3. DisplayLink decode is a hardware wall on RK3399 (A72)

Decompression is **memory-stall-bound**: A72 IPC ≈ 0.84, ~28M L2 misses/s. The comp16
`DecompTable16Colors` working set = 334×256×16 B = **1.37 MB > the 1 MB shared A72 L2**,
so every lookup misses to DRAM. No software layout fixes a pointer-chasing dependency
chain whose working set exceeds cache.

- **Table-merge optimization tried and reverted:** interleaving decompTable + colors
  into a stride-9 array halved L2 misses per lookup but added a non-power-of-2 multiply
  per iteration *and* broke the comp8 NEON path (single `Vector64<byte>` load became 8
  scalar reads). Net **50% decode regression**. Reverted.
- With N displays, N×1.37 MB competes for the same 1 MB L2 — contention scales with
  display count.
- **Productivity only by design.** Idle: `fill16=99%`, feed=5% (trivially cheap).
  Video/gaming saturates the decompressor — unsupported, not a fixable bug.

## 4. DisplayLink 30 Hz is NOT offerable on Windows (EDID can't fix it)

Goal was a 30 Hz mode to cut bandwidth. **The DisplayLink Windows driver synthesizes
its own mode list and has a ~50 Hz floor baked in** — it offers 50 Hz, a rate we
advertise nowhere (our DTDs are 60+30, CTA VICs 16=60 / 34=30, never 50). EDID/CTA
tweaks cannot move it.

- A CEA-861 extension block *was* added anyway (`build_default_edid` in `udl_device.c`,
  256-byte EDID with a Video Data Block of CE VICs) — clean per `edid-decode`, helps
  real sinks / Linux vhci hosts, **inert against the DL Windows driver**.
- Dropping to 50 Hz did **not** reduce the CPU pegging anyway (load is per-USB-
  transaction, not per-frame). Refresh reduction is a dead end for the CPU problem.

If 30 Hz is ever truly needed, the path is reverse-engineering the DL driver's mode
table / vendor-descriptor gating — **not** EDID.

## 5. Armbian footgun — installing linux-headers can wipe `/lib/modules`

`apt install linux-headers-current-rockchip64` can **drag a kernel-image upgrade** along
(the meta-packages move together). On 2026-05-30 this jumped
26.2.0-trunk.821 → 26.8.0-trunk.61 and left
`/lib/modules/6.18.33-current-rockchip64/kernel/` **empty** — WiFi driver gone (wlan0
boot timeout), `libcomposite`/`usb_f_rndis` unloadable, out-of-tree `.ko` fails with
"Unknown symbol … err -2" (the `.ko` was *deleted*, not a TRIM_UNUSED_KSYMS issue).
Wired NIC driver is a module too, so Ethernet may also die.

**Diagnose:** `find /lib/modules/$(uname -r)` shows no `kernel/` subdir;
`grep linux-image /var/log/apt/history.log` shows the piggybacked upgrade.

**Recover (offline-capable):** the kernel image `.deb` ships BOTH `/boot/Image-<ver>`
and `/lib/modules/<family>/kernel/…`. `dpkg -i <cached .deb>` (from
`/var/cache/apt/archives` or `/root`, no network needed) restores everything — but it
**must match the running family** (`6.18.33` / `26.8.0-trunk.61`); a different version
installs a parallel `/lib/modules/<other>/` tree and won't repair the running one.
Verify before install: `dpkg-deb -c file.deb | grep kernel/.*wireless`.

---

## Deploy / verify cheat-sheet

```sh
# One-time OTG enablement (boot-persistent overlay), then reboot:
sudo ./install_usb0_peripheral_overlay.sh
sudo reboot

# If the gadget goes into the DCP-misclassified stuck state:
sudo ./reset_usb0_otg_stack.sh

# System setup + services:
cd ~/breezy-box && git pull && make
sudo ./setup_system.sh

# Decode profiling when investigating CPU:
BREEZY_UDL_DECODE_STATS=1 ./displaylink_kms_renderer   # comp16/comp8 split, sync%/decode%
```

## Quick reference

| Thing | Value |
|---|---|
| SoC / GPU | RK3399 (2×A72 + 4×A53) / Mali-T860 |
| OTG controller | dwc3 `fe800000.usb`, High-Speed only (usb2phy) |
| OTG IRQ | 85, pinned CPU0, ~2.5–5.5k/s under load |
| Gadget-mode overlay | `rk3399-rock-4c-plus-usb0-peripheral` |
| IMOD overlay (null result) | `rk3399-rock-4c-plus-usb0-imod`, `--interval 2000` |
| A72 L2 / comp16 working set | 1 MB shared / 1.37 MB → always misses |
| Sync interval env | `BREEZY_UDL_SYNC_INTERVAL_MS` (default 12) |
| Decode stats env | `BREEZY_UDL_DECODE_STATS=1` |
