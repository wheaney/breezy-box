# Supported hardware — board-specific notes

Per-board "definitive notes" so future changes can be made with minimal regressions.
Each file documents the quirks, gotchas, and load-bearing constraints that bit us
during bring-up — read the relevant board's file before touching anything that
interacts with DRM/KMS, the OTG gadget stack, the network stack, or systemd units.

| Board | SoC / GPU | Notes |
|---|---|---|
| Radxa Cubie A7S | Allwinner A733 / Mali | [radxa-cubie-a7s.md](radxa-cubie-a7s.md) — current daily board |
| Radxa Rock Pi 4C+ | Rockchip RK3399 / Mali-T860 | [rock-4c-plus.md](rock-4c-plus.md) — original bring-up board |

## Cross-board constants (true everywhere)

- **DisplayLink decode is a memory-stall hardware wall on ARM** — productivity content
  only; video/gaming saturates the decompressor by design. Working set exceeds shared
  L2 on every board tested; contention scales with display count. (Details per board.)
- **The OTG gadget port is the wired-to-host path** (USB RNDIS `usb0`, `192.168.7.x`).
  A native Ethernet port, if used, is a *separate* direct link on `192.168.77.x`.
- **The renderer is a system service on `tty1`/seat0** so it can take DRM master. Any
  console daemon (getty, kmscon, …) on that VT will steal the seat/master — setup masks
  them generically (scoped to the renderer's VT).
- **Web UI on `:8443` (HTTPS), noVNC on `:8080`, USB/IP on `:3240`.** A `:443→:8443`
  redirect needs nftables/iptables present.

## Adding a new board

When bringing up a new SBC, create `docs/hardware/<board>.md` and capture, at minimum:
1. SoC/GPU, OS image, **package manager**.
2. OTG controller name + whether it's HS-only or SuperSpeed-capable.
3. Which **network stack** is active (NetworkManager vs systemd-networkd) — this changes
   how the wired link and port redirects must be configured.
4. Whether a **console daemon** grabs DRM master on the renderer's VT (`fuser -v
   /dev/dri/card*`).
5. Whether the DRM driver supports `DRM_CAP_ASYNC_PAGE_FLIP` (vsync-off path).
6. Any DT overlays / kernel quirks needed to get the gadget port working.
