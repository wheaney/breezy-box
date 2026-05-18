# Breezy Box

Breezy Box is an XR dock prototype for a single-board computer. The active direction in this repo is back to a native DRM/KMS renderer that can present multiple network display streams as textures inside one 3D scene.

The current code now provides a compileable `kmscube`-style render scaffold. It does not ingest real network video yet; it renders synthetic per-stream textures so the display, scene, and scanout path can be developed in-process first.

## Target Architecture

The intended shape is:

1. Cable and transport layer: expose one USB OTG network function or another host-to-SBC link.
2. Stream ingestion layer: receive multiple network display streams, either from external receiver daemons or from in-process protocol handlers.
3. Render layer: one DRM/KMS app owns the GBM/EGL/GLES render loop, uploads each stream into a texture, and places those textures onto panels in a 3D scene.
4. XR layer: the same process eventually applies head pose, panel transforms, focus rules, and compositor logic before scanout.

## Current Renderer Anchor

`kmscube_stream_scene_demo.c` is the current render anchor.

It is intentionally shaped like the part of a `kmscube` fork you would want to keep:

* DRM connector and CRTC selection.
* GBM device and scanout surface creation.
* EGL context creation on top of GBM.
* A GLES draw loop that renders multiple textured panels in 3D.
* One scanout path that owns the final fullscreen present.

Right now the per-stream textures are synthetic and CPU-generated. That makes this file a safe place to stabilize the renderer contract before wiring in real receiver output.

## Kmscube Fork Fit

If you plan to use a `kmscube` fork, that is the right base layer for this repo.

The practical split should be:

* Let the fork keep owning DRM device setup, GBM/EGL initialization, swap, and scanout.
* Replace `stream_surface_update_pixels()` with a real frame source fed by network receivers.
* Keep `build_panel_mvp()` or an equivalent scene-graph layer as the Breezy Box-specific placement logic.
* Keep the final renderer single-process so multiple stream textures can be composed into one XR scene without crossing process boundaries.

The current demo mirrors that split so the fork integration can stay local instead of rewriting the whole repo around the fork later.

## Build

Build dependencies:

```sh
sudo apt install build-essential libdrm-dev libgbm-dev libegl1-mesa-dev libgles2-mesa-dev
```

Build the current scene demo:

```sh
make
```

Or build just the native renderer target:

```sh
make kmscube_stream_scene_demo
```

## Run

The demo must run on a Linux console where it can become DRM master on a real KMS device.

Example:

```sh
sudo ./kmscube_stream_scene_demo --stream-count 3 --device /dev/dri/card0
```

Useful options:

* `--stream-count <n>` chooses how many textured panels to draw.
* `--stream-width <pixels>` sets the per-stream texture width.
* `--stream-height <pixels>` sets the per-stream texture height.
* `--frames <n>` renders a bounded number of frames and exits.
* `--verbose` prints the selected DRM mode and GL renderer details.

Example short run for bring-up:

```sh
sudo ./kmscube_stream_scene_demo --stream-count 4 --frames 300 --verbose
```

## Network Ingress Helpers

The USB/network scripts are still useful, but they are now auxiliary to the renderer rather than the main product:

* `setup_usb_network_gadget.sh` creates one USB network gadget via configfs.
* `network_display_receiver_supervisor.py` can still supervise out-of-process receiver daemons during a transition period.
* `network_display_receivers.example.json` remains an example control-plane config.

The existing OTG helpers are also still relevant when the SBC transport path is USB-based:

* `install_usb0_peripheral_overlay.sh`
* `reset_usb0_otg_stack.sh`
* `watch_usb_gadget_attach.sh`
* `capture_usb_gadget_state.sh`

## Development Notes

Useful targets:

```sh
make
make validate
make dry-run-example
```

`make validate` builds the native renderer and runs a Python syntax check on the transitional supervisor. `make dry-run-example` expands the example network-receiver config without touching the live network stack.

## Next Integration Step

The next meaningful renderer change is to replace the synthetic texture generator with a real stream upload path.

The simplest first cut is:

1. keep the receiver process out-of-process,
2. hand frames to the renderer through shared memory or a ring buffer,
3. upload them with `glTexSubImage2D`,
4. only move to dma-buf or zero-copy once scene composition is stable.

That keeps the risk in the right order: first prove multi-stream composition in one DRM/KMS app, then optimize ingestion.