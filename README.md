# Breezy Box

Breezy Box is an XR dock prototype for a single-board computer. The active direction in this repo is a native DRM/KMS renderer that can present multiple network display streams as textures inside one 3D scene.

The current renderer can now ingest real network video streams in-process through repeated GStreamer pipeline inputs. When the platform exposes the needed EGL dma-buf import extensions, decoded NV12 dmabufs are imported directly into GL textures; streams that have not produced real frames yet still fall back to synthetic test panels.

## Target Architecture

The intended shape is:

1. Cable and transport layer: expose one USB OTG network function or another host-to-SBC link.
2. Stream ingestion layer: receive multiple network display streams, either from external receiver daemons or from in-process protocol handlers.
3. Render layer: one DRM/KMS app owns the GBM/EGL/GLES render loop, uploads each stream into a texture, and places those textures onto panels in a 3D scene.
4. XR layer: the same process eventually applies head pose, panel transforms, focus rules, and compositor logic before scanout.

## Current Renderer Anchor

`breezy_drm_scene_demo.c` is the current render anchor.

It owns the renderer directly instead of wrapping a third-party demo app:

* DRM connector and CRTC selection.
* GBM device and scanout surface creation.
* EGL context creation on top of GBM.
* A GLES draw loop that renders multiple textured panels in 3D.
* One scanout path that owns the final fullscreen present.

The current per-stream texture paths are:

* synthetic CPU-generated test panels,
* repeated `--stream-pipeline` GStreamer inputs that are expected to decode to `video/x-raw(memory:DMABuf),format=NV12`.

That makes this file the integration point for both scene composition and network video ingest.

## Reference Submodule

The upstream `kmscube` source under `modules/kmscube` is reference material only. It is not part of the Breezy Box build and is not invoked by the top-level `Makefile`.

The most relevant files for the Breezy Box renderer are:

* `modules/kmscube/kmscube.c`: the main app entry that wires DRM backend, GBM/EGL init, and the selected render mode together.
* `modules/kmscube/common.h`: the core extension boundary, especially `struct cube`, `struct gbm`, and `struct egl`.
* `modules/kmscube/drm-common.c`, `drm-legacy.c`, and `drm-atomic.c`: the scanout and present backends.
* `modules/kmscube/cube-video.c`: the closest existing example of a texture-fed render path inside the kmscube abstraction.
* `modules/kmscube/gst-decoder.c`: the current GStreamer-to-GL texture bridge used by `cube-video.c`.

That reference is there to inform the next steps in the self-owned renderer, not to become a required dependency.

## Build

Build dependencies:

```sh
sudo apt install build-essential \
	libdrm-dev libgbm-dev libegl1-mesa-dev libgles2-mesa-dev \
	libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
	gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good
```

Build the current scene demo:

```sh
make
```

Or build just the native renderer target:

```sh
make breezy_drm_scene_demo
```

## Run

The demo must run on a Linux console where it can become DRM master on a real KMS device.

Synthetic panel example:

```sh
sudo ./breezy_drm_scene_demo --stream-count 3 --device /dev/dri/card0
```

Useful options:

* `--stream-count <n>` chooses how many textured panels to draw.
* `--stream-width <pixels>` sets the per-stream texture width.
* `--stream-height <pixels>` sets the per-stream texture height.
* `--stream-pipeline <gst>` adds one network/video ingest pipeline. If `--stream-count` is omitted, the stream count defaults to the number of configured pipelines.
* `--frames <n>` renders a bounded number of frames and exits.
* `--verbose` prints the selected DRM mode and GL renderer details.

Example short run for bring-up:

```sh
sudo ./breezy_drm_scene_demo --stream-count 4 --frames 300 --verbose
```

## Working As A Network Endpoint

The current renderer is not a custom socket server on its own. The network endpoint is the GStreamer source element you supply via `--stream-pipeline`.

That means the practical receiver model is:

1. Bring up one transport interface to the board, such as USB OTG networking with `setup_usb_network_gadget.sh`.
2. Give the SBC an IP on that interface.
3. Start `breezy_drm_scene_demo` with one `--stream-pipeline` per incoming stream.
4. Have the sender push an encoded stream to the address/port that each pipeline is listening on.

For the current in-process renderer, one interface plus one port per stream is the simplest shape. Multiple IP aliases are still possible, but they are optional unless you specifically need per-stream bind addresses.

### 1. Bring Up The USB Network Link

On the SBC:

```sh
sudo ./setup_usb_network_gadget.sh
sudo ip link set usb0 up
sudo ip addr add 192.168.2.2/24 dev usb0
```

On the host, assign the other side of the point-to-point link:

```sh
sudo ip link set <host-usb-iface> up
sudo ip addr add 192.168.2.1/24 dev <host-usb-iface>
```

If you want separate per-stream bind addresses on the SBC, add aliases such as:

```sh
sudo ip addr add 192.168.2.3/24 dev usb0
sudo ip addr add 192.168.2.4/24 dev usb0
```

The older `network_display_receiver_supervisor.py` helper is still useful if you want those addresses managed for out-of-process receiver daemons, but it is not required for the in-process GStreamer renderer path.

### 2. Choose A Decoder Path That Produces DMABuf NV12

Your pipeline fragment must stop at the decoder stage. The renderer appends its own:

```sh
! queue max-size-buffers=2 leaky=downstream ! video/x-raw(memory:DMABuf),format=NV12 ! appsink ...
```

So do not add your own sink when using `--stream-pipeline`.

On Rockchip, the exact decoder element name depends on the image and installed plugins. Check the target with:

```sh
gst-inspect-1.0 | grep -E 'mpp|v4l2.*(265|vp9)|h265dec|vp9dec'
```

Typical candidates are `mppvideodec`, `v4l2slh265dec`, or `v4l2slvp9dec`.

### 3. Start The Receiver Renderer

Example H.265 RTP receiver on the SBC:

```sh
sudo ./breezy_drm_scene_demo \
	--device /dev/dri/card0 \
	--verbose \
	--stream-pipeline 'udpsrc port=5600 caps="application/x-rtp,media=video,encoding-name=H265,payload=96,clock-rate=90000" ! rtph265depay ! h265parse ! mppvideodec'
```

Two-stream example:

```sh
sudo ./breezy_drm_scene_demo \
	--device /dev/dri/card0 \
	--verbose \
	--stream-pipeline 'udpsrc port=5600 caps="application/x-rtp,media=video,encoding-name=H265,payload=96,clock-rate=90000" ! rtph265depay ! h265parse ! mppvideodec' \
	--stream-pipeline 'udpsrc port=5601 caps="application/x-rtp,media=video,encoding-name=H265,payload=96,clock-rate=90000" ! rtph265depay ! h265parse ! mppvideodec'
```

If your target uses stateless V4L2 decoders instead of MPP, replace `mppvideodec` with the appropriate decoder element reported by `gst-inspect-1.0`.

### 4. Send A Test Stream From The Host

Example host-side sender for stream 1:

```sh
gst-launch-1.0 -v \
	videotestsrc is-live=true pattern=smpte ! \
	video/x-raw,width=1280,height=720,framerate=60/1 ! \
	x265enc tune=zerolatency speed-preset=ultrafast bitrate=6000 key-int-max=60 ! \
	h265parse ! rtph265pay pt=96 config-interval=1 ! \
	udpsink host=192.168.2.2 port=5600
```

Example host-side sender for stream 2:

```sh
gst-launch-1.0 -v \
	videotestsrc is-live=true pattern=ball ! \
	video/x-raw,width=1280,height=720,framerate=60/1 ! \
	x265enc tune=zerolatency speed-preset=ultrafast bitrate=6000 key-int-max=60 ! \
	h265parse ! rtph265pay pt=96 config-interval=1 ! \
	udpsink host=192.168.2.2 port=5601
```

### 5. What To Check When It Does Not Work

* `--verbose` prints whether dmabuf import is available. If it reports `DMABUF import: unavailable`, this zero-copy path will not work on that EGL/GLES stack.
* If the app logs `stream pipeline must output NV12 video/x-raw(memory:DMABuf) buffers`, your decoder chain is not producing the required caps.
* If the app logs `stream pipeline did not produce DMABuf-backed NV12 planes`, the decoder is outputting system memory instead of dmabufs.
* The app still needs DRM master on a real KMS device, so run it from a Linux console with access to `/dev/dri/card*`.

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

The next renderer improvements are now around making the network ingest path more product-like rather than introducing the first video path at all.

The most likely follow-on work is:

1. ship built-in pipeline presets instead of raw GStreamer fragments,
2. support per-stream transport configuration in a real config file,
3. handle dma-buf modifiers and more decoder output variants,
4. move from demo panel placement to real XR scene logic.