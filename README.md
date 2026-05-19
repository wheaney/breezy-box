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

Host-side helper dependencies for the GNOME virtual-monitor sender:

```sh
sudo apt install python3-pydbus python3-gi gir1.2-gstreamer-1.0 \
	gstreamer1.0-pipewire gstreamer1.0-plugins-base \
	gstreamer1.0-plugins-good gstreamer1.0-plugins-bad
```

Sink-side helper dependencies for the WFD-MICE bridge that targets Windows and GNOME Network Displays:

```sh
sudo apt install python3-gi gir1.2-gstreamer-1.0 avahi-utils \
	gstreamer1.0-tools gstreamer1.0-plugins-base \
	gstreamer1.0-plugins-good gstreamer1.0-plugins-bad
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

Important protocol note:

`CDC-NCM + IP` by itself still is not enough. The USB Ethernet link only provides transport. The SBC also needs to implement an actual receiver protocol that the host knows how to discover and speak.

The nuance is:

* GNOME Network Displays now has multiple discovery/provider paths. Its source tree includes Wi-Fi Direct Miracast, Chromecast discovery via mDNS on `_googlecast._tcp`, and WFD-MICE discovery via mDNS/DNS-SD on `_display._tcp`.
* Windows supports Miracast over Infrastructure (MICE), which is the relevant Miracast-over-IP path for a CDC-NCM link.
* macOS AirPlay is still a separate receiver protocol family.

So a CDC-NCM link can be auto-discoverable to GNOME Network Displays and Windows if the SBC implements a real WFD-MICE sink over the USB Ethernet network. The repo now includes a first WFD-MICE bridge in `wfd_mice_sink.py`.

That helper does all of the following:

* advertise `_display._tcp` on the USB network with the expected TXT metadata such as `p2pMAC`,
* listen for MICE signalling on TCP port `7250`,
* parse the sender's `SOURCE_READY` message,
* connect back to the sender's WFD RTSP server on port `7236`,
* relay the WFD H.264 stream to a fixed local RTP port that `breezy_drm_scene_demo` already knows how to decode.

This is intentionally a bridge into the current renderer rather than a full native WFD stack inside `breezy_drm_scene_demo.c`. The present limitations are:

* the sink side currently handles the `SOURCE_READY` bootstrap and normal RTSP playback path, not the full WFD control surface,
* the bridge currently relays video only and ignores WFD audio,
* the standards-based path is H.264-oriented because WFD sources expose H.264 over MPEG-TS/RTSP.

That means the auto-discoverable Windows/GNOME path now exists in this repo, but the older custom H.265/VP9 RTP path is still separate and still relevant when you control both ends of the link. There is still no single current `CDC-NCM + Miracast/AirPlay` endpoint in this repo that GNOME, Windows, and macOS will all discover almost out of the box.

That means the practical receiver model is:

1. Bring up one transport interface to the board, such as USB OTG networking with `setup_usb_network_gadget.sh`.
2. Give the SBC an IP on that interface.
3. Start either the manual RTP renderer path or the new `wfd_mice_sink.py` bridge.
4. For WFD-MICE, let GNOME Network Displays or Windows discover the sink and initiate the stream.
5. For custom senders, keep using the direct RTP pipelines described below.

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

After either side reboots or the USB gadget resets, make sure you re-run the host-side link and address setup too. GNOME Network Displays discovery depends on both ends of the USB Ethernet link being back up on the same subnet.

If you want separate per-stream bind addresses on the SBC, add aliases such as:

```sh
sudo ip addr add 192.168.2.3/24 dev usb0
sudo ip addr add 192.168.2.4/24 dev usb0
```

The older `network_display_receiver_supervisor.py` helper is still useful if you want those addresses managed for out-of-process receiver daemons, but it is not required for the in-process GStreamer renderer path.

### 2. Start The WFD-MICE Bridge For Windows And GNOME

The simplest Windows/GNOME bring-up path on the SBC is:

```sh
python3 ./wfd_mice_sink.py \
	--interface usb0 \
	--launch-renderer \
	--renderer-device /dev/dri/card0 \
	--renderer-decoder-fragment mppvideodec
```

That command does three things:

* publishes a discoverable `_display._tcp` sink with `p2pMAC=<usb0-mac>`,
* listens for MICE `SOURCE_READY` on TCP `7250`,
* auto-launches `breezy_drm_scene_demo` with a static local H.264 RTP relay pipeline on `127.0.0.1:5600`.

If you prefer to launch the renderer yourself, omit `--launch-renderer`. The helper prints the exact renderer command it expects.

The discoverable network-display endpoint is `wfd_mice_sink.py` itself. Running `breezy_drm_scene_demo` directly does not advertise `_display._tcp`, does not listen for MICE `SOURCE_READY`, and will never show up in GNOME Network Displays or Windows by itself.

For the WFD-MICE path, do not substitute the manual H.265 examples from the next section. The bridge relays WFD into a local H.264 RTP stream for the renderer, so if you launch the renderer yourself, use the exact command printed by `wfd_mice_sink.py`.

Once that helper is running:

* GNOME Network Displays should discover the sink on the USB network and use the WFD-MICE path.
* Windows should discover it through the Miracast over Infrastructure path on the same link.

Host firewall note:

* GNOME Network Displays' upstream firewalld integration explicitly expects the source-side RTSP server to be reachable on TCP port `7236`.
* If the host has UFW, firewalld, or another inbound firewall policy enabled, allow TCP `7236` from the SBC to the host or the WFD bridge will fail before the RTSP session starts.

The relay bridge converts the WFD RTSP session into a local H.264 RTP stream for the renderer. Because of that, the decoder fragment for the standards-based path should be an H.264 decoder such as `mppvideodec` or a stateless V4L2 H.264 decoder.

Internally, the helper now drives the source with a minimal RTSP `OPTIONS` / `DESCRIBE` / `SETUP` / `PLAY` exchange and receives the WFD media on fixed local RTP/RTCP ports (default `16384/16385`) before repayloading the H.264 elementary stream to the renderer's local relay port.

If you need GNOME virtual-display extension semantics rather than GNOME Network Displays discovery, keep using `wired_projection_gnome_sender.py`. GNOME Network Displays streams an existing selected monitor; it does not replace Mutter's compositor-level virtual-display path.

### 3. Choose A Decoder Path That Produces DMABuf NV12

Your pipeline fragment must stop at the decoder stage. The renderer appends its own:

```sh
! queue max-size-buffers=2 leaky=downstream ! video/x-raw(memory:DMABuf),format=NV12 ! appsink ...
```

So do not add your own sink when using `--stream-pipeline`.

On Rockchip, the exact decoder element name depends on the image and installed plugins. Check the target with:

```sh
gst-inspect-1.0 | grep -E 'mpp|v4l2.*(264|265|vp9)|h264dec|h265dec|vp9dec'
```

Typical candidates are `mppvideodec`, `v4l2slh264dec`, `v4l2slh265dec`, or `v4l2slvp9dec`.

### 4. Start The Receiver Renderer

This section is for manual/custom RTP senders. It is not the entrypoint for GNOME Network Displays or Windows discovery.

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

## Wired GNOME Projection

If your host is GNOME on Wayland, the repo now includes a helper that creates a GNOME virtual monitor and sends that monitor over the CDC-NCM link.

Start the SBC receiver first:

```sh
sudo ./breezy_drm_scene_demo \
	--device /dev/dri/card0 \
	--verbose \
	--stream-pipeline 'udpsrc port=5600 caps="application/x-rtp,media=video,encoding-name=H265,payload=96,clock-rate=90000" ! rtph265depay ! h265parse ! mppvideodec'
```

Then on the GNOME host:

```sh
python3 ./wired_projection_gnome_sender.py \
	--sink-host 192.168.2.2 \
	--sink-port 5600 \
	--width 1920 \
	--height 1080 \
	--framerate 60 \
	--codec h265
```

The helper asks Mutter to create a virtual display. GNOME should then expose that display in Settings > Displays, where you can arrange it as an extended monitor. Once Mutter exposes the PipeWire stream for that virtual display, the helper encodes it and sends it over RTP to the SBC.

Current scope of this wired path:

* GNOME/Wayland host support only.
* The host-side auto-discovery UI is not GNOME Network Displays; the desktop-recognized monitor comes from Mutter's virtual-display API.
* The repo now also has a separate WFD-MICE bridge for GNOME Network Displays and Windows auto-discovery over the CDC-NCM link.
* macOS AirPlay discovery would still need its own separate sink/source integration; it is not interchangeable with the wired GNOME path or with WFD-MICE.

### 5. Send A Test Stream From The Host

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