# Breezy Box

A Breezy Box is an "XR dock" -- a no-software, no-setup device for multi-display, XR productivity on any PC, any OS, and any USB port using just a single-board-computer.

## Hardware Requirements

To spoof a dock that can attach multiple moniters over standard USB, we'll be using the "old" DisplayLink protocol (the newer one is encrypted and hasn't been reverse engineered). This requires USB OTG hardware so our device can act as a USB peripheral. Many SBCs come with the appropriate hardware, but devices that share the OTG port with power delivery like the Raspberry Pi seem to fight against attempts to use the port OTG purposes. So at the moment, the hardware requirements are:
1. Can run Linux (the necessary chipset drivers are available), ideally something like Armbian 
2. At least one USB port that supports OTG (doesn't matter if USB-A or USB-C)
3. Power delivery via a port separate from OTG
4. USB-C with DisplayPort Alt Mode support

### Candidate SBCs

* Libre Computer Renegade Elite ROC-RK3399-PC **maybe best overall package including price**
* FriendlyElec NanoPC-T6
* Radxa Rock 5B **requires extra work to separate power deliver from the dp-alt port**
* Orange Pi 5 Plus

## Software components

Breezy Box devices will run the bare minimum Linux kernel for the hardware, without a typical desktop environment. Instead, a port of [Breezy Desktop](https://github.com/wheaney/breezy-desktop) will run as a DRM Master and be responsible for the 3D rendering of the desktop captures provided over DisplayLink, in combination with head tracking data provided by the [XR driver](https://github.com/wheaney/XRLinuxDriver).

For the rendering and UI, nearly every component has been written multiple times over with the GTK, KCM, and decky applications and the GNOME and KWin extensions. We'll be reusing heavily from these apps to try to keep the amount of new stuff to an extreme minimum.

### Breezy Desktop as a Standalone DRM/KMS application

We'll want to control the full rendering stack -- and not have to deal with rendering on the terms of a desktop environment like GNOME or KDE, or even a similar application like gamescope -- to get the absolute minimum rendering latency and a minimal footprint. A custom version of Breezy Desktop can be built as a DRM Master application that Linux will be configured to boot directly into instead of a standard desktop environment.

#### JavaScript reuse

Both the GNOME and KWin extensions use JavaScript syntax for a lot of the logic to determine display placement, display focus, and even vertex placement for creating curved and flat displays using a mesh. This new application will be in C and can use the QuickJS library to reuse the JavaScipt functions that are already written. We'll need to do some work to pull that logic out into shared functions (right now the logic is duplicated but has been kept mostly in sync).

Ideally the application will rely on gsettings to allow us to reuse the schema and allow us to prevent a rewrite of the UI (see the next section).

### UI

Breezy Desktop relies heavily on a UI to allow the user to configure and interact with the XR Effect in real time. Ideally the Breezy Box will be a no-software solution, but still provide a powerful UI. One way to achieve this is to host the application remotely, so it runs directly on the Breezy Box, but is rendered on the host system.

#### GDK Broadway

In researching if there's a way to host the UI remotely, I came across GDK Broadway: 

> The GDK Broadway backend provides support for displaying GTK applications in a web browser, using HTML5 and web sockets.

It claims to not be well maintained, and I haven't tested it yet, so I don't know if it works with Adwaita or if it'll run into other problems with the current version of the Python GTK UI. So it's probably a long-shot, but if it does work, it seems like the ideal approach: users get the full-blown UI and they can access it using their browser without any installation needed. We would just need to expose the GDK Broadway web server over USB.

Some changes would be needed to allow the UI to switch how it creates virtual displays, based on whether it's running on GNOME or on a Breezy Box. Keyboard shortcuts are the biggest unknown, as those are OS-specific and may be difficult to use to trigger changes over USB. At the very least we may be able to provide solutions so people can create DIY shortcuts.

### DisplayLink spoofing

We need a new component that will use a libcomposite integration to create the desired displays. It would always spoof at least one DisplayLink display by default, but would be able to create and remove displays in real-time based on UI interactions, and also run at startup to create the configured number of displays when connected.

This component would also be responsible for decoding the damage data coming in over the DisplayLink protocol and updating a DRM buffer with the latest textures for use in the 3D rendering. Although I've listed this as its own component, this would most likely live as a separate thread in the DRM/KMS rendering application, in order to have simpler access to shared memory for rendering.

Current prototype status:
The active path is now the Raw Gadget implementation in `displaylink_gadget_raw_gadget.c`. It owns ep0 completely, so it can answer the DisplayLink-specific vendor descriptor (`0x5f`), channel-select, and EDID control requests directly from userspace and log the subsequent control traffic. It also drains bulk OUT traffic after `SET_CONFIGURATION` and now feeds that stream into the `modules/udl_sink` decoder so the latest frame can be reconstructed in userspace.

Build notes:

```sh
sudo apt install build-essential libsdl2-dev
git submodule update --init --recursive
make
```

The resulting `displaylink_gadget_raw_gadget` binary is the only maintained prototype now. It auto-detects the first UDC when possible; otherwise pass both the UDC driver and device names explicitly. Example:

Before running it, make sure the Raw Gadget misc device exists. On modular kernels that usually means loading the module first:

```sh
sudo modprobe raw_gadget
ls -l /dev/raw-gadget
```

If `/dev/raw-gadget` still does not exist after that, the current kernel likely does not expose `CONFIG_USB_RAW_GADGET`.

If `/sys/class/udc` exists but is empty, this is not an auto-detect naming problem. It means the running image is not exposing any gadget-capable UDC at all for the selected port, so passing `--udc-device` and `--udc-driver` will not help until the kernel/DT side is fixed. On the Rock Pi 4C+ OTG path, the shortest checks are:

```sh
ls -la /sys/class/udc
lsmod | grep -E 'dwc|udc|gadget|raw_gadget'
sudo ./inspect_usb_gadget_dt.sh
```

On a fresh image, an empty `/sys/class/udc` usually means the OTG controller is not configured for peripheral mode, the relevant gadget/UDC drivers are missing, or the board kernel simply does not expose a UDC for that port.

On the current Rock Pi 4C+ Armbian image, the stock board DTB configures `fe800000` as host-only. If you want the maintained Raw Gadget path to survive reboots or fresh images, install the matching boot overlay once on the SBC:

```sh
sudo ./install_usb0_peripheral_overlay.sh
sudo reboot
```

That installer writes `/boot/overlay-user/rk3399-rock-4c-plus-usb0-peripheral.dtbo` and ensures `/boot/armbianEnv.txt` includes the matching `user_overlays` entry.

After reboot, the shortest sanity checks are:

```sh
tr '\0' '\n' < /proc/device-tree/usb@fe800000/usb@fe800000/dr_mode
ls -la /sys/class/udc
```

The expected good state is `dr_mode=peripheral` and a populated `/sys/class/udc/fe800000.usb`.

When the Rock Pi falls into the persistent bad OTG state, restarting only the raw-gadget userspace process is not enough. The shorter discriminator is:

- good: `current_speed=high-speed` and the OTG extcon behind `usb2phy@e450` reports `CDP=1`
- bad: `current_speed=UNKNOWN` and the same OTG extcon reports `DCP=1`

To attempt a deeper in-place recovery, unbind and rebind the `fe800000.usb` `dwc3` device together with its `usb2phy@e450` OTG PHY provider:

```sh
sudo ./reset_usb0_otg_stack.sh
```

The helper auto-detects the `dwc3` platform device behind the selected UDC, finds the matching `otg-port` PHY consumer under `/sys/class/phy`, unbinds both drivers in dependency order, rebinds them, then prints the resulting UDC and extcon state.

If you only want to inspect what it would touch first:

```sh
sudo ./reset_usb0_otg_stack.sh --dry-run
```

If that recovery succeeds, the post-reset state should move away from `DCP=1`, the UDC should stop reporting `current_speed=UNKNOWN`, and the watcher should eventually show a fresh attach when the host retries enumeration.

```sh
sudo ./displaylink_gadget_raw_gadget --udc-device fe800000.usb --verbose
```

The raw-gadget binary now accepts `--decode-width` and `--decode-height` to size the sink storage, `--no-decode` to fall back to pure bulk draining while debugging the USB path, `--capture-stream /tmp/deck.udlcap` to record replayable bulk OUT traffic, `--dump-image /tmp/udl.ppm` to write the latest decoded frame as a binary PPM image on exit, `--show-window` to display the decoded framebuffer live in a basic SDL2 window, and `--usb-speed high|super` to select the advertised link speed. The default is back to high-speed because forcing super-speed changed host behavior enough to break the current viewer-oriented bring-up path.

Example with a live viewer window and an exit snapshot:

```sh
sudo ./displaylink_gadget_raw_gadget --udc-device fe800000.usb --show-window --dump-image /tmp/udl.ppm --verbose
```

If you want to retry the newer USB 3 descriptor path explicitly, add `--usb-speed super`.

For low-perturbation capture on a machine like the Steam Deck, the shortest path is to turn decode off and dump the real bulk OUT stream directly:

```sh
sudo ./displaylink_gadget_raw_gadget --udc-device fe800000.usb --no-decode --capture-stream /tmp/deck-wayland.udlcap
```

That capture format starts with the 8-byte magic `UDLCAP01`, then a 32-bit version and 32-bit reserved field. Each packet record then stores a 64-bit monotonic timestamp in nanoseconds, a 32-bit payload length, a 32-bit reserved field, and the exact bulk packet payload. This preserves host packet boundaries so the stream can be replayed later into the transport layer without the Deck attached.

If you want to test the host against a real monitor identity instead of the built-in synthetic EDID, pass a 128-byte base EDID blob directly:

```sh
sudo ./displaylink_gadget_raw_gadget --udc-device fe800000.usb --edid-file /path/to/monitor.edid --verbose
```

If auto-detection does not find the correct driver name, rerun with both values, for example:

```sh
sudo ./displaylink_gadget_raw_gadget --udc-driver fe800000.usb --udc-device fe800000.usb --verbose
```

### In-process multi-session demo

There is now also a small in-process demo binary at `displaylink_multi_session_demo`. This is the shortest path to exercise the new reusable session API and output-ring contract without bringing in the full DRM/KMS renderer yet.

Each session is started inside the same process with its own `--session` block, and the demo composites the latest published frames side-by-side into one canvas. By default it runs headless and can dump a final composite image on exit; add `--show-window` to preview the composed output in a single SDL2 window.

The hardware constraint is unchanged: this demo still needs one distinct, existing UDC per live raw-gadget session. If `/sys/class/udc` only shows one controller, the board can only host one live DisplayLink raw-gadget session at a time. The demo now preflights that and fails early when a requested `--udc-device` does not exist or when two sessions name the same UDC.

Example with one shared viewer window:

```sh
sudo ./displaylink_multi_session_demo --show-window \
	--session --udc-device fe800000.usb --udc-driver fe800000.usb --monitor-name Left \
	--session --udc-device fe900000.usb --udc-driver fe900000.usb --monitor-name Right
```

Example headless run that writes the composed result on exit:

```sh
sudo ./displaylink_multi_session_demo --dump-image /tmp/composite.ppm \
	--session --udc-device fe800000.usb --udc-driver fe800000.usb --monitor-name Left \
	--session --udc-device fe900000.usb --udc-driver fe900000.usb --monitor-name Right
```

If your board only exposes one UDC such as `fe800000.usb`, use a single live session here. A two-session live demo requires two entries under `/sys/class/udc`.

The current demo is intentionally small: it uses the new in-process `displaylink_session` object model, renderer-owned ring buffers, and a simple horizontal layout. It is a test seam, not the final DRM/KMS presenter.

### Multi-display supervisor prototype

For a quick multi-display prototype, there is now a small supervisor at `displaylink_gadget_supervisor.py`. It watches a JSON config file and launches one `displaylink_gadget_raw_gadget` child per enabled display entry.

This is still one UDC per display. The supervisor does not multiplex several DisplayLink adapters through a single UDC; each configured display must name its own `udc_device` and `udc_driver`.

Start by copying and editing `displaylink_multi_display.example.json`, then sanity-check the expanded child commands without touching hardware:

```sh
python3 ./displaylink_gadget_supervisor.py ./displaylink_multi_display.example.json --dry-run
```

To run it for real, launch the supervisor as root so the child raw-gadget processes inherit the required privileges:

```sh
sudo python3 ./displaylink_gadget_supervisor.py /path/to/displays.json
```

If you want to preview both decoded displays as separate windows during development, set `"show_window": true` on each display entry. The SDL window title now follows `monitor_name`, so two entries such as `Breezy Box 1` and `Breezy Box 2` show up as two distinct preview windows.

```json
{
	"name": "display-1",
	"udc_device": "replace-with-first-udc",
	"udc_driver": "replace-with-first-udc",
	"serial_string": "BREEZY0001",
	"monitor_name": "Breezy Box 1",
	"show_window": true
}
```

Those preview windows are a local-debug feature. They work best when you run the supervisor from a graphical desktop session that also has permission to access the Raw Gadget device. On a headless SBC, or when root cannot open windows on the current Wayland session, use `dump_image_path` or the real DRM/KMS renderer instead.

The supervisor reloads the config once per second. Adding, removing, enabling, disabling, or changing a display entry causes the matching raw-gadget child to be started, stopped, or restarted.

Each display entry can override the existing raw-gadget settings, including `decode_width`, `decode_height`, `usb_speed`, `edid_path`, `capture_stream_path`, `dump_image_path`, `show_window`, `serial_string`, `manufacturer_string`, `product_string`, and `monitor_name`.

The built-in EDID is still a fixed 1920x1080@60 personality unless `edid_path` is supplied. In this prototype, per-display `decode_width` and `decode_height` now also drive the vendor descriptor pixel limit, but fully user-defined resolutions and framerates still need per-display EDID blobs.

Longer term, the right shape is to move the supervisor into the same renderer process and replace the current process-per-display model with in-process display sessions. Each session would own its gadget control loop, bulk drain, decode transport, and render target, so decoded damage can be applied directly to shared render buffers instead of copied across process boundaries.

When the remaining issue is on the host side rather than in the gadget itself, there is also a small capture helper that dumps the relevant DRM and Mutter state into one directory. This is useful when a hotplug reproduces the `No available CRTC ... not found` path and you want one repeatable bundle instead of rerunning ad hoc commands:

```sh
./capture_host_display_state.sh --driver udl
```

That capture includes `modetest -M udl`, Mutter `DisplayConfig.GetResources`, `DisplayConfig.GetCurrentState`, a filtered user journal excerpt, `/dev/dri` listings, and basic DRM sysfs connector state.

When the missing piece is the root-only kernel side, there is also a companion helper that captures DRM debugfs state and filtered kernel logs into a second bundle:

```sh
sudo ./capture_root_display_state.sh --driver udl
```

That root capture includes `/sys/kernel/debug/dri/*/{state,summary,clients,framebuffer,name}` where available, filtered kernel journal output, `dmesg`, `modetest -M udl`, and matching DRM sysfs connector state.

When the failure is lower than DRM and the SBC-side UDC never gets past `not attached`, there is also a dedicated gadget/PHY capture helper:

```sh
sudo ./capture_usb_gadget_state.sh --udc fe800000.usb
```

That capture bundles `/sys/class/udc`, role-switch, Type-C, extcon, PHY, selected controller sysfs, and filtered USB/UDC kernel logs so persistent attach failures can be compared across boots and cable/power cycles.

It also snapshots the mutable boot/config surfaces that can explain a failure state that suddenly persists across reboots: `/boot/armbianEnv.txt`, boot script strings/checksums, the active DTB symlink and Rock Pi DTB checksum, `/boot/overlay-user`, configfs gadget and DT overlay state, selected `/proc/device-tree` nodes, USB-related local system config under `/etc` and `/usr/local`, and matching package/history excerpts.

## Limitations

The decision to go with DisplayLink has a couple repurcussions that mainly impact streaming and gaming. Since Breezy Box is intended to be a productivity device, the simplicity that DisplayLink offers this design seems worth that tradeoff.

* **DRM video will be blocked** -- since the DisplayLink driver runs as a form of screen capture, DRM-restricted video will get blocked. Non-DRM video and other workarounds (such as disabling hardware rendering for browser-based video) should still work. Gaming should also still work, but the next point is important.
* **CPU consumption** - the DisplayLink driver captures and encodes changes (damage) to the display as a process on the host CPU. For productivity, damage areas tend to be minimal (e.g. typing or moving a mouse), so the CPU consumption for a few displays should also be minimal. For watching videos or gaming, the damage area may encompass an entire display, and be much more demanding on the host CPU. This also impacts our Breezy Box: our SBC boards should have enough processing power to handle a few incoming displays, but there may be a limit to how many displays can be present or how many videos can be playing at a time before overloading the SBC and dropping frames.