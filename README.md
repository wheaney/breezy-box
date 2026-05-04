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
The FunctionFS gadget prototype in `displaylink_gadget_ffs.c` now links against the `modules/udl_sink` submodule and incrementally decodes real bulk OUT traffic into the sink-side UDL decoder. That transport path is intentionally narrow: it handles USB chunk reassembly and command framing so split bulk reads can still be decoded, and it can now write the decoded frame to an inspectable binary PPM image on exit.

There is now also a parallel GadgetFS prototype in `displaylink_gadget_gadgetfs.c`. Unlike the FunctionFS version, it owns the device-level ep0 control path in userspace and can answer the specific UDL probe requests the Linux host driver cares about: the vendor firmware descriptor, the standard channel-select vendor request, and per-byte EDID reads. This is intended as the next experiment for full old-DisplayLink impersonation.

Build notes:

```sh
git submodule update --init --recursive
make
```

The resulting `displaylink_gadget_ffs` binary accepts `--decode-width` and `--decode-height` to size the sink storage, `--no-decode` to fall back to raw bulk logging while debugging the USB transport, and `--dump-image out.ppm` to snapshot the latest decoded frame when the process exits.

The `displaylink_gadget_gadgetfs` prototype is a separate binary for boards where `gadgetfs` is available. It mounts `gadgetfs`, opens the UDC device node under `/dev/gadget`, configures bulk endpoints, and feeds the host's UDL bulk traffic into the same sink-side decoder. Example:

```sh
sudo ./displaylink_gadget_gadgetfs --device-name fe800000.usb --verbose --dump-image /tmp/udl.ppm
```

## Limitations

The decision to go with DisplayLink has a couple repurcussions that mainly impact streaming and gaming. Since Breezy Box is intended to be a productivity device, the simplicity that DisplayLink offers this design seems worth that tradeoff.

* **DRM video will be blocked** -- since the DisplayLink driver runs as a form of screen capture, DRM-restricted video will get blocked. Non-DRM video and other workarounds (such as disabling hardware rendering for browser-based video) should still work. Gaming should also still work, but the next point is important.
* **CPU consumption** - the DisplayLink driver captures and encodes changes (damage) to the display as a process on the host CPU. For productivity, damage areas tend to be minimal (e.g. typing or moving a mouse), so the CPU consumption for a few displays should also be minimal. For watching videos or gaming, the damage area may encompass an entire display, and be much more demanding on the host CPU. This also impacts our Breezy Box: our SBC boards should have enough processing power to handle a few incoming displays, but there may be a limit to how many displays can be present or how many videos can be playing at a time before overloading the SBC and dropping frames.