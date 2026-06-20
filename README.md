# Breezy Box

A Breezy Box is an "XR dock" -- a no-software, no-setup device for multi-display, XR productivity on any PC, any OS, and any USB port using just a single-board-computer.

## Hardware Requirements

To spoof a dock that can attach multiple moniters over standard USB, we'll be using the "old" DisplayLink protocol (the newer one is encrypted and hasn't been reverse engineered). This requires USB OTG hardware so our device can act as a USB peripheral. Many SBCs come with the appropriate hardware, but devices that share the OTG port with power delivery like the Raspberry Pi seem to fight against attempts to use the port OTG purposes. So at the moment, the hardware requirements are:
1. Can run Linux (the necessary chipset drivers are available), ideally something like Armbian 
2. Either:
 * An Ethernet port OR
 * At least one USB port that supports OTG (doesn't matter if USB-A or USB-C). Power delivery via a port separate from OTG
3. USB-C with DisplayPort Alt Mode support

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

#### NoVNC web portal

NoVNC allows for hosting remote apps over a web server, so I can launch and host the UI on the SBC and access it using a web browser on the host with full rendering and functionality. It would be made available over the ethernet connection we have with the host PC.

Some changes would be needed to allow the UI to switch how it creates virtual displays, based on whether it's running on GNOME or on a Breezy Box. Keyboard shortcuts are the biggest unknown, as those are OS-specific and may be difficult to use to trigger changes over USB. At the very least we may be able to provide solutions so people can create DIY shortcuts.

### DisplayLink spoofing

We need a new component that will use a libcomposite integration to create the desired displays. It would always spoof at least one DisplayLink display by default, but would be able to create and remove displays in real-time based on UI interactions, and also run at startup to create the configured number of displays when connected.

This component would also be responsible for decoding the damage data coming in over the DisplayLink protocol and updating a DRM buffer with the latest textures for use in the 3D rendering. Although I've listed this as its own component, this would most likely live as a separate thread in the DRM/KMS rendering application, in order to have simpler access to shared memory for rendering.

But unfortunately, a single USB port can only host one DisplayLink device (you can't composite multiple DL devices into one). So to host more than one device, we'll need a USB-over-the-network solution.

### USB/IP

Hosting a USB/IP endpoint allows us to expose multiple DisplayLink devices over the network. But the SBC is intended to be an offline device, and WLAN latency isn't great for the DisplayLink, so the need for a direct ethernet link becomes a must. Fortunately, USB OTG composite devices can also function as ethernet links to the host, and most SBCs have ethernet ports available.

### Wired Ethernet connections to the host

We can provide two ethernet options:
1. the OTG composite device can behave like an RNDIS ethernet gadget (or a composite of RNDIS and another standard for MacOS compat)
2. the ethernet port on the SBC can be wired directly to the host (either using a USB-ethernet adapter or direct eth-to-eth if the host and SBC have the native ports for it)

Both can advertise avahi compatible host names to make connections easier, and overlay messaging in the glasses can help get it set up. This connection would also serve to access the UI over NoVNC.


## Limitations

The decision to go with DisplayLink has a couple repurcussions that mainly impact streaming and gaming. Since Breezy Box is intended to be a productivity device, the simplicity that DisplayLink offers this design seems worth that tradeoff.

* **DRM video will be blocked** -- since the DisplayLink driver runs as a form of screen capture, DRM-restricted video will get blocked. Non-DRM video and other workarounds (such as disabling hardware rendering for browser-based video) should still work. Gaming should also still work, but the next point is important.
* **CPU consumption** - the DisplayLink driver captures and encodes changes (damage) to the display as a process on the host CPU. For productivity, damage areas tend to be minimal (e.g. typing or moving a mouse), so the CPU consumption for a few displays should also be minimal. For watching videos or gaming, the damage area may encompass an entire display, and be much more demanding on the host CPU. This also impacts our Breezy Box: our SBC boards should have enough processing power to handle a few incoming displays, but there may be a limit to how many displays can be present or how many videos can be playing at a time before overloading the SBC and dropping frames.
