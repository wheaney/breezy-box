# Development workflow on the SBC

This doc covers how to iterate quickly on the SBC — stopping, rebuilding, and
relaunching services manually — without fighting systemd.

## First-time setup

After running `sudo ./setup_system.sh`, reboot once so the getty autologin
takes effect.  The tty1 login shell starts `breezy.target` via `.bash_profile`.

**Do not enable linger** for the breezy user (`loginctl enable-linger`).  Linger
starts a seat-less user session at boot before the tty1 login, and `breezy.target`
lands there instead — where it has no seat and the renderer can't take DRM master.

## Watching logs

```sh
# All breezy services, interleaved:
journalctl --user -f -u "breezy-*"

# The renderer is a SYSTEM service (kiosk on tty1, needs DRM master):
journalctl -f -u breezy-renderer

# The headless services are --user services:
journalctl --user -f -u breezy-web
```

## Stopping services for a rebuild

```sh
# Stop everything under the target:
systemctl --user stop breezy.target

# Or stop one service while leaving the rest running.
# Note: the renderer is a SYSTEM service, so it needs sudo (not --user):
sudo systemctl stop breezy-renderer
```

`breezy.target` uses `PartOf=` so stopping the target cascades to all its
user units; stopping an individual unit only takes that one down.  The renderer
is a separate system service and is not part of the user target.

## Rebuilding

```sh
cd ~/breezy-box          # or wherever your checkout lives
make deps                # first time only — fetches mongoose into src/vendor/
make                     # builds displaylink_kms_renderer and breezy_web
```

Install the new binaries wherever the units expect them (`%h/.local/bin/`):

```sh
cp displaylink_kms_renderer ~/.local/bin/
cp breezy_web ~/.local/bin/
```

## Launching manually (without systemd)

This is useful when you want to see stdout/stderr directly in your terminal
and step through a crash.

### Renderer

The KMS renderer must run on the **active VT** (tty1, on seat0) to become DRM
master.  In production it's a **system** service bound to tty1
(`breezy-renderer.service`).  To run it by hand, stop that service so it frees
tty1, then run from tty1 directly (or after `sudo chvt 1`):

```sh
# Stop the system renderer service first (note: sudo, not --user):
sudo systemctl stop breezy-renderer

# Run directly from tty1 (or after `sudo chvt 1`):
./displaylink_kms_renderer [your usual args]
```

When done, restart the service to hand tty1 back to the kiosk renderer:

```sh
sudo systemctl start breezy-renderer
```

### Web server

No VT restriction — run from any terminal:

```sh
systemctl --user stop breezy-web

# Runs on 8081 (port 80 -> 8081 redirect is handled by nftables/iptables via setup_system.sh):
./breezy_web --port 8081 --web-root web/
```

### VNC stack (Xvfb + breezydesktop + x11vnc + noVNC)

```sh
systemctl --user stop breezy-xvfb breezy-desktop breezy-x11vnc breezy-novnc

# Start manually in order (each in its own terminal or backgrounded):
Xvfb :1 -screen 0 1920x1080x24 &
DISPLAY=:1 GTK_A11Y=none ~/.local/bin/breezydesktop &
x11vnc -display :1 -nopw -shared -forever -loop \
       -encodings "tight zrle hextile" -ncache 10 -defer 10 -dnow -xkb &
/usr/share/novnc/utils/novnc_proxy --vnc localhost:5900 --listen 8080
```

## Switching back to systemd management

```sh
systemctl --user start breezy.target
```

Or restart just one unit (renderer is a system service, so sudo):

```sh
sudo systemctl restart breezy-renderer
```

## Enabling / disabling on boot

```sh
# Disable breezy.target (don't start at login):
systemctl --user disable breezy.target

# Re-enable:
systemctl --user enable breezy.target
```

## Common pitfalls

**"Failed to connect to bus: No such file or directory"**
The user systemd session hasn't started yet.  Either log in as the breezy user
interactively first, or make sure `loginctl enable-linger <user>` was run.

**Renderer crashes with "DRM: failed to set master"**
Something else holds DRM master on the active VT (e.g. a previous renderer
instance that wasn't cleaned up, or the display manager if running).  Check
with `fuser /dev/dri/card0` and kill the offending process.

**x11vnc exits immediately with "Can't open display :1"**
Xvfb isn't up yet.  Add a short delay or wait for `breezy-xvfb` to report
active: `systemctl --user is-active breezy-xvfb`.

**Web UI not reachable on port 80**
`breezy-web` runs on 8081; port 80 is redirected to it by an nftables/iptables
rule installed by `setup_system.sh`.  If the redirect isn't active, re-run
`sudo ./setup_system.sh` or access directly on port 8081.
