"""Breezy Box runtime environment.

Stub implementation — provides just enough to get the UI building and
installing on the box. Real enable/disable/verify logic to be filled in once
the surrounding plumbing is in place.

Packaged into the UI's ``runtimes`` subpackage by the box package script via:
    ui-gtk/bin/package <version> ui/
"""

import gettext
import json
import logging
import os

from gi.repository import Adw, Gio, GLib, GObject, Gtk

from ..files import get_config_dir
from ..runtimeenvironment import ExtraField, ExtraTab, RuntimeEnvironment
from ..settingsmanager import SettingsManager

_ = gettext.gettext
logger = logging.getLogger('breezy_ui')

# Custom resolutions on the box are capped at 1080p.
MAX_DISPLAY_WIDTH = 1920
MAX_DISPLAY_HEIGHT = 1080

# Monitor labels follow this format; the integer is the lowest free slot,
# reused when a lower-numbered display is removed.
MONITOR_LABEL_FORMAT = _("Breezy Display %d")

# DisplayLink device defaults mirrored from the box server's built-in defaults
# (see configured_device_default in src/server.c). The UI owns the full device
# entry it writes, so it fills these in rather than relying on the config's
# top-level "defaults" block.
DEFAULT_REFRESH_HZ = 60
DEFAULT_MANUFACTURER_STRING = "DisplayLink"
DEFAULT_PRODUCT_STRING = "DisplayLink Adapter"
DISPLAYLINK_VENDOR_ID = "0x17e9"
DISPLAYLINK_PRODUCT_ID = "0x037a"


class BreezyBoxDisplayManager(GObject.GObject):
    """Reads and writes the box's virtual displays.

    The box has no live virtual-display process manager like the GNOME runtime;
    displays are persisted as entries in the ``devices`` array of
    ``$XDG_CONFIG_HOME/breezy-box/config.json``. This manager is the single
    owner of that array from the UI's perspective: it loads it on construction,
    exposes it via the ``displays`` property (matching the interface
    ConnectedDevice binds to), and rewrites the file on every change.

    Each display carries a ``monitor_name`` of the form "Breezy Display N".
    ``N`` is the lowest positive integer not currently in use, so removing a
    lower-numbered display frees that slot for the next addition.
    """

    __gproperties__ = {
        'displays': (object, 'Displays', 'A list of the displays', GObject.ParamFlags.READWRITE)
    }

    def __init__(self):
        GObject.GObject.__init__(self)
        self._config_path = os.path.join(get_config_dir(), 'breezy-box', 'config.json')
        self.displays = []
        self._load()

    # --- persistence ------------------------------------------------------

    def _read_config(self):
        try:
            with open(self._config_path, 'r') as f:
                return json.load(f)
        except FileNotFoundError:
            return {}
        except Exception as e:
            logger.error("Failed to read box config %s: %s", self._config_path, e)
            return {}

    def _load(self):
        config = self._read_config()
        devices = config.get('devices')
        self.set_property('displays', devices if isinstance(devices, list) else [])

    def _save(self):
        config = self._read_config()
        config['devices'] = self.displays
        try:
            os.makedirs(os.path.dirname(self._config_path), exist_ok=True)
            with open(self._config_path, 'w') as f:
                json.dump(config, f, indent=2)
        except Exception as e:
            logger.error("Failed to write box config %s: %s", self._config_path, e)

    # --- monitor-number allocation ----------------------------------------

    def _monitor_number(self, display):
        """Return the integer N parsed from a display's "Breezy Display N"
        monitor_name, or None if it doesn't match."""
        name = display.get('monitor_name', '')
        prefix = MONITOR_LABEL_FORMAT.split('%d')[0]
        if name.startswith(prefix):
            try:
                return int(name[len(prefix):].strip())
            except ValueError:
                return None
        return None

    def _next_monitor_number(self):
        used = {n for n in (self._monitor_number(d) for d in self.displays) if n is not None}
        n = 1
        while n in used:
            n += 1
        return n

    def monitor_label(self, display):
        return display.get('monitor_name') or _("Breezy Display")

    # --- layout -----------------------------------------------------------

    def _next_position(self):
        """Tile the new display to the right of the right-most existing one:
        x = (right-most display's x + its width), y = that display's y. The
        first display sits at the origin."""
        if not self.displays:
            return 0, 0
        rightmost = max(self.displays, key=lambda d: d.get('x', 0))
        x = rightmost.get('x', 0) + rightmost.get('decode_width', MAX_DISPLAY_WIDTH)
        y = rightmost.get('y', 0)
        return x, y

    # --- mutations --------------------------------------------------------

    def create_virtual_display(self, width, height, framerate):
        number = self._next_monitor_number()
        index = number - 1
        x, y = self._next_position()

        device = {
            'name': f"display-{number}",
            'busid': f"breezy-box-{index}",
            'device_path': f"/devices/platform/breezy-box/displaylink{index}",
            'serial_string': f"BREEZY{number:04d}",
            'monitor_name': MONITOR_LABEL_FORMAT % number,
            'manufacturer_string': DEFAULT_MANUFACTURER_STRING,
            'product_string': DEFAULT_PRODUCT_STRING,
            'vendor_id': DISPLAYLINK_VENDOR_ID,
            'product_id': DISPLAYLINK_PRODUCT_ID,
            'decode_width': int(round(width)),
            'decode_height': int(round(height)),
            'refresh_hz': framerate or DEFAULT_REFRESH_HZ,
            'x': x,
            'y': y,
        }
        self.set_property('displays', self.displays + [device])
        self._save()
        return device

    def destroy_virtual_display(self, monitor_name):
        remaining = [d for d in self.displays if d.get('monitor_name') != monitor_name]
        if len(remaining) == len(self.displays):
            return False
        self.set_property('displays', remaining)
        self._save()
        return True

    # --- GObject property plumbing ----------------------------------------

    def do_set_property(self, prop, value):
        if prop.name == 'displays':
            self.displays = value

    def do_get_property(self, prop):
        if prop.name == 'displays':
            return self.displays


class BreezyBoxRearrangeDisplaysDialog(Adw.Window):
    """Stub dialog for rearranging the box's virtual displays.

    Empty shell for now — opens with a header bar and a placeholder. The real
    drag-to-arrange layout editor will replace the placeholder content.
    """

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.set_title(_("Rearrange Displays"))
        self.set_modal(True)
        self.set_default_size(640, 480)

        toolbar_view = Adw.ToolbarView()
        toolbar_view.add_top_bar(Adw.HeaderBar())

        status = Adw.StatusPage()
        status.set_icon_name("video-display-symbolic")
        status.set_title(_("Rearrange Displays"))
        status.set_description(_("Display arrangement is coming soon."))
        status.set_vexpand(True)
        toolbar_view.set_content(status)

        self.set_content(toolbar_view)


class BreezyBoxVirtualDisplaysWidget(Adw.ExpanderRow):
    """Box-specific virtual-displays interface.

    Mirrors the look and behavior of the built-in virtual-displays controls,
    with box-specific caveats:

      * custom resolutions are capped at 1080p;
      * instead of launching the host's display settings, the box either shows
        a "keep this tab open" hint (when the host manages the layout) or a
        "Rearrange displays" button that opens a box-side dialog.

    The injection point (ExtraField) adds a single row to a built-in
    preferences group, so the whole interface lives in one expandable row: the
    expander's title carries the resolution picker + add/remove buttons, and the
    nested rows are one per existing display plus a final host-management row.
    """

    def __init__(self, runtime):
        super().__init__()
        self.runtime = runtime
        self.display_manager = runtime.virtual_display_manager
        self.settings = SettingsManager.get_instance().settings

        self._display_rows = []
        self._custom_resolution_options = []

        self.set_title(_("Virtual displays"))
        # Show the configured displays on load rather than hiding them behind a
        # collapsed expander.
        self.set_expanded(True)

        # --- header controls: resolution picker + add/remove -------------
        self.resolution_menu = Gtk.ComboBoxText()
        self.resolution_menu.append('create_1080p_display', _("1080p"))
        self.resolution_menu.append('add_custom_resolution', _("Add custom"))
        self.resolution_menu.set_active_id('create_1080p_display')
        self.resolution_menu.set_valign(Gtk.Align.CENTER)
        self.add_suffix(self.resolution_menu)

        self.remove_custom_resolution_button = Gtk.Button(
            icon_name='list-remove-symbolic', valign=Gtk.Align.CENTER, visible=False,
            tooltip_text=_("Remove custom resolution"))
        self.remove_custom_resolution_button.add_css_class('flat')
        self.add_suffix(self.remove_custom_resolution_button)

        self.add_button = Gtk.Button(
            icon_name='list-add-symbolic', valign=Gtk.Align.CENTER,
            tooltip_text=_("Add a virtual display"))
        self.add_button.add_css_class('flat')
        self.add_suffix(self.add_button)

        # --- host window-management row ----------------------------------
        self.host_management_row = Adw.ActionRow()
        self.rearrange_button = Gtk.Button(
            label=_("Rearrange displays"), valign=Gtk.Align.CENTER)
        self.rearrange_button.add_css_class('flat')
        self.host_management_row.add_suffix(self.rearrange_button)
        self.add_row(self.host_management_row)

        # --- wiring ------------------------------------------------------
        self.resolution_menu.connect('changed', self._on_resolution_menu_changed)
        self.add_button.connect('clicked', self._on_add_virtual_display)
        self.remove_custom_resolution_button.connect('clicked', self._on_custom_resolution_remove)
        self.rearrange_button.connect('clicked', self._on_rearrange)

        self.display_manager.connect('notify::displays', self._on_displays_changed)
        self.settings.connect(
            'changed::breezy-box-host-window-management-available',
            self._refresh_host_management_row)

        self._refresh_host_management_row()
        self._rebuild_display_rows()

    # --- display rows -----------------------------------------------------

    def _rebuild_display_rows(self):
        for row in self._display_rows:
            self.remove(row)
        self._display_rows = []

        # Re-anchor the host-management row below the freshly added display
        # rows by removing it first and re-adding it last.
        self.remove(self.host_management_row)

        for display in self.display_manager.displays:
            row = Adw.ActionRow()
            row.set_title(self.display_manager.monitor_label(display))
            row.set_subtitle(f"{display.get('decode_width')} x {display.get('decode_height')}")

            icon = Gtk.Image.new_from_icon_name("video-display-symbolic")
            row.add_prefix(icon)

            remove_button = Gtk.Button(
                icon_name='user-trash-symbolic', valign=Gtk.Align.CENTER,
                tooltip_text=_("Remove display"))
            remove_button.add_css_class('flat')
            remove_button.connect(
                'clicked', self._on_remove_virtual_display, display.get('monitor_name'))
            row.add_suffix(remove_button)

            self.add_row(row)
            self._display_rows.append(row)

        self.add_row(self.host_management_row)

    def _on_displays_changed(self, *args):
        GLib.idle_add(self._rebuild_display_rows)

    # --- host window management -------------------------------------------

    def _refresh_host_management_row(self, *args):
        host_manages = self.settings.get_boolean(
            'breezy-box-host-window-management-available')
        if host_manages:
            self.host_management_row.set_title(_(
                "Keep this browser tab open to sync layout changes from your PC"))
            self.rearrange_button.set_visible(False)
        else:
            self.host_management_row.set_title(_("Rearrange displays"))
            self.rearrange_button.set_visible(True)

    def _on_rearrange(self, *args):
        dialog = BreezyBoxRearrangeDisplaysDialog()
        dialog.set_transient_for(self.get_ancestor(Gtk.Window))
        dialog.present()

    # --- add / custom resolution ------------------------------------------

    def _on_add_virtual_display(self, *args):
        resolution = self.resolution_menu.get_active_id()
        if resolution == 'create_1080p_display':
            width, height = 1920, 1080
        elif resolution in self._custom_resolution_options:
            width, height = (int(v) for v in resolution.split('x'))
        else:
            return

        logger.info("Adding box virtual display %sx%s", width, height)
        self.display_manager.create_virtual_display(width, height, DEFAULT_REFRESH_HZ)

    def _on_remove_virtual_display(self, button, monitor_name):
        self.display_manager.destroy_virtual_display(monitor_name)

    def _on_resolution_menu_changed(self, widget):
        resolution = widget.get_active_id()
        self.remove_custom_resolution_button.set_visible(
            resolution in self._custom_resolution_options)

        adding_custom = resolution == 'add_custom_resolution'
        self.add_button.set_sensitive(not adding_custom)
        if adding_custom:
            # Imported lazily: the dialog's @Gtk.Template binds to a GResource
            # that is only registered once the app's resource bundle is loaded.
            from ..customresolutiondialog import CustomResolutionDialog
            dialog = CustomResolutionDialog(
                self._on_custom_resolution_add,
                max_width=MAX_DISPLAY_WIDTH, max_height=MAX_DISPLAY_HEIGHT)
            dialog.set_transient_for(self.get_ancestor(Gtk.Window))
            dialog.present()

    def _on_custom_resolution_add(self, width, height):
        # Cap custom resolutions at 1080p.
        width = min(int(round(width)), MAX_DISPLAY_WIDTH)
        height = min(int(round(height)), MAX_DISPLAY_HEIGHT)

        resolution = f"{width}x{height}"
        if resolution not in self._custom_resolution_options:
            self._custom_resolution_options.append(resolution)
            # Insert custom options above the trailing "Add custom" entry.
            self.resolution_menu.insert(1, resolution, resolution)

        self.resolution_menu.set_active_id(resolution)

    def _on_custom_resolution_remove(self, *args):
        resolution = self.resolution_menu.get_active_id()
        if resolution not in self._custom_resolution_options:
            return

        index = self._custom_resolution_options.index(resolution)
        self._custom_resolution_options.remove(resolution)
        # Custom options occupy positions 1..N (after the 1080p entry).
        self.resolution_menu.remove(1 + index)
        self.resolution_menu.set_active_id('create_1080p_display')


class BreezyBoxRuntimeEnvironment(RuntimeEnvironment):
    """Runs Breezy on a headless Breezy Box.

    The box is always "installed" and "enabled" from the UI's perspective —
    there is no separate Shell extension to toggle. Enable/disable, verification,
    and update checking are stubs to be wired up once the box-side services are
    ready.
    """

    APP_NAMESPACE = 'breezy_box'

    # The box runtime is always present; no separate component to install.
    def is_installed(self):
        return True

    def is_enabled(self):
        return True

    def enable(self):
        pass

    def disable(self):
        pass

    # No separate verification step on the box.
    def verify(self):
        return True

    # Updates are handled by the OS / package manager, not the UI.
    def check_for_update(self, current_version, callback):
        return None

    # The box always has a device in context; skip the dedicated no-device view.
    @property
    def shows_no_device_view(self):
        return False

    # Virtual displays are supported on the box via the config-backed manager.
    def is_virtual_display_supported(self):
        return True

    def _create_virtual_display_manager(self):
        return BreezyBoxDisplayManager()

    @property
    def excluded_field_ids(self):
        # These settings either don't apply to a headless box or are controlled
        # by the host OS / package manager rather than this UI. The built-in
        # virtual-display rows are hidden in favour of a box-specific interface
        # (see extra_fields below).
        return frozenset([
            'effect_enable_switch',
            'disable_physical_displays_switch',
            'use_optimal_monitor_config_switch',
            'use_highest_refresh_rate_switch',
            'text_scaling_scale',
            'remove_virtual_displays_on_disable_switch',
            'virtual_displays_row',
            'launch_display_settings_row',
            'headset_as_primary_switch',
            'headset_display_as_viewport_center_switch',
        ])

    @property
    def multiple_displays_fields_always_unlocked(self):
        # The box has no connected-display awareness in the UI, so the
        # wrapping scheme and monitor spacing fields are always available.
        return True

    @property
    def excluded_tab_names(self):
        # Replace the standard shortcuts tab with a box-specific one below.
        return frozenset(['shortcuts'])

    @property
    def extra_tabs(self):
        return [ExtraTab(
            name='host_shortcuts',
            title=_('Keyboard Shortcuts'),
            icon_name='preferences-desktop-keyboard-shortcuts-symbolic',
            build_widget=self._build_shortcuts_widget,
        )]

    @property
    def extra_fields(self):
        # Re-add a box-specific virtual-displays field where the hidden built-in
        # one used to live: the bottom of the Features group (general, column 0).
        return [ExtraField(
            tab_name='general',
            column=0,
            build_widget=self._build_virtual_displays_row,
        )]

    def reset_excluded_fields(self, settings):
        for key in ('disable-physical-displays', 'use-optimal-monitor-config',
                    'use-highest-refresh-rate', 'remove-virtual-displays-on-disable'):
            settings.reset(key)

    def _build_shortcuts_widget(self):
        label = Gtk.Label()
        label.set_wrap(True)
        label.set_xalign(0)
        label.set_margin_top(24)
        label.set_margin_bottom(24)
        label.set_margin_start(24)
        label.set_margin_end(24)
        label.set_text(_(
            "Keyboard shortcuts for Breezy are triggered on your host machine, "
            "not on the Breezy Box itself.\n\n"
            "To set up shortcuts, configure them on your host using your desktop "
            "environment's keyboard-shortcut settings. Each shortcut should send "
            "the corresponding key combination to the Breezy Box over the network "
            "or input-forwarding tool you are using (e.g. Barrier, Input Leap, or "
            "a custom script)."
        ))
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
        box.append(label)
        return box

    def _build_virtual_displays_row(self):
        return BreezyBoxVirtualDisplaysWidget(self)
