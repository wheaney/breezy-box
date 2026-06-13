"""Breezy Box runtime environment.

Stub implementation — provides just enough to get the UI building and
installing on the box. Real enable/disable/verify logic to be filled in once
the surrounding plumbing is in place.

Packaged into the UI's ``runtimes`` subpackage by the box package script via:
    ui-gtk/bin/package <version> ui/
"""

import gettext
import logging

from gi.repository import Adw, Gtk
from ..runtimeenvironment import ExtraGroup, ExtraTab, RuntimeEnvironment

_ = gettext.gettext
logger = logging.getLogger('breezy_ui')


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

    # Virtual displays are not supported on the box.
    def is_virtual_display_supported(self):
        return False

    @property
    def excluded_field_ids(self):
        # These settings either don't apply to a headless box or are controlled
        # by the host OS / package manager rather than this UI. The built-in
        # virtual-display rows are hidden in favour of a box-specific interface
        # (see extra_groups below).
        return frozenset([
            'effect_enable_switch',
            'disable_physical_displays_switch',
            'use_optimal_monitor_config_switch',
            'use_highest_refresh_rate_switch',
            'text_scaling_scale',
            'remove_virtual_displays_on_disable_switch',
            'virtual_displays_row',
            'launch_display_settings_row',
        ])

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
    def extra_groups(self):
        return [ExtraGroup(
            tab_name='general',
            build_widget=self._build_virtual_displays_group,
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

    def _build_virtual_displays_group(self):
        # Stub box-specific virtual-displays interface. The real implementation
        # (driven by the box's own display management) will replace this — most
        # likely loading its layout from a .ui shipped alongside this module via
        # Gtk.Builder (bin/package copies runtime *.ui into the runtimes/ dir;
        # set the 'breezydesktop' translation domain on the builder).
        group = Adw.PreferencesGroup()
        group.set_title(_("Virtual Displays"))
        group.set_width_request(450)

        row = Adw.ActionRow()
        row.set_title(_("Virtual displays"))
        row.set_subtitle(_("Box-specific virtual display management is coming soon."))
        group.add(row)
        return group
