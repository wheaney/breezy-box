"""Breezy Box runtime environment.

Stub implementation — provides just enough to get the UI building and
installing on the box. Real enable/disable/verify logic to be filled in once
the surrounding plumbing is in place.

Packaged into the UI's ``runtimes`` subpackage by the box package script via:
    ui-gtk/bin/package <version> ui/
"""

import logging

from ..runtimeenvironment import RuntimeEnvironment

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
