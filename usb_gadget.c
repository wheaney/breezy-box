#define _POSIX_C_SOURCE 200809L

#include "usb_gadget.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mount.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#define CONFIGFS_ROOT  "/sys/kernel/config"
#define GADGET_BASE    CONFIGFS_ROOT "/usb_gadget"
#define UDC_CLASS_DIR  "/sys/class/udc"

static int run_cmd(const char *const argv[]);

/* ------------------------------------------------------------------ helpers */

static int write_configfs(const char *path, const char *value)
{
    int fd = open(path, O_WRONLY);
    if (fd < 0) {
        fprintf(stderr, "usb_gadget: open(%s): %s\n", path, strerror(errno));
        return -1;
    }
    size_t len = strlen(value);
    ssize_t n = write(fd, value, len);
    int err = errno;
    close(fd);
    if (n < 0 || (size_t)n != len) {
        fprintf(stderr, "usb_gadget: write(%s): %s\n", path, strerror(err));
        return -1;
    }
    return 0;
}


static int mkdir_ok(const char *path)
{
    if (mkdir(path, 0755) == 0 || errno == EEXIST)
        return 0;
    fprintf(stderr, "usb_gadget: mkdir(%s): %s\n", path, strerror(errno));
    return -1;
}

static void rm_link(const char *path)
{
    if (unlink(path) < 0 && errno != ENOENT)
        fprintf(stderr, "usb_gadget: unlink(%s): %s\n", path, strerror(errno));
}

static void rm_dir(const char *path)
{
    if (rmdir(path) < 0 && errno != ENOENT)
        fprintf(stderr, "usb_gadget: rmdir(%s): %s\n", path, strerror(errno));
}

static int run_cmd(const char *const argv[])
{
    pid_t pid = fork();
    if (pid < 0) {
        perror("usb_gadget: fork");
        return -1;
    }
    if (pid == 0) {
        execvp(argv[0], (char *const *)argv);
        _exit(127);
    }
    int status;
    if (waitpid(pid, &status, 0) < 0) {
        perror("usb_gadget: waitpid");
        return -1;
    }
    return (WIFEXITED(status) && WEXITSTATUS(status) == 0) ? 0 : -1;
}

static bool parse_unsigned_env(const char *name, unsigned *value_out)
{
    const char *value;
    char *end = NULL;
    unsigned long parsed;

    if (!name || !value_out)
        return false;

    value = getenv(name);
    if (!value || value[0] == '\0')
        return false;

    errno = 0;
    parsed = strtoul(value, &end, 0);
    if (errno != 0 || !end || *end != '\0' || parsed > UINT_MAX) {
        fprintf(stderr, "usb_gadget: ignoring invalid %s=%s\n", name, value);
        return false;
    }

    *value_out = (unsigned)parsed;
    return true;
}

/* ------------------------------------------------------------------ configfs */

static int ensure_configfs(void)
{
    /* Best-effort module loads; these are no-ops if already loaded */
    { const char *const argv[] = {"modprobe", "libcomposite", NULL}; run_cmd(argv); }
    { const char *const argv[] = {"modprobe", "usb_f_rndis",  NULL}; run_cmd(argv); }

    /* Mount configfs if not already present; EBUSY means already mounted */
    if (mount("none", CONFIGFS_ROOT, "configfs", 0, NULL) < 0 && errno != EBUSY) {
        fprintf(stderr, "usb_gadget: mount configfs: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

static int detect_udc(char *out, size_t cap)
{
    DIR *d = opendir(UDC_CLASS_DIR);
    if (!d) {
        fprintf(stderr, "usb_gadget: opendir(%s): %s\n", UDC_CLASS_DIR, strerror(errno));
        return -1;
    }
    char found[USB_GADGET_NAME_MAX] = "";
    int n = 0;
    struct dirent *e;
    while ((e = readdir(d)) != NULL) {
        if (e->d_name[0] == '.')
            continue;
        if (n++ == 0) {
            size_t len = strlen(e->d_name);

            if (len >= sizeof(found)) {
                fprintf(stderr, "usb_gadget: UDC name too long: %s\n", e->d_name);
                closedir(d);
                return -1;
            }
            memcpy(found, e->d_name, len + 1u);
        }
    }
    closedir(d);
    if (n == 0) {
        fprintf(stderr, "usb_gadget: no UDC found under %s\n", UDC_CLASS_DIR);
        return -1;
    }
    if (n > 1) {
        fprintf(stderr, "usb_gadget: multiple UDCs found; set udc_name explicitly\n");
        return -1;
    }
    snprintf(out, cap, "%s", found);
    return 0;
}

static void teardown_configfs(const char *root)
{
    char p[512];

    /* Unbind UDC before touching the directory tree */
    snprintf(p, sizeof(p), "%s/UDC", root);
    if (access(p, F_OK) == 0) {
        int fd = open(p, O_WRONLY);
        if (fd >= 0) {
            (void)write(fd, "\n", 1);
            close(fd);
        }
    }

    snprintf(p, sizeof(p), "%s/configs/c.1/rndis.usb0", root); rm_link(p);
    snprintf(p, sizeof(p), "%s/os_desc/c.1",            root); rm_link(p);

    snprintf(p, sizeof(p), "%s/functions/rndis.usb0/os_desc/interface.rndis", root);
    rm_dir(p);
    snprintf(p, sizeof(p), "%s/functions/rndis.usb0", root); rm_dir(p);

    snprintf(p, sizeof(p), "%s/configs/c.1/strings/0x409", root); rm_dir(p);
    snprintf(p, sizeof(p), "%s/configs/c.1",               root); rm_dir(p);
    snprintf(p, sizeof(p), "%s/strings/0x409",             root); rm_dir(p);

    rm_dir(root);
}

/* ------------------------------------------------------------------ public API */

void usb_gadget_config_defaults(struct usb_gadget_config *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    snprintf(cfg->gadget_name,         sizeof(cfg->gadget_name),         "breezy-composite");
    snprintf(cfg->serial_string,       sizeof(cfg->serial_string),       "BREEZY0001");
    snprintf(cfg->manufacturer_string, sizeof(cfg->manufacturer_string), "Breezy Box");
    snprintf(cfg->product_string,        sizeof(cfg->product_string),        "Breezy Box OTG RNDIS");
    snprintf(cfg->rndis_dev_mac,         sizeof(cfg->rndis_dev_mac),         "02:1a:11:00:00:01");
    snprintf(cfg->rndis_host_mac,        sizeof(cfg->rndis_host_mac),        "02:1a:11:00:00:02");
    snprintf(cfg->rndis_netdev,          sizeof(cfg->rndis_netdev),          "usb0");
    snprintf(cfg->rndis_ip_cidr,         sizeof(cfg->rndis_ip_cidr),         "192.168.7.2/30");
    (void)parse_unsigned_env("BREEZY_RNDIS_MTU", &cfg->rndis_mtu);
}

int usb_gadget_setup(const struct usb_gadget_config *cfg,
                     struct usb_gadget_state *state)
{
    char udc[USB_GADGET_NAME_MAX];
    char root[256];
    char p[512], q[512];

    memset(state, 0, sizeof(*state));

    /* Unload the legacy g_rndis_displaylink module if active so the RNDIS
     * configfs gadget can claim the UDC cleanly. */
    { const char *const a[] = {"rmmod", "g_rndis_displaylink", NULL}; (void)run_cmd(a); }

    if (ensure_configfs() < 0)
        return -1;

    if (cfg->udc_name[0] != '\0')
        snprintf(udc, sizeof(udc), "%s", cfg->udc_name);
    else if (detect_udc(udc, sizeof(udc)) < 0)
        return -1;

    snprintf(root, sizeof(root), "%s/%s", GADGET_BASE, cfg->gadget_name);

    /* Remove any leftover gadget before building the new one. */
    teardown_configfs(root);

    if (mkdir_ok(root) < 0)
        return -1;

    /* Populate state early so partial failures leave a teardown-safe condition. */
    snprintf(state->gadget_root, sizeof(state->gadget_root), "%s", root);
    snprintf(state->udc_name,    sizeof(state->udc_name),    "%s", udc);
    snprintf(state->netdev_name, sizeof(state->netdev_name), "%s", cfg->rndis_netdev);

#define WF(rel, val) \
    do { snprintf(p, sizeof(p), "%s/" rel, root); \
         if (write_configfs(p, val) < 0) return -1; } while (0)
#define MD(rel) \
    do { snprintf(p, sizeof(p), "%s/" rel, root); \
         if (mkdir_ok(p) < 0) return -1; } while (0)

    /*
     * RNDIS requires the Miscellaneous Device class (EF/02/01) so Windows
     * recognises the OS descriptor and loads the inbox RNDIS driver without
     * needing a third-party INF.
     */
    WF("idVendor",  "0x1d6b");
    WF("idProduct", "0x0104");
    WF("bcdUSB",    "0x0200");
    WF("bcdDevice", "0x0100");
    WF("bDeviceClass",    "0xEF");
    WF("bDeviceSubClass", "0x02");
    WF("bDeviceProtocol", "0x01");

    /* Device strings */
    MD("strings/0x409");
    WF("strings/0x409/serialnumber", cfg->serial_string);
    WF("strings/0x409/manufacturer", cfg->manufacturer_string);
    WF("strings/0x409/product",      cfg->product_string);

    /* Configuration c.1 */
    MD("configs/c.1");
    MD("configs/c.1/strings/0x409");
    WF("configs/c.1/strings/0x409/configuration", "Breezy RNDIS");
    WF("configs/c.1/MaxPower", "250");

    /*
     * OS descriptor: tells Windows to use the RNDIS driver automatically.
     * The os_desc/use flag must be set before binding to the UDC.
     */
    WF("os_desc/use",        "1");
    WF("os_desc/b_vendor_code", "0xcd");

    /* RNDIS function instance */
    MD("functions/rndis.usb0");
    WF("functions/rndis.usb0/dev_addr",  cfg->rndis_dev_mac);
    WF("functions/rndis.usb0/host_addr", cfg->rndis_host_mac);
    WF("functions/rndis.usb0/os_desc/interface.rndis/compatible_id",    "RNDIS");
    WF("functions/rndis.usb0/os_desc/interface.rndis/sub_compatible_id", "5162001");

    /* Wire function into config and expose it via os_desc. */
    snprintf(p, sizeof(p), "%s/functions/rndis.usb0", root);
    snprintf(q, sizeof(q), "%s/configs/c.1/rndis.usb0", root);
    if (symlink(p, q) < 0) {
        fprintf(stderr, "usb_gadget: symlink(%s): %s\n", q, strerror(errno));
        return -1;
    }
    snprintf(p, sizeof(p), "%s/configs/c.1", root);
    snprintf(q, sizeof(q), "%s/os_desc/c.1", root);
    if (symlink(p, q) < 0 && errno != EEXIST) {
        fprintf(stderr, "usb_gadget: symlink(%s): %s\n", q, strerror(errno));
        return -1;
    }

    /*
     * Bind to UDC — this makes the gadget live on the bus.
     *
     * After binding, force a USB re-enumeration by briefly unbinding and
     * rebinding the gadget.  Without this, a host that was already connected
     * when the app started will not see the gadget appear: the UDC was either
     * unbound or bound to a different gadget before we ran, so the host never
     * received an attach event.  The unbind/rebind causes a disconnect pulse on
     * the D+ line followed by a fresh enumeration, which works regardless of
     * whether the host was plugged in before or after startup.
     */
    WF("UDC", udc);

    {
        /* Brief pause so the host has time to register the initial attach
         * before we immediately toggle it.  50 ms is well within the USB
         * debounce window but long enough to avoid the host missing the
         * disconnect on fast systems. */
        struct timespec ts = {0, 50000000L};
        nanosleep(&ts, NULL);
    }

    snprintf(p, sizeof(p), "%s/UDC", root);
    {
        int fd = open(p, O_WRONLY);
        if (fd >= 0) {
            (void)write(fd, "\n", 1);
            close(fd);
        }
    }

    {
        struct timespec ts = {0, 50000000L};
        nanosleep(&ts, NULL);
    }

    WF("UDC", udc);

#undef WF
#undef MD

    /* Bring up network interface. */
    { const char *const argv[] = {"ip", "addr", "flush", "dev", cfg->rndis_netdev, NULL};
      run_cmd(argv); }  /* best-effort; interface may not exist yet */
    if (cfg->rndis_mtu > 0u) {
        char mtu[16];
        const char *const argv[] = {"ip", "link", "set", "dev", cfg->rndis_netdev, "mtu", mtu, NULL};

        snprintf(mtu, sizeof(mtu), "%u", cfg->rndis_mtu);
        if (run_cmd(argv) < 0)
            return -1;
    }
    { const char *const argv[] = {"ip", "link", "set", cfg->rndis_netdev, "up", NULL};
      if (run_cmd(argv) < 0) return -1; }
    { const char *const argv[] = {"ip", "addr", "add", cfg->rndis_ip_cidr, "dev", cfg->rndis_netdev, NULL};
      if (run_cmd(argv) < 0) return -1; }

    state->active = true;
    printf("usb_gadget: RNDIS gadget on UDC %s, iface %s %s",
           udc, cfg->rndis_netdev, cfg->rndis_ip_cidr);
    if (cfg->rndis_mtu > 0u)
        printf(" mtu=%u", cfg->rndis_mtu);
    printf("\n");
    return 0;
}

void usb_gadget_teardown(struct usb_gadget_state *state)
{
    if (!state || !state->active)
        return;

    if (state->netdev_name[0]) {
        { const char *const argv[] = {"ip", "addr", "flush", "dev", state->netdev_name, NULL};
          run_cmd(argv); }
        { const char *const argv[] = {"ip", "link", "set", state->netdev_name, "down", NULL};
          run_cmd(argv); }
    }

    if (state->gadget_root[0]) {
        teardown_configfs(state->gadget_root);
    }

    memset(state, 0, sizeof(*state));
}
