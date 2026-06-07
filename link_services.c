#define _DEFAULT_SOURCE
#define _POSIX_C_SOURCE 200809L

#include "link_services.h"

#include <arpa/inet.h>
#include <dirent.h>
#include <errno.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#define LS_DNSMASQ_PIDFILE "/run/breezy-dnsmasq.pid"
#define LS_DNSMASQ_LEASES  "/run/breezy-dnsmasq.leases"
#define LS_MDNS_LINK_PIDFILE "/run/breezy-avahi-publish-link.pid"
#define LS_MDNS_WLAN_PIDFILE "/run/breezy-avahi-publish-wlan.pid"

static void kill_matching_publishers(const char *fqdn);
static void kill_pidfile(const char *path);

/* Run a command to completion; returns 0 on exit status 0, -1 otherwise. */
static int run_cmd(const char *const argv[])
{
    pid_t pid = fork();
    if (pid < 0) {
        perror("link_services: fork");
        return -1;
    }
    if (pid == 0) {
        execvp(argv[0], (char *const *)argv);
        _exit(127);
    }
    int status;
    if (waitpid(pid, &status, 0) < 0) {
        perror("link_services: waitpid");
        return -1;
    }
    return (WIFEXITED(status) && WEXITSTATUS(status) == 0) ? 0 : -1;
}

/*
 * Launch a long-running command in the background, recording its pid in pidfile
 * so it can be stopped later (mirrors how dnsmasq is tracked).  Used for tools
 * that hold their state only while running (e.g. avahi-publish-address).
 * Returns 0 if the child was forked, -1 on fork failure.
 */
static int spawn_cmd(const char *pidfile, const char *const argv[])
{
    pid_t pid = fork();
    if (pid < 0) {
        perror("link_services: fork");
        return -1;
    }
    if (pid == 0) {
        execvp(argv[0], (char *const *)argv);
        _exit(127);
    }
    FILE *f = fopen(pidfile, "w");
    if (f) {
        fprintf(f, "%ld\n", (long)pid);
        fclose(f);
    }

    /*
     * If the helper exits immediately (for example on a name collision), reap
     * it here so startup can report a real failure instead of pretending the
     * long-running publisher is alive.
     */
    {
        struct timespec ts = {0, 100000000L};
        int status;
        int i;

        for (i = 0; i < 20; i++) {
            pid_t rc = waitpid(pid, &status, WNOHANG);

            if (rc == pid) {
                (void)unlink(pidfile);
                return (WIFEXITED(status) && WEXITSTATUS(status) == 0) ? 0 : -1;
            }
            if (rc < 0)
                break;
            nanosleep(&ts, NULL);
        }
    }

    return 0;
}

static int spawn_publish_address(const char *pidfile,
                                 const char *fqdn,
                                 const char *ip)
{
    const char *const argv[] = {"avahi-publish-address", "--no-reverse", fqdn, ip, NULL};

    return spawn_cmd(pidfile, argv);
}

static bool iface_has_wireless_dir(const char *ifname)
{
    char path[128];
    struct stat st;

    if (!ifname || ifname[0] == '\0')
        return false;
    snprintf(path, sizeof(path), "/sys/class/net/%s/wireless", ifname);
    return stat(path, &st) == 0 && S_ISDIR(st.st_mode);
}

static int detect_non_link_ipv4(const char *exclude_iface, char *out, size_t cap)
{
    struct ifaddrs *ifaddr = NULL;
    struct ifaddrs *ifa;
    char fallback[LS_IP_MAX] = "";

    if (!out || cap == 0)
        return -1;
    out[0] = '\0';

    if (getifaddrs(&ifaddr) != 0)
        return -1;

    for (ifa = ifaddr; ifa; ifa = ifa->ifa_next) {
        const struct sockaddr_in *sin;
        char addrbuf[INET_ADDRSTRLEN];
        uint32_t addr;

        if (!ifa->ifa_name || !ifa->ifa_addr)
            continue;
        if (ifa->ifa_addr->sa_family != AF_INET)
            continue;
        if ((ifa->ifa_flags & IFF_UP) == 0)
            continue;
        if ((ifa->ifa_flags & IFF_LOOPBACK) != 0)
            continue;
        if (exclude_iface && strcmp(ifa->ifa_name, exclude_iface) == 0)
            continue;

        sin = (const struct sockaddr_in *)ifa->ifa_addr;
        addr = ntohl(sin->sin_addr.s_addr);
        if ((addr & 0xffff0000u) == 0xa9fe0000u)
            continue;
        if (!inet_ntop(AF_INET, &sin->sin_addr, addrbuf, sizeof(addrbuf)))
            continue;

        if (iface_has_wireless_dir(ifa->ifa_name)) {
            snprintf(out, cap, "%s", addrbuf);
            freeifaddrs(ifaddr);
            return 0;
        }
        if (fallback[0] == '\0')
            snprintf(fallback, sizeof(fallback), "%s", addrbuf);
    }

    freeifaddrs(ifaddr);
    if (fallback[0] == '\0')
        return -1;

    snprintf(out, cap, "%s", fallback);
    return 0;
}

static int read_proc_cmdline(pid_t pid, char *buf, size_t cap)
{
    char path[64];
    FILE *f;
    size_t n;

    if (cap == 0)
        return -1;
    snprintf(path, sizeof(path), "/proc/%ld/cmdline", (long)pid);
    f = fopen(path, "rb");
    if (!f)
        return -1;
    n = fread(buf, 1, cap - 1, f);
    fclose(f);
    if (n == 0)
        return -1;
    buf[n] = '\0';
    return 0;
}

static void kill_matching_publishers(const char *fqdn)
{
    DIR *d;
    struct dirent *e;

    if (!fqdn || fqdn[0] == '\0')
        return;

    d = opendir("/proc");
    if (!d)
        return;

    while ((e = readdir(d)) != NULL) {
        char *argv0;
        char *argv1;
        char *argv2;
        char cmdline[512];
        char *end;
        long pid_l;
        pid_t pid;

        pid_l = strtol(e->d_name, &end, 10);
        if (*e->d_name == '\0' || *end != '\0' || pid_l <= 1)
            continue;
        pid = (pid_t)pid_l;
        if (read_proc_cmdline(pid, cmdline, sizeof(cmdline)) != 0)
            continue;

        argv0 = cmdline;
        argv1 = argv0 + strlen(argv0) + 1;
        if (argv1 >= cmdline + sizeof(cmdline))
            continue;
        argv2 = argv1 + strlen(argv1) + 1;
        if (strcmp(argv0, "avahi-publish-address") != 0)
            continue;
        if (strcmp(argv1, fqdn) != 0 &&
            (argv2 >= cmdline + sizeof(cmdline) || strcmp(argv2, fqdn) != 0))
            continue;

        (void)kill(pid, SIGTERM);
    }

    closedir(d);
}

/* Kill a process recorded in a pid-file and remove the file (idempotent). */
static void kill_pidfile(const char *path)
{
    FILE *f = fopen(path, "r");
    long pid = 0;

    if (!f)
        return;
    if (fscanf(f, "%ld", &pid) == 1 && pid > 1) {
        struct timespec ts = {0, 100000000L};
        int i;

        (void)kill((pid_t)pid, SIGTERM);
        for (i = 0; i < 10; i++) {
            if (kill((pid_t)pid, 0) < 0 && errno == ESRCH)
                break;
            nanosleep(&ts, NULL);
        }
    }
    fclose(f);
    (void)unlink(path);
}

void link_services_config_defaults(struct link_services_config *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    snprintf(cfg->iface,      sizeof(cfg->iface),      "usb0");
    snprintf(cfg->host_ip,    sizeof(cfg->host_ip),    "192.168.7.1");
    snprintf(cfg->lease_time, sizeof(cfg->lease_time), "1h");
    snprintf(cfg->link_ip,    sizeof(cfg->link_ip),    "192.168.7.2");
    snprintf(cfg->link_name,  sizeof(cfg->link_name),  "breezy");
    snprintf(cfg->wlan_name,  sizeof(cfg->wlan_name),  "breezywlan");
    cfg->mtu = 0u;
}

/*
 * Launch dnsmasq as a DHCP-only server bound to one interface, handing out a
 * single fixed address to the host.  dnsmasq daemonizes itself, so run_cmd
 * returns once it has backgrounded; the pid-file is used to stop it later.
 *
 *   --port=0        : no DNS service, DHCP only
 *   --dhcp-option=3 : send an empty router option (do NOT make the host route
 *                     its traffic through the gadget)
 *   --dhcp-option=6 : send an empty DNS option (host keeps its own resolver)
 * The subnet mask is inferred from the interface's configured prefix.
 */
static int start_dhcp(const struct link_services_config *cfg)
{
    char a_iface[64];
    char a_range[200];
    char a_mtu[32];

    kill_pidfile(LS_DNSMASQ_PIDFILE); /* clear any stale instance from a previous run */

    snprintf(a_iface, sizeof(a_iface), "--interface=%s", cfg->iface);
    snprintf(a_range, sizeof(a_range), "--dhcp-range=%s,%s,%s",
             cfg->host_ip, cfg->host_ip, cfg->lease_time);

    const char *argv[16];
    size_t argc = 0u;

    argv[argc++] = "dnsmasq";
    argv[argc++] = "--conf-file=/dev/null";
    argv[argc++] = "--pid-file=" LS_DNSMASQ_PIDFILE;
    argv[argc++] = "--dhcp-leasefile=" LS_DNSMASQ_LEASES;
    argv[argc++] = a_iface;
    argv[argc++] = "--bind-interfaces";
    argv[argc++] = "--except-interface=lo";
    argv[argc++] = "--port=0";
    argv[argc++] = "--dhcp-authoritative";
    argv[argc++] = a_range;
    argv[argc++] = "--dhcp-option=3";
    argv[argc++] = "--dhcp-option=6";
    /*
     * DHCP option 26 (interface MTU): push the jumbo link MTU to the host so a
     * raised gadget MTU actually shrinks the host's TCP segment / NCM datagram
     * count on the bulk-OUT path, without the user configuring the host by hand.
     */
    if (cfg->mtu > 0u) {
        snprintf(a_mtu, sizeof(a_mtu), "--dhcp-option=26,%u", cfg->mtu);
        argv[argc++] = a_mtu;
    }
    argv[argc++] = "--quiet-dhcp";
    argv[argc++] = NULL;

    return run_cmd(argv);
}

/*
 * Publish the mDNS names via the system avahi daemon as explicit address
 * aliases:
 *   - link_name is pinned to link_ip so it ALWAYS resolves to the USB link.
 *   - wlan_name is pinned to the first non-link IPv4 address (prefer a
 *     wireless interface) so it resolves to the Wi-Fi/LAN path instead of the
 *     USB gadget IP.
 * Returns 0 if the responder is available, -1 only when avahi itself cannot be
 * started.
 */
static int start_mdns(const struct link_services_config *cfg,
                      struct link_services_state *state)
{
    /* Best-effort: make sure the responder is running (no-op if already up). */
    { const char *const a[] = {"systemctl", "start", "avahi-daemon", NULL};
      if (run_cmd(a) != 0)
          return -1; }

    if (cfg->wlan_name[0] != '\0') {
        char fqdn[LS_NAME_MAX + 8];
        char wlan_ip[LS_IP_MAX];

        snprintf(fqdn, sizeof(fqdn), "%s.local", cfg->wlan_name);
        kill_pidfile(LS_MDNS_WLAN_PIDFILE);
        kill_matching_publishers(fqdn);

        if (detect_non_link_ipv4(cfg->iface, wlan_ip, sizeof(wlan_ip)) == 0) {
            if (spawn_publish_address(LS_MDNS_WLAN_PIDFILE, fqdn, wlan_ip) == 0) {
                if (state)
                    state->wlan_name_set = true;
            } else {
                fprintf(stderr,
                    "link_services: could not pin %s to %s via avahi\n",
                    fqdn, wlan_ip);
            }
        } else {
            fprintf(stderr,
                "link_services: could not find a non-link IPv4 address for %s.local\n",
                cfg->wlan_name);
        }
    }

    /* Re-pin the link name, clearing any publisher left by a previous run. */
    if (cfg->link_name[0] != '\0' && cfg->link_ip[0] != '\0') {
        char fqdn[LS_NAME_MAX + 8];

        snprintf(fqdn, sizeof(fqdn), "%s.local", cfg->link_name);
        kill_pidfile(LS_MDNS_LINK_PIDFILE);
        kill_matching_publishers(fqdn);
        if (spawn_publish_address(LS_MDNS_LINK_PIDFILE, fqdn, cfg->link_ip) == 0) {
            if (state)
                state->link_name_pinned = true;
        } else {
            fprintf(stderr,
                "link_services: could not pin %s to %s via avahi (name already in use or stale publisher still exiting)\n",
                fqdn, cfg->link_ip);
        }
    }

    return 0;
}

int link_services_start(const struct link_services_config *cfg,
                        struct link_services_state *state)
{
    if (state)
        memset(state, 0, sizeof(*state));
    if (!cfg || cfg->iface[0] == '\0')
        return -1;

    if (cfg->host_ip[0] != '\0') {
        if (start_dhcp(cfg) == 0) {
            if (state)
                state->dhcp_running = true;
            printf("link_services: DHCP serving %s to the host on %s\n",
                   cfg->host_ip, cfg->iface);
        } else {
            fprintf(stderr,
                "link_services: dnsmasq unavailable; the host must set its own IP "
                "on %s (install 'dnsmasq' to make this automatic)\n", cfg->iface);
        }
    }

    bool want_mdns = cfg->link_name[0] != '\0' || cfg->wlan_name[0] != '\0';
    if (want_mdns) {
        if (start_mdns(cfg, state) == 0) {
            if (state && state->link_name_pinned) {
                printf("link_services: publishing %s.local for the link address %s via mDNS\n",
                       cfg->link_name, cfg->link_ip);
            }
            if (state && state->wlan_name_set) {
                printf("link_services: publishing %s.local for the non-link address via mDNS\n",
                       cfg->wlan_name);
            }
        } else {
            fprintf(stderr,
                "link_services: avahi unavailable; .local names will not resolve "
                "(install 'avahi-daemon' and 'avahi-utils')\n");
        }
    }

    return 0;
}

void link_services_stop(struct link_services_state *state)
{
    kill_pidfile(LS_DNSMASQ_PIDFILE);
    kill_pidfile(LS_MDNS_LINK_PIDFILE);
    kill_pidfile(LS_MDNS_WLAN_PIDFILE);
    if (state)
        memset(state, 0, sizeof(*state));
}
