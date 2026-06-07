#define _POSIX_C_SOURCE 200809L
#define _DEFAULT_SOURCE

#include "breezy_overlay.h"
#include "link_services.h"
#include "overlay_text.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/* ----------------------------------------------------------------
 * Network helpers
 * ---------------------------------------------------------------- */

/*
 * Synchronously probe host connectivity with ping (ICMP echo).
 * Runs at most once per poll cycle (~1 s) so the blocking wait is fine:
 * ping returns immediately on a reply, or after the -W timeout otherwise.
 *
 *   ping -q -c 1 -W 1 -I <iface> <host_ip>
 *
 * Exit 0  → got a reply → host connected.
 * Non-zero → timeout/error → host gone.
 */
static bool arping_probe(struct breezy_overlay *ov)
{
	if (ov->otg_iface[0] == '\0' || ov->host_ip[0] == '\0')
		return false;

	pid_t pid = fork();
	if (pid < 0)
		return ov->arping_result;

	if (pid == 0) {
		int devnull = open("/dev/null", O_WRONLY);
		if (devnull >= 0) {
			dup2(devnull, STDOUT_FILENO);
			dup2(devnull, STDERR_FILENO);
			close(devnull);
		}
		execlp("ping", "ping", "-q", "-c", "1", "-W", "1",
		       "-I", ov->otg_iface, ov->host_ip, (char *)NULL);
		_exit(127);
	}

	int status;
	waitpid(pid, &status, 0);
	ov->arping_result = WIFEXITED(status) && WEXITSTATUS(status) == 0;
	return ov->arping_result;
}

static bool detect_wlan_ip(const char *exclude_iface, char *out, size_t cap)
{
	struct ifaddrs *ifaddr = NULL;
	char fallback[BO_ADDR_MAX] = "";
	bool found = false;

	if (!out || cap == 0)
		return false;
	out[0] = '\0';

	if (getifaddrs(&ifaddr) != 0)
		return false;

	for (struct ifaddrs *ifa = ifaddr; ifa; ifa = ifa->ifa_next) {
		const struct sockaddr_in *sin;
		char addrbuf[INET_ADDRSTRLEN];
		uint32_t addr;

		if (!ifa->ifa_name || !ifa->ifa_addr)
			continue;
		if (ifa->ifa_addr->sa_family != AF_INET)
			continue;
		if (!(ifa->ifa_flags & IFF_UP))
			continue;
		if (ifa->ifa_flags & IFF_LOOPBACK)
			continue;
		if (exclude_iface && strcmp(ifa->ifa_name, exclude_iface) == 0)
			continue;

		sin  = (const struct sockaddr_in *)ifa->ifa_addr;
		addr = ntohl(sin->sin_addr.s_addr);
		if ((addr & 0xffff0000u) == 0xa9fe0000u) /* 169.254.x.x */
			continue;
		if (!inet_ntop(AF_INET, &sin->sin_addr, addrbuf, sizeof(addrbuf)))
			continue;

		{
			char wpath[128];
			struct stat st;
			snprintf(wpath, sizeof(wpath),
				 "/sys/class/net/%s/wireless", ifa->ifa_name);
			bool is_wireless = (stat(wpath, &st) == 0 &&
					    S_ISDIR(st.st_mode));
			if (is_wireless) {
				snprintf(out, cap, "%s", addrbuf);
				found = true;
				break;
			}
		}
		if (fallback[0] == '\0')
			snprintf(fallback, sizeof(fallback), "%s", addrbuf);
	}
	freeifaddrs(ifaddr);

	if (!found && fallback[0] != '\0') {
		snprintf(out, cap, "%s", fallback);
		found = true;
	}
	return found;
}

/* ----------------------------------------------------------------
 * Text builder
 * ---------------------------------------------------------------- */

static void rebuild_text(struct breezy_overlay *ov)
{
	char msg[OT_MAX_TEXT];

	switch (ov->state) {
	case BREEZY_OVERLAY_NO_GLASSES:
		snprintf(msg, sizeof(msg),
			 "Please connect a supported pair of XR glasses");
		break;

	case BREEZY_OVERLAY_NO_HOST:
		if (ov->wlan_mdns[0] && ov->wlan_ip[0]) {
			snprintf(msg, sizeof(msg),
				 "Connect to a host using the USB OTG port,\n"
				 "or visit https://%s or https://%s to get started wirelessly",
				 ov->wlan_mdns, ov->wlan_ip);
		} else if (ov->wlan_mdns[0]) {
			snprintf(msg, sizeof(msg),
				 "Connect to a host using the USB OTG port,\n"
				 "or visit https://%s to get started wirelessly",
				 ov->wlan_mdns);
		} else {
			snprintf(msg, sizeof(msg),
				 "Connect to a host using the USB OTG port");
		}
		break;

	case BREEZY_OVERLAY_NO_CLIENTS: {
		const char *mdns = ov->link_mdns[0] ? ov->link_mdns
						     : ov->wlan_mdns;
		const char *ip   = ov->link_ip[0]   ? ov->link_ip
						     : ov->wlan_ip;
		if (mdns[0] && ip[0]) {
			snprintf(msg, sizeof(msg),
				 "No USB/IP clients connected.\n"
				 "To get started, visit https://%s or https://%s",
				 mdns, ip);
		} else if (mdns[0]) {
			snprintf(msg, sizeof(msg),
				 "No USB/IP clients connected.\n"
				 "To get started, visit https://%s", mdns);
		} else if (ip[0]) {
			snprintf(msg, sizeof(msg),
				 "No USB/IP clients connected.\n"
				 "To get started, visit https://%s", ip);
		} else {
			snprintf(msg, sizeof(msg),
				 "No USB/IP clients connected.");
		}
		break;
	}

	case BREEZY_OVERLAY_NORMAL: {
		/* Prefer the eth/OTG path when the host is on it. */
		const char *mdns = (ov->host_connected && ov->link_mdns[0])
				       ? ov->link_mdns : ov->wlan_mdns;
		const char *ip   = (ov->host_connected && ov->link_ip[0])
				       ? ov->link_ip   : ov->wlan_ip;
		if (mdns[0] && ip[0]) {
			snprintf(msg, sizeof(msg),
				 "To configure your experience, visit\n"
				 "https://%s or https://%s",
				 mdns, ip);
		} else if (mdns[0]) {
			snprintf(msg, sizeof(msg),
				 "To configure your experience, visit https://%s",
				 mdns);
		} else if (ip[0]) {
			snprintf(msg, sizeof(msg),
				 "To configure your experience, visit https://%s",
				 ip);
		} else {
			msg[0] = '\0';
		}
		break;
	}
	}

	overlay_text_update(&ov->tex, msg);
}

/* ----------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------- */

void breezy_overlay_set_addresses(struct breezy_overlay *ov,
				   const struct link_services_config *link_cfg,
				   const char *otg_netdev)
{
	if (!ov)
		return;

	ov->link_ip[0]   = '\0';
	ov->host_ip[0]   = '\0';
	ov->link_mdns[0] = '\0';
	ov->arping_result = false;
	ov->wlan_mdns[0] = '\0';
	ov->wlan_ip[0]   = '\0';
	ov->otg_iface[0] = '\0';

	if (otg_netdev && otg_netdev[0])
		snprintf(ov->otg_iface, sizeof(ov->otg_iface), "%s", otg_netdev);

	if (link_cfg) {
		if (link_cfg->link_ip[0])
			snprintf(ov->link_ip, sizeof(ov->link_ip),
				 "%s", link_cfg->link_ip);

		if (link_cfg->host_ip[0])
			snprintf(ov->host_ip, sizeof(ov->host_ip),
				 "%s", link_cfg->host_ip);

		if (link_cfg->link_name[0])
			snprintf(ov->link_mdns, sizeof(ov->link_mdns),
				 "%s.local", link_cfg->link_name);

		if (link_cfg->wlan_name[0])
			snprintf(ov->wlan_mdns, sizeof(ov->wlan_mdns),
				 "%s.local", link_cfg->wlan_name);
	}

	/* Attempt an initial wlan IP detection.
	 * Failure is fine — we'll retry on each update call. */
	detect_wlan_ip(ov->otg_iface[0] ? ov->otg_iface : NULL,
		       ov->wlan_ip, sizeof(ov->wlan_ip));
}

void breezy_overlay_update(struct breezy_overlay *ov,
			    bool glasses_active,
			    size_t imported_count,
			    size_t device_count)
{
	(void)device_count;

	if (!ov)
		return;

	/* Re-poll wlan IP in case it changed (Wi-Fi assigned after startup). */
	char new_wlan_ip[BO_ADDR_MAX];
	if (detect_wlan_ip(ov->otg_iface[0] ? ov->otg_iface : NULL,
			   new_wlan_ip, sizeof(new_wlan_ip))) {
		if (strcmp(new_wlan_ip, ov->wlan_ip) != 0)
			snprintf(ov->wlan_ip, sizeof(ov->wlan_ip), "%s", new_wlan_ip);
	} else {
		ov->wlan_ip[0] = '\0';
	}

	/* If USB/IP sessions are already imported the host is reachable (possibly
	 * over Wi-Fi), so skip the OTG arping probe that would otherwise block
	 * wireless-only connections from advancing past NO_HOST. */
	bool host_connected = (imported_count > 0u) || arping_probe(ov);
	fprintf(stderr, "breezy_overlay: iface=%s host_ip=%s arping=%d glasses=%d imported=%zu state=%d\n",
		ov->otg_iface, ov->host_ip, (int)host_connected,
		(int)glasses_active, imported_count, (int)ov->state);

	enum breezy_overlay_state next;
	if (!glasses_active)
		next = BREEZY_OVERLAY_NO_GLASSES;
	else if (!host_connected)
		next = BREEZY_OVERLAY_NO_HOST;
	else if (imported_count == 0u)
		next = BREEZY_OVERLAY_NO_CLIENTS;
	else
		next = BREEZY_OVERLAY_NORMAL;

	bool changed = (next != ov->state) ||
		       (host_connected != ov->host_connected);

	ov->state          = next;
	ov->host_connected = host_connected;

	if (changed || !ov->tex.initialized)
		rebuild_text(ov);
}

void breezy_overlay_destroy(struct breezy_overlay *ov)
{
	if (!ov)
		return;
	overlay_text_destroy(&ov->tex);
	memset(ov, 0, sizeof(*ov));
}
