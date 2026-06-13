#define _POSIX_C_SOURCE 200809L
#define _DEFAULT_SOURCE

#include "breezy_state.h"
#include "link_services.h"

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
 * Synchronously probe OTG host connectivity with ping (ICMP echo).
 * Runs at most once per poll cycle (~1 s) so the blocking wait is fine:
 * ping returns immediately on a reply, or after the -W timeout otherwise.
 * Carrier/operstate are not reliable on USB gadget interfaces, so we always
 * ping rather than relying on link state.
 *
 *   ping -q -c 1 -W 1 -I <iface> <host_ip>
 *
 * Exit 0  → got a reply → host connected.
 * Non-zero → timeout/error → host gone.
 */
static bool arping_probe(struct breezy_state *bs)
{
	if (bs->otg_iface[0] == '\0' || bs->host_ip[0] == '\0')
		return false;

	pid_t pid = fork();
	if (pid < 0)
		return bs->arping_result;

	if (pid == 0) {
		int devnull = open("/dev/null", O_WRONLY);
		if (devnull >= 0) {
			dup2(devnull, STDOUT_FILENO);
			dup2(devnull, STDERR_FILENO);
			close(devnull);
		}
		execlp("ping", "ping", "-q", "-c", "1", "-W", "1",
		       "-I", bs->otg_iface, bs->host_ip, (char *)NULL);
		_exit(127);
	}

	int status;
	waitpid(pid, &status, 0);
	bs->arping_result = WIFEXITED(status) && WEXITSTATUS(status) == 0;
	return bs->arping_result;
}

/*
 * Probe wired Ethernet host presence via carrier state.
 * Unlike USB gadget interfaces, real Ethernet carrier is reliable: if the
 * link is up the host NIC is running on the other end of the cable.
 * No blocking — reads a single byte from sysfs.
 */
static bool eth_carrier_probe(const struct breezy_state *bs)
{
	char path[128];
	FILE *f;
	int val = 0;

	if (bs->eth_iface[0] == '\0')
		return false;

	snprintf(path, sizeof(path), "/sys/class/net/%s/carrier", bs->eth_iface);
	f = fopen(path, "r");
	if (!f)
		return false;
	fscanf(f, "%d", &val);
	fclose(f);
	return val == 1;
}

static bool detect_wlan_ip(const char *exclude_iface, char *out, size_t cap)
{
	struct ifaddrs *ifaddr = NULL;
	char fallback[BS_ADDR_MAX] = "";
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
 * Public API
 * ---------------------------------------------------------------- */

void breezy_state_set_addresses(struct breezy_state *bs,
				 const struct link_services_config *link_cfg,
				 const char *otg_netdev)
{
	if (!bs)
		return;

	bs->link_ip[0]    = '\0';
	bs->host_ip[0]    = '\0';
	bs->link_mdns[0]  = '\0';
	bs->arping_result = false;
	bs->wlan_mdns[0]  = '\0';
	bs->wlan_ip[0]    = '\0';
	bs->otg_iface[0]  = '\0';

	if (otg_netdev && otg_netdev[0])
		snprintf(bs->otg_iface, sizeof(bs->otg_iface), "%s", otg_netdev);

	if (link_cfg) {
		if (link_cfg->link_ip[0])
			snprintf(bs->link_ip, sizeof(bs->link_ip),
				 "%s", link_cfg->link_ip);

		if (link_cfg->host_ip[0])
			snprintf(bs->host_ip, sizeof(bs->host_ip),
				 "%s", link_cfg->host_ip);

		if (link_cfg->link_name[0])
			snprintf(bs->link_mdns, sizeof(bs->link_mdns),
				 "%s.local", link_cfg->link_name);

		if (link_cfg->wlan_name[0])
			snprintf(bs->wlan_mdns, sizeof(bs->wlan_mdns),
				 "%s.local", link_cfg->wlan_name);
	}

	/* Attempt an initial wlan IP detection.
	 * Failure is fine — we'll retry on each update call. */
	detect_wlan_ip(bs->otg_iface[0] ? bs->otg_iface : NULL,
		       bs->wlan_ip, sizeof(bs->wlan_ip));
}

void breezy_state_set_eth_link(struct breezy_state *bs,
				const char *eth_iface,
				const char *eth_link_ip,
				const char *eth_host_ip)
{
	if (!bs)
		return;

	bs->eth_iface[0]   = '\0';
	bs->eth_link_ip[0] = '\0';
	bs->eth_host_ip[0] = '\0';

	if (eth_iface && eth_iface[0])
		snprintf(bs->eth_iface, sizeof(bs->eth_iface), "%s", eth_iface);
	if (eth_link_ip && eth_link_ip[0])
		snprintf(bs->eth_link_ip, sizeof(bs->eth_link_ip), "%s", eth_link_ip);
	if (eth_host_ip && eth_host_ip[0])
		snprintf(bs->eth_host_ip, sizeof(bs->eth_host_ip), "%s", eth_host_ip);
}

bool breezy_state_update(struct breezy_state *bs,
			  bool xr_driver_up,
			  bool glasses_active,
			  size_t imported_count,
			  size_t device_count,
			  bool verbose)
{
	(void)device_count;

	if (!bs)
		return false;

	/* Re-poll wlan IP in case it changed (Wi-Fi assigned after startup). */
	char new_wlan_ip[BS_ADDR_MAX];
	if (detect_wlan_ip(bs->otg_iface[0] ? bs->otg_iface : NULL,
			   new_wlan_ip, sizeof(new_wlan_ip))) {
		if (strcmp(new_wlan_ip, bs->wlan_ip) != 0)
			snprintf(bs->wlan_ip, sizeof(bs->wlan_ip), "%s", new_wlan_ip);
	} else {
		bs->wlan_ip[0] = '\0';
	}

	/*
	 * Probe each direct-link path independently so the overlay can show the
	 * correct IP for whichever one the host is on.
	 *
	 * OTG: ping the DHCP-assigned host IP.  When USB/IP clients are already
	 *   imported we skip the probe and use the cached result — the 1 s ping
	 *   timeout is unnecessary when we already know the host is reachable.
	 *
	 * Wired Ethernet: check carrier state only (sysfs read, non-blocking).
	 *   Real Ethernet carrier is reliable: carrier up means a host NIC is
	 *   running on the other end of the cable.
	 */
	bool otg_connected  = (imported_count > 0u) ? bs->arping_result
	                                             : arping_probe(bs);
	bool eth_connected  = eth_carrier_probe(bs);
	bool host_connected = otg_connected || eth_connected || (imported_count > 0u);

	if (verbose)
		printf("breezy_state: otg_iface=%s host_ip=%s otg=%d "
		       "eth_iface=%s eth=%d xr_driver=%d glasses=%d imported=%zu mode=%d\n",
		       bs->otg_iface, bs->host_ip, (int)otg_connected,
		       bs->eth_iface, (int)eth_connected,
		       (int)xr_driver_up, (int)glasses_active, imported_count, (int)bs->mode);

	enum breezy_state_mode next;
	if (!xr_driver_up)
		next = BREEZY_STATE_XR_DRIVER_DOWN;
	else if (!glasses_active)
		next = BREEZY_STATE_NO_GLASSES;
	else if (!host_connected)
		next = BREEZY_STATE_NO_HOST;
	else if (imported_count == 0u)
		next = BREEZY_STATE_NO_CLIENTS;
	else
		next = BREEZY_STATE_NORMAL;

	bool changed = (next != bs->mode) ||
		       (host_connected != bs->host_connected) ||
		       (otg_connected  != bs->otg_connected)  ||
		       (eth_connected  != bs->eth_connected);

	if (otg_connected != bs->otg_connected) {
		if (otg_connected)
			printf("breezy: OTG host connected (%s, host %s)\n",
			       bs->otg_iface[0] ? bs->otg_iface : "?", bs->host_ip);
		else
			printf("breezy: OTG host disconnected\n");
	}

	if (eth_connected != bs->eth_connected) {
		if (eth_connected)
			printf("breezy: wired Ethernet host connected (%s, host %s)\n",
			       bs->eth_iface[0] ? bs->eth_iface : "?", bs->eth_host_ip);
		else
			printf("breezy: wired Ethernet host disconnected\n");
	}

	static const char *const mode_names[] = {
		[BREEZY_STATE_XR_DRIVER_DOWN] = "xr-driver-down",
		[BREEZY_STATE_NO_GLASSES]     = "no-glasses",
		[BREEZY_STATE_NO_HOST]        = "no-host",
		[BREEZY_STATE_NO_CLIENTS]     = "no-clients",
		[BREEZY_STATE_NORMAL]         = "normal",
	};
	if (next != bs->mode)
		printf("breezy: state %s -> %s\n",
		       mode_names[bs->mode], mode_names[next]);

	bs->mode           = next;
	bs->host_connected = host_connected;
	bs->otg_connected  = otg_connected;
	bs->eth_connected  = eth_connected;

	return changed;
}

void breezy_state_format_message(const struct breezy_state *bs,
				  char *buf, size_t cap)
{
	if (!bs || !buf || cap == 0)
		return;

	/*
	 * Pick which IP to advertise based on which direct-link path is active.
	 * Priority: wired Ethernet > OTG > wlan (falling back when unavailable).
	 * The mDNS name (breezy.local) is the same for both wired paths; it
	 * resolves to the correct IP on whichever interface the host queries from.
	 */
	const char *active_mdns;
	const char *active_ip;
	if (bs->eth_connected && bs->eth_link_ip[0]) {
		active_mdns = bs->link_mdns[0] ? bs->link_mdns : bs->wlan_mdns;
		active_ip   = bs->eth_link_ip;
	} else if (bs->otg_connected && bs->link_ip[0]) {
		active_mdns = bs->link_mdns[0] ? bs->link_mdns : bs->wlan_mdns;
		active_ip   = bs->link_ip;
	} else {
		/* Wifi, or imports-only where the specific path is unknown. */
		active_mdns = bs->wlan_mdns[0] ? bs->wlan_mdns : bs->link_mdns;
		active_ip   = bs->wlan_ip;
	}

	switch (bs->mode) {
	case BREEZY_STATE_XR_DRIVER_DOWN:
		snprintf(buf, cap,
			 "XR driver service is not running.\n"
			 "Visit %s to restart it.",
			 (active_mdns && active_mdns[0]) ? active_mdns
			 : (active_ip && active_ip[0])   ? active_ip
			                                 : "breezy.local");
		break;

	case BREEZY_STATE_NO_GLASSES:
		snprintf(buf, cap,
			 "Please connect a supported pair of XR glasses");
		break;

	case BREEZY_STATE_NO_HOST: {
		bool has_eth = bs->eth_iface[0] != '\0';
		const char *port_msg = has_eth
			? "Connect a host using the USB OTG or Ethernet port"
			: "Connect to a host using the USB OTG port";
		if (bs->wlan_mdns[0] && bs->wlan_ip[0]) {
			snprintf(buf, cap,
				 "%s,\nor visit https://%s or https://%s wirelessly",
				 port_msg, bs->wlan_mdns, bs->wlan_ip);
		} else if (bs->wlan_mdns[0]) {
			snprintf(buf, cap,
				 "%s,\nor visit https://%s wirelessly",
				 port_msg, bs->wlan_mdns);
		} else {
			snprintf(buf, cap, "%s", port_msg);
		}
		break;
	}

	case BREEZY_STATE_NO_CLIENTS:
		if (active_mdns[0] && active_ip[0]) {
			snprintf(buf, cap,
				 "No USB/IP clients connected.\n"
				 "To get started, visit https://%s or https://%s",
				 active_mdns, active_ip);
		} else if (active_mdns[0]) {
			snprintf(buf, cap,
				 "No USB/IP clients connected.\n"
				 "To get started, visit https://%s", active_mdns);
		} else if (active_ip[0]) {
			snprintf(buf, cap,
				 "No USB/IP clients connected.\n"
				 "To get started, visit https://%s", active_ip);
		} else {
			snprintf(buf, cap,
				 "No USB/IP clients connected.");
		}
		break;

	case BREEZY_STATE_NORMAL:
		if (active_mdns[0] && active_ip[0]) {
			snprintf(buf, cap,
				 "To configure your experience, visit\n"
				 "https://%s or https://%s",
				 active_mdns, active_ip);
		} else if (active_mdns[0]) {
			snprintf(buf, cap,
				 "To configure your experience, visit https://%s",
				 active_mdns);
		} else if (active_ip[0]) {
			snprintf(buf, cap,
				 "To configure your experience, visit https://%s",
				 active_ip);
		} else {
			buf[0] = '\0';
		}
		break;
	}
}
