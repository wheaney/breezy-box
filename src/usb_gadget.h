#pragma once

#include <stdbool.h>
#include <stddef.h>

#define USB_GADGET_NAME_MAX   64
#define USB_GADGET_NETDEV_MAX 32
#define USB_GADGET_IP_MAX     48   /* IPv4/6 CIDR, e.g. "192.168.7.2/30" */
#define USB_GADGET_MAC_MAX    18   /* "xx:xx:xx:xx:xx:xx\0" */
#define USB_GADGET_STR_MAX    128

/*
 * Configuration for the OTG RNDIS USB gadget.
 * Fill with usb_gadget_config_defaults() then override as needed.
 */
struct usb_gadget_config {
    /*
     * UDC device name (e.g. "fe980000.usb").
     * Leave empty to auto-detect when exactly one UDC is present.
     */
    char udc_name[USB_GADGET_NAME_MAX];

    /* Directory name under /sys/kernel/config/usb_gadget/ */
    char gadget_name[USB_GADGET_NAME_MAX];

    /* USB device identity strings */
    char serial_string[USB_GADGET_STR_MAX];
    char manufacturer_string[USB_GADGET_STR_MAX];
    char product_string[USB_GADGET_STR_MAX];

    /* RNDIS network function */
    char rndis_dev_mac[USB_GADGET_MAC_MAX];   /* gadget-side MAC */
    char rndis_host_mac[USB_GADGET_MAC_MAX];  /* host-side MAC */
    char rndis_netdev[USB_GADGET_NETDEV_MAX]; /* kernel net interface (e.g. "usb0") */
    char rndis_ip_cidr[USB_GADGET_IP_MAX];    /* e.g. "192.168.7.2/30" */
    unsigned rndis_mtu;                       /* 0 = leave interface MTU at kernel default */
};

/* Runtime state owned by the caller; zero-initialise before use */
struct usb_gadget_state {
    bool active;
    bool adopted;   /* true when gadget was pre-configured externally; teardown skips configfs */
    char gadget_root[256];                /* /sys/kernel/config/usb_gadget/<name> */
    char udc_name[USB_GADGET_NAME_MAX];
    char netdev_name[USB_GADGET_NETDEV_MAX];
};

/* Populate cfg with sensible defaults */
void usb_gadget_config_defaults(struct usb_gadget_config *cfg);

/*
 * Configure the RNDIS gadget and bring up the network interface.
 * Requires CAP_NET_ADMIN and write access to /sys/kernel/config.
 * Returns 0 on success, -1 on error (message printed to stderr).
 * On partial failure, state is left in a condition that usb_gadget_teardown()
 * can safely clean up.
 */
int usb_gadget_setup(const struct usb_gadget_config *cfg,
                     struct usb_gadget_state *state);

/*
 * Tear down the gadget and bring down the network interface.
 * Idempotent: safe to call even if setup was never called or failed partway.
 */
void usb_gadget_teardown(struct usb_gadget_state *state);
