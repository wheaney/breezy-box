# SPDX-License-Identifier: GPL-2.0
obj-m := g_rndis_displaylink.o
g_rndis_displaylink-objs := composite.o f_displaylink.o

KDIR ?= /lib/modules/$(shell uname -r)/build

# Use $(CURDIR) rather than $(PWD): sudo strips the PWD environment variable, which
# would otherwise leave M= empty when running `sudo make install`.
all:
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules

clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean

# Install into /lib/modules so `modprobe g_rndis_displaylink` works (and survives
# reboots).  Run as root: sudo make install
install:
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules_install
	depmod -a

.PHONY: all clean install
