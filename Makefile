CC ?= cc
PKG_CONFIG ?= pkg-config
USBG_PKG ?= libusbgx
CPPFLAGS ?=
CFLAGS ?= -O2 -g -std=c11 -Wall -Wextra -Wpedantic
USBG_CFLAGS ?= $(shell $(PKG_CONFIG) --cflags $(USBG_PKG) 2>/dev/null)
USBG_LIBS ?= $(shell $(PKG_CONFIG) --libs $(USBG_PKG) 2>/dev/null)
UDL_SINK_DIR := modules/udl_sink

FFS_TARGET := displaylink_gadget_ffs
FFS_SOURCES := displaylink_gadget_ffs.c $(UDL_SINK_DIR)/src/udl_sink.c

GADGETFS_TARGET := displaylink_gadget_gadgetfs
GADGETFS_SOURCES := displaylink_gadget_gadgetfs.c $(UDL_SINK_DIR)/src/udl_sink.c

TARGETS := $(FFS_TARGET) $(GADGETFS_TARGET)

CPPFLAGS += -I$(UDL_SINK_DIR)/include

all: $(TARGETS)

check-ffs-deps:
	@if [ -n "$(strip $(USBG_LIBS))" ]; then \
		exit 0; \
	fi; \
	if ! command -v $(PKG_CONFIG) >/dev/null 2>&1; then \
		echo "error: $(PKG_CONFIG) was not found. Install a pkg-config provider and libusbgx development files:" >&2; \
		echo "  sudo apt install build-essential pkg-config libusbgx-dev" >&2; \
		echo "Or override USBG_LIBS manually, for example: make USBG_LIBS=-lusbgx" >&2; \
		exit 1; \
	fi; \
	echo "error: could not resolve $(USBG_PKG) linker flags via $(PKG_CONFIG)." >&2; \
	echo "Install the libusbgx development package or override USBG_CFLAGS/USBG_LIBS manually." >&2; \
	echo "  sudo apt install build-essential pkg-config libusbgx-dev" >&2; \
	exit 1

$(FFS_TARGET): check-ffs-deps $(FFS_SOURCES)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(USBG_CFLAGS) -o $@ $(FFS_SOURCES) $(USBG_LIBS)

$(GADGETFS_TARGET): $(GADGETFS_SOURCES)
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $(GADGETFS_SOURCES)

clean:
	rm -f $(TARGETS)

.PHONY: all clean check-ffs-deps