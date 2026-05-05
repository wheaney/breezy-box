CC ?= cc
PKG_CONFIG ?= pkg-config
USBG_PKG ?= libusbgx
CPPFLAGS ?=
CFLAGS ?= -O2 -g -std=c11 -Wall -Wextra -Wpedantic
USBG_PKG_CFLAGS := $(shell $(PKG_CONFIG) --cflags $(USBG_PKG) 2>/dev/null)
USBG_PKG_LIBS := $(shell $(PKG_CONFIG) --libs $(USBG_PKG) 2>/dev/null)
USBG_FALLBACK_LIBS ?= -lusbgx
USBG_CFLAGS ?= $(USBG_PKG_CFLAGS)
USBG_LIBS ?= $(if $(strip $(USBG_PKG_LIBS)),$(USBG_PKG_LIBS),$(USBG_FALLBACK_LIBS))
UDL_SINK_DIR := modules/udl_sink

FFS_TARGET := displaylink_gadget_ffs
FFS_SOURCES := displaylink_gadget_ffs.c $(UDL_SINK_DIR)/src/udl_sink.c

GADGETFS_TARGET := displaylink_gadget_gadgetfs
GADGETFS_SOURCES := displaylink_gadget_gadgetfs.c $(UDL_SINK_DIR)/src/udl_sink.c

RAW_GADGET_TARGET := displaylink_gadget_raw_gadget
RAW_GADGET_SOURCES := displaylink_gadget_raw_gadget.c

TARGETS := $(FFS_TARGET) $(GADGETFS_TARGET) $(RAW_GADGET_TARGET)

CPPFLAGS += -I$(UDL_SINK_DIR)/include

all: $(TARGETS)

check-ffs-deps:
	@printf '%s\n' '#include <usbg/usbg.h>' 'int main(void) { return 0; }' | \
		$(CC) $(CPPFLAGS) $(CFLAGS) $(USBG_CFLAGS) -x c - -o /dev/null $(USBG_LIBS) >/dev/null 2>&1 && \
		exit 0; \
	echo "error: unable to compile/link against libusbgx." >&2; \
	echo "Tried USBG_CFLAGS='$(USBG_CFLAGS)' USBG_LIBS='$(USBG_LIBS)'" >&2; \
	echo "Install the development packages or override USBG_CFLAGS/USBG_LIBS manually." >&2; \
	echo "  sudo apt install build-essential pkg-config libusbgx-dev libconfig-dev" >&2; \
	echo "  make USBG_LIBS=-lusbgx" >&2; \
	exit 1

$(FFS_TARGET): check-ffs-deps $(FFS_SOURCES)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(USBG_CFLAGS) -o $@ $(FFS_SOURCES) $(USBG_LIBS)

$(GADGETFS_TARGET): $(GADGETFS_SOURCES)
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $(GADGETFS_SOURCES)

$(RAW_GADGET_TARGET): $(RAW_GADGET_SOURCES)
	$(CC) $(CPPFLAGS) $(CFLAGS) -pthread -o $@ $(RAW_GADGET_SOURCES)

clean:
	rm -f $(TARGETS)

.PHONY: all clean check-ffs-deps