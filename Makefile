CC ?= cc
CPPFLAGS ?=
CFLAGS ?= -O2 -g -std=c11 -Wall -Wextra -Wpedantic
USBG_CFLAGS ?= $(shell pkg-config --cflags libusbgx 2>/dev/null)
USBG_LIBS ?= $(shell pkg-config --libs libusbgx 2>/dev/null)
UDL_SINK_DIR := modules/udl_sink

FFS_TARGET := displaylink_gadget_ffs
FFS_SOURCES := displaylink_gadget_ffs.c $(UDL_SINK_DIR)/src/udl_sink.c

GADGETFS_TARGET := displaylink_gadget_gadgetfs
GADGETFS_SOURCES := displaylink_gadget_gadgetfs.c $(UDL_SINK_DIR)/src/udl_sink.c

TARGETS := $(FFS_TARGET) $(GADGETFS_TARGET)

CPPFLAGS += -I$(UDL_SINK_DIR)/include

all: $(TARGETS)

$(FFS_TARGET): $(FFS_SOURCES)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(USBG_CFLAGS) -o $@ $(FFS_SOURCES) $(USBG_LIBS)

$(GADGETFS_TARGET): $(GADGETFS_SOURCES)
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $(GADGETFS_SOURCES)

clean:
	rm -f $(TARGETS)

.PHONY: all clean