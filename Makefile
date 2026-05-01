CC ?= cc
CPPFLAGS ?=
CFLAGS ?= -O2 -g -std=c11 -Wall -Wextra -Wpedantic
USBG_CFLAGS ?= $(shell pkg-config --cflags libusbgx 2>/dev/null)
USBG_LIBS ?= $(shell pkg-config --libs libusbgx 2>/dev/null)
UDL_SINK_DIR := modules/udl_sink

TARGET := displaylink_gadget_ffs
SOURCES := displaylink_gadget_ffs.c $(UDL_SINK_DIR)/src/udl_sink.c

CPPFLAGS += -I$(UDL_SINK_DIR)/include

all: $(TARGET)

$(TARGET): $(SOURCES)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(USBG_CFLAGS) -o $@ $(SOURCES) $(USBG_LIBS)

clean:
	rm -f $(TARGET)

.PHONY: all clean