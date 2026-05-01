CC ?= cc
CFLAGS ?= -O2 -g -std=c11 -Wall -Wextra -Wpedantic
USBG_CFLAGS ?= $(shell pkg-config --cflags libusbgx 2>/dev/null)
USBG_LIBS ?= $(shell pkg-config --libs libusbgx 2>/dev/null)

TARGET := displaylink_gadget_ffs
SOURCES := displaylink_gadget_ffs.c

all: $(TARGET)

$(TARGET): $(SOURCES)
	$(CC) $(CFLAGS) $(USBG_CFLAGS) -o $@ $(SOURCES) $(USBG_LIBS)

clean:
	rm -f $(TARGET)

.PHONY: all clean