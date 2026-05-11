CC ?= cc
CPPFLAGS ?=
CFLAGS ?= -O2 -g -std=c11 -Wall -Wextra -Wpedantic

RAW_GADGET_TARGET := displaylink_gadget_raw_gadget
RAW_GADGET_SOURCES := displaylink_gadget_raw_gadget.c

TARGETS := $(RAW_GADGET_TARGET)

all: $(TARGETS)

$(RAW_GADGET_TARGET): $(RAW_GADGET_SOURCES)
	$(CC) $(CPPFLAGS) $(CFLAGS) -pthread -o $@ $(RAW_GADGET_SOURCES)

clean:
	rm -f $(TARGETS)

.PHONY: all clean