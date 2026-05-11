CC ?= cc
CPPFLAGS ?=
CFLAGS ?= -O2 -g -std=c11 -Wall -Wextra -Wpedantic
UDL_SINK_DIR := modules/udl_sink

RAW_GADGET_TARGET := displaylink_gadget_raw_gadget
RAW_GADGET_SOURCES := displaylink_gadget_raw_gadget.c $(UDL_SINK_DIR)/src/udl_sink.c

TARGETS := $(RAW_GADGET_TARGET)

CPPFLAGS += -I$(UDL_SINK_DIR)/include

all: $(TARGETS)

$(RAW_GADGET_TARGET): $(RAW_GADGET_SOURCES)
	$(CC) $(CPPFLAGS) $(CFLAGS) -pthread -o $@ $(RAW_GADGET_SOURCES)

clean:
	rm -f $(TARGETS)

.PHONY: all clean