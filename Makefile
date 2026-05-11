CC ?= cc
PKG_CONFIG ?= pkg-config
CPPFLAGS ?=
CFLAGS ?= -O2 -g -std=c11 -Wall -Wextra -Wpedantic
UDL_SINK_DIR := modules/udl_sink
SDL2_CFLAGS ?= $(shell $(PKG_CONFIG) --cflags sdl2)
SDL2_LIBS ?= $(shell $(PKG_CONFIG) --libs sdl2)

RAW_GADGET_TARGET := displaylink_gadget_raw_gadget
RAW_GADGET_SOURCES := displaylink_gadget_raw_gadget.c $(UDL_SINK_DIR)/src/udl_sink.c

TARGETS := $(RAW_GADGET_TARGET)

CPPFLAGS += -I$(UDL_SINK_DIR)/include

all: $(TARGETS)

$(RAW_GADGET_TARGET): $(RAW_GADGET_SOURCES)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(SDL2_CFLAGS) -pthread -o $@ $(RAW_GADGET_SOURCES) $(SDL2_LIBS)

clean:
	rm -f $(TARGETS)

.PHONY: all clean