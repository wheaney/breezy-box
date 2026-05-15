CC ?= cc
PKG_CONFIG ?= pkg-config
CPPFLAGS ?=
CFLAGS ?= -O2 -g -std=c11 -Wall -Wextra -Wpedantic
UDL_SINK_DIR := modules/udl_sink
SDL2_CFLAGS ?= $(shell $(PKG_CONFIG) --cflags sdl2)
SDL2_LIBS ?= $(shell $(PKG_CONFIG) --libs sdl2)

RAW_GADGET_TARGET := displaylink_gadget_raw_gadget
MULTI_SESSION_DEMO_TARGET := displaylink_multi_session_demo
RAW_GADGET_SOURCES := displaylink_gadget_raw_gadget.c displaylink_compositor.c $(UDL_SINK_DIR)/src/udl_sink.c
MULTI_SESSION_DEMO_SOURCES := displaylink_multi_session_demo.c $(RAW_GADGET_SOURCES)

TARGETS := $(RAW_GADGET_TARGET) $(MULTI_SESSION_DEMO_TARGET)

CPPFLAGS += -I$(UDL_SINK_DIR)/include

all: $(TARGETS)

$(RAW_GADGET_TARGET): $(RAW_GADGET_SOURCES)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(SDL2_CFLAGS) -pthread -o $@ $(RAW_GADGET_SOURCES) $(SDL2_LIBS)

$(MULTI_SESSION_DEMO_TARGET): $(MULTI_SESSION_DEMO_SOURCES)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(SDL2_CFLAGS) -DDISPLAYLINK_SESSION_NO_MAIN -pthread -o $@ $(MULTI_SESSION_DEMO_SOURCES) $(SDL2_LIBS)

clean:
	rm -f $(TARGETS)

.PHONY: all clean