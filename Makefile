CC ?= cc
PKG_CONFIG ?= pkg-config
PYTHON ?= python3
CPPFLAGS ?=
CFLAGS ?= -O2 -g -std=c11 -Wall -Wextra -Wpedantic

SUPERVISOR := network_display_receiver_supervisor.py
USB_GADGET_SETUP := setup_usb_network_gadget.sh
EXAMPLE_CONFIG := network_display_receivers.example.json
SCENE_DEMO_TARGET := kmscube_stream_scene_demo
SCENE_DEMO_SOURCES := kmscube_stream_scene_demo.c
DRM_CFLAGS ?= $(shell $(PKG_CONFIG) --cflags libdrm gbm egl glesv2)
DRM_LIBS ?= $(shell $(PKG_CONFIG) --libs libdrm gbm egl glesv2)

TARGETS := $(SCENE_DEMO_TARGET)

all: $(TARGETS)

$(SCENE_DEMO_TARGET): $(SCENE_DEMO_SOURCES)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(DRM_CFLAGS) -o $@ $(SCENE_DEMO_SOURCES) $(DRM_LIBS) -lm

validate: $(SCENE_DEMO_TARGET)
	$(PYTHON) -m py_compile $(SUPERVISOR)

dry-run-example:
	$(PYTHON) ./$(SUPERVISOR) ./$(EXAMPLE_CONFIG) --dry-run

clean:
	rm -f $(TARGETS)
	rm -rf __pycache__

.PHONY: all validate dry-run-example clean