CC ?= cc
PKG_CONFIG ?= pkg-config
PYTHON ?= python3
CPPFLAGS ?=
CFLAGS ?= -O2 -g -std=c11 -Wall -Wextra -Wpedantic
GST_PKG_DEPS := gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0 gstreamer-allocators-1.0
HAVE_GSTREAMER := $(shell $(PKG_CONFIG) --exists $(GST_PKG_DEPS) && printf 1 || printf 0)

SUPERVISOR := network_display_receiver_supervisor.py
USB_GADGET_SETUP := setup_usb_network_gadget.sh
EXAMPLE_CONFIG := network_display_receivers.example.json
SCENE_DEMO_TARGET := breezy_drm_scene_demo
SCENE_DEMO_SOURCES := breezy_drm_scene_demo.c
DRM_CFLAGS ?= $(shell $(PKG_CONFIG) --cflags libdrm gbm egl glesv2)
DRM_LIBS ?= $(shell $(PKG_CONFIG) --libs libdrm gbm egl glesv2)
SCENE_DEMO_CPPFLAGS :=
SCENE_DEMO_CFLAGS :=
SCENE_DEMO_LIBS :=

ifeq ($(HAVE_GSTREAMER),1)
SCENE_DEMO_CPPFLAGS += -DBREEZY_HAVE_GSTREAMER=1
SCENE_DEMO_CFLAGS += $(shell $(PKG_CONFIG) --cflags $(GST_PKG_DEPS))
SCENE_DEMO_LIBS += $(shell $(PKG_CONFIG) --libs $(GST_PKG_DEPS))
endif

TARGETS := $(SCENE_DEMO_TARGET)

all: $(TARGETS)

$(SCENE_DEMO_TARGET): $(SCENE_DEMO_SOURCES)
	$(CC) $(CPPFLAGS) $(SCENE_DEMO_CPPFLAGS) $(CFLAGS) $(SCENE_DEMO_CFLAGS) $(DRM_CFLAGS) -o $@ $(SCENE_DEMO_SOURCES) $(DRM_LIBS) $(SCENE_DEMO_LIBS) -lm

validate: $(SCENE_DEMO_TARGET)
	$(PYTHON) -m py_compile $(SUPERVISOR)

dry-run-example:
	$(PYTHON) ./$(SUPERVISOR) ./$(EXAMPLE_CONFIG) --dry-run

clean:
	rm -f $(TARGETS)
	rm -rf __pycache__

.PHONY: all validate dry-run-example clean