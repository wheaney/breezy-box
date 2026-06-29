CC ?= cc
DOTNET ?= $(shell command -v dotnet 2>/dev/null || echo $(HOME)/.dotnet/dotnet)
PKG_CONFIG ?= pkg-config
PYTHON3 ?= python3
PROFILE ?= 0
CPPFLAGS ?=
CFLAGS ?= -O2 -g -std=c11 -Wall -Wextra -Wpedantic
PROFILE_CFLAGS :=
PROFILE_SUFFIX :=
ifeq ($(PROFILE),1)
	PROFILE_CFLAGS += -fno-omit-frame-pointer -fno-optimize-sibling-calls
	PROFILE_SUFFIX := -profile
endif
COMPAT_DIR := compat
UNAME_M := $(shell uname -m)
ifeq ($(UNAME_M),aarch64)
  DOTNET_RID := linux-arm64
else
  DOTNET_RID := linux-x64
endif
ZEROKVM_BRIDGE_LIBDIR := $(abspath modules/ZeroKVM/src/ZeroKvm.NativeBridge/bin/Release/net10.0/$(DOTNET_RID)/publish)
ZEROKVM_BRIDGE_INCLUDEDIR := $(abspath modules/ZeroKVM/src/ZeroKvm.NativeBridge)
ZEROKVM_LIB := $(ZEROKVM_BRIDGE_LIBDIR)/ZeroKvm.NativeBridge.so
DRM_CFLAGS ?= $(shell $(PKG_CONFIG) --cflags libdrm)
DRM_LIBS ?= $(shell $(PKG_CONFIG) --libs libdrm)
GBM_LIBS ?= $(shell $(PKG_CONFIG) --libs gbm)
EGL_LIBS ?= $(shell $(PKG_CONFIG) --libs egl)
GLES2_LIBS ?= $(shell $(PKG_CONFIG) --libs glesv2)
JSON_C_CFLAGS ?= $(shell $(PKG_CONFIG) --cflags json-c)
JSON_C_LIBS ?= $(shell $(PKG_CONFIG) --libs json-c)
GIO_CFLAGS ?= $(shell $(PKG_CONFIG) --cflags gio-2.0)
GIO_LIBS ?= $(shell $(PKG_CONFIG) --libs gio-2.0)
SRC_DIR := src
JS_BUNDLE := $(SRC_DIR)/display_placement_bundle.h
JS_SHARED_DIR := shared
QUICKJS_DIR := modules/quickjs
QUICKJS_VERSION := $(shell cat $(QUICKJS_DIR)/VERSION 2>/dev/null || echo unknown)
QUICKJS_DEFS := -D_GNU_SOURCE -DCONFIG_VERSION=\"$(QUICKJS_VERSION)\"
QUICKJS_SRCS := $(QUICKJS_DIR)/quickjs.c $(QUICKJS_DIR)/dtoa.c $(QUICKJS_DIR)/libregexp.c \
                $(QUICKJS_DIR)/libunicode.c $(QUICKJS_DIR)/cutils.c
QUICKJS_OBJDIR := $(QUICKJS_DIR)/.obj$(PROFILE_SUFFIX)
QUICKJS_OBJS := $(patsubst $(QUICKJS_DIR)/%.c,$(QUICKJS_OBJDIR)/%.o,$(QUICKJS_SRCS))

KMS_RENDERER_TARGET_BASE := displaylink_kms_renderer
KMS_RENDERER_PROFILE_TARGET := $(KMS_RENDERER_TARGET_BASE)-profile
KMS_RENDERER_TARGET := $(KMS_RENDERER_TARGET_BASE)$(PROFILE_SUFFIX)
COMPAT_SOURCES := $(COMPAT_DIR)/src/udl_sink.c
KMS_COMPOSITOR_SOURCES := $(addprefix $(SRC_DIR)/,displaylink_kms_renderer.c display_renderer.c overlay_text.c breezy_state.c usbip.c udl_device.c udl_runtime.c server.c display_placement.c smooth_follow.c breezy_imu.c breezy_settings.c breezy_driver_control.c usb_gadget.c link_services.c ffs_gadget.c raw_gadget.c) $(COMPAT_SOURCES)

CPPFLAGS += -I$(COMPAT_DIR)/include -I$(ZEROKVM_BRIDGE_INCLUDEDIR) -I$(SRC_DIR)
LDFLAGS += -L$(ZEROKVM_BRIDGE_LIBDIR) -Wl,-rpath,$(ZEROKVM_BRIDGE_LIBDIR)
LDLIBS += -l:ZeroKvm.NativeBridge.so

WEB_SERVER_TARGET := breezy_web
WEB_SERVER_SOURCES := $(SRC_DIR)/breezy_web.c $(SRC_DIR)/vendor/mongoose.c
MONGOOSE_H := $(SRC_DIR)/vendor/mongoose.h
MONGOOSE_C := $(SRC_DIR)/vendor/mongoose.c
MONGOOSE_URL := https://raw.githubusercontent.com/cesanta/mongoose/master

all: $(KMS_RENDERER_TARGET) $(WEB_SERVER_TARGET)

profile:
	$(MAKE) PROFILE=1 $(KMS_RENDERER_PROFILE_TARGET)

# FORCE causes dotnet publish to run on every make invocation; the C targets only
# relink when the .so is actually newer than the binary.
$(ZEROKVM_LIB): FORCE
	$(DOTNET) publish modules/ZeroKVM/src/ZeroKvm.NativeBridge/ZeroKvm.NativeBridge.csproj \
		-r $(DOTNET_RID) -c Release

# QuickJS uses GNU C extensions; compile with gnu11 and suppressed diagnostic noise
$(QUICKJS_OBJDIR)/%.o: $(QUICKJS_DIR)/%.c | $(QUICKJS_OBJDIR)
	$(CC) $(filter-out -Wpedantic -std=%,$(CFLAGS)) $(PROFILE_CFLAGS) -std=gnu11 \
		-Wno-sign-compare -Wno-missing-field-initializers -Wno-unused-parameter \
		$(QUICKJS_DEFS) -I$(QUICKJS_DIR) -c -o $@ $<

$(QUICKJS_OBJDIR):
	mkdir -p $@

$(KMS_RENDERER_TARGET): $(KMS_COMPOSITOR_SOURCES) $(ZEROKVM_LIB) $(JS_BUNDLE) $(QUICKJS_OBJS)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(PROFILE_CFLAGS) $(DRM_CFLAGS) $(JSON_C_CFLAGS) $(GIO_CFLAGS) -I$(QUICKJS_DIR) -pthread \
		-o $@ $(KMS_COMPOSITOR_SOURCES) $(QUICKJS_OBJS) \
		$(DRM_LIBS) $(GBM_LIBS) $(EGL_LIBS) $(GLES2_LIBS) $(JSON_C_LIBS) $(GIO_LIBS) -lm $(LDFLAGS) $(LDLIBS)

$(JS_BUNDLE): $(JS_SHARED_DIR)/math.js $(JS_SHARED_DIR)/displayPlacement.js $(JS_SHARED_DIR)/smoothFollow.js $(JS_SHARED_DIR)/zoomOnFocus.js tools/gen_js_bundle.py
	$(PYTHON3) tools/gen_js_bundle.py $(JS_SHARED_DIR)/math.js $(JS_SHARED_DIR)/displayPlacement.js $(JS_SHARED_DIR)/smoothFollow.js $(JS_SHARED_DIR)/zoomOnFocus.js > $(SRC_DIR)/display_placement_bundle.h.tmp
	@if cmp -s $(SRC_DIR)/display_placement_bundle.h.tmp $@ 2>/dev/null; then rm -f $(SRC_DIR)/display_placement_bundle.h.tmp; else mv $(SRC_DIR)/display_placement_bundle.h.tmp $@; fi

deps: $(MONGOOSE_H) $(MONGOOSE_C)

$(MONGOOSE_H) $(MONGOOSE_C):
	@mkdir -p $(SRC_DIR)/vendor
	curl -fsSL $(MONGOOSE_URL)/mongoose.h -o $(MONGOOSE_H)
	curl -fsSL $(MONGOOSE_URL)/mongoose.c -o $(MONGOOSE_C)

$(WEB_SERVER_TARGET): $(WEB_SERVER_SOURCES)
	$(CC) $(CFLAGS) -D_POSIX_C_SOURCE=200809L -D_DEFAULT_SOURCE -DMG_ENABLE_EPOLL=1 -DMG_TLS=MG_TLS_BUILTIN -I$(SRC_DIR) $(JSON_C_CFLAGS) -o $@ $(WEB_SERVER_SOURCES) $(JSON_C_LIBS)

clean:
	rm -f $(KMS_RENDERER_TARGET_BASE) $(KMS_RENDERER_PROFILE_TARGET) $(JS_BUNDLE) $(WEB_SERVER_TARGET)
	rm -rf $(QUICKJS_DIR)/.obj $(QUICKJS_DIR)/.obj-profile

.PHONY: all clean deps FORCE profile
