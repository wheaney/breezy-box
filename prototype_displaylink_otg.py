#!/usr/bin/env python3
"""
Prototype Breezy Box pipeline:

1. Configure an OTG USB gadget via configfs, matching the user's ACM-based
   shell steps.
2. Read bytes from the gadget-side endpoint (or stdin / a capture file).
3. Decode the older DisplayLink UDL bulk protocol used by the Linux UDL
   drivers for 16-bit framebuffer updates.
4. Upload the decoded framebuffer into an OpenGL texture and draw it on a
   rotating quad in a simple 3D viewport.

Important limitation:
The gadget profile below intentionally mirrors the supplied shell steps and
creates a CDC ACM function on /dev/ttyGS0. Stock DisplayLink host drivers do
not bind to CDC ACM; they match a vendor-defined interface class with bulk and
control endpoints. This script is therefore a transport prototype for getting
UDL packets into userspace, not a full hardware-faithful DisplayLink gadget.
"""

from __future__ import annotations

import argparse
import ctypes
import logging
import math
import os
import selectors
import struct
import subprocess
import sys
import termios
import threading
import time
import tty as tty_module
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterator, Optional


LOGGER = logging.getLogger("breezy_box.displaylink")

UDL_MSG_BULK = 0xAF

UDL_CMD_WRITEREG = 0x20
UDL_CMD_WRITERAW16 = 0x68
UDL_CMD_WRITERL16 = 0x69
UDL_CMD_WRITECOPY16 = 0x6A
UDL_CMD_WRITERLX16 = 0x6B

UDL_REG_COLORDEPTH = 0x00
UDL_REG_HPIXELS = 0x0F
UDL_REG_VPIXELS = 0x17
UDL_REG_PIXELCLOCK5KHZ = 0x1B
UDL_REG_BLANKMODE = 0x1F
UDL_REG_BASE16BPP_ADDR2 = 0x20
UDL_REG_BASE16BPP_ADDR1 = 0x21
UDL_REG_BASE16BPP_ADDR0 = 0x22
UDL_REG_BASE8BPP_ADDR2 = 0x26
UDL_REG_BASE8BPP_ADDR1 = 0x27
UDL_REG_BASE8BPP_ADDR0 = 0x28
UDL_REG_VIDREG = 0xFF

UDL_BLANKMODE_NAMES = {
    0x00: "on",
    0x01: "blanked",
    0x03: "vsync-off",
    0x05: "hsync-off",
    0x07: "powerdown",
}


class GadgetError(RuntimeError):
    """Raised when configfs gadget setup fails."""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Prototype DisplayLink-over-OTG receiver for Breezy Box. "
            "It can set up the ACM gadget, ingest UDL bulk packets, and "
            "render the decoded framebuffer on a textured quad."
        )
    )
    parser.add_argument(
        "--source",
        choices=("tty", "file", "stdin", "demo"),
        default="tty",
        help="Where to read the incoming byte stream from.",
    )
    parser.add_argument(
        "--source-file",
        type=Path,
        help="Binary capture file to decode when --source=file.",
    )
    parser.add_argument(
        "--tty-path",
        type=Path,
        default=Path("/dev/ttyGS0"),
        help="Gadget-side tty device exposed by the ACM function.",
    )
    parser.add_argument(
        "--chunk-size",
        type=int,
        default=64 * 1024,
        help="Maximum read size for each transport chunk.",
    )
    parser.add_argument(
        "--width",
        type=int,
        default=1024,
        help="Fallback framebuffer width before a mode write arrives.",
    )
    parser.add_argument(
        "--height",
        type=int,
        default=768,
        help="Fallback framebuffer height before a mode write arrives.",
    )
    parser.add_argument(
        "--window-width",
        type=int,
        default=1600,
        help="Viewport window width.",
    )
    parser.add_argument(
        "--window-height",
        type=int,
        default=900,
        help="Viewport window height.",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Decode and log status without opening the 3D viewport.",
    )
    parser.add_argument(
        "--setup-only",
        action="store_true",
        help="Only configure the gadget and exit.",
    )
    parser.add_argument(
        "--no-gadget-setup",
        action="store_true",
        help="Skip configfs gadget setup and only run the ingest / render path.",
    )
    parser.add_argument(
        "--unbind-on-exit",
        action="store_true",
        help="Unbind the gadget from the UDC before exiting.",
    )
    parser.add_argument(
        "--usb-gadget-root",
        type=Path,
        default=Path("/sys/kernel/config/usb_gadget"),
        help="Configfs usb_gadget root.",
    )
    parser.add_argument(
        "--gadget-name",
        default="g1",
        help="Configfs gadget directory name.",
    )
    parser.add_argument(
        "--udc",
        default="fe800000.usb",
        help="UDC name to bind the gadget to.",
    )
    parser.add_argument(
        "--vendor-id",
        default="0x17e9",
        help="USB vendor ID to write into idVendor.",
    )
    parser.add_argument(
        "--product-id",
        default=None,
        help="Optional USB product ID. A fallback is used if configfs needs one.",
    )
    parser.add_argument(
        "--manufacturer",
        default="Breezy Box",
        help="Optional manufacturer string descriptor.",
    )
    parser.add_argument(
        "--product",
        default="DisplayLink OTG Prototype",
        help="Optional product string descriptor.",
    )
    parser.add_argument(
        "--serial-number",
        default="breezy-box-prototype",
        help="Optional serial number string descriptor.",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Enable verbose logging.",
    )
    return parser.parse_args()


def configure_logging(verbose: bool) -> None:
    logging.basicConfig(
        level=logging.DEBUG if verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
        datefmt="%H:%M:%S",
    )


def require_root() -> None:
    if os.geteuid() != 0:
        raise GadgetError("Run with sudo when gadget setup is enabled.")


def write_text_file(path: Path, value: str) -> None:
    path.write_text(value, encoding="ascii")


def read_text_file(path: Path) -> str:
    return path.read_text(encoding="ascii").strip()


def ensure_symlink(link_path: Path, target: str) -> None:
    if link_path.is_symlink():
        if os.readlink(link_path) == target:
            return
        link_path.unlink()
    elif link_path.exists():
        raise GadgetError(f"Expected symlink at {link_path}, found another entry.")
    os.symlink(target, link_path)


def load_kernel_module(module_name: str) -> None:
    LOGGER.info("Loading kernel module %s", module_name)
    subprocess.run(["modprobe", module_name], check=True)


@dataclass(frozen=True)
class GadgetConfig:
    root: Path
    gadget_name: str
    udc_name: str
    vendor_id: str
    product_id: Optional[str]
    manufacturer: str
    product: str
    serial_number: str
    max_power_ma: int = 250
    configuration_label: str = "DisplayLink"
    language: str = "0x409"
    function_name: str = "acm.usb0"
    config_name: str = "c.1"

    @property
    def gadget_dir(self) -> Path:
        return self.root / self.gadget_name

    @property
    def function_dir(self) -> Path:
        return self.gadget_dir / "functions" / self.function_name

    @property
    def config_dir(self) -> Path:
        return self.gadget_dir / "configs" / self.config_name

    @property
    def gadget_strings_dir(self) -> Path:
        return self.gadget_dir / "strings" / self.language

    @property
    def config_strings_dir(self) -> Path:
        return self.config_dir / "strings" / self.language

    @property
    def function_link(self) -> Path:
        return self.config_dir / self.function_name


def setup_displaylink_acm_gadget(config: GadgetConfig) -> None:
    require_root()
    if not config.root.exists():
        raise GadgetError(f"Configfs gadget root does not exist: {config.root}")

    load_kernel_module("libcomposite")

    config.gadget_dir.mkdir(exist_ok=True)
    write_text_file(config.gadget_dir / "idVendor", config.vendor_id)

    product_id = config.product_id
    if product_id is None:
        product_file = config.gadget_dir / "idProduct"
        current_product = read_text_file(product_file) if product_file.exists() else ""
        product_id = current_product or "0x0100"
        LOGGER.warning(
            "No idProduct was supplied. Using %s so the gadget can bind cleanly.",
            product_id,
        )
    write_text_file(config.gadget_dir / "idProduct", product_id)

    config.function_dir.mkdir(parents=True, exist_ok=True)
    config.config_strings_dir.mkdir(parents=True, exist_ok=True)
    config.gadget_strings_dir.mkdir(parents=True, exist_ok=True)

    write_text_file(config.config_dir / "MaxPower", str(config.max_power_ma))
    write_text_file(
        config.config_strings_dir / "configuration",
        config.configuration_label,
    )
    write_text_file(config.gadget_strings_dir / "manufacturer", config.manufacturer)
    write_text_file(config.gadget_strings_dir / "product", config.product)
    write_text_file(config.gadget_strings_dir / "serialnumber", config.serial_number)

    link_target = os.path.relpath(config.function_dir, config.config_dir)
    ensure_symlink(config.function_link, link_target)

    current_udc = read_text_file(config.gadget_dir / "UDC") if (config.gadget_dir / "UDC").exists() else ""
    if current_udc and current_udc != config.udc_name:
        LOGGER.info("Rebinding gadget from %s to %s", current_udc, config.udc_name)
        write_text_file(config.gadget_dir / "UDC", "")

    write_text_file(config.gadget_dir / "UDC", config.udc_name)
    LOGGER.warning(
        "The ACM function mirrors the shell steps, but stock DisplayLink drivers "
        "expect a vendor-defined interface class with bulk/control endpoints, not CDC ACM."
    )


def unbind_displaylink_gadget(config: GadgetConfig) -> None:
    require_root()
    udc_file = config.gadget_dir / "UDC"
    if udc_file.exists():
        LOGGER.info("Unbinding gadget %s", config.gadget_name)
        write_text_file(udc_file, "")


def be24(data: bytes) -> int:
    return (data[0] << 16) | (data[1] << 8) | data[2]


def rgb565_be_to_le(data: bytes) -> bytearray:
    swapped = bytearray(len(data))
    swapped[0::2] = data[1::2]
    swapped[1::2] = data[0::2]
    return swapped


class FrameBuffer:
    def __init__(self, width: int, height: int) -> None:
        self.width = max(1, width)
        self.height = max(1, height)
        self.buffer = bytearray(self.width * self.height * 2)
        self.lock = threading.Lock()
        self.version = 0
        self.last_update = time.monotonic()

    @property
    def size_bytes(self) -> int:
        return len(self.buffer)

    def resize(self, width: int, height: int) -> None:
        width = max(1, width)
        height = max(1, height)
        with self.lock:
            if width == self.width and height == self.height:
                return
            self.width = width
            self.height = height
            self.buffer = bytearray(width * height * 2)
            self.version += 1
            self.last_update = time.monotonic()
        LOGGER.info("Framebuffer resized to %dx%d", width, height)

    def write_pixels(self, device_addr: int, pixels_le: bytes) -> None:
        if device_addr < 0 or not pixels_le:
            return
        with self.lock:
            if device_addr >= len(self.buffer):
                LOGGER.debug("Dropping write at 0x%x beyond framebuffer", device_addr)
                return
            max_len = min(len(pixels_le), len(self.buffer) - device_addr)
            self.buffer[device_addr : device_addr + max_len] = pixels_le[:max_len]
            self.version += 1
            self.last_update = time.monotonic()

    def copy_pixels(self, src_addr: int, pixel_count: int, dst_addr: int) -> None:
        if pixel_count <= 0:
            return
        byte_count = pixel_count * 2
        with self.lock:
            if src_addr < 0 or dst_addr < 0:
                return
            if src_addr >= len(self.buffer) or dst_addr >= len(self.buffer):
                return
            byte_count = min(byte_count, len(self.buffer) - src_addr, len(self.buffer) - dst_addr)
            if byte_count <= 0:
                return
            self.buffer[dst_addr : dst_addr + byte_count] = self.buffer[src_addr : src_addr + byte_count]
            self.version += 1
            self.last_update = time.monotonic()

    def snapshot(self) -> tuple[int, int, bytes, int]:
        with self.lock:
            return self.width, self.height, bytes(self.buffer), self.version

    def fill_test_pattern(self) -> None:
        with self.lock:
            for y_coord in range(self.height):
                for x_coord in range(self.width):
                    red = (x_coord * 31) // max(1, self.width - 1)
                    green = (y_coord * 63) // max(1, self.height - 1)
                    blue = ((x_coord // 32) ^ (y_coord // 32)) & 0x1F
                    pixel = (red << 11) | (green << 5) | blue
                    struct.pack_into("<H", self.buffer, (y_coord * self.width + x_coord) * 2, pixel)
            self.version += 1
            self.last_update = time.monotonic()


class DisplayLinkState:
    def __init__(self, width: int, height: int) -> None:
        self.framebuffer = FrameBuffer(width, height)
        self.registers = bytearray(256)
        self.bytes_received = 0
        self.commands_decoded = 0
        self.decode_errors = 0
        self.color_depth = 16
        self.base16 = 0
        self.base8 = 0
        self.pixel_clock_5khz = 0
        self.blank_mode = 0x00

    def set_register(self, register: int, value: int) -> None:
        self.registers[register] = value & 0xFF
        if register == UDL_REG_COLORDEPTH:
            self.color_depth = 16 if value == 0 else 24
        elif register in (UDL_REG_BASE16BPP_ADDR2, UDL_REG_BASE16BPP_ADDR1, UDL_REG_BASE16BPP_ADDR0):
            self.base16 = (
                (self.registers[UDL_REG_BASE16BPP_ADDR2] << 16)
                | (self.registers[UDL_REG_BASE16BPP_ADDR1] << 8)
                | self.registers[UDL_REG_BASE16BPP_ADDR0]
            )
        elif register in (UDL_REG_BASE8BPP_ADDR2, UDL_REG_BASE8BPP_ADDR1, UDL_REG_BASE8BPP_ADDR0):
            self.base8 = (
                (self.registers[UDL_REG_BASE8BPP_ADDR2] << 16)
                | (self.registers[UDL_REG_BASE8BPP_ADDR1] << 8)
                | self.registers[UDL_REG_BASE8BPP_ADDR0]
            )
        elif register in (UDL_REG_PIXELCLOCK5KHZ, UDL_REG_PIXELCLOCK5KHZ + 1):
            self.pixel_clock_5khz = (
                self.registers[UDL_REG_PIXELCLOCK5KHZ]
                | (self.registers[UDL_REG_PIXELCLOCK5KHZ + 1] << 8)
            )
        elif register == UDL_REG_BLANKMODE:
            self.blank_mode = value

        if register in (UDL_REG_HPIXELS, UDL_REG_HPIXELS + 1, UDL_REG_VPIXELS, UDL_REG_VPIXELS + 1):
            width = (self.registers[UDL_REG_HPIXELS] << 8) | self.registers[UDL_REG_HPIXELS + 1]
            height = (self.registers[UDL_REG_VPIXELS] << 8) | self.registers[UDL_REG_VPIXELS + 1]
            if width and height:
                self.framebuffer.resize(width, height)

    def status_line(self) -> str:
        blank = UDL_BLANKMODE_NAMES.get(self.blank_mode, f"0x{self.blank_mode:02x}")
        return (
            f"bytes={self.bytes_received} commands={self.commands_decoded} "
            f"errors={self.decode_errors} mode={self.framebuffer.width}x{self.framebuffer.height} "
            f"depth={self.color_depth} blank={blank}"
        )


class UDLStreamDecoder:
    def __init__(self, state: DisplayLinkState) -> None:
        self.state = state
        self.pending = bytearray()

    def feed(self, chunk: bytes) -> None:
        if not chunk:
            return
        self.state.bytes_received += len(chunk)
        self.pending.extend(chunk)

        while True:
            if not self.pending:
                return

            sync_index = self.pending.find(bytes((UDL_MSG_BULK,)))
            if sync_index < 0:
                self.pending.clear()
                return
            if sync_index > 0:
                del self.pending[:sync_index]

            if len(self.pending) == 1:
                return

            command = self.pending[1]
            if command == UDL_MSG_BULK:
                del self.pending[:1]
                continue

            if command == UDL_CMD_WRITEREG:
                if len(self.pending) < 4:
                    return
                register = self.pending[2]
                value = self.pending[3]
                self.state.set_register(register, value)
                self.state.commands_decoded += 1
                del self.pending[:4]
                continue

            if command == UDL_CMD_WRITERAW16:
                if len(self.pending) < 6:
                    return
                pixel_count = self.pending[5] or 256
                packet_len = 6 + pixel_count * 2
                if len(self.pending) < packet_len:
                    return
                device_addr = be24(self.pending[2:5])
                payload = rgb565_be_to_le(self.pending[6:packet_len])
                self.state.framebuffer.write_pixels(device_addr, payload)
                self.state.commands_decoded += 1
                del self.pending[:packet_len]
                continue

            if command == UDL_CMD_WRITERL16:
                if len(self.pending) < 8:
                    return
                device_addr = be24(self.pending[2:5])
                pixel_count = self.pending[5] or 256
                pixel_le = rgb565_be_to_le(self.pending[6:8])
                payload = pixel_le * pixel_count
                self.state.framebuffer.write_pixels(device_addr, payload)
                self.state.commands_decoded += 1
                del self.pending[:8]
                continue

            if command == UDL_CMD_WRITECOPY16:
                if len(self.pending) < 9:
                    return
                src_addr = be24(self.pending[2:5])
                pixel_count = self.pending[5] or 256
                dst_addr = be24(self.pending[6:9])
                self.state.framebuffer.copy_pixels(src_addr, pixel_count, dst_addr)
                self.state.commands_decoded += 1
                del self.pending[:9]
                continue

            if command == UDL_CMD_WRITERLX16:
                if not self._decode_writerlx16():
                    return
                continue

            self.state.decode_errors += 1
            LOGGER.debug("Unsupported UDL command 0x%02x, resyncing", command)
            del self.pending[:1]

    def _decode_writerlx16(self) -> bool:
        if len(self.pending) < 7:
            return False

        device_addr = be24(self.pending[2:5])
        total_pixels = self.pending[5] or 256
        position = 6
        pixels_remaining = total_pixels
        decoded = bytearray()

        while pixels_remaining > 0:
            if len(self.pending) <= position:
                return False

            raw_pixels = self.pending[position] or 256
            position += 1
            raw_bytes = raw_pixels * 2

            if len(self.pending) < position + raw_bytes:
                return False

            raw_segment = rgb565_be_to_le(self.pending[position : position + raw_bytes])
            decoded.extend(raw_segment)
            position += raw_bytes
            pixels_remaining -= raw_pixels

            if pixels_remaining <= 0:
                break

            if len(self.pending) <= position:
                return False

            repeat_extra = self.pending[position] + 1
            position += 1

            if len(raw_segment) < 2 or repeat_extra > pixels_remaining:
                self.state.decode_errors += 1
                LOGGER.debug("Malformed WRITERLX16 span, abandoning packet")
                del self.pending[:1]
                return True

            decoded.extend(raw_segment[-2:] * repeat_extra)
            pixels_remaining -= repeat_extra

        self.state.framebuffer.write_pixels(device_addr, decoded)
        self.state.commands_decoded += 1
        del self.pending[:position]
        return True


class InputPump(threading.Thread):
    def __init__(
        self,
        name: str,
        iterator_factory: Callable[[threading.Event], Iterator[bytes]],
        decoder: UDLStreamDecoder,
        stop_event: threading.Event,
    ) -> None:
        super().__init__(name=name, daemon=True)
        self.iterator_factory = iterator_factory
        self.decoder = decoder
        self.stop_event = stop_event
        self.error: Optional[BaseException] = None

    def run(self) -> None:
        try:
            for chunk in self.iterator_factory(self.stop_event):
                if self.stop_event.is_set():
                    return
                self.decoder.feed(chunk)
        except BaseException as exc:  # pragma: no cover - surfaced to main thread
            self.error = exc
            self.stop_event.set()


def iter_tty_chunks(path: Path, chunk_size: int, stop_event: threading.Event) -> Iterator[bytes]:
    if not path.exists():
        raise FileNotFoundError(
            f"Transport device {path} does not exist. The ACM gadget profile should expose it when bound."
        )

    file_descriptor = os.open(path, os.O_RDWR | os.O_NONBLOCK)
    selector = selectors.DefaultSelector()
    original_termios = termios.tcgetattr(file_descriptor)
    tty_module.setraw(file_descriptor, when=termios.TCSANOW)
    selector.register(file_descriptor, selectors.EVENT_READ)

    try:
        while not stop_event.is_set():
            ready = selector.select(timeout=0.1)
            if not ready:
                continue
            try:
                data = os.read(file_descriptor, chunk_size)
            except BlockingIOError:
                continue
            if data:
                yield data
    finally:
        selector.close()
        termios.tcsetattr(file_descriptor, termios.TCSANOW, original_termios)
        os.close(file_descriptor)


def iter_file_chunks(path: Path, chunk_size: int, stop_event: threading.Event) -> Iterator[bytes]:
    with path.open("rb") as handle:
        while not stop_event.is_set():
            data = handle.read(chunk_size)
            if not data:
                return
            yield data


def iter_stdin_chunks(chunk_size: int, stop_event: threading.Event) -> Iterator[bytes]:
    while not stop_event.is_set():
        data = os.read(sys.stdin.fileno(), chunk_size)
        if not data:
            return
        yield data


def make_iterator_factory(args: argparse.Namespace) -> Optional[Callable[[threading.Event], Iterator[bytes]]]:
    if args.source == "tty":
        return lambda stop_event: iter_tty_chunks(args.tty_path, args.chunk_size, stop_event)
    if args.source == "file":
        if not args.source_file:
            raise ValueError("--source-file is required when --source=file")
        return lambda stop_event: iter_file_chunks(args.source_file, args.chunk_size, stop_event)
    if args.source == "stdin":
        return lambda stop_event: iter_stdin_chunks(args.chunk_size, stop_event)
    return None


VERTEX_SHADER_SOURCE = """
#version 330 core

in vec3 position;
in vec2 texcoord;

out vec2 uv;

uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;

void main() {
    uv = vec2(texcoord.x, 1.0 - texcoord.y);
    gl_Position = projection * view * model * vec4(position, 1.0);
}
"""


FRAGMENT_SHADER_SOURCE = """
#version 330 core

in vec2 uv;
out vec4 frag_color;

uniform sampler2D frame_texture;
uniform float blank_factor;

void main() {
    vec3 color = texture(frame_texture, uv).rgb;
    frag_color = vec4(color * blank_factor, 1.0);
}
"""


def run_viewport(state: DisplayLinkState, stop_event: threading.Event, args: argparse.Namespace) -> None:
    try:
        import pyglet
        from pyglet.gl import (
            GL_LINEAR,
            GL_RGB,
            GL_TEXTURE0,
            GL_TEXTURE_2D,
            GL_TEXTURE_MAG_FILTER,
            GL_TEXTURE_MIN_FILTER,
            GL_TRIANGLES,
            GL_UNSIGNED_SHORT_5_6_5,
            GLuint,
            glActiveTexture,
            glBindTexture,
            glClearColor,
            glEnable,
            glGenTextures,
            glPixelStorei,
            glTexImage2D,
            glTexParameteri,
            glTexSubImage2D,
        )
        from pyglet.graphics.shader import Shader, ShaderProgram
        from pyglet.math import Mat4, Vec3
    except ModuleNotFoundError as exc:  # pragma: no cover - depends on local environment
        raise RuntimeError(
            "pyglet is required for viewport rendering. Install it with 'pip install pyglet'."
        ) from exc

    pyglet.options["shadow_window"] = False

    class DisplayViewportWindow(pyglet.window.Window):
        def __init__(self) -> None:
            super().__init__(
                width=args.window_width,
                height=args.window_height,
                caption="Breezy Box DisplayLink Prototype",
                resizable=True,
                vsync=True,
            )
            glClearColor(0.06, 0.08, 0.11, 1.0)
            glEnable(pyglet.gl.GL_DEPTH_TEST)

            self.program = ShaderProgram(
                Shader(VERTEX_SHADER_SOURCE, "vertex"),
                Shader(FRAGMENT_SHADER_SOURCE, "fragment"),
            )

            self.vertex_list = self.program.vertex_list_indexed(
                4,
                GL_TRIANGLES,
                [0, 1, 2, 0, 2, 3],
                position=(
                    "f",
                    [
                        -1.0,
                        -0.5625,
                        0.0,
                        1.0,
                        -0.5625,
                        0.0,
                        1.0,
                        0.5625,
                        0.0,
                        -1.0,
                        0.5625,
                        0.0,
                    ],
                ),
                texcoord=("f", [0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0]),
            )

            self.texture_id = GLuint()
            glGenTextures(1, ctypes.byref(self.texture_id))
            glBindTexture(GL_TEXTURE_2D, self.texture_id)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
            glPixelStorei(pyglet.gl.GL_UNPACK_ALIGNMENT, 1)

            width, height, pixels, version = state.framebuffer.snapshot()
            self.texture_size = (0, 0)
            self.frame_version = -1
            self.status_label = pyglet.text.Label(
                "",
                x=12,
                y=self.height - 12,
                anchor_x="left",
                anchor_y="top",
                multiline=True,
                width=max(200, self.width - 24),
            )
            self.projection = Mat4.perspective_projection(
                aspect=self.width / self.height,
                z_near=0.1,
                z_far=100.0,
                fov=60.0,
            )
            self.view = Mat4.from_translation(Vec3(0.0, 0.0, -3.0))
            self._upload_texture(width, height, pixels)
            self.frame_version = version

        def _upload_texture(self, width: int, height: int, pixel_bytes: bytes) -> None:
            glActiveTexture(GL_TEXTURE0)
            glBindTexture(GL_TEXTURE_2D, self.texture_id)
            buffer_view = (ctypes.c_ubyte * len(pixel_bytes)).from_buffer_copy(pixel_bytes)
            if self.texture_size != (width, height):
                glTexImage2D(
                    GL_TEXTURE_2D,
                    0,
                    GL_RGB,
                    width,
                    height,
                    0,
                    GL_RGB,
                    GL_UNSIGNED_SHORT_5_6_5,
                    buffer_view,
                )
                self.texture_size = (width, height)
            else:
                glTexSubImage2D(
                    GL_TEXTURE_2D,
                    0,
                    0,
                    0,
                    width,
                    height,
                    GL_RGB,
                    GL_UNSIGNED_SHORT_5_6_5,
                    buffer_view,
                )

        def on_resize(self, width: int, height: int) -> None:
            super().on_resize(width, height)
            safe_height = max(1, height)
            self.projection = Mat4.perspective_projection(
                aspect=width / safe_height,
                z_near=0.1,
                z_far=100.0,
                fov=60.0,
            )
            self.status_label.y = height - 12
            self.status_label.width = max(200, width - 24)

        def on_close(self) -> None:
            stop_event.set()
            super().on_close()

        def on_draw(self) -> None:
            self.clear()

            width, height, pixels, version = state.framebuffer.snapshot()
            if version != self.frame_version or self.texture_size != (width, height):
                self._upload_texture(width, height, pixels)
                self.frame_version = version

            angle = time.monotonic() * 0.35
            model = (
                Mat4.from_translation(Vec3(0.0, 0.0, 0.0))
                @ Mat4.from_rotation(angle, Vec3(0.0, 1.0, 0.0))
                @ Mat4.from_rotation(-0.2, Vec3(1.0, 0.0, 0.0))
            )

            self.program.use()
            self.program["projection"] = self.projection
            self.program["view"] = self.view
            self.program["model"] = model
            self.program["frame_texture"] = 0
            self.program["blank_factor"] = 0.2 if state.blank_mode == 0x07 else 1.0

            glActiveTexture(GL_TEXTURE0)
            glBindTexture(GL_TEXTURE_2D, self.texture_id)
            self.vertex_list.draw(GL_TRIANGLES)

            self.status_label.text = (
                "Breezy Box DisplayLink OTG Prototype\n"
                f"{state.status_line()}\n"
                f"base16=0x{state.base16:06x} pixel_clock={state.pixel_clock_5khz * 5} kHz"
            )
            self.status_label.draw()

    window = DisplayViewportWindow()

    def tick(_delta: float) -> None:
        if stop_event.is_set():
            window.close()

    pyglet.clock.schedule_interval(tick, 1 / 30)
    pyglet.app.run()


def run_headless_loop(pump: Optional[InputPump], state: DisplayLinkState, stop_event: threading.Event) -> None:
    try:
        while not stop_event.is_set():
            if pump and pump.error:
                raise pump.error
            LOGGER.info(state.status_line())
            if pump and not pump.is_alive():
                return
            time.sleep(1.0)
    except KeyboardInterrupt:
        stop_event.set()


def build_gadget_config(args: argparse.Namespace) -> GadgetConfig:
    return GadgetConfig(
        root=args.usb_gadget_root,
        gadget_name=args.gadget_name,
        udc_name=args.udc,
        vendor_id=args.vendor_id,
        product_id=args.product_id,
        manufacturer=args.manufacturer,
        product=args.product,
        serial_number=args.serial_number,
    )


def main() -> int:
    args = parse_args()
    configure_logging(args.verbose)

    state = DisplayLinkState(args.width, args.height)
    state.framebuffer.fill_test_pattern()

    config = build_gadget_config(args)
    gadget_configured = False

    if not args.no_gadget_setup:
        setup_displaylink_acm_gadget(config)
        gadget_configured = True

    if args.setup_only:
        return 0

    decoder = UDLStreamDecoder(state)
    stop_event = threading.Event()
    pump: Optional[InputPump] = None
    iterator_factory = make_iterator_factory(args)

    if iterator_factory is not None:
        pump = InputPump("displaylink-input", iterator_factory, decoder, stop_event)
        pump.start()
    else:
        LOGGER.info("Demo mode enabled: showing the test pattern until the window closes.")

    try:
        if args.headless:
            run_headless_loop(pump, state, stop_event)
        else:
            run_viewport(state, stop_event, args)

        if pump and pump.error:
            raise pump.error
        return 0
    finally:
        stop_event.set()
        if pump:
            pump.join(timeout=1.0)
        if args.unbind_on_exit and gadget_configured:
            unbind_displaylink_gadget(config)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except GadgetError as exc:
        LOGGER.error("%s", exc)
        raise SystemExit(1) from exc
