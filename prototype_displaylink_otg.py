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
        "--allow-root-viewport",
        action="store_true",
        help=(
            "Allow viewport creation while running as root. This is disabled by default because "
            "KDE Plasma, especially on Wayland, commonly blocks root GUI OpenGL contexts."
        ),
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
        default="0x0104",
        help="USB product ID to write into idProduct.",
    )
    parser.add_argument(
        "--max-power",
        type=int,
        default=250,
        help="MaxPower value for the gadget configuration, in mA.",
    )
    parser.add_argument(
        "--configuration-label",
        default="DisplayLink",
        help="Configuration string written under configs/c.1/strings/0x409/configuration.",
    )
    parser.add_argument(
        "--manufacturer",
        default="DisplayLink",
        help="Optional manufacturer string descriptor.",
    )
    parser.add_argument(
        "--product",
        default="DisplayLink Adapter",
        help="Optional product string descriptor.",
    )
    parser.add_argument(
        "--serial-number",
        default="DEADBEEF0001",
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
    load_kernel_module("libcomposite")
    if not config.root.exists():
        raise GadgetError(f"Configfs gadget root does not exist: {config.root}")

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

    # Configfs resolves the symlink target path when the link is created.
    # The shell recipe worked because it was run from inside the gadget
    # directory first; this script may run from anywhere, so use the absolute
    # configfs path for the function item.
    link_target = str(config.function_dir)
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
#version 120

attribute vec3 position;
attribute vec2 texcoord;

varying vec2 uv;

uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;

void main() {
    uv = vec2(texcoord.x, 1.0 - texcoord.y);
    gl_Position = projection * view * model * vec4(position, 1.0);
}
"""


FRAGMENT_SHADER_SOURCE = """
#version 120

varying vec2 uv;

uniform sampler2D frame_texture;
uniform float blank_factor;

void main() {
    vec3 color = texture2D(frame_texture, uv).rgb;
    gl_FragColor = vec4(color * blank_factor, 1.0);
}
"""


def run_viewport(state: DisplayLinkState, stop_event: threading.Event, args: argparse.Namespace) -> None:
    display_name = os.environ.get("DISPLAY")
    wayland_display = os.environ.get("WAYLAND_DISPLAY")
    xauthority = os.environ.get("XAUTHORITY")

    if os.geteuid() == 0 and not args.allow_root_viewport:
        raise RuntimeError(
            "Viewport rendering is running as root. KDE Plasma sessions, especially Wayland sessions, "
            "commonly reject root OpenGL windows and fail to create a usable GL context. "
            "Run the gadget setup as root first, then run the viewer as your logged-in desktop user. "
            "Recommended split: 'sudo python3 prototype_displaylink_otg.py --setup-only' and then "
            "'python3 prototype_displaylink_otg.py --no-gadget-setup'. If you intentionally want to try a root viewport, "
            "pass --allow-root-viewport and preserve your display environment. "
            f"Current environment: DISPLAY={display_name!r}, WAYLAND_DISPLAY={wayland_display!r}, XAUTHORITY={xauthority!r}."
        )

    if not display_name and not wayland_display:
        raise RuntimeError(
            "No desktop display session is visible to this process. A viewport requires either DISPLAY or WAYLAND_DISPLAY. "
            f"Current environment: DISPLAY={display_name!r}, WAYLAND_DISPLAY={wayland_display!r}, XAUTHORITY={xauthority!r}."
        )

    try:
        import pygame
        from pygame.locals import DOUBLEBUF, OPENGL, QUIT

        from OpenGL.GL import (
            GL_ARRAY_BUFFER,
            GL_COLOR_BUFFER_BIT,
            GL_COMPILE_STATUS,
            GL_DEPTH_BUFFER_BIT,
            GL_DEPTH_TEST,
            GL_ELEMENT_ARRAY_BUFFER,
            GL_FALSE,
            GL_FLOAT,
            GL_FRAGMENT_SHADER,
            GL_LINEAR,
            GL_LINK_STATUS,
            GL_RGB,
            GL_STATIC_DRAW,
            GL_TEXTURE0,
            GL_TEXTURE_2D,
            GL_TEXTURE_MAG_FILTER,
            GL_TEXTURE_MIN_FILTER,
            GL_TRIANGLES,
            GL_TRUE,
            GL_UNPACK_ALIGNMENT,
            GL_UNSIGNED_INT,
            GL_UNSIGNED_SHORT_5_6_5,
            GL_VERTEX_SHADER,
            glActiveTexture,
            glAttachShader,
            glBindAttribLocation,
            glBindBuffer,
            glBindTexture,
            glBufferData,
            glClear,
            glClearColor,
            glCompileShader,
            glCreateProgram,
            glCreateShader,
            glDeleteShader,
            glDisableVertexAttribArray,
            glDrawElements,
            glEnable,
            glEnableVertexAttribArray,
            glGenBuffers,
            glGenTextures,
            glGetAttribLocation,
            glGetProgramInfoLog,
            glGetProgramiv,
            glGetShaderInfoLog,
            glGetShaderiv,
            glGetString,
            glGetUniformLocation,
            glLinkProgram,
            glPixelStorei,
            glShaderSource,
            glTexImage2D,
            glTexParameteri,
            glTexSubImage2D,
            glUniform1f,
            glUniform1i,
            glUniformMatrix4fv,
            glUseProgram,
            glVertexAttribPointer,
            glViewport,
            GL_VERSION,
            GL_SHADING_LANGUAGE_VERSION,
        )
    except ModuleNotFoundError as exc:  # pragma: no cover - depends on local environment
        raise RuntimeError(
            "Viewport import failed while using interpreter "
            f"{sys.executable}. Missing module: {exc.name!r}. "
            "Install both pygame and PyOpenGL into the same interpreter environment."
        ) from exc
    except ImportError as exc:  # pragma: no cover - depends on local environment
        raise RuntimeError(
            "Viewport import failed even though pygame/PyOpenGL were found, while using interpreter "
            f"{sys.executable}: {exc}. This usually means an SDL or OpenGL runtime dependency is missing."
        ) from exc

    def perspective_matrix(fov_degrees: float, aspect: float, z_near: float, z_far: float) -> tuple[float, ...]:
        f = 1.0 / math.tan(math.radians(fov_degrees) / 2.0)
        return (
            f / aspect,
            0.0,
            0.0,
            0.0,
            0.0,
            f,
            0.0,
            0.0,
            0.0,
            0.0,
            (z_far + z_near) / (z_near - z_far),
            -1.0,
            0.0,
            0.0,
            (2.0 * z_far * z_near) / (z_near - z_far),
            0.0,
        )

    def multiply_mat4(left: tuple[float, ...], right: tuple[float, ...]) -> tuple[float, ...]:
        result = [0.0] * 16
        for row in range(4):
            for col in range(4):
                result[row * 4 + col] = sum(
                    left[row * 4 + idx] * right[idx * 4 + col] for idx in range(4)
                )
        return tuple(result)

    def translation_matrix(x_pos: float, y_pos: float, z_pos: float) -> tuple[float, ...]:
        return (
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            x_pos, y_pos, z_pos, 1.0,
        )

    def rotation_x_matrix(angle: float) -> tuple[float, ...]:
        sine = math.sin(angle)
        cosine = math.cos(angle)
        return (
            1.0, 0.0, 0.0, 0.0,
            0.0, cosine, sine, 0.0,
            0.0, -sine, cosine, 0.0,
            0.0, 0.0, 0.0, 1.0,
        )

    def rotation_y_matrix(angle: float) -> tuple[float, ...]:
        sine = math.sin(angle)
        cosine = math.cos(angle)
        return (
            cosine, 0.0, -sine, 0.0,
            0.0, 1.0, 0.0, 0.0,
            sine, 0.0, cosine, 0.0,
            0.0, 0.0, 0.0, 1.0,
        )

    def mat4_to_ctypes(matrix: tuple[float, ...]) -> ctypes.Array[ctypes.c_float]:
        return (ctypes.c_float * 16)(*matrix)

    def compile_shader(source: str, shader_type: int) -> int:
        shader = glCreateShader(shader_type)
        glShaderSource(shader, source)
        glCompileShader(shader)
        status = glGetShaderiv(shader, GL_COMPILE_STATUS)
        if status != GL_TRUE:
            shader_kind = "vertex" if shader_type == GL_VERTEX_SHADER else "fragment"
            raise RuntimeError(
                f"{shader_kind} shader compilation failed: {glGetShaderInfoLog(shader).decode('utf-8', errors='replace')}"
            )
        return shader

    def link_program(vertex_shader: int, fragment_shader: int) -> int:
        program = glCreateProgram()
        glAttachShader(program, vertex_shader)
        glAttachShader(program, fragment_shader)
        glBindAttribLocation(program, 0, b"position")
        glBindAttribLocation(program, 1, b"texcoord")
        glLinkProgram(program)
        status = glGetProgramiv(program, GL_LINK_STATUS)
        if status != GL_TRUE:
            raise RuntimeError(
                f"shader program link failed: {glGetProgramInfoLog(program).decode('utf-8', errors='replace')}"
            )
        return program

    def create_window_with_fallbacks() -> pygame.Surface:
        attempts: list[str] = []
        templates = [
            ("gl2.1 depth24", (2, 1, 24)),
            ("gl2.1 depth16", (2, 1, 16)),
            ("gl2.1 depth0", (2, 1, 0)),
            ("gl1.5 depth16", (1, 5, 16)),
            ("gl1.5 depth0", (1, 5, 0)),
        ]

        pygame.init()
        for description, (major, minor, depth_size) in templates:
            try:
                pygame.display.quit()
                pygame.display.init()
                pygame.display.gl_set_attribute(pygame.GL_CONTEXT_MAJOR_VERSION, major)
                pygame.display.gl_set_attribute(pygame.GL_CONTEXT_MINOR_VERSION, minor)
                pygame.display.gl_set_attribute(pygame.GL_DOUBLEBUFFER, 1)
                pygame.display.gl_set_attribute(pygame.GL_DEPTH_SIZE, depth_size)
                LOGGER.info("Trying viewport config: %s", description)
                surface = pygame.display.set_mode(
                    (args.window_width, args.window_height),
                    OPENGL | DOUBLEBUF,
                )
                return surface
            except pygame.error as exc:
                attempts.append(f"{description}: {exc}")

        raise RuntimeError(
            "pygame found a display server but could not create an OpenGL context with any fallback config. "
            f"Current environment: DISPLAY={display_name!r}, WAYLAND_DISPLAY={wayland_display!r}, XAUTHORITY={xauthority!r}. "
            f"Attempts: {'; '.join(attempts)}"
        )

    surface = create_window_with_fallbacks()
    pygame.display.set_caption("Breezy Box DisplayLink Prototype")

    version = glGetString(GL_VERSION)
    shader_version = glGetString(GL_SHADING_LANGUAGE_VERSION)
    LOGGER.info(
        "Viewport OpenGL: version=%s glsl=%s",
        version.decode("utf-8", errors="replace") if version else "unknown",
        shader_version.decode("utf-8", errors="replace") if shader_version else "unknown",
    )

    glClearColor(0.06, 0.08, 0.11, 1.0)
    glEnable(GL_DEPTH_TEST)

    vertex_shader = compile_shader(VERTEX_SHADER_SOURCE, GL_VERTEX_SHADER)
    fragment_shader = compile_shader(FRAGMENT_SHADER_SOURCE, GL_FRAGMENT_SHADER)
    program = link_program(vertex_shader, fragment_shader)
    glDeleteShader(vertex_shader)
    glDeleteShader(fragment_shader)

    vertex_data = (ctypes.c_float * 20)(
        -1.0, -0.5625, 0.0, 0.0, 0.0,
         1.0, -0.5625, 0.0, 1.0, 0.0,
         1.0,  0.5625, 0.0, 1.0, 1.0,
        -1.0,  0.5625, 0.0, 0.0, 1.0,
    )
    index_data = (ctypes.c_uint * 6)(0, 1, 2, 0, 2, 3)

    vertex_buffer = glGenBuffers(1)
    index_buffer = glGenBuffers(1)

    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer)
    glBufferData(GL_ARRAY_BUFFER, ctypes.sizeof(vertex_data), vertex_data, GL_STATIC_DRAW)

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffer)
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, ctypes.sizeof(index_data), index_data, GL_STATIC_DRAW)

    position_attrib = glGetAttribLocation(program, b"position")
    texcoord_attrib = glGetAttribLocation(program, b"texcoord")
    projection_uniform = glGetUniformLocation(program, b"projection")
    view_uniform = glGetUniformLocation(program, b"view")
    model_uniform = glGetUniformLocation(program, b"model")
    texture_uniform = glGetUniformLocation(program, b"frame_texture")
    blank_uniform = glGetUniformLocation(program, b"blank_factor")

    texture_id = glGenTextures(1)
    glBindTexture(GL_TEXTURE_2D, texture_id)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1)

    texture_size = (0, 0)
    frame_version = -1
    last_caption_update = 0.0

    def upload_texture(width: int, height: int, pixel_bytes: bytes) -> tuple[int, int]:
        glActiveTexture(GL_TEXTURE0)
        glBindTexture(GL_TEXTURE_2D, texture_id)
        if texture_size != (width, height):
            glTexImage2D(
                GL_TEXTURE_2D,
                0,
                GL_RGB,
                width,
                height,
                0,
                GL_RGB,
                GL_UNSIGNED_SHORT_5_6_5,
                pixel_bytes,
            )
            return (width, height)

        glTexSubImage2D(
            GL_TEXTURE_2D,
            0,
            0,
            0,
            width,
            height,
            GL_RGB,
            GL_UNSIGNED_SHORT_5_6_5,
            pixel_bytes,
        )
        return texture_size

    width, height, pixels, version_id = state.framebuffer.snapshot()
    texture_size = upload_texture(width, height, pixels)
    frame_version = version_id

    viewport_width = args.window_width
    viewport_height = args.window_height
    projection = perspective_matrix(60.0, viewport_width / max(1, viewport_height), 0.1, 100.0)
    view = translation_matrix(0.0, 0.0, -3.0)

    clock = pygame.time.Clock()
    try:
        while not stop_event.is_set():
            if pygame.get_init():
                for event in pygame.event.get():
                    if event.type == QUIT:
                        stop_event.set()

            if stop_event.is_set():
                break

            if pygame.display.get_surface() is None:
                raise RuntimeError("pygame display surface disappeared during viewport rendering")

            width, height, pixels, version_id = state.framebuffer.snapshot()
            if version_id != frame_version or texture_size != (width, height):
                texture_size = upload_texture(width, height, pixels)
                frame_version = version_id

            angle = time.monotonic() * 0.35
            model = multiply_mat4(
                multiply_mat4(translation_matrix(0.0, 0.0, 0.0), rotation_y_matrix(angle)),
                rotation_x_matrix(-0.2),
            )

            glViewport(0, 0, viewport_width, viewport_height)
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            glUseProgram(program)
            glUniformMatrix4fv(projection_uniform, 1, GL_FALSE, mat4_to_ctypes(projection))
            glUniformMatrix4fv(view_uniform, 1, GL_FALSE, mat4_to_ctypes(view))
            glUniformMatrix4fv(model_uniform, 1, GL_FALSE, mat4_to_ctypes(model))
            glUniform1i(texture_uniform, 0)
            glUniform1f(blank_uniform, 0.2 if state.blank_mode == 0x07 else 1.0)

            glActiveTexture(GL_TEXTURE0)
            glBindTexture(GL_TEXTURE_2D, texture_id)
            glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer)
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffer)

            stride = 5 * ctypes.sizeof(ctypes.c_float)
            glEnableVertexAttribArray(position_attrib)
            glVertexAttribPointer(position_attrib, 3, GL_FLOAT, GL_FALSE, stride, ctypes.c_void_p(0))
            glEnableVertexAttribArray(texcoord_attrib)
            glVertexAttribPointer(
                texcoord_attrib,
                2,
                GL_FLOAT,
                GL_FALSE,
                stride,
                ctypes.c_void_p(3 * ctypes.sizeof(ctypes.c_float)),
            )
            glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, ctypes.c_void_p(0))
            glDisableVertexAttribArray(position_attrib)
            glDisableVertexAttribArray(texcoord_attrib)

            now = time.monotonic()
            if now - last_caption_update >= 0.5:
                pygame.display.set_caption(
                    "Breezy Box DisplayLink Prototype | "
                    f"{state.status_line()} | "
                    f"base16=0x{state.base16:06x} pclk={state.pixel_clock_5khz * 5}kHz"
                )
                last_caption_update = now

            pygame.display.flip()
            clock.tick(60)
    finally:
        pygame.quit()


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
        max_power_ma=args.max_power,
        configuration_label=args.configuration_label,
    )


def main() -> int:
    args = parse_args()
    configure_logging(args.verbose)

    state = DisplayLinkState(args.width, args.height)
    if args.source == "demo":
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
        LOGGER.info(
            "Waiting for incoming transport data on source=%s. Until bytes arrive, the viewport will stay blank.",
            args.source,
        )
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