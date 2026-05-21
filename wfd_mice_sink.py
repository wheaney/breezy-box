#!/usr/bin/env python3

import argparse
import fcntl
import hashlib
import importlib
import ipaddress
import logging
import os
import random
import re
import shlex
import shutil
import signal
import socket
import socketserver
import subprocess
import sys
import threading
import time
import struct
from dataclasses import dataclass
from typing import Dict, Optional


LOGGER = logging.getLogger("wfd_mice_sink")
SERVICE_TYPE = "_display._tcp"
DEFAULT_SERVICE_NAME = "Breezy Box"
DEFAULT_SIGNALLING_PORT = 7250
DEFAULT_RTSP_PORT = 7236
DEFAULT_RTSP_PATH = "/wfd1.0/streamid=0"
DEFAULT_RTSP_READY_TIMEOUT_SEC = 30.0
DEFAULT_RTSP_CONNECT_TIMEOUT_SEC = 0.75
DEFAULT_RTSP_SOCKET_TIMEOUT_SEC = 5.0
DEFAULT_RTSP_POST_PLAY_DRAIN_SEC = 2.0
DEFAULT_RTSP_KEEPALIVE_INTERVAL_SEC = 15
DEFAULT_WFD_CLIENT_RTP_PORT = 16384
DEFAULT_WFD_VIDEO_FORMATS = "00 00 03 10 0001ffff 1fffffff 00001fff 00 0000 0000 10 none none"
DEFAULT_WFD_AUDIO_CODECS = "AAC 00000007 00"
DEFAULT_PROBE_BLACK_LOG_BUDGET = 20
DEFAULT_INPUT_RTP_LOG_BUDGET = 12
DEFAULT_INPUT_RTP_IDLE_LOG_BUDGET = 6
DEFAULT_ENCODED_VIDEO_LOG_BUDGET = 10
DEFAULT_ENCODED_VIDEO_IDLE_LOG_BUDGET = 6
DEFAULT_RELAY_HOST = "127.0.0.1"
DEFAULT_RELAY_PORT = 5600
DEFAULT_PAYLOAD_TYPE = 96
DEFAULT_DECODER_FRAGMENT = "decodebin ! videoconvert"

GLib = None
Gst = None
APP = None


class MiceProtocolError(Exception):
    pass


class RtspSessionError(Exception):
    pass


class RtspStatusError(RtspSessionError):
    def __init__(self, status_code, reason):
        super().__init__(f"RTSP {status_code} {reason}")
        self.status_code = status_code
        self.reason = reason


@dataclass
class SourceReadyMessage:
    version: int
    command: int
    friendly_name: str
    rtsp_port: int
    source_id: str
    tlvs: Dict[int, bytes]


@dataclass
class RtspResponse:
    status_code: int
    reason: str
    headers: Dict[str, list[str]]
    body: bytes


def ensure_runtime_dependencies():
    global GLib
    global Gst

    if GLib is not None and Gst is not None:
        return

    try:
        gi = importlib.import_module("gi")

        gi.require_version("GLib", "2.0")
        gi.require_version("Gst", "1.0")

        imported_glib = importlib.import_module("gi.repository.GLib")
        imported_gst = importlib.import_module("gi.repository.Gst")
    except ImportError as exc:
        raise RuntimeError(
            "missing runtime dependencies; install python3-gi, gir1.2-gstreamer-1.0, "
            "gstreamer1.0-tools, gstreamer1.0-plugins-base, gstreamer1.0-plugins-good, "
            "gstreamer1.0-plugins-bad, and avahi-utils"
        ) from exc

    GLib = imported_glib
    Gst = imported_gst


def require_element(factory_name):
    element = Gst.ElementFactory.make(factory_name, None)
    if element is None:
        raise RuntimeError(f"required GStreamer element '{factory_name}' is unavailable")
    element.set_state(Gst.State.NULL)


def normalize_mac(mac_text):
    pieces = mac_text.replace("-", ":").split(":")
    if len(pieces) != 6:
        raise ValueError("MAC address must contain 6 octets")
    normalized = []
    for piece in pieces:
        if len(piece) == 0 or len(piece) > 2:
            raise ValueError("MAC address octets must be one or two hex digits")
        normalized.append(f"{int(piece, 16):02X}")
    return ":".join(normalized)


def read_interface_mac(interface_name):
    path = os.path.join("/sys/class/net", interface_name, "address")
    with open(path, "r", encoding="utf-8") as file_obj:
        return normalize_mac(file_obj.read().strip())


def read_interface_ipv4(interface_name):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        request = struct.pack("256s", interface_name[:15].encode("utf-8"))
        response = fcntl.ioctl(sock.fileno(), 0x8915, request)
    except OSError as exc:
        raise RuntimeError(
            f"failed to read IPv4 address for interface '{interface_name}'; bring the interface up and assign an address first"
        ) from exc
    finally:
        sock.close()

    return socket.inet_ntoa(response[20:24])


def derive_local_admin_mac(seed_text):
    digest = hashlib.sha256(seed_text.encode("utf-8")).digest()
    octets = [0x02, digest[0], digest[1], digest[2], digest[3], digest[4]]
    return ":".join(f"{value:02X}" for value in octets)


def build_service_host_name(service_name, bind_host=None, p2p_mac=None):
    label = re.sub(r"[^a-z0-9-]+", "-", service_name.lower()).strip("-")
    if not label:
        label = "breezy-box"
    label = label[:20].rstrip("-") or "breezy-box"
    if bind_host:
        try:
            ip_obj = ipaddress.ip_address(bind_host)
            if isinstance(ip_obj, ipaddress.IPv4Address):
                label = f"{label}-{'-'.join(bind_host.split('.'))}"
            else:
                label = f"{label}-{str(ip_obj).replace(':', '-')[:24].strip('-')}"
        except ValueError:
            pass
    if p2p_mac:
        mac_suffix = p2p_mac.replace(":", "").lower()[-6:]
        label = f"{label}-{mac_suffix}"
    label = f"{label}-{os.getpid()}"
    return f"{label}.local"


def choose_p2p_mac(args):
    if args.p2p_mac:
        return normalize_mac(args.p2p_mac)
    if args.interface:
        try:
            return read_interface_mac(args.interface)
        except OSError as exc:
            raise RuntimeError(f"failed to read MAC address for interface '{args.interface}': {exc}") from exc
    return derive_local_admin_mac(args.service_name or socket.gethostname())


def resolve_bind_host(args):
    if args.bind_host != "0.0.0.0":
        return args.bind_host
    if not args.interface:
        return args.bind_host
    return read_interface_ipv4(args.interface)


def decode_utf16_text(raw_bytes):
    if not raw_bytes:
        return ""
    for encoding in ("utf-16", "utf-16-le", "utf-16-be"):
        try:
            return raw_bytes.decode(encoding).lstrip("\ufeff").rstrip("\x00")
        except UnicodeError:
            continue
    return raw_bytes.hex()


def recv_exact(sock, count):
    chunks = []
    remaining = count
    while remaining > 0:
        chunk = sock.recv(remaining)
        if not chunk:
            raise MiceProtocolError("unexpected EOF while reading signalling message")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def recv_mice_message(sock):
    header = recv_exact(sock, 2)
    total_length = int.from_bytes(header, "big")
    if total_length < 4:
        raise MiceProtocolError(f"invalid MICE message length {total_length}")
    return header + recv_exact(sock, total_length - 2)


def parse_source_ready_message(payload):
    if len(payload) < 4:
        raise MiceProtocolError("MICE message shorter than 4 bytes")

    total_length = int.from_bytes(payload[:2], "big")
    if total_length != len(payload):
        raise MiceProtocolError(
            f"MICE message length field {total_length} does not match payload length {len(payload)}"
        )

    version = payload[2]
    command = payload[3]
    if version != 0x01:
        raise MiceProtocolError(f"unsupported MICE protocol version {version}")
    if command != 0x01:
        raise MiceProtocolError(f"unsupported MICE command {command}")

    cursor = 4
    tlvs = {}
    while cursor < len(payload):
        if cursor + 3 > len(payload):
            raise MiceProtocolError("truncated TLV header in SOURCE_READY")
        tlv_type = payload[cursor]
        tlv_length = int.from_bytes(payload[cursor + 1:cursor + 3], "big")
        cursor += 3
        if cursor + tlv_length > len(payload):
            raise MiceProtocolError(f"truncated TLV payload for type {tlv_type}")
        tlvs[tlv_type] = payload[cursor:cursor + tlv_length]
        cursor += tlv_length

    raw_rtsp_port = tlvs.get(0x02)
    if raw_rtsp_port is not None and len(raw_rtsp_port) != 2:
        raise MiceProtocolError("RTSP port TLV must be exactly 2 bytes")

    return SourceReadyMessage(
        version=version,
        command=command,
        friendly_name=decode_utf16_text(tlvs.get(0x00, b"")),
        rtsp_port=int.from_bytes(raw_rtsp_port, "big") if raw_rtsp_port else DEFAULT_RTSP_PORT,
        source_id=tlvs.get(0x03, b"").decode("ascii", errors="replace"),
        tlvs=tlvs,
    )


def normalize_rtsp_path(rtsp_path):
    if not rtsp_path:
        return "/"
    if rtsp_path.startswith("/"):
        return rtsp_path
    return f"/{rtsp_path}"


def build_rtsp_path_candidates(rtsp_path):
    primary = normalize_rtsp_path(rtsp_path)
    candidates = [primary]

    if primary.endswith("/streamid=0"):
        fallback = primary[: -len("/streamid=0")] or "/"
        if fallback not in candidates:
            candidates.append(fallback)

    return candidates


def build_rtsp_url(source_host, rtsp_port, rtsp_path):
    return f"rtsp://{source_host}:{rtsp_port}{normalize_rtsp_path(rtsp_path)}"


def rtsp_header_value(headers, name, default=None):
    values = headers.get(name.lower())
    if not values:
        return default
    return values[0]


def parse_rtsp_headers(lines):
    headers = {}
    for line in lines:
        if ":" not in line:
            continue
        name, value = line.split(":", 1)
        headers.setdefault(name.strip().lower(), []).append(value.strip())
    return headers


def parse_wfd_parameters(body):
    parameters = {}
    if not body:
        return parameters

    body_text = body.decode("utf-8", errors="replace")
    for raw_line in body_text.splitlines():
        line = raw_line.strip()
        if not line:
            continue
        if ":" in line:
            name, value = line.split(":", 1)
            parameters[name.strip().lower()] = value.strip()
        else:
            parameters[line.strip().lower()] = None
    return parameters


def first_parameter_token(value):
    if not value:
        return None
    return value.split(None, 1)[0]


def summary_is_black(summary):
    if summary is None or "error" in summary:
        return False

    if summary.get("avg_rgb") != (0, 0, 0):
        return False

    for point in summary.get("points", []):
        if point[:3] != (0, 0, 0):
            return False

    return True


def resolve_rtsp_url(base_url, control_value):
    if not control_value or control_value == "*":
        return base_url
    if control_value.startswith("rtsp://"):
        return control_value
    if control_value.startswith("/"):
        match = re.match(r"^(rtsp://[^/]+)", base_url)
        if not match:
            raise RtspSessionError(f"unable to resolve absolute RTSP control path against {base_url!r}")
        return f"{match.group(1)}{control_value}"
    return f"{base_url.rstrip('/')}/{control_value}"


def resolve_rtsp_control_url(base_url, response_headers, sdp_text):
    content_base = rtsp_header_value(response_headers, "content-base", base_url)
    control_values = []
    for line in sdp_text.splitlines():
        line = line.strip()
        if line.startswith("a=control:"):
            control_value = line[len("a=control:"):].strip()
            if control_value and control_value != "*":
                control_values.append(control_value)

    if control_values:
        return resolve_rtsp_url(content_base, control_values[-1])
    return resolve_rtsp_url(content_base, "streamid=0")


def build_wfd_parameter_response(requested_parameters, client_rtp_port):
    lines = []

    if "wfd_content_protection" in requested_parameters:
        lines.append("wfd_content_protection: none")
    if "wfd_video_formats" in requested_parameters:
        lines.append(f"wfd_video_formats: {DEFAULT_WFD_VIDEO_FORMATS}")
    if "wfd_audio_codecs" in requested_parameters:
        lines.append(f"wfd_audio_codecs: {DEFAULT_WFD_AUDIO_CODECS}")
    if "wfd_3d_video_formats" in requested_parameters:
        lines.append("wfd_3d_video_formats: none")
    if "wfd_display_edid" in requested_parameters:
        lines.append("wfd_display_edid: none")
    if "wfd_idr_request_capability" in requested_parameters:
        lines.append("wfd_idr_request_capability: 1")
    if "microsoft_cursor" in requested_parameters:
        lines.append("microsoft_cursor: none")
    if "wfd_client_rtp_ports" in requested_parameters:
        lines.append(
            f"wfd_client_rtp_ports: RTP/AVP/UDP;unicast {client_rtp_port} {client_rtp_port + 1} mode=play"
        )

    if not lines:
        return b""
    return ("\r\n".join(lines) + "\r\n").encode("utf-8")


def build_wfd_idr_request():
    return b"wfd_idr_request\r\n"


def build_wfd_trigger(trigger_method):
    return f"wfd_trigger_method: {trigger_method}\r\n".encode("utf-8")


def wait_for_tcp_endpoint(host, port, ready_timeout_sec, connect_timeout_sec):
    deadline = time.monotonic() + ready_timeout_sec
    last_error = None
    start_time = time.monotonic()
    next_log_after = start_time

    LOGGER.info(
        "waiting up to %.1fs for source RTSP server at %s:%u",
        ready_timeout_sec,
        host,
        port,
    )

    while True:
        try:
            connected_socket = socket.create_connection((host, port), timeout=connect_timeout_sec)
            LOGGER.info(
                "source RTSP server at %s:%u became reachable after %.2fs",
                host,
                port,
                time.monotonic() - start_time,
            )
            return connected_socket
        except OSError as exc:
            last_error = exc
            now = time.monotonic()
            if now >= next_log_after:
                LOGGER.info(
                    "source RTSP server at %s:%u not reachable yet after %.2fs: %s",
                    host,
                    port,
                    now - start_time,
                    exc,
                )
                next_log_after = now + 2.0
            if now >= deadline:
                break
            time.sleep(0.25)

    raise RuntimeError(
        f"unable to reach source RTSP server at {host}:{port}; last socket error: {last_error}. "
        f"This usually means the source has not opened TCP {port} yet or a host firewall is blocking it. "
        f"If the host is still showing 'connecting', try a larger --rtsp-ready-timeout-sec value."
    )


def build_renderer_pipeline(args):
    return (
        'udpsrc address={relay_host} port={relay_port} '
        'caps="application/x-rtp,media=video,encoding-name=H264,payload={payload_type},clock-rate=90000" '
        '! rtph264depay '
        '! h264parse disable-passthrough=true '
        '! video/x-h264,stream-format=byte-stream,alignment=au '
        '! {decoder_fragment}'
    ).format(
        relay_host=args.relay_host,
        relay_port=args.relay_port,
        payload_type=args.payload_type,
        decoder_fragment=args.renderer_decoder_fragment,
    )


def build_renderer_command(args):
    renderer_pipeline = build_renderer_pipeline(args)
    if args.renderer_command:
        rendered = args.renderer_command.format(
            renderer_pipeline=renderer_pipeline,
            relay_host=args.relay_host,
            relay_port=args.relay_port,
            payload_type=args.payload_type,
        )
        return shlex.split(rendered)

    argv = [
        args.renderer_binary,
        "--device",
        args.renderer_device,
        "--verbose",
        "--stream-pipeline",
        renderer_pipeline,
    ]
    argv.extend(args.renderer_extra_arg)
    return argv


def build_relay_pipeline_description(args):
    return (
        'udpsrc port={wfd_client_rtp_port} '
        'caps="application/x-rtp,media=video,encoding-name=MP2T,payload=33,clock-rate=90000" ! '
        'queue max-size-buffers=8 ! '
        'application/x-rtp,media=video,encoding-name=MP2T,payload=33,clock-rate=90000 ! '
        'rtpmp2tdepay ! tsdemux name=demux '
        '[dynamic video/x-h264 pad] ! queue max-size-buffers=8 leaky=downstream ! tee name=video_tee '
        'video_tee. ! queue max-size-buffers=8 leaky=downstream ! '
        'h264parse config-interval=-1 disable-passthrough=true ! '
        'video/x-h264,stream-format=byte-stream,alignment=au ! '
        'rtph264pay pt={payload_type} config-interval=1 ! '
        'udpsink host={relay_host} port={relay_port} sync=false async=false '
        'video_tee. ! queue max-size-buffers=1 leaky=downstream ! '
        'decodebin ! videoconvert ! video/x-raw,format=RGBA ! '
        'appsink name=probe_sink max-buffers=1 drop=true sync=false emit-signals=true'
    ).format(
        wfd_client_rtp_port=args.wfd_client_rtp_port,
        payload_type=args.payload_type,
        relay_host=args.relay_host,
        relay_port=args.relay_port,
    )


def summarize_rgba_buffer(buffer, caps):
    if buffer is None or caps is None or caps.get_size() == 0:
        return None

    structure = caps.get_structure(0)
    success_width, width = structure.get_int("width")
    success_height, height = structure.get_int("height")
    pixel_format = structure.get_string("format")
    if not success_width or not success_height or width <= 0 or height <= 0:
        return None
    if pixel_format != "RGBA":
        return {
            "width": width,
            "height": height,
            "format": pixel_format,
            "error": "unexpected-format",
        }

    success, map_info = buffer.map(Gst.MapFlags.READ)
    if not success:
        return {
            "width": width,
            "height": height,
            "format": pixel_format,
            "error": "map-failed",
        }

    try:
        data = map_info.data
        if len(data) < width * height * 4:
            return {
                "width": width,
                "height": height,
                "format": pixel_format,
                "error": "buffer-too-small",
                "size": len(data),
            }

        stride = len(data) // height
        if stride < width * 4:
            return {
                "width": width,
                "height": height,
                "format": pixel_format,
                "error": "stride-too-small",
                "stride": stride,
            }

        total_red = 0
        total_green = 0
        total_blue = 0
        sampled_pixels = 0
        step_y = max(1, height // 8)
        step_x = max(1, width // 8)

        for y in range(0, height, step_y):
            row_offset = y * stride
            for x in range(0, width, step_x):
                offset = row_offset + x * 4
                total_red += data[offset + 0]
                total_green += data[offset + 1]
                total_blue += data[offset + 2]
                sampled_pixels += 1

        points = []
        for x, y in ((0, 0), (width // 2, height // 2), (width - 1, height - 1)):
            offset = y * stride + x * 4
            points.append((data[offset + 0], data[offset + 1], data[offset + 2], data[offset + 3]))

        return {
            "width": width,
            "height": height,
            "format": pixel_format,
            "stride": stride,
            "avg_rgb": (
                total_red // max(sampled_pixels, 1),
                total_green // max(sampled_pixels, 1),
                total_blue // max(sampled_pixels, 1),
            ),
            "points": points,
        }
    finally:
        buffer.unmap(map_info)


def format_command(argv):
    return " ".join(shlex.quote(part) for part in argv)


class ServicePublisher:
    def __init__(self, service_name, signalling_port, txt_records, disabled, host_name=None, host_address=None):
        self.service_name = service_name
        self.signalling_port = signalling_port
        self.txt_records = txt_records
        self.disabled = disabled
        self.host_name = host_name
        self.host_address = host_address
        self.service_process = None
        self.address_process = None

    def start(self):
        if self.disabled:
            LOGGER.info("service advertisement disabled")
            return

        publisher = shutil.which("avahi-publish-service")
        address_publisher = shutil.which("avahi-publish-address")
        if not publisher or not address_publisher:
            raise RuntimeError("avahi-publish-service and avahi-publish-address are required for WFD-MICE discovery")

        if self.host_name and self.host_address:
            self.host_name = self._start_address_with_retry(address_publisher, self.host_name, self.host_address)
            LOGGER.info(
                "publishing %s on %s via Avahi as %s -> %s",
                self.service_name,
                SERVICE_TYPE,
                self.host_name,
                self.host_address,
            )
        else:
            LOGGER.info("publishing %s on %s via Avahi", self.service_name, SERVICE_TYPE)

        argv = [publisher]
        if self.host_name:
            argv.extend(["-H", self.host_name])
        argv.extend([self.service_name, SERVICE_TYPE, str(self.signalling_port)])
        argv.extend(self.txt_records)
        self.service_process = subprocess.Popen(argv)

    def _start_address_with_retry(self, address_publisher, host_name, host_address):
        base_host_name = host_name.removesuffix(".local")
        for attempt in range(6):
            candidate = host_name if attempt == 0 else f"{base_host_name}-{random.randint(1000, 9999)}.local"
            address_process = subprocess.Popen(
                [address_publisher, "--no-reverse", candidate, host_address],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                text=True,
            )
            time.sleep(0.2)
            exit_code = address_process.poll()
            if exit_code is None:
                self.address_process = address_process
                return candidate

            stderr_output = ""
            if address_process.stderr is not None:
                stderr_output = address_process.stderr.read().strip()
            if "Local name collision" in stderr_output and attempt < 5:
                LOGGER.warning("mDNS host name %s collided locally; retrying with a fresh name", candidate)
                continue
            raise RuntimeError(
                f"failed to publish mDNS host record {candidate} -> {host_address}: "
                f"{stderr_output or f'exit code {exit_code}'}"
            )

        raise RuntimeError(f"failed to publish a unique mDNS host record for {host_address}")

    def stop(self):
        for process in (self.service_process, self.address_process):
            if process is None or process.poll() is not None:
                continue
            process.terminate()
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        self.service_process = None
        self.address_process = None


class RendererProcess:
    def __init__(self, args):
        self.args = args
        self.process = None

    def suggested_command(self):
        return build_renderer_command(self.args)

    def ensure_started(self):
        if not self.args.launch_renderer:
            return
        if self.process is not None and self.process.poll() is None:
            return
        argv = self.suggested_command()
        LOGGER.info("starting renderer: %s", format_command(argv))
        self.process = subprocess.Popen(argv)

    def restart(self):
        if not self.args.launch_renderer:
            return
        self.stop()
        self.ensure_started()

    def stop(self):
        if self.process is None or self.process.poll() is not None:
            return
        self.process.terminate()
        try:
            self.process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()


class RtspRelay:
    def __init__(self, args):
        self.args = args
        self.pipeline = None
        self.bus = None
        self.current_url = None
        self.current_source_host = None
        self.current_source_ready = None
        self.path_candidates = []
        self.path_index = 0
        self.control_socket = None
        self.control_buffer = bytearray()
        self.control_thread = None
        self.control_thread_stop = threading.Event()
        self.rtcp_socket = None
        self.cseq = 0
        self.session_id = None
        self.aggregate_url = None
        self.presentation_url = None
        self.sent_sink_options = False
        self.handshake_complete = False
        self.keepalive_source_id = 0
        self.demux = None
        self.video_queue = None
        self.auxiliary_elements = []
        self.probe_black_log_budget = DEFAULT_PROBE_BLACK_LOG_BUDGET
        self.probe_non_black_logged = False
        self.probe_frame_count = 0
        self.input_rtp_log_budget = DEFAULT_INPUT_RTP_LOG_BUDGET
        self.input_rtp_idle_log_budget = DEFAULT_INPUT_RTP_IDLE_LOG_BUDGET
        self.input_rtp_packet_count = 0
        self.input_rtp_last_seq = None
        self.input_rtp_last_timestamp = None
        self.input_rtp_last_payload_type = None
        self.input_rtp_last_packet_monotonic = None
        self.encoded_video_log_budget = DEFAULT_ENCODED_VIDEO_LOG_BUDGET
        self.encoded_video_idle_log_budget = DEFAULT_ENCODED_VIDEO_IDLE_LOG_BUDGET
        self.encoded_video_buffer_count = 0
        self.encoded_video_last_pts_ms = None
        self.encoded_video_last_buffer_monotonic = None
        self.encoded_video_watchdog_source_id = 0

    def start(self, source_host, source_ready):
        self.stop()
        self.current_source_host = source_host
        self.current_source_ready = source_ready
        self.path_candidates = build_rtsp_path_candidates(self.args.rtsp_path)

        self._start_current_candidate(self.path_candidates[0])

    def _start_current_candidate(self, rtsp_path):
        description = build_relay_pipeline_description(self.args)
        next_url = build_rtsp_url(self.current_source_host,
                                  self.current_source_ready.rtsp_port,
                                  rtsp_path)

        LOGGER.info(
            "starting WFD relay from %s (source='%s', source-id='%s')",
            next_url,
            self.current_source_ready.friendly_name or "unknown",
            self.current_source_ready.source_id or "unknown",
        )
        if self.args.verbose:
            print(f"GStreamer relay pipeline: {description}", flush=True)

        connected_socket = wait_for_tcp_endpoint(self.current_source_host,
                             self.current_source_ready.rtsp_port,
                             self.args.rtsp_ready_timeout_sec,
                             self.args.rtsp_connect_timeout_sec)

        self._bind_rtcp_socket()
        self.pipeline = self._build_relay_pipeline()
        self.bus = self.pipeline.get_bus()
        self.bus.add_signal_watch()
        self.bus.connect("message", self._on_message)
        state_result = self.pipeline.set_state(Gst.State.PLAYING)
        if state_result == Gst.StateChangeReturn.FAILURE:
            connected_socket.close()
            self.stop()
            raise RuntimeError(f"failed to start relay pipeline for {next_url}")
        self.encoded_video_watchdog_source_id = GLib.timeout_add_seconds(
            5,
            self._on_encoded_video_watchdog,
        )

        self._open_control_connection(connected_socket)
        self.aggregate_url = next_url
        self.current_url = next_url
        self._run_source_driven_handshake()
        self._start_control_thread()

    def _build_relay_pipeline(self):
        pipeline = Gst.Pipeline.new("wfd-relay")
        if pipeline is None:
            raise RuntimeError("failed to create WFD relay pipeline")

        udpsrc = Gst.ElementFactory.make("udpsrc", "wfd_udpsrc")
        source_queue = Gst.ElementFactory.make("queue", "wfd_source_queue")
        depay = Gst.ElementFactory.make("rtpmp2tdepay", "wfd_rtpmp2tdepay")
        demux = Gst.ElementFactory.make("tsdemux", "wfd_tsdemux")
        video_queue = Gst.ElementFactory.make("queue", "wfd_video_queue")
        video_tee = Gst.ElementFactory.make("tee", "wfd_video_tee")
        relay_queue = Gst.ElementFactory.make("queue", "wfd_relay_queue")
        parser = Gst.ElementFactory.make("h264parse", "wfd_h264parse")
        capsfilter = Gst.ElementFactory.make("capsfilter", "wfd_h264_caps")
        pay = Gst.ElementFactory.make("rtph264pay", "wfd_rtph264pay")
        sink = Gst.ElementFactory.make("udpsink", "wfd_udpsink")
        probe_queue = Gst.ElementFactory.make("queue", "wfd_probe_queue")
        probe_parser = Gst.ElementFactory.make("h264parse", "wfd_probe_h264parse")
        probe_h264_caps = Gst.ElementFactory.make("capsfilter", "wfd_probe_h264_caps")
        probe_decodebin = Gst.ElementFactory.make("decodebin", "wfd_probe_decodebin")
        probe_convert = Gst.ElementFactory.make("videoconvert", "wfd_probe_videoconvert")
        probe_caps = Gst.ElementFactory.make("capsfilter", "wfd_probe_caps")
        probe_sink = Gst.ElementFactory.make("appsink", "wfd_probe_sink")

        elements = [
            udpsrc,
            source_queue,
            depay,
            demux,
            video_queue,
            video_tee,
            relay_queue,
            parser,
            capsfilter,
            pay,
            sink,
            probe_queue,
            probe_parser,
            probe_h264_caps,
            probe_decodebin,
            probe_convert,
            probe_caps,
            probe_sink,
        ]
        if any(element is None for element in elements):
            raise RuntimeError("failed to create one or more WFD relay GStreamer elements")

        udpsrc.set_property(
            "caps",
            Gst.Caps.from_string(
                "application/x-rtp,media=video,encoding-name=MP2T,payload=33,clock-rate=90000"
            ),
        )
        udpsrc.set_property("port", self.args.wfd_client_rtp_port)
        source_queue.set_property("max-size-buffers", 8)
        video_queue.set_property("max-size-buffers", 8)
        video_queue.set_property("leaky", 2)
        relay_queue.set_property("max-size-buffers", 8)
        relay_queue.set_property("leaky", 2)
        parser.set_property("config-interval", -1)
        parser.set_property("disable-passthrough", True)
        capsfilter.set_property(
            "caps",
            Gst.Caps.from_string("video/x-h264,stream-format=byte-stream,alignment=au"),
        )
        pay.set_property("pt", self.args.payload_type)
        pay.set_property("config-interval", 1)
        sink.set_property("host", self.args.relay_host)
        sink.set_property("port", self.args.relay_port)
        sink.set_property("sync", False)
        sink.set_property("async", False)
        probe_queue.set_property("max-size-buffers", 1)
        probe_queue.set_property("leaky", 2)
        probe_parser.set_property("config-interval", -1)
        probe_parser.set_property("disable-passthrough", True)
        probe_h264_caps.set_property(
            "caps",
            Gst.Caps.from_string("video/x-h264,stream-format=byte-stream,alignment=au"),
        )
        probe_caps.set_property("caps", Gst.Caps.from_string("video/x-raw,format=RGBA"))
        probe_sink.set_property("emit-signals", True)
        probe_sink.set_property("max-buffers", 1)
        probe_sink.set_property("drop", True)
        probe_sink.set_property("sync", False)

        pipeline.add(udpsrc)
        pipeline.add(source_queue)
        pipeline.add(depay)
        pipeline.add(demux)
        pipeline.add(video_queue)
        pipeline.add(video_tee)
        pipeline.add(relay_queue)
        pipeline.add(parser)
        pipeline.add(capsfilter)
        pipeline.add(pay)
        pipeline.add(sink)
        pipeline.add(probe_queue)
        pipeline.add(probe_parser)
        pipeline.add(probe_h264_caps)
        pipeline.add(probe_decodebin)
        pipeline.add(probe_convert)
        pipeline.add(probe_caps)
        pipeline.add(probe_sink)

        for upstream, downstream in ((udpsrc, source_queue),
                                     (source_queue, depay),
                                     (depay, demux),
                                     (video_queue, video_tee),
                                     (relay_queue, parser),
                                     (parser, capsfilter),
                                     (capsfilter, pay),
                                     (pay, sink),
                                     (probe_queue, probe_parser),
                                     (probe_parser, probe_h264_caps),
                                     (probe_h264_caps, probe_decodebin),
                                     (probe_convert, probe_caps),
                                     (probe_caps, probe_sink)):
            if not upstream.link(downstream):
                raise RuntimeError(
                    f"failed to link WFD relay element {upstream.get_name()} -> {downstream.get_name()}"
                )

        relay_pad = video_tee.request_pad_simple("src_%u")
        probe_pad = video_tee.request_pad_simple("src_%u")
        relay_sink_pad = relay_queue.get_static_pad("sink")
        probe_sink_pad = probe_queue.get_static_pad("sink")
        if relay_pad is None or probe_pad is None or relay_sink_pad is None or probe_sink_pad is None:
            raise RuntimeError("failed to request tee pads for relay pipeline branches")
        if relay_pad.link(relay_sink_pad) != Gst.PadLinkReturn.OK:
            raise RuntimeError("failed to link relay tee branch")
        if probe_pad.link(probe_sink_pad) != Gst.PadLinkReturn.OK:
            raise RuntimeError("failed to link probe tee branch")

        source_src_pad = source_queue.get_static_pad("src")
        if source_src_pad is None:
            raise RuntimeError("failed to look up WFD relay source queue source pad")
        source_src_pad.add_probe(Gst.PadProbeType.BUFFER, self._on_input_rtp_packet)

        video_src_pad = video_queue.get_static_pad("src")
        if video_src_pad is None:
            raise RuntimeError("failed to look up WFD relay video queue source pad")
        video_src_pad.add_probe(Gst.PadProbeType.BUFFER, self._on_encoded_video_buffer)

        demux.connect("pad-added", self._on_demux_pad_added, video_queue)
        probe_decodebin.connect("pad-added", self._on_probe_decodebin_pad_added, probe_convert)
        probe_sink.connect("new-sample", self._on_probe_sample)
        self.demux = demux
        self.video_queue = video_queue
        return pipeline

    def _on_probe_decodebin_pad_added(self, decodebin, pad, probe_convert):
        del decodebin
        sink_pad = probe_convert.get_static_pad("sink")
        if sink_pad is None or sink_pad.is_linked():
            return
        link_result = pad.link(sink_pad)
        if link_result != Gst.PadLinkReturn.OK:
            raise RuntimeError(f"failed to link probe decodebin pad: {link_result.value_nick}")

    def _on_input_rtp_packet(self, pad, info):
        del pad
        buffer = info.get_buffer()
        if buffer is None:
            return Gst.PadProbeReturn.OK

        self.input_rtp_packet_count += 1
        self.input_rtp_idle_log_budget = DEFAULT_INPUT_RTP_IDLE_LOG_BUDGET
        self.input_rtp_last_packet_monotonic = time.monotonic()

        packet_size = buffer.get_size()
        payload_type = None
        marker = False
        sequence = None
        timestamp = None

        success, map_info = buffer.map(Gst.MapFlags.READ)
        if success:
            try:
                data = map_info.data
                if len(data) >= 12:
                    version = data[0] >> 6
                    if version == 2:
                        payload_type = data[1] & 0x7F
                        marker = bool(data[1] & 0x80)
                        sequence = int.from_bytes(data[2:4], byteorder="big")
                        timestamp = int.from_bytes(data[4:8], byteorder="big")
            finally:
                buffer.unmap(map_info)

        self.input_rtp_last_payload_type = payload_type
        self.input_rtp_last_seq = sequence
        self.input_rtp_last_timestamp = timestamp

        should_log = False
        if self.input_rtp_log_budget > 0:
            should_log = True
            self.input_rtp_log_budget -= 1
        elif self.input_rtp_packet_count % 300 == 0:
            should_log = True

        if should_log:
            print(
                "Relay input RTP packet: "
                f"packet={self.input_rtp_packet_count} "
                f"size={packet_size} "
                f"pt={payload_type if payload_type is not None else 'unknown'} "
                f"seq={sequence if sequence is not None else 'unknown'} "
                f"timestamp={timestamp if timestamp is not None else 'unknown'} "
                f"marker={int(marker)}",
                flush=True,
            )

        return Gst.PadProbeReturn.OK

    def _on_encoded_video_buffer(self, pad, info):
        del pad
        buffer = info.get_buffer()
        if buffer is None:
            return Gst.PadProbeReturn.OK

        self.encoded_video_buffer_count += 1
        self.encoded_video_idle_log_budget = DEFAULT_ENCODED_VIDEO_IDLE_LOG_BUDGET
        self.encoded_video_last_buffer_monotonic = time.monotonic()

        pts_ms = None
        if buffer.pts != Gst.CLOCK_TIME_NONE:
            pts_ms = buffer.pts / Gst.MSECOND
        self.encoded_video_last_pts_ms = pts_ms

        dts_ms = None
        if buffer.dts != Gst.CLOCK_TIME_NONE:
            dts_ms = buffer.dts / Gst.MSECOND

        buffer_flags = buffer.get_flags()
        flag_names = ["delta" if buffer_flags & Gst.BufferFlags.DELTA_UNIT else "key"]
        if buffer_flags & Gst.BufferFlags.DISCONT:
            flag_names.append("discont")
        if buffer_flags & Gst.BufferFlags.CORRUPTED:
            flag_names.append("corrupted")

        should_log = False
        if self.encoded_video_log_budget > 0:
            should_log = True
            self.encoded_video_log_budget -= 1
        elif self.encoded_video_buffer_count % 120 == 0:
            should_log = True

        if should_log:
            print(
                "Relay encoded video buffer: "
                f"buffer={self.encoded_video_buffer_count} "
                f"size={buffer.get_size()} "
                f"pts_ms={pts_ms if pts_ms is not None else 'none'} "
                f"dts_ms={dts_ms if dts_ms is not None else 'none'} "
                f"flags={','.join(flag_names)}",
                flush=True,
            )

        return Gst.PadProbeReturn.OK

    def _on_encoded_video_watchdog(self):
        if self.pipeline is None:
            self.encoded_video_watchdog_source_id = 0
            return GLib.SOURCE_REMOVE

        if (
            self.input_rtp_packet_count > 0
            and self.input_rtp_last_packet_monotonic is not None
            and self.input_rtp_idle_log_budget > 0
        ):
            input_idle_ms = (time.monotonic() - self.input_rtp_last_packet_monotonic) * 1000.0
            if input_idle_ms >= 5000.0:
                print(
                    "Relay input RTP idle: "
                    f"packets={self.input_rtp_packet_count} "
                    f"last_pt={self.input_rtp_last_payload_type if self.input_rtp_last_payload_type is not None else 'unknown'} "
                    f"last_seq={self.input_rtp_last_seq if self.input_rtp_last_seq is not None else 'unknown'} "
                    f"last_timestamp={self.input_rtp_last_timestamp if self.input_rtp_last_timestamp is not None else 'unknown'} "
                    f"idle_ms={input_idle_ms:.0f}",
                    flush=True,
                )
                self.input_rtp_idle_log_budget -= 1

        if (
            self.encoded_video_buffer_count == 0
            or self.encoded_video_last_buffer_monotonic is None
            or self.encoded_video_idle_log_budget <= 0
        ):
            return GLib.SOURCE_CONTINUE

        idle_ms = (time.monotonic() - self.encoded_video_last_buffer_monotonic) * 1000.0
        if idle_ms < 5000.0:
            return GLib.SOURCE_CONTINUE

        print(
            "Relay encoded video idle: "
            f"buffers={self.encoded_video_buffer_count} "
            f"last_pts_ms={self.encoded_video_last_pts_ms if self.encoded_video_last_pts_ms is not None else 'none'} "
            f"idle_ms={idle_ms:.0f}",
            flush=True,
        )
        self.encoded_video_idle_log_budget -= 1
        return GLib.SOURCE_CONTINUE

    def _on_probe_sample(self, sink):
        sample = sink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.ERROR

        self.probe_frame_count += 1
        summary = summarize_rgba_buffer(sample.get_buffer(), sample.get_caps())
        buffer = sample.get_buffer()
        pts_ms = None
        if buffer is not None and buffer.pts != Gst.CLOCK_TIME_NONE:
            pts_ms = buffer.pts / Gst.MSECOND
        should_log = False
        if summary is None or "error" in (summary or {}):
            should_log = True
        elif summary_is_black(summary):
            if self.probe_black_log_budget > 0:
                should_log = True
                self.probe_black_log_budget -= 1
        elif not self.probe_non_black_logged:
            should_log = True
            self.probe_non_black_logged = True

        if should_log:
            if summary is None:
                print(
                    f"Relay probe frame summary: frame={self.probe_frame_count} unavailable",
                    flush=True,
                )
            elif "error" in summary:
                print(
                    "Relay probe frame summary: "
                    f"frame={self.probe_frame_count} "
                    f"{summary.get('width', 0)}x{summary.get('height', 0)} "
                    f"pts_ms={pts_ms if pts_ms is not None else 'none'} "
                    f"format={summary.get('format')} error={summary['error']}",
                    flush=True,
                )
            else:
                p0, p1, p2 = summary["points"]
                avg_r, avg_g, avg_b = summary["avg_rgb"]
                print(
                    "Relay probe frame summary: "
                    f"frame={self.probe_frame_count} "
                    f"{summary['width']}x{summary['height']} "
                    f"pts_ms={pts_ms if pts_ms is not None else 'none'} "
                    f"stride={summary['stride']} "
                    f"avg_rgb=({avg_r},{avg_g},{avg_b}) "
                    f"p0={p0} p1={p1} p2={p2}",
                    flush=True,
                )
        return Gst.FlowReturn.OK

    def _on_demux_pad_added(self, demux, pad, video_queue):
        pipeline = demux.get_parent()
        if pipeline is None:
            raise RuntimeError("failed to look up relay pipeline for tsdemux pad handling")
        caps = pad.get_current_caps() or pad.query_caps(None)
        structure_name = None
        if caps is not None and caps.get_size() > 0:
            structure_name = caps.get_structure(0).get_name()

        if structure_name != "video/x-h264":
            if self.args.verbose:
                print(f"Ignoring demux pad with caps: {caps.to_string() if caps is not None else 'unknown'}", flush=True)
            self._attach_auxiliary_pad_branch(pipeline, pad, caps)
            return

        sink_pad = video_queue.get_static_pad("sink")
        if sink_pad is None:
            raise RuntimeError("failed to look up WFD relay video queue sink pad")
        if sink_pad.is_linked():
            return

        link_result = pad.link(sink_pad)
        if link_result != Gst.PadLinkReturn.OK:
            raise RuntimeError(f"failed to link tsdemux video pad to relay queue: {link_result.value_nick}")
        if self.args.verbose:
            print(f"Linked demux video pad with caps: {caps.to_string() if caps is not None else 'unknown'}", flush=True)

    def _attach_auxiliary_pad_branch(self, pipeline, pad, caps):
        if pad.is_linked():
            return

        queue_name = f"wfd_aux_queue_{len(self.auxiliary_elements)}"
        sink_name = f"wfd_aux_sink_{len(self.auxiliary_elements)}"
        queue = Gst.ElementFactory.make("queue", queue_name)
        sink = Gst.ElementFactory.make("fakesink", sink_name)
        if queue is None or sink is None:
            raise RuntimeError("failed to create auxiliary relay elements for non-video demux pad")

        queue.set_property("max-size-buffers", 8)
        queue.set_property("leaky", 2)
        sink.set_property("sync", False)
        sink.set_property("async", False)

        pipeline.add(queue)
        pipeline.add(sink)
        if not queue.link(sink):
            raise RuntimeError("failed to link auxiliary relay queue to fakesink")

        queue.sync_state_with_parent()
        sink.sync_state_with_parent()

        sink_pad = queue.get_static_pad("sink")
        if sink_pad is None:
            raise RuntimeError("failed to look up auxiliary relay queue sink pad")
        link_result = pad.link(sink_pad)
        if link_result != Gst.PadLinkReturn.OK:
            raise RuntimeError(f"failed to link auxiliary demux pad: {link_result.value_nick}")

        self.auxiliary_elements.extend([queue, sink])
        if self.args.verbose:
            print(f"Draining auxiliary demux pad with caps: {caps.to_string() if caps is not None else 'unknown'}", flush=True)

    def _describe_with_fallback(self):
        last_error = None
        for index, rtsp_path in enumerate(self.path_candidates[self.path_index:], start=self.path_index):
            describe_url = build_rtsp_url(
                self.current_source_host,
                self.current_source_ready.rtsp_port,
                rtsp_path,
            )
            self.path_index = index
            self.current_url = describe_url
            self.aggregate_url = describe_url
            try:
                return self._request_expect_ok(
                    "DESCRIBE",
                    describe_url,
                    headers={"Accept": "application/sdp"},
                )
            except RtspStatusError as exc:
                last_error = exc
                if exc.status_code == 404 and index + 1 < len(self.path_candidates):
                    next_url = build_rtsp_url(
                        self.current_source_host,
                        self.current_source_ready.rtsp_port,
                        self.path_candidates[index + 1],
                    )
                    LOGGER.warning("RTSP path %s returned 404; retrying with %s", describe_url, next_url)
                    continue
                raise

        if last_error is not None:
            raise last_error
        raise RtspSessionError("no RTSP path candidates available for DESCRIBE")

    def _bind_rtcp_socket(self):
        self.rtcp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rtcp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.rtcp_socket.bind(("", self.args.wfd_client_rtp_port + 1))

    def _open_control_connection(self, connected_socket=None):
        self.control_socket = connected_socket
        if self.control_socket is None:
            self.control_socket = socket.create_connection(
                (self.current_source_host, self.current_source_ready.rtsp_port),
                timeout=self.args.rtsp_connect_timeout_sec,
            )
        self.control_socket.settimeout(DEFAULT_RTSP_SOCKET_TIMEOUT_SEC)
        self.control_buffer.clear()
        self.cseq = 0

    def _run_source_driven_handshake(self):
        while not self.handshake_complete:
            try:
                first_line, headers, body = self._read_message()
            except socket.timeout as exc:
                raise RtspSessionError(
                    "timed out waiting for the source-driven WFD handshake; "
                    "the source never sent the expected OPTIONS/GET_PARAMETER/SET_PARAMETER sequence"
                ) from exc

            if first_line.startswith("RTSP/"):
                if self.args.verbose:
                    print(f"RTSP unsolicited status line: {first_line}", flush=True)
                continue

            self._handle_server_request(first_line, headers, body)

    def _start_control_thread(self):
        if self.control_thread is not None and self.control_thread.is_alive():
            return
        self.control_thread_stop.clear()
        self.control_thread = threading.Thread(
            target=self._control_loop,
            name="wfd-rtsp-control",
            daemon=True,
        )
        self.control_thread.start()

    def _control_loop(self):
        while not self.control_thread_stop.is_set():
            try:
                first_line, headers, body = self._read_message()
            except socket.timeout:
                continue
            except Exception as exc:
                if self.control_thread_stop.is_set():
                    break
                LOGGER.error("RTSP control loop failed: %s", exc)
                GLib.idle_add(self.stop)
                break

            if first_line.startswith("RTSP/"):
                if self.args.verbose:
                    print(f"RTSP unsolicited status line: {first_line}", flush=True)
                continue

            try:
                self._handle_server_request(first_line, headers, body)
            except Exception as exc:
                LOGGER.error("failed to handle RTSP server request: %s", exc)
                GLib.idle_add(self.stop)
                break

    def _send_bytes(self, payload):
        if self.control_socket is None:
            raise RtspSessionError("RTSP control socket is not connected")
        self.control_socket.sendall(payload)

    def _drain_server_requests(self, wait_timeout_sec):
        if self.control_socket is None or wait_timeout_sec <= 0:
            return

        deadline = time.monotonic() + wait_timeout_sec
        original_timeout = self.control_socket.gettimeout()

        try:
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break

                self.control_socket.settimeout(remaining)
                try:
                    first_line, headers, body = self._read_message()
                except socket.timeout:
                    break

                if first_line.startswith("RTSP/"):
                    if self.args.verbose:
                        print(f"RTSP unsolicited status line: {first_line}", flush=True)
                    continue

                self._handle_server_request(first_line, headers, body)
        finally:
            self.control_socket.settimeout(
                DEFAULT_RTSP_SOCKET_TIMEOUT_SEC if original_timeout is None else original_timeout
            )

    def _request_expect_ok(self, method, uri, headers=None, body=b""):
        response = self._send_request(method, uri, headers=headers, body=body)
        if self.args.verbose:
            print(
                f"RTSP response: {response.status_code} {response.reason} "
                f"for {method} {uri}",
                flush=True,
            )
        if response.status_code != 200:
            raise RtspStatusError(response.status_code, response.reason)
        return response

    def _send_request(self, method, uri, headers=None, body=b""):
        if headers is None:
            headers = {}

        body = body or b""
        self.cseq += 1
        lines = [
            f"{method} {uri} RTSP/1.0",
            f"CSeq: {self.cseq}",
            "User-Agent: BreezyBox/0.1",
        ]
        if self.session_id and method not in ("OPTIONS", "DESCRIBE"):
            lines.append(f"Session: {self.session_id}")
        for name, value in headers.items():
            lines.append(f"{name}: {value}")
        if body:
            lines.append(f"Content-Length: {len(body)}")
        lines.append("")
        lines.append("")

        payload = "\r\n".join(lines).encode("utf-8") + body
        if self.args.verbose:
            print(f"RTSP request: {method} {uri}", flush=True)
        self._send_bytes(payload)
        return self._read_response()

    def _read_response(self):
        while True:
            first_line, headers, body = self._read_message()
            if first_line.startswith("RTSP/"):
                parts = first_line.split(" ", 2)
                if len(parts) < 3:
                    raise RtspSessionError(f"malformed RTSP status line: {first_line!r}")
                if self.args.verbose:
                    print(f"RTSP status line: {first_line}", flush=True)
                return RtspResponse(status_code=int(parts[1]),
                                    reason=parts[2],
                                    headers=headers,
                                    body=body)

            self._handle_server_request(first_line, headers, body)

    def _read_message(self):
        if self.control_socket is None:
            raise RtspSessionError("RTSP control socket is unavailable")

        first_line = self._read_line_from_socket()

        header_lines = []
        while True:
            line = self._read_line_from_socket()
            if not line:
                break
            header_lines.append(line)

        headers = parse_rtsp_headers(header_lines)
        content_length = int(rtsp_header_value(headers, "content-length", "0") or "0")
        body = self._read_exact_from_socket(content_length) if content_length > 0 else b""
        if content_length > 0 and len(body) != content_length:
            raise RtspSessionError("RTSP control connection closed while reading body")

        return first_line, headers, body

    def _read_line_from_socket(self):
        while True:
            newline_index = self.control_buffer.find(b"\n")
            if newline_index >= 0:
                line = bytes(self.control_buffer[: newline_index + 1])
                del self.control_buffer[: newline_index + 1]
                return line.decode("utf-8", errors="replace").rstrip("\r\n")

            self._recv_into_control_buffer()

    def _read_exact_from_socket(self, count):
        while len(self.control_buffer) < count:
            self._recv_into_control_buffer()

        payload = bytes(self.control_buffer[:count])
        del self.control_buffer[:count]
        return payload

    def _recv_into_control_buffer(self):
        if self.control_socket is None:
            raise RtspSessionError("RTSP control socket is unavailable")

        chunk = self.control_socket.recv(4096)
        if not chunk:
            raise RtspSessionError("RTSP control connection closed unexpectedly")
        self.control_buffer.extend(chunk)

    def _handle_server_request(self, first_line, headers, body):
        parts = first_line.split(" ", 2)
        if len(parts) < 3:
            raise RtspSessionError(f"malformed RTSP request line: {first_line!r}")

        method = parts[0]
        if self.args.verbose:
            print(f"RTSP server request: {first_line}", flush=True)
            if headers:
                print(f"RTSP server request headers: {headers}", flush=True)
            if body:
                print(
                    "RTSP server request body: "
                    f"{body.decode('utf-8', errors='replace').rstrip()}",
                    flush=True,
                )

        if method == "OPTIONS":
            self._handle_source_options(headers)
            return
        if method == "GET_PARAMETER":
            self._handle_source_get_parameter(headers, body)
            return
        if method == "SET_PARAMETER":
            self._handle_source_set_parameter(headers, body)
            return

        self._send_server_response(headers, 200, "OK")

    def _handle_source_options(self, request_headers):
        self._send_server_response(
            request_headers,
            200,
            "OK",
            {"Public": "org.wfa.wfd1.0, GET_PARAMETER, SET_PARAMETER"},
        )

        if self.sent_sink_options:
            return

        self.sent_sink_options = True
        self._request_expect_ok("OPTIONS", "*", headers={"Require": "org.wfa.wfd1.0"})

    def _handle_source_get_parameter(self, request_headers, body):
        requested_parameters = parse_wfd_parameters(body)
        response_body = build_wfd_parameter_response(
            requested_parameters.keys(),
            self.args.wfd_client_rtp_port,
        )
        response_headers = {}
        if response_body:
            response_headers["Content-Type"] = "text/parameters"
        self._send_server_response(request_headers, 200, "OK", response_headers, response_body)

    def _handle_source_set_parameter(self, request_headers, body):
        response_headers = {}
        content_type = rtsp_header_value(request_headers, "content-type")
        if content_type:
            response_headers["Content-Type"] = content_type
        self._send_server_response(request_headers, 200, "OK", response_headers)

        parameters = parse_wfd_parameters(body)
        presentation_url = first_parameter_token(parameters.get("wfd_presentation_url"))
        if presentation_url:
            if presentation_url != self.presentation_url:
                LOGGER.info("source presentation URL: %s", presentation_url)
            self.presentation_url = presentation_url
            self.aggregate_url = presentation_url
            self.current_url = presentation_url

        trigger_method = parameters.get("wfd_trigger_method")
        if not trigger_method:
            return

        trigger_method = trigger_method.upper()
        LOGGER.info("source requested WFD trigger: %s", trigger_method)
        if trigger_method == "SETUP":
            self._setup_and_play()

    def _setup_and_play(self):
        if self.handshake_complete:
            return
        if not self.presentation_url:
            raise RtspSessionError("source requested SETUP before providing wfd_presentation_URL")

        setup_response = self._request_expect_ok(
            "SETUP",
            self.presentation_url,
            headers={
                "Transport": (
                    f"RTP/AVP/UDP;unicast;client_port="
                    f"{self.args.wfd_client_rtp_port}-{self.args.wfd_client_rtp_port + 1}"
                )
            },
        )
        session_header = rtsp_header_value(setup_response.headers, "session")
        if not session_header:
            raise RtspSessionError("RTSP SETUP response did not include a Session header")
        self.session_id = session_header.split(";", 1)[0].strip()

        self._request_expect_ok(
            "PLAY",
            self.presentation_url,
            headers={"Range": "npt=0.000-"},
        )

        try:
            LOGGER.info("requesting initial IDR frame from source")
            self._request_expect_ok(
                "SET_PARAMETER",
                self.presentation_url,
                headers={"Content-Type": "text/parameters"},
                body=build_wfd_idr_request(),
            )
        except Exception as exc:
            LOGGER.warning("failed to request initial IDR frame: %s", exc)

        if self.keepalive_source_id:
            GLib.source_remove(self.keepalive_source_id)
        self.keepalive_source_id = GLib.timeout_add_seconds(
            self.args.rtsp_keepalive_interval_sec,
            self._keepalive,
        )

        self.handshake_complete = True

    def _send_server_response(self, request_headers, status_code, reason, extra_headers=None, body=b""):
        if extra_headers is None:
            extra_headers = {}

        body = body or b""
        lines = [f"RTSP/1.0 {status_code} {reason}"]
        cseq = rtsp_header_value(request_headers, "cseq")
        if cseq is not None:
            lines.append(f"CSeq: {cseq}")
        if self.session_id:
            lines.append(f"Session: {self.session_id}")
        for name, value in extra_headers.items():
            lines.append(f"{name}: {value}")
        if body:
            lines.append(f"Content-Length: {len(body)}")
        lines.append("")
        lines.append("")
        self._send_bytes("\r\n".join(lines).encode("utf-8") + body)

    def _keepalive(self):
        try:
            self._request_expect_ok(
                "GET_PARAMETER",
                self.aggregate_url or self.current_url,
                headers={"Content-Type": "text/parameters"},
            )
        except Exception as exc:
            LOGGER.error("RTSP keepalive failed: %s", exc)
            self.stop()
            return GLib.SOURCE_REMOVE
        return GLib.SOURCE_CONTINUE

    def stop(self):
        if self.keepalive_source_id:
            GLib.source_remove(self.keepalive_source_id)
            self.keepalive_source_id = 0
        if self.encoded_video_watchdog_source_id:
            GLib.source_remove(self.encoded_video_watchdog_source_id)
            self.encoded_video_watchdog_source_id = 0
        self.control_thread_stop.set()
        if self.control_socket is not None:
            self.control_socket.close()
            self.control_socket = None
        self.control_buffer.clear()
        if (
            self.control_thread is not None
            and self.control_thread.is_alive()
            and threading.current_thread() is not self.control_thread
        ):
            self.control_thread.join(timeout=1.0)
        self.control_thread = None
        self.control_thread_stop.clear()
        if self.rtcp_socket is not None:
            self.rtcp_socket.close()
            self.rtcp_socket = None
        if self.bus is not None:
            self.bus.remove_signal_watch()
            self.bus = None
        if self.pipeline is not None:
            self.pipeline.set_state(Gst.State.NULL)
            self.pipeline = None
        self.session_id = None
        self.aggregate_url = None
        self.current_url = None
        self.demux = None
        self.video_queue = None
        self.auxiliary_elements = []
        self.probe_black_log_budget = DEFAULT_PROBE_BLACK_LOG_BUDGET
        self.probe_non_black_logged = False
        self.probe_frame_count = 0
        self.input_rtp_log_budget = DEFAULT_INPUT_RTP_LOG_BUDGET
        self.input_rtp_idle_log_budget = DEFAULT_INPUT_RTP_IDLE_LOG_BUDGET
        self.input_rtp_packet_count = 0
        self.input_rtp_last_seq = None
        self.input_rtp_last_timestamp = None
        self.input_rtp_last_payload_type = None
        self.input_rtp_last_packet_monotonic = None
        self.encoded_video_log_budget = DEFAULT_ENCODED_VIDEO_LOG_BUDGET
        self.encoded_video_idle_log_budget = DEFAULT_ENCODED_VIDEO_IDLE_LOG_BUDGET
        self.encoded_video_buffer_count = 0
        self.encoded_video_last_pts_ms = None
        self.encoded_video_last_buffer_monotonic = None
        self.presentation_url = None
        self.sent_sink_options = False
        self.handshake_complete = False

    def _on_message(self, bus, message):
        del bus
        if message.type == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            LOGGER.error("relay pipeline error: %s", err)
            if debug:
                LOGGER.error("relay pipeline debug info: %s", debug)
            self.stop()
        elif message.type == Gst.MessageType.EOS:
            LOGGER.info("relay pipeline reached EOS")
            self.stop()


class MiceRequestHandler(socketserver.BaseRequestHandler):
    def handle(self):
        try:
            payload = recv_mice_message(self.request)
            source_ready = parse_source_ready_message(payload)
        except Exception as exc:
            LOGGER.warning("failed to parse MICE signalling from %s: %s", self.client_address[0], exc)
            return

        LOGGER.info(
            "received SOURCE_READY from %s (friendly='%s', rtsp-port=%u, source-id='%s')",
            self.client_address[0],
            source_ready.friendly_name or "unknown",
            source_ready.rtsp_port,
            source_ready.source_id or "unknown",
        )
        self.server.controller.on_source_ready(self.client_address[0], source_ready)


class MiceTcpServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, server_address, controller):
        super().__init__(server_address, MiceRequestHandler)
        self.controller = controller


class Application:
    def __init__(self, args):
        self.args = args
        self.loop = GLib.MainLoop()
        self.publisher = ServicePublisher(
            service_name=args.service_name,
            signalling_port=args.signalling_port,
            txt_records=[f"p2pMAC={args.p2p_mac_resolved}"],
            disabled=args.disable_discovery,
            host_name=args.service_host_name,
            host_address=args.bind_host,
        )
        self.renderer = RendererProcess(args)
        self.relay = RtspRelay(args)
        self.server = MiceTcpServer((args.bind_host, args.signalling_port), self)
        self.server_thread = threading.Thread(target=self.server.serve_forever, daemon=True)

    def start(self):
        self.renderer.ensure_started()
        self.publisher.start()
        self.server_thread.start()

        print(
            f"Advertising WFD-MICE sink '{self.args.service_name}' on {SERVICE_TYPE} port {self.args.signalling_port} ",
            flush=True,
            end="",
        )
        print(f"with p2pMAC={self.args.p2p_mac_resolved}.", flush=True)
        print(f"Listening for MICE signalling on {self.args.bind_host}:{self.args.signalling_port}.", flush=True)

        if not self.args.launch_renderer:
            print("Suggested renderer command:", flush=True)
            print(format_command(self.renderer.suggested_command()), flush=True)

        print(
            f"Waiting for Windows or GNOME Network Displays to connect, then relaying RTSP to "
            f"{self.args.relay_host}:{self.args.relay_port}.",
            flush=True,
        )

        self.loop.run()

    def on_source_ready(self, source_host, source_ready):
        GLib.idle_add(self._start_session, source_host, source_ready)

    def _start_session(self, source_host, source_ready):
        try:
            if self.args.launch_renderer:
                self.renderer.restart()
            else:
                self.renderer.ensure_started()
            self.relay.start(source_host, source_ready)
        except Exception as exc:
            LOGGER.error("failed to start WFD relay: %s", exc)
        return GLib.SOURCE_REMOVE

    def shutdown(self):
        self.server.shutdown()
        self.server.server_close()
        self.publisher.stop()
        self.relay.stop()
        self.renderer.stop()
        self.loop.quit()


def handle_signal(signum, frame):
    del signum
    del frame
    if APP is not None:
        APP.shutdown()


def validate_environment(args):
    if not args.disable_discovery and shutil.which("avahi-publish-service") is None:
        raise RuntimeError("avahi-publish-service is required unless --disable-discovery is used")

    for element_name in (
        "udpsrc",
        "rtpmp2tdepay",
        "tsdemux",
        "fakesink",
        "h264parse",
        "rtph264pay",
        "udpsink",
    ):
        require_element(element_name)

    if args.launch_renderer:
        renderer_argv = build_renderer_command(args)
        binary = renderer_argv[0]
        if "/" in binary:
            if not os.path.exists(binary):
                raise RuntimeError(f"renderer binary '{binary}' does not exist")
        elif shutil.which(binary) is None:
            raise RuntimeError(f"renderer binary '{binary}' is unavailable in PATH")


def build_example_source_ready(rtsp_port=DEFAULT_RTSP_PORT,
                               friendly_name="GNOME",
                               source_id="GnomeMICEDisplay"):
    friendly_payload = ("\ufeff" + friendly_name).encode("utf-16-le")
    message = bytearray()
    message.extend(b"\x00\x00")
    message.append(0x01)
    message.append(0x01)
    message.extend(b"\x00")
    message.extend(len(friendly_payload).to_bytes(2, "big"))
    message.extend(friendly_payload)
    message.extend(b"\x02\x00\x02")
    message.extend(rtsp_port.to_bytes(2, "big"))
    source_id_bytes = source_id.encode("ascii")
    message.extend(b"\x03")
    message.extend(len(source_id_bytes).to_bytes(2, "big"))
    message.extend(source_id_bytes)
    total_length = len(message)
    message[0:2] = total_length.to_bytes(2, "big")
    return bytes(message)


def run_self_test(args):
    example = build_example_source_ready()
    parsed = parse_source_ready_message(example)
    if parsed.friendly_name != "GNOME":
        raise AssertionError(f"unexpected friendly name {parsed.friendly_name!r}")
    if parsed.rtsp_port != DEFAULT_RTSP_PORT:
        raise AssertionError(f"unexpected RTSP port {parsed.rtsp_port}")
    if parsed.source_id != "GnomeMICEDisplay":
        raise AssertionError(f"unexpected source id {parsed.source_id!r}")

    renderer_pipeline = build_renderer_pipeline(args)
    if "rtph264depay" not in renderer_pipeline:
        raise AssertionError("renderer pipeline did not include rtph264depay")

    relay_pipeline = build_relay_pipeline_description(args)
    if "udpsrc" not in relay_pipeline or "rtpmp2tdepay" not in relay_pipeline:
        raise AssertionError("relay pipeline is missing required WFD elements")

    path_candidates = build_rtsp_path_candidates(DEFAULT_RTSP_PATH)
    if path_candidates != ["/wfd1.0/streamid=0", "/wfd1.0"]:
        raise AssertionError(f"unexpected RTSP path candidates {path_candidates!r}")

    control_url = resolve_rtsp_control_url(
        "rtsp://192.168.2.1:7236/wfd1.0",
        {"content-base": ["rtsp://192.168.2.1:7236/wfd1.0/"]},
        "v=0\r\na=control:*\r\nm=video 0 RTP/AVP 33\r\na=control:streamid=0\r\n",
    )
    if control_url != "rtsp://192.168.2.1:7236/wfd1.0/streamid=0":
        raise AssertionError(f"unexpected RTSP control URL {control_url!r}")

    try:
        wait_for_tcp_endpoint("127.0.0.1", 9, ready_timeout_sec=0.0, connect_timeout_sec=0.01)
    except RuntimeError as exc:
        if "last socket error" not in str(exc):
            raise AssertionError(f"unexpected RTSP probe error message {exc!r}") from exc
    else:
        raise AssertionError("expected TCP endpoint probe to fail for discard port test")

    print("self-test passed", flush=True)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Advertise a WFD-MICE sink for GNOME and Windows, then relay the source RTSP session into breezy_drm_scene_demo."
    )
    parser.add_argument("--service-name", default=DEFAULT_SERVICE_NAME, help=f"mDNS service name to advertise (default: {DEFAULT_SERVICE_NAME!r})")
    parser.add_argument("--service-host-name", help="mDNS host name to publish for the sink address (default: derived from --service-name)")
    parser.add_argument("--interface", help="network interface whose MAC address should be used for the p2pMAC TXT record")
    parser.add_argument("--p2p-mac", help="override the p2pMAC TXT record value")
    parser.add_argument("--bind-host", default="0.0.0.0", help="TCP address to bind the MICE signalling listener to (default: 0.0.0.0)")
    parser.add_argument("--signalling-port", type=int, default=DEFAULT_SIGNALLING_PORT, help=f"MICE signalling TCP port (default: {DEFAULT_SIGNALLING_PORT})")
    parser.add_argument("--rtsp-path", default=DEFAULT_RTSP_PATH, help=f"RTSP path exposed by the source (default: {DEFAULT_RTSP_PATH})")
    parser.add_argument("--rtsp-ready-timeout-sec", type=float, default=DEFAULT_RTSP_READY_TIMEOUT_SEC, help=f"how long to wait for the source RTSP TCP port to become reachable before failing (default: {DEFAULT_RTSP_READY_TIMEOUT_SEC})")
    parser.add_argument("--rtsp-connect-timeout-sec", type=float, default=DEFAULT_RTSP_CONNECT_TIMEOUT_SEC, help=f"per-attempt TCP connect timeout for the source RTSP probe (default: {DEFAULT_RTSP_CONNECT_TIMEOUT_SEC})")
    parser.add_argument("--rtsp-keepalive-interval-sec", type=int, default=DEFAULT_RTSP_KEEPALIVE_INTERVAL_SEC, help=f"interval for RTSP GET_PARAMETER keepalives once playback starts (default: {DEFAULT_RTSP_KEEPALIVE_INTERVAL_SEC})")
    parser.add_argument("--wfd-client-rtp-port", type=int, default=DEFAULT_WFD_CLIENT_RTP_PORT, help=f"local RTP port exposed to the source during SETUP; RTCP uses the next port (default: {DEFAULT_WFD_CLIENT_RTP_PORT})")
    parser.add_argument("--relay-host", default=DEFAULT_RELAY_HOST, help=f"host address for the local RTP relay (default: {DEFAULT_RELAY_HOST})")
    parser.add_argument("--relay-port", type=int, default=DEFAULT_RELAY_PORT, help=f"UDP port for the local RTP relay (default: {DEFAULT_RELAY_PORT})")
    parser.add_argument("--payload-type", type=int, default=DEFAULT_PAYLOAD_TYPE, help=f"RTP payload type for the relay stream (default: {DEFAULT_PAYLOAD_TYPE})")
    parser.add_argument("--disable-discovery", action="store_true", help="skip Avahi service publication and only run the TCP signalling listener")
    parser.add_argument("--launch-renderer", action="store_true", help="start breezy_drm_scene_demo automatically with the recommended static relay pipeline")
    parser.add_argument("--renderer-binary", default="./breezy_drm_scene_demo", help="renderer binary to launch when --launch-renderer is set")
    parser.add_argument("--renderer-device", default="/dev/dri/card0", help="DRM device to pass to the renderer launcher")
    parser.add_argument("--renderer-decoder-fragment", default=DEFAULT_DECODER_FRAGMENT, help=f"decoder fragment appended after h264parse in the renderer pipeline (default: {DEFAULT_DECODER_FRAGMENT})")
    parser.add_argument("--renderer-command", help="full renderer command template; supports {renderer_pipeline}, {relay_host}, {relay_port}, and {payload_type}")
    parser.add_argument("--renderer-extra-arg", action="append", default=[], help="extra renderer argument to append when using the default renderer launcher; repeat as needed")
    parser.add_argument("--self-test", action="store_true", help="run a local protocol and pipeline self-test, then exit")
    parser.add_argument("--verbose", action="store_true", help="print the generated GStreamer relay pipeline")

    args = parser.parse_args(argv)
    try:
        args.p2p_mac_resolved = choose_p2p_mac(args)
        args.bind_host = resolve_bind_host(args)
        if not args.service_host_name:
            args.service_host_name = build_service_host_name(
                args.service_name,
                args.bind_host,
                args.p2p_mac_resolved,
            )
    except Exception as exc:
        parser.error(str(exc))

    try:
        ipaddress.ip_address(args.bind_host)
    except ValueError:
        if args.bind_host != "0.0.0.0":
            parser.error("--bind-host must be 0.0.0.0 or a valid IPv4/IPv6 address")

    if args.signalling_port < 1 or args.signalling_port > 65535:
        parser.error("--signalling-port must be between 1 and 65535")
    if args.relay_port < 1 or args.relay_port > 65535:
        parser.error("--relay-port must be between 1 and 65535")
    if args.payload_type < 0 or args.payload_type > 127:
        parser.error("--payload-type must be between 0 and 127")
    if args.rtsp_ready_timeout_sec < 0:
        parser.error("--rtsp-ready-timeout-sec must be non-negative")
    if args.rtsp_connect_timeout_sec <= 0:
        parser.error("--rtsp-connect-timeout-sec must be greater than zero")
    if args.rtsp_keepalive_interval_sec <= 0:
        parser.error("--rtsp-keepalive-interval-sec must be greater than zero")
    if args.wfd_client_rtp_port < 1 or args.wfd_client_rtp_port > 65534:
        parser.error("--wfd-client-rtp-port must be between 1 and 65534")

    return args


def main(argv):
    global APP

    args = parse_args(argv)
    logging.basicConfig(
        level=logging.INFO,
        format="%(levelname)s: %(message)s",
    )

    if args.self_test:
        run_self_test(args)
        return 0

    try:
        ensure_runtime_dependencies()
        Gst.init(None)
        validate_environment(args)
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    APP = Application(args)
    signal.signal(signal.SIGTERM, handle_signal)
    signal.signal(signal.SIGINT, handle_signal)

    try:
        APP.start()
    except KeyboardInterrupt:
        APP.shutdown()
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        if APP is not None:
            APP.shutdown()
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))