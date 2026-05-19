#!/usr/bin/env python3

import argparse
import fcntl
import hashlib
import importlib
import ipaddress
import logging
import os
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
DEFAULT_RTSP_KEEPALIVE_INTERVAL_SEC = 15
DEFAULT_WFD_CLIENT_RTP_PORT = 16384
DEFAULT_RELAY_HOST = "127.0.0.1"
DEFAULT_RELAY_PORT = 5600
DEFAULT_PAYLOAD_TYPE = 96
DEFAULT_DECODER_FRAGMENT = "mppvideodec"

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


def build_service_host_name(service_name, bind_host=None):
    label = re.sub(r"[^a-z0-9-]+", "-", service_name.lower()).strip("-")
    if not label:
        label = "breezy-box"
    if bind_host:
        try:
            ip_obj = ipaddress.ip_address(bind_host)
            if isinstance(ip_obj, ipaddress.IPv4Address):
                label = f"{label}-{'-'.join(bind_host.split('.'))}"
            else:
                label = f"{label}-{str(ip_obj).replace(':', '-')[:24].strip('-')}"
        except ValueError:
            pass
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
        '! rtph264depay ! h264parse ! {decoder_fragment}'
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
        '[dynamic video/x-h264 pad] ! queue max-size-buffers=8 leaky=downstream ! '
        'h264parse config-interval=-1 ! '
        'rtph264pay pt={payload_type} config-interval=1 ! '
        'udpsink host={relay_host} port={relay_port} sync=false async=false'
    ).format(
        wfd_client_rtp_port=args.wfd_client_rtp_port,
        payload_type=args.payload_type,
        relay_host=args.relay_host,
        relay_port=args.relay_port,
    )


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

        argv = [publisher]
        if self.host_name:
            argv.extend(["-H", self.host_name])
        argv.extend([self.service_name, SERVICE_TYPE, str(self.signalling_port)])
        argv.extend(self.txt_records)
        if self.host_name and self.host_address:
            LOGGER.info(
                "publishing %s on %s via Avahi as %s -> %s",
                self.service_name,
                SERVICE_TYPE,
                self.host_name,
                self.host_address,
            )
            self.address_process = subprocess.Popen([address_publisher, self.host_name, self.host_address])
        else:
            LOGGER.info("publishing %s on %s via Avahi", self.service_name, SERVICE_TYPE)
        self.service_process = subprocess.Popen(argv)

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
        self.control_reader = None
        self.rtcp_socket = None
        self.cseq = 0
        self.session_id = None
        self.aggregate_url = None
        self.keepalive_source_id = 0
        self.demux = None
        self.video_queue = None

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

        self._open_control_connection(connected_socket)
        self.aggregate_url = next_url
        self.current_url = next_url

        self._request_expect_ok("OPTIONS", next_url)
        describe_response = self._describe_with_fallback()
        sdp_text = describe_response.body.decode("utf-8", errors="replace")
        self.aggregate_url = rtsp_header_value(describe_response.headers, "content-base", next_url)
        control_url = resolve_rtsp_control_url(self.aggregate_url, describe_response.headers, sdp_text)

        setup_response = self._request_expect_ok(
            "SETUP",
            control_url,
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
            self.aggregate_url,
            headers={"Range": "npt=0.000-"},
        )
        self.keepalive_source_id = GLib.timeout_add_seconds(
            self.args.rtsp_keepalive_interval_sec,
            self._keepalive,
        )

    def _build_relay_pipeline(self):
        pipeline = Gst.Pipeline.new("wfd-relay")
        if pipeline is None:
            raise RuntimeError("failed to create WFD relay pipeline")

        udpsrc = Gst.ElementFactory.make("udpsrc", "wfd_udpsrc")
        source_queue = Gst.ElementFactory.make("queue", "wfd_source_queue")
        depay = Gst.ElementFactory.make("rtpmp2tdepay", "wfd_rtpmp2tdepay")
        demux = Gst.ElementFactory.make("tsdemux", "wfd_tsdemux")
        video_queue = Gst.ElementFactory.make("queue", "wfd_video_queue")
        parser = Gst.ElementFactory.make("h264parse", "wfd_h264parse")
        pay = Gst.ElementFactory.make("rtph264pay", "wfd_rtph264pay")
        sink = Gst.ElementFactory.make("udpsink", "wfd_udpsink")

        elements = [udpsrc, source_queue, depay, demux, video_queue, parser, pay, sink]
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
        parser.set_property("config-interval", -1)
        pay.set_property("pt", self.args.payload_type)
        pay.set_property("config-interval", 1)
        sink.set_property("host", self.args.relay_host)
        sink.set_property("port", self.args.relay_port)
        sink.set_property("sync", False)
        sink.set_property("async", False)

        pipeline.add(udpsrc)
        pipeline.add(source_queue)
        pipeline.add(depay)
        pipeline.add(demux)
        pipeline.add(video_queue)
        pipeline.add(parser)
        pipeline.add(pay)
        pipeline.add(sink)

        for upstream, downstream in ((udpsrc, source_queue),
                                     (source_queue, depay),
                                     (depay, demux),
                                     (video_queue, parser),
                                     (parser, pay),
                                     (pay, sink)):
            if not upstream.link(downstream):
                raise RuntimeError(
                    f"failed to link WFD relay element {upstream.get_name()} -> {downstream.get_name()}"
                )

        demux.connect("pad-added", self._on_demux_pad_added, video_queue)
        self.demux = demux
        self.video_queue = video_queue
        return pipeline

    def _on_demux_pad_added(self, demux, pad, video_queue):
        del demux
        caps = pad.get_current_caps() or pad.query_caps(None)
        structure_name = None
        if caps is not None and caps.get_size() > 0:
            structure_name = caps.get_structure(0).get_name()

        if structure_name != "video/x-h264":
            if self.args.verbose:
                print(f"Ignoring demux pad with caps: {caps.to_string() if caps is not None else 'unknown'}", flush=True)
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
        self.control_reader = self.control_socket.makefile("rb")
        self.cseq = 0

    def _send_bytes(self, payload):
        if self.control_socket is None:
            raise RtspSessionError("RTSP control socket is not connected")
        self.control_socket.sendall(payload)

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

            self._handle_server_request(first_line, headers)

    def _read_message(self):
        if self.control_reader is None:
            raise RtspSessionError("RTSP control reader is unavailable")

        first_line = self.control_reader.readline()
        if not first_line:
            raise RtspSessionError("RTSP control connection closed unexpectedly")
        first_line = first_line.decode("utf-8", errors="replace").rstrip("\r\n")

        header_lines = []
        while True:
            line = self.control_reader.readline()
            if not line:
                raise RtspSessionError("RTSP control connection closed while reading headers")
            line = line.decode("utf-8", errors="replace").rstrip("\r\n")
            if not line:
                break
            header_lines.append(line)

        headers = parse_rtsp_headers(header_lines)
        content_length = int(rtsp_header_value(headers, "content-length", "0") or "0")
        body = self.control_reader.read(content_length) if content_length > 0 else b""
        if content_length > 0 and len(body) != content_length:
            raise RtspSessionError("RTSP control connection closed while reading body")

        return first_line, headers, body

    def _handle_server_request(self, first_line, headers):
        parts = first_line.split(" ", 2)
        if len(parts) < 3:
            raise RtspSessionError(f"malformed RTSP request line: {first_line!r}")

        method = parts[0]
        if self.args.verbose:
            print(f"RTSP server request: {first_line}", flush=True)

        response_headers = {}
        if method == "OPTIONS":
            response_headers["Public"] = "OPTIONS, DESCRIBE, SETUP, PLAY, PAUSE, TEARDOWN, GET_PARAMETER, SET_PARAMETER"
        elif method == "GET_PARAMETER":
            response_headers["Content-Type"] = "text/parameters"

        self._send_server_response(headers, 200, "OK", response_headers)

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
        if self.control_reader is not None:
            self.control_reader.close()
            self.control_reader = None
        if self.control_socket is not None:
            self.control_socket.close()
            self.control_socket = None
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
            args.service_host_name = build_service_host_name(args.service_name, args.bind_host)
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