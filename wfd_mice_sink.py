#!/usr/bin/env python3

import argparse
import hashlib
import importlib
import logging
import os
import shlex
import shutil
import signal
import socket
import socketserver
import subprocess
import sys
import threading
from dataclasses import dataclass
from typing import Dict, Optional


LOGGER = logging.getLogger("wfd_mice_sink")
SERVICE_TYPE = "_display._tcp"
DEFAULT_SERVICE_NAME = "Breezy Box"
DEFAULT_SIGNALLING_PORT = 7250
DEFAULT_RTSP_PORT = 7236
DEFAULT_RTSP_PATH = "/wfd1.0/streamid=0"
DEFAULT_RELAY_HOST = "127.0.0.1"
DEFAULT_RELAY_PORT = 5600
DEFAULT_PAYLOAD_TYPE = 96
DEFAULT_DECODER_FRAGMENT = "mppvideodec"

GLib = None
Gst = None
APP = None


class MiceProtocolError(Exception):
    pass


@dataclass
class SourceReadyMessage:
    version: int
    command: int
    friendly_name: str
    rtsp_port: int
    source_id: str
    tlvs: Dict[int, bytes]


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


def derive_local_admin_mac(seed_text):
    digest = hashlib.sha256(seed_text.encode("utf-8")).digest()
    octets = [0x02, digest[0], digest[1], digest[2], digest[3], digest[4]]
    return ":".join(f"{value:02X}" for value in octets)


def choose_p2p_mac(args):
    if args.p2p_mac:
        return normalize_mac(args.p2p_mac)
    if args.interface:
        try:
            return read_interface_mac(args.interface)
        except OSError as exc:
            raise RuntimeError(f"failed to read MAC address for interface '{args.interface}': {exc}") from exc
    return derive_local_admin_mac(args.service_name or socket.gethostname())


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


def is_rtsp_not_found_error(err, debug):
    err_text = str(err).lower()
    debug_text = (debug or "").lower()
    return "404" in err_text or "not found" in err_text or "404" in debug_text or "not found" in debug_text


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


def build_relay_pipeline_description(args, source_host, rtsp_port, rtsp_path):
    location = build_rtsp_url(source_host, rtsp_port, rtsp_path)
    return (
        'rtspsrc location="{location}" latency={latency_ms} do-rtsp-keep-alive=true name=src '
        'src. ! queue max-size-buffers=8 ! '
        'application/x-rtp,media=video,encoding-name=MP2T,payload=33,clock-rate=90000 ! '
        'rtpmp2tdepay ! tsdemux name=demux '
        'demux. ! queue max-size-buffers=8 leaky=downstream ! '
        'h264parse config-interval=-1 ! '
        'rtph264pay pt={payload_type} config-interval=1 ! '
        'udpsink host={relay_host} port={relay_port} sync=false async=false'
    ).format(
        location=location,
        latency_ms=args.latency_ms,
        payload_type=args.payload_type,
        relay_host=args.relay_host,
        relay_port=args.relay_port,
    )


def format_command(argv):
    return " ".join(shlex.quote(part) for part in argv)


class ServicePublisher:
    def __init__(self, service_name, signalling_port, txt_records, disabled):
        self.service_name = service_name
        self.signalling_port = signalling_port
        self.txt_records = txt_records
        self.disabled = disabled
        self.process = None

    def start(self):
        if self.disabled:
            LOGGER.info("service advertisement disabled")
            return

        publisher = shutil.which("avahi-publish-service")
        if not publisher:
            raise RuntimeError("avahi-publish-service is required for WFD-MICE discovery")

        argv = [publisher, self.service_name, SERVICE_TYPE, str(self.signalling_port)]
        argv.extend(self.txt_records)
        LOGGER.info("publishing %s on %s via Avahi", self.service_name, SERVICE_TYPE)
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
        self.retry_source_id = 0

    def start(self, source_host, source_ready):
        self.stop()
        self.current_source_host = source_host
        self.current_source_ready = source_ready
        self.path_candidates = build_rtsp_path_candidates(self.args.rtsp_path)
        self.path_index = 0
        self._start_current_candidate()

    def _start_current_candidate(self):
        rtsp_path = self.path_candidates[self.path_index]
        description = build_relay_pipeline_description(self.args,
                                                       self.current_source_host,
                                                       self.current_source_ready.rtsp_port,
                                                       rtsp_path)
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

        self.pipeline = Gst.parse_launch(description)
        self.bus = self.pipeline.get_bus()
        self.bus.add_signal_watch()
        self.bus.connect("message", self._on_message)
        state_result = self.pipeline.set_state(Gst.State.PLAYING)
        if state_result == Gst.StateChangeReturn.FAILURE:
            self.stop()
            raise RuntimeError(f"failed to start relay pipeline for {next_url}")
        self.current_url = next_url

    def _retry_next_candidate(self):
        self.retry_source_id = 0
        if self.current_source_host is None or self.current_source_ready is None:
            return GLib.SOURCE_REMOVE
        if self.path_index + 1 >= len(self.path_candidates):
            return GLib.SOURCE_REMOVE

        self.path_index += 1
        try:
            self._start_current_candidate()
        except Exception as exc:
            LOGGER.error("failed to retry WFD relay: %s", exc)
        return GLib.SOURCE_REMOVE

    def stop(self):
        if self.retry_source_id:
            GLib.source_remove(self.retry_source_id)
            self.retry_source_id = 0
        if self.bus is not None:
            self.bus.remove_signal_watch()
            self.bus = None
        if self.pipeline is not None:
            self.pipeline.set_state(Gst.State.NULL)
            self.pipeline = None
        self.current_url = None

    def _on_message(self, bus, message):
        del bus
        if message.type == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            failed_url = self.current_url
            if is_rtsp_not_found_error(err, debug) and self.path_index + 1 < len(self.path_candidates):
                next_path = self.path_candidates[self.path_index + 1]
                next_url = build_rtsp_url(self.current_source_host,
                                          self.current_source_ready.rtsp_port,
                                          next_path)
                LOGGER.warning("RTSP path %s returned 404; retrying with %s", failed_url, next_url)
                self.stop()
                self.retry_source_id = GLib.idle_add(self._retry_next_candidate)
                return
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
        "rtspsrc",
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

    relay_pipeline = build_relay_pipeline_description(args, "192.168.2.1", DEFAULT_RTSP_PORT, args.rtsp_path)
    if "rtspsrc" not in relay_pipeline or "rtpmp2tdepay" not in relay_pipeline:
        raise AssertionError("relay pipeline is missing required WFD elements")

    path_candidates = build_rtsp_path_candidates(DEFAULT_RTSP_PATH)
    if path_candidates != ["/wfd1.0/streamid=0", "/wfd1.0"]:
        raise AssertionError(f"unexpected RTSP path candidates {path_candidates!r}")

    print("self-test passed", flush=True)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Advertise a WFD-MICE sink for GNOME and Windows, then relay the source RTSP session into breezy_drm_scene_demo."
    )
    parser.add_argument("--service-name", default=DEFAULT_SERVICE_NAME, help=f"mDNS service name to advertise (default: {DEFAULT_SERVICE_NAME!r})")
    parser.add_argument("--interface", help="network interface whose MAC address should be used for the p2pMAC TXT record")
    parser.add_argument("--p2p-mac", help="override the p2pMAC TXT record value")
    parser.add_argument("--bind-host", default="0.0.0.0", help="TCP address to bind the MICE signalling listener to (default: 0.0.0.0)")
    parser.add_argument("--signalling-port", type=int, default=DEFAULT_SIGNALLING_PORT, help=f"MICE signalling TCP port (default: {DEFAULT_SIGNALLING_PORT})")
    parser.add_argument("--rtsp-path", default=DEFAULT_RTSP_PATH, help=f"RTSP path exposed by the source (default: {DEFAULT_RTSP_PATH})")
    parser.add_argument("--latency-ms", type=int, default=120, help="rtspsrc latency for the relay client in milliseconds (default: 120)")
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
    except Exception as exc:
        parser.error(str(exc))

    if args.signalling_port < 1 or args.signalling_port > 65535:
        parser.error("--signalling-port must be between 1 and 65535")
    if args.relay_port < 1 or args.relay_port > 65535:
        parser.error("--relay-port must be between 1 and 65535")
    if args.payload_type < 0 or args.payload_type > 127:
        parser.error("--payload-type must be between 0 and 127")
    if args.latency_ms < 0:
        parser.error("--latency-ms must be non-negative")

    return args


def main(argv):
    global APP

    args = parse_args(argv)
    logging.basicConfig(
        level=logging.INFO if args.verbose else logging.WARNING,
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