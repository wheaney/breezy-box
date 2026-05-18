#!/usr/bin/env python3

import argparse
import logging
import signal
import sys


LOGGER = logging.getLogger("wired_projection_gnome_sender")
SCREEN_CAST_IFACE = "org.gnome.Mutter.ScreenCast"

GLib = None
Gst = None
pydbus = None


def ensure_runtime_dependencies():
    global GLib
    global Gst
    global pydbus

    if GLib is not None and Gst is not None and pydbus is not None:
        return

    try:
        import pydbus as imported_pydbus
        import gi

        gi.require_version("GLib", "2.0")
        gi.require_version("Gst", "1.0")

        from gi.repository import GLib as imported_glib
        from gi.repository import Gst as imported_gst
    except ImportError as exc:
        raise RuntimeError(
            "missing runtime dependencies; install python3-pydbus, python3-gi, "
            "gir1.2-gstreamer-1.0, and gstreamer1.0-pipewire"
        ) from exc

    pydbus = imported_pydbus
    GLib = imported_glib
    Gst = imported_gst


def require_element(factory_name):
    element = Gst.ElementFactory.make(factory_name, None)
    if element is None:
        raise RuntimeError(f"required GStreamer element '{factory_name}' is unavailable")


def build_pipeline_description(args, node_id):
    base = (
        "pipewiresrc path={node_id} do-timestamp=true keepalive-time=1000 ! "
        "queue max-size-buffers=2 leaky=downstream ! "
        "videoconvert ! videoscale ! "
        "video/x-raw,format=I420,width={width},height={height},framerate={framerate}/1 ! "
    )
    if args.codec == "h265":
        return (
            base +
            "x265enc tune=zerolatency speed-preset=ultrafast bitrate={bitrate_kbps} key-int-max={keyframe_interval} ! "
            "h265parse ! rtph265pay pt={payload_type} config-interval=1 ! "
            "udpsink host={sink_host} port={sink_port} sync=false async=false"
        ).format(
            node_id=node_id,
            width=args.width,
            height=args.height,
            framerate=args.framerate,
            bitrate_kbps=args.bitrate_kbps,
            keyframe_interval=max(1, args.framerate),
            payload_type=args.payload_type,
            sink_host=args.sink_host,
            sink_port=args.sink_port,
        )

    return (
        base +
        "vp9enc deadline=1 cpu-used=8 row-mt=true target-bitrate={target_bitrate} keyframe-max-dist={keyframe_interval} ! "
        "rtpvp9pay pt={payload_type} picture-id-mode=7-bit ! "
        "udpsink host={sink_host} port={sink_port} sync=false async=false"
    ).format(
        node_id=node_id,
        width=args.width,
        height=args.height,
        framerate=args.framerate,
        target_bitrate=args.bitrate_kbps * 1000,
        keyframe_interval=max(1, args.framerate),
        payload_type=args.payload_type,
        sink_host=args.sink_host,
        sink_port=args.sink_port,
    )


class WiredProjectionSession:
    def __init__(self, args, on_closed_cb):
        self.args = args
        self.on_closed_cb = on_closed_cb
        self.screen_cast_session = None
        self.stream = None
        self.pipeline = None

    def create(self):
        bus = pydbus.SessionBus()
        screen_cast = bus.get(SCREEN_CAST_IFACE, "/org/gnome/Mutter/ScreenCast")
        session_path = screen_cast.CreateSession([])
        self.screen_cast_session = bus.get(SCREEN_CAST_IFACE, session_path)
        self.screen_cast_session.onClosed = self._on_session_closed

        stream_path = self.screen_cast_session.RecordVirtual({
            "is-platform": GLib.Variant.new_boolean(True),
        })
        self.stream = bus.get(SCREEN_CAST_IFACE, stream_path)
        self.stream.onPipeWireStreamAdded = self._on_pipewire_stream_added
        self.screen_cast_session.Start()

        print(
            "Created GNOME virtual display. Open Settings > Displays to enable or arrange it if needed.",
            flush=True,
        )

    def terminate(self):
        try:
            if self.stream is not None:
                self.stream.Stop()
        except Exception as exc:
            LOGGER.error("Failed to stop screencast stream: %s", exc)

        try:
            if self.pipeline is not None:
                self.pipeline.send_event(Gst.Event.new_eos())
                self.pipeline.set_state(Gst.State.NULL)
        except Exception as exc:
            LOGGER.error("Failed to stop sender pipeline: %s", exc)

        self.pipeline = None
        self.stream = None
        self.on_closed_cb()

    def _on_session_closed(self):
        self.terminate()

    def _on_message(self, bus, message):
        del bus
        if message.type == Gst.MessageType.EOS:
            LOGGER.info("sender pipeline reached EOS")
            self.terminate()
        elif message.type == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            LOGGER.error("sender pipeline error: %s", err)
            if debug:
                LOGGER.error("sender pipeline debug info: %s", debug)
            self.terminate()

    def _on_pipewire_stream_added(self, node_id):
        description = build_pipeline_description(self.args, node_id)

        print(
            f"Streaming GNOME virtual display via {self.args.codec.upper()} RTP to "
            f"{self.args.sink_host}:{self.args.sink_port} (PipeWire node {node_id}).",
            flush=True,
        )
        if self.args.verbose:
            print(f"GStreamer sender pipeline: {description}", flush=True)

        self.pipeline = Gst.parse_launch(description)
        self.pipeline.get_bus().connect("message", self._on_message)
        self.pipeline.set_state(Gst.State.PLAYING)


SESSION = None
LOOP = None


def handle_signal(signum, frame):
    del signum
    del frame
    global SESSION
    if SESSION is not None:
        SESSION.terminate()


def validate_environment(args):
    bus = pydbus.SessionBus()
    bus.get(SCREEN_CAST_IFACE, "/org/gnome/Mutter/ScreenCast")

    require_element("pipewiresrc")
    require_element("videoconvert")
    require_element("videoscale")
    require_element("udpsink")
    if args.codec == "h265":
        require_element("x265enc")
        require_element("h265parse")
        require_element("rtph265pay")
    else:
        require_element("vp9enc")
        require_element("rtpvp9pay")


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Create a GNOME virtual monitor and stream it over the wired Breezy Box link."
    )
    parser.add_argument("--sink-host", required=True, help="SBC IP address that is running breezy_drm_scene_demo")
    parser.add_argument("--sink-port", type=int, default=5600, help="UDP port on the SBC receiver (default: 5600)")
    parser.add_argument("--width", type=int, default=1920, help="Virtual monitor width (default: 1920)")
    parser.add_argument("--height", type=int, default=1080, help="Virtual monitor height (default: 1080)")
    parser.add_argument("--framerate", type=int, default=60, help="Virtual monitor frame rate (default: 60)")
    parser.add_argument("--codec", choices=("h265", "vp9"), default="h265", help="Video codec to send (default: h265)")
    parser.add_argument("--bitrate-kbps", type=int, default=6000, help="Encoder target bitrate in kbps (default: 6000)")
    parser.add_argument("--payload-type", type=int, default=96, help="RTP payload type (default: 96)")
    parser.add_argument("--verbose", action="store_true", help="Print the generated sender pipeline")
    return parser.parse_args(argv)


def main(argv):
    global LOOP
    global SESSION

    args = parse_args(argv)
    logging.basicConfig(level=logging.INFO if args.verbose else logging.WARNING,
                        format="%(levelname)s: %(message)s")

    try:
        ensure_runtime_dependencies()
        Gst.init(None)
        validate_environment(args)
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    signal.signal(signal.SIGTERM, handle_signal)
    signal.signal(signal.SIGINT, handle_signal)

    LOOP = GLib.MainLoop()
    SESSION = WiredProjectionSession(args, LOOP.quit)
    GLib.idle_add(SESSION.create)

    try:
        LOOP.run()
    except KeyboardInterrupt:
        if SESSION is not None:
            SESSION.terminate()

    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))