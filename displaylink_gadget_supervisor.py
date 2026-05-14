#!/usr/bin/env python3

import argparse
import json
import os
import shlex
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Optional


DEFAULT_BINARY_NAME = "displaylink_gadget_raw_gadget"
DEFAULT_POLL_INTERVAL_SEC = 1.0
STOP_REQUESTED = False


class ConfigError(Exception):
	pass


@dataclass
class DesiredDisplay:
	name: str
	argv: List[str]
	signature: str


@dataclass
class ManagedDisplay:
	name: str
	argv: List[str]
	signature: str
	process: subprocess.Popen


def on_signal(signo, frame):
	global STOP_REQUESTED
	del signo, frame
	STOP_REQUESTED = True


def ensure_object(value, label):
	if value is None:
		return {}
	if not isinstance(value, dict):
		raise ConfigError(f"{label} must be a JSON object")
	return value


def ensure_list(value, label):
	if not isinstance(value, list):
		raise ConfigError(f"{label} must be a JSON array")
	return value


def ensure_string(value, label):
	if not isinstance(value, str) or not value:
		raise ConfigError(f"{label} must be a non-empty string")
	return value


def ensure_bool(value, label):
	if not isinstance(value, bool):
		raise ConfigError(f"{label} must be a boolean")
	return value


def ensure_positive_int(value, label):
	if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
		raise ConfigError(f"{label} must be a positive integer")
	return value


def ensure_positive_number(value, label):
	if isinstance(value, bool) or not isinstance(value, (int, float)) or value <= 0:
		raise ConfigError(f"{label} must be a positive number")
	return float(value)


def resolve_path(base_dir, path_value, label):
	path = ensure_string(path_value, label)
	if os.path.isabs(path):
		return path
	return os.path.normpath(os.path.join(base_dir, path))


def format_hex16(value, label):
	parsed = value

	if isinstance(parsed, str):
		try:
			parsed = int(parsed, 0)
		except ValueError as exc:
			raise ConfigError(f"{label} must be an integer or hex string") from exc
	elif isinstance(parsed, bool) or not isinstance(parsed, int):
		raise ConfigError(f"{label} must be an integer or hex string")

	if parsed < 0 or parsed > 0xFFFF:
		raise ConfigError(f"{label} must fit in 16 bits")

	return f"0x{parsed:04x}"


def append_option(argv, flag, value):
	argv.extend([flag, value])


def build_display(display_index, defaults, display, config_dir, binary_path):
	merged = dict(defaults)
	merged.update(display)

	enabled = merged.get("enabled", True)
	if not ensure_bool(enabled, f"displays[{display_index}].enabled"):
		return None

	name = merged.get("name", f"display-{display_index + 1}")
	name = ensure_string(name, f"displays[{display_index}].name")

	udc_device = ensure_string(merged.get("udc_device"), f"displays[{display_index}].udc_device")
	udc_driver = merged.get("udc_driver", udc_device)
	udc_driver = ensure_string(udc_driver, f"displays[{display_index}].udc_driver")

	serial_string = merged.get("serial_string", f"BREEZY{display_index + 1:04d}")
	serial_string = ensure_string(serial_string, f"displays[{display_index}].serial_string")

	monitor_name = merged.get("monitor_name", name)
	monitor_name = ensure_string(monitor_name, f"displays[{display_index}].monitor_name")

	argv = [binary_path]
	append_option(argv, "--udc-device", udc_device)
	append_option(argv, "--udc-driver", udc_driver)
	append_option(argv, "--serial-string", serial_string)
	append_option(argv, "--monitor-name", monitor_name)

	string_options = (
		("raw_device_path", "--raw-device", True),
		("edid_path", "--edid-file", True),
		("capture_stream_path", "--capture-stream", True),
		("dump_image_path", "--dump-image", True),
		("manufacturer_string", "--manufacturer", False),
		("product_string", "--product-string", False),
		("usb_speed", "--usb-speed", False),
	)
	for key, flag, is_path in string_options:
		value = merged.get(key)
		if value is None:
			continue
		if is_path:
			value = resolve_path(config_dir, value, f"displays[{display_index}].{key}")
		else:
			value = ensure_string(value, f"displays[{display_index}].{key}")
		append_option(argv, flag, value)

	int_options = (
		("vendor_id", "--vendor-id", True),
		("product_id", "--product-id", True),
		("decode_width", "--decode-width", False),
		("decode_height", "--decode-height", False),
		("window_scale", "--window-scale", False),
	)
	for key, flag, is_hex in int_options:
		value = merged.get(key)
		if value is None:
			continue
		if is_hex:
			append_option(argv, flag, format_hex16(value, f"displays[{display_index}].{key}"))
		else:
			append_option(argv, flag, str(ensure_positive_int(value, f"displays[{display_index}].{key}")))

	show_window = merged.get("show_window", False)
	show_window = ensure_bool(show_window, f"displays[{display_index}].show_window")
	decode_stream = merged.get("decode_stream", True)
	decode_stream = ensure_bool(decode_stream, f"displays[{display_index}].decode_stream")
	startup_soft_reconnect = merged.get("startup_soft_reconnect", True)
	startup_soft_reconnect = ensure_bool(startup_soft_reconnect, f"displays[{display_index}].startup_soft_reconnect")
	verbose = merged.get("verbose", False)
	verbose = ensure_bool(verbose, f"displays[{display_index}].verbose")

	if show_window:
		argv.append("--show-window")
	if not decode_stream:
		argv.append("--no-decode")
	if not startup_soft_reconnect:
		argv.append("--no-startup-reconnect")
	if verbose:
		argv.append("--verbose")

	if not decode_stream and merged.get("dump_image_path"):
		raise ConfigError(
			f"displays[{display_index}] sets dump_image_path but decode_stream is false")
	if not decode_stream and show_window:
		raise ConfigError(
			f"displays[{display_index}] sets show_window but decode_stream is false")

	extra_args = merged.get("extra_args", [])
	if not isinstance(extra_args, list):
		raise ConfigError(f"displays[{display_index}].extra_args must be a JSON array")
	for arg_index, extra_arg in enumerate(extra_args):
		argv.append(ensure_string(extra_arg,
			f"displays[{display_index}].extra_args[{arg_index}]"))

	signature = json.dumps(argv, separators=(",", ":"))
	return DesiredDisplay(name=name, argv=argv, signature=signature)


def load_config(config_path):
	config_dir = os.path.dirname(config_path)
	default_binary_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), DEFAULT_BINARY_NAME)

	with open(config_path, "r", encoding="utf-8") as file_obj:
		config = json.load(file_obj)

	config = ensure_object(config, "top-level config")
	binary_path = config.get("binary_path", default_binary_path)
	binary_path = resolve_path(config_dir, binary_path, "binary_path")
	poll_interval_sec = config.get("poll_interval_sec", DEFAULT_POLL_INTERVAL_SEC)
	poll_interval_sec = ensure_positive_number(poll_interval_sec, "poll_interval_sec")
	defaults = ensure_object(config.get("defaults", {}), "defaults")
	displays = ensure_list(config.get("displays"), "displays")

	desired = {}
	used_udcs = set()
	used_serials = set()

	for display_index, display in enumerate(displays):
		display = ensure_object(display, f"displays[{display_index}]")
		spec = build_display(display_index, defaults, display, config_dir, binary_path)
		if spec is None:
			continue
		if spec.name in desired:
			raise ConfigError(f"display name '{spec.name}' is repeated")

		udc_device = spec.argv[spec.argv.index("--udc-device") + 1]
		serial_string = spec.argv[spec.argv.index("--serial-string") + 1]
		if udc_device in used_udcs:
			raise ConfigError(f"udc_device '{udc_device}' is assigned to more than one display")
		if serial_string in used_serials:
			raise ConfigError(f"serial_string '{serial_string}' is assigned to more than one display")

		used_udcs.add(udc_device)
		used_serials.add(serial_string)
		desired[spec.name] = spec

	return poll_interval_sec, desired


def format_command(argv):
	return " ".join(shlex.quote(arg) for arg in argv)


def start_display(spec):
	print(f"[{spec.name}] starting: {format_command(spec.argv)}", flush=True)
	process = subprocess.Popen(spec.argv)
	return ManagedDisplay(name=spec.name,
			     argv=list(spec.argv),
			     signature=spec.signature,
			     process=process)


def stop_display(managed, reason):
	if managed.process.poll() is not None:
		return

	print(f"[{managed.name}] stopping: {reason}", flush=True)
	managed.process.terminate()
	try:
		managed.process.wait(timeout=5.0)
	except subprocess.TimeoutExpired:
		print(f"[{managed.name}] did not exit after SIGTERM, sending SIGKILL", flush=True)
		managed.process.kill()
		managed.process.wait()


def reconcile(desired, managed):
	for name, running in list(managed.items()):
		exit_code = running.process.poll()
		if exit_code is not None:
			print(f"[{name}] exited with status {exit_code}", flush=True)
			del managed[name]

	for name, running in list(managed.items()):
		spec = desired.get(name)
		if spec is None:
			stop_display(running, "removed from config")
			del managed[name]
			continue
		if spec.signature != running.signature:
			stop_display(running, "config changed")
			del managed[name]

	for name in sorted(desired):
		if name in managed:
			continue
		try:
			managed[name] = start_display(desired[name])
		except OSError as exc:
			print(f"[{name}] failed to start: {exc}", file=sys.stderr, flush=True)


def stop_all(managed):
	for name in sorted(managed):
		stop_display(managed[name], "supervisor shutting down")
	for name in list(managed):
		del managed[name]


def main(argv):
	parser = argparse.ArgumentParser(
		description=(
			"Watch a JSON config file and run one displaylink_gadget_raw_gadget child "
			"per enabled display entry."))
	parser.add_argument("config_path", help="Path to the JSON config file")
	parser.add_argument("--dry-run", action="store_true",
			    help="Print the child commands implied by the config and exit")
	args = parser.parse_args(argv)

	config_path = os.path.abspath(args.config_path)
	managed = {}
	last_mtime_ns = None
	last_good_desired = {}
	last_good_poll_interval = DEFAULT_POLL_INTERVAL_SEC
	loaded_once = False

	for signo in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP, signal.SIGQUIT):
		signal.signal(signo, on_signal)

	while not STOP_REQUESTED:
		reload_needed = not loaded_once
		current_mtime_ns = last_mtime_ns

		try:
			current_mtime_ns = os.stat(config_path).st_mtime_ns
			reload_needed = reload_needed or current_mtime_ns != last_mtime_ns
		except OSError as exc:
			if not loaded_once:
				print(f"Unable to read config {config_path}: {exc}", file=sys.stderr)
				return 1
			print(f"Config temporarily unreadable, keeping last good state: {exc}",
			      file=sys.stderr,
			      flush=True)

		if reload_needed:
			try:
				poll_interval_sec, desired = load_config(config_path)
			except (ConfigError, json.JSONDecodeError, OSError) as exc:
				if not loaded_once:
					print(f"Config error in {config_path}: {exc}", file=sys.stderr)
					return 1
				print(f"Config reload failed, keeping last good state: {exc}",
				      file=sys.stderr,
				      flush=True)
			else:
				last_mtime_ns = current_mtime_ns
				last_good_desired = desired
				last_good_poll_interval = poll_interval_sec
				loaded_once = True
				if args.dry_run:
					if not desired:
						print("No displays are enabled in the config.")
					for name in sorted(desired):
						print(f"[{name}] {format_command(desired[name].argv)}")
					return 0

		if loaded_once and not args.dry_run:
			reconcile(last_good_desired, managed)

		if STOP_REQUESTED:
			break

		time.sleep(last_good_poll_interval if loaded_once else DEFAULT_POLL_INTERVAL_SEC)

	stop_all(managed)
	return 0


if __name__ == "__main__":
	sys.exit(main(sys.argv[1:]))