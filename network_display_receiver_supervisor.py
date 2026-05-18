#!/usr/bin/env python3

import argparse
import ipaddress
import json
import os
import shlex
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Set, Tuple


DEFAULT_POLL_INTERVAL_SEC = 1.0
STOP_REQUESTED = False


class ConfigError(Exception):
    pass


@dataclass
class DesiredDisplay:
    name: str
    argv: List[str]
    bind_cidr: str
    environment: Dict[str, str]
    working_directory: Optional[str]
    signature: str


@dataclass
class ManagedDisplay:
    name: str
    signature: str
    process: subprocess.Popen


def on_signal(signo, frame):
    del signo, frame
    global STOP_REQUESTED
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


def ensure_positive_number(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or value <= 0:
        raise ConfigError(f"{label} must be a positive number")
    return float(value)


def ensure_prefix_length(value, label):
    if isinstance(value, bool) or not isinstance(value, int) or value < 1 or value > 32:
        raise ConfigError(f"{label} must be an integer between 1 and 32")
    return value


def ensure_ipv4_address(value, label):
    text = ensure_string(value, label)
    try:
        ipaddress.IPv4Address(text)
    except ipaddress.AddressValueError as exc:
        raise ConfigError(f"{label} must be a valid IPv4 address") from exc
    return text


def resolve_path(base_dir, path_value, label):
    path = ensure_string(path_value, label)
    if os.path.isabs(path):
        return path
    return os.path.normpath(os.path.join(base_dir, path))


def render_template(template, context, label):
    text = ensure_string(template, label)
    try:
        return text.format_map(context)
    except KeyError as exc:
        raise ConfigError(f"{label} references unknown placeholder {exc}") from exc
    except ValueError as exc:
        raise ConfigError(f"{label} contains an invalid format string") from exc


def render_string_list(values, context, label):
    rendered = []
    for index, value in enumerate(ensure_list(values, label)):
        rendered.append(render_template(value, context, f"{label}[{index}]") )
    return rendered


def render_environment(default_env, display_env, context, base_label):
    merged = {}
    for source_label, source in ((f"{base_label}.defaults", default_env),
                                 (f"{base_label}.display", display_env)):
        for key, value in ensure_object(source, source_label).items():
            if not isinstance(key, str) or not key:
                raise ConfigError(f"{source_label} keys must be non-empty strings")
            merged[key] = render_template(value, context, f"{source_label}.{key}")
    return merged


def resolve_command_argv(argv, config_dir):
    resolved = list(argv)
    if resolved and "/" in resolved[0] and not os.path.isabs(resolved[0]):
        resolved[0] = os.path.normpath(os.path.join(config_dir, resolved[0]))
    return resolved


def build_display(display_index, interface, defaults, config_defaults, display, config_dir):
    merged = dict(defaults)
    merged.update(display)

    enabled = ensure_bool(merged.get("enabled", True), f"displays[{display_index}].enabled")
    if not enabled:
        return None

    name = ensure_string(merged.get("name", f"SBC Monitor {display_index + 1}"),
                         f"displays[{display_index}].name")
    bind_ip = ensure_ipv4_address(merged.get("bind_ip"), f"displays[{display_index}].bind_ip")
    prefix_length = ensure_prefix_length(merged.get("prefix_length", 24),
                                         f"displays[{display_index}].prefix_length")
    bind_cidr = f"{bind_ip}/{prefix_length}"

    context = {
        "index": str(display_index + 1),
        "interface": interface,
        "name": name,
        "bind_ip": bind_ip,
        "bind_cidr": bind_cidr,
    }

    receiver_command = merged.get("receiver_command", config_defaults.get("receiver_command"))
    if receiver_command is None:
        raise ConfigError(
            f"displays[{display_index}] must define receiver_command or rely on top-level receiver_command")
    argv = render_string_list(receiver_command, context, f"displays[{display_index}].receiver_command")
    argv = resolve_command_argv(argv, config_dir)

    extra_args = render_string_list(merged.get("extra_args", []),
                                    context,
                                    f"displays[{display_index}].extra_args")
    argv.extend(extra_args)

    working_directory = merged.get("working_directory")
    if working_directory is not None:
        working_directory = render_template(working_directory,
                                            context,
                                            f"displays[{display_index}].working_directory")
        if not os.path.isabs(working_directory):
            working_directory = os.path.normpath(os.path.join(config_dir, working_directory))

    environment = render_environment(defaults.get("environment", {}),
                                     display.get("environment", {}),
                                     context,
                                     f"displays[{display_index}].environment")

    signature = json.dumps({
        "argv": argv,
        "bind_cidr": bind_cidr,
        "environment": environment,
        "working_directory": working_directory,
    }, sort_keys=True, separators=(",", ":"))

    return DesiredDisplay(name=name,
                          argv=argv,
                          bind_cidr=bind_cidr,
                          environment=environment,
                          working_directory=working_directory,
                          signature=signature)


def load_config(config_path):
    config_dir = os.path.dirname(config_path)

    with open(config_path, "r", encoding="utf-8") as file_obj:
        config = json.load(file_obj)

    config = ensure_object(config, "top-level config")
    interface = ensure_string(config.get("interface"), "interface")
    poll_interval_sec = ensure_positive_number(
        config.get("poll_interval_sec", DEFAULT_POLL_INTERVAL_SEC),
        "poll_interval_sec")
    defaults = ensure_object(config.get("defaults", {}), "defaults")
    displays = ensure_list(config.get("displays"), "displays")

    desired = {}
    used_bind_cidrs = set()

    for display_index, display in enumerate(displays):
        display = ensure_object(display, f"displays[{display_index}]")
        spec = build_display(display_index,
                             interface,
                             defaults,
                             config,
                             display,
                             config_dir)
        if spec is None:
            continue
        if spec.name in desired:
            raise ConfigError(f"display name '{spec.name}' is repeated")
        if spec.bind_cidr in used_bind_cidrs:
            raise ConfigError(f"bind_ip '{spec.bind_cidr}' is assigned to more than one display")
        used_bind_cidrs.add(spec.bind_cidr)
        desired[spec.name] = spec

    return interface, poll_interval_sec, desired


def format_command(argv):
    return " ".join(shlex.quote(arg) for arg in argv)


def start_display(spec):
    print(f"[{spec.name}] starting: {format_command(spec.argv)}", flush=True)
    env = os.environ.copy()
    env.update(spec.environment)
    process = subprocess.Popen(spec.argv, cwd=spec.working_directory, env=env)
    return ManagedDisplay(name=spec.name, signature=spec.signature, process=process)


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


def run_ip_command(args, *, check):
    return subprocess.run(["ip", *args],
                          check=check,
                          text=True,
                          stdout=subprocess.DEVNULL,
                          stderr=subprocess.PIPE)


def format_process_error(exc):
    if isinstance(exc, subprocess.CalledProcessError):
        stderr = (exc.stderr or "").strip()
        if stderr:
            return stderr
    return str(exc)


def cleanup_interface_addresses(interface, applied_addresses):
    for cidr in sorted(applied_addresses):
        try:
            run_ip_command(["address", "del", cidr, "dev", interface], check=True)
        except (OSError, subprocess.CalledProcessError) as exc:
            print(f"Failed to remove {cidr} from {interface}: {format_process_error(exc)}",
                  file=sys.stderr,
                  flush=True)


def sync_interface_addresses(interface, desired, applied_addresses):
    desired_cidrs = {spec.bind_cidr for spec in desired.values()}

    if not desired_cidrs:
        cleanup_interface_addresses(interface, applied_addresses)
        return True, set()

    try:
        run_ip_command(["link", "show", "dev", interface], check=True)
        run_ip_command(["link", "set", "dev", interface, "up"], check=True)
        for cidr in sorted(applied_addresses - desired_cidrs):
            run_ip_command(["address", "del", cidr, "dev", interface], check=True)
        for cidr in sorted(desired_cidrs):
            run_ip_command(["address", "replace", cidr, "dev", interface], check=True)
    except (OSError, subprocess.CalledProcessError) as exc:
        print(f"Unable to prepare {interface}: {format_process_error(exc)}",
              file=sys.stderr,
              flush=True)
        return False, applied_addresses

    return True, desired_cidrs


def main(argv):
    parser = argparse.ArgumentParser(
        description=(
            "Watch a JSON config file, assign per-display IP aliases on one USB network interface, "
            "and run one receiver daemon instance per enabled display entry."))
    parser.add_argument("config_path", help="Path to the JSON config file")
    parser.add_argument("--dry-run", action="store_true",
                        help="Print the IP alias commands and receiver commands implied by the config")
    args = parser.parse_args(argv)

    config_path = os.path.abspath(args.config_path)
    managed = {}
    applied_addresses = set()
    last_good_interface = None
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
                interface, poll_interval_sec, desired = load_config(config_path)
            except (ConfigError, json.JSONDecodeError, OSError) as exc:
                if not loaded_once:
                    print(f"Config error in {config_path}: {exc}", file=sys.stderr)
                    return 1
                print(f"Config reload failed, keeping last good state: {exc}",
                      file=sys.stderr,
                      flush=True)
            else:
                if loaded_once and last_good_interface and interface != last_good_interface:
                    stop_all(managed)
                    cleanup_interface_addresses(last_good_interface, applied_addresses)
                    applied_addresses = set()

                last_mtime_ns = current_mtime_ns
                last_good_interface = interface
                last_good_desired = desired
                last_good_poll_interval = poll_interval_sec
                loaded_once = True

                if args.dry_run:
                    if not desired:
                        print("No displays are enabled in the config.")
                        return 0
                    print(f"Interface: {interface}")
                    for name in sorted(desired):
                        print(f"[{name}] ip address replace {desired[name].bind_cidr} dev {interface}")
                        print(f"[{name}] {format_command(desired[name].argv)}")
                    return 0

        if loaded_once and not args.dry_run:
            ready, applied_addresses = sync_interface_addresses(last_good_interface,
                                                                last_good_desired,
                                                                applied_addresses)
            if ready:
                reconcile(last_good_desired, managed)
            else:
                stop_all(managed)

        if STOP_REQUESTED:
            break

        time.sleep(last_good_poll_interval if loaded_once else DEFAULT_POLL_INTERVAL_SEC)

    stop_all(managed)
    if last_good_interface:
        cleanup_interface_addresses(last_good_interface, applied_addresses)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))