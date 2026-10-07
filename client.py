import argparse
import json
import select
import socket
import struct
import sys
import time


STATES = ("guarded", "not_ready", "ready", "manual", "automatic", "holding")
TELEMETRY = {
    0: ("heartbeat", "<BBq", ("state", "ready", "timestamp_ms")),
    1: (
        "attitude",
        "<6dq",
        (
            "roll_radians", "pitch_radians", "yaw_radians",
            "roll_radians_per_second", "pitch_radians_per_second",
            "yaw_radians_per_second", "timestamp_ms",
        ),
    ),
    2: (
        "global_position",
        "<6dBq",
        (
            "x_meters", "y_meters", "z_meters",
            "x_meters_per_second", "y_meters_per_second",
            "z_meters_per_second", "gnss_valid", "timestamp_ms",
        ),
    ),
    3: (
        "battery_status",
        "<3dq",
        ("voltage_volts", "current_amps", "charge_amp_hours", "timestamp_ms"),
    ),
}


def timestamp_ms():
    return time.monotonic_ns() // 1_000_000


def send_command(sock, peer, command):
    sock.sendto(struct.pack("<BqB", command, timestamp_ms(), 4), peer)


def send_target(sock, peer, elapsed):
    sock.sendto(
        struct.pack("<6dqB", elapsed, 0.0, 1.0, 0.0, 0.0, 0.0, timestamp_ms(), 5),
        peer,
    )


def decode_telemetry(data):
    if not data or data[-1] not in TELEMETRY:
        raise ValueError("missing or unknown telemetry type")
    name, layout, fields = TELEMETRY[data[-1]]
    values = struct.unpack(layout, data[:-1])
    message = dict(zip(fields, values))
    if name == "heartbeat":
        if message["state"] >= len(STATES) or message["ready"] not in (0, 1):
            raise ValueError("invalid heartbeat state or readiness flag")
        message["state"] = STATES[message["state"]]
        message["ready"] = bool(message["ready"])
    if name == "global_position":
        if message["gnss_valid"] not in (0, 1):
            raise ValueError("invalid GNSS validity flag")
        message["gnss_valid"] = bool(message["gnss_valid"])
    return {"type": name, **message}


def receive_until(sock, deadline):
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return
        readable, _, _ = select.select([sock], [], [], remaining)
        if readable:
            data, _ = sock.recvfrom(65535)
            try:
                message = decode_telemetry(data)
            except (ValueError, struct.error) as error:
                print(f"Malformed telemetry: {error}", file=sys.stderr, flush=True)
            else:
                print(json.dumps(message), flush=True)


def run(sock, peer):
    send_command(sock, peer, 0)
    receive_until(sock, time.monotonic() + 2.0)

    send_command(sock, peer, 2)
    start = time.monotonic()
    end = start + 10.0
    next_target = start
    while time.monotonic() < end:
        now = time.monotonic()
        if now >= next_target:
            send_target(sock, peer, now - start)
            next_target += 0.1
            if next_target <= now:
                next_target = now + 0.1
        receive_until(sock, min(next_target, end))

    send_command(sock, peer, 3)
    receive_until(sock, time.monotonic() + 2.0)


def main():
    parser = argparse.ArgumentParser(
        description="Send a 14-second command sequence and print all received telemetry."
    )
    parser.add_argument("--bind-address", default="127.0.0.1")
    parser.add_argument("--bind-port", type=int, default=14550)
    parser.add_argument("--peer-address", default="127.0.0.1")
    parser.add_argument("--peer-port", type=int, default=14560)
    args = parser.parse_args()
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.bind((args.bind_address, args.bind_port))
            run(sock, (args.peer_address, args.peer_port))
    except KeyboardInterrupt:
        return 130
    except OSError as error:
        print(f"client: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
