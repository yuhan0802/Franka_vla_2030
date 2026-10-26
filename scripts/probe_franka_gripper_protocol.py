#!/usr/bin/env python3
"""Read raw Franka gripper protocol-v3 state without issuing motion commands.

Uses only the standard library. Sends one Connect request, then receives UDP
feedback; never sends Move, Grasp, Stop, or Homing. Run while policy tasks and
other gripper clients are stopped, e.g.:
    python3 scripts/probe_franka_gripper_protocol.py 172.16.0.2
"""

import argparse
import datetime
import json
import socket
import struct


HEADER = struct.Struct("<HII")
STATE = struct.Struct("<Idd?H")


def recv_exact(stream, size):
    result = bytearray()
    while len(result) < size:
        chunk = stream.recv(size - len(result))
        if not chunk:
            raise RuntimeError("gripper closed TCP connection during handshake")
        result.extend(chunk)
    return bytes(result)


def probe(host, samples, timeout):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
        udp.bind(("0.0.0.0", 0))
        udp.settimeout(timeout)
        with socket.create_connection((host, 1338), timeout=timeout) as tcp:
            # Command 0 = Connect; payload = protocol version and UDP port.
            tcp.sendall(HEADER.pack(0, 0, 14) + struct.pack("<HH", 3, udp.getsockname()[1]))
            command, command_id, size = HEADER.unpack(recv_exact(tcp, HEADER.size))
            if (command, command_id, size) != (0, 0, 14):
                raise RuntimeError(f"unexpected Connect response header: {(command, command_id, size)}")
            status, version = struct.unpack("<HH", recv_exact(tcp, 4))
            if status != 0 or version != 3:
                raise RuntimeError(f"Connect failed: status={status}, protocol_version={version}")
            peer_ip = tcp.getpeername()[0]
            print(json.dumps({"event": "connected", "robot_ip": peer_ip,
                              "protocol_version": version, "motion_commands_sent": False}), flush=True)
            for _ in range(samples):
                packet, source = udp.recvfrom(65535)
                if source[0] != peer_ip or len(packet) != STATE.size:
                    raise RuntimeError(f"unexpected UDP source/size: {source}, {len(packet)} bytes")
                message_id, width, max_width, grasped, temperature = STATE.unpack(packet)
                issues = []
                if max_width <= 0:
                    issues.append("nonpositive_homed_max_width")
                if width < 0:
                    issues.append("negative_reported_width")
                if width > max_width + 0.001:
                    issues.append("reported_width_exceeds_homed_max")
                print(json.dumps({"event": "raw_gripper_state",
                                  "utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
                                  "source_ip": source[0], "source_port": source[1],
                                  "packet_bytes": len(packet), "packet_hex": packet.hex(),
                                  "message_id": message_id, "width_m": width,
                                  "max_width_m": max_width, "is_grasped": grasped,
                                  "temperature_c": temperature, "diagnostic_issues": issues}), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("robot_ip", nargs="?", default="172.16.0.2")
    parser.add_argument("--samples", type=int, default=5)
    parser.add_argument("--timeout", type=float, default=3)
    args = parser.parse_args()
    if not 1 <= args.samples <= 10000 or not 0 < args.timeout <= 60:
        parser.error("samples must be in [1, 10000], timeout in (0, 60]")
    probe(args.robot_ip, args.samples, args.timeout)


if __name__ == "__main__":
    main()
