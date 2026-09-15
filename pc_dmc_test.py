#!/usr/bin/env python3
"""Minimal DMC test client for DragonFrame-style USB traffic.

Usage examples:
    python pc_dmc_test.py COM21 --sequence hi
    python pc_dmc_test.py COM21 --type 0x0001 --id 1
    python pc_dmc_test.py COM21 --sequence hi,status,config,move,stop
"""

import argparse
import serial
import time
from typing import List, Tuple


DMC_MSG_HI = 0x0001
DMC_MSG_FLAG_ACK = 0x8000

DEFAULT_SEQUENCE = "hi,status,config,move,stop,jog,speed"

SEQUENCE_ALIASES = {
    "hi": (DMC_MSG_HI, b""),
    "status": (0x0030, b""),
    "config": (0x0037, bytes([1, 1])),
    "move": (0x0031, bytes([1]) + (1000).to_bytes(4, byteorder="little", signed=True)),
    "stop": (0x0032, bytes([1])),
    "reset": (0x0035, bytes([1]) + (0).to_bytes(4, byteorder="little", signed=True)),
    "jog": (
        0x0036,
        bytes([1])
        + (10000).to_bytes(2, byteorder="little", signed=False)
        + (250).to_bytes(4, byteorder="little", signed=True),
    ),
    "speed": (
        0x0038,
        bytes([1])
        + (1000).to_bytes(4, byteorder="little", signed=True)
        + (500).to_bytes(4, byteorder="little", signed=True),
    ),
    "limits": (
        0x0039,
        bytes([1, 1])
        + (0).to_bytes(4, byteorder="little", signed=True)
        + bytes([1])
        + (2000).to_bytes(4, byteorder="little", signed=True)
        + bytes([0]),
    ),
    "position": (0x0034, b""),
    "gio": (0x0021, (0x00000005).to_bytes(4, byteorder="little", signed=False)),
    "gio_out": (0x0021, (0x00000005).to_bytes(4, byteorder="little", signed=False)),
    "gio_in": (0x0022, b""),
    "gio_cam": (0x0023, (0x00000001).to_bytes(4, byteorder="little", signed=False)),
    "dmx": (
        0x0020,
        (1).to_bytes(4, byteorder="little", signed=False)
        + (3).to_bytes(2, byteorder="little", signed=False)
        + bytes([0, 0xFF, 0x80, 0x00]),
    ),
}


def compute_checksum(data: bytes) -> int:
    """Compute the DMC checksum over the given bytes.

    This matches the bridge implementation in src/main.cpp.
    """
    sum1 = 0
    sum2 = 0
    remaining = len(data)
    offset = 0

    while remaining > 0:
        chunk = 20 if remaining > 20 else remaining
        remaining -= chunk
        for i in range(chunk):
            b = data[offset + i]
            sum2 += sum1 + b
            sum1 += b
        offset += chunk
        sum1 %= 0xFF
        sum2 %= 0xFF

    return (sum2 << 8) | sum1


def encode_checksum(raw_checksum: int) -> int:
    low = raw_checksum & 0xFF
    high = (raw_checksum >> 8) & 0xFF
    c0 = 0xFF - ((low + high) % 0xFF)
    c1 = 0xFF - ((low + c0) % 0xFF)
    return (c1 << 8) | c0


def build_dmc_frame(msg_type: int, msg_id: int, payload: bytes = b"") -> bytes:
    header = b"DF"
    msg_id_bytes = msg_id.to_bytes(4, byteorder="little", signed=False)
    type_bytes = msg_type.to_bytes(2, byteorder="little", signed=False)
    length_bytes = len(payload).to_bytes(2, byteorder="little", signed=False)
    frame_without_checksum = header + msg_id_bytes + type_bytes + length_bytes + payload
    checksum = encode_checksum(compute_checksum(frame_without_checksum))
    checksum_bytes = checksum.to_bytes(2, byteorder="little", signed=False)
    return frame_without_checksum + checksum_bytes


def parse_frames(data: bytes) -> Tuple[List[dict], bytes]:
    frames = []
    i = 0
    while i + 12 <= len(data):
        marker = data.find(b"DF", i)
        if marker < 0:
            return frames, data[i:]
        i = marker
        if i + 10 > len(data):
            return frames, data[i:]
        msg_id = int.from_bytes(data[i + 2 : i + 6], "little")
        msg_type = int.from_bytes(data[i + 6 : i + 8], "little")
        length = int.from_bytes(data[i + 8 : i + 10], "little")
        total = 10 + length + 2
        if i + total > len(data):
            return frames, data[i:]
        payload = data[i + 10 : i + 10 + length]
        frames.append(
            {
                "id": msg_id,
                "type": msg_type,
                "length": length,
                "payload": payload,
            }
        )
        i += total
    return frames, data[i:]


def describe_frame(frame: dict) -> str:
    msg_type = frame["type"]
    ack = bool(msg_type & DMC_MSG_FLAG_ACK)
    base_type = msg_type & ~DMC_MSG_FLAG_ACK
    payload = frame["payload"]
    parts = [
        f"id={frame['id']}",
        f"type=0x{msg_type:04X}",
        f"len={frame['length']}",
    ]
    if ack and len(payload) >= 4:
        status = int.from_bytes(payload[:4], "little")
        parts.append(f"ack_status=0x{status:04X}")
    elif base_type == DMC_MSG_HI and len(payload) >= 36:
        name = payload[:32].split(b"\x00", 1)[0].decode("utf-8", "replace")
        major, minor, rev = payload[32], payload[33], payload[34]
        motors = payload[35]
        extra = f'name="{name}" version={major}.{minor}.{rev} motors={motors}'
        if len(payload) >= 51:
            dmx = int.from_bytes(payload[36:38], "little")
            gio_out, gio_in, hw = payload[38], payload[39], payload[40]
            frames = int.from_bytes(payload[41:45], "little")
            caps = int.from_bytes(payload[45:49], "little")
            proto = int.from_bytes(payload[49:51], "little")
            extra += (
                f" dmx={dmx} gio_out={gio_out} gio_in={gio_in} hw_limits={hw}"
                f" frames={frames} caps=0x{caps:08X} proto={proto}"
            )
        parts.append(extra)
    elif base_type == 0x0030 and len(payload) >= 4:
        status = int.from_bytes(payload[:4], "little")
        extra = f"moving_mask=0x{status:08X}"
        if len(payload) >= 5:
            extra += f" dmx_busy={payload[4]}"
        parts.append(extra)
    elif base_type == 0x0034 and len(payload) >= 8:
        move_time = int.from_bytes(payload[:4], "little")
        positions = [
            int.from_bytes(payload[i : i + 4], "little", signed=True)
            for i in range(4, len(payload), 4)
        ]
        parts.append(f"move_time={move_time} pos={positions}")
    elif base_type == 0x0022 and len(payload) >= 4:
        bits = int.from_bytes(payload[:4], "little")
        parts.append(f"gio_in=0x{bits:08X}")
    return " ".join(parts)


def sequence_from_name(name: str) -> Tuple[int, bytes]:
    key = name.strip().lower()
    if key not in SEQUENCE_ALIASES:
        raise ValueError(
            f"Unknown packet name '{name}'. Available: {', '.join(sorted(SEQUENCE_ALIASES))}"
        )
    return SEQUENCE_ALIASES[key]


def parse_sequence(sequence: str) -> List[Tuple[int, bytes]]:
    if not sequence:
        return [sequence_from_name(token) for token in DEFAULT_SEQUENCE.split(",")]

    packets = []
    for token in sequence.split(","):
        token = token.strip()
        if not token:
            continue
        if token.startswith("0x") or token.startswith("0X"):
            packets.append((int(token, 16), b""))
        else:
            packets.append(sequence_from_name(token))
    return packets


def read_available(port: serial.Serial, timeout_s: float = 2.0) -> bytes:
    port.timeout = 0.05
    data = bytearray()
    deadline = time.time() + timeout_s
    last_rx = time.time()
    while time.time() < deadline:
        try:
            chunk = port.read(port.in_waiting or 1)
        except Exception:
            break
        if chunk:
            data.extend(chunk)
            last_rx = time.time()
            continue
        if data and (time.time() - last_rx) > 0.08:
            break
        time.sleep(0.01)
    return bytes(data)


def dump_hex(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


def print_rx(label: str, rx: bytes) -> None:
    if not rx:
        print(f"{label}: no response")
        return
    print(f"{label} hex: {dump_hex(rx)}")
    frames, leftover = parse_frames(rx)
    if not frames:
        print(f"{label}: could not parse DMC frames")
        return
    for idx, frame in enumerate(frames, start=1):
        print(f"{label} frame {idx}: {describe_frame(frame)}")
    if leftover:
        print(f"{label} leftover: {dump_hex(leftover)}")


def run_session(
    port_name: str,
    baud: int,
    packets: List[Tuple[int, bytes]],
    start_id: int,
    timeout_s: float,
    delay_s: float,
) -> None:
    ser = serial.Serial(port_name, baud, timeout=1)
    try:
        time.sleep(0.4)
        boot_rx = read_available(ser, timeout_s=0.6)
        if boot_rx:
            print_rx("BOOT", boot_rx)

        for idx, (msg_type, payload) in enumerate(packets, start=1):
            msg_id = start_id + idx - 1
            frame = build_dmc_frame(msg_type, msg_id, payload)
            print(f"\nPacket {idx}/{len(packets)}")
            print(f"TX: type=0x{msg_type:04X} id={msg_id} {dump_hex(frame)}")
            ser.write(frame)
            ser.flush()
            rx = read_available(ser, timeout_s=timeout_s)
            print_rx("RX", rx)
            if idx != len(packets):
                time.sleep(delay_s)
    finally:
        ser.close()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Send one or more DragonFrame-style DMC packets to a USB serial port."
    )
    parser.add_argument("port", help="COM port, e.g. COM21")
    parser.add_argument(
        "--type",
        type=lambda x: int(x, 0),
        default=None,
        help="Single DMC msg type, e.g. 0x0001 or 0x0030",
    )
    parser.add_argument("--id", type=int, default=1, help="Starting message ID")
    parser.add_argument("--payload", default="", help="Hex payload, e.g. 01e8030000")
    parser.add_argument(
        "--sequence",
        default=DEFAULT_SEQUENCE,
        help="Comma-separated packet names: hi,status,config,move,stop,reset,jog,speed,limits,position,gio,gio_in,gio_cam,dmx",
    )
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--read-timeout", type=float, default=2.0)
    parser.add_argument("--delay", type=float, default=0.25, help="Delay between packets in a sequence")
    args = parser.parse_args()

    if args.type is not None:
        payload = bytes.fromhex(args.payload) if args.payload else b""
        packets = [(args.type, payload)]
    else:
        packets = parse_sequence(args.sequence)

    run_session(args.port, args.baud, packets, args.id, args.read_timeout, args.delay)


if __name__ == "__main__":
    main()
