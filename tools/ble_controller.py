#!/usr/bin/env python3
"""Small Windows BLE controller for the Crawler command service.

Install the only host-side dependency with:
    python -m pip install bleak

Examples:
    python tools/ble_controller.py --scan
    python tools/ble_controller.py --name Crawler-S3 --enable --count 20
    python tools/ble_controller.py --address XX:XX:XX:XX:XX:XX --stop
"""

from __future__ import annotations

import argparse
import asyncio
import struct
import sys
from typing import Any


SERVICE_UUID = "7f1f0001-9f2e-4c9c-9d53-4e2a4d4b0101"
COMMAND_UUID = "7f1f0002-9f2e-4c9c-9d53-4e2a4d4b0101"
STATUS_UUID = "7f1f0003-9f2e-4c9c-9d53-4e2a4d4b0101"
PROTOCOL_VERSION = 1


def build_packet(
    forward_mm_per_second: int,
    lateral_mm_per_second: int,
    sequence: int,
    enable: bool,
    emergency_stop: bool,
    clear_fault: bool,
) -> bytes:
    if not -1500 <= forward_mm_per_second <= 1500:
        raise ValueError("forward velocity must be between -1500 and +1500 mm/s")
    if not -1500 <= lateral_mm_per_second <= 1500:
        raise ValueError("lateral velocity must be between -1500 and +1500 mm/s")
    flags = (1 if enable else 0) | (2 if emergency_stop else 0) | (
        4 if clear_fault else 0
    )
    return struct.pack(
        "<BBhhH",
        PROTOCOL_VERSION,
        flags,
        forward_mm_per_second,
        lateral_mm_per_second,
        sequence & 0xFFFF,
    )


def print_status(_: Any, data: bytearray) -> None:
    if len(data) != struct.calcsize("<BBBBHII"):
        print(f"status: unexpected {len(data)} bytes: {data.hex()}")
        return
    version, state, fault, connected, sequence, inference_us, missed = struct.unpack(
        "<BBBBHII", data
    )
    print(
        "status: "
        f"version={version} state={state} fault={fault} "
        f"connected={bool(connected)} sequence={sequence} "
        f"inference={inference_us}us missed={missed}"
    )


async def scan_devices() -> int:
    try:
        from bleak import BleakScanner
    except ImportError:
        print("Install bleak first: python -m pip install bleak", file=sys.stderr)
        return 2

    devices = await BleakScanner.discover(timeout=5.0)
    for device in devices:
        print(f"{device.address}\t{device.name or '(unnamed)'}")
    return 0


async def send_commands(args: argparse.Namespace) -> int:
    try:
        from bleak import BleakClient, BleakScanner
    except ImportError:
        print("Install bleak first: python -m pip install bleak", file=sys.stderr)
        return 2

    target: Any = args.address
    if target is None:
        devices = await BleakScanner.discover(timeout=5.0)
        matches = [d for d in devices if (d.name or "") == args.name]
        if len(matches) != 1:
            print(
                f"Expected exactly one BLE device named {args.name!r}; "
                f"found {len(matches)}.",
                file=sys.stderr,
            )
            return 1
        target = matches[0]

    try:
        async with BleakClient(target) as client:
            print(f"Connected to {target}")
            sequence = args.sequence
            try:
                try:
                    await client.start_notify(STATUS_UUID, print_status)
                except Exception as exc:  # status is helpful but not required to send
                    print(f"Warning: status notifications unavailable: {exc}")

                count = args.count
                sent = 0
                while count == 0 or sent < count:
                    packet = build_packet(
                        args.forward,
                        args.lateral,
                        sequence,
                        args.enable and not args.stop,
                        args.stop,
                        args.clear_fault,
                    )
                    await client.write_gatt_char(COMMAND_UUID, packet, response=True)
                    print(
                        f"sent sequence={sequence & 0xFFFF} "
                        f"forward={args.forward} lateral={args.lateral} "
                        f"flags=0x{packet[1]:02x}"
                    )
                    sequence = (sequence + 1) & 0xFFFF
                    sent += 1
                    if count == 1:
                        await asyncio.sleep(0.25)
                        break
                    await asyncio.sleep(args.period)
            finally:
                # Always remove enable and command velocity before disconnecting.
                zero_packet = build_packet(0, 0, sequence, False, False, False)
                try:
                    await client.write_gatt_char(
                        COMMAND_UUID, zero_packet, response=True
                    )
                    print(f"sent zero/dead-man sequence={sequence & 0xFFFF}")
                except Exception as exc:
                    print(f"Warning: could not send zero command on exit: {exc}")
    except Exception as exc:
        print(f"BLE connection failed: {exc}", file=sys.stderr)
        return 1
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scan", action="store_true", help="list nearby BLE devices")
    parser.add_argument("--address", help="BLE address from --scan")
    parser.add_argument("--name", default="Crawler-S3", help="device name to find")
    parser.add_argument("--forward", type=int, default=0, help="forward velocity in mm/s")
    parser.add_argument("--lateral", type=int, default=0, help="lateral velocity in mm/s")
    parser.add_argument("--sequence", type=int, default=1)
    parser.add_argument("--enable", action="store_true", help="request arming")
    parser.add_argument("--stop", action="store_true", help="send emergency stop")
    parser.add_argument("--clear-fault", action="store_true")
    parser.add_argument(
        "--count",
        type=int,
        default=1,
        help="number of packets; 0 sends continuously (default: 1)",
    )
    parser.add_argument("--period", type=float, default=0.02, help="seconds between packets")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.scan:
        return asyncio.run(scan_devices())
    if args.count < 0 or args.period < 0:
        print("count and period must be non-negative", file=sys.stderr)
        return 2
    return asyncio.run(send_commands(args))


if __name__ == "__main__":
    raise SystemExit(main())
