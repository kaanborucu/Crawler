#!/usr/bin/env python3
"""Read a DualSense on Windows and send safe test commands to Crawler-S3.

Install dependencies:
    py -m pip install pygame bleak

Mapping:
    L1       emergency hold / freeze current pose
    Triangle raw-position mode
    Square   policy mode
    Circle   scripted sweep mode

Policy mode uses the left stick: vertical = forward, horizontal = lateral.
Raw mode maps left Y, left X, and right Y to joints 0, 1, and 2. Raw commands
are intentionally limited to +/-0.5 rad by default.
"""

from __future__ import annotations

import argparse
import asyncio
import sys
from typing import Any

import struct


SERVICE_UUID = "7f1f0001-9f2e-4c9c-9d53-4e2a4d4b0101"
COMMAND_UUID = "7f1f0002-9f2e-4c9c-9d53-4e2a4d4b0101"
STATUS_UUID = "7f1f0003-9f2e-4c9c-9d53-4e2a4d4b0101"

MODE_POLICY = 0
MODE_RAW = 1
MODE_SWEEP = 2

FLAG_ENABLE = 0x01
FLAG_ESTOP = 0x02
FLAG_CLEAR_FAULT = 0x04
FLAG_RAW = 0x08
FLAG_SWEEP = 0x10

AXIS_LEFT_X = 0
AXIS_LEFT_Y = 1
AXIS_RIGHT_Y = 3

BUTTON_CIRCLE = 1
BUTTON_SQUARE = 2
BUTTON_TRIANGLE = 3
# pygame reports L1 as button 9 for this DualSense/Windows mapping.
BUTTON_L1 = 9
BUTTON_CROSS = 0


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def deadzone(value: float, threshold: float = 0.08) -> float:
    if abs(value) <= threshold:
        return 0.0
    sign = 1.0 if value >= 0.0 else -1.0
    return sign * (abs(value) - threshold) / (1.0 - threshold)


def build_packet(
    mode: int,
    forward_mm_per_second: int,
    lateral_mm_per_second: int,
    raw_position_milliradians: tuple[int, int, int],
    sequence: int,
    enable: bool,
    emergency_stop: bool,
    clear_fault: bool,
) -> bytes:
    flags = 0
    if enable:
        flags |= FLAG_ENABLE
    if emergency_stop:
        flags |= FLAG_ESTOP
    if clear_fault:
        flags |= FLAG_CLEAR_FAULT
    if mode == MODE_RAW:
        flags |= FLAG_RAW
    elif mode == MODE_SWEEP:
        flags |= FLAG_SWEEP

    return struct.pack(
        "<BBhhhhhH",
        2,
        flags,
        int(clamp(forward_mm_per_second, -1500, 1500)),
        int(clamp(lateral_mm_per_second, -1500, 1500)),
        *(
            int(clamp(value, -1571, 1571))
            for value in raw_position_milliradians
        ),
        sequence & 0xFFFF,
    )


def status_callback(_: Any, data: bytearray) -> None:
    if len(data) != struct.calcsize("<BBBBHII"):
        print(f"status: unexpected {len(data)} bytes: {data.hex()}")
        return
    version, state, fault, connected, sequence, inference_us, missed = struct.unpack(
        "<BBBBHII", data
    )
    print(
        "status: "
        f"state={state} fault={fault} connected={bool(connected)} "
        f"sequence={sequence} inference={inference_us}us missed={missed}"
    )


def read_joystick(joystick: Any, mode: int, raw_range_rad: float) -> tuple[int, int, tuple[int, int, int]]:
    import pygame

    pygame.event.pump()
    left_x = deadzone(joystick.get_axis(AXIS_LEFT_X))
    left_y = deadzone(joystick.get_axis(AXIS_LEFT_Y))
    right_y = deadzone(joystick.get_axis(AXIS_RIGHT_Y))

    forward = int(clamp(-left_y * 1500.0, -1500.0, 1500.0))
    lateral = int(clamp(left_x * 1500.0, -1500.0, 1500.0))
    raw = (
        int(clamp(-left_y * raw_range_rad * 1000.0, -1571.0, 1571.0)),
        int(clamp(left_x * raw_range_rad * 1000.0, -1571.0, 1571.0)),
        int(clamp(-right_y * raw_range_rad * 1000.0, -1571.0, 1571.0)),
    )
    return forward, lateral, raw


async def find_crawler(name: str) -> Any:
    from bleak import BleakScanner

    discovered = await BleakScanner.discover(timeout=5.0, return_adv=True)
    if isinstance(discovered, dict):
        entries = list(discovered.values())
        devices = [device for device, _ in entries]
        matches = [
            device
            for device, advertisement in entries
            if (device.name or "") == name
            or any(
                uuid.lower() == SERVICE_UUID.lower()
                for uuid in (advertisement.service_uuids or [])
            )
        ]
    else:
        devices = discovered
        matches = [device for device in devices if (device.name or "") == name]
    if len(matches) != 1:
        print(
            f"Expected exactly one crawler BLE device named {name!r} or "
            f"advertising service {SERVICE_UUID}; "
            f"found {len(matches)}.",
            file=sys.stderr,
        )
        for device in devices:
            print(f"  {device.address}\t{device.name or '(unnamed)'}")
        raise RuntimeError("Crawler-S3 not found")
    return matches[0]


async def run(args: argparse.Namespace) -> int:
    try:
        from bleak import BleakClient
    except ImportError:
        print("Install dependencies with: py -m pip install pygame bleak", file=sys.stderr)
        return 2

    target = args.address if args.address else await find_crawler(args.name)
    if isinstance(target, str):
        print(f"Connecting directly to {target}")
    else:
        print(f"Connecting to {target.address} ({target.name})")

    sequence = 1
    selected_mode: int | None = None
    pygame = None

    try:
        async with BleakClient(target) as client:
            # Import pygame only after Bleak has scanned and connected. Its
            # Windows initialization changes the thread to STA, while Bleak
            # requires MTA during scanner/client setup.
            import pygame as pygame_module

            pygame = pygame_module
            pygame.init()
            pygame.joystick.init()
            if pygame.joystick.get_count() == 0:
                print("No joystick found. Pair/connect the DualSense to Windows first.", file=sys.stderr)
                return 1

            joystick = pygame.joystick.Joystick(0)
            joystick.init()
            print(f"Controller: {joystick.get_name()} axes={joystick.get_numaxes()} buttons={joystick.get_numbuttons()}")
            print("Connected. Press Square, Triangle, or Circle to select a mode.")
            try:
                await client.start_notify(STATUS_UUID, status_callback)
            except Exception as exc:
                print(f"Status notifications unavailable: {exc}")

            while True:
                pygame.event.pump()
                if joystick.get_button(BUTTON_SQUARE):
                    selected_mode = MODE_POLICY
                elif joystick.get_button(BUTTON_TRIANGLE):
                    selected_mode = MODE_RAW
                elif joystick.get_button(BUTTON_CIRCLE):
                    selected_mode = MODE_SWEEP

                emergency = bool(joystick.get_button(BUTTON_L1))
                clear_fault = bool(joystick.get_button(BUTTON_CROSS)) and not emergency
                mode = MODE_POLICY if selected_mode is None else selected_mode
                forward, lateral, raw = read_joystick(
                    joystick, mode, args.raw_range_rad
                )
                if emergency:
                    enable = False
                    forward = 0
                    lateral = 0
                else:
                    enable = selected_mode is not None

                packet = build_packet(
                    mode,
                    forward,
                    lateral,
                    raw,
                    sequence,
                    enable,
                    emergency,
                    clear_fault,
                )
                await client.write_gatt_char(COMMAND_UUID, packet, response=False)
                sequence = (sequence + 1) & 0xFFFF
                await asyncio.sleep(args.period)
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        print(f"Bridge stopped: {exc}", file=sys.stderr)
        return 1
    finally:
        if pygame is not None:
            pygame.quit()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name", default="Crawler-S3", help="crawler BLE name")
    parser.add_argument(
        "--address",
        help="connect directly to a known Windows BLE address, skipping discovery",
    )
    parser.add_argument(
        "--period", type=float, default=0.02, help="command period in seconds"
    )
    parser.add_argument(
        "--raw-range-rad",
        type=float,
        default=0.5,
        help="raw-stick position range in radians (default: 0.5)",
    )
    return parser.parse_args()


if __name__ == "__main__":
    raise SystemExit(asyncio.run(run(parse_args())))
