#!/usr/bin/env python3
"""Windows BLE bridge for the Crawler command service.

Install the host-side dependencies with:
    python -m pip install bleak pygame

Examples:
    python tools/ble_controller.py --scan
    python tools/ble_controller.py --name Crawler-S3 --enable --count 20
    python tools/ble_controller.py --address XX:XX:XX:XX:XX:XX --stop
    python tools/ble_controller.py --list-gamepads
    python tools/ble_controller.py --gamepad-test
    python tools/ble_controller.py --gamepad --enable-button 0
"""

from __future__ import annotations

import argparse
import asyncio
import struct
import sys
import time
from typing import Any


SERVICE_UUID = "7f1f0001-9f2e-4c9c-9d53-4e2a4d4b0101"
COMMAND_UUID = "7f1f0002-9f2e-4c9c-9d53-4e2a4d4b0101"
STATUS_UUID = "7f1f0003-9f2e-4c9c-9d53-4e2a4d4b0101"
PROTOCOL_VERSION = 1

# SDL/Pygame's standard gamepad button order. It is shared by XInput devices
# such as the 8BitDo 2.4 GHz receiver and mapped DualSense controllers.
GAMEPAD_BUTTON_NAMES = {
    0: "A/Cross",
    1: "B/Circle",
    2: "X/Square",
    3: "Y/Triangle",
    4: "left bumper",
    5: "right bumper",
    6: "back/create",
    7: "start/options",
    8: "guide",
    9: "left stick",
    10: "right stick",
}


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


def build_packet_v2(
    mode: str,
    raw_position_rad: list[float],
    sequence: int,
    enable: bool,
    emergency_stop: bool,
    clear_fault: bool,
    calibration: bool,
    center_position: bool = False,
) -> bytes:
    if len(raw_position_rad) != 3:
        raise ValueError("exactly three raw joint positions are required")
    mode_flags = {"policy": 0, "position": 0x08, "wave": 0x10}
    if mode not in mode_flags:
        raise ValueError(f"unsupported gamepad mode: {mode}")
    positions = [round(value * 1000.0) for value in raw_position_rad]
    if any(value < -1571 or value > 1571 for value in positions):
        raise ValueError("raw joint position must be between -1.571 and +1.571 rad")
    flags = mode_flags[mode]
    if enable:
        flags |= 0x01
    if emergency_stop:
        flags |= 0x02
    if clear_fault:
        flags |= 0x04
    if calibration:
        flags |= 0x20
    if center_position:
        flags |= 0x40
    return struct.pack(
        "<BBhhhhhH",
        2,
        flags,
        0,
        0,
        positions[0],
        positions[1],
        positions[2],
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


def device_advertises_service(device: Any, service_uuid: str) -> bool:
    """Support Bleak's Windows advertisement shape across package versions."""
    wanted = service_uuid.lower()
    candidates = []

    metadata = getattr(device, "metadata", None)
    if isinstance(metadata, dict):
        candidates.extend(metadata.get("uuids") or [])

    details = getattr(device, "details", None)
    advertisement_args = getattr(getattr(details, "adv", None), "advertisement", None)
    candidates.extend(getattr(advertisement_args, "service_uuids", []) or [])

    return any(str(candidate).lower() == wanted for candidate in candidates)


def select_ble_target(devices: list[Any], name: str) -> Any | None:
    name_matches = [device for device in devices if (device.name or "") == name]
    if len(name_matches) == 1:
        return name_matches[0]
    if len(name_matches) > 1:
        return None

    service_matches = [
        device
        for device in devices
        if device_advertises_service(device, SERVICE_UUID)
    ]
    if len(service_matches) == 1:
        print(
            f"BLE name {name!r} was not reported; using the Crawler service "
            f"at {service_matches[0].address}."
        )
        return service_matches[0]
    return None


def normalize_axis(value: float, deadzone: float) -> float:
    """Apply a radial-independent stick dead zone and rescale to full range."""
    magnitude = abs(value)
    if magnitude <= deadzone:
        return 0.0
    scaled = (magnitude - deadzone) / (1.0 - deadzone)
    return max(-1.0, min(1.0, (-scaled if value < 0 else scaled)))


def import_pygame() -> Any:
    try:
        import pygame
    except ImportError:
        print(
            "Install gamepad support with: python -m pip install pygame",
            file=sys.stderr,
        )
        raise
    return pygame


def initialize_gamepads() -> Any:
    pygame = import_pygame()
    pygame.init()
    pygame.joystick.init()
    return pygame


def gamepad_description(joystick: Any, index: int) -> str:
    instance_id = joystick.get_instance_id()
    guid = joystick.get_guid() if hasattr(joystick, "get_guid") else "unknown"
    return f"[{index}] {joystick.get_name()} instance={instance_id} guid={guid}"


def list_gamepads() -> int:
    try:
        pygame = initialize_gamepads()
    except ImportError:
        return 2

    try:
        count = pygame.joystick.get_count()
        if count == 0:
            print("No gamepads detected.")
            return 1
        for index in range(count):
            joystick = pygame.joystick.Joystick(index)
            joystick.init()
            print(
                f"{gamepad_description(joystick, index)} "
                f"axes={joystick.get_numaxes()} "
                f"buttons={joystick.get_numbuttons()}"
            )
        return 0
    finally:
        pygame.quit()


def select_gamepad(args: argparse.Namespace, pygame: Any) -> Any:
    count = pygame.joystick.get_count()
    if count == 0:
        raise RuntimeError("No gamepad detected. Check the 8BitDo receiver or pairing.")

    if args.gamepad_index is not None:
        if not 0 <= args.gamepad_index < count:
            raise RuntimeError(
                f"Gamepad index {args.gamepad_index} is unavailable; detected {count}."
            )
        index = args.gamepad_index
    elif args.gamepad_name:
        matches = []
        for index in range(count):
            joystick = pygame.joystick.Joystick(index)
            joystick.init()
            if args.gamepad_name.lower() in joystick.get_name().lower():
                matches.append(index)
        if len(matches) != 1:
            raise RuntimeError(
                f"Expected one gamepad containing {args.gamepad_name!r}; "
                f"found {len(matches)}. Use --list-gamepads or --gamepad-index."
            )
        index = matches[0]
    else:
        index = 0
        if count > 1:
            print(
                "Multiple gamepads detected; using index 0. "
                "Select another with --gamepad-index."
            )

    joystick = pygame.joystick.Joystick(index)
    joystick.init()
    print(f"Using gamepad {gamepad_description(joystick, index)}")
    if joystick.get_numaxes() < 3:
        raise RuntimeError("Selected gamepad does not expose the required stick axes.")
    return joystick


def button_pressed(joystick: Any, button_index: int) -> bool:
    return 0 <= button_index < joystick.get_numbuttons() and bool(
        joystick.get_button(button_index)
    )


def dpad_up_pressed(joystick: Any) -> bool:
    """Return true for the standard XInput/Pygame D-pad up direction."""
    if joystick.get_numhats() <= 0:
        return False
    return joystick.get_hat(0)[1] > 0


def read_gamepad_input(
    joystick: Any, args: argparse.Namespace, pygame: Any,
    selected_mode: str | None,
) -> tuple[str | None, int, int, list[float], bool, bool, bool, bool, bool]:
    """Read the fixed Crawler button map and return the next command state."""
    pygame.event.pump()

    emergency_stop = button_pressed(joystick, args.stop_button)
    clear_fault = button_pressed(joystick, args.clear_fault_button)
    calibration = button_pressed(joystick, args.calibration_button)
    center_position = dpad_up_pressed(joystick)
    if button_pressed(joystick, args.policy_button):
        selected_mode = "policy"
    elif button_pressed(joystick, args.position_button):
        selected_mode = "position"

    left_x = normalize_axis(joystick.get_axis(0), args.deadzone)
    left_y = normalize_axis(joystick.get_axis(1), args.deadzone)
    right_x = normalize_axis(joystick.get_axis(2), args.deadzone)
    lateral = left_x
    forward = -left_y
    forward_mm_s = round(forward * args.max_speed)
    lateral_mm_s = round(lateral * args.max_speed)

    wave = button_pressed(joystick, args.wave_button)
    mode = "position" if center_position else ("wave" if wave else selected_mode)
    enable = mode in ("policy", "position", "wave")
    if emergency_stop or clear_fault or calibration:
        enable = False

    raw_position = [0.0, 0.0, 0.0]
    if mode == "position" and not center_position:
        raw_position = [
            left_x * args.max_position,
            -left_y * args.max_position,
            right_x * args.max_position,
        ]
    return (mode, forward_mm_s, lateral_mm_s, raw_position, enable,
            emergency_stop, clear_fault, calibration, center_position)


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


async def send_gamepad_commands(args: argparse.Namespace) -> int:
    try:
        from bleak import BleakClient, BleakScanner
    except ImportError:
        print("Install BLE support with: python -m pip install bleak", file=sys.stderr)
        return 2

    # Initialize Pygame only after Bleak has entered its Windows BLE event
    # loop. Pygame can initialize the main thread as a GUI/STA apartment,
    # which prevents Bleak's WinRT callbacks from running.
    pygame = None
    joystick = None
    target: Any = args.address
    try:
        if target is None:
            devices = await BleakScanner.discover(timeout=5.0)
            target = select_ble_target(devices, args.name)
            if target is None:
                print(
                    f"Expected exactly one BLE device named {args.name!r}; "
                    "no unique device matched by name or Crawler service UUID.",
                    file=sys.stderr,
                )
                return 1

        async with BleakClient(target) as client:
            print(f"Connected to Crawler BLE service at {target}")
            try:
                pygame = initialize_gamepads()
                joystick = select_gamepad(args, pygame)
            except (ImportError, RuntimeError) as exc:
                print(f"Gamepad setup failed: {exc}", file=sys.stderr)
                return 2

            try:
                try:
                    await client.start_notify(STATUS_UUID, print_status)
                except Exception as exc:
                    print(f"Warning: status notifications unavailable: {exc}")

                sequence = args.sequence
                sent = 0
                last_print = 0.0
                selected_mode: str | None = None
                while args.count == 0 or sent < args.count:
                    (
                        mode,
                        forward,
                        lateral,
                        raw_position,
                        enable,
                        emergency_stop,
                        clear_fault,
                        calibration,
                        center_position,
                    ) = read_gamepad_input(joystick, args, pygame, selected_mode)
                    if mode not in ("wave",) and not center_position:
                        selected_mode = mode
                    packet = build_packet_v2(
                        mode or "policy",
                        raw_position,
                        sequence,
                        enable,
                        emergency_stop,
                        clear_fault,
                        calibration,
                        center_position,
                    )
                    await client.write_gatt_char(COMMAND_UUID, packet, response=True)

                    now = time.monotonic()
                    if now - last_print >= 1.0 or sent == 0:
                        print(
                            f"gamepad seq={sequence & 0xFFFF} "
                            f"mode={mode or 'none'} forward={forward} lateral={lateral} "
                            f"raw={raw_position} "
                            f"enable={enable} stop={emergency_stop} "
                            f"clear={clear_fault} calibration={calibration} "
                            f"center={center_position}"
                        )
                        last_print = now

                    sequence = (sequence + 1) & 0xFFFF
                    sent += 1
                    await asyncio.sleep(args.period)
            finally:
                # Dead-man behavior: disable and stop before disconnecting.
                zero_packet = build_packet_v2(
                    "policy", [0.0, 0.0, 0.0], sequence, False, False, False, False
                )
                try:
                    await client.write_gatt_char(
                        COMMAND_UUID, zero_packet, response=True
                    )
                    print(f"sent zero/dead-man sequence={sequence & 0xFFFF}")
                except Exception as exc:
                    print(f"Warning: could not send zero command on exit: {exc}")
    except KeyboardInterrupt:
        print("Gamepad bridge stopped.")
        return 0
    except Exception as exc:
        print(f"BLE/gamepad bridge failed: {exc}", file=sys.stderr)
        return 1
    finally:
        if pygame is not None:
            pygame.quit()
    return 0


def gamepad_test(args: argparse.Namespace) -> int:
    """Print decoded gamepad input without connecting to the robot."""
    try:
        pygame = initialize_gamepads()
        joystick = select_gamepad(args, pygame)
    except (ImportError, RuntimeError) as exc:
        print(f"Gamepad setup failed: {exc}", file=sys.stderr)
        return 2

    try:
        sequence = args.sequence
        sent = 0
        selected_mode: str | None = None
        while args.count == 0 or sent < args.count:
            (mode, forward, lateral, raw_position, enable, stop, clear,
             calibration, center_position) = read_gamepad_input(
                 joystick, args, pygame, selected_mode
             )
            if mode not in ("wave",) and not center_position:
                selected_mode = mode
            print(
                f"seq={sequence & 0xFFFF} mode={mode or 'none'} "
                f"forward={forward} lateral={lateral} raw={raw_position} "
                f"enable={enable} stop={stop} clear={clear} "
                f"calibration={calibration} center={center_position}"
            )
            sequence = (sequence + 1) & 0xFFFF
            sent += 1
            time.sleep(args.period)
    except KeyboardInterrupt:
        print("Gamepad test stopped.")
        return 0
    finally:
        pygame.quit()
    return 0


async def send_commands(args: argparse.Namespace) -> int:
    try:
        from bleak import BleakClient, BleakScanner
    except ImportError:
        print("Install bleak first: python -m pip install bleak", file=sys.stderr)
        return 2

    target: Any = args.address
    if target is None:
        try:
            devices = await BleakScanner.discover(timeout=5.0)
            target = select_ble_target(devices, args.name)
        except Exception as exc:
            print(f"BLE discovery failed: {exc}", file=sys.stderr)
            return 1
        if target is None:
            print(
                f"Expected exactly one BLE device named {args.name!r}; "
                "no unique device matched by name or Crawler service UUID.",
                file=sys.stderr,
            )
            return 1

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
    parser.add_argument(
        "--list-gamepads", action="store_true", help="list local Windows gamepads"
    )
    parser.add_argument(
        "--gamepad-test",
        action="store_true",
        help="print gamepad input without connecting to Crawler",
    )
    parser.add_argument(
        "--gamepad",
        action="store_true",
        help="read a DualSense/XInput gamepad and send commands",
    )
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
    parser.add_argument(
        "--gamepad-index", type=int, help="local gamepad index from --list-gamepads"
    )
    parser.add_argument(
        "--gamepad-name", help="case-insensitive substring used to select a gamepad"
    )
    parser.add_argument(
        "--deadzone", type=float, default=0.12, help="stick dead zone from 0 to <1"
    )
    parser.add_argument(
        "--max-speed",
        type=int,
        default=1500,
        help="full-stick command in mm/s, from 0 to 1500",
    )
    parser.add_argument(
        "--policy-button",
        type=int,
        default=2,
        help="standard button index for policy mode (X/Square=2)",
    )
    parser.add_argument(
        "--position-button",
        type=int,
        default=3,
        help="standard button index for position mode (Y/Triangle=3)",
    )
    parser.add_argument(
        "--wave-button",
        type=int,
        default=1,
        help="standard button index for the position wave (B/Circle=1)",
    )
    parser.add_argument(
        "--stop-button",
        type=int,
        default=4,
        help="standard button index for emergency stop (L1/left bumper=4)",
    )
    parser.add_argument(
        "--clear-fault-button",
        type=int,
        default=0,
        help="standard button index for clearing E-stop (A/Cross=0)",
    )
    parser.add_argument(
        "--calibration-button",
        type=int,
        default=5,
        help="standard button index for calibration (R1/right bumper=5)",
    )
    parser.add_argument(
        "--max-position",
        type=float,
        default=1.0,
        help="full-stick raw position offset in radians",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.scan:
        return asyncio.run(scan_devices())
    if args.list_gamepads:
        return list_gamepads()
    if args.count < 0 or args.period < 0:
        print("count and period must be non-negative", file=sys.stderr)
        return 2
    if not 0.0 <= args.deadzone < 1.0:
        print("deadzone must be between 0 and 1", file=sys.stderr)
        return 2
    if not 0 <= args.max_speed <= 1500:
        print("max-speed must be between 0 and 1500 mm/s", file=sys.stderr)
        return 2
    if not 0.0 < args.max_position <= 1.571:
        print("max-position must be greater than 0 and at most 1.571 rad", file=sys.stderr)
        return 2
    if args.gamepad_test:
        return gamepad_test(args)
    if args.gamepad:
        return asyncio.run(send_gamepad_commands(args))
    return asyncio.run(send_commands(args))


if __name__ == "__main__":
    raise SystemExit(main())
