"""BLE console for c_hold.ino.

Install bleak manually with: python -m pip install bleak
Run from the repository root: python firmware/hold_reference.py
Run as a file, not repeatedly inside the Python interactive prompt.
Restart the interpreter after updating from older versions with background input threads.
"""

import asyncio
import math
import queue
import threading

from bleak import BleakClient, BleakScanner

SERVICE = "12345678-1234-1234-1234-123456789000"
COMMAND = "12345678-1234-1234-1234-123456789001"
P_REPORT = "12345678-1234-1234-1234-123456789003"

HELP = """
C_HOLD          Start: downward estimated motion = C 100; upward/zero = C 0
P / STATUS      Show motion estimates and motor output (no PID tuning)
STOP / OFF      Stop the motors
C 40            Manual C test; cancels an active hold (-100..100)
SCAN            Print slave I2C scan to Serial Monitor
HELP            Show these commands
QUIT            Send STOP and disconnect

Hold switches directly between 0 and +100%, without ramping or an acceleration latch.
Hold still for the first second of C_HOLD while phase=CALIBRATING (motor off).
Then release: vertical velocity starts at zero and accumulates each IMU update.
Each new C_HOLD resets the estimate. IMU velocity can still drift.
Startup readings are simply averaged; no noise scoring or rejection.
P shows vz_est (m/s): positive=up, negative=down; az_est (m/s^2); motor output.
STOP, QUIT, disconnect, manual override, failed/non-finite IMU data, I2C failure,
and command-queue errors still stop motors.
"""


def normalize_command(text):
    command = " ".join(text.upper().split())
    command = {"P": "C_HOLD P", "STATUS": "C_HOLD STATUS",
            "OFF": "C_HOLD OFF", "EXIT": "QUIT", "HOLD": "C_HOLD"}.get(command, command)
    if command in ("", "C_HOLD", "C_HOLD P", "C_HOLD STATUS",
                "C_HOLD OFF", "STOP", "SCAN", "HELP", "QUIT"):
        return command
    parts = command.split()
    if len(parts) == 2 and parts[0] in ("C_HOLD", "C"):
        value = float(parts[1])
        low, high = (100, 100) if parts[0] == "C_HOLD" else (-100, 100)
        if math.isfinite(value) and low <= value <= high:
            return f"{parts[0]} {value:g}"
    raise ValueError("Use HELP for commands; C_HOLD uses fixed 0/100 output.")


class ConsoleInput:
    """One stdin owner across reconnects within this module/interpreter."""

    def __init__(self):
        self.commands = queue.Queue()
        self.request = threading.Event()
        self.pending = False
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        while True:
            self.request.wait()
            self.request.clear()
            try:
                text = input("> ")
            except EOFError:
                text = "QUIT"
            self.commands.put(text)

    async def next_command(self, client, disconnected):
        # Reuse an outstanding read after disconnect; never create another
        # thread competing with it for stdin. Prompt only after processing.
        if not self.pending:
            self.pending = True
            self.request.set()
        while client.is_connected and not disconnected.is_set():
            try:
                text = self.commands.get_nowait()
                self.pending = False
                return text
            except queue.Empty:
                await asyncio.sleep(0.05)
        return None


_console_input = None


async def send(client, command):
    await asyncio.wait_for(
        client.write_gatt_char(COMMAND, command.encode(), response=True), 3.0
    )


async def read_report(client):
    # Explicitly bypass the Windows value cache.
    data = await asyncio.wait_for(
        client.read_gatt_char(P_REPORT, use_cached=False), 3.0
    )
    return data.decode("utf-8", errors="replace")


def report_fields(report):
    fields = dict(part.strip().split("=", 1) for part in report.split(";") if "=" in part)
    if fields.get("fw") != "hold-ack1":
        raise RuntimeError("Flash the updated c_hold.ino (hold-ack1); this firmware has no command acknowledgement.")
    return fields


async def send_checked(client, command):
    before = await read_report(client)
    fields = report_fields(before)
    expected = int(fields["rx"]) + 1
    await send(client, command)
    deadline = asyncio.get_running_loop().time() + 5.0
    latest = before
    while client.is_connected and asyncio.get_running_loop().time() < deadline:
        await asyncio.sleep(0.1)
        latest = await read_report(client)
        fields = report_fields(latest)
        if int(fields["done"]) >= expected:
            return latest
    raise RuntimeError(
        f"{command}: BLE write delivered but firmware did not confirm processing. "
        f"Compare rx/done/tick in these reports. Before: {before}\nAfter: {latest}"
    )


async def console(client, disconnected):
    global _console_input
    if _console_input is None:
        _console_input = ConsoleInput()
    while client.is_connected and not disconnected.is_set():
        text = await _console_input.next_command(client, disconnected)
        if text is None:
            break
        try:
            command = normalize_command(text)
        except ValueError as error:
            print(error)
            continue
        if command == "QUIT":
            return
        if not command:
            continue
        if command == "HELP":
            print(HELP)
            continue
        if command in ("C_HOLD P", "C_HOLD STATUS"):
            if command == "C_HOLD STATUS":
                await send_checked(client, command)
                print("Detailed hold state requested in Serial Monitor.")
            # This snapshot is maintained every 200 ms by the sketch.
            print(await read_report(client))
            continue
        print("Received:", command, "— waiting for firmware acknowledgement", flush=True)
        report = await send_checked(client, command)
        print("Processed:", command)
        print(report)
    print("Bluetooth disconnected.")


async def main():
    print(f"Hold console on/off-input2: {__file__}")
    print("Scanning for BalloonRobot...")
    device = await BleakScanner.find_device_by_filter(
        lambda device, adv: SERVICE in [u.lower() for u in (adv.service_uuids or [])],
        timeout=20.0,
    )
    if device is None:
        print("Not found. Power the master and close other BLE clients, then retry.")
        return

    disconnected = asyncio.Event()
    async with BleakClient(
        device, disconnected_callback=lambda _: disconnected.set(), timeout=20.0
    ) as client:
        if client.services.get_characteristic(COMMAND) is None:
            print("Motor command characteristic missing; check the flashed sketch.")
            return
        try:
            if client.services.get_characteristic(P_REPORT) is None:
                print("Flash the updated c_hold.ino: its P-report characteristic is missing.")
                return
            # Confirm STOP has drained older commands before accepting a start.
            await send_checked(client, "STOP")
            print(f"Connected: {device.name or 'BalloonRobot'} ({device.address})")
            print(HELP)
            await console(client, disconnected)
        finally:
            if client.is_connected:
                try:
                    await send(client, "STOP")
                except Exception as error:
                    print(f"Could not confirm STOP: {error}")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nStopped.")
    except Exception as error:
        print(f"Bluetooth error: {type(error).__name__}: {error}")
