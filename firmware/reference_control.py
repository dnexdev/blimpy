"""Reference BLE controller for esp32_master.ino; run this on the laptop.

Use the service discovery and command writes here as a reference when integrating
the master ESP32 into the main control code.
"""

import asyncio
from bleak import BleakScanner, BleakClient

SERVICE_UUID = "12345678-1234-1234-1234-123456789000"
COMMAND_UUID = "12345678-1234-1234-1234-123456789001"

async def main():
    print("Scanning...")

    device = await BleakScanner.find_device_by_filter(
        lambda d, adv:
            SERVICE_UUID.lower()
            in [u.lower() for u in adv.service_uuids],
        timeout=10.0
    )

    if device is None:
        print("BalloonRobot service not found")
        return

    print(f"Found: {device.name} {device.address}")

    async with BleakClient(device) as client:
        print("Connected:", client.is_connected)

        print("Commands:")
        print("C 40")
        print("D 40")
        print("E 40")
        print("F 40")
        print("ALL 30")
        print("MOTORS 20 30 -10 0")
        print("STOP")

        while True:
            command = input("> ").strip()

            if not command:
                continue

            await client.write_gatt_char(
                COMMAND_UUID,
                command.encode(),
                response=False
            )

if __name__ == "__main__":
    asyncio.run(main())
