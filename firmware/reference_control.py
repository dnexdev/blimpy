import asyncio
from bleak import BleakScanner, BleakClient

SERVICE = "12345678-1234-1234-1234-123456789000"
COMMAND = "12345678-1234-1234-1234-123456789001"

async def main():
    print("Finding ESP32...")
    device = await BleakScanner.find_device_by_filter(
        lambda d, a: SERVICE in [u.lower() for u in a.service_uuids],
        timeout=20,
    )

    if device is None:
        print("Not found. Close other BLE scripts and retry.")
        return

    async with BleakClient(device) as client:
        await client.write_gatt_char(COMMAND, b"STOP", response=True)
        print("""
CONNECTED — enter commands:
  C 40                  Motor C forward at 40%
  D -40                 Motor D reverse at 40%
  E 30                  Motor E forward at 30%
  F 30                  Motor F forward at 30%
  ALL 20                All motors at 20%
  MOTORS 10 20 30 40     Set C D E F individually
  STOP                  Stop all motors
  QUIT                  Stop and disconnect
""")
        try:
            while client.is_connected:
                command = (await asyncio.to_thread(input, "> ")).strip().upper()
                if command == "QUIT":
                    break
                if command:
                    await client.write_gatt_char(
                        COMMAND, command.encode(), response=True
                    )
                    print("Sent:", command)
        finally:
            if client.is_connected:
                await client.write_gatt_char(COMMAND, b"STOP", response=True)

try:
    asyncio.run(main())
except KeyboardInterrupt:
    print("\nStopped")
except Exception as e:
    print("Error:", e)