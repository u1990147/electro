import asyncio
from bleak import BleakClient, BleakError

SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"

async def run():
    address = "EC:E3:34:7B:77:16"
    try:
        async with BleakClient(address) as client:
            print("Connectat")
            value = await client.read_gatt_char(CHARACTERISTIC_UUID)
            print(f"Informació rebuda: {value.decode()}")
    except BleakError as e:
        print(f"Error rebuda: {e}")
if __name__ == "__main__":
    asyncio.run(run())