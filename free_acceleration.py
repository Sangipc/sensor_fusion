# Stream data from a single Movella DOT

import numpy as np
import asyncio
from bleak import BleakClient

measurement_characteristic_uuid = '15172001-4947-11e9-8646-d663bd873d93'
short_payload_characteristic_uuid = "15172004-4947-11e9-8646-d663bd873d93"

def notification_callback(sender, data):
    print(f"Received raw data: {data.hex()}")
    print(encode_free_acceleration(data))

def encode_free_acceleration(bytes_):
    # These bytes are grouped according to Movella's BLE specification doc
    data_segments = np.dtype([
        ('timestamp', np.uint32),
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32),
        ('zero_padding', np.uint32)
        ])
    formatted_data = np.frombuffer(bytes_, dtype=data_segments)
    return formatted_data

async def main():
    address = "F0B5B7CD-AF55-10FD-A875-1C573AFE6CA8" # Movella DOT UUID

    async with BleakClient(address) as client:
        # Check if connection was successful
        print(f"Client connection: {client.is_connected}") # prints True or False

        # Subscribe to notifications from the Short Payload Characteristic
        await client.start_notify(short_payload_characteristic_uuid, notification_callback)

        # Set and turn on the Free Acceleration measurement mode
        binary_message = b"\x01\x01\x06"
        await client.write_gatt_char(measurement_characteristic_uuid, binary_message, response=True)

        await asyncio.sleep(10.0) # How long to stream data for

asyncio.run(main())