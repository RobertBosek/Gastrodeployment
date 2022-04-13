# # pip3 install bleak

import asyncio
import traceback
import logging
import random
from bleak import BleakClient

# BLE peripheral ID
address = "84:0D:8E:25:B0:8A" # M5Stack
# address = "84:0D:8E:25:9D:02" # M5Stack2
# address =   "B4:E6:2D:D5:B9:6F" # WEMOS D32 Pro
UUID =      "441c5e53-1f8b-401d-9222-0e32f400fd15"  # service ID

def on_new_data(sender: int, data: bytearray):
    sensor_data = data.decode('utf_8')
    print(sensor_data)

async def run():
    random.seed()
    client = BleakClient(address)
    while True:
        try:
            await client.connect()
            if client.is_connected:
                # while True:
                await client.start_notify(UUID, on_new_data)
            else:
                print("Failed to connect.")
        except KeyboardInterrupt:
            if client.is_connected:
                logging.info("Disconnecting...")
                await client.disconnect()
                logging.info("Disconnected.")
            else:
                loggin.info("Client not connected. No disconnect necessary.")
        except Exception as e:
            logging.info("Connection failed. Retrying...")
            await asyncio.sleep(random.random())
            # logging.error(traceback.format_exc())

asyncio.run(run())
