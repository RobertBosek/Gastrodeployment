
# pip3 install bleak

import asyncio
from bleak import BleakScanner


async def run():
    print('hallo')
    devices = await BleakScanner.discover()

    for d in devices:
        print('hallo2')
        address = str(d).split(' ')[0][:-1]
        name = str(d).replace(address, '')[2:]
        if address == 'F6:18:2C:8C:2F:B6':  # MAC-Address of BLE Module in the Cube
            print('Found BLE Beacon:', name)
            return

loop = asyncio.get_event_loop()
loop.run_until_complete(run())