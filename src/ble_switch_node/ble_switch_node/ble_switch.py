import threading
import asyncio
from bleak import BleakScanner


class BLESwitch:

    def __init__(self, node=None):
        self.node = node
        self.switch_active = False
        self.read_lock = threading.Lock()
        self.started = False

    def start(self):
        if self.started:  # Prevent the thread from starting it again if it is already running
            print('Already running')
            return None
        else:
            self.started = True
            self.thread = threading.Thread(target=self.__scan, args=())
            self.thread.start()
            return self

    def __scan(self):
        while self.started:
            # Get the newest frame from the camera
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(self.run())


    async def run(self):
        devices = await BleakScanner.discover()
        temp_status = False
        for d in devices:
            address = str(d).split(' ')[0][:-1]
            name = str(d).replace(address, '')[2:]
            if address == 'F6:18:2C:8C:2F:B6':  # MAC-Address of BLE Module in the Cube
                #print('Found BLE Beacon:', name)
                temp_status = True
                break

        if self.node is not None:
            self.node.get_logger().info("switch status %s" % ('on' if temp_status else 'off'))
            if self.switch_active != temp_status:
                self.switch_active = temp_status
                self.node.get_logger().info("sending request: turn frames %s" % ('on' if temp_status else 'off'))
                self.node.send_request(self.switch_active)

    # Stop the thread
    def stop(self):
        if self.started:
            self.started = False
            self.thread.join()


def main(args=None):
    loop = asyncio.get_event_loop()
    b = BLESwitch()
    loop.run_until_complete(b.run())


if __name__ == '__main__':
    main()

