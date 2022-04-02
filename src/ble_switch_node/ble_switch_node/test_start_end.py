from simple_webcam import simple_webcam_node as camnode
import time

import asyncio


import rclpy
from rclpy.node import Node


class RemoteService(Node):

    def __init__(self):
        super().__init__('RemoteService')
        self.webcamnode = None
        loop = asyncio.get_event_loop()
        loop.run_until_complete(self.run())

        time.sleep(10)
        print('off')
        self.switch_off()

        '''
        time.sleep(10)
        self.switch_on()
        '''

    async def run(self):
        self.webcamnode = camnode.WebcamNode()
        print("started")




    def switch_on(self):
        self.webcamnode.camera.start()

    def switch_off(self):
        self.webcamnode.camera.stop()


def main(args=None):
    rclpy.init(args=args)

    remote_service = RemoteService()

    rclpy.spin(remote_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()