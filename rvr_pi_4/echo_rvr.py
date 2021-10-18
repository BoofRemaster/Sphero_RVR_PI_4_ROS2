from sphero_sdk.common.commands.power import sleep
import rclpy
from rclpy.node import Node

import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../')))

import asyncio 
from sphero_sdk import SpheroRvrAsync
from sphero_sdk import SerialAsyncDal
from sphero_sdk import SpheroRvrTargets


loop = asyncio.get_event_loop()

rvr = SpheroRvrAsync(
    dal=SerialAsyncDal(
        loop
    )
)
class Echo_Node(Node):
    def __init__(self):
        super().__init__("echo_rvr")
        print('Node Created.')
        self.get_logger().info("'This Node is for checking the connection to the RVR!'")
        loop.run_until_complete(self.echo_rvr())
        print('\nEcho Completed.')
        print('Ending Program.')

    async def echo_rvr(self):
        echo_response = await rvr.echo(
        data=[0, 1, 2],
        target=SpheroRvrTargets.primary.value
        )
        print('Echo response 1: ', echo_response)

        echo_response = await rvr.echo(
            data=[4, 5, 6],
            target=SpheroRvrTargets.secondary.value
        )
        print('Echo response 2: ', echo_response)

        await rvr.close()

def main(args=None):
    rclpy.init(args=args)
    node = Echo_Node()
    # rclpy.spin(node)
    node.destroy_node()
    print('Node Destroyed.')
    print('Shutting Down...')
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    loop.close()