import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../')))

import asyncio
from sphero_sdk import SerialAsyncDal
from sphero_sdk import SpheroRvrAsync
from sphero_sdk import SpheroRvrTargets

import rclpy
from rclpy.node import Node

import tty
import termios

loop = asyncio.get_event_loop()

rvr = SpheroRvrAsync(
    dal=SerialAsyncDal(
        loop
    )
)

class Drive_Node(Node):
    def __init__(self):
        super().__init__("drive_rvr")
        print('Node Created.')
        self.get_logger().info("'This Node is for driving the RVR!'")
        print('To drive the RVR use the WSAD keys.\nW - Foward | S - Reverse | D - Turn Right | A - Turn Left | Space - Stop All Action | esc - Exit Program')
        # drive variables
        self.speed = 0 # 0-255
        self.heading = 0 # 0-359 {0 - Foward, 90 - Right, 180 - Back, 270 - Left}
        self.flag = 0 # {0 - Drive, 1 - Reverse}

        loop.run_until_complete(self.drive_rvr())

    def get_key(self):
        tty.setcbreak(sys.stdin)
        key_press = ord(sys.stdin.read(1))
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))

        if key_press == 119:
            return 'w'
        elif key_press == 97:
            return 'a'
        elif key_press == 100:
            return 'd'
        elif key_press == 115:
            return 's'
        elif key_press == 19:
            return 'space'
        elif key_press == 27:
            return 'esc'

    async def drive_rvr(self):
        while True:
            key_press = self.get_key()

            if key_press == 'w':
                if self.flag == 1:
                    print('Drive.')
                    self.speed = 0
                else:
                    print('Speeding Up Forward.')
                    self.speed += 51
                self.flag = 0
            
            elif key_press == 's':
                if self.flag == 0:
                    print('Reverse.')
                    self.speed = 0
                else:
                    print('Speeding Up Reverse.')
                    self.speed += 51
                self.flag = 1
            
            elif key_press == 'a':
                print('Turning Left.')
                self.heading -= 10
            
            elif key_press == 'd':
                print('Turning Right.')
                self.heading += 10
            
            elif key_press == 'space':
                print('Stopping Drive Actions.')
                self.speed = 0
                self.flag = 0

            # Exit the program
            elif key_press == 'esc':
                print('Exiting Drive Mode.')
                print('You may have to type "reset" into terminal upon exiting.\nCurrent bug where terminal will not show what you are typing after the program stops.')
                break
            
            # Manage drive constraints
            if self.speed > 255:
                self.speed = 255
            elif self.speed < -255:
                self.speed = -255
            
            if self.heading > 359:
                self.heading = self.heading - 359
            elif self.heading < 0:
                self.heading = 359 + self.heading
            
            await rvr.drive_with_heading(self.speed, self.heading, self.flag)
            await asyncio.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = Drive_Node()
    #rclpy.spin(node)
    node.destroy_node()
    print('Node Destroyed.')
    print('Program Terminated')
    rclpy.shutdown()

if __name__ == "__main__":
    main()
