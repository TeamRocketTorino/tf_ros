import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

import serial
import sys

from rocket_visual.rocketdata import RocketData


class FramePublisher(Node):

    def __init__(self, port, baudrate):
        super().__init__('broad_serial')

        # Declare and acquire `rocket_name` parameter
        self.rocketname = self.declare_parameter(
            'rocketname', 'base_footprint').get_parameter_value().string_value

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        with serial.Serial(port, baudrate, timeout=1) as ser:
            while(True):
                line = ser.readline()   # read a '\n' terminated line
                try:
                    self.handle_serial_line(line)
                except:
                    pass


    def handle_serial_line(self, line):
        rd = RocketData().from_string(line.decode())

        if rd==None:
            return
        
        t = rd.to_tf2('world', self.rocketname)

        if t==None:
            return

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)


def main():
    port = sys.argv[1] or '/dev/ttyACM0'
    baudrate = int(sys.argv[2]) or 115200

    rclpy.init()
    node = FramePublisher(port, baudrate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
