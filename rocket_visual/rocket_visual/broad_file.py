import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

import serial
import os

from rocket_visual.rocketdata import RocketData


class FramePublisher(Node):

    def __init__(self):
        super().__init__('broad_file')

        # Declare and acquire `rocket_name` parameter
        self.rocketname = self.declare_parameter(
            'rocketname', 'frame').get_parameter_value().string_value

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)


        filename = self.declare_parameter("file", "").get_parameter_value().string_value

        file = open(filename, 'r')
        for line in file:
            self.handle_file_line(line)


    def handle_file_line(self, line):
        rd = RocketData().from_string(line)

        if rd==None:
            return
        
        t = rd.to_tf2('world', self.rocketname)

        if t==None:
            return

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
