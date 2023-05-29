from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from tf2_ros import TransformBroadcaster

import serial

from rocket_visual.rocketdata import RocketData, rocketdata_from_array


class FramePublisher(Node):

    def __init__(self):
        super().__init__('broad')

        # Declare and acquire `rocket_name` parameter
        self.rocketname = self.declare_parameter(
            'rocketname', 'frame').get_parameter_value().string_value

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        with serial.Serial('/dev/ttyACM0', 115200, timeout=1) as ser:
            while(True):
                line = ser.readline()   # read a '\n' terminated line
                self.handle_serial_line(line)


    def handle_serial_line(self, line):
        if(len(line) == 0 or not isinstance(line[0], int)):
            return

        line_str = str(line).strip()[2:]
        values = line_str.split(",")

        if(len(values) != 14):
            return
        
        values = values[:13]
        rd = rocketdata_from_array(values)

        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp.sec = int(rd.timestamp/1000000)
        t.header.stamp.nanosec = int(rd.timestamp%1000000)*1000
        t.header.frame_id = 'world'
        t.child_frame_id = self.rocketname

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = rd.altitude

        t.transform.rotation.x = rd.q0
        t.transform.rotation.y = rd.q1
        t.transform.rotation.z = rd.q2
        t.transform.rotation.w = rd.q3

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
