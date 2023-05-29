from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from tf2_ros import TransformBroadcaster

import serial
import math
from dataclasses import dataclass

@dataclass
class RocketData:
    timestamp: int = 0
    q0: float = 0.0
    q1: float = 0.0
    q2: float = 0.0
    q3: float = 0.0
    ax: float = 0.0
    ay: float = 0.0
    az: float = 0.0
    gx: float = 0.0
    gy: float = 0.0
    gz: float = 0.0
    temperature: float = 0.0
    pressure: float = 0.0
    altitude: float = 0.0


def rocketdata_from_array(array):
    rd = RocketData()
    rd.timestamp = int(array[0])

    rd.ax = float(array[1])
    rd.ay = float(array[2])
    rd.az = float(array[3])
    rd.gx = float(array[4])
    rd.gy = float(array[5])
    rd.gz = float(array[6])

    rd.q1 = float(array[7])
    rd.q2 = float(array[8])
    rd.q3 = float(array[9])
    rd.q0 = math.sqrt(1.0 - (rd.q1**2 + rd.q2**2 + rd.q3**2))

    rd.pressure = float(array[10])
    rd.altitude = float(array[11])
    rd.temperature = float(array[12])
    return rd


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
