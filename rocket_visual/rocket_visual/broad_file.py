import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

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

        frequency = 111.0
        self.timer = self.create_timer(1.0/frequency, self.timer_callback) # same frequency as real sensor
        
        self.file = open(filename, 'r')

    def timer_callback(self):
        line = self.file.readline()

        if not line:
            raise SystemExit
            
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
    except SystemExit:
        pass

    rclpy.shutdown()
