from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

class FramePublisher(Node):

    def __init__(self):
        super().__init__('broad')

        # Declare and acquire `rocket_name` parameter
        self.rocketname = self.declare_parameter(
            'rocketname', 'frame').get_parameter_value().string_value

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # callback function on each message
        self.subscription = self.create_subscription(
            TransformStamped,
            f'data',
            self.handle_rocket_attitude,
            50)
        self.subscription  # prevent unused variable warning

    def handle_rocket_attitude(self, msg):
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = self.rocketname

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = msg.transform.translation.z

        t.transform.rotation.x = t.transform.rotation.x
        t.transform.rotation.y = t.transform.rotation.x
        t.transform.rotation.z = t.transform.rotation.x
        t.transform.rotation.w = t.transform.rotation.x

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
