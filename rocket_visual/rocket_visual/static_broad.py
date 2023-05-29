import sys

from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class StaticFramePublisher(Node):

    def __init__(self, transformation):
        super().__init__('static_broad')

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        # Publish static transforms once at startup
        self.msg = rclpy.wait_for_message('flight_data',TransformStamped)
        self.make_transforms(self.msg)

    def make_transforms(self, msg):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'rocket_fixed'

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        t.transform.rotation.x = msg.transform.rotation.x
        t.transform.rotation.y = msg.transform.rotation.y
        t.transform.rotation.z = msg.transform.rotation.z
        t.transform.rotation.w = msg.transform.rotation.w

        self.tf_static_broadcaster.sendTransform(t)


def main():
    logger = rclpy.logging.get_logger('logger')

    # pass parameters and initialize node
    rclpy.init()
    node = StaticFramePublisher(sys.argv)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
