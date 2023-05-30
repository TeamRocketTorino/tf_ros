import sys

from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class StaticFramePublisher(Node):

    def __init__(self):
        super().__init__('static_broad')

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1.0, self.make_transforms)

    def make_transforms(self):

        msg = None

        to_frame_rel = 'base_footprint'
        from_frame_rel = 'world'

        try:
            msg = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
        if msg is not None:
            t = TransformStamped()

            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'world'
            t.child_frame_id = 'fixed_frame'

            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            # t.transform.translation.z = msg.transform.translation.z

            t.transform.rotation.x = msg.transform.rotation.x
            t.transform.rotation.y = msg.transform.rotation.y
            t.transform.rotation.z = msg.transform.rotation.z
            t.transform.rotation.w = msg.transform.rotation.w

            self.tf_static_broadcaster.sendTransform(t)

            raise SystemExit  

def main():
    logger = rclpy.logging.get_logger('logger')

    # pass parameters and initialize node
    rclpy.init()
    node = StaticFramePublisher()
    try:
        rclpy.spin(node)
    except SystemExit:                 # <--- process the exception 
        rclpy.logging.get_logger("Quitting").info('Done')
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
