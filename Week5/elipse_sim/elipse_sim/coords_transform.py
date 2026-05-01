import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


class CoordsTransformNode(Node):

    def __init__(self):
        super().__init__('coords_transform_node')

        # ── TF broadcaster ────────────────────────────────────────────────────
        self.tf_broadcaster = TransformBroadcaster(self)

        # ── Subscriber ────────────────────────────────────────────────────────
        self.create_subscription(Odometry, 'odom', self.odom_cb, 10)

        self.get_logger().info('Coords transform node started')

    # ── Odometry callback ─────────────────────────────────────────────────────
    def odom_cb(self, msg: Odometry):

        t = TransformStamped()

        # ── Header ────────────────────────────────────────────────────────────
        t.header.stamp    = msg.header.stamp
        t.header.frame_id = f'/odom'
        t.child_frame_id  = f'/base_footprint'

        # ── Position ──────────────────────────────────────────────────────────
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        # ── Orientation (directly from odom, already corrected there) ─────────
        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = CoordsTransformNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()