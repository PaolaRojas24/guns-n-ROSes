import math
import rclpy
from rclpy.node import Node

import tf2_ros
from geometry_msgs.msg import Point


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class PursuitNode(Node):

    def __init__(self):
        super().__init__('pursuit_node')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('follow_distance', 0.5)
        self.declare_parameter('control_rate',    20.0)

        self.follow_dist = self.get_parameter('follow_distance').value
        rate_hz          = self.get_parameter('control_rate').value

        # ── TF listener ───────────────────────────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Subscription (controller feedback) ────────────────────────────
        self.create_subscription(Point, 'next_point', self._next_point_cb, 10)

        # ── Publisher ─────────────────────────────────────────────────────
        self.pub_setpoint = self.create_publisher(Point, 'setpoint', 10)

        # ── Control timer ─────────────────────────────────────────────────
        self.create_timer(1.0 / rate_hz, self._control_loop)

        self.get_logger().info(
            f'PursuitNode ready – follow_distance={self.follow_dist} m | '
            f'frame: robot2/odom → robot1/base_footprint'
        )

    # ── Timer callback ────────────────────────────────────────────────────

    def _control_loop(self):
        try:
            # Express robot1/base_footprint IN robot2/odom frame.
            # This is exactly the coordinate space robot2's controller works in.
            tf = self.tf_buffer.lookup_transform(
                'robot2/odom',            # target: robot2's reference frame
                'robot1/base_footprint',  # source: robot1's body frame
                rclpy.time.Time(),        # latest available transform
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'TF lookup failed: {e}', throttle_duration_sec=2.0)
            return

        t   = tf.transform.translation
        yaw = yaw_from_quaternion(tf.transform.rotation)

        # Compute point follow_distance metres behind robot1
        follow_x = t.x - self.follow_dist * math.cos(yaw)
        follow_y = t.y - self.follow_dist * math.sin(yaw)

        self.pub_setpoint.publish(Point(x=follow_x, y=follow_y, z=0.0))

        self.get_logger().debug(
            f'robot1 in robot2/odom=({t.x:.2f},{t.y:.2f}) '
            f'yaw={math.degrees(yaw):.1f}° '
            f'→ follow_point=({follow_x:.2f},{follow_y:.2f})'
        )

    def _next_point_cb(self, msg: Point):
        self.get_logger().debug(f'next_point ← x={msg.x:.3f}  y={msg.y:.3f}')


# ── Entry point ────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = PursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()