import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import numpy as np

class DronePublisher(Node):

    def __init__(self):
        super().__init__('puzzlebot_jointPub')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('wheel_radius', 0.05)   # m
        self.declare_parameter('wheel_base',   0.108)  # m

        self.r = self.get_parameter('wheel_radius').value
        self.L = self.get_parameter('wheel_base').value

        # ── Wheel angle state ─────────────────────────────────────────────────
        self.angle_r = 0.0
        self.angle_l = 0.0

        self.prev_time = None

        # ── Publisher ─────────────────────────────────────────────────────────
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # ── Subscriber ────────────────────────────────────────────────────────
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        self.get_logger().info('Joint state publisher started')

    # ── Odometry callback ─────────────────────────────────────────────────────
    def odom_cb(self, msg: Odometry):
        now = self.get_clock().now()

        if self.prev_time is None:
            self.prev_time = now
            return

        dt = (now - self.prev_time).nanoseconds / 1e9
        self.prev_time = now

        if dt <= 0.0:
            return

        # ── Recover wheel speeds from odom twist ──────────────────────────────
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z

        wr = (v + w * self.L / 2.0) / self.r
        wl = (v - w * self.L / 2.0) / self.r

        # ── Integrate wheel angles ────────────────────────────────────────────
        self.angle_r += wr * dt
        self.angle_l += wl * dt

        # ── Publish JointState ────────────────────────────────────────────────
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name     = ['base_link_to_wheel_r', 'base_link_to_wheel_l']
        js.position = [self.angle_r, self.angle_l]
        js.velocity = [wr, wl]
        js.effort   = []

        self.joint_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = DronePublisher()
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