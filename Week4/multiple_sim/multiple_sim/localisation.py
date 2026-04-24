import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import numpy as np
import transforms3d
import signal
import math


class DeadReckoning(Node):
    def __init__(self):
        super().__init__('localisation_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('wheel_radius', 0.05)   # m
        self.declare_parameter('wheel_base',   0.108)  # m

        self.r = self.get_parameter('wheel_radius').value
        self.L = self.get_parameter('wheel_base').value

        self.declare_parameter('robot_name', 'robot')
        self.robot_name = self.get_parameter('robot_name').value

        # ── State ─────────────────────────────────────────────────────────────
        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0

        self.wr = 0.0   # right wheel rad/s
        self.wl = 0.0   # left  wheel rad/s

        self.prev_time = None

        # ── Publishers ────────────────────────────────────────────────────────
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(Float32, 'wr', self.wr_cb, 10)
        self.create_subscription(Float32, 'wl', self.wl_cb, 10)

        # ── Timer (integration loop at 50 Hz) ─────────────────────────────────
        self.create_timer(0.02, self.update)

        self.get_logger().info(
            f'Dead reckoning node started — r={self.r} m, L={self.L} m'
        )

    # ── Wheel velocity callbacks ───────────────────────────────────────────────
    def wr_cb(self, msg: Float32):
        self.wr = msg.data

    def wl_cb(self, msg: Float32):
        self.wl = msg.data

    # ── Integration loop ──────────────────────────────────────────────────────
    def update(self):
        now = self.get_clock().now()

        if self.prev_time is None:
            self.prev_time = now
            return

        dt = (now - self.prev_time).nanoseconds / 1e9
        self.prev_time = now

        if dt <= 0.0:
            return

        # ── Differential-drive kinematics (your model) ────────────────────────
        v_robot = self.r * (self.wr + self.wl) / 2.0
        w_robot = self.r * (self.wl - self.wr) / self.L

        # ── Pose integration ──────────────────────────────────────────────────
        self.x   += v_robot * math.cos(self.yaw + math.pi / 2.0) * dt
        self.y   += v_robot * math.sin(self.yaw + math.pi / 2.0) * dt
        self.yaw += w_robot * dt

        # Normalize yaw to [-π, π]
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        # ── Quaternion (your URDF correction) ────────────────────────────────
        yaw_corrected = self.yaw + math.pi / 2.0
        qz = math.sin(yaw_corrected / 2.0)
        qw = math.cos(yaw_corrected / 2.0)

        # ── Publish Odometry message ──────────────────────────────────────────
        odom = Odometry()
        odom.header.stamp    = now.to_msg()
        odom.header.frame_id = f'{self.robot_name}/odom'
        odom.child_frame_id  = f'{self.robot_name}/base_footprint'

        # Pose
        odom.pose.pose.position.x    = self.x
        odom.pose.pose.position.y    = self.y
        odom.pose.pose.position.z    = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # Twist
        odom.twist.twist.linear.x  = v_robot
        odom.twist.twist.linear.y  = 0.0
        odom.twist.twist.linear.z  = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = w_robot

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = DeadReckoning()
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