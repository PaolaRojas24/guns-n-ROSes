import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Float32
import numpy as np
import transforms3d
import math


class KinematicModelNode(Node):
    def __init__(self):
        super().__init__('kinematic_model_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('wheel_radius', 0.05)   # m
        self.declare_parameter('wheel_base',   0.108)  # m

        self.r = self.get_parameter('wheel_radius').value
        self.L = self.get_parameter('wheel_base').value

        # ── State ─────────────────────────────────────────────────────────────
        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0

        self.v = 0.0   # linear  velocity from cmd_vel
        self.w = 0.0   # angular velocity from cmd_vel

        self.prev_time = None

        # ── Publishers ────────────────────────────────────────────────────────
        self.wr_pub   = self.create_publisher(Float32, '/wr',   10)
        self.wl_pub   = self.create_publisher(Float32, '/wl',   10)
        self.pose_pub = self.create_publisher(Pose,    '/pose', 10)

        # ── Subscriber ────────────────────────────────────────────────────────
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)

        # ── Timer (integration loop at 50 Hz) ─────────────────────────────────
        self.create_timer(0.02, self.update)

        self.get_logger().info(
            f'Kinematic model node started — r={self.r} m, L={self.L} m'
        )

    # ── cmd_vel callback ──────────────────────────────────────────────────────
    def cmd_vel_cb(self, msg: Twist):
        self.v = msg.linear.x
        self.w = msg.angular.z

    # ── Integration and publish loop ──────────────────────────────────────────
    def update(self):
        now = self.get_clock().now()

        if self.prev_time is None:
            self.prev_time = now
            return

        dt = (now - self.prev_time).nanoseconds / 1e9
        self.prev_time = now

        if dt <= 0.0:
            return

        # ── Inverse kinematics: cmd_vel → wr, wl ─────────────────────────────
        wr = (self.v + self.w * self.L / 2.0) / self.r
        wl = (self.v - self.w * self.L / 2.0) / self.r

        # ── Publish wheel speeds ──────────────────────────────────────────────
        wr_msg = Float32()
        wl_msg = Float32()
        wr_msg.data = wr
        wl_msg.data = wl
        self.wr_pub.publish(wr_msg)
        self.wl_pub.publish(wl_msg)

        # ── Pose integration ──────────────────────────────────────────────────
        self.x   += self.v * math.cos(self.yaw) * dt
        self.y   += self.v * math.sin(self.yaw) * dt
        self.yaw += self.w * dt

        # Normalize yaw to [-π, π]
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        # ── Quaternion via transforms3d ───────────────────────────────────────
        # axangle2quat returns (w, x, y, z) — reorder for ROS (x, y, z, w)
        q = transforms3d.axangles.axangle2quat([0, 0, 1], self.yaw)

        # ── Publish Pose ──────────────────────────────────────────────────────
        pose = Pose()
        pose.position.x    = self.x
        pose.position.y    = self.y
        pose.position.z    = 0.0
        pose.orientation.x = q[1]
        pose.orientation.y = q[2]
        pose.orientation.z = q[3]
        pose.orientation.w = q[0]

        self.pose_pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = KinematicModelNode()
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