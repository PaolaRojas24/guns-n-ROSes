"""Puzzlebot Sim — modelo cinemático diferencial. Publica wr, wl y pose."""
import math

import rclpy
import transforms3d
from geometry_msgs.msg import Pose, Twist
from rclpy.node import Node
from std_msgs.msg import Float32


class KinematicModelNode(Node):

    def __init__(self):
        super().__init__('kinematic_model_node')

        # ── Parámetros ────────────────────────────────────────────────────────
        self.declare_parameter('wheel_radius', 0.05)   # m
        self.declare_parameter('wheel_base',   0.19)   # m

        self.r = self.get_parameter('wheel_radius').value
        self.L = self.get_parameter('wheel_base').value

        # ── Estado interno ────────────────────────────────────────────────────
        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0

        self.v = 0.0   # velocidad lineal  [m/s]  de cmd_vel
        self.w = 0.0   # velocidad angular [rad/s] de cmd_vel

        self.prev_time = None

        # ── Publicadores ──────────────────────────────────────────────────────
        self.wr_pub   = self.create_publisher(Float32, 'wr',   10)
        self.wl_pub   = self.create_publisher(Float32, 'wl',   10)
        self.pose_pub = self.create_publisher(Pose,    'pose', 10)

        # ── Suscriptores ──────────────────────────────────────────────────────
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, 10)

        # ── Timer de integración a 50 Hz ──────────────────────────────────────
        self.create_timer(0.02, self.update)

        self.get_logger().info(
            f'Kinematic model node started — r={self.r} m, L={self.L} m'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def cmd_vel_cb(self, msg: Twist):
        self.v = msg.linear.x
        self.w = msg.angular.z

    # ── Loop de integración ───────────────────────────────────────────────────

    def update(self):
        now = self.get_clock().now()

        if self.prev_time is None:
            self.prev_time = now
            return

        dt = (now - self.prev_time).nanoseconds / 1e9
        self.prev_time = now

        if dt <= 0.0:
            return

        # Cinemática inversa: cmd_vel → velocidades de rueda
        wr = (self.v + self.w * self.L / 2.0) / self.r
        wl = (self.v - self.w * self.L / 2.0) / self.r

        wr_msg      = Float32()
        wl_msg      = Float32()
        wr_msg.data = wr
        wl_msg.data = wl
        self.wr_pub.publish(wr_msg)
        self.wl_pub.publish(wl_msg)

        # Integración de la pose
        self.x   += self.v * math.cos(self.yaw) * dt
        self.y   += self.v * math.sin(self.yaw) * dt
        self.yaw += self.w * dt

        # Normalización de yaw a [-π, π]
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        # Conversión yaw → cuaternión mediante transforms3d
        q = transforms3d.euler.euler2quat(0.0, 0.0, self.yaw)

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
